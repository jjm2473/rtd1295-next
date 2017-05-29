/*
 * linux/drivers/usb/fujitsu_otg/f_usb20hdc_otg.c - F_USB20HDC USB
 * host controller driver
 *
 * Copyright (C) FUJITSU ELECTRONICS INC. 2013. All rights reserved.
 * Copyright (C) 2012 FUJITSU SEMICONDUCTOR LIMITED.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */
/* #define DEBUG */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/of.h>
#include <asm/unaligned.h>
#include <linux/pm_runtime.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include <linux/platform_data/dma-mb8ac0300-hdmac.h>
#include "f_usb20hdc_phy.h"

#define CLR_DMA_CHAN 0
#define SET_DMA_CHAN 1

struct f_usb20hdc_otg *g_f_otg;

/*
 * when host and device mode run without otg driver, it has do reset
 * by itself. However when they run with otg driver code, reset job
 * must be handled by otg, in order to prevent otg interrupt setting
 * being clear accidentally by other code.
 */
void f_usb20hdc_soft_reset_ip(struct f_usb20hdc_otg *f_otg)
{
	void __iomem *addr = f_otg->reg_base + (0x8000);
	void __iomem *end_addr = f_otg->reg_base + (0x10000);

	/*controller reset*/
	if (get_register_bits(f_otg->reg_base, F_USB20HDC_REG_PORTSC, 21, 1) &&
	get_register_bits(f_otg->reg_base, F_USB20HDC_REG_PORTSC, 22, 1)) {
		set_port_power_ctl_req(f_otg->reg_base, 0);
		set_soft_reset(f_otg->reg_base);
		enable_bus_power_supply(f_otg->reg_base, 1);
		set_port_power_ctl_req(f_otg->reg_base, 1);
	} else {
		set_soft_reset(f_otg->reg_base);
	}

	/*phy setting*/
	set_id_pull_up(f_otg->reg_base, 1);

	if (!get_id(f_otg->reg_base)) {
		set_port_power_ctl_req(f_otg->reg_base, 1);
		enable_bus_power_supply(f_otg->reg_base, 1);
	}

	if (f_otg->mode == F_USB20HDC_MODE_DUAL_ROLE) {
		/*otg id interrupt setting*/
		set_id_ren(f_otg->reg_base, 1);
		set_id_fen(f_otg->reg_base, 1);
	}

	if (f_otg->mode != F_USB20HDC_MODE_HOST) {
		/*
		 * host mode donnt care id and vbus interrupt
		 * and otg_inten should be 1 forever in other mode.
		*/
		set_otg_inten(f_otg->reg_base, 1);
	} else
		set_otg_inten(f_otg->reg_base, 0);

	/*
	 * initialize F_USB20HDC system configuration register
	 * [notice]:set of soft_reset bit is prohibition
	 */
	set_byte_order(f_otg->reg_base, 0);
	set_burst_wait(f_otg->reg_base, 1);

#if 1
	/* make sure irq disable already here */
	while (addr != end_addr) {
		__raw_writel(0x0, addr);
		addr += 4;
	}
#endif
}

/*
 * This interrupt might happen with the connected interrupt.
 * so it better be as soon as possible, and let initialized host
 * to process connected interrupt.
 */
static void switch_to_host_event(struct work_struct *work)
{
	struct delayed_work *d_work = container_of(work,
		struct delayed_work, work);
	struct f_usb20hdc_otg *f_otg = container_of(d_work,
		struct f_usb20hdc_otg, switch_to_host);

	dev_info(f_otg->dev, "switch_to_host_event() is started.");

	mutex_lock(&f_otg->role_switch_lock);
	if (f_otg->host_working == 1) {
		dev_info(f_otg->dev, "switch_to_host_event(): no need.");
		mutex_unlock(&f_otg->role_switch_lock);
		return;
	}

	/*suspend udc driver*/
	if (f_udc_otg_suspend(f_otg)) {
		dev_err(f_otg->dev, "%s() %d.", __func__, __LINE__);
		mutex_unlock(&f_otg->role_switch_lock);
		return;
	}

	/*do reset for host mode initial*/
	f_usb20hdc_soft_reset_ip(f_otg);

	/*register Fujitsu hcd driver*/
	if (0 == f_usb20hdc_hcd_probe(f_otg)) {
		f_otg->host_working = 1;
		dev_info(f_otg->dev, "host mode is enabled.");
	} else
		dev_err(f_otg->dev, "failed to enable host mode.");

	mutex_unlock(&f_otg->role_switch_lock);
	dev_info(f_otg->dev, "switch_to_host_event() is ended.");
}

/*
 * This interrupt might happen with the disconnected interrupt.
 * so it better be delayed, let host mode finish disconnect
 * procedure before unregisterring host mode driver.
 */
static void switch_to_gadget_event(struct work_struct *work)
{
	struct delayed_work *d_work = container_of(work ,
		struct delayed_work, work);
	struct f_usb20hdc_otg *f_otg = container_of(d_work,
		struct f_usb20hdc_otg, switch_to_gadget);

	dev_info(f_otg->dev, "switch_to_gadget_event() is started.");

	mutex_lock(&f_otg->role_switch_lock);
	if (f_otg->host_working == 0) {
		dev_err(f_otg->dev, "switch_to_gadget_event(): no need.");
		mutex_unlock(&f_otg->role_switch_lock);
		return;
	}

	/*unregister Fujitsu hcd driver*/
	f_usb20hdc_hcd_remove(f_otg);
	f_otg->host_working = 0;

	/*do reset for device mode initial*/
	f_usb20hdc_soft_reset_ip(f_otg);

	/*resume Fujitsu udc driver*/
	if (f_udc_otg_resume(f_otg)) {
		dev_err(f_otg->dev, "%s() %d.", __func__, __LINE__);
		mutex_unlock(&f_otg->role_switch_lock);
		return;
	}

	mutex_unlock(&f_otg->role_switch_lock);
	dev_info(f_otg->dev, "switch_to_gadget_event() is ended.");
}

/*
 * Interrupt handler.  OTG/host/peripheral share the same int line.
 * OTG driver process only event of OTG id change event.
 */
static irqreturn_t otg_isr(int irq, void *dev_id)
{
	u8 id = 0;
	struct f_usb20hdc_otg *f_otg = (struct f_usb20hdc_otg *)dev_id;
	struct device *dev = f_otg->dev;

	dev_dbg(dev, "%s() is started [%p].\n", __func__, f_otg);

	if (!get_otg_int(f_otg->reg_base) ||
					!get_id_c(f_otg->reg_base)) {
		/*don't process events are not "OTG ID change"*/
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return IRQ_NONE;
	}

	/*only process the OTG ID change event*/
	id = get_id(f_otg->reg_base);
	dev_info(dev, "%s(): OTG ID changed.(id:%d)\n", __func__, id);
	clear_id_c(f_otg->reg_base);
	if (f_otg->mode == F_USB20HDC_MODE_DUAL_ROLE) {
		dev_info(dev, "%s(): execute role-switch.\n", __func__);
		switch (id) {
		case 0:
			/* host should be initialized as soon as possible*/
			cancel_delayed_work(&f_otg->switch_to_gadget);
			schedule_delayed_work(&f_otg->switch_to_host, 0);
			break;
		case 1:
			/* postpone gadget initial after host's disconnection*/
			cancel_delayed_work(&f_otg->switch_to_host);
			schedule_delayed_work(&f_otg->switch_to_gadget, 100);
			break;
		default:
			dev_err(dev, "%s(): OTG ID is invalid : %d\n",
				__func__, id);
			break;
		}
	}
	/* f_otg->otg_mode is for debugging purpose */
	f_otg->otg_id = id;
	dev_dbg(dev, "%s() is ended.\n", __func__);
	return IRQ_HANDLED;
}

static int vbus_open(struct inode *inode, struct file *file)
{
	struct f_usb20hdc_otg *f_otg = inode->i_private;

	dev_info(f_otg->dev, "%s()\n", __func__);

	file->private_data = f_otg;

	return 0;
}

static ssize_t vbus_read(struct file *file,
	char __user *user_buf, size_t count, loff_t *ppos)
{
	char buf[20];
	struct f_usb20hdc_otg *f_otg;

	f_otg = file->private_data;
	dev_info(f_otg->dev, "%s()\n", __func__);

	snprintf(buf, ARRAY_SIZE(buf),
		"VBUS is %d.\n", get_vbus_vld(f_otg->reg_base));

	return simple_read_from_buffer(user_buf, count, ppos,
		buf, strlen(buf)+1);
}

static ssize_t  vbus_write(struct file *file,
	const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[2];
	ssize_t ret = 0;
	struct f_usb20hdc_otg *f_otg;

	f_otg = file->private_data;
	dev_info(f_otg->dev, "%s()\n", __func__);

	if (count > sizeof(buf))
		return -EINVAL;

	ret = simple_write_to_buffer(buf,
		sizeof(buf), ppos, userbuf, count);

	switch (buf[0]) {
	case '0':
		/*set power 0*/
		enable_bus_power_supply(f_otg->reg_base, 0);
		 break;
	case '1':
		/*set power 1*/
		enable_bus_power_supply(f_otg->reg_base, 1);
		break;
	default:
		dev_err(f_otg->dev, "input is invalid!\n");
		break;
	}

	return ret;
}

static const struct file_operations vbus_control = {
	.open			= vbus_open,
	.read			= vbus_read,
	.write			= vbus_write,
};

static int f_usb20hdc_otg_clk_enable(struct f_usb20hdc_otg *f_usb20hdc)
{
	int i = 0;
	struct device *dev = f_usb20hdc->dev;

	if (dev == NULL) {
		pr_err("no device found.\n");
		return -1;
	}

	dev_info(dev, "%s() is started.\n", __func__);

	for (i = 0; i < CLK_MAX_NUM; i++) {
		f_usb20hdc->clk_table[i] = of_clk_get(dev->of_node, i);
		if (IS_ERR(f_usb20hdc->clk_table[i]) && (i == 0)) {
			dev_err(dev, "%s():no clock is found.\n", __func__);
			break;
		}
		if (!IS_ERR(f_usb20hdc->clk_table[i])) {
			clk_prepare_enable(f_usb20hdc->clk_table[i]);
			dev_info(dev, "%s():Clock[%d] is found.", __func__, i);
		}
	}
	dev_info(dev, "%s() is ended.\n", __func__);
	return 0;
}

static int f_usb20hdc_otg_clk_disable(struct f_usb20hdc_otg *f_otg)
{
	int clk_index;
	struct device *dev = f_otg->dev;

	dev_info(dev, "%s() is started.\n", __func__);

	if (dev == NULL) {
		dev_err(dev, "no device found.\n");
		return -1;
	}

	clk_index = CLK_MAX_NUM;
	while (clk_index--) {
		if (!IS_ERR(f_otg->clk_table[clk_index]))
			clk_disable_unprepare(f_otg->clk_table[clk_index]);
	}

	dev_info(dev, "%s() is ended.\n", __func__);
	return 0;
}

/* attach external DMA controller and allocate a DMA coherent buffer */
int f_usb20hdc_dma_attach(struct f_usb20hdc_dma_data *dma_data,
	int ch_num, int size)
{
	int count, ret;
	struct device *dev = g_f_otg->dev;


	ret = hdmac_register_dma_req_dev(CHIP_INDEX(F_USB20HDC_DMA_CH1), dev);
	if (ret) {
		dev_err(dev,
				"%s: failed to register as dma_req_dev\n",
				__func__);
		return ret;
	}

	for (count = F_USB20HDC_DMA_CH1; count < ch_num; count++) {
		/* allocate HDMAC channel for a F_USB20HDC DMAC device */
		ret = hdmac_get_channel(dma_data[count].hdmac_channel,
						HDMAC_AUTOSTART_DISABLE);
		if (ret) {
			dev_err(dev, "%s():ex-DMA channel %d attach failed at %d.\n",
				__func__, count, ret);
			return -ENOMEM;
		}

#if defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
		/* allocate noncachable buffer for the F_USB20HDC DMAC device */
		dma_data[count].buffer = dma_alloc_coherent(dev, size,
				&dma_data[count].dma_buffer, GFP_KERNEL);
		if (dma_data[count].buffer == NULL) {
			dev_err(dev, "%s():DMA channel %d dma_alloc_coherent() failed.\n",
					__func__, count);
			hdmac_free(dma_data[count].hdmac_channel);
			return -ENOMEM;
		}
#endif
	}

	return 0;
}

int f_usb20hdc_dma_detach(struct f_usb20hdc_dma_data *dma_data,
	int ch_num, int size)
{
	int count;
	int ret;

	struct device *dev = g_f_otg->dev;

	for (count = F_USB20HDC_DMA_CH1; count < ch_num; count++) {
		hdmac_free(dma_data[count].hdmac_channel);
#if defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
		dma_free_coherent(dev, size,
				dma_data[count].buffer,
				dma_data[count].dma_buffer);
#endif
	}

	ret = hdmac_unregister_dma_req_dev(CHIP_INDEX(F_USB20HDC_DMA_CH1), dev);
	if (ret) {
		dev_err(dev,
				"%s: failed to unregister as dma_req_dev\n",
				__func__);
		return ret;
	}

	return 0;
}

static int f_usb20hdc_otg_probe(struct platform_device *pdev)
{
	int irq;
	int result = 0;
	struct f_usb20hdc_otg *f_otg;
	struct device *dev;
	int index;
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	int count;
#endif

	/* check argument */
	if (unlikely(!pdev))
		return -EINVAL;

	dev = &pdev->dev;
	dev_info(dev, "F_USB20HDC OTG driver probe start.\n");

	/* allocate and save private driver data in global rather in dev*/
	f_otg = kzalloc(sizeof(struct f_usb20hdc_otg), GFP_KERNEL);
	if (f_otg == NULL) {
		dev_err(dev, "private driver data malloc failed!");
		return -EINVAL;
	}
	/*
	 * dev'sprivate_data has to be reserved for hcd,
	 * so we save otg's private in global.
	*/
	g_f_otg = f_otg;
	f_otg->dev = &pdev->dev;
	f_otg->pdev = pdev;

	f_otg->root = debugfs_create_dir(dev_name(&pdev->dev), NULL);
	if (!f_otg->root) {
		dev_err(&pdev->dev, "debugfs_create_dir fail\n");
		return -ENOMEM;
	}

	f_otg->file = debugfs_create_file("vbus_control", S_IRUGO | S_IWUGO
		, f_otg->root, f_otg, &vbus_control);
	if (!f_otg->file) {
		dev_err(&pdev->dev, "debugfs_create_file fail\n");
		return -ENOMEM;
	}


	if (of_property_read_u32(pdev->dev.of_node, "mode", &index)) {
		dev_info(&pdev->dev, "no mode selection in dt\n");
		index = -1;
	} else {
		dev_info(&pdev->dev, "mode %d selected in dt\n", index);
	}

	/* confiure mode based on choice on Kconfig or device tree blob */
	if (IS_ENABLED(CONFIG_USB_F_USB20HDC_OTG_HOST_ONLY)) {
		f_otg->mode = F_USB20HDC_MODE_HOST;
		dev_info(dev, "%s():select Host Mode in dts.\n", __func__);
	} else if (IS_ENABLED(CONFIG_USB_F_USB20HDC_OTG_GADGET_ONLY)) {
		f_otg->mode = F_USB20HDC_MODE_DEVICE;
		dev_info(dev, "%s():select Device Mode in dts.\n", __func__);
	} else if (IS_ENABLED(CONFIG_USB_F_USB20HDC_OTG_DUAL_ROLE)) {
		f_otg->mode = index == -1 ?
			F_USB20HDC_MODE_DUAL_ROLE :
			index;
		dev_info(dev, "%s():select Dual Mode in dts:%d.\n", __func__, f_otg->mode);
	} else {
		f_otg->mode = -1;
		dev_err(dev, "%s():invalid mode select.\n", __func__);
		goto err_clk;
	}

	/* get a resource for a F_USB20HDC device */
	f_otg->mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!f_otg->mem_res) {
		dev_err(dev, "%s():platform_get_resource() failed.\n",
								__func__);
		result = -ENODEV;
		goto err_res;
	}
	f_otg->mem_size = f_otg->mem_res->end - f_otg->mem_res->start + 1;
	f_otg->mem_start = f_otg->mem_res->start;

	/* get a register base address for a F_USB20HDC device */
	f_otg->reg_base = remap_iomem_region(f_otg->mem_start,
		f_otg->mem_size);
	if (!f_otg->reg_base) {
		dev_err(dev, "%s():remap_iomem_region() failed.\n",
								__func__);
		result = -ENODEV;
		goto err_res;
	}

	/* get an IRQ for a F_USB20HDC device */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev,
			"%s():platform_get_irq() for F_USB20HDC failed at %d.\n",
			__func__, irq);
		result = -ENODEV;
		goto err_irq;
	}

	result = request_irq(irq, otg_isr, IRQF_SHARED,
						"f_usb20hdc_otg", f_otg);
	if (result) {
		dev_err(dev,
			"%s():request_irq() for F_USB20HDC is failed at %d.\n",
							      __func__, result);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		goto err_irq;
	}
	disable_irq(irq);

#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	if (pdev->dev.platform_data == NULL) {
		const int *p;
		int len;
		if (pdev->dev.of_node) {

			p = of_get_property(pdev->dev.of_node,
							      "dma_dreq", &len);
			if (!p || len != 2 * sizeof(int)) {
				result = -EINVAL;
				goto err_clk;
			}
			f_otg->pdata.dma_dreq[0] = be32_to_cpu(*p++);
			f_otg->pdata.dma_dreq[1] = be32_to_cpu(*p++);
			dev_info(dev, "%s(): dma_dreq[0]:0x%x, dma_dreq[1]:0x%x.\n",
				__func__, f_otg->pdata.dma_dreq[0],
				f_otg->pdata.dma_dreq[1]);

			p = of_get_property(pdev->dev.of_node,
							"hdmac_channel", &len);
			if (!p || len != 2 * sizeof(int)) {
				result = -EINVAL;
				goto err_clk;
			}
			f_otg->pdata.hdmac_channel[0] = be32_to_cpu(*p++);
			f_otg->pdata.hdmac_channel[1] = be32_to_cpu(*p++);
		} else {
			dev_err(dev, "%s():DMA platform data invalid.\n",
								__func__);
			result = -EINVAL;
			goto err_clk;
		}
	} else
		f_otg->pdata = *(struct f_usb20hdc_pdata *)
						pdev->dev.platform_data;

	/* prepare DMA data for hcd and udc */
	for (count = F_USB20HDC_DMA_CH1;
		count < F_USB20HDC_MAX_DMA_CHANNELS; count++) {
		/* initialize F_USB20HDC Host DMA device data */
		f_otg->dma_data[count].dreq = f_otg->pdata.dma_dreq[count];
		dev_info(dev, "%s(): dma_dreq[%d]:0x%x.\n", __func__,
			count, f_otg->pdata.dma_dreq[count]);
		f_otg->dma_data[count].hdmac_channel = f_otg->pdata.hdmac_channel[count];
		f_otg->dma_data[count].buffer = NULL;
		f_otg->dma_data[count].dma_buffer = ~(dma_addr_t)0;
		f_otg->dma_data[count].epbuf_dma_addr = get_epbuf_dma_address(
						f_otg->mem_start, count);
	}
#endif

	/* initial role-switch lock */
	mutex_init(&f_otg->role_switch_lock);

	/*driver private data initialization*/
	f_otg->reg_base = f_otg->reg_base;
	f_otg->irq = irq;
	f_otg->host_working = 0;
	f_otg->hcd = NULL;
	f_otg->f_usb20hdc_udc = NULL;
	INIT_DELAYED_WORK(&f_otg->switch_to_host, switch_to_host_event);
	INIT_DELAYED_WORK(&f_otg->switch_to_gadget, switch_to_gadget_event);

	/* prevent role-switch from otg isr at the same time */
	mutex_lock(&f_otg->role_switch_lock);

	/* enable power, clock, IRQ */
	pm_runtime_enable(&pdev->dev);
	result = pm_runtime_get_sync(&pdev->dev);
	if (result < 0) {
		dev_err(dev, "get_sync failed with err %d\n", result);
		goto err_irq;
	}

	/* reset controller for hcd and udc driver */
	clear_id_c(f_otg->reg_base);
	f_usb20hdc_soft_reset_ip(f_otg);

	/* initialize f_usb20hdc dual-role usb driver */
	switch (f_otg->mode) {
	case F_USB20HDC_MODE_HOST:
		result = f_usb20hdc_hcd_probe(f_otg);
		if (!result)
			f_otg->host_working = 1;
		break;
	case F_USB20HDC_MODE_DEVICE:
	case F_USB20HDC_MODE_DUAL_ROLE:
		result = f_usb20hdc_udc_probe(f_otg);
		break;
	default:
		result = -1;
		dev_err(dev, "invalid mode!\n");
		break;
	}
	if (!result)
		dev_info(dev, "successfully register Fujitsu usb 2.0 dual-role module!\n");
	else {
		mutex_unlock(&f_otg->role_switch_lock);
		dev_err(dev, "Failed to register Fujitsu usb 2.0 dual-role module!\n");
		result = -ENODEV;
		goto err_irq;
	}

	device_set_wakeup_capable(dev, 1);

	/* start role-switching first time */
	if (f_otg->mode == F_USB20HDC_MODE_DUAL_ROLE) {
		switch (get_id(f_otg->reg_base)) {
		case 0: /* ID 0 means Host mode choosed by OTG cable */
			if (f_udc_otg_suspend(f_otg) != 0) {
				result = -ENODEV;
				break;
			}
			if (!f_usb20hdc_hcd_probe(f_otg)) {
				f_otg->host_working = 1;
				dev_dbg(dev, "host mode is activated!\n");
				result = 0;
			} else
				result = -ENODEV;
			break;
		case 1:  /* ID 1 means Device mode choosed by OTG cable */
			result = 0;
			dev_dbg(dev, "device mode is still activated!\n");
			break;
		default:
			result = -ENODEV;
			break;

		}
	}
	if (result != 0) {
		mutex_unlock(&f_otg->role_switch_lock);
		dev_err(dev, "failed to switch role!\n");
		goto err_irq;
	}

	/* now we could let otg isr to do whatever he wants  */
	mutex_unlock(&f_otg->role_switch_lock);

	dev_info(dev, "F_USB20HDC OTG driver is registered.\n");
	return 0;

err_irq:
	unmap_iomem_region(f_otg->reg_base);
err_res:
	f_usb20hdc_otg_clk_disable(f_otg);
err_clk:
	kfree(f_otg);
	dev_err(dev, "F_USB20HDC OTG driver is failed to be registered.\n");
	return result;
}

#if defined(CONFIG_USB_F_USB20HDC_OTG_MODULE)
static int f_usb20hdc_otg_remove(struct platform_device *pdev)
{
	struct f_usb20hdc_otg *f_otg = g_f_otg;
	struct device *dev = &pdev->dev;

	dev_info(dev, "F_USB20HDC OTG driver remove start.\n");

	/* check argument */
	if (unlikely(!pdev))
		return -EINVAL;

	dev_dbg(dev, "%s() is started.\n", __func__);

	disable_irq(f_otg->irq);
	free_irq(f_otg->irq, f_otg);

	/* get a device driver parameter */
	if (!f_otg) {
		dev_err(dev, "%s():platform_get_drvdata() is failed.\n",
								__func__);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return -EINVAL;
	}

	device_set_wakeup_capable(dev, 0);

	/* turn off irq first which might schedule new work*/
	cancel_delayed_work(&f_otg->switch_to_host);
	cancel_delayed_work(&f_otg->switch_to_gadget);

	/* deinit gadget and hcd driver before disabling clock*/
	if (f_otg->mode != F_USB20HDC_MODE_HOST)
		f_usb20hdc_udc_remove(f_otg);
	if (f_otg->host_working)
		f_usb20hdc_hcd_remove(f_otg);

	/*inform suspend handler don't do udc, hcd suspend*/
	f_otg->host_working = 0;

	/*disable power, clock, irq, and suspend hcd, udc */
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	/* remove debugfs after revome hcd & udc */
	debugfs_remove(f_otg->file);
	debugfs_remove(f_otg->root);

	unmap_iomem_region(f_otg->reg_base);
	kfree(f_otg);

	dev_info(dev, "F_USB20HDC OTG driver remove end.\n");
	return 0;
}
#endif

#ifdef CONFIG_PM
#ifdef CONFIG_PM_RUNTIME

#ifdef CONFIG_USB_F_USB20HDC_OTG_USE_WAKEUP
static int f_usb20hdc_otg_ucdwakeup_suspend(struct device *dev)
{
	struct f_usb20hdc_otg *f_otg = g_f_otg;
	int i;
	struct hdmac_req _hdmac_req = {.dmaca = 0, .dmacb = 0};

	disable_irq(f_otg->irq);

	for (i = 0; i < F_USB20HDC_MAX_DMA_CHANNELS; i++) {
		/* clear HDMACA_IS_MASK */
		_hdmac_req.dmaca = HDMACA_IS_MASK;
		hdmac_set_chan_dmac(f_otg->dma_data[i].hdmac_channel,
			&_hdmac_req, CLR_DMA_CHAN);

		/* set HDMACA_IS_SW */
		_hdmac_req.dmaca = HDMACA_IS_SW;
		hdmac_set_chan_dmac(f_otg->dma_data[i].hdmac_channel,
			&_hdmac_req, SET_DMA_CHAN);
	}

	return f_usb20hdc_udc_suspend(f_otg);

}
#endif

static int f_usb20hdc_otg_runtime_suspend(struct device *dev)
{
	struct f_usb20hdc_otg *f_otg = g_f_otg;

	if (f_otg->host_working)
		f_usb20hdc_hcd_suspend(f_otg);
	if (f_otg->mode != F_USB20HDC_MODE_HOST)
		f_usb20hdc_udc_suspend(f_otg);

	/* disable OTG ID interrupt factor */
	set_id_ren(f_otg->reg_base, 0);
	set_id_fen(f_otg->reg_base, 0);
	clear_id_c(f_otg->reg_base);

	/* disable clock and interrupt */
	disable_irq(f_otg->irq);
	f_usb20hdc_otg_clk_disable(f_otg);

	return 0;
}

#ifdef CONFIG_USB_F_USB20HDC_OTG_USE_WAKEUP
static int f_usb20hdc_otg_ucdwakeup_resume(struct device *dev)
{
	struct f_usb20hdc_otg *f_otg = g_f_otg;
	int i;
	int ret = 0;
	struct hdmac_req _hdmac_req = {.dmaca = 0, .dmacb = 0};

	for (i = 0; i < F_USB20HDC_MAX_DMA_CHANNELS; i++) {
		/* clear HDMACA_IS_SW */
		_hdmac_req.dmaca = HDMACA_IS_MASK;
		hdmac_set_chan_dmac(f_otg->dma_data[i].hdmac_channel,
			&_hdmac_req, CLR_DMA_CHAN);

		/* set HDMACA_IS_DERQH */
		_hdmac_req.dmaca = HDMACA_IS_DERQH;
		hdmac_set_chan_dmac(f_otg->dma_data[i].hdmac_channel,
			&_hdmac_req, SET_DMA_CHAN);
	}

	ret = f_usb20hdc_udc_resume(f_otg);

	enable_irq(f_otg->irq);

	return ret;
}
#endif

static int f_usb20hdc_otg_runtime_resume(struct device *dev)
{
	struct f_usb20hdc_otg *f_otg = g_f_otg;

	dev_info(dev, "%s() is started.\n", __func__);

	/* enable clock and interrupt will be enabled last */
	f_usb20hdc_otg_clk_enable(f_otg);

#ifdef COLD_RESUME_SUPPORT
	/* recovery from power-off or hibernation */
	f_usb20hdc_soft_reset_ip(f_otg);
#else
	set_id_ren(f_otg->reg_base, 1);
	set_id_fen(f_otg->reg_base, 1);
#endif

	if (f_otg->host_working)
		f_usb20hdc_hcd_resume(f_otg);
	if (f_otg->mode != F_USB20HDC_MODE_HOST)
		f_usb20hdc_udc_resume(f_otg);

	/*
	 * enable interrupt safely at last, after register
	 * been recovered from power-off or hibernation.
	 */
	enable_irq(f_otg->irq);

	return 0;
}
#endif

static int f_usb20hdc_otg_suspend(struct device *dev)
{
	struct f_usb20hdc_otg *f_otg = g_f_otg;

	if (pm_runtime_status_suspended(dev))
		return 0;

	if ((f_otg->mode != F_USB20HDC_MODE_HOST) &&
		device_may_wakeup(dev))
		return f_usb20hdc_otg_ucdwakeup_suspend(dev);

	return f_usb20hdc_otg_runtime_suspend(dev);
}

static int f_usb20hdc_otg_resume(struct device *dev)
{
	struct f_usb20hdc_otg *f_otg = g_f_otg;

	if (pm_runtime_status_suspended(dev))
		return 0;

	if ((f_otg->mode != F_USB20HDC_MODE_HOST) &&
		device_may_wakeup(dev))
		return f_usb20hdc_otg_ucdwakeup_resume(dev);

	return f_usb20hdc_otg_runtime_resume(dev);
}

int power_off_simulation(struct device *dev)
{
#ifdef POWER_OFF_SIMULATION
	struct f_usb20hdc_otg *f_otg = g_f_otg;
	void __iomem *addr = f_otg->reg_base;
	void __iomem *end_addr = f_otg->reg_base + (0x10000);

	/* make sure irq disable already here */
	f_usb20hdc_otg_clk_enable(f_otg);
	while (addr != end_addr) {
		__raw_writel(0xffffffff, addr);
		addr += 4;
	}
	f_usb20hdc_otg_clk_disable(f_otg);
#endif
	return 0;
}

int power_on_simulation(struct device *dev)
{
#ifdef POWER_OFF_SIMULATION
	dev_info(dev, "f_usb20hdc_otg_resume_no_irq() is executed.\n");
#endif
	return 0;
}

static const struct dev_pm_ops f_usb20hdc_otg_ops = {
	.suspend = f_usb20hdc_otg_suspend,
	.resume = f_usb20hdc_otg_resume,
	.suspend_noirq = power_off_simulation,
	.resume_noirq = power_on_simulation,
	SET_RUNTIME_PM_OPS(f_usb20hdc_otg_runtime_suspend,
		f_usb20hdc_otg_runtime_resume, NULL)
};

#define _f_usb20hdc_otg_ops (&f_usb20hdc_otg_ops)
#else
#define _f_usb20hdc_otg_ops NULL
#endif /* CONFIG_PM */

static const struct of_device_id f_usb20hdc_otg_dt_ids[] = {
	{ .compatible = "fujitsu,f_usb20hdc_otg" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, f_usb20hdc_otg_dt_ids);

struct platform_driver f_usb20hdc_otg_driver = {
	.probe = f_usb20hdc_otg_probe,
	.remove = __exit_p(f_usb20hdc_otg_remove),
	.driver = {
		.name = "f_usb20hdc_otg",
		.owner = THIS_MODULE,
		.pm = _f_usb20hdc_otg_ops,
		.of_match_table = f_usb20hdc_otg_dt_ids,
	},
};

static int otg_init(void)
{
	pr_info("f_usb20hdc_otg module init start.\n");

	return platform_driver_register(&f_usb20hdc_otg_driver);
}
module_init(otg_init);

/* F_USB20HDC OTG device exit */
static void __exit otg_exit(void)
{
	platform_driver_unregister(&f_usb20hdc_otg_driver);

	pr_info("f_usb20hdc_otg module exit.\n");
}
module_exit(otg_exit);

/* F_USB20HDC OTG device module definition */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Fujitsu Semiconductor Limited");
MODULE_DESCRIPTION("F_USB20HDC USB OTG controller driver");
MODULE_ALIAS("platform:f_usb20hdc_otg");

