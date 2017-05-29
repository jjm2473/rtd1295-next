/*
 * mipilli_mb86s70.c F_MIPILLI_LP Controller Driver
 * Copyright (C) 2013 Fujitsu Semi, Ltd
 * Author: Slash Huang <slash.huang@tw.fujitsu.com>
 *
 *   This code is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This code is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/configfs.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <asm/irq.h>
#include <linux/timer.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/uaccess.h>
#include "mipilli_mb86s70.h"

struct lli_dev *gp_mb86s70_lli_dev;
u32 *glli_major;
u32 glli_num;


void en_lli_chan(u8 num)
{
	gp_mb86s70_lli_dev->act_lli_ch = num;
}


u32 lli_get_driver_version(struct lli_dev *dev_lli)
{
	return dev_lli->lli_drv_ver;
}

void init_mb86s70_lli_reg(struct lli_dev *dev_lli, u8 num)
{
	struct lli_data *lli_ch = NULL;

	dev_lli->act_lli_ch = num;
	lli_ch = &dev_lli->chan_data[num];
	set_lo_cfg(lli_ch->lli_defcfg);
}



static irqreturn_t lli_irq(int irqno, void *dev_id)
{
	s16 i = 0;
	struct lli_dev *dev_lli = (struct lli_dev *)dev_id;
	struct lli_data *lli_ch = NULL;
	u32 status = 0;
	unsigned long flags = 0;
	/* int ch_id = 0; */

	while ((irqno != dev_lli->chan_data[i].irq) && (i < glli_num)) {
		i++;
	};

	if (i >= glli_num) {
		DBG_MSG("lli_irq Err:irqno=%d\n", irqno);
		spin_unlock_irqrestore(&lli_ch->mb86s70_irq_lock, flags);
		return IRQ_HANDLED;
	}

	en_lli_chan(i);
	lli_ch = &dev_lli->chan_data[i];

	/* ch_id = MAJOR(lli_ch->devt); */
	spin_lock_irqsave(&lli_ch->mb86s70_irq_lock, flags);
	status = lli_lo_rd(MIPI_USER_CTRL_INTST);
	lli_lo_wt(MIPI_USER_CTRL_INTCLR, status);

	if (status & (B_SIGSET | B_SEQERR | B_CRCERR |
			B_PHYSYMERR | B_FIFOOVF |
			B_CRSTAST | B_RSTONERRDET |
			B_MNTFTlERR)) {

		#if DBG_FLAG

		if (status & B_SIGSET)
			DBG_MSG("B_SIGSET\n");

		if (status & B_SEQERR)
			DBG_MSG("SEQ Error\n");

		if (status & B_CRCERR)
			DBG_MSG("CRC Error\n");

		if (status & B_PHYSYMERR)
			DBG_MSG("PHY SYMBOL Error\n");

		if (status & B_FIFOOVF)
			DBG_MSG("FIFO of Data Link Layer is OverFlow\n");

		if (status & B_CRSTAST)
			DBG_MSG("Cool Reset\n");

		if (status & B_RSTONERRDET)
			DBG_MSG("Reset On Error\n");

		if (status & B_MNTFTlERR)
			DBG_MSG("Mount Fail\n");

		#endif
	} else {
		complete(&lli_ch->msg_cmp);
	}

	spin_unlock_irqrestore(&lli_ch->mb86s70_irq_lock, flags);

	return IRQ_HANDLED;
}

static int
lli_open(struct inode *inode, struct file *file)
{
	int status = -1;
	int i = 0;
	int lli_major_num = 0;
	struct lli_data *lli_ch = NULL;

	file->private_data = (struct lli_dev *)gp_mb86s70_lli_dev;
	gp_mb86s70_lli_dev->cur_lli_major_num = imajor(inode);
	lli_major_num = imajor(inode);

	while ((lli_major_num != glli_major[i]) && (i < glli_num)) {
		i++;
	};

	if (i >= glli_num)
		return -1;

	en_lli_chan(i);
	lli_ch = &gp_mb86s70_lli_dev->chan_data[i];

	if (down_trylock(&(lli_ch->openf_sem)) == 0)
		status = 0;

	return status;
}

static int
lli_release(struct inode *inode, struct file *file)
{
	s8 status = -1;
	s8 i = 0;
	u32 lli_major_num;
	struct lli_data *lli_ch = NULL;
	struct lli_dev *dev_lli = NULL;

	dev_lli = (struct lli_dev *)file->private_data;
	lli_major_num = imajor(inode);

	while ((lli_major_num != glli_major[i]) && (i < glli_num)) {
		i++;
	};

	if (i >= glli_num)
		return -1;

	en_lli_chan(i);
	lli_ch = &gp_mb86s70_lli_dev->chan_data[i];


	up(&(lli_ch->openf_sem));

	status = 0;
	return status;
}

static ssize_t
lli_mmap(struct file *f, struct vm_area_struct *vma)
{
	int r = 0, i = 0;
	int lli_major_num = 0;
	u64 lli_mem_addr = 0;
	struct lli_dev *dev_lli;


	dev_lli = (struct lli_dev *)f->private_data;
	lli_major_num = dev_lli->cur_lli_major_num;

	while ((lli_major_num != glli_major[i]) && (i < glli_num)) {
		i++;
	};

	if (i >= glli_num)
		return -1;

	en_lli_chan(i);

	lli_mem_addr = LLI0_MEM_ADDR + LLI_MEM_OFF * i;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	r = remap_pfn_range(vma, vma->vm_start,
	    __phys_to_pfn(lli_mem_addr),
	    vma->vm_end-vma->vm_start,
	    vma->vm_page_prot);

	if (r < 0) {
		ERR_MSG("lli_mmap:Error->lli_mmap error\n");
		return -1;
	}

	return 0;
}


static
ssize_t lli_write(struct file *file,
			const char __user *buf, size_t count, loff_t *ppos)
{
	s8 status = 0, i = 0;
	u16 lli_major_num;
	struct lli_dev *dev_lli;

	dev_lli = (struct lli_dev *)file->private_data;
	lli_major_num = dev_lli->cur_lli_major_num;

	while ((lli_major_num != glli_major[i]) && (i < glli_num)) {
		i++;
	};

	if (i >= glli_num)
		return -1;

	en_lli_chan(i);

	if (down_trylock(&(dev_lli->buf_sem)) != 0) {
		ERR_MSG("lli_write:Error down_trylock fail\n");
		status = -1;
	}

	up(&(dev_lli->buf_sem));

	return status;
}




static ssize_t
lli_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	s8 status = 0, i = 0;
	u16 lli_major_num;
	struct lli_dev *dev_lli;

	dev_lli = (struct lli_dev *)file->private_data;
	lli_major_num = dev_lli->cur_lli_major_num;

	while ((lli_major_num != glli_major[i]) && (i < glli_num)) {
		i++;
	};

	if (i >= glli_num)
		return -1;

	en_lli_chan(i);

	if (down_trylock(&(dev_lli->buf_sem)) != 0) {
		ERR_MSG("lli_read:Error down_trylock fail\n");
		status = -1;
	}

	up(&(dev_lli->buf_sem));
	return status;
}



static long lli_ioctl(struct file *filp,
				unsigned cmd, unsigned long arg)
{
	u32 lli_major_num;
	struct ioctl_reg ioctl_iil_reg;
	struct ioctl_memmap ioctl_iil_memap;
	struct ioctl_cfg arg_lli_cfg;
	struct lli_data *the_lli;
	struct lli_dev *dev_lli;
	s8 r = 0, i = 0;
	u32 drv_ver;

	dev_lli = (struct lli_dev *)filp->private_data;
	lli_major_num = dev_lli->cur_lli_major_num;

	while ((lli_major_num != glli_major[i]) && (i < glli_num)) {
		i++;
	};
	if (i >= glli_num)
		return -1;

	en_lli_chan(i);
	the_lli = &dev_lli->chan_data[ACT_LLI_CH];

	switch (cmd) {

	case LLI_IOCTL_M_MOUNT: {

		#ifdef DBG_IO_CTL
		DBG_MSG("LLI_IOCTL_M_MOUNT\n");
		#endif

		the_lli->master_or_slave = AS_MASTER;

		if (the_lli->mount_stat == true) {
			#ifdef DBG_IO_CTL
			DBG_MSG("%s had mounted\n", the_lli->lli_chan_name);
			#endif
			return 1;
		}

		return mipilli_master_mount(ACT_LLI_CH);
	}

	case LLI_IOCTL_M_UNMOUNT: {

		#ifdef DBG_IO_CTL
		DBG_MSG("LLI_IOCTL_M_UNMOUNT\n");
		#endif

		if (the_lli->mount_stat == false) {
			#ifdef DBG_IO_CTL
			DBG_MSG("%s had un-mounted\n",
			the_lli->lli_chan_name);
			#endif
			return 1;
		}

		return mipilli_master_unmount(ACT_LLI_CH);
	}

	case LLI_IOCTL_S_MOUNT: {

		#ifdef DBG_IO_CTL
		DBG_MSG("LLI_IOCTL_S_MOUNT\n");
		#endif

		the_lli->master_or_slave = AS_SLAVE;

		if (the_lli->mount_stat == true) {
			#ifdef DBG_IO_CTL
			DBG_MSG("%s had mounted\n",
			the_lli->lli_chan_name);
			#endif
			return 1;
		}

		return mipilli_slave_mount(/*the_lli*/ACT_LLI_CH);
	}

	/*
	case LLI_IOCTL_S_MAIN_LOOP:{
		return 0;
	}

	case LLI_IOCTL_SET_LOCAL_CONFIG:{
		DBG_MSG("LLI_IOCTL_SET_LOCAL_CONFIG\n");
		return 0;
	}
	*/
	case LLI_IOCTL_CONFIG_UPDATE: {

		#ifdef DBG_IO_CTL
		DBG_MSG("LLI_IOCTL_CONFIG_UPDATE\n");
		#endif

		if (the_lli->mount_stat == false) {
			#ifdef DBG_IO_CTL
			ERR_MSG("%s had no mounted\n",
			the_lli->lli_chan_name);
			#endif
			return -1;
		}

		if (the_lli->master_or_slave == AS_SLAVE) {
			#ifdef DBG_IO_CTL
			ERR_MSG("This channel is SLAVE\n");
			#endif
			return -1;
		}

		r = copy_from_user(&arg_lli_cfg,
				(struct ioctl_cfg __user *)arg,
				sizeof(struct ioctl_cfg));
		if (r)
			return -1;

		/* read configure to user */
		if (arg_lli_cfg.direct == 0) {
			memcpy(&arg_lli_cfg.lli_defcfg,
			    the_lli->lli_defcfg, sizeof(struct lli_cfg));

			r = copy_to_user((struct ioctl_cfg __user *)arg,
			    &arg_lli_cfg,
			    sizeof(struct ioctl_cfg));
			if (r)
				return -1;

			return 0;

		} else {/* write configure to mipi-lli */
			memcpy(the_lli->lli_defcfg, &arg_lli_cfg.lli_defcfg,
				sizeof(struct lli_cfg));

			r = mipilli_config_update(ACT_LLI_CH, TX_LANE_CNT,
					RX_LANE_CNT);
			if (r < 0)
				return -1;
			else
				return 0;
		}

		r = mipilli_config_update(ACT_LLI_CH, TX_LANE_CNT, RX_LANE_CNT);
		if (r < 0)
			return -1;
		else
			return 0;
	}

	case LLI_IOCTL_CHANGE_MEMMAP: {

		#ifdef DBG_IO_CTL
		DBG_MSG("LLI_IOCTL_CHANGE_MEMMAP\n");
		DBG_MSG("fromAddr=0x%x\n", ioctl_iil_memap.fromAddr);
		DBG_MSG("toAddr=0x%x\n", ioctl_iil_memap.toAddr);
		DBG_MSG("size=0x%x\n", ioctl_iil_memap.size);
		#endif
		r = copy_from_user(&ioctl_iil_memap,
			    (struct ioctl_memmap __user *)arg,
			    sizeof(struct ioctl_memmap));
		if (r)
			return -1;

		return mipilli_change_memmap(ioctl_iil_memap.fromAddr,
				    ioctl_iil_memap.toAddr,
				    ioctl_iil_memap.size);
	}

	/*
	case LLI_IOCTL_SET_REMOTE_SIG:{
		signal_set_val = arg;
		return mipilli_write_signal(signal_set_val);
	}
	*/

	case LLI_IOCTL_WRITE_REG: {

		#ifdef DBG_IO_CTL
		DBG_MSG("LLI_IOCTL_WRITE_REG\n");
		#endif

		r = copy_from_user(&ioctl_iil_reg,
			    (struct ioctl_reg __user *)arg,
			    sizeof(struct ioctl_reg));

		if (r > 0)
			return -1;

		if ((ioctl_iil_reg.reg_addr & LOCAL_ADDR_BIT))
			lli_lo_wt(ioctl_iil_reg.reg_addr,
			    ioctl_iil_reg.reg_data);
		else {
			if (the_lli->mount_stat == false) {
				DBG_MSG("%s no mounte\n",
				the_lli->lli_chan_name);
				return -1;
			}
			lli_rm_wt(ioctl_iil_reg.reg_addr,
			    ioctl_iil_reg.reg_data);
		}

		return 0;
	}

	case LLI_IOCTL_READ_REG: {

		#ifdef DBG_IO_CTL
		DBG_MSG("LLI_IOCTL_READ_REG\n");
		#endif

		r = copy_from_user(&ioctl_iil_reg,
			    (struct ioctl_reg __user *)arg,
			    sizeof(struct ioctl_reg));

		if (r > 0)
			return -1;

		if ((ioctl_iil_reg.reg_addr & LOCAL_ADDR_BIT)) {

			#ifdef DBG_IO_CTL
			DBG_MSG(
			    "ReadReg->Local reg_addr=0x%x reg_data=0x%x\n",
			    ioctl_iil_reg.reg_addr,
			    ioctl_iil_reg.reg_data);
			#endif

			ioctl_iil_reg.reg_data =
			    lli_lo_rd(ioctl_iil_reg.reg_addr);

		} else {

			if (the_lli->mount_stat == false) {
				DBG_MSG("%s had mounted\n",
				the_lli->lli_chan_name);
				return -1;
			}

			ioctl_iil_reg.reg_data =
			    lli_rm_rd(ioctl_iil_reg.reg_addr);

		}

		r = copy_to_user((struct ioctl_reg __user *)arg,
			    &ioctl_iil_reg,
			    sizeof(struct ioctl_reg));
		if (r > 0)
			return -1;

		return 0;
	}

	case LLI_IOCTL_GET_VERSION: {
		#ifdef DBG_IO_CTL
		DBG_MSG("LLI_IOCTL_GET_VERSION\n");
		#endif
		r = copy_from_user(&drv_ver,
			    (unsigned long __user *)arg,
			    sizeof(unsigned long));
		if (r > 0)
			return -1;

		drv_ver = lli_get_driver_version(dev_lli);

		r = copy_to_user((unsigned long __user *)arg,
			    &drv_ver, sizeof(unsigned long));
		if (r > 0)
			return -1;

		return 0;
	}

	default:
		break;

	}

	return -EINVAL;
}



const struct file_operations lli_fops = {
	.owner = THIS_MODULE,
	.write = lli_write,
	.read = lli_read,
	.open = lli_open,
	.release = lli_release,
	.mmap = lli_mmap,
	.unlocked_ioctl = lli_ioctl,
};


static int lli_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource lli_plat_res;
	struct resource *lli_io_area;
	struct lli_dev *mb86s70_lli_dev = NULL;
	struct device_node *dev_np;

	dev_t devt;
	void __iomem *regs;
	int ret = 0, i = 0;
	int *irq_num = NULL;
	int r = -ENODEV;
	char class_name[10];
	char driver_name[10];
	char chardev_name[10];
	char dev_node_n[10];
	int clk_num = 0;
	struct clk *lli_clk;
	const char *irq_name;

	while (1) {
		sprintf(dev_node_n, "mipilli%d", i);
		/* printk("device_name=%s\n", dev_node_n); */
		dev_np = of_find_node_by_name(pdev->dev.of_node, dev_node_n);
		if (!dev_np)
			break;
		i++;
	}

	if (i <= 0) {
		ERR_MSG("can not find node in device tree\n");
		goto out_err;
	}

	glli_num = i;

	mb86s70_lli_dev =
	    devm_kzalloc(dev, sizeof(struct lli_dev), GFP_KERNEL);

	if (mb86s70_lli_dev == NULL) {
		ERR_MSG("lli_probe:mb86s70_lli_dev create fail\n");
		return -ENOMEM;
	}

	gp_mb86s70_lli_dev = mb86s70_lli_dev;
	mb86s70_lli_dev->lli_drv_ver = MB86S70_DRV_VER;

	mb86s70_lli_dev->chan_data =
	    devm_kzalloc(dev, glli_num * sizeof(struct lli_data),
		    GFP_KERNEL);

	irq_num =
		    devm_kzalloc(dev, glli_num * sizeof(int), GFP_KERNEL);

	for (i = 0; i < glli_num; i++) {
		mb86s70_lli_dev->chan_data[i].dev = NULL;
		mb86s70_lli_dev->chan_data[i].regs = NULL;
		mb86s70_lli_dev->chan_data[i].lli_class = NULL;
		mb86s70_lli_dev->chan_data[i].lli_defcfg = NULL;
		mb86s70_lli_dev->chan_data[i].ptr_slave_th = NULL;
		mb86s70_lli_dev->chan_data[i].mount_stat = false;
		mb86s70_lli_dev->chan_data[i].master_or_slave = AS_MASTER;
	}

	glli_major = devm_kzalloc(dev, glli_num * sizeof(u32),
				    GFP_KERNEL);


	/* ----------- Resource -------------- */
	for (i = 0; i < glli_num; i++) {
		sprintf(dev_node_n, "mipilli%d", i);
		dev_np = of_find_node_by_name(pdev->dev.of_node, dev_node_n);

		/* memery resource */
		if (of_address_to_resource(dev_np, 0, &lli_plat_res)) {
			ERR_MSG("%s can not get resource\n", dev_node_n);
		} else {
			lli_io_area =
			    request_mem_region(lli_plat_res.start,
				    lli_plat_res.end - lli_plat_res.start + 1,
				    pdev->name);

			if (lli_io_area == NULL) {
				ERR_MSG("%s cannot request I/O\n", dev_node_n);
				return -ENOMEM;
			}
			mb86s70_lli_dev->chan_data[i].rcs = lli_io_area;

			regs = ioremap(lli_plat_res.start,
				    lli_plat_res.end - lli_plat_res.start + 1);

			if (!regs) {
				ERR_MSG("%s ioremap fail\n", dev_node_n);
				return -ENOMEM;
			}

			mb86s70_lli_dev->chan_data[i].regs = regs;
		}

		/* irq resource */
		irq_num[i] = of_irq_to_resource(dev_np, 0, NULL);
		if (irq_num[i] <= 0) {
			ERR_MSG("%s ioremap fail\n", dev_node_n);
			ret = -EINVAL;
			goto out_err_2;
		}

		mb86s70_lli_dev->chan_data[i].irq = irq_num[i];

		of_property_read_string(dev_np, "irq-name", &irq_name);

		/* find the IRQ for this unit */
		r = request_irq(mb86s70_lli_dev->chan_data[i].irq,
			    lli_irq, IRQF_TRIGGER_RISING/*IRQF_SHARED*/,
			    irq_name, mb86s70_lli_dev);
		if (r != 0) {
			ERR_MSG("lli:request_irq [%d] fail\n", i);
			ret = -EINVAL;
			goto out_err_2;
		}

		/* clock */
		for (clk_num = 0; clk_num < LLI_CLK_NUM; clk_num++) {
			lli_clk = of_clk_get(dev_np, clk_num);
			if (IS_ERR(lli_clk)) {
				dev_err(&pdev->dev,
					"get clk err %d\n", clk_num);
				goto out_err_2;
			}
			ret = clk_prepare_enable(lli_clk);
			if (ret < 0) {
				dev_err(&pdev->dev,
					"enable clk err %d\n", clk_num);
				goto out_err_2;
			}
			mb86s70_lli_dev->chan_data[i].clocks[clk_num] = lli_clk;
		}
		mb86s70_lli_dev->chan_data[i].dev = dev;
	}

	/* -------- Char I/F Init -------------- */
	for (i = 0; i < glli_num; i++) {

		sprintf(class_name, "lli-%d", i);

		sprintf(mb86s70_lli_dev->chan_data[i].lli_chan_name, "%s",
		    class_name);

		mb86s70_lli_dev->chan_data[i].lli_class =
		    class_create(THIS_MODULE, class_name);

		if (IS_ERR(mb86s70_lli_dev->chan_data[i].lli_class)) {
			ERR_MSG(
			"lli_probe: chan_data[%d].lli_class Err\n", i);
			goto out_err_2;
		}

		sprintf(chardev_name, "lli-%d", i);

		r = alloc_chrdev_region(&devt, 0, 1, chardev_name);
		if (r < 0) {
			ERR_MSG("chardev_name =%s failed\n", chardev_name);
			goto out_err_2;
		}

		mb86s70_lli_dev->chan_data[i].devt = devt;
		glli_major[i] = MAJOR(mb86s70_lli_dev->chan_data[i].devt);

		cdev_init(&mb86s70_lli_dev->chan_data[i].cdev, &lli_fops);
		mb86s70_lli_dev->chan_data[i].cdev.owner = THIS_MODULE;

		r = cdev_add(&mb86s70_lli_dev->chan_data[i].cdev, devt, 1);
		if (r) {
			dev_err(dev, "cdev_add() failed\n");
			goto out_err_2;
		}

		sprintf(driver_name, "%s-%d", DRIVER_NAME, i);

		mb86s70_lli_dev->chan_data[i].dev =
			device_create(mb86s70_lli_dev->chan_data[i].lli_class,
			    NULL, devt, NULL, "%s", driver_name);

		if (IS_ERR(mb86s70_lli_dev->chan_data[i].dev)) {
			ERR_MSG("driver_name =%s device_create Fail\n",
			driver_name);
			goto out_err_1;
		}

		mb86s70_lli_dev->chan_data[i].lli_defcfg = &lli_def_cfg;

		/*initial spin_lock*/
		spin_lock_init(&mb86s70_lli_dev->chan_data[i].mb86s70_irq_lock);
		sema_init(&mb86s70_lli_dev->chan_data[i].openf_sem, 1);
		init_completion(&mb86s70_lli_dev->chan_data[i].msg_cmp);
	}

	for (i = 0; i < glli_num; i++)
		init_mb86s70_lli_reg(mb86s70_lli_dev, i);

	sema_init(&mb86s70_lli_dev->buf_sem, 1);
	platform_set_drvdata(pdev, mb86s70_lli_dev);

	if (irq_num)
		devm_kfree(dev, irq_num);

	return ret;

out_err_1:
	for (i = 0; i < glli_num; i++) {
		if (!IS_ERR(mb86s70_lli_dev->chan_data[i].lli_class)) {
			class_destroy(mb86s70_lli_dev->chan_data[i].lli_class);
			cdev_del(&gp_mb86s70_lli_dev->chan_data[i].cdev);
		}
	}

	free_irq(mb86s70_lli_dev->chan_data[i].irq,
		mb86s70_lli_dev);


out_err_2:
	for (i = 0; i < glli_num; i++) {
		if (mb86s70_lli_dev->chan_data[i].regs)
			iounmap(mb86s70_lli_dev->chan_data[i].regs);

		for (clk_num = 0; clk_num < LLI_CLK_NUM; clk_num++) {
			lli_clk = mb86s70_lli_dev->chan_data[i].clocks[clk_num];
			if (lli_clk)
				clk_disable_unprepare(lli_clk);
		}
	}

	if (mb86s70_lli_dev->chan_data)
		devm_kfree(dev, mb86s70_lli_dev->chan_data);

	if (glli_major)
		devm_kfree(dev, glli_major);

	if (irq_num)
		devm_kfree(dev, irq_num);

out_err:
	DBG_MSG("lli_probe error\n");
	ret = -1;
	return ret;
}

static int lli_remove(struct platform_device *pdev)
{
	int i = 0;
	struct lli_dev *dev_lli;
	struct lli_data *lli_ch;
	int clk_num = 0;
	struct clk *lli_clk;

	dev_lli = platform_get_drvdata(pdev);
	lli_ch = dev_lli->chan_data;

	for (i = 0; i < glli_num; i++) {

		if (lli_ch[i].regs)
			iounmap(lli_ch[i].regs);

		release_mem_region(lli_ch[i].rcs->start,
			resource_size(lli_ch[i].rcs));

		for (clk_num = 0; clk_num < LLI_CLK_NUM; clk_num++) {
			lli_clk = lli_ch[i].clocks[clk_num];
			if (lli_clk)
				clk_disable_unprepare(lli_clk);
		}

		free_irq(lli_ch[i].irq, dev_lli);
		device_destroy(lli_ch[i].lli_class, lli_ch[i].devt);
		class_destroy(lli_ch[i].lli_class);
		cdev_del(&lli_ch[i].cdev);
		unregister_chrdev_region(lli_ch[i].devt, 1);
	}

	if (dev_lli->chan_data)
		devm_kfree(&pdev->dev, dev_lli->chan_data);

	if (dev_lli)
		devm_kfree(&pdev->dev, dev_lli);

	if (glli_major)
		devm_kfree(&pdev->dev, glli_major);

	return 0;
}

#ifdef CONFIG_PM
/*
static int lli_suspend(struct device *dev)
{
	return 0;
}


static int lli_resume(struct device *dev)
{
	return 0;
}
*/
#endif


static const struct of_device_id f_mb86s70_mipilli_dt_ids[] = {
	{ .compatible = "fujitsu,mb86s70-mipilli"},
	{ /* sentinel  */ }
};

MODULE_DEVICE_TABLE(of, f_mb86s70_mipilli_dt_ids);

static struct platform_driver lli_driver = {
	.probe = lli_probe,
	.remove = lli_remove,
	/* .suspend = lli_suspend, */
	/* .resume = lli_resume, */
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = f_mb86s70_mipilli_dt_ids,
	},
};


static int __init mb86s70_lli_init(void)
{
	return platform_driver_register(&lli_driver);
}

static void __exit mb86s70_lli_cleanup(void)
{
	platform_driver_unregister(&lli_driver);
}

module_init(mb86s70_lli_init);
module_exit(mb86s70_lli_cleanup);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Fujitsu Semiconductor Limited");
MODULE_DESCRIPTION("MIPI LLI Driver for MB86S70");
MODULE_ALIAS("platform:lli-mb86s70");
