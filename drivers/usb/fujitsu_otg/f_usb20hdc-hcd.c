/*
 * linux/drivers/usb/host/f_usb20hdc-hcd.c - F_USB20HDC USB
 * host controller driver
 *
 * Copyright (C) FUJITSU ELECTRONICS INC. 2011. All rights reserved.
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
/* #define LOWLEVEL_DEBUG */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
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
#include <linux/debugfs.h>
#include <linux/uaccess.h>

#include <asm/unaligned.h>

#include <linux/platform_data/dma-mb8ac0300-hdmac.h>

#include "f_usb20hdc-hcd.h"

void __iomem *reg_base;

/* only 0x07ff bits of wMaxPacketSize are for packet size... */
#define max_packet(max_packet_size) ((max_packet_size) & 0x07ff)
#define is_highbandwidth(max_packet_size) ((max_packet_size) & 0x1800)

#ifdef DEBUG
void dbg_print_connection(struct device *pdev,
	void __iomem *base_addr)
{
	if (get_port_connection_rhs(base_addr)) {
		dev_info(pdev, "%s() : connection state is %d.\n",
			__func__, get_port_low_speed_rhs(base_addr));
		dev_info(pdev, "%s() : line state is %d.\n",
			__func__, get_linestate(base_addr));
		/* Error detection */
		if (get_port_low_speed_rhs(base_addr)) {
			/* low speed device connection analysis */
			if (2 != get_linestate(base_addr))
				dev_info(pdev,
					"%s() : speed detection error\n",
								     __func__);
		} else {
			/* non-low speed device connection analysis */
			if (1 != get_linestate(base_addr))
				dev_info(pdev,
					"%s() : speed detection error\n",
								     __func__);
		}
	} else
		dev_info(pdev, "%s() : no connection.\n", __func__);


	if (get_port_connection_c(base_addr))
		dev_info(pdev, "%s() : connection has changed!\n", __func__);
	else
		dev_info(pdev, "%s() : connection has no change.\n", __func__);
}
#endif

static void initialize_controller(struct f_usb20hdc_hcd *f_usb20hdc)
{
	void *base_addr = f_usb20hdc->register_base_address;
	u32 counter;

	/*
	 * check device mode
	 * [notice]:It is processing nothing, when dev_en bit is set
	 */
	if (get_dev_en(base_addr))
		return;

	/* initialize F_USB20HDC system configuration register */
	f_usb20hdc_soft_reset_ip(f_usb20hdc->f_otg);

	/* initialize F_USB20HDC mode register */
	set_host_en(base_addr, 0);

	/*
	 * initialize F_USB20HDC global interrupt register
	 * [notice]:otg_inten bit is always enable
	 */
	set_host_inten(base_addr, 0);
	set_dev_inten(base_addr, 0);
	set_phy_err_inten(base_addr, 0);
	set_cmd_inten(base_addr, 0);
	for (counter = F_USB20HDC_DMA_CH1;
			counter < F_USB20HDC_MAX_DMA_CHANNELS; counter++)
		set_dma_inten(base_addr, counter, 0);
	for (counter = F_USB20HDC_EP0;
			counter < F_USB20HDC_HCD_MAX_EP; counter++)
		set_dev_ep_inten(base_addr, counter, 0);
	clear_phy_err_int(base_addr);
	clear_cmd_int(base_addr);
	for (counter = F_USB20HDC_DMA_CH1;
			counter < F_USB20HDC_MAX_DMA_CHANNELS; counter++)
		clear_dma_int(base_addr, counter);

	/* initialize F_USB20HDC port control / status / event register */
	if (f_usb20hdc->wakeup_from_poweroff == 0) {
		set_port_power_ctl_req(base_addr, 1);
		enable_bus_power_supply(base_addr, 0);
	}
	set_forcefs_req(base_addr, f_usb20hdc->highspeed_support ? 0 : 1);
	set_port_enable_req(base_addr, 0);
	set_port_wakeup_req(base_addr, 1);
	set_sofstart_inten(base_addr, 0);
	set_frameov_inten(base_addr, 0);
	set_port_connection_c_inten(base_addr, 0);
	set_port_enable_c_inten(base_addr, 0);
	set_port_suspend_c_inten(base_addr, 0);
	set_port_ov_curr_c_inten(base_addr, 0);
	set_port_reset_c_inten(base_addr, 0);
	for (counter = F_USB20HDC_EP0;
			counter < F_USB20HDC_HCD_MAX_EP; counter++)
		set_trans_done_inten(base_addr, counter, 0);
	set_sofevtinterval(base_addr, SOFEVTINTERVAL_8UFRAME);
	clear_port_connection_c(base_addr);
	clear_port_enable_c(base_addr);
	clear_port_suspend_c(base_addr);
	clear_port_ov_curr_c(base_addr);
	clear_port_reset_c(base_addr);
	clear_sofstart(base_addr);
	clear_frameov(base_addr);
	for (counter = F_USB20HDC_EP0;
			counter < F_USB20HDC_HCD_MAX_EP; counter++)
		clear_trans_done(base_addr, counter);
	set_frmidx(base_addr, 0, 7);
	set_frminit(base_addr, F_USB20HDC_HCD_CONFIG_FRMINIT_BIT_VALUE);

	set_hcrun(base_addr, 0);
	set_startlink(base_addr, F_USB20HDC_EP0);

	/* initialize F_USB20HDC otg register */
	set_dm_pull_down(base_addr, 0);
	set_dp_pull_down(base_addr, 0);
	clear_tmrout_c(base_addr);
	clear_vbus_vld_c(base_addr);

	set_otg_tmrout_ren(base_addr, 0);
	set_start_tmr(base_addr, 0);
	set_tmr_init_val(base_addr, 0);

	/* initialize F_USB20HDC dma register */
	for (counter = F_USB20HDC_DMA_CH1;
			counter < F_USB20HDC_MAX_DMA_CHANNELS; counter++) {
		set_dma_st(base_addr, counter, 0);
		set_dma_mode(base_addr, counter, DMA_MODE_DEMAND);
		set_dma_sendnull(base_addr, counter, 0);
		set_dma_int_empty(base_addr, counter, 0);
		set_dma_spr(base_addr, counter, 0);
		set_dma_ep(base_addr, counter, 0);
		set_dma_blksize(base_addr, counter, 0);
		set_dma_tci(base_addr, counter, 0);
	}

	/* initialize F_USB20HDC ram register */
	for (counter = F_USB20HDC_EP0;
			counter < F_USB20HDC_HCD_MAX_EP; counter++) {
		clear_hcepctrl(base_addr, counter);
		clear_epconf(base_addr, counter);
	}
	for (counter = 0; counter < F_USB20HDC_HCD_MAX_EP *
			F_USB20HDC_EP_BUFFER_COUNT; counter++)
		clear_epcount(base_addr, counter);

	return;
}

static void initialize_td_schedule_list(void *base_addr)
{
	set_startlink(base_addr, F_USB20HDC_EP3);
	set_nextlink(base_addr, F_USB20HDC_EP3,
			F_USB20HDC_EP4);
	set_nextlink(base_addr, F_USB20HDC_EP4,
			F_USB20HDC_EP5);
	set_nextlink(base_addr, F_USB20HDC_EP5,
			F_USB20HDC_EP6);
	set_nextlink(base_addr, F_USB20HDC_EP6,
			F_USB20HDC_EP7);
	set_nextlink(base_addr, F_USB20HDC_EP7,
			F_USB20HDC_EP0);
	set_nextlink(base_addr, F_USB20HDC_EP0,
			F_USB20HDC_EP2);
	set_nextlink(base_addr, F_USB20HDC_EP2,
			F_USB20HDC_EP1);
	set_nextlink(base_addr, F_USB20HDC_EP1,
			F_USB20HDC_EP0);
}

static void initialize_endpoint_configure(struct f_usb20hdc_hcd *f_usb20hdc)
{
	u32 counter, buffer_counter;
	struct f_usb20hdc_hcd_ep *ep;

	for (counter = F_USB20HDC_EP0;
			counter < F_USB20HDC_HCD_MAX_EP; counter++) {
		ep = &f_usb20hdc->endpoint[counter];
		ep->endpoint_channel = counter;
		ep->inuse_devnum = 0;
		ep->inuse_epnum = 0;
		if (counter < F_USB20HDC_EP3)
			ep->max_packet_size = f_usb20hdc->highspeed_support ?
				endpoint_configuration_data[counter].
					hs_maxpacket :
				endpoint_configuration_data[
					counter].fs_maxpacket;
		ep->buffer_size = counter < F_USB20HDC_EP3 ?
			endpoint_configuration_data[counter].buffer_size :
			F_USB20HDC_HCD_MAX_BUFFER_PER_EP;
		ep->buffers = counter < F_USB20HDC_EP3 ?
			endpoint_configuration_data[counter].buffers :
			F_USB20HDC_EP_BUFFER_COUNT;
		ep->buffer_address_offset[0] =
			counter == F_USB20HDC_EP0 ?
			get_epbuf_address_offset() :
			f_usb20hdc->endpoint[counter - 1].buffer_address_offset[
			f_usb20hdc->endpoint[counter - 1].buffers - 1] +
			f_usb20hdc->endpoint[counter - 1].buffer_size;
		ep->buffer_address_offset[0] = (ep->buffer_address_offset[0] +
							0x3) & ~0x3;
		for (buffer_counter = 1;
			buffer_counter < ep->buffers; buffer_counter++) {
			ep->buffer_address_offset[buffer_counter] =
				ep->buffer_address_offset[buffer_counter - 1] +
				ep->buffer_size;
			ep->buffer_address_offset[buffer_counter] =
				(ep->buffer_address_offset[buffer_counter] +
					0x3) & ~0x3;
		}
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
		ep->usb_dma_channel =
			endpoint_configuration_data[counter].usb_dma_channel;
#endif
	}

	return;
}

static void set_endpoint_configure(struct f_usb20hdc_hcd *f_usb20hdc,
	u8 endpoint_channel, u8 transfer_type)
{
	struct f_usb20hdc_hcd_ep *ep = &f_usb20hdc->endpoint[endpoint_channel];

	ep->max_packet_size = f_usb20hdc->highspeed_support ?
				endpoint_configuration_data[
				transfer_type].hs_maxpacket :
				endpoint_configuration_data[
				transfer_type].fs_maxpacket;
	ep->buffers = endpoint_configuration_data[transfer_type].buffers;
	ep->buffer_size = endpoint_configuration_data[
						transfer_type].buffer_size;
	ep->transfer_type = transfer_type;

	return;
}

static u8 is_endpoint_buffer_usable(void)
{
	u32 counter;
	u32 buffer_size = 0;

	/* calculate RAM buffer size */
	buffer_size += 256;
	buffer_size += endpoint_configuration_data[
				F_USB20HDC_EP0].buffer_size *
					F_USB20HDC_EP_BUFFER_COUNT;
	for (counter = F_USB20HDC_EP1;
				counter < F_USB20HDC_EP3; counter++)
		buffer_size += endpoint_configuration_data[
					counter].buffer_size *
				endpoint_configuration_data[counter].buffers;
	for (; counter < F_USB20HDC_HCD_MAX_EP; counter++)
		buffer_size += F_USB20HDC_HCD_MAX_BUFFER_PER_EP *
						F_USB20HDC_EP_BUFFER_COUNT;

	return buffer_size <= F_USB20HDC_HCD_EP_BUFFER_RAM_SIZE ? 1 : 0;
}

static u8 is_host_mode_usage(struct f_usb20hdc_hcd *f_usb20hdc)
{
	return get_host_en(f_usb20hdc->register_base_address) ? 1 : 0;
}

static u8 is_device_mode_usage(struct f_usb20hdc_hcd *f_usb20hdc)
{
	return get_dev_en(f_usb20hdc->register_base_address) ? 1 : 0;
}

static u8 is_split_transaction_required(
	struct f_usb20hdc_hcd_ep *endpoint)
{
	struct f_usb20hdc_hcd_req *request = endpoint->request;
	struct usb_device *parent;
	u8 high_speed_parent_exist;

	for (high_speed_parent_exist = 0, parent =
		request->urb->dev->parent; parent; parent = parent->parent) {
		if ((parent->level) && (parent->speed == USB_SPEED_HIGH)) {
			high_speed_parent_exist = 1;
			break;
		}
	}

	return (request->urb->dev->speed != USB_SPEED_HIGH) &&
					(high_speed_parent_exist) ? 1 : 0;
}

static void create_split_token_data(struct f_usb20hdc_hcd_ep *endpoint)
{
	struct f_usb20hdc_hcd_req *request = endpoint->request;
	struct usb_device *parent;
	struct usb_device *children;

	for (parent = request->urb->dev->parent, children = request->urb->dev;
		parent; parent = parent->parent, children = children->parent) {
		if ((parent->level) && (parent->speed == USB_SPEED_HIGH)) {
			endpoint->split.hub_address = parent->devnum;
			endpoint->split.port_address = children->portnum;
			endpoint->split.startbit =
				children->speed == USB_SPEED_FULL ? 0 : 1;
			break;
		}
	}

	return;
}

#ifdef VIRTUAL_DEVICE_ADDRESS
int free_all_mapped_addr(struct usb_hcd *hcd)
{
	struct f_usb20hdc_hcd *f_usb20hdc;
	struct device *pdev;
	struct dev_addr_mapping *table;
	int i = 0;

	/* get parameter */
	f_usb20hdc = hcd_to_f_usb20hdc(hcd);
	pdev = hcd->self.controller;
	table = f_usb20hdc->dev_addr_table;

	/* lookup a child logical addr of this hub addr.*/
	for (i = 0; i < MAX_DEV_ADDR; i++)
		table[i].used = 0;

	dbg_print(pdev, "%s() zero address has no mapped addr.",
			__func__);
	return 0;
}

int free_mapped_addr(struct usb_hcd *hcd, u8 log_addr)
{
	struct f_usb20hdc_hcd *f_usb20hdc;
	struct device *pdev;
	struct dev_addr_mapping *table;
	int i;
	int j;

	/* get parameter */
	f_usb20hdc = hcd_to_f_usb20hdc(hcd);
	pdev = hcd->self.controller;
	table = f_usb20hdc->dev_addr_table;

	/* zero is the config addr in USB spec, we just return it as phy addr */
	if (log_addr == 0) {
		dev_err(pdev, "%s() zero address has no mapped addr.",
			__func__);
		return -1;
	}

	/* lookup a mapped structure and free it*/
	for (i = 0; i < MAX_DEV_ADDR; i++) {
		if (table[i].logical_addr == log_addr) {
			/* free the mapped addr*/
			dbg_print(pdev,
				"%s() logical addr:%d (phy addr:%d) is freed.\n",
				__func__, log_addr, (i+1));
			table[i].used = 0;

			/* search any child if removed addr is a parent */
			for (j = 0; j < MAX_DEV_ADDR; j++) {
				if (table[i].hub_addr == log_addr)
					free_mapped_addr(hcd,
							table[i].logical_addr);
			}
			return 0;
		}
	}

	/*no mapped addr is available*/
	dbg_print(pdev, "%s() nothing has been found to be freed.\n", __func__);
	return -1;
}

int free_disconnected_mapped_addr(struct usb_hcd *hcd,
	u8 hub_addr, u8 port_num)
{
	struct f_usb20hdc_hcd *f_usb20hdc;
	struct device *pdev;
	struct dev_addr_mapping *table;
	int i;

	/* get parameter */
	f_usb20hdc = hcd_to_f_usb20hdc(hcd);
	pdev = hcd->self.controller;
	table = f_usb20hdc->dev_addr_table;

	/* zero is the config addr in USB spec, we just return it as phy addr */
	if (hub_addr == 0) {
		dev_err(pdev, "%s() zero address can't be used.", __func__);
		return -1;
	}

	/* lookup a child logical addr of this hub addr.*/
	for (i = 0; i < MAX_DEV_ADDR; i++) {
		if ((table[i].hub_addr == hub_addr) &&
					(table[i].port_num == port_num)) {
			dbg_print(pdev,
				"%s() the freed logical:%d (phy:%d) is a child of hub addr:%d.\n",
				__func__,
				table[i].logical_addr, (i+1), hub_addr);
			free_mapped_addr(hcd, table[i].logical_addr);
			return 0;
		}
	}

	dbg_print(pdev, "%s() nothing has been found to be freed.\n", __func__);
	return -1;
}

/*
 *return value : >= 0 means successfully
 *< 0 means failed.
*/
int get_mapped_addr(struct usb_hcd *hcd, u8 log_addr)
{
	struct f_usb20hdc_hcd *f_usb20hdc;
	struct device *pdev;
	struct dev_addr_mapping *table;
	int i;

	/* get parameter */
	f_usb20hdc = hcd_to_f_usb20hdc(hcd);
	pdev = hcd->self.controller;
	table = f_usb20hdc->dev_addr_table;

	/* zero is the config addr in USB spec, we just return it as phy addr */
	if (log_addr == 0) {
		dbg_print(pdev, "%s() zero address has no mapped addr.",
			__func__);
		return 0;
	}

	/* lookup a mapped physical address and return it*/
	for (i = 0; i < MAX_DEV_ADDR; i++) {
		if ((table[i].used == 1) &&
					(table[i].logical_addr == log_addr)) {
			dbg_print(pdev,
				"%s() logical addr:%d is mapped to phy addr:%d.\n",
				__func__, log_addr, (i+1));
			return i + 1;
		}
	}

	/*no phy addr is available*/
	dev_err(pdev, "%s() no available physical address.\n", __func__);
	return -1;
}

int create_mapped_addr(struct usb_hcd *hcd, struct urb *urb, u8 log_addr)
{
	struct f_usb20hdc_hcd *f_usb20hdc;
	struct device *pdev;
	struct dev_addr_mapping *table;
	struct usb_device *parent;
	struct usb_device *children;
	int i;

	/* get parameter */
	f_usb20hdc = hcd_to_f_usb20hdc(hcd);
	pdev = hcd->self.controller;
	table = f_usb20hdc->dev_addr_table;

	/* zero is the config addr in USB spec, we just return it as phy addr */
	if (log_addr == 0) {
		dev_err(pdev, "%s() zero address can't be used to setup.",
			__func__);
		return -1;
	}

	/* check if there is already a mapped physical address */
	for (i = 0; i < MAX_DEV_ADDR; i++) {
		if ((table[i].used == 1) &&
					(table[i].logical_addr == log_addr)) {
			dev_err(pdev, "%s() this address has been mapped before. (log:%d, phy:%d).\n",
				__func__, log_addr, i + 1);
			return -1;
		}
	}

	/* search to allocate a usable physical address */
	for (i = 0; i < MAX_DEV_ADDR; i++) {
		if (table[i].used == 0) {
			dbg_print(pdev,
				"%s() logical addr:%d is set to phy addr:%d.\n",
				__func__, log_addr, (i+1));
			table[i].logical_addr = log_addr;
			table[i].used = 1;
			break;
		}
	}

	/* check if any physical addr is available */
	if (i == MAX_DEV_ADDR) {
		dev_err(pdev, "%s() no physical addr is usable.\n", __func__);
		return -1;
	}

	/* fill information which will be used in recycling this mapped addr */
	for (parent = urb->dev->parent, children = urb->dev;
		parent; parent = parent->parent, children = children->parent) {
		if (parent->level) {
			dbg_print(pdev,
				"%s() connected hub addr:%d and port num:%d.\n",
				__func__, parent->devnum, children->portnum);
			table[i].hub_addr = parent->devnum;
			table[i].port_num = children->portnum;
			dbg_print(pdev,
				"%s() logical addr:%d is mapped to phy addr:%d.\n",
				__func__, log_addr, (i+1));
			return i + 1;
		}
	}

	/*this device is plug into root hub, so no parent device*/
	table[i].hub_addr = 0;
	table[i].port_num = 0;
	dbg_print(pdev,
		"%s() it connected to root ( logical addr:%d is mapped to phy addr:%d).",
		__func__, log_addr, (i+1));
	return i + 1;
}

static void set_hubaddr(void __iomem *base_addr, u8 ep_ch,
	u8 address, struct usb_hcd *hcd)
{
	int addr;

	addr = get_mapped_addr(hcd, address);
	set_register_bits(base_addr, F_USB20HDC_REG_HCEPCTRL2_0 + ep_ch,
				8, 4, addr);
}

static void set_funcaddr(void __iomem *base_addr, u8 ep_ch,
	u8 address, struct usb_hcd *hcd)
{
	int addr;

	addr = get_mapped_addr(hcd, address);
	set_register_bits(base_addr, F_USB20HDC_REG_HCEPCTRL2_0 + ep_ch,
				16, 4, addr);
}
#endif

static void create_hub_descriptor(struct f_usb20hdc_hcd *f_usb20hdc,
	struct usb_hub_descriptor *desc)
{
	/* create hub descriptor */
	memset(desc, 0, sizeof(*desc));
	desc->bDescLength = 9;
	desc->bDescriptorType = 0x29;
	desc->bNbrPorts = 1;
	desc->wHubCharacteristics = cpu_to_le16(0x0009);
	desc->bPwrOn2PwrGood = 10;
	desc->bHubContrCurrent = 0;
	desc->u.hs.DeviceRemovable[0] = 0x00;
	desc->u.hs.DeviceRemovable[1] = 0xff;
}

static void set_frame_interval(struct f_usb20hdc_hcd_ep *endpoint)
{
	u32 counter;
	struct f_usb20hdc_hcd *f_usb20hdc = endpoint->f_usb20hdc;
	struct f_usb20hdc_hcd_req *request = endpoint->request;
#ifdef LOWLEVEL_DEBUG
	struct usb_hcd *hcd = container_of((void *)f_usb20hdc,
		struct usb_hcd , hcd_priv);
	struct device *pdev = hcd->self.controller;
#endif
	void *base_addr = f_usb20hdc->register_base_address;
	u8 endpoint_channel = endpoint->endpoint_channel;
	u32 frame_interval;

	/* calculate frame interval */
	if (request->urb->dev->speed == USB_SPEED_HIGH) {
		if (!(request->urb->interval / 8)) {
			set_intervalsel(base_addr, endpoint_channel, 0);
			set_fmtsel(base_addr, endpoint_channel, 0);
			set_interval(base_addr, endpoint_channel, 0);
			return;
		}
		frame_interval = request->urb->interval / 8;
	} else
		frame_interval = request->urb->interval;

	/* search interval register set value */
	for (counter = 0;; counter++) {
		if (frame_interval == (1 << counter))
			break;
		if (((1 << counter) < frame_interval) &&
				(frame_interval <= (1 << (counter + 1))))
			break;
		if (counter >= 8) {
			counter = 7;
			break;
		}
	}

	dbg_print(pdev,
		"%s() : counter is %d, urb->interval is %d, urb->dev->speed is %d",
		__func__, counter, request->urb->interval,
		request->urb->dev->speed);

	/* check Interrupt SPLIT transaction */
	if ((is_split_transaction_required(endpoint)) &&
			(usb_pipetype(request->urb->pipe) == PIPE_INTERRUPT)) {
		set_intervalsel(base_addr, endpoint_channel, 0);
		set_fmtsel(base_addr, endpoint_channel, 0);
		set_interval(base_addr, endpoint_channel, 3);
	} else {
		set_intervalsel(base_addr, endpoint_channel, 1);
		set_fmtsel(base_addr, endpoint_channel, 0);
		set_interval(base_addr, endpoint_channel, counter);
	}
}

static struct f_usb20hdc_hcd_req *allocate_request_memory(
	struct f_usb20hdc_hcd_ep *endpoint)
{
	struct f_usb20hdc_hcd *f_usb20hdc = endpoint->f_usb20hdc;
	struct usb_hcd *hcd = f_usb20hdc_to_hcd(f_usb20hdc);
	struct device *dev = hcd->self.controller;
	struct f_usb20hdc_hcd_req *request;
#ifdef LOWLEVEL_DEBUG
	u8 endpoint_channel = endpoint->endpoint_channel;
#endif

	/* allocate request memory and zero clear request memory */
	request = kzalloc(sizeof(struct f_usb20hdc_hcd_req), GFP_ATOMIC);
	if (!request) {
		dev_err(dev, "%s():kzalloc failed.\n", __func__);
		return 0;
	}
	dbg_print(dev, "%s(): endpoint %u allocate memory is 0x%p.\n",
					__func__, endpoint_channel, request);

	/* initialize list data */
	INIT_LIST_HEAD(&request->queue);

	return request;
}


static void free_request_memory(struct f_usb20hdc_hcd_ep *endpoint,
	struct f_usb20hdc_hcd_req *request)
{
#ifdef LOWLEVEL_DEBUG
	struct f_usb20hdc_hcd *f_usb20hdc = endpoint->f_usb20hdc;
	struct usb_hcd *hcd = f_usb20hdc_to_hcd(f_usb20hdc);
	struct device *dev = hcd->self.controller;
	u8 endpoint_channel = endpoint->endpoint_channel;
#endif
	/* free memory */
	WARN_ON(!list_empty(&request->queue));
	kfree(request);
	dbg_print(dev, "%s(): endpoint %u free memory is 0x%p.\n",
		__func__, endpoint_channel, request);

	return;
}

static void notify_transfer_request_complete(struct f_usb20hdc_hcd_ep *endpoint,
	struct f_usb20hdc_hcd_req *request, int status)
{
	struct f_usb20hdc_hcd *f_usb20hdc = endpoint->f_usb20hdc;
	struct usb_hcd *hcd = f_usb20hdc_to_hcd(f_usb20hdc);
#ifdef LOWLEVEL_DEBUG
	u8 endpoint_channel = endpoint->endpoint_channel;
#endif
	struct urb *urb = request->urb;

	/* delete and initialize list */
	list_del_init(&request->queue);

	/* set request status */
	if (request->urb->status == -EINPROGRESS)
		request->urb->status = status;
	else
		status = request->urb->status;

	/* clear request execute */
	request->request_execute = 0;
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	request->dmac_int_occurred = 0;
	request->usb_dma_int_occurred = 0;
#endif
	dbg_print(hcd->self.controller,
		"%s(): endpoint %u request is completed at urb = 0x%p, ",
		__func__,
		endpoint_channel, &request->urb);
	dbg_print(hcd->self.controller,
		"%s(): transfer_buffer_length = %u, actual_length = %u, status = %d.\n",
		__func__,
		 request->urb->transfer_buffer_length,
		request->urb->actual_length, status);

	/* free request memory */
	free_request_memory(endpoint, request);

	/* clear current request of the endpoint */
	if (endpoint->request == request)
		endpoint->request = NULL;

	/* unlink URB to ep */
	usb_hcd_unlink_urb_from_ep(hcd, urb);

	/* notify request complete for upper */
	spin_unlock(&f_usb20hdc->lock);
	usb_hcd_giveback_urb(hcd, urb, status);
	spin_lock(&f_usb20hdc->lock);

	return;
}

static void dequeue_all_transfer_request(struct f_usb20hdc_hcd_ep *endpoint,
	int status)
{
	struct f_usb20hdc_hcd *f_usb20hdc = endpoint->f_usb20hdc;
	struct f_usb20hdc_hcd_req *request;
	u32 counter;
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
#if !defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
	struct usb_hcd *hcd = f_usb20hdc_to_hcd(f_usb20hdc);
	enum dma_data_direction dir;
#endif
	void *base_addr = f_usb20hdc->register_base_address;
	s8 usb_dma_channel = endpoint->usb_dma_channel;

	f_usb20hdc->bulkin_remain = 0;
	/* stop DMA transfer */
	if ((usb_dma_channel != -1) && (endpoint->request) &&
		(endpoint->request->request_execute)) {
		set_dma_st(base_addr, usb_dma_channel, 0);
		set_dma_inten(base_addr, usb_dma_channel, 0);
		spin_unlock(&f_usb20hdc->lock);
		hdmac_stop_nowait(f_usb20hdc->dma_data[
				endpoint->usb_dma_channel].hdmac_channel);
		spin_lock(&f_usb20hdc->lock);
	}
#endif

	/* check dynamic allocated endpoint still necessary */
	if (endpoint->transfer_type == TRANSFER_TYPE_INTERRUPT_IN ||
		endpoint->transfer_type == TRANSFER_TYPE_INTERRUPT_OUT)
		f_usb20hdc->eps_inuse_map &=
					~(0x0001 << endpoint->endpoint_channel);
	else if (endpoint->transfer_type == TRANSFER_TYPE_ISOCHRONOUS_IN ||
		endpoint->transfer_type == TRANSFER_TYPE_ISOCHRONOUS_OUT) {
		f_usb20hdc->eps_inuse_map &=
					~(0x0001 << endpoint->endpoint_channel);
		endpoint->inuse_devnum = 0;
		endpoint->inuse_epnum = 0;
	}

	/* dequeue all transfer request */
	for (counter = (u32)-1;
		(counter) && (!list_empty(&endpoint->queue)); counter--) {
		request = list_entry(endpoint->queue.next,
					struct f_usb20hdc_hcd_req, queue);
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
#if !defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
		if (usb_pipein(request->urb->pipe))
			dir = DMA_FROM_DEVICE;
		else
			dir = DMA_TO_DEVICE;
		/* check DMA transfer buffer unmap */
		if ((request->urb->transfer_dma != (dma_addr_t)0) &&
			(request->urb->transfer_dma != ~(dma_addr_t)0)) {
			if (request->dma_transfer_buffer_map) {
				/*
				 * unmap DMA transfer buf and
				 * sync DMA transfer buf
				 */
				dma_unmap_single(hcd->self.controller,
						request->urb->transfer_dma,
						request->urb->actual_length,
						dir);
				request->urb->transfer_dma = ~(dma_addr_t)0;
				request->dma_transfer_buffer_map = 0;
			} else {
				/* synchronize DMA transfer buffer */
				dma_sync_single_for_cpu(
						hcd->self.controller,
						request->urb->transfer_dma,
						request->urb->actual_length,
						dir);
			}
		}
#endif
#endif
		notify_transfer_request_complete(endpoint, request, status);
	}

	return;
}

#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
static void end_dma_transfer(u32 channel, void *data, int state);
static u8 set_dma_transfer(struct f_usb20hdc_hcd_ep *endpoint,
	dma_addr_t source, dma_addr_t destination, u32 bytes)
{
	struct f_usb20hdc_hcd *f_usb20hdc = endpoint->f_usb20hdc;
	void *base_addr = f_usb20hdc->register_base_address;
	struct device *dev = f_usb20hdc->dev;
	u8 endpoint_channel = endpoint->endpoint_channel;
	s8 usb_dma_channel = endpoint->usb_dma_channel;
	struct f_usb20hdc_dma_data *dma_data =
					&f_usb20hdc->dma_data[usb_dma_channel];
	int result;

	dma_data->endpoint_channel = endpoint_channel;
	dma_data->hdmac_req.dmacb = HDMACB_MS_BLOCK | HDMACB_TT_2CYCLE |
					HDMACB_EI | HDMACB_CI | HDMACB_TW_WORD;
	dma_data->hdmac_req.dmacb |= usb_pipein(endpoint->request->urb->pipe) ?
							HDMACB_FS : HDMACB_FD;
	dma_data->hdmac_req.dmaca = dma_data->dreq | HDMACA_BT_INCR16 |
						(((bytes + 63) / 64) - 1);
	dev_dbg(dev, "dma_data->hdmac_req.dmaca is 0x%x.\n",
		dma_data->hdmac_req.dmaca);
	dma_data->hdmac_req.req_data.size = bytes;
	dma_data->hdmac_req.req_data.src = source;
	dma_data->hdmac_req.req_data.dst = destination;
	dma_data->hdmac_req.req_data.irq_data = endpoint;
	dma_data->hdmac_req.req_data.callback_fn = end_dma_transfer;

	result = hdmac_enqueue(dma_data->hdmac_channel, &dma_data->hdmac_req);
	if (result) {
		dev_err(dev, "%s():HDMAC request enqueue  failed at 0x%d.\n",
			__func__, result);
		return 0;
	}

	/* set f_usb20hdc controller DMA register */
	set_dma_tci(base_addr, usb_dma_channel, bytes);
	set_dma_blksize(base_addr, usb_dma_channel, 4);
	set_dma_mode(base_addr, usb_dma_channel, DMA_MODE_DEMAND);
	set_dma_ep(base_addr, usb_dma_channel, endpoint_channel);
	if (!usb_pipein(endpoint->request->urb->pipe)) {
		set_dma_int_empty(base_addr, usb_dma_channel, 1);
		set_dma_sendnull(base_addr, usb_dma_channel, 0);
	} else
		set_dma_spr(base_addr, usb_dma_channel, 0);
	clear_dma_int(base_addr, usb_dma_channel);
	set_dma_inten(base_addr, usb_dma_channel, 1);

	return 1;
}
#endif

static u8 set_control_transfer(struct f_usb20hdc_hcd_ep *endpoint)
{
	struct f_usb20hdc_hcd *f_usb20hdc = endpoint->f_usb20hdc;
	struct f_usb20hdc_hcd_req *request = endpoint->request;
	struct usb_hcd *hcd = f_usb20hdc_to_hcd(f_usb20hdc);
#ifdef LOWLEVEL_DEBUG
	struct device *pdev = hcd->self.controller;
#endif
	void *base_addr = f_usb20hdc->register_base_address;
	u8 ep_channel = endpoint->endpoint_channel;
	u32 *transfer_data;
	u8 index = get_appptr(base_addr, ep_channel);

	dbg_print(pdev, "%s() starts in %s (%s) in ep%d (to addr%d ep%d).",
		__func__, request->urb->dev->product,
		request->urb->dev->manufacturer, ep_channel,
		usb_pipedevice(request->urb->pipe),
		usb_pipeendpoint(request->urb->pipe));

	set_speed(base_addr, ep_channel,
		request->urb->dev->speed == USB_SPEED_HIGH ? SPEED_HIGH_SPEED :
		request->urb->dev->speed == USB_SPEED_FULL ? SPEED_FULL_SPEED :
			SPEED_LOW_SPEED);
	set_bnum(base_addr, ep_channel, 1);
	set_sc(base_addr, ep_channel, 0);
	set_et(base_addr, ep_channel, TYPE_CONTROL);
	set_sendpid(base_addr, ep_channel, PID_SETUP);
	set_toggle_clear(base_addr, ep_channel);
	clear_errcnt_clr(base_addr, ep_channel);
	clear_status_clr(base_addr, ep_channel);

	if (is_split_transaction_required(endpoint)) {
		create_split_token_data(endpoint);
		set_startbit(base_addr, ep_channel, endpoint->split.startbit);
		set_hubportnumber(base_addr, ep_channel,
			endpoint->split.port_address);
		set_hubaddr(base_addr, ep_channel,
			endpoint->split.hub_address, hcd);
	}
	set_funcaddr(base_addr, ep_channel,
		usb_pipedevice(request->urb->pipe), hcd);
	set_endpnumber(base_addr, ep_channel,
		usb_pipeendpoint(request->urb->pipe));

	set_base(base_addr, ep_channel, endpoint->buffer_address_offset[0]);
	set_size(base_addr, ep_channel, request->urb->ep->desc.wMaxPacketSize);
	set_countidx(base_addr, ep_channel,
			ep_channel * F_USB20HDC_EP_BUFFER_COUNT);

	/* set transfer data pointer */
	transfer_data = (u32 *)request->urb->setup_packet;
	prefetch(transfer_data);

	/* write SETUP transfer data to buffer */
	write_epbuf(base_addr, endpoint->buffer_address_offset[index],
		transfer_data, 8);

	/* notify buffer write bytes */
	set_appcnt(base_addr,
		ep_channel * F_USB20HDC_EP_BUFFER_COUNT + index, 8);

	/* enable SETUP transfer */
	set_bufwr(base_addr, ep_channel);

	/* update control transfer stage */
	f_usb20hdc->ctrl_stage = CONTROL_TRANSFER_STAGE_SETUP;

	/* set request execute */
	request->request_execute = 1;

	set_trans_done_inten(base_addr, ep_channel, 1);
	set_start(base_addr, ep_channel);

	dbg_print(pdev, "%s() end in %s (%s) in ep%d (to addr%d ep%d).",
		__func__, request->urb->dev->product,
		request->urb->dev->manufacturer, ep_channel,
		usb_pipedevice(request->urb->pipe),
		usb_pipeendpoint(request->urb->pipe));
	return 1;
}

static u8 set_bulk_in_transfer(struct f_usb20hdc_hcd_ep *endpoint)
{
	struct f_usb20hdc_hcd *f_usb20hdc = endpoint->f_usb20hdc;
	struct f_usb20hdc_hcd_req *request = endpoint->request;
	void *base_addr = f_usb20hdc->register_base_address;
	u8 ep_channel = endpoint->endpoint_channel;
	struct usb_hcd *hcd = f_usb20hdc_to_hcd(f_usb20hdc);
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	u32 bytes;
	u8 start_flag = 0;
	u8 hcptr, appptr, bcnt;

	f_usb20hdc->bulkin_remain = 0;

	hcptr = get_hcptr(base_addr, ep_channel);
	appptr = get_appptr(base_addr, ep_channel);
	bcnt = get_bnum(base_addr, ep_channel) + 1;

	if (get_trans_en(base_addr, ep_channel)) {
		if ((request->urb->transfer_buffer_length <=
				(request->urb->ep->desc.wMaxPacketSize
				* bcnt))) {
			start_flag = 1;
		}
		goto dma_trans;
	}

	if (!get_empty(base_addr, ep_channel)) {

		if ((request->urb->transfer_buffer_length <=
				(request->urb->ep->desc.wMaxPacketSize
				* bcnt))) {
			if (!((hcptr != appptr) && (((hcptr < appptr ?
				bcnt + hcptr - appptr : hcptr - appptr) *
				request->urb->ep->desc.wMaxPacketSize) <
				request->urb->transfer_buffer_length))) {
				start_flag = 1;
			}
		}
		goto dma_trans;
	}

#endif

	set_speed(base_addr, ep_channel,
		request->urb->dev->speed == USB_SPEED_HIGH ? SPEED_HIGH_SPEED :
		request->urb->dev->speed == USB_SPEED_FULL ? SPEED_FULL_SPEED :
			SPEED_LOW_SPEED);
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	set_bnum(base_addr, ep_channel, request->urb->transfer_buffer_length ?
		endpoint->buffers : 1);
#else
	set_bnum(base_addr, ep_channel, request->urb->transfer_buffer_length >
		request->urb->ep->desc.wMaxPacketSize ? endpoint->buffers : 1);
#endif
	set_sc(base_addr, ep_channel, 0);
	set_et(base_addr, ep_channel, TYPE_BULK);
	set_sendpid(base_addr, ep_channel, PID_IN);
	clear_errcnt_clr(base_addr, ep_channel);
	clear_status_clr(base_addr, ep_channel);

	usb_gettoggle(request->urb->dev,
			usb_pipeendpoint(request->urb->pipe), 0) ?
			set_toggle_set(base_addr, ep_channel) :
			set_toggle_clear(base_addr, ep_channel);

	if (is_split_transaction_required(endpoint)) {
		create_split_token_data(endpoint);
		set_startbit(base_addr, ep_channel, endpoint->split.startbit);
		set_hubportnumber(base_addr, ep_channel,
			endpoint->split.port_address);
		set_hubaddr(base_addr, ep_channel,
			endpoint->split.hub_address, hcd);
	}
	set_funcaddr(base_addr, ep_channel,
		usb_pipedevice(request->urb->pipe), hcd);
	set_endpnumber(base_addr, ep_channel,
		usb_pipeendpoint(request->urb->pipe));

	set_base(base_addr, ep_channel, endpoint->buffer_address_offset[0]);
	set_size(base_addr, ep_channel, request->urb->ep->desc.wMaxPacketSize);
	set_countidx(base_addr, ep_channel,
			ep_channel * F_USB20HDC_EP_BUFFER_COUNT);

#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
dma_trans:
	if ((endpoint->usb_dma_channel != -1)
		&& (request->urb->transfer_buffer_length != 0)) {
		/* calculate this time transfer byte */
		bytes = request->urb->transfer_buffer_length <
			F_USB20HDC_HCD_MAX_DMA_TRANSFER_SIZE ?
			request->urb->transfer_buffer_length :
			F_USB20HDC_HCD_MAX_DMA_TRANSFER_SIZE;

#if defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
		if (set_dma_transfer(endpoint,
			(dma_addr_t)f_usb20hdc->dma_data[
			endpoint->usb_dma_channel].epbuf_dma_addr,
			(dma_addr_t)f_usb20hdc->dma_data[
			endpoint->usb_dma_channel].dma_buffer, bytes)) {
			/* set request execute */
			request->request_execute = 1;

			/* start DMA transfer */
			hdmac_start(f_usb20hdc->dma_data[
				endpoint->usb_dma_channel].hdmac_channel);
			set_dma_st(base_addr, endpoint->usb_dma_channel, 1);
			set_trans_done_inten(base_addr, ep_channel, 1);

			if (start_flag == 0)
				set_start(base_addr, ep_channel);

			return 1;
		}
#else
		/* check DMA transfer buffer mapping */
		if ((request->urb->transfer_dma == (dma_addr_t)0) ||
			(request->urb->transfer_dma == ~(dma_addr_t)0)) {

			/*
			 * map DMA transfer buffer and synchronize DMA transfer
			 * buffer.
			 */
			request->urb->transfer_dma =
				dma_map_single(hcd->self.controller,
					request->urb->transfer_buffer,
					request->urb->transfer_buffer_length,
					DMA_FROM_DEVICE);
			request->dma_transfer_buffer_map = 1;
		} else {
			/* synchronize DMA transfer buffer */
			dma_sync_single_for_device(hcd->self.controller,
					request->urb->transfer_dma,
					request->urb->transfer_buffer_length,
					DMA_FROM_DEVICE);
			request->dma_transfer_buffer_map = 0;
		}

		/* check DMA transfer address align */
		if (request->urb->transfer_dma & 0x3) {
			dev_dbg(hcd->self.controller,
				"%s():DMA can't setup dma address 0x%llx\n",
				__func__, (u64)request->urb->transfer_dma);
			if (request->dma_transfer_buffer_map) {
				dma_unmap_single(hcd->self.controller,
					request->urb->transfer_dma,
					request->urb->transfer_buffer_length,
					DMA_FROM_DEVICE);
				request->urb->transfer_dma = ~(dma_addr_t)0;
				request->dma_transfer_buffer_map = 0;
			}
			goto pio_trans;
		}

		if (set_dma_transfer(endpoint,
			(dma_addr_t)f_usb20hdc->dma_data[
			endpoint->usb_dma_channel].epbuf_dma_addr,
			(dma_addr_t)request->urb->transfer_dma, bytes)) {
			/* set request execute */
			request->request_execute = 1;

			/* start DMA transfer */
			hdmac_start(f_usb20hdc->dma_data[
				endpoint->usb_dma_channel].hdmac_channel);
			set_dma_st(base_addr, endpoint->usb_dma_channel, 1);
			set_trans_done_inten(base_addr, ep_channel, 1);

			if (start_flag == 0)
				set_start(base_addr, ep_channel);

			return 1;
		} else {
			dev_dbg(hcd->self.controller,
				"%s():set_dma_transfer() is failed.\n",
				__func__);
			if (request->dma_transfer_buffer_map) {
				dma_unmap_single(hcd->self.controller,
					request->urb->transfer_dma,
					request->urb->transfer_buffer_length,
					DMA_FROM_DEVICE);
				request->urb->transfer_dma = ~(dma_addr_t)0;
				request->dma_transfer_buffer_map = 0;
			}
		}
#endif
	}
#endif
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
#if !defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
pio_trans:
#endif
#endif
	/* set request execute */
	request->request_execute = 1;

	set_trans_done_inten(base_addr, ep_channel, 1);
	set_start(base_addr, ep_channel);

	return 1;
}

static u8 set_bulk_out_transfer(struct f_usb20hdc_hcd_ep *endpoint)
{
	struct f_usb20hdc_hcd *f_usb20hdc = endpoint->f_usb20hdc;
	struct usb_hcd *hcd = f_usb20hdc_to_hcd(f_usb20hdc);
	struct f_usb20hdc_hcd_req *request = endpoint->request;
	void *base_addr = f_usb20hdc->register_base_address;
	u8 ep_channel = endpoint->endpoint_channel;
	u32 *transfer_data;
	u8 index = get_appptr(base_addr, ep_channel);
	u32 bytes;

	set_speed(base_addr, ep_channel,
		request->urb->dev->speed == USB_SPEED_HIGH ? SPEED_HIGH_SPEED :
		request->urb->dev->speed == USB_SPEED_FULL ? SPEED_FULL_SPEED :
			SPEED_LOW_SPEED);
	set_bnum(base_addr, ep_channel, endpoint->buffers);
	set_sc(base_addr, ep_channel, 0);
	set_et(base_addr, ep_channel, TYPE_BULK);
	set_sendpid(base_addr, ep_channel, PID_OUT);
	clear_errcnt_clr(base_addr, ep_channel);
	clear_status_clr(base_addr, ep_channel);

	usb_gettoggle(request->urb->dev,
			usb_pipeendpoint(request->urb->pipe), 1) ?
			set_toggle_set(base_addr, ep_channel) :
			set_toggle_clear(base_addr, ep_channel);

	if (is_split_transaction_required(endpoint)) {
		create_split_token_data(endpoint);
		set_startbit(base_addr, ep_channel, endpoint->split.startbit);
		set_hubportnumber(base_addr, ep_channel,
			endpoint->split.port_address);
		set_hubaddr(base_addr, ep_channel,
			endpoint->split.hub_address, hcd);
	}
	set_funcaddr(base_addr, ep_channel,
		usb_pipedevice(request->urb->pipe), hcd);
	set_endpnumber(base_addr, ep_channel,
		usb_pipeendpoint(request->urb->pipe));

	set_base(base_addr, ep_channel, endpoint->buffer_address_offset[0]);
	set_size(base_addr, ep_channel, request->urb->ep->desc.wMaxPacketSize);
	set_countidx(base_addr, ep_channel,
			ep_channel * F_USB20HDC_EP_BUFFER_COUNT);

#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	if ((endpoint->usb_dma_channel != -1)
		&& (request->urb->transfer_buffer_length != 0)) {
		/* calculate this time transfer byte */
		bytes = request->urb->transfer_buffer_length <
			F_USB20HDC_HCD_MAX_DMA_TRANSFER_SIZE ?
			request->urb->transfer_buffer_length :
			F_USB20HDC_HCD_MAX_DMA_TRANSFER_SIZE;

#if defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
		if (set_dma_transfer(endpoint,
			(dma_addr_t)f_usb20hdc->dma_data[
			endpoint->usb_dma_channel].dma_buffer,
			(dma_addr_t)f_usb20hdc->dma_data[
			endpoint->usb_dma_channel].epbuf_dma_addr, bytes)) {
			/* copy bulk OUT transfer data to noncachable buffer */
			memcpy(f_usb20hdc->dma_data[
				endpoint->usb_dma_channel].buffer,
				request->urb->transfer_buffer, bytes);

			/* set request execute */
			request->request_execute = 1;
			request->dmac_int_occurred = 0;
			request->usb_dma_int_occurred = 0;

			/* start DMA transfer */
			hdmac_start(f_usb20hdc->dma_data[
				endpoint->usb_dma_channel].hdmac_channel);
			set_dma_st(base_addr, endpoint->usb_dma_channel, 1);
			set_trans_done_inten(base_addr, ep_channel, 1);
			set_start(base_addr, ep_channel);

			return 1;
		}
#else
		/* check DMA transfer buffer mapping */
		if ((request->urb->transfer_dma == (dma_addr_t)0) ||
			(request->urb->transfer_dma == ~(dma_addr_t)0)) {
			/*
			 * map DMA transfer buffer and synchronize DMA transfer
			 * buffer.
			 */
			request->urb->transfer_dma =
				dma_map_single(hcd->self.controller,
					request->urb->transfer_buffer,
					request->urb->transfer_buffer_length,
					DMA_TO_DEVICE);
			request->dma_transfer_buffer_map = 1;
		} else {
			/* synchronize DMA transfer buffer */
			dma_sync_single_for_device(hcd->self.controller,
					request->urb->transfer_dma,
					request->urb->transfer_buffer_length,
					DMA_TO_DEVICE);
			request->dma_transfer_buffer_map = 0;
		}

		/* check DMA transfer address align */
		if (request->urb->transfer_dma & 0x3) {
			dev_dbg(hcd->self.controller,
				"%s():DMA can't setup dma address = 0x%llx\n",
				__func__, (u64)request->urb->transfer_dma);
			if (request->dma_transfer_buffer_map) {
				dma_unmap_single(hcd->self.controller,
					request->urb->transfer_dma,
					request->urb->transfer_buffer_length,
					DMA_TO_DEVICE);
				request->urb->transfer_dma = ~(dma_addr_t)0;
				request->dma_transfer_buffer_map = 0;
			}
			goto pio_trans;
		}

		if (set_dma_transfer(endpoint,
			(dma_addr_t)request->urb->transfer_dma,
			(dma_addr_t)f_usb20hdc->dma_data[
			endpoint->usb_dma_channel].epbuf_dma_addr, bytes)) {
			/* set request execute */
			request->request_execute = 1;
			request->dmac_int_occurred = 0;
			request->usb_dma_int_occurred = 0;

			/* start DMA transfer */
			hdmac_start(f_usb20hdc->dma_data[
				endpoint->usb_dma_channel].hdmac_channel);
			set_dma_st(base_addr, endpoint->usb_dma_channel, 1);
			set_trans_done_inten(base_addr, ep_channel, 1);
			set_start(base_addr, ep_channel);

			return 1;
		} else {
			dev_dbg(hcd->self.controller,
				"%s():set_dma_transfer() is failed.\n",
				__func__);
			if (request->dma_transfer_buffer_map) {
				dma_unmap_single(hcd->self.controller,
					request->urb->transfer_dma,
					request->urb->transfer_buffer_length,
					DMA_TO_DEVICE);
				request->urb->transfer_dma = ~(dma_addr_t)0;
				request->dma_transfer_buffer_map = 0;
			}
		}
#endif
	}
#endif
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
#if !defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
pio_trans:
#endif
#endif
	/* calculate transfer data pointer */
	transfer_data = (u32 *)(request->urb->transfer_buffer
						+ request->urb->actual_length);
	prefetch(transfer_data);

	/* calculate this time transfer byte */
	bytes = (request->urb->transfer_buffer_length -
			request->urb->actual_length) <
			request->urb->ep->desc.wMaxPacketSize ?
			request->urb->transfer_buffer_length -
			request->urb->actual_length :
			request->urb->ep->desc.wMaxPacketSize;
	dev_dbg(hcd->self.controller,
			"endpoint %u PIO is setup at length = %u, actual = %u, ",
			ep_channel, request->urb->transfer_buffer_length,
			request->urb->actual_length);
	dev_dbg(hcd->self.controller, "max packet = %u, this time = %u.\n",
			request->urb->ep->desc.wMaxPacketSize,
			(u32)bytes);
	/* set request execute */
	request->request_execute = 1;

	/* check buffer write bytes */
	if (bytes)
		/* write bulk OUT transfer data to buffer */
		write_epbuf(base_addr, (endpoint->buffer_address_offset[0] +
			(request->urb->ep->desc.wMaxPacketSize * index)),
							transfer_data, bytes);

	/* notify buffer write bytes */
	set_appcnt(base_addr, ep_channel *
		F_USB20HDC_EP_BUFFER_COUNT + index, bytes);

	/* enable bulk OUT transfer */
	set_bufwr(base_addr, ep_channel);

	set_trans_done_inten(base_addr, ep_channel, 1);
	set_start(base_addr, ep_channel);

	return 1;
}

static u8 set_interrupt_in_transfer(
	struct f_usb20hdc_hcd_ep *endpoint)
{
	struct f_usb20hdc_hcd *f_usb20hdc = endpoint->f_usb20hdc;
	struct f_usb20hdc_hcd_req *request = endpoint->request;
	struct usb_hcd *hcd = f_usb20hdc_to_hcd(f_usb20hdc);
	struct device *pdev = hcd->self.controller;
	void *base_addr = f_usb20hdc->register_base_address;
	u8 ep_channel = endpoint->endpoint_channel;
	u8 split_require = is_split_transaction_required(endpoint);

	dev_dbg(pdev, "%s() starts in %s (%s) in ep%d (to addr%d ep%d).",
		__func__, request->urb->dev->product,
		request->urb->dev->manufacturer, ep_channel,
		usb_pipedevice(request->urb->pipe),
		usb_pipeendpoint(request->urb->pipe));

	set_speed(base_addr, ep_channel,
		request->urb->dev->speed == USB_SPEED_HIGH ? SPEED_HIGH_SPEED :
		request->urb->dev->speed == USB_SPEED_FULL ? SPEED_FULL_SPEED :
			SPEED_LOW_SPEED);
	set_bnum(base_addr, ep_channel,
		(request->urb->transfer_buffer_length >
		request->urb->ep->desc.wMaxPacketSize) || (split_require) ?
			endpoint->buffers : 1);
	set_sc(base_addr, ep_channel, 0);
	set_et(base_addr, ep_channel, TYPE_INTERRUPT);
	set_sendpid(base_addr, ep_channel, PID_IN);
	clear_errcnt_clr(base_addr, ep_channel);
	clear_status_clr(base_addr, ep_channel);

	usb_gettoggle(request->urb->dev,
			usb_pipeendpoint(request->urb->pipe), 0) ?
			set_toggle_set(base_addr, ep_channel) :
			set_toggle_clear(base_addr, ep_channel);

	set_frame_interval(endpoint);

	if (split_require) {
		create_split_token_data(endpoint);
		set_startbit(base_addr, ep_channel, endpoint->split.startbit);
		set_hubportnumber(base_addr, ep_channel,
			endpoint->split.port_address);
		set_hubaddr(base_addr, ep_channel,
			endpoint->split.hub_address, hcd);
	}
	set_funcaddr(base_addr, ep_channel,
		usb_pipedevice(request->urb->pipe), hcd);
	set_endpnumber(base_addr, ep_channel,
		usb_pipeendpoint(request->urb->pipe));

	set_base(base_addr, ep_channel, endpoint->buffer_address_offset[0]);
	set_size(base_addr, ep_channel, request->urb->ep->desc.wMaxPacketSize);
	set_countidx(base_addr, ep_channel,
			ep_channel * F_USB20HDC_EP_BUFFER_COUNT);

	/* set request execute */
	request->request_execute = 1;

	set_trans_done_inten(base_addr, ep_channel, 1);
	set_start(base_addr, ep_channel);
	dev_dbg(pdev, "%s() end in %s (%s) in ep%d (to addr%d ep%d).",
		__func__, request->urb->dev->product,
		request->urb->dev->manufacturer, ep_channel,
		usb_pipedevice(request->urb->pipe),
		usb_pipeendpoint(request->urb->pipe));

	return 1;
}

static u8 set_interrupt_out_transfer(
	struct f_usb20hdc_hcd_ep *endpoint)
{
	struct f_usb20hdc_hcd *f_usb20hdc = endpoint->f_usb20hdc;
	struct usb_hcd *hcd = f_usb20hdc_to_hcd(f_usb20hdc);
	struct f_usb20hdc_hcd_req *request = endpoint->request;
	void *base_addr = f_usb20hdc->register_base_address;
	u8 ep_channel = endpoint->endpoint_channel;
	u32 *transfer_data;
	u8 index = get_appptr(base_addr, ep_channel);
	u32 bytes;
	u8 split_require = is_split_transaction_required(endpoint);

	set_speed(base_addr, ep_channel,
		request->urb->dev->speed == USB_SPEED_HIGH ? SPEED_HIGH_SPEED :
		request->urb->dev->speed == USB_SPEED_FULL ? SPEED_FULL_SPEED :
			SPEED_LOW_SPEED);
	set_bnum(base_addr, ep_channel,
		(request->urb->transfer_buffer_length >
		request->urb->ep->desc.wMaxPacketSize) || (split_require) ?
			endpoint->buffers : 1);
	set_sc(base_addr, ep_channel, 0);
	set_et(base_addr, ep_channel, TYPE_INTERRUPT);
	set_sendpid(base_addr, ep_channel, PID_OUT);
	clear_errcnt_clr(base_addr, ep_channel);
	clear_status_clr(base_addr, ep_channel);

	usb_gettoggle(request->urb->dev,
			usb_pipeendpoint(request->urb->pipe), 1) ?
			set_toggle_set(base_addr, ep_channel) :
			set_toggle_clear(base_addr, ep_channel);

	set_frame_interval(endpoint);

	if (is_split_transaction_required(endpoint)) {
		create_split_token_data(endpoint);
		set_startbit(base_addr, ep_channel, endpoint->split.startbit);
		set_hubportnumber(base_addr, ep_channel,
			endpoint->split.port_address);
		set_hubaddr(base_addr, ep_channel,
			endpoint->split.hub_address, hcd);
	}
	set_funcaddr(base_addr, ep_channel,
		usb_pipedevice(request->urb->pipe), hcd);
	set_endpnumber(base_addr, ep_channel,
		usb_pipeendpoint(request->urb->pipe));

	set_base(base_addr, ep_channel, endpoint->buffer_address_offset[0]);
	set_size(base_addr, ep_channel, request->urb->ep->desc.wMaxPacketSize);
	set_countidx(base_addr, ep_channel,
			ep_channel * F_USB20HDC_EP_BUFFER_COUNT);

	/* calculate transfer data pointer */
	transfer_data = (u32 *)
		(request->urb->transfer_buffer + request->urb->actual_length);
	prefetch(transfer_data);

	/* calculate this time transfer byte */
	bytes = (request->urb->transfer_buffer_length -
			request->urb->actual_length) <
			request->urb->ep->desc.wMaxPacketSize ?
			request->urb->transfer_buffer_length -
			request->urb->actual_length :
			request->urb->ep->desc.wMaxPacketSize;
	dev_dbg(hcd->self.controller,
		"endpoint %u PIO is setup at length = %u, actual = %u, ",
		ep_channel, request->urb->transfer_buffer_length,
		request->urb->actual_length);
	dev_dbg(hcd->self.controller, "max packet = %u, this time = %u.\n",
		request->urb->ep->desc.wMaxPacketSize, (u32)bytes);

	/* set request execute */
	request->request_execute = 1;

	/* check buffer write bytes */
	if (bytes)
		/* write interrupt OUT transfer data to buffer */
		write_epbuf(base_addr, (endpoint->buffer_address_offset[0] +
			(request->urb->ep->desc.wMaxPacketSize * index)),
							transfer_data, bytes);

	/* notify buffer write bytes */
	set_appcnt(base_addr,
		ep_channel * F_USB20HDC_EP_BUFFER_COUNT + index, bytes);

	/* enable interrupt OUT transfer */
	set_bufwr(base_addr, ep_channel);

	set_trans_done_inten(base_addr, ep_channel, 1);
	set_start(base_addr, ep_channel);

	return 1;
}

static u8 set_isochronous_in_transfer(
	struct f_usb20hdc_hcd_ep *endpoint)
{
	struct f_usb20hdc_hcd *f_usb20hdc = endpoint->f_usb20hdc;
	struct usb_hcd *hcd = f_usb20hdc_to_hcd(f_usb20hdc);
	struct f_usb20hdc_hcd_req *request = endpoint->request;
	void *base_addr = f_usb20hdc->register_base_address;
	u8 ep_channel = endpoint->endpoint_channel;

	set_speed(base_addr, ep_channel,
		request->urb->dev->speed == USB_SPEED_HIGH ? SPEED_HIGH_SPEED :
		request->urb->dev->speed == USB_SPEED_FULL ? SPEED_FULL_SPEED :
			SPEED_LOW_SPEED);
	set_bnum(base_addr, ep_channel,
		request->urb->number_of_packets > 1 ? endpoint->buffers : 1);
	set_sc(base_addr, ep_channel, 0);
	set_et(base_addr, ep_channel, TYPE_ISOCHRONOUS);
	set_sendpid(base_addr, ep_channel, PID_IN);
	clear_errcnt_clr(base_addr, ep_channel);
	clear_status_clr(base_addr, ep_channel);
	set_toggle_clear(base_addr, ep_channel);

	set_frame_interval(endpoint);

	if (is_split_transaction_required(endpoint)) {
		create_split_token_data(endpoint);
		set_startbit(base_addr, ep_channel, endpoint->split.startbit);
		set_hubportnumber(base_addr, ep_channel,
			endpoint->split.port_address);
		set_hubaddr(base_addr, ep_channel,
			endpoint->split.hub_address, hcd);
	}
	set_funcaddr(base_addr, ep_channel,
		usb_pipedevice(request->urb->pipe), hcd);
	set_endpnumber(base_addr, ep_channel,
		usb_pipeendpoint(request->urb->pipe));

	set_base(base_addr, ep_channel, endpoint->buffer_address_offset[0]);
	set_size(base_addr, ep_channel, request->maxpacket);
	set_countidx(base_addr, ep_channel,
			ep_channel * F_USB20HDC_EP_BUFFER_COUNT);

	/* initialize transfer packet count */
	endpoint->transfer_packets = 0;


	/* set request execute */
	request->request_execute = 1;

	set_trans_done_inten(base_addr, ep_channel, 1);
	set_start(base_addr, ep_channel);

	return 1;
}

static u8 set_isochronous_out_transfer(
	struct f_usb20hdc_hcd_ep *endpoint)
{
	struct f_usb20hdc_hcd *f_usb20hdc = endpoint->f_usb20hdc;
	struct usb_hcd *hcd = f_usb20hdc_to_hcd(f_usb20hdc);
	struct f_usb20hdc_hcd_req *request = endpoint->request;
	void *base_addr = f_usb20hdc->register_base_address;
	u8 ep_channel = endpoint->endpoint_channel;
	u32 *transfer_data;
	u8 index = get_appptr(base_addr, ep_channel);
	u32 bytes;

	set_speed(base_addr, ep_channel,
		request->urb->dev->speed == USB_SPEED_HIGH ? SPEED_HIGH_SPEED :
		request->urb->dev->speed == USB_SPEED_FULL ? SPEED_FULL_SPEED :
			SPEED_LOW_SPEED);
	set_bnum(base_addr, ep_channel, endpoint->buffers);
	set_sc(base_addr, ep_channel, 0);
	set_et(base_addr, ep_channel, TYPE_ISOCHRONOUS);
	set_sendpid(base_addr, ep_channel, PID_OUT);
	clear_errcnt_clr(base_addr, ep_channel);
	clear_status_clr(base_addr, ep_channel);
	set_toggle_clear(base_addr, ep_channel);

	set_frame_interval(endpoint);

	if (is_split_transaction_required(endpoint)) {
		create_split_token_data(endpoint);
		set_startbit(base_addr, ep_channel, endpoint->split.startbit);
		set_hubportnumber(base_addr, ep_channel,
			endpoint->split.port_address);
		set_hubaddr(base_addr, ep_channel,
			endpoint->split.hub_address, hcd);
	}
	set_funcaddr(base_addr, ep_channel,
		usb_pipedevice(request->urb->pipe), hcd);
	set_endpnumber(base_addr, ep_channel,
		usb_pipeendpoint(request->urb->pipe));

	set_base(base_addr, ep_channel, endpoint->buffer_address_offset[0]);
	set_size(base_addr, ep_channel, request->maxpacket);
	set_countidx(base_addr, ep_channel,
			ep_channel * F_USB20HDC_EP_BUFFER_COUNT);

	/* initialize transfer packet count */
	endpoint->transfer_packets = 0;

	/* calculate transfer data pointer */
	transfer_data = (u32 *)(request->urb->transfer_buffer +
					request->urb->iso_frame_desc[0].offset);
	prefetch(transfer_data);

	/* calculate this time transfer byte */
	bytes = request->urb->iso_frame_desc[0].length < request->maxpacket ?
			request->urb->iso_frame_desc[0].length :
			request->maxpacket;

	dev_dbg(hcd->self.controller,
		"endpoint %u PIO is setup at  length = %u, actual = %u, ",
		ep_channel, request->urb->iso_frame_desc[0].length,
		request->urb->iso_frame_desc[0].actual_length);
	dev_dbg(hcd->self.controller,
		"max packet = %u, packets = %u, this time packet = %u.\n",
		request->maxpacket, request->urb->number_of_packets,
		(u32)endpoint->transfer_packets);

	/* set request execute */
	request->request_execute = 1;

	/* check buffer write bytes */
	if (bytes)
		/* write isochronous OUT transfer data to buffer */
		write_epbuf(base_addr, (endpoint->buffer_address_offset[0] +
				(request->maxpacket * index)),
				transfer_data, bytes);

	/* notify buffer write bytes */
	set_appcnt(base_addr, ep_channel * F_USB20HDC_EP_BUFFER_COUNT +
		index, bytes);

	/* enable isochronous OUT transfer */
	set_bufwr(base_addr, ep_channel);

	set_trans_done_inten(base_addr, ep_channel, 1);
	set_start(base_addr, ep_channel);

	return 1;
}

static u8 end_control_transfer(struct f_usb20hdc_hcd_ep *endpoint)
{
	struct f_usb20hdc_hcd *f_usb20hdc = endpoint->f_usb20hdc;
	struct usb_hcd *hcd = f_usb20hdc_to_hcd(f_usb20hdc);
	struct f_usb20hdc_hcd_req *request = endpoint->request;
	struct device *pdev = hcd->self.controller;
	void *base_addr = f_usb20hdc->register_base_address;
	u8 ep_channel = endpoint->endpoint_channel;
	u32 *transfer_data;
#ifdef VIRTUAL_DEVICE_ADDRESS
	struct usb_ctrlrequest *dev_req;
	u32 data;
	int hub_addr;
#endif
	u8 index = get_appptr(base_addr, ep_channel);
	u32 bytes;

	dbg_print(pdev, "%s() starts in %s (%s) in ep%d (to addr%d ep%d).",
		__func__, request->urb->dev->product,
		request->urb->dev->manufacturer, ep_channel,
		usb_pipedevice(request->urb->pipe),
		usb_pipeendpoint(request->urb->pipe));

	/* process control transfer stage */
	switch (f_usb20hdc->ctrl_stage) {
	case CONTROL_TRANSFER_STAGE_SETUP:
		if (usb_pipein(request->urb->pipe)) {
			/* update control transfer stage */
			f_usb20hdc->ctrl_stage = CONTROL_TRANSFER_STAGE_IN_DATA;

			set_sendpid(base_addr, ep_channel, PID_IN);
			set_toggle_set(base_addr, ep_channel);

			set_start(base_addr, ep_channel);
		} else if (request->urb->transfer_buffer_length) {
			/* update control transfer stage */
			f_usb20hdc->ctrl_stage =
					CONTROL_TRANSFER_STAGE_OUT_DATA;

			set_sendpid(base_addr, ep_channel, PID_OUT);
			set_toggle_set(base_addr, ep_channel);

			/* calculate transfer data pointer */
			transfer_data = (u32 *)
						(request->urb->transfer_buffer +
						request->urb->actual_length);
			prefetch(transfer_data);

			/* calculate this time transfer byte */
			bytes = (request->urb->transfer_buffer_length -
					request->urb->actual_length) <
					request->urb->ep->desc.wMaxPacketSize ?
					request->urb->transfer_buffer_length -
					request->urb->actual_length :
					request->urb->ep->desc.wMaxPacketSize;

			/* write control OUT transfer data to buffer */
			write_epbuf(base_addr,
					endpoint->buffer_address_offset[index],
					transfer_data, bytes);

			/* notify buffer write bytes */
			set_appcnt(base_addr, ep_channel *
					F_USB20HDC_EP_BUFFER_COUNT +
					index, bytes);

			/* enable control OUT transfer */
			set_bufwr(base_addr, ep_channel);

			set_start(base_addr, ep_channel);
		} else {
			/* update control transfer stage */
			f_usb20hdc->ctrl_stage =
					CONTROL_TRANSFER_STAGE_IN_STATUS;

			set_sendpid(base_addr, ep_channel, PID_IN);
			set_toggle_set(base_addr, ep_channel);

			set_start(base_addr, ep_channel);
		}
		break;
	case CONTROL_TRANSFER_STAGE_IN_DATA:
		/* check STALL response */
		if (get_status_stall(base_addr, ep_channel)) {
			/* clear satus interrupt factor */
			clear_status_clr(base_addr, ep_channel);
			set_trans_done_inten(base_addr, ep_channel, 0);
			endpoint->transfer_status = -EPIPE;
			dbg_print(pdev,
				"%s() end with STALL in %s(%s) in ep%d(to addr%d ep%d).",
				__func__, request->urb->dev->product,
				request->urb->dev->manufacturer, ep_channel,
				usb_pipedevice(request->urb->pipe),
				usb_pipeendpoint(request->urb->pipe));
			return 1;
		}

		/* calculate transfer data pointer */
		transfer_data = (u32 *)
					(request->urb->transfer_buffer +
					request->urb->actual_length);
		prefetch(transfer_data);

		/* get control IN transfer byte */
		bytes = get_phycnt(base_addr, ep_channel *
				F_USB20HDC_EP_BUFFER_COUNT + index);
		if (request->urb->transfer_buffer_length <
					(request->urb->actual_length + bytes))
			bytes = request->urb->transfer_buffer_length -
					request->urb->actual_length;
		dbg_print(hcd->self.controller,
			"endpoint %u PIO is end at length = %u, actual = %u, ",
			ep_channel, request->urb->transfer_buffer_length,
			request->urb->actual_length);
		dbg_print(hcd->self.controller,
			"max packet = %u, this time = %u.\n",
			request->urb->ep->desc.wMaxPacketSize,
			(u32)bytes);

		/* check buffer read bytes */
		if (bytes)
			/* read control IN transfer data form buffer */
			read_epbuf(base_addr,
					endpoint->buffer_address_offset[index],
					transfer_data, bytes);

		/* update actual bytes */
		request->urb->actual_length += bytes;

		/* enable next control IN transfer */
		set_bufrd(base_addr, ep_channel);

		/* check transfer request complete */
		if ((request->urb->actual_length >=
			request->urb->transfer_buffer_length) ||
			(bytes % request->urb->ep->desc.wMaxPacketSize) ||
			(!bytes)) {
			/* update control transfer stage */
			f_usb20hdc->ctrl_stage =
					CONTROL_TRANSFER_STAGE_OUT_STATUS;

			set_sendpid(base_addr, ep_channel, PID_OUT);
			set_toggle_set(base_addr, ep_channel);

			/* notify buffer write bytes */
			set_appcnt(base_addr, ep_channel *
				F_USB20HDC_EP_BUFFER_COUNT + index, 0);

			/* enable control OUT transfer */
			set_bufwr(base_addr, ep_channel);
#ifdef VIRTUAL_DEVICE_ADDRESS
			/* hack into the GetPortStatus' disconnection event */
			dev_req = (struct usb_ctrlrequest *)
						request->urb->setup_packet;
			hub_addr = usb_pipedevice(request->urb->pipe);
			if ((dev_req->bRequestType == 0xa3) &&
						(dev_req->bRequest == 0x0)) {
				dbg_print(pdev,
					"%s() : GetPortStatus request (HubAddr:%d, PortNum:%d)",
					__func__, hub_addr,
					dev_req->wIndex);

				/* read the port connection state*/
				memcpy(&data, request->urb->transfer_buffer, 4);
				data &= (~0xfffffffe);
				dbg_print(pdev,
					"%s() : GetPortStatus data (connection data:%d, BytesRead:%d)",
					__func__,
					data, request->urb->actual_length);

				/*release the mapped addr when it's unplug */
				if (!data)
					free_disconnected_mapped_addr(hcd,
						hub_addr, dev_req->wIndex);
			}
#endif
		}

		set_start(base_addr, ep_channel);
		break;
	case CONTROL_TRANSFER_STAGE_OUT_DATA:
		/* check STALL response */
		if (get_status_stall(base_addr, ep_channel)) {
			/* clear satus interrupt factor */
			clear_status_clr(base_addr, ep_channel);
			set_trans_done_inten(base_addr, ep_channel, 0);
			endpoint->transfer_status = -EPIPE;
			dbg_print(pdev,
				"%s() end with STALL in %s (%s) in ep%d (to addr%d ep%d).",
				__func__, request->urb->dev->product,
				request->urb->dev->manufacturer, ep_channel,
				usb_pipedevice(request->urb->pipe),
				usb_pipeendpoint(request->urb->pipe));
			return 1;
		}

		/* update actual bytes */
		request->urb->actual_length +=
					(request->urb->transfer_buffer_length -
					request->urb->actual_length) <
					request->urb->ep->desc.wMaxPacketSize ?
					request->urb->transfer_buffer_length -
					request->urb->actual_length :
					request->urb->ep->desc.wMaxPacketSize;

		/* check transfer request complete */
		if (request->urb->actual_length >=
					request->urb->transfer_buffer_length) {
			/* update control transfer stage */
			f_usb20hdc->ctrl_stage =
					CONTROL_TRANSFER_STAGE_IN_STATUS;

			set_sendpid(base_addr, ep_channel, PID_IN);
			set_toggle_set(base_addr, ep_channel);

			set_start(base_addr, ep_channel);
		} else {
			/* calculate transfer data pointer */
			transfer_data = (u32 *)
						(request->urb->transfer_buffer +
						request->urb->actual_length);
			prefetch(transfer_data);

			/* calculate this time transfer byte */
			bytes = (request->urb->transfer_buffer_length -
					request->urb->actual_length) <
					request->urb->ep->desc.wMaxPacketSize ?
					request->urb->transfer_buffer_length -
					request->urb->actual_length :
					request->urb->ep->desc.wMaxPacketSize;

			/* write control OUT transfer data to buffer */
			write_epbuf(base_addr,
					endpoint->buffer_address_offset[index],
					transfer_data, bytes);

			/* notify buffer write bytes */
			set_appcnt(base_addr, ep_channel *
					F_USB20HDC_EP_BUFFER_COUNT +
					index, bytes);

			/* enable control OUT transfer */
			set_bufwr(base_addr, ep_channel);

			set_start(base_addr, ep_channel);
		}
		break;
	case CONTROL_TRANSFER_STAGE_IN_STATUS:
		/* check STALL response */
		if (get_status_stall(base_addr, ep_channel)) {
#ifdef VIRTUAL_DEVICE_ADDRESS
			/* free the mapped addr if SET_ADDRESS request failed */
			dev_req = (struct usb_ctrlrequest *)
				request->urb->setup_packet;
			if ((dev_req->bRequestType == 0x0) &&
						(dev_req->bRequest == 0x5))
				free_mapped_addr(hcd,
					usb_pipedevice(request->urb->pipe));
#endif
			/* clear satus interrupt factor */
			clear_status_clr(base_addr, ep_channel);
			set_trans_done_inten(base_addr, ep_channel, 0);
			endpoint->transfer_status = -EPIPE;
			dbg_print(pdev,
				"%s() end with STALL in %s(%s) in ep%d(to addr%d ep%d).",
				__func__, request->urb->dev->product,
				request->urb->dev->manufacturer, ep_channel,
				usb_pipedevice(request->urb->pipe),
				usb_pipeendpoint(request->urb->pipe));
			return 1;
		}

		/* get control IN transfer byte */
		bytes = get_phycnt(base_addr, ep_channel *
				F_USB20HDC_EP_BUFFER_COUNT + index);

		/* enable next control IN transfer */
		set_bufrd(base_addr, ep_channel);

		/* complete request */
		set_trans_done_inten(base_addr, ep_channel, 0);
		endpoint->transfer_status = !bytes ? 0 : -EPROTO;
		dbg_print(pdev,
			"%s() end with complete in %s (%s) in ep%d (to addr%d ep%d).",
			__func__, request->urb->dev->product,
			request->urb->dev->manufacturer, ep_channel,
			usb_pipedevice(request->urb->pipe),
			usb_pipeendpoint(request->urb->pipe));
#ifdef VIRTUAL_DEVICE_ADDRESS
		/* hack into hub port reset request */
		dev_req = (struct usb_ctrlrequest *)
					request->urb->setup_packet;
		hub_addr = usb_pipedevice(request->urb->pipe);
		if ((dev_req->bRequestType == 0x23) &&
				(dev_req->bRequest == 0x3) &&
				(dev_req->wValue == 0x4)) {
			dev_dbg(pdev,
				"%s() hub port reset request: (hub:%d, port_num:%d)",
				__func__, hub_addr, dev_req->wIndex);
			free_disconnected_mapped_addr(hcd, hub_addr,
				dev_req->wIndex);
		}
#endif
		return 1;
	case CONTROL_TRANSFER_STAGE_OUT_STATUS:
		/* check STALL response */
		if (get_status_stall(base_addr, ep_channel)) {
			/* clear satus interrupt factor */
			clear_status_clr(base_addr, ep_channel);
			endpoint->transfer_status = -EPIPE;
			dbg_print(pdev,
				"%s() end with STALL in %s (%s) in ep%d (to addr%d ep%d).",
				__func__, request->urb->dev->product,
				request->urb->dev->manufacturer, ep_channel,
				usb_pipedevice(request->urb->pipe),
				usb_pipeendpoint(request->urb->pipe));
			return 1;
		}

		/* complete request */
		set_trans_done_inten(base_addr, ep_channel, 0);
		endpoint->transfer_status = 0;
		dbg_print(pdev,
			"%s() end with complete in %s (%s) in ep%d (to addr%d ep%d).",
			__func__, request->urb->dev->product,
			request->urb->dev->manufacturer, ep_channel,
			usb_pipedevice(request->urb->pipe),
			usb_pipeendpoint(request->urb->pipe));
		return 1;
	default:
		break;
	}

	dbg_print(pdev,
		"%s() end with not complete in %s (%s) in ep%d (to addr%d ep%d).",
		__func__, request->urb->dev->product,
		request->urb->dev->manufacturer, ep_channel,
		usb_pipedevice(request->urb->pipe),
		usb_pipeendpoint(request->urb->pipe));
	return 0;
}

static u8 end_bulk_in_transfer(struct f_usb20hdc_hcd_ep *endpoint)
{
	struct f_usb20hdc_hcd *f_usb20hdc = endpoint->f_usb20hdc;
	struct usb_hcd *hcd = f_usb20hdc_to_hcd(f_usb20hdc);
	struct f_usb20hdc_hcd_req *request = endpoint->request;
	void *base_addr = f_usb20hdc->register_base_address;
	u8 ep_channel = endpoint->endpoint_channel;
	u32 *transfer_data;
	u8 index;
	u32 bytes;
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	dma_addr_t dma_buffer;
	u8 start_flag = 0;
	u8 hcptr, appptr, bcnt;
#endif

	/* check STALL or HALT status */
	if (get_status_stall(base_addr, ep_channel) ||
		get_status_halt(base_addr, ep_channel)) {
		/* clear satus interrupt factor */
		clear_status_clr(base_addr, ep_channel);
		set_trans_done_inten(base_addr, ep_channel, 0);
		set_stop(base_addr, ep_channel);
		set_init(base_addr, ep_channel);
		set_toggle_clear(base_addr, ep_channel);
		endpoint->transfer_status = -EPIPE;
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
		if (request->urb->transfer_buffer_length != 0) {
			spin_unlock(&f_usb20hdc->lock);
			/* stop hdmac transfer */
			hdmac_stop_nowait(f_usb20hdc->dma_data[
				endpoint->usb_dma_channel].hdmac_channel);
			spin_lock(&f_usb20hdc->lock);

			if (endpoint->usb_dma_channel != -1)
				goto done;
		}
#endif
		return 1;
	}

#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	if ((endpoint->usb_dma_channel != -1)
		&& (request->urb->transfer_buffer_length != 0)) {
		/* get this time transfer byte */
		bytes = get_dma_tc(base_addr, endpoint->usb_dma_channel);

#if defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
		/* copy bulk IN transfer data form noncachable buffer */
		memcpy((request->urb->transfer_buffer +
			request->urb->actual_length),
			f_usb20hdc->dma_data[endpoint->usb_dma_channel].buffer,
			bytes);
#endif
		/* update actual bytes */
		request->urb->actual_length += bytes;

		/* check transfer request complete */
		if ((request->urb->actual_length >=
			request->urb->transfer_buffer_length) ||
			get_dma_sp(base_addr, endpoint->usb_dma_channel)) {
			endpoint->transfer_status = 0;
			clear_trans_done(base_addr, ep_channel);
			set_trans_done_inten(base_addr, ep_channel, 0);

#if !defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
			/* check DMA transfer buffer unmap */
			if ((request->urb->transfer_dma != (dma_addr_t)0) &&
			    (request->urb->transfer_dma != ~(dma_addr_t)0)) {

				if (request->dma_transfer_buffer_map) {
					/*
					 *unmap DMA transfer buf and
					 *sync DMA transfer buf
					 */
					dma_unmap_single(hcd->self.controller,
						request->urb->transfer_dma,
						request->urb->actual_length,
						DMA_FROM_DEVICE);
					request->urb->transfer_dma =
								~(dma_addr_t)0;
					request->dma_transfer_buffer_map = 0;
				} else {
					/* synchronize DMA transfer buffer */
					dma_sync_single_for_cpu(
						hcd->self.controller,
						request->urb->transfer_dma,
						request->urb->actual_length,
						DMA_FROM_DEVICE);
				}
			}
#endif
			goto done;
		}

		/* calculate this time transfer byte */
		bytes = (request->urb->transfer_buffer_length -
					request->urb->actual_length) <
					F_USB20HDC_HCD_MAX_DMA_TRANSFER_SIZE ?
					(request->urb->transfer_buffer_length -
					request->urb->actual_length) :
					F_USB20HDC_HCD_MAX_DMA_TRANSFER_SIZE;
		dev_dbg(hcd->self.controller,
			"endpoint %u DMA is setup at length = %u, actual = %u, ",
			ep_channel, request->urb->transfer_buffer_length,
			request->urb->actual_length);
		dev_dbg(hcd->self.controller,
			"max packet = %u, this time = %u.\n",
			request->urb->ep->desc.wMaxPacketSize,
			(u32)bytes);

		f_usb20hdc->bulkin_remain = 0;

		hcptr = get_hcptr(base_addr, ep_channel);
		appptr = get_appptr(base_addr, ep_channel);
		bcnt = get_bnum(base_addr, ep_channel) + 1;

		if (get_trans_en(base_addr, ep_channel)) {
			if ((bytes <= (request->urb->ep->desc.wMaxPacketSize
								* bcnt)))
				start_flag = 1;

			goto dma_trans;
		}

		if (!get_empty(base_addr, ep_channel)) {
			if ((bytes <= (request->urb->ep->desc.wMaxPacketSize
								* bcnt))) {
				if (!((hcptr != appptr) && (((hcptr < appptr ?
					bcnt + hcptr - appptr : hcptr - appptr)
					* request->urb->ep->desc.wMaxPacketSize)
					< bytes)))
					start_flag = 1;
			}
			goto dma_trans;
		}
dma_trans:
#if defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
		dma_buffer = f_usb20hdc->dma_data[
					endpoint->usb_dma_channel].dma_buffer;
#else
		dma_buffer = (dma_addr_t)request->urb->transfer_dma +
						request->urb->actual_length;
#endif
		if (set_dma_transfer(endpoint,
			(dma_addr_t)f_usb20hdc->dma_data[
				endpoint->usb_dma_channel].epbuf_dma_addr,
					(dma_addr_t)dma_buffer, bytes)) {
			/* set request execute */
			request->request_execute = 1;

			/* start DMA transfer */
			hdmac_start(f_usb20hdc->dma_data[
				endpoint->usb_dma_channel].hdmac_channel);
			set_dma_st(base_addr, endpoint->usb_dma_channel, 1);
			if (start_flag == 0)
				set_start(base_addr, ep_channel);

			return 0;
		} else
			endpoint->transfer_status = -EPIPE;

done:
		set_dma_st(base_addr, endpoint->usb_dma_channel, 0);
		set_dma_inten(base_addr, endpoint->usb_dma_channel, 0);
		usb_settoggle(request->urb->dev,
				usb_pipeendpoint(request->urb->pipe), 0,
				get_toggle(base_addr, ep_channel));
		if (get_trans_en(base_addr, ep_channel) ||
			!get_empty(base_addr, ep_channel))
			f_usb20hdc->bulkin_remain =
					usb_pipedevice(request->urb->pipe);
		else
			f_usb20hdc->bulkin_remain = 0;
		return 1;
	}
#endif
	for (;;) {
		/* get this time index */
		index = get_appptr(base_addr, ep_channel);

		/* calculate transfer data pointer */
		transfer_data = (u32 *)
					(request->urb->transfer_buffer +
					request->urb->actual_length);
		prefetch(transfer_data);

		/* get bulk IN transfer byte */
		bytes = get_phycnt(base_addr, ep_channel *
				F_USB20HDC_EP_BUFFER_COUNT + index);
		if (request->urb->transfer_buffer_length <
					(request->urb->actual_length + bytes))
			bytes = request->urb->transfer_buffer_length -
					request->urb->actual_length;
		dev_dbg(hcd->self.controller,
			"endpoint %u PIO is end at length = %u, actual = %u, ",
			ep_channel, request->urb->transfer_buffer_length,
			request->urb->actual_length);
		dev_dbg(hcd->self.controller,
			"max packet = %u, this time = %u.\n",
			request->urb->ep->desc.wMaxPacketSize,
			(u32)bytes);

		/* check buffer read bytes */
		if (bytes)
			/* read bulk IN transfer data form buffer */
			read_epbuf(base_addr,
			    (endpoint->buffer_address_offset[0] +
			    (request->urb->ep->desc.wMaxPacketSize * index)),
							transfer_data, bytes);

		/* update actual bytes */
		request->urb->actual_length += bytes;

		/* enable next bulk IN transfer */
		set_bufrd(base_addr, ep_channel);

		/* check transfer request complete */
		if ((request->urb->actual_length >=
			request->urb->transfer_buffer_length) ||
			(bytes % request->urb->ep->desc.wMaxPacketSize) ||
								     (!bytes)) {
			/* complete request */
			set_trans_done_inten(base_addr, ep_channel, 0);
			usb_settoggle(request->urb->dev,
					usb_pipeendpoint(request->urb->pipe), 0,
					get_toggle(base_addr, ep_channel));
			endpoint->transfer_status = 0;

			return 1;
		}

		/* check next bulk IN transfer start */
		if (((request->urb->actual_length +
			request->urb->ep->desc.wMaxPacketSize) <
			request->urb->transfer_buffer_length) ||
				get_empty(base_addr, ep_channel)) {
			set_start(base_addr, ep_channel);
			break;
		}
	}

	return 0;
}

static u8 end_bulk_out_transfer(struct f_usb20hdc_hcd_ep *endpoint)
{
	struct f_usb20hdc_hcd *f_usb20hdc = endpoint->f_usb20hdc;
	struct usb_hcd *hcd = f_usb20hdc_to_hcd(f_usb20hdc);
	struct f_usb20hdc_hcd_req *request = endpoint->request;
	void *base_addr = f_usb20hdc->register_base_address;
	u8 ep_channel = endpoint->endpoint_channel;
	u32 *transfer_data;
	u8 index = get_appptr(base_addr, ep_channel);
	u32 bytes;
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	dma_addr_t dma_buffer;
#endif
	/* check STALL or HALT status */
	if (get_status_stall(base_addr, ep_channel) ||
		get_status_halt(base_addr, ep_channel)) {
		/* clear satus interrupt factor */
		clear_status_clr(base_addr, ep_channel);
		set_trans_done_inten(base_addr, ep_channel, 0);
		set_stop(base_addr, ep_channel);
		set_init(base_addr, ep_channel);
		set_toggle_clear(base_addr, ep_channel);
		endpoint->transfer_status = -EPIPE;
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
		if (request->urb->transfer_buffer_length != 0) {
			spin_unlock(&f_usb20hdc->lock);
			/* stop hdmac transfer */
			hdmac_stop_nowait(f_usb20hdc->dma_data[
				endpoint->usb_dma_channel].hdmac_channel);
			spin_lock(&f_usb20hdc->lock);

			if (endpoint->usb_dma_channel != -1)
				goto done;
		}
#endif
		return 1;
	}

#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	if ((endpoint->usb_dma_channel != -1)
		&& (request->urb->transfer_buffer_length != 0)) {
		/* get this time transfer byte */
		bytes = get_dma_tc(base_addr, endpoint->usb_dma_channel);

		/* update actual bytes */
		request->urb->actual_length += bytes;

		/* check transfer request complete */
		if (request->urb->actual_length >=
					request->urb->transfer_buffer_length) {
			endpoint->transfer_status = 0;
#if !defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
			/* check DMA transfer buffer unmap */
			if ((request->urb->transfer_dma != (dma_addr_t)0) &&
			       (request->urb->transfer_dma != ~(dma_addr_t)0)) {
				if (request->dma_transfer_buffer_map) {
					/*
					 *unmap DMA transfer buf and
					 *sync DMA transfer buf
					 */
					dma_unmap_single(hcd->self.controller,
						request->urb->transfer_dma,
						request->urb->actual_length,
						DMA_TO_DEVICE);
					request->urb->transfer_dma =
								~(dma_addr_t)0;
					request->dma_transfer_buffer_map = 0;
				} else {
					/* synchronize DMA transfer buffer */
					dma_sync_single_for_cpu(
						hcd->self.controller,
						request->urb->transfer_dma,
						request->urb->actual_length,
								DMA_TO_DEVICE);
				}
			}
#endif
			goto done;
		}

		/* calculate this time transfer byte */
		bytes = (request->urb->transfer_buffer_length -
				request->urb->actual_length) <
				F_USB20HDC_HCD_MAX_DMA_TRANSFER_SIZE ?
				(request->urb->transfer_buffer_length -
				request->urb->actual_length) :
				F_USB20HDC_HCD_MAX_DMA_TRANSFER_SIZE;
		dev_dbg(hcd->self.controller,
			"endpoint %u DMA is setup at length = %u, actual = %u, ",
			ep_channel, request->urb->transfer_buffer_length,
			request->urb->actual_length);
		dev_dbg(hcd->self.controller,
			"max packet = %u, this time = %u.\n",
			request->urb->ep->desc.wMaxPacketSize,
			(u32)bytes);

#if defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
		dma_buffer = f_usb20hdc->dma_data[
					endpoint->usb_dma_channel].dma_buffer;
#else
		dma_buffer = (dma_addr_t)request->urb->transfer_dma +
						request->urb->actual_length;
#endif
		if (set_dma_transfer(endpoint, (dma_addr_t)dma_buffer,
				(dma_addr_t)f_usb20hdc->dma_data[
				endpoint->usb_dma_channel].epbuf_dma_addr,
								       bytes)) {
#if defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
			/* copy bulk OUT transfer data to noncachable buffer */
			memcpy(f_usb20hdc->dma_data[
				endpoint->usb_dma_channel].buffer,
				(request->urb->transfer_buffer +
					request->urb->actual_length),
								bytes);
#endif

			/* set request execute */
			request->request_execute = 1;
			request->dmac_int_occurred = 0;
			request->usb_dma_int_occurred = 0;

			/* start DMA transfer */
			hdmac_start(f_usb20hdc->dma_data[
				endpoint->usb_dma_channel].hdmac_channel);
			set_dma_st(base_addr, endpoint->usb_dma_channel, 1);
			set_start(base_addr, ep_channel);

			return 0;
		} else {
#if !defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
			dev_dbg(hcd->self.controller,
				"%s():set_dma_transfer() is failed.\n",
				__func__);
			if (request->dma_transfer_buffer_map) {
				dma_unmap_single(hcd->self.controller,
					request->urb->transfer_dma,
					request->urb->transfer_buffer_length,
					DMA_TO_DEVICE);
				request->urb->transfer_dma = ~(dma_addr_t)0;
				request->dma_transfer_buffer_map = 0;
			}
#endif
			endpoint->transfer_status = -EPIPE;
		}

done:
		set_dma_st(base_addr, endpoint->usb_dma_channel, 0);
		set_dma_inten(base_addr, endpoint->usb_dma_channel, 0);
		usb_settoggle(request->urb->dev,
				usb_pipeendpoint(request->urb->pipe), 1,
				get_toggle(base_addr, ep_channel));

		return 1;
	}
#endif

	/* update actual bytes */
	request->urb->actual_length += (request->urb->transfer_buffer_length -
					request->urb->actual_length) <
					request->urb->ep->desc.wMaxPacketSize ?
					request->urb->transfer_buffer_length -
					request->urb->actual_length :
					request->urb->ep->desc.wMaxPacketSize;

	/* check transfer request complete */
	if (request->urb->actual_length >=
					request->urb->transfer_buffer_length) {
		/* complete request */
		set_trans_done_inten(base_addr, ep_channel, 0);
		usb_settoggle(request->urb->dev,
					usb_pipeendpoint(request->urb->pipe), 1,
					get_toggle(base_addr, ep_channel));
		endpoint->transfer_status = 0;
		return 1;
	}

	/* calculate transfer data pointer */
	transfer_data = (u32 *)
				(request->urb->transfer_buffer +
				request->urb->actual_length);
	prefetch(transfer_data);

	/* calculate this time transfer byte */
	bytes = (request->urb->transfer_buffer_length -
			request->urb->actual_length) <
			request->urb->ep->desc.wMaxPacketSize ?
			request->urb->transfer_buffer_length -
			request->urb->actual_length :
			request->urb->ep->desc.wMaxPacketSize;
	dev_dbg(hcd->self.controller,
			"endpoint %u PIO is setup at length = %u, actual = %u, ",
			ep_channel, request->urb->transfer_buffer_length,
			request->urb->actual_length);
	dev_dbg(hcd->self.controller,
			"max packet = %u, this time = %u.\n",
			request->urb->ep->desc.wMaxPacketSize,
			(u32)bytes);

	/* check buffer write bytes */
	if (bytes)
		/* write bulk OUT transfer data to buffer */
		write_epbuf(base_addr, (endpoint->buffer_address_offset[0] +
			(request->urb->ep->desc.wMaxPacketSize * index)),
							transfer_data, bytes);

	/* notify buffer write bytes */
	set_appcnt(base_addr, ep_channel *
		F_USB20HDC_EP_BUFFER_COUNT + index, bytes);

	/* enable next bulk OUT transfer */
	set_bufwr(base_addr, ep_channel);

	set_start(base_addr, ep_channel);

	return 0;
}

static u8 end_interrupt_in_transfer(
	struct f_usb20hdc_hcd_ep *endpoint)
{
	struct f_usb20hdc_hcd *f_usb20hdc = endpoint->f_usb20hdc;
	struct usb_hcd *hcd = f_usb20hdc_to_hcd(f_usb20hdc);
	struct f_usb20hdc_hcd_req *request = endpoint->request;
	struct device *pdev = hcd->self.controller;
	void *base_addr = f_usb20hdc->register_base_address;
	u8 ep_channel = endpoint->endpoint_channel;
	u32 *transfer_data;
	u8 index;
	u32 bytes;

	dev_dbg(pdev, "%s() starts in %s (%s) in ep%d (to addr%d ep%d).",
		__func__, request->urb->dev->product,
		request->urb->dev->manufacturer, ep_channel,
		usb_pipedevice(request->urb->pipe),
		usb_pipeendpoint(request->urb->pipe));

	/* check STALL or HALT status */
	if (get_status_stall(base_addr, ep_channel) ||
		get_status_halt(base_addr, ep_channel)) {
		/* clear satus interrupt factor */
		dev_dbg(pdev,
			"%s() : end with error (errcnt:%d) in ep%d (to %s: addr%d ep%d).",
			__func__, get_errcnt(base_addr, ep_channel),
			ep_channel,
			request->urb->dev->product,
			usb_pipedevice(request->urb->pipe),
			usb_pipeendpoint(request->urb->pipe));
		clear_status_clr(base_addr, ep_channel);
		set_trans_done_inten(base_addr, ep_channel, 0);
		set_stop(base_addr, ep_channel);
		set_init(base_addr, ep_channel);
		set_toggle_clear(base_addr, ep_channel);
		endpoint->transfer_status = -EPIPE;
		return 1;
	}

	for (;;) {
		/* get this time index */
		index = get_appptr(base_addr, ep_channel);

		/* calculate transfer data pointer */
		transfer_data = (u32 *)
					(request->urb->transfer_buffer +
					request->urb->actual_length);
		prefetch(transfer_data);

		/* get Interrupt IN transfer byte */
		bytes = get_phycnt(base_addr, ep_channel *
				F_USB20HDC_EP_BUFFER_COUNT + index);
		if (request->urb->transfer_buffer_length <
					(request->urb->actual_length + bytes))
			bytes = request->urb->transfer_buffer_length -
				request->urb->actual_length;
		dev_dbg(hcd->self.controller,
			"endpoint %u PIO is end at length = %u, actual = %u, ",
			ep_channel, request->urb->transfer_buffer_length,
			request->urb->actual_length);
		dev_dbg(hcd->self.controller,
			"max packet = %u, this time = %u.\n",
			request->urb->ep->desc.wMaxPacketSize,
			(u32)bytes);

		/* check buffer read bytes */
		if (bytes)
			/* read interrupt IN transfer data form buffer */
			read_epbuf(base_addr,
			    (endpoint->buffer_address_offset[0] +
			    (request->urb->ep->desc.wMaxPacketSize * index)),
							  transfer_data, bytes);

		/* update actual bytes */
		request->urb->actual_length += bytes;

		/* enable next interrupt IN transfer */
		set_bufrd(base_addr, ep_channel);

		/* check transfer request complete */
		if ((request->urb->actual_length >=
			request->urb->transfer_buffer_length) ||
			(bytes % request->urb->ep->desc.wMaxPacketSize) ||
			(!bytes)) {
			/* complete request */
			set_trans_done_inten(base_addr, ep_channel, 0);
			usb_settoggle(request->urb->dev,
					usb_pipeendpoint(request->urb->pipe), 0,
					get_toggle(base_addr, ep_channel));
			endpoint->transfer_status = 0;
			dev_dbg(pdev,
				"%s() : end with transfer complete in ep%d (to %s: addr%d ep%d).",
				__func__,
				ep_channel, request->urb->dev->product,
				usb_pipedevice(request->urb->pipe),
				usb_pipeendpoint(request->urb->pipe));
			dev_dbg(pdev, "%s() : actual_length is %d.", __func__,
				request->urb->actual_length);
			return 1;
		}

		/* check next interrupt IN transfer start */
		if (((request->urb->actual_length +
			request->urb->ep->desc.wMaxPacketSize) <
			request->urb->transfer_buffer_length) ||
			(get_empty(base_addr, ep_channel))) {
			dev_dbg(pdev,
				"%s() : end with not complete (errcnt:%d in ep%d) (to %s: addr%d ep%d)",
				__func__, get_errcnt(base_addr, ep_channel),
				ep_channel,
				request->urb->dev->product,
				usb_pipedevice(request->urb->pipe),
				usb_pipeendpoint(request->urb->pipe));
			dev_dbg(pdev, "%s() : actual_length is %d.", __func__,
				request->urb->actual_length);

			set_start(base_addr, ep_channel);
			break;
		} else
			dev_dbg(pdev,
				"%s() : transfer not complete (errcnt:%d in ep%d) (to %s: addr%d ep%d)",
				__func__, get_errcnt(base_addr, ep_channel),
				ep_channel, request->urb->dev->product,
				usb_pipedevice(request->urb->pipe),
				usb_pipeendpoint(request->urb->pipe));
	}

	/*means there is still transfers later.*/
	return 0;
}

static u8 end_interrupt_out_transfer(
	struct f_usb20hdc_hcd_ep *endpoint)
{
	struct f_usb20hdc_hcd *f_usb20hdc = endpoint->f_usb20hdc;
	struct usb_hcd *hcd = f_usb20hdc_to_hcd(f_usb20hdc);
	struct f_usb20hdc_hcd_req *request = endpoint->request;
	void *base_addr = f_usb20hdc->register_base_address;
	u8 ep_channel = endpoint->endpoint_channel;
	u32 *transfer_data;
	u8 index = get_appptr(base_addr, ep_channel);
	u32 bytes;

	/* check STALL or HALT status */
	if (get_status_stall(base_addr, ep_channel) ||
		get_status_halt(base_addr, ep_channel)) {
		/* clear satus interrupt factor */
		clear_status_clr(base_addr, ep_channel);
		set_trans_done_inten(base_addr, ep_channel, 0);
		set_stop(base_addr, ep_channel);
		set_init(base_addr, ep_channel);
		set_toggle_clear(base_addr, ep_channel);
		endpoint->transfer_status = -EPIPE;
		return 1;
	}

	/* update actual bytes */
	request->urb->actual_length += (request->urb->transfer_buffer_length -
					request->urb->actual_length) <
					request->urb->ep->desc.wMaxPacketSize ?
					request->urb->transfer_buffer_length -
					request->urb->actual_length :
					request->urb->ep->desc.wMaxPacketSize;

	/* check transfer request complete */
	if (request->urb->actual_length >=
					request->urb->transfer_buffer_length) {
		/* complete request */
		set_trans_done_inten(base_addr, ep_channel, 0);
		usb_settoggle(request->urb->dev,
				usb_pipeendpoint(request->urb->pipe), 1,
				get_toggle(base_addr, ep_channel));
		endpoint->transfer_status = 0;
		return 1;
	}

	/* calculate transfer data pointer */
	transfer_data = (u32 *)(request->urb->transfer_buffer +
						request->urb->actual_length);
	prefetch(transfer_data);

	/* calculate this time transfer byte */
	bytes = (request->urb->transfer_buffer_length -
			request->urb->actual_length) <
			request->urb->ep->desc.wMaxPacketSize ?
			request->urb->transfer_buffer_length -
			request->urb->actual_length :
			request->urb->ep->desc.wMaxPacketSize;
	dev_dbg(hcd->self.controller,
		"endpoint %u PIO is setup at length = %u, actual = %u, ",
		ep_channel, request->urb->transfer_buffer_length,
		request->urb->actual_length);
	dev_dbg(hcd->self.controller,
		"max packet = %u, this time = %u.\n",
		request->urb->ep->desc.wMaxPacketSize, (u32)bytes);

	/* check buffer write bytes */
	if (bytes)
		/* write interrupt OUT transfer data to buffer */
		write_epbuf(base_addr, (endpoint->buffer_address_offset[0] +
			(request->urb->ep->desc.wMaxPacketSize * index)),
							transfer_data, bytes);

	/* notify buffer write bytes */
	set_appcnt(base_addr, ep_channel * F_USB20HDC_EP_BUFFER_COUNT +
			index, bytes);

	/* enable next interrupt OUT transfer */
	set_bufwr(base_addr, ep_channel);

	set_start(base_addr, ep_channel);

	return 0;
}

static u8 end_isochronous_in_transfer(
	struct f_usb20hdc_hcd_ep *endpoint)
{
	struct f_usb20hdc_hcd *f_usb20hdc = endpoint->f_usb20hdc;
	struct usb_hcd *hcd = f_usb20hdc_to_hcd(f_usb20hdc);
	struct f_usb20hdc_hcd_req *request = endpoint->request;
	void *base_addr = f_usb20hdc->register_base_address;
	u8 ep_channel = endpoint->endpoint_channel;
	u32 *transfer_data;
	u8 index;
	u32 bytes;

	/* check HALT status */
	if (get_status_halt(base_addr, ep_channel)) {
		/* clear satus interrupt factor */
		clear_status_clr(base_addr, ep_channel);
		set_trans_done_inten(base_addr, ep_channel, 0);
		set_stop(base_addr, ep_channel);
		set_init(base_addr, ep_channel);
		set_toggle_clear(base_addr, ep_channel);
		endpoint->transfer_status = -EPIPE;
		return 1;
	}

	for (;;) {
		/* get this time index */
		index = get_appptr(base_addr, ep_channel);

		/* calculate transfer data pointer */
		transfer_data = (u32 *)
					(request->urb->transfer_buffer +
					request->urb->iso_frame_desc[
					endpoint->transfer_packets].offset);
		prefetch(transfer_data);

		/* get isochronous IN transfer byte */
		bytes = get_phycnt(base_addr, ep_channel *
				F_USB20HDC_EP_BUFFER_COUNT + index);
		if (request->urb->iso_frame_desc[
			endpoint->transfer_packets].length < bytes)
			bytes = request->urb->iso_frame_desc[
					endpoint->transfer_packets].length;
		dev_dbg(hcd->self.controller,
			"endpoint %u PIO is end at length = %u, actual = %u, max packet = %u, ",
			ep_channel,
			request->urb->iso_frame_desc[
				endpoint->transfer_packets].length,
			request->urb->iso_frame_desc[
				endpoint->transfer_packets].actual_length,
			request->maxpacket);
		dev_dbg(hcd->self.controller,
			"packets = %u, this time packet = %u, this time = %u.\n",
			request->urb->number_of_packets,
			(u32)endpoint->transfer_packets,
			(u32)bytes);

		/* check buffer read bytes */
		if (bytes)
			/* read isochronous IN transfer data form buffer */
			read_epbuf(base_addr,
				(endpoint->buffer_address_offset[0] +
				(request->maxpacket * index)),
				transfer_data, bytes);

		/* update actual bytes */
		request->urb->actual_length += bytes;
		request->urb->iso_frame_desc[
			endpoint->transfer_packets].actual_length = bytes;

		/* set request status */
		request->urb->iso_frame_desc[
					endpoint->transfer_packets].status = 0;

		/* update transfer packet count */
		endpoint->transfer_packets++;

		/* enable next isochronous IN transfer */
		set_bufrd(base_addr, ep_channel);

		/* check transfer request complete */
		if (request->urb->number_of_packets <=
						endpoint->transfer_packets) {
			/* complete request */
			set_trans_done_inten(base_addr, ep_channel, 0);
			usb_settoggle(request->urb->dev,
				usb_pipeendpoint(request->urb->pipe), 0, 0);
			endpoint->transfer_status = 0;
			return 1;
		}

		/* check next interrupt IN transfer start */
		if (((request->urb->actual_length +
				request->urb->ep->desc.wMaxPacketSize) <
				request->urb->transfer_buffer_length) ||
				(get_empty(base_addr, ep_channel))) {
			set_start(base_addr, ep_channel);
			break;
		}
	}

	return 0;
}

static u8 end_isochronous_out_transfer(
	struct f_usb20hdc_hcd_ep *endpoint)
{
	struct f_usb20hdc_hcd *f_usb20hdc = endpoint->f_usb20hdc;
	struct usb_hcd *hcd = f_usb20hdc_to_hcd(f_usb20hdc);
	struct f_usb20hdc_hcd_req *request = endpoint->request;
	void *base_addr = f_usb20hdc->register_base_address;
	u8 ep_channel = endpoint->endpoint_channel;
	u32 *transfer_data;
	u8 index = get_appptr(base_addr, ep_channel);
	u32 bytes;

	/* check HALT status */
	if (get_status_halt(base_addr, ep_channel)) {
		/* clear satus interrupt factor */
		clear_status_clr(base_addr, ep_channel);
		set_trans_done_inten(base_addr, ep_channel, 0);
		set_stop(base_addr, ep_channel);
		set_init(base_addr, ep_channel);
		set_toggle_clear(base_addr, ep_channel);
		endpoint->transfer_status = -EPIPE;
		return 1;
	}

	/* update actual bytes */
	request->urb->actual_length +=
		request->urb->iso_frame_desc[endpoint->transfer_packets].length;
	request->urb->iso_frame_desc[
		endpoint->transfer_packets].actual_length =
			request->urb->iso_frame_desc[
				endpoint->transfer_packets].length;

	/* set request status */
	request->urb->iso_frame_desc[endpoint->transfer_packets].status = 0;

	/* update transfer packet count */
	endpoint->transfer_packets++;

	/* check transfer request complete */
	if (request->urb->number_of_packets <= endpoint->transfer_packets) {
		/* complete request */
		set_trans_done_inten(base_addr, ep_channel, 0);
		usb_settoggle(request->urb->dev,
			usb_pipeendpoint(request->urb->pipe), 1, 0);
		endpoint->transfer_status = 0;
		return 1;
	}

	/* calculate transfer data pointer */
	transfer_data = (u32 *)(request->urb->transfer_buffer +
					request->urb->iso_frame_desc[
					endpoint->transfer_packets].offset);
	prefetch(transfer_data);

	/* calculate this time transfer byte */
	bytes = request->urb->iso_frame_desc[
				endpoint->transfer_packets].length <
			request->maxpacket ? request->urb->iso_frame_desc[
				endpoint->transfer_packets].length :
			request->maxpacket;
	dev_dbg(hcd->self.controller,
		"endpoint %u PIO is setup at length = %u, actual = %u, ",
		ep_channel,
		request->urb->iso_frame_desc[
				endpoint->transfer_packets].length,
		request->urb->iso_frame_desc[
				endpoint->transfer_packets].actual_length);
	dev_dbg(hcd->self.controller,
		"max packet = %u, packets = %u, this time packet = %u.\n",
		request->maxpacket, request->urb->number_of_packets,
		(u32)endpoint->transfer_packets);

	/* check buffer write bytes */
	if (bytes)
		/* write isochronous OUT transfer data to buffer */
		write_epbuf(base_addr, (endpoint->buffer_address_offset[0] +
				(request->maxpacket * index)),
							transfer_data, bytes);

	/* notify buffer write bytes */
	set_appcnt(base_addr, ep_channel * F_USB20HDC_EP_BUFFER_COUNT +
			index, bytes);

	/* enable next isochronous OUT transfer */
	set_bufwr(base_addr, ep_channel);

	set_start(base_addr, ep_channel);

	return 0;
}

static void on_end_transfer(struct f_usb20hdc_hcd_ep *endpoint)
{
	static u8 (* const set_transfer_function[])(
						struct f_usb20hdc_hcd_ep *) = {
		set_control_transfer,
		set_bulk_in_transfer,
		set_bulk_out_transfer,
		set_interrupt_in_transfer,
		set_interrupt_out_transfer,
		set_isochronous_in_transfer,
		set_isochronous_out_transfer,
	};
	static u8 (* const end_transfer_function[])(
						struct f_usb20hdc_hcd_ep *) = {
		end_control_transfer,
		end_bulk_in_transfer,
		end_bulk_out_transfer,
		end_interrupt_in_transfer,
		end_interrupt_out_transfer,
		end_isochronous_in_transfer,
		end_isochronous_out_transfer,
	};
	struct f_usb20hdc_hcd *f_usb20hdc = endpoint->f_usb20hdc;
	struct usb_hcd *hcd = f_usb20hdc_to_hcd(f_usb20hdc);
	struct device *dev = hcd->self.controller;
	struct f_usb20hdc_hcd_req *request = endpoint->request;
	u8 ep_channel = endpoint->endpoint_channel;
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	struct list_head *p;
#endif
	dev_dbg(dev, "%s() is started. (caller:%pS)",
		__func__,  __builtin_return_address(0));

	/* check transfer continue */
	if (!end_transfer_function[endpoint->transfer_type](endpoint)) {
		/* to be continued */
		dev_dbg(dev, "%s() is ended.", __func__);
		return;
	}

	/* check dynamic allocated endpoint still necessary */
	if (usb_pipetype(endpoint->request->urb->pipe) == PIPE_INTERRUPT)
		f_usb20hdc->eps_inuse_map &=
					~(0x0001 << endpoint->endpoint_channel);
	else if (usb_pipetype(endpoint->request->urb->pipe) ==
		PIPE_ISOCHRONOUS && list_is_singular(&endpoint->queue)) {
		f_usb20hdc->eps_inuse_map &=
					~(0x0001 << endpoint->endpoint_channel);
		endpoint->inuse_devnum = 0;
		endpoint->inuse_epnum = 0;
	}

	/* notify request complete */
	notify_transfer_request_complete(endpoint, request,
						endpoint->transfer_status);

	/* check if next queue empty or disconnect*/
	if (list_empty(&endpoint->queue) ||
		!get_port_connection_rhs(f_usb20hdc->register_base_address)) {
		dev_dbg(dev, "%s() is ended.", __func__);
		return;
	}

	if (endpoint->request && endpoint->request->request_execute) {
		dev_dbg(dev, "%s() is ended.", __func__);
		return;
	}

	/* get next request */
	request = list_entry(endpoint->queue.next,
					struct f_usb20hdc_hcd_req, queue);
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	if (endpoint->transfer_type == TRANSFER_TYPE_BULK_IN &&
		(f_usb20hdc->bulkin_remain != 0)) {
		list_for_each(p, &endpoint->queue) {
			request = list_entry(p, struct f_usb20hdc_hcd_req,
					queue);
			if ((request) && (f_usb20hdc->bulkin_remain ==
					usb_pipedevice(request->urb->pipe)))
				break;
		}

		if (f_usb20hdc->bulkin_remain !=
				usb_pipedevice(request->urb->pipe)) {
			dev_dbg(dev, "%s() is ended.", __func__);
			return;
		}
	}
#endif
	dev_dbg(dev, "%s(): endpoint %u is next queue at request = 0x%p, ",
			__func__, ep_channel, request);
	dev_dbg(dev, "%s(): transfer_buffer_length = %u, transfer_buffer = 0x%p.\n",
			__func__, request->urb->transfer_buffer_length,
			request->urb->transfer_buffer);

	/* save request */
	endpoint->request = request;

	/* check the got next request a request under current execution */
	if (request->request_execute) {
		dev_dbg(dev, "%s() is ended.", __func__);
		return;
	}

	/* set transfer request */
	dev_dbg(dev, "%s(): call the \"set transfer\"\n", __func__);
	set_transfer_function[endpoint->transfer_type](endpoint);
	dev_dbg(dev, "%s(): end call of \"set transfer\"\n", __func__);

	dev_dbg(dev, "%s() is ended.", __func__);
	return;
}

#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
static void end_dma_transfer(u32 channel, void *data, int state)
{
	static u8 (* const set_transfer_function[])(
						struct f_usb20hdc_hcd_ep *) = {
		set_control_transfer,
		set_bulk_in_transfer,
		set_bulk_out_transfer,
		set_interrupt_in_transfer,
		set_interrupt_out_transfer,
		set_isochronous_in_transfer,
		set_isochronous_out_transfer,
	};
	struct f_usb20hdc_hcd_ep *endpoint = data;
	struct f_usb20hdc_hcd *f_usb20hdc = endpoint->f_usb20hdc;
	struct usb_hcd *hcd = f_usb20hdc_to_hcd(f_usb20hdc);
	struct device *dev = hcd->self.controller;
	struct f_usb20hdc_hcd_req *request;
	void *base_addr = f_usb20hdc->register_base_address;
	u8 ep_channel = endpoint->endpoint_channel;
	s8 usb_dma_channel = endpoint->usb_dma_channel;
#if !defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
	enum dma_data_direction dir;
#endif
	dev_dbg(dev, "%s() is started. (caller:%pS)", __func__,
		__builtin_return_address(0));

	spin_lock(&f_usb20hdc->lock);

	/* check argument */
	if (unlikely(!endpoint || !endpoint->request || channel !=
			f_usb20hdc->dma_data[usb_dma_channel].hdmac_channel)) {
		spin_unlock(&f_usb20hdc->lock);
		return;
	}

	request = endpoint->request;

	/* DMA transfer end state factor */
	switch (state) {
	case HDMACB_SS_ADD_OVERFLOW:
	case HDMACB_SS_SOURCE_ACCESS_ERROR:
	case HDMACB_SS_DESTINATION_ACCESS_ERROR:
		dev_err(dev,
				"%s() endpoint %u is abort at request = 0x%p, actual_length = %u, ",
				__func__, ep_channel, request,
				request->urb->actual_length);
		dev_err(dev,
				 "%s() transfer_buffer = 0x%p, state = %x.\n",
				__func__, request->urb->transfer_buffer, state);
		/* disable DMA transfer */
		set_dma_st(base_addr, usb_dma_channel, 0);
		set_dma_inten(base_addr, usb_dma_channel, 0);

#if !defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
		if (usb_pipein(request->urb->pipe))
			dir = DMA_FROM_DEVICE;
		else
			dir = DMA_TO_DEVICE;
		/* check DMA transfer buffer unmap */
		if ((request->urb->transfer_dma != (dma_addr_t)0) &&
			(request->urb->transfer_dma != ~(dma_addr_t)0)) {
			if (request->dma_transfer_buffer_map) {
				/*
				 * unmap DMA transfer buf and
				 * sync DMA transfer buf.
				 */
				dma_unmap_single(hcd->self.controller,
					request->urb->transfer_dma,
					request->urb->actual_length,
					dir);
				request->urb->transfer_dma =
							~(dma_addr_t)0;
				request->dma_transfer_buffer_map = 0;
			} else {
				/* synchronize DMA transfer buffer */
				dma_sync_single_for_cpu(
					hcd->self.controller,
					request->urb->transfer_dma,
					request->urb->actual_length,
					dir);
			}
		}
#endif

		/* notify request complete */
		notify_transfer_request_complete(endpoint, request, -EPIPE);

		/* check if next queue empty or disconnect*/
		if (list_empty(&endpoint->queue)
			|| !get_port_connection_rhs(base_addr))
			break;

		/* get next request */
		request = list_entry(endpoint->queue.next,
					struct f_usb20hdc_hcd_req, queue);
		dev_dbg(dev,
			"%s(): endpoint %u is next queue at request = 0x%p, ",
			__func__, ep_channel, request);
		dev_dbg(dev,
			"%s(): transfer_buffer_length = %u, transfer_buffer = 0x%p.\n",
			__func__, request->urb->transfer_buffer_length,
			request->urb->transfer_buffer);

		/* save request */
		endpoint->request = request;

		/* check the next request is under current execution */
		if (request->request_execute)
			break;

		/* set transfer request */
		set_transfer_function[endpoint->transfer_type](endpoint);

		break;
	case HDMACB_SS_TRANSFER_STOP_REQUEST:
	case HDMACB_SS_NORMAL_END:
		dev_dbg(dev, "%s(): DMA channel %u interrupt is occurred.\n",
				__func__, (u32)usb_dma_channel);
		/* process in transfer end */
		if (usb_pipein(request->urb->pipe)) {
			dev_dbg(dev, "%s(): urb->pipe is IN.\n", __func__);
			on_end_transfer(endpoint);
		} else {
			/*
			 * process of transfer end need be done
			 * when usb controller's dma interrupt is handled before
			 * hdmac's interrupt,
			 */
			dev_dbg(dev, "%s(): urb->pipe is OUT.\n", __func__);
			endpoint->request->request_execute = 0;
			endpoint->request->dmac_int_occurred = 1;
			if (endpoint->request->usb_dma_int_occurred == 1)
				on_end_transfer(endpoint);
		}
		break;
	default:
		break;
	}

	spin_unlock(&f_usb20hdc->lock);

	dev_dbg(dev, "%s() is ended.\n", __func__);

	return;
}
#endif

static irqreturn_t f_usb20hdc_hcd_irq(struct usb_hcd *hcd)
{
	u32 counter;
	struct f_usb20hdc_hcd *f_usb20hdc;
	void *base_addr;
	struct device *dev = hcd->self.controller;
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	struct f_usb20hdc_hcd_ep *endpoint;
#endif

	dev_dbg(dev, "%s() is started.\n", __func__);

	/* check argument */
	if (unlikely(!hcd)) {
		dev_dbg(dev, "%s() is ended at non process.\n", __func__);
		return IRQ_NONE;
	}

	/* get parameter */
	f_usb20hdc = hcd_to_f_usb20hdc(hcd);
	base_addr = f_usb20hdc->register_base_address;

	/* check F_USB20HDC controller interrupt request assert */
	if (unlikely(hcd->irq != f_usb20hdc->irq)) {
		dev_dbg(dev, "%s() is ended at non process.\n", __func__);
		return IRQ_NONE;
	}

	/* check F_USB20HDC controller host mode usage */
	if (!is_host_mode_usage(f_usb20hdc)) {
		dev_dbg(dev, "%s() is ended at non process.\n", __func__);
		return IRQ_NONE;
	}

	/* check F_USB20HDC controller host interrupt request assert */
	if ((get_host_int(base_addr)) &&
		(!get_host_inten(base_addr))) {
		dev_dbg(dev, "%s() is ended at non process.\n", __func__);
		return IRQ_NONE;
	}

	/* get spin lock */
	spin_lock(&f_usb20hdc->lock);

	/* port status change interrupt factor */
	if (get_dport_evt(base_addr)) {

#ifdef CONFIG_PM_RUNTIME
		/* queue a HC resume work */
		if (hcd->state == HC_STATE_SUSPENDED) {
			dev_info(dev, "port event is going to wake up HC.\n");
			usb_hcd_resume_root_hub(hcd);
		}
#endif
		/* port connection change interrupt factor */
		if ((get_port_connection_c(base_addr)) &&
			(get_port_connection_c_inten(base_addr))) {

			dev_info(dev, "port connection change interrupt is occurred.\n");

			/* disable port connection change interrupt */
			dbg_print_connection(hcd->self.controller, base_addr);
			set_port_connection_c_inten(base_addr, 0);
			dbg_print_connection(hcd->self.controller, base_addr);

			/* process root hub status polling */
			spin_unlock(&f_usb20hdc->lock);
			usb_hcd_poll_rh_status(hcd);

			dev_dbg(dev, "%s() is ended.\n",  __func__);
			return IRQ_HANDLED;
		}

		/* port enable change interrupt factor */
		if ((get_port_enable_c(base_addr)) &&
			(get_port_enable_c_inten(base_addr))) {

			dev_info(dev, "port enable change interrupt is occurred.\n");

			/* disable port enable change interrupt */
			set_port_enable_c_inten(base_addr, 0);

			/* process root hub status polling */
			spin_unlock(&f_usb20hdc->lock);
			usb_hcd_poll_rh_status(hcd);

			dev_dbg(dev, "%s() is ended.\n", __func__);
			return IRQ_HANDLED;
		}

		/* port suspend change interrupt factor */
		if ((get_port_suspend_c(base_addr)) &&
			(get_port_suspend_c_inten(base_addr))) {

			dev_dbg(dev, "port suspend change interrupt is occurred.\n");

			/* disable port suspend change interrupt */
			set_port_suspend_c_inten(base_addr, 0);

			/* process root hub status polling */
			spin_unlock(&f_usb20hdc->lock);
			usb_hcd_poll_rh_status(hcd);

			dev_dbg(dev, "%s() is ended.\n", __func__);
			return IRQ_HANDLED;
		}

		/* port over current change interrupt factor */
		if ((get_port_ov_curr_c(base_addr)) &&
			(get_port_ov_curr_c_inten(base_addr))) {

			dev_dbg(dev, "port over current change  interrupt is occurred.\n");

			/* disable port over current change interrupt */
			set_port_ov_curr_c_inten(base_addr, 0);

			/* process root hub status polling */
			spin_unlock(&f_usb20hdc->lock);
			usb_hcd_poll_rh_status(hcd);

			dev_dbg(dev, "%s() is ended.\n", __func__);
			return IRQ_HANDLED;
		}

		/* port reset change interrupt factor */
		if ((get_port_reset_c(base_addr)) &&
			(get_port_reset_c_inten(base_addr))) {

			dev_dbg(dev, "port reset change interrupt is occurred.\n");

			/* disable port reset change interrupt */
			set_port_reset_c_inten(base_addr, 0);

			/* process root hub status polling */
			spin_unlock(&f_usb20hdc->lock);
			usb_hcd_poll_rh_status(hcd);

			dev_dbg(dev, "%s() is ended.\n", __func__);
			return IRQ_HANDLED;
		}
	}

#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	/* DMA channel x interrupt factor */
	for (counter = F_USB20HDC_DMA_CH1;
			counter < F_USB20HDC_MAX_DMA_CHANNELS; counter++) {
		if ((get_dma_int(base_addr, counter)) &&
			(get_dma_inten(base_addr, counter))) {
			dev_dbg(dev, "%s(): DMA channel %u interrupt is occurred.\n",
				__func__, counter);

			/* clear trans_done interrupt factor */
			clear_dma_int(base_addr, counter);

			endpoint = &f_usb20hdc->endpoint[f_usb20hdc->dma_data[
						counter].endpoint_channel];

			if (!endpoint->request) {
				spin_unlock(&f_usb20hdc->lock);
				dev_dbg(dev, "%s() is ended.\n", __func__);
				return IRQ_HANDLED;
			}

			if (!usb_pipein(endpoint->request->urb->pipe)) {
				/*
				 * make sure the process of transfer end is done
				 * after hdmac's interrupt occurred,
				 * because the hdmac driver need to change its
				 * state at interrupt handler.
				 */
				dev_dbg(dev, "%s(): urb->pipe is OUT.\n",
					__func__);
				endpoint->request->usb_dma_int_occurred = 1;
				if (endpoint->request->dmac_int_occurred == 1)
					on_end_transfer(endpoint);
			} else {
				/* check transfer request complete */
				dev_dbg(dev, "%s(): urb->pipe is IN.\n",
					__func__);
				if ((get_dma_tc(base_addr, counter) <
					get_dma_tci(base_addr, counter)) &&
					get_dma_sp(base_addr, counter)) {

					/* stop hdmac transfer */
					spin_unlock(&f_usb20hdc->lock);
					dev_dbg(dev, "%s(): call hdmac_stop_nowait().\n",
						__func__);
					hdmac_stop_nowait(f_usb20hdc->dma_data[
							counter].hdmac_channel);
					spin_lock(&f_usb20hdc->lock);
				}
			}
			spin_unlock(&f_usb20hdc->lock);

			dev_dbg(dev, "%s() is ended.\n", __func__);
			return IRQ_HANDLED;
		}
	}
#endif

	/* endpoint x interrupt factor */
	for (counter = F_USB20HDC_EP0;
				counter < F_USB20HDC_HCD_MAX_EP; counter++) {
		if ((get_trans_done(base_addr, counter)) &&
			(get_trans_done_inten(base_addr, counter))) {
			dev_dbg(dev, "%s(): trans_done endpoint %u interrrupt is occurred.\n",
						__func__, (u32)counter);

			/* clear trans_done interrupt factor */
			clear_trans_done(base_addr, counter);

			if (!f_usb20hdc->endpoint[counter].request) {
				spin_unlock(&f_usb20hdc->lock);
				dev_dbg(dev, "%s() is ended.\n", __func__);
				return IRQ_HANDLED;
			}
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
			if ((f_usb20hdc->endpoint[counter].
					usb_dma_channel != -1) &&
				!get_status_stall(base_addr, counter) &&
				!get_status_halt(base_addr, counter) &&
				f_usb20hdc->endpoint[counter].
					request->urb->transfer_buffer_length) {
				spin_unlock(&f_usb20hdc->lock);
				dev_dbg(dev, "%s() is ended.\n", __func__);
				return IRQ_HANDLED;
			}
#endif
			/* process transfer end */
			on_end_transfer(&f_usb20hdc->endpoint[counter]);

			spin_unlock(&f_usb20hdc->lock);
			dev_dbg(dev, "%s() is ended.\n", __func__);
			return IRQ_HANDLED;
		}
	}

	/* release spin lock */
	spin_unlock(&f_usb20hdc->lock);

	dev_dbg(dev, "%s() is ended.\n", __func__);
	return IRQ_HANDLED;
}

static int f_usb20hdc_hcd_reset(struct usb_hcd *hcd)
{
	u32 counter;
	struct f_usb20hdc_hcd *f_usb20hdc;
	void *base_addr;
	unsigned long flags;

	/* check argument */
	if (unlikely(!hcd))
		return -EINVAL;

	dev_info(hcd->self.controller, "%s() is started.\n", __func__);

	/* get parameter */
	f_usb20hdc = hcd_to_f_usb20hdc(hcd);
	base_addr = f_usb20hdc->register_base_address;

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* check F_USB20HDC controller device mode usage */
	if (unlikely(is_device_mode_usage(f_usb20hdc))) {
		dev_err(hcd->self.controller, "%s():device mode is usage.\n",
								__func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_info(hcd->self.controller, "%s() is ended.\n", __func__);
		return -ESHUTDOWN;
	}

	/* halt F_USB20HDC controller */
	set_host_inten(base_addr, 0);
	set_port_connection_c_inten(base_addr, 0);
	set_port_enable_c_inten(base_addr, 0);
	set_port_suspend_c_inten(base_addr, 0);
	set_port_ov_curr_c_inten(base_addr, 0);
	set_port_reset_c_inten(base_addr, 0);
	for (counter = F_USB20HDC_EP0;
				counter < F_USB20HDC_HCD_MAX_EP; counter++) {
		set_trans_done_inten(base_addr, counter, 0);
		set_stop(base_addr, counter);
	}
	set_hcrun(base_addr, 0);

	/* initialize F_USB20HDC controller */
	initialize_controller(f_usb20hdc);

	/* disable port power */
	if (f_usb20hdc->wakeup_from_poweroff == 0)
		enable_bus_power_supply(base_addr, 0);

	/*
	 * Since a F_USB20HDC controller is a controller corresponding
	 * to dual speed,it sets a transaction translator flag
	 */
	hcd->has_tt = 1;

	f_usb20hdc->next_statechange = jiffies;

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dev_info(hcd->self.controller, "%s() is ended.\n", __func__);

	return 0;
}

static int f_usb20hdc_hcd_start(struct usb_hcd *hcd)
{
	u32 counter;
	struct f_usb20hdc_hcd *f_usb20hdc;
	void __iomem *base_addr;
	unsigned long flags;

	/* check argument */
	if (unlikely(!hcd))
		return -EINVAL;

	dev_info(hcd->self.controller, "%s() is started.\n", __func__);

	/* get parameter */
	f_usb20hdc = hcd_to_f_usb20hdc(hcd);
	base_addr = f_usb20hdc->register_base_address;

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* check F_USB20HDC controller device mode usage */
	if (unlikely(is_device_mode_usage(f_usb20hdc))) {
		dev_err(hcd->self.controller, "%s():device mode is usage.\n",
								__func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_info(hcd->self.controller, "%s() is ended.\n", __func__);
		return -ESHUTDOWN;
	}

	/* set hdc parameter */
	hcd->uses_new_polling = 1;
	hcd->state = HC_STATE_RUNNING;
	clear_bit(HCD_FLAG_POLL_RH, &hcd->flags);

	/* initialize endpoint configure */
	initialize_endpoint_configure(f_usb20hdc);

	/* start F_USB20HDC controller */
	set_dm_pull_down(base_addr, 1);
	set_dp_pull_down(base_addr, 1);
	set_host_en(base_addr, 1);
	enable_bus_power_supply(base_addr, 1);

	/* make the endpoint transfer link list */
	initialize_td_schedule_list(base_addr);
	for (counter = F_USB20HDC_EP0;
			counter < F_USB20HDC_HCD_MAX_EP; counter++) {
		set_nlinkinvalid(base_addr, counter, 0);
	}

	/* enable interrupt factor */
	set_host_inten(base_addr, 1);
	set_port_connection_c_inten(base_addr, 1);
	set_port_enable_c_inten(base_addr, 1);
	set_port_suspend_c_inten(base_addr, 1);
	set_port_ov_curr_c_inten(base_addr, 1);
	set_port_reset_c_inten(base_addr, 1);
	set_hcrun(base_addr, 1);

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dev_info(hcd->self.controller, "%s() is ended.\n", __func__);

	return 0;
}

static int f_usb20hdc_hcd_get_frame_number(struct usb_hcd *hcd)
{
	struct f_usb20hdc_hcd *f_usb20hdc;
	u16 frame_number;
	unsigned long flags;

	/* check argument */
	if (unlikely(!hcd))
		return -EINVAL;

	dev_dbg(hcd->self.controller, "%s() is started.\n", __func__);

	/* get parameter */
	f_usb20hdc = hcd_to_f_usb20hdc(hcd);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* check F_USB20HDC controller device mode usage */
	if (unlikely(is_device_mode_usage(f_usb20hdc))) {
		dev_err(hcd->self.controller, "%s():device mode is usage.\n",
								__func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(hcd->self.controller, "%s() is ended.\n", __func__);
		return -EPROTO;
	}

	/* get frame number */
	frame_number = get_sofv(f_usb20hdc->register_base_address);
	dev_dbg(hcd->self.controller, "frame number is %u.\n", frame_number);

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dev_dbg(hcd->self.controller, "%s() is ended.\n", __func__);

	return (int)frame_number;
}


static void f_usb20hdc_hcd_stop(struct usb_hcd *hcd)
{
	u32 counter;
	struct f_usb20hdc_hcd *f_usb20hdc;
	unsigned long flags;

	/* check argument */
	if (unlikely(!hcd))
		return;

	dev_info(hcd->self.controller, "%s() is started.\n", __func__);

	/* get parameter */
	f_usb20hdc = hcd_to_f_usb20hdc(hcd);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* check F_USB20HDC controller device mode usage */
	if (unlikely(is_device_mode_usage(f_usb20hdc))) {
		dev_err(hcd->self.controller, "%s():device mode is usage.\n",
								__func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_info(hcd->self.controller, "%s() is ended.\n", __func__);
		return;
	}

	/* disable interrupt factor */
	set_host_inten(f_usb20hdc->register_base_address, 0);
	set_port_connection_c_inten(f_usb20hdc->register_base_address, 0);
	set_port_enable_c_inten(f_usb20hdc->register_base_address, 0);
	set_port_suspend_c_inten(f_usb20hdc->register_base_address, 0);
	set_port_ov_curr_c_inten(f_usb20hdc->register_base_address, 0);
	set_port_reset_c_inten(f_usb20hdc->register_base_address, 0);
	for (counter = F_USB20HDC_EP0;
				counter < F_USB20HDC_HCD_MAX_EP; counter++)
		set_trans_done_inten(f_usb20hdc->register_base_address,
								counter, 0);

	/* clear interrupt factor */
	clear_port_connection_c(f_usb20hdc->register_base_address);
	clear_port_enable_c(f_usb20hdc->register_base_address);
	clear_port_suspend_c(f_usb20hdc->register_base_address);
	clear_port_ov_curr_c(f_usb20hdc->register_base_address);
	clear_port_reset_c(f_usb20hdc->register_base_address);
	for (counter = F_USB20HDC_EP0;
				counter < F_USB20HDC_HCD_MAX_EP; counter++)
		clear_trans_done(f_usb20hdc->register_base_address, counter);

	/* stop F_USB20HDC controller */
	set_hcrun(f_usb20hdc->register_base_address, 0);

	/* reset F_USB20HDC controller */
	initialize_controller(f_usb20hdc);

	/* dequeue all previous transfer request */
	for (counter = F_USB20HDC_EP0;
				counter < F_USB20HDC_HCD_MAX_EP; counter++)
		dequeue_all_transfer_request(&f_usb20hdc->endpoint[counter],
								-ESHUTDOWN);

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dev_info(hcd->self.controller, "%s() is ended.\n", __func__);

	return;
}

static int f_usb20hdc_hcd_urb_enqueue(struct usb_hcd *hcd, struct urb *urb,
	gfp_t mem_flags)
{
	static u8 (* const set_transfer_function[])(
						struct f_usb20hdc_hcd_ep *) = {
		set_control_transfer,
		set_bulk_in_transfer,
		set_bulk_out_transfer,
		set_interrupt_in_transfer,
		set_interrupt_out_transfer,
		set_isochronous_in_transfer,
		set_isochronous_out_transfer,
	};
	struct f_usb20hdc_hcd *f_usb20hdc;
	struct f_usb20hdc_hcd_req *request;
	struct f_usb20hdc_hcd_ep *endpoint;
	struct usb_ctrlrequest *dev_req;
	u16 eps_inuse_mask = 0, maxp = 0;
	u8 endpoint_channel;
	u8 dynamic_alloc = 0;
	unsigned long flags;
	int result;
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	struct list_head *p;
	struct f_usb20hdc_hcd_req *req;
#endif
	struct device *pdev;
#ifdef VIRTUAL_DEVICE_ADDRESS
	int addr;
#endif

	/* check argument */
	if (unlikely((!hcd) || (!urb)))
		return -EINVAL;

	/* get parameter */
	f_usb20hdc = hcd_to_f_usb20hdc(hcd);
	pdev = hcd->self.controller;

	dbg_print(pdev, "%s() is started.\n", __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* check F_USB20HDC controller device mode usage */
	if (unlikely(is_device_mode_usage(f_usb20hdc))) {
		dev_err(pdev, "%s():device mode is usage.\n",
								__func__);
		result = -ESHUTDOWN;
		goto error_not_linked;
	}

	/* check if bus disconnect*/
	if ((!get_port_connection_rhs(f_usb20hdc->register_base_address)) ||
		(urb->dev->state == USB_STATE_NOTATTACHED)) {
		result = -ESHUTDOWN;
		goto error_not_linked;
	}

	/* check parameter */
	if (unlikely((!urb->dev) || (!urb->ep) ||
		((urb->transfer_buffer_length) && (!urb->transfer_buffer)) ||
		((usb_pipetype(urb->pipe) == PIPE_CONTROL) &&
			(!urb->setup_packet)))) {
		dev_err(pdev, "%s():request parameter is error.\n",
			__func__);
		result = -EINVAL;
		goto error_not_linked;
	}

	/* solution for restriction of only 4 bits device address in this IP */
	if (urb->setup_packet) {
		dev_req = (struct usb_ctrlrequest *)urb->setup_packet;
		if ((dev_req->bRequestType == 0x0) &&
					(dev_req->bRequest == 0x5)) {
#ifdef VIRTUAL_DEVICE_ADDRESS
			/* setup address request */
			addr = create_mapped_addr(hcd, urb, dev_req->wValue);
			if (addr < 0) {
				dev_err(pdev, "%s() : logical address %d can't be mapped.",
					__func__, dev_req->wValue);
				result = -EINVAL;
				goto error_not_linked;
			} else {
				dev_req->wValue = addr;
				dbg_print(pdev,
					"%s() : mapped address %d is going to set.",
					__func__, dev_req->wValue);
			}
#else
			if (dev_req->wValue > 15) {
				dev_err(pdev, "%s() : logical address %d can't be set.",
					__func__, dev_req->wValue);
				result = -EINVAL;
				goto error_not_linked;
			}
#endif
		}
	}

	/* set URB parameter */
	urb->status = -EINPROGRESS;
	urb->actual_length = 0;
	urb->error_count = 0;

	/* link URB to ep */
	result = usb_hcd_link_urb_to_ep(hcd, urb);
	if (result)
		goto error_not_linked;

	/* set endpoint parameter */
	switch (usb_pipetype(urb->pipe)) {
	case PIPE_CONTROL:
		endpoint_channel = F_USB20HDC_EP0;
		endpoint = &f_usb20hdc->endpoint[endpoint_channel];
		endpoint->transfer_type = TRANSFER_TYPE_CONTROL;
		break;
	case PIPE_BULK:
		if (usb_pipein(urb->pipe)) {
			endpoint_channel = F_USB20HDC_EP1;
			endpoint = &f_usb20hdc->endpoint[endpoint_channel];
			endpoint->transfer_type = TRANSFER_TYPE_BULK_IN;
		} else {
			endpoint_channel = F_USB20HDC_EP2;
			endpoint = &f_usb20hdc->endpoint[endpoint_channel];
			endpoint->transfer_type = TRANSFER_TYPE_BULK_OUT;
		}
		break;
	case PIPE_INTERRUPT:
		for (endpoint_channel = F_USB20HDC_EP3;
				endpoint_channel < F_USB20HDC_HCD_MAX_EP;
				endpoint_channel++) {
			eps_inuse_mask = 0x0001 << endpoint_channel;
			if (!(f_usb20hdc->eps_inuse_map & eps_inuse_mask)) {
				f_usb20hdc->eps_inuse_map |= eps_inuse_mask;
				dynamic_alloc = 1;
				dbg_print(pdev,
					"%s():interrupt ep is dynamic.\n",
					__func__);
				break;
			}
		}
		if (endpoint_channel == F_USB20HDC_HCD_MAX_EP) {
			dev_err(pdev, "%s():endpoint is out of use.\n",
				__func__);
			result = -ENOSPC;
			goto error;
		}
		if (usb_pipein(urb->pipe))
			set_endpoint_configure(f_usb20hdc, endpoint_channel,
						TRANSFER_TYPE_INTERRUPT_IN);
		else
			set_endpoint_configure(f_usb20hdc, endpoint_channel,
						TRANSFER_TYPE_INTERRUPT_OUT);
		break;
	case PIPE_ISOCHRONOUS:
		for (endpoint_channel = F_USB20HDC_EP3;
				endpoint_channel < F_USB20HDC_HCD_MAX_EP;
				endpoint_channel++) {
			endpoint = &f_usb20hdc->endpoint[endpoint_channel];
			if (!list_empty(&endpoint->queue) &&
				(endpoint->inuse_devnum ==
					usb_pipedevice(urb->pipe)) &&
					(endpoint->inuse_epnum ==
						usb_pipeendpoint(urb->pipe))) {
				dynamic_alloc = 1;
				break;
			}
		}
		if (!dynamic_alloc) {
			for (endpoint_channel = F_USB20HDC_EP3;
				endpoint_channel < F_USB20HDC_HCD_MAX_EP;
							endpoint_channel++) {
				eps_inuse_mask = 0x0001 << endpoint_channel;
				if (!(f_usb20hdc->eps_inuse_map &
							eps_inuse_mask)) {
					f_usb20hdc->eps_inuse_map |=
							eps_inuse_mask;
					dynamic_alloc = 1;
					break;
				}
			}
		}
		if (endpoint_channel == F_USB20HDC_HCD_MAX_EP) {
			dev_err(pdev, "%s():endpoint is out of use\n",
				__func__);
			result = -ENOSPC;
			goto error;
		}
		f_usb20hdc->endpoint[endpoint_channel].inuse_devnum =
					usb_pipedevice(urb->pipe);
		f_usb20hdc->endpoint[endpoint_channel].inuse_epnum =
					usb_pipeendpoint(urb->pipe);
		if (usb_pipein(urb->pipe))
			set_endpoint_configure(f_usb20hdc, endpoint_channel,
						TRANSFER_TYPE_ISOCHRONOUS_IN);
		else
			set_endpoint_configure(f_usb20hdc, endpoint_channel,
						TRANSFER_TYPE_ISOCHRONOUS_OUT);
		break;
	default:
		dev_err(pdev, "%s():request parameter is error.\n", __func__);
		result = -EINVAL;
		goto error;
	}

	dbg_print(pdev,
		"%s():endpoint %d is used by %s (%s) and eps_inuse_map is 0x%x\n",
		__func__, endpoint_channel,
		urb->dev->product, urb->dev->manufacturer,
		f_usb20hdc->eps_inuse_map);

	/* get allocated endpoint */
	endpoint = &f_usb20hdc->endpoint[endpoint_channel];

	/* get the value of wMaxPacketSize */
	maxp = usb_maxpacket(urb->dev, urb->pipe, !usb_pipein(urb->pipe));
	if (is_highbandwidth(maxp)) {
		/* hardware does not support high bandwidth */
		dev_err(pdev, "%s(): Host does not support high bandwidth.\n",
			__func__);

		result = -EOPNOTSUPP;
		goto error;
	}

	/* check maximum packet size, only 0x07ff bits are for packet size */
	if (endpoint->max_packet_size < max_packet(maxp)) {
		dev_err(pdev, "%s():MaxPacketSize (%d) is over than (%d).\n",
			__func__, max_packet(maxp),
			endpoint->max_packet_size);

		result = -ENOSPC;
		goto error;
	}

	/* allocate request memory */
	request = allocate_request_memory(endpoint);
	if (!request) {
		result = -ENOMEM;
		goto error;
	}

	/* save URB */
	request->urb = urb;
	request->endpoint_channel = endpoint->endpoint_channel;
	request->maxpacket = max_packet(maxp);
	urb->hcpriv = request;

#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	/* check current queue execute */
	if (!list_empty(&endpoint->queue)) {
		if (endpoint->transfer_type != TRANSFER_TYPE_BULK_IN)
			goto add_list_tail;
		else {
			if (endpoint->request &&
					endpoint->request->request_execute)
				goto add_list_tail;
			else if (f_usb20hdc->bulkin_remain != 0 &&
						f_usb20hdc->bulkin_remain ==
						usb_pipedevice(urb->pipe)) {
					list_for_each(p, &endpoint->queue) {
					req = list_entry(p,
					struct f_usb20hdc_hcd_req, queue);
					if (f_usb20hdc->bulkin_remain ==
							usb_pipedevice(req->
								urb->pipe))
						goto add_list_tail;
				}
			} else
				goto add_list_tail;
		}
	} else if (endpoint->transfer_type == TRANSFER_TYPE_BULK_IN) {
		if (f_usb20hdc->bulkin_remain != 0 &&
				f_usb20hdc->bulkin_remain !=
					usb_pipedevice(urb->pipe))
			goto add_list_tail;
	}
#else
	/* check current queue execute */
	if (!list_empty(&endpoint->queue))
		goto add_list_tail;
#endif

	/* save request */
	endpoint->request = request;

	/* set transfer */
	set_transfer_function[endpoint->transfer_type](endpoint);

add_list_tail:
	/* add list tail */
	list_add_tail(&request->queue, &endpoint->queue);

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dbg_print(pdev, "%s() is ended.\n", __func__);

	return 0;
error:
	if (dynamic_alloc) {
		f_usb20hdc->eps_inuse_map &= ~eps_inuse_mask;
		if (usb_pipetype(urb->pipe) == PIPE_ISOCHRONOUS) {
			endpoint->inuse_devnum = 0;
			endpoint->inuse_epnum = 0;
		}
	}

	usb_hcd_unlink_urb_from_ep(hcd, urb);

error_not_linked:
	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
	dbg_print(pdev, "%s() is ended with err_not_link.\n", __func__);

	return result;
}

static int f_usb20hdc_hcd_urb_dequeue(struct usb_hcd *hcd, struct urb *urb,
	int status)
{
	struct f_usb20hdc_hcd *f_usb20hdc;
	struct f_usb20hdc_hcd_req *request;
	struct f_usb20hdc_hcd_ep *endpoint;
	void *base_addr;
	unsigned long flags;
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
#if !defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
	enum dma_data_direction dir;
#endif
#endif
	/* check argument */
	if (unlikely((!hcd) || (!urb)))
		return -EINVAL;

	dbg_print(hcd->self.controller, "%s() is started.\n", __func__);

	/* get parameter */
	f_usb20hdc = hcd_to_f_usb20hdc(hcd);
	base_addr = f_usb20hdc->register_base_address;

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* check F_USB20HDC controller device mode usage */
	if (unlikely(is_device_mode_usage(f_usb20hdc))) {
		dev_err(hcd->self.controller, "%s():device mode is usage.\n",
								__func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dbg_print(hcd->self.controller, "%s() is ended.\n", __func__);
		return -ESHUTDOWN;
	}

	/* ckeck URB unlink */
	if (usb_hcd_check_unlink_urb(hcd, urb, status)) {
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dbg_print(hcd->self.controller, "%s() is ended.\n", __func__);
		return 0;
	}

	request = urb->hcpriv;
	endpoint = &f_usb20hdc->endpoint[request->endpoint_channel];

	/* check list entry */
	list_for_each_entry(request, &endpoint->queue, queue) {
		if ((request) && (request->urb == urb))
			break;
	}

	/* check dequeue request mismatch */
	if (unlikely((!request) || (request->urb != urb))) {
		dev_err(hcd->self.controller, "%s():endpoint %u request is mismatch.\n",
				__func__,
				endpoint->endpoint_channel);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dbg_print(hcd->self.controller, "%s() is ended.\n", __func__);
		return -EINVAL;
	}

	/* abort request transfer */
	if (request->request_execute) {
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
		if (endpoint->usb_dma_channel != -1) {
			/* abort DMA transfer */
			set_dma_st(base_addr, endpoint->usb_dma_channel, 0);
			set_dma_inten(base_addr, endpoint->usb_dma_channel, 0);
			spin_unlock(&f_usb20hdc->lock);
			hdmac_stop_nowait(f_usb20hdc->dma_data[
				endpoint->usb_dma_channel].hdmac_channel);
			spin_lock(&f_usb20hdc->lock);
#if !defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
			if (usb_pipein(request->urb->pipe))
				dir = DMA_FROM_DEVICE;
			else
				dir = DMA_TO_DEVICE;
			/* check DMA transfer buffer unmap */
			if ((request->urb->transfer_dma != (dma_addr_t)0) &&
						(request->urb->transfer_dma !=
						~(dma_addr_t)0)) {
				if (request->dma_transfer_buffer_map) {
					/*
					 * unmap DMA transfer buf and
					 * sync DMA transfer buf.
					 */
					dma_unmap_single(hcd->self.controller,
						request->urb->transfer_dma,
						request->urb->actual_length,
						dir);
					request->urb->transfer_dma =
								~(dma_addr_t)0;
					request->dma_transfer_buffer_map = 0;
				} else {
					/* synchronize DMA transfer buffer */
					dma_sync_single_for_cpu(
						hcd->self.controller,
						request->urb->transfer_dma,
						request->urb->actual_length,
						dir);
				}
			}
#endif
		}
#endif
		set_trans_done_inten(base_addr, endpoint->endpoint_channel, 0);
		set_stop(base_addr, endpoint->endpoint_channel);
		set_init(base_addr, endpoint->endpoint_channel);
	}

	/* check dynamic allocated endpoint still necessary */
	if (usb_pipetype(urb->pipe) == PIPE_INTERRUPT)
		f_usb20hdc->eps_inuse_map &=
					~(0x0001 << endpoint->endpoint_channel);
	else if (usb_pipetype(urb->pipe) == PIPE_ISOCHRONOUS &&
					list_is_singular(&endpoint->queue)) {
		f_usb20hdc->eps_inuse_map &=
					~(0x0001 << endpoint->endpoint_channel);
		endpoint->inuse_devnum = 0;
		endpoint->inuse_epnum = 0;
	}

	/* notify request complete */
	notify_transfer_request_complete(endpoint, request, -ESHUTDOWN);

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dbg_print(hcd->self.controller, "%s() is ended.\n", __func__);

	return 0;
}

static int f_usb20hdc_hcd_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	struct f_usb20hdc_hcd *f_usb20hdc;
	struct device *dev = hcd->self.controller;
	unsigned long flags;

	/* check argument */
	if (unlikely((!hcd) || (!buf)))
		return 0;

	dev_dbg(dev, "%s() is started from %pS.\n", __func__,
		__builtin_return_address(0));

	/* if !USB_SUSPEND, root hub timers won't get shut down ... */
	if (!HC_IS_RUNNING(hcd->state)) {
		dev_info(dev, "%s() HC is not running. (status:0)\n", __func__);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return 0;
	}

	/* get parameter */
	f_usb20hdc = hcd_to_f_usb20hdc(hcd);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* check F_USB20HDC controller device mode usage */
	if (unlikely(is_device_mode_usage(f_usb20hdc))) {
		dev_err(dev, "%s():device mode is usage. (no status)\n",
								__func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return -ESHUTDOWN;
	}

	/* initialize hub status data */
	*buf = 0;

	/* check port status change */
	if (!get_dport_evt(f_usb20hdc->register_base_address)) {
		dev_dbg(dev, "%s() No port event. (status:0)\n", __func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return 0;
	}

	/* no hub change reports (bit 0) for now (power, ...) */
	*buf = 1 << 1;

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dev_dbg(dev, "%s() is ended.  (status:1<<1)\n", __func__);

	return 1;
}

static int f_usb20hdc_hcd_hub_control(struct usb_hcd *hcd, u16 type_req,
	u16 w_value, u16 w_index, char *buf, u16 w_length)
{
	u32 counter;
	struct f_usb20hdc_hcd *f_usb20hdc;
	void *base_addr;
	u32 status;
	u32 selector;
	unsigned long flags;
	struct device *pdev;

	/* check argument */
	if (unlikely((!hcd) || (!buf)))
		return -EINVAL;

	/* get parameter */
	f_usb20hdc = hcd_to_f_usb20hdc(hcd);
	base_addr = f_usb20hdc->register_base_address;
	pdev = hcd->self.controller;

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb20hdc->lock, flags);
	dev_dbg(pdev, "%s() is started from %pS.\n", __func__,
		__builtin_return_address(0));

	/* check F_USB20HDC controller device mode usage */
	if (unlikely(is_device_mode_usage(f_usb20hdc))) {
		dev_err(pdev, "%s():device mode is usage.\n",
								__func__);
		dev_dbg(pdev, "%s() is ended.\n", __func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		return -ESHUTDOWN;
	}

	switch (type_req) {
	case ClearHubFeature:
		switch (w_value) {
		case C_HUB_LOCAL_POWER:
		case C_HUB_OVER_CURRENT:
			/* no hub-wide feature/status flags */
			break;
		default:
			dev_dbg(pdev, "%s() is ended.\n", __func__);
			spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
			return -EPIPE;
		}
		break;
	case ClearPortFeature:
		if ((!w_index) || (w_index >
				F_USB20HDC_HCD_ROOT_HUB_MAX_PORT)) {
			dev_dbg(pdev, "%s() is ended.\n", __func__);
			spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
			return -EPIPE;
		}
		w_index--;

		switch (w_value) {
		case USB_PORT_FEAT_ENABLE:
			dev_dbg(pdev, "%s : Clear PORT_FEAT_ENABLE.\n",
				__func__);
			set_port_enable_req(base_addr, 0);
			break;
		case USB_PORT_FEAT_C_ENABLE:
			dev_dbg(pdev, "%s : Clear PORT_FEAT_C_ENABLE.\n",
				__func__);
			clear_port_enable_c(base_addr);
			set_port_enable_c_inten(base_addr, 1);
			break;
		case USB_PORT_FEAT_SUSPEND:
			break;
		case USB_PORT_FEAT_C_SUSPEND:
			dev_dbg(pdev, "%s : Clear PORT_FEAT_C_SUSPEND.\n",
				__func__);
			clear_port_suspend_c(base_addr);
			set_port_suspend_c_inten(base_addr, 1);
			break;
		case USB_PORT_FEAT_POWER:
			dev_info(pdev, "%s : Clear PORT_FEAT_POWER.\n",
				__func__);
			enable_bus_power_supply(base_addr, 0);
			break;
		case USB_PORT_FEAT_C_CONNECTION:
			dev_dbg(pdev, "%s : Clear PORT_FEAT_C_CONNECTION.\n",
				__func__);
			clear_port_connection_c(base_addr);
			set_port_connection_c_inten(base_addr, 1);
			break;
		case USB_PORT_FEAT_C_OVER_CURRENT:
			dev_dbg(pdev, "%s : Clear PORT_FEAT_C_OVER_CURRENT.\n",
				__func__);
			clear_port_ov_curr_c(base_addr);
			set_port_ov_curr_c_inten(base_addr, 1);
			break;
		case USB_PORT_FEAT_C_RESET:
			/* GetPortStatus clears reset */
			break;
		default:
			dev_dbg(pdev, "%s() is ended.\n", __func__);
			spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
			return -EPIPE;
		}
		break;
	case GetHubDescriptor:
		dev_dbg(pdev, "%s : GetHubDescriptor.\n", __func__);
		create_hub_descriptor(f_usb20hdc,
					(struct usb_hub_descriptor *)buf);
		break;
	case GetHubStatus:
		dev_dbg(pdev, "%s : GetHubStatus.\n", __func__);
		/* no hub-wide feature/status flags */
		memset(buf, 0, 4);
		break;
	case GetPortStatus:
		if ((!w_index) || (w_index >
				F_USB20HDC_HCD_ROOT_HUB_MAX_PORT)) {
			dev_dbg(pdev, "%s() is ended.\n", __func__);
			spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
			return -EPIPE;
		}
		w_index--;
		status = 0;

		if (get_port_connection_c(base_addr)
#ifdef CONFIG_PM_RUNTIME
						|| (f_usb20hdc->wakeup_req == 1)
#endif
						) {
#ifdef CONFIG_PM_RUNTIME
			if (f_usb20hdc->wakeup_req == 1) {
				dev_info(pdev, "%s : execute wakeup_req.\n",
				__func__);
				f_usb20hdc->wakeup_req = 0;
				/*
				 * disabling port make usb stack can run
				 * configuring device from scratch.
				 */
				set_port_enable_req(base_addr, 0);
				while (get_port_enable_rhs(base_addr))
					;
			}
#endif
			status |= USB_PORT_STAT_C_CONNECTION << 16;
			dev_dbg(pdev, "%s : GetPortStatus : connection change!.\n",
				__func__);
			if (get_port_connection_rhs(base_addr)) {
				dev_dbg(pdev, "%s : GetPortStatus : connected!.\n",
					__func__);
				for (counter = F_USB20HDC_EP1;
					counter < F_USB20HDC_HCD_MAX_EP;
					counter++)
					set_toggle_clear(base_addr, counter);
			} else {
				dev_dbg(pdev, "%s : GetPortStatus : disconnected!.\n",
					__func__);
#ifdef VIRTUAL_DEVICE_ADDRESS
				free_all_mapped_addr(hcd);
#endif
				/*
				 * abort previous transfer and dequeue
				 * all previous transfer request
				 */
				for (counter = F_USB20HDC_EP0;
					counter < F_USB20HDC_HCD_MAX_EP;
					counter++) {
					set_trans_done_inten(base_addr,
								counter, 0);
					set_stop(base_addr, counter);
					set_init(base_addr, counter);
					set_toggle_clear(base_addr, counter);
					dequeue_all_transfer_request(
						&f_usb20hdc->endpoint[counter],
						-ESHUTDOWN);
				}
			}
		}

		if (get_port_enable_c(base_addr)) {
			dev_dbg(pdev, "%s : GetPortStatus : port enable change!.\n",
				__func__);
			status |= USB_PORT_STAT_C_ENABLE << 16;
		}

		if (get_port_ov_curr_c(base_addr)) {
			status |= USB_PORT_STAT_C_OVERCURRENT << 16;
			enable_bus_power_supply(base_addr, 0);
			dev_dbg(pdev, "%s : GetPortStatus : over current change!.\n",
				__func__);
		}

		/* whoever resets must GetPortStatus to complete it!! */
		if (get_port_reset_c(base_addr)) {
			clear_port_reset_c(base_addr);
			set_port_reset_c_inten(base_addr, 1);
			status |= USB_PORT_STAT_C_RESET << 16;
			dev_dbg(pdev, "%s : GetPortStatus : reset change!.\n",
				__func__);
		}

		/*
		 * Even if OWNER is set, there's no harm letting khubd
		 * see the wPortStatus values (they should all be 0 except
		 * for PORT_POWER anyway).
		 */
		if (get_port_connection_rhs(base_addr)) {
			status |= USB_PORT_STAT_CONNECTION;
			status |= get_port_high_speed_rhs(base_addr) ?
					USB_PORT_STAT_HIGH_SPEED :
					get_port_low_speed_rhs(base_addr) ?
					USB_PORT_STAT_LOW_SPEED : 0;
			dev_dbg(pdev, "%s : GetPortStatus : low is %d, high is %d.\n",
				__func__, get_port_low_speed_rhs(base_addr),
				get_port_high_speed_rhs(base_addr));
		}

		if (get_port_enable_rhs(base_addr)) {
			status |= USB_PORT_STAT_ENABLE;
			dev_dbg(pdev, "%s : GetPortStatus : port enable.\n",
				__func__);
#ifdef HUB_RESET_WORKAROUND
			if (f_usb20hdc->reset_start == 1) {
				/*reset has been finish*/
				f_usb20hdc->reset_start = 0;
				f_usb20hdc->reset_failed_cnt = 0;
			}
#endif
		} else {
#ifdef HUB_RESET_WORKAROUND
			if (f_usb20hdc->reset_start == 1)
				/*reset hasn't been finish*/
				f_usb20hdc->reset_failed_cnt++;
#endif
		}

		if (get_port_over_current_rhs(base_addr)) {
			status |= USB_PORT_STAT_OVERCURRENT;
			dev_dbg(pdev, "%s : GetPortStatus : over current.\n",
				__func__);
		}

		if (get_port_reset_rhs(base_addr)) {
			status |= USB_PORT_STAT_RESET;
			dev_dbg(pdev, "%s : GetPortStatus : reset.\n",
				__func__);
		}

		if (get_port_power_rhs(base_addr)) {
			status |= USB_PORT_STAT_POWER;
			dev_dbg(pdev, "%s : GetPortStatus : power.\n",
				__func__);
		}

		if (get_port_suspended_rhs(base_addr)) {
			status |= USB_PORT_STAT_C_SUSPEND << 16;
			dev_dbg(pdev, "%s : GetPortStatus : suspend.\n",
				__func__);
		}

		put_unaligned_le32(status, buf);
		break;
	case SetHubFeature:
		switch (w_value) {
		case C_HUB_LOCAL_POWER:
		case C_HUB_OVER_CURRENT:
			/* no hub-wide feature/status flags */
			break;
		default:
			dev_dbg(pdev, "%s() is ended.\n", __func__);
			spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
			return -EPIPE;
		}
		break;
	case SetPortFeature:
		selector = w_index >> 8;
		w_index &= 0xff;

		if ((!w_index) || (w_index >
				F_USB20HDC_HCD_ROOT_HUB_MAX_PORT)) {
			dev_dbg(pdev, "%s() is ended.\n", __func__);
			spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
			return -EPIPE;
		}
		w_index--;

		switch (w_value) {
		case USB_PORT_FEAT_SUSPEND:
			if ((hcd->driver->bus_resume == NULL) ||
					(hcd->driver->bus_suspend == NULL)) {
				/*
				 * deny usb core request to suspend, HC can't be
				 * waked up without these two function.
				 * note : PM_RUNTIME config may have usb core to
				 * make suspend request when RH have no attached
				 * device or only hub devices are connected.
				*/
				dev_info(pdev, "%s() : SetPort : PORT_FEAT_SUSPEND failed!\n",
					__func__);
				dev_dbg(pdev, "%s() is ended.\n", __func__);
				spin_unlock_irqrestore(&f_usb20hdc->lock,
					flags);
				return -EPIPE;
			} else {
				dev_info(pdev, "%s() : SetPort : PORT_FEAT_SUSPEND\n",
					__func__);
				break;
			}
		case USB_PORT_FEAT_POWER:
			dev_dbg(pdev, "%s() : SetPort : PORT_FEAT_POWER\n",
				__func__);
			enable_bus_power_supply(base_addr, 1);
			break;
		case USB_PORT_FEAT_RESET:
			if (get_port_resuming_rhs(base_addr)) {
				dev_dbg(pdev, "%s() : SetPort with failure : PORT_FEAT_RESET\n",
					__func__);
				dev_dbg(pdev, "%s() is ended.\n", __func__);
				spin_unlock_irqrestore(&f_usb20hdc->lock,
					flags);
				return -EPIPE;
			}
			dev_info(pdev, "%s() : SetPort : PORT_FEAT_RESET\n",
				__func__);
#ifdef VIRTUAL_DEVICE_ADDRESS
			free_all_mapped_addr(hcd);
#endif
#ifdef HUB_RESET_WORKAROUND
			if (f_usb20hdc->reset_failed_cnt < 3) {
				/* normal reset procedure */
				set_port_enable_req(base_addr, 1);
				set_port_reset_req(base_addr);
				f_usb20hdc->reset_start = 1;
			} else {
				/* use power on/off to recovery failed reset
				 * after it's done, hardware can do reset again
				*/
				f_usb20hdc->reset_start = 0;
				f_usb20hdc->reset_failed_cnt = 0;
				dev_info(pdev,
					"%s() : SetPort : PORT_FEAT_RESET error retry is 3.\n",
					__func__);

				/* set detection reset */
				enable_bus_power_supply(base_addr, 0);
				enable_bus_power_supply(base_addr, 1);
			}
#else
			set_port_enable_req(base_addr, 1);
			set_port_reset_req(base_addr);
#endif
			break;
		/* For downstream facing ports (these):  one hub port is put
		 * into test mode according to USB2 11.24.2.13, then the hub
		 * must be reset (which for root hub now means rmmod+modprobe,
		 * or else system reboot).
		 * Valid Selector Codes are showed below:
		 * 1H : Test_J
		 * 2H : Test_K
		 * 3H : Test_SE0_NAK
		 * 4H : Test_Packet
		 * 5H : Test_Force_Enable
		 * See EHCI 2.3.9 and 4.14 for information
		 * about the EHCI-specific stuff.
		 */
		case USB_PORT_FEAT_TEST:
			dev_dbg(pdev, "%s() : SetPort : PORT_FEAT_TEST\n",
				__func__);
			if ((!selector) || (selector > 5)) {
				dev_dbg(pdev, "%s() is ended.\n", __func__);
				spin_unlock_irqrestore(&f_usb20hdc->lock,
					flags);
				return -EPIPE;
			}
			break;
		default:
			dev_info(pdev, "%s() : SetPort : unsupport operation.(w_value:%d)",
				__func__, w_value);
			dev_dbg(pdev, "%s() is ended.\n", __func__);
			spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
			return -EPIPE;
		}
		break;
	default:
		dev_dbg(pdev, "%s() is ended.\n", __func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		return -EPIPE;
	}

	/* release spin lock and enable interrupt, return interrupt status */
	dev_dbg(pdev, "%s() is ended.\n", __func__);
	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	return 0;
}

#if defined(CONFIG_PM)
static int f_usb20hdc_bus_suspend(struct usb_hcd *hcd)
{
	unsigned long		flags;
	unsigned long		counter;
	struct f_usb20hdc_hcd	*f_usb20hdc = hcd_to_f_usb20hdc(hcd);
	struct device *dev = hcd->self.controller;
	void			*base_addr = f_usb20hdc->register_base_address;
	unsigned char		ep_channel;
	unsigned long		suspended = 0;

	dev_info(dev, "%s() is started [enable:%d] [suspend:%d].\n", __func__,
		get_port_enable_rhs(base_addr),
		get_port_suspended_rhs(base_addr));

	if (time_before(jiffies, f_usb20hdc->next_statechange))
		usleep_range(5 * 1000, 10 * 1000);

	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	if (HC_IS_RUNNING(hcd->state)) {
		/* stop transmition */
		dev_info(dev, "%s(): HC_IS_RUNNING.\n", __func__);
		for (counter = F_USB20HDC_EP0;
			counter < F_USB20HDC_HCD_MAX_EP; counter++) {
			ep_channel =
				f_usb20hdc->endpoint[counter].endpoint_channel;
			/* clear status interrupt factor */
			clear_status_clr(base_addr, ep_channel);
			set_trans_done_inten(base_addr, ep_channel, 0);
			set_stop(base_addr, ep_channel);
			set_init(base_addr, ep_channel);
			set_toggle_clear(base_addr, ep_channel);
			dequeue_all_transfer_request(
						&f_usb20hdc->endpoint[counter],
						-ESHUTDOWN);
		}

		hcd->state = HC_STATE_QUIESCING;
	}

	/* suspend request */
	if (get_port_enable_rhs(base_addr)
		&& !get_port_suspended_rhs(base_addr)) {
		set_port_suspend_req(base_addr);
		suspended = 1;
		dev_info(dev, "Assert suspend signal on USB bus.\n");
	}

	/* disable irqs INTEN */
	set_host_inten(base_addr, 0);
	set_dev_inten(base_addr, 0);
	set_phy_err_inten(base_addr, 0);
	set_cmd_inten(base_addr, 0);
	for (counter = F_USB20HDC_DMA_CH1;
			counter < F_USB20HDC_MAX_DMA_CHANNELS; counter++)
		set_dma_inten(base_addr, counter, 0);

	for (counter = F_USB20HDC_EP0;
			counter < F_USB20HDC_HCD_MAX_EP; counter++)
		set_dev_ep_inten(base_addr, counter, 0);

	/* clear status INTEN */
	clear_phy_err_int(base_addr);
	clear_cmd_int(base_addr);

	for (counter = F_USB20HDC_DMA_CH1;
			counter < F_USB20HDC_MAX_DMA_CHANNELS; counter++)
		clear_dma_int(base_addr, counter);

	/* disable irqs HOSTINTEN */
	set_sofstart_inten(base_addr, 0);
	set_frameov_inten(base_addr, 0);
	set_port_connection_c_inten(base_addr, 0);
	set_port_enable_c_inten(base_addr, 0);
	set_port_suspend_c_inten(base_addr, 0);
	set_port_ov_curr_c_inten(base_addr, 0);
	set_port_reset_c_inten(base_addr, 0);
	for (counter = F_USB20HDC_EP0;
			counter < F_USB20HDC_HCD_MAX_EP; counter++)
		set_trans_done_inten(base_addr, counter, 0);

	/* clear status HOSTINTEN */
	clear_port_connection_c(base_addr);
	clear_port_enable_c(base_addr);
	clear_port_suspend_c(base_addr);
	clear_port_ov_curr_c(base_addr);
	clear_port_reset_c(base_addr);
	clear_sofstart(base_addr);
	clear_frameov(base_addr);
	for (counter = F_USB20HDC_EP0;
			counter < F_USB20HDC_HCD_MAX_EP; counter++)
		clear_trans_done(base_addr, counter);

	hcd->state = HC_STATE_SUSPENDED;

	if (hcd->self.root_hub->do_remote_wakeup) {
		dev_info(dev, "Set interrupt factor of receive wakup signal.\n");

		/* enable wakup source */
		set_port_suspend_c_inten(base_addr, 1);
		set_port_connection_c_inten(base_addr, 1);
		set_host_inten(base_addr, 1);
		set_port_wakeup_req(base_addr, 1);
	} else {
		dev_info(dev, "no need for remote_wakeup\n");
#if 0
		/* advanced experiment for powe-off usb device */
		set_port_wakeup_req(base_addr, 0);
		enable_bus_power_supply(base_addr, 0);
#endif
	}

	/* stop F_USB20HDC controller */
	set_hcrun(f_usb20hdc->register_base_address, 0);

	f_usb20hdc->next_statechange = jiffies + msecs_to_jiffies(10);

	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	/* wait 10ms for controller to enter low-power mode */
	if (suspended)
		usleep_range(10 * 1000, 20 * 1000);

	dev_info(dev, "%s() is end. [enable:%d] [suspend:%d]\n", __func__,
		get_port_enable_rhs(base_addr),
		get_port_suspended_rhs(base_addr));
	return 0;
}

static int f_usb20hdc_bus_resume(struct usb_hcd *hcd)
{
	unsigned long		flags;
	unsigned long		counter;
	struct f_usb20hdc_hcd	*f_usb20hdc = hcd_to_f_usb20hdc(hcd);
	struct device *dev = hcd->self.controller;
	void			*base_addr = f_usb20hdc->register_base_address;

	dev_info(dev, "%s() is started.\n", __func__);

#ifdef PHY_DEFECT_WORKAROUND
	/* nasty workaround for slow firmware in hotplug defect */
	enable_bus_power_supply(base_addr, 1);
	msleep(10*1000);
#endif

	if (time_before(jiffies, f_usb20hdc->next_statechange))
		usleep_range(5 * 1000, 10 * 1000);

	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	if (HCD_HW_ACCESSIBLE(hcd) == 0) {
		dev_info(dev, "%s() resume work failed.\n", __func__);

		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

		return -ESHUTDOWN;
	}

	/* disable irqs INTEN */
	set_host_inten(base_addr, 0);
	set_dev_inten(base_addr, 0);
	set_phy_err_inten(base_addr, 0);
	set_cmd_inten(base_addr, 0);
	for (counter = F_USB20HDC_DMA_CH1;
			counter < F_USB20HDC_MAX_DMA_CHANNELS; counter++)
		set_dma_inten(base_addr, counter, 0);

	for (counter = F_USB20HDC_EP0;
			counter < F_USB20HDC_HCD_MAX_EP; counter++)
		set_dev_ep_inten(base_addr, counter, 0);

	/* clear status INTEN */
	clear_phy_err_int(base_addr);
	clear_cmd_int(base_addr);

	for (counter = F_USB20HDC_DMA_CH1;
			counter < F_USB20HDC_MAX_DMA_CHANNELS; counter++)
		clear_dma_int(base_addr, counter);

	/* disable irqs HOSTINTEN */
	set_sofstart_inten(base_addr, 0);
	set_frameov_inten(base_addr, 0);
	set_port_connection_c_inten(base_addr, 0);
	set_port_enable_c_inten(base_addr, 0);
	set_port_suspend_c_inten(base_addr, 0);
	set_port_ov_curr_c_inten(base_addr, 0);
	set_port_reset_c_inten(base_addr, 0);
	for (counter = F_USB20HDC_EP0;
			counter < F_USB20HDC_HCD_MAX_EP; counter++)
		set_trans_done_inten(base_addr, counter, 0);

	/* clear status HOSTINTEN */
	if (!hcd->self.root_hub->do_remote_wakeup) {
		/* clear these bits only when remote wakeup is disabled */
		clear_port_connection_c(base_addr);
		clear_port_suspend_c(base_addr);
	}
	clear_port_enable_c(base_addr);
	clear_port_ov_curr_c(base_addr);
	clear_port_reset_c(base_addr);
	clear_sofstart(base_addr);
	clear_frameov(base_addr);
	for (counter = F_USB20HDC_EP0;
			counter < F_USB20HDC_HCD_MAX_EP; counter++)
		clear_trans_done(base_addr, counter);

	if (get_port_suspended_rhs(base_addr)) {
		if (hcd->self.root_hub->do_remote_wakeup) {
			/* disable wakeup */
			set_port_wakeup_req(base_addr, 0);
		}
		set_port_resume_req(base_addr);

		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

		usleep_range(20 * 1000, 30 * 1000);

		spin_lock_irqsave(&f_usb20hdc->lock, flags);

	}

	/* start F_USB20HDC controller */
	set_dm_pull_down(base_addr, 1);
	set_dp_pull_down(base_addr, 1);
	set_host_en(base_addr, 1);

	/* make the endpoint transfer link list */
	initialize_td_schedule_list(base_addr);
	for (counter = F_USB20HDC_EP0;
		counter < F_USB20HDC_HCD_MAX_EP; counter++) {
		set_nlinkinvalid(base_addr, counter, 0);
	}

	f_usb20hdc->next_statechange = jiffies + msecs_to_jiffies(5);

	hcd->state = HC_STATE_RUNNING;

	/* now safely enable interrupt factor */
	set_host_inten(base_addr, 1);
	set_port_connection_c_inten(base_addr, 1);
	set_port_enable_c_inten(base_addr, 1);
	set_port_suspend_c_inten(base_addr, 1);
	set_port_ov_curr_c_inten(base_addr, 1);
	set_port_reset_c_inten(base_addr, 1);
	set_hcrun(base_addr, 1);

	/*
	 * set wakup_req for hub_control().
	 * if HC is wakeup when only hub device is in connected tree,
	 * then connection change event to roothub is a MUST,
	 * which is the only event can make waked-up root hub to run
	 * configuration job, rather to fall into suspend again.
	*/
#ifdef CONFIG_PM_RUNTIME
	f_usb20hdc->wakeup_req = 1;
	dev_info(dev, "%s() make a wakeup_req to hub_control().\n", __func__);
#endif

	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dev_info(dev, "%s() is ended.\n", __func__);
	return 0;
}
#endif

static int reset_connection_open(struct inode *inode, struct file *file)
{

	struct f_usb20hdc_hcd *f_usb20hdc = inode->i_private;

	dev_info(f_usb20hdc->dev, "%s()\n", __func__);

	file->private_data = f_usb20hdc;

	return 0;
}

static ssize_t  reset_connection_write(struct file *file,
	const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[2];
	ssize_t ret = 0;
	struct f_usb20hdc_hcd *f_hcd;
	struct device *dev;

	f_hcd = file->private_data;
	dev = f_hcd->dev;
	dev_info(f_hcd->dev, "%s()\n", __func__);

	if(count > sizeof(buf)) 
		return -EINVAL; 

	ret = simple_write_to_buffer(buf, sizeof(buf), ppos, userbuf, count);

	switch (buf[0]) {
	default: /* reset of device detection */
		set_port_enable_req(f_hcd->register_base_address, 0);
		while (get_port_enable_rhs(f_hcd->register_base_address))
			;
		free_all_mapped_addr(f_hcd->f_otg->hcd);
		enable_bus_power_supply(f_hcd->register_base_address, 0);
		enable_bus_power_supply(f_hcd->register_base_address, 1);
		break;
	}

	return ret;
}

static const struct file_operations reset_connection_fops = {
	.open			= reset_connection_open,
	.write			= reset_connection_write,
};

int f_usb20hdc_hcd_probe(struct f_usb20hdc_otg *f_otg)
{
	static const struct hc_driver f_usb20hdc_hc_driver = {
		.description		= "f_usb20hdc_hcd",
		.product_desc		= "F_USB20HDC HCD",
		.hcd_priv_size		= sizeof(struct f_usb20hdc_hcd),
		.irq			= f_usb20hdc_hcd_irq,
		.flags			= HCD_MEMORY | HCD_USB2,
		.reset			= f_usb20hdc_hcd_reset,
		.start			= f_usb20hdc_hcd_start,
		.stop			= f_usb20hdc_hcd_stop,
		.shutdown		= NULL,
		.get_frame_number	= f_usb20hdc_hcd_get_frame_number,
		.urb_enqueue		= f_usb20hdc_hcd_urb_enqueue,
		.urb_dequeue		= f_usb20hdc_hcd_urb_dequeue,
		.endpoint_disable	= NULL,
		.endpoint_reset		= NULL,
		.hub_status_data	= f_usb20hdc_hcd_hub_status_data,
		.hub_control		= f_usb20hdc_hcd_hub_control,
#if defined(CONFIG_PM)
		.bus_suspend		= f_usb20hdc_bus_suspend,
		.bus_resume		= f_usb20hdc_bus_resume,
#endif
		.clear_tt_buffer_complete	= NULL,
	};
	u32 counter;
	struct f_usb20hdc_hcd *f_usb20hdc;
	struct f_usb20hdc_hcd_ep *endpoint;
	void __iomem *register_base_address;
	struct usb_hcd *hcd = NULL;
	int result;
	struct platform_device *pdev = f_otg->pdev;
	struct device *dev = f_otg->dev;

	/* check argument */
	if (unlikely(!pdev))
		return -EINVAL;

	dev_dbg(dev, "%s() is started.\n", __func__);

	/* create hcd and save it in f_otg*/
	hcd = usb_create_hcd(&f_usb20hdc_hc_driver, &pdev->dev,
							"f_usb20hdc_hcd");
	if (!hcd) {
		dev_err(dev, "%s():usb_create_hcd() failed.\n",
								__func__);
		result = -ENOMEM;
		goto done;
	}

	/* setup the F_USB20HDC HCD device driver structure parameter */
	f_usb20hdc = hcd_to_f_usb20hdc(hcd);
	f_usb20hdc->dev = &pdev->dev;
	f_usb20hdc->f_otg = f_otg;

#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	for (counter = F_USB20HDC_DMA_CH1;
		counter < F_USB20HDC_MAX_DMA_CHANNELS; counter++) {
		f_usb20hdc->dma_data[counter] = f_otg->dma_data[counter];
	}
#endif

	/* Create the debugfs */
	f_usb20hdc->file = debugfs_create_file("reset_connection",
		S_IWUGO, f_otg->root, f_usb20hdc, &reset_connection_fops);
	if (!f_usb20hdc->file) {
		dev_err(&pdev->dev, "debugfs_create_file fail\n");
		return -ENOMEM;
	}

	/* get a register base address for a F_USB20HDC device */
	register_base_address = remap_iomem_region(f_otg->mem_start,
				f_otg->mem_size);
	if (!register_base_address) {
		dev_err(dev, "%s():remap_iomem_region() failed.\n",
								__func__);
		result = -ENODEV;
		goto err_res;
	}

	/* setup the hcd structure parameter */
	hcd->rsrc_start = f_otg->mem_start;
	hcd->rsrc_len = f_otg->mem_size;
	hcd->regs = register_base_address;

	/* initialize a F_USB20HDC HCD device driver structure */
	spin_lock_init(&f_usb20hdc->lock);
	f_usb20hdc->resource = f_otg->mem_res;
	f_usb20hdc->register_base_address = register_base_address;
	f_usb20hdc->irq = f_otg->irq;
	f_usb20hdc->eps_inuse_map = 0x0007;
	f_usb20hdc->bulkin_remain = 0;
	f_usb20hdc->wakeup_from_poweroff = 0;
#ifdef CONFIG_PM_RUNTIME
	f_usb20hdc->wakeup_req  = 0;
#endif
#ifdef HUB_RESET_WORKAROUND
	f_usb20hdc->reset_start = 0;
	f_usb20hdc->reset_failed_cnt = 0;
#endif
#ifdef VIRTUAL_DEVICE_ADDRESS
	memset((void *)f_usb20hdc->dev_addr_table, 0,
		sizeof(f_usb20hdc->dev_addr_table[MAX_DEV_ADDR]));
#endif
	initialize_endpoint_configure(f_usb20hdc);
	for (counter = F_USB20HDC_EP0;
				counter < F_USB20HDC_HCD_MAX_EP; counter++) {
		endpoint = &f_usb20hdc->endpoint[counter];
		endpoint->f_usb20hdc = f_usb20hdc;
		endpoint->request = NULL;
		INIT_LIST_HEAD(&endpoint->queue);
		endpoint->transfer_packets = 0;
		endpoint->transfer_type = 0;
		endpoint->transfer_status = 0;
	}

	/* check endpoint buffer size */
	if (!is_endpoint_buffer_usable()) {
		dev_err(dev, "%s():F_USB20HDC endpoint RAM buffer is insufficient.\n",
			__func__);
		result = -ENODEV;
		goto err_irq;
	}

#if (F_USB20HDC_HCD_USE_HIGH_SPEED_MODE == 1)
	f_usb20hdc->highspeed_support = 1;
#else
	f_usb20hdc->highspeed_support = 0;
#endif
	f_usb20hdc->ctrl_stage = CONTROL_TRANSFER_STAGE_SETUP;

#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	result = f_usb20hdc_dma_attach(f_usb20hdc->dma_data,
		F_USB20HDC_MAX_DMA_CHANNELS,
		F_USB20HDC_HCD_MAX_DMA_TRANSFER_SIZE);
	if (result < 0) {
		dev_err(dev, "failed to attach DMA.\n");
		goto err_dma;
	}
#endif

	/* initialize F_USB20HDC controller */
	initialize_controller(f_usb20hdc);

	/* add hcd device */
	result = usb_add_hcd(hcd, f_usb20hdc->irq, get_irq_flag());
	if (result) {
		dev_err(dev, "%s():usb_add_hcd() failed at %d.\n",
							__func__, result);
		goto err_data;
	}
	f_otg->hcd = hcd;

	/* driver registering log output */
	dev_info(dev, "F_USB20HDC HCD driver (version %s) is registered.\n",
		F_USB20HDC_HCD_CONFIG_DRIVER_VERSION);

	dev_dbg(dev, "%s() is ended [%p].\n", __func__, f_otg);

	return 0;

err_data:
	platform_set_drvdata(pdev, NULL);
	f_otg->hcd = NULL;
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
err_dma:
	f_usb20hdc_dma_detach(f_usb20hdc->dma_data,
		F_USB20HDC_MAX_DMA_CHANNELS,
		F_USB20HDC_HCD_MAX_DMA_TRANSFER_SIZE);
#endif
err_irq:
	unmap_iomem_region(register_base_address);
err_res:
	usb_put_hcd(hcd);
done:
	dev_dbg(dev, "%s() is ended.\n", __func__);

	return result;
}

int f_usb20hdc_hcd_remove(struct f_usb20hdc_otg *f_otg)
{
	struct usb_hcd *hcd = f_otg->hcd;
	struct f_usb20hdc_hcd *f_usb20hdc = hcd_to_f_usb20hdc(hcd);
	struct platform_device *pdev = f_otg->pdev;

	/* check argument */
	if (unlikely(!pdev))
		return -EINVAL;

	dev_dbg(&pdev->dev, "%s() is started.\n", __func__);

	debugfs_remove(f_usb20hdc->file);

	/* get hcd data */
	if (!hcd) {
		dev_err(&pdev->dev, "%s(): hcd is NULL.\n", __func__);
		dev_dbg(&pdev->dev, "%s(): is ended.\n", __func__);
		return -EINVAL;
	}

	/* remove hcd device */
	usb_remove_hcd(hcd);

	/* disable F_USB20HDC controller interrupt */
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	f_usb20hdc_dma_detach(f_usb20hdc->dma_data,
		F_USB20HDC_MAX_DMA_CHANNELS,
		F_USB20HDC_HCD_MAX_DMA_TRANSFER_SIZE);
#endif

	/* release device resource */
	platform_set_drvdata(pdev, NULL);
	f_otg->hcd = NULL;
	unmap_iomem_region(f_usb20hdc->register_base_address);

	/* free F_USB20HDC HCD device driver structure data memory */
	usb_put_hcd(hcd);

	/* driver deregistering log output */
	dev_info(&pdev->dev, "F_USB20HDC HCD driver is deregistered.\n");

	dev_dbg(&pdev->dev, "%s() is ended.\n", __func__);

	return 0;
}

#ifdef CONFIG_PM
int f_usb20hdc_hcd_suspend(struct f_usb20hdc_otg *f_otg)
{
	struct usb_hcd		*hcd = f_otg->hcd;
	struct f_usb20hdc_hcd	*f_usb20hdc;
	unsigned long flags = 0;
	struct device *dev = f_otg->dev;

	dev_info(dev, "%s() is started.\n", __func__);

	/* check parameter */
	if (!hcd) {
		dev_err(dev, "%s():no hcd instance.\n", __func__);
		dev_info(dev, "%s() is ended.\n", __func__);
		return -EINVAL;
	} else
		f_usb20hdc = hcd_to_f_usb20hdc(hcd);

	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* disable the hcd interrupt factor and set hcd flag*/
	set_host_inten(f_usb20hdc->register_base_address, 0);
	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

#if 0 /* not supported now */
	/* disable clock and enable remote wakeup irq */
	if (hcd->self.root_hub->do_remote_wakeup) {
		dev_info(dev, "%s(): enable_irq_wake().\n", __func__);
		enable_irq_wake(f_usb20hdc->irq);
	}
#endif

#if 0 /* need more investigation in resume from off-mode */
	set_port_power_ctl_req(f_usb20hdc->register_base_address, 0);
#endif
	dev_info(dev, "%s() is ended [%p].\n", __func__, f_otg);
	return 0;
}

int f_usb20hdc_hcd_resume(struct f_usb20hdc_otg *f_otg)
{
	struct usb_hcd		*hcd = f_otg->hcd;
	struct f_usb20hdc_hcd	*f_usb20hdc;
	unsigned long flags = 0;
#if 0
	int counter;
	struct f_usb20hdc_hcd_ep *endpoint;
#endif
	struct device *dev = f_otg->dev;

	dev_info(dev, "%s() is started.\n", __func__);

	/* check parameter */
	if (!hcd) {
		dev_err(dev, "%s():no hcd instance.\n", __func__);
		dev_info(dev, "%s() is ended.\n", __func__);
		return -EINVAL;
	} else
		f_usb20hdc = hcd_to_f_usb20hdc(hcd);

	/* supply power early for slow firmware be ready quickly */
	enable_bus_power_supply(f_otg->reg_base, 1);

#if 0 /*not supported now*/
	/* disable remote wakeup irq */
	if (hcd->self.root_hub->do_remote_wakeup) {
		dev_info(dev, "%s(): disable_irq_wake().\n", __func__);
		disable_irq_wake(f_usb20hdc->irq);
	}
#endif

	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* enable the hcd interrupt factor and set hcd flag*/
	set_host_inten(f_usb20hdc->register_base_address, 1);
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

#ifdef COLD_RESUME_SUPPORT
	f_usb20hdc->wakeup_from_poweroff = 1;
#if 0
	/* recovery from power off or hibernation */
	initialize_endpoint_configure(f_usb20hdc);
	for (counter = F_USB20HDC_EP0;
				counter < F_USB20HDC_HCD_MAX_EP; counter++) {
		endpoint = &f_usb20hdc->endpoint[counter];
		endpoint->f_usb20hdc = f_usb20hdc;
		endpoint->request = NULL;
		INIT_LIST_HEAD(&endpoint->queue);
		endpoint->transfer_packets = 0;
		endpoint->transfer_type = 0;
		endpoint->transfer_status = 0;
	}
	/* initialize F_USB20HDC controller */
	initialize_controller(f_usb20hdc);

#endif
	f_usb20hdc_hcd_reset(hcd);
	f_usb20hdc_hcd_start(hcd);
	f_usb20hdc->wakeup_from_poweroff = 0;
#endif

	dev_info(dev, "%s() is ended [%p].\n", __func__, f_otg);
	return 0;
}

int f_usb20hdc_hcd_restore(struct f_usb20hdc_otg *f_otg)
{
	struct usb_hcd		*hcd = f_otg->hcd;
	struct device *dev = hcd->self.controller;

	dev_info(dev, "%s() is started.\n", __func__);
	usb_root_hub_lost_power(hcd->self.root_hub);
	dev_info(dev, "%s() is ended.\n", __func__);
	return 0;
}
#endif /* CONFIG_PM */
