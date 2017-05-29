/*
 * linux/drivers/usb/gadget/f_usb30_udc.c - F_USB30 USB3.0 Function
 * Controller Driver
 *
 * based on F_USB20LP USB2.0 Function Controller Driver
 *
 * Copyright (C) 2002 Intrinsyc, Inc. (Frank Becker)
 * Copyright (C) 2003 Robert Schwebel, Pengutronix
 * Copyright (C) 2003 Benedikt Spranger, Pengutronix
 * Copyright (C) 2003 David Brownell
 * Copyright (C) 2003 Joshua Wise
 * Copyright (C) 2006 - 2007 Lineo Solutions, Inc.
 * Copyright (C) FUJITSU ELECTRONICS INC. 2011. All rights reserved.
 * Copyright (C) 2011 - 2012 FUJITSU SEMICONDUCTOR LIMITED
 */

/* #define DEBUG */
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include <linux/dma-mapping.h>

#include <linux/irqchip/irq-mb8ac0300.h>

#include "f_usb30_udc.h"

/* F_USB30 UDC device driver request structure */
struct f_usb30_request {
	struct usb_request req;			/* USB request structure */
	struct list_head queue;			/* request queue head */
	unsigned char dma_buffer_map;		/* DMA trans buffer map flag */
};

/* F_USB30 UDC device driver endpoint structure */
struct f_usb30_ep {
	struct usb_ep ep;			/* endpoint structure */
	struct f_usb30_udc *udc;		/*
						 * F_USB30 UDC device
						 * driver structure
						 */
	unsigned char attributes;		/* endpoint attributes */
	signed char interface_channel;		/* interface channle */
	unsigned char alternate_channels;	/* alternate channle count */
	signed char dma_channel;		/* DMA channel */
	struct f_usb30_request *req;		/* current request structure */
	struct list_head queue;			/* endpoint queue head */
	unsigned char enabled;			/* endpoint enabled flag */
	unsigned char halt;			/* transfer halt flag */
	unsigned char force_halt;		/* transfer force halt flag */
	unsigned char dma_transfer;		/* DMA transfer flag */
#if defined(CONFIG_USB_GADGET_F_USB30_USED_DMA_TRANSFER)
	void __iomem *ss_dma_fifo_addr;		/*
						 * SS DMA transfer endpoint's
						 * FIFO pyhsical address
						 */
	void __iomem *hs_dma_fifo_addr;		/*
						 * HS/FS DMA transfer endpoint's
						 * FIFO pyhsical address
						 */
#endif
};

/* F_USB30 UDC device driver structure */
struct f_usb30_udc {
	struct clk *clk;			/* clock */
	struct usb_gadget gadget;		/* gadget structure */
	struct usb_gadget_driver *gadget_driver;/* gadget driver structure */
	spinlock_t lock;			/* device lock */
	struct resource *device_resource;	/* F_USB30 device resource */
	void __iomem *register_base_address;	/* F_USB30 reg base addres */
	int hs_irq;				/* F_USB30 HF IRQ number */
	int ss_irq;				/* F_USB30 SS IRQ number */
#if defined(CONFIG_ARCH_MB8AC0300)
	int vbus_on_irq;			/* vbus on detect extint num */
	int vbus_off_irq;			/* vbus off detect extint num */
	int vbus_on_hwirq, vbus_off_hwirq;
	unsigned char vbus_active_level;	/* vbus detect active level */
#else
/* for other soc */
#endif
#if defined(CONFIG_USB_GADGET_F_USB30_USED_DMA_TRANSFER)
	int dmac_in_irq;			/* DMAC in IRQ number */
	int dmac_out_irq;			/* DMAC out IRQ number */
	unsigned char dma_ep[F_USB30_MAX_DMAC];
						/* DMA endpoint channel array */
#if defined(CONFIG_USB_GADGET_F_USB30_DMA_USE_BOUNCE_BUF)
	void *buffer[F_USB30_MAX_DMAC];		/*
						 * DMA transfer noncachable
						 * buffer's virtual address
						 */
	dma_addr_t dma_buffer[F_USB30_MAX_DMAC];/*
						 * DMA transfer noncachable
						 * buffer's physical address
						 */
#endif
#endif
	struct f_usb30_ep udc_endpoint[F_USB30_MAX_EP];
						/*
						 * F_USB30 UDC device driver
						 * endpoint structure array
						 */
	unsigned char vbus;			/* bus connect status flag */
	enum usb_device_state device_state;	/* USB device state */
	enum usb_device_state device_state_last;/* last USB device state */
	unsigned char configure_value_last;	/* last configure value */
	enum f_usb30_ss_link_state link_state;/* ss link training state */
	int ss_discnt;				/* link training fail counter */
	enum f_usb30_ctrl_stage ctrl_stage;	/* control transfer stage */
	unsigned char ctrl_pri_dir;		/*
						 * control transfer
						 * priority-processing
						 * direction flag
						 */
};

static struct f_usb30_udc *f_usb30_data;

static void initialize_dma_controller(struct f_usb30_udc *f_usb30)
{
#if defined(CONFIG_USB_GADGET_F_USB30_USED_DMA_TRANSFER)
	unsigned long counter;
	void *base_addr = f_usb30->register_base_address;

	/* initialize F_USB30 DMA controller register */
	for (counter = 0; counter < F_USB30_MAX_DMAC; counter++) {
		set_dmainit(base_addr, counter, 1);
		set_mskintuexdreq(base_addr, counter, 1);
		set_mskintstarterr(base_addr, counter, 1);
		set_mskintslverr(base_addr, counter, 1);
		set_mskintdecerr(base_addr, counter, 1);
		set_mskintaxifin(base_addr, counter, 1);
		set_mskintusbunder(base_addr, counter, 1);
		set_mskintusbover(base_addr, counter, 1);
		set_mskintexokay(base_addr, counter, 1);
		set_mskintnull(base_addr, counter, 1);
		set_mskintusbfin(base_addr, counter, 1);
		set_mskintremainfin(base_addr, counter, 1);
		clear_intuexdreq(base_addr, counter);
		clear_intstarterr(base_addr, counter);
		clear_intslverr(base_addr, counter);
		clear_intdecerr(base_addr, counter);
		clear_intaxifin(base_addr, counter);
		clear_intusbunder(base_addr, counter);
		clear_intusbover(base_addr, counter);
		clear_intexokay(base_addr, counter);
		clear_intnull(base_addr, counter);
		clear_intusbfin(base_addr, counter);
		clear_intremainfin(base_addr, counter);
	}
#endif
	return;
}

#if defined(CONFIG_USB_GADGET_F_USB30_USED_DMA_TRANSFER)
static void set_dma_controller(struct f_usb30_ep *endpoint,
	 unsigned long source, unsigned long destination, unsigned long bytes)
{
	struct f_usb30_udc *f_usb30 = endpoint->udc;
	unsigned char ep_channel = endpoint->ep.address &
						 USB_ENDPOINT_NUMBER_MASK;
	void *base_addr = f_usb30->register_base_address;

	/* check DMA channel */
	if (unlikely(endpoint->dma_channel == -1))
		return;

	/* check DMA transfer and DMA abort active */
	if (unlikely(get_dmaactive(base_addr, endpoint->dma_channel) ||
			 get_abortactive(base_addr, endpoint->dma_channel)))
		return;

	/* setup F_USB30 DMA controller register */
	f_usb30->dma_ep[endpoint->dma_channel] = ep_channel;
	set_issue(base_addr, endpoint->dma_channel, 1);
	set_bursttype(base_addr, endpoint->dma_channel, DMAC_BURST_INCREMENT);
	endpoint->ep.address & USB_DIR_IN ?
		 set_maxlen(base_addr, endpoint->dma_channel, source & 0x7 ?
			 DMAC_MAX_BURST_8 : DMAC_MAX_BURST_16) :
		 set_maxlen(base_addr, endpoint->dma_channel, destination & 0x7
			 ? DMAC_MAX_BURST_8 : DMAC_MAX_BURST_16);
	set_nullcntl(base_addr, endpoint->dma_channel, 0);
	set_abortcntl(base_addr, endpoint->dma_channel, 1);
	set_axiaddress(base_addr, endpoint->dma_channel, endpoint->ep.address &
			 USB_DIR_IN ? source : destination);
	set_dmadatasize(base_addr, endpoint->dma_channel, bytes);
	if (!(endpoint->ep.address & USB_DIR_IN))
		f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 set_ss_tcnt(base_addr, ep_channel, bytes) :
			 set_hs_tcnt(base_addr, ep_channel, bytes);

	return;
}
#endif

static void enable_dma_transfer(struct f_usb30_ep *endpoint,
				 unsigned char enable)
{
#if defined(CONFIG_USB_GADGET_F_USB30_USED_DMA_TRANSFER)
	struct f_usb30_udc *f_usb30 = endpoint->udc;
	unsigned char ep_channel = endpoint->ep.address &
						 USB_ENDPOINT_NUMBER_MASK;
	void *base_addr = f_usb30->register_base_address;

	/* check DMA channel */
	if (unlikely(endpoint->dma_channel == -1))
		return;

	if (enable) {
		/* check DMA transfer and DMA abort active */
		if (unlikely(get_dmaactive(base_addr, endpoint->dma_channel) ||
			 get_abortactive(base_addr, endpoint->dma_channel)))
			return;

		/* enable DMA transfer */
		endpoint->dma_transfer = 1;
		if (endpoint->ep.address & USB_DIR_IN) {
			clear_intuexdreq(base_addr, endpoint->dma_channel);
			clear_intstarterr(base_addr, endpoint->dma_channel);
			clear_intslverr(base_addr, endpoint->dma_channel);
			clear_intdecerr(base_addr, endpoint->dma_channel);
			clear_intexokay(base_addr, endpoint->dma_channel);
			clear_intusbfin(base_addr, endpoint->dma_channel);
			clear_intaxifin(base_addr, endpoint->dma_channel);
			clear_intnull(base_addr, endpoint->dma_channel);
			clear_intremainfin(base_addr, endpoint->dma_channel);
			set_mskintuexdreq(base_addr, endpoint->dma_channel, 0);
			set_mskintstarterr(base_addr, endpoint->dma_channel, 0);
			set_mskintslverr(base_addr, endpoint->dma_channel, 0);
			set_mskintdecerr(base_addr, endpoint->dma_channel, 0);
			set_mskintexokay(base_addr, endpoint->dma_channel, 0);
			set_mskintusbfin(base_addr, endpoint->dma_channel, 0);
			set_mskintremainfin(base_addr, endpoint->dma_channel,
						0);
			set_mskintnull(base_addr, endpoint->dma_channel, 0);
			set_mskintaxifin(base_addr, endpoint->dma_channel, 0);
		} else {
			clear_intuexdreq(base_addr, endpoint->dma_channel);
			clear_intstarterr(base_addr, endpoint->dma_channel);
			clear_intslverr(base_addr, endpoint->dma_channel);
			clear_intdecerr(base_addr, endpoint->dma_channel);
			clear_intaxifin(base_addr, endpoint->dma_channel);
			clear_intusbfin(base_addr, endpoint->dma_channel);
			clear_intusbunder(base_addr, endpoint->dma_channel);
			clear_intusbover(base_addr, endpoint->dma_channel);
			clear_intexokay(base_addr, endpoint->dma_channel);
			clear_intnull(base_addr, endpoint->dma_channel);
			clear_intremainfin(base_addr, endpoint->dma_channel);
			set_mskintuexdreq(base_addr, endpoint->dma_channel, 0);
			set_mskintstarterr(base_addr, endpoint->dma_channel, 0);
			set_mskintslverr(base_addr, endpoint->dma_channel, 0);
			set_mskintdecerr(base_addr, endpoint->dma_channel, 0);
			set_mskintaxifin(base_addr, endpoint->dma_channel, 0);
			set_mskintusbfin(base_addr, endpoint->dma_channel, 0);
			set_mskintusbunder(base_addr, endpoint->dma_channel, 0);
			set_mskintusbover(base_addr, endpoint->dma_channel, 0);
			set_mskintexokay(base_addr, endpoint->dma_channel, 0);
			set_mskintnull(base_addr, endpoint->dma_channel, 0);
			set_mskintremainfin(base_addr, endpoint->dma_channel,
						0);
			f_usb30->gadget.speed == USB_SPEED_SUPER ?
				 set_ss_mskdend(base_addr, ep_channel, 0) :
				 set_hs_mskdend(base_addr, ep_channel, 0);
			if (endpoint->attributes != USB_ENDPOINT_XFER_ISOC) {
				/* enable SPDD mode */
				if (f_usb30->gadget.speed ==
							 USB_SPEED_SUPER) {
					clear_ss_intspd(base_addr, ep_channel);
					set_ss_mskspd(base_addr, ep_channel, 0);
					set_ss_enspd(base_addr, ep_channel, 0);
				} else {
					clear_hs_intspdd(base_addr, ep_channel);
					set_hs_mskspdd(base_addr, ep_channel,
							1);
					set_hs_enspdd(base_addr, ep_channel, 0);
				}
			}
		}
		f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 set_ss_dmamode(base_addr, ep_channel, 1) :
			 set_hs_dmamode(base_addr, ep_channel, 1);
		set_dmastart(base_addr, endpoint->dma_channel, 1);
		f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 set_ss_mskdmareq(base_addr, ep_channel, 0) :
			 set_hs_mskdmareq(base_addr, ep_channel, 0);
	} else {
		f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 set_ss_mskdmareq(base_addr, ep_channel, 1) :
			 set_hs_mskdmareq(base_addr, ep_channel, 1);
		set_mskintuexdreq(base_addr, endpoint->dma_channel, 1);
		set_mskintstarterr(base_addr, endpoint->dma_channel, 1);
		set_mskintslverr(base_addr, endpoint->dma_channel, 1);
		set_mskintdecerr(base_addr, endpoint->dma_channel, 1);
		set_mskintaxifin(base_addr, endpoint->dma_channel, 1);
		set_mskintusbunder(base_addr, endpoint->dma_channel, 1);
		set_mskintusbover(base_addr, endpoint->dma_channel, 1);
		set_mskintexokay(base_addr, endpoint->dma_channel, 1);
		set_mskintnull(base_addr, endpoint->dma_channel, 1);
		set_mskintusbfin(base_addr, endpoint->dma_channel, 1);
		set_mskintremainfin(base_addr, endpoint->dma_channel, 1);
		clear_intuexdreq(base_addr, endpoint->dma_channel);
		clear_intstarterr(base_addr, endpoint->dma_channel);
		clear_intslverr(base_addr, endpoint->dma_channel);
		clear_intdecerr(base_addr, endpoint->dma_channel);
		clear_intaxifin(base_addr, endpoint->dma_channel);
		clear_intusbunder(base_addr, endpoint->dma_channel);
		clear_intusbover(base_addr, endpoint->dma_channel);
		clear_intexokay(base_addr, endpoint->dma_channel);
		clear_intnull(base_addr, endpoint->dma_channel);
		clear_intusbfin(base_addr, endpoint->dma_channel);
		clear_intremainfin(base_addr, endpoint->dma_channel);
		if (!(endpoint->ep.address & USB_DIR_IN)) {
			f_usb30->gadget.speed == USB_SPEED_SUPER ?
				 set_ss_mskdend(base_addr, ep_channel, 1) :
				 set_hs_mskdend(base_addr, ep_channel, 1);
			f_usb30->gadget.speed == USB_SPEED_SUPER ?
				 clear_ss_intdend(base_addr, ep_channel) :
				 clear_hs_intdend(base_addr, ep_channel);
			if (endpoint->attributes != USB_ENDPOINT_XFER_ISOC) {
				/* disable SPDD mode */
				if (f_usb30->gadget.speed ==
						 USB_SPEED_SUPER) {
					set_ss_mskspd(base_addr, ep_channel, 1);
					clear_ss_intspd(base_addr, ep_channel);
					set_ss_enspd(base_addr, ep_channel, 0);

				} else {
					set_hs_mskspdd(base_addr, ep_channel,
							1);
					clear_hs_intspdd(base_addr, ep_channel);
					set_hs_enspdd(base_addr, ep_channel, 0);
				}
			}
		}
		f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 set_ss_dmamode(base_addr, ep_channel, 0) :
			 set_hs_dmamode(base_addr, ep_channel, 0);
		endpoint->dma_transfer = 0;
	}
#endif
	return;
}

static unsigned char set_in_transfer_dma(struct f_usb30_ep *endpoint)
{
#if defined(CONFIG_USB_GADGET_F_USB30_USED_DMA_TRANSFER)
	struct f_usb30_udc *f_usb30 = endpoint->udc;
	struct f_usb30_request *request = endpoint->req;
	unsigned char ep_channel = endpoint->ep.address &
						 USB_ENDPOINT_NUMBER_MASK;
	dma_addr_t dma_addr;
	unsigned long bytes;

	/* check argument */
	if (unlikely(!endpoint->enabled))
		return 0;

	/* calculate this time transfer byte */
	bytes = request->req.length < F_USB30_DMAC_TRANS_MAX_BYTES ?
			 request->req.length : F_USB30_DMAC_TRANS_MAX_BYTES;

#if defined(CONFIG_USB_GADGET_F_USB30_DMA_USE_BOUNCE_BUF)
	/* copy IN transfer data to noncachable buffer */
	memcpy(f_usb30->buffer[endpoint->dma_channel], request->req.buf, bytes);
#else
	/* check DMA transfer buffer mapping */
	if (request->req.dma == F_USB30_DMA_ADDR_INVALID) {
		/* map DMA transfer buffer and sync DMA transfer buffer */
		request->req.dma = dma_map_single(f_usb30->gadget.dev.parent,
						 request->req.buf,
						 request->req.length,
						 DMA_TO_DEVICE);
		request->dma_buffer_map = 1;
	} else {
		dma_sync_single_for_device(f_usb30->gadget.dev.parent,
						 request->req.dma,
						 request->req.length,
						 DMA_TO_DEVICE);
		request->dma_buffer_map = 0;
	}
#endif
	dev_dbg(f_usb30->gadget.dev.parent,
		"endpoint %u DMA is setup at length = %u, actual = %u, max packet = %u, this time = %u.\n",
		ep_channel, request->req.length, request->req.actual,
		endpoint->ep.maxpacket, (unsigned int) bytes);

	/* update actual byte */
	request->req.actual = bytes;

	/* set StreamID if stream transfer enabled */
	if ((f_usb30->gadget.speed == USB_SPEED_SUPER) &&
		 endpoint->ep.comp_desc->bmAttributes &&
		 !get_ss_streamactive(f_usb30->register_base_address,
					 ep_channel))
		set_ss_streamid(f_usb30->register_base_address,
				 ep_channel, request->req.stream_id);

	/* set dma transfer source address */
#if defined(CONFIG_USB_GADGET_F_USB30_DMA_USE_BOUNCE_BUF)
	dma_addr = f_usb30->dma_buffer[endpoint->dma_channel];
#else
	dma_addr = request->req.dma;
#endif

	/* setup DMA transfer */
	set_dma_controller(endpoint, (unsigned long)dma_addr,
			 f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 (unsigned long) endpoint->ss_dma_fifo_addr :
			 (unsigned long) endpoint->hs_dma_fifo_addr, bytes);

	/* enable DMA transfer */
	enable_dma_transfer(endpoint, 1);

	return 1;
#else
	return 0;
#endif
}

static unsigned char set_out_transfer_dma(struct f_usb30_ep *endpoint)
{
#if defined(CONFIG_USB_GADGET_F_USB30_USED_DMA_TRANSFER)
	struct f_usb30_udc *f_usb30 = endpoint->udc;
	struct f_usb30_request *request = endpoint->req;
	unsigned char ep_channel = endpoint->ep.address &
						 USB_ENDPOINT_NUMBER_MASK;
	dma_addr_t dma_addr;
	unsigned long bytes;

	/* check argument */
	if (unlikely(!endpoint->enabled))
		return 0;

#if !defined(CONFIG_USB_GADGET_F_USB30_DMA_USE_BOUNCE_BUF)
	/* check DMA transfer buffer mapping */
	if (request->req.dma == F_USB30_DMA_ADDR_INVALID) {
		/* map DMA transfer buffer and sync DMA transfer buffer */
		request->req.dma = dma_map_single(f_usb30->gadget.dev.parent,
						 request->req.buf,
						 request->req.length,
						 DMA_FROM_DEVICE);
		request->dma_buffer_map = 1;
	} else {
		dma_sync_single_for_device(f_usb30->gadget.dev.parent,
						 request->req.dma,
						 request->req.length,
						 DMA_FROM_DEVICE);
		request->dma_buffer_map = 0;
	}
#endif
	/* calculate this time transfer byte */
	bytes = request->req.length < F_USB30_DMAC_TRANS_MAX_BYTES ?
		 request->req.length : F_USB30_DMAC_TRANS_MAX_BYTES;

	dev_dbg(f_usb30->gadget.dev.parent,
		"endpoint %u DMA is setup at length = %u, actual = %u, max packet = %u, this time = %u.\n",
		ep_channel, request->req.length, request->req.actual,
		endpoint->ep.maxpacket, (unsigned int) bytes);

	/* set total data size and StreamID if stream transfer enabled */
	if ((f_usb30->gadget.speed == USB_SPEED_SUPER) &&
		 endpoint->ep.comp_desc->bmAttributes &&
		 !get_ss_streamactive(f_usb30->register_base_address,
					 ep_channel)) {
		set_ss_stcnt(f_usb30->register_base_address,
				 ep_channel, bytes);
		set_ss_streamid(f_usb30->register_base_address,
				 ep_channel, request->req.stream_id);
	}

#if defined(CONFIG_USB_GADGET_F_USB30_DMA_USE_BOUNCE_BUF)
	dma_addr = f_usb30->dma_buffer[endpoint->dma_channel];
#else
	dma_addr = request->req.dma;
#endif

	/* setup DMA transfer */
	set_dma_controller(endpoint, f_usb30->gadget.speed ==
			 USB_SPEED_SUPER ?
			 (unsigned long) endpoint->ss_dma_fifo_addr :
			 (unsigned long) endpoint->hs_dma_fifo_addr,
			 (unsigned long) dma_addr, bytes);

	/* enable DMA transfer */
	enable_dma_transfer(endpoint, 1);

	return 1;
#else
	return 0;
#endif
}

static void initialize_endpoint_hw(struct f_usb30_ep *,
	 unsigned char, unsigned char);
static void abort_in_transfer_dma(struct f_usb30_ep *endpoint,
	 unsigned char initialize)
{
#if defined(CONFIG_USB_GADGET_F_USB30_USED_DMA_TRANSFER)
	struct f_usb30_udc *f_usb30 = endpoint->udc;
#if !defined(CONFIG_USB_GADGET_F_USB30_DMA_USE_BOUNCE_BUF)
	struct f_usb30_request *request = endpoint->req;
#endif
	unsigned char ep_channel = endpoint->ep.address &
						 USB_ENDPOINT_NUMBER_MASK;

	/* disable DMA transfer */
	enable_dma_transfer(endpoint, 0);

	/* stop stream transfer */
	if ((f_usb30->gadget.speed == USB_SPEED_SUPER) &&
		 endpoint->ep.comp_desc->bmAttributes &&
		 get_ss_streamactive(f_usb30->register_base_address,
					 ep_channel))
		set_ss_streamactive(f_usb30->register_base_address,
					 ep_channel, 0);

#if !defined(CONFIG_USB_GADGET_F_USB30_DMA_USE_BOUNCE_BUF)
	/* check DMA transfer buffer unmap */
	if ((endpoint->dma_transfer) &&
		 (request->req.dma != F_USB30_DMA_ADDR_INVALID)) {
		if (request->dma_buffer_map) {
			/* unmap DMA trans buffer and sync DMA trans buffer */
			dma_unmap_single(f_usb30->gadget.dev.parent,
					 request->req.dma,
					 request->req.length,
					 DMA_TO_DEVICE);
			request->req.dma = F_USB30_DMA_ADDR_INVALID;
			request->dma_buffer_map = 0;
		} else {
			dma_sync_single_for_cpu(f_usb30->gadget.dev.parent,
						 request->req.dma,
						 request->req.length,
						 DMA_TO_DEVICE);
		}
	}
#endif
	/* disable IntEmpty interrupt */
	f_usb30->gadget.speed == USB_SPEED_SUPER ?
		 set_ss_mskempty(f_usb30->register_base_address,
				 ep_channel, 1) :
		 set_hs_mskempty(f_usb30->register_base_address,
				 ep_channel, 1);

	if (!initialize)
		return;

	/* initialize endpoint */
	initialize_endpoint_hw(endpoint, 1, 0);
#endif
	return;
}

static void abort_out_transfer_dma(struct f_usb30_ep *endpoint,
	 unsigned char initialize)
{
#if defined(CONFIG_USB_GADGET_F_USB30_USED_DMA_TRANSFER)
	struct f_usb30_udc *f_usb30 = endpoint->udc;
#if !defined(CONFIG_USB_GADGET_F_USB30_DMA_USE_BOUNCE_BUF)
	struct f_usb30_request *request = endpoint->req;
#endif
	unsigned char ep_channel = endpoint->ep.address &
						 USB_ENDPOINT_NUMBER_MASK;

	/* disable DMA transfer */
	enable_dma_transfer(endpoint, 0);

	/* stop stream transfer */
	if ((f_usb30->gadget.speed == USB_SPEED_SUPER) &&
		 endpoint->ep.comp_desc->bmAttributes &&
		 get_ss_streamactive(f_usb30->register_base_address,
					 ep_channel))
		set_ss_streamactive(f_usb30->register_base_address,
					 ep_channel, 0);
#if !defined(CONFIG_USB_GADGET_F_USB30_DMA_USE_BOUNCE_BUF)
	/* check DMA transfer buffer unmap */
	if ((endpoint->dma_transfer) &&
		 (request->req.dma != F_USB30_DMA_ADDR_INVALID)) {
		if (request->dma_buffer_map) {
			/* unmap DMA trans buffer and sync DMA trans buffer */
			dma_unmap_single(f_usb30->gadget.dev.parent,
					 request->req.dma,
					 request->req.length,
					 DMA_FROM_DEVICE);
			request->req.dma = F_USB30_DMA_ADDR_INVALID;
			request->dma_buffer_map = 0;
		} else {
			dma_sync_single_for_cpu(f_usb30->gadget.dev.parent,
						 request->req.dma,
						 request->req.length,
						 DMA_FROM_DEVICE);
		}
	}
#endif
	if (!initialize)
		return;

	/* initialize endpoint */
	initialize_endpoint_hw(endpoint, 0, 1);
#endif
	return;
}

static unsigned char end_in_transfer_dma(struct f_usb30_ep *endpoint)
{
#if defined(CONFIG_USB_GADGET_F_USB30_USED_DMA_TRANSFER)
	struct f_usb30_udc *f_usb30 = endpoint->udc;
	struct f_usb30_request *request = endpoint->req;
	unsigned char ep_channel = endpoint->ep.address &
						 USB_ENDPOINT_NUMBER_MASK;
	void *base_addr = f_usb30->register_base_address;
	dma_addr_t dma_addr;
	unsigned long bytes;

	/* check empty wait */
	if ((f_usb30->gadget.speed == USB_SPEED_SUPER ?
		 (!get_ss_mskempty(base_addr, ep_channel)) :
		 (!get_hs_mskempty(base_addr, ep_channel)))) {
		/* complete request */
		abort_in_transfer_dma(endpoint, 0);
		return 1;
	}

	/* check transfer remain byte */
	if (request->req.length == request->req.actual) {
#if defined(CONFIG_USB_GADGET_F_USB30_BULK_IN_END_NOTIFY_TIMING_TO_HOST)
		/* check bulk transfer type */
		if (endpoint->attributes == USB_ENDPOINT_XFER_BULK) {
			/* clear & enable IntEmpty interrupt */
			f_usb30->gadget.speed == USB_SPEED_SUPER ?
				 clear_ss_intempty(base_addr, ep_channel) :
				 clear_hs_intempty(base_addr, ep_channel);
			f_usb30->gadget.speed == USB_SPEED_SUPER ?
				 set_ss_mskempty(base_addr, ep_channel, 0) :
				 set_hs_mskempty(base_addr, ep_channel, 0);
			return 0;
		}
#endif
		/* complete request */
		abort_in_transfer_dma(endpoint, 0);
		return 1;
	}

	/* calculate this time transfer byte */
	bytes = (request->req.length - request->req.actual) <
		 F_USB30_DMAC_TRANS_MAX_BYTES ?
		 request->req.length - request->req.actual :
		 F_USB30_DMAC_TRANS_MAX_BYTES;
	dev_dbg(f_usb30->gadget.dev.parent,
		"endpoint %u DMA is setup at length = %u, actual = %u, max packet = %u, this time = %u.\n",
		ep_channel, request->req.length, request->req.actual,
		endpoint->ep.maxpacket, (unsigned int) bytes);
#if defined(CONFIG_USB_GADGET_F_USB30_DMA_USE_BOUNCE_BUF)
	/* copy IN transfer data to noncachable buffer */
	memcpy(f_usb30->buffer[endpoint->dma_channel],
				 request->req.buf + request->req.actual, bytes);

	dma_addr = f_usb30->dma_buffer[endpoint->dma_channel];
#else
	dma_addr = request->req.dma + request->req.actual;
#endif

	/* update actual byte */
	request->req.actual += bytes;

	/* setup DMA transfer */
	set_dma_controller(endpoint, (unsigned long) dma_addr,
			 f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 (unsigned long) endpoint->ss_dma_fifo_addr :
			 (unsigned long) endpoint->hs_dma_fifo_addr, bytes);

	/* enable DMA transfer */
	enable_dma_transfer(endpoint, 1);
#endif
	return 0;
}

static unsigned char end_out_transfer_dma(struct f_usb30_ep *endpoint)
{
#if defined(CONFIG_USB_GADGET_F_USB30_USED_DMA_TRANSFER)
	struct f_usb30_udc *f_usb30 = endpoint->udc;
	struct f_usb30_request *request = endpoint->req;
	unsigned char ep_channel = endpoint->ep.address &
						 USB_ENDPOINT_NUMBER_MASK;
	void *base_addr = f_usb30->register_base_address;
	dma_addr_t dma_addr;
	unsigned long bytes;

	/* get this time transfer byte */
	if (f_usb30->gadget.speed == USB_SPEED_SUPER) {
		bytes = get_ss_tcnt(base_addr, ep_channel);
	} else {
		bytes = get_hs_tcnt(base_addr, ep_channel);
		if (bytes == (unsigned long) -1)
			bytes = (request->req.length - request->req.actual) <
				F_USB30_DMAC_TRANS_MAX_BYTES ?
				(request->req.length - request->req.actual) :
				F_USB30_DMAC_TRANS_MAX_BYTES;
		else
			bytes = request->req.length - bytes;
	}
#if defined(CONFIG_USB_GADGET_F_USB30_DMA_USE_BOUNCE_BUF)
	/* copy OUT transfer data from noncachable buffer */
	memcpy(request->req.buf + request->req.actual,
				 f_usb30->buffer[endpoint->dma_channel], bytes);
#endif
	/* update actual bytes */
	request->req.actual += bytes;

	/* check transfer request complete */
	if ((request->req.length <= request->req.actual) ||
		 (bytes % endpoint->ep.maxpacket) || (!bytes)) {
		/* complete request */
		abort_out_transfer_dma(endpoint, 0);
		return 1;
	}

	/* calculate this time transfer byte */
	bytes = (request->req.length - request->req.actual) <
		 F_USB30_DMAC_TRANS_MAX_BYTES ?
		 (request->req.length - request->req.actual) :
		 F_USB30_DMAC_TRANS_MAX_BYTES;
	dev_dbg(f_usb30->gadget.dev.parent,
		"endpoint %u DMA is setup at length = %u, actual = %u, max packet = %u, this time = %u.\n",
		ep_channel, request->req.length, request->req.actual,
		endpoint->ep.maxpacket, (unsigned int) bytes);

#if defined(CONFIG_USB_GADGET_F_USB30_DMA_USE_BOUNCE_BUF)
	dma_addr = f_usb30->dma_buffer[endpoint->dma_channel];
#else
	dma_addr = request->req.dma + request->req.actual;
#endif

	/* setup DMA transfer */
	set_dma_controller(endpoint,
			 f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 (unsigned long) endpoint->ss_dma_fifo_addr :
			 (unsigned long) endpoint->hs_dma_fifo_addr,
			 (unsigned long) dma_addr, bytes);

	/* enable DMA transfer */
	enable_dma_transfer(endpoint, 1);
#endif
	return 0;
}

#if defined(CONFIG_USB_GADGET_F_USB30_USED_DMA_TRANSFER)
static void on_end_in_transfer(struct f_usb30_ep *);
static void on_end_out_transfer(struct f_usb30_ep *);
static irqreturn_t on_dma_transfer_contoller(int irq, void *dev_id)
{
	unsigned long counter;
	struct f_usb30_udc *f_usb30 = dev_id;
	void *base_addr = f_usb30->register_base_address;
	struct f_usb30_ep *endpoint = &f_usb30->udc_endpoint[0];
	unsigned char *channel = &f_usb30->dma_ep[0];

	/* check argument */
	if (unlikely(!dev_id))
		return IRQ_NONE;

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is started.\n",
							 __func__);

	/* DMA controller interrupt request assert check */
	if (unlikely((irq != f_usb30->dmac_in_irq) &&
			 (irq != f_usb30->dmac_out_irq))) {
		dev_dbg(f_usb30->gadget.dev.parent,
			"%s() is ended at non process.\n", __func__);
		return IRQ_NONE;
	}

	/* get spin lock */
	spin_lock(&f_usb30->lock);

	for (counter = 0; counter < F_USB30_MAX_DMAC; counter++) {
		if (endpoint[channel[counter]].ep.address & USB_DIR_IN) {
			/* DMA IN transfer end interrupt factor */
			if ((get_intusbfin(base_addr, counter)) &&
				 (!get_mskintusbfin(base_addr, counter))) {
				/* clear UsbFin interrupt factor */
				clear_intusbfin(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					 "UsbFin%u interrrupt occurred.\n",
						 (unsigned int) counter);

				/* process IN transfer end */
				on_end_in_transfer(&endpoint[channel[counter]]);
			} else if ((get_intstarterr(base_addr, counter)) &&
				 (!get_mskintstarterr(base_addr, counter))) {
				/* clear StartErr interrupt factor */
				clear_intstarterr(base_addr, counter);

				dev_err(f_usb30->gadget.dev.parent,
					 "StartErr%u interrrupt occurred.\n",
					 (unsigned int) counter);

				/* abort IN transfer */
				set_dmaabort(base_addr, counter, 1);
			} else if ((get_intslverr(base_addr, counter)) &&
				 (!get_mskintslverr(base_addr, counter))) {
				/* clear SlvErr interrupt factor */
				clear_intslverr(base_addr, counter);

				dev_err(f_usb30->gadget.dev.parent,
					 "SlvErr%u interrrupt occurred.\n",
					 (unsigned int) counter);

				/* abort IN transfer */
				set_dmaabort(base_addr, counter, 1);
			} else if ((get_intdecerr(base_addr, counter)) &&
				 (!get_mskintdecerr(base_addr, counter))) {
				/* clear DecErr interrupt factor */
				clear_intdecerr(base_addr, counter);

				dev_err(f_usb30->gadget.dev.parent,
					 "DecErr%u interrrupt occurred.\n",
					 (unsigned int) counter);

				/* abort IN transfer */
				set_dmaabort(base_addr, counter, 1);
			} else if ((get_intuexdreq(base_addr, counter)) &&
				 (!get_mskintuexdreq(base_addr, counter))) {
				/* clear UexDreq interrupt factor */
				clear_intuexdreq(base_addr, counter);

				dev_err(f_usb30->gadget.dev.parent,
					 "UexDreq%u interrrupt occurred.\n",
					 (unsigned int) counter);

				/* abort IN transfer */
				set_dmaabort(base_addr, counter, 1);
			} else if ((get_intaxifin(base_addr, counter)) &&
				 (!get_mskintaxifin(base_addr, counter))) {
				/* clear AXIFin interrupt factor */
				clear_intaxifin(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					 "AXIFin%u interrrupt occurred.\n",
					 (unsigned int) counter);
			} else if ((get_intexokay(base_addr, counter)) &&
				 (!get_mskintexokay(base_addr, counter))) {
				/* clear ExOkay interrupt factor */
				clear_intexokay(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					 "ExOkay%u interrrupt occurred.\n",
					 (unsigned int) counter);
			} else if ((get_intremainfin(base_addr, counter)) &&
				 (!get_mskintremainfin(base_addr, counter))) {
				/* clear RemainFin interrupt factor */
				clear_intremainfin(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					 "RemainFin%u interrrupt occurred.\n",
					 (unsigned int) counter);
			} else if ((get_intnull(base_addr, counter)) &&
				 (!get_mskintnull(base_addr, counter))) {
				/* clear RemainFin interrupt factor */
				clear_intnull(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					 "RemainFin%u interrrupt occurred.\n",
					 (unsigned int) counter);
			} else {
				dev_dbg(f_usb30->gadget.dev.parent,
					 "Other interrrupt occurred.\n");
			}
		} else {
			/* DMA OUT transfer end interrupt factor */
			if ((get_intaxifin(base_addr, counter)) &&
				 (!get_mskintaxifin(base_addr, counter))) {
				/* clear AXIFin interrupt factor */
				clear_intaxifin(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					 "AXIFin%u interrrupt occurred.\n",
					 (unsigned int) counter);

				/* process OUT transfer end */
				on_end_out_transfer(
						&endpoint[channel[counter]]);
			} else if ((get_intusbover(base_addr, counter)) &&
				 (!get_mskintusbover(base_addr, counter))) {
				/* clear UsbOver interrupt factor */
				clear_intusbover(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					 "UsbOver%u interrrupt occurred.\n",
					 (unsigned int) counter);

				/* process OUT transfer end */
				on_end_out_transfer(
						&endpoint[channel[counter]]);
			} else if ((get_intusbunder(base_addr, counter)) &&
				 (!get_mskintusbunder(base_addr, counter))) {
				/* clear UsbUnder interrupt factor */
				clear_intusbunder(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					 "UsbUnder%u interrrupt occurred.\n",
					 (unsigned int) counter);

				/* abort OUT transfer */
				set_dmaabort(base_addr, counter, 1);
			} else if ((get_intnull(base_addr, counter)) &&
				 (!get_mskintnull(base_addr, counter))) {
				/* clear Null interrupt factor */
				clear_intnull(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					 "Null%u interrrupt occurred.\n",
					 (unsigned int) counter);

				/* process OUT transfer end */
				on_end_out_transfer(
						&endpoint[channel[counter]]);
			} else if ((get_intstarterr(base_addr, counter)) &&
				 (!get_mskintstarterr(base_addr, counter))) {
				/* clear StartErr interrupt factor */
				clear_intstarterr(base_addr, counter);

				dev_err(f_usb30->gadget.dev.parent,
					 "StartErr%u interrrupt occurred.\n",
					 (unsigned int) counter);

				/* abort OUT transfer */
				set_dmaabort(base_addr, counter, 1);
			} else if ((get_intslverr(base_addr, counter)) &&
				 (!get_mskintslverr(base_addr, counter))) {
				/* clear SlvErr interrupt factor */
				clear_intslverr(base_addr, counter);

				dev_err(f_usb30->gadget.dev.parent,
					 "SlvErr%u interrrupt occurred.\n",
					 (unsigned int) counter);

				/* abort OUT transfer */
				set_dmaabort(base_addr, counter, 1);
			} else if ((get_intdecerr(base_addr, counter)) &&
				 (!get_mskintdecerr(base_addr, counter))) {
				/* clear DecErr interrupt factor */
				clear_intdecerr(base_addr, counter);

				dev_err(f_usb30->gadget.dev.parent,
					 "DecErr%u interrrupt occurred.\n",
					 (unsigned int) counter);

				/* abort OUT transfer */
				set_dmaabort(base_addr, counter, 1);
			} else if ((get_intuexdreq(base_addr, counter)) &&
				 (!get_mskintuexdreq(base_addr, counter))) {
				/* clear UexDreq interrupt factor */
				clear_intuexdreq(base_addr, counter);

				dev_err(f_usb30->gadget.dev.parent,
					 "UexDreq%u interrrupt occurred.\n",
					 (unsigned int) counter);

				/* abort OUT transfer */
				set_dmaabort(base_addr, counter, 1);
			} else if ((get_intusbfin(base_addr, counter)) &&
				 (!get_mskintusbfin(base_addr, counter))) {
				/* clear UsbFin interrupt factor */
				clear_intusbfin(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					 "UsbFin%u interrrupt occurred.\n",
					 (unsigned int) counter);
			} else if ((get_intexokay(base_addr, counter)) &&
				 (!get_mskintexokay(base_addr, counter))) {
				/* clear ExOkay interrupt factor */
				clear_intexokay(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					 "ExOkay%u interrrupt occurred.\n",
					 (unsigned int) counter);
			} else if ((get_intremainfin(base_addr, counter)) &&
				 (!get_mskintremainfin(base_addr, counter))) {
				/* clear RemainFin interrupt factor */
				clear_intremainfin(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					 "RemainFin%u interrrupt occurred.\n",
					 (unsigned int) counter);
			} else {
				dev_dbg(f_usb30->gadget.dev.parent,
					 "Other interrrupt occurred.\n");
			}
		}
	}

	/* release spin lock */
	spin_unlock(&f_usb30->lock);
	dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n", __func__);

	return IRQ_HANDLED;
}
#endif

static void initialize_controller(struct f_usb30_udc *f_usb30, int ss)
{
	unsigned long counter;
	unsigned long alt_counter;
	void *base_addr = f_usb30->register_base_address;

	/* initialize F_USB30 DMA controller register */
	initialize_dma_controller(f_usb30);

	/* initialize F_USB30 USB3.0 controller register */
	if (ss) {
		/* initialize F_USB30 SS device control/status register */
		set_ss_selfpw(base_addr, 1);
		set_ss_mskfncsusp(base_addr, 1);
		set_ss_msku2inactto(base_addr, 1);
		set_ss_msksetup(base_addr, 1);
		set_ss_msksetconf(base_addr, 1);
		set_ss_msksuspendb(base_addr, 1);
		set_ss_msksuspende(base_addr, 1);
		set_ss_mskpolltou0(base_addr, 1);
		set_ss_mskenterpoll(base_addr, 1);
		set_ss_mskssdisable(base_addr, 1);
		set_ss_mskvdtest(base_addr, 1);
		set_ss_msksshrstb(base_addr, 1);
		set_ss_msksshrste(base_addr, 1);
		set_ss_msksswrstb(base_addr, 1);
		set_ss_msksswrste(base_addr, 1);
		clear_ss_intfncsusp(base_addr);
		clear_ss_intu2inactto(base_addr);
		clear_ss_intsetup(base_addr);
		clear_ss_intsetconf(base_addr);
		clear_ss_intsuspendb(base_addr);
		clear_ss_intsuspende(base_addr);
		clear_ss_intpolltou0(base_addr);
		clear_ss_intenterpoll(base_addr);
		clear_ss_intssdisable(base_addr);
		clear_ss_intvdtest(base_addr);
		clear_ss_intsshrstb(base_addr);
		clear_ss_intsshrste(base_addr);
		clear_ss_intsswrstb(base_addr);
		clear_ss_intsswrste(base_addr);

		/* disable F_USB30 SS U1/U2 status auto entry */
		set_ss_cnt_u1inactto(base_addr, 0xFF);
		set_ss_cnt_u2inactto(base_addr, 0xFF);
		set_ss_blki_u1inactto(base_addr, 0xFF);
		set_ss_blki_u2inactto(base_addr, 0xFF);
		set_ss_blko_u1inactto(base_addr, 0xFF);
		set_ss_blko_u2inactto(base_addr, 0xFF);

		/* set to reject U1/U2 status entry request from HOST  */
		set_ss_ssovrden(base_addr, 1);
		set_ss_rjct_rcvu1(base_addr, 1);
		set_ss_rjct_rcvu2(base_addr, 1);
		set_ss_ssovrden(base_addr, 0);

		/* initialize F_USB30 SS ep0 control/status controller reg */
		set_ss_reqstall0(base_addr, 0);
		set_ss_nrdyresp0(base_addr, 0);
		set_ss_rewdifo0(base_addr, 0);
		set_ss_mskep(base_addr, ENDPOINT0, 1);
		set_ss_mskready0i(base_addr, 1);
		set_ss_mskready0o(base_addr, 1);
		set_ss_mskpktpnd0(base_addr, 1);
		set_ss_mskstalled0(base_addr, 1);
		set_ss_msknrdy0(base_addr, 1);
		set_ss_mskclstall0(base_addr, 1);
		clear_ss_intready0i(base_addr);
		clear_ss_intready0o(base_addr);
		clear_ss_intpktpnd0(base_addr);
		clear_ss_intstalled0(base_addr);
		clear_ss_intnrdy0(base_addr);
		clear_ss_intclstall0(base_addr);

		/* initialize F_USB30 SS epx control/status controller reg */
		for (counter = ENDPOINT1;
			 counter < F_USB30_MAX_EP; counter++) {
			set_ss_reqstall(base_addr, counter, 0);
			set_ss_rewdifo(base_addr, counter, 0);
			set_ss_stalldis(base_addr, counter, 0);
			set_ss_inistall(base_addr, counter, 0);
			set_ss_nrdyresp(base_addr, counter, 0);
			set_ss_enspr(base_addr, counter, 0);
			set_ss_enspd(base_addr, counter, 0);
			set_ss_dmamode(base_addr, counter, 0);
			set_ss_mskdmareq(base_addr, counter, 1);
			set_ss_mskep(base_addr, counter, 1);
			set_ss_mskspr(base_addr, counter, 1);
			set_ss_mskspd(base_addr, counter, 1);
			set_ss_mskclstream(base_addr, counter, 1);
			set_ss_mskready(base_addr, counter, 1);
			set_ss_mskpktpnd(base_addr, counter, 1);
			set_ss_msksdend(base_addr, counter, 1);
			set_ss_mskdend(base_addr, counter, 1);
			set_ss_mskempty(base_addr, counter, 1);
			set_ss_mskstalled(base_addr, counter, 1);
			set_ss_msknrdy(base_addr, counter, 1);
			set_ss_mskclstall(base_addr, counter, 1);
			clear_ss_intspr(base_addr, counter);
			clear_ss_intspd(base_addr, counter);
			clear_ss_intclstream(base_addr, counter);
			clear_ss_intready(base_addr, counter);
			clear_ss_intpktpnd(base_addr, counter);
			clear_ss_intsdend(base_addr, counter);
			clear_ss_intdend(base_addr, counter);
			clear_ss_intempty(base_addr, counter);
			clear_ss_intstalled(base_addr, counter);
			clear_ss_intnrdy(base_addr, counter);
			clear_ss_intclstall(base_addr, counter);
		}

		/* initialize F_USB30 SS intf/alt controller register */
		set_ss_numintf(base_addr, F_USB30_MAX_INTF);
		set_ss_firstintf(base_addr, INTERFACE0, 1);
		set_ss_numaintf(base_addr, INTERFACE0, 1);
		clear_ss_achg(base_addr, INTERFACE0);
		for (counter = INTERFACE1;
				 counter < F_USB30_MAX_INTF; counter++) {
			set_ss_firstintf(base_addr, counter, 0);
			set_ss_numaintf(base_addr, counter, 1);
			clear_ss_achg(base_addr, counter);
		}

		/* initialize F_USB30 SS endpoint config controller reg */
		set_ss_configwren(base_addr, 1);
		for (counter = F_USB30_MAX_EP - 1;
				 counter > ENDPOINT0; counter--) {
			set_ss_epnum(base_addr, counter, counter);
			set_ss_epconf(base_addr, counter, 1);
			set_ss_intf(base_addr, counter, 1);
			set_ss_altmap(base_addr, counter, 1);
			set_ss_numpmode(base_addr, counter, 0);
			set_ss_diseob(base_addr, counter, 0);
			set_ss_maxburst(base_addr, counter, 7);
		}
		set_ss_configwren(base_addr, 0);

	} else {
		/* initialize F_USB30 HS cpu access controller register */
		set_hs_softreset(base_addr);

		/* select High Speed or Full Speed by Gadget Driver */
		set_hs_reqspeed(base_addr,
				 gadget_is_dualspeed(&f_usb30->gadget) ?
				 REQ_SPEED_HIGH_SPEED :
				 REQ_SPEED_FULL_SPEED);

		/* initialize HS/FS device control/status register */
		set_hs_reqresume(base_addr, 0);
		set_hs_enrmtwkup(base_addr, 1);
		set_hs_selfpwr(base_addr, 1);
		set_hs_physusp(base_addr, 0);
		set_hs_pmode(base_addr, 0);
		set_hs_lmode(base_addr, 0);
		set_hs_sofsel(base_addr, 0);
		set_hs_lpbkphy(base_addr, 0);
		set_hs_fscalib(base_addr, 3);
		set_hs_hscalib(base_addr, 2);
		set_hs_mskerraticerr(base_addr, 1);
		set_hs_msksof(base_addr, 1);
		set_hs_mskusbrstb(base_addr, 1);
		set_hs_mskusbrste(base_addr, 1);
		set_hs_msksuspendb(base_addr, 1);
		set_hs_msksuspende(base_addr, 1);
		set_hs_msksetup(base_addr, 1);
		set_hs_msksetconf(base_addr, 1);
		clear_hs_interraticerr(base_addr);
		clear_hs_intsof(base_addr);
		clear_hs_intusbrstb(base_addr);
		clear_hs_intusbrste(base_addr);
		clear_hs_intsuspendb(base_addr);
		clear_hs_intsuspende(base_addr);
		clear_hs_intsetup(base_addr);
		clear_hs_intsetconf(base_addr);

		/* initialize HS/FS ep0 control/status controller reg */
		set_hs_testmode0(base_addr, 0);
		set_hs_reqstall0(base_addr, 0);
		set_hs_seltx0i(base_addr, 0);
		set_hs_seltx0o(base_addr, 0);
		set_hs_mskep(base_addr, ENDPOINT0, 1);
		set_hs_mskready0i(base_addr, 1);
		set_hs_mskready0o(base_addr, 1);
		set_hs_mskping0o(base_addr, 1);
		set_hs_mskstalled0(base_addr, 1);
		set_hs_msknack0(base_addr, 1);
		set_hs_mskclstall0(base_addr, 1);
		clear_hs_intready0i(base_addr);
		clear_hs_intready0o(base_addr);
		clear_hs_intping0o(base_addr);
		clear_hs_intstalled0(base_addr);
		clear_hs_intnack0(base_addr);
		clear_hs_intclstall0(base_addr);

		/* initialize HS/FS epx control/status controller reg */
		for (counter = ENDPOINT1; counter < F_USB30_MAX_EP;
			 counter++) {
			set_hs_reqstall(base_addr, counter, 0);
			set_hs_nackresp(base_addr, counter, 1);
			set_hs_nullresp(base_addr, counter, 0);
			set_hs_toggledis(base_addr, counter, 0);
			set_hs_stalldis(base_addr, counter, 0);
			set_hs_seltx(base_addr, counter, 0);
			set_hs_enspr(base_addr, counter, 0);
			set_hs_enspdd(base_addr, counter, 0);
			set_hs_mskdmareq(base_addr, counter, 1);
			set_hs_dmamode(base_addr, counter, 0);
			set_hs_mskep(base_addr, counter, 1);
			set_hs_mskspr(base_addr, counter, 1);
			set_hs_mskspdd(base_addr, counter, 1);
			set_hs_mskready(base_addr, counter, 1);
			set_hs_mskping(base_addr, counter, 1);
			set_hs_mskachgif(base_addr, counter, 1);
			set_hs_mskdend(base_addr, counter, 1);
			set_hs_mskempty(base_addr, counter, 1);
			set_hs_mskstalled(base_addr, counter, 1);
			set_hs_msknack(base_addr, counter, 1);
			set_hs_mskclstall(base_addr, counter, 1);
			clear_hs_intspr(base_addr, counter);
			clear_hs_intspdd(base_addr, counter);
			clear_hs_intready(base_addr, counter);
			clear_hs_intping(base_addr, counter);
			clear_hs_intachgif(base_addr, counter);
			clear_hs_intdend(base_addr, counter);
			clear_hs_intempty(base_addr, counter);
			clear_hs_intstalled(base_addr, counter);
			clear_hs_intnack(base_addr, counter);
			clear_hs_intclstall(base_addr, counter);
		}

		/* initialize HS/FS interface/alternate controller register */
		set_hs_numintf(base_addr, 0);
		set_hs_testmodeif(base_addr, 0);
		for (counter = INTERFACE0;
			 counter < F_USB30_MAX_INTF; counter++) {
			set_hs_numaltintf(base_addr, counter, 0);
			set_hs_mskachgif(base_addr, counter, 1);
			clear_hs_intachgif(base_addr, counter);
		}

		/* initialize HS/FS endpoint config controller reg */
		set_hs_configwren(base_addr, 1);
		set_hs_makeupdata(base_addr);
		for (counter = F_USB30_MAX_EP - 1;
			 counter > ENDPOINT0; counter--) {
			for (alt_counter = 0; alt_counter < F_USB30_MAX_ALT;
				 alt_counter++) {
				set_hs_epnum(base_addr, counter,
						alt_counter, 0);
				set_hs_io(base_addr, counter, alt_counter, 0);
				set_hs_type(base_addr, counter, alt_counter,
						 TYPE_UNUSED);
				set_hs_conf(base_addr, counter, alt_counter, 0);
				set_hs_intf(base_addr, counter, alt_counter, 0);
				set_hs_alt(base_addr, counter, alt_counter, 0);
				set_hs_size(base_addr, counter, alt_counter, 0);
				set_hs_numtr(base_addr, counter, alt_counter);
			}
		}
		set_hs_epnum(base_addr, ENDPOINT0, 0, 0);
		set_hs_io(base_addr, ENDPOINT0, 0, 0);
		set_hs_type(base_addr, ENDPOINT0, 0, TYPE_UNUSED);
		set_hs_conf(base_addr, ENDPOINT0, 0, 0);
		set_hs_intf(base_addr, ENDPOINT0, 0, 0);
		set_hs_alt(base_addr, ENDPOINT0, 0, 0);
		set_hs_size(base_addr, ENDPOINT0, 0, 0);
		set_hs_numtr(base_addr, ENDPOINT0, 0);
		set_hs_configwren(base_addr, 0);

		/* wait PHY reset release */
		for (counter = 0xffff; ((counter) &&
				 (get_hs_phyreset(base_addr))); counter--)
			;
	}
	return;
}

static void initialize_endpoint_hw(struct f_usb30_ep *endpoint,
	 unsigned char in_fifo, unsigned char out_fifo)
{
	struct f_usb30_udc *f_usb30 = endpoint->udc;
	unsigned char ep_channel = endpoint->ep.address &
						 USB_ENDPOINT_NUMBER_MASK;
	void *base_addr = f_usb30->register_base_address;

	if (f_usb30->gadget.speed == USB_SPEED_SUPER) {
		if (ep_channel == ENDPOINT0) {
			if (in_fifo) {
				/* check new SETUP transfer or reset */
				if ((get_ss_intsetup(base_addr)) ||
					 (get_ss_intsshrstb(base_addr)) ||
					 (get_ss_intsswrstb(base_addr)))
					in_fifo = 0;
				else
					/* initialize IN FIFO */
					set_ss_init0i(base_addr);
			}

			if (out_fifo) {
				/* check new SETUP transfer or reset */
				if ((get_ss_intsetup(base_addr)) ||
					 (get_ss_intsshrstb(base_addr)) ||
					 (get_ss_intsswrstb(base_addr)))
					out_fifo = 0;
				else
					/* initialize OUT FIFO */
					set_ss_init0o(base_addr);
			}

			/* initialize endpoint control / status register */
			set_ss_nrdyresp0(base_addr, 0);
			set_ss_reqstall0(base_addr, 0);
			if (in_fifo) {
				clear_ss_intready0i(base_addr);
				set_ss_mskready0i(base_addr, 1);
			}
			if (out_fifo)
				set_ss_mskready0o(base_addr, 1);
			set_ss_mskpktpnd0(base_addr, 1);
			set_ss_mskstalled0(base_addr, 0);
			set_ss_msknrdy0(base_addr, 1);
			set_ss_mskclstall0(base_addr, 0);
			set_ss_mskep(base_addr, ep_channel, 0);
		} else {
			/* check endpoint transfer direction */
			if (((endpoint->ep.address & USB_DIR_IN) &&
				 (in_fifo)) || ((!(endpoint->ep.address &
				 USB_DIR_IN)) && (out_fifo)))
				/* initialize FIFO */
				set_ss_init(base_addr, ep_channel);

			/* initialize endpoint control / status register */
			set_ss_rewdifo(base_addr, ep_channel, 0);
			set_ss_stalldis(base_addr, ep_channel, 0);
			set_ss_inistall(base_addr, ep_channel, 0);
			set_ss_nrdyresp(base_addr, ep_channel, 0);
			set_ss_enspr(base_addr, ep_channel, 0);
			set_ss_enspd(base_addr, ep_channel, 0);
			set_ss_mskdmareq(base_addr, ep_channel, 1);
			set_ss_dmamode(base_addr, ep_channel, 0);
			clear_ss_intready(base_addr, ep_channel);
			clear_ss_intempty(base_addr, ep_channel);
			set_ss_mskep(base_addr, ep_channel, 0);
			set_ss_mskspr(base_addr, ep_channel, 1);
			set_ss_mskspd(base_addr, ep_channel, 1);
			set_ss_mskclstream(base_addr, ep_channel, 1);
			set_ss_mskready(base_addr, ep_channel, 1);
			set_ss_mskpktpnd(base_addr, ep_channel, 1);
			set_ss_msksdend(base_addr, ep_channel, 1);
			set_ss_mskdend(base_addr, ep_channel, 1);
			set_ss_mskempty(base_addr, ep_channel, 1);
			set_ss_mskstalled(base_addr, ep_channel, 0);
			set_ss_msknrdy(base_addr, ep_channel, 1);
			set_ss_mskclstall(base_addr, ep_channel, 0);
		}
	} else {
		if (ep_channel == ENDPOINT0) {
			if (in_fifo) {
				/* check new SETUP transfer or bus reset */
				if ((get_hs_intsetup(base_addr)) ||
					 (get_hs_intusbrstb(base_addr)))
					in_fifo = 0;
				else
					/* initialize IN FIFO */
					set_hs_init0i(base_addr);
			}

			if (out_fifo) {
				/* check new SETUP transfer or bus reset */
				if ((get_hs_intsetup(base_addr)) ||
					 (get_hs_intusbrstb(base_addr)))
					out_fifo = 0;
				else
					/* initialize OUT FIFO */
					set_hs_init0o(base_addr);
			}

			/* initialize endpoint control / status register */
			set_hs_testmode0(base_addr, 0);
			set_hs_reqstall0(base_addr, 0);
			set_hs_seltx0i(base_addr, 0);
			set_hs_seltx0o(base_addr, 0);
			if (in_fifo) {
				clear_hs_intready0i(base_addr);
				set_hs_mskready0i(base_addr, 1);
			}
			if (out_fifo)
				set_hs_mskready0o(base_addr, 1);
			set_hs_mskping0o(base_addr, 1);
			set_hs_mskstalled0(base_addr, 0);
			set_hs_msknack0(base_addr, 1);
			set_hs_mskclstall0(base_addr, 0);
			set_hs_mskep(base_addr, ep_channel, 0);
		} else {
			/* check endpoint transfer direction */
			if (((endpoint->ep.address & USB_DIR_IN) &&
				 (in_fifo)) || ((!(endpoint->ep.address &
				  USB_DIR_IN)) && (out_fifo)))
				/* initialize FIFO */
				set_hs_init(base_addr, ep_channel);

			/* initialize endpoint control / status register */
			set_hs_nackresp(base_addr, ep_channel, 0);
			set_hs_nullresp(base_addr, ep_channel, 0);
			set_hs_toggledis(base_addr, ep_channel, 0);
			set_hs_stalldis(base_addr, ep_channel, 0);
			set_hs_seltx(base_addr, ep_channel, 0);
			set_hs_enspr(base_addr, ep_channel, 0);
			set_hs_enspdd(base_addr, ep_channel, 0);
			set_hs_mskdmareq(base_addr, ep_channel, 1);
			set_hs_dmamode(base_addr, ep_channel, 0);
			clear_hs_intready(base_addr, ep_channel);
			clear_hs_intempty(base_addr, ep_channel);
			set_hs_mskready(base_addr, ep_channel, 1);
			set_hs_mskspr(base_addr, ep_channel, 1);
			set_hs_mskspdd(base_addr, ep_channel, 1);
			set_hs_mskping(base_addr, ep_channel, 1);
			set_hs_mskachgif(base_addr, ep_channel, 1);
			set_hs_mskempty(base_addr, ep_channel, 1);
			set_hs_mskstalled(base_addr, ep_channel, 0);
			set_hs_msknack(base_addr, ep_channel, 1);
			set_hs_mskclstall(base_addr, ep_channel, 0);
			set_hs_mskep(base_addr, ep_channel, 0);
		}
	}

	return;
}

static void initialize_endpoint_configure(struct f_usb30_udc *f_usb30)
{
	static unsigned char ep_type[] = {
		F_USB30_TRANS_TYPE_EP0,
		F_USB30_TRANS_TYPE_EP1,
		F_USB30_TRANS_TYPE_EP2,
		F_USB30_TRANS_TYPE_EP3,
		F_USB30_TRANS_TYPE_EP4,
		F_USB30_TRANS_TYPE_EP5,
		F_USB30_TRANS_TYPE_EP6,
		F_USB30_TRANS_TYPE_EP7,
		F_USB30_TRANS_TYPE_EP8,
		F_USB30_TRANS_TYPE_EP9,
		F_USB30_TRANS_TYPE_EP10,
		F_USB30_TRANS_TYPE_EP11,
		F_USB30_TRANS_TYPE_EP12,
		F_USB30_TRANS_TYPE_EP13,
		F_USB30_TRANS_TYPE_EP14,
		F_USB30_TRANS_TYPE_EP15,
	};
	static unsigned char ep_dir[] = {
		F_USB30_TRANS_DIR_EP0,
		F_USB30_TRANS_DIR_EP1,
		F_USB30_TRANS_DIR_EP2,
		F_USB30_TRANS_DIR_EP3,
		F_USB30_TRANS_DIR_EP4,
		F_USB30_TRANS_DIR_EP5,
		F_USB30_TRANS_DIR_EP6,
		F_USB30_TRANS_DIR_EP7,
		F_USB30_TRANS_DIR_EP8,
		F_USB30_TRANS_DIR_EP9,
		F_USB30_TRANS_DIR_EP10,
		F_USB30_TRANS_DIR_EP11,
		F_USB30_TRANS_DIR_EP12,
		F_USB30_TRANS_DIR_EP13,
		F_USB30_TRANS_DIR_EP14,
		F_USB30_TRANS_DIR_EP15,
	};
	static signed char ep_intf[] = {
		F_USB30_INTF_CHANNEL_EP0,
		F_USB30_INTF_CHANNEL_EP1,
		F_USB30_INTF_CHANNEL_EP2,
		F_USB30_INTF_CHANNEL_EP3,
		F_USB30_INTF_CHANNEL_EP4,
		F_USB30_INTF_CHANNEL_EP5,
		F_USB30_INTF_CHANNEL_EP6,
		F_USB30_INTF_CHANNEL_EP7,
		F_USB30_INTF_CHANNEL_EP8,
		F_USB30_INTF_CHANNEL_EP9,
		F_USB30_INTF_CHANNEL_EP10,
		F_USB30_INTF_CHANNEL_EP11,
		F_USB30_INTF_CHANNEL_EP12,
		F_USB30_INTF_CHANNEL_EP13,
		F_USB30_INTF_CHANNEL_EP14,
		F_USB30_INTF_CHANNEL_EP15,
	};
	static const char * const ep_name[] = {
		F_USB30_NAME_STRING_EP0,
		F_USB30_NAME_STRING_EP1,
		F_USB30_NAME_STRING_EP2,
		F_USB30_NAME_STRING_EP3,
		F_USB30_NAME_STRING_EP4,
		F_USB30_NAME_STRING_EP5,
		F_USB30_NAME_STRING_EP6,
		F_USB30_NAME_STRING_EP7,
		F_USB30_NAME_STRING_EP8,
		F_USB30_NAME_STRING_EP9,
		F_USB30_NAME_STRING_EP10,
		F_USB30_NAME_STRING_EP11,
		F_USB30_NAME_STRING_EP12,
		F_USB30_NAME_STRING_EP13,
		F_USB30_NAME_STRING_EP14,
		F_USB30_NAME_STRING_EP15,
	};
	static signed char ep_dma_ch[] = {
#if defined(CONFIG_USB_GADGET_F_USB30_USED_DMA_TRANSFER)
		F_USB30_DMAC_CHANNEL_EP0,
		F_USB30_DMAC_CHANNEL_EP1,
		F_USB30_DMAC_CHANNEL_EP2,
		F_USB30_DMAC_CHANNEL_EP3,
		F_USB30_DMAC_CHANNEL_EP4,
		F_USB30_DMAC_CHANNEL_EP5,
		F_USB30_DMAC_CHANNEL_EP6,
		F_USB30_DMAC_CHANNEL_EP7,
		F_USB30_DMAC_CHANNEL_EP8,
		F_USB30_DMAC_CHANNEL_EP9,
		F_USB30_DMAC_CHANNEL_EP10,
		F_USB30_DMAC_CHANNEL_EP11,
		F_USB30_DMAC_CHANNEL_EP12,
		F_USB30_DMAC_CHANNEL_EP13,
		F_USB30_DMAC_CHANNEL_EP14,
		F_USB30_DMAC_CHANNEL_EP15,
#else
		-1,		/* endpoint0 */
		-1,		/* endpoint1 */
		-1,		/* endpoint2 */
		-1,		/* endpoint3 */
		-1,		/* endpoint4 */
		-1,		/* endpoint5 */
		-1,		/* endpoint6 */
		-1,		/* endpoint7 */
		-1,		/* endpoint8 */
		-1,		/* endpoint9 */
		-1,		/* endpoint10 */
		-1,		/* endpoint11 */
		-1,		/* endpoint12 */
		-1,		/* endpoint13 */
		-1,		/* endpoint14 */
		-1,		/* endpoint15 */
#endif
	};
	static unsigned char intf_alt[] = {
		F_USB30_ALT_INTF0,
		F_USB30_ALT_INTF1,
		F_USB30_ALT_INTF2,
		F_USB30_ALT_INTF3,
		F_USB30_ALT_INTF4,
		F_USB30_ALT_INTF5,
		F_USB30_ALT_INTF6,
		F_USB30_ALT_INTF7,
		F_USB30_ALT_INTF8,
		F_USB30_ALT_INTF9,
		F_USB30_ALT_INTF10,
		F_USB30_ALT_INTF11,
		F_USB30_ALT_INTF12,
		F_USB30_ALT_INTF13,
		F_USB30_ALT_INTF14,
		F_USB30_ALT_INTF15,
	};
	struct f_usb30_ep *endpoint = &f_usb30->udc_endpoint[0];
	unsigned long counter = 0;

	/* initialzie endpoint 0 configure data */
	endpoint[ENDPOINT0].ep.name = ep_name[ENDPOINT0];
	endpoint[ENDPOINT0].ep.maxpacket =
			 ep_fifo_size[counter][
			f_usb30->gadget.speed == USB_SPEED_UNKNOWN ?
			 USB_SPEED_SUPER : f_usb30->gadget.speed][0];
	endpoint[ENDPOINT0].ep.address = 0;
	endpoint[ENDPOINT0].attributes = USB_ENDPOINT_XFER_CONTROL;
	endpoint[ENDPOINT0].interface_channel = -1;
	endpoint[ENDPOINT0].alternate_channels = 0;
	endpoint[ENDPOINT0].dma_channel = ep_dma_ch[ENDPOINT0];

	for (counter = ENDPOINT1; counter < F_USB30_MAX_EP; counter++) {
		/* initialzie endpoint configure data */
		endpoint[counter].ep.name = ep_name[counter];
		endpoint[counter].ep.address = counter | ep_dir[counter];
		endpoint[counter].attributes = ep_type[counter];
		endpoint[counter].interface_channel = ep_intf[counter];
		endpoint[counter].alternate_channels = intf_alt[endpoint[
						counter].interface_channel];
		endpoint[counter].dma_channel = ep_dma_ch[counter];
		endpoint[counter].ep.maxpacket =
			ep_fifo_size[counter][
			 f_usb30->gadget.speed == USB_SPEED_UNKNOWN ?
			 USB_SPEED_SUPER : f_usb30->gadget.speed][0];
	}

	return;
}

static void initialize_endpoint(struct f_usb30_udc *f_usb30,
	 const struct usb_ep_ops *udc_ep_ops)
{
	unsigned long counter = 0;

	initialize_endpoint_configure(f_usb30);
	f_usb30->gadget.ep0 = &f_usb30->udc_endpoint[0].ep;
	INIT_LIST_HEAD(&f_usb30->gadget.ep0->ep_list);
	INIT_LIST_HEAD(&f_usb30->gadget.ep_list);
	for (counter = ENDPOINT0; counter < F_USB30_MAX_EP; counter++) {
		struct f_usb30_ep *endpoint = &f_usb30->udc_endpoint[0];
		endpoint[counter].ep.driver_data = NULL;
		endpoint[counter].ep.ops = udc_ep_ops;
		list_add_tail(&endpoint[counter].ep.ep_list,
				 counter == ENDPOINT0 ?
				 &f_usb30->gadget.ep0->ep_list :
				 &f_usb30->gadget.ep_list);
		endpoint[counter].udc = f_usb30;
		endpoint[counter].enabled = 0;
		endpoint[counter].req = NULL;
		INIT_LIST_HEAD(&endpoint[counter].queue);
		endpoint[counter].halt = 0;
		endpoint[counter].force_halt = 0;
		endpoint[counter].dma_transfer = 0;
#if defined(CONFIG_USB_GADGET_F_USB30_USED_DMA_TRANSFER)
		if (endpoint[counter].dma_channel != -1) {
			if (endpoint[counter].ep.address & USB_DIR_IN) {
				endpoint[counter].ss_dma_fifo_addr =
				 (void __iomem *) get_ss_epinbuf_address(
				     f_usb30->device_resource->start, counter);
				endpoint[counter].hs_dma_fifo_addr =
				 (void __iomem *) get_hs_epinbuf_address(
				     f_usb30->device_resource->start, counter);
			} else {
				endpoint[counter].ss_dma_fifo_addr =
				 (void __iomem *) get_ss_epoutbuf_address(
				     f_usb30->device_resource->start, counter);
				endpoint[counter].hs_dma_fifo_addr =
				 (void __iomem *) get_hs_epoutbuf_address(
				     f_usb30->device_resource->start, counter);
			}
		} else {
			endpoint[counter].ss_dma_fifo_addr = NULL;
			endpoint[counter].hs_dma_fifo_addr = NULL;
		}
#endif
	}
}

static void configure_endpoint(struct f_usb30_udc *f_usb30)
{
	static const unsigned char transfer_type_register[] = {
		TYPE_CONTROL,		/* control transfer */
		TYPE_ISOCHRONOUS,	/* isochronout transfer */
		TYPE_BULK,		/* bulk transfer */
		TYPE_INTERRUPT,		/* interrupt transfer */
	};
	unsigned long counter;
	unsigned long alt_counter;
	void *base_addr = f_usb30->register_base_address;
	struct f_usb30_ep *endpoint = &f_usb30->udc_endpoint[0];

	if (f_usb30->gadget.speed == USB_SPEED_SUPER) {
		/* enable endpoint configuration */
		set_ss_configwren(base_addr, 1);

		/* configure endpoint x */
		for (counter = F_USB30_MAX_EP - 1;
			 counter > ENDPOINT0; counter--) {
			set_ss_epnum(base_addr, counter, counter);
			set_ss_epconf(base_addr, counter, 1);
			set_ss_intf(base_addr, counter,
					 endpoint[counter].interface_channel);
			set_ss_altmap(base_addr, counter, 1);
			set_ss_numpmode(base_addr, counter, 1);
			set_ss_diseob(base_addr, counter, 1);
			set_ss_maxburst(base_addr, counter,
					 endpoint[counter].ep.maxburst - 1);
		}

		/* disable endpoint configuration */
		set_ss_configwren(base_addr, 0);
	} else {
		/* enable endpoint configuration */
		set_hs_configwren(base_addr, 1);

		/* configure endpoint x */
		for (counter = F_USB30_MAX_EP - 1;
			 counter > ENDPOINT0; counter--) {
			for (alt_counter = 0; alt_counter <
				 endpoint[counter].alternate_channels;
				 alt_counter++) {
				set_hs_epnum(base_addr, counter, alt_counter,
					 counter);
				set_hs_io(base_addr, counter, alt_counter,
					 endpoint[counter].ep.address &
					 USB_DIR_IN ? 1 : 0);
				set_hs_type(base_addr, counter, alt_counter,
					 transfer_type_register[endpoint[
					counter].attributes]);
				set_hs_conf(base_addr, counter, alt_counter, 1);
				set_hs_intf(base_addr, counter, alt_counter,
					 endpoint[counter].interface_channel);
				set_hs_alt(base_addr, counter, alt_counter,
					 alt_counter);
				set_hs_size(base_addr, counter, alt_counter,
					ep_fifo_size[counter][
					f_usb30->gadget.speed ==
					 USB_SPEED_HIGH ? USB_SPEED_HIGH :
					 USB_SPEED_FULL][0]);
				set_hs_numtr(base_addr, counter, alt_counter);
			}
		}

		/* configure endpoint 0 */
		set_hs_epnum(base_addr, ENDPOINT0, 0, 0);
		set_hs_io(base_addr, ENDPOINT0, 0, 0);
		set_hs_type(base_addr, ENDPOINT0, 0, TYPE_CONTROL);
		set_hs_conf(base_addr, ENDPOINT0, 0, 0);
		set_hs_intf(base_addr, ENDPOINT0, 0, 0);
		set_hs_alt(base_addr, ENDPOINT0, 0, 0);
		set_hs_size(base_addr, ENDPOINT0, 0,
				 ep_fifo_size[counter][
				f_usb30->gadget.speed == USB_SPEED_HIGH ?
				 USB_SPEED_HIGH : USB_SPEED_FULL][0]);
		set_hs_numtr(base_addr, ENDPOINT0, 0);

		/* disable endpoint configuration */
		set_hs_configwren(base_addr, 0);
	}

	return;
}

static void configure_interface(struct f_usb30_udc *f_usb30)
{
	static unsigned char intf_alt[] = {
		F_USB30_ALT_INTF0,
		F_USB30_ALT_INTF1,
		F_USB30_ALT_INTF2,
		F_USB30_ALT_INTF3,
		F_USB30_ALT_INTF4,
		F_USB30_ALT_INTF5,
		F_USB30_ALT_INTF6,
		F_USB30_ALT_INTF7,
		F_USB30_ALT_INTF8,
		F_USB30_ALT_INTF9,
		F_USB30_ALT_INTF10,
		F_USB30_ALT_INTF11,
		F_USB30_ALT_INTF12,
		F_USB30_ALT_INTF13,
		F_USB30_ALT_INTF14,
		F_USB30_ALT_INTF15,
	};
	unsigned long counter;
	void *base_addr = f_usb30->register_base_address;

	if (f_usb30->gadget.speed == USB_SPEED_SUPER) {
		/* configure interface & alternate */
		set_ss_numintf(base_addr, F_USB30_MAX_INTF);
		for (counter = INTERFACE0;
				 counter < F_USB30_MAX_INTF; counter++)
			set_ss_numaintf(base_addr, counter, intf_alt[counter]);
	} else {
		/* configure interface & alternate */
		set_hs_numintf(base_addr, F_USB30_MAX_INTF);
		for (counter = INTERFACE0;
				 counter < F_USB30_MAX_INTF; counter++) {
			set_hs_numaltintf(base_addr, counter,
						intf_alt[counter]);
			set_hs_mskachgif(base_addr, counter, 0);
		}
	}

	return;
}

static void set_bus_speed(struct f_usb30_udc *f_usb30, int ss)
{
	unsigned long counter;
	void *base_addr = f_usb30->register_base_address;
	struct f_usb30_ep *endpoint = &f_usb30->udc_endpoint[0];

	/* set current bus speed */
	if (ss) {
		/* set bus speed to super */
		f_usb30->gadget.speed = USB_SPEED_SUPER;
	} else {
		switch (get_hs_crtspeed(base_addr)) {
		case CRT_SPEED_HIGH_SPEED:
			f_usb30->gadget.speed = USB_SPEED_HIGH;
			break;
		case CRT_SPEED_FULL_SPEED:
			f_usb30->gadget.speed = USB_SPEED_FULL;
			break;
		default:
			f_usb30->gadget.speed = USB_SPEED_UNKNOWN;
			return;
		}
	}

	/* set endpoint max packet */
	for (counter = ENDPOINT0; counter < F_USB30_MAX_EP; counter++)
		endpoint[counter].ep.maxpacket = ep_fifo_size[counter][
						f_usb30->gadget.speed][0];

	return;
}

static void set_device_state(struct f_usb30_udc *f_usb30,
	 unsigned char device_state)
{
	if (f_usb30->device_state == device_state)
		return;

	/* set device state */
	f_usb30->device_state_last = f_usb30->device_state;
	f_usb30->device_state = device_state;
	dev_dbg(f_usb30->gadget.dev.parent, "device state is is %u.\n",
							 device_state);

	return;
}

static unsigned short get_fifo_bytes(struct f_usb30_ep *endpoint)
{
	struct f_usb30_udc *f_usb30 = endpoint->udc;
	unsigned char ep_channel = endpoint->ep.address &
						 USB_ENDPOINT_NUMBER_MASK;
	unsigned short bytes;
	void *base_addr = f_usb30->register_base_address;

	if (f_usb30->gadget.speed == USB_SPEED_SUPER) {
		/* get current bytes in FIFO */
		if (ep_channel == ENDPOINT0)
			bytes = f_usb30->ctrl_stage ==
				 F_USB30_STAGE_IN_DATA ?
				 get_ss_sizerd0i(base_addr) :
				 get_ss_sizewr0o(base_addr);
		else
			bytes = get_ss_sizerdwr(base_addr, ep_channel);
	} else {
		/* get current bytes in FIFO */
		if (ep_channel == ENDPOINT0)
			bytes = f_usb30->ctrl_stage ==
				 F_USB30_STAGE_IN_DATA ?
				 get_hs_txrxsize0i(base_addr) :
				 get_hs_txrxsize0o(base_addr);
		else
			bytes = get_hs_txrxsize(base_addr, ep_channel);
	}

	return bytes;
}

static unsigned char is_setup_transferred(struct f_usb30_udc *f_usb30)
{
	void *base_addr = f_usb30->register_base_address;

	if (f_usb30->gadget.speed == USB_SPEED_SUPER) {
		return (get_ss_intsetup(base_addr)) ||
			 (get_ss_intsswrstb(base_addr)) ||
			 (get_ss_intsswrste(base_addr)) ||
			 (get_ss_sswrst(base_addr)) ? 1 : 0;
	} else {
		return (get_hs_intsetup(base_addr)) ||
			 (get_hs_intusbrstb(base_addr)) ||
			 (get_hs_intusbrste(base_addr)) ||
			 (get_hs_busreset(base_addr)) ? 1 : 0;
	}
}

static unsigned char set_in_transfer_pio(struct f_usb30_ep *endpoint)
{
	unsigned long counter;
	struct f_usb30_udc *f_usb30 = endpoint->udc;
	struct f_usb30_request *request = endpoint->req;
	unsigned char ep_channel = endpoint->ep.address &
						 USB_ENDPOINT_NUMBER_MASK;
	unsigned long *buffer;
	unsigned char *buffer_byte;
	unsigned char byte_access;
	unsigned long bytes;
	void *base_addr = f_usb30->register_base_address;

	/* check argument */
	if (unlikely((ep_channel != ENDPOINT0) && (!endpoint->enabled)))
		return 0;

	/* check new SETUP transfer */
	if ((ep_channel == ENDPOINT0) && (is_setup_transferred(f_usb30))) {
		dev_err(f_usb30->gadget.dev.parent,
			"new SETUP tranfer is occurred.\n");
		return 0;
	}

	if (f_usb30->gadget.speed == USB_SPEED_SUPER) {
		/* check transfer data setup */
		if (!(ep_channel == ENDPOINT0 ? get_ss_ready0i(base_addr) :
			 get_ss_ready(base_addr, ep_channel))) {
			dev_err(f_usb30->gadget.dev.parent,
				"endpoint %u is busy.\n", ep_channel);
			return 0;
		}
	} else {
		/* check transfer data setup */
		if (!(ep_channel == ENDPOINT0 ? get_hs_ready0i(base_addr) :
			 get_hs_readyi(base_addr, ep_channel))) {
			dev_err(f_usb30->gadget.dev.parent,
				 "endpoint %u is busy.\n", ep_channel);
			return 0;
		}
	}

	/* get transfer buffer */
	buffer = (unsigned long *) (request->req.buf + request->req.actual);
	prefetch(buffer);

	/* calculate this time transfer byte */
	bytes = (request->req.length - request->req.actual) <
		 endpoint->ep.maxpacket ? request->req.length -
		 request->req.actual : endpoint->ep.maxpacket;
	dev_dbg(f_usb30->gadget.dev.parent,
		"endpoint %u PIO is setup at length = %u, actual = %u, max packet = %u, this time = %u.\n",
		ep_channel, request->req.length, request->req.actual,
		endpoint->ep.maxpacket, (unsigned int) bytes);

	/* update actual byte */
	request->req.actual = bytes;

	/* write IN transfer data buffer */
	for (counter = bytes; counter >= 4; counter -= 4) {
		f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 set_ss_epinbuf(base_addr, ep_channel, *buffer) :
			 set_hs_epinbuf(base_addr, ep_channel, *buffer);
		buffer++;
	}

	for (byte_access = 0, buffer_byte = (unsigned char *) buffer; counter;
		 counter--, byte_access++) {
		f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 set_ss_epinbuf_byte(base_addr, ep_channel, byte_access,
						 *buffer_byte) :
			 set_hs_epinbuf_byte(base_addr, ep_channel, byte_access,
						 *buffer_byte);
		buffer_byte++;
	}

	if (ep_channel == ENDPOINT0) {
		/* check new SETUP transfer */
		if (!is_setup_transferred(f_usb30)) {
			/* enable IN transfer & IntReady0i interrupt */
			f_usb30->gadget.speed == USB_SPEED_SUPER ?
				 enable_ss_ready0i(base_addr) :
				 enable_hs_ready0i(base_addr);
			f_usb30->gadget.speed == USB_SPEED_SUPER ?
				 set_ss_mskready0i(base_addr, 0) :
				 set_hs_mskready0i(base_addr, 0);
		}
	} else {
		/* enable IN transfer & IntReady interrupt */
		f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 enable_ss_ready(base_addr, ep_channel) :
			 enable_hs_readyi(base_addr, ep_channel);
		f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 set_ss_mskready(base_addr, ep_channel, 0) :
			 set_hs_mskready(base_addr, ep_channel, 0);
	}

	return 1;
}

static unsigned char set_out_transfer_pio(struct f_usb30_ep *endpoint)
{
	struct f_usb30_udc *f_usb30 = endpoint->udc;
	struct f_usb30_request *request = endpoint->req;
	unsigned char ep_channel = endpoint->ep.address &
						 USB_ENDPOINT_NUMBER_MASK;
	void *base_addr = f_usb30->register_base_address;

	/* check argument */
	if (unlikely((ep_channel != ENDPOINT0) && (!endpoint->enabled)))
		return 0;

	/* check new SETUP transfer */
	if ((ep_channel == ENDPOINT0) && (is_setup_transferred(f_usb30))) {
		dev_err(f_usb30->gadget.dev.parent,
			"new SETUP tranfer is occurred.\n");
		return 0;
	}

	dev_dbg(f_usb30->gadget.dev.parent,
		"endpoint %u PIO is setup at length = %u, actual = %u, max packet = %u.\n",
		ep_channel, request->req.length, request->req.actual,
		endpoint->ep.maxpacket);
	(void) request;

	if (ep_channel == ENDPOINT0) {
		/* check new SETUP transfer */
		if (!is_setup_transferred(f_usb30))
			/* enable IntReady0o interrupt */
			f_usb30->gadget.speed == USB_SPEED_SUPER ?
				 set_ss_mskready0o(base_addr, 0) :
				 set_hs_mskready0o(base_addr, 0);
	} else {
		/* enable IntReady interrupt */
		f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 set_ss_mskready(base_addr, ep_channel, 0) :
			 set_hs_mskready(base_addr, ep_channel, 0);
	}

	return 1;
}

static void abort_in_transfer_pio(struct f_usb30_ep *endpoint,
	 unsigned char initialize)
{
	struct f_usb30_udc *f_usb30 = endpoint->udc;
	unsigned char ep_channel = endpoint->ep.address &
						 USB_ENDPOINT_NUMBER_MASK;
	void *base_addr = f_usb30->register_base_address;

	/* disable endpoint interrupt */
	if (ep_channel == ENDPOINT0) {
		f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 set_ss_mskready0i(base_addr, 1) :
			 set_hs_mskready0i(base_addr, 1);
	} else {
		f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 set_ss_mskready(base_addr, ep_channel, 1) :
			 set_hs_mskready(base_addr, ep_channel, 1);
		f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 set_ss_mskempty(base_addr, ep_channel, 1) :
			 set_hs_mskempty(base_addr, ep_channel, 1);
	}

	if (!initialize)
		return;

	/* initialize endpoint */
	initialize_endpoint_hw(endpoint, 1, 0);

	return;
}

static void abort_out_transfer_pio(struct f_usb30_ep *endpoint,
	 unsigned char initialize)
{
	struct f_usb30_udc *f_usb30 = endpoint->udc;
	unsigned char ep_channel = endpoint->ep.address &
						 USB_ENDPOINT_NUMBER_MASK;
	void *base_addr = f_usb30->register_base_address;

	/* disable endpoint interrupt */
	if (ep_channel == ENDPOINT0)
		f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 set_ss_mskready0o(base_addr, 1) :
			 set_hs_mskready0o(base_addr, 1);
	else
		f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 set_ss_mskready(base_addr, ep_channel, 1) :
			 set_hs_mskready(base_addr, ep_channel, 1);

	if (!initialize)
		return;

	/* initialize endpoint */
	initialize_endpoint_hw(endpoint, 0, 1);

	return;
}

static unsigned char end_in_transfer_pio(struct f_usb30_ep *endpoint)
{
	unsigned long counter;
	struct f_usb30_udc *f_usb30 = endpoint->udc;
	struct f_usb30_request *request = endpoint->req;
	unsigned char ep_channel = endpoint->ep.address &
						 USB_ENDPOINT_NUMBER_MASK;
	unsigned long *buffer;
	unsigned char *buffer_byte;
	unsigned long bytes;
	unsigned char byte_access;
	void *base_addr = f_usb30->register_base_address;

	/* check empty wait */
	if ((ep_channel != ENDPOINT0) &&
		 (f_usb30->gadget.speed == USB_SPEED_SUPER ?
		 (!get_ss_mskempty(base_addr, ep_channel)) :
		 (!get_hs_mskempty(base_addr, ep_channel)))) {
		/* complete request */
		abort_in_transfer_pio(endpoint, 0);
		return 1;
	}

	/* check transfer remain byte */
	if (request->req.length == request->req.actual) {
#if defined(CONFIG_USB_GADGET_F_USB30_BULK_IN_END_NOTIFY_TIMING_TO_HOST)
		/* check bulk transfer type */
		if (endpoint->attributes == USB_ENDPOINT_XFER_BULK) {
			/* clear & enable IntEmpty interrupt */
			f_usb30->gadget.speed == USB_SPEED_SUPER ?
				 set_ss_mskempty(base_addr, ep_channel, 0) :
				 set_hs_mskempty(base_addr, ep_channel, 0);
			return 0;
		}
#endif

		/* complete request */
		abort_in_transfer_pio(endpoint, 0);
		return 1;
	}

	if (ep_channel == ENDPOINT0) {
		/* check transfer data write enable */
		if (f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 (!get_ss_ready0i(base_addr)) :
			 (!get_hs_ready0i(base_addr))) {
			/* abort IN transfer */
			abort_in_transfer_pio(endpoint, 0);
			dev_err(f_usb30->gadget.dev.parent,
				"endpoint 0 IN is busy.\n");
			return 0;
		}
	} else {
		/* check transfer data write enable */
		if (f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 (!get_ss_ready(base_addr, ep_channel)) :
			 (!get_hs_readyi(base_addr, ep_channel))) {
			/* abort IN transfer */
			abort_in_transfer_pio(endpoint, 1);
			dev_err(f_usb30->gadget.dev.parent,
				"endpoint %u is busy.\n", ep_channel);
			return 0;
		}
	}

	/* get transfer buffer */
	buffer = (unsigned long *) (request->req.buf + request->req.actual);
	prefetch(buffer);

	/* calculate this time transfer byte */
	bytes = (request->req.length - request->req.actual) <
		 endpoint->ep.maxpacket ? request->req.length -
		 request->req.actual : endpoint->ep.maxpacket;
	dev_dbg(f_usb30->gadget.dev.parent,
		"endpoint %u PIO is setup at length = %u, actual = %u, max packet = %u, this time = %u.\n",
		ep_channel, request->req.length, request->req.actual,
		endpoint->ep.maxpacket, (unsigned int) bytes);

	/* update actual bytes */
	request->req.actual += bytes;

	/* write IN transfer data buffer */
	for (counter = bytes; counter >= 4; counter -= 4) {
		f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 set_ss_epinbuf(base_addr, ep_channel, *buffer) :
			 set_hs_epinbuf(base_addr, ep_channel, *buffer);
		buffer++;
	}
	for (byte_access = 0, buffer_byte = (unsigned char *) buffer; counter;
		 counter--, byte_access++) {
		f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 set_ss_epinbuf_byte(base_addr, ep_channel, byte_access,
						 *buffer_byte) :
			 set_hs_epinbuf_byte(base_addr, ep_channel, byte_access,
						 *buffer_byte);
		buffer_byte++;
	}

	if (ep_channel == ENDPOINT0) {
		/* check new SETUP transfer */
		if (!is_setup_transferred(f_usb30))
			/* enable IN transfer */
			f_usb30->gadget.speed == USB_SPEED_SUPER ?
				 enable_ss_ready0i(base_addr) :
				 enable_hs_ready0i(base_addr);
	} else {
		/* enable IN transfer */
		f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 enable_ss_ready(base_addr, ep_channel) :
			 enable_hs_readyi(base_addr, ep_channel);
	}

	return 0;
}

static unsigned char end_out_transfer_pio(struct f_usb30_ep *endpoint)
{
	unsigned long counter;
	struct f_usb30_udc *f_usb30 = endpoint->udc;
	struct f_usb30_request *request = endpoint->req;
	unsigned char ep_channel = endpoint->ep.address &
						 USB_ENDPOINT_NUMBER_MASK;
	unsigned long data;
	unsigned long *buffer;
	unsigned char *buffer_byte;
	unsigned long bytes;
	unsigned char byte_access;
	void *base_addr = f_usb30->register_base_address;

	if (ep_channel == ENDPOINT0) {
		/* check transfer data read enable */
		if (f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 (!get_ss_ready0o(base_addr)) :
			 (!get_hs_ready0o(base_addr))) {
			/* abort OUT transfer */
			abort_out_transfer_pio(endpoint, 0);
			dev_err(f_usb30->gadget.dev.parent,
				 "endpoint 0 OUT is busy.\n");

			/* check new SETUP transfer */
			if ((is_setup_transferred(f_usb30)) &&
				 (request->req.length == 0)) {
				request->req.actual = 0;
				return 1;
			}
			return 0;
		}
	} else {
		/* check transfer data read enable */
		if (f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 (!get_ss_ready(base_addr, ep_channel)) :
			 (!get_hs_readyo(base_addr, ep_channel))) {
			/* abort OUT transfer */
			abort_out_transfer_pio(endpoint, 1);
			dev_err(f_usb30->gadget.dev.parent,
				"endpoint %u is busy.\n",
							 ep_channel);
			return 0;
		}
	}

	/* get transfer buffer */
	buffer = (unsigned long *) (request->req.buf + request->req.actual);
	prefetch(buffer);

	/* get OUT transfer byte */
	if (f_usb30->gadget.speed == USB_SPEED_SUPER)
		bytes = ep_channel == ENDPOINT0 ?
			 get_ss_sizewr0o(base_addr) :
			 get_ss_sizerdwr(base_addr, ep_channel);
	else
		bytes = ep_channel == ENDPOINT0 ?
			 get_hs_txrxsize0o(base_addr) :
			 get_hs_txrxsize(base_addr, ep_channel);

	if (request->req.length < (request->req.actual + bytes))
		bytes = request->req.length - request->req.actual;
	dev_dbg(f_usb30->gadget.dev.parent,
		"endpoint %u PIO is end at length = %u, actual = %u, max packet = %u, this time = %u.\n",
		ep_channel, request->req.length, request->req.actual,
		endpoint->ep.maxpacket, (unsigned int) bytes);

	/* update actual bytes */
	request->req.actual += bytes;

	/* read OUT transfer data buffer */
	for (counter = bytes; counter >= 4; counter -= 4) {
		*buffer = f_usb30->gadget.speed == USB_SPEED_SUPER ?
				 get_ss_epoutbuf(base_addr, ep_channel) :
				 get_hs_epoutbuf(base_addr, ep_channel);
		buffer++;
	}
	if (counter) {
		data = f_usb30->gadget.speed == USB_SPEED_SUPER ?
				 get_ss_epoutbuf(base_addr, ep_channel) :
				 get_hs_epoutbuf(base_addr, ep_channel);
		for (byte_access = 0, buffer_byte = (unsigned char *) buffer;
			 counter; counter--, byte_access++) {
			*buffer_byte =
				 (unsigned char) (data >> (8 * byte_access));
			buffer_byte++;
		}
	}

	if (ep_channel == ENDPOINT0) {
		/* check new SETUP transfer */
		if (!is_setup_transferred(f_usb30))
			/* enable next OUT transfer */
			f_usb30->gadget.speed == USB_SPEED_SUPER ?
				 enable_ss_ready0o(base_addr) :
				 enable_hs_ready0o(base_addr);
	} else {
		/* enable next OUT transfer */
		f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 enable_ss_ready(base_addr, ep_channel) :
			 enable_hs_readyo(base_addr, ep_channel);
	}

	/* check transfer request complete */
	if ((request->req.length <= request->req.actual) ||
		 (bytes % endpoint->ep.maxpacket) || (!bytes)) {
		/* complete request */
		abort_out_transfer_pio(endpoint, 0);
		return 1;
	}

	return 0;
}

static unsigned char set_in_transfer(struct f_usb30_ep *endpoint)
{
	static unsigned char (*const set_in_transfer_function[]) (
		struct f_usb30_ep *) = {
			set_in_transfer_pio, set_in_transfer_dma,};
	struct f_usb30_request *request = endpoint->req;
	return set_in_transfer_function[(endpoint->dma_channel != -1) &&
		 (request->req.length) ? 1 : 0] (endpoint);
}

static unsigned char set_out_transfer(struct f_usb30_ep *endpoint)
{
	static unsigned char (*const set_out_transfer_function[]) (
		struct f_usb30_ep *) = {
			set_out_transfer_pio, set_out_transfer_dma,};
	return set_out_transfer_function[endpoint->dma_channel != -1 ? 1 : 0]
		 (endpoint);
}

static void abort_in_transfer(struct f_usb30_ep *endpoint,
	 unsigned char initialize)
{
	static void (*const abort_in_transfer_function[]) (
		struct f_usb30_ep *, unsigned char) = {
			abort_in_transfer_pio, abort_in_transfer_dma,};
	return abort_in_transfer_function[endpoint->dma_transfer ? 1 : 0]
		 (endpoint, initialize);
}

static void abort_out_transfer(struct f_usb30_ep *endpoint,
	 unsigned char initialize)
{
	static void (*const abort_out_transfer_function[]) (
		struct f_usb30_ep *, unsigned char) = {
			abort_out_transfer_pio, abort_out_transfer_dma,};
	return abort_out_transfer_function[endpoint->dma_transfer ? 1 : 0]
		 (endpoint, initialize);
}

static unsigned char end_in_transfer(struct f_usb30_ep *endpoint)
{
	static unsigned char (*const end_in_transfer_function[]) (
		struct f_usb30_ep *) = {
			end_in_transfer_pio, end_in_transfer_dma,};
	return end_in_transfer_function[endpoint->dma_transfer ? 1 : 0]
		 (endpoint);
}

static unsigned char end_out_transfer(struct f_usb30_ep *endpoint)
{
	static unsigned char (*const end_out_transfer_function[]) (
		struct f_usb30_ep *) = {end_out_transfer_pio,
							 end_out_transfer_dma,};
	return end_out_transfer_function[endpoint->dma_transfer ? 1 : 0]
		 (endpoint);
}

static void halt_transfer(struct f_usb30_ep *endpoint)
{
	unsigned long counter;
	struct f_usb30_udc *f_usb30 = endpoint->udc;
	unsigned char ep_channel = endpoint->ep.address &
						 USB_ENDPOINT_NUMBER_MASK;
	void *base_addr = f_usb30->register_base_address;

	if (ep_channel == ENDPOINT0) {
		/* check new SETUP transfer */
		if (!is_setup_transferred(f_usb30)) {
			/* halt endpoint 0 */
			if (f_usb30->gadget.speed == USB_SPEED_SUPER) {
				if (!get_ss_stalled0(base_addr))
					set_ss_reqstall0(base_addr, 1);
			} else {
				if (!get_hs_stalled0(base_addr))
					set_hs_reqstall0(base_addr, 1);
			}
		}
	} else {
		/* check isochronous endpoint */
		if (endpoint->attributes == USB_ENDPOINT_XFER_ISOC)
			return;

		/* halt endpoint x */
		if (f_usb30->gadget.speed == USB_SPEED_SUPER) {
			if (!get_ss_stalled(base_addr, ep_channel)) {
				/* initialize FIFO */
				endpoint->ep.address & USB_DIR_IN ?
					 abort_in_transfer(endpoint, 0) :
					 abort_out_transfer(endpoint, 0);
				/* wait & check endpoint buffer empty */
				for (counter = 0xFFFF; (counter &&
					(!get_ss_empty(base_addr, ep_channel)))
					; counter--)
					;
				set_ss_reqstall(base_addr, ep_channel, 1);
			}
		} else {
			if (!get_hs_stalled(base_addr, ep_channel)) {
				set_hs_reqstall(base_addr, ep_channel, 1);
				/* initialize FIFO */
				endpoint->ep.address & USB_DIR_IN ?
					 abort_in_transfer(endpoint, 1) :
					 abort_out_transfer(endpoint, 1);
			}
		}
	}

	return;
}

static void notify_transfer_request_complete(struct f_usb30_ep *endpoint,
	 int status)
{
	struct f_usb30_udc *f_usb30 = endpoint->udc;
	struct f_usb30_request *request = endpoint->req;
	unsigned char ep_channel = endpoint->ep.address &
						 USB_ENDPOINT_NUMBER_MASK;
	unsigned char halt = endpoint->halt;

	/* delete and initialize list */
	list_del_init(&request->queue);

	if (request->req.status == -EINPROGRESS)
		request->req.status = status;
	else
		status = request->req.status;

	dev_dbg(f_usb30->gadget.dev.parent,
		"endpoint %u request is completed at request = 0x%p, length = %u, actual = %u, status = %d.\n",
		ep_channel, &request->req, request->req.length,
		request->req.actual, status);

	/* notify request complete for gadget driver */
	if (request->req.complete) {
		endpoint->halt = 1;
		spin_unlock(&f_usb30->lock);
		request->req.complete(&endpoint->ep, &request->req);
		spin_lock(&f_usb30->lock);
		endpoint->halt = halt;
	}

	return;
}

static void dequeue_all_transfer_request(struct f_usb30_ep *endpoint,
	 int status)
{
	unsigned long counter;
#if !defined(CONFIG_USB_GADGET_F_USB30_DMA_USE_BOUNCE_BUF)
	struct f_usb30_udc *f_usb30 = endpoint->udc;
#endif
	struct f_usb30_request *request;

	/* dequeue all transfer request */
	for (counter = (unsigned long) -1;
		 (counter) && (!list_empty(&endpoint->queue)); counter--) {
		request = list_entry(endpoint->queue.next,
					 struct f_usb30_request, queue);
#if !defined(CONFIG_USB_GADGET_F_USB30_DMA_USE_BOUNCE_BUF)
		/* check DMA transfer buffer unmap */
		if ((endpoint->dma_transfer) &&
			 (request->req.dma != F_USB30_DMA_ADDR_INVALID)) {
			if (request->dma_buffer_map) {
				/*
				 * unmap DMA transfer buffer and
				 * synchronize DMA transfer buffer
				 */
				dma_unmap_single(f_usb30->gadget.dev.parent,
						 request->req.dma,
						 request->req.length,
						 endpoint->ep.address &
						 USB_DIR_IN ?
						 DMA_TO_DEVICE :
						 DMA_FROM_DEVICE);
				request->req.dma = F_USB30_DMA_ADDR_INVALID;
				request->dma_buffer_map = 0;
			} else {
				/* synchronize DMA transfer buffer for CPU */
				dma_sync_single_for_cpu(f_usb30->
							gadget.dev.parent,
							 request->req.dma,
							 request->req.length,
							 endpoint->ep.address &
							 USB_DIR_IN ?
							 DMA_TO_DEVICE :
							 DMA_FROM_DEVICE);
			}
		}
#endif
		endpoint->req = request;
		notify_transfer_request_complete(endpoint, status);
	}

	return;
}

void connect_ss_host(struct f_usb30_udc *f_usb30, unsigned char connect)
{
	void *base_addr = f_usb30->register_base_address;
	struct f_usb30_ep *endpoint = &f_usb30->udc_endpoint[0];
	unsigned long counter;

	if (connect) {
		/* enable SS clock */
		if (get_ssclkstp(base_addr))
			set_ssclkstpen(base_addr, 0);

		/* reset F_USB30 SS controller */
		initialize_controller(f_usb30, 1);

		set_ss_msksetup(base_addr, 0);
		set_ss_msksetconf(base_addr, 0);
		set_ss_mskpolltou0(base_addr, 0);
		set_ss_mskenterpoll(base_addr, 0);
		set_ss_mskssdisable(base_addr, 0);
		set_ss_msksuspendb(base_addr, 0);
		set_ss_msksuspende(base_addr, 0);
		set_ss_msksshrstb(base_addr, 0);
		set_ss_msksshrste(base_addr, 0);
		set_ss_msksswrstb(base_addr, 0);
		set_ss_msksswrste(base_addr, 0);
		set_ss_mskdev(base_addr, 0);

		/* change device state */
		set_device_state(f_usb30, USB_STATE_POWERED);

		/* initialize endpoint configure data */
		initialize_endpoint_configure(f_usb30);

		/* start superspeed link training */
		set_ss_connect(base_addr, 1);
	} else {
		/* mask and clear interrupt factor */
		set_ss_mskfncsusp(base_addr, 1);
		set_ss_msku2inactto(base_addr, 1);
		set_ss_msksetup(base_addr, 1);
		set_ss_msksetconf(base_addr, 1);
		set_ss_msksuspendb(base_addr, 1);
		set_ss_msksuspende(base_addr, 1);
		set_ss_mskpolltou0(base_addr, 1);
		set_ss_mskenterpoll(base_addr, 1);
		set_ss_mskssdisable(base_addr, 1);
		set_ss_mskvdtest(base_addr, 1);
		set_ss_msksshrstb(base_addr, 1);
		set_ss_msksshrste(base_addr, 1);
		set_ss_msksswrstb(base_addr, 1);
		set_ss_msksswrste(base_addr, 1);
		clear_ss_intfncsusp(base_addr);
		clear_ss_intu2inactto(base_addr);
		clear_ss_intsetup(base_addr);
		clear_ss_intsetconf(base_addr);
		clear_ss_intsuspendb(base_addr);
		clear_ss_intsuspende(base_addr);
		clear_ss_intpolltou0(base_addr);
		clear_ss_intenterpoll(base_addr);
		clear_ss_intssdisable(base_addr);
		clear_ss_intvdtest(base_addr);
		clear_ss_intsshrstb(base_addr);
		clear_ss_intsshrste(base_addr);
		clear_ss_intsswrstb(base_addr);
		clear_ss_intsswrste(base_addr);

		/* change device state */
		set_device_state(f_usb30, USB_STATE_NOTATTACHED);

		/* set bus disconnect */
		set_ss_disconnect(base_addr, 1);
		set_ss_softreset(base_addr, 1);
		set_ss_softreset(base_addr, 0);
		/* abort previous transfer */
		abort_in_transfer(&endpoint[ENDPOINT0], 0);
		abort_out_transfer(&endpoint[ENDPOINT0], 0);
		endpoint[ENDPOINT0].halt = 0;
		endpoint[ENDPOINT0].force_halt = 0;
		for (counter = ENDPOINT1;
				 counter < F_USB30_MAX_EP; counter++) {
			endpoint[counter].ep.address & USB_DIR_IN ?
				 abort_in_transfer(&endpoint[counter], 0) :
				 abort_out_transfer(&endpoint[counter], 0);
			endpoint[counter].halt = 0;
			endpoint[counter].force_halt = 0;
		}

		/* reset F_USB30 SS controller */
		initialize_controller(f_usb30, 1);

		/* dequeue all previous transfer request */
		for (counter = ENDPOINT0;
				 counter < F_USB30_MAX_EP; counter++) {
			endpoint[counter].halt = 1;
			dequeue_all_transfer_request(&endpoint[counter],
							 -ESHUTDOWN);
		}

		/* initialize endpoint list data */
		INIT_LIST_HEAD(&f_usb30->gadget.ep0->ep_list);
		INIT_LIST_HEAD(&f_usb30->gadget.ep_list);
		for (counter = ENDPOINT0;
				 counter < F_USB30_MAX_EP; counter++) {
			list_add_tail(&endpoint[counter].ep.ep_list,
					 counter == ENDPOINT0 ?
					 &f_usb30->gadget.ep0->ep_list :
					 &f_usb30->gadget.ep_list);
			INIT_LIST_HEAD(&endpoint[counter].queue);
		}

		/* initialize F_USB30 UDC device driver structure data */
		f_usb30->gadget.speed = USB_SPEED_UNKNOWN;
		f_usb30->configure_value_last = 0;
		f_usb30->ctrl_stage = F_USB30_STAGE_SETUP;
		f_usb30->ctrl_pri_dir = 1;
	}

	return;
}

void connect_hs_host(struct f_usb30_udc *f_usb30, unsigned char connect)
{
	void *base_addr = f_usb30->register_base_address;
	struct f_usb30_ep *endpoint = &f_usb30->udc_endpoint[0];
	unsigned long counter;

	if (connect) {
		/* enable SS clock */
		if (get_hsclkstp(base_addr))
			set_hsclkstpen(base_addr, 0);

		/* reset F_USB30 HS/FS controller */
		initialize_controller(f_usb30, 0);

		/* unmask interrupt factor */
		set_hs_mskerraticerr(base_addr, 0);
		set_hs_msksof(base_addr, 1);
		set_hs_mskusbrstb(base_addr, 0);
		set_hs_mskusbrste(base_addr, 0);
		set_hs_msksuspendb(base_addr, 0);
		set_hs_msksuspende(base_addr, 0);
		set_hs_msksetup(base_addr, 0);
		set_hs_msksetconf(base_addr, 0);

		/* change device state */
		set_device_state(f_usb30, USB_STATE_POWERED);

		/* initialize endpoint configure data */
		initialize_endpoint_configure(f_usb30);

		/* set normal mode */
		set_hs_physusp(base_addr, 0);

		/* pull-up D+ terminal */
		set_hs_disconnect(base_addr, 0);

	} else {
		/* mask and clear interrupt factor */
		set_hs_mskerraticerr(base_addr, 1);
		set_hs_msksof(base_addr, 1);
		set_hs_mskusbrstb(base_addr, 1);
		set_hs_mskusbrste(base_addr, 1);
		set_hs_msksuspendb(base_addr, 1);
		set_hs_msksuspende(base_addr, 1);
		set_hs_msksetup(base_addr, 1);
		set_hs_msksetconf(base_addr, 1);
		clear_hs_interraticerr(base_addr);
		clear_hs_intsof(base_addr);
		clear_hs_intusbrstb(base_addr);
		clear_hs_intusbrste(base_addr);
		clear_hs_intsuspendb(base_addr);
		clear_hs_intsuspende(base_addr);
		clear_hs_intsetup(base_addr);
		clear_hs_intsetconf(base_addr);

		/* change device state */
		set_device_state(f_usb30, USB_STATE_NOTATTACHED);

		/* pull-down D+ terminal */
		set_hs_disconnect(base_addr, 1);

		/* abort previous transfer */
		abort_in_transfer(&endpoint[ENDPOINT0], 0);
		abort_out_transfer(&endpoint[ENDPOINT0], 0);
		endpoint[ENDPOINT0].halt = 0;
		endpoint[ENDPOINT0].force_halt = 0;
		for (counter = ENDPOINT1;
				 counter < F_USB30_MAX_EP; counter++) {
			endpoint[counter].ep.address & USB_DIR_IN ?
				 abort_in_transfer(&endpoint[counter], 1) :
				 abort_out_transfer(&endpoint[counter], 1);
			endpoint[counter].halt = 0;
			endpoint[counter].force_halt = 0;
		}

		/* reset F_USB30 HS/FS controller */
		initialize_controller(f_usb30, 0);

		/* dequeue all previous transfer request */
		for (counter = ENDPOINT0;
				 counter < F_USB30_MAX_EP; counter++) {
			endpoint[counter].halt = 1;
			dequeue_all_transfer_request(&endpoint[counter],
							 -ESHUTDOWN);
		}

		/* initialize endpoint list data */
		INIT_LIST_HEAD(&f_usb30->gadget.ep0->ep_list);
		INIT_LIST_HEAD(&f_usb30->gadget.ep_list);
		for (counter = ENDPOINT0;
				 counter < F_USB30_MAX_EP; counter++) {
			list_add_tail(&endpoint[counter].ep.ep_list,
					 counter == ENDPOINT0 ?
					 &f_usb30->gadget.ep0->ep_list :
					 &f_usb30->gadget.ep_list);
			INIT_LIST_HEAD(&endpoint[counter].queue);
		}

		/* initialize F_USB30 UDC device driver structure data */
		f_usb30->gadget.speed = USB_SPEED_UNKNOWN;
		f_usb30->configure_value_last = 0;
		f_usb30->ctrl_stage = F_USB30_STAGE_SETUP;
		f_usb30->ctrl_pri_dir = 1;
	}

	return;
}

static void on_begin_warm_or_bus_reset(struct f_usb30_udc *f_usb30)
{
	struct f_usb30_ep *endpoint = &f_usb30->udc_endpoint[0];
	unsigned long counter;

	/* abort previous transfer & initialize all endpoint */
	abort_in_transfer(&endpoint[ENDPOINT0], 1);
	abort_out_transfer(&endpoint[ENDPOINT0], 1);
	endpoint[ENDPOINT0].halt = 0;
	endpoint[ENDPOINT0].force_halt = 0;
	endpoint[ENDPOINT0].dma_transfer = 0;
	for (counter = ENDPOINT1; counter < F_USB30_MAX_EP; counter++) {
		endpoint[counter].ep.address & USB_DIR_IN ?
			 abort_in_transfer(&endpoint[counter], 1) :
			 abort_out_transfer(&endpoint[counter], 1);
		endpoint[counter].halt = 0;
		endpoint[counter].force_halt = 0;
		endpoint[counter].dma_transfer = 0;
	}

	/* initialize F_USB30 DMA controller register */
	initialize_dma_controller(f_usb30);

	/* dequeue all previous transfer request */
	for (counter = ENDPOINT0; counter < F_USB30_MAX_EP; counter++)
		dequeue_all_transfer_request(&endpoint[counter], -ECONNABORTED);

	/* initialize endpoint list data */
	INIT_LIST_HEAD(&f_usb30->gadget.ep0->ep_list);
	INIT_LIST_HEAD(&f_usb30->gadget.ep_list);
	for (counter = ENDPOINT0; counter < F_USB30_MAX_EP; counter++) {
		list_add_tail(&endpoint[counter].ep.ep_list,
				 counter == ENDPOINT0 ?
				 &f_usb30->gadget.ep0->ep_list :
				 &f_usb30->gadget.ep_list);
		INIT_LIST_HEAD(&endpoint[counter].queue);
	}

#if defined(CONFIG_USB_GADGET_F_USB30_USED_BUS_RESET_DUMMY_DISCONNECT_NOTIFY)
	/* check configured */
	if (f_usb30->configure_value_last) {
		/*
		 * notify dummy disconnect to gadget driver
		 * for gadget driver re-init
		 */
		if ((f_usb30->gadget_driver) &&
			 (f_usb30->gadget_driver->disconnect)) {
			spin_unlock(&f_usb30->lock);
			f_usb30->gadget_driver->disconnect(
							&f_usb30->gadget);
			spin_lock(&f_usb30->lock);
		}
	}
#endif

	/* initialize F_USB30 UDC device driver structure data */
	f_usb30->gadget.speed = USB_SPEED_UNKNOWN;
	f_usb30->configure_value_last = 0;
	f_usb30->ctrl_stage = F_USB30_STAGE_SETUP;
	f_usb30->ctrl_pri_dir = 1;

	return;
}

static void on_end_warm_reset(struct f_usb30_udc *f_usb30)
{
	/* configure endpoint */
	configure_endpoint(f_usb30);

	/* configure interface */
	configure_interface(f_usb30);

	/* change link and device state */
	f_usb30->link_state = F_USB30_LINK_STATE_TRAINING;
	set_device_state(f_usb30, USB_STATE_POWERED);

	return;
}

static void on_end_bus_reset(struct f_usb30_udc *f_usb30)
{
	void *base_addr = f_usb30->register_base_address;

	/* check SS link state */
	if ((!get_ss_ssdiserr(base_addr)) && (f_usb30->ss_discnt < 3)
		&& (f_usb30->gadget_driver->max_speed == USB_SPEED_SUPER)) {
		/* enable interrupt factor */
		set_ss_mskenterpoll(base_addr, 0);

		/* set SS link training once more */
		set_ss_rxdetonce(base_addr, 1);
		set_ss_connect(base_addr, 1);
	}

	/* set current bus speed */
	set_bus_speed(f_usb30, 0);

	/* configure endpoint */
	configure_endpoint(f_usb30);

	/* configure interface */
	configure_interface(f_usb30);

	/* change device state */
	set_device_state(f_usb30, USB_STATE_DEFAULT);

	return;
}

static void on_begin_hot_reset(struct f_usb30_udc *f_usb30)
{
	struct f_usb30_ep *endpoint = &f_usb30->udc_endpoint[0];
	unsigned long counter;

	/* abort previous transfer & initialize all endpoint */
	abort_in_transfer(&endpoint[ENDPOINT0], 1);
	abort_out_transfer(&endpoint[ENDPOINT0], 1);
	endpoint[ENDPOINT0].halt = 0;
	endpoint[ENDPOINT0].force_halt = 0;
	for (counter = ENDPOINT1; counter < F_USB30_MAX_EP; counter++) {
		endpoint[counter].ep.address & USB_DIR_IN ?
			 abort_in_transfer(&endpoint[counter], 1) :
			 abort_out_transfer(&endpoint[counter], 1);
		endpoint[counter].halt = 0;
		endpoint[counter].force_halt = 0;
		endpoint[counter].dma_transfer = 0;
	}

	/* initialize F_USB30 DMA controller register */
	initialize_dma_controller(f_usb30);

	/* dequeue all previous transfer request */
	for (counter = ENDPOINT0; counter < F_USB30_MAX_EP; counter++)
		dequeue_all_transfer_request(&endpoint[counter], -ECONNABORTED);

	/* initialize endpoint list data */
	INIT_LIST_HEAD(&f_usb30->gadget.ep0->ep_list);
	INIT_LIST_HEAD(&f_usb30->gadget.ep_list);
	for (counter = ENDPOINT0; counter < F_USB30_MAX_EP; counter++) {
		list_add_tail(&endpoint[counter].ep.ep_list,
				 counter == ENDPOINT0 ?
				 &f_usb30->gadget.ep0->ep_list :
				 &f_usb30->gadget.ep_list);
		INIT_LIST_HEAD(&endpoint[counter].queue);
	}

	/* check configured */
	if (f_usb30->configure_value_last) {
		/*
		 * notify dummy disconnect to gadget driver
		 * for gadget driver re-init
		 */
		if ((f_usb30->gadget_driver) &&
			 (f_usb30->gadget_driver->disconnect)) {
			spin_unlock(&f_usb30->lock);
			f_usb30->gadget_driver->disconnect(
							&f_usb30->gadget);
			spin_lock(&f_usb30->lock);
		}
	}

	/* initialize F_USB30 UDC device driver structure data */
	f_usb30->configure_value_last = 0;

	return;
}

static void on_end_hot_reset(struct f_usb30_udc *f_usb30)
{
	/* current non process */
	return;
}

static void on_ss_disabled(struct f_usb30_udc *f_usb30)
{
	void *base_addr = f_usb30->register_base_address;

	/* change link state */
	f_usb30->link_state = F_USB30_LINK_STATE_DISABLED;

	/* increase disable counter */
	f_usb30->ss_discnt++;

	if (f_usb30->ss_discnt == 1) {
		/* disconnect SS host */
		connect_ss_host(f_usb30, 0);

		/* connect HS/FS host */
		connect_hs_host(f_usb30, 1);
	}

	if (f_usb30->ss_discnt == 3) {
		/* change link state */
		f_usb30->link_state = F_USB30_LINK_STATE_HSENABLED;

		/* disable SS clock */
		if (!get_ssclkstp(base_addr)) {
			set_ssclkstpen(base_addr, 1);
			set_ssclkstp(base_addr, 1);
		}

		/* disable Enterpoll interrupt factor */
		set_ss_mskenterpoll(base_addr, 1);
	}

	return;
}

static void on_enterpoll(struct f_usb30_udc *f_usb30)
{
	void *base_addr = f_usb30->register_base_address;

	/* change link state */
	f_usb30->link_state = F_USB30_LINK_STATE_TRAINING;

	/* reset HS/FS controller */
	set_hs_softreset(base_addr);

	/* mask HS/FS interrupt factor */
	set_hs_mskerraticerr(base_addr, 1);
	set_hs_msksof(base_addr, 1);
	set_hs_mskusbrstb(base_addr, 1);
	set_hs_mskusbrste(base_addr, 1);
	set_hs_msksuspendb(base_addr, 1);
	set_hs_msksuspende(base_addr, 1);
	set_hs_msksetup(base_addr, 1);
	set_hs_msksetconf(base_addr, 1);

	/* connect SS host */
	connect_ss_host(f_usb30, 1);

	return;
}

static void on_polltou0(struct f_usb30_udc *f_usb30)
{
	void *base_addr = f_usb30->register_base_address;

	/* disable interrupt factor */
	set_ss_mskenterpoll(base_addr, 1);

	/* clear disable counter */
	f_usb30->ss_discnt = 0;

	/* disable HS/FS clock */
	if (!get_hsclkstp(base_addr)) {
		set_hsclkstpen(base_addr, 1);
		set_hsclkstp(base_addr, 1);
	}

	/* set bus speed to superspeed */
	set_bus_speed(f_usb30, 1);

	/* configure endpoint */
	configure_endpoint(f_usb30);

	/* configure interface */
	configure_interface(f_usb30);

	/* enable SS endpoint0 */
	initialize_endpoint_hw(&f_usb30->udc_endpoint[0], 0, 0);

	/* change link and device state */
	f_usb30->link_state = F_USB30_LINK_STATE_SSENABLED;
	set_device_state(f_usb30, USB_STATE_DEFAULT);

	return;
}

static void on_suspend(struct f_usb30_udc *f_usb30)
{
	/* check parameter */
	if (f_usb30->gadget.speed == USB_SPEED_UNKNOWN)
		return;

	/* change device state */
	set_device_state(f_usb30, USB_STATE_SUSPENDED);

	/* notify suspend to gadget driver */
	if ((f_usb30->gadget_driver) &&
		 (f_usb30->gadget_driver->suspend)) {
		spin_unlock(&f_usb30->lock);
		f_usb30->gadget_driver->suspend(&f_usb30->gadget);
		spin_lock(&f_usb30->lock);
	}

	return;
}

static void on_wakeup(struct f_usb30_udc *f_usb30)
{
	/* check parameter */
	if (f_usb30->gadget.speed == USB_SPEED_UNKNOWN)
		return;

	/* change device state */
	set_device_state(f_usb30, f_usb30->device_state_last);

	/* notify resume to gadget driver */
	if ((f_usb30->gadget_driver) &&
		 (f_usb30->gadget_driver->resume)) {
		spin_unlock(&f_usb30->lock);
		f_usb30->gadget_driver->resume(&f_usb30->gadget);
		spin_lock(&f_usb30->lock);
	}

	return;
}

static void on_configure(struct f_usb30_udc *f_usb30)
{
	unsigned long counter;
	unsigned char cfg_value = f_usb30->gadget.speed == USB_SPEED_SUPER ?
			 get_ss_conf(f_usb30->register_base_address) :
			 get_hs_conf(f_usb30->register_base_address);
	struct f_usb30_ep *endpoint = &f_usb30->udc_endpoint[0];
	struct usb_ctrlrequest ctrlreq;

	/* check configure value change */
	if (cfg_value == f_usb30->configure_value_last)
		return;

	dev_dbg(f_usb30->gadget.dev.parent, "configure value is %u.\n",
						 cfg_value);

	f_usb30->configure_value_last = cfg_value;

	/* check configure value */
	if (cfg_value) {
		/* change device state */
		set_device_state(f_usb30, USB_STATE_CONFIGURED);
	} else {
		for (counter = ENDPOINT1; counter < F_USB30_MAX_EP; counter++)
			/* abort transfer */
			endpoint[counter].ep.address & USB_DIR_IN ?
				 abort_in_transfer(&endpoint[counter], 1) :
				 abort_out_transfer(&endpoint[counter], 1);

		/* change device state */
		set_device_state(f_usb30, USB_STATE_ADDRESS);
	}

	/* dequeue all previous transfer request */
	for (counter = ENDPOINT0; counter < F_USB30_MAX_EP; counter++)
		dequeue_all_transfer_request(&endpoint[counter], -ECONNABORTED);

	/* create SET_CONFIGURATION SETUP data */
	ctrlreq.bRequestType = USB_DIR_OUT | USB_TYPE_STANDARD |
				 USB_RECIP_DEVICE;
	ctrlreq.bRequest = USB_REQ_SET_CONFIGURATION;
	ctrlreq.wValue = cfg_value;
	ctrlreq.wIndex = 0;
	ctrlreq.wLength = 0;

	/* update control transfer stage */
	f_usb30->ctrl_stage = F_USB30_STAGE_SPECIAL;
	dev_dbg(f_usb30->gadget.dev.parent,
		"next control transfer stage is special stage.\n");

	/* notify SET_CONFIGURATION to gadget driver */
	if ((f_usb30->gadget_driver) && (f_usb30->gadget_driver->setup)) {
		spin_unlock(&f_usb30->lock);
		f_usb30->gadget_driver->setup(&f_usb30->gadget, &ctrlreq);
		spin_lock(&f_usb30->lock);
	}

	return;
}

static void on_end_control_setup_transfer(struct f_usb30_ep *endpoint)
{
	struct f_usb30_udc *f_usb30 = endpoint->udc;
	union {
		struct usb_ctrlrequest ctrlreq;
		unsigned long word[2];
	} setup;
	unsigned long bytes;
	int result;
	void *base_addr = f_usb30->register_base_address;

	/* check transfer data get disable */
	if (f_usb30->gadget.speed == USB_SPEED_SUPER) {
		if (!get_ss_ready0o(base_addr))
			return;
		/* get SS SETUP transfer byte */
		bytes = get_ss_sizewr0o(base_addr);
	} else {
		if (!get_hs_ready0o(base_addr))
			return;
		/* get HS/FS SETUP transfer byte */
		bytes = get_hs_txrxsize0o(base_addr);
	}

	/* check SETUP transfer byte */
	if (bytes != 8) {
		dev_err(f_usb30->gadget.dev.parent,
			"SETUP tranfer byte is invalid at bytes = %u.\n",
			(unsigned int) bytes);
		/* check new setup transfer */
		if (!is_setup_transferred(f_usb30))
			/* protocol stall */
			halt_transfer(endpoint);
		return;
	}

	/* clear transfer halt */
	endpoint->halt = 0;
	endpoint->force_halt = 0;

	/* dequeue all previous control transfer request */
	dequeue_all_transfer_request(endpoint, -EPROTO);

	/* read SETUP transfer data */
	if (f_usb30->gadget.speed == USB_SPEED_SUPER) {
		setup.word[0] = get_ss_epoutbuf(base_addr, ENDPOINT0);
		setup.word[1] = get_ss_epoutbuf(base_addr, ENDPOINT0);
	} else {
		setup.word[0] = get_hs_epoutbuf(base_addr, ENDPOINT0);
		setup.word[1] = get_hs_epoutbuf(base_addr, ENDPOINT0);
	}

	/* check new setup transfer */
	if (is_setup_transferred(f_usb30))
		return;

	if (f_usb30->gadget.speed == USB_SPEED_SUPER) {
		/* enable next SS OUT transfer */
		enable_ss_ready0o(base_addr);

		/* disable SS IntReady0i / IntReady0o interrupt */
		set_ss_mskready0i(base_addr, 1);
		set_ss_mskready0o(base_addr, 1);
	} else {
		/* enable HS/FS next OUT transfer */
		enable_hs_ready0o(base_addr);

		/* disable HS/FS IntReady0i / IntReady0o interrupt */
		set_hs_mskready0i(base_addr, 1);
		set_hs_mskready0o(base_addr, 1);
	}

	/* update control transfer stage */
	if (setup.ctrlreq.bRequestType & USB_DIR_IN) {
		f_usb30->ctrl_stage = F_USB30_STAGE_IN_DATA;
		dev_dbg(f_usb30->gadget.dev.parent,
			"next control transfer stage is IN data stage.\n");
	} else {
		if (setup.ctrlreq.wLength) {
			f_usb30->ctrl_stage = F_USB30_STAGE_OUT_DATA;
			dev_dbg(f_usb30->gadget.dev.parent,
			 "next control transfer stage is OUT data stage.\n");
		} else {
			f_usb30->ctrl_stage = F_USB30_STAGE_IN_STATUS;
			dev_dbg(f_usb30->gadget.dev.parent,
			 "next control transfer stage is IN status stage.\n");
		}
	}

	/* notify SETUP transfer to gadget driver */
	if ((f_usb30->gadget_driver) && (f_usb30->gadget_driver->setup)) {
		spin_unlock(&f_usb30->lock);
		result = f_usb30->gadget_driver->setup(&f_usb30->gadget,
							 &setup.ctrlreq);
		if (result < 0) {
			if (!is_setup_transferred(f_usb30))
				/* protocol stall */
				halt_transfer(endpoint);
			dev_err(f_usb30->gadget.dev.parent,
			 "SETUP transfer to gadget driver is failed at %d.\n",
			 result);
			spin_lock(&f_usb30->lock);
			return;
		}
		spin_lock(&f_usb30->lock);
	} else {
		if (!is_setup_transferred(f_usb30))
			/* protocol stall */
			halt_transfer(endpoint);
		return;
	}

	if (is_setup_transferred(f_usb30))
		return;

	/* enable status stage transfer */
	if (f_usb30->gadget.speed == USB_SPEED_SUPER) {
		if (setup.ctrlreq.bRequestType & USB_DIR_IN) {
			f_usb30->ctrl_pri_dir = 1;
			set_ss_mskready0o(base_addr, 0);
		} else {
			f_usb30->ctrl_pri_dir =
					 setup.ctrlreq.wLength ? 0 : 1;
			enable_ss_ready0i(base_addr);
			set_ss_mskready0i(base_addr, 0);
		}
	} else {
		if (setup.ctrlreq.bRequestType & USB_DIR_IN) {
			f_usb30->ctrl_pri_dir = 1;
			set_hs_mskready0o(base_addr, 0);
		} else {
			f_usb30->ctrl_pri_dir =
					 setup.ctrlreq.wLength ? 0 : 1;
			enable_hs_ready0i(base_addr);
			set_hs_mskready0i(base_addr, 0);
		}
	}

	return;
}

static void on_end_control_in_transfer(struct f_usb30_ep *endpoint)
{
	struct f_usb30_udc *f_usb30 = endpoint->udc;

	/* process control transfer stage */
	switch (f_usb30->ctrl_stage) {
	case F_USB30_STAGE_IN_DATA:
		/* check new SETUP transfer */
		if (is_setup_transferred(f_usb30))
			break;

		/* check IN transfer continue */
		if (!end_in_transfer_pio(endpoint))
			break;

		/* update control transfer stage */
		f_usb30->ctrl_stage = F_USB30_STAGE_OUT_STATUS;
		dev_dbg(f_usb30->gadget.dev.parent,
			"next control transfer stage is OUT status stage.\n");

		/* check new SETUP transfer */
		if (!is_setup_transferred(f_usb30)) {
			if (f_usb30->gadget.speed == USB_SPEED_SUPER) {
				/* setup NULL packet for USB host */
				set_ss_ctldone0(f_usb30->
						register_base_address, 1);
			} else {
				/* setup NULL packet for USB host */
				enable_hs_ready0i(f_usb30->
						register_base_address);
			}
		}
		break;
	case F_USB30_STAGE_OUT_DATA:
	case F_USB30_STAGE_IN_STATUS:
		/* update control transfer stage */
		f_usb30->ctrl_stage = F_USB30_STAGE_SETUP;
		dev_dbg(f_usb30->gadget.dev.parent,
			"next control transfer stage is SETUP stage.\n");

		/* notify request complete */
		notify_transfer_request_complete(endpoint, 0);
		break;
	case F_USB30_STAGE_OUT_STATUS:
	case F_USB30_STAGE_SPECIAL:
		/* non process */
		break;
	default:
		/* check new SETUP transfer */
		if (!is_setup_transferred(f_usb30))
			/* protocol stall */
			halt_transfer(endpoint);

		/* update control transfer stage */
		f_usb30->ctrl_stage = F_USB30_STAGE_SETUP;
		dev_dbg(f_usb30->gadget.dev.parent,
			"next control transfer stage is SETUP stage.\n");
		break;
	}

	return;
}

static void on_end_control_out_transfer(struct f_usb30_ep *endpoint)
{
	struct f_usb30_udc *f_usb30 = endpoint->udc;

	/* process control transfer stage */
	switch (f_usb30->ctrl_stage) {
	case F_USB30_STAGE_OUT_DATA:
		/* check new SETUP transfer */
		if (is_setup_transferred(f_usb30))
			break;

		/* check OUT transfer continue */
		if (!end_out_transfer_pio(endpoint))
			break;

		/* update control transfer stage */
		f_usb30->ctrl_stage = F_USB30_STAGE_IN_STATUS;
		dev_dbg(f_usb30->gadget.dev.parent,
			"next control transfer stage is IN status stage.\n");
		break;
	case F_USB30_STAGE_IN_DATA:
	case F_USB30_STAGE_OUT_STATUS:
		/* update control transfer stage */
		f_usb30->ctrl_stage = F_USB30_STAGE_SETUP;
		dev_dbg(f_usb30->gadget.dev.parent,
			"next control transfer stage is SETUP stage.\n");

		/* notify request complete */
		notify_transfer_request_complete(endpoint, 0);
		break;
	case F_USB30_STAGE_SPECIAL:
		/* non process */
		break;
	default:
		/* check new SETUP transfer */
		if (!is_setup_transferred(f_usb30))
			/* protocol stall */
			halt_transfer(endpoint);

		/* update control transfer stage */
		f_usb30->ctrl_stage = F_USB30_STAGE_SETUP;
		dev_dbg(f_usb30->gadget.dev.parent,
			"next control transfer stage is SETUP stage.\n");
		break;
	}

	return;
}

static void on_end_in_transfer(struct f_usb30_ep *endpoint)
{
	struct f_usb30_udc *f_usb30 = endpoint->udc;
	struct f_usb30_request *request;
	unsigned char ep_channel = endpoint->ep.address &
						 USB_ENDPOINT_NUMBER_MASK;

	/* check IN transfer continue */
	if (!end_in_transfer(endpoint))
		/* to be continued */
		return;

	/* notify request complete */
	notify_transfer_request_complete(endpoint, 0);

	/* check next queue empty */
	if (list_empty(&endpoint->queue))
		return;

	/* get next request */
	request = list_entry(endpoint->queue.next,
				 struct f_usb30_request, queue);
	dev_dbg(f_usb30->gadget.dev.parent,
		"endpoint %u is next queue at request = 0x%p, length = %u, buf = 0x%p.\n",
		ep_channel, request, request->req.length, request->req.buf);
	dev_dbg(f_usb30->gadget.dev.parent,
		"endpoint halt status is %u.\n", endpoint->halt);

	/* save request */
	endpoint->req = request;

	/* set endpoint x IN trasfer request */
	if (!set_in_transfer(endpoint)) {
		dev_err(f_usb30->gadget.dev.parent,
			"set_in_transfer() of endpoint %u is failed.\n",
			 ep_channel);
		dequeue_all_transfer_request(endpoint, -EL2HLT);
	}

	return;
}

static void on_end_out_transfer(struct f_usb30_ep *endpoint)
{
	struct f_usb30_udc *f_usb30 = endpoint->udc;
	struct f_usb30_request *request;
	unsigned char ep_channel = endpoint->ep.address &
						 USB_ENDPOINT_NUMBER_MASK;

	/* check OUT transfer continue */
	if (!end_out_transfer(endpoint))
		/* to be continued */
		return;

	/* notify request complete */
	notify_transfer_request_complete(endpoint, 0);

	/* check next queue empty */
	if (list_empty(&endpoint->queue))
		return;

	/* get next request */
	request = list_entry(endpoint->queue.next,
				 struct f_usb30_request, queue);
	dev_dbg(f_usb30->gadget.dev.parent,
		"endpoint %u is next queue at request = 0x%p, length = %u, buf = 0x%p.\n",
		ep_channel, request, request->req.length, request->req.buf);
	dev_dbg(f_usb30->gadget.dev.parent,
		"endpoint halt status is %u.\n", endpoint->halt);

	/* save request */
	endpoint->req = request;

	/* set endpoint x OUT transfer request */
	if (!set_out_transfer(endpoint)) {
		dev_err(f_usb30->gadget.dev.parent,
			"set_in_transfer() of endpoint %u is failed.\n",
			ep_channel);
		dequeue_all_transfer_request(endpoint, -EL2HLT);
	}

	return;
}

static void on_halt_transfer(struct f_usb30_ep *endpoint)
{
	/* set transfer halt */
	endpoint->halt = 1;

	return;
}

static void on_clear_transfer_halt(struct f_usb30_ep *endpoint)
{
	struct f_usb30_udc *f_usb30 = endpoint->udc;
	struct f_usb30_request *request;
	unsigned char ep_channel = endpoint->ep.address &
						 USB_ENDPOINT_NUMBER_MASK;

	/* check force halt */
	if (endpoint->force_halt) {
		/* re-halt transfer */
		halt_transfer(endpoint);
		return;
	}

	/* clear transfer halt */
	endpoint->halt = 0;

	/* check next queue empty */
	if (list_empty(&endpoint->queue))
		return;

	/* get next request */
	request = list_entry(endpoint->queue.next,
				 struct f_usb30_request, queue);

	/* check the got next request a request under current execution */
	if (request->req.actual)
		return;

	/* save request */
	endpoint->req = request;

	/* set endpoint x transfer request */
	if (!(endpoint->ep.address & USB_DIR_IN ? set_in_transfer(endpoint) :
		 set_out_transfer(endpoint))) {
		dev_err(f_usb30->gadget.dev.parent,
			"set_in_transfer() of endpoint %u is failed.\n",
			ep_channel);
		dequeue_all_transfer_request(endpoint, -EL2HLT);
	}

	return;
}

static void on_set_alternate(struct f_usb30_udc *f_usb30,
	 unsigned char interface_channel)
{
	unsigned long counter;
	struct usb_ctrlrequest ctrlreq;
	struct f_usb30_ep *endpoint = &f_usb30->udc_endpoint[0];

	/* setup NULL packet for USB host */
	unsigned char alternate = f_usb30->gadget.speed == USB_SPEED_SUPER ?
				 get_ss_alt(f_usb30->
				register_base_address, interface_channel) :
				 get_hs_curaif(f_usb30->
				register_base_address, interface_channel);


	/* set endpoint max packet */
	for (counter = ENDPOINT0; counter < F_USB30_MAX_EP; counter++)
		if (endpoint[counter].interface_channel ==
					 (signed char) interface_channel)
			endpoint[counter].ep.maxpacket =
				ep_fifo_size[counter][
				f_usb30->gadget.speed][alternate];

	/* dequeue all previous transfer request */
	for (counter = ENDPOINT0; counter < F_USB30_MAX_EP; counter++)
		if (endpoint[counter].interface_channel ==
					 (signed char) interface_channel)
			dequeue_all_transfer_request(&endpoint[counter],
							 -ECONNABORTED);

	if (f_usb30->gadget.speed == USB_SPEED_SUPER) {
		for (counter = ENDPOINT1;
				counter < F_USB30_MAX_EP; counter++)
			if (endpoint[counter].ep.comp_desc->bmAttributes)
				set_ss_enstream(f_usb30->
						register_base_address,
						 counter,
						 alternate ? 1 : 0);
	}

	/* create SET_INTERFACE SETUP data */
	ctrlreq.bRequestType = USB_DIR_OUT | USB_TYPE_STANDARD |
				 USB_RECIP_INTERFACE;
	ctrlreq.bRequest = USB_REQ_SET_INTERFACE;
	ctrlreq.wValue = alternate;
	ctrlreq.wIndex = interface_channel;
	ctrlreq.wLength = 0;

	/* update control transfer stage */
	f_usb30->ctrl_stage = F_USB30_STAGE_SPECIAL;
	dev_dbg(f_usb30->gadget.dev.parent,
		"next control transfer stage is special stage.\n");

	/* notify SET_INTERFACE to gadget driver */
	if ((f_usb30->gadget_driver) && (f_usb30->gadget_driver->setup)) {
		spin_unlock(&f_usb30->lock);
		f_usb30->gadget_driver->setup(&f_usb30->gadget, &ctrlreq);
		spin_lock(&f_usb30->lock);
	}
	return;
}

static void on_hungup_controller(struct f_usb30_udc *f_usb30)
{
	/* disconnect host */
	dev_dbg(f_usb30->gadget.dev.parent, "D+ terminal is pull-down.\n");
	connect_hs_host(f_usb30, 0);

	/* notify disconnect to gadget driver */
	if ((f_usb30->gadget_driver) &&
		 (f_usb30->gadget_driver->disconnect)) {
		spin_unlock(&f_usb30->lock);
		f_usb30->gadget_driver->disconnect(&f_usb30->gadget);
		spin_lock(&f_usb30->lock);
	}

	/* check bus connect */
	if (f_usb30->vbus) {
		/* connect host */
		dev_dbg(f_usb30->gadget.dev.parent,
			"D+ terminal is pull-up.\n");
		connect_hs_host(f_usb30, 1);
	}

	return;
}

#if defined(CONFIG_ARCH_MB8AC0300)
static irqreturn_t on_bus_connect(int irq, void *dev_id)
{
	struct f_usb30_udc *f_usb30;

	/* check argument */
	if (unlikely(!dev_id))
		return IRQ_NONE;

	/* get a device driver parameter */
	f_usb30 = (struct f_usb30_udc *) dev_id;

	/* F_USB30 vbus on detect external interrupt number check */
	if (unlikely(irq != f_usb30->vbus_on_irq)) {
		dev_dbg(f_usb30->gadget.dev.parent,
			"%s() is ended at non process.\n", __func__);
		return IRQ_NONE;
	}

	if (f_usb30->vbus)
		return IRQ_HANDLED;

	/* get spin lock */
	spin_lock(&f_usb30->lock);

	/* vbus on */
	f_usb30->vbus = 1;

	/* clear disable counter */
	f_usb30->ss_discnt = 0;

	/* change vbus on detect external interrupt type */
	exiu_irq_set_type(f_usb30->vbus_on_hwirq,
				 f_usb30->vbus_active_level ?
				 IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING);

	/* change link state */
	f_usb30->link_state = F_USB30_LINK_STATE_ATTACHED;

	/* connect host */
	f_usb30->gadget_driver->max_speed == USB_SPEED_SUPER ?
		 connect_ss_host(f_usb30, 1) :
		 connect_hs_host(f_usb30, 1);

	/* release spin lock */
	spin_unlock(&f_usb30->lock);
	dev_dbg(f_usb30->gadget.dev.parent, "Link training starts.\n");

	return IRQ_HANDLED;
}

static irqreturn_t on_bus_disconnect(int irq, void *dev_id)
{
	struct f_usb30_udc *f_usb30;

	/* check argument */
	if (unlikely(!dev_id))
		return IRQ_NONE;

	/* get a device driver parameter */
	f_usb30 = (struct f_usb30_udc *) dev_id;

	/* F_USB30 vbus off detect external interrupt number check */
	if (unlikely(irq != f_usb30->vbus_off_irq)) {
		dev_dbg(f_usb30->gadget.dev.parent,
			"%s() is ended at non process.\n", __func__);
		return IRQ_NONE;
	}

	/* get spin lock */
	spin_lock(&f_usb30->lock);

	/* vbus off */
	f_usb30->vbus = 0;

	/* change link state */
	f_usb30->link_state = F_USB30_LINK_STATE_NOTATTACHED;

	/* disconnect host */
	f_usb30->gadget.speed == USB_SPEED_SUPER ?
		 connect_ss_host(f_usb30, 0) :
		 connect_hs_host(f_usb30, 0);
	/* notify disconnect to gadget driver */
	if ((f_usb30->gadget_driver) &&
		 (f_usb30->gadget_driver->disconnect)) {
		spin_unlock(&f_usb30->lock);
		f_usb30->gadget_driver->disconnect(&f_usb30->gadget);
		spin_lock(&f_usb30->lock);
	}

	/* release spin lock */
	spin_unlock(&f_usb30->lock);
	dev_dbg(f_usb30->gadget.dev.parent, "vbus is off .\n");

	return IRQ_HANDLED;
}
#else
/* for other soc */
#endif

static irqreturn_t on_usb_hs_function_controller(int irq, void *dev_id)
{
	unsigned long counter;
	struct f_usb30_udc *f_usb30 = dev_id;
	void *base_addr = f_usb30->register_base_address;
	struct f_usb30_ep *endpoint = &f_usb30->udc_endpoint[0];

	/* check argument */
	if (unlikely(!dev_id))
		return IRQ_NONE;

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is started.\n",
							 __func__);

	/* F_USB30 HS/FS controller interrupt request assert check */
	if (unlikely(irq != f_usb30->hs_irq)) {
		dev_dbg(f_usb30->gadget.dev.parent,
			"%s() is ended at non process.\n", __func__);
		return IRQ_NONE;
	}

	/* get spin lock */
	spin_lock(&f_usb30->lock);

	/* PHY hung-up interrupt factor */
	if ((get_hs_interraticerr(base_addr)) &&
		 (!get_hs_mskerraticerr(base_addr))) {
		/* clear IntErraticErr interrupt factor */
		clear_hs_interraticerr(base_addr);

		dev_dbg(f_usb30->gadget.dev.parent,
			"IntErraticErr interrrupt occurred.\n");

		/* process hung-up controller */
		on_hungup_controller(f_usb30);

		spin_unlock(&f_usb30->lock);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);

		return IRQ_HANDLED;
	}

	/* bus reset begin interrupt factor */
	if ((get_hs_intusbrstb(base_addr)) && (!get_hs_mskusbrstb(base_addr))) {
		/* clear IntUsbRstb interrupt factor */
		clear_hs_intusbrstb(base_addr);

		dev_dbg(f_usb30->gadget.dev.parent,
			"IntUsbRstb interrrupt occurred.\n");

		/* process bus reset begin */
		on_begin_warm_or_bus_reset(f_usb30);

		spin_unlock(&f_usb30->lock);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);

		return IRQ_HANDLED;
	}

	/* bus reset end interrupt factor */
	if ((get_hs_intusbrste(base_addr)) && (!get_hs_mskusbrste(base_addr))) {
		/* clear IntUsbRste interrupt factor */
		clear_hs_intusbrste(base_addr);

		dev_dbg(f_usb30->gadget.dev.parent,
			"IntUsbRste interrrupt occurred.\n");

		/* process bus reset end */
		on_end_bus_reset(f_usb30);

		spin_unlock(&f_usb30->lock);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);

		return IRQ_HANDLED;
	}

	/* suspend begin interrupt factor */
	if ((get_hs_intsuspendb(base_addr)) &&
		 (!get_hs_msksuspendb(base_addr))) {
		/* clear IntSuspendb interrupt factor */
		clear_hs_intsuspendb(base_addr);

		dev_dbg(f_usb30->gadget.dev.parent,
			"IntSuspendb interrrupt occurred.\n");

		/* process bus suspend */
		on_suspend(f_usb30);

		spin_unlock(&f_usb30->lock);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);

		return IRQ_HANDLED;
	}

	/* suspend end interrupt factor */
	if ((get_hs_intsuspende(base_addr)) &&
		 (!get_hs_msksuspende(base_addr))) {
		/* clear IntSuspende interrupt factor */
		clear_hs_intsuspende(base_addr);

		dev_dbg(f_usb30->gadget.dev.parent,
			"IntSuspende interrrupt occurred.\n");

		/* process bus wakeup */
		on_wakeup(f_usb30);

		spin_unlock(&f_usb30->lock);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);

		return IRQ_HANDLED;
	}

	/* SETUP transfer interrupt factor */
	if ((get_hs_intsetup(base_addr)) && (!get_hs_msksetup(base_addr))) {
		/* clear IntSetup interrupt factor */
		clear_hs_intsetup(base_addr);

		dev_dbg(f_usb30->gadget.dev.parent,
			"IntSetup interrrupt is occurred.\n");

		/* wait & check register access enable */
		for (counter = 0xFFFF;
			 ((counter) && ((!get_hs_intready0i(base_addr)) ||
			(!get_hs_intready0o(base_addr)))); counter--)
			;
		if ((get_hs_intready0i(base_addr)) &&
			 (get_hs_intready0o(base_addr))) {
			clear_hs_intready0i(base_addr);
			clear_hs_intready0o(base_addr);
			/* process control SETUP transfer end */
			on_end_control_setup_transfer(&endpoint[ENDPOINT0]);
		} else {
			/* protocol stall */
			halt_transfer(&endpoint[ENDPOINT0]);
		}

		spin_unlock(&f_usb30->lock);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);

		return IRQ_HANDLED;
	}

	/* SET_CONFIGURATION interrupt factor */
	if ((get_hs_intsetconf(base_addr)) && (!get_hs_msksetconf(base_addr))) {
		/* clear IntSetConf interrupt factor */
		clear_hs_intsetconf(base_addr);

		dev_dbg(f_usb30->gadget.dev.parent,
			"IntSetConf interrrupt occurred.\n");

		/* process configure to host */
		on_configure(f_usb30);

		spin_unlock(&f_usb30->lock);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);

		return IRQ_HANDLED;
	}

	/* endpoint x interrupt factor */
	for (counter = ENDPOINT1; counter < F_USB30_MAX_EP; counter++) {
		if ((get_hs_intep(base_addr, counter)) &&
			 (!get_hs_mskep(base_addr, counter))) {

			dev_dbg(f_usb30->gadget.dev.parent,
				 "IntEp%u interrrupt occurred.\n",
				 (unsigned int) counter);

			if ((get_hs_intready(base_addr, counter)) &&
				 (!get_hs_mskready(base_addr, counter))) {
				/* clear IntReady x interrupt factor */
				clear_hs_intready(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					 "IntReady interrrupt occurred.\n");

				/* process IN / OUT transfer end */
				endpoint[counter].ep.address & USB_DIR_IN ?
					on_end_in_transfer(&endpoint[counter]) :
					on_end_out_transfer(&endpoint[counter]);

				spin_unlock(&f_usb30->lock);
				dev_dbg(f_usb30->gadget.dev.parent,
					 "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}

			if ((get_hs_intempty(base_addr, counter)) &&
				 (!get_hs_mskempty(base_addr, counter))) {
				/* clear IntEmpty x interrupt factor */
				clear_hs_intempty(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					 "IntEmpty interrrupt occurred.\n");

				/* process IN transfer end */
				if (endpoint[counter].ep.address & USB_DIR_IN)
					on_end_in_transfer(&endpoint[counter]);

				spin_unlock(&f_usb30->lock);
				dev_dbg(f_usb30->gadget.dev.parent,
					 "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}

			if ((get_hs_intdend(base_addr, counter)) &&
				 (!get_hs_mskdend(base_addr, counter))) {
				/* clear IntDEnd x interrupt factor */
				clear_hs_intdend(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					 "IntDEnd interrrupt occurred.\n");

				/* process IN / OUT transfer end */
				endpoint[counter].ep.address & USB_DIR_IN ?
					on_end_in_transfer(&endpoint[counter]) :
					on_end_out_transfer(&endpoint[counter]);

				spin_unlock(&f_usb30->lock);
				dev_dbg(f_usb30->gadget.dev.parent,
					 "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}

			if ((get_hs_intspdd(base_addr, counter)) &&
				 (!get_hs_mskspdd(base_addr, counter))) {
				/* clear IntSPDD x interrupt factor */
				clear_hs_intspdd(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					 "IntSPDD interrrupt occurred.\n");

				/* process OUT transfer end */
				if (!(endpoint[counter].ep.address &
								 USB_DIR_IN))
					on_end_out_transfer(&endpoint[counter]);

				spin_unlock(&f_usb30->lock);
				dev_dbg(f_usb30->gadget.dev.parent,
					 "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}

			if ((get_hs_intstalled(base_addr, counter)) &&
				 (!get_hs_mskstalled(base_addr, counter))) {
				/* clear IntStalled x interrupt factor */
				clear_hs_intstalled(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
							 "IntStalled interrrupt"
							" is occurred.\n");

				/* process transfer halt */
				on_halt_transfer(&endpoint[counter]);

				spin_unlock(&f_usb30->lock);
				dev_dbg(f_usb30->gadget.dev.parent,
					 "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}

			if ((get_hs_intclstall(base_addr, counter)) &&
				 (!get_hs_mskclstall(base_addr, counter))) {
				/* clear IntClStall x interrupt factor */
				clear_hs_intclstall(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
						 "IntClStall interrrupt "
						 "is occurred.\n");

				/* process transfer halt clear */
				on_clear_transfer_halt(&endpoint[counter]);

				spin_unlock(&f_usb30->lock);
				dev_dbg(f_usb30->gadget.dev.parent,
					 "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}

			dev_dbg(f_usb30->gadget.dev.parent,
					 "other interrrupt occurred.\n");

			/* clear endpoint x interrupt */
			clear_hs_intspr(base_addr, counter);
			clear_hs_intspdd(base_addr, counter);
			clear_hs_intready(base_addr, counter);
			clear_hs_intping(base_addr, counter);
			clear_hs_intdend(base_addr, counter);
			clear_hs_intempty(base_addr, counter);
			clear_hs_intstalled(base_addr, counter);
			clear_hs_intnack(base_addr, counter);
			clear_hs_intclstall(base_addr, counter);

			spin_unlock(&f_usb30->lock);
			dev_dbg(f_usb30->gadget.dev.parent,
				 "%s() is ended.\n", __func__);

			return IRQ_HANDLED;
		}
	}

	/* interface x interrupt factor */
	for (counter = INTERFACE0; counter < F_USB30_MAX_EP; counter++) {
		if ((get_hs_intachgif(base_addr, counter)) &&
			 (!get_hs_mskachgif(base_addr, counter))) {
			/* clear IntAChgIf x interrupt factor */
			clear_hs_intachgif(base_addr, counter);

			dev_dbg(f_usb30->gadget.dev.parent,
				"IntAChgIf%u interrrupt occurred.\n",
				(unsigned int) counter);

			/* process alternate set */
			on_set_alternate(f_usb30, counter);

			spin_unlock(&f_usb30->lock);
			dev_dbg(f_usb30->gadget.dev.parent,
					 "%s() is ended.\n", __func__);

			return IRQ_HANDLED;
		}
	}

	/* endpoint 0 interrupt factor */
	if ((get_hs_intep(base_addr, ENDPOINT0)) &&
		 (!get_hs_mskep(base_addr, ENDPOINT0))) {

		dev_dbg(f_usb30->gadget.dev.parent,
			"IntEp0 interrrupt is occurred.\n");

		/* check priority direction */
		if (f_usb30->ctrl_pri_dir) {
			/* priority is given to IN transfer */
			if ((get_hs_intready0i(base_addr)) &&
				 (!get_hs_mskready0i(base_addr))) {
				/* clear IntReady0i interrupt factor */
				clear_hs_intready0i(base_addr);

				dev_dbg(f_usb30->gadget.dev.parent,
						 "IntReady0i interrrupt "
						 "is occurred.\n");

				/* process control IN transfer end */
				on_end_control_in_transfer(
					&endpoint[ENDPOINT0]);

				spin_unlock(&f_usb30->lock);
				dev_dbg(f_usb30->gadget.dev.parent,
					 "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}

			if ((get_hs_intready0o(base_addr)) &&
				 (!get_hs_mskready0o(base_addr))) {
				/* clear IntReady0o interrupt factor */
				clear_hs_intready0o(base_addr);

				dev_dbg(f_usb30->gadget.dev.parent,
						 "IntReady0o interrrupt "
						 "is occurred.\n");

				/* process control OUT transfer end */
				on_end_control_out_transfer(
					&endpoint[ENDPOINT0]);

				spin_unlock(&f_usb30->lock);
				dev_dbg(f_usb30->gadget.dev.parent,
					 "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}
		} else {
			/* priority is given to OUT transfer */
			if ((get_hs_intready0o(base_addr)) &&
				 (!get_hs_mskready0o(base_addr))) {
				/* clear IntReady0o interrupt factor */
				clear_hs_intready0o(base_addr);

				dev_dbg(f_usb30->gadget.dev.parent,
						 "IntReady0o interrrupt "
						 "is occurred.\n");

				/* process control OUT transfer end */
				on_end_control_out_transfer(
					&endpoint[ENDPOINT0]);

				spin_unlock(&f_usb30->lock);
				dev_dbg(f_usb30->gadget.dev.parent,
					 "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}

			if ((get_hs_intready0i(base_addr)) &&
				 (!get_hs_mskready0i(base_addr))) {
				/* clear IntReady0i interrupt factor */
				clear_hs_intready0i(base_addr);

				dev_dbg(f_usb30->gadget.dev.parent,
						 "IntReady0i interrrupt "
						 "is occurred.\n");

				/* process control IN transfer end */
				on_end_control_in_transfer(
					&endpoint[ENDPOINT0]);

				spin_unlock(&f_usb30->lock);
				dev_dbg(f_usb30->gadget.dev.parent,
					 "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}
		}

		if ((get_hs_intstalled0(base_addr)) &&
			 (!get_hs_mskstalled0(base_addr))) {
			/* clear IntStalled0 interrupt factor */
			clear_hs_intstalled0(base_addr);

			dev_dbg(f_usb30->gadget.dev.parent,
				"IntStalled0 interrrupt occurred.\n");

			/* process transfer halt */
			on_halt_transfer(&endpoint[ENDPOINT0]);

			spin_unlock(&f_usb30->lock);
			dev_dbg(f_usb30->gadget.dev.parent,
				 "%s() is ended.\n", __func__);

			return IRQ_HANDLED;
		}

		if ((get_hs_intclstall0(base_addr)) &&
			 (!get_hs_mskclstall0(base_addr))) {
			/* clear IntClStall0 interrupt factor */
			clear_hs_intclstall0(base_addr);

			dev_dbg(f_usb30->gadget.dev.parent,
				"IntClStall0 interrrupt occurred.\n");

			/* process transfer halt clear */
			on_clear_transfer_halt(
				&endpoint[ENDPOINT0]);

			spin_unlock(&f_usb30->lock);
			dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n"
								, __func__);

			return IRQ_HANDLED;
		}

		dev_dbg(f_usb30->gadget.dev.parent,
			"other interrrupt is occurred.\n");

		/* clear endpoint 0 interrupt */
		clear_hs_intready0o(base_addr);
		clear_hs_intready0i(base_addr);
		clear_hs_intping0o(base_addr);
		clear_hs_intstalled0(base_addr);
		clear_hs_intnack0(base_addr);
		clear_hs_intclstall0(base_addr);

		spin_unlock(&f_usb30->lock);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);

		return IRQ_HANDLED;
	}

	spin_unlock(&f_usb30->lock);
	dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n"
						, __func__);

	return IRQ_HANDLED;
}

static irqreturn_t on_usb_ss_function_controller(int irq, void *dev_id)
{
	unsigned long counter;
	struct f_usb30_udc *f_usb30 = dev_id;
	void *base_addr = f_usb30->register_base_address;
	struct f_usb30_ep *endpoint = &f_usb30->udc_endpoint[0];

	/* check argument */
	if (unlikely(!dev_id))
		return IRQ_NONE;

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is started.\n",
							 __func__);

	/* F_USB30 SS controller interrupt request assert check */
	if (unlikely(irq != f_usb30->ss_irq)) {
		dev_dbg(f_usb30->gadget.dev.parent,
			"%s() is ended at non process.\n", __func__);
		return IRQ_NONE;
	}

	/* get spin lock */
	spin_lock(&f_usb30->lock);

	/* warm reset begin interrupt factor */
	if ((get_ss_intsswrstb(base_addr)) && (!get_ss_msksswrstb(base_addr))) {
		/* clear IntSSWRstb interrupt factor */
		clear_ss_intsswrstb(base_addr);

		dev_dbg(f_usb30->gadget.dev.parent,
			"SS IntSSWRstb interrrupt occurred.\n");

		/* process warm reset begin */
		on_begin_warm_or_bus_reset(f_usb30);

		spin_unlock(&f_usb30->lock);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);

		return IRQ_HANDLED;
	}

	/* warm reset end interrupt factor */
	if ((get_ss_intsswrste(base_addr)) && (!get_ss_msksswrste(base_addr))) {
		/* clear IntSSWRste interrupt factor */
		clear_ss_intsswrste(base_addr);

		dev_dbg(f_usb30->gadget.dev.parent,
			"SS IntSSWRste interrrupt occurred.\n");

		/* process warm reset end */
		on_end_warm_reset(f_usb30);

		spin_unlock(&f_usb30->lock);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);

		return IRQ_HANDLED;
	}

	/* hot reset begin interrupt factor */
	if ((get_ss_intsshrstb(base_addr)) && (!get_ss_msksshrstb(base_addr))) {
		/* clear IntSSHRstb interrupt factor */
		clear_ss_intsshrstb(base_addr);

		dev_dbg(f_usb30->gadget.dev.parent,
			"SS IntSSHRstb interrrupt occurred.\n");

		if (f_usb30->link_state == F_USB30_LINK_STATE_SSENABLED)
			/* process hot reset begin */
			on_begin_hot_reset(f_usb30);

		spin_unlock(&f_usb30->lock);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);

		return IRQ_HANDLED;
	}

	/* hot reset end interrupt factor */
	if ((get_ss_intsshrste(base_addr)) && (!get_ss_msksshrste(base_addr))) {
		/* clear IntSSHRste interrupt factor */
		clear_ss_intsshrste(base_addr);

		dev_dbg(f_usb30->gadget.dev.parent,
			"SS IntSSHRste interrrupt occurred.\n");

		/* process hot reset end */
		on_end_hot_reset(f_usb30);

		spin_unlock(&f_usb30->lock);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);

		return IRQ_HANDLED;
	}

	/* SS.Disabled interrupt factor */
	if ((get_ss_intssdisable(base_addr)) &&
		 (!get_ss_mskssdisable(base_addr))) {
		/* clear IntSSDisable interrupt factor */
		clear_ss_intssdisable(base_addr);

		dev_dbg(f_usb30->gadget.dev.parent,
			"SS IntSSDisable interrrupt occurred.\n");

		/* process moving to SS.Disabled state */
		on_ss_disabled(f_usb30);

		spin_unlock(&f_usb30->lock);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);

		return IRQ_HANDLED;
	}

	/* EnterPoll interrupt factor */
	if ((get_ss_intenterpoll(base_addr)) &&
		 (!get_ss_mskenterpoll(base_addr))) {
		/* clear IntEnterPoll interrupt factor */
		clear_ss_intenterpoll(base_addr);

		dev_dbg(f_usb30->gadget.dev.parent,
			"SS IntEnterPoll interrrupt occurred.\n");

		/* process moving HS/FS to Poll state */
		on_enterpoll(f_usb30);

		spin_unlock(&f_usb30->lock);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);

		return IRQ_HANDLED;
	}

	/* PolltoU0 interrupt factor */
	if ((get_ss_intpolltou0(base_addr)) &&
		 (!get_ss_mskpolltou0(base_addr))) {
		/* clear IntPolltoU0 interrupt factor */
		clear_ss_intpolltou0(base_addr);

		dev_dbg(f_usb30->gadget.dev.parent,
			"SS IntPolltoU0 interrrupt occurred.\n");

		/* process moving Poll to U0 state */
		on_polltou0(f_usb30);

		spin_unlock(&f_usb30->lock);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);

		return IRQ_HANDLED;
	}

	/* SS Suspend begin interrupt factor */
	if ((get_ss_intsuspendb(base_addr)) &&
		 (!get_ss_msksuspendb(base_addr))) {
		/* clear IntSuspendb interrupt factor */
		clear_ss_intsuspendb(base_addr);

		dev_dbg(f_usb30->gadget.dev.parent,
			"SS IntSuspendb interrrupt occurred.\n");

		/* process bus suspend */
		on_suspend(f_usb30);

		spin_unlock(&f_usb30->lock);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);

		return IRQ_HANDLED;
	}

	/* SS Suspend end interrupt factor */
	if ((get_ss_intsuspende(base_addr)) &&
		 (!get_ss_msksuspende(base_addr))) {
		/* clear IntSuspende interrupt factor */
		clear_ss_intsuspende(base_addr);

		dev_dbg(f_usb30->gadget.dev.parent,
			"SS IntSuspende interrrupt occurred.\n");

		/* process bus wakeup */
		on_wakeup(f_usb30);

		spin_unlock(&f_usb30->lock);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);

		return IRQ_HANDLED;
	}

	/* SS SETUP transfer interrupt factor */
	if ((get_ss_intsetup(base_addr)) && (!get_ss_msksetup(base_addr))) {
		/* clear SS IntSetup interrupt factor */
		clear_ss_intsetup(base_addr);

		dev_dbg(f_usb30->gadget.dev.parent,
			"SS IntSetup interrrupt is occurred.\n");

		/* wait & check register access enable */
		for (counter = 0xFFFF;
			 ((counter) && ((!get_ss_intready0i(base_addr)) ||
			(!get_ss_intready0o(base_addr)))); counter--)
			;
		if ((get_ss_intready0i(base_addr)) &&
			 (get_ss_intready0o(base_addr))) {
			clear_ss_intready0i(base_addr);
			clear_ss_intready0o(base_addr);
			/* process control SETUP transfer end */
			on_end_control_setup_transfer(&endpoint[ENDPOINT0]);
		} else {
			/* protocol stall */
			halt_transfer(&endpoint[ENDPOINT0]);
		}

		spin_unlock(&f_usb30->lock);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);

		return IRQ_HANDLED;
	}

	/* SS SET_CONFIGURATION interrupt factor */
	if ((get_ss_intsetconf(base_addr)) && (!get_ss_msksetconf(base_addr))) {
		/* clear SS IntSetConf interrupt factor */
		clear_ss_intsetconf(base_addr);

		dev_dbg(f_usb30->gadget.dev.parent,
			"SS IntSetConf interrrupt occurred.\n");

		/* process configure to host */
		on_configure(f_usb30);

		spin_unlock(&f_usb30->lock);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);

		return IRQ_HANDLED;
	}

	/* endpoint x interrupt factor */
	for (counter = ENDPOINT1; counter < F_USB30_MAX_EP; counter++) {
		if ((get_ss_intep(base_addr, counter)) &&
			 (!get_ss_mskep(base_addr, counter))) {

			dev_dbg(f_usb30->gadget.dev.parent,
				 "SS IntEp%u interrrupt occurred.\n",
				 (unsigned int) counter);

			if ((get_ss_intready(base_addr, counter)) &&
				 (!get_ss_mskready(base_addr, counter))) {
				/* clear SS IntReady x interrupt factor */
				clear_ss_intready(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					 "SS IntReady interrrupt occurred.\n");

				/* process IN / OUT transfer end */
				endpoint[counter].ep.address & USB_DIR_IN ?
					on_end_in_transfer(&endpoint[counter]) :
					on_end_out_transfer(&endpoint[counter]);

				spin_unlock(&f_usb30->lock);
				dev_dbg(f_usb30->gadget.dev.parent,
					 "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}

			if ((get_ss_intempty(base_addr, counter)) &&
				 (!get_ss_mskempty(base_addr, counter))) {
				/* clear SS IntEmpty x interrupt factor */
				clear_ss_intempty(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					 "SS IntEmpty interrrupt occurred.\n");

				/* process IN transfer end */
				if (endpoint[counter].ep.address & USB_DIR_IN)
					on_end_in_transfer(&endpoint[counter]);

				spin_unlock(&f_usb30->lock);
				dev_dbg(f_usb30->gadget.dev.parent,
					 "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}

			if ((get_ss_intdend(base_addr, counter)) &&
				 (!get_ss_mskdend(base_addr, counter))) {
				/* clear SS IntDEnd x interrupt factor */
				clear_ss_intdend(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					 "SS IntDEnd interrrupt occurred.\n");

				/* process IN / OUT transfer end */
				endpoint[counter].ep.address & USB_DIR_IN ?
					on_end_in_transfer(&endpoint[counter]) :
					on_end_out_transfer(&endpoint[counter]);

				spin_unlock(&f_usb30->lock);
				dev_dbg(f_usb30->gadget.dev.parent,
					 "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}

			if ((get_ss_intspd(base_addr, counter)) &&
				 (!get_ss_mskspd(base_addr, counter))) {
				/* clear IntSPDD x interrupt factor */
				clear_ss_intspd(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					 "SS IntSPDD interrrupt occurred.\n");

				/* process OUT transfer end */
				if (!(endpoint[counter].ep.address&USB_DIR_IN))
					on_end_out_transfer(&endpoint[counter]);

				spin_unlock(&f_usb30->lock);
				dev_dbg(f_usb30->gadget.dev.parent,
					 "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}

			if ((get_ss_intstalled(base_addr, counter)) &&
				 (!get_ss_mskstalled(base_addr, counter))) {
				/* clear IntStalled x interrupt factor */
				clear_ss_intstalled(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					"IntStalled interrrupt is occurred.\n");

				/* process transfer halt */
				on_halt_transfer(&endpoint[counter]);

				spin_unlock(&f_usb30->lock);
				dev_dbg(f_usb30->gadget.dev.parent,
					 "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}

			if ((get_ss_intclstall(base_addr, counter)) &&
				 (!get_ss_mskclstall(base_addr, counter))) {
				/* clear IntClStall x interrupt factor */
				clear_ss_intclstall(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					"IntClStall interrrupt is occurred.\n");

				/* process transfer halt clear */
				on_clear_transfer_halt(&endpoint[counter]);

				spin_unlock(&f_usb30->lock);
				dev_dbg(f_usb30->gadget.dev.parent,
					 "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}

			if ((get_ss_intsdend(base_addr, counter)) &&
				 (!get_ss_msksdend(base_addr, counter))) {
				/* clear IntSDend x interrupt factor */
				clear_ss_intsdend(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					 "SS IntSDend interrrupt occurred.\n");

				/* process OUT transfer end */
				if (!(endpoint[counter].ep.address&USB_DIR_IN))
					on_end_out_transfer(&endpoint[counter]);

				spin_unlock(&f_usb30->lock);
				dev_dbg(f_usb30->gadget.dev.parent,
					 "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}

			if ((get_ss_intclstream(base_addr, counter)) &&
				 (!get_ss_mskclstream(base_addr, counter))) {
				/* clear IntClStream x interrupt factor */
				clear_ss_intclstream(base_addr, counter);

				dev_dbg(f_usb30->gadget.dev.parent,
					"SS IntClStream interrrupt occurred\n");

				/* process OUT transfer end */
				if (!(endpoint[counter].ep.address&USB_DIR_IN))
					on_end_out_transfer(&endpoint[counter]);

				spin_unlock(&f_usb30->lock);
				dev_dbg(f_usb30->gadget.dev.parent,
					 "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}

			dev_dbg(f_usb30->gadget.dev.parent,
					 "other interrrupt occurred.\n");

			/* clear endpoint x interrupt */
			clear_ss_intspr(base_addr, counter);
			clear_ss_intspd(base_addr, counter);
			clear_ss_intdend(base_addr, counter);
			clear_ss_intsdend(base_addr, counter);
			clear_ss_intempty(base_addr, counter);
			clear_ss_intclstream(base_addr, counter);
			clear_ss_intready(base_addr, counter);
			clear_ss_intpktpnd(base_addr, counter);
			clear_ss_intstalled(base_addr, counter);
			clear_ss_intnrdy(base_addr, counter);
			clear_ss_intclstall(base_addr, counter);

			spin_unlock(&f_usb30->lock);
			dev_dbg(f_usb30->gadget.dev.parent,
				 "%s() is ended.\n", __func__);

			return IRQ_HANDLED;
		}
	}

	/* SS SET_INTERFACE interrupt factor */
	if ((get_ss_intsetintf(base_addr)) && (!get_ss_msksetintf(base_addr))) {
		/* clear SS IntSetIntf interrupt factor */
		clear_ss_intsetintf(base_addr);

		dev_dbg(f_usb30->gadget.dev.parent,
			"SS IntSetIntf interrrupt occurred.\n");

		/* process alternate set */
		on_set_alternate(f_usb30, get_ss_achg(base_addr, 0));

		spin_unlock(&f_usb30->lock);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
			 __func__);

		return IRQ_HANDLED;
	}

	/* endpoint 0 interrupt factor */
	if ((get_ss_intep(base_addr, ENDPOINT0)) &&
		 (!get_ss_mskep(base_addr, ENDPOINT0))) {

		dev_dbg(f_usb30->gadget.dev.parent,
			"IntEp0 interrrupt is occurred.\n");

		/* check priority direction */
		if (f_usb30->ctrl_pri_dir) {
			/* priority is given to IN transfer */
			if ((get_ss_intready0i(base_addr)) &&
				 (!get_ss_mskready0i(base_addr))) {
				/* clear SS IntReady0i interrupt factor */
				clear_ss_intready0i(base_addr);

				dev_dbg(f_usb30->gadget.dev.parent,
						 "SS IntReady0i interrrupt "
						 "is occurred.\n");

				/* process control IN transfer end */
				on_end_control_in_transfer(
							&endpoint[ENDPOINT0]);

				spin_unlock(&f_usb30->lock);
				dev_dbg(f_usb30->gadget.dev.parent,
					 "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}

			if ((get_ss_intready0o(base_addr)) &&
				 (!get_ss_mskready0o(base_addr))) {
				/* clear SS IntReady0o interrupt factor */
				clear_ss_intready0o(base_addr);

				dev_dbg(f_usb30->gadget.dev.parent,
						 "SS IntReady0o interrrupt "
						 "is occurred.\n");

				/* process control OUT transfer end */
				on_end_control_out_transfer(
					&endpoint[ENDPOINT0]);

				spin_unlock(&f_usb30->lock);
				dev_dbg(f_usb30->gadget.dev.parent,
					 "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}
		} else {
			/* priority is given to OUT transfer */
			if ((get_ss_intready0o(base_addr)) &&
				 (!get_ss_mskready0o(base_addr))) {
				/* clear SS IntReady0o interrupt factor */
				clear_ss_intready0o(base_addr);

				dev_dbg(f_usb30->gadget.dev.parent,
						 "SS IntReady0o interrrupt "
						 "is occurred.\n");

				/* process control OUT transfer end */
				on_end_control_out_transfer(
					&endpoint[ENDPOINT0]);

				spin_unlock(&f_usb30->lock);
				dev_dbg(f_usb30->gadget.dev.parent,
					 "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}

			if ((get_ss_intready0i(base_addr)) &&
				 (!get_ss_mskready0i(base_addr))) {
				/* clear SS IntReady0i interrupt factor */
				clear_ss_intready0i(base_addr);

				dev_dbg(f_usb30->gadget.dev.parent,
						 "SS IntReady0i interrrupt "
						 "is occurred.\n");

				/* process control IN transfer end */
				on_end_control_in_transfer(
					&endpoint[ENDPOINT0]);

				spin_unlock(&f_usb30->lock);
				dev_dbg(f_usb30->gadget.dev.parent,
					 "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}
		}

		if ((get_ss_intstalled0(base_addr)) &&
			 (!get_ss_mskstalled0(base_addr))) {
			/* clear SS IntStalled0 interrupt factor */
			clear_ss_intstalled0(base_addr);

			dev_dbg(f_usb30->gadget.dev.parent,
				"SS IntStalled0 interrrupt occurred.\n");

			/* process transfer halt */
			on_halt_transfer(&endpoint[ENDPOINT0]);

			spin_unlock(&f_usb30->lock);
			dev_dbg(f_usb30->gadget.dev.parent,
				 "%s() is ended.\n", __func__);

			return IRQ_HANDLED;
		}

		if ((get_ss_intclstall0(base_addr)) &&
			 (!get_ss_mskclstall0(base_addr))) {
			/* clear SS IntClStall0 interrupt factor */
			clear_ss_intclstall0(base_addr);

			dev_dbg(f_usb30->gadget.dev.parent,
				"SS IntClStall0 interrrupt occurred.\n");

			/* process transfer halt clear */
			on_clear_transfer_halt(&endpoint[ENDPOINT0]);

			spin_unlock(&f_usb30->lock);
			dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n"
								, __func__);

			return IRQ_HANDLED;
		}

		dev_dbg(f_usb30->gadget.dev.parent,
			"other interrrupt is occurred.\n");

		/* clear endpoint 0 interrupt */
		clear_ss_intready0o(base_addr);
		clear_ss_intready0i(base_addr);
		clear_ss_intpktpnd0(base_addr);
		clear_ss_intstalled0(base_addr);
		clear_ss_intnrdy0(base_addr);
		clear_ss_intclstall0(base_addr);

		spin_unlock(&f_usb30->lock);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);

		return IRQ_HANDLED;
	}

	spin_unlock(&f_usb30->lock);
	dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n"
						, __func__);

	return IRQ_HANDLED;
}

#define to_fusb30_udc(g)  (container_of((g), struct f_usb30_udc, gadget))

/**
 * f_usb30_udc_start - probe a gadget driver
 * @driver: the driver being registered
 *
 * When a driver is successfully registered, it will receive control requests
 * including set_configuration, which enables non-control requests.  Then
 * usb traffic follows until a disconnect is reported.  Then a host may connect
 * again, or the driver might get unbound.
 *
 * Returns 0 if no error, -EINVAL -EBUSY or other negative errno on failure
 */
static int f_usb30_udc_start(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct f_usb30_udc *f_usb30 = to_fusb30_udc(g);
	int result;

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is started.\n",
						 __func__);

	/* initialize endpoint configure data */
	initialize_endpoint_configure(f_usb30);

	/* entry gadget driver structure */
	f_usb30->gadget_driver = driver;

	/* initialize F_USB30 UDC device driver structure data */
	f_usb30->gadget.speed = USB_SPEED_UNKNOWN;
	f_usb30->device_state = USB_STATE_NOTATTACHED;
	f_usb30->device_state_last = USB_STATE_NOTATTACHED;
	f_usb30->configure_value_last = 0;
	f_usb30->ctrl_stage = F_USB30_STAGE_SETUP;
	f_usb30->ctrl_pri_dir = 1;

#if defined(CONFIG_ARCH_MB8AC0300)
	f_usb30->vbus = 0;
	/* set vbus on detect external interrupt type */
	exiu_irq_set_type(f_usb30->vbus_on_hwirq,
				 f_usb30->vbus_active_level ?
				 IRQF_TRIGGER_HIGH : IRQF_TRIGGER_LOW);
	exiu_irq_set_type(f_usb30->vbus_off_hwirq,
				 f_usb30->vbus_active_level ?
				 IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING);
	/* entry a F_USB30 vbus off IRQ */
	result = request_irq(f_usb30->vbus_off_irq, on_bus_disconnect,
				 0, "f_usb30_udc_vbus_off", f_usb30);
	if (result) {
		dev_err(f_usb30->gadget.dev.parent,
			"request_irq() for F_USB30 vbus off is failed at %d.\n",
			result);
		free_irq(f_usb30->vbus_on_irq, f_usb30);
		return result;
	}

	/* entry a F_USB30 vbus on IRQ */
	result = request_irq(f_usb30->vbus_on_irq, on_bus_connect,
				 0, "f_usb30_udc_vbus_on", f_usb30);
	if (result) {
		dev_err(f_usb30->gadget.dev.parent,
			"request_irq() for F_USB30 vbus on is failed at %d.\n",
			result);
		return result;
	}
#else
/* for other soc */
#endif
	return 0;
}

/**
 * f_usb30_stop - Unregister the gadget driver
 * @driver:	gadget driver
 *
 * Returns 0 if no error, -ENODEV -EINVAL or other negative errno on failure
 */
static int f_usb30_udc_stop(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	unsigned long flags;
	unsigned long counter;
	struct f_usb30_udc *f_usb30 = to_fusb30_udc(g);
	struct f_usb30_ep *endpoint = &f_usb30->udc_endpoint[0];

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is started.\n",
						 __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb30->lock, flags);

	/* initialize F_USB30 controller */
	initialize_controller(f_usb30, f_usb30->gadget.speed ==
				 USB_SPEED_SUPER ? 1 : 0);

#if defined(CONFIG_ARCH_MB8AC0300)
	/* disable vbus on external interrupt */
	free_irq(f_usb30->vbus_on_irq, f_usb30);
	free_irq(f_usb30->vbus_off_irq, f_usb30);
#else
/* for other soc */
#endif

	/* dequeue all previous transfer request */
	for (counter = ENDPOINT0; counter < F_USB30_MAX_EP; counter++) {
		endpoint[counter].halt = 1;
		dequeue_all_transfer_request(&endpoint[counter], -ESHUTDOWN);
	}

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb30->lock, flags);

	/* initialize endpoint list data */
	INIT_LIST_HEAD(&f_usb30->gadget.ep0->ep_list);
	INIT_LIST_HEAD(&f_usb30->gadget.ep_list);
	for (counter = ENDPOINT0; counter < F_USB30_MAX_EP; counter++) {
		list_add_tail(&endpoint[counter].ep.ep_list,
			 counter == ENDPOINT0 ?
			 &f_usb30->gadget.ep0->ep_list :
			 &f_usb30->gadget.ep_list);
		INIT_LIST_HEAD(&endpoint[counter].queue);
		endpoint[counter].halt = 0;
		endpoint[counter].force_halt = 0;
	}

	f_usb30->gadget_driver = NULL;

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
						 __func__);

	return 0;
}

/**
 * f_usb30_udc_ep_enable - Enables usb endpoint
 * @ep:		usb endpoint
 * @desc:	usb endpoint descriptor
 *
 * Returns 0 if no error, -EINVAL -ESHUTDOWN -ERANGE on failure
 */
static int f_usb30_udc_ep_enable(struct usb_ep *ep,
	 const struct usb_endpoint_descriptor *desc)
{
	struct f_usb30_udc *f_usb30;
	struct f_usb30_ep *endpoint;
	unsigned char ep_channel;
	unsigned long flags;

	/* check argument */
	if (unlikely((!ep) || (!desc)))
		return -EINVAL;

	/* get parameter */
	endpoint = container_of(ep, struct f_usb30_ep, ep);
	ep_channel = endpoint->ep.address & USB_ENDPOINT_NUMBER_MASK;
	f_usb30 = endpoint->udc;

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is started.\n",
						 __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb30->lock, flags);

	/* check parameter */
	if (unlikely((ep_channel == ENDPOINT0) || (endpoint->enabled))) {
		dev_err(f_usb30->gadget.dev.parent,
			 "endpoint parameter is invalid.\n");
		spin_unlock_irqrestore(&f_usb30->lock, flags);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);
		return -EINVAL;
	}

	/* check endpoint descriptor parameter */
	if (unlikely((desc->bDescriptorType != USB_DT_ENDPOINT) ||
		 (endpoint->ep.address != desc->bEndpointAddress) ||
		/*
		 * This additional check is necessary since ep_matches()
		 * in epautoconf.c may choose BULK-type endpoint
		 * in case of INTERRUPT transfer.
		 */
		 (desc->bmAttributes == USB_ENDPOINT_XFER_INT ?
			 ((endpoint->attributes != USB_ENDPOINT_XFER_INT) &&
			 (endpoint->attributes != USB_ENDPOINT_XFER_BULK)) :
			 desc->bmAttributes != endpoint->attributes))) {
		dev_err(f_usb30->gadget.dev.parent,
			"endpoint descriptor is invalid.\n");
		spin_unlock_irqrestore(&f_usb30->lock, flags);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);
		return -EINVAL;
	}

	/* check gadget driver parameter */
	if (unlikely(!f_usb30->gadget_driver)) {
		dev_err(f_usb30->gadget.dev.parent,
			"device state is invalid.\n");
		spin_unlock_irqrestore(&f_usb30->lock, flags);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);
		return -ESHUTDOWN;
	}

	/* check max packet size */
	if (unlikely(endpoint->ep.maxpacket <  usb_endpoint_maxp(desc))) {
		dev_err(f_usb30->gadget.dev.parent,
			"endpoint %u max packet size is invalid.\n",
			ep_channel);
		spin_unlock_irqrestore(&f_usb30->lock, flags);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);
		return -ERANGE;
	}

	/* set endpoint parameter */
	endpoint->ep.desc = desc;
	dev_dbg(f_usb30->gadget.dev.parent, "end point %u is enabled.\n",
							 ep_channel);

	/* initialize endpoint */
	initialize_endpoint_hw(endpoint, 0, 0);

	/* set endpoint parameter */
	endpoint->enabled = 1;
	endpoint->halt = 0;

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb30->lock, flags);

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
						 __func__);

	return 0;
}

/**
 * f_usb30_udc_ep_disable - Disable usb endpoint
 * @ep:	usb endpoint
 *
 * Returns 0 if no error, -EINVAL on failure
 */
static int f_usb30_udc_ep_disable(struct usb_ep *ep)
{
	struct f_usb30_udc *f_usb30;
	struct f_usb30_ep *endpoint;
	unsigned char ep_channel;
	unsigned long flags;

	/* check argument */
	if (unlikely(!ep))
		return -EINVAL;

	/* get parameter */
	endpoint = container_of(ep, struct f_usb30_ep, ep);
	ep_channel = endpoint->ep.address & USB_ENDPOINT_NUMBER_MASK;
	f_usb30 = endpoint->udc;

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is started.\n",
						 __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb30->lock, flags);

	/* check parameter */
	if (unlikely((ep_channel == ENDPOINT0) || (!endpoint->enabled))) {
		dev_err(f_usb30->gadget.dev.parent,
			"endpoint parameter is invalid.\n");
		spin_unlock_irqrestore(&f_usb30->lock, flags);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);
		return -EINVAL;
	}

	dev_dbg(f_usb30->gadget.dev.parent, "end point %u is disabled.\n",
							 ep_channel);

	/* disable DMA transfer */
	enable_dma_transfer(endpoint, 0);

	/* initialize endpoint */
	initialize_endpoint_hw(endpoint, 1, 1);

	/* set endpoint parameter */
	endpoint->enabled = 0;
	endpoint->halt = 1;

	/* dequeue all previous transfer request */
	dequeue_all_transfer_request(endpoint, -ESHUTDOWN);

	endpoint->ep.desc = NULL;

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb30->lock, flags);

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
						 __func__);

	return 0;
}

/**
 * f_usb30_udc_ep_alloc_request - Allocate usb request
 * @ep:	usb endpoint
 * @gfp_flags:	memory flag
 *
 * Returns request if no error, 0 on failure
 */
static struct usb_request *f_usb30_udc_ep_alloc_request(struct usb_ep *ep,
	 gfp_t gfp_flags)
{
	struct f_usb30_udc *f_usb30;
	struct f_usb30_request *request;
	struct f_usb30_ep *endpoint;
	unsigned char ep_channel;

	/* check argument */
	if (unlikely(!ep))
		return 0;

	/* get parameter */
	endpoint = container_of(ep, struct f_usb30_ep, ep);
	ep_channel = endpoint->ep.address & USB_ENDPOINT_NUMBER_MASK;
	f_usb30 = endpoint->udc;

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is started.\n",
						 __func__);

	/* allocate memory and zero clear memory */
	request = kzalloc(sizeof(*request), gfp_flags);
	if (!request) {
		dev_err(f_usb30->gadget.dev.parent, "kzalloc is failed.\n");
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);
		return 0;
	}
	dev_dbg(f_usb30->gadget.dev.parent,
		"endpoint %u allocate memory is 0x%p.\n", ep_channel, request);

	/* initialize list data */
	INIT_LIST_HEAD(&request->queue);
	request->req.dma = F_USB30_DMA_ADDR_INVALID;

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
						 __func__);

	return &request->req;
}

/**
 * f_usb30_udc_ep_free_request - Free usb request
 * @ep:	usb endpoint
 * @req:	usb request
 *
 * Wrapper around kfree to free request
 */
static void f_usb30_udc_ep_free_request(struct usb_ep *ep,
	 struct usb_request *req)
{
	struct f_usb30_udc *f_usb30;
	struct f_usb30_request *request;
	struct f_usb30_ep *endpoint;
	unsigned char ep_channel;

	/* check argument */
	if (unlikely((!ep) || (!req)))
		return;

	/* get parameter */
	endpoint = container_of(ep, struct f_usb30_ep, ep);
	request = container_of(req, struct f_usb30_request, req);
	ep_channel = endpoint->ep.address & USB_ENDPOINT_NUMBER_MASK;
	f_usb30 = endpoint->udc;

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is started.\n",
						 __func__);

	/* free memory */
	WARN_ON(!list_empty(&request->queue));
	kfree(request);
	dev_dbg(f_usb30->gadget.dev.parent,
		"endpoint %u free memory is 0x%p.\n", ep_channel, request);

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
						 __func__);

	return;
}

/**
 * f_usb30_udc_ep_queue - Queue a request into an IN endpoint
 * @ep:	usb endpoint
 * @req:	usb request
 * @gfp_flags:	flags
 *
 * Returns 0 if no error, -EINVAL -ESHUTDOWN -EL2HLT on failure
 */
static int f_usb30_udc_ep_queue(struct usb_ep *ep,
	 struct usb_request *req, gfp_t gfp_flags)
{
	int ret = 0;
	struct f_usb30_udc *f_usb30;
	struct f_usb30_request *request;
	struct f_usb30_ep *endpoint;
	unsigned char ep_channel;
	unsigned long flags;

	/* check argument */
	if (unlikely((!ep) || (!req)))
		return -EINVAL;

	/* get parameter */
	endpoint = container_of(ep, struct f_usb30_ep, ep);
	request = container_of(req, struct f_usb30_request, req);
	ep_channel = endpoint->ep.address & USB_ENDPOINT_NUMBER_MASK;
	f_usb30 = endpoint->udc;

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is started.\n",
						 __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb30->lock, flags);

	/* check parameter */
	if (unlikely((!req->complete) || (!req->buf) ||
		 (!list_empty(&request->queue)) ||
		 ((ep_channel != ENDPOINT0) &&
		 (!req->length) && (!req->zero)))) {
		dev_err(f_usb30->gadget.dev.parent,
			"request parameter is invalid.\n");
		ret = -EINVAL;
		goto done;
	}
	if (unlikely((ep_channel != ENDPOINT0) && (!endpoint->enabled))) {
		dev_err(f_usb30->gadget.dev.parent,
			"endpoint parameter is invalid.\n");
		ret = -EINVAL;
		goto done;
	}

	dev_dbg(f_usb30->gadget.dev.parent,
	 "endpoint %u is queue at request = 0x%p, length = %u, buf = 0x%p.\n",
	 ep_channel, request, request->req.length, request->req.buf);
	dev_dbg(f_usb30->gadget.dev.parent,
		"endpoint halt status is %u.\n", endpoint->halt);

	/* check state */
	if ((!f_usb30->gadget_driver) ||
		 (f_usb30->gadget.speed == USB_SPEED_UNKNOWN)) {
		dev_dbg(f_usb30->gadget.dev.parent, "device state is reset.\n");
		ret = -ESHUTDOWN;
		goto done;
	}

	/* initialize request parameter */
	request->req.status = -EINPROGRESS;
	request->req.actual = 0;

	/* check current queue execute */
	if ((!list_empty(&endpoint->queue)) || (endpoint->halt))
		goto queue;

	/* save request */
	endpoint->req = request;

	if (ep_channel == ENDPOINT0) {
		/* request endpoint 0 */
		switch (f_usb30->ctrl_stage) {
		case F_USB30_STAGE_IN_DATA:
			if (!set_in_transfer_pio(endpoint)) {
				dev_err(f_usb30->gadget.dev.parent,
					 "set_in_transfer_pio() of endpoint 0 "
					 "is failed.\n");
				ret = -EL2HLT;
				goto done;
			}
			break;
		case F_USB30_STAGE_OUT_DATA:
			if (!set_out_transfer_pio(endpoint)) {
				dev_err(f_usb30->gadget.dev.parent,
					 "set_out_transfer_pio() of endpoint 0 "
					 "is failed.\n");
				ret = -EL2HLT;
				goto done;
			}
			break;
		case F_USB30_STAGE_IN_STATUS:
			ret = 0;
			goto done;
		case F_USB30_STAGE_SPECIAL:
			f_usb30->ctrl_stage = F_USB30_STAGE_SETUP;
			dev_dbg(f_usb30->gadget.dev.parent,
			 "next control transfer stage is SETUP stage.\n");
			notify_transfer_request_complete(endpoint, 0);
			ret = 0;
			goto done;
		default:
			dev_dbg(f_usb30->gadget.dev.parent,
				"control transfer stage is changed at %u.\n",
				f_usb30->ctrl_stage);
			ret = -EL2HLT;
			goto done;
		}
	} else {
		/* request endpoint x */
		if (endpoint->ep.address & USB_DIR_IN) {
			if (!set_in_transfer(endpoint)) {
				dev_err(f_usb30->gadget.dev.parent,
					 "set_in_transfer() of endpoint %u is "
					 "failed.\n", ep_channel);
				ret = -EL2HLT;
				goto done;
			}
		} else {
			if (!set_out_transfer(endpoint)) {
				dev_err(f_usb30->gadget.dev.parent,
					 "set_out_transfer() of endpoint %u is "
					 "failed.\n", ep_channel);
				ret = -EL2HLT;
				goto done;
			}
		}
	}

queue:
	/* add list tail */
	list_add_tail(&request->queue, &endpoint->queue);

done:
	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb30->lock, flags);

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
						 __func__);

	return ret;
}

/**
 * f_usb30_udc_ep_dequeue - Dequeue one request
 * @ep:	usb endpoint
 * @req:	usb request
 *
 * Returns 0 if no error, -EINVAL on failure
 */
static int f_usb30_udc_ep_dequeue(struct usb_ep *ep,
	 struct usb_request *req)
{
	int ret = 0;
	struct f_usb30_udc *f_usb30;
	struct f_usb30_request *request;
	struct f_usb30_ep *endpoint;
	unsigned char ep_channel;
	unsigned long flags;

	/* check argument */
	if (unlikely((!ep) || (!req)))
		return -EINVAL;

	/* get parameter */
	endpoint = container_of(ep, struct f_usb30_ep, ep);
	request = container_of(req, struct f_usb30_request, req);
	ep_channel = endpoint->ep.address & USB_ENDPOINT_NUMBER_MASK;
	f_usb30 = endpoint->udc;

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is started.\n",
						 __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb30->lock, flags);

	/* check parameter */
	if (unlikely((ep_channel != ENDPOINT0) && (!endpoint->enabled))) {
		dev_err(f_usb30->gadget.dev.parent, "endpoint is disabled.\n");
		ret = -EINVAL;
		goto done;
	}

	/* check queue empty */
	if (!list_empty(&endpoint->queue)) {
		/* check list entry */
		list_for_each_entry(request, &endpoint->queue, queue) {
			if ((request) && (&request->req == req))
				break;
		}

		/* check dequeue request mismatch */
		if (unlikely((!request) || (&request->req != req))) {
			dev_err(f_usb30->gadget.dev.parent,
				"endpoint %u request is mismatch.\n",
				ep_channel);
			ret = -EINVAL;
			goto done;
		}
	} else {
		dev_err(f_usb30->gadget.dev.parent,
			"endpoint %u request queue is empty.\n", ep_channel);
		ret = -EINVAL;
		goto done;
	}

	/* abort request transfer */
	endpoint->req = request;
	if (ep_channel == ENDPOINT0) {
		abort_in_transfer(endpoint, 0);
		abort_out_transfer(endpoint, 0);
	} else {
		endpoint->ep.address & USB_DIR_IN ?
		 abort_in_transfer(endpoint, 0) :
		 abort_out_transfer(endpoint, 0);
	}

	notify_transfer_request_complete(endpoint, -ECONNRESET);

	/* check next queue empty */
	if (list_empty(&endpoint->queue))
		goto done;

	/* get next request */
	request = list_entry(endpoint->queue.next,
				 struct f_usb30_request, queue);

	/* check the got next request a request under current execution */
	if (request->req.actual)
		goto done;

	/* save request */
	endpoint->req = request;

	/* set endpoint x transfer request */
	if (!(endpoint->ep.address & USB_DIR_IN ? set_in_transfer(endpoint) :
		 set_out_transfer(endpoint))) {
		dev_err(f_usb30->gadget.dev.parent,
			"set_in_transfer() of endpoint %u is failed.\n",
			 ep_channel);
		dequeue_all_transfer_request(endpoint, -EL2HLT);
	}

done:
	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb30->lock, flags);

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
						 __func__);

	return ret;
}

/**
 * f_usb30_udc_ep_set_halt - Halts operations on one endpoint
 * @ep:	usb endpoint
 * @value:1--set halt  0--clear halt
 *
 * Returns 0 if no error, -EINVAL -EPROTO -EAGAIN on failure
 */
static int f_usb30_udc_ep_set_halt(struct usb_ep *ep, int value)
{
	struct f_usb30_udc *f_usb30;
	struct f_usb30_ep *endpoint;
	unsigned char ep_channel;
	unsigned long flags;

	/* check argument */
	if (unlikely((!ep) || ((value != 0) && (value != 1))))
		return -EINVAL;

	/* get parameter */
	endpoint = container_of(ep, struct f_usb30_ep, ep);
	ep_channel = endpoint->ep.address & USB_ENDPOINT_NUMBER_MASK;
	f_usb30 = endpoint->udc;

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is started.\n",
						 __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb30->lock, flags);

	/* check parameter */
	if (unlikely((ep_channel != ENDPOINT0) && (!endpoint->enabled))) {
		dev_err(f_usb30->gadget.dev.parent,
			"endpoint parameter is invalid.\n");
		spin_unlock_irqrestore(&f_usb30->lock, flags);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);
		return -EINVAL;
	}

	/* check isochronous endpoint */
	if (unlikely(endpoint->attributes == USB_ENDPOINT_XFER_ISOC)) {
		dev_err(f_usb30->gadget.dev.parent,
			"endpoint %u is isochoronous transfer.\n", ep_channel);
		spin_unlock_irqrestore(&f_usb30->lock, flags);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);
		return -EPROTO;
	}

	if (value) {
		/* check current queue execute */
		if (!list_empty(&endpoint->queue)) {
			dev_dbg(f_usb30->gadget.dev.parent,
				"endpoint %u is execute queue.\n", ep_channel);
			spin_unlock_irqrestore(&f_usb30->lock, flags);
			dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n"
								, __func__);
			return -EAGAIN;
		}

		/* set halt */
		dev_dbg(f_usb30->gadget.dev.parent,
			"endpoint %u transfer is halted.\n", ep_channel);
		halt_transfer(endpoint);
		endpoint->halt = 1;
	} else {
		/* clear halt */
		dev_dbg(f_usb30->gadget.dev.parent,
			"endpoint %u transfer halt is cleared.\n", ep_channel);
		initialize_endpoint_hw(endpoint, 1, 1);
		endpoint->halt = 0;
		endpoint->force_halt = 0;
	}

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb30->lock, flags);

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
						 __func__);

	return 0;
}

/**
 * f_usb30_udc_ep_set_wedge - Wedges operations on one endpoint
 * @ep:	usb endpoint
 *
 * Returns 0 if no error, -EINVAL -EPROTO -EAGAIN on failure
 */
static int f_usb30_udc_ep_set_wedge(struct usb_ep *ep)
{
	struct f_usb30_udc *f_usb30;
	struct f_usb30_ep *endpoint;
	unsigned char ep_channel;
	unsigned long flags;

	/* check argument */
	if (unlikely(!ep))
		return -EINVAL;

	/* get parameter */
	endpoint = container_of(ep, struct f_usb30_ep, ep);
	ep_channel = endpoint->ep.address & USB_ENDPOINT_NUMBER_MASK;
	f_usb30 = endpoint->udc;

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is started.\n",
						 __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb30->lock, flags);

	/* check parameter */
	if (unlikely((ep_channel != ENDPOINT0) && (!endpoint->enabled))) {
		dev_err(f_usb30->gadget.dev.parent,
			"endpoint parameter is invalid.\n");
		spin_unlock_irqrestore(&f_usb30->lock, flags);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);
		return -EINVAL;
	}

	/* check isochronous endpoint */
	if (unlikely(endpoint->attributes == USB_ENDPOINT_XFER_ISOC)) {
		dev_err(f_usb30->gadget.dev.parent,
			"endpoint %u is isochoronous transfer.\n", ep_channel);
		spin_unlock_irqrestore(&f_usb30->lock, flags);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);
		return -EPROTO;
	}

	/* check current queue execute */
	if (!list_empty(&endpoint->queue)) {
		dev_dbg(f_usb30->gadget.dev.parent,
			"endpoint %u queue is execute.\n", ep_channel);
		spin_unlock_irqrestore(&f_usb30->lock, flags);
		dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
							 __func__);
		return -EAGAIN;
	}

	/* set force halt */
	dev_dbg(f_usb30->gadget.dev.parent,
		"endpoint %u transfer is halted.\n", ep_channel);
	halt_transfer(endpoint);
	endpoint->halt = 1;
	endpoint->force_halt = 1;

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb30->lock, flags);

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
						 __func__);

	return 0;
}

/**
 * f_usb30_udc_ep_fifo_status - Get how many bytes in physical endpoint
 * @ep:	usb endpoint
 *
 * Returns number of bytes in OUT fifos. -EINVAL on failure
 */
static int f_usb30_udc_ep_fifo_status(struct usb_ep *ep)
{
	struct f_usb30_udc *f_usb30;
	struct f_usb30_ep *endpoint;
	unsigned char ep_channel;
	unsigned short bytes;
	unsigned long flags;

	/* check argument */
	if (unlikely(!ep))
		return -EINVAL;

	/* get parameter */
	endpoint = container_of(ep, struct f_usb30_ep, ep);
	ep_channel = endpoint->ep.address & USB_ENDPOINT_NUMBER_MASK;
	f_usb30 = endpoint->udc;

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is started.\n",
						 __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb30->lock, flags);

	/* get current bytes in FIFO */
	bytes = get_fifo_bytes(endpoint);
	dev_dbg(f_usb30->gadget.dev.parent, "bytes in FIFO is %u.\n", bytes);

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb30->lock, flags);

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
						 __func__);

	return (int) bytes;
}

/**
 * f_usb30_udc_ep_fifo_flush - Flushes one endpoint
 * @ep:	usb endpoint
 *
 * Discards all data in one endpoint(IN or OUT).
 */
static void f_usb30_udc_ep_fifo_flush(struct usb_ep *ep)
{
	struct f_usb30_udc *f_usb30;
	struct f_usb30_ep *endpoint;
	unsigned char ep_channel;
	unsigned long flags;

	/* check argument */
	if (unlikely(!ep))
		return;

	/* get parameter */
	endpoint = container_of(ep, struct f_usb30_ep, ep);
	ep_channel = endpoint->ep.address & USB_ENDPOINT_NUMBER_MASK;
	f_usb30 = endpoint->udc;

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is started.\n",
						 __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb30->lock, flags);

	/* initialize endpoint FIFO */
	dev_dbg(f_usb30->gadget.dev.parent, "endpoint %u FIFO is flush.\n",
						 ep_channel);
	initialize_endpoint_hw(endpoint, 1, 1);

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb30->lock, flags);

	dev_dbg(f_usb30->gadget.dev.parent, "%s() is ended.\n",
						 __func__);

	return;
}

static struct usb_ep_ops f_usb30_udc_ep_ops = {
	.enable = f_usb30_udc_ep_enable,
	.disable = f_usb30_udc_ep_disable,
	.alloc_request = f_usb30_udc_ep_alloc_request,
	.free_request = f_usb30_udc_ep_free_request,
	.queue = f_usb30_udc_ep_queue,
	.dequeue = f_usb30_udc_ep_dequeue,
	.set_halt = f_usb30_udc_ep_set_halt,
	.set_wedge = f_usb30_udc_ep_set_wedge,
	.fifo_status = f_usb30_udc_ep_fifo_status,
	.fifo_flush = f_usb30_udc_ep_fifo_flush,
};

static struct usb_gadget_ops f_usb30_udc_gadget_ops = {
	.udc_start	= f_usb30_udc_start,
	.udc_stop	= f_usb30_udc_stop,
};

/**
 * f_usb30_udc_probe - Probes the udc device
 * @pdev:	platform device
 *
 * Perform basic init : allocates memory, initialize, entry IRQ
 *
 * Returns 0 if no error, -EINVAL -ENODEV -EBUSY -ENOMEM -EIO or other negative
 * errno on failure
 */
static int __init f_usb30_udc_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct f_usb30_udc *f_usb30;
	struct resource *device_resource;
	resource_size_t device_resource_size;
	struct resource *f_usb30_resource;
	void __iomem *register_base_address;
	int hs_irq;
	int ss_irq;
	struct clk *clk;
#if defined(CONFIG_USB_GADGET_F_USB30_USED_DMA_TRANSFER)
	unsigned long counter;
	int dmac_in_irq;
	int dmac_out_irq;
#endif
	int result;

	/* check argument */
	if (unlikely(!pdev)) {
		ret = -EINVAL;
		goto done;
	}

	dev_dbg(&pdev->dev, "%s() is started.\n", __func__);

	/* get clock for F_USB30 device */
	clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_dbg(&pdev->dev, "clock not found.\n");
		ret = PTR_ERR(clk);
		goto done;
	}
	clk_prepare_enable(clk);

	/* get a resource for a F_USB30 device */
	device_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!device_resource) {
		dev_err(&pdev->dev, "platform_get_resource() is failed.\n");
		ret = -ENODEV;
		goto err_res;
	}
	device_resource_size = device_resource->end -
					 device_resource->start + 1;

	/* reserve a F_USB30 device resource region */
	f_usb30_resource = request_mem_region(device_resource->start,
				 device_resource_size, "f_usb30_udc");
	if (!f_usb30_resource) {
		dev_err(&pdev->dev, "request_mem_region() is failed.\n");
		ret = -EBUSY;
		goto err_res;
	}

	/* get a register base address for a F_USB30 device */
	register_base_address = remap_iomem_region(device_resource->start,
						   device_resource_size);
	if (!register_base_address) {
		dev_err(&pdev->dev, "remap_iomem_region() is failed.\n");
		ret = -ENODEV;
		goto err_mem;
	}

	/* get an IRQ for a F_USB30 HS/FS device */
	hs_irq = platform_get_irq(pdev, 0);
	if (hs_irq < 0) {
		dev_err(&pdev->dev,
		 "platform_get_irq() for F_USB30 HS/FS is failed at %d.\n",
		 hs_irq);
		ret = -ENODEV;
		goto err_map;
	}

	/* get an IRQ for a F_USB30 SS device */
	ss_irq = platform_get_irq(pdev, 1);
	if (ss_irq < 0) {
		dev_err(&pdev->dev,
			"platform_get_irq() for F_USB30 SS is failed at %d.\n",
			ss_irq);
		ret = -ENODEV;
		goto err_map;
	}

#if defined(CONFIG_USB_GADGET_F_USB30_USED_DMA_TRANSFER)
	/* get an IRQ for a F_USB30 DMAC in */
	dmac_in_irq = platform_get_irq(pdev, 2);
	if (dmac_in_irq < 0) {
		dev_err(&pdev->dev,
		 "platform_get_irq() for F_USB30 DMAC in is failed at %d.\n",
		 dmac_in_irq);
		ret = -ENODEV;
		goto err_map;
	}

	/* get an IRQ for a F_USB30 DMAC out */
	dmac_out_irq = platform_get_irq(pdev, 3);
	if (dmac_out_irq < 0) {
		dev_err(&pdev->dev,
		 "platform_get_irq() for F_USB30 DMAC out is failed at %d.\n",
		 dmac_out_irq);
		ret = -ENODEV;
		goto err_map;
	}
#endif

	/* allocate F_USB30 UDC device driver structure data memory */
	f_usb30 = kzalloc(sizeof(*f_usb30), GFP_KERNEL);
	if (!f_usb30) {
		dev_err(&pdev->dev, "kzalloc() is failed.\n");
		ret = -ENOMEM;
		goto err_map;
	}

	/* initialize a F_USB30 UDC device driver structure */
	f_usb30->gadget.ops = &f_usb30_udc_gadget_ops;
	f_usb30->gadget.speed = USB_SPEED_UNKNOWN;
	f_usb30->gadget.max_speed = USB_SPEED_SUPER;
	f_usb30->gadget.name = "f_usb30_udc";
	dev_set_name(&f_usb30->gadget.dev, "gadget");
	f_usb30->gadget_driver = NULL;
	spin_lock_init(&f_usb30->lock);
	f_usb30->device_resource = device_resource;
	f_usb30->register_base_address = register_base_address;
	f_usb30->hs_irq = hs_irq;
	f_usb30->ss_irq = ss_irq;
	f_usb30->clk = clk;
	initialize_endpoint(f_usb30, &f_usb30_udc_ep_ops);
#if defined(CONFIG_ARCH_MB8AC0300)
	f_usb30->vbus = 0;
	f_usb30->vbus_active_level = F_USB30_VBUS_ACTIVE_LEVEL;
	of_property_read_u32(pdev->dev.of_node, "vbus_on_hwirq",
		 &f_usb30->vbus_on_hwirq);
	of_property_read_u32(pdev->dev.of_node, "vbus_off_hwirq",
		 &f_usb30->vbus_off_hwirq);
	f_usb30->vbus_on_irq = platform_get_irq(pdev, 4);
	f_usb30->vbus_off_irq = platform_get_irq(pdev, 5);
#else
/* for other soc */
#endif
	f_usb30->link_state = F_USB30_LINK_STATE_NOTATTACHED;
	f_usb30->device_state = USB_STATE_NOTATTACHED;
	f_usb30->device_state_last = USB_STATE_NOTATTACHED;
	f_usb30->configure_value_last = 0;
	f_usb30->ctrl_stage = F_USB30_STAGE_SETUP;
	f_usb30->ctrl_pri_dir = 1;
#if defined(CONFIG_USB_GADGET_F_USB30_USED_DMA_TRANSFER)
	f_usb30->dmac_in_irq = dmac_in_irq;
	f_usb30->dmac_out_irq = dmac_out_irq;
	for (counter = 0; counter < F_USB30_MAX_DMAC; counter++)
		f_usb30->dma_ep[counter] = 0;
#endif

	f_usb30_data = f_usb30;

	/* setup the private data of a device driver */
	platform_set_drvdata(pdev, f_usb30);

	/* initialize F_USB30 controller */
	initialize_controller(f_usb30, 1);

	/* entry a F_USB30 SS device IRQ */
	result = request_irq(f_usb30->ss_irq, on_usb_ss_function_controller,
				 0, "f_usb30_udc_ss", f_usb30);
	if (result) {
		dev_err(&pdev->dev,
			"request_irq() for F_USB30 SS is failed at %d.\n",
			result);
		ret = result;
		goto err_ssirq;
	}

	/* entry a F_USB30 HS/FS device IRQ */
	result = request_irq(f_usb30->hs_irq, on_usb_hs_function_controller,
				 0, "f_usb30_udc_hs", f_usb30);
	if (result) {
		dev_err(&pdev->dev,
			"request_irq() for F_USB30 HS/FS is failed at %d.\n",
			result);
		ret = result;
		goto err_hsirq;
	}

#if defined(CONFIG_USB_GADGET_F_USB30_USED_DMA_TRANSFER)
	/* entry a F_USB30 DMAC in IRQ */
	result = request_irq(f_usb30->dmac_in_irq, on_dma_transfer_contoller,
				 0, "f_usb30_dmac_in_udc", f_usb30);
	if (result) {
		dev_err(&pdev->dev,
			"request_irq() for F_USB30 DMAC IN is failed at %d.\n",
			result);
		ret = result;
		goto err_dmain;
	}
	/* entry a F_USB30 DMAC out IRQ */
	result = request_irq(f_usb30->dmac_out_irq, on_dma_transfer_contoller,
				 0, "f_usb30_dmac_out_udc", f_usb30);
	if (result) {
		dev_err(&pdev->dev,
		 "request_irq() for F_USB30 DMAC OUT is failed at %d.\n",
		 result);
		ret = result;
		goto err_dmaout;
	}
#if defined(CONFIG_USB_GADGET_F_USB30_DMA_USE_BOUNCE_BUF)
	/* allocate noncachable buffer for the F_USB20HDC DMAC device */
	for (counter = F_USB30_IN_DMAC; counter < F_USB30_MAX_DMAC; counter++) {
		f_usb30->buffer[counter] = dma_alloc_coherent(NULL,
						F_USB30_DMAC_TRANS_MAX_BYTES,
						&f_usb30->dma_buffer[counter],
						GFP_KERNEL);
		if (f_usb30->buffer[counter] == NULL) {
			dev_err(&pdev->dev,
			 "%s():DMA channel %ld dma_alloc_coherent() failed.\n",
			 __func__, counter);
			result = -ENOMEM;
			goto err_non;
		}
	}
#endif
#endif

	result = usb_add_gadget_udc(&pdev->dev, &f_usb30->gadget);
	if (result) {
		ret = result;
		dev_err(&pdev->dev, "usb_add_gadget_udc failed!\n");
		goto err_non;
	}

	/* driver registering log output */
	dev_info(&pdev->dev,
		 "F_USB30 UDC driver (version %s) is registered.\n",
		 F_USB30_DRIVER_VERSION);

	dev_dbg(&pdev->dev, "%s() is ended.\n", __func__);

	return ret;

err_non:
#if defined(CONFIG_USB_GADGET_F_USB30_USED_DMA_TRANSFER)
#if defined(CONFIG_USB_GADGET_F_USB30_DMA_USE_BOUNCE_BUF)
	for (counter = F_USB30_IN_DMAC; counter < F_USB30_MAX_DMAC; counter++) {
		if (f_usb30->buffer[counter] != NULL)
			dma_free_coherent(NULL, F_USB30_DMAC_TRANS_MAX_BYTES,
						f_usb30->buffer[counter],
						f_usb30->dma_buffer[counter]);
	}
#endif
err_dmaout:
	free_irq(f_usb30->dmac_in_irq, f_usb30);
err_dmain:
	free_irq(f_usb30->hs_irq, f_usb30);
#endif
err_hsirq:
	free_irq(f_usb30->ss_irq, f_usb30);
err_ssirq:
	platform_set_drvdata(pdev, NULL);
	kfree(f_usb30);
err_map:
	unmap_iomem_region(register_base_address);
err_mem:
	release_mem_region(device_resource->start, device_resource_size);
err_res:
	clk_disable_unprepare(clk);
	clk_put(clk);
done:
	dev_dbg(&pdev->dev, "%s() is ended.\n", __func__);
	return ret;
}

/**
 * f_usb30_udc_remove - Removes the udc device driver
 * @pdev:	platform device
 *
 * Returns 0 if no error, -EINVAL on failure
 */
static int __exit f_usb30_udc_remove(struct platform_device *pdev)
{
	unsigned long counter;
	struct f_usb30_udc *f_usb30;

	/* check argument */
	if (unlikely(!pdev))
		return -EINVAL;

	dev_dbg(&pdev->dev, "%s() is started.\n", __func__);

	/* get a device driver parameter */
	f_usb30 = platform_get_drvdata(pdev);
	if (!f_usb30) {
		dev_err(&pdev->dev, "platform_get_drvdata() is failed.\n");
		dev_dbg(&pdev->dev, "%s() is ended.\n", __func__);
		return -EINVAL;
	}

	usb_del_gadget_udc(&f_usb30->gadget);

	/* disable F_USB30 controller interrupt */
#if defined(CONFIG_USB_GADGET_F_USB30_USED_DMA_TRANSFER)
#if defined(CONFIG_USB_GADGET_F_USB30_DMA_USE_BOUNCE_BUF)
	for (counter = F_USB30_IN_DMAC; counter < F_USB30_MAX_DMAC; counter++)
		dma_free_coherent(NULL, F_USB30_DMAC_TRANS_MAX_BYTES,
					f_usb30->buffer[counter],
					f_usb30->dma_buffer[counter]);
#endif
	free_irq(f_usb30->dmac_in_irq, f_usb30);
	free_irq(f_usb30->dmac_out_irq, f_usb30);
#endif
	free_irq(f_usb30->hs_irq, f_usb30);
	free_irq(f_usb30->ss_irq, f_usb30);

	spin_lock(&f_usb30->lock);
	/* dequeue all previous transfer request */
	for (counter = ENDPOINT0; counter < F_USB30_MAX_EP; counter++) {
		f_usb30->udc_endpoint[counter].halt = 1;
		dequeue_all_transfer_request(&f_usb30->udc_endpoint[counter],
						 -ESHUTDOWN);
	}
	spin_unlock(&f_usb30->lock);

	/* notify disconnect to gadget driver */
	if ((f_usb30->gadget_driver) &&
		 (f_usb30->gadget_driver->disconnect))
		f_usb30->gadget_driver->disconnect(&f_usb30->gadget);

	/* notify unbind to gadget driver */
	if ((f_usb30->gadget_driver) && (f_usb30->gadget_driver->unbind))
		f_usb30->gadget_driver->unbind(&f_usb30->gadget);

	/* disable F_USB30 device clock */
	clk_disable_unprepare(f_usb30->clk);
	clk_put(f_usb30->clk);

	/* release device resource */
	platform_set_drvdata(pdev, NULL);
	unmap_iomem_region(f_usb30->register_base_address);
	release_mem_region(f_usb30->device_resource->start,
			   f_usb30->device_resource->end -
			   f_usb30->device_resource->start + 1);

	/* free F_USB30 UDC device driver structure data memory */
	kfree(f_usb30);

	dev_dbg(&pdev->dev, "%s() is ended.\n", __func__);

	return 0;
}

static const struct of_device_id f_usb30_dt_ids[] = {
	{ .compatible = "fujitsu,f_usb30_udc" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, f_usb30_dt_ids);

/* F_USB30 UDC device driver structure */
static struct platform_driver f_usb30_udc_driver = {
	.remove = __exit_p(f_usb30_udc_remove),
	.driver = {
		.name = "f_usb30_udc",
		.owner = THIS_MODULE,
		.of_match_table = f_usb30_dt_ids,
		},
};

/**
 * f_usb30_udc_init - initialize module
 *
 * Returns 0 if no error, negative errno on failure
 */
static int __init f_usb30_udc_init(void)
{
	return platform_driver_probe(&f_usb30_udc_driver,
					 f_usb30_udc_probe);
}

module_init(f_usb30_udc_init);

/**
 * f_usb30_udc_exit - exit module
 */
static void __exit f_usb30_udc_exit(void)
{
	platform_driver_unregister(&f_usb30_udc_driver);
}

module_exit(f_usb30_udc_exit);

/* F_USB30 UDC device module definition */
MODULE_AUTHOR("Fujitsu Semiconductor Limited");
MODULE_DESCRIPTION("F_USB30 USB3.0 Function Controller Driver");
MODULE_ALIAS("platform:f_usb30_udc");
MODULE_LICENSE("GPL");
