/*
 * linux/drivers/usb/gadget/f_usb20hdc_udc.c - F_USB20HDC USB function
 * controller driver
 *
 * Copyright (C) FUJITSU ELECTRONICS INC. 2011-2012. All rights reserved.
 * Copyright (C) 2012 FUJITSU SEMICONDUCTOR LIMITED.
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
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/debugfs.h>
#include <linux/sysfs.h>

#include <linux/platform_data/dma-mb8ac0300-hdmac.h>

#include "f_usb20hdc_udc.h"

static struct f_usb20hdc_udc *f_usb20hdc_data;

#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
static void on_end_dma_transfer(u32 channel, void *data, int state);
static u8 set_dma_transfer(struct f_usb20hdc_udc_ep *endpoint,
	dma_addr_t source, dma_addr_t destination, u32 bytes, u8 last_transfer)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	void *base_addr = f_usb20hdc->reg_base;
	u8 endpoint_channel = endpoint->endpoint_channel;
	s8 usb_dma_channel = endpoint->usb_dma_channel;
	struct f_usb20hdc_dma_data *dma_data =
					&f_usb20hdc->dma_data[usb_dma_channel];
	int result;

	/* check DMA channel */
	if (unlikely(usb_dma_channel == -1))
		return 0;

	/* set HDMAC transfer request parameter */
	dma_data->hdmac_req.dmacb = HDMACB_TT_2CYCLE | HDMACB_MS_BLOCK |
					HDMACB_EI | HDMACB_CI | HDMACB_TW_WORD;
	dma_data->hdmac_req.dmacb |= endpoint->transfer_direction ?
							HDMACB_FD : HDMACB_FS;
#if (F_USB20HDC_UDC_USE_DMA_BURST_TRANSFER == 1)
	dma_data->hdmac_req.dmaca = dma_data->dreq | HDMACA_BT_INCR16 |
							((bytes + 63) / 64 - 1);
	set_dma_blksize(base_addr, usb_dma_channel, 64);
#else
	/* fix me to implement non-burst transfer */
	dma_data->hdmac_req.dmaca = dma_data->dreq | HDMACA_BT_SINGLE |
							((bytes + 3) / 4 - 1);
	set_dma_blksize(base_addr, usb_dma_channel, 4);
#endif
	dma_data->hdmac_req.req_data.size = bytes;
	dma_data->hdmac_req.req_data.src = source;
	dma_data->hdmac_req.req_data.dst = destination;
	dma_data->hdmac_req.req_data.irq_data = endpoint;
	dma_data->hdmac_req.req_data.callback_fn = on_end_dma_transfer;

	result = hdmac_enqueue(dma_data->hdmac_channel, &dma_data->hdmac_req);
	if (result) {
		dev_err(f_usb20hdc->gadget.dev.parent,
			"%s():HDMAC request enqueue failed at 0x%d.\n",
							__func__, result);
		return 0;
	}

	/* setup F_USB20HDC controller register */
	set_dma_mode(base_addr, usb_dma_channel, DMA_MODE_DEMAND);
	if (endpoint->transfer_direction) {
		if (last_transfer) {
			set_dma_sendnull(base_addr, usb_dma_channel,
						endpoint->null_packet ? 1 : 0);
			set_dma_int_empty(base_addr, usb_dma_channel,
					endpoint->in_trans_end_timing ? 1 : 0);
		} else {
			set_dma_sendnull(base_addr, usb_dma_channel, 0);
			set_dma_int_empty(base_addr, usb_dma_channel, 0);
		}
	} else {
		set_dma_spr(base_addr, usb_dma_channel, 0);
	}
	set_dma_ep(base_addr, usb_dma_channel, endpoint_channel);
	set_dma_tci(base_addr, usb_dma_channel, bytes);

	return 1;
}

static void enable_dma_transfer(struct f_usb20hdc_udc_ep *endpoint, u8 enable)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	void *base_addr = f_usb20hdc->reg_base;
	s8 usb_dma_channel = endpoint->usb_dma_channel;

	/* check DMA channel */
	if (unlikely(usb_dma_channel == -1))
		return;

	if (enable) {
		/* enable DMA transfer */
		if (hdmac_start(f_usb20hdc->dma_data[
				endpoint->usb_dma_channel].hdmac_channel) != 0){
			endpoint->dma_transfer = 0;
			dev_err(f_usb20hdc->gadget.dev.parent,
				"%s():hdmac_start() failed\n", __func__);
			return;
		}
		set_dma_inten(base_addr, usb_dma_channel, 1);
		set_dma_st(base_addr, usb_dma_channel, 1);
	} else {
		/* disable DMA transfer */
		set_dma_st(base_addr, usb_dma_channel, 0);
		set_dma_inten(base_addr, usb_dma_channel, 0);
		clear_dma_int(base_addr, usb_dma_channel);
		endpoint->dma_transfer = 0;
		f_usb20hdc->dma_data[
			endpoint->usb_dma_channel].endpoint_channel = -1;
	}

	return;
}
#endif

static void abort_in_transfer_dma(struct f_usb20hdc_udc_ep *endpoint, u8 init);
static void abort_out_transfer_dma(struct f_usb20hdc_udc_ep *endpoint, u8 init);
static u8 set_in_transfer_dma(struct f_usb20hdc_udc_ep *endpoint)
{
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	struct f_usb20hdc_udc_req *request = endpoint->request;
#ifdef LOWLEVEL_DEBUG
	u8 endpoint_channel = endpoint->endpoint_channel;
#endif
	dma_addr_t dma_addr;
	u32 bytes;

	/* check argument */
	if (unlikely(!endpoint->endpoint.desc)) {
		enable_dma_transfer(endpoint, 0);
		return 0;
	}

	/* check NULL packet IN transfer for last packet of transaction */
	if ((request->request.length) && (!(request->request.length %
		endpoint->endpoint.maxpacket)) && (request->request.zero))
		endpoint->null_packet = 1;

	/* calculate this time transfer byte */
	bytes = request->request.length < F_USB20HDC_UDC_DMA_TRANS_MAX_BYTES ?
		request->request.length : F_USB20HDC_UDC_DMA_TRANS_MAX_BYTES;
	if (bytes / endpoint->endpoint.maxpacket)
		bytes -= bytes % endpoint->endpoint.maxpacket;

#if !defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)

	/* check DMA transfer buffer mapping */
	if (request->request.dma == ~(dma_addr_t)0) {
		/* map DMA transfer buffer and synchronize DMA transfer buffer*/
		request->request.dma =
			dma_map_single(f_usb20hdc->gadget.dev.parent,
					request->request.buf,
					request->request.length, DMA_TO_DEVICE);
		request->dma_transfer_buffer_map = 1;
	} else {
		/* synchronize DMA transfer buffer */
		dma_sync_single_for_device(f_usb20hdc->gadget.dev.parent,
					request->request.dma,
					request->request.length, DMA_TO_DEVICE);
		request->dma_transfer_buffer_map = 0;
	}

	/* check DMA transfer address align */
	if (request->request.dma & 0x3) {
		dbg_print(f_usb20hdc->gadget.dev.parent,
			"%s():DMA controller can not setup at dma address = 0x%p.\n",
			__func__, (void *)request->request.dma);
		enable_dma_transfer(endpoint, 0);
		if (request->dma_transfer_buffer_map) {
			dma_unmap_single(f_usb20hdc->gadget.dev.parent,
					request->request.dma,
					request->request.length, DMA_TO_DEVICE);
			request->request.dma = ~(dma_addr_t)0;
			request->dma_transfer_buffer_map = 0;
		}
		return 0;
	}
#endif

	/* set dma transfer source address */
#if defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
	dma_addr = f_usb20hdc->dma_data[endpoint->usb_dma_channel].dma_buffer;
#else
	dma_addr = request->request.dma;
#endif

	/* setup DMA transfer */
	if (!set_dma_transfer(endpoint, (dma_addr_t)dma_addr,
			(dma_addr_t)f_usb20hdc->dma_data[
			endpoint->usb_dma_channel].epbuf_dma_addr,
			     bytes, request->request.length <= bytes ? 1 : 0)) {
		dev_err(f_usb20hdc->gadget.dev.parent,
			"%s():set_dma_transfer() failed.\n", __func__);
		abort_in_transfer_dma(endpoint, 0);

		return 0;
	}

#if defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
	/* copy IN transfer data to noncachable buffer */
	memcpy(f_usb20hdc->dma_data[endpoint->usb_dma_channel].buffer,
				request->request.buf, bytes);
#endif

	dbg_print(f_usb20hdc->gadget.dev.parent,
		"endpoint %u DMA is setup at length = %u, actual = %u, ",
		endpoint_channel, request->request.length,
		request->request.actual);
	dbg_print(f_usb20hdc->gadget.dev.parent,
		"max packet = %u, this time = %u.\n",
		endpoint->endpoint.maxpacket,
		(u32)bytes);

	/* set request execute */
	request->request_execute = 1;
	request->dmac_int_occurred = 0;
	request->usb_dma_int_occurred = 0;

	/* enable DMA transfer */
	enable_dma_transfer(endpoint, 1);

	return 1;
#else
	return 0;
#endif
}

static u8 set_out_transfer_dma(struct f_usb20hdc_udc_ep *endpoint)
{
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	struct f_usb20hdc_udc_req *request = endpoint->request;
#ifdef LOWLEVEL_DEBUG
	u8 endpoint_channel = endpoint->endpoint_channel;
#endif
	dma_addr_t dma_addr;
	u32 bytes;

	/* check argument */
	if (unlikely(!endpoint->endpoint.desc)) {
		enable_dma_transfer(endpoint, 0);
		return 0;
	}

	/* calculate this time transfer byte */
	bytes = request->request.length < F_USB20HDC_UDC_DMA_TRANS_MAX_BYTES ?
		request->request.length : F_USB20HDC_UDC_DMA_TRANS_MAX_BYTES;
	if (bytes / endpoint->endpoint.maxpacket)
		bytes -= bytes % endpoint->endpoint.maxpacket;

#if !defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
	/* check DMA transfer buffer mapping */
	if (request->request.dma == ~(dma_addr_t)0) {
		/* map DMA transfer buffer and synchronize DMA transfer buffer*/
		request->request.dma =
				dma_map_single(f_usb20hdc->gadget.dev.parent,
						request->request.buf,
						request->request.length,
						DMA_FROM_DEVICE);
		request->dma_transfer_buffer_map = 1;
	} else {
		/* synchronize DMA transfer buffer */
		dma_sync_single_for_device(f_usb20hdc->gadget.dev.parent,
						request->request.dma,
						request->request.length,
						DMA_FROM_DEVICE);
		request->dma_transfer_buffer_map = 0;
	}

	/* check DMA transfer address align and DMA transfer length */
	if ((request->request.dma & 0x3) || (request->request.length & 0x3)) {
		dbg_print(f_usb20hdc->gadget.dev.parent,
			"%s():DMA controller can not setup at dma address = 0x%p, dma length = %u.\n",
			__func__, (void *)request->request.dma,
			request->request.length);
		enable_dma_transfer(endpoint, 0);
		if (request->dma_transfer_buffer_map) {
			dma_unmap_single(f_usb20hdc->gadget.dev.parent,
					request->request.dma,
					request->request.length,
					DMA_FROM_DEVICE);
			request->request.dma = ~(dma_addr_t)0;
			request->dma_transfer_buffer_map = 0;
		}
		return 0;
	}
#endif

	/* set dma transfer destination address */
#if defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
	dma_addr = f_usb20hdc->dma_data[endpoint->usb_dma_channel].dma_buffer;
#else
	dma_addr = request->request.dma;
#endif

	/* setup DMA transfer */
	if (!set_dma_transfer(endpoint, (dma_addr_t)f_usb20hdc->dma_data[
				endpoint->usb_dma_channel].epbuf_dma_addr,
				(dma_addr_t)dma_addr, bytes, 0)) {
		dev_err(f_usb20hdc->gadget.dev.parent,
			"%s():set_dma_transfer() failed.\n", __func__);
		abort_out_transfer_dma(endpoint, 0);

		return 0;
	}

	dbg_print(f_usb20hdc->gadget.dev.parent,
		"endpoint %u DMA is setup at length = %u, actual = %u, ",
		endpoint_channel, request->request.length,
		request->request.actual);
	dbg_print(f_usb20hdc->gadget.dev.parent,
		"max packet = %u, this time = %u.\n",
		endpoint->endpoint.maxpacket,
		(u32)bytes);

	/* set request execute */
	request->request_execute = 1;

	/* enable DMA transfer */
	enable_dma_transfer(endpoint, 1);

	return 1;
#else
	return 0;
#endif
}

static void initialize_endpoint(struct f_usb20hdc_udc_ep *, u8, u8, u8);
static void abort_in_transfer_dma(struct f_usb20hdc_udc_ep *endpoint, u8 init)
{
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
#if !defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	struct f_usb20hdc_udc_req *request = endpoint->request;
	u8 dma_transfer = endpoint->dma_transfer;
#endif
	/* disable DMA transfer */
	enable_dma_transfer(endpoint, 0);

#if !defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
	/* check DMA transfer buffer unmap */
	if ((dma_transfer) && (request->request.dma != ~(dma_addr_t)0)) {
		if (request->dma_transfer_buffer_map) {
			/* unmap DMA transfer buf and sync DMA transfer buf */
			dma_unmap_single(f_usb20hdc->gadget.dev.parent,
					request->request.dma,
					request->request.length, DMA_TO_DEVICE);
			request->request.dma = ~(dma_addr_t)0;
			request->dma_transfer_buffer_map = 0;
		} else {
			/* synchronize DMA transfer buffer */
			dma_sync_single_for_cpu(f_usb20hdc->gadget.dev.parent,
						request->request.dma,
						request->request.length,
						DMA_TO_DEVICE);
		}
	}
#endif
	/* clear NULL packet transfer */
	endpoint->null_packet = 0;

	if (!init)
		return;

	/* initialize endpoint */
	initialize_endpoint(endpoint, 1, 0, 0);
#endif
	return;
}

static void abort_out_transfer_dma(struct f_usb20hdc_udc_ep *endpoint, u8 init)
{
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
#if !defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	struct f_usb20hdc_udc_req *request = endpoint->request;
	u8 dma_transfer = endpoint->dma_transfer;
#endif
	/* disable DMA transfer */
	enable_dma_transfer(endpoint, 0);

#if !defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
	/* check DMA transfer buffer unmap */
	if ((dma_transfer) && (request->request.dma != ~(dma_addr_t)0)) {
		if (request->dma_transfer_buffer_map) {
			/* unmap DMA transfer buf and sync DMA transfer buf */
			dma_unmap_single(f_usb20hdc->gadget.dev.parent,
					request->request.dma,
					request->request.length,
					DMA_FROM_DEVICE);
			request->request.dma = ~(dma_addr_t)0;
			request->dma_transfer_buffer_map = 0;
		} else {
			/* synchronize DMA transfer buffer */
			dma_sync_single_for_cpu(f_usb20hdc->gadget.dev.parent,
						request->request.dma,
						request->request.length,
						DMA_FROM_DEVICE);
		}
	}
#endif
	if (!init)
		return;

	/* initialize endpoint */
	initialize_endpoint(endpoint, 1, 0, 0);
#endif
	return;
}

static u8 end_in_transfer_dma(struct f_usb20hdc_udc_ep *endpoint)
{
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	struct f_usb20hdc_udc_req *request = endpoint->request;
#ifdef LOWLEVEL_DEBUG
	u8 endpoint_channel = endpoint->endpoint_channel;
#endif
	s8 usb_dma_channel = endpoint->usb_dma_channel;
	dma_addr_t dma_addr;
	u32 bytes;

	/* get this time transfer byte */
	bytes = get_dma_tc(f_usb20hdc->reg_base, usb_dma_channel);

	dbg_print(f_usb20hdc->gadget.dev.parent,
		"endpoint %u DMA is end at length = %u, actual = %u, ",
		endpoint_channel, request->request.length,
		request->request.actual);
	dbg_print(f_usb20hdc->gadget.dev.parent,
		"max packet = %u, this time = %u.\n",
		endpoint->endpoint.maxpacket,
		(u32)bytes);

	/* update actual bytes */
	request->request.actual += bytes;

	/* check transfer remain byte */
	if (!(request->request.length - request->request.actual)) {
		/* complete request */
		abort_in_transfer_dma(endpoint, 0);
		return 1;
	}

	/* calculate this time transfer byte */
	bytes = (request->request.length - request->request.actual) <
			F_USB20HDC_UDC_DMA_TRANS_MAX_BYTES ?
			(request->request.length - request->request.actual) :
				F_USB20HDC_UDC_DMA_TRANS_MAX_BYTES;
	if (bytes / endpoint->endpoint.maxpacket)
		bytes -= bytes % endpoint->endpoint.maxpacket;

#if defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
	/* copy IN transfer data to noncachable buffer */
	memcpy(f_usb20hdc->dma_data[endpoint->usb_dma_channel].buffer,
			(request->request.buf + request->request.actual),
			bytes);
#endif

	/* update dma transfer source address */
#if defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
	dma_addr = f_usb20hdc->dma_data[endpoint->usb_dma_channel].dma_buffer;
#else
	dma_addr = (dma_addr_t)(request->request.dma + request->request.actual);
#endif

	/* setup DMA transfer */
	if (!set_dma_transfer(endpoint, (dma_addr_t)dma_addr,
		(dma_addr_t)f_usb20hdc->dma_data[
			endpoint->usb_dma_channel].epbuf_dma_addr,
			     bytes, request->request.length <= bytes ? 1 : 0)) {
		dev_err(f_usb20hdc->gadget.dev.parent,
			"%s():set_dma_transfer() failed.\n", __func__);
		abort_in_transfer_dma(endpoint, 0);

		return 0;
	}

	dbg_print(f_usb20hdc->gadget.dev.parent,
		"endpoint %u DMA is setup at length = %u, actual = %u, ",
		endpoint_channel, request->request.length,
		request->request.actual);
	dbg_print(f_usb20hdc->gadget.dev.parent,
		"max packet = %u, this time = %u.\n",
		endpoint->endpoint.maxpacket,
		(u32)bytes);

	/* clear hdmac and usb dma int flag */
	request->dmac_int_occurred = 0;
	request->usb_dma_int_occurred = 0;

	/* enable DMA transfer */
	enable_dma_transfer(endpoint, 1);
#endif
	return 0;
}

static u8 end_out_transfer_dma(struct f_usb20hdc_udc_ep *endpoint)
{
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	void *base_addr = f_usb20hdc->reg_base;
	struct f_usb20hdc_udc_req *request = endpoint->request;
#ifdef LOWLEVEL_DEBUG
	u8 endpoint_channel = endpoint->endpoint_channel;
#endif
	s8 usb_dma_channel = endpoint->usb_dma_channel;
	dma_addr_t dma_addr;
	u32 bytes;

	/* get this time transfer byte */
	bytes = get_dma_tc(base_addr, usb_dma_channel);

	dbg_print(f_usb20hdc->gadget.dev.parent,
		"endpoint %u DMA is end at length = %u, actual = %u, ",
		endpoint_channel, request->request.length,
		request->request.actual);
	dbg_print(f_usb20hdc->gadget.dev.parent,
		"max packet = %u, this time = %u.\n",
		endpoint->endpoint.maxpacket,
		(u32)bytes);

#if defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
	/* copy bulk OUT transfer data from noncachable buffer */
	memcpy((request->request.buf + request->request.actual),
		f_usb20hdc->dma_data[endpoint->usb_dma_channel].buffer,	bytes);
#endif
	/* update actual bytes */
	request->request.actual += bytes;

	/* check transfer request complete */
	if ((request->request.length <= request->request.actual) ||
		(get_dma_sp(base_addr, usb_dma_channel))) {
		/* complete request */
		abort_out_transfer_dma(endpoint, 0);
		return 1;
	}

	/* calculate this time transfer byte */
	bytes = (request->request.length - request->request.actual) <
			F_USB20HDC_UDC_DMA_TRANS_MAX_BYTES ?
			(request->request.length - request->request.actual) :
			F_USB20HDC_UDC_DMA_TRANS_MAX_BYTES;
	if (bytes / endpoint->endpoint.maxpacket)
		bytes -= bytes % endpoint->endpoint.maxpacket;

	/* update dma transfer destination address */
#if defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
	dma_addr = f_usb20hdc->dma_data[endpoint->usb_dma_channel].dma_buffer;
#else
	dma_addr = (dma_addr_t)(request->request.dma + request->request.actual);
#endif

	/* setup DMA transfer */
	if (!set_dma_transfer(endpoint, (dma_addr_t)f_usb20hdc->dma_data[
				endpoint->usb_dma_channel].epbuf_dma_addr,
				(dma_addr_t)dma_addr, bytes, 0)) {
		dev_err(f_usb20hdc->gadget.dev.parent,
			"%s():set_dma_transfer() failed.\n", __func__);
		abort_out_transfer_dma(endpoint, 0);

		return 0;
	}

	dbg_print(f_usb20hdc->gadget.dev.parent,
		"endpoint %u DMA is setup at length = %u, actual = %u, ",
		endpoint_channel, request->request.length,
		request->request.actual);
	dbg_print(f_usb20hdc->gadget.dev.parent,
		"max packet = %u, this time = %u.\n",
		endpoint->endpoint.maxpacket,
		(u32)bytes);

	/* enable DMA transfer */
	enable_dma_transfer(endpoint, 1);
#endif
	return 0;
}

static u8 is_dma_transfer_usable(struct f_usb20hdc_udc_ep *endpoint)
{
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;

	if ((endpoint->usb_dma_channel == -1) || (endpoint->dma_transfer) ||
		(f_usb20hdc->dma_data[
			endpoint->usb_dma_channel].endpoint_channel != -1))
		return 0;
	f_usb20hdc->dma_data[endpoint->usb_dma_channel].endpoint_channel =
						endpoint->endpoint_channel;
	endpoint->dma_transfer = 1;
	return 1;
#else
	return 0;
#endif
}

static u8 is_pio_transfer_auto_change_usable(struct f_usb20hdc_udc_ep *endpoint)
{
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	if ((endpoint->pio_auto_change) || (endpoint->usb_dma_channel == -1))
		return 1;
	return 0;
#else
	return 1;
#endif
}

#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
static void dequeue_all_transfer_request(struct f_usb20hdc_udc_ep *, int);
static void on_end_in_transfer(struct f_usb20hdc_udc_ep *endpoint);
static void on_end_out_transfer(struct f_usb20hdc_udc_ep *endpoint);
static void on_end_dma_transfer(u32 channel, void *data, int state)
{
	struct f_usb20hdc_udc_ep *endpoint = data;
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;

	dev_dbg(f_usb20hdc->gadget.dev.parent,
			"%s() is started.\n", __func__);
	spin_lock(&f_usb20hdc->lock);
	/* check argument */
	if (unlikely(!endpoint || endpoint->request == NULL ||
			channel != endpoint->f_usb20hdc->dma_data[
				endpoint->usb_dma_channel].hdmac_channel)) {
		spin_unlock(&f_usb20hdc->lock);
		return;
	}

	/* DMA transfer end interrupt factor */
	switch (state) {
	case HDMACB_SS_ADD_OVERFLOW:
	case HDMACB_SS_SOURCE_ACCESS_ERROR:
	case HDMACB_SS_DESTINATION_ACCESS_ERROR:
		/* dequeue all previous requests */
		dequeue_all_transfer_request(endpoint, -EL2HLT);

		break;
	case HDMACB_SS_TRANSFER_STOP_REQUEST:
		endpoint->transfer_direction ?
			on_end_in_transfer(endpoint) :
			on_end_out_transfer(endpoint);
		break;
	case HDMACB_SS_NORMAL_END:
		if (endpoint->transfer_direction) {
			endpoint->request->dmac_int_occurred = 1;
			if (endpoint->request->usb_dma_int_occurred)
				on_end_in_transfer(endpoint);
		} else
			on_end_out_transfer(endpoint);
		break;
	default:
		break;
	}

	spin_unlock(&f_usb20hdc->lock);

	dev_dbg(f_usb20hdc->gadget.dev.parent, "%s() is ended.\n", __func__);

	return;
}
#endif

static void initialize_udc_controller(struct f_usb20hdc_udc *f_usb20hdc,
								u8 preinit)
{
	u32 counter;
	struct device *dev = f_usb20hdc->dev;

	/*
	 * check host mode
	 * [notice]:It is processing nothing, when host_en bit is set
	 */
	if (get_host_en(f_usb20hdc->reg_base))
		return;

	/* check pre-initalize */
	if (preinit) {
		dev_info(dev, "initialize_udc_controller(): preinit.\n");
		/* initialize F_USB20HDC system configuration register */
		counter = 10;
		do {
			f_usb20hdc_soft_reset_ip(f_usb20hdc->f_otg);
			counter--;
		} while (get_phyreset(f_usb20hdc->reg_base) && counter);
		if (!counter) {
			dev_err(dev, "%s(preinit) was failed.\n", __func__);
			return;
		}

		/* pre-initialize F_USB20HDC otg register */
		set_dm_pull_down(f_usb20hdc->reg_base, 0);
		set_dp_pull_down(f_usb20hdc->reg_base, 0);
		clear_tmrout_c(f_usb20hdc->reg_base);
		clear_vbus_vld_c(f_usb20hdc->reg_base);
		set_otg_tmrout_ren(f_usb20hdc->reg_base, 0);
		set_start_tmr(f_usb20hdc->reg_base, 0);
		set_tmr_init_val(f_usb20hdc->reg_base, 0);
		return;
	}
	dev_dbg(dev, "initialize_udc_controller(): non-preinit.\n");

	/*
	 * initialize F_USB20HDC mode register
	 * [notice]:set of dev_int_mode / dev_addr_load_mode bit is prohibition
	 */
	set_dev_en(f_usb20hdc->reg_base, 0);

	/*
	 * initialize F_USB20HDC global interrupt register
	 * [notice]:otg_inten bit is always enable
	 */
	set_dev_inten(f_usb20hdc->reg_base, 0);
	set_phy_err_inten(f_usb20hdc->reg_base, 0);
	set_cmd_inten(f_usb20hdc->reg_base, 0);
	for (counter = F_USB20HDC_DMA_CH1; counter < F_USB20HDC_MAX_DMA_CHANNELS; counter++)
		set_dma_inten(f_usb20hdc->reg_base, counter, 0);

	for (counter = F_USB20HDC_EP0; counter < F_USB20HDC_UDC_MAX_EP; counter++)
		set_dev_ep_inten(f_usb20hdc->reg_base, counter, 0);

	clear_phy_err_int(f_usb20hdc->reg_base);
	clear_cmd_int(f_usb20hdc->reg_base);
	for (counter = F_USB20HDC_DMA_CH1; counter <
			F_USB20HDC_MAX_DMA_CHANNELS; counter++)
		clear_dma_int(f_usb20hdc->reg_base, counter);

	/* initialize F_USB20HDC device control / status / address register */
	set_reqspeed(f_usb20hdc->reg_base,
			gadget_is_dualspeed(&f_usb20hdc->gadget) ?
				REQ_SPEED_HIGH_SPEED : REQ_SPEED_FULL_SPEED);
	set_reqresume(f_usb20hdc->reg_base, 0);
	set_enrmtwkup(f_usb20hdc->reg_base, 0);
	set_suspende_inten(f_usb20hdc->reg_base, 0);
	set_suspendb_inten(f_usb20hdc->reg_base, 0);
	set_sof_inten(f_usb20hdc->reg_base, 0);
	set_setup_inten(f_usb20hdc->reg_base, 0);
	set_usbrste_inten(f_usb20hdc->reg_base, 0);
	set_usbrstb_inten(f_usb20hdc->reg_base, 0);
	set_status_ok_inten(f_usb20hdc->reg_base, 0);
	set_status_ng_inten(f_usb20hdc->reg_base, 0);
	clear_suspende_int(f_usb20hdc->reg_base);
	clear_suspendb_int(f_usb20hdc->reg_base);
	clear_sof_int(f_usb20hdc->reg_base);
	clear_setup_int(f_usb20hdc->reg_base);
	clear_usbrste_int(f_usb20hdc->reg_base);
	clear_usbrstb_int(f_usb20hdc->reg_base);
	clear_status_ok_int(f_usb20hdc->reg_base);
	clear_status_ng_int(f_usb20hdc->reg_base);
	set_func_addr(f_usb20hdc->reg_base, 0);
	set_dev_configured(f_usb20hdc->reg_base, 0);

	/* initialize F_USB20HDC dma register */
	for (counter = F_USB20HDC_DMA_CH1;
			counter < F_USB20HDC_MAX_DMA_CHANNELS; counter++) {
		set_dma_st(f_usb20hdc->reg_base, counter, 0);
		set_dma_mode(f_usb20hdc->reg_base,
					counter, DMA_MODE_DEMAND);
		set_dma_sendnull(f_usb20hdc->reg_base, counter, 0);
		set_dma_int_empty(f_usb20hdc->reg_base, counter, 0);
		set_dma_spr(f_usb20hdc->reg_base, counter, 0);
		set_dma_ep(f_usb20hdc->reg_base, counter, 0);
		set_dma_blksize(f_usb20hdc->reg_base, counter, 0);
		set_dma_tci(f_usb20hdc->reg_base, counter, 0);
	}

	/* initialize F_USB20HDC test control register */
	set_testp(f_usb20hdc->reg_base, 0);
	set_testj(f_usb20hdc->reg_base, 0);
	set_testk(f_usb20hdc->reg_base, 0);
	set_testse0nack(f_usb20hdc->reg_base, 0);

	/* initialize F_USB20HDC ram register */
	for (counter = F_USB20HDC_EP0;
		counter < F_USB20HDC_UDC_MAX_EP; counter++) {
		clear_epctrl(f_usb20hdc->reg_base, counter);
		clear_epconf(f_usb20hdc->reg_base, counter);
	}
	for (counter = 0; counter < F_USB20HDC_UDC_MAX_EP *
				F_USB20HDC_EP_BUFFER_COUNT; counter++)
		clear_epcount(f_usb20hdc->reg_base, counter);

	/* wait PHY reset release */
	for (counter = 0xffff; ((counter) &&
		(get_phyreset(f_usb20hdc->reg_base))); counter--)
		;

	return;
}

/*
 * flag = 1, Setting for entering usb-taiki mode
 * flag = 0, Setting for exiting usb-taiki mode
 */
void set_udc_wakeup_interrupt(struct f_usb20hdc_udc_ep *endpoint, u8 flag)
{
	u8 endpoint_channel = endpoint->endpoint_channel;
	struct f_usb20hdc_udc *f_usb20hdc;
	void __iomem *base_addr;
	u8 ep_dir, ep_et;

	f_usb20hdc = endpoint->f_usb20hdc;
	if (!f_usb20hdc) {
		pr_debug("f_usb20hdc is NULL\n");
		return;
	}

	base_addr = f_usb20hdc->reg_base;
	if (!base_addr) {
		pr_debug("base_addr is NULL\n");
		return;
	}

	if (endpoint_channel == F_USB20HDC_EP0) {
		/* No need to monitor EP0 interrupts for wakeup */
		return;
	}
	if(get_dev_en(base_addr) && f_usb20hdc->udc_exclu_suspend_int) {
		/* suspendb_inten */
		clear_suspendb_int(base_addr);
		set_suspendb_inten(base_addr, flag ? 0 : 1);
		/* suspende_inten */
		clear_suspende_int(base_addr);
		set_suspende_inten(base_addr, flag ? 0 : 1);
	}

	ep_dir = get_dir(base_addr, endpoint_channel);
	ep_et = get_et(base_addr, endpoint_channel);
	if ((ep_dir == 0) && (ep_et == TYPE_BULK)) {
		clear_readyi_ready_int_clr(base_addr, endpoint_channel);
		set_readyi_ready_inten(base_addr, endpoint_channel, flag ? 1 : 0);
	}
}

static void initialize_endpoint(struct f_usb20hdc_udc_ep *endpoint,
	u8 fifo, u8 stall, u8 toggle)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	void *base_addr = f_usb20hdc->reg_base;
	u8 endpoint_channel = endpoint->endpoint_channel;
	u8 endpoint_enable = get_ep_en(base_addr, endpoint_channel);

	if (endpoint_channel == F_USB20HDC_EP0) {
		/* check new SETUP transfer */
		if ((get_setup_int(base_addr)) ||
			(get_usbrstb_int(base_addr)) ||
				(get_usbrste_int(base_addr)) ||
					(get_busreset(base_addr))) {
			fifo = 0;
			stall = 0;
			toggle = 0;
		}

		if (fifo) {
			if ((endpoint_enable) &&
				(get_ep_en(base_addr, F_USB20HDC_EP0)))
				/* disable endpoint */
				set_stop(base_addr, F_USB20HDC_EP0);

			/* initialize FIFO */
			set_init(base_addr, F_USB20HDC_EP0);
		}

		if (stall) {
			/* initialize endpoint stall */
			set_stall_clear(base_addr, F_USB20HDC_EP0);
		}

		if (toggle) {
			/* initialize endpoint data toggle bit */
			set_toggle_clear(base_addr, F_USB20HDC_EP0);
		}

		/* initialize endpoint control / status register */
		if (fifo) {
			clear_readyi_ready_int_clr(base_addr,
							F_USB20HDC_EP0);
			set_readyi_ready_inten(base_addr,
							F_USB20HDC_EP0, 0);
			set_readyo_empty_inten(base_addr,
							F_USB20HDC_EP0, 0);
		}
		set_ping_inten(base_addr, F_USB20HDC_EP0, 0);
		set_stalled_inten(base_addr, F_USB20HDC_EP0, 1);
		set_nack_inten(base_addr, F_USB20HDC_EP0, 0);
		set_dev_ep_inten(base_addr, F_USB20HDC_EP0, 1);

		if ((endpoint_enable) &&
			(!get_ep_en(base_addr, F_USB20HDC_EP0)))
			/* re-enable endpoint */
			set_start(base_addr, F_USB20HDC_EP0);
	} else {
		if (fifo) {
			if ((endpoint_enable) &&
				(get_ep_en(base_addr, endpoint_channel)))
				/* disable endpoint */
				set_stop(base_addr, endpoint_channel);

			/* initialize FIFO */
			set_init(base_addr, endpoint_channel);
		}

		if (stall) {
			/* initialize endpoint stall */
			set_stall_clear(base_addr, endpoint_channel);
		}

		if (toggle) {
			/* initialize endpoint data toggle bit */
			set_toggle_clear(base_addr, endpoint_channel);
		}

		/* initialize endpoint control / status register */
		clear_readyi_ready_int_clr(base_addr, endpoint_channel);
		clear_readyo_empty_int_clr(base_addr, endpoint_channel);
		set_readyi_ready_inten(base_addr, endpoint_channel, 0);
		set_readyo_empty_inten(base_addr, endpoint_channel, 0);
		set_ping_inten(base_addr, endpoint_channel, 0);
		set_stalled_inten(base_addr, endpoint_channel, 1);
		set_nack_inten(base_addr, endpoint_channel, 0);
		set_dev_ep_inten(base_addr, endpoint_channel, 1);

		if ((endpoint_enable) &&
			(!get_ep_en(base_addr, endpoint_channel)))
			/* re-enable endpoint */
			set_start(base_addr, endpoint_channel);
	}

	return;
}

static void initialize_endpoint_configure(struct f_usb20hdc_udc *f_usb20hdc)
{
	u32 counter, buffer_counter;
	struct f_usb20hdc_udc_ep *ep;
	void *base_addr = f_usb20hdc->reg_base;
	u8 high_speed = f_usb20hdc->gadget_driver ?
				f_usb20hdc->gadget_driver->max_speed ==
					USB_SPEED_HIGH ? 1 : 0 : 1;

	/* initialzie endpoint 0 configure data */
	ep = &f_usb20hdc->endpoint[F_USB20HDC_EP0];
	ep->endpoint.name = ep_config_data[F_USB20HDC_EP0].name;
	ep->endpoint.maxpacket = high_speed ?
			ep_config_data[F_USB20HDC_EP0].hs_maxpacket :
			ep_config_data[F_USB20HDC_EP0].fs_maxpacket;
	ep->endpoint_channel = F_USB20HDC_EP0;
	ep->transfer_direction = 0;
	ep->transfer_type = USB_ENDPOINT_XFER_CONTROL;
	ep->buffer_address_offset[0] = get_epbuf_address_offset();
	ep->buffer_address_offset[1] =
			ep->buffer_address_offset[0] + ep->endpoint.maxpacket;
	ep->buffer_address_offset[1] =
			(ep->buffer_address_offset[1] + 0x3) & ~0x3;
	ep->buffer_size = ep_config_data[F_USB20HDC_EP0].buffer_size;
	ep->buffers = ep_config_data[F_USB20HDC_EP0].buffers;
	ep->pio_auto_change =
			ep_config_data[F_USB20HDC_EP0].pio_auto_change;
	ep->in_trans_end_timing =
			ep_config_data[F_USB20HDC_EP0].trans_end_timing;
	ep->usb_dma_channel =
			ep_config_data[F_USB20HDC_EP0].usb_dma_channel;

	/* configure endpoint 0 */
	set_et(base_addr, F_USB20HDC_EP0, TYPE_CONTROL);
	set_dir(base_addr, F_USB20HDC_EP0, 0);
	set_bnum(base_addr, F_USB20HDC_EP0, 1);
	set_hiband(base_addr, F_USB20HDC_EP0, 0);
	set_base(base_addr, F_USB20HDC_EP0, ep->buffer_address_offset[0]);
	set_size(base_addr, F_USB20HDC_EP0, ep->endpoint.maxpacket);
	set_countidx(base_addr, F_USB20HDC_EP0, 0);

	for (counter = F_USB20HDC_EP1;
		counter < F_USB20HDC_UDC_MAX_EP; counter++) {
		/* initialzie endpoint configure data */
		ep = &f_usb20hdc->endpoint[counter];
		ep->endpoint.name = ep_config_data[counter].name;
		ep->endpoint.maxpacket = high_speed ?
					ep_config_data[counter].hs_maxpacket :
					ep_config_data[counter].fs_maxpacket;
		ep->endpoint_channel = counter;
		ep->transfer_direction = 0;
		ep->transfer_type = 0;
		ep->buffer_size = ep_config_data[counter].buffer_size;
		ep->buffers = ep_config_data[counter].buffers;
		ep->buffer_address_offset[0] =
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
		ep->pio_auto_change = ep_config_data[counter].pio_auto_change;
		ep->in_trans_end_timing =
			ep_config_data[counter].trans_end_timing;
		ep->usb_dma_channel = ep_config_data[counter].usb_dma_channel;
	}

	return;
}

static u8 configure_endpoint(struct f_usb20hdc_udc_ep *endpoint)
{
	static const u8 transfer_type_register[] = {
		TYPE_CONTROL,		/* control transfer */
		TYPE_ISOCHRONOUS,	/* isochronout transfer */
		TYPE_BULK,		/* bulk transfer */
		TYPE_INTERRUPT,		/* interrupt transfer */
	};
	u32 counter;
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	void *base_addr = f_usb20hdc->reg_base;
	enum usb_device_speed speed = f_usb20hdc->gadget.speed;
	u8 ep_ch = endpoint->endpoint.desc->bEndpointAddress &
				USB_ENDPOINT_NUMBER_MASK;
	u8 ep_dir = endpoint->endpoint.desc->bEndpointAddress &
				USB_ENDPOINT_DIR_MASK ? 1 : 0;
	u8 ep_type = endpoint->endpoint.desc->bmAttributes &
				USB_ENDPOINT_XFERTYPE_MASK;
	u16 maxpacket = usb_endpoint_maxp(endpoint->endpoint.desc) & 0x7ff;
	u16 ep_hiband = (usb_endpoint_maxp(endpoint->endpoint.desc) >> 11)
				& 0x3;
	u16 ep_buf_offset = endpoint->buffer_address_offset[0];
	u16 ep_bufs = endpoint->buffers;

	/* check endpoint transfer buffer size */
	if (endpoint->buffer_size < maxpacket)
		return 0;

	/* check endpoint transfer packet maximum byte count violation */
	switch (ep_type) {
	case USB_ENDPOINT_XFER_CONTROL:
		if (((speed == USB_SPEED_FULL) && (maxpacket % 8) &&
			(maxpacket > 64)) || ((speed == USB_SPEED_HIGH) &&
				(maxpacket != 64)))
			return 0;
		break;
	case USB_ENDPOINT_XFER_ISOC:
		if (((speed == USB_SPEED_FULL) && (maxpacket > 1023)) ||
			((speed == USB_SPEED_HIGH) && (maxpacket > 1024)))
			return 0;
		break;
	case USB_ENDPOINT_XFER_INT:
		if (((speed == USB_SPEED_FULL) && (maxpacket > 64)) ||
			((speed == USB_SPEED_HIGH) && (maxpacket > 1024)))
			return 0;
		break;
	case USB_ENDPOINT_XFER_BULK:
		if (((speed == USB_SPEED_FULL) &&
			(maxpacket % 8) && (maxpacket > 64)) ||
			((speed == USB_SPEED_HIGH) && (maxpacket != 512)))
			return 0;
		break;
	default:
		return 0;
	}

	/* set endpoint configure data */
	endpoint->endpoint.maxpacket = maxpacket;
	endpoint->transfer_direction = ep_dir;
	endpoint->transfer_type = ep_type;
	for (counter = 1; counter < F_USB20HDC_EP_BUFFER_COUNT; counter++) {
		endpoint->buffer_address_offset[counter] =
			endpoint->buffer_address_offset[counter - 1] +
			maxpacket;
		endpoint->buffer_address_offset[counter] =
			(endpoint->buffer_address_offset[counter] + 0x3) & ~0x3;
	}

	/* configure endpoint x */
	set_et(base_addr, ep_ch, transfer_type_register[ep_type]);
	set_dir(base_addr, ep_ch, ep_dir ? 1 : 0);
	set_bnum(base_addr, ep_ch, ep_bufs);
	set_hiband(base_addr, ep_ch, ep_hiband);
	set_base(base_addr, ep_ch, ep_buf_offset);
	set_size(base_addr, ep_ch, maxpacket);
	set_countidx(base_addr, ep_ch, ep_ch * F_USB20HDC_EP_BUFFER_COUNT);

	return 1;
}

static u8 is_endpoint_buffer_usable(void)
{
	u32 counter;
	u32 buffer_size = 0;

	/* calculate RAM buffer size */
	buffer_size += 256;
	buffer_size += ep_config_data[F_USB20HDC_EP0].buffer_size *
				F_USB20HDC_EP_BUFFER_COUNT;
	for (counter = F_USB20HDC_EP1;
				counter < F_USB20HDC_UDC_MAX_EP; counter++)
		buffer_size += ep_config_data[counter].buffer_size *
				ep_config_data[counter].buffers;

	return buffer_size <= F_USB20HDC_UDC_EP_BUFFER_RAM_SIZE ? 1 : 0;
}

static u8 is_host_mode_usage(struct f_usb20hdc_udc *f_usb20hdc)
{
	return get_host_en(f_usb20hdc->reg_base) ? 1 : 0;
}

static u8 is_device_mode_usage(struct f_usb20hdc_udc *f_usb20hdc)
{
	return get_dev_en(f_usb20hdc->reg_base) ? 1 : 0;
}

static void enable_endpoint(struct f_usb20hdc_udc_ep *endpoint, u8 enable)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	u8 endpoint_channel = endpoint->endpoint_channel;

	if (enable) {
		/* initialize endpoint */
		initialize_endpoint(endpoint, 1, 1, 1);

		/* set endpoint parameter */
		endpoint->halt = 0;
		endpoint->null_packet = 0;

		/* enable endpoint */
		set_start(f_usb20hdc->reg_base, endpoint_channel);
	} else {
		/*
		 * [notice]:use of an abort_xx_transfer() is prohibition,
		 * because cache becomes panic.
		 */
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
		/* disable DMA transfer */
		enable_dma_transfer(endpoint, 0);
#endif
		/* disable endpoint */
		set_stop(f_usb20hdc->reg_base, endpoint_channel);

		/* initialize endpoint */
		initialize_endpoint(endpoint, 1, 1, 1);

		/* set endpoint parameter */
		endpoint->halt = 1;
		endpoint->force_halt = 0;
		endpoint->null_packet = 0;
	}

	return;
}

static void set_bus_speed(struct f_usb20hdc_udc *f_usb20hdc)
{
	/* set current bus speed */
	f_usb20hdc->gadget.speed =
		get_crtspeed(f_usb20hdc->reg_base) ==
			CRT_SPEED_HIGH_SPEED ? USB_SPEED_HIGH : USB_SPEED_FULL;

	/* set endpoint 0 max packet */
	f_usb20hdc->endpoint[F_USB20HDC_EP0].endpoint.maxpacket =
		f_usb20hdc->gadget.speed == USB_SPEED_HIGH ?
		ep_config_data[F_USB20HDC_EP0].hs_maxpacket :
		ep_config_data[F_USB20HDC_EP0].fs_maxpacket;
	set_size(f_usb20hdc->reg_base, F_USB20HDC_EP0,
		f_usb20hdc->endpoint[F_USB20HDC_EP0].endpoint.maxpacket);

	return;
}

static void set_device_state(struct f_usb20hdc_udc *f_usb20hdc, u8 device_state)
{
	dev_dbg(f_usb20hdc->dev, "%s() is started from %pS.\n",
		__func__, __builtin_return_address(0));

	if (f_usb20hdc->device_state == device_state)
		return;

	/* set device state */
	f_usb20hdc->device_state_last = f_usb20hdc->device_state;
	f_usb20hdc->device_state = device_state;
	dev_dbg(f_usb20hdc->dev, "device state:%u, last state:%u\n",
		device_state, f_usb20hdc->device_state_last);

	return;
}

static u16 get_fifo_bytes(struct f_usb20hdc_udc_ep *endpoint)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	void *base_addr = f_usb20hdc->reg_base;
	u8 endpoint_channel = endpoint->endpoint_channel;
	u8 index = get_appptr(base_addr, endpoint_channel);

	/* get current bytes in FIFO */
	return endpoint->transfer_direction ?
		get_appcnt(base_addr, endpoint_channel == F_USB20HDC_EP0 ?
		0 : endpoint_channel * F_USB20HDC_EP_BUFFER_COUNT + index) :
		get_phycnt(base_addr, endpoint_channel == F_USB20HDC_EP0 ?
		1 : endpoint_channel * F_USB20HDC_EP_BUFFER_COUNT + index);
}

static void notify_transfer_request_complete(struct f_usb20hdc_udc_ep *endpoint,
	int status)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	struct f_usb20hdc_udc_req *request = endpoint->request;
	u8 endpoint_channel = endpoint->endpoint_channel;
	u8 halt = endpoint->halt;

	if (!request)
		return;

	/* delete and initialize list */
	list_del_init(&request->queue);

	/* set request status */
	if (request->request.status == -EINPROGRESS)
		request->request.status = status;
	else
		status = request->request.status;

	/* clear request execute */
	request->request_execute = 0;
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	request->dmac_int_occurred = 0;
	request->usb_dma_int_occurred = 0;
#endif
	endpoint->request = NULL;

	dev_dbg(f_usb20hdc->gadget.dev.parent,
		"endpoint %u request is completed at request = 0x%p, ",
		endpoint_channel, &request->request);
	dev_dbg(f_usb20hdc->gadget.dev.parent,
		"length = %u, actual = %u, status = %d.\n",
		request->request.length,
		request->request.actual, status);

	/* check request complete notify for gadget driver */
	if (!request->request.complete)
		return;

	/* notify request complete for gadget driver */
	endpoint->halt = 1;
	spin_unlock(&f_usb20hdc->lock);
	request->request.complete(&endpoint->endpoint, &request->request);
	spin_lock(&f_usb20hdc->lock);
	endpoint->halt = halt;

	return;
}

static void dequeue_all_transfer_request(struct f_usb20hdc_udc_ep *endpoint,
	int status)
{
	u32 counter;
	struct f_usb20hdc_udc_req *request;
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
#if !defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
	void *dma_virtual_address;
#endif
	if (endpoint->dma_transfer) {
		/* abort in / out DMA transfer */
		endpoint->transfer_direction ?
		abort_in_transfer_dma(endpoint, 1) :
		abort_out_transfer_dma(endpoint, 1);
		spin_unlock(&f_usb20hdc->lock);
		hdmac_stop_nowait(f_usb20hdc->dma_data[
				endpoint->usb_dma_channel].hdmac_channel);
		spin_lock(&f_usb20hdc->lock);
	}
#endif
	/* dequeue all transfer request */
	for (counter = (u32)-1; (counter) &&
			(!list_empty(&endpoint->queue)); counter--) {
		request = list_entry(endpoint->queue.next,
					struct f_usb20hdc_udc_req, queue);
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
#if !defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
		/* check DMA transfer buffer unmap */
		if ((endpoint->dma_transfer) &&
			(request->request.dma != ~(dma_addr_t)0)) {
			if (request->dma_transfer_buffer_map) {
				/*
				 * unmap DMA transfer buffer and
				 * synchronize DMA transfer buffer
				 */
				dma_unmap_single(f_usb20hdc->gadget.dev.parent,
						request->request.dma,
						request->request.length,
						endpoint->transfer_direction ?
						DMA_TO_DEVICE :
						DMA_FROM_DEVICE);
				request->request.dma = ~(dma_addr_t)0;
				request->dma_transfer_buffer_map = 0;
			} else {
				dma_virtual_address = dma_to_virt(
						f_usb20hdc->gadget.dev.parent,
						request->request.dma);
				/* check DMA transfer buffer synchronize */
				if ((virt_addr_valid(dma_virtual_address)) &&
					(virt_addr_valid(dma_virtual_address +
					request->request.length - 1)) &&
					(!((VMALLOC_START <=
						(u32)request->request.buf) &&
						((u32)request->request.buf <=
						VMALLOC_END))))
					/* sync DMA transfer buffer for CPU */
					dma_sync_single_for_cpu(
						f_usb20hdc->gadget.dev.parent,
						request->request.dma,
						request->request.length,
						endpoint->transfer_direction ?
						DMA_TO_DEVICE :
						DMA_FROM_DEVICE);
			}
		}
#endif
#endif
		endpoint->request = request;
		notify_transfer_request_complete(endpoint, status);
	}

	return;
}

static u8 is_setup_transferred(struct f_usb20hdc_udc *f_usb20hdc)
{
	return (get_setup_int(f_usb20hdc->reg_base)) ||
		(get_usbrstb_int(f_usb20hdc->reg_base)) ||
		(get_usbrste_int(f_usb20hdc->reg_base)) ||
		(get_busreset(f_usb20hdc->reg_base)) ? 1 : 0;
}

static u8 set_in_transfer_pio(struct f_usb20hdc_udc_ep *endpoint)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	void *base_addr = f_usb20hdc->reg_base;
	struct f_usb20hdc_udc_req *request = endpoint->request;
	u8 endpoint_channel = endpoint->endpoint_channel;
	u32 *transfer_data;
	u8 index = endpoint_channel == F_USB20HDC_EP0 ? 0 :
					get_appptr(base_addr, endpoint_channel);
	u32 bytes;

	/* check argument */
	if (unlikely((endpoint_channel != F_USB20HDC_EP0) &&
					!endpoint->endpoint.desc))
		return 0;

	/* check new SETUP transfer */
	if ((endpoint_channel == F_USB20HDC_EP0) &&
			(is_setup_transferred(f_usb20hdc))) {
		dev_err(f_usb20hdc->gadget.dev.parent,
			"%s():new SETUP tranfer is occurred.\n", __func__);
		return 0;
	}

	/* check transfer data setup */
	if (endpoint_channel == F_USB20HDC_EP0 ?
		get_fulli(base_addr) :
		get_fullo_full(base_addr, endpoint_channel)) {
		dev_err(f_usb20hdc->gadget.dev.parent,
			"%s():endpoint %u buffer  is full.\n",
			__func__, endpoint_channel);
		return 0;
	}

	/* check NULL packet IN transfer for last packet of transaction */
	if ((request->request.length) &&
		(!(request->request.length % endpoint->endpoint.maxpacket)) &&
			(request->request.zero))
		endpoint->null_packet = 1;

	/* calculate transfer data pointer */
	transfer_data = (u32 *)(request->request.buf + request->request.actual);
	prefetch(transfer_data);

	/* calculate this time transfer byte */
	bytes = (request->request.length - request->request.actual) <
			endpoint->endpoint.maxpacket ? request->request.length -
			request->request.actual : endpoint->endpoint.maxpacket;
	dev_dbg(f_usb20hdc->gadget.dev.parent,
		"endpoint %u PIO is setup at length = %u, actual = %u, ",
		endpoint_channel, request->request.length,
		request->request.actual);
	dev_dbg(f_usb20hdc->gadget.dev.parent,
		"max packet = %u, this time = %u.\n",
		endpoint->endpoint.maxpacket,	(u32)bytes);

	/* update actual byte */
	request->request.actual = bytes;

	/* set request execute */
	request->request_execute = 1;

	/* check buffer write bytes */
	if (bytes)
		/* write IN transfer data to buffer */
		write_epbuf(base_addr, endpoint->buffer_address_offset[index],
				transfer_data, bytes);

	/* notify buffer write bytes */
	set_appcnt(base_addr, endpoint_channel *
				F_USB20HDC_EP_BUFFER_COUNT + index, bytes);

	if (endpoint_channel == F_USB20HDC_EP0) {
		/* check new SETUP transfer */
		if (!is_setup_transferred(f_usb20hdc)) {
			/* enable IN transfer & readyi interrupt */
			set_bufwr(base_addr, F_USB20HDC_EP0);
			set_readyi_ready_inten(base_addr, F_USB20HDC_EP0,
				1);
		}
	} else {
		/* check IN transfer end timing and endpoint buffer count */
		if ((endpoint->in_trans_end_timing) && (endpoint->buffers >= 2))
			/*
			 * clear empty interrupt factor
			 * [notice]:It is mandatory processing.
			 */
			clear_readyo_empty_int_clr(base_addr, endpoint_channel);

		/* check DMA transfer usable */
		if (endpoint->usb_dma_channel != -1)
			/*
			 * clear ready interrupt factor
			 * [notice]:It is mandatory processing.
			 */
			clear_readyi_ready_int_clr(base_addr, endpoint_channel);

		/* enable IN transfer & ready interrupt */
		set_bufwr(base_addr, endpoint_channel);
		set_readyi_ready_inten(base_addr, endpoint_channel, 1);
	}

	return 1;
}

static u8 set_out_transfer_pio(struct f_usb20hdc_udc_ep *endpoint)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	void *base_addr = f_usb20hdc->reg_base;
	struct f_usb20hdc_udc_req *request = endpoint->request;
	u8 endpoint_channel = endpoint->endpoint_channel;

	/* check argument */
	if (unlikely((endpoint_channel != F_USB20HDC_EP0) &&
						(!endpoint->endpoint.desc)))
		return 0;

	/* check new SETUP transfer */
	if ((endpoint_channel == F_USB20HDC_EP0) &&
			(is_setup_transferred(f_usb20hdc))) {
		dev_err(f_usb20hdc->gadget.dev.parent,
			"%s():new SETUP tranfer is occurred.\n", __func__);
		return 0;
	}

	dev_dbg(f_usb20hdc->gadget.dev.parent,
		"endpoint %u PIO is setup at length = %u, actual = %u, max packet = %u.\n",
		endpoint_channel, request->request.length,
		request->request.actual, endpoint->endpoint.maxpacket);

	/* set request execute */
	request->request_execute = 1;

	if (endpoint_channel == F_USB20HDC_EP0) {
		/* check new SETUP transfer */
		if (!is_setup_transferred(f_usb20hdc)) {
			/* enable readyo interrupt */
			set_readyo_empty_inten(base_addr, F_USB20HDC_EP0,
				1);
			set_bufrd(base_addr, F_USB20HDC_EP0);
		}
	} else {
		/* check DMA transfer usable */
		if (endpoint->usb_dma_channel != -1) {
			/*
			 * clear ready interrupt factor
			 * [notice]:It is mandatory processing.
			 */
			set_nackresp(base_addr, endpoint_channel, 1);
			if (get_emptyo_empty(base_addr, endpoint_channel))
				clear_readyi_ready_int_clr(base_addr,
							endpoint_channel);
			set_nackresp(base_addr, endpoint_channel, 0);
		}

		/* enable ready interrupt */
		set_readyi_ready_inten(base_addr, endpoint_channel, 1);
	}

	return 1;
}

static void abort_in_transfer_pio(struct f_usb20hdc_udc_ep *endpoint, u8 init)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	void *base_addr = f_usb20hdc->reg_base;
	u8 endpoint_channel = endpoint->endpoint_channel;

	/* disable endpoint interrupt */
	if (endpoint_channel == F_USB20HDC_EP0) {
		set_readyi_ready_inten(base_addr, F_USB20HDC_EP0, 0);
	} else {
		set_readyi_ready_inten(base_addr, endpoint_channel, 0);
		set_readyo_empty_inten(base_addr, endpoint_channel, 0);
	}

	/* clear NULL packet transfer */
	endpoint->null_packet = 0;

	if (!init)
		return;

	/* initialize endpoint */
	initialize_endpoint(endpoint, 1, 0, 0);

	return;
}

static void abort_out_transfer_pio(struct f_usb20hdc_udc_ep *endpoint, u8 init)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	void *base_addr = f_usb20hdc->reg_base;
	u8 endpoint_channel = endpoint->endpoint_channel;

	/* disable endpoint interrupt */
	endpoint_channel == F_USB20HDC_EP0 ?
		set_readyo_empty_inten(base_addr, F_USB20HDC_EP0, 0) :
		set_readyi_ready_inten(base_addr, endpoint_channel, 0);

	if (!init)
		return;

	/* initialize endpoint */
	initialize_endpoint(endpoint, 1, 0, 0);

	return;
}

static u8 end_in_transfer_pio(struct f_usb20hdc_udc_ep *endpoint)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	void *base_addr = f_usb20hdc->reg_base;
	struct f_usb20hdc_udc_req *request = endpoint->request;
	u8 endpoint_channel = endpoint->endpoint_channel;
	u32 *transfer_data;
	u8 index = endpoint_channel == F_USB20HDC_EP0 ? 0 :
					get_appptr(base_addr, endpoint_channel);
	u32 bytes;

	/* check empty wait */
	if ((endpoint_channel != F_USB20HDC_EP0) &&
		(get_readyo_empty_inten(base_addr, endpoint_channel))) {
		/* complete request */
		abort_in_transfer_pio(endpoint, 0);
		return 1;
	}

	/* check transfer remain byte */
	if (!(request->request.length - request->request.actual)) {
		/* check NULL packet IN transfer for last packet transaction */
		if (endpoint->null_packet) {
			if (endpoint_channel == F_USB20HDC_EP0) {
				/* check new SETUP transfer */
				if (is_setup_transferred(f_usb20hdc)) {
					abort_in_transfer_pio(endpoint, 0);
					return 0;
				}
				/* notify buffer write bytes */
				set_appcnt(base_addr, 0, 0);

				/* enable IN transfer */
				set_bufwr(base_addr, F_USB20HDC_EP0);
			} else {
				/* notify buffer write bytes */
				set_appcnt(base_addr, endpoint_channel *
						F_USB20HDC_EP_BUFFER_COUNT +
							index, 0);

				/* enable IN transfer */
				set_bufwr(base_addr, endpoint_channel);
			}
			endpoint->null_packet = 0;
			return 0;
		} else {
			/* check IN transfer end timing and ep buffer count */
			if ((endpoint->in_trans_end_timing) &&
				(endpoint->buffers >= 2)) {
				/* enable empty interrupt */
				set_readyo_empty_inten(base_addr,
							endpoint_channel, 1);
				return 0;
			}

			/* complete request */
			abort_in_transfer_pio(endpoint, 0);
			return 1;
		}
	}

	/* check transfer data setup */
	if (endpoint_channel == F_USB20HDC_EP0 ?
		get_fulli(base_addr) :
		get_fullo_full(base_addr, endpoint_channel)) {
		/* abort IN transfer */
		abort_in_transfer_pio(endpoint,
				endpoint_channel == F_USB20HDC_EP0 ? 0 : 1);
		dev_err(f_usb20hdc->gadget.dev.parent,
			"%s():endpoint %u buffer  is full.\n",
			__func__, endpoint_channel);
		return 0;
	}

	/* calculate transfer data pointer */
	transfer_data = (u32 *)(request->request.buf + request->request.actual);
	prefetch(transfer_data);

	/* calculate this time transfer byte */
	bytes = (request->request.length - request->request.actual) <
			endpoint->endpoint.maxpacket ? request->request.length -
			request->request.actual : endpoint->endpoint.maxpacket;
	dev_dbg(f_usb20hdc->gadget.dev.parent,
		"endpoint %u PIO is setup at length = %u, actual = %u, ",
		endpoint_channel, request->request.length,
		request->request.actual);
	dev_dbg(f_usb20hdc->gadget.dev.parent,
		"max packet = %u, this time = %u.\n",
		endpoint->endpoint.maxpacket,	(u32)bytes);

	/* update actual bytes */
	request->request.actual += bytes;

	/* check buffer write bytes */
	if (bytes)
		/* write IN transfer data to buffer */
		write_epbuf(base_addr, endpoint->buffer_address_offset[index],
							transfer_data, bytes);

	/* notify buffer write bytes */
	set_appcnt(base_addr, endpoint_channel *
				F_USB20HDC_EP_BUFFER_COUNT + index, bytes);

	if (endpoint_channel == F_USB20HDC_EP0) {
		/* check new SETUP transfer */
		if (!is_setup_transferred(f_usb20hdc))
			/* enable IN transfer */
			set_bufwr(base_addr, F_USB20HDC_EP0);
	} else {
		/* enable IN transfer */
		set_bufwr(base_addr, endpoint_channel);
	}

	return 0;
}

static u8 end_out_transfer_pio(struct f_usb20hdc_udc_ep *endpoint)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	void *base_addr = f_usb20hdc->reg_base;
	struct f_usb20hdc_udc_req *request = endpoint->request;
	u8 endpoint_channel = endpoint->endpoint_channel;
	u32 *transfer_data;
	u32 bytes;
	u8 index = endpoint_channel == F_USB20HDC_EP0 ? 1 :
					get_appptr(base_addr, endpoint_channel);

	/* check transfer data read enable */
	if ((endpoint_channel != F_USB20HDC_EP0) &&
		get_emptyo_empty(base_addr, endpoint_channel)) {
		/* abort OUT transfer */
		abort_out_transfer_pio(endpoint, 1);
		dev_err(f_usb20hdc->gadget.dev.parent,
			"%s():endpoint %u is empty.%d\n",
			__func__, endpoint_channel,
			get_nack_inten(base_addr, endpoint_channel));

		return 0;
	}

	/* calculate transfer data pointer */
	transfer_data = (u32 *)(request->request.buf + request->request.actual);
	prefetch(transfer_data);

	/* get OUT transfer byte */
	bytes = get_phycnt(base_addr, endpoint_channel *
					F_USB20HDC_EP_BUFFER_COUNT + index);
	if (request->request.length < (request->request.actual + bytes))
		bytes = request->request.length - request->request.actual;
	dev_dbg(f_usb20hdc->gadget.dev.parent,
		"endpoint %u PIO is end at length = %u, actual = %u, ",
		endpoint_channel, request->request.length,
		request->request.actual);
	dev_dbg(f_usb20hdc->gadget.dev.parent,
		"max packet = %u, this time = %u.\n",
		endpoint->endpoint.maxpacket, (u32)bytes);

	/* update actual bytes */
	request->request.actual += bytes;

	/* check buffer read bytes */
	if (bytes)
		/* read OUT transfer data from buffer */
		read_epbuf(base_addr, endpoint->buffer_address_offset[index],
							transfer_data, bytes);

	if (endpoint_channel != F_USB20HDC_EP0)
		set_bufrd(base_addr, endpoint_channel); /* enable next OUT transfer */

	/* check transfer request complete */
	if ((request->request.length <= request->request.actual) ||
		(bytes % endpoint->endpoint.maxpacket) || (!bytes)) {
		/* complete request */
		abort_out_transfer_pio(endpoint, 0);
		return 1;
	}

	if (endpoint_channel == F_USB20HDC_EP0) {
		/* check new SETUP transfer */
		if (!is_setup_transferred(f_usb20hdc))
			/* enable next OUT transfer */
			set_bufrd(base_addr, F_USB20HDC_EP0);
	}

	return 0;
}

static u8 set_in_transfer(struct f_usb20hdc_udc_ep *endpoint)
{
	static u8 (* const set_in_transfer_function[])(
						struct f_usb20hdc_udc_ep *) = {
		set_in_transfer_pio,
		set_in_transfer_dma,
	};
	struct f_usb20hdc_udc_req *request = endpoint->request;
	u8 index = 1;
	u8 result;

	/* check NULL packet transfer */
	if (!request->request.length)
		return set_in_transfer_pio(endpoint);

	/* check DMA transfer usable */
	if (!is_dma_transfer_usable(endpoint)) {
		if (!is_pio_transfer_auto_change_usable(endpoint))
			return 0;
		index = 0;
	}

	/* get IN transfer process result */
	result = set_in_transfer_function[index](endpoint);

	/* check PIO transfer auto change usable */
	if ((!result) && (index == 1) &&
				(is_pio_transfer_auto_change_usable(endpoint)))
		return set_in_transfer_pio(endpoint);

	return result;
}

static u8 set_out_transfer(struct f_usb20hdc_udc_ep *endpoint)
{
	static u8 (* const set_out_transfer_function[])(
						struct f_usb20hdc_udc_ep *) = {
		set_out_transfer_pio,
		set_out_transfer_dma,
	};
	struct f_usb20hdc_udc_req *request = endpoint->request;
	u8 index = 1;
	u8 result;

	/* check NULL packet transfer */
	if (!request->request.length)
		return set_out_transfer_pio(endpoint);

	/* check DMA transfer usable */
	if (!is_dma_transfer_usable(endpoint)) {
		if (!is_pio_transfer_auto_change_usable(endpoint))
			return 0;
		index = 0;
	}

	/* get OUT transfer process result */
	result = set_out_transfer_function[index](endpoint);

	/* check PIO transfer auto change usable */
	if ((!result) && (index == 1) &&
				(is_pio_transfer_auto_change_usable(endpoint)))
		return set_out_transfer_pio(endpoint);

	return result;
}

static void abort_in_transfer(struct f_usb20hdc_udc_ep *endpoint, u8 initalize)
{
	static void (* const abort_in_transfer_function[])(
					struct f_usb20hdc_udc_ep *, u8) = {
		abort_in_transfer_pio,
		abort_in_transfer_dma,
	};
	return abort_in_transfer_function[endpoint->dma_transfer ?
						1 : 0](endpoint, initalize);
}

static void abort_out_transfer(struct f_usb20hdc_udc_ep *endpoint, u8 initalize)
{
	static void (* const abort_out_transfer_function[])(
					struct f_usb20hdc_udc_ep *, u8) = {
		abort_out_transfer_pio,
		abort_out_transfer_dma,
	};
	return abort_out_transfer_function[endpoint->dma_transfer ?
						1 : 0](endpoint, initalize);
}

static u8 end_in_transfer(struct f_usb20hdc_udc_ep *endpoint)
{
	static u8 (* const end_in_transfer_function[])(
						struct f_usb20hdc_udc_ep *) = {
		end_in_transfer_pio,
		end_in_transfer_dma,
	};
	return end_in_transfer_function[endpoint->dma_transfer ?
							1 : 0](endpoint);
}

static u8 end_out_transfer(struct f_usb20hdc_udc_ep *endpoint)
{
	static u8 (* const end_out_transfer_function[])(
						struct f_usb20hdc_udc_ep *) = {
		end_out_transfer_pio,
		end_out_transfer_dma,
	};
	return end_out_transfer_function[endpoint->dma_transfer ?
							1 : 0](endpoint);
}

static void halt_transfer(struct f_usb20hdc_udc_ep *endpoint, u8 halt,
	u8 force_halt)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	void *base_addr = f_usb20hdc->reg_base;
	struct f_usb20hdc_udc_req *request;
	u8 endpoint_channel = endpoint->endpoint_channel;

	/* check isochronous endpoint */
	if (endpoint->transfer_type == USB_ENDPOINT_XFER_ISOC)
		return;

	if (halt) {
		if (endpoint_channel == F_USB20HDC_EP0) {
			/* check new SETUP transfer */
			if (is_setup_transferred(f_usb20hdc))
				return;
		}

		/* set transfer halt */
		endpoint->halt = 1;
		if (force_halt)
			endpoint->force_halt = 1;

		/* check endpoint x halt clear */
		if (!get_stall(base_addr, endpoint_channel))
			/* halt endpoint x */
			set_stall_set(base_addr, endpoint_channel);
	} else {
		/* check force halt */
		if (endpoint->force_halt) {
			/* always clear endpoint x data toggle bit */
			set_toggle_clear(base_addr, endpoint_channel);
			return;
		}

		/* check endpoint x halt */
		if (get_stall(base_addr, endpoint_channel))
			/* clear endpoint x halt */
			set_stall_clear(base_addr, endpoint_channel);

		/* always clear endpoint x data toggle bit */
		set_toggle_clear(base_addr, endpoint_channel);

		/* clear transfer halt */
		endpoint->halt = 0;

		/* check next queue empty */
		if (list_empty(&endpoint->queue))
			return;

		/* get next request */
		request = list_entry(endpoint->queue.next,
					struct f_usb20hdc_udc_req, queue);

		/* check the got next request is under current execution */
		if (request->request_execute)
			return;

		/* save request */
		endpoint->request = request;

		/* set endpoint x transfer request */
		if (!(endpoint->transfer_direction ? set_in_transfer(endpoint) :
						set_out_transfer(endpoint))) {
			dev_err(f_usb20hdc->gadget.dev.parent,
				"%s():%s of endpoint %u is failed.\n",
				__func__, endpoint->transfer_direction ?
				"set_in_transfer()" : "set_out_transfer()",
				endpoint_channel);
			dequeue_all_transfer_request(endpoint, -EL2HLT);
		}
	}

	return;
}

static u8 respond_standard_request_get_status(
	struct f_usb20hdc_udc_ep *endpoint, struct usb_ctrlrequest ctrlreq)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	void *base_addr = f_usb20hdc->reg_base;
	u16 value;
	struct device *dev = f_usb20hdc->dev;

	/* check reqest target */
	switch (ctrlreq.bRequestType & USB_RECIP_MASK) {
	case USB_RECIP_DEVICE:
		/* check request parameter */
		if ((!(ctrlreq.bRequestType & USB_DIR_IN)) ||
			(ctrlreq.wValue) || (ctrlreq.wIndex) ||
			(ctrlreq.wLength != 2))
			return 0;

		/* check device state */
		switch (f_usb20hdc->device_state) {
		case USB_STATE_ADDRESS:
		case USB_STATE_CONFIGURED:
			break;
		default:
			dev_dbg(dev, "%s (%d): abnormal device_state.\n",
				__func__, __LINE__);
			return 0;
		}

		/* create IN tranfer data */
		value = f_usb20hdc->selfpowered ?
					(1 << USB_DEVICE_SELF_POWERED) : 0;
		value |= get_enrmtwkup(base_addr) ?
					(1 << USB_DEVICE_REMOTE_WAKEUP) : 0x0;

		/* write IN transfer data to buffer */
		write_epbuf(base_addr, endpoint->buffer_address_offset[0],
							(u32 *)&value, 2);

		/* notify buffer write bytes */
		set_appcnt(base_addr, 0, 2);

		/* check new SETUP transfer */
		if (!is_setup_transferred(f_usb20hdc))
			/* enable IN transfer & readyi interrupt */
			set_bufwr(base_addr, F_USB20HDC_EP0);
		break;
	case USB_RECIP_INTERFACE:
		/* check request parameter */
		if ((!(ctrlreq.bRequestType & USB_DIR_IN)) ||
			(ctrlreq.wValue) || (ctrlreq.wLength != 2))
			return 0;

		/* check device state */
		switch (f_usb20hdc->device_state) {
		case USB_STATE_ADDRESS:
		case USB_STATE_CONFIGURED:
			break;
		default:
			dev_dbg(dev, "%s (%d): abnormal device_state.\n",
				__func__, __LINE__);
			return 0;
		}

		/* create IN tranfer data */
		value = 0x0000;

		/* write IN transfer data to buffer */
		write_epbuf(base_addr, endpoint->buffer_address_offset[0],
							(u32 *)&value, 2);

		/* notify buffer write bytes */
		set_appcnt(base_addr, 0, 2);

		/* check new SETUP transfer */
		if (!is_setup_transferred(f_usb20hdc))
			/* enable IN transfer & readyi interrupt */
			set_bufwr(base_addr, F_USB20HDC_EP0);
		break;
	case USB_RECIP_ENDPOINT:
		/* check request parameter */
		if ((!(ctrlreq.bRequestType & USB_DIR_IN)) ||
			(ctrlreq.wValue) || (ctrlreq.wLength != 2))
			return 0;

		/* check device state */
		switch (f_usb20hdc->device_state) {
		case USB_STATE_ADDRESS:
		case USB_STATE_CONFIGURED:
			break;
		default:
			dev_dbg(dev, "%s (%d): abnormal device_state.\n",
				__func__, __LINE__);
			return 0;
		}

		/* create IN tranfer data */
		value = get_stall(base_addr, ctrlreq.wIndex & 0xf) ?
								0x0001 : 0x0000;

		/* write IN transfer data to buffer */
		write_epbuf(base_addr, endpoint->buffer_address_offset[0],
							(u32 *)&value, 2);

		/* notify buffer write bytes */
		set_appcnt(base_addr, 0, 2);

		/* check new SETUP transfer */
		if (!is_setup_transferred(f_usb20hdc))
			/* enable IN transfer & readyi interrupt */
			set_bufwr(base_addr, F_USB20HDC_EP0);
		break;
	default:
		return 0;
	}

	return 1;
}

static u8 respond_standard_request_clear_feature(
	struct f_usb20hdc_udc_ep *endpoint, struct usb_ctrlrequest ctrlreq)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	int result;
	struct device *dev = f_usb20hdc->dev;

	/* check reqest target */
	switch (ctrlreq.bRequestType & USB_RECIP_MASK) {
	case USB_RECIP_DEVICE:
		/* check request parameter */
		if ((ctrlreq.bRequestType & USB_DIR_IN) ||
			(ctrlreq.wValue != USB_DEVICE_REMOTE_WAKEUP) ||
				(ctrlreq.wIndex & 0xff) || (ctrlreq.wLength))
			return 0;

		/* check device state */
		switch (f_usb20hdc->device_state) {
		case USB_STATE_ADDRESS:
		case USB_STATE_CONFIGURED:
			break;
		default:
			dev_dbg(dev, "%s (%d): abnormal device_state.\n",
				__func__, __LINE__);
			return 0;
		}

		/* disable remote wakeup */
		set_enrmtwkup(f_usb20hdc->reg_base, 0);
		break;
	case USB_RECIP_INTERFACE:
		/* check request parameter */
		if ((ctrlreq.bRequestType & USB_DIR_IN) ||
			(ctrlreq.wLength))
			return 0;

		/* check device state */
		switch (f_usb20hdc->device_state) {
		case USB_STATE_ADDRESS:
		case USB_STATE_CONFIGURED:
			break;
		default:
			dev_dbg(dev, "%s (%d): abnormal device_state.\n",
				__func__, __LINE__);
			return 0;
		}

		/* check SETUP transfer callback to gadget driver */
		if ((!f_usb20hdc->gadget_driver) ||
			(!f_usb20hdc->gadget_driver->setup))
			return 0;

		/* notify SETUP transfer to gadget driver */
		spin_unlock(&f_usb20hdc->lock);
		result = f_usb20hdc->gadget_driver->setup(&f_usb20hdc->gadget,
								&ctrlreq);
		spin_lock(&f_usb20hdc->lock);
		if (result < 0) {
			dev_err(dev, "%s():%s of setup() is failed at %d.\n",
				__func__,
				f_usb20hdc->gadget_driver->driver.name, result);
			return 0;
		}

		/* check gadget driver processing result */
		if (result) {
			/* delay NULL packet transfer for status stage */
			f_usb20hdc->ctrl_status_delay = 1;
			f_usb20hdc->ctrl_pri_dir =
				(ctrlreq.bRequestType & USB_DIR_IN) ||
					(!ctrlreq.wLength) ? 1 : 0;
		}
		break;
	case USB_RECIP_ENDPOINT:
		/* check request parameter */
		if ((ctrlreq.bRequestType & USB_DIR_IN) ||
			(ctrlreq.wValue != USB_ENDPOINT_HALT) ||
				(ctrlreq.wLength))
			return 0;

		/* check device state */
		switch (f_usb20hdc->device_state) {
		case USB_STATE_ADDRESS:
		case USB_STATE_CONFIGURED:
			break;
		default:
			dev_dbg(dev, "%s (%d): abnormal device_state.\n",
				__func__, __LINE__);
			return 0;
		}

		/* clear transfer halt */
		halt_transfer(&f_usb20hdc->endpoint[ctrlreq.wIndex & 0xf],
			0, 0);
		break;
	default:
		return 0;
	}

	return 1;
}

static u8 respond_standard_request_set_feature(
	struct f_usb20hdc_udc_ep *endpoint, struct usb_ctrlrequest ctrlreq)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	void *base_addr = f_usb20hdc->reg_base;
	int result;
	struct device *dev = f_usb20hdc->dev;

	/* check reqest target */
	switch (ctrlreq.bRequestType & USB_RECIP_MASK) {
	case USB_RECIP_DEVICE:
		/* check request parameter */
		if ((ctrlreq.bRequestType & USB_DIR_IN) ||
			(ctrlreq.wIndex & 0xff) || (ctrlreq.wLength))
			return 0;

		/* check request parameter of wValue */
		switch (ctrlreq.wValue) {
		case USB_DEVICE_REMOTE_WAKEUP:
			/* check device state */
			switch (f_usb20hdc->device_state) {
			case USB_STATE_ADDRESS:
			case USB_STATE_CONFIGURED:
				break;
			default:
				dev_dbg(dev, "%s (%d): abnormal device_state.\n",
					__func__, __LINE__);
				return 0;
			}

			/* enable remote wakeup */
			set_enrmtwkup(base_addr, 1);
			break;
		case USB_DEVICE_TEST_MODE:
			/* check device state */
			switch (f_usb20hdc->device_state) {
			case USB_STATE_DEFAULT:
			case USB_STATE_ADDRESS:
			case USB_STATE_CONFIGURED:
				break;
			default:
				dev_dbg(dev, "%s (%d): abnormal device_state.\n",
					__func__, __LINE__);
				return 0;
			}

			/* check test selector */
			f_usb20hdc->test_selector = (ctrlreq.wIndex >> 8) & 0xff;
			break;
		default:
			return 0;
		}
		break;
	case USB_RECIP_INTERFACE:
		/* check request parameter */
		if ((ctrlreq.bRequestType & USB_DIR_IN) || (ctrlreq.wLength))
			return 0;

		/* check device state */
		switch (f_usb20hdc->device_state) {
		case USB_STATE_ADDRESS:
		case USB_STATE_CONFIGURED:
			break;
		default:
			dev_dbg(dev, "%s (%d): abnormal device_state.\n",
				__func__, __LINE__);
			return 0;
		}

		/* check SETUP transfer callback to gadget driver */
		if ((!f_usb20hdc->gadget_driver) ||
					(!f_usb20hdc->gadget_driver->setup))
			return 0;

		/* notify SETUP transfer to gadget driver */
		spin_unlock(&f_usb20hdc->lock);
		result = f_usb20hdc->gadget_driver->setup(&f_usb20hdc->gadget,
								&ctrlreq);
		spin_lock(&f_usb20hdc->lock);
		if (result < 0) {
			dev_err(dev, "%s():%s of setup() is failed at %d.\n",
				__func__,
				f_usb20hdc->gadget_driver->driver.name, result);
			return 0;
		}

		/* check gadget driver processing result */
		if (result) {
			/* delay NULL packet transfer for status stage */
			f_usb20hdc->ctrl_status_delay = 1;
			f_usb20hdc->ctrl_pri_dir =
				(ctrlreq.bRequestType & USB_DIR_IN) ||
					(!ctrlreq.wLength) ? 1 : 0;
		}
		break;
	case USB_RECIP_ENDPOINT:
		/* check request parameter */
		if ((ctrlreq.bRequestType & USB_DIR_IN) ||
			(ctrlreq.wValue != USB_ENDPOINT_HALT) ||
				(ctrlreq.wLength))
			return 0;

		/* check device state */
		switch (f_usb20hdc->device_state) {
		case USB_STATE_ADDRESS:
		case USB_STATE_CONFIGURED:
			break;
		default:
			dev_dbg(dev, "%s (%d): abnormal device_state.\n",
				__func__, __LINE__);
			return 0;
		}

		/* halt transfer */
		halt_transfer(&f_usb20hdc->endpoint[ctrlreq.wIndex & 0xf],
			1, 0);
		break;
	default:
		return 0;
	}

	return 1;
}

static u8 respond_standard_request_set_address(
	struct f_usb20hdc_udc_ep *endpoint, struct usb_ctrlrequest ctrlreq)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	void *base_addr = f_usb20hdc->reg_base;
	struct device *dev = f_usb20hdc->dev;

	/* check reqest target and request parameter */
	if (((ctrlreq.bRequestType & USB_RECIP_MASK) != USB_RECIP_DEVICE) ||
		(ctrlreq.bRequestType & USB_DIR_IN) ||
			(ctrlreq.wIndex) || (ctrlreq.wLength))
		return 0;

	/* check device state */
	switch (f_usb20hdc->device_state) {
	case USB_STATE_DEFAULT:
	case USB_STATE_ADDRESS:
		break;
	default:
		dev_dbg(dev, "%s (%d): abnormal device_state.\n",
			__func__, __LINE__);
		return 0;
	}

	/* set address value */
	set_func_addr(base_addr, ctrlreq.wValue & 0xff);

	/* change device state */
	set_device_state(f_usb20hdc, USB_STATE_ADDRESS);

	return 1;
}

static u8 respond_standard_request_get_descriptor(
	struct f_usb20hdc_udc_ep *endpoint, struct usb_ctrlrequest ctrlreq)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	int result;
	struct device *dev = f_usb20hdc->dev;

	/* check reqest target and request parameter */
	if ((((ctrlreq.bRequestType & USB_RECIP_MASK) != USB_RECIP_DEVICE) &&
		((ctrlreq.bRequestType & USB_RECIP_MASK) !=
			USB_RECIP_INTERFACE)) ||
		(!(ctrlreq.bRequestType & USB_DIR_IN)) ||
			(((ctrlreq.wValue >> 8) != USB_DT_STRING) &&
				(ctrlreq.wIndex)))
		return 0;

	/* check device state */
	switch (f_usb20hdc->device_state) {
	case USB_STATE_DEFAULT:
	case USB_STATE_ADDRESS:
	case USB_STATE_CONFIGURED:
		break;
	default:
		dev_dbg(dev, "%s (%d): abnormal device_state.\n",
			__func__, __LINE__);
		return 0;
	}

	/* check SETUP transfer callback to gadget driver */
	if ((!f_usb20hdc->gadget_driver) || (!f_usb20hdc->gadget_driver->setup))
		return 0;

	/* notify SETUP transfer to gadget driver */
	spin_unlock(&f_usb20hdc->lock);
	result = f_usb20hdc->gadget_driver->setup(&f_usb20hdc->gadget,
		&ctrlreq);
	spin_lock(&f_usb20hdc->lock);
	if (result < 0) {
		dev_err(dev, "%s():%s of setup() is failed at %d.\n",
			__func__,
			f_usb20hdc->gadget_driver->driver.name, result);
		return 0;
	}

	/* check gadget driver processing result */
	if (result) {
		/* delay NULL packet transfer for status stage */
		f_usb20hdc->ctrl_status_delay = 1;
		f_usb20hdc->ctrl_pri_dir =
				(ctrlreq.bRequestType & USB_DIR_IN) ||
						(!ctrlreq.wLength) ? 1 : 0;
	}

	return 1;
}

static u8 respond_standard_request_set_descriptor(
	struct f_usb20hdc_udc_ep *endpoint, struct usb_ctrlrequest ctrlreq)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	int result;
	struct device *dev = f_usb20hdc->dev;

	/* check reqest target and request parameter */
	if ((((ctrlreq.bRequestType & USB_RECIP_MASK) != USB_RECIP_DEVICE) &&
		((ctrlreq.bRequestType & USB_RECIP_MASK) !=
			USB_RECIP_INTERFACE)) ||
		(ctrlreq.bRequestType & USB_DIR_IN) ||
		(((ctrlreq.wValue >> 8) != USB_DT_STRING) && (ctrlreq.wIndex)))
		return 0;

	/* check device state */
	switch (f_usb20hdc->device_state) {
	case USB_STATE_ADDRESS:
	case USB_STATE_CONFIGURED:
		break;
	default:
		dev_dbg(dev, "%s (%d): abnormal device_state.\n",
			__func__, __LINE__);
		return 0;
	}

	/* check SETUP transfer callback to gadget driver */
	if ((!f_usb20hdc->gadget_driver) || (!f_usb20hdc->gadget_driver->setup))
		return 0;

	/* notify SETUP transfer to gadget driver */
	spin_unlock(&f_usb20hdc->lock);
	result = f_usb20hdc->gadget_driver->setup(&f_usb20hdc->gadget,
		&ctrlreq);
	spin_lock(&f_usb20hdc->lock);
	if (result < 0) {
		dev_err(dev, "%s():%s of setup() is failed at %d.\n",
			__func__, f_usb20hdc->gadget_driver->driver.name,
			result);
		return 0;
	}

	/* check gadget driver processing result */
	if (result) {
		/* delay NULL packet transfer for status stage */
		f_usb20hdc->ctrl_status_delay = 1;
		f_usb20hdc->ctrl_pri_dir =
				(ctrlreq.bRequestType & USB_DIR_IN) ||
						(!ctrlreq.wLength) ? 1 : 0;
	}

	return 1;
}

static u8 respond_standard_request_get_configuration(
	struct f_usb20hdc_udc_ep *endpoint, struct usb_ctrlrequest ctrlreq)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	void *base_addr = f_usb20hdc->reg_base;
	u8 value;
	int result;
	struct device *dev = f_usb20hdc->dev;

	/* check reqest target and request parameter */
	if (((ctrlreq.bRequestType & USB_RECIP_MASK) != USB_RECIP_DEVICE) ||
		(!(ctrlreq.bRequestType & USB_DIR_IN)) ||
		(ctrlreq.wValue) || (ctrlreq.wIndex) || (ctrlreq.wLength != 1))
		return 0;

	/* check device state */
	switch (f_usb20hdc->device_state) {
	case USB_STATE_ADDRESS:
		/* create IN tranfer data */
		value = 0;

		/* write IN transfer data to buffer */
		write_epbuf(base_addr, endpoint->buffer_address_offset[0],
							(u32 *)&value, 1);

		/* notify buffer write bytes */
		set_appcnt(base_addr, 0, 1);

		/* check new SETUP transfer */
		if (!is_setup_transferred(f_usb20hdc))
			/* enable IN transfer & readyi interrupt */
			set_bufwr(base_addr, F_USB20HDC_EP0);
		break;
	case USB_STATE_CONFIGURED:
		/* check SETUP transfer callback to gadget driver */
		if ((!f_usb20hdc->gadget_driver) ||
			(!f_usb20hdc->gadget_driver->setup))
			return 0;

		/* notify SETUP transfer to gadget driver */
		spin_unlock(&f_usb20hdc->lock);
		result = f_usb20hdc->gadget_driver->setup(&f_usb20hdc->gadget,
								&ctrlreq);
		spin_lock(&f_usb20hdc->lock);
		if (result < 0) {
			dev_err(dev, "%s():%s of setup() is failed at %d.\n",
				__func__,
				f_usb20hdc->gadget_driver->driver.name, result);
			return 0;
		}

		/* check gadget driver processing result */
		if (result) {
			/* delay NULL packet transfer for status stage */
			f_usb20hdc->ctrl_status_delay = 1;
			f_usb20hdc->ctrl_pri_dir =
				(ctrlreq.bRequestType & USB_DIR_IN) ||
					(!ctrlreq.wLength) ? 1 : 0;
		}
		break;
	default:
		dev_dbg(dev, "%s (%d): abnormal device_state.\n",
			__func__, __LINE__);
		return 0;
	}

	return 1;
}

static u8 respond_standard_request_set_configuration(
	struct f_usb20hdc_udc_ep *endpoint, struct usb_ctrlrequest ctrlreq)
{
	u32 counter;
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	void *base_addr = f_usb20hdc->reg_base;
	u8 configure_value = ctrlreq.wValue & 0xff;
	u8 configure_value_last = f_usb20hdc->configure_value_last;
	int result;
	struct device *dev = f_usb20hdc->dev;

	/* check reqest target and request parameter */
	if (((ctrlreq.bRequestType & USB_RECIP_MASK) != USB_RECIP_DEVICE) ||
		(ctrlreq.bRequestType & USB_DIR_IN) ||
			(ctrlreq.wIndex) || (ctrlreq.wLength))
		return 0;

	/* check device state */
	switch (f_usb20hdc->device_state) {
	case USB_STATE_ADDRESS:
	case USB_STATE_CONFIGURED:
		break;
	default:
		dev_dbg(dev, "%s (%d): abnormal device_state.\n",
			__func__, __LINE__);
		return 0;
	}

	f_usb20hdc->configure_value_last = configure_value;
	/* check configure value */
	if (configure_value) {
		/* check configure value change */
		if ((configure_value != configure_value_last) &&
			(configure_value_last)) {
			for (counter = F_USB20HDC_EP1;
				counter < F_USB20HDC_UDC_MAX_EP; counter++)
				/* abort transfer */
				f_usb20hdc->endpoint[
					counter].transfer_direction ?
						abort_in_transfer(
							&f_usb20hdc->endpoint[
								counter], 1) :
						abort_out_transfer(
							&f_usb20hdc->endpoint[
								counter], 1);
		}

		/* change device state */
		set_device_state(f_usb20hdc, USB_STATE_CONFIGURED);
		dev_info(f_usb20hdc->dev, "configure_value=0x%02x, configure_value_last=0x%02x, USB_STATE_CONFIGURED\n", configure_value, configure_value_last);
	} else {
		/* check configure value change */
		if (configure_value != configure_value_last) {
			for (counter = F_USB20HDC_EP1;
				counter < F_USB20HDC_UDC_MAX_EP; counter++)
				/* abort transfer */
				f_usb20hdc->endpoint[
					counter].transfer_direction ?
						abort_in_transfer(
							&f_usb20hdc->endpoint[
								counter], 1) :
						abort_out_transfer(
							&f_usb20hdc->endpoint[
								counter], 1);
		}

		/* change device state */
		set_device_state(f_usb20hdc, USB_STATE_ADDRESS);
		dev_info(f_usb20hdc->dev, "configure_value=0x%02x, configure_value_last=0x%02x, USB_STATE_ADDRESS\n", configure_value, configure_value_last);
	}

	/* dequeue all previous transfer request */
	for (counter = F_USB20HDC_EP0;
		counter < F_USB20HDC_UDC_MAX_EP; counter++)
		dequeue_all_transfer_request(&f_usb20hdc->endpoint[counter],
								-ECONNABORTED);

	/* set configuration value */
	set_dev_configured(base_addr, configure_value ? 1 : 0);

	/* check SETUP transfer callback to gadget driver */
	if ((!f_usb20hdc->gadget_driver) || (!f_usb20hdc->gadget_driver->setup))
		return 0;

	/* notify SETUP transfer to gadget driver */
	spin_unlock(&f_usb20hdc->lock);
	result = f_usb20hdc->gadget_driver->setup(&f_usb20hdc->gadget,
		&ctrlreq);
	spin_lock(&f_usb20hdc->lock);
	if (result < 0) {
		f_usb20hdc->configure_value_last = configure_value_last;
		set_device_state(f_usb20hdc, configure_value_last ?
						USB_STATE_CONFIGURED :
						USB_STATE_ADDRESS);
		set_dev_configured(base_addr, configure_value_last ? 1 : 0);
		dev_err(dev, "%s():%s of setup() is failed at %d.\n",
			__func__,
			f_usb20hdc->gadget_driver->driver.name, result);
		return 0;
	}

	/* check gadget driver processing result */
	if (result) {
		/* delay NULL packet transfer for status stage */
		f_usb20hdc->ctrl_status_delay = 1;
		f_usb20hdc->ctrl_pri_dir =
			(ctrlreq.bRequestType & USB_DIR_IN) ||
						(!ctrlreq.wLength) ? 1 : 0;
	}

	return 1;
}

static u8 respond_standard_request_get_interface(
	struct f_usb20hdc_udc_ep *endpoint, struct usb_ctrlrequest ctrlreq)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	int result;
	struct device *dev = f_usb20hdc->dev;

	/* check reqest target and request parameter */
	if (((ctrlreq.bRequestType & USB_RECIP_MASK) != USB_RECIP_INTERFACE) ||
		(!(ctrlreq.bRequestType & USB_DIR_IN)) ||
			(ctrlreq.wValue) || (ctrlreq.wLength != 1))
		return 0;

	/* check device state */
	switch (f_usb20hdc->device_state) {
	case USB_STATE_CONFIGURED:
		break;
	default:
		dev_dbg(dev, "%s (%d): abnormal device_state.\n",
			__func__, __LINE__);
		return 0;
	}

	/* check SETUP transfer callback to gadget driver */
	if ((!f_usb20hdc->gadget_driver) || (!f_usb20hdc->gadget_driver->setup))
		return 0;

	/* notify SETUP transfer to gadget driver */
	spin_unlock(&f_usb20hdc->lock);
	result = f_usb20hdc->gadget_driver->setup(&f_usb20hdc->gadget,
		&ctrlreq);
	spin_lock(&f_usb20hdc->lock);
	if (result < 0) {
		dev_err(dev,
			"%s():%s of setup() is failed at %d.\n", __func__,
			f_usb20hdc->gadget_driver->driver.name, result);
		return 0;
	}

	/* check gadget driver processing result */
	if (result) {
		/* delay NULL packet transfer for status stage */
		f_usb20hdc->ctrl_status_delay = 1;
		f_usb20hdc->ctrl_pri_dir =
			(ctrlreq.bRequestType & USB_DIR_IN) ||
						(!ctrlreq.wLength) ? 1 : 0;
	}

	return 1;
}

static u8 respond_standard_request_set_interface(
	struct f_usb20hdc_udc_ep *endpoint, struct usb_ctrlrequest ctrlreq)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	int result;
	struct device *dev = f_usb20hdc->dev;

	/* check reqest target and request parameter */
	if (((ctrlreq.bRequestType & USB_RECIP_MASK) != USB_RECIP_INTERFACE) ||
		(ctrlreq.bRequestType & USB_DIR_IN) ||
			(ctrlreq.wLength))
		return 0;

	/* check device state */
	switch (f_usb20hdc->device_state) {
	case USB_STATE_CONFIGURED:
		break;
	default:
		dev_dbg(dev, "%s (%d): abnormal device_state.\n",
			__func__, __LINE__);
		return 0;
	}

	/* check SETUP transfer callback to gadget driver */
	if ((!f_usb20hdc->gadget_driver) || (!f_usb20hdc->gadget_driver->setup))
		return 0;

	/* notify SETUP transfer to gadget driver */
	spin_unlock(&f_usb20hdc->lock);
	result = f_usb20hdc->gadget_driver->setup(&f_usb20hdc->gadget,
		&ctrlreq);
	spin_lock(&f_usb20hdc->lock);
	if (result < 0) {
		dev_err(dev, "%s():%s of setup() is failed at %d.\n", __func__,
			f_usb20hdc->gadget_driver->driver.name, result);
		return 0;
	}

	/* check gadget driver processing result */
	if (result) {
		/* delay NULL packet transfer for status stage */
		f_usb20hdc->ctrl_status_delay = 1;
		f_usb20hdc->ctrl_pri_dir =
				(ctrlreq.bRequestType & USB_DIR_IN) ||
						(!ctrlreq.wLength) ? 1 : 0;
	}

	return 1;
}

static u8 respond_standard_request_sync_frame(
	struct f_usb20hdc_udc_ep *endpoint, struct usb_ctrlrequest ctrlreq)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	int result;
	struct device *dev = f_usb20hdc->dev;

	/* check reqest target and request parameter */
	if (((ctrlreq.bRequestType & USB_RECIP_MASK) != USB_RECIP_ENDPOINT) ||
		(!(ctrlreq.bRequestType & USB_DIR_IN)) ||
			(ctrlreq.wValue) || (ctrlreq.wLength != 2))
		return 0;

	/* check device state */
	switch (f_usb20hdc->device_state) {
	case USB_STATE_CONFIGURED:
		break;
	default:
		dev_dbg(dev, "%s (%d): abnormal device_state.\n",
			__func__, __LINE__);
		return 0;
	}

	/* check SETUP transfer callback to gadget driver */
	if ((!f_usb20hdc->gadget_driver) || (!f_usb20hdc->gadget_driver->setup))
		return 0;

	/* notify SETUP transfer to gadget driver */
	spin_unlock(&f_usb20hdc->lock);
	result = f_usb20hdc->gadget_driver->setup(&f_usb20hdc->gadget,
		&ctrlreq);
	spin_lock(&f_usb20hdc->lock);
	if (result < 0) {
		dev_err(dev, "%s():%s of setup() is failed at %d.\n", __func__,
			f_usb20hdc->gadget_driver->driver.name, result);
		return 0;
	}

	/* check gadget driver processing result */
	if (result) {
		/* delay NULL packet transfer for status stage */
		f_usb20hdc->ctrl_status_delay = 1;
		f_usb20hdc->ctrl_pri_dir =
				(ctrlreq.bRequestType & USB_DIR_IN) ||
						(!ctrlreq.wLength) ? 1 : 0;
	}

	return 1;
}

static u8 respond_standard_request_undefined(
	struct f_usb20hdc_udc_ep *endpoint, struct usb_ctrlrequest ctrlreq)
{
	/* always error return */
	return 0;
}

static u8 respond_class_request(
	struct f_usb20hdc_udc_ep *endpoint, struct usb_ctrlrequest ctrlreq)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	int result;
	struct device *dev = f_usb20hdc->dev;

	/* check SETUP transfer callback to gadget driver */
	if ((!f_usb20hdc->gadget_driver) || (!f_usb20hdc->gadget_driver->setup))
		return 0;

	/* notify SETUP transfer to gadget driver */
	spin_unlock(&f_usb20hdc->lock);
	result = f_usb20hdc->gadget_driver->setup(&f_usb20hdc->gadget,
		&ctrlreq);
	spin_lock(&f_usb20hdc->lock);
	if (result < 0) {
		dev_err(dev, "%s():%s of setup() is failed at %d.\n", __func__,
			f_usb20hdc->gadget_driver->driver.name, result);
		return 0;
	}

	/* check gadget driver processing result */
	if (result == 0) {
		/* delay NULL packet transfer for status stage */
		f_usb20hdc->ctrl_status_delay = 1;
		f_usb20hdc->ctrl_pri_dir =
				(ctrlreq.bRequestType & USB_DIR_IN) ||
						(!ctrlreq.wLength) ? 1 : 0;
	}

	return 1;
}

static u8 respond_vendor_request(
	struct f_usb20hdc_udc_ep *endpoint, struct usb_ctrlrequest ctrlreq)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	int result;

	/* check SETUP transfer callback to gadget driver */
	if ((!f_usb20hdc->gadget_driver) || (!f_usb20hdc->gadget_driver->setup))
		return 0;

	/* notify SETUP transfer to gadget driver */
	spin_unlock(&f_usb20hdc->lock);
	result = f_usb20hdc->gadget_driver->setup(&f_usb20hdc->gadget,
		&ctrlreq);
	spin_lock(&f_usb20hdc->lock);
	if (result < 0) {
		dev_err(f_usb20hdc->gadget.dev.parent,
			"%s():%s of setup() is failed at %d.\n", __func__,
			f_usb20hdc->gadget_driver->driver.name, result);
		return 0;
	}

	/* check gadget driver processing result */
	if (result) {
		/* delay NULL packet transfer for status stage */
		f_usb20hdc->ctrl_status_delay = 1;
		f_usb20hdc->ctrl_pri_dir =
				(ctrlreq.bRequestType & USB_DIR_IN) ||
						(!ctrlreq.wLength) ? 1 : 0;
	}

	return 1;
}

static void enable_host_connect(struct f_usb20hdc_udc *f_usb20hdc, u8 enable)
{
	u32 counter;
	struct f_usb20hdc_udc_ep *endpoint;

	if (enable) {
		/* initialize F_USB20HDC controller */
		initialize_udc_controller(f_usb20hdc, 0);

		/* enable device mode */
		set_host_en(f_usb20hdc->reg_base, 0);
		set_dev_en(f_usb20hdc->reg_base, 1);
		set_dev_int_mode(f_usb20hdc->reg_base, 1);
		set_dev_addr_load_mode(f_usb20hdc->reg_base, 1);

		/* enable interrupt factor */
		set_dev_inten(f_usb20hdc->reg_base, 1);
		set_phy_err_inten(f_usb20hdc->reg_base, 1);
#ifdef GADGET_CONNECTION_BUG
		clear_suspende_int(f_usb20hdc->reg_base);
		clear_suspendb_int(f_usb20hdc->reg_base);
		set_suspende_inten(f_usb20hdc->reg_base, 0);
		set_suspendb_inten(f_usb20hdc->reg_base, 0);
#else
		set_suspende_inten(f_usb20hdc->reg_base, 1);
		set_suspendb_inten(f_usb20hdc->reg_base, 1);
#endif
		set_setup_inten(f_usb20hdc->reg_base, 1);
		set_usbrste_inten(f_usb20hdc->reg_base, 1);
		set_usbrstb_inten(f_usb20hdc->reg_base, 1);

		/* change device state */
		set_device_state(f_usb20hdc, USB_STATE_POWERED);

		/* initialize endpoint configure data */
		initialize_endpoint_configure(f_usb20hdc);

		/* pull-up D+ terminal */
		dev_info(f_usb20hdc->gadget.dev.parent,
						"D+ terminal is pull-up.\n");
		enable_dplus_line_pullup(f_usb20hdc->reg_base, 1);
		dev_info(f_usb20hdc->gadget.dev.parent,
			"DP:%d, DM:%d",
			get_linestate(f_usb20hdc->reg_base) & 0x00000001,
			get_linestate(f_usb20hdc->reg_base) & 0x00000002);
	} else {
		/* disable interrupt factor */
		set_dev_inten(f_usb20hdc->reg_base, 0);
		set_phy_err_inten(f_usb20hdc->reg_base, 0);
		set_suspende_inten(f_usb20hdc->reg_base, 0);
		set_suspendb_inten(f_usb20hdc->reg_base, 0);
		set_setup_inten(f_usb20hdc->reg_base, 0);
		set_usbrste_inten(f_usb20hdc->reg_base, 0);
		set_usbrstb_inten(f_usb20hdc->reg_base, 0);
		set_vbus_vld_fen(f_usb20hdc->reg_base, 0);
		set_vbus_vld_ren(f_usb20hdc->reg_base, 0);

		/* clear interrupt factor */
		clear_phy_err_int(f_usb20hdc->reg_base);
		clear_cmd_int(f_usb20hdc->reg_base);
		clear_suspende_int(f_usb20hdc->reg_base);
		clear_suspendb_int(f_usb20hdc->reg_base);
		clear_sof_int(f_usb20hdc->reg_base);
		clear_setup_int(f_usb20hdc->reg_base);
		clear_usbrste_int(f_usb20hdc->reg_base);
		clear_usbrstb_int(f_usb20hdc->reg_base);
		clear_status_ok_int(f_usb20hdc->reg_base);
		clear_status_ng_int(f_usb20hdc->reg_base);

		/* change device state */
		set_device_state(f_usb20hdc, USB_STATE_NOTATTACHED);

		/* pull-down D+ terminal */
		dev_dbg(f_usb20hdc->gadget.dev.parent,
						"D+ terminal is pull-down.\n");
		enable_dplus_line_pullup(f_usb20hdc->reg_base, 0);

		/* abort previous transfer */
		abort_in_transfer(&f_usb20hdc->endpoint[F_USB20HDC_EP0], 0);
		abort_out_transfer(&f_usb20hdc->endpoint[
			F_USB20HDC_EP0], 0);
		enable_endpoint(&f_usb20hdc->endpoint[F_USB20HDC_EP0], 0);
		for (counter = F_USB20HDC_EP1;
			counter < F_USB20HDC_UDC_MAX_EP; counter++) {
			endpoint = &f_usb20hdc->endpoint[counter];
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
			if ((endpoint->usb_dma_channel != -1) &&
				endpoint->dma_transfer) {
				spin_unlock(&f_usb20hdc->lock);
				hdmac_stop_nowait(f_usb20hdc->dma_data[
				endpoint->usb_dma_channel].hdmac_channel);
				spin_lock(&f_usb20hdc->lock);
			}
#endif
			endpoint->transfer_direction ?
				abort_in_transfer(endpoint, 0) :
				abort_out_transfer(endpoint, 0);
			enable_endpoint(endpoint, 0);
		}

		/* reset F_USB20HDC controller */
#ifdef GADGET_CONNECTION_BUG
		/* reset USB OTG controller */
		f_usb20hdc_soft_reset_ip(f_usb20hdc->f_otg);

		/* initial gadget mode */
		initialize_udc_controller(f_usb20hdc, 1);
		initialize_endpoint_configure(f_usb20hdc);
#endif
		initialize_udc_controller(f_usb20hdc, 0);

		/* dequeue all previous transfer request */
		for (counter = F_USB20HDC_EP0;
			counter < F_USB20HDC_UDC_MAX_EP; counter++)
			dequeue_all_transfer_request(
				&f_usb20hdc->endpoint[counter], -ESHUTDOWN);

		/* initialize endpoint list data */
		INIT_LIST_HEAD(&f_usb20hdc->gadget.ep0->ep_list);
		INIT_LIST_HEAD(&f_usb20hdc->gadget.ep_list);
		for (counter = F_USB20HDC_EP0;
			counter < F_USB20HDC_UDC_MAX_EP; counter++) {
			list_add_tail(&f_usb20hdc->endpoint[
					counter].endpoint.ep_list,
					counter == F_USB20HDC_EP0 ?
					&f_usb20hdc->gadget.ep0->ep_list :
					&f_usb20hdc->gadget.ep_list);
			INIT_LIST_HEAD(&f_usb20hdc->endpoint[counter].queue);
		}

		/* initialize F_USB20HDC UDC device driver structure data */
		f_usb20hdc->gadget.speed = USB_SPEED_UNKNOWN;
		f_usb20hdc->configure_value_last = 0;
		f_usb20hdc->ctrl_stage = F_USB20HDC_STAGE_SETUP;
		f_usb20hdc->ctrl_pri_dir = 1;
	}

	return;
}

static void enable_communicate(struct f_usb20hdc_udc *f_usb20hdc, u8 enable)
{
	void *base_addr = f_usb20hdc->reg_base;
	struct device *dev = f_usb20hdc->dev;

	dev_dbg(dev, "enable_communicate() is started.(on/off:%d)\n", enable);
#if (F_USB20HDC_UDC_USE_AUTO_STALL_RECOVERY == 1)
	if (!enable)
		/* delete halt transfer error recovery */
		del_timer_sync(&f_usb20hdc->halt_transfer_error_recovery_timer);
#endif
	if (enable) {
		/* get current bus connect */
		f_usb20hdc->bus_connect = is_bus_connected(base_addr);

		/* check bus connect */
		if (f_usb20hdc->bus_connect) {
			dev_info(dev, "enable_communicate(): bus connected.\n");
			/* set bus disconnect detect */
			set_bus_connect_detect(base_addr, 0);

			/* enable host connect */
			enable_host_connect(f_usb20hdc, 1);
		} else {
			dev_info(dev, "enable_communicate(): bus isn't connected.\n");
			/* set bus connect detect */
			set_bus_connect_detect(base_addr, 1);
		}
	} else {
		/* disable host connect */
		enable_host_connect(f_usb20hdc, 0);
	}

	dev_dbg(dev, "enable_communicate() is ended.\n");
	return;
}

static void on_detect_bus_connect(u32 data)
{
	struct f_usb20hdc_udc *f_usb20hdc = (struct f_usb20hdc_udc *)data;
	void *base_addr = f_usb20hdc->reg_base;
	struct device *dev = f_usb20hdc->dev;

	/* get current bus connect */
	f_usb20hdc->bus_connect = is_bus_connected(base_addr);

	/* check bus connect */
	if (f_usb20hdc->bus_connect) {
#if 0
		/* notify resume to gadget driver */
		if ((f_usb20hdc->gadget_driver) &&
			(f_usb20hdc->gadget_driver->resume)) {
			spin_unlock(&f_usb20hdc->lock);
			f_usb20hdc->gadget_driver->resume(&f_usb20hdc->gadget);
			spin_lock(&f_usb20hdc->lock);
		}
#endif
		dev_info(dev, "%s(): connected to host.\n", __func__);
		/* set bus disconnect detect */
		set_bus_connect_detect(base_addr, 0);

		/* check F_USB20HDC UDC device driver register */
		if (f_usb20hdc->device_add) {
			dev_info(dev, "%s(): class driver is connected.\n",
							__func__);
			enable_host_connect(f_usb20hdc, 1);
		}
	} else {
		dev_info(dev, "%s(): no connection to host.\n", __func__);
		/* check F_USB20HDC UDC device driver register */
		if (f_usb20hdc->device_add) {
			dev_info(dev, "%s(): class driver is connected.\n",
						__func__);
			enable_host_connect(f_usb20hdc, 0);
			set_bus_connect_detect(base_addr, 1);

			/* notify disconnect to gadget driver */
			if ((f_usb20hdc->gadget_driver) &&
				(f_usb20hdc->gadget_driver->disconnect)) {
				spin_unlock(&f_usb20hdc->lock);
				dev_info(f_usb20hdc->dev,
					"call gadget_driver->disconnect()\n");
				f_usb20hdc->gadget_driver->disconnect(
							&f_usb20hdc->gadget);
				spin_lock(&f_usb20hdc->lock);
			}
		} else {
			/* set bus connect detect */
			set_bus_connect_detect(base_addr, 1);
		}
	}

	return;
}

static void on_begin_bus_reset(struct f_usb20hdc_udc *f_usb20hdc)
{
	u32 counter;
	struct f_usb20hdc_udc_ep *endpoint;

	dev_dbg(f_usb20hdc->dev, "on_begin_bus_reset() is started\n");
	/* abort previous transfer,initialize and disable all endpoint */
	abort_in_transfer(&f_usb20hdc->endpoint[F_USB20HDC_EP0], 0);
	abort_out_transfer(&f_usb20hdc->endpoint[F_USB20HDC_EP0], 0);
	enable_endpoint(&f_usb20hdc->endpoint[F_USB20HDC_EP0], 0);
	for (counter = F_USB20HDC_EP1;
		counter < F_USB20HDC_UDC_MAX_EP; counter++) {
		endpoint = &f_usb20hdc->endpoint[counter];
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
		if ((endpoint->usb_dma_channel != -1)
						&& endpoint->dma_transfer) {
			spin_unlock(&f_usb20hdc->lock);
			hdmac_stop_nowait(f_usb20hdc->dma_data[
				endpoint->usb_dma_channel].hdmac_channel);
			spin_lock(&f_usb20hdc->lock);
		}
#endif
		endpoint->transfer_direction ?
			abort_in_transfer(endpoint, 0) :
			abort_out_transfer(endpoint, 0);
		enable_endpoint(endpoint, 0);
	}

	/* re-enable endpoint 0 */
	enable_endpoint(&f_usb20hdc->endpoint[F_USB20HDC_EP0], 1);

	/* dequeue all previous transfer request */
	for (counter = F_USB20HDC_EP0;
		counter < F_USB20HDC_UDC_MAX_EP; counter++)
		dequeue_all_transfer_request(&f_usb20hdc->endpoint[counter],
								-ECONNABORTED);

	/* initialize endpoint list data */
	INIT_LIST_HEAD(&f_usb20hdc->gadget.ep0->ep_list);
	INIT_LIST_HEAD(&f_usb20hdc->gadget.ep_list);
	for (counter = F_USB20HDC_EP0;
		counter < F_USB20HDC_UDC_MAX_EP; counter++) {
		list_add_tail(&f_usb20hdc->endpoint[counter].endpoint.ep_list,
				counter == F_USB20HDC_EP0 ?
					&f_usb20hdc->gadget.ep0->ep_list :
					&f_usb20hdc->gadget.ep_list);
		INIT_LIST_HEAD(&f_usb20hdc->endpoint[counter].queue);
	}

#if (F_USB20HDC_UDC_BUS_RESET_NOTIFY == 1)
	/* check configured */
	if (f_usb20hdc->configure_value_last) {
		/* notify dummy disconnect to gadget driver */
		if ((f_usb20hdc->gadget_driver) &&
			(f_usb20hdc->gadget_driver->disconnect)) {
			spin_unlock(&f_usb20hdc->lock);
			f_usb20hdc->gadget_driver->disconnect(
							&f_usb20hdc->gadget);
			spin_lock(&f_usb20hdc->lock);
		}
	}
#endif
	/* initialize F_USB20HDC UDC device driver structure data */
	f_usb20hdc->gadget.speed = USB_SPEED_UNKNOWN;
	f_usb20hdc->configure_value_last = 0;
	f_usb20hdc->ctrl_stage = F_USB20HDC_STAGE_SETUP;
	f_usb20hdc->ctrl_pri_dir = 1;
	dev_dbg(f_usb20hdc->dev, "on_begin_bus_reset() is ended\n");
	return;
}

static void on_end_bus_reset(struct f_usb20hdc_udc *f_usb20hdc)
{
	dev_dbg(f_usb20hdc->dev, "on_end_bus_reset() is started\n");
	/* set current bus speed */
	set_bus_speed(f_usb20hdc);
#ifdef GADGET_CONNECTION_BUG
	clear_suspende_int(f_usb20hdc->reg_base);
	clear_suspendb_int(f_usb20hdc->reg_base);
	set_suspende_inten(f_usb20hdc->reg_base, 1);
	set_suspendb_inten(f_usb20hdc->reg_base, 1);
#endif
	/* change device state */
	set_device_state(f_usb20hdc, USB_STATE_DEFAULT);
	dev_dbg(f_usb20hdc->dev, "on_end_bus_reset() is ended\n");
	return;
}

static void on_suspend(struct f_usb20hdc_udc *f_usb20hdc)
{
	/* check parameter */
	if (f_usb20hdc->gadget.speed == USB_SPEED_UNKNOWN) {
		dev_err(f_usb20hdc->dev, "on_suspend(): err\n");
		return;
	}

	/* change device state */
	set_device_state(f_usb20hdc, USB_STATE_SUSPENDED);

	/* notify suspend to gadget driver */
	if ((f_usb20hdc->gadget_driver) &&
		(f_usb20hdc->gadget_driver->suspend)) {
		spin_unlock(&f_usb20hdc->lock);
		dev_dbg(f_usb20hdc->dev, "call gadget_driver->suspend()\n");
		f_usb20hdc->gadget_driver->suspend(&f_usb20hdc->gadget);
		spin_lock(&f_usb20hdc->lock);
	}
	return;
}

static void on_wakeup(struct f_usb20hdc_udc *f_usb20hdc)
{
	/* check parameter */
	if (f_usb20hdc->gadget.speed == USB_SPEED_UNKNOWN) {
		dev_err(f_usb20hdc->dev, "on_wakeup(): err\n");
		return;
	}

	/* change device state */
	set_device_state(f_usb20hdc, f_usb20hdc->device_state_last);

	/* notify resume to gadget driver */
	if ((f_usb20hdc->gadget_driver) &&
		(f_usb20hdc->gadget_driver->resume)) {
		spin_unlock(&f_usb20hdc->lock);
		dev_info(f_usb20hdc->dev, "call gadget_driver->resume()\n");
		f_usb20hdc->gadget_driver->resume(&f_usb20hdc->gadget);
		spin_lock(&f_usb20hdc->lock);
	}

	return;
}

static void on_end_transfer_sof(struct f_usb20hdc_udc *f_usb20hdc)
{
	/* current non process */
	return;
}

static void on_end_control_setup_transfer(struct f_usb20hdc_udc_ep *endpoint)
{
#define	STANDARD_REQUEST_MAXIMUM	13
	static u8 (* const standard_request_respond_function[])(
			struct f_usb20hdc_udc_ep *, struct usb_ctrlrequest) = {
		respond_standard_request_get_status,
		respond_standard_request_clear_feature,
		respond_standard_request_undefined,
		respond_standard_request_set_feature,
		respond_standard_request_undefined,
		respond_standard_request_set_address,
		respond_standard_request_get_descriptor,
		respond_standard_request_set_descriptor,
		respond_standard_request_get_configuration,
		respond_standard_request_set_configuration,
		respond_standard_request_get_interface,
		respond_standard_request_set_interface,
		respond_standard_request_sync_frame,
	};
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	void *base_addr = f_usb20hdc->reg_base;
	struct usb_ctrlrequest ctrlreq;
	u32 bytes;

	/* get SETUP transfer byte */
	bytes = get_phycnt(base_addr, 1);

	/* check SETUP transfer byte */
	if (bytes != 8) {
		dev_err(f_usb20hdc->gadget.dev.parent,
			"%s():SETUP tranfer byte is mismatch at bytes = %u.\n",
				__func__, (u32)bytes);
		/* check new setup transfer */
		if (!is_setup_transferred(f_usb20hdc))
			/* protocol stall */
			halt_transfer(endpoint, 1, 0);
		return;
	}

	/* clear status stage delay */
	f_usb20hdc->ctrl_status_delay = 0;

	/* clear transfer halt */
	endpoint->halt = 0;
	endpoint->force_halt = 0;
	endpoint->null_packet = 0;

	/* dequeue all previous control transfer request */
	dequeue_all_transfer_request(endpoint, -EPROTO);

	/* read SETUP transfer data from buffer */
	read_epbuf(base_addr, endpoint->buffer_address_offset[1],
							(u32 *)&ctrlreq, bytes);

	/* check new setup transfer */
	if (is_setup_transferred(f_usb20hdc))
		return;

	/* enable next OUT transfer */
	set_bufrd(base_addr, F_USB20HDC_EP0);

	/* disable readyi / readyo interrupt */
	set_readyi_ready_inten(base_addr, F_USB20HDC_EP0, 0);
	set_readyo_empty_inten(base_addr, F_USB20HDC_EP0, 0);

	/* update control transfer stage */
	if (ctrlreq.bRequestType & USB_DIR_IN) {
		f_usb20hdc->ctrl_stage = F_USB20HDC_STAGE_IN_DATA;
		dev_dbg(f_usb20hdc->gadget.dev.parent,
			"next control transfer stage is IN data stage.\n");
	} else {
		if (ctrlreq.wLength) {
			f_usb20hdc->ctrl_stage = F_USB20HDC_STAGE_OUT_DATA;
			dev_dbg(f_usb20hdc->gadget.dev.parent,
				"next control transfer stage is OUT data stage.\n");
		} else {
			f_usb20hdc->ctrl_stage = F_USB20HDC_STAGE_IN_STATUS;
			dev_dbg(f_usb20hdc->gadget.dev.parent,
				"next control transfer stage is IN status stage.\n");
		}
	}

	/* check request type */
	switch (ctrlreq.bRequestType & USB_TYPE_MASK) {
	case USB_TYPE_STANDARD:
		/* process standard request respond */
		if ((ctrlreq.bRequest >= STANDARD_REQUEST_MAXIMUM) ||
			(!standard_request_respond_function[ctrlreq.bRequest](endpoint, ctrlreq))) {
			if (!is_setup_transferred(f_usb20hdc))
				/* protocol stall */
				halt_transfer(endpoint, 1, 0);
		}

		/* check NULL packet IN transfer for status stage delay */
		if (f_usb20hdc->ctrl_status_delay)
			return;
		break;
	case USB_TYPE_CLASS:
		/* process class request respond */
		if (!respond_class_request(endpoint, ctrlreq)) {
			if (!is_setup_transferred(f_usb20hdc))
				/* protocol stall */
				halt_transfer(endpoint, 1, 0);
		}

		/* check NULL packet IN transfer for status stage delay */
		if (f_usb20hdc->ctrl_status_delay)
			return;
		break;
	case USB_TYPE_VENDOR:
		/* process vendor request respond */
		if (!respond_vendor_request(endpoint, ctrlreq)) {
			if (!is_setup_transferred(f_usb20hdc))
				/* protocol stall */
				halt_transfer(endpoint, 1, 0);
		}

		/* check NULL packet IN transfer for status stage delay */
		if (f_usb20hdc->ctrl_status_delay)
			return;
		break;
	default:
		if (!is_setup_transferred(f_usb20hdc))
			/* protocol stall */
			halt_transfer(endpoint, 1, 0);
		break;
	}

	if (is_setup_transferred(f_usb20hdc))
		return;

	/* enable status stage transfer */
	if (ctrlreq.bRequestType & USB_DIR_IN) {
		f_usb20hdc->ctrl_pri_dir = 1;
		set_readyo_empty_inten(base_addr, F_USB20HDC_EP0, 1);
	} else {
		f_usb20hdc->ctrl_pri_dir = ctrlreq.wLength ? 0 : 1;
		set_appcnt(base_addr, 0, 0);
		set_bufwr(base_addr, F_USB20HDC_EP0);
		set_readyi_ready_inten(base_addr, F_USB20HDC_EP0, 1);
	}

	return;
}

static void on_end_control_in_transfer(struct f_usb20hdc_udc_ep *endpoint)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	void *base_addr = f_usb20hdc->reg_base;

	/* process control transfer stage */
	switch (f_usb20hdc->ctrl_stage) {
	case F_USB20HDC_STAGE_IN_DATA:
		/* check new SETUP transfer */
		if (is_setup_transferred(f_usb20hdc))
			break;

		/* check IN transfer continue */
		if (!end_in_transfer_pio(endpoint))
			break;

		/* update control transfer stage */
		f_usb20hdc->ctrl_stage = F_USB20HDC_STAGE_OUT_STATUS;
		dev_dbg(f_usb20hdc->gadget.dev.parent,
			"next control transfer stage is OUT status stage.\n");
		break;
	case F_USB20HDC_STAGE_OUT_DATA:
	case F_USB20HDC_STAGE_IN_STATUS:
		/* update control transfer stage */
		f_usb20hdc->ctrl_stage = F_USB20HDC_STAGE_SETUP;
		dev_dbg(f_usb20hdc->gadget.dev.parent,
			"next control transfer stage is SETUP stage.\n");

		/* TEST MODE */
		switch (f_usb20hdc->test_selector) {
		case 1:
			/* set Test_J */
			set_testj(base_addr, 1);
			break;
		case 2:
			/* set Test_K */
			set_testk(base_addr, 1);
			break;
		case 3:
			/* set Test_SE_NAK */
			set_testse0nack(base_addr, 1);
			/* set dev_addr_load_mode */
			set_dev_addr_load_mode(base_addr, 0);
			/* set func_addr */
			set_func_addr(base_addr, 1);
			break;
		case 4:
			/* set Test_Packet */
			set_testp(base_addr, 1);
			break;
		default:
			break;
		}
		f_usb20hdc->test_selector = 0;
		break;
	case F_USB20HDC_STAGE_OUT_STATUS:
		/* non process */
		break;
	default:
		/* check new SETUP transfer */
		if (!is_setup_transferred(f_usb20hdc))
			/* protocol stall */
			halt_transfer(endpoint, 1, 0);

		/* update control transfer stage */
		f_usb20hdc->ctrl_stage = F_USB20HDC_STAGE_SETUP;
		dev_dbg(f_usb20hdc->gadget.dev.parent,
			"next control transfer stage is SETUP stage.\n");
		break;
	}

	return;
}

static void on_end_control_out_transfer(struct f_usb20hdc_udc_ep *endpoint)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	void *base_addr = f_usb20hdc->reg_base;

	/* process control transfer stage */
	switch (f_usb20hdc->ctrl_stage) {
	case F_USB20HDC_STAGE_OUT_DATA:
		/* check new SETUP transfer */
		if (is_setup_transferred(f_usb20hdc))
			break;

		/* check OUT transfer continue */
		if (!end_out_transfer_pio(endpoint))
			break;

		/* notify request complete */
		notify_transfer_request_complete(endpoint, 0);

		/* update control transfer stage */
		f_usb20hdc->ctrl_stage = F_USB20HDC_STAGE_IN_STATUS;
		if (f_usb20hdc->ctrl_status_delay) {
			f_usb20hdc->ctrl_pri_dir = 1;
			set_appcnt(base_addr, 0, 0);
			set_bufwr(base_addr, F_USB20HDC_EP0);
			set_readyi_ready_inten(base_addr,
				F_USB20HDC_EP0, 1);
		}
		dev_dbg(f_usb20hdc->gadget.dev.parent,
			"next control transfer stage is IN status stage.\n");
		break;
	case F_USB20HDC_STAGE_IN_DATA:
	case F_USB20HDC_STAGE_OUT_STATUS:
		/* update control transfer stage */
		f_usb20hdc->ctrl_stage = F_USB20HDC_STAGE_SETUP;
		dev_dbg(f_usb20hdc->gadget.dev.parent,
			"next control transfer stage is SETUP stage.\n");

		/* notify request complete */
		notify_transfer_request_complete(endpoint, 0);
		break;
	default:
		/* check new SETUP transfer */
		if (!is_setup_transferred(f_usb20hdc))
			/* protocol stall */
			halt_transfer(endpoint, 1, 0);

		/* update control transfer stage */
		f_usb20hdc->ctrl_stage = F_USB20HDC_STAGE_SETUP;
		dev_dbg(f_usb20hdc->gadget.dev.parent,
			"next control transfer stage is SETUP stage.\n");
		break;
	}

	return;
}

static void on_end_in_transfer(struct f_usb20hdc_udc_ep *endpoint)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	struct f_usb20hdc_udc_req *request;
	u8 endpoint_channel = endpoint->endpoint_channel;

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
					struct f_usb20hdc_udc_req, queue);
	dev_dbg(f_usb20hdc->gadget.dev.parent,
		"endpoint %u is next queue at request = 0x%p, length = %u, buf = 0x%p.\n",
		endpoint_channel,
		request, request->request.length, request->request.buf);

	dev_dbg(f_usb20hdc->gadget.dev.parent,
			"endpoint %u halt status is %u.\n",
			endpoint_channel, endpoint->halt);

	/* check the got next request a request under current execution */
	if (request->request_execute)
		return;

	/* save request */
	endpoint->request = request;

	/* set endpoint x IN trasfer request */
	if (!set_in_transfer(endpoint)) {
		dev_err(f_usb20hdc->gadget.dev.parent,
			"%s():set_in_transfer() of endpoint %u is failed.\n",
			__func__, endpoint_channel);
		dequeue_all_transfer_request(endpoint, -EL2HLT);
	}

	return;
}

static void on_end_out_transfer(struct f_usb20hdc_udc_ep *endpoint)
{
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	struct f_usb20hdc_udc_req *request;
	u8 endpoint_channel = endpoint->endpoint_channel;

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
					struct f_usb20hdc_udc_req, queue);
	dev_dbg(f_usb20hdc->gadget.dev.parent,
		"endpoint %u is next queue at request = 0x%p, length = %u, buf = 0x%p.\n",
		endpoint_channel,
		request, request->request.length, request->request.buf);
	dev_dbg(f_usb20hdc->gadget.dev.parent,
			"endpoint %u halt status is %u.\n",
					endpoint_channel, endpoint->halt);

	/* check the got next request a request under current execution */
	if (request->request_execute)
		return;

	/* save request */
	endpoint->request = request;

	/* set endpoint x OUT transfer request */
	if (!set_out_transfer(endpoint)) {
		dev_err(f_usb20hdc->gadget.dev.parent,
			"%s():set_out_transfer() of endpoint %u is failed.\n",
			__func__, endpoint_channel);
		dequeue_all_transfer_request(endpoint, -EL2HLT);
	}

	return;
}

static void on_halt_transfer(struct f_usb20hdc_udc_ep *endpoint)
{
#if (F_USB20HDC_UDC_USE_AUTO_STALL_RECOVERY == 1)
	struct f_usb20hdc_udc *f_usb20hdc = endpoint->f_usb20hdc;
	u8 endpoint_channel = endpoint->endpoint_channel;

	/* check halt transfer error recovery perform need */
	if ((endpoint_channel != F_USB20HDC_EP0) || (endpoint->halt))
		return;

	dev_err(f_usb20hdc->gadget.dev.parent,
				"auto STALL error recovery is pefromed.\n");

	/* pre-initialize F_USB20HDC controller */
	initialize_udc_controller(f_usb20hdc, 1);

	/* disable host connect */
	enable_host_connect(f_usb20hdc, 0);

	/* notify disconnect to gadget driver */
	if ((f_usb20hdc->gadget_driver) &&
		(f_usb20hdc->gadget_driver->disconnect)) {
		spin_unlock(&f_usb20hdc->lock);
		f_usb20hdc->gadget_driver->disconnect(&f_usb20hdc->gadget);
		spin_lock(&f_usb20hdc->lock);
	}

	/* update halt transfer error recovery timer */
	mod_timer(&f_usb20hdc->halt_transfer_error_recovery_timer,
			jiffies + msecs_to_jiffies(
				F_USB20HDC_UDC_AUTO_STALL_RECOVERY_TIME));
#endif
	return;
}

#if (F_USB20HDC_UDC_USE_AUTO_STALL_RECOVERY == 1)
static void on_recovery_halt_transfer_error(unsigned long data)
{
	struct f_usb20hdc_udc *f_usb20hdc;

	/* check argument */
	if (unlikely(!data))
		return;

	/* get a device driver parameter */
	f_usb20hdc = (struct f_usb20hdc_udc *)data;

	dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is started.\n", __func__);

	/* enable communicate and set bus connect / disconnect detect */
	enable_communicate(f_usb20hdc, 1);

	dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is ended.\n", __func__);
	return;
}
#endif

static void on_recovery_controller_hungup(struct f_usb20hdc_udc *f_usb20hdc)
{
#if (F_USB20HDC_UDC_USE_HANGUP_RECOVERY == 1)
	dev_err(f_usb20hdc->gadget.dev.parent,
				"controller hung-up recovery is pefromed.\n");

	/* disable host connect */
	enable_host_connect(f_usb20hdc, 0);

	/* notify disconnect to gadget driver */
	if ((f_usb20hdc->gadget_driver) &&
		(f_usb20hdc->gadget_driver->disconnect)) {
		spin_unlock(&f_usb20hdc->lock);
		f_usb20hdc->gadget_driver->disconnect(&f_usb20hdc->gadget);
		spin_lock(&f_usb20hdc->lock);
	}

	/* check bus connect */
	if (f_usb20hdc->bus_connect)
		/* enable host connect */
		enable_host_connect(f_usb20hdc, 1);
#endif
	return;
}

static irqreturn_t on_usb_function(int irq, void *dev_id)
{
	u32 counter, nak, i;
	struct f_usb20hdc_udc *f_usb20hdc = dev_id;
	void *base_addr = f_usb20hdc->reg_base;
	struct device *dev;
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	struct f_usb20hdc_udc_ep *endpoint;
#endif

	/* check argument */
	if (unlikely(!dev_id))
		return IRQ_NONE;

	dev = f_usb20hdc->gadget.dev.parent;

	/* check F_USB20HDC controller interrupt request assert */
	if (unlikely(irq != f_usb20hdc->irq)) {
		dev_dbg(dev, "%s() is ended at non process.\n", __func__);
		return IRQ_NONE;
	}

	spin_lock(&f_usb20hdc->lock);

	/* bus connect interrupt factor */
	if ((get_otg_int(base_addr)) && (get_otg_inten(base_addr)) &&
		(get_vbus_vld_c(base_addr))) {
		/* clear vbus_vld interrupt factor */
		clear_vbus_vld_c(base_addr);

		dev_dbg(dev, "VBUS status change interrupt occured (vld:%d).\n",
			is_bus_connected(base_addr));

		/* process bus connect detect */
		on_detect_bus_connect((u32)f_usb20hdc);

		spin_unlock(&f_usb20hdc->lock);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return IRQ_HANDLED;
	}

	/* check F_USB20HDC controller device mode usage */
	if (!is_device_mode_usage(f_usb20hdc)) {
		spin_unlock(&f_usb20hdc->lock);
		dev_dbg(dev, "%s() is ended at non process.\n", __func__);
		return IRQ_HANDLED;
	}

	/* PHY hung-up interrupt factor */
	if ((get_phy_err_int(base_addr)) && (get_phy_err_inten(base_addr))) {
		/* clear phy_err interrupt factor */
		clear_phy_err_int(base_addr);

		dev_dbg(dev, "phy_err interrrupt is occurred.\n");

		/* process recovery controller hung-up */
		on_recovery_controller_hungup(f_usb20hdc);

		spin_unlock(&f_usb20hdc->lock);
		dev_dbg(dev, "%s() is ended.\n", __func__);

		return IRQ_HANDLED;
	}

	/* bus reset begin interrupt factor */
	if ((get_usbrstb_int(base_addr)) && (get_usbrstb_inten(base_addr))) {
		/* clear usbrstb interrupt factor */
		clear_usbrstb_int(base_addr);

		dev_dbg(dev, "usbrstb interrrupt is occurred.\n");

		/* process bus reset begin */
		on_begin_bus_reset(f_usb20hdc);

		spin_unlock(&f_usb20hdc->lock);
		dev_dbg(dev, "%s() is ended.\n", __func__);

		return IRQ_HANDLED;
	}

	/* bus reset end interrupt factor */
	if ((get_usbrste_int(base_addr)) && (get_usbrste_inten(base_addr))) {
		/* clear usbrste interrupt factor */
		clear_usbrste_int(base_addr);

		dev_dbg(dev, "usbrste interrrupt is occurred.\n");

		/* process bus reset end */
		on_end_bus_reset(f_usb20hdc);

		spin_unlock(&f_usb20hdc->lock);
		dev_dbg(dev, "%s() is ended.\n", __func__);

		return IRQ_HANDLED;
	}

	/* suspend end interrupt factor */
	if ((get_suspende_int(base_addr)) && (get_suspende_inten(base_addr))) {
		/* clear suspende interrupt factor */
		clear_suspende_int(base_addr);

		dev_dbg(dev, "suspende interrrupt is occurred.\n");

		/* process bus wakeup */
		on_wakeup(f_usb20hdc);

		spin_unlock(&f_usb20hdc->lock);
		dev_dbg(dev, "%s() is ended.\n", __func__);

		return IRQ_HANDLED;
	}

	/* SOF interrupt factor */
	if ((get_sof_int(base_addr)) && (get_sof_inten(base_addr))) {
		/* clear sof interrupt factor */
		clear_sof_int(base_addr);

		dev_dbg(dev, "sof interrrupt is occurred.\n");

		/* process SOF transfer end */
		on_end_transfer_sof(f_usb20hdc);

		spin_unlock(&f_usb20hdc->lock);
		dev_dbg(dev, "%s() is ended.\n", __func__);

		return IRQ_HANDLED;
	}

	/* SETUP transfer interrupt factor */
	if ((get_setup_int(base_addr)) && (get_setup_inten(base_addr))) {
		/* clear setup interrupt factor */
		clear_setup_int(base_addr);

		dev_dbg(dev, "setup interrrupt is occurred.\n");

		/* clear readyi / ready0 interrupt factor */
		clear_readyi_ready_int_clr(base_addr, F_USB20HDC_EP0);
		clear_readyo_empty_int_clr(base_addr, F_USB20HDC_EP0);

		/* process control SETUP transfer end */
		on_end_control_setup_transfer(
				&f_usb20hdc->endpoint[F_USB20HDC_EP0]);

		spin_unlock(&f_usb20hdc->lock);
		dev_dbg(dev, "%s() is ended.\n", __func__);

		return IRQ_HANDLED;
	}

	for (i = F_USB20HDC_EP0; i < F_USB20HDC_UDC_MAX_EP; i++) {
		endpoint = &f_usb20hdc->endpoint[i];
		if (endpoint && endpoint->endpoint_channel != 255) {
			nak = get_nack_int(f_usb20hdc->reg_base,
				endpoint->endpoint_channel);
			if (get_nack_int(f_usb20hdc->reg_base,
				endpoint->endpoint_channel) &&
				get_nack_inten(f_usb20hdc->reg_base,
				endpoint->endpoint_channel)) {
				dev_dbg(dev, "%s ep chan nak INT %d\n",
					__func__, endpoint->endpoint_channel);
				clear_nack_int_clr(f_usb20hdc->reg_base,
					endpoint->endpoint_channel);
			}
		}
	}

#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	/* endpoint x dma interrupt factor */
	for (counter = F_USB20HDC_DMA_CH1;
			counter < F_USB20HDC_MAX_DMA_CHANNELS; counter++) {
		if ((get_dma_int(base_addr, counter)) &&
			(get_dma_inten(base_addr, counter))) {
			/* clear dma interrupt factor */
			clear_dma_int(base_addr, counter);

			dev_dbg(dev, "dma %u interrrupt is occurred.\n",
					(u32)counter);

			endpoint = &f_usb20hdc->endpoint[f_usb20hdc->dma_data[counter].endpoint_channel];

			/* process IN / OUT transfer end */
			if (endpoint->transfer_direction) {
				endpoint->request->usb_dma_int_occurred = 1;
				if (endpoint->request->dmac_int_occurred)
					on_end_in_transfer(endpoint);
			} else {
				if (get_dma_tc(base_addr, counter) <
					get_dma_tci(base_addr, counter) &&
					get_dma_sp(base_addr, counter)) {
					spin_unlock(&f_usb20hdc->lock);
					hdmac_stop_nowait(f_usb20hdc->dma_data[
							counter].hdmac_channel);
					spin_lock(&f_usb20hdc->lock);
				}
			}

			dev_dbg(dev, "%s() is ended.\n", __func__);
			spin_unlock(&f_usb20hdc->lock);

			return IRQ_HANDLED;
		}
	}
#endif
	/* endpoint x interrupt factor */
	for (counter = F_USB20HDC_EP1;
		counter < F_USB20HDC_UDC_MAX_EP; counter++) {
		if ((get_dev_ep_int(base_addr, counter)) &&
			(get_dev_ep_inten(base_addr, counter))) {

			dev_dbg(dev, "dev_ep endpoint %u int is occurred.\n",
				(u32)counter);

			if ((get_readyi_ready_int(base_addr, counter)) &&
				(get_readyi_ready_inten(base_addr, counter))) {
				/* clear ready interrupt factor */
				clear_readyi_ready_int_clr(base_addr, counter);

				dbg_print(dev, "ready int is occurred.\n");

				/* process IN / OUT transfer end */
				f_usb20hdc->endpoint[
					counter].transfer_direction ?
						on_end_in_transfer(
							&f_usb20hdc->endpoint[
								counter]) :
						on_end_out_transfer(
							&f_usb20hdc->endpoint[
								counter]);

				spin_unlock(&f_usb20hdc->lock);
				dev_dbg(dev, "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}


			if ((get_ping_int(base_addr, counter)) &&
				(get_ping_inten(base_addr, counter))) {
				/* clear ping interrupt factor */
				clear_ping_int_clr(base_addr, counter);
				spin_unlock(&f_usb20hdc->lock);
				return IRQ_HANDLED;
			}

			if ((get_nack_int(base_addr, counter)) &&
				(get_nack_inten(base_addr, counter))) {
				/* clear nack interrupt factor */
				clear_nack_int_clr(base_addr, counter);
				spin_unlock(&f_usb20hdc->lock);
				return IRQ_HANDLED;
			}

			if ((get_readyo_empty_int(base_addr, counter)) &&
				(get_readyo_empty_inten(base_addr, counter))) {
				/* clear empty interrupt factor */
				clear_readyo_empty_int_clr(base_addr, counter);

				dev_dbg(dev, "empty int is occurred.\n");

				/* process IN transfer end */
				if (f_usb20hdc->endpoint[
						counter].transfer_direction)
					on_end_in_transfer(
						&f_usb20hdc->endpoint[counter]);

				spin_unlock(&f_usb20hdc->lock);
				dev_dbg(dev, "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}

			if ((get_stalled_int(base_addr, counter)) &&
				(get_stalled_inten(base_addr, counter))) {
				/* clear stalled interrupt factor */
				clear_stalled_int_clr(base_addr, counter);
				dev_dbg(dev, "stalled int is occurred.\n");

				/* process transfer halt */
				on_halt_transfer(
					&f_usb20hdc->endpoint[counter]);

				spin_unlock(&f_usb20hdc->lock);
				dev_dbg(dev, "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}

			dev_dbg(dev, "other interrrupt is occurred.\n");

			/* clear endpoint x interrupt */
			clear_readyi_ready_int_clr(base_addr, counter);
			clear_readyo_empty_int_clr(base_addr, counter);
			clear_ping_int_clr(base_addr, counter);
			clear_stalled_int_clr(base_addr, counter);
			clear_nack_int_clr(base_addr, counter);

			spin_unlock(&f_usb20hdc->lock);
			dev_dbg(dev, "%s() is ended.\n", __func__);

			return IRQ_HANDLED;
		}
	}

	/* endpoint 0 interrupt factor */
	if ((get_dev_ep_int(base_addr, F_USB20HDC_EP0)) &&
		(get_dev_ep_inten(base_addr, F_USB20HDC_EP0))) {

		dbg_print(dev, "dev_ep endpoint 0 int is occurred.\n");

		/* check priority direction */
		if (f_usb20hdc->ctrl_pri_dir) {
			/* priority is given to IN transfer */
			if ((get_readyi_ready_int(base_addr,
							F_USB20HDC_EP0)) &&
				(get_readyi_ready_inten(base_addr,
							F_USB20HDC_EP0))) {
				/* clear readyi interrupt factor */
				clear_readyi_ready_int_clr(base_addr,
							F_USB20HDC_EP0);

				dbg_print(dev, "readyi int is occurred.\n");

				/* process control IN transfer end */
				on_end_control_in_transfer(
						&f_usb20hdc->endpoint[
							F_USB20HDC_EP0]);

				spin_unlock(&f_usb20hdc->lock);
				dev_dbg(dev, "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}

			if ((get_readyo_empty_int(base_addr,
							F_USB20HDC_EP0)) &&
				(get_readyo_empty_inten(base_addr,
							F_USB20HDC_EP0))) {
				/* clear readyo interrupt factor */
				clear_readyo_empty_int_clr(base_addr,
							F_USB20HDC_EP0);

				dbg_print(dev, "readyo int is occurred.\n");

				/* process control OUT transfer end */
				on_end_control_out_transfer(
						&f_usb20hdc->endpoint[
							F_USB20HDC_EP0]);

				spin_unlock(&f_usb20hdc->lock);
				dev_dbg(dev, "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}
		} else {
			/* priority is given to OUT transfer */
			if ((get_readyo_empty_int(base_addr,
							F_USB20HDC_EP0)) &&
				(get_readyo_empty_inten(base_addr,
							F_USB20HDC_EP0))) {
				/* clear readyo interrupt factor */
				clear_readyo_empty_int_clr(base_addr,
							F_USB20HDC_EP0);

				dbg_print(dev, "readyo int is occurred.\n");

				/* process control OUT transfer end */
				on_end_control_out_transfer(
						&f_usb20hdc->endpoint[
							F_USB20HDC_EP0]);

				spin_unlock(&f_usb20hdc->lock);
				dev_dbg(dev, "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}

			if ((get_readyi_ready_int(base_addr,
							F_USB20HDC_EP0)) &&
				(get_readyi_ready_inten(base_addr,
							F_USB20HDC_EP0))) {
				/* clear readyi interrupt factor */
				clear_readyi_ready_int_clr(base_addr,
							F_USB20HDC_EP0);

				dbg_print(dev, "readyi int is occurred.\n");

				/* process control IN transfer end */
				on_end_control_in_transfer(
						&f_usb20hdc->endpoint[
							F_USB20HDC_EP0]);

				spin_unlock(&f_usb20hdc->lock);
				dev_dbg(dev, "%s() is ended.\n", __func__);

				return IRQ_HANDLED;
			}
		}

		if ((get_stalled_int(base_addr, F_USB20HDC_EP0)) &&
			(get_stalled_inten(base_addr, F_USB20HDC_EP0))) {
			/* clear stalled interrupt factor */
			clear_stalled_int_clr(base_addr, F_USB20HDC_EP0);

			dbg_print(dev, "stalled 0 interrrupt is occurred.\n");

			/* process transfer halt */
			on_halt_transfer(
				&f_usb20hdc->endpoint[F_USB20HDC_EP0]);

			spin_unlock(&f_usb20hdc->lock);
			dev_dbg(dev, "%s() is ended.\n", __func__);

			return IRQ_HANDLED;
		}

		dev_dbg(dev, "other interrrupt is occurred.\n");

		/* clear endpoint 0 interrupt */
		clear_readyo_empty_int_clr(base_addr, F_USB20HDC_EP0);
		clear_readyi_ready_int_clr(base_addr, F_USB20HDC_EP0);
		clear_ping_int_clr(base_addr, F_USB20HDC_EP0);
		clear_stalled_int_clr(base_addr, F_USB20HDC_EP0);
		clear_nack_int_clr(base_addr, F_USB20HDC_EP0);

		spin_unlock(&f_usb20hdc->lock);
		dev_dbg(dev, "%s() is ended.\n", __func__);

		return IRQ_HANDLED;
	}

	/* suspend begin interrupt factor */
	if ((get_suspendb_int(base_addr)) && (get_suspendb_inten(base_addr))) {
		/* clear suspendb interrupt factor */
		clear_suspendb_int(base_addr);

		dev_info(dev, "suspendb interrrupt is occurred.\n");

		/* process bus suspend */
		on_suspend(f_usb20hdc);

		spin_unlock(&f_usb20hdc->lock);
		dev_dbg(dev, "%s() is ended.\n", __func__);

		return IRQ_HANDLED;
	}

	spin_unlock(&f_usb20hdc->lock);
	dev_dbg(dev, "%s() is ended.\n", __func__);

	return IRQ_HANDLED;
}

#define to_fusb20_udc(g)  (container_of((g), struct f_usb20hdc_udc, gadget))

static int f_usb20hdc_udc_start(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct f_usb20hdc_udc *f_usb20hdc = to_fusb20_udc(g);
	u8 otg_suspended;
	struct device *dev = f_usb20hdc->gadget.dev.parent;

	dev_dbg(dev, "%s() is started.\n", __func__);

#if defined(CONFIG_USB_F_USB20HDC_OTG_DUAL_ROLE)
	otg_suspended = f_usb20hdc->otg_suspend_state;
#else
	/*OTG role-switch function doesn't exist*/
	otg_suspended = 0;
#endif

	/* entry gadget driver structure */
	f_usb20hdc->gadget_driver = driver;

	if (!otg_suspended) {
		dev_dbg(dev, "%s() : initialize ep data.\n", __func__);
		/* initialize endpoint configure data */
		initialize_endpoint_configure(f_usb20hdc);
	}

	/* initialize F_USB20HDC UDC device driver structure data */
	f_usb20hdc->gadget.speed = USB_SPEED_UNKNOWN;
	f_usb20hdc->bus_connect = 0;
	f_usb20hdc->device_state = USB_STATE_ATTACHED;
	f_usb20hdc->device_state_last = USB_STATE_NOTATTACHED;
	f_usb20hdc->configure_value_last = 0;
	f_usb20hdc->ctrl_stage = F_USB20HDC_STAGE_SETUP;
	f_usb20hdc->ctrl_pri_dir = 1;

	/* set F_USB20HDC UDC device driver register */
	f_usb20hdc->device_add = 1;

	if (!otg_suspended) {
		dev_info(dev, "%s() : setup recovery timer.\n", __func__);
#if (F_USB20HDC_UDC_USE_AUTO_STALL_RECOVERY == 1)
		setup_timer(&f_usb20hdc->halt_transfer_error_recovery_timer,
			on_recovery_halt_transfer_error, (u32)f_usb20hdc);
#endif
	}

	if (!otg_suspended) {
		dev_dbg(dev, "%s() : enable communicate.\n", __func__);
		/* enable communicate */
		enable_communicate(f_usb20hdc, 1);
	}

	dev_info(dev, "%s is registered.\n",
					f_usb20hdc->gadget_driver->driver.name);

	dev_dbg(dev, "%s() is ended.\n", __func__);

#if defined(CONFIG_USB_F_USB20HDC_OTG_DUAL_ROLE)
	/*this is for otg resume/suspend */
	f_usb20hdc->gadget_connected = 1;
#endif

	dev_info(dev, "High level class is connected to Fujitsu gadget driver.\n");
	return 0;
}

static int f_usb20hdc_udc_stop(struct usb_gadget *g,
					struct usb_gadget_driver *driver)
{
	struct f_usb20hdc_udc *f_usb20hdc = to_fusb20_udc(g);
	u8 device_add_last;
	unsigned long flags;
	struct device *dev = f_usb20hdc->gadget.dev.parent;

	dev_dbg(dev, "%s() is started.\n", __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* clear F_USB20HDC UDC device driver register */
	device_add_last = f_usb20hdc->device_add;
	f_usb20hdc->device_add = 0;

	/* disable communicate */
	enable_communicate(f_usb20hdc, 0);

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	f_usb20hdc->gadget_driver = NULL;

	dev_dbg(dev, "%s() is ended.\n", __func__);

#if defined(CONFIG_USB_F_USB20HDC_OTG_DUAL_ROLE)
	/*this is for otg resume/suspend */
	f_usb20hdc->gadget_connected = 0;
#endif

	dev_info(dev, "High level class is disconnected from"
		"Fujitsu gadget driver.\n");
	return 0;
}

static int f_usb20hdc_udc_ep_enable(struct usb_ep *ep,
	const struct usb_endpoint_descriptor *desc)
{
	struct f_usb20hdc_udc *f_usb20hdc;
	struct f_usb20hdc_udc_ep *endpoint;
	u8 endpoint_channel;
	unsigned long flags;
	struct device *dev;

	/* check argument */
	if (unlikely((!ep) || (!desc)))
		return -EINVAL;

	/* get parameter */
	endpoint = container_of(ep, struct f_usb20hdc_udc_ep, endpoint);
	endpoint_channel = endpoint->endpoint_channel;
	f_usb20hdc = endpoint->f_usb20hdc;
	dev = f_usb20hdc->gadget.dev.parent;

	dev_dbg(dev, "%s() is started.\n", __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(f_usb20hdc))) {
		dev_err(dev, "%s():host mode is usage.\n", __func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return -EPROTO;
	}

	/* check parameter */
	if (unlikely(endpoint_channel == F_USB20HDC_EP0)) {
		dev_err(dev, "%s():endpoint 0 enable is invalid.\n", __func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return -EINVAL;
	}

	/* check endpoint descriptor parameter */
	if (unlikely(desc->bDescriptorType != USB_DT_ENDPOINT)) {
		dev_err(dev, "%s():endpoint %u descriptor type is error.\n",
				__func__, endpoint_channel);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return -EINVAL;
	}

	/* check gadget driver parameter */
	if (unlikely(!f_usb20hdc->gadget_driver)) {
		dev_err(dev, "%s():gadget driver parameter is none.\n",
			__func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return -ESHUTDOWN;
	}

	dev_dbg(dev, "endpoint %u is enabled.\n", endpoint_channel);

	/* set endpoint parameter */
	endpoint->endpoint.desc = desc;

	/* configure endpoint */
	if (!configure_endpoint(endpoint)) {
		endpoint->endpoint.desc = NULL;
		dev_err(dev, "%s():endpoint %u configure is failed.\n",
				__func__, endpoint_channel);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return -EINVAL;
	}

	/* dequeue all previous transfer request */
	dequeue_all_transfer_request(endpoint, -ECONNABORTED);

	/* enable endpoint */
	enable_endpoint(endpoint, 1);

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dev_dbg(dev, "%s() is ended.\n", __func__);

	return 0;
}

static int f_usb20hdc_udc_ep_disable(struct usb_ep *ep)
{
	struct f_usb20hdc_udc *f_usb20hdc;
	struct f_usb20hdc_udc_ep *endpoint;
	u8 endpoint_channel;
	unsigned long flags;
	struct device *dev;

	/* check argument */
	if (unlikely(!ep))
		return -EINVAL;

	/* get parameter */
	endpoint = container_of(ep, struct f_usb20hdc_udc_ep, endpoint);
	endpoint_channel = endpoint->endpoint_channel;
	f_usb20hdc = endpoint->f_usb20hdc;
	dev = f_usb20hdc->gadget.dev.parent;

	dev_dbg(dev, "%s() is started.\n", __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(f_usb20hdc))) {
		dev_err(dev, "%s():host mode is usage.\n", __func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return -EPROTO;
	}

	/* check parameter */
	if (unlikely(endpoint_channel == F_USB20HDC_EP0)) {
		dev_err(dev, "%s():endpoint 0 disable is invalid.\n", __func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return -EINVAL;
	}

	dev_dbg(dev, "endpoint %u is disabled.\n", endpoint_channel);

	/* set endpoint parameter */
	endpoint->endpoint.desc = NULL;

	/* disable endpoint */
	enable_endpoint(endpoint, 0);

	/* dequeue all previous transfer request */
	dequeue_all_transfer_request(endpoint, -ESHUTDOWN);

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dev_dbg(dev, "%s() is ended.\n", __func__);

	return 0;
}

static struct usb_request *f_usb20hdc_udc_ep_alloc_request(struct usb_ep *ep,
	gfp_t gfp_flags)
{
	struct f_usb20hdc_udc *f_usb20hdc;
	struct f_usb20hdc_udc_req *request;
	struct f_usb20hdc_udc_ep *endpoint;
	u8 endpoint_channel;
	struct device *dev;
	/*
	 * [notice]:Acquisition of a spin lock is prohibition.
	 */

	/* check argument */
	if (unlikely(!ep))
		return 0;

	/* get parameter */
	endpoint = container_of(ep, struct f_usb20hdc_udc_ep, endpoint);
	endpoint_channel = endpoint->endpoint_channel;
	f_usb20hdc = endpoint->f_usb20hdc;
	dev = f_usb20hdc->gadget.dev.parent;

	dev_dbg(dev, "%s() is started.\n", __func__);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(f_usb20hdc))) {
		dev_err(dev, "%s():host mode is usage.\n", __func__);
#if !defined(CONFIG_USB_F_USB20HDC_OTG_DUAL_ROLE)
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return 0;
#endif
	}

	/* allocate memory and zero clear memory */
	request = kzalloc(sizeof(*request), gfp_flags);
	if (!request) {
		dev_err(dev, "%s():kzalloc is failed.\n", __func__);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return 0;
	}
	dev_dbg(dev, "endpoint %u allocate memory is 0x%p.\n",
		endpoint_channel, request);

	/* initialize list data */
	INIT_LIST_HEAD(&request->queue);
	request->request.dma = ~(dma_addr_t)0;

	dev_dbg(dev, "%s() is ended.\n", __func__);

	return &request->request;
}

static void f_usb20hdc_udc_ep_free_request(struct usb_ep *ep,
	struct usb_request *req)
{
	struct f_usb20hdc_udc *f_usb20hdc;
	struct f_usb20hdc_udc_req *request;
	struct f_usb20hdc_udc_ep *endpoint;
	u8 endpoint_channel;
	struct device *dev;

	/*
	 * [notice]:Acquisition of a spin lock is prohibition.
	 */

	/* check argument */
	if (unlikely((!ep) || (!req)))
		return;

	/* get parameter */
	endpoint = container_of(ep, struct f_usb20hdc_udc_ep, endpoint);
	request = container_of(req, struct f_usb20hdc_udc_req, request);
	endpoint_channel = endpoint->endpoint_channel;
	f_usb20hdc = endpoint->f_usb20hdc;
	dev = f_usb20hdc->gadget.dev.parent;

	dev_dbg(dev, "%s() is started.\n", __func__);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(f_usb20hdc))) {
		dev_err(dev, "%s():host mode is usage.\n", __func__);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return;
	}

	/* free memory */
	WARN_ON(!list_empty(&request->queue));
	kfree(request);
	dev_dbg(dev, "endpoint %u free memory is 0x%p.\n",
		endpoint_channel, request);

	dev_dbg(dev, "%s() is ended.\n", __func__);

	return;
}

static int f_usb20hdc_udc_ep_queue(struct usb_ep *ep, struct usb_request *req,
	gfp_t gfp_flags)
{
	struct f_usb20hdc_udc *f_usb20hdc;
	struct f_usb20hdc_udc_req *request;
	struct f_usb20hdc_udc_ep *endpoint;
	u8 endpoint_channel;
	unsigned long flags;
	struct device *dev;

	/* check argument */
	if (unlikely((!ep) || (!req)))
		return -EINVAL;

	/* get parameter */
	endpoint = container_of(ep, struct f_usb20hdc_udc_ep, endpoint);
	request = container_of(req, struct f_usb20hdc_udc_req, request);
	endpoint_channel = endpoint->endpoint_channel;
	f_usb20hdc = endpoint->f_usb20hdc;
	dev = f_usb20hdc->gadget.dev.parent;

	dev_dbg(dev, "%s() is started.\n", __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(f_usb20hdc))) {
		dev_err(dev, "%s():host mode is usage.\n", __func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return -ESHUTDOWN;
	}

	/* check parameter */
	if (unlikely((!req->buf) || (!list_empty(&request->queue)))) {
		dev_err(dev, "%s():request parameter is error.\n", __func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return -EINVAL;
	}
	if (unlikely((endpoint_channel != F_USB20HDC_EP0) &&
				!endpoint->endpoint.desc)) {
		dev_err(dev, "%s():endpoint %u descriptor is none.\n",
				__func__, endpoint_channel);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return -EINVAL;
	}

	/* check gadget driver parameter */
	if (unlikely(!f_usb20hdc->gadget_driver)) {
		dev_err(dev, "%s():gadget driver parameter is none.\n",
			__func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return -ESHUTDOWN;
	}

	/* check state */
	if (f_usb20hdc->gadget.speed == USB_SPEED_UNKNOWN) {
		dev_dbg(dev, "device state is reset.\n");
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return -ESHUTDOWN;
	}

	dev_dbg(dev,
		"endpoint %u is queue at request = 0x%p, length = %u, buf = 0x%p.\n",
		endpoint_channel, request,
		request->request.length, request->request.buf);
	dev_dbg(dev, "endpoint %u halt status is %u.\n",
		endpoint_channel, endpoint->halt);

	/* initialize request parameter */
	request->request.status = -EINPROGRESS;
	request->request.actual = 0;

	/* check current queue execute */
	if ((!list_empty(&endpoint->queue)) || (endpoint->halt)) {
		/* add list tail */
		list_add_tail(&request->queue, &endpoint->queue);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return 0;
	}

	/* save request */
	endpoint->request = request;

	if (endpoint_channel == F_USB20HDC_EP0) {
		/* request endpoint 0 */
		switch (f_usb20hdc->ctrl_stage) {
		case F_USB20HDC_STAGE_IN_DATA:
		case F_USB20HDC_STAGE_IN_STATUS:
			if (!set_in_transfer_pio(endpoint)) {
				dev_err(dev,
					"%s():set_in_transfer_pio() of endpoint 0 is failed.\n",
					__func__);
				notify_transfer_request_complete(
							endpoint, -EL2HLT);
				spin_unlock_irqrestore(&f_usb20hdc->lock,
									flags);
				dev_dbg(dev, "%s() is ended.\n", __func__);
				return -EL2HLT;
			}
			break;
		case F_USB20HDC_STAGE_OUT_DATA:
		case F_USB20HDC_STAGE_OUT_STATUS:
			if (!set_out_transfer_pio(endpoint)) {
				dev_err(dev,
					"%s():set_out_transfer_pio() of endpoint 0 is failed.\n",
					__func__);
				notify_transfer_request_complete(
							endpoint, -EL2HLT);
				spin_unlock_irqrestore(&f_usb20hdc->lock,
					flags);
				dev_dbg(dev, "%s() is ended.\n", __func__);
				return -EL2HLT;
			}
			break;
		default:
			dev_dbg(dev, "control transfer stage is changed at %u.\n",
				f_usb20hdc->ctrl_stage);
			notify_transfer_request_complete(endpoint, -EL2HLT);
			spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
			dev_dbg(dev, "%s() is ended.\n", __func__);
			return -EL2HLT;
		}
	} else {
		/* request endpoint x */
		if (endpoint->transfer_direction) {
			if (!set_in_transfer(endpoint)) {
				dev_err(dev,
					"%s():set_in_transfer() of endpoint %u is failed.\n",
					__func__, endpoint_channel);
				notify_transfer_request_complete(
							endpoint, -EL2HLT);
				spin_unlock_irqrestore(
						&f_usb20hdc->lock, flags);
				dev_dbg(dev, "%s() is ended.\n", __func__);
				return -EL2HLT;
			}
		} else {
			if (!set_out_transfer(endpoint)) {
				dev_err(dev,
					"%s():set_out_transfer() of endpoint %u is failed.\n",
					__func__, endpoint_channel);
				notify_transfer_request_complete(
							endpoint, -EL2HLT);
				spin_unlock_irqrestore(&f_usb20hdc->lock,
									flags);
				dev_dbg(dev, "%s() is ended.\n", __func__);
				return -EL2HLT;
			}
		}
	}

	/* add list tail */
	list_add_tail(&request->queue, &endpoint->queue);

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dev_dbg(dev, "%s() is ended.\n", __func__);

	return 0;
}

static int f_usb20hdc_udc_ep_dequeue(struct usb_ep *ep, struct usb_request *req)
{
	struct f_usb20hdc_udc *f_usb20hdc;
	struct f_usb20hdc_udc_req *request;
	struct f_usb20hdc_udc_ep *endpoint;
	u8 endpoint_channel;
	unsigned long flags;
	struct device *dev;

	/* check argument */
	if (unlikely((!ep) || (!req)))
		return -EINVAL;

	/* get parameter */
	endpoint = container_of(ep, struct f_usb20hdc_udc_ep, endpoint);
	request = container_of(req, struct f_usb20hdc_udc_req, request);
	endpoint_channel = endpoint->endpoint_channel;
	f_usb20hdc = endpoint->f_usb20hdc;
	dev = f_usb20hdc->gadget.dev.parent;

	dev_dbg(dev, "%s() is started.\n", __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(f_usb20hdc))) {
		dev_err(dev, "%s():host mode is usage.\n", __func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return -EPROTO;
	}

	/* check parameter */
	if (unlikely((endpoint_channel != F_USB20HDC_EP0) &&
					(!endpoint->endpoint.desc))) {
		dev_err(dev, "%s():endpoint %u descriptor is none.\n",
				__func__, endpoint_channel);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return -EINVAL;
	}

	/*
	 * check queue empty
	 * [notice]:It is mandatory processing.
	 */
	if (!list_empty(&endpoint->queue)) {
		/* check list entry */
		list_for_each_entry(request, &endpoint->queue, queue) {
			if ((request) && (&request->request == req))
				break;
		}

		/* check dequeue request mismatch */
		if (unlikely((!request) || (&request->request != req))) {
			dev_err(dev, "%s():endpoint %u request is mismatch.\n",
				__func__, endpoint_channel);
			spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
			dev_dbg(dev, "%s() is ended.\n", __func__);
			return -EINVAL;
		}
	} else {
		dev_err(dev, "%s():endpoint %u request queue is epmty.\n",
			__func__, endpoint_channel);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return -EINVAL;
	}

	/* abort request transfer */
	endpoint->request = request;
	if (endpoint_channel == F_USB20HDC_EP0) {
		abort_in_transfer(endpoint, 0);
		abort_out_transfer(endpoint, 0);
	} else {
		endpoint->transfer_direction ? abort_in_transfer(endpoint, 0) :
						abort_out_transfer(endpoint, 0);
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
		if (endpoint->dma_transfer) {
			spin_unlock(&f_usb20hdc->lock);
			hdmac_stop_nowait(f_usb20hdc->dma_data[
				endpoint->usb_dma_channel].hdmac_channel);
			spin_lock(&f_usb20hdc->lock);
		}
#endif
	}

	notify_transfer_request_complete(endpoint, -ECONNRESET);

	/* check next queue empty */
	if (list_empty(&endpoint->queue)) {
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return 0;
	}

	/* get next request */
	request = list_entry(endpoint->queue.next,
					struct f_usb20hdc_udc_req, queue);

	/* check the got next request a request under current execution */
	if (request->request_execute) {
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return 0;
	}

	/* save request */
	endpoint->request = request;

	/* set endpoint x transfer request */
	if (!(endpoint->transfer_direction ? set_in_transfer(endpoint) :
						set_out_transfer(endpoint))) {
		dev_err(dev, "%s():%s of endpoint %u is failed.\n",
			endpoint->transfer_direction ? "set_in_transfer()" :
							"set_out_transfer()",
							__func__,
							endpoint_channel);
		dequeue_all_transfer_request(endpoint, -EL2HLT);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(dev, "%s() is ended.\n", __func__);
		return 0;
	}

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dev_dbg(dev, "%s() is ended.\n", __func__);

	return 0;
}

static int f_usb20hdc_udc_ep_set_halt(struct usb_ep *ep, int value)
{
	struct f_usb20hdc_udc *f_usb20hdc;
	struct f_usb20hdc_udc_ep *endpoint;
	u8 endpoint_channel;
	unsigned long flags;

	/* check argument */
	if (unlikely((!ep) || ((value != 0) && (value != 1))))
		return -EINVAL;

	/* get parameter */
	endpoint = container_of(ep, struct f_usb20hdc_udc_ep, endpoint);
	endpoint_channel = endpoint->endpoint_channel;
	f_usb20hdc = endpoint->f_usb20hdc;

	dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is started.\n", __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(f_usb20hdc))) {
		dev_err(f_usb20hdc->gadget.dev.parent,
				"%s():host mode is usage.\n", __func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is ended.\n", __func__);
		return -EPROTO;
	}

	/* check parameter */
	if (unlikely((endpoint_channel != F_USB20HDC_EP0) &&
						(!endpoint->endpoint.desc))) {
		dev_err(f_usb20hdc->gadget.dev.parent,
				"%s():endpoint %u descriptor is none.\n",
				__func__, endpoint_channel);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is ended.\n", __func__);
		return -EINVAL;
	}

	/* check isochronous endpoint */
	if (unlikely(endpoint->transfer_type == USB_ENDPOINT_XFER_ISOC)) {
		dev_dbg(f_usb20hdc->gadget.dev.parent,
				"%s():endpoint %u is isochoronous transfer.\n",
				__func__, endpoint_channel);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is ended.\n", __func__);
		return -EPROTO;
	}

	if (value) {
		/* check current queue execute */
		if (!list_empty(&endpoint->queue)) {
			dev_dbg(f_usb20hdc->gadget.dev.parent,
					"endpoint %u is execute queue.\n",
					endpoint_channel);
			spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
			dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is ended.\n", __func__);
			return -EAGAIN;
		}

		/* set halt */
		dev_dbg(f_usb20hdc->gadget.dev.parent,
					"endpoint %u transfer is halted.\n",
					endpoint_channel);
		halt_transfer(endpoint, 1, 0);
	} else {
		/* clear halt */
		dev_dbg(f_usb20hdc->gadget.dev.parent,
				"endpoint %u transfer halt is cleared.\n",
				endpoint_channel);
		halt_transfer(endpoint, 0, 0);
		endpoint->force_halt = 0;
	}

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dev_dbg(f_usb20hdc->gadget.dev.parent, "%s() is ended.\n", __func__);

	return 0;
}

static int f_usb20hdc_udc_ep_set_wedge(struct usb_ep *ep)
{
	struct f_usb20hdc_udc *f_usb20hdc;
	struct f_usb20hdc_udc_ep *endpoint;
	u8 endpoint_channel;
	unsigned long flags;

	/* check argument */
	if (unlikely(!ep))
		return -EINVAL;

	/* get parameter */
	endpoint = container_of(ep, struct f_usb20hdc_udc_ep, endpoint);
	endpoint_channel = endpoint->endpoint_channel;
	f_usb20hdc = endpoint->f_usb20hdc;

	dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is started.\n", __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(f_usb20hdc))) {
		dev_err(f_usb20hdc->gadget.dev.parent,
				"%s():host mode is usage.\n", __func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is ended.\n", __func__);
		return -EPROTO;
	}

	/* check parameter */
	if (unlikely((endpoint_channel != F_USB20HDC_EP0) &&
					(!endpoint->endpoint.desc))) {
		dev_err(f_usb20hdc->gadget.dev.parent,
				"%s():endpoint %u descriptor is none.\n",
				__func__, endpoint_channel);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is ended.\n", __func__);
		return -EINVAL;
	}

	/* check isochronous endpoint */
	if (unlikely(endpoint->transfer_type == USB_ENDPOINT_XFER_ISOC)) {
		dev_err(f_usb20hdc->gadget.dev.parent,
				"%s():endpoint %u is isochoronous transfer.\n",
			__func__, endpoint_channel);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is ended.\n", __func__);
		return -EPROTO;
	}

	/* check current queue execute */
	if (!list_empty(&endpoint->queue)) {
		dev_err(f_usb20hdc->gadget.dev.parent,
					"endpoint %u queue is execute.\n",
					endpoint_channel);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is ended.\n", __func__);
		return -EAGAIN;
	}

	/* set force halt */
	dev_err(f_usb20hdc->gadget.dev.parent,
			"endpoint %u transfer is wedged.\n", endpoint_channel);
	halt_transfer(endpoint, 1, 1);

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dev_dbg(f_usb20hdc->gadget.dev.parent, "%s() is ended.\n", __func__);

	return 0;
}

static int f_usb20hdc_udc_ep_fifo_status(struct usb_ep *ep)
{
	struct f_usb20hdc_udc *f_usb20hdc;
	struct f_usb20hdc_udc_ep *endpoint;
	u8 endpoint_channel;
	u16 bytes;
	unsigned long flags;

	/* check argument */
	if (unlikely(!ep))
		return -EINVAL;

	/* get parameter */
	endpoint = container_of(ep, struct f_usb20hdc_udc_ep, endpoint);
	endpoint_channel = endpoint->endpoint_channel;
	f_usb20hdc = endpoint->f_usb20hdc;

	dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is started.\n", __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(f_usb20hdc))) {
		dev_err(f_usb20hdc->gadget.dev.parent,
				"%s():host mode is usage.\n", __func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is ended.\n", __func__);
		return -EPROTO;
	}

	/* get current bytes in FIFO */
	bytes = get_fifo_bytes(endpoint);
	dev_dbg(f_usb20hdc->gadget.dev.parent, "bytes in FIFO is %u.\n", bytes);

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dev_dbg(f_usb20hdc->gadget.dev.parent, "%s() is ended.\n", __func__);

	return (int)bytes;
}

static void f_usb20hdc_udc_ep_fifo_flush(struct usb_ep *ep)
{
	struct f_usb20hdc_udc *f_usb20hdc;
	struct f_usb20hdc_udc_ep *endpoint;
	u8 endpoint_channel;
	unsigned long flags;

	/* check argument */
	if (unlikely(!ep))
		return;

	/* get parameter */
	endpoint = container_of(ep, struct f_usb20hdc_udc_ep, endpoint);
	endpoint_channel = endpoint->endpoint_channel;
	f_usb20hdc = endpoint->f_usb20hdc;

	dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is started.\n", __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(f_usb20hdc))) {
		dev_err(f_usb20hdc->gadget.dev.parent,
				"%s():host mode is usage.\n", __func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is ended.\n", __func__);
		return;
	}

	/* initialize endpoint FIFO */
	dev_dbg(f_usb20hdc->gadget.dev.parent,
			"endpoint %u FIFO is flush.\n", endpoint_channel);
	initialize_endpoint(endpoint, 1, 0, 0);

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dev_dbg(f_usb20hdc->gadget.dev.parent, "%s() is ended.\n", __func__);

	return;
}

static int f_usb20hdc_udc_gadget_get_frame(struct usb_gadget *gadget)
{
	struct f_usb20hdc_udc *f_usb20hdc;
	u16 frame_number;
	unsigned long flags;

	/* check argument */
	if (unlikely(!gadget))
		return -EINVAL;

	/* get a device driver parameter */
	f_usb20hdc = container_of(gadget, struct f_usb20hdc_udc, gadget);

	dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is started.\n", __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(f_usb20hdc))) {
		dev_err(f_usb20hdc->gadget.dev.parent,
				"%s():host mode is usage.\n", __func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is ended.\n", __func__);
		return -EPROTO;
	}

	/* get frame number */
	frame_number = get_timstamp(f_usb20hdc->reg_base);
	dev_dbg(f_usb20hdc->gadget.dev.parent,
					"frame number is %u.\n", frame_number);

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dev_dbg(f_usb20hdc->gadget.dev.parent, "%s() is ended.\n", __func__);

	return (int)frame_number;
}

static int f_usb20hdc_udc_gadget_wakeup(struct usb_gadget *gadget)
{
	struct f_usb20hdc_udc *f_usb20hdc;
	unsigned long flags;

	/* check argument */
	if (unlikely(!gadget))
		return -EINVAL;

	/* get a device driver parameter */
	f_usb20hdc = container_of(gadget, struct f_usb20hdc_udc, gadget);

	dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is started.\n", __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(f_usb20hdc))) {
		dev_err(f_usb20hdc->gadget.dev.parent,
				"%s():host mode is usage.\n", __func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is ended.\n", __func__);
		return -EPROTO;
	}

	/* check device state */
	if (f_usb20hdc->device_state != USB_STATE_SUSPENDED) {
		dev_err(f_usb20hdc->gadget.dev.parent,
				"%s():device state is invalid at %u.\n",
				__func__, f_usb20hdc->device_state);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is ended.\n", __func__);
		return -EPROTO;
	}

	/* output resume */
	set_reqresume(f_usb20hdc->reg_base, 1);
	dev_dbg(f_usb20hdc->gadget.dev.parent, "resume is output.\n");

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dev_dbg(f_usb20hdc->gadget.dev.parent, "%s() is ended.\n", __func__);

	return 0;
}

static int f_usb20hdc_udc_gadget_set_selfpowered(struct usb_gadget *gadget,
	int is_selfpowered)
{
	struct f_usb20hdc_udc *f_usb20hdc;
	unsigned long flags;

	/* check argument */
	if (unlikely(!gadget))
		return -EINVAL;

	/* get a device driver parameter */
	f_usb20hdc = container_of(gadget, struct f_usb20hdc_udc, gadget);

	dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is started.\n", __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(f_usb20hdc))) {
		dev_err(f_usb20hdc->gadget.dev.parent,
				"%s():host mode is usage.\n", __func__);
#if !defined(CONFIG_USB_F_USB20HDC_OTG_DUAL_ROLE)
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is ended.\n", __func__);
		return -EPROTO;
#endif
	}

	/* set self-powered status */
	f_usb20hdc->selfpowered = is_selfpowered ? 1 : 0;
	dev_dbg(f_usb20hdc->gadget.dev.parent, "self power status is %s.\n",
				is_selfpowered ? "selfpowered" : "buspowered");

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dev_dbg(f_usb20hdc->gadget.dev.parent, "%s() is ended.\n", __func__);

	return 0;
}

static int f_usb20hdc_udc_gadget_vbus_session(struct usb_gadget *gadget,
	int is_active)
{
	struct f_usb20hdc_udc *f_usb20hdc;
	unsigned long flags;

	/* check argument */
	if (unlikely(!gadget))
		return -EINVAL;

	/* get a device driver parameter */
	f_usb20hdc = container_of(gadget, struct f_usb20hdc_udc, gadget);

	dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is started.\n", __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(f_usb20hdc))) {
		dev_err(f_usb20hdc->gadget.dev.parent,
				"%s():host mode is usage.\n", __func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is ended.\n", __func__);
		return -EPROTO;
	}

	/* check bus active */
	if (is_active) {
		/* set bus disconnect detect */
		set_bus_connect_detect(f_usb20hdc->reg_base, 0);

		/* enable host connect */
		enable_host_connect(f_usb20hdc, 1);
	} else {
		/* disable host connect */
		enable_host_connect(f_usb20hdc, 0);

		/* set bus connect detect */
		set_bus_connect_detect(f_usb20hdc->reg_base, 1);

		/* notify disconnect to gadget driver */
		spin_unlock(&f_usb20hdc->lock);
		if ((f_usb20hdc->gadget_driver) &&
					(f_usb20hdc->gadget_driver->disconnect))
			f_usb20hdc->gadget_driver->disconnect(
							&f_usb20hdc->gadget);
		spin_lock(&f_usb20hdc->lock);
	}

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is ended.\n", __func__);

	return 0;
}

static int f_usb20hdc_udc_gadget_vbus_draw(struct usb_gadget *gadget,
	u32 ma)
{
	/* it is not implementation */
	return -EOPNOTSUPP;
}

static int f_usb20hdc_udc_gadget_pullup(struct usb_gadget *gadget, int is_on)
{
	struct f_usb20hdc_udc *f_usb20hdc;
	unsigned long flags;

	/* check argument */
	if (unlikely(!gadget))
		return -EINVAL;

	/* get a device driver parameter */
	f_usb20hdc = container_of(gadget, struct f_usb20hdc_udc, gadget);

	dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is started.\n", __func__);

	/* get spin lock and disable interrupt, save interrupt status */
	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* check F_USB20HDC controller host mode usage */
	if (unlikely(is_host_mode_usage(f_usb20hdc))) {
		dev_err(f_usb20hdc->gadget.dev.parent,
				"%s():host mode is usage.\n", __func__);
		spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
		dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is ended.\n", __func__);
		return -EPROTO;
	}

	/* check connect request */
	if (is_on) {
		/* check bus connect status */
		if (!f_usb20hdc->bus_connect) {
			dev_err(f_usb20hdc->gadget.dev.parent,
					"%s():D+ terminal can not pull-up.\n",
					__func__);
			spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
			dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is ended.\n", __func__);
			return -EPROTO;
		}

		/* set bus disconnect detect */
		set_bus_connect_detect(f_usb20hdc->reg_base, 0);

		/* enable host connect */
		enable_host_connect(f_usb20hdc, 1);
	} else {
		/* check bus connect status */
		if (!f_usb20hdc->bus_connect) {
			spin_unlock_irqrestore(&f_usb20hdc->lock, flags);
			dev_dbg(f_usb20hdc->gadget.dev.parent,
					"%s() is ended.\n", __func__);
			return 0;
		}

		/* disable host connect */
		enable_host_connect(f_usb20hdc, 0);

		/* set bus connect detect */
		set_bus_connect_detect(f_usb20hdc->reg_base, 1);
	}

	/* release spin lock and enable interrupt, return interrupt status */
	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dev_dbg(f_usb20hdc->gadget.dev.parent, "%s() is ended.\n", __func__);

	return 0;
}

static int f_usb20hdc_udc_gadget_ioctl(struct usb_gadget *gadget, unsigned code,
	unsigned long param)
{
	/* it is not implementation */
	return -ENOIOCTLCMD;
}

static void f_usb20hdc_udc_release(struct device *dev)
{
	/* current non process */
	return;
}

static const struct usb_ep_ops f_usb20hdc_udc_ep_ops = {
	.enable			= f_usb20hdc_udc_ep_enable,
	.disable		= f_usb20hdc_udc_ep_disable,
	.alloc_request		= f_usb20hdc_udc_ep_alloc_request,
	.free_request		= f_usb20hdc_udc_ep_free_request,
	.queue			= f_usb20hdc_udc_ep_queue,
	.dequeue		= f_usb20hdc_udc_ep_dequeue,
	.set_halt		= f_usb20hdc_udc_ep_set_halt,
	.set_wedge		= f_usb20hdc_udc_ep_set_wedge,
	.fifo_status		= f_usb20hdc_udc_ep_fifo_status,
	.fifo_flush		= f_usb20hdc_udc_ep_fifo_flush,
};

static const struct usb_gadget_ops f_usb20hdc_udc_gadget_ops = {
	.get_frame		= f_usb20hdc_udc_gadget_get_frame,
	.wakeup			= f_usb20hdc_udc_gadget_wakeup,
	.set_selfpowered	= f_usb20hdc_udc_gadget_set_selfpowered,
	.vbus_session		= f_usb20hdc_udc_gadget_vbus_session,
	.vbus_draw		= f_usb20hdc_udc_gadget_vbus_draw,
	.pullup			= f_usb20hdc_udc_gadget_pullup,
	.ioctl			= f_usb20hdc_udc_gadget_ioctl,
	.udc_start		= f_usb20hdc_udc_start,
	.udc_stop		= f_usb20hdc_udc_stop,
};

#ifdef CONFIG_USB_F_USB20HDC_OTG_USE_WAKEUP
static ssize_t wakeup_exclude_suspend_int_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n",f_usb20hdc_data->udc_exclu_suspend_int);
}

static ssize_t wakeup_exclude_suspend_int_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t n)
{
	unsigned short value;

	if (sscanf(buf, "%hu", &value)!= 1 ||
		(value != 0 && value != 1)) {
		pr_err("%s:Invalid value\n", __func__);
		return -EINVAL;
	}

	f_usb20hdc_data->udc_exclu_suspend_int = value;
	return n;
}

static struct kobj_attribute sysfs_wakeup_exclu_suspend_int =
	__ATTR(wakeup_exclude_suspend_int, 0644,
	wakeup_exclude_suspend_int_show, wakeup_exclude_suspend_int_store);
#endif

int f_usb20hdc_udc_probe(struct f_usb20hdc_otg *f_otg)
{
	u32 counter;
	struct f_usb20hdc_udc *f_usb20hdc;
	struct platform_device *pdev = f_otg->pdev;
	void __iomem *reg_base;
	int result;

	/* check argument */
	if (unlikely(!pdev))
		return -EINVAL;

	/* allocate F_USB20HDC UDC device driver structure data memory */
	f_usb20hdc = kzalloc(sizeof(*f_usb20hdc), GFP_KERNEL);
	if (!f_usb20hdc) {
		dev_err(&pdev->dev, "%s():kzalloc() is failed.\n", __func__);
		result = -ENOMEM;
		goto err_nomem;
	}

	f_usb20hdc->dev = &pdev->dev;

#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	for (counter = F_USB20HDC_DMA_CH1;
		counter < F_USB20HDC_MAX_DMA_CHANNELS; counter++) {
		f_usb20hdc->dma_data[counter] = f_otg->dma_data[counter];
		f_usb20hdc->dma_data[counter].endpoint_channel = -1;
	}
#endif

	/* get a register base address for a F_USB20HDC device */
	reg_base = remap_iomem_region(f_otg->mem_start, f_otg->mem_size);
	if (!reg_base) {
		dev_err(&pdev->dev, "%s:remap_iomem_region is failed\n",
			__func__);
		result = -ENODEV;
		goto err_res;
	}

	/* check endpoint buffer size */
	if (!is_endpoint_buffer_usable()) {
		dev_err(&pdev->dev, "%s :F_USB20HDC endpoint RAM buffer is insufficient\n",
			__func__);
		result = -ENODEV;
		goto err_irq_buf;
	}

	/* initialize a F_USB20HDC UDC device driver structure */
	f_usb20hdc->f_otg = f_otg;
	f_usb20hdc->test_selector = 0;
	f_usb20hdc->gadget.ops = &f_usb20hdc_udc_gadget_ops;
	f_usb20hdc->gadget.ep0 = &f_usb20hdc->endpoint[0].endpoint;
	INIT_LIST_HEAD(&f_usb20hdc->gadget.ep0->ep_list);
	INIT_LIST_HEAD(&f_usb20hdc->gadget.ep_list);
	f_usb20hdc->gadget.speed = USB_SPEED_UNKNOWN;
	f_usb20hdc->gadget.max_speed = USB_SPEED_HIGH;
	f_usb20hdc->gadget.is_otg = 0;
	f_usb20hdc->gadget.is_a_peripheral = 0;
	f_usb20hdc->gadget.b_hnp_enable = 0;
	f_usb20hdc->gadget.a_hnp_support = 0;
	f_usb20hdc->gadget.a_alt_hnp_support = 0;
	f_usb20hdc->gadget.name = "f_usb20hdc_udc";
	dev_set_name(&f_usb20hdc->gadget.dev, "gadget");
	f_usb20hdc->gadget.dev.release = f_usb20hdc_udc_release;
	f_usb20hdc->gadget_driver = NULL;
	spin_lock_init(&f_usb20hdc->lock);
#if (F_USB20HDC_UDC_USE_AUTO_STALL_RECOVERY == 1)
	setup_timer(&f_usb20hdc->halt_transfer_error_recovery_timer,
			on_recovery_halt_transfer_error, (u32)f_usb20hdc);
#endif
	f_usb20hdc->resource = f_otg->mem_res;
	f_usb20hdc->reg_base = reg_base;
	f_usb20hdc->irq =  f_otg->irq;
	initialize_endpoint_configure(f_usb20hdc);
	for (counter = F_USB20HDC_EP0;
			counter < F_USB20HDC_UDC_MAX_EP; counter++) {
		f_usb20hdc->endpoint[counter].endpoint.driver_data = NULL;
		f_usb20hdc->endpoint[counter].endpoint.ops =
							&f_usb20hdc_udc_ep_ops;
		list_add_tail(&f_usb20hdc->endpoint[counter].endpoint.ep_list,
				counter == F_USB20HDC_EP0 ?
					&f_usb20hdc->gadget.ep0->ep_list :
					&f_usb20hdc->gadget.ep_list);
		f_usb20hdc->endpoint[counter].f_usb20hdc = f_usb20hdc;
		f_usb20hdc->endpoint[counter].endpoint.desc = NULL;
		f_usb20hdc->endpoint[counter].request = NULL;
		INIT_LIST_HEAD(&f_usb20hdc->endpoint[counter].queue);
		f_usb20hdc->endpoint[counter].halt = 0;
		f_usb20hdc->endpoint[counter].force_halt = 0;
		f_usb20hdc->endpoint[counter].null_packet = 0;
		f_usb20hdc->endpoint[counter].dma_transfer = 0;
	}
	f_usb20hdc->device_add = 0;
	f_usb20hdc->bus_connect = 0;
	f_usb20hdc->selfpowered = 1;
	f_usb20hdc->device_state = USB_STATE_NOTATTACHED;
	f_usb20hdc->device_state_last = USB_STATE_NOTATTACHED;
	f_usb20hdc->configure_value_last = 0;
	f_usb20hdc->ctrl_stage = F_USB20HDC_STAGE_SETUP;
	f_usb20hdc->ctrl_pri_dir = 1;
	f_usb20hdc->ctrl_status_delay = 0;
#if defined(CONFIG_USB_F_USB20HDC_OTG_DUAL_ROLE)
	/*this is for otg resume and suspend function*/
	f_usb20hdc->otg_suspend_state = 0;
	f_usb20hdc->gadget_connected = 0;
#endif
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	result = f_usb20hdc_dma_attach(f_usb20hdc->dma_data,
		F_USB20HDC_MAX_DMA_CHANNELS,
		F_USB20HDC_UDC_DMA_TRANS_MAX_BYTES);
	if (result < 0) {
		dev_err(&pdev->dev, "Failed to attach DMA.\n");
		goto err_dma;
	}
#endif

	/* save the private data of a device driver */
	f_otg->f_usb20hdc_udc = f_usb20hdc;
	f_usb20hdc_data = f_usb20hdc;

	/* pre-initialize F_USB20HDC controller */
	initialize_udc_controller(f_usb20hdc, 1);

	/* entry a F_USB20HDC device IRQ */
	result = request_irq(f_usb20hdc->irq, on_usb_function, get_irq_flag(),
						"f_usb20hdc_udc", f_usb20hdc);
	if (result) {
		dev_err(&pdev->dev,
			"%s():request_irq() for F_USB20HDC is failed at %d.\n",
							      __func__, result);
		goto err_req_irq;
	}

	result = usb_add_gadget_udc(&pdev->dev, &f_usb20hdc->gadget);
	if (result) {
		dev_err(&pdev->dev, "usb_add_gadget_udc failed!\n");
		goto err_add_gadget;
	}
#ifdef CONFIG_USB_F_USB20HDC_OTG_USE_WAKEUP
	result = sysfs_create_file(&pdev->dev.kobj,
		&sysfs_wakeup_exclu_suspend_int.attr);
	if (result)
		dev_err(&pdev->dev,
			"sysfs_wakeup_exclu_suspend_int failed\n");
#endif

	/* driver registering log output */
	dev_info(&pdev->dev,
		"F_USB20HDC UDC driver (version %s) is registered\n",
		F_USB20HDC_UDC_DRIVER_VERSION);

	return 0;

err_add_gadget:
	free_irq(f_usb20hdc->irq, f_usb20hdc);
err_req_irq:
	f_otg->f_usb20hdc_udc = NULL;
	f_usb20hdc_data = NULL;
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
err_dma:
	f_usb20hdc_dma_detach(f_usb20hdc->dma_data,
		F_USB20HDC_MAX_DMA_CHANNELS,
		F_USB20HDC_UDC_DMA_TRANS_MAX_BYTES);
#endif
err_irq_buf:
	unmap_iomem_region(reg_base);
err_res:
	kfree(f_usb20hdc);
err_nomem:
	dev_dbg(&pdev->dev, "%s() is ended.\n", __func__);
	return result;
}

#ifdef CONFIG_PM
int f_usb20hdc_udc_ucdwakeup_suspend(struct f_usb20hdc_otg *f_otg)
{
	struct f_usb20hdc_udc *f_usb20hdc = f_otg->f_usb20hdc_udc;
	struct f_usb20hdc_udc_ep *endpoint;
	int i;

	for (i = F_USB20HDC_EP0; i < F_USB20HDC_UDC_MAX_EP; i++) {
		endpoint = &f_usb20hdc->endpoint[i];
		if (!endpoint)
			continue;
		set_udc_wakeup_interrupt(endpoint, 1);
	}

	return 0;
}


int f_usb20hdc_udc_suspend(struct f_usb20hdc_otg *f_otg)
{
	struct f_usb20hdc_udc *f_usb20hdc = f_otg->f_usb20hdc_udc;
	struct device *dev = f_otg->dev;
	unsigned long flags;

	if (!f_usb20hdc) {
		dev_err(dev, "%s():no udc instance.\n", __func__);
		return -EINVAL;
	}

	if (device_may_wakeup(dev)) {
		return f_usb20hdc_udc_ucdwakeup_suspend(f_otg);
	}

#if defined(CONFIG_USB_F_USB20HDC_OTG_DUAL_ROLE)
	if (f_usb20hdc->otg_suspend_state) {
		dev_info(dev, "%s() do nothing casue it's otg suspended.\n",
			__func__);
		return 0;
	}
#endif

	/* disable dev mode interrupt factor*/
	set_dev_inten(f_usb20hdc->reg_base, 0);
	set_vbus_vld_ren(f_usb20hdc->reg_base, 0);
	set_vbus_vld_fen(f_usb20hdc->reg_base, 0);
	clear_vbus_vld_c(f_usb20hdc->reg_base);

	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* disconnect from the host if gadget driver is registered */
	if (f_usb20hdc->device_add)
		enable_communicate(f_usb20hdc, 0);

	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dev_dbg(dev, "%s() is ended.\n", __func__);
	return 0;
}

/* skip initializatio process on resume for USBHCD Wake-Up */
int f_usb20hdc_udc_ucdwakeup_resume(struct f_usb20hdc_otg *f_otg)
{
	struct f_usb20hdc_udc *f_usb20hdc = f_otg->f_usb20hdc_udc;
	void *base_addr = f_usb20hdc->reg_base;
	struct device *dev = f_otg->dev;
	struct f_usb20hdc_udc_ep *endpoint;
	int i;

	dev_dbg(dev, "%s() start\n", __func__);

	if (!f_usb20hdc)
		return 0;

	for (i = F_USB20HDC_EP0; i < F_USB20HDC_UDC_MAX_EP; i++) {
		endpoint = &f_usb20hdc->endpoint[i];
		if (!endpoint)
			continue;

		dev_info(dev, "endpoint[%d] epctrl_int=0x%04x, epctrl_inten=0x%04x\n",
				 i,
				 get_epctrl_register_bits(base_addr, F_USB20HDC_REG_EPCTRL0 + i, 24, 8),
				 get_epctrl_register_bits(base_addr, F_USB20HDC_REG_EPCTRL0 + i, 16, 8));

		set_udc_wakeup_interrupt(endpoint, 0);
	}

	dev_dbg(dev, "%s() finished\n", __func__);

	return 0;
}

int f_usb20hdc_udc_resume(struct f_usb20hdc_otg *f_otg)
{
	struct f_usb20hdc_udc *f_usb20hdc = f_otg->f_usb20hdc_udc;
	struct device *dev = f_otg->dev;
	unsigned long flags;

	dev_dbg(dev, "%s() is started.\n", __func__);

	if (!f_usb20hdc)
		return 0;

	if (device_may_wakeup(dev)) {
		return f_usb20hdc_udc_ucdwakeup_resume(f_otg);
	}

	/* get a device driver parameter */
	if (!f_usb20hdc) {
		dev_info(dev, "%s():no udc instance.\n", __func__);
		return -EINVAL;
	}

#if defined(CONFIG_USB_F_USB20HDC_OTG_DUAL_ROLE)
	if (f_usb20hdc->otg_suspend_state) {
		dev_info(dev, "%s() do nothing casue it's otg suspended.\n",
			__func__);
		return 0;
	}
#endif

	spin_lock_irqsave(&f_usb20hdc->lock, flags);

#ifdef COLD_RESUME_SUPPORT
	/* pre-initialize F_USB20HDC controller */
	initialize_udc_controller(f_usb20hdc, 1);
#endif

	/* enable communicate if gadget driver is registered */
	if (f_usb20hdc->device_add) {
		dev_info(dev, "%s() class driver were connected.\n", __func__);

#ifdef COLD_RESUME_SUPPORT
		/* restore gadget setting : initialize endpoint configure */
		initialize_endpoint_configure(f_usb20hdc);
#endif
#if (F_USB20HDC_UDC_USE_AUTO_STALL_RECOVERY == 1)
		setup_timer(&f_usb20hdc->halt_transfer_error_recovery_timer,
			on_recovery_halt_transfer_error, (u32)f_usb20hdc);
#endif
		enable_communicate(f_usb20hdc, 1);
	}

	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	/* enable dev mode interrupt factor */
	set_vbus_vld_ren(f_usb20hdc->reg_base, 1);
	set_vbus_vld_fen(f_usb20hdc->reg_base, 1);

	dev_dbg(dev, "%s() is ended.\n", __func__);
	return 0;
}

int f_usb20hdc_udc_freeze(struct f_usb20hdc_otg *f_otg)
{
	struct f_usb20hdc_udc *f_usb20hdc = f_otg->f_usb20hdc_udc;
	struct device *dev = f_otg->dev;
	unsigned long flags;

	dev_dbg(dev, "%s() is started.\n", __func__);

	/* get a device driver parameter */
	if (!f_usb20hdc) {
		dev_err(dev, "%s():platform_get_drvdata() is failed.\n",
								__func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* disconnect from the host if gadget driver is registered */
	if (f_usb20hdc->device_add)
		enable_communicate(f_usb20hdc, 0);

	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dev_dbg(dev, "%s() is ended.\n", __func__);

	return 0;
}

int f_usb20hdc_udc_restore(struct f_usb20hdc_otg *f_otg)
{
	struct f_usb20hdc_udc *f_usb20hdc = f_otg->f_usb20hdc_udc;
	struct device *dev = f_otg->dev;
	unsigned long flags;

	dev_dbg(dev, "%s() is started.\n", __func__);

	/* get a device driver parameter */
	if (!f_usb20hdc) {
		dev_err(dev, "%s():platform_get_drvdata() is failed.\n",
								__func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* enable communicate if gadget driver is registered */
	if (f_usb20hdc->device_add) {
#if (F_USB20HDC_UDC_USE_AUTO_STALL_RECOVERY == 1)
		setup_timer(&f_usb20hdc->halt_transfer_error_recovery_timer,
			on_recovery_halt_transfer_error, (u32)f_usb20hdc);
#endif
		enable_communicate(f_usb20hdc, 1);
	}

	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dev_dbg(dev, "%s() is ended.\n", __func__);

	return 0;
}

int f_usb20hdc_udc_thaw(struct f_usb20hdc_otg *f_otg)
{
	struct f_usb20hdc_udc *f_usb20hdc = f_otg->f_usb20hdc_udc;
	struct device *dev = f_otg->dev;
	unsigned long flags;

	dev_dbg(dev, "%s() is started.\n", __func__);

	/* get a device driver parameter */
	if (!f_usb20hdc) {
		dev_err(dev, "%s():platform_get_drvdata() is failed.\n",
								__func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&f_usb20hdc->lock, flags);

	/* enable communicate if gadget driver is registered */
	if (f_usb20hdc->device_add) {
#if (F_USB20HDC_UDC_USE_AUTO_STALL_RECOVERY == 1)
		setup_timer(&f_usb20hdc->halt_transfer_error_recovery_timer,
			on_recovery_halt_transfer_error, (u32)f_usb20hdc);
#endif
		enable_communicate(f_usb20hdc, 1);
	}

	spin_unlock_irqrestore(&f_usb20hdc->lock, flags);

	dev_dbg(dev, "%s() is ended.\n", __func__);

	return 0;
}

#endif /*CONFIG_PM*/

int f_usb20hdc_udc_remove(struct f_usb20hdc_otg *f_otg)
{
	struct f_usb20hdc_udc *f_usb20hdc = f_otg->f_usb20hdc_udc;
	struct platform_device *pdev = f_otg->pdev;

	/* check argument */
	if (unlikely(!pdev))
		return -EINVAL;

	dev_dbg(&pdev->dev, "%s() is started.\n", __func__);

#ifdef CONFIG_USB_F_USB20HDC_OTG_USE_WAKEUP
	sysfs_remove_file(&pdev->dev.kobj,
		&sysfs_wakeup_exclu_suspend_int.attr);
#endif /* CONFIG_USB_F_USB20HDC_OTG_USE_WAKEUP */


	/* get a device driver parameter */
	if (!f_usb20hdc) {
		dev_err(&pdev->dev, "%s():platform_get_drvdata() is failed.\n",
								__func__);
		dev_dbg(&pdev->dev, "%s() is ended.\n", __func__);
		return -EINVAL;
	}

	usb_del_gadget_udc(&f_usb20hdc->gadget);

	/* free HDMAC channel and noncachable buffers */
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	f_usb20hdc_dma_detach(f_usb20hdc->dma_data,
		F_USB20HDC_MAX_DMA_CHANNELS,
		F_USB20HDC_UDC_DMA_TRANS_MAX_BYTES);
#endif

#if defined(CONFIG_USB_F_USB20HDC_OTG_DUAL_ROLE)
	if (!f_usb20hdc->otg_suspend_state)
#endif
		free_irq(f_usb20hdc->irq, f_usb20hdc);

	/* disable dev mode interrupt factor*/
	set_dev_inten(f_usb20hdc->reg_base, 0);
	set_vbus_vld_ren(f_usb20hdc->reg_base, 0);
	set_vbus_vld_fen(f_usb20hdc->reg_base, 0);
	clear_vbus_vld_c(f_usb20hdc->reg_base);

	/* disable communicate */
	enable_communicate(f_usb20hdc, 0);

	/* notify disconnect to gadget driver */
	if ((f_usb20hdc->gadget_driver) &&
			(f_usb20hdc->gadget_driver->disconnect)) {
		f_usb20hdc->gadget_driver->disconnect(&f_usb20hdc->gadget);
		dev_info(&pdev->dev, "gadget_driver->disconnect().\n");
	}

	/* notify unbind to gadget driver */
	if ((f_usb20hdc->gadget_driver) &&
		(f_usb20hdc->gadget_driver->unbind)) {
		f_usb20hdc->gadget_driver->unbind(&f_usb20hdc->gadget);
		dev_info(&pdev->dev, "gadget_driver->unbind().\n");
	}

	/* release device resource */
	f_otg->f_usb20hdc_udc = NULL;
	unmap_iomem_region(f_usb20hdc->reg_base);

	/* free F_USB20HDC UDC device driver structure data memory */
	kfree(f_usb20hdc);

	/* driver deregistering log output */
	dev_info(&pdev->dev, "F_USB20HDC UDC driver is deregistered.\n");

	dev_dbg(&pdev->dev, "%s() is ended.\n", __func__);

	return 0;
}

#if defined(CONFIG_USB_F_USB20HDC_OTG_DUAL_ROLE)
int f_udc_otg_resume(struct f_usb20hdc_otg *f_otg)
{
	struct f_usb20hdc_udc *f_usb20hdc = f_otg->f_usb20hdc_udc;
	struct device *dev = f_otg->dev;
	int ret = 0;

	dev_dbg(dev, "%s() is started from %pS.\n", __func__,
			__builtin_return_address(0));

	if (f_usb20hdc == NULL) {
		dev_err(dev, "Fujitsu udc failed to otg_resume\n");
		return -1;
	}

	/* pre-initialize F_USB20HDC controller */
	initialize_udc_controller(f_usb20hdc, 1);

	/* entry a F_USB20HDC device IRQ */
	ret = request_irq(f_usb20hdc->irq, on_usb_function,
		get_irq_flag(), "f_usb20hdc_udc", f_usb20hdc);

	if (ret < 0) {
		dev_err(dev, "Fujitsu udc failed to resume\n");
		return -1;
	}

	if (f_usb20hdc->gadget_connected) {
		dev_info(dev, "Fujitsu udc's high level gadget connected state : %d",
			f_usb20hdc->gadget_connected);

		/* restore gadget setting : initialize endpoint configure */
		initialize_endpoint_configure(f_usb20hdc);

#if (F_USB20HDC_UDC_USE_AUTO_STALL_RECOVERY == 1)
		setup_timer(
			&f_usb20hdc->halt_transfer_error_recovery_timer,
			on_recovery_halt_transfer_error, (u32)f_usb20hdc);
#endif

		/* restore gadget setting : enable communicate */
		enable_communicate(f_usb20hdc, 1);
	}

#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	ret = f_usb20hdc_dma_attach(f_usb20hdc->dma_data,
		F_USB20HDC_MAX_DMA_CHANNELS,
		F_USB20HDC_UDC_DMA_TRANS_MAX_BYTES);
	if (ret < 0) {
		dev_err(dev, "Failed to attach DMA.\n");
		return -1;
	}
#endif

	dev_dbg(dev, "Fujitsu udc resume.\n");
	f_usb20hdc->otg_suspend_state = 0;
	return 0;
}

int f_udc_otg_suspend(struct f_usb20hdc_otg *f_otg)
{
	struct f_usb20hdc_udc *f_usb20hdc = f_otg->f_usb20hdc_udc;
	struct device *dev = f_otg->dev;

	dev_dbg(dev, "%s() is started from %pS.\n", __func__,
			__builtin_return_address(0));

	if (f_usb20hdc == NULL) {
		pr_err("Fujitsu udc failed to otg_suspend\n");
		return -1;
	}

	/* disable communicate */
	enable_communicate(f_usb20hdc, 0);

	/* quarantine udc driver from any hardware event*/
	free_irq(f_usb20hdc->irq, f_usb20hdc);

	/* free HDMAC channel and noncachable buffers */
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	f_usb20hdc_dma_detach(f_usb20hdc->dma_data,
		F_USB20HDC_MAX_DMA_CHANNELS,
		F_USB20HDC_UDC_DMA_TRANS_MAX_BYTES);
#endif

	dev_dbg(dev, "USB Gadget suspended.\n");
	f_usb20hdc->otg_suspend_state = 1;
	return 0;
}
#endif
