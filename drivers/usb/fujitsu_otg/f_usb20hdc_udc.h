/*
 * linux/drivers/usb/gadget/f_usb20hdc_udc.h - F_USB20HDC USB function
 * controller driver
 *
 * Copyright (C) FUJITSU ELECTRONICS INC. 2011-2012. All rights reserved.
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

#ifndef _F_USB20HDC_UDC_H
#define _F_USB20HDC_UDC_H

#include <linux/platform_data/f_usb20hdc.h>

enum f_usb20hdc_udc_ctrl_stage {
	F_USB20HDC_STAGE_SETUP = 0,		/* SETUP stage */
	F_USB20HDC_STAGE_IN_DATA,		/* IN data stage */
	F_USB20HDC_STAGE_OUT_DATA,		/* OUT data stage */
	F_USB20HDC_STAGE_IN_STATUS,		/* IN status stage */
	F_USB20HDC_STAGE_OUT_STATUS,		/* OUT status stage */
	F_USB20HDC_STAGE_MAX,			/* max value */
};

/* F_USB20HDC driver version */
#define F_USB20HDC_UDC_DRIVER_VERSION		"1.0.5"





/* endpoint channel count */
#define F_USB20HDC_UDC_MAX_EP			6

/* endpoint buffer RAM size */
#define F_USB20HDC_UDC_EP_BUFFER_RAM_SIZE	8192	/* x1[bytes] */

/* F_USB20HDC bus reset dummy disconnect notify */
#define F_USB20HDC_UDC_BUS_RESET_NOTIFY		1	/* 1:notify use */
							/* 0:notify unuse */

/* F_USB20HDC controller hangup recovery mode */
#define F_USB20HDC_UDC_USE_HANGUP_RECOVERY	1	/* 1:recovery use */
							/* 0:recovery unuse */

/* F_USB20HDC stall error recovery mode */
#define F_USB20HDC_UDC_USE_AUTO_STALL_RECOVERY	1	/* 1:recovery use */
							/* 0:recovery unuse */

/* F_USB20HDC stall error recovery wait time */
#define F_USB20HDC_UDC_AUTO_STALL_RECOVERY_TIME	1	/* (1 to 60000)x1[ms] */

/* F_USB20HDC DMA burst transfer mode */
#define F_USB20HDC_UDC_USE_DMA_BURST_TRANSFER	1	/* 1:burst mode */
							/* 0:normal mode */

/* F_USB20HDC DMA controller transfer maximum byte */
#if defined(CONFIG_USB_GADGET_F_USB20HDC_DMA_USE_BOUNCE_BUF) ||\
	defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
#define F_USB20HDC_UDC_DMA_TRANS_MAX_BYTES	16384	/* x1[bytes] */
#else
#define F_USB20HDC_UDC_DMA_TRANS_MAX_BYTES	4194304	/* x1[bytes] */
#endif

/* F_USB20HDC UDC device driver request structure */
struct f_usb20hdc_udc_req {
	struct usb_request request;		/* USB request structure */
	struct list_head queue;			/* request queue head */
	u8 request_execute;			/* request execute flag */
	u8 dma_transfer_buffer_map;		/* DMA trans buffer map flag */
#if defined(CONFIG_USB_GADGET_F_USB20HDC_USED_DMA_TRANSFER) ||\
	defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	u8 dmac_int_occurred;			/* HDMAC trans done irq flag */
	u8 usb_dma_int_occurred;		/*
						 * USB controller's DMA transfer
						 * done irq occurred
						 */
#endif
};

/* F_USB20HDC UDC device driver endpoint structure */
struct f_usb20hdc_udc_ep {
	struct usb_ep endpoint;			/* endpoint structure */
	struct f_usb20hdc_udc *f_usb20hdc;	/* F_USB20HDC driver struct */
	u8 endpoint_channel;			/* endpoint channel */
	u8 transfer_direction;			/* endpoint trans dir flag */
	u8 transfer_type;			/* endpoint transfer type */
	u16 buffer_address_offset[F_USB20HDC_EP_BUFFER_COUNT];
						/* endpoint buffer addr offset*/
	u16 buffer_size;			/* endpoint buffer size */
	u8 buffers;				/* endpoint buffer count */
	u8 pio_auto_change;			/* PIO trans auto change flag */
	u8 in_trans_end_timing;			/*
						 * IN transfer end notify timing
						 * to USB host flag
						 */
	s8 usb_dma_channel;			/* USB controller DMA channel */
	struct f_usb20hdc_udc_req *request;	/* current request structure */
	struct list_head queue;			/* endpoint queue head */
	u8 halt;				/* transfer halt flag */
	u8 force_halt;				/* transfer force halt flag */
	u8 null_packet;				/* NULL packet transfer flag */
	u8 dma_transfer;			/* DMA transfer flag */
};

/* F_USB20HDC UDC device driver structure */
struct f_usb20hdc_udc {
	struct device *dev;
	struct clk *clk_table[CLK_MAX_NUM];
	struct usb_gadget gadget;		/* gadget structure */
	struct usb_gadget_driver *gadget_driver;/* gadget driver structure */
	spinlock_t lock;			/* mutex */
#if (F_USB20HDC_UDC_USE_AUTO_STALL_RECOVERY == 1)
	struct timer_list halt_transfer_error_recovery_timer;
						/*
						 * timer structure for halt
						 * transfer error recovery
						 */
#endif
	struct resource *resource;		/* F_USB20HDC device resource */
	void __iomem *reg_base;	/* F_USB20HDC reg base address*/
	int irq;				/* F_USB20HDC IRQ number */
	struct f_usb20hdc_udc_ep endpoint[F_USB20HDC_UDC_MAX_EP];
						/*
						 * F_USB20HDC UDC driver
						 * endpoint structure array
						 */
	u8 device_add;				/* device driver register flag*/
	u8 bus_connect;				/* bus connect status flag */
	u8 selfpowered;				/* self-powered flag */
	enum usb_device_state device_state;	/* USB device state */
	enum usb_device_state device_state_last;/* last USB device state */
	u8 configure_value_last;		/* last configure value */
	enum f_usb20hdc_udc_ctrl_stage	ctrl_stage;/* control transfer stage */
	u8 ctrl_pri_dir;			/*
						 * control transfer
						 * priority-processing
						 * direction flag
						 */
	u8 ctrl_status_delay;			/*
						 * control transfer status
						 * stage delay flag
						 */
#if defined(CONFIG_USB_GADGET_F_USB20HDC_USED_DMA_TRANSFER) ||\
	defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	struct f_usb20hdc_dma_data
			dma_data[F_USB20HDC_MAX_DMA_CHANNELS];
						/* DMA transfer data array */
#endif
	/*this is for otg resume and suspend function*/
#if (defined(CONFIG_USB_F_USB20HDC_OTG) ||\
	defined(CONFIG_USB_F_USB20HDC_OTG_MODULE))
	u8 gadget_connected;
	u8 otg_suspend_state;
#endif
	struct f_usb20hdc_otg *f_otg;
	u32 test_selector;
	u32 udc_exclu_suspend_int;
};

static const struct endpont_cb ep_config_data[F_USB20HDC_UDC_MAX_EP] = {
	/* endpoint 0 */
	[0] = {
		.name					= "ep0",
		.hs_maxpacket				= 64,
		.fs_maxpacket				= 64,
		.buffer_size				= 64,
		.buffers				= 1,
		.pio_auto_change			= 0,
		.trans_end_timing			= 0,
		.usb_dma_channel			= -1,
	},
	/* endpoint 1 */
	[1] = {
		.name					= "ep1-int",
		.hs_maxpacket				= 1024,
		.fs_maxpacket				= 64,
		.buffer_size				= 1024,
		.buffers				= 1,
		.pio_auto_change			= 1,
		.trans_end_timing			= 1,
		.usb_dma_channel			= -1,
	},
	/* endpoint 2 */
	[2] = {
		.name					= "ep2-bulk",
		.hs_maxpacket				= 512,
		.fs_maxpacket				= 64,
		.buffer_size				= 512,
		.buffers				= 2,
		.pio_auto_change			= 1,
		.trans_end_timing			= 1,
		.usb_dma_channel			= 0,
	},
	/* endpoint 3 */
	[3] = {
		.name					= "ep3-bulk",
		.hs_maxpacket				= 512,
		.fs_maxpacket				= 64,
		.buffer_size				= 512,
		.buffers				= 2,
		.pio_auto_change			= 1,
		.trans_end_timing			= 1,
		.usb_dma_channel			= 1,
	},
	/* endpoint 4 */
	[4] = {
		.name					= "ep4-iso",
		.hs_maxpacket				= 1024,
		.fs_maxpacket				= 1023,
		.buffer_size				= 1024,
		.buffers				= 2,
		.pio_auto_change			= 1,
		.trans_end_timing			= 1,
		.usb_dma_channel			= 0,
	},
	/* endpoint 5 */
	[5] = {
		.name					= "ep5-iso",
		.hs_maxpacket				= 1024,
		.fs_maxpacket				= 1023,
		.buffer_size				= 1024,
		.buffers				= 2,
		.pio_auto_change			= 1,
		.trans_end_timing			= 1,
		.usb_dma_channel			= 1,
	},
};

static inline u8 get_nullresp(void __iomem *base_addr, u8 ep_chan);
static inline u8 get_nackresp(void __iomem *base_addr, u8 ep_chan);
static inline u8 get_readyi_ready_inten(void __iomem *base_addr, u8 ep_chan);
static inline u8 get_readyo_empty_inten(void __iomem *base_addr, u8 ep_chan);
static inline u8 get_ping_inten(void __iomem *base_addr, u8 ep_chan);
static inline u8 get_stalled_inten(void __iomem *base_addr, u8 ep_chan);
static inline u8 get_nack_inten(void __iomem *base_addr, u8 ep_chan);
static inline u8 get_et(void __iomem *base_addr, u8 ep_ch);
static inline u8 get_dir(void __iomem *base_addr, u8 ep_chan);
static inline u8 get_bnum(void __iomem *base_addr, u8 ep_chan);
static inline u8 get_hiband(void __iomem *base_addr, u8 ep_chan);

static void ctrl_epcmd_reg_cache_bits(void __iomem *base_addr,
	u32 register_id, u32 *register_cache)
{
/* EPx Command register bit feild position */
#define F_USB20HDC_REG_EPCMD_BIT_START			0 /* start */
#define F_USB20HDC_REG_EPCMD_BIT_STOP			1 /* stop */
#define F_USB20HDC_REG_EPCMD_BIT_INIT			2 /* init */
#define F_USB20HDC_REG_EPCMD_BIT_BUFWR			3 /* bufwr */
#define F_USB20HDC_REG_EPCMD_BIT_BUFRD			4 /* bufrd */
#define F_USB20HDC_REG_EPCMD_BIT_STALL_SET		5 /* stall_set */
#define F_USB20HDC_REG_EPCMD_BIT_STALL_CLR		6 /* stall_clr */
#define F_USB20HDC_REG_EPCMD_BIT_TOGGLE_SET		7 /* toggle_set */
#define F_USB20HDC_REG_EPCMD_BIT_TOGGLE_CLR		8 /* toggle_clr */
#define F_USB20HDC_REG_EPCMD_BIT_NULLRESP		9 /* nullresp */
#define F_USB20HDC_REG_EPCMD_BIT_NACKRESP		10 /* nackresp */
#define F_USB20HDC_REG_EPCMD_BIT_WRITE_EN		11 /* write_en */
#define F_USB20HDC_REG_EPCMD_BIT_READYI_READY_INTEN	12 /*
							    * readyi_inten[Ep 0]
							    * / ready_inten
							    * [Ep1-15]
							    */
#define F_USB20HDC_REG_EPCMD_BIT_READYO_EMPTY_INTEN	13 /*
							    * readyo_inten[Ep 0]
							    * / empty_inten
							    * [Ep 1-15]
							    */
#define F_USB20HDC_REG_EPCMD_BIT_PING_INTEN		14 /* ping_inten */
#define F_USB20HDC_REG_EPCMD_BIT_STALLED_INTEN		15 /* stalled_inten */
#define F_USB20HDC_REG_EPCMD_BIT_NACK_INTEN		16 /* nack_inten */
#define F_USB20HDC_REG_EPCMD_BIT_READYI_READY_INT_CLR	18 /*
							    * readyi_int_clr
							    * [Ep 0] /
							    * ready_int_clr
							    * [Ep 1-15]
							    */
#define F_USB20HDC_REG_EPCMD_BIT_READYO_EMPTY_INT_CLR	19 /*
							    * readyo_int_clr
							    * [Ep 0] /
							    * ready_empty_clr
							    * [Ep 1-15]
							    */
#define F_USB20HDC_REG_EPCMD_BIT_PING_INT_CLR		20 /* ping_int_clr */
#define F_USB20HDC_REG_EPCMD_BIT_STALLED_INT_CLR	21 /* stalled_int_clr */
#define F_USB20HDC_REG_EPCMD_BIT_NACK_INT_CLR		22 /* nack_int_clr */
#define F_USB20HDC_REG_EPCMD_BIT_ET			23 /* et */
#define F_USB20HDC_REG_EPCMD_BIT_DIR			25 /* dir */
#define F_USB20HDC_REG_EPCMD_BIT_BNUM			26 /* bnum */
#define F_USB20HDC_REG_EPCMD_BIT_HIBAND			28 /* hiband */

	u8 endpoint_channel = register_id - F_USB20HDC_REG_EPCMD0;

	*register_cache &=
		~(((u32)1 << F_USB20HDC_REG_EPCMD_BIT_START) |
		((u32)1 << F_USB20HDC_REG_EPCMD_BIT_STOP) |
		((u32)1 << F_USB20HDC_REG_EPCMD_BIT_INIT) |
		((u32)1 << F_USB20HDC_REG_EPCMD_BIT_BUFWR) |
		((u32)1 << F_USB20HDC_REG_EPCMD_BIT_BUFRD) |
		((u32)1 << F_USB20HDC_REG_EPCMD_BIT_STALL_SET) |
		((u32)1 << F_USB20HDC_REG_EPCMD_BIT_STALL_CLR) |
		((u32)1 << F_USB20HDC_REG_EPCMD_BIT_TOGGLE_SET) |
		((u32)1 << F_USB20HDC_REG_EPCMD_BIT_TOGGLE_CLR) |
		((u32)1 <<
				F_USB20HDC_REG_EPCMD_BIT_READYI_READY_INT_CLR) |
		((u32)1 <<
				F_USB20HDC_REG_EPCMD_BIT_READYO_EMPTY_INT_CLR) |
		((u32)1 << F_USB20HDC_REG_EPCMD_BIT_PING_INT_CLR) |
		((u32)1 << F_USB20HDC_REG_EPCMD_BIT_STALLED_INT_CLR) |
		((u32)1 << F_USB20HDC_REG_EPCMD_BIT_NACK_INT_CLR));

	*register_cache |=
		(((u32)get_nullresp(base_addr, endpoint_channel) <<
				F_USB20HDC_REG_EPCMD_BIT_NULLRESP) |
		((u32)get_nackresp(base_addr, endpoint_channel) <<
				F_USB20HDC_REG_EPCMD_BIT_NACKRESP) |
		((u32)1 << F_USB20HDC_REG_EPCMD_BIT_WRITE_EN) |
		((u32)get_readyi_ready_inten(base_addr, endpoint_channel) <<
				F_USB20HDC_REG_EPCMD_BIT_READYI_READY_INTEN) |
		((u32)get_readyo_empty_inten(base_addr, endpoint_channel) <<
				F_USB20HDC_REG_EPCMD_BIT_READYO_EMPTY_INTEN) |
		((u32)get_ping_inten(base_addr, endpoint_channel)<<
				F_USB20HDC_REG_EPCMD_BIT_PING_INTEN) |
		((u32)get_stalled_inten(base_addr, endpoint_channel) <<
				F_USB20HDC_REG_EPCMD_BIT_STALLED_INTEN) |
		((u32)get_nack_inten(base_addr, endpoint_channel)<<
				F_USB20HDC_REG_EPCMD_BIT_NACK_INTEN) |
		((u32)get_et(base_addr, endpoint_channel) <<
				F_USB20HDC_REG_EPCMD_BIT_ET) |
		((u32)get_dir(base_addr, endpoint_channel) <<
				F_USB20HDC_REG_EPCMD_BIT_DIR) |
		((u32)get_bnum(base_addr, endpoint_channel) <<
				F_USB20HDC_REG_EPCMD_BIT_BNUM) |
		((u32)get_hiband(base_addr, endpoint_channel) <<
				F_USB20HDC_REG_EPCMD_BIT_HIBAND));

	return;
}

static inline void ctrl_reg_cache_bits(
	void __iomem *base_addr, u32 reg_id,
	u32 *reg_cache)
{
	if (reg_id == F_USB20HDC_REG_INTS) {
		ctrl_ints_reg_cache_bits(base_addr, reg_id, reg_cache);
	} else if (reg_id == F_USB20HDC_REG_DEVS) {
		ctrl_devs_reg_cache_bits(base_addr, reg_id, reg_cache);
	} else if (reg_id == F_USB20HDC_REG_OTGSTSC) {
		ctrl_otgstsc_reg_cache_bits(base_addr, reg_id, reg_cache);
	} else if ((reg_id >= F_USB20HDC_REG_EPCMD0) &&
			(reg_id <= F_USB20HDC_REG_EPCMD15)) {
		ctrl_epcmd_reg_cache_bits(base_addr, reg_id, reg_cache);
	} else {
		ctrl_default_reg_cache_bits(base_addr, reg_id, reg_cache);
	}
}

/*endpoint command registers which will have different
 *function in different mode*/
static inline void set_stop(void __iomem *base_addr, u8 ep_ch)
{
	f_cmd0_spin(base_addr, ep_ch);

	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch, 1, 1, 1);
	get_register_bits(base_addr, F_USB20HDC_REG_DEVS, 17, 1) ?
						udelay(250) : mdelay(2);
	f_cmd0_spin(base_addr, ep_ch);
}

static inline void set_stall_set(void __iomem *base_addr, u8 ep_chan)
{
	f_cmd0_spin(base_addr, ep_chan);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_chan, 5, 1, 1);
	f_cmd0_spin(base_addr, ep_chan);
}

static inline void set_stall_clear(void __iomem *base_addr, u8 ep_chan)
{
	f_cmd0_spin(base_addr, ep_chan);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_chan, 6, 1, 1);
	f_cmd0_spin(base_addr, ep_chan);
}

static inline void set_nackresp(void __iomem *base_addr, u8 ep_chan, u8 nak)
{
	f_cmd0_spin(base_addr, ep_chan);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_chan,
								10, 1, nak);
	f_cmd0_spin(base_addr, ep_chan);
}

static inline void set_readyi_ready_inten(void __iomem *base_addr,
	u8 ep_ch, u8 en)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch,
								12, 1, en);
	f_cmd0_spin(base_addr, ep_ch);
}

static inline void set_readyo_empty_inten(void __iomem *base_addr,
	u8 ep_ch, u8 en)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch,
								13, 1, en);
	f_cmd0_spin(base_addr, ep_ch);
}

static inline void set_ping_inten(void __iomem *base_addr, u8 ep_ch, u8 en)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch,
								14, 1, en);
	f_cmd0_spin(base_addr, ep_ch);
}

static inline void set_stalled_inten(void __iomem *base_addr, u8 ep_chan, u8 en)
{
	f_cmd0_spin(base_addr, ep_chan);

	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_chan,
								15, 1, en);
	f_cmd0_spin(base_addr, ep_chan);
}

static inline void set_nack_inten(void __iomem *base_addr, u8 ep_ch, u8 en)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch,
								16, 1, en);
	f_cmd0_spin(base_addr, ep_ch);
}

static inline void clear_readyi_ready_int_clr(void __iomem *base_addr,
								u8 ep_ch)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch, 18, 1, 1);
	f_cmd0_spin(base_addr, ep_ch);
}

static inline void clear_readyo_empty_int_clr(void __iomem *base_addr,
								u8 ep_ch)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch, 19, 1, 1);
	f_cmd0_spin(base_addr, ep_ch);

}

static inline void clear_ping_int_clr(void __iomem *base_addr, u8 ep_ch)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch, 20, 1, 1);
	f_cmd0_spin(base_addr, ep_ch);
}

static inline void clear_stalled_int_clr(void __iomem *base_addr, u8 ep_ch)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch, 21, 1, 1);
	f_cmd0_spin(base_addr, ep_ch);
}

static inline void clear_nack_int_clr(void __iomem *base_addr, u8 ep_ch)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch, 22, 1, 1);
	f_cmd0_spin(base_addr, ep_ch);
}

/* transfer type symbolic constant */
#define TYPE_UNUSED		0x0	/* unused */
#define TYPE_CONTROL		0x0	/* control transfer */
#define TYPE_ISOCHRONOUS	0x1	/* isochronous transfer */
#define TYPE_BULK		0x2	/* bulk transfer */
#define TYPE_INTERRUPT		0x3	/* interrupt transfer */

static inline void set_dir(void __iomem *base_addr, u8 ep_ch, u8 in)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch,
								25, 1, in);
	f_cmd0_spin(base_addr, ep_ch);
}

static inline void set_hiband(void __iomem *base_addr, u8 ep_ch, u8 pks)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch, 28, 2,
		pks);
	f_cmd0_spin(base_addr, ep_ch);
}

/*device mode register*/
static inline void set_reqspeed(void __iomem *base_addr, u8 speed)
{
	set_register_bits(base_addr, F_USB20HDC_REG_DEVC, 0, 1, speed);
}

/* bus speed request symbolic constant */
#define REQ_SPEED_HIGH_SPEED	0	/* high-speed request */
#define REQ_SPEED_FULL_SPEED	1	/* full-speed request */

static inline void set_reqresume(void __iomem *base_addr, u8 request)
{
	set_register_bits(base_addr, F_USB20HDC_REG_DEVC, 2, 1, request);
}

static inline void set_enrmtwkup(void __iomem *base_addr, u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_DEVC, 3, 1, enable);
}

static inline u8 get_enrmtwkup(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_DEVC, 3, 1);
}

static inline void set_disconnect(void __iomem *base_addr, u8 disconnect)
{
	set_register_bits(base_addr, F_USB20HDC_REG_DEVC, 5, 1, disconnect);
}

static inline void set_physusp(void __iomem *base_addr, u8 force_susped)
{
	set_register_bits(base_addr, F_USB20HDC_REG_DEVC, 16, 1, force_susped);
}

static inline void set_suspende_inten(void __iomem *base_addr, u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_DEVC, 24, 1, enable);
}

static inline u8 get_suspende_inten(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_DEVC, 24, 1);
}

static inline void set_suspendb_inten(void __iomem *base_addr, u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_DEVC, 25, 1, enable);
}

static inline u8 get_suspendb_inten(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_DEVC, 25, 1);
}

static inline void set_sof_inten(void __iomem *base_addr, u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_DEVC, 26, 1, enable);
}

static inline u8 get_sof_inten(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_DEVC, 26, 1);
}

static inline void set_setup_inten(void __iomem *base_addr, u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_DEVC, 27, 1, enable);
}

static inline u8 get_setup_inten(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_DEVC, 27, 1);
}

static inline void set_usbrste_inten(void __iomem *base_addr, u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_DEVC, 28, 1, enable);
}

static inline u8 get_usbrste_inten(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_DEVC, 28, 1);
}

static inline void set_usbrstb_inten(void __iomem *base_addr, u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_DEVC, 29, 1, enable);
}

static inline u8 get_usbrstb_inten(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_DEVC, 29, 1);
}

static inline void set_status_ok_inten(void __iomem *base_addr, u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_DEVC, 30, 1, enable);
}

static inline void set_status_ng_inten(void __iomem *base_addr, u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_DEVC, 31, 1, enable);
}

static inline u8 get_suspend(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_DEVS, 0, 1);
}

static inline u8 get_busreset(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_DEVS, 1, 1);
}

static inline u8 get_phyreset(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_DEVS, 16, 1);
}

static inline u8 get_crtspeed(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_DEVS, 17, 1);
}

	/* bus speed symbolic constant */
	#define CRT_SPEED_HIGH_SPEED	0	/* high-speed */
	#define CRT_SPEED_FULL_SPEED	1	/* full-speed */

static inline void clear_suspende_int(void __iomem *base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_DEVS, 24, 1, 0);
}

static inline u8 get_suspende_int(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_DEVS, 24, 1);
}

static inline void clear_suspendb_int(void __iomem *base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_DEVS, 25, 1, 0);
}

static inline u8 get_suspendb_int(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_DEVS, 25, 1);
}

static inline void clear_sof_int(void __iomem *base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_DEVS, 26, 1, 0);
}

static inline u8 get_sof_int(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_DEVS, 26, 1);
}

static inline void clear_setup_int(void __iomem *base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_DEVS, 27, 1, 0);
}

static inline u8 get_setup_int(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_DEVS, 27, 1);
}

static inline void clear_usbrste_int(void __iomem *base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_DEVS, 28, 1, 0);
}

static inline u8 get_usbrste_int(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_DEVS, 28, 1);
}

static inline void clear_usbrstb_int(void __iomem *base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_DEVS, 29, 1, 0);
	return;
}

static inline u8 get_usbrstb_int(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_DEVS, 29, 1);
}

static inline void clear_status_ok_int(void __iomem *base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_DEVS, 30, 1, 0);
}

static inline void clear_status_ng_int(void __iomem *base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_DEVS, 31, 1, 0);
}

static inline void set_func_addr(void __iomem *base_addr, u8 address)
{
	set_register_bits(base_addr, F_USB20HDC_REG_FADDR, 0, 7, address);
}

static inline void set_dev_configured(void __iomem *base_addr, u8 configured)
{
	set_register_bits(base_addr, F_USB20HDC_REG_FADDR, 8, 1, configured);
}

static inline u16 get_timstamp(void __iomem *base_addr)
{
	return (u16)get_register_bits(base_addr, F_USB20HDC_REG_TSTAMP, 0, 11);
}

/*endpoint control area*/
static inline void clear_epctrl(void __iomem *base_addr, u8 ep_ch)
{
	set_register_bits(base_addr, F_USB20HDC_REG_EPCTRL0 + ep_ch, 0, 32,
		ep_ch == F_USB20HDC_EP0 ? 0x00000500 : 0x00000400);
}

static inline u8 get_emptyo_empty(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_EPCTRL0 + ep_ch, 10, 1);
}

static inline u8 get_fullo_full(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_EPCTRL0 + ep_ch, 11, 1);
}

static inline u8 get_ep_en(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
		F_USB20HDC_REG_EPCTRL0 + ep_ch, 0, 1);
}

static inline u8 get_bnum(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
		F_USB20HDC_REG_EPCTRL0 + ep_ch, 4, 2);
}

static inline u8 get_appptr(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
		F_USB20HDC_REG_EPCTRL0 + ep_ch, 6, 2);
}

static inline u8 get_stall(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
		F_USB20HDC_REG_EPCTRL0 + ep_ch, 12, 1);
}

static inline u8 get_et(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
		F_USB20HDC_REG_EPCTRL0 + ep_ch, 1, 2);
}

static inline u8 get_hiband(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_EPCTRL0 + ep_ch, 14, 2);
}

static inline u8 get_nullresp(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_EPCTRL0 + ep_ch, 16, 1);
}

static inline u8 get_nackresp(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_EPCTRL0 + ep_ch, 17, 1);
}

static inline u8 get_readyi_ready_inten(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_EPCTRL0 + ep_ch, 18, 1);
}

static inline u8 get_dir(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
					F_USB20HDC_REG_EPCTRL0 + ep_ch, 3, 1);
}

static inline u8 get_fulli(void __iomem *base_addr)
{
	return (u8)get_epctrl_register_bits(base_addr,
					F_USB20HDC_REG_EPCTRL0, 9, 1);
}

static inline u8 get_readyo_empty_inten(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_EPCTRL0 + ep_ch, 19, 1);
}

static inline u8 get_ping_inten(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_EPCTRL0 + ep_ch, 20, 1);
}

static inline u8 get_stalled_inten(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_EPCTRL0 + ep_ch, 21, 1);
}

static inline u8 get_nack_inten(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_EPCTRL0 + ep_ch, 22, 1);
}

static inline u8 get_readyi_ready_int(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_EPCTRL0 + ep_ch, 26, 1);
}

static inline u8 get_readyo_empty_int(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_EPCTRL0 + ep_ch, 27, 1);
}

static inline u8 get_stalled_int(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_EPCTRL0 + ep_ch, 29, 1);
}

static inline u8 get_ping_int(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_EPCTRL0 + ep_ch, 28, 1);
}

static inline u8 get_nack_int(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_EPCTRL0 + ep_ch, 30, 1);
}

static inline void set_bus_connect_detect(
	void __iomem *virtual_base_address, u8 connect)
{
	if (connect) {
		/* set bus connect detect */
		set_vbus_vld_fen(virtual_base_address, 0);
		set_vbus_vld_ren(virtual_base_address, 1);
	} else {
		/* set bus disconnect detect */
		set_vbus_vld_ren(virtual_base_address, 0);
		set_vbus_vld_fen(virtual_base_address, 1);
	}
}

static inline u8 is_bus_connected(void __iomem *virtual_base_address)
{
	/* get bus connected */
	return get_vbus_vld(virtual_base_address) ? 1 : 0;
}

static inline void enable_dplus_line_pullup(
			void __iomem *virtual_base_address, u8 enable)
{
	if (enable) {
		/* pull-up D+ terminal */
		set_physusp(virtual_base_address, 0);
		set_disconnect(virtual_base_address, 0);
	} else {
		/* pull-down D+ terminal */
		set_disconnect(virtual_base_address, 1);
		set_physusp(virtual_base_address, 1);
	}
}
static inline u8 get_linestate(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
						F_USB20HDC_REG_PORTSTSC, 5, 2);
}

extern void f_usb20hdc_soft_reset_ip(struct f_usb20hdc_otg *f_otg);

#ifdef LOWLEVEL_DEBUG
#define dbg_print dev_dbg
#else
#define dbg_print(...) do { } while (0)
#endif

#endif
