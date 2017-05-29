/*
 * linux/drivers/usb/host/f_usb20hdc-hcd.h - F_USB20HDC USB
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

#ifndef _F_USB20HDC_HCD_H
#define _F_USB20HDC_HCD_H

/* workaroud for some hub can't be initialized */
#define HUB_RESET_WORKAROUND

/*
 * using mapped device address is due to the only 4 bits device
 * addressing capability in F_USB20HDC IP.
*/
#define VIRTUAL_DEVICE_ADDRESS

#include <linux/platform_data/f_usb20hdc.h>

/* F_USB20HDC HCD driver transfer type enumeration */
enum f_usb20hdc_trans_type {
	TRANSFER_TYPE_CONTROL = 0,		/* control Transfer */
	TRANSFER_TYPE_BULK_IN,			/* bulk IN Transfer */
	TRANSFER_TYPE_BULK_OUT,			/* bulk OUT Transfer */
	TRANSFER_TYPE_INTERRUPT_IN,		/* interrupt IN Transfer */
	TRANSFER_TYPE_INTERRUPT_OUT,		/* interrupt OUT Transfer */
	TRANSFER_TYPE_ISOCHRONOUS_IN,		/* isochronous IN Transfer */
	TRANSFER_TYPE_ISOCHRONOUS_OUT,		/* isochronous OUT Transfer */
};

/* F_USB20HDC HCD driver control transfer stage enumeration */
enum f_usb20hdc_ctrl_stage {
	CONTROL_TRANSFER_STAGE_SETUP = 0,	/* SETUP stage */
	CONTROL_TRANSFER_STAGE_IN_DATA,		/* IN data stage */
	CONTROL_TRANSFER_STAGE_OUT_DATA,	/* OUT data stage */
	CONTROL_TRANSFER_STAGE_IN_STATUS,	/* IN status stage */
	CONTROL_TRANSFER_STAGE_OUT_STATUS,	/* OUT status stage */
};

/* F_USB20HDC driver version */
#define F_USB20HDC_HCD_CONFIG_DRIVER_VERSION	"0.9.4"


/* F_USB20HDC HCFRMINIT register frminit bit value */
#define F_USB20HDC_HCD_CONFIG_FRMINIT_BIT_VALUE	7499


/* endpoint channel count */
#define F_USB20HDC_HCD_MAX_EP			8

/* maximium buffer size per endpoint */
#define F_USB20HDC_HCD_MAX_BUFFER_PER_EP	512	/* x1[bytes] */

/* endpoint buffer RAM size */
#define F_USB20HDC_HCD_EP_BUFFER_RAM_SIZE	32768	/* x1[bytes] */

#if defined(CONFIG_USB_F_USB20HDC_HCD_USE_BOUNCE_BUF) ||\
	defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF)
/* F_USB20HDC maximium DMA transfer size */
#define F_USB20HDC_HCD_MAX_DMA_TRANSFER_SIZE	65536	/* x1[bytes] */
#else
/* F_USB20HDC maximium DMA transfer size */
#define F_USB20HDC_HCD_MAX_DMA_TRANSFER_SIZE	4194304	/* x1[bytes] */
#endif

/* F_USB20HDC speed mode */
#define F_USB20HDC_HCD_USE_HIGH_SPEED_MODE	1	/* 1:high speed mode */
							/* 0:full speed mode */

/* F_USB20HDC maximium ports of root hub */
#define F_USB20HDC_HCD_ROOT_HUB_MAX_PORT	1

/* F_USB20HDC HCD device driver request structure */
struct f_usb20hdc_hcd_req {
	struct urb *urb;		/* USB request block structure */
	struct list_head queue;		/* request queue head */
	u8 request_execute;	/* request execute flag */
	u8 endpoint_channel;	/* endpoint that handling the request */
	u16 maxpacket;	/* max pkt size for endpoint req to */
#if defined(CONFIG_USB_F_USB20HDC_HCD_USED_DMA_TRANSFER) ||\
	defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
#if !(defined(CONFIG_USB_F_USB20HDC_HCD_USE_BOUNCE_BUF) ||\
	defined(CONFIG_USB_F_USB20HDC_OTG_USE_BOUNCE_BUF))
	u8 dma_transfer_buffer_map;	/* DMA trans buffer map flag */
#endif
	u8 dmac_int_occurred;
	u8 usb_dma_int_occurred;
#endif
};

/* F_USB20HDC HCD split token structure */
struct f_usb20hdc_hcd_split_token {
	u8 hub_address;	/* hub address */
	u8 port_address;	/* port address */
	u8 startbit;		/* Start bit */
};

/* F_USB20HDC HCD device driver endpoint structure */
struct f_usb20hdc_hcd_ep {
	struct f_usb20hdc_hcd *f_usb20hdc;		/*
							 * F_USB20HDC HCD
							 * driver structure
							 */
	u8 endpoint_channel;			/* ep channel */
	enum f_usb20hdc_trans_type transfer_type;	/* ep trans type*/
	u16 max_packet_size;			/* max packet size*/
	u32 transfer_packets;			/* trans pkt count */
	int transfer_status;				/* transfer status */
	u16 buffer_address_offset[F_USB20HDC_EP_BUFFER_COUNT];
							/*
							 * endpoint buffer
							 * address offset
							 */
	u16 buffer_size;			/* endpoint buf size */
	u8 buffers;				/* endpoint buf count */
	struct f_usb20hdc_hcd_req *request;		/* current request */
	struct list_head queue;				/* endpoint list head */
	u8 inuse_devnum;			/*
							 * device number for
							 * dynamic endpoints
							 * in use
							 */
	u8 inuse_epnum;			/*
							 * endpoint number for
							 * dynamic endpoints
							 * in use
							 */
	struct f_usb20hdc_hcd_split_token split;	/* split token */
#if defined(CONFIG_USB_F_USB20HDC_HCD_USED_DMA_TRANSFER) ||\
	defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	s8 usb_dma_channel;			/*
							 * USB controller's DMA
							 * channel using by
							 * endpoint currently
							 */
#endif
};

#define MAX_DEV_ADDR 15 /* 1~15 is available */
struct dev_addr_mapping {
	u8 logical_addr;
	u8 hub_addr;
	u8 port_num;
	u8 used;
};

/* F_USB20HDC HCD device driver structure */
struct f_usb20hdc_hcd {
	struct device *dev;
	struct clk *clk_table[CLK_MAX_NUM];
	spinlock_t lock;			/* mutex */
	struct resource *resource;		/* F_USB20HDC dev resource */
	void __iomem *register_base_address;	/* F_USB20HDC reg base addr */
	int irq;				/* F_USB20HDC IRQ number */
	struct f_usb20hdc_hcd_ep endpoint[F_USB20HDC_HCD_MAX_EP];
						/*
						 * F_USB20HDC HCD device driver
						 * endpoint structure array
						 */
	u16 eps_inuse_map;		/* map for endpoints in use */
	u8 highspeed_support;	/* high-speed support flag */
	u8 bulkin_remain;		/* endpoint remain TX data */
	enum f_usb20hdc_ctrl_stage ctrl_stage;	/* control transfer stage */
#if defined(CONFIG_USB_F_USB20HDC_HCD_USED_DMA_TRANSFER) ||\
	defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	struct f_usb20hdc_dma_data
				dma_data[F_USB20HDC_MAX_DMA_CHANNELS];
						/* DMA transfer data array */
#endif
	unsigned long next_statechange;
#ifdef CONFIG_PM_RUNTIME
	u8 wakeup_req;
#endif
#ifdef HUB_RESET_WORKAROUND
	u8 reset_start;
	u8 reset_failed_cnt;
#endif
#ifdef VIRTUAL_DEVICE_ADDRESS
	struct dev_addr_mapping dev_addr_table[MAX_DEV_ADDR];
#endif
	struct f_usb20hdc_otg *f_otg;
	u8 wakeup_from_poweroff; /*0 :wake up, 1: suspended*/
	struct dentry *file;
};

static const struct endpont_cb endpoint_configuration_data[
			F_USB20HDC_HCD_MAX_EP] = {
	/* endpoint 0 */
	[0] = {
		.name						= "ep0",
		.hs_maxpacket			= 64,
		.fs_maxpacket			= 64,
		.buffer_size					= 64,
		.buffers					= 2,
		.pio_auto_change			= 0,
		.trans_end_timing	= 0,
		.usb_dma_channel				= -1,
		.dma_channel					= -1,
	},
	/* endpoint 1 */
	[1] = {
		.name						= "ep1-bulk",
		.hs_maxpacket			= 512,
		.fs_maxpacket			= 64,
		.buffer_size					= 512,
		.buffers					= 2,
		.pio_auto_change			= 1,
		.trans_end_timing	= 1,
		.usb_dma_channel				= 0,
		.dma_channel					= 0,
	},
	/* endpoint 2 */
	[2] = {
		.name						= "ep2-bulk",
		.hs_maxpacket			= 512,
		.fs_maxpacket			= 64,
		.buffer_size					= 512,
		.buffers					= 2,
		.pio_auto_change			= 1,
		.trans_end_timing	= 1,
		.usb_dma_channel				= 1,
		.dma_channel					= 1,
	},
	/* endpoint 3 */
	[3] = {
		.name						= "ep3-int",
		.hs_maxpacket			= 512,
		.fs_maxpacket			= 64,
		.buffer_size					= 512,
		.buffers					= 2,
		.pio_auto_change			= 1,
		.trans_end_timing	= 1,
		.usb_dma_channel				= -1,
		.dma_channel					= -1,
	},
	/* endpoint 4 */
	[4] = {
		.name						= "ep4-int",
		.hs_maxpacket			= 512,
		.fs_maxpacket			= 64,
		.buffer_size					= 512,
		.buffers					= 2,
		.pio_auto_change			= 1,
		.trans_end_timing	= 1,
		.usb_dma_channel				= -1,
		.dma_channel					= -1,
	},
	/* endpoint 5 */
	[5] = {
		.name						= "ep5-iso",
		.hs_maxpacket			= 1024,
		.fs_maxpacket			= 1023,
		.buffer_size					= 1024,
		.buffers					= 1,
		.pio_auto_change			= 1,
		.trans_end_timing	= 1,
		.usb_dma_channel				= -1,
		.dma_channel					= -1,
	},
	/* endpoint 6 */
	[6] = {
		.name						= "ep6-iso",
		.hs_maxpacket			= 1024,
		.fs_maxpacket			= 1023,
		.buffer_size					= 1024,
		.buffers					= 1,
		.pio_auto_change			= 1,
		.trans_end_timing	= 1,
		.usb_dma_channel				= -1,
		.dma_channel					= -1,
	},
	/* endpoint 7 */
	[7] = {
		.name						= "ep7-int",
		.hs_maxpacket			= 512,
		.fs_maxpacket			= 64,
		.buffer_size					= 512,
		.buffers					= 2,
		.pio_auto_change			= 1,
		.trans_end_timing	= 1,
		.usb_dma_channel				= -1,
		.dma_channel					= -1,
	},
};

static inline u8 get_nextlink(void __iomem *base_addr, u8 ep_ch);
static inline u8 get_nlinkinvalid(void __iomem *base_addr, u8 ep_ch);
static inline u8 get_sendpid(void __iomem *base_addr, u8 ep_ch);
static inline u8 get_et(void __iomem *base_addr, u8 ep_ch);
static inline u8 get_sc(void __iomem *base_addr, u8 ep_ch);
static inline u8 get_bnum(void __iomem *base_addr, u8 ep_ch);
static inline u8 get_speed(void __iomem *base_addr, u8 ep_ch);

static inline void ctrl_epcmd_reg_cache_bits(void __iomem *base_addr,
	u32 register_id, u32 *register_cache)
{
/* EPx Command register bit feild position */
#define F_USB20HDC_REG_EPCMD_BIT_START		0 /* start */
#define F_USB20HDC_REG_EPCMD_BIT_STOP		1 /* stop */
#define F_USB20HDC_REG_EPCMD_BIT_INIT		2 /* init */
#define F_USB20HDC_REG_EPCMD_BIT_BUFWR		3 /* bufwr */
#define F_USB20HDC_REG_EPCMD_BIT_BUFRD		4 /* bufrd */
#define F_USB20HDC_REG_EPCMD_BIT_TOGGLE_SET	7 /* toggle_set */
#define F_USB20HDC_REG_EPCMD_BIT_TOGGLE_CLR	8 /* toggle_clr */
#define F_USB20HDC_REG_EPCMD_BIT_WRITE_EN	11 /* write_en */
#define F_USB20HDC_REG_EPCMD_BIT_NEXTLINK	12 /* nextlink */
#define F_USB20HDC_REG_EPCMD_BIT_NLINKINVALID	15 /* nlinkinvalid */
#define F_USB20HDC_REG_EPCMD_BIT_SENDPID	16 /* sendpid */
#define F_USB20HDC_REG_EPCMD_BIT_ERRCNT_CLR	18 /* errcnt_clr */
#define F_USB20HDC_REG_EPCMD_BIT_STATUS_CLR	19 /* status_clr */
#define F_USB20HDC_REG_EPCMD_BIT_ET		23 /* et */
#define F_USB20HDC_REG_EPCMD_BIT_SC		25 /* sc */
#define F_USB20HDC_REG_EPCMD_BIT_BNUM		26 /* bnum */
#define F_USB20HDC_REG_EPCMD_BIT_SPEED		28 /* speed */

	u8 ep_channel = register_id - F_USB20HDC_REG_EPCMD0;

	*register_cache &=
		~(((u32)1 << F_USB20HDC_REG_EPCMD_BIT_START) |
		((u32)1 << F_USB20HDC_REG_EPCMD_BIT_STOP) |
		((u32)1 << F_USB20HDC_REG_EPCMD_BIT_INIT) |
		((u32)1 << F_USB20HDC_REG_EPCMD_BIT_BUFWR) |
		((u32)1 << F_USB20HDC_REG_EPCMD_BIT_BUFRD) |
		((u32)1 << F_USB20HDC_REG_EPCMD_BIT_TOGGLE_SET) |
		((u32)1 << F_USB20HDC_REG_EPCMD_BIT_TOGGLE_CLR) |
		((u32)1 << F_USB20HDC_REG_EPCMD_BIT_ERRCNT_CLR) |
		((u32)1 << F_USB20HDC_REG_EPCMD_BIT_STATUS_CLR));

	*register_cache |=
		(((u32)1 << F_USB20HDC_REG_EPCMD_BIT_WRITE_EN) |
		((u32)get_nextlink(base_addr, ep_channel) <<
					F_USB20HDC_REG_EPCMD_BIT_NEXTLINK) |
		((u32)get_nlinkinvalid(base_addr, ep_channel) <<
				F_USB20HDC_REG_EPCMD_BIT_NLINKINVALID) |
		((u32)get_sendpid(base_addr, ep_channel) <<
					F_USB20HDC_REG_EPCMD_BIT_SENDPID) |
		((u32)get_et(base_addr, ep_channel) <<
					F_USB20HDC_REG_EPCMD_BIT_ET) |
		((u32)get_sc(base_addr, ep_channel) <<
					F_USB20HDC_REG_EPCMD_BIT_SC) |
		((u32)get_bnum(base_addr, ep_channel) <<
					F_USB20HDC_REG_EPCMD_BIT_BNUM) |
		((u32)get_speed(base_addr, ep_channel) <<
					F_USB20HDC_REG_EPCMD_BIT_SPEED));

	return;
}

static inline void ctrl_reg_cache_bits(
	void __iomem *base_addr, u32 reg_id,
	u32 *reg_cache)
{
	if (reg_id == F_USB20HDC_REG_INTS) {
		ctrl_ints_reg_cache_bits(base_addr, reg_id, reg_cache);
	} else if (reg_id == F_USB20HDC_REG_PORTSC) {
		ctrl_portsc_reg_cache_bits(base_addr, reg_id, reg_cache);
	} else if (reg_id == F_USB20HDC_REG_PORTSTSC) {
		ctrl_portstsc_reg_cache_bits(base_addr, reg_id, reg_cache);
	} else if (reg_id == F_USB20HDC_REG_HOSTEVENTSRC) {
		ctrl_hosteventsrc_reg_cache_bits(base_addr, reg_id, reg_cache);
	} else if (reg_id == F_USB20HDC_REG_OTGSTSC) {
		ctrl_otgstsc_reg_cache_bits(base_addr, reg_id, reg_cache);
	} else if ((reg_id >= F_USB20HDC_REG_EPCMD0) &&
			(reg_id <= F_USB20HDC_REG_EPCMD7)) {
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
	get_epctrl_register_bits(base_addr, F_USB20HDC_REG_HCEPCTRL1_0 + ep_ch,
		26, 2) == 2 ? udelay(250) : mdelay(2);
	f_cmd0_spin(base_addr, ep_ch);
}

static void set_nextlink(void __iomem *base_addr, u8 ep_ch,
							u8 hcep)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr,
			F_USB20HDC_REG_EPCMD0 + ep_ch, 12, 3, hcep);
	f_cmd0_spin(base_addr, ep_ch);
}

static inline void set_nlinkinvalid(void __iomem *base_addr,
					u8 ep_ch, u8 inv)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch,
								15, 1, inv);
	f_cmd0_spin(base_addr, ep_ch);
}

static inline void set_sendpid(void __iomem *base_addr, u8 ep_ch,
							u8 pid)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch,
								16, 2, pid);
	f_cmd0_spin(base_addr, ep_ch);
}

/* PID symbolic constant */
#define PID_OUT			0	/* OUT */
#define PID_IN			1	/* IN */
#define PID_SETUP		2	/* SETUP */
#define PID_PING		3	/* PING */

static inline void clear_errcnt_clr(void __iomem *base_addr,
							u8 ep_ch)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch, 18, 1, 1);
	f_cmd0_spin(base_addr, ep_ch);
}

static inline void clear_status_clr(void __iomem *base_addr,
							u8 ep_ch)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch, 19, 1, 1);
	f_cmd0_spin(base_addr, ep_ch);
}

/* transfer type symbolic constant */
#define TYPE_UNUSED		0x0	/* unused */
#define TYPE_CONTROL		0x0	/* control transfer */
#define TYPE_ISOCHRONOUS	0x1	/* isochronous transfer */
#define TYPE_BULK		0x2	/* bulk transfer */
#define TYPE_INTERRUPT		0x3	/* interrupt transfer */

static inline void set_sc(void __iomem *base_addr, u8 ep_ch,
							u8 csp)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch,
								25, 1, csp);
	f_cmd0_spin(base_addr, ep_ch);
}

static inline void set_speed(void __iomem *base_addr, u8 ep_ch,
							u8 speed)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch,
								28, 2, speed);
	f_cmd0_spin(base_addr, ep_ch);
		;
}

/* endpoint speed request symbolic constant */
#define SPEED_FULL_SPEED	0	/* full-speed */
#define SPEED_LOW_SPEED		1	/* lowl-speed */
#define SPEED_HIGH_SPEED	2	/* high-speed */

/*Host mode register*/
static inline u8 get_port_over_current_rhs(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
						F_USB20HDC_REG_PORTSC, 0, 1);
}

static inline u8 get_port_power_ctl_rhs(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
						F_USB20HDC_REG_PORTSC, 1, 1);
}

static inline u8 get_forcefs_rhs(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
						F_USB20HDC_REG_PORTSC, 3, 1);
}

static inline u8 get_port_connection_rhs(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
						F_USB20HDC_REG_PORTSC, 4, 1);
}

static inline u8 get_port_reset_rhs(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
						F_USB20HDC_REG_PORTSC, 5, 1);
}

static inline u8 get_port_enable_rhs(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
						F_USB20HDC_REG_PORTSC, 6, 1);
}

static inline u8 get_port_low_speed_rhs(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
						F_USB20HDC_REG_PORTSC, 7, 1);
}

static inline u8 get_port_high_speed_rhs(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
						F_USB20HDC_REG_PORTSC, 8, 1);
}

static inline u8 get_port_wakeup_rhs(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
						F_USB20HDC_REG_PORTSC, 9, 1);
}

static inline u8 get_port_suspended_rhs(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
						F_USB20HDC_REG_PORTSC, 10, 1);
}

static inline u8 get_port_resuming_rhs(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
						F_USB20HDC_REG_PORTSC, 11, 1);
}

static inline void set_forcefs_req(void __iomem *base_addr,
	u8 force_fs)
{
	set_register_bits(base_addr, F_USB20HDC_REG_PORTSC, 23, 1, force_fs);
}

static inline void set_port_reset_req(void __iomem *base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_PORTSC, 25, 1, 1);
}

static inline void set_port_enable_req(void __iomem *base_addr,
	u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_PORTSC, 26, 1, enable);
}

static inline void set_port_wakeup_req(void __iomem *base_addr,
	u8 wakeup_on)
{
	set_register_bits(base_addr, F_USB20HDC_REG_PORTSC, 29, 1, wakeup_on);
}

static inline void set_port_suspend_req(void __iomem *base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_PORTSC, 30, 1, 1);
	return;
}

static inline void set_port_resume_req(void __iomem *base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_PORTSC, 31, 1, 1);
	return;
}

static inline void clear_port_connection_c(void __iomem *base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_PORTSTSC, 0, 1, 0);
}

static inline u8 get_port_connection_c(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
						F_USB20HDC_REG_PORTSTSC, 0, 1);
}

static inline void clear_port_enable_c(void __iomem *base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_PORTSTSC, 1, 1, 0);
}

static inline u8 get_port_enable_c(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
						F_USB20HDC_REG_PORTSTSC, 1, 1);
}

static inline void clear_port_suspend_c(void __iomem *base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_PORTSTSC, 2, 1, 0);
}

static inline u8 get_port_suspend_c(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
						F_USB20HDC_REG_PORTSTSC, 2, 1);
}

static inline void clear_port_ov_curr_c(void __iomem *base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_PORTSTSC, 3, 1, 0);
}

static inline u8 get_port_ov_curr_c(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
						F_USB20HDC_REG_PORTSTSC, 3, 1);
}

static inline void clear_port_reset_c(void __iomem *base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_PORTSTSC, 4, 1, 0);
}

static inline u8 get_port_reset_c(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
						F_USB20HDC_REG_PORTSTSC, 4, 1);
}

static inline u8 get_linestate(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
						F_USB20HDC_REG_PORTSTSC, 5, 2);
}

/* line state symbolic constant */
#define LINESTATE_DP_LOW_DM_LOW		0x0 /* D+ low level and D- low  */
#define LINESTATE_DP_HIGH_DM_LOW	0x1 /* D+ high level and D- low */
#define LINESTATE_DP_LOW_DM_HIGH	0x2 /* D+ low level and D- high  */
#define LINESTATE_DP_HIGH_DM_HIGH	0x3 /* D+ high level and D- high */

static inline void clear_sofstart(void __iomem *base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HOSTEVENTSRC, 0, 1, 0);
}

static inline u8 get_sofstart(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
					F_USB20HDC_REG_HOSTEVENTSRC, 0, 1);
}

static inline void clear_frameov(void __iomem *base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HOSTEVENTSRC, 1, 1, 0);
}

static inline u8 get_frameov(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
					F_USB20HDC_REG_HOSTEVENTSRC, 1, 1);
}

static inline u8 get_dport_evt(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
					F_USB20HDC_REG_HOSTEVENTSRC, 2, 1);
}

static inline void clear_trans_done(void __iomem *base_addr,
							u8 ep_ch)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HOSTEVENTSRC,
				8 + ep_ch, 1, 0);
}

static inline u8 get_trans_done(void __iomem *base_addr,
	u8 ep_ch)
{
	return (u8)get_register_bits(base_addr,
				F_USB20HDC_REG_HOSTEVENTSRC, 8 + ep_ch, 1);
}

static inline void set_sofstart_inten(void __iomem *base_addr,
							u8 en)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HOSTINTEN, 0, 1, en);
}

static inline u8 get_sofstart_inten(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
					F_USB20HDC_REG_HOSTINTEN, 0, 1);
}

static inline void set_frameov_inten(void __iomem *base_addr, u8 en)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HOSTINTEN, 1, 1, en);
}

static inline u8 get_frameov_inten(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
					F_USB20HDC_REG_HOSTINTEN, 1, 1);
}

static inline void set_port_connection_c_inten(void __iomem *base_addr,
	u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HOSTINTEN, 2, 1, enable);
}

static inline u8 get_port_connection_c_inten(
						void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
					F_USB20HDC_REG_HOSTINTEN, 2, 1);
}

static inline void set_port_enable_c_inten(void __iomem *base_addr,
	u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HOSTINTEN, 3, 1, enable);
}

static inline u8 get_port_enable_c_inten(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
					F_USB20HDC_REG_HOSTINTEN, 3, 1);
}

static inline void set_port_suspend_c_inten(void __iomem *base_addr,
	u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HOSTINTEN, 4, 1, enable);
}

static inline u8 get_port_suspend_c_inten(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
					F_USB20HDC_REG_HOSTINTEN, 4, 1);
}

static inline void set_port_ov_curr_c_inten(void __iomem *base_addr,
	u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HOSTINTEN, 5, 1, enable);
}

static inline u8 get_port_ov_curr_c_inten(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
					F_USB20HDC_REG_HOSTINTEN, 5, 1);
}

static inline void set_port_reset_c_inten(void __iomem *base_addr,
	u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HOSTINTEN, 6, 1, enable);
}

static inline u8 get_port_reset_c_inten(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
					F_USB20HDC_REG_HOSTINTEN, 6, 1);
}

static inline void set_trans_done_inten(void __iomem *base_addr,
	u8 ep_ch, u8 enable)
{
	set_register_bits(base_addr,
				F_USB20HDC_REG_HOSTINTEN, 8 + ep_ch, 1, enable);
}

static inline u8 get_trans_done_inten(void __iomem *base_addr,
	u8 ep_ch)
{
	return (u8)get_register_bits(base_addr,
				F_USB20HDC_REG_HOSTINTEN, 8 + ep_ch, 1);
}

static inline void set_sofevtinterval(void __iomem *base_addr,
	u8 interval)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HOSTINTEN,
						16, 3, interval);
}

/* SOF event interval symbolic constant */
#define SOFEVTINTERVAL_1UFRAME		0x0	/* 1uFrame(125[us] */
#define SOFEVTINTERVAL_2UFRAME		0x1	/* 2uFrame(250[us] */
#define SOFEVTINTERVAL_4UFRAME		0x2	/* 4uFrame(500[us] */
#define SOFEVTINTERVAL_8UFRAME		0x3	/* 8uFrame(1[ms] */
#define SOFEVTINTERVAL_16UFRAME		0x4	/* 16uFrame(2[ms] */
#define SOFEVTINTERVAL_32UFRAME		0x5	/* 32uFrame(4[ms] */
#define SOFEVTINTERVAL_64UFRAME		0x6	/* 64uFrame(8[ms] */
#define SOFEVTINTERVAL_128UFRAME	0x7	/* 128uFrame(16[ms] */

static inline void set_frmidx(void __iomem *base_addr,
	u16 frame, u8 uframe)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HCFRMIDX, 0, 3, uframe);
	set_register_bits(base_addr, F_USB20HDC_REG_HCFRMIDX, 3, 11, frame);
	return;
}

static inline u16 get_sofv(void __iomem *base_addr)
{
	return (u16)get_register_bits(base_addr,
					F_USB20HDC_REG_HCFRMIDX, 16, 11);
}

static inline void set_frminit(void __iomem *base_addr, u16 count)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HCFRMINIT, 0, 13, count);
}

static inline u16 get_frmcnt(void __iomem *base_addr)
{
	return (u16)get_register_bits(base_addr,
					F_USB20HDC_REG_HCFRMINIT, 16, 16);
}

static inline void set_hcrun(void __iomem *base_addr, u16 run)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HCCTRL, 0, 1, run);
}

static inline u8 get_hcrun(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
						F_USB20HDC_REG_HCCTRL, 0, 1);
}

static inline void set_startlink(void __iomem *base_addr, u8 ep_ch)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HCSTLINK, 0, 3, ep_ch);
}



/*endpoint control area*/
static inline void clear_hcepctrl(void __iomem *base_addr, u8 ep_ch)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HCEPCTRL1_0 + ep_ch,
				0, 32, 0x00000400);
	set_register_bits(base_addr, F_USB20HDC_REG_HCEPCTRL2_0 + ep_ch,
				0, 32, 0x00000000);
}

static inline u8 get_empty(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_HCEPCTRL1_0 + ep_ch, 10, 1);
}

static inline u8 get_full(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_HCEPCTRL1_0 + ep_ch, 11, 1);
}

static inline u8 get_trans_en(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
		F_USB20HDC_REG_HCEPCTRL1_0 + ep_ch, 0, 1);
}

static inline u8 get_bnum(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
			F_USB20HDC_REG_HCEPCTRL1_0 + ep_ch, 4, 2);
}

static inline u8 get_appptr(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
		F_USB20HDC_REG_HCEPCTRL1_0 + ep_ch, 6, 2);
}

static inline u8 get_sc(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
			F_USB20HDC_REG_HCEPCTRL1_0 + ep_ch, 12, 1);
}

static inline u8 get_et(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
		F_USB20HDC_REG_HCEPCTRL1_0 + ep_ch, 1, 2);
}

static inline u8 get_errcnt(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_HCEPCTRL1_0 + ep_ch, 14, 2);
}

static inline u8 get_status_halt(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_HCEPCTRL1_0 + ep_ch, 16, 1);
}

static inline u8 get_status_stall(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_HCEPCTRL1_0 + ep_ch, 17, 1);
}

static inline u8 get_status_missed_micro_frame(void __iomem *base_addr,
	u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_HCEPCTRL1_0 + ep_ch, 18, 1);
}

static inline u8 get_hcptr(void __iomem *base_addr,
	u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_HCEPCTRL1_0 + ep_ch, 8, 2);
}

static inline u8 get_toggle(void __iomem *base_addr,
	u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_HCEPCTRL1_0 + ep_ch, 13, 1);
}

static inline u8 get_nyetcnt(void __iomem *base_addr,
	u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_HCEPCTRL1_0 + ep_ch, 20, 2);
}
static inline u8 get_nextlink(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_HCEPCTRL1_0 + ep_ch, 28, 3);
}

static inline u8 get_nlinkinvalid(void __iomem *base_addr,
	u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_HCEPCTRL1_0 + ep_ch, 31, 1);
}

static inline u8 get_sendpid(void __iomem *base_addr,
	u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_HCEPCTRL1_0 + ep_ch, 24, 2);
}

static inline u8 get_speed(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_epctrl_register_bits(base_addr,
				F_USB20HDC_REG_HCEPCTRL1_0 + ep_ch, 26, 2);
}

static inline void set_startbit(void __iomem *base_addr, u8 ep_ch,
	u8 low_speed)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HCEPCTRL2_0 + ep_ch,
				1, 1, low_speed);
}

static inline void set_hubportnumber(void __iomem *base_addr,
	u8 ep_ch, u8 number)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HCEPCTRL2_0 + ep_ch,
				4, 4, number);
}

#ifndef VIRTUAL_DEVICE_ADDRESS
static inline void set_hubaddr(void __iomem *base_addr, u8 ep_ch,
	u8 addr, struct usb_hcd *hcd)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HCEPCTRL2_0 + ep_ch, 8, 4,
		addr);
}
#endif

static inline void set_endpnumber(void __iomem *base_addr, u8 ep_ch,
	u8 number)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HCEPCTRL2_0 + ep_ch,
				12, 4, number);
}

#ifndef VIRTUAL_DEVICE_ADDRESS
static inline void set_funcaddr(void __iomem *base_addr, u8 ep_ch,
	u8 addr, struct usb_hcd *hcd)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HCEPCTRL2_0 + ep_ch, 16, 4,
		addr);
}
#endif
static inline void set_interval(void __iomem *base_addr, u8 ep_ch,
	u16 interval)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HCEPCTRL2_0 + ep_ch,
				20, 10, interval);
}

static inline void set_fmtsel(void __iomem *base_addr, u8 ep_ch,
	u8 format)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HCEPCTRL2_0 + ep_ch,
				30, 1, format);
}

static inline void set_intervalsel(void __iomem *base_addr,
				u8 ep_ch, u8 unit)
{
	set_register_bits(base_addr, F_USB20HDC_REG_HCEPCTRL2_0 + ep_ch,
				31, 1, unit);
}

static inline struct f_usb20hdc_hcd *hcd_to_f_usb20hdc(struct usb_hcd *hcd)
{
	return (struct f_usb20hdc_hcd *)(hcd->hcd_priv);
}

static inline struct usb_hcd *f_usb20hdc_to_hcd(
	struct f_usb20hdc_hcd *f_usb20hdc)
{
	return container_of((void *)f_usb20hdc, struct usb_hcd, hcd_priv);
}

extern void f_usb20hdc_soft_reset_ip(struct f_usb20hdc_otg *f_otg);

#ifdef DEBUG
void dbg_print_connection(struct device *pdev, void __iomem *base_addr);
#else
#define dbg_print_connection(pdev, base_addr) do { } while (0)
#endif

#ifdef LOWLEVEL_DEBUG
#define dbg_print dev_dbg
#else
#define dbg_print(...) do { } while (0)
#endif

#endif
