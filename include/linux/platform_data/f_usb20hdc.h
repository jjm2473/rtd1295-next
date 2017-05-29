/*
 *  linux/arch/arm/mach-mb8ac0300/include/mach/f_usb20hdc.h
 *
 * Copyright (C) 2012 FUJITSU SEMICONDUCTOR LIMITED.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __MACH_F_USB20HDC_H
#define __MACH_F_USB20HDC_H

/* defect of faked full-speed device on DP/DM */
/* #define PHY_DEFECT_WORKAROUND */

/*
 * it's bad for USB suspend interrupt might be
 * earlier than VBUS drop-to-low interrupt.
 * This is still not identified as soft or hard defect.
 */
#define GADGET_CONNECTION_BUG

/* faked power-off IP simulation*/
#define POWER_OFF_SIMULATION

/* make controller can resume from power off or hibernation */
#define COLD_RESUME_SUPPORT

/*
 * F_USB20HDC IP has maximun two clock source so far,
 * it has two clock source from clock modules in mb8ac0300 platfoem,
 * and has one source in mb8aa0350.
*/
#define CLK_MAX_NUM	3

static inline void ctrl_reg_cache_bits(void __iomem *base_addr, u32 register_id,
	u32 *register_cache);

/* F_USB20HDC USB controller DMA channel count */
#define F_USB20HDC_MAX_DMA_CHANNELS		2

/* platform data information structure of f_usb20hdc driver */
struct f_usb20hdc_pdata {
	u32	dma_dreq[F_USB20HDC_MAX_DMA_CHANNELS];		/* dreq number
								 * for usb dma
								 * channel array
								 */
	u32	hdmac_channel[F_USB20HDC_MAX_DMA_CHANNELS];	/* HDMAC channel
								 * for usb dma
								 * channel array
								 */
};

/* F_USB20HDC C_HSEL area address offset */
#define F_USB20HDC_C_HSEL_ADDR_OFFSET	0x00000
/* F_USB20HDC D_HSEL area address offset */
#define F_USB20HDC_D_HSEL_ADDR_OFFSET	0x10000

/* endpoint counter count per one endpoint */
#define F_USB20HDC_EP_BUFFER_COUNT		2

/* endpoint configuration structure array table */
struct endpont_cb {
	/*
	 * endpoint name string
	 * [notice]:The following constant definition is used.
	 *	endpoint 0 is "ep0" fixation
	 *	unused	= ""
	 *	used	= "ep(number)(in or out)-(bulk or iso or int)"
	 *	[Example]
	 *	"ep1", "ep2", ... address is fixed, not direction or type
	 *	"ep1in, "ep2out", ... address and direction are fixed, not type
	 *	"ep1-bulk", "ep2-int", ... address and type are fixed, not dir
	 *	"ep1in-bulk", "ep2out-iso", ... all three are fixed
	 */
	char *name;
	/*
	 * endpoint transfer maximum packet size for high-speed
	 * [notice]:unusing it is referred to as '0'
	 */
	u16	hs_maxpacket;
	/*
	 * endpoint transfer maximum packet size for full-speed
	 * [notice]:unusing it is referred to as '0'
	 */
	u16	fs_maxpacket;
	/*
	 * endpoint buffer size(x1[bytes])
	 * [notice]:unusing it is referred to as '0'
	 */
	u16	buffer_size;
	/*
	 * endpoint buffer count
	 * [notice]:unusing it is referred to as '0', and endpoint 0 is 1 fixed
	 */
	u8	buffers;
	/*
	 * PIO transfer auto change flag
	 * [notice]:unusing it is referred to as '0', and endpoint 0 is 0 fixed
	 */
	u8	pio_auto_change;
	/*
	 * IN transfer end notify timing to USB host flag
	 * [notice]:unusing it is referred to as '0', and endpoint 0 is 0 fixed
	 */
	u8	trans_end_timing;
	/*
	 * USB controller DMA channel for endpoint
	 * [notice]:unusing it is referred to as '-1'
	 */
	s8	usb_dma_channel;
	/*
	 * DMA controller DMA channel for endpoint
	 * [notice]:unusing it is referred to as '-1'
	 */
	s8	dma_channel;
};

/* F_USB20HDC device driver DMA data structure */
struct f_usb20hdc_dma_data {
	u32 hdmac_channel;			/* HDMAC transfer channel */
	u32 dreq;				/* HDMAC transfer dreq */
	s8 endpoint_channel;			/* endpoint channel using DMA */
	struct hdmac_req hdmac_req;		/* HDMAC request structure */
	void *buffer;				/*
						 * DMA transfer noncachable
						 * buffer's virtual address
						 */
	dma_addr_t dma_buffer;			/*
						 * DMA transfer noncachable
						 * buffer's physical address
						 */
	dma_addr_t epbuf_dma_addr;		/*
						 * DMA transfer buffer
						 * pyhsical address
						 */
};

/* F_USB20HDC HCD device driver structure */
struct f_usb20hdc_otg {
	struct platform_device *pdev;
	struct device *dev;
	struct clk *clk_table[CLK_MAX_NUM];
	void __iomem *reg_base;	/* F_USB20HDC reg base addr */
	int irq;				/* F_USB20HDC IRQ number */
	struct resource *mem_res;
	resource_size_t mem_start;
	resource_size_t mem_size;
	u8 host_working;
	 int otg_id;
	struct delayed_work switch_to_host;
	struct delayed_work switch_to_gadget;
	struct mutex role_switch_lock;
	struct usb_hcd *hcd;
	void *f_usb20hdc_udc;
#if defined(CONFIG_USB_F_USB20HDC_OTG_USED_DMA_TRANSFER)
	struct f_usb20hdc_pdata pdata;
	struct f_usb20hdc_dma_data dma_data[F_USB20HDC_MAX_DMA_CHANNELS];
#endif
	s8 mode;
	struct dentry *root;
	struct dentry *file;
	char debug_str[20];
};

/* F_USB20HDC controller register ID Enumeration constant */
#define F_USB20HDC_REG_CONF		0	/* System Configuration */
#define F_USB20HDC_REG_MODE		1	/* Oepration Mode */
#define F_USB20HDC_REG_INTEN		2	/* Global Interrupt Enable */
#define F_USB20HDC_REG_INTS		3	/* Global Interrupt Status */
#define F_USB20HDC_REG_EPCMD0		4	/* EP0 Command */
#define F_USB20HDC_REG_EPCMD1		5	/* EP1 Command */
#define F_USB20HDC_REG_EPCMD2		6	/* EP2 Command */
#define F_USB20HDC_REG_EPCMD3		7	/* EP3 Command */
#define F_USB20HDC_REG_EPCMD4		8	/* EP4 Command */
#define F_USB20HDC_REG_EPCMD5		9	/* EP5 Command */
#define F_USB20HDC_REG_EPCMD6		10	/* EP6 Command */
#define F_USB20HDC_REG_EPCMD7		11	/* EP7 Command */
#define F_USB20HDC_REG_EPCMD8		12	/* EP8 Command *//*dev mode*/
#define F_USB20HDC_REG_EPCMD9		13	/* EP9 Command *//*dev mode*/
#define F_USB20HDC_REG_EPCMD10		14	/* EP10 Command *//*dev mode*/
#define F_USB20HDC_REG_EPCMD11		15	/* EP11 Command *//*dev mode*/
#define F_USB20HDC_REG_EPCMD12		16	/* EP12 Command *//*dev mode*/
#define F_USB20HDC_REG_EPCMD13		17	/* EP13 Command *//*dev mode*/
#define F_USB20HDC_REG_EPCMD14		18	/* EP14 Command *//*dev mode*/
#define F_USB20HDC_REG_EPCMD15		19	/* EP15 Command *//*dev mode*/
#define F_USB20HDC_REG_DEVC		20 /* Device Control */ /*dev mode*/
#define F_USB20HDC_REG_DEVS		21	/* Device Status *//*dev mode*/
#define F_USB20HDC_REG_FADDR		22 /* Function Address *//*dev mode*/
#define F_USB20HDC_REG_TSTAMP		23 /* Device Time Stamp *//*dev mode*/
#define F_USB20HDC_REG_PORTSC		24 /* Port Status  Control */
#define F_USB20HDC_REG_PORTSTSC		25 /* Port Status Change */
#define F_USB20HDC_REG_HOSTEVENTSRC	26 /* Host Event Factor */
#define F_USB20HDC_REG_HOSTINTEN	27 /* Host Interrupt Enable */
#define F_USB20HDC_REG_HCFRMIDX		28 /* HC Frame Index */
#define F_USB20HDC_REG_HCFRMINIT	29 /* HC Frame Init */
#define F_USB20HDC_REG_HCCTRL		30 /* HC Control */
#define F_USB20HDC_REG_HCSTLINK		31 /* HC Start Link */
#define F_USB20HDC_REG_OTGC		32	/* OTG Control */
#define F_USB20HDC_REG_OTGSTS		33	/* OTG Status */
#define F_USB20HDC_REG_OTGSTSC		34	/* OTG Status Change */
#define F_USB20HDC_REG_OTGSTSFALL	35	/* OTG Status Fall Detect */
#define F_USB20HDC_REG_OTGSTSRISE	36	/* OTG Status Rise Detect */
#define F_USB20HDC_REG_OTGTC		37 /* OTG Timer Control */
#define F_USB20HDC_REG_OTGT		38 /* OTG Timer */
#define F_USB20HDC_REG_DMAC1		39 /* DMA1 Control */
#define F_USB20HDC_REG_DMAS1		40 /* DMA1 Status */
#define F_USB20HDC_REG_DMATCI1		41 /* DMA1 Total Trans Bytes */
#define F_USB20HDC_REG_DMATC1		42 /* DMA1 Total Trans Bytes Counter */
#define F_USB20HDC_REG_DMAC2		43 /* DMA2 Control */
#define F_USB20HDC_REG_DMAS2		44 /* DMA2 Status */
#define F_USB20HDC_REG_DMATCI2		45 /* DMA2 Total Trans Bytes */
#define F_USB20HDC_REG_DMATC2		46 /* DMA2 Total Trans Bytes Counter */
#define F_USB20HDC_REG_TESTC		47	/* Test Control */
#define F_USB20HDC_REG_HCEPCTRL1_0	48 /* HCEP1_0 Control */
#define F_USB20HDC_REG_HCEPCTRL1_1	49 /* HCEP1_1 Control */
#define F_USB20HDC_REG_HCEPCTRL1_2	50 /* HCEP1_2 Control */
#define F_USB20HDC_REG_HCEPCTRL1_3	51 /* HCEP1_3 Control */
#define F_USB20HDC_REG_HCEPCTRL1_4	52 /* HCEP1_4 Control */
#define F_USB20HDC_REG_HCEPCTRL1_5	53 /* HCEP1_5 Control */
#define F_USB20HDC_REG_HCEPCTRL1_6	54 /* HCEP1_6 Control */
#define F_USB20HDC_REG_HCEPCTRL1_7	55 /* HCEP1_7 Control */
#define F_USB20HDC_REG_HCEPCTRL2_0	56 /* HCEP2_0 Control */
#define F_USB20HDC_REG_HCEPCTRL2_1	57 /* HCEP2_1 Control */
#define F_USB20HDC_REG_HCEPCTRL2_2	58 /* HCEP2_2 Control */
#define F_USB20HDC_REG_HCEPCTRL2_3	59 /* HCEP2_3 Control */
#define F_USB20HDC_REG_HCEPCTRL2_4	60 /* HCEP2_4 Control */
#define F_USB20HDC_REG_HCEPCTRL2_5	61 /* HCEP2_5 Control */
#define F_USB20HDC_REG_HCEPCTRL2_6	62 /* HCEP2_6 Control */
#define F_USB20HDC_REG_HCEPCTRL2_7	63 /* HCEP2_7 Control */
#define F_USB20HDC_REG_EPCTRL0		64	/* EP0 Control *//*dev mode*/
#define F_USB20HDC_REG_EPCTRL1		65	/* EP1 Control *//*dev mode*/
#define F_USB20HDC_REG_EPCTRL2		66	/* EP2 Control *//*dev mode*/
#define F_USB20HDC_REG_EPCTRL3		67	/* EP3 Control *//*dev mode*/
#define F_USB20HDC_REG_EPCTRL4		68	/* EP4 Control *//*dev mode*/
#define F_USB20HDC_REG_EPCTRL5		69	/* EP5 Control *//*dev mode*/
#define F_USB20HDC_REG_EPCTRL6		70	/* EP6 Control *//*dev mode*/
#define F_USB20HDC_REG_EPCTRL7		71	/* EP7 Control *//*dev mode*/
#define F_USB20HDC_REG_EPCTRL8		72	/* EP8 Control *//*dev mode*/
#define F_USB20HDC_REG_EPCTRL9		73	/* EP9 Control *//*dev mode*/
#define F_USB20HDC_REG_EPCTRL10		74	/* EP10 Control *//*dev mode*/
#define F_USB20HDC_REG_EPCTRL11		75	/* EP11 Control *//*dev mode*/
#define F_USB20HDC_REG_EPCTRL12		76	/* EP12 Control *//*dev mode*/
#define F_USB20HDC_REG_EPCTRL13		77	/* EP13 Control *//*dev mode*/
#define F_USB20HDC_REG_EPCTRL14		78	/* EP14 Control *//*dev mode*/
#define F_USB20HDC_REG_EPCTRL15		79	/* EP15 Control *//*dev mode*/
#define F_USB20HDC_REG_EPCONF0		80 /* HCEP0 Config */
#define F_USB20HDC_REG_EPCONF1		81 /* HCEP1 Config */
#define F_USB20HDC_REG_EPCONF2		82 /* HCEP2 Config */
#define F_USB20HDC_REG_EPCONF3		83 /* HCEP3 Config */
#define F_USB20HDC_REG_EPCONF4		84 /* HCEP4 Config */
#define F_USB20HDC_REG_EPCONF5		85 /* HCEP5 Config */
#define F_USB20HDC_REG_EPCONF6		86 /* HCEP6 Config */
#define F_USB20HDC_REG_EPCONF7		87 /* HCEP7 Config */
#define F_USB20HDC_REG_EPCONF8		88	/* EP8 Config *//*dev mode*/
#define F_USB20HDC_REG_EPCONF9		89	/* EP9 Config *//*dev mode*/
#define F_USB20HDC_REG_EPCONF10		90	/* EP10 Config *//*dev mode*/
#define F_USB20HDC_REG_EPCONF11		91	/* EP11 Config *//*dev mode*/
#define F_USB20HDC_REG_EPCONF12		92	/* EP12 Config *//*dev mode*/
#define F_USB20HDC_REG_EPCONF13		93	/* EP13 Config *//*dev mode*/
#define F_USB20HDC_REG_EPCONF14		94	/* EP14 Config *//*dev mode*/
#define F_USB20HDC_REG_EPCONF15		95	/* EP15 Config *//*dev mode*/
#define F_USB20HDC_REG_EPCOUNT0		96 /* HCEP0 Counter */
#define F_USB20HDC_REG_EPCOUNT1		97 /* HCEP1 Counter */
#define F_USB20HDC_REG_EPCOUNT2		98 /* HCEP2 Counter */
#define F_USB20HDC_REG_EPCOUNT3		99 /* HCEP3 Counter */
#define F_USB20HDC_REG_EPCOUNT4		100 /* HCEP4 Counter */
#define F_USB20HDC_REG_EPCOUNT5		101 /* HCEP5 Counter */
#define F_USB20HDC_REG_EPCOUNT6		102 /* HCEP6 Counter */
#define F_USB20HDC_REG_EPCOUNT7		103 /* HCEP7 Counter */
#define F_USB20HDC_REG_EPCOUNT8		104 /* HCEP8 Counter */
#define F_USB20HDC_REG_EPCOUNT9		105 /* HCEP9 Counter */
#define F_USB20HDC_REG_EPCOUNT10		106 /* HCEP10 Counter */
#define F_USB20HDC_REG_EPCOUNT11		107 /* HCEP11 Counter */
#define F_USB20HDC_REG_EPCOUNT12		108 /* HCEP12 Counter */
#define F_USB20HDC_REG_EPCOUNT13		109 /* HCEP13 Counter */
#define F_USB20HDC_REG_EPCOUNT14		110 /* HCEP14 Counter */
#define F_USB20HDC_REG_EPCOUNT15		111 /* HCEP15 Counter */
#define F_USB20HDC_REG_EPCOUNT16		112 /* HCEP16 Counter */
#define F_USB20HDC_REG_EPCOUNT17		113 /* HCEP17 Counter */
#define F_USB20HDC_REG_EPCOUNT18		114 /* HCEP18 Counter */
#define F_USB20HDC_REG_EPCOUNT19		115 /* HCEP19 Counter */
#define F_USB20HDC_REG_EPCOUNT20		116 /* HCEP20 Counter */
#define F_USB20HDC_REG_EPCOUNT21		117 /* HCEP21 Counter */
#define F_USB20HDC_REG_EPCOUNT22		118 /* HCEP22 Counter */
#define F_USB20HDC_REG_EPCOUNT23		119 /* HCEP23 Counter */
#define F_USB20HDC_REG_EPCOUNT24		120 /* HCEP24 Counter */
#define F_USB20HDC_REG_EPCOUNT25		121 /* HCEP25 Counter */
#define F_USB20HDC_REG_EPCOUNT26		122 /* HCEP26 Counter */
#define F_USB20HDC_REG_EPCOUNT27		123 /* HCEP27 Counter */
#define F_USB20HDC_REG_EPCOUNT28		124 /* HCEP28 Counter */
#define F_USB20HDC_REG_EPCOUNT29	125 /* HCEP29 Counter */
#define F_USB20HDC_REG_EPCOUNT30	126 /* HCEP30 Counter */
#define F_USB20HDC_REG_EPCOUNT31	127 /* HCEP31 Counter */
#define F_USB20HDC_REG_EPBUF		128 /* HCEP Buffer  */
#define F_USB20HDC_REG_DMA1		129 /* DMA 1 Addr Convert Area */
#define F_USB20HDC_REG_DMA2		130 /* DMA 1 Addr Convert Area */
#define F_USB20HDC_REG_MAX		131 /* Max Value */

/* DMAx Control register ID table array */
static const u32 dmac_register[] = {
	F_USB20HDC_REG_DMAC1,	/* DMA1 Control */
	F_USB20HDC_REG_DMAC2,	/* DMA1 Control */
};

/* DMAx Status register ID table array */
static const u32 dmas_register[] = {
	F_USB20HDC_REG_DMAS1,	/* DMA1 Status */
	F_USB20HDC_REG_DMAS2,	/* DMA1 Status */
};

/* DMAx Total Transfer Bytes Setting register ID table array */
static const u32 dmatci_register[] = {
	F_USB20HDC_REG_DMATCI1,	/* DMA1 Total Transfer Bytes Setting */
	F_USB20HDC_REG_DMATCI2,	/* DMA2 Total Transfer Bytes Setting */
};

/* DMAx Total Transfer Bytes Counter register ID table array */
static const u32 dmatc_register[] = {
	F_USB20HDC_REG_DMATC1,	/* DMA1 Total Transfer Bytes Counter */
	F_USB20HDC_REG_DMATC2,	/* DMA2 Total Transfer Bytes Counter */
};

#define U2H_R (1 << 0)
#define U2H_W (1 << 1)

/* F_USB20HDC controller register structures array */
static const struct {
	u32 address_offset;	/* register address offset */
	u8 access;	/* register access flags  */
} f_usb20hdc_register[F_USB20HDC_REG_MAX] = {
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0000, U2H_R | U2H_W },
						/* F_USB20HDC_REG_CONF */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0004, U2H_R | U2H_W },
						/* F_USB20HDC_REG_MODE */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0008, U2H_R | U2H_W },
					/* F_USB20HDC_REG_INTEN */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x000c, U2H_R | U2H_W },
					/* F_USB20HDC_REG_INTS */
					/* 0x00100x003FReserved */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0040, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCMD0 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0044, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCMD1 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0048, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCMD2 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x004c, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCMD3 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0050, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCMD4 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0054, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCMD5 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0058, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCMD6 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x005c, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCMD7 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0060, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCMD8 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0064, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCMD9 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0068, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCMD10 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x006c, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCMD11 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0070, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCMD12 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0074, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCMD13 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0078, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCMD14 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x007c, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCMD15 */
					/* 0x00800x01ffReserved */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0200, U2H_R | U2H_W },
					/* F_USB20HDC_REG_DEVC */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0204, U2H_R | U2H_W },
					/* F_USB20HDC_REG_DEVS */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0208, U2H_R | U2H_W },
					/* F_USB20HDC_REG_FADDR */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x020c, U2H_R},
					/* F_USB20HDC_REG_TSTAMP */
					/* 0x021`0x02ffFReserved */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0100, U2H_R | U2H_W },
					/* F_USB20HDC_REG_PORTSC */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0104, U2H_R | U2H_W },
					/* F_USB20HDC_REG_PORTSTSC */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0108, U2H_R | U2H_W },
					/* F_USB20HDC_REG_HOSTEVENTSRC */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x010c, U2H_R | U2H_W },
					/* F_USB20HDC_REG_HOSTINTEN */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0110, U2H_R | U2H_W },
					/* F_USB20HDC_REG_HCFRMIDX */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0114, U2H_R | U2H_W },
					/* F_USB20HDC_REG_HCFRMINIT */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0118, U2H_R | U2H_W },
					/* F_USB20HDC_REG_HCCTRL */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x011c, U2H_R | U2H_W },
					/* F_USB20HDC_REG_HCSTLINK */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0300, U2H_R | U2H_W },
					/* F_USB20HDC_REG_OTGC */
					/* 0x0304`0x030fFReserved */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0310, U2H_R },
					/* F_USB20HDC_REG_OTGSTS */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0314, U2H_R | U2H_W },
					/* F_USB20HDC_REG_OTGSTSC */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0318, U2H_R | U2H_W },
					/* F_USB20HDC_REG_OTGSTSFALL */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x031c, U2H_R | U2H_W },
					/* F_USB20HDC_REG_OTGSTSRISE */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0320, U2H_R | U2H_W },
					/* F_USB20HDC_REG_OTGTC */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0324, U2H_R | U2H_W },
					/* F_USB20HDC_REG_OTGT */
					/* 0x032`0x03ffFReserved */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0400, U2H_R | U2H_W },
					/* F_USB20HDC_REG_DMAC1 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0404, U2H_R },
					/* F_USB20HDC_REG_DMAS1 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0408, U2H_R | U2H_W },
					/* F_USB20HDC_REG_DMATCI1 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x040c, U2H_R },
					/* F_USB20HDC_REG_DMATC1 */
					/* 0x04100x041fReserved */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0420, U2H_R | U2H_W },
					/* F_USB20HDC_REG_DMAC2 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0424, U2H_R },
					/* F_USB20HDC_REG_DMAS2 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0428, U2H_R | U2H_W },
					/* F_USB20HDC_REG_DMATCI2 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x042c, U2H_R },
					/* F_USB20HDC_REG_DMATC2 */
					/* 0x043`0x04ffReserved */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x0500, U2H_R | U2H_W },
					/* F_USB20HDC_REG_TESTC */
					/* 0x0504`0x7fffFReserved */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8000, U2H_R | U2H_W },
					/* F_USB20HDC_REG_HCEPCTRL1_0 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8008, U2H_R | U2H_W },
					/* F_USB20HDC_REG_HCEPCTRL1_1 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8010, U2H_R | U2H_W },
					/* F_USB20HDC_REG_HCEPCTRL1_2 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8018, U2H_R | U2H_W },
					/* F_USB20HDC_REG_HCEPCTRL1_3 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8020, U2H_R | U2H_W },
					/* F_USB20HDC_REG_HCEPCTRL1_4 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8028, U2H_R | U2H_W },
					/* F_USB20HDC_REG_HCEPCTRL1_5 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8030, U2H_R | U2H_W },
					/* F_USB20HDC_REG_HCEPCTRL1_6 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8038, U2H_R | U2H_W },
					/* F_USB20HDC_REG_HCEPCTRL1_7 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8004, U2H_R | U2H_W },
					/* F_USB20HDC_REG_HCEPCTRL2_0 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x800c, U2H_R | U2H_W },
					/* F_USB20HDC_REG_HCEPCTRL2_1 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8014, U2H_R | U2H_W },
					/* F_USB20HDC_REG_HCEPCTRL2_2 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x801c, U2H_R | U2H_W },
					/* F_USB20HDC_REG_HCEPCTRL2_3 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8024, U2H_R | U2H_W },
					/* F_USB20HDC_REG_HCEPCTRL2_4 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x802c, U2H_R | U2H_W },
					/* F_USB20HDC_REG_HCEPCTRL2_5 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8034, U2H_R | U2H_W },
					/* F_USB20HDC_REG_HCEPCTRL2_6 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x803c, U2H_R | U2H_W },
					/* F_USB20HDC_REG_HCEPCTRL2_7 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8000, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCTRL0 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8004, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCTRL1 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8008, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCTRL2 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x800c, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCTRL3 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8010, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCTRL4 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8014, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCTRL5 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8018, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCTRL6 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x801c, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCTRL7 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8020, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCTRL8 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8024, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCTRL9 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8028, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCTRL10 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x802c, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCTRL11 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8030, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCTRL12 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8034, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCTRL13 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8038, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCTRL14 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x803c, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCTRL15 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8040, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCONF0 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8044, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCONF1 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8048, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCONF2 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x804c, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCONF3 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8050, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCONF4 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8054, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCONF5 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8058, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCONF6 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x805c, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCONF7 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8060, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCONF8 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8064, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCONF9 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8068, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCONF10 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x806c, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCONF11 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8070, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCONF12 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8074, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCONF13 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8078, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCONF14 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x807c, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCONF15 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8080, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT0 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8084, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT1 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8088, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT2 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x808c, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT3 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8090, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT4 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8094, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT5 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8098, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT6 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x809c, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT7 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80a0, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT8 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80a4, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT9 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80a8, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT10 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80ac, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT11 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80b0, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT12 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80b4, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT13 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80b8, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT14 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80bc, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT15 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80c0, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT16 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80c4, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT17 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80c8, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT18 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80cc, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT19 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80d0, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT20 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80d4, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT21 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80d8, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT22 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80dc, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT23 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80e0, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT24 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80e4, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT25 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80e8, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT26 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80ec, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT27 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80f0, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT28 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80f4, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT29 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80f8, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT30 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x80fc, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPCOUNT31 */
	{ F_USB20HDC_C_HSEL_ADDR_OFFSET + 0x8100, U2H_R | U2H_W },
					/* F_USB20HDC_REG_EPBUF */
	{ F_USB20HDC_D_HSEL_ADDR_OFFSET + 0x0000, U2H_R | U2H_W },
					/* F_USB20HDC_REG_DMA1 */
	{ F_USB20HDC_D_HSEL_ADDR_OFFSET + 0x8000, U2H_R | U2H_W }
					/* F_USB20HDC_REG_DMA2 */
};

static inline void ctrl_default_reg_cache_bits(
	void __iomem *base_addr, u32 register_id, u32 *register_cache)
{
	return;
}

static inline void ctrl_ints_reg_cache_bits(void __iomem *base_addr,
	u32 register_id, u32 *register_cache)
{
/* Global Interrupt Status register bit feild position */
#define F_USB20HDC_REG_INTS_BIT_PHY_ERR_INT	3 /* phy_err_int */
#define F_USB20HDC_REG_INTS_BIT_CMD_INT		4 /* cmd_int */
#define F_USB20HDC_REG_INTS_BIT_DMA1_INT	8 /* dma1_int */
#define F_USB20HDC_REG_INTS_BIT_DMA2_INT	9 /* dma2_int */

	*register_cache |=
		(((u32)1 << F_USB20HDC_REG_INTS_BIT_PHY_ERR_INT) |
		((u32)1 << F_USB20HDC_REG_INTS_BIT_CMD_INT) |
		((u32)1 << F_USB20HDC_REG_INTS_BIT_DMA1_INT) |
		((u32)1 << F_USB20HDC_REG_INTS_BIT_DMA2_INT));
	return;
}

static inline void ctrl_otgstsc_reg_cache_bits(
	void __iomem *base_address, u32 register_id,
	u32 *register_cache)
{
/* OTG Status Change register bit feild position */
#define F_USB20HDC_REG_OTGSTS_BIT_OTG_TMROUT_C		0 /* otg_tmrout_c */
#define F_USB20HDC_REG_OTGSTS_BIT_ID_C			6 /* id_c */
#define F_USB20HDC_REG_OTGSTS_BIT_VBUS_VLD_C		10 /* vbus_vld_c */

	*register_cache |=
		(((u32)1 << F_USB20HDC_REG_OTGSTS_BIT_OTG_TMROUT_C) |
		((u32)1 << F_USB20HDC_REG_OTGSTS_BIT_ID_C) |
		((u32)1 << F_USB20HDC_REG_OTGSTS_BIT_VBUS_VLD_C));
	return;
}

static inline void ctrl_devs_reg_cache_bits(void __iomem *base_addr,
	u32 register_id, u32 *register_cache)
{
/* Device Status register bit feild position */
#define F_USB20HDC_REG_DEVS_BIT_SUSPENDE_INT	24	/* suspende_int */
#define F_USB20HDC_REG_DEVS_BIT_SUSPENDB_INT	25	/* suspendb_int */
#define F_USB20HDC_REG_DEVS_BIT_SOF_INT		26	/* sof_int */
#define F_USB20HDC_REG_DEVS_BIT_SETUP_INT	27	/* setup_int */
#define F_USB20HDC_REG_DEVS_BIT_USBRSTE_INT	28	/* usbrste_int */
#define F_USB20HDC_REG_DEVS_BIT_USBRSTB_INT	29	/* usbrstb_int */
#define F_USB20HDC_REG_DEVS_BIT_STATUS_OK_INT	30	/* status_ok_int */
#define F_USB20HDC_REG_DEVS_BIT_STATUS_NG_INT	31	/* status_ng_int */

	*register_cache |=
		(((u32)1 << F_USB20HDC_REG_DEVS_BIT_SUSPENDE_INT) |
		((u32)1 << F_USB20HDC_REG_DEVS_BIT_SUSPENDB_INT) |
		((u32)1 << F_USB20HDC_REG_DEVS_BIT_SOF_INT) |
		((u32)1 << F_USB20HDC_REG_DEVS_BIT_SETUP_INT) |
		((u32)1 << F_USB20HDC_REG_DEVS_BIT_USBRSTE_INT) |
		((u32)1 << F_USB20HDC_REG_DEVS_BIT_USBRSTB_INT) |
		((u32)1 << F_USB20HDC_REG_DEVS_BIT_STATUS_OK_INT) |
		((u32)1 << F_USB20HDC_REG_DEVS_BIT_STATUS_NG_INT));
	return;
}


static inline void ctrl_portsc_reg_cache_bits(
	void __iomem *base_address, u32 register_id,
	u32 *register_cache)
{
/* Port Status / Control register bit feild position */
#define F_USB20HDC_REG_PORTSC_BIT_RESET_REQ	25 /* port_reset_req */
#define F_USB20HDC_REG_PORTSC_BIT_SUSPEND_REQ	30 /* port_suspend_req */
#define F_USB20HDC_REG_PORTSC_BIT_RESUME_REQ	31 /* port_resume_req */

	*register_cache &=
		~(((u32)1 << F_USB20HDC_REG_PORTSC_BIT_RESET_REQ) |
		((u32)1 << F_USB20HDC_REG_PORTSC_BIT_SUSPEND_REQ) |
		((u32)1 << F_USB20HDC_REG_PORTSC_BIT_RESUME_REQ));
	return;
}

static inline void ctrl_portstsc_reg_cache_bits(
	void __iomem *base_address, u32 register_id,
	u32 *register_cache)
{
/* Port Status Change register bit feild position */
#define F_USB20HDC_REG_PORTSTSC_BIT_CONNECTION_C	0 /* port_conection */
#define F_USB20HDC_REG_PORTSTSC_BIT_ENABLE_C		1 /* port_enable_c */
#define F_USB20HDC_REG_PORTSTSC_BIT_SUSPEND_C		2 /* port_suspend_c */
#define F_USB20HDC_REG_PORTSTSC_BIT_OV_CURR_C		3 /* port_ov_curr_c */
#define F_USB20HDC_REG_PORTSTSC_BIT_RESET_C		4 /* port_reset_c */

	*register_cache |= (((u32)1 <<
				F_USB20HDC_REG_PORTSTSC_BIT_CONNECTION_C) |
		((u32)1 << F_USB20HDC_REG_PORTSTSC_BIT_ENABLE_C) |
		((u32)1 << F_USB20HDC_REG_PORTSTSC_BIT_SUSPEND_C) |
		((u32)1 << F_USB20HDC_REG_PORTSTSC_BIT_OV_CURR_C) |
		((u32)1 << F_USB20HDC_REG_PORTSTSC_BIT_RESET_C));
	return;
}

static inline void ctrl_hosteventsrc_reg_cache_bits(
	void __iomem *base_address, u32 register_id,
	u32 *register_cache)
{
/* Host Event Source register bit feild position */
#define F_USB20HDC_REG_HOSTEVENTSRC_BIT_SOFSTART	0 /* sofstart */
#define F_USB20HDC_REG_HOSTEVENTSRC_BIT_FRAMEOV		1 /* frameov */
#define F_USB20HDC_REG_HOSTEVENTSRC_BIT_TRANS_DONE	8 /* trans_done[0:7] */

	*register_cache |= (((u32)1 <<
				F_USB20HDC_REG_HOSTEVENTSRC_BIT_SOFSTART) |
		((u32)1 << F_USB20HDC_REG_HOSTEVENTSRC_BIT_FRAMEOV) |
		((u32)0xff <<
				F_USB20HDC_REG_HOSTEVENTSRC_BIT_TRANS_DONE));
	return;
}

static inline u32 get_epctrl_register_bits(
	void __iomem *base_addr,
	u32 register_id, u8 start_bit,
	u8 bit_length)
{
	u32 counter;
	u32 register_cache[3] = {0, 0, 0};
	u32 mask = (u32)-1 >> (32 - bit_length);

	if (!(f_usb20hdc_register[register_id].access & U2H_R))
		return 0;

	for (counter = 0xffff; counter; counter--) {
		register_cache[0] = __raw_readl(base_addr +
					f_usb20hdc_register[
					register_id].address_offset) >>
					start_bit & mask;
		register_cache[1] = __raw_readl(base_addr +
					f_usb20hdc_register[
					register_id].address_offset) >>
					start_bit & mask;
		register_cache[2] = __raw_readl(base_addr +
					f_usb20hdc_register[
					register_id].address_offset) >>
					start_bit & mask;
		if ((register_cache[0] == register_cache[1]) &&
			(register_cache[1] == register_cache[2]))
			break;
	}

	return register_cache[2];
}

static inline void set_register_bits(void __iomem *base_addr,
	u32 register_id, u8 start_bit,
	u8 bit_length, u32 value)
{
	u32 register_cache = 0;
	u32 mask = (u32)-1 >> (32 - bit_length);

	value &= mask;

	if (f_usb20hdc_register[register_id].access & U2H_R)
		register_cache = __raw_readl(base_addr +
			f_usb20hdc_register[register_id].address_offset);

	ctrl_reg_cache_bits(base_addr, register_id, &register_cache);

	register_cache &= ~(mask << start_bit);
	register_cache |= (value << start_bit);

	if (f_usb20hdc_register[register_id].access & U2H_W)
		__raw_writel(register_cache, base_addr +
			f_usb20hdc_register[register_id].address_offset);

	return;
}

static inline u32 get_register_bits(void __iomem *base_addr,
	u32 register_id, u8 start_bit,
	u8 bit_length)
{
	u32 register_cache = 0;
	u32 mask = (u32)-1 >> (32 - bit_length);

	if (f_usb20hdc_register[register_id].access & U2H_R)
		register_cache = __raw_readl(base_addr +
			f_usb20hdc_register[register_id].address_offset);

	return register_cache >> start_bit & mask;
}

/* host mode reg function */
static inline void set_port_power_ctl_req(void __iomem *base_addr,
					u8 power_control_on)
{
	set_register_bits(base_addr, F_USB20HDC_REG_PORTSC, 21, 1,
				power_control_on);
}

static inline void set_port_power_req(void __iomem *base_addr,
						u8 power_on)
{
	set_register_bits(base_addr, F_USB20HDC_REG_PORTSC, 22, 1, power_on);
}

static inline u8 get_port_power_rhs(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
						F_USB20HDC_REG_PORTSC, 2, 1);
}

/*common registers in both host mode and device mode*/
static inline void set_byte_order(void __iomem *base_addr, u8 big_endian)
{
	set_register_bits(base_addr, F_USB20HDC_REG_CONF, 0, 1, big_endian);
}

static inline void set_burst_wait(void __iomem *base_addr, u8 waits)
{
	set_register_bits(base_addr, F_USB20HDC_REG_CONF, 1, 1, waits);
}

static inline void set_soft_reset(void __iomem *base_addr)
{
	u32 counter;
	set_register_bits(base_addr, F_USB20HDC_REG_CONF, 2, 1, 1);
	for (counter = 0xffff; ((counter) && (get_register_bits(base_addr,
				F_USB20HDC_REG_CONF, 2, 1))); counter--)
				;
}

static inline void set_host_en(void __iomem *base_addr, u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_MODE, 0, 1, enable);
}

static inline u8 get_host_en(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_MODE, 0, 1);
}

static inline void set_dev_en(void __iomem *base_addr, u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_MODE, 1, 1, enable);
}

static inline u8 get_dev_en(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_MODE, 1, 1);
}

static inline void set_host_inten(void __iomem *base_addr, u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_INTEN, 0, 1, enable);
}

static inline u8 get_host_inten(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_INTEN, 0, 1);
}

static inline void set_dev_int_mode(void __iomem *base_addr, u8 mode)
{
	set_register_bits(base_addr, F_USB20HDC_REG_MODE, 2, 1, mode);
}

static inline void set_dev_addr_load_mode(void __iomem *base_addr, u8 mode)
{
	set_register_bits(base_addr, F_USB20HDC_REG_MODE, 3, 1, mode);
}

static inline void set_dev_inten(void __iomem *base_addr, u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_INTEN, 1, 1, enable);
}

static inline u8 get_dev_inten(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
					F_USB20HDC_REG_INTEN, 1, 1);
}

static inline void set_otg_inten(void __iomem *base_addr, u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_INTEN, 2, 1, enable);
}

static inline u8 get_otg_inten(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_INTEN, 2, 1);
}

static inline void set_phy_err_inten(void __iomem *base_addr, u8 en)
{
	set_register_bits(base_addr, F_USB20HDC_REG_INTEN, 3, 1, en);
}

static inline u8 get_phy_err_inten(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_INTEN, 3, 1);
}

static inline void set_cmd_inten(void __iomem *base_addr, u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_INTEN, 4, 1, enable);
}

static inline u8 get_cmd_inten(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_INTEN, 4, 1);
}

static inline void set_dma_inten(void __iomem *base_addr, u8 dma_chan, u8 en)
{
	set_register_bits(base_addr, F_USB20HDC_REG_INTEN, 8 + dma_chan, 1, en);
}

/* USB DMA channel symbolic constant */
#define F_USB20HDC_DMA_CH1	0	/* USB DMA channel 1 */
#define F_USB20HDC_DMA_CH2	1	/* USB DMA channel 2 */

static inline u8 get_dma_inten(void __iomem *base_addr, u8 dma_ch)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_INTEN,
		8 + dma_ch, 1);
}

static inline void set_dev_ep_inten(void __iomem *base_addr, u8 ep_ch, u8 en)
{
	set_register_bits(base_addr, F_USB20HDC_REG_INTEN, 16 + ep_ch, 1, en);
}

/* endpoint channel symbolic constant */
#define F_USB20HDC_EP0	0	/* endpoint 0 */
#define F_USB20HDC_EP1	1	/* endpoint 1 */
#define F_USB20HDC_EP2	2	/* endpoint 2 */
#define F_USB20HDC_EP3	3	/* endpoint 3 */
#define F_USB20HDC_EP4	4	/* endpoint 4 */
#define F_USB20HDC_EP5	5	/* endpoint 5 */
#define F_USB20HDC_EP6	6	/* endpoint 6 */
#define F_USB20HDC_EP7	7	/* endpoint 7 */
#define F_USB20HDC_EP8	8	/* endpoint 8 */
#define F_USB20HDC_EP9	9	/* endpoint 9 */
#define F_USB20HDC_EP10	10	/* endpoint 10 */
#define F_USB20HDC_EP11	11	/* endpoint 11 */
#define F_USB20HDC_EP12	12	/* endpoint 12 */
#define F_USB20HDC_EP13	13	/* endpoint 13 */
#define F_USB20HDC_EP14	14	/* endpoint 14 */
#define F_USB20HDC_EP15	15	/* endpoint 15 */

static inline u8 get_dev_ep_inten(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_INTEN,
		16 + ep_ch, 1);
}

static inline u8 get_host_int(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_INTS,
						0, 1);
}

static inline u8 get_dev_int(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_INTS,
						1, 1);
}

static inline u8 get_otg_int(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_INTS, 2, 1);
}

static inline u8 get_phy_err_int(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_INTS, 3, 1);
}

static inline void clear_phy_err_int(void __iomem *base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_INTS, 3, 1, 0);
}

static inline u8 get_cmd_int(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_INTS,
						4, 1);
}

static inline void clear_cmd_int(void __iomem *base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_INTS, 4, 1, 0);
}

static inline u8 get_dma_int(void __iomem *base_addr, u8 dma_ch)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_INTS,
						8 + dma_ch, 1);
}

static inline void clear_dma_int(void __iomem *base_addr, u8 dma_ch)
{
	set_register_bits(base_addr, F_USB20HDC_REG_INTS, 8 + dma_ch, 1, 0);
}

static inline u8 get_dev_ep_int(void __iomem *base_addr, u8 ep_ch)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_INTS,
						16 + ep_ch, 1);
}


/*endpoint command register*/
static void f_cmd0_spin(void __iomem *base_addr, u8 ep_chan)
{
	u32 counter;
	for (counter = 0xffff; ((counter) && (get_register_bits(base_addr,
			F_USB20HDC_REG_EPCMD0 + ep_chan, 31, 1))); counter--)
		;
}

static inline void set_start(void __iomem *base_addr, u8 ep_ch)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch, 0, 1, 1);
	f_cmd0_spin(base_addr, ep_ch);
}


static inline void set_init(void __iomem *base_addr, u8 ep_ch)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch, 2, 1, 1);
	f_cmd0_spin(base_addr, ep_ch);
}

static inline void set_bufwr(void __iomem *base_addr, u8 ep_ch)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch, 3, 1, 1);
	f_cmd0_spin(base_addr, ep_ch);
}

static inline void set_bufrd(void __iomem *base_addr, u8 ep_ch)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch, 4, 1, 1);
	f_cmd0_spin(base_addr, ep_ch);
}

static inline void set_toggle_clear(void __iomem *base_addr, u8 ep_ch)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch, 8, 1, 1);
	f_cmd0_spin(base_addr, ep_ch);
}

static inline void set_toggle_set(void __iomem *base_addr, u8 ep_ch)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch, 7, 1, 1);
	f_cmd0_spin(base_addr, ep_ch);
}

static inline void set_et(void __iomem *base_addr, u8 ep_ch, u8 type)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch,
								23, 2, type);
	f_cmd0_spin(base_addr, ep_ch);
}


static inline void set_bnum(void __iomem *base_addr, u8 ep_ch, u8 buffers)
{
	f_cmd0_spin(base_addr, ep_ch);
	set_register_bits(base_addr, F_USB20HDC_REG_EPCMD0 + ep_ch,
		26, 2, buffers - 1);
	f_cmd0_spin(base_addr, ep_ch);
}


/*OTG register in F_USB20HDC Macro*/
static inline void set_dm_pull_down(void __iomem *base_addr, u8 pull_down)
{
	set_register_bits(base_addr, F_USB20HDC_REG_OTGC, 7, 1, pull_down);
}

static inline void set_dp_pull_down(void __iomem *base_addr, u8 pull_down)
{
	set_register_bits(base_addr, F_USB20HDC_REG_OTGC, 8, 1, pull_down);
}

static inline void set_id_pull_up(void __iomem *base_addr, u8 pull_up)
{
	set_register_bits(base_addr, F_USB20HDC_REG_OTGC, 9, 1, pull_up);
}

static inline u8 get_otg_tmrout(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,	F_USB20HDC_REG_OTGSTS, 0, 1);
}

static inline u8 get_id(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,	F_USB20HDC_REG_OTGSTS, 6, 1);
}

static inline u8 get_tmrout_c(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_OTGSTSC, 0, 1);
}

static inline u8 get_id_c(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_OTGSTSC, 6, 1);
}

static inline u8 get_vbus_vld(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_OTGSTS, 10, 1);
}

static inline void clear_tmrout_c(void __iomem *base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_OTGSTSC, 0, 1, 0);
}

static inline void clear_id_c(void __iomem *base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_OTGSTSC, 6, 1, 0);
}

static inline u8 get_vbus_vld_c(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_OTGSTSC, 10, 1);
}

static inline void clear_vbus_vld_c(void __iomem *base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_OTGSTSC, 10, 1, 0);
}

static inline void set_vbus_vld_fen(void __iomem *base_addr, u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_OTGSTSFALL, 10, 1, enable);
}

static inline void set_vbus_vld_ren(void __iomem *base_addr, u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_OTGSTSRISE, 10, 1, enable);
}

static inline u8 get_vbus_vld_ren(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
					F_USB20HDC_REG_OTGSTSRISE, 10, 1);
}

static inline u8 get_vbus_vld_fen(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
					F_USB20HDC_REG_OTGSTSFALL, 10, 1);
}

static inline void set_otg_tmrout_ren(void __iomem *base_addr, u8 en)
{
	set_register_bits(base_addr, F_USB20HDC_REG_OTGSTSRISE, 0, 1, en);
}

static inline u8 get_otg_tmrout_ren(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,	F_USB20HDC_REG_OTGSTSRISE,
		0, 1);
}

static inline void set_id_fen(void __iomem *base_addr, u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_OTGSTSFALL, 6, 1, enable);
}

static inline void set_id_ren(void __iomem *base_addr, u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_OTGSTSRISE, 6, 1, enable);
}

static inline u8 get_id_fen(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr,
					F_USB20HDC_REG_OTGSTSFALL, 6, 1);
}

static inline u8 get_id_ren(void __iomem *base_addr)
{
	return (u8)get_register_bits(base_addr, F_USB20HDC_REG_OTGSTSRISE,
		6, 1);
}

static inline void set_start_tmr(void __iomem *base_addr, u8 start)
{
	set_register_bits(base_addr, F_USB20HDC_REG_OTGTC, 0, 1, start);
}

static inline void set_tmr_init_val(void __iomem *base_addr, u16 val)
{
	set_register_bits(base_addr, F_USB20HDC_REG_OTGT, 0, 16, val);
}


/*DMA register in F_USB20HDC Macro*/
static inline void set_dma_st(void __iomem *base_addr, u8 dma_ch, u8 start)
{
	set_register_bits(base_addr, dmac_register[dma_ch], 0, 1, start);
}

static inline void set_dma_mode(void __iomem *base_addr, u8 dma_ch, u8 mode)
{
	set_register_bits(base_addr, dmac_register[dma_ch], 2, 1, mode);
}

/* DMA mode symbolic constant */
#define DMA_MODE_DEMAND	0	/* demand transter */
#define DMA_MODE_BLOCK	1	/* block transter */

static inline void set_dma_sendnull(void __iomem *base_addr, u8 dma_ch, u8 send)
{
	set_register_bits(base_addr, dmac_register[dma_ch], 3, 1, send);
}

static inline u8 get_dma_busy(void __iomem *base_addr, u8 dma_ch)
{
	return (u8)get_register_bits(base_addr, dmac_register[dma_ch], 1, 1);
}

static inline u8 get_dma_ep(void __iomem *base_addr, u8 dma_ch)
{
	return (u8)get_register_bits(base_addr, dmac_register[dma_ch], 8, 4);
}

static inline u8 get_dma_np(void __iomem *base_addr, u8 dma_ch)
{
	return (u8)get_register_bits(base_addr, dmas_register[dma_ch], 0, 1);
}

static inline void set_dma_int_empty(void __iomem *base_addr, u8 dma_ch,
	u8 empty)
{
	set_register_bits(base_addr, dmac_register[dma_ch], 4, 1, empty);
}

static inline void set_dma_spr(void __iomem *base_addr, u8 dma_ch, u8 spr)
{
	set_register_bits(base_addr, dmac_register[dma_ch], 5, 1, spr);
}

static inline void set_dma_ep(void __iomem *base_addr, u8 dma_ch, u8 ep_ch)
{
	set_register_bits(base_addr, dmac_register[dma_ch], 8, 4, ep_ch);
}

static inline void set_dma_blksize(void __iomem *base_addr, u8 dma_ch, u16 size)
{
	set_register_bits(base_addr, dmac_register[dma_ch], 16, 11, size);
}

static inline u8 get_dma_sp(void __iomem *base_addr, u8 dma_ch)
{
	return (u8)get_register_bits(base_addr, dmas_register[dma_ch], 1, 1);
}

static inline void set_dma_tci(void __iomem *base_addr, u8 dma_ch, u32 bytes)
{
	set_register_bits(base_addr, dmatci_register[dma_ch], 0, 32, bytes);
}

static inline u32 get_dma_tci(void __iomem *base_addr, u8 dma_ch)
{
	return (u32)get_register_bits(base_addr,
					dmatci_register[dma_ch], 0, 32);
}

static inline u32 get_dma_tc(void __iomem *base_addr, u8 dma_ch)
{
	return (u32)get_register_bits(base_addr, dmatc_register[dma_ch], 0, 32);
}

/*TEST register in F_USB20HDC Macro*/
static inline void set_testp(void __iomem *base_addr, u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_TESTC, 0, 1, enable);
}

static inline void set_testj(void __iomem *base_addr, u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_TESTC, 1, 1, enable);
}

static inline void set_testk(void __iomem *base_addr, u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_TESTC, 2, 1, enable);
}

static inline void set_testse0nack(void __iomem *base_addr, u8 enable)
{
	set_register_bits(base_addr, F_USB20HDC_REG_TESTC, 3, 1, enable);
}

/*endpoint configuration and count register in F_USB20HDC Macro*/
static inline void clear_epconf(void __iomem *base_addr, u8 ep_ch)
{
	set_register_bits(base_addr, F_USB20HDC_REG_EPCONF0 + ep_ch,
				0, 32, 0x00000000);
}

static inline void set_base(void __iomem *base_addr, u8 ep_ch,
	u16 buffer_base_addr)
{
	set_register_bits(base_addr, F_USB20HDC_REG_EPCONF0 + ep_ch, 0, 13,
				(buffer_base_addr & 0x7fff) >> 2);
}

static inline void set_size(void __iomem *base_addr, u8 ep_ch,
	u16 size)
{
	set_register_bits(base_addr, F_USB20HDC_REG_EPCONF0 + ep_ch,
		13, 11, size);
}

static inline void set_countidx(void __iomem *base_addr, u8 ep_ch, u8 index)
{
	set_register_bits(base_addr, F_USB20HDC_REG_EPCONF0 + ep_ch,
		24, 5, index);
}

static inline void clear_epcount(void __iomem *base_addr, u8 index)
{
	set_register_bits(base_addr, F_USB20HDC_REG_EPCOUNT0 + index,
				0, 32, 0x00000000);
}

static inline void set_appcnt(void __iomem *base_addr, u8 index,
	u16 bytes)
{
	set_register_bits(base_addr, F_USB20HDC_REG_EPCOUNT0 +
		index, 0, 11, bytes);
}

static inline u16 get_appcnt(void __iomem *base_addr, u8 index)
{
	return (u16) get_register_bits(base_addr,
				F_USB20HDC_REG_EPCOUNT0 + index, 0, 11);
}

static inline u16 get_phycnt(void __iomem *base_addr, u8 index)
{
	return (u16)get_register_bits(base_addr,
				F_USB20HDC_REG_EPCOUNT0 + index, 16, 11);
}

static inline u8 get_rdatapid(void __iomem *base_addr,
	u8 index)
{
	return (u16)get_register_bits(base_addr,
				F_USB20HDC_REG_EPCOUNT0 + index, 30, 2);
}
/* receive data PID symbolic constant */
#define RDATAPID_DATA0	0	/* DATA0 */
#define RDATAPID_DATA1	1	/* DATA1 */
#define RDATAPID_DATA2	2	/* DATA2 */
#define RDATAPID_MDATA	3	/* MDATA */

static inline u32 get_epbuf_address_offset(void)
{
	return f_usb20hdc_register[F_USB20HDC_REG_EPBUF].address_offset -
					F_USB20HDC_C_HSEL_ADDR_OFFSET;
}

static inline unsigned long get_epbuf_dma_address(phys_addr_t base_addr,
	u8 dma_ch)
{
	return (unsigned long)(base_addr + f_usb20hdc_register[
			F_USB20HDC_REG_DMA1 + dma_ch].address_offset);
}

static inline void write_epbuf(void __iomem *base_addr, u32 offset,
	u32 *data, u32 bytes)
{
	memcpy_toio(base_addr + F_USB20HDC_C_HSEL_ADDR_OFFSET +
		offset, data, bytes);
}

static inline void read_epbuf(void __iomem *base_addr, u32 offset, u32 *data,
	u32 bytes)
{
	memcpy_fromio(data, base_addr + F_USB20HDC_C_HSEL_ADDR_OFFSET +
		offset, bytes);
}

static inline void __iomem *remap_iomem_region(unsigned long physical_base_addr,
	unsigned long size)
{
	/* convert physical address to virtula address */
	return (void __iomem *)ioremap(physical_base_addr, size);
}

static inline void unmap_iomem_region(void __iomem *virtual_base_addr)
{
	iounmap(virtual_base_addr);
}

static inline u32 get_irq_flag(void)
{
	return IRQF_SHARED;
}

static inline u32 get_dma_irq_flag(u8 dma_chan)
{
	return IRQF_SHARED;
}


static inline void enable_bus_power_supply(void __iomem *base_addr,
	u8 enable)
{
	u32 counter;

	set_port_power_req(base_addr, enable ? 1 : 0);
	if (enable)
		for (counter = 0xffff; ((counter) &&
				(!get_port_power_rhs(base_addr))); counter--)
			;
	else
		for (counter = 0xffff; ((counter) &&
				(get_port_power_rhs(base_addr))); counter--)
			;
}

int f_usb20hdc_dma_attach(struct f_usb20hdc_dma_data *dma_data,
	int ch_num, int size);
int f_usb20hdc_dma_detach(struct f_usb20hdc_dma_data *dma_data,
	int ch_num, int size);


#endif /* __MACH_F_USB20HDC_H */
