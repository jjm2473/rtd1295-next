/*
 * linux/drivers/usb/gadget/f_usb30_udc.h - F_USB30 USB3.0 Function
 * Controller Driver
 *
 * based on F_USB20LP USB2.0 Function Controller Driver
 *
 * Copyright (C) 2003 Robert Schwebel <r.schwebel@pengutronix.de>, Pengutronix
 * Copyright (C) 2003 David Brownell
 * Copyright (C) 2006 Lineo Solutions, Inc.
 * Copyright (C) 2006 - 2007 Lineo Solutions, Inc.
 * Copyright (C) FUJITSU ELECTRONICS INC. 2011. All rights reserved.
 * Copyright (C) 2011 - 2012 FUJITSU SEMICONDUCTOR LIMITED
 */

#ifndef __F_USB30_UDC_H
#define __F_USB30_UDC_H

#define F_USB30_DRIVER_VERSION		"1.0.0"

enum f_usb30_ctrl_stage {
	F_USB30_STAGE_SETUP = 0,		/* SETUP stage                */
	F_USB30_STAGE_IN_DATA,			/* IN data stage              */
	F_USB30_STAGE_OUT_DATA,			/* OUT data stage             */
	F_USB30_STAGE_IN_STATUS,		/* IN status stage            */
	F_USB30_STAGE_OUT_STATUS,		/* OUT status stage           */
	F_USB30_STAGE_SPECIAL,			/*
						 * special stage for
						 * SET_CONFIGURATION
						 * or SET_INTERFACE request
						 */
	F_USB30_STAGE_MAX,			/* max value                  */
};

enum f_usb30_ss_link_state {
	F_USB30_LINK_STATE_NOTATTACHED = 0,	/* vbus is invalid state      */
	F_USB30_LINK_STATE_ATTACHED,		/* vbus is valid state        */
	F_USB30_LINK_STATE_TRAINING,		/* ss link training state     */
	F_USB30_LINK_STATE_SSENABLED,		/* SuperSpeed enumerate state */
	F_USB30_LINK_STATE_HSENABLED,		/* HS/FS enumerated state     */
	F_USB30_LINK_STATE_DISABLED,		/* link training fail state   */
};

#if defined(CONFIG_ARCH_MB8AC0300)
#define F_USB30_VBUS_ON_IRQ		62	/* vbus on detect extint num  */
#define F_USB30_VBUS_OFF_IRQ		64	/* vbus off detect extint num */
#define F_USB30_VBUS_ACTIVE_LEVEL	1	/* vbus detect active level   */
#else
/* for other soc */
#endif

#define F_USB30_DMA_ADDR_INVALID	(~(dma_addr_t)0)

#if defined(CONFIG_USB_GADGET_F_USB30_USED_DMA_TRANSFER)
#define F_USB30_DMAC_REG_OFFSET		0x1100
#endif

/* interface count */
#define F_USB30_MAX_INTF		1

/* alternate count */
#define F_USB30_MAX_ALT			1

/* interface alternate channel count */
#define F_USB30_ALT_INTF0		1
#define F_USB30_ALT_INTF1		0
#define F_USB30_ALT_INTF2		0
#define F_USB30_ALT_INTF3		0
#define F_USB30_ALT_INTF4		0
#define F_USB30_ALT_INTF5		0
#define F_USB30_ALT_INTF6		0
#define F_USB30_ALT_INTF7		0
#define F_USB30_ALT_INTF8		0
#define F_USB30_ALT_INTF9		0
#define F_USB30_ALT_INTF10		0
#define F_USB30_ALT_INTF11		0
#define F_USB30_ALT_INTF12		0
#define F_USB30_ALT_INTF13		0
#define F_USB30_ALT_INTF14		0
#define F_USB30_ALT_INTF15		0

/* endpoint count */
#define F_USB30_MAX_EP			6

/* endpoint FIFO size table array */
static const unsigned short ep_fifo_size[F_USB30_MAX_EP][USB_SPEED_SUPER + 1]
					[F_USB30_MAX_ALT] = {
	/*unkown,low,    full ,   high, wireless,super */
	{{},	{},	{64,},	{64,},	{},	{512,},},	/* endpoint 0 */
	{{},	{},	{64,},	{512,},	{},	{1024,},},	/* endpoint 1 */
	{{},	{},	{64,},	{512,},	{},	{1024,},},	/* endpoint 2 */
	{{},	{},	{8,},	{64,},	{},	{64,},},	/* endpoint 3 */
	{{},	{},	{64,},	{512,},	{},	{1024,},},	/* endpoint 4 */
	{{},	{},	{64,},	{512,},	{},	{1024,},},	/* endpoint 5 */
};

/*
 * endpoint transfer type
 * [notice]:The following constant definition is used.
 *	unused		= 0
 *	control transfer	= USB_ENDPOINT_XFER_CONTROL
 *	isochronous transfer	= USB_ENDPOINT_XFER_ISOC
 *	interrupt transfer	= USB_ENDPOINT_XFER_INT
 *	bulk transfer		= USB_ENDPOINT_XFER_BULK
 */
#define F_USB30_TRANS_TYPE_EP0 USB_ENDPOINT_XFER_CONTROL
#define F_USB30_TRANS_TYPE_EP1 USB_ENDPOINT_XFER_BULK
#define F_USB30_TRANS_TYPE_EP2 USB_ENDPOINT_XFER_BULK
#define F_USB30_TRANS_TYPE_EP3 USB_ENDPOINT_XFER_INT
#define F_USB30_TRANS_TYPE_EP4 USB_ENDPOINT_XFER_BULK
#define F_USB30_TRANS_TYPE_EP5 USB_ENDPOINT_XFER_BULK
#define F_USB30_TRANS_TYPE_EP6	0
#define F_USB30_TRANS_TYPE_EP7	0
#define F_USB30_TRANS_TYPE_EP8	0
#define F_USB30_TRANS_TYPE_EP9	0
#define F_USB30_TRANS_TYPE_EP10	0
#define F_USB30_TRANS_TYPE_EP11	0
#define F_USB30_TRANS_TYPE_EP12	0
#define F_USB30_TRANS_TYPE_EP13	0
#define F_USB30_TRANS_TYPE_EP14	0
#define F_USB30_TRANS_TYPE_EP15	0

/*
 * endpoint transfer direction
 * [notice]:The following constant definition is used.
 *	unused	= 0
 *	IN transfer	= USB_DIR_IN
 *	OUT transfer	= USB_DIR_OUT
 */
#define F_USB30_TRANS_DIR_EP0 USB_DIR_OUT
#define F_USB30_TRANS_DIR_EP1 USB_DIR_IN
#define F_USB30_TRANS_DIR_EP2 USB_DIR_OUT
#define F_USB30_TRANS_DIR_EP3 USB_DIR_IN
#define F_USB30_TRANS_DIR_EP4 USB_DIR_OUT
#define F_USB30_TRANS_DIR_EP5 USB_DIR_IN
#define F_USB30_TRANS_DIR_EP6	0
#define F_USB30_TRANS_DIR_EP7	0
#define F_USB30_TRANS_DIR_EP8	0
#define F_USB30_TRANS_DIR_EP9	0
#define F_USB30_TRANS_DIR_EP10	0
#define F_USB30_TRANS_DIR_EP11	0
#define F_USB30_TRANS_DIR_EP12	0
#define F_USB30_TRANS_DIR_EP13	0
#define F_USB30_TRANS_DIR_EP14	0
#define F_USB30_TRANS_DIR_EP15	0

/*
 * endpoint name string
 * [notice]:The following constant definition is used.
 *	unused	= ""
 *	used		= "ep(number)(in or out)-(type)"
 */
#define F_USB30_NAME_STRING_EP0		"ep0"
#define F_USB30_NAME_STRING_EP1		"ep1in-bulk"
#define F_USB30_NAME_STRING_EP2		"ep2out-bulk"
#define F_USB30_NAME_STRING_EP3		"ep3in-int"
#define F_USB30_NAME_STRING_EP4		"ep4out-bulk"
#define F_USB30_NAME_STRING_EP5		"ep5in-bulk"
#define F_USB30_NAME_STRING_EP6		""
#define F_USB30_NAME_STRING_EP7		""
#define F_USB30_NAME_STRING_EP8		""
#define F_USB30_NAME_STRING_EP9		""
#define F_USB30_NAME_STRING_EP10	""
#define F_USB30_NAME_STRING_EP11	""
#define F_USB30_NAME_STRING_EP12	""
#define F_USB30_NAME_STRING_EP13	""
#define F_USB30_NAME_STRING_EP14	""
#define F_USB30_NAME_STRING_EP15	""

/*
 * endpoint interface channel
 * [notice]:unusing it is referred to as '-1'
 */
#define F_USB30_INTF_CHANNEL_EP0	-1
#define F_USB30_INTF_CHANNEL_EP1	0
#define F_USB30_INTF_CHANNEL_EP2	0
#define F_USB30_INTF_CHANNEL_EP3	0
#define F_USB30_INTF_CHANNEL_EP4	0
#define F_USB30_INTF_CHANNEL_EP5	0
#define F_USB30_INTF_CHANNEL_EP6	-1
#define F_USB30_INTF_CHANNEL_EP7	-1
#define F_USB30_INTF_CHANNEL_EP8	-1
#define F_USB30_INTF_CHANNEL_EP9	-1
#define F_USB30_INTF_CHANNEL_EP10	-1
#define F_USB30_INTF_CHANNEL_EP11	-1
#define F_USB30_INTF_CHANNEL_EP12	-1
#define F_USB30_INTF_CHANNEL_EP13	-1
#define F_USB30_INTF_CHANNEL_EP14	-1
#define F_USB30_INTF_CHANNEL_EP15	-1

#if defined(CONFIG_USB_GADGET_F_USB30_USED_DMA_TRANSFER)
/* F_USB30 DMA controller channel count */
#define F_USB30_IN_DMAC			0
#define F_USB30_OUT_DMAC		1
#define F_USB30_MAX_DMAC		2

/* F_USB30 DMA controller transfer max byte */
#if defined(CONFIG_USB_GADGET_F_USB30_DMA_USE_BOUNCE_BUF)
#define F_USB30_DMAC_TRANS_MAX_BYTES	16384
#else
#define F_USB30_DMAC_TRANS_MAX_BYTES	4294967295U
#endif

/*
 * DMA controller channel for endpoint
 * [notice]:unusing it is referred to as '-1'
 */
#define F_USB30_DMAC_CHANNEL_EP0	-1
#define F_USB30_DMAC_CHANNEL_EP1	0
#define F_USB30_DMAC_CHANNEL_EP2	1
#define F_USB30_DMAC_CHANNEL_EP3	-1
#define F_USB30_DMAC_CHANNEL_EP4	-1
#define F_USB30_DMAC_CHANNEL_EP5	-1
#define F_USB30_DMAC_CHANNEL_EP6	-1
#define F_USB30_DMAC_CHANNEL_EP7	-1
#define F_USB30_DMAC_CHANNEL_EP8	-1
#define F_USB30_DMAC_CHANNEL_EP9	-1
#define F_USB30_DMAC_CHANNEL_EP10	-1
#define F_USB30_DMAC_CHANNEL_EP11	-1
#define F_USB30_DMAC_CHANNEL_EP12	-1
#define F_USB30_DMAC_CHANNEL_EP13	-1
#define F_USB30_DMAC_CHANNEL_EP14	-1
#define F_USB30_DMAC_CHANNEL_EP15	-1
#endif

/* F_USB30 controller register ID Enumeration constant */
#define F_USB30_REG_HSCPAC		0   /* HS/FS CPU Access Control       */
#define F_USB30_REG_HSDVC		1   /* HS/FS Device Control           */
#define F_USB30_REG_HSDVS		2   /* HS/FS Device Status            */
#define F_USB30_REG_HSEPIC		3   /* HS/FS EP Interrupt Control     */
#define F_USB30_REG_HSEPIS		4   /* HS/FS EP Interrupt Status      */
#define F_USB30_REG_HSEPDC		5   /* HS/FS EP DMA Control           */
#define F_USB30_REG_HSEPDS		6   /* HS/FS EP DMA Status            */
#define F_USB30_REG_HSTSTAMP		7   /* HS/FS Time Stamp               */
#define F_USB30_REG_HSEPTCSEL		8   /* HS/FS EP Byte Count Select     */
#define F_USB30_REG_HSEPTC1		9   /* HS/FS EP1 Total Byte Count     */
#define F_USB30_REG_HSEPTC2		10  /* HS/FS EP2 Total Byte Count     */
#define F_USB30_REG_HSEPRS0		11  /* HS/FS EP0 Rx Size              */
#define F_USB30_REG_HSEPRS1		12  /* HS/FS EP1 Rx Size              */
#define F_USB30_REG_HSEPRS2		13  /* HS/FS EP2 Rx Size              */
#define F_USB30_REG_HSEPRS3		14  /* HS/FS EP3 Rx Size              */
#define F_USB30_REG_HSEPRS4		15  /* HS/FS EP4 Rx Size              */
#define F_USB30_REG_HSEPRS5		16  /* HS/FS EP5 Rx Size              */
#define F_USB30_REG_HSCUSTOMC		17  /* HS/FS Custom Control           */
#define F_USB30_REG_HSCALIB		18  /* HS/FS Calibration              */
#define F_USB30_REG_HSEPLPBK		19  /* HS/FS Loop Back Selector       */
#define F_USB30_REG_HSINTFALTNUM	20  /* HS/FS Interface Alt Number     */
#define F_USB30_REG_HSEPC0		21  /* HS/FS EP0 Control              */
#define F_USB30_REG_HSEPS0		22  /* HS/FS EP0 Status               */
#define F_USB30_REG_HSEPC1		23  /* HS/FS EP1 Control              */
#define F_USB30_REG_HSEPS1		24  /* HS/FS EP1 Status               */
#define F_USB30_REG_HSEPC2		25  /* HS/FS EP2 Control              */
#define F_USB30_REG_HSEPS2		26  /* HS/FS EP2 Status               */
#define F_USB30_REG_HSEPC3		27  /* HS/FS EP3 Control              */
#define F_USB30_REG_HSEPS3		28  /* HS/FS EP3 Status               */
#define F_USB30_REG_HSEPC4		29  /* HS/FS EP4 Control              */
#define F_USB30_REG_HSEPS4		30  /* HS/FS EP4 Status               */
#define F_USB30_REG_HSEPC5		31  /* HS/FS EP5 Control              */
#define F_USB30_REG_HSEPS5		32  /* HS/FS EP5 Status               */
#define F_USB30_REG_HSALTC		33  /* HS/FS Alternate Control        */
#define F_USB30_REG_HSALTS		34  /* HS/FS Alternate Status         */
#define F_USB30_REG_HSEPIB0		35  /* HS/FS EP0 In Buffer            */
#define F_USB30_REG_HSEPIB1		36  /* HS/FS EP1 In Buffer            */
#define F_USB30_REG_HSEPIB3		37  /* HS/FS EP3 In Buffer            */
#define F_USB30_REG_HSEPIB5		38  /* HS/FS EP5 In Buffer            */
#define F_USB30_REG_HSEPOB0		39  /* HS/FS EP0 Out Buffer           */
#define F_USB30_REG_HSEPOB2		40  /* HS/FS EP2 Out Buffer           */
#define F_USB30_REG_HSEPOB4		41  /* HS/FS EP4 Out Buffer           */
#define F_USB30_REG_MAKEUP_DATA		42  /* HS/FS MAKE-UP DATA AREA        */
#define F_USB30_REG_MAKEUP0		43  /* HS/FS EP0 MAKE-UP AREA         */
#define F_USB30_REG_MAKEUP1		44  /* HS/FS EP1 MAKE-UP AREA         */
#define F_USB30_REG_MAKEUP2		45  /* HS/FS EP2 MAKE-UP AREA         */
#define F_USB30_REG_MAKEUP3		46  /* HS/FS EP3 MAKE-UP AREA         */
#define F_USB30_REG_MAKEUP4		47  /* HS/FS EP4 MAKE-UP AREA         */
#define F_USB30_REG_MAKEUP5		48  /* HS/FS EP5 MAKE-UP AREA         */
#define F_USB30_REG_CLKC		49  /* Clock Control                  */
#define F_USB30_REG_CLKCE		50  /* Clock Control Enable           */
#define F_USB30_REG_SSCPAC		51  /* SS CPU Access Control          */
#define F_USB30_REG_SSDVC		52  /* SS Device Control              */
#define F_USB30_REG_SSDVS		53  /* SS Device Status               */
#define F_USB30_REG_SSIRQC		54  /* SS Interrupt Request Control   */
#define F_USB30_REG_SSIRQS		55  /* SS Interrupt Request Status    */
#define F_USB30_REG_SSEPDC		56  /* SS EP DMA Control              */
#define F_USB30_REG_SSEPDS		57  /* SS EP DMA Status               */
#define F_USB30_REG_SSEPTCSEL		58  /* SS EP Byte Count Status Select */
#define F_USB30_REG_SSEPTC1		59  /* SS EP1 Total Byte Count        */
#define F_USB30_REG_SSEPTC2		60  /* SS EP2 Total Byte Count        */
#define F_USB30_REG_SSEPSZ0I		61  /* SS EP0 In Size                 */
#define F_USB30_REG_SSEPSZ0O		62  /* SS EP0 Out Size                */
#define F_USB30_REG_SSEPSZ1I		63  /* SS EP1 In Size                 */
#define F_USB30_REG_SSEPSZ2O		64  /* SS EP2 Out Size                */
#define F_USB30_REG_SSEPSZ3I		65  /* SS EP3 In Size                 */
#define F_USB30_REG_SSEPSZ4O		66  /* SS EP4 Out Size                */
#define F_USB30_REG_SSEPSZ5I		67  /* SS EP5 In Size                 */
#define F_USB30_REG_SSCUSC		68  /* SS Custom Control              */
#define F_USB30_REG_SSEPLPBK		69  /* SS Loopback Selector           */
#define F_USB30_REG_SSINTFC		70  /* SS Interface Control           */
#define F_USB30_REG_SSINTFS		71  /* SS Interface Status            */
#define F_USB30_REG_SSEPC0		72  /* SS EP0 Control                 */
#define F_USB30_REG_SSEPS0		73  /* SS EP0 Status                  */
#define F_USB30_REG_SSEPC1		74  /* SS EP1 Control                 */
#define F_USB30_REG_SSEPS1		75  /* SS EP1 Status                  */
#define F_USB30_REG_SSEPC2		76  /* SS EP2 Control                 */
#define F_USB30_REG_SSEPS2		77  /* SS EP2 Status                  */
#define F_USB30_REG_SSEPC3		78  /* SS EP3 Control                 */
#define F_USB30_REG_SSEPS3		79  /* SS EP3 Status                  */
#define F_USB30_REG_SSEPC4		80  /* SS EP4 Control                 */
#define F_USB30_REG_SSEPS4		81  /* SS EP4 Status                  */
#define F_USB30_REG_SSEPC5		82  /* SS EP5 Control                 */
#define F_USB30_REG_SSEPS5		83  /* SS EP5 Status                  */
#define F_USB30_REG_SSEPIB0		84  /* SS EP0 In Buffer               */
#define F_USB30_REG_SSEPIB1		85  /* SS EP1 In Buffer               */
#define F_USB30_REG_SSEPIB3		86  /* SS EP3 In Buffer               */
#define F_USB30_REG_SSEPIB5		87  /* SS EP5 In Buffer               */
#define F_USB30_REG_SSEPOB0		88  /* SS EP0 Out Buffer              */
#define F_USB30_REG_SSEPOB2		89  /* SS EP2 Out Buffer              */
#define F_USB30_REG_SSEPOB4		90  /* SS EP4 Out Buffer              */
#define F_USB30_REG_SSCFGEP1		91  /* SS EP1 Makeup                  */
#define F_USB30_REG_SSCFGEP2		92  /* SS EP2 Makeup                  */
#define F_USB30_REG_SSCFGEP3		93  /* SS EP3 Makeup                  */
#define F_USB30_REG_SSCFGEP4		94  /* SS EP4 Makeup                  */
#define F_USB30_REG_SSCFGEP5		95  /* SS EP5 Makeup                  */
#define F_USB30_REG_SSEXCFGDV		96  /* SS Extended Device Makeup      */
#define F_USB30_REG_SSEXCFGEP1		97  /* SS Extended EP1 Makeup         */
#define F_USB30_REG_SSEXCFGEP2		98  /* SS Extended EP2 Makeup         */
#define F_USB30_REG_SSEXCFGEP4		99  /* SS Extended EP4 Makeup         */
#define F_USB30_REG_SSEXCFGEP5		100 /* SS Extended EP5 Makeup         */
#define F_USB30_REG_SSBSC1		101 /* SS EP1 Bulk Stream Control     */
#define F_USB30_REG_SSBSC2		102 /* SS EP2 Bulk Stream Control     */
#define F_USB30_REG_SSBSC5		103 /* SS EP5 Bulk Stream Control     */
#define F_USB30_REG_SSEPSTC1		104 /* SS EP1 Stream Total Byte Count */
#define F_USB30_REG_SSEPSTC2		105 /* SS EP2 Stream Total Byte Count */
#define F_USB30_REG_SSERCC		106 /* SS Error Counter Control       */
#define F_USB30_REG_SSDlECNT1		107 /* SS Data Link Error Count 1     */
#define F_USB30_REG_SSDlECNT2		108 /* SS Data Link Error Count 2     */
#define F_USB30_REG_SSDlECNT3		109 /* SS Data Link Error Count 3     */
#define F_USB30_REG_SSDlECNT4		110 /* SS Data Link Error Count 4     */
#define F_USB30_REG_SSPlECNT		111 /* SS PHY Error Count             */
#define F_USB30_REG_SSLNC		112 /* SS Link Control                */
#define F_USB30_REG_SSTRC1		113 /* SS Transaction Control1        */
#define F_USB30_REG_SSTRC2		114 /* SS Transaction Control2        */
#define F_USB30_REG_SSTRS1		115 /* SS Transaction Status1         */
#define F_USB30_REG_SSTRS2		116 /* SS Transaction Status2         */
#define F_USB30_REG_SSDlC1		117 /* SS Data Link Control1          */
#define F_USB30_REG_SSDlC2		118 /* SS Data Link Control2          */
#define F_USB30_REG_SSDlC3		119 /* SS Data Link Control3          */
#define F_USB30_REG_SSDlC4		120 /* SS Data Link Control4          */
#define F_USB30_REG_SSPMC1		121 /* SS PM Control1                 */
#define F_USB30_REG_SSPMC2		122 /* SS PM Control2                 */
#define F_USB30_REG_SSPMC3		123 /* SS PM Control3                 */
#define F_USB30_REG_SSPMC4		124 /* SS PM Control4                 */
#define F_USB30_REG_SSPMC5		125 /* SS PM Control5                 */
#define F_USB30_REG_SSVDTC1		126 /* SS Vendor Device Test Control1 */
#define F_USB30_REG_SSVDTC2		127 /* SS Vendor Device Test Control2 */
#define F_USB30_REG_SSVDTS1		128 /* SS Vendor Device Test Status1  */
#define F_USB30_REG_SSVDTS2		129 /* SS Vendor Device Test Status2  */
#define F_USB30_REG_SSPlC1		130 /* SS PHY Control1                */
#define F_USB30_REG_SSPlC2		131 /* SS PHY Control2                */
#define F_USB30_REG_SSPlC3		132 /* SS PHY Control3                */
#define F_USB30_REG_SSPlC4		133 /* SS PHY Control4                */
#define F_USB30_REG_SSPlC5		134 /* SS PHY Control5                */
#define F_USB30_REG_SSPlC6		135 /* SS PHY Control6                */
#define F_USB30_REG_SSPlC7		136 /* SS PHY Control7                */
#define F_USB30_REG_SSPlC8		137 /* SS PHY Control8                */
#define F_USB30_REG_SSPlC9		138 /* SS PHY Control9                */
#define F_USB30_REG_SSPlC10		139 /* SS PHY Control10               */
#define F_USB30_REG_SSPlC11		140 /* SS PHY Control11               */
#define F_USB30_REG_SSPlC12		141 /* SS PHY Control12               */
#define F_USB30_REG_SSPlS		142 /* SS PHY Status                  */
#define F_USB30_REG_MAX			143 /* Max Value                      */

/* HS/FS END POINT x CONTROL register ID table array */
static const unsigned long hsepc_register[] = {
	F_USB30_REG_HSEPC0,		/* HS/FS EP0 Control */
	F_USB30_REG_HSEPC1,		/* HS/FS EP1 Control */
	F_USB30_REG_HSEPC2,		/* HS/FS EP2 Control */
	F_USB30_REG_HSEPC3,		/* HS/FS EP3 Control */
	F_USB30_REG_HSEPC4,		/* HS/FS EP4 Control */
	F_USB30_REG_HSEPC5,		/* HS/FS EP5 Control */
};

/* HS/FS END POINT x STATUS register ID table array */
static const unsigned long hseps_register[] = {
	F_USB30_REG_HSEPS0,		/* HS/FS EP0 Status  */
	F_USB30_REG_HSEPS1,		/* HS/FS EP1 Status  */
	F_USB30_REG_HSEPS2,		/* HS/FS EP2 Status  */
	F_USB30_REG_HSEPS3,		/* HS/FS EP3 Status  */
	F_USB30_REG_HSEPS4,		/* HS/FS EP4 Status  */
	F_USB30_REG_HSEPS5,		/* HS/FS EP5 Status  */
};

/* SS END POINT x CONTROL register ID table array */
static const unsigned long ssepc_register[] = {
	F_USB30_REG_SSEPC0,		/* SS EP0 Control */
	F_USB30_REG_SSEPC1,		/* SS EP1 Control */
	F_USB30_REG_SSEPC2,		/* SS EP2 Control */
	F_USB30_REG_SSEPC3,		/* SS EP3 Control */
	F_USB30_REG_SSEPC4,		/* SS EP4 Control */
	F_USB30_REG_SSEPC5,		/* SS EP5 Control */
};

/* SS END POINT x STATUS register ID table array */
static const unsigned long sseps_register[] = {
	F_USB30_REG_SSEPS0,		/* SS EP0 Status  */
	F_USB30_REG_SSEPS1,		/* SS EP1 Status  */
	F_USB30_REG_SSEPS2,		/* SS EP2 Status  */
	F_USB30_REG_SSEPS3,		/* SS EP3 Status  */
	F_USB30_REG_SSEPS4,		/* SS EP4 Status  */
	F_USB30_REG_SSEPS5,		/* SS EP5 Status  */
};

/* SS END POINT x STATUS register ID table array */
static const unsigned long ssepsz_register[] = {
	F_USB30_REG_SSEPSZ1I,		/* SS EP1 In Size  */
	F_USB30_REG_SSEPSZ2O,		/* SS EP2 Out Size */
	F_USB30_REG_SSEPSZ3I,		/* SS EP3 In Size  */
	F_USB30_REG_SSEPSZ4O,		/* SS EP4 Out Size */
	F_USB30_REG_SSEPSZ5I,		/* SS EP5 In Size  */
};

#define U30_R (1 << 0)
#define U30_W (1 << 1)

/* F_USB30 controller register structures array */
static const struct {
	unsigned short address_offset;	/* register address offset */
	unsigned char access;	/* register access flags  */
} f_usb30_register[F_USB30_REG_MAX] = {
	{ 0x0000, U30_R | U30_W },	/* F_USB30_REG_HSCPAC      */
	{ 0x0004, U30_R | U30_W },	/* F_USB30_REG_HSDVC       */
	{ 0x0008, U30_R | U30_W },	/* F_USB30_REG_HSDVS       */
	{ 0x000c, U30_R | U30_W },	/* F_USB30_REG_HSEPIC      */
	{ 0x0010, U30_R },	/* F_USB30_REG_HSEPIS      */
	{ 0x0014, U30_R | U30_W },	/* F_USB30_REG_HSEPDC      */
	{ 0x0018, U30_R },	/* F_USB30_REG_HSEPDS      */
	{ 0x001c, U30_R },	/* F_USB30_REG_HSTSTAMP    */
	{ 0x0020, U30_R | U30_W },	/* F_USB30_REG_HSEPTCSEL   */
	{ 0x0024, U30_R | U30_W },	/* F_USB30_REG_HSEPTC1     */
	{ 0x0028, U30_R | U30_W },	/* F_USB30_REG_HSEPTC2     */
	{ 0x0070, U30_R | U30_W },	/* F_USB30_REG_HSEPRS0     */
	{ 0x0078, U30_R | U30_W },	/* F_USB30_REG_HSEPRS1     */
	{ 0x0080, U30_R | U30_W },	/* F_USB30_REG_HSEPRS2     */
	{ 0x0088, U30_R | U30_W },	/* F_USB30_REG_HSEPRS3     */
	{ 0x0090, U30_R | U30_W },	/* F_USB30_REG_HSEPRS4     */
	{ 0x0098, U30_R | U30_W },	/* F_USB30_REG_HSEPRS5     */
	{ 0x00f0, U30_R | U30_W },	/* F_USB30_REG_HSCUSTOMC   */
	{ 0x00f4, U30_R | U30_W },	/* F_USB30_REG_HSCALIB     */
	{ 0x00f8, U30_R | U30_W },	/* F_USB30_REG_HSEPLPBK    */
	{ 0x00fc, U30_R | U30_W },	/* F_USB30_REG_HSINTFALTNUM*/
	{ 0x0100, U30_R | U30_W },	/* F_USB30_REG_HSEPC0      */
	{ 0x0104, U30_R | U30_W },	/* F_USB30_REG_HSEPS0      */
	{ 0x0108, U30_R | U30_W },	/* F_USB30_REG_HSEPC1      */
	{ 0x010c, U30_R | U30_W },	/* F_USB30_REG_HSEPS1      */
	{ 0x0110, U30_R | U30_W },	/* F_USB30_REG_HSEPC2      */
	{ 0x0114, U30_R | U30_W },	/* F_USB30_REG_HSEPS2      */
	{ 0x0118, U30_R | U30_W },	/* F_USB30_REG_HSEPC3      */
	{ 0x011c, U30_R | U30_W },	/* F_USB30_REG_HSEPS3      */
	{ 0x0120, U30_R | U30_W },	/* F_USB30_REG_HSEPC4      */
	{ 0x0124, U30_R | U30_W },	/* F_USB30_REG_HSEPS4      */
	{ 0x0128, U30_R | U30_W },	/* F_USB30_REG_HSEPC5      */
	{ 0x012c, U30_R | U30_W },	/* F_USB30_REG_HSEPS5      */
	{ 0x0178, U30_R | U30_W },	/* F_USB30_REG_HSALTC      */
	{ 0x017c, U30_R | U30_W },	/* F_USB30_REG_HSALTS      */
	{ 0x0180, U30_R | U30_W },	/* F_USB30_REG_HSEPIB0     */
	{ 0x0184, U30_R | U30_W },	/* F_USB30_REG_HSEPIB1     */
	{ 0x018c, U30_R | U30_W },	/* F_USB30_REG_HSEPIB3     */
	{ 0x0194, U30_R | U30_W },	/* F_USB30_REG_HSEPIB5     */
	{ 0x01c0, U30_R },	/* F_USB30_REG_HSEPOB0     */
	{ 0x01c8, U30_R },	/* F_USB30_REG_HSEPOB2     */
	{ 0x01d0, U30_R },	/* F_USB30_REG_HSEPOB4     */
	{ 0x0200, U30_R | U30_W },	/* F_USB30_REG_MAKEUP_DATA */
	{ 0x0204, U30_R | U30_W },	/* F_USB30_REG_MAKEUP_EP0  */
	/* F_USB30_REG_MAKEUP_EP1  */
	{ 0x0208 + (F_USB30_MAX_ALT * 0), U30_R | U30_W },
	/* F_USB30_REG_MAKEUP_EP2  */
	{ 0x0218 + (F_USB30_MAX_ALT * 4), U30_R | U30_W },
	{ 0x0228, U30_R | U30_W },	/* F_USB30_REG_MAKEUP_EP3  */
	{ 0x023c, U30_R | U30_W },	/* F_USB30_REG_MAKEUP_EP4  */
	{ 0x024c, U30_R | U30_W },	/* F_USB30_REG_MAKEUP_EP5  */
	{ 0x0400, U30_R | U30_W },	/* F_USB30_REG_CLKC        */
	{ 0x0404, U30_R | U30_W },	/* F_USB30_REG_CLKCE       */
	{ 0x0800, U30_R | U30_W },	/* F_USB30_REG_SSCPAC      */
	{ 0x0804, U30_R | U30_W },	/* F_USB30_REG_SSDVC       */
	{ 0x0808, U30_R | U30_W },	/* F_USB30_REG_SSDVS       */
	{ 0x080c, U30_R | U30_W },	/* F_USB30_REG_SSIRQC      */
	{ 0x0810, U30_R },	/* F_USB30_REG_SSIRQS      */
	{ 0x0814, U30_R | U30_W },	/* F_USB30_REG_SSEPDC      */
	{ 0x0818, U30_R },	/* F_USB30_REG_SSEPDS      */
	{ 0x0820, U30_R | U30_W },	/* F_USB30_REG_SSEPTCSEL   */
	{ 0x0824, U30_R | U30_W },	/* F_USB30_REG_SSEPTC1     */
	{ 0x0828, U30_R | U30_W },	/* F_USB30_REG_SSEPTC2     */
	{ 0x0870, U30_R },	/* F_USB30_REG_SSEPSZ0I    */
	{ 0x0874, U30_R },	/* F_USB30_REG_SSEPSZ0O    */
	{ 0x0878, U30_R },	/* F_USB30_REG_SSEPSZ1I    */
	{ 0x0884, U30_R },	/* F_USB30_REG_SSEPSZ2O    */
	{ 0x0888, U30_R },	/* F_USB30_REG_SSEPSZ3I    */
	{ 0x0894, U30_R },	/* F_USB30_REG_SSEPSZ4O    */
	{ 0x0898, U30_R },	/* F_USB30_REG_SSEPSZ5I    */
	{ 0x08f0, U30_R | U30_W },	/* F_USB30_REG_SSCUSC      */
	{ 0x08f4, U30_R | U30_W },	/* F_USB30_REG_SSEPLPBK    */
	{ 0x08f8, U30_R | U30_W },	/* F_USB30_REG_SSINTFC     */
	{ 0x08fc, U30_R | U30_W },	/* F_USB30_REG_SSINTFS     */
	{ 0x0900, U30_R | U30_W },	/* F_USB30_REG_SSEPC0      */
	{ 0x0904, U30_R | U30_W },	/* F_USB30_REG_SSEPS0      */
	{ 0x0908, U30_R | U30_W },	/* F_USB30_REG_SSEPC1      */
	{ 0x090c, U30_R | U30_W },	/* F_USB30_REG_SSEPS1      */
	{ 0x0910, U30_R | U30_W },	/* F_USB30_REG_SSEPC2      */
	{ 0x0914, U30_R | U30_W },	/* F_USB30_REG_SSEPS2      */
	{ 0x0918, U30_R | U30_W },	/* F_USB30_REG_SSEPC3      */
	{ 0x091c, U30_R | U30_W },	/* F_USB30_REG_SSEPS3      */
	{ 0x0920, U30_R | U30_W },	/* F_USB30_REG_SSEPC4      */
	{ 0x0924, U30_R | U30_W },	/* F_USB30_REG_SSEPS4      */
	{ 0x0928, U30_R | U30_W },	/* F_USB30_REG_SSEPC5      */
	{ 0x092c, U30_R | U30_W },	/* F_USB30_REG_SSEPS5      */
	{ 0x0980, U30_R | U30_W },	/* F_USB30_REG_SSEPIB0     */
	{ 0x0984, U30_R | U30_W },	/* F_USB30_REG_SSEPIB1     */
	{ 0x098c, U30_R | U30_W },	/* F_USB30_REG_SSEPIB3     */
	{ 0x0994, U30_R | U30_W },	/* F_USB30_REG_SSEPIB5     */
	{ 0x09c0, U30_R },	/* F_USB30_REG_SSEPOB0     */
	{ 0x09c8, U30_R },	/* F_USB30_REG_SSEPOB2     */
	{ 0x09d0, U30_R },	/* F_USB30_REG_SSEPOB4     */
	{ 0x0a08, U30_R | U30_W },	/* F_USB30_REG_SSCFGEP1    */
	{ 0x0a0c, U30_R | U30_W },	/* F_USB30_REG_SSCFGEP2    */
	{ 0x0a10, U30_R | U30_W },	/* F_USB30_REG_SSCFGEP3    */
	{ 0x0a14, U30_R | U30_W },	/* F_USB30_REG_SSCFGEP4    */
	{ 0x0a18, U30_R | U30_W },	/* F_USB30_REG_SSCFGEP5    */
	{ 0x0b00, U30_R | U30_W },	/* F_USB30_REG_SSEXCFGDV   */
	{ 0x0b08, U30_R | U30_W },	/* F_USB30_REG_SSEXCFGEP1  */
	{ 0x0b0c, U30_R | U30_W },	/* F_USB30_REG_SSEXCFGEP2  */
	{ 0x0b14, U30_R | U30_W },	/* F_USB30_REG_SSEXCFGEP4  */
	{ 0x0b18, U30_R | U30_W },	/* F_USB30_REG_SSEXCFGEP5  */
	{ 0x0b54, U30_R | U30_W },	/* F_USB30_REG_SSBSC1      */
	{ 0x0b58, U30_R | U30_W },	/* F_USB30_REG_SSBSC2      */
	{ 0x0b64, U30_R | U30_W },	/* F_USB30_REG_SSBSC5      */
	{ 0x0b94, U30_R | U30_W },	/* F_USB30_REG_SSEPSTC1    */
	{ 0x0b98, U30_R | U30_W },	/* F_USB30_REG_SSEPSTC2    */
	{ 0x0c00, U30_R | U30_W },	/* F_USB30_REG_SSERCC      */
	{ 0x0c04, U30_R },	/* F_USB30_REG_SSDlECNT1   */
	{ 0x0c08, U30_R },	/* F_USB30_REG_SSDlECNT2   */
	{ 0x0c0c, U30_R },	/* F_USB30_REG_SSDlECNT3   */
	{ 0x0c10, U30_R },	/* F_USB30_REG_SSDlECNT4   */
	{ 0x0c14, U30_R },	/* F_USB30_REG_SSPlECNT    */
	{ 0x0c20, U30_R | U30_W },	/* F_USB30_REG_SSLNC       */
	{ 0x0c40, U30_R | U30_W },	/* F_USB30_REG_SSTRC1      */
	{ 0x0c44, U30_R | U30_W },	/* F_USB30_REG_SSTRC2      */
	{ 0x0c48, U30_R },	/* F_USB30_REG_SSTRS1      */
	{ 0x0c4c, U30_R },	/* F_USB30_REG_SSTRS2      */
	{ 0x0c50, U30_R | U30_W },	/* F_USB30_REG_SSDlC1      */
	{ 0x0c54, U30_R | U30_W },	/* F_USB30_REG_SSDlC2      */
	{ 0x0c58, U30_R | U30_W },	/* F_USB30_REG_SSDlC3      */
	{ 0x0c5c, U30_R | U30_W },	/* F_USB30_REG_SSDlC4      */
	{ 0x0c60, U30_R | U30_W },	/* F_USB30_REG_SSPMC1      */
	{ 0x0c64, U30_R | U30_W },	/* F_USB30_REG_SSPMC2      */
	{ 0x0c68, U30_R | U30_W },	/* F_USB30_REG_SSPMC3      */
	{ 0x0c6c, U30_R | U30_W },	/* F_USB30_REG_SSPMC4      */
	{ 0x0c70, U30_R | U30_W },	/* F_USB30_REG_SSPMC5      */
	{ 0x0c80, U30_R | U30_W },	/* F_USB30_REG_SSVDTC1     */
	{ 0x0c84, U30_R | U30_W },	/* F_USB30_REG_SSVDTC2     */
	{ 0x0c88, U30_R },	/* F_USB30_REG_SSVDTS1     */
	{ 0x0c8c, U30_R },	/* F_USB30_REG_SSVDTS2     */
	{ 0x0c90, U30_R | U30_W },	/* F_USB30_REG_SSPlC1      */
	{ 0x0c94, U30_R | U30_W },	/* F_USB30_REG_SSPlC2      */
	{ 0x0c98, U30_R | U30_W },	/* F_USB30_REG_SSPlC3      */
	{ 0x0c9c, U30_R | U30_W },	/* F_USB30_REG_SSPlC4      */
	{ 0x0ca0, U30_R | U30_W },	/* F_USB30_REG_SSPlC5      */
	{ 0x0ca4, U30_R | U30_W },	/* F_USB30_REG_SSPlC6      */
	{ 0x0ca8, U30_R | U30_W },	/* F_USB30_REG_SSPlC7      */
	{ 0x0cac, U30_R | U30_W },	/* F_USB30_REG_SSPlC8      */
	{ 0x0cb0, U30_R | U30_W },	/* F_USB30_REG_SSPlC9      */
	{ 0x0cb4, U30_R | U30_W },	/* F_USB30_REG_SSPlC10     */
	{ 0x0cb8, U30_R | U30_W },	/* F_USB30_REG_SSPlC11     */
	{ 0x0cbc, U30_R | U30_W },	/* F_USB30_REG_SSPlC12     */
	{ 0x0cd0, U30_R },	/* F_USB30_REG_SSPlS       */
};

static inline void control_f_usb30_default_register_cache_bits(
	unsigned long *register_cache)
{
	return;
}

static inline void control_f_usb30_hsdvs_register_cache_bits(
	unsigned long *register_cache)
{
	/* DEVICE STATUS register bit feild position */
#define F_USB30_REGISTER_HSDVS_BIT_INTSUSPENDE		24 /* IntSuspende   */
#define F_USB30_REGISTER_HSDVS_BIT_INTSUSPENDB		25 /* IntSuspendb   */
#define F_USB30_REGISTER_HSDVS_BIT_INTSOF		26 /* IntSof        */
#define F_USB30_REGISTER_HSDVS_BIT_INTSETUP		27 /* IntSetup      */
#define F_USB30_REGISTER_HSDVS_BIT_INTUSBRSTE		28 /* IntUsbRste    */
#define F_USB30_REGISTER_HSDVS_BIT_INTUSBRSTB		29 /* IntUsbRstb    */
#define F_USB30_REGISTER_HSDVS_BIT_INTSETCONF		30 /* IntSetConf    */
#define F_USB30_REGISTER_HSDVS_BIT_INTERRATICERR	31 /* IntErraticErr */

	*register_cache |=
	   (((unsigned long) 1 << F_USB30_REGISTER_HSDVS_BIT_INTSUSPENDE) |
	    ((unsigned long) 1 << F_USB30_REGISTER_HSDVS_BIT_INTSUSPENDB) |
	    ((unsigned long) 1 << F_USB30_REGISTER_HSDVS_BIT_INTSOF) |
	    ((unsigned long) 1 << F_USB30_REGISTER_HSDVS_BIT_INTSETUP) |
	    ((unsigned long) 1 << F_USB30_REGISTER_HSDVS_BIT_INTUSBRSTE) |
	    ((unsigned long) 1 << F_USB30_REGISTER_HSDVS_BIT_INTUSBRSTB) |
	    ((unsigned long) 1 << F_USB30_REGISTER_HSDVS_BIT_INTSETCONF) |
	    ((unsigned long) 1 << F_USB30_REGISTER_HSDVS_BIT_INTERRATICERR));
	return;
}

static inline void control_f_usb30_hseps0_register_cache_bits(
	unsigned long *register_cache)
{
	/* END POINT 0 STATUS register bit feild position */
#define F_USB30_REGISTER_HSEPS0_BIT_READY0I		3  /* Ready0i     */
#define F_USB30_REGISTER_HSEPS0_BIT_READY0O		4  /* Ready0o     */
#define F_USB30_REGISTER_HSEPS0_BIT_INTREADY0I		16 /* IntReady0i  */
#define F_USB30_REGISTER_HSEPS0_BIT_INTREADY0O		17 /* IntReady0o  */
#define F_USB30_REGISTER_HSEPS0_BIT_INTPING0		18 /* IntPing0    */
#define F_USB30_REGISTER_HSEPS0_BIT_INTSTALLED0		21 /* IntStalled0 */
#define F_USB30_REGISTER_HSEPS0_BIT_INTNACK0		22 /* IntNack0    */
#define F_USB30_REGISTER_HSEPS0_BIT_INTCLSTALL0		23 /* IntClStall0 */

	*register_cache &=
	      ~(((unsigned long) 1 << F_USB30_REGISTER_HSEPS0_BIT_READY0I) |
		((unsigned long) 1 << F_USB30_REGISTER_HSEPS0_BIT_READY0O));

	*register_cache |=
	   (((unsigned long) 1 << F_USB30_REGISTER_HSEPS0_BIT_INTREADY0I) |
	    ((unsigned long) 1 << F_USB30_REGISTER_HSEPS0_BIT_INTREADY0O) |
	    ((unsigned long) 1 << F_USB30_REGISTER_HSEPS0_BIT_INTPING0) |
	    ((unsigned long) 1 << F_USB30_REGISTER_HSEPS0_BIT_INTSTALLED0) |
	    ((unsigned long) 1 << F_USB30_REGISTER_HSEPS0_BIT_INTNACK0) |
	    ((unsigned long) 1 << F_USB30_REGISTER_HSEPS0_BIT_INTCLSTALL0));

	return;
}

static inline void control_f_usb30_hseps_register_cache_bits(
	unsigned long *register_cache)
{
	/* END POINT STATUS register bit feild position */
#define F_USB30_REGISTER_HSEPS_BIT_READYI		2	/* Readyi     */
#define F_USB30_REGISTER_HSEPS_BIT_READYO		3	/* Readyo     */
#define F_USB30_REGISTER_HSEPS_BIT_INTSPR		13	/* IntSPR     */
#define F_USB30_REGISTER_HSEPS_BIT_INTSPDD		14	/* IntSPDD    */
#define F_USB30_REGISTER_HSEPS_BIT_INTREADY		16	/* IntReady   */
#define F_USB30_REGISTER_HSEPS_BIT_INTPING		17	/* IntPing    */
#define F_USB30_REGISTER_HSEPS_BIT_INTACHG		18	/* IntAChg    */
#define F_USB30_REGISTER_HSEPS_BIT_INTDEND		19	/* IntDEnd    */
#define F_USB30_REGISTER_HSEPS_BIT_INTEMPTY		20	/* IntEmpty   */
#define F_USB30_REGISTER_HSEPS_BIT_INTSTALLED		21	/* IntStalled */
#define F_USB30_REGISTER_HSEPS_BIT_INTNACK		22	/* IntNack    */
#define F_USB30_REGISTER_HSEPS_BIT_INTCLSTALL		23	/* IntClStall */

	*register_cache &=
	     ~(((unsigned long) 1 << F_USB30_REGISTER_HSEPS_BIT_READYI) |
	      ((unsigned long) 1 << F_USB30_REGISTER_HSEPS_BIT_READYO));

	*register_cache |=
	     (((unsigned long) 1 << F_USB30_REGISTER_HSEPS_BIT_INTSPR) |
	      ((unsigned long) 1 << F_USB30_REGISTER_HSEPS_BIT_INTSPDD) |
	      ((unsigned long) 1 << F_USB30_REGISTER_HSEPS_BIT_INTREADY) |
	      ((unsigned long) 1 << F_USB30_REGISTER_HSEPS_BIT_INTPING) |
	      ((unsigned long) 1 << F_USB30_REGISTER_HSEPS_BIT_INTACHG) |
	      ((unsigned long) 1 << F_USB30_REGISTER_HSEPS_BIT_INTDEND) |
	      ((unsigned long) 1 << F_USB30_REGISTER_HSEPS_BIT_INTEMPTY) |
	      ((unsigned long) 1 << F_USB30_REGISTER_HSEPS_BIT_INTSTALLED) |
	      ((unsigned long) 1 << F_USB30_REGISTER_HSEPS_BIT_INTNACK) |
	      ((unsigned long) 1 << F_USB30_REGISTER_HSEPS_BIT_INTCLSTALL));

	return;
}

static inline void control_f_usb30_hsalts_register_cache_bits(
	unsigned long *register_cache)
{
	/* ALTERNATE STATUS register bit feild position */
#define F_USB30_REGISTER_HSALTS_BIT_INTACHGIF0	0      /* intAChgIf0 */
#define F_USB30_REGISTER_HSALTS_BIT_INTACHGIF1	1      /* intAChgIf1 */

	*register_cache |=
	   (((unsigned long) 1 << F_USB30_REGISTER_HSALTS_BIT_INTACHGIF0) |
	    ((unsigned long) 1 << F_USB30_REGISTER_HSALTS_BIT_INTACHGIF1));

	return;
}

static inline void control_f_usb30_ssdvs_register_cache_bits(
	unsigned long *register_cache)
{
	/* DEVICE STATUS register bit feild position */
#define F_USB30_REGISTER_SSDVS_BIT_INTSSWRSTE		12 /* IntSSWRste      */
#define F_USB30_REGISTER_SSDVS_BIT_INTSSWRSTB		13 /* IntSSWRstb      */
#define F_USB30_REGISTER_SSDVS_BIT_INTSSHRSTE		14 /* IntSSHRste      */
#define F_USB30_REGISTER_SSDVS_BIT_INTSSHRSTB		15 /* IntSSHRstb      */
#define F_USB30_REGISTER_SSDVS_BIT_INTVDTEST		20 /* IntVDTest       */
#define F_USB30_REGISTER_SSDVS_BIT_INTSSDISABLE		21 /* IntSSDisable    */
#define F_USB30_REGISTER_SSDVS_BIT_INTENTERPOLL		22 /* IntEnterPoll    */
#define F_USB30_REGISTER_SSDVS_BIT_INTPOLLTOU0		23 /* IntPolltoU0     */
#define F_USB30_REGISTER_SSDVS_BIT_INTSUSPENDE		24 /* IntSuspende     */
#define F_USB30_REGISTER_SSDVS_BIT_INTSUSPENDB		25 /* IntSuspendb     */
#define F_USB30_REGISTER_SSDVS_BIT_INTSETINTF		26 /* IntSetIntf      */
#define F_USB30_REGISTER_SSDVS_BIT_INTSETUP		27 /* IntSetup        */
#define F_USB30_REGISTER_SSDVS_BIT_INTU2INACTTO		29 /* IntU2InactTO    */
#define F_USB30_REGISTER_SSDVS_BIT_INTSETCONF		30 /* IntSetConf      */
#define F_USB30_REGISTER_SSDVS_BIT_INTFNCSUSP		31 /* IntFncSusp      */

	*register_cache |=
	    (((unsigned long) 1 << F_USB30_REGISTER_SSDVS_BIT_INTSSWRSTE) |
	     ((unsigned long) 1 << F_USB30_REGISTER_SSDVS_BIT_INTSSWRSTB) |
	     ((unsigned long) 1 << F_USB30_REGISTER_SSDVS_BIT_INTSSHRSTE) |
	     ((unsigned long) 1 << F_USB30_REGISTER_SSDVS_BIT_INTSSHRSTB) |
	     ((unsigned long) 1 << F_USB30_REGISTER_SSDVS_BIT_INTVDTEST) |
	     ((unsigned long) 1 << F_USB30_REGISTER_SSDVS_BIT_INTSSDISABLE) |
	     ((unsigned long) 1 << F_USB30_REGISTER_SSDVS_BIT_INTENTERPOLL) |
	     ((unsigned long) 1 << F_USB30_REGISTER_SSDVS_BIT_INTPOLLTOU0) |
	     ((unsigned long) 1 << F_USB30_REGISTER_SSDVS_BIT_INTSUSPENDE) |
	     ((unsigned long) 1 << F_USB30_REGISTER_SSDVS_BIT_INTSUSPENDB) |
	     ((unsigned long) 1 << F_USB30_REGISTER_SSDVS_BIT_INTSETINTF) |
	     ((unsigned long) 1 << F_USB30_REGISTER_SSDVS_BIT_INTSETUP) |
	     ((unsigned long) 1 << F_USB30_REGISTER_SSDVS_BIT_INTU2INACTTO) |
	     ((unsigned long) 1 << F_USB30_REGISTER_SSDVS_BIT_INTSETCONF) |
	     ((unsigned long) 1 << F_USB30_REGISTER_SSDVS_BIT_INTFNCSUSP));

	return;
}

static inline void control_f_usb30_sseps0_register_cache_bits(
	unsigned long *register_cache)
{
	/* END POINT 0 STATUS register bit feild position */
#define F_USB30_REGISTER_SSEPS0_BIT_READY0I		3  /* Ready0i     */
#define F_USB30_REGISTER_SSEPS0_BIT_READY0O		4  /* Ready0o     */
#define F_USB30_REGISTER_SSEPS0_BIT_INTREADY0I		16 /* IntReady0i  */
#define F_USB30_REGISTER_SSEPS0_BIT_INTREADY0O		17 /* IntReady0o  */
#define F_USB30_REGISTER_SSEPS0_BIT_INTPKTPND0		18 /* IntPktPnd0  */
#define F_USB30_REGISTER_SSEPS0_BIT_INTSTALLED0		21 /* IntStalled0 */
#define F_USB30_REGISTER_SSEPS0_BIT_INTNRDY0		22 /* IntNrdy0    */
#define F_USB30_REGISTER_SSEPS0_BIT_INTCLSTALL0		23 /* IntClStall0 */

	*register_cache &=
	   ~(((unsigned long) 1 << F_USB30_REGISTER_SSEPS0_BIT_READY0I) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS0_BIT_READY0O));

	*register_cache |=
	   (((unsigned long) 1 << F_USB30_REGISTER_SSEPS0_BIT_INTREADY0I) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS0_BIT_INTREADY0O) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS0_BIT_INTPKTPND0) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS0_BIT_INTSTALLED0) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS0_BIT_INTNRDY0) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS0_BIT_INTCLSTALL0));

	return;
}

static inline void control_f_usb30_sseps1_register_cache_bits(
	unsigned long *register_cache)
{
	/* END POINT STATUS register bit feild position */
#define F_USB30_REGISTER_SSEPS_BIT_READY1I		2  /* Ready1i       */
#define F_USB30_REGISTER_SSEPS_BIT_STREAMACTIVE1	7  /* StreamActive1 */
#define F_USB30_REGISTER_SSEPS_BIT_INTCLSTREAM1		15 /* IntClStream1  */
#define F_USB30_REGISTER_SSEPS_BIT_INTREADY1I		16 /* IntReady1i    */
#define F_USB30_REGISTER_SSEPS_BIT_INTPKTPND1		17 /* IntPktPnd1    */
#define F_USB30_REGISTER_SSEPS_BIT_INTSDEND1		18 /* IntSDEnd1     */
#define F_USB30_REGISTER_SSEPS_BIT_INTDEND1		19 /* IntDEnd1      */
#define F_USB30_REGISTER_SSEPS_BIT_INTEMPTY1		20 /* IntEmpty1     */
#define F_USB30_REGISTER_SSEPS_BIT_INTSTALLED1		21 /* IntStalled1   */
#define F_USB30_REGISTER_SSEPS_BIT_INTNRDY1		22 /* IntNrdy1      */
#define F_USB30_REGISTER_SSEPS_BIT_INTCLSTALL1		23 /* IntClStall1   */

	*register_cache &=
	   ~((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_READY1I);

	*register_cache |=
	   (((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_STREAMACTIVE1) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTCLSTREAM1) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTREADY1I) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTPKTPND1) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTSDEND1) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTDEND1) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTEMPTY1) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTSTALLED1) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTNRDY1) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTCLSTALL1));

	return;
}

static inline void control_f_usb30_sseps2_register_cache_bits(
	unsigned long *register_cache)
{
	/* END POINT STATUS register bit feild position */
#define F_USB30_REGISTER_SSEPS_BIT_READY2O		3  /* Ready2o       */
#define F_USB30_REGISTER_SSEPS_BIT_STREAMACTIVE2	7  /* StreamActive2 */
#define F_USB30_REGISTER_SSEPS_BIT_INTSPR2		13 /* IntSPR2       */
#define F_USB30_REGISTER_SSEPS_BIT_INTSPD2		14 /* IntSPD2       */
#define F_USB30_REGISTER_SSEPS_BIT_INTCLSTREAM2		15 /* IntClStream2  */
#define F_USB30_REGISTER_SSEPS_BIT_INTREADY2O		16 /* IntReady2o    */
#define F_USB30_REGISTER_SSEPS_BIT_INTPKTPND2		17 /* IntPktPnd2    */
#define F_USB30_REGISTER_SSEPS_BIT_INTSDEND2		18 /* IntSDEnd2     */
#define F_USB30_REGISTER_SSEPS_BIT_INTDEND2		19 /* IntDEnd2      */
#define F_USB30_REGISTER_SSEPS_BIT_INTEMPTY2		20 /* IntEmpty2     */
#define F_USB30_REGISTER_SSEPS_BIT_INTSTALLED2		21 /* IntStalled2   */
#define F_USB30_REGISTER_SSEPS_BIT_INTNRDY2		22 /* IntNrdy2      */
#define F_USB30_REGISTER_SSEPS_BIT_INTCLSTALL2		23 /* IntClStall2   */

	*register_cache &=
	   ~((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_READY2O);

	*register_cache |=
	   (((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_STREAMACTIVE2) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTSPR2) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTSPD2) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTCLSTREAM2) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTREADY2O) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTPKTPND2) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTSDEND2) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTDEND2) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTEMPTY2) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTSTALLED2) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTNRDY2) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTCLSTALL2));

	return;
}

static inline void control_f_usb30_sseps3_register_cache_bits(
	unsigned long *register_cache)
{
	/* END POINT STATUS register bit feild position */
#define F_USB30_REGISTER_SSEPS_BIT_READY3I		2  /* Ready3i        */
#define F_USB30_REGISTER_SSEPS_BIT_INTREADY3I		16 /* IntReady3i     */
#define F_USB30_REGISTER_SSEPS_BIT_INTPKTPND3		17 /* IntPktPnd3     */
#define F_USB30_REGISTER_SSEPS_BIT_INTSTALLED3		21 /* IntStalled3    */
#define F_USB30_REGISTER_SSEPS_BIT_INTNRDY3		22 /* IntNrdy3       */
#define F_USB30_REGISTER_SSEPS_BIT_INTCLSTALL3		23 /* IntClStall3    */

	*register_cache &=
	   ~((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_READY3I);

	*register_cache |=
	   (((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTREADY3I) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTPKTPND3) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTSTALLED3) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTNRDY3) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTCLSTALL3));

	return;
}

static inline void control_f_usb30_sseps4_register_cache_bits(
	unsigned long *register_cache)
{
	/* END POINT STATUS register bit feild position */
#define F_USB30_REGISTER_SSEPS_BIT_READY4O		3  /* Ready4o        */
#define F_USB30_REGISTER_SSEPS_BIT_INTREADY4O		16 /* IntReady4o     */
#define F_USB30_REGISTER_SSEPS_BIT_INTPKTPND4		17 /* IntPktPnd4     */
#define F_USB30_REGISTER_SSEPS_BIT_INTSTALLED4		21 /* IntStalled4    */
#define F_USB30_REGISTER_SSEPS_BIT_INTNRDY4		22 /* IntNrdy4       */
#define F_USB30_REGISTER_SSEPS_BIT_INTCLSTALL4		23 /* IntClStall4    */

	*register_cache &=
	   ~((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_READY4O);

	*register_cache |=
	   (((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTREADY4O) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTPKTPND4) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTSTALLED4) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTNRDY4) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTCLSTALL4));

	return;
}

static inline void control_f_usb30_sseps5_register_cache_bits(
	unsigned long *register_cache)
{
	/* END POINT STATUS register bit feild position */
#define F_USB30_REGISTER_SSEPS_BIT_READY5I		2  /* Ready5i       */
#define F_USB30_REGISTER_SSEPS_BIT_STREAMACTIVE5	7  /* StreamActive5 */
#define F_USB30_REGISTER_SSEPS_BIT_INTCLSTREAM5		15 /* IntClStream5  */
#define F_USB30_REGISTER_SSEPS_BIT_INTREADY5I		16 /* IntReady5i    */
#define F_USB30_REGISTER_SSEPS_BIT_INTPKTPND5		17 /* IntPktPnd5    */
#define F_USB30_REGISTER_SSEPS_BIT_INTSTALLED5		21 /* IntStalled5   */
#define F_USB30_REGISTER_SSEPS_BIT_INTNRDY5		22 /* IntNrdy5      */
#define F_USB30_REGISTER_SSEPS_BIT_INTCLSTALL5		23 /* IntClStall5   */

	*register_cache &=
	   ~((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_READY5I);

	*register_cache |=
	   (((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_STREAMACTIVE5) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTCLSTREAM5) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTREADY5I) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTPKTPND5) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTSTALLED5) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTNRDY5) |
	    ((unsigned long) 1 << F_USB30_REGISTER_SSEPS_BIT_INTCLSTALL5));

	return;
}

static inline void control_f_usb30_register_cache_bits(
	unsigned long register_id, unsigned long *register_cache)
{
	static const unsigned char register_cache_bits_control[] = {
		0,		/* HS/FS CPU Access Control       */
		0,		/* HS/FS Device Control           */
		1,		/* HS/FS Device Status            */
		0,		/* HS/FS EP Interrupt Control     */
		0,		/* HS/FS EP Interrupt Status      */
		0,		/* HS/FS EP DMA Control           */
		0,		/* HS/FS EP DMA Status            */
		0,		/* HS/FS Time Stamp               */
		0,		/* HS/FS EP Byte Count Select     */
		0,		/* HS/FS EP1 Total Byte Count     */
		0,		/* HS/FS EP2 Total Byte Count     */
		0,		/* HS/FS EP0 Rx Size              */
		0,		/* HS/FS EP1 Rx Size              */
		0,		/* HS/FS EP2 Rx Size              */
		0,		/* HS/FS EP3 Rx Size              */
		0,		/* HS/FS EP4 Rx Size              */
		0,		/* HS/FS EP5 Rx Size              */
		0,		/* HS/FS Custom Control           */
		0,		/* HS/FS Calibration              */
		0,		/* HS/FS Loop Back Selector       */
		0,		/* HS/FS Interface Alt Number     */
		0,		/* HS/FS EP0 Control              */
		2,		/* HS/FS EP0 Status               */
		0,		/* HS/FS EP1 Control              */
		3,		/* HS/FS EP1 Status               */
		0,		/* HS/FS EP2 Control              */
		3,		/* HS/FS EP2 Status               */
		0,		/* HS/FS EP3 Control              */
		3,		/* HS/FS EP3 Status               */
		0,		/* HS/FS EP4 Control              */
		3,		/* HS/FS EP4 Status               */
		0,		/* HS/FS EP5 Control              */
		3,		/* HS/FS EP5 Status               */
		0,		/* HS/FS Alternate Control        */
		4,		/* HS/FS Alternate Status         */
		0,		/* HS/FS EP0 In Buffer            */
		0,		/* HS/FS EP1 In Buffer            */
		0,		/* HS/FS EP3 In Buffer            */
		0,		/* HS/FS EP5 In Buffer            */
		0,		/* HS/FS EP0 Out Buffer           */
		0,		/* HS/FS EP2 Out Buffer           */
		0,		/* HS/FS EP4 Out Buffer           */
		0,		/* HS/FS Make-Up Area             */
		0,		/* HS/FS EP0 Make-Up Area         */
		0,		/* HS/FS EP1 Make-Up Area         */
		0,		/* HS/FS EP2 Make-Up Area         */
		0,		/* HS/FS EP3 Make-Up Area         */
		0,		/* HS/FS EP4 Make-Up Area         */
		0,		/* HS/FS EP5 Make-Up Area         */
		0,		/* Clock Control                  */
		0,		/* Clock Control Enable           */
		0,		/* SS CPU Access Control          */
		0,		/* SS Device Control              */
		5,		/* SS Device Status               */
		0,		/* SS Interrupt Request Control   */
		0,		/* SS Interrupt Request Status    */
		0,		/* SS EP DMA Control              */
		0,		/* SS EP DMA Status               */
		0,		/* SS EP Byte Count Status Select */
		0,		/* SS EP1 Total Byte Count        */
		0,		/* SS EP2 Total Byte Count        */
		0,		/* SS EP0 In Size                 */
		0,		/* SS EP0 Out Size                */
		0,		/* SS EP1 In Size                 */
		0,		/* SS EP2 Out Size                */
		0,		/* SS EP3 In Size                 */
		0,		/* SS EP4 Out Size                */
		0,		/* SS EP5 In Size                 */
		0,		/* SS Custom Control              */
		0,		/* SS Loopback Selector           */
		0,		/* SS Interface Control           */
		0,		/* SS Interface Status            */
		0,		/* SS EP0 Control                 */
		6,		/* SS EP0 Status                  */
		0,		/* SS EP1 Control                 */
		7,		/* SS EP1 Status                  */
		0,		/* SS EP2 Control                 */
		8,		/* SS EP2 Status                  */
		0,		/* SS EP3 Control                 */
		9,		/* SS EP3 Status                  */
		0,		/* SS EP4 Control                 */
		10,		/* SS EP4 Status                  */
		0,		/* SS EP5 Control                 */
		11,		/* SS EP5 Status                  */
		0,		/* SS EP0 In Buffer               */
		0,		/* SS EP1 In Buffer               */
		0,		/* SS EP3 In Buffer               */
		0,		/* SS EP5 In Buffer               */
		0,		/* SS EP0 Out Buffer              */
		0,		/* SS EP2 Out Buffer              */
		0,		/* SS EP4 Out Buffer              */
		0,		/* SS EP1 Makeup                  */
		0,		/* SS EP2 Makeup                  */
		0,		/* SS EP3 Makeup                  */
		0,		/* SS EP4 Makeup                  */
		0,		/* SS EP5 Makeup                  */
		0,		/* SS Extended Device Makeup      */
		0,		/* SS Extended EP1 Makeup         */
		0,		/* SS Extended EP2 Makeup         */
		0,		/* SS Extended EP4 Makeup         */
		0,		/* SS Extended EP5 Makeup         */
		0,		/* SS EP1 Bulk Stream Control     */
		0,		/* SS EP2 Bulk Stream Control     */
		0,		/* SS EP5 Bulk Stream Control     */
		0,		/* SS EP1 Stream Total Byte Count */
		0,		/* SS EP2 Stream Total Byte Count */
		0,		/* SS Error Counter Control       */
		0,		/* SS Data Link Error Count 1     */
		0,		/* SS Data Link Error Count 2     */
		0,		/* SS Data Link Error Count 3     */
		0,		/* SS Data Link Error Count 4     */
		0,		/* SS PHY Error Count             */
		0,		/* SS Link Control                */
		0,		/* SS Transaction Control1        */
		0,		/* SS Transaction Control2        */
		0,		/* SS Transaction Status1         */
		0,		/* SS Transaction Status2         */
		0,		/* SS Data Link Control1          */
		0,		/* SS Data Link Control2          */
		0,		/* SS Data Link Control3          */
		0,		/* SS Data Link Control4          */
		0,		/* SS PM Control1                 */
		0,		/* SS PM Control2                 */
		0,		/* SS PM Control3                 */
		0,		/* SS PM Control4                 */
		0,		/* SS PM Control5                 */
		0,		/* SS Vendor Device Test Control1 */
		0,		/* SS Vendor Device Test Control2 */
		0,		/* SS Vendor Device Test Status1  */
		0,		/* SS Vendor Device Test Status2  */
		0,		/* SS PHY Control1                */
		0,		/* SS PHY Control2                */
		0,		/* SS PHY Control3                */
		0,		/* SS PHY Control4                */
		0,		/* SS PHY Control5                */
		0,		/* SS PHY Control6                */
		0,		/* SS PHY Control7                */
		0,		/* SS PHY Control8                */
		0,		/* SS PHY Control9                */
		0,		/* SS PHY Control10               */
		0,		/* SS PHY Control11               */
		0,		/* SS PHY Control12               */
		0,		/* SS PHY Status                  */
	};

	static void (*const register_cache_bits_control_function[])(
		unsigned long *) = {
			control_f_usb30_default_register_cache_bits,
			control_f_usb30_hsdvs_register_cache_bits,
			control_f_usb30_hseps0_register_cache_bits,
			control_f_usb30_hseps_register_cache_bits,
			control_f_usb30_hsalts_register_cache_bits,
			control_f_usb30_ssdvs_register_cache_bits,
			control_f_usb30_sseps0_register_cache_bits,
			control_f_usb30_sseps1_register_cache_bits,
			control_f_usb30_sseps2_register_cache_bits,
			control_f_usb30_sseps3_register_cache_bits,
			control_f_usb30_sseps4_register_cache_bits,
			control_f_usb30_sseps5_register_cache_bits,};

	register_cache_bits_control_function[
		register_cache_bits_control[register_id]](register_cache);
	return;
}

static inline void set_f_usb30_register_bits(void __iomem *base_address,
	 unsigned long register_id, unsigned char start_bit,
	 unsigned char bit_length, unsigned long value)
{
	unsigned long register_cache;
	unsigned long mask = (unsigned long) -1 >> (32 - bit_length);

	value &= mask;

	if (f_usb30_register[register_id].access & U30_R)
		register_cache = __raw_readl(base_address +
			f_usb30_register[register_id].address_offset);

	control_f_usb30_register_cache_bits(register_id, &register_cache);

	register_cache &= ~(mask << start_bit);
	register_cache |= (value << start_bit);

	if (f_usb30_register[register_id].access & U30_W)
		__raw_writel(register_cache, base_address +
			f_usb30_register[register_id].address_offset);

	return;
}

static inline unsigned long get_f_usb30_register_bits(
	void __iomem *base_address, unsigned long register_id,
	 unsigned char start_bit, unsigned char bit_length)
{
	unsigned long register_cache;
	unsigned long mask = (unsigned long) -1 >> (32 - bit_length);

	if (f_usb30_register[register_id].access & U30_R)
		register_cache = __raw_readl(base_address +
			f_usb30_register[register_id].address_offset);

	return register_cache >> start_bit & mask;
}

static inline void set_f_usb30_makeup_register_bits(
	void __iomem *base_address, unsigned long register_id,
	unsigned char alternate, unsigned char start_bit,
	 unsigned char bit_length, unsigned long value)
{
	unsigned long register_cache;
	unsigned long mask = (unsigned long) -1 >> (32 - bit_length);

	value &= mask;

	if (f_usb30_register[register_id].access & U30_R)
		register_cache = __raw_readl(base_address +
			 f_usb30_register[register_id].address_offset +
			 alternate * 4);

	register_cache &= ~(mask << start_bit);
	register_cache |= (value << start_bit);

	if (f_usb30_register[register_id].access & U30_W)
		__raw_writel(register_cache, base_address +
			 f_usb30_register[register_id].address_offset +
			 alternate * 4);

	return;
}

static inline unsigned long get_f_usb30_makeup_register_bits(
	void __iomem *base_address, unsigned long register_id,
	 unsigned char alternate, unsigned char start_bit,
	 unsigned char bit_length)
{
	unsigned long register_cache;
	unsigned long mask = (unsigned long) -1 >> (32 - bit_length);

	if (f_usb30_register[register_id].access & U30_R)
		register_cache = __raw_readl(base_address +
			 f_usb30_register[register_id].address_offset +
			 alternate * 4);

	return register_cache >> start_bit & mask;
}

static inline void set_hs_softreset(void __iomem *base_address)
{
	unsigned long counter;
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSCPAC, 3, 1, 1);
	for (counter = 50; counter; counter--)
		;
	for (counter = 0xffff; ((counter) &&
		 (get_f_usb30_register_bits(base_address,
		 F_USB30_REG_HSCPAC, 3, 1))); counter--)
		;
	return;
}

static inline void set_hs_configwren(void __iomem *base_address,
	 unsigned char enable)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSCPAC, 16, 1, enable);
	return;
}

static inline void set_hs_reqspeed(void __iomem *base_address,
	 unsigned char bus_speed)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSDVC, 0, 2, bus_speed);
	return;
}

	/* bus speed request symbolic constant */
#define REQ_SPEED_HIGH_SPEED	0	/* high-speed request */
#define REQ_SPEED_FULL_SPEED	1	/* full-speed request */

static inline void set_hs_reqresume(void __iomem *base_address,
	 unsigned char request)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSDVC, 2, 1, request);
	return;
}

static inline void set_hs_enrmtwkup(void __iomem *base_address,
	 unsigned char enable)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSDVC, 3, 1, enable);
	return;
}

static inline unsigned char get_hs_enrmtwkup(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSDVC, 3, 1);
}

static inline void set_hs_selfpwr(void __iomem *base_address,
	 unsigned char self_power)
{
	set_f_usb30_register_bits(base_address,
					F_USB30_REG_HSDVC, 4, 1, self_power);
	return;
}

static inline void set_hs_disconnect(void __iomem *base_address,
	 unsigned char dis_connect)
{
	set_f_usb30_register_bits(base_address,
				 F_USB30_REG_HSDVC, 5, 1, dis_connect);
	return;
}

static inline void set_hs_physusp(void __iomem *base_address,
	 unsigned char force)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSDVC, 14, 1, force);
	return;
}

static inline void set_hs_lpbkphy(void __iomem *base_address,
	 unsigned char test)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSDVC, 17, 1, test);
	return;
}

static inline void set_hs_pmode(void __iomem *base_address,
	 unsigned char phy_mode)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSDVC, 18, 1, phy_mode);
	return;
}

static inline void set_hs_lmode(void __iomem *base_address,
	 unsigned char link_mode)
{
	set_f_usb30_register_bits(base_address,
					F_USB30_REG_HSDVC, 19, 1, link_mode);
	return;
}

static inline void set_hs_sofsel(void __iomem *base_address,
	 unsigned char palse)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSDVC, 23, 1, palse);
	return;
}

static inline void set_hs_msksuspende(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSDVC, 24, 1, mask);
	return;
}

static inline unsigned char get_hs_msksuspende(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSDVC, 24, 1);
}

static inline void set_hs_msksuspendb(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSDVC, 25, 1, mask);
	return;
}

static inline unsigned char get_hs_msksuspendb(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSDVC, 25, 1);
}

static inline void set_hs_msksof(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSDVC, 26, 1, mask);
	return;
}

static inline unsigned char get_hs_msksof(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSDVC, 26, 1);
}

static inline void set_hs_msksetup(void __iomem *base_address,
	unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSDVC, 27, 1, mask);
	return;
}

static inline unsigned char get_hs_msksetup(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSDVC, 27, 1);
}

static inline void set_hs_mskusbrste(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSDVC, 28, 1, mask);
	return;
}

static inline unsigned char get_hs_mskusbrste(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSDVC, 28, 1);
}

static inline void set_hs_mskusbrstb(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSDVC, 29, 1, mask);
	return;
}

static inline unsigned char get_hs_mskusbrstb(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSDVC, 29, 1);
}

static inline void set_hs_msksetconf(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSDVC, 30, 1, mask);
	return;
}

static inline unsigned char get_hs_msksetconf(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSDVC, 30, 1);
}

static inline void set_hs_mskerraticerr(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSDVC, 31, 1, mask);
	return;
}

static inline unsigned char get_hs_mskerraticerr(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSDVC, 31, 1);
}

static inline unsigned char get_hs_suspend(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSDVS, 8, 1);
}

static inline unsigned char get_hs_busreset(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSDVS, 9, 1);
}

static inline unsigned char get_hs_phyreset(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSDVS, 15, 1);
}

static inline unsigned char get_hs_crtspeed(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSDVS, 16, 2);
}

	/* bus speed symbolic constant */
#define CRT_SPEED_HIGH_SPEED	0	/* high-speed */
#define CRT_SPEED_FULL_SPEED	1	/* full-speed */
#define CRT_SPEED_LOW_SPEED	2	/* low-speed */

static inline unsigned char get_hs_conf(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSDVS, 20, 4);
}

static inline void clear_hs_intsuspende(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSDVS, 24, 1, 0);
	return;
}

static inline unsigned char get_hs_intsuspende(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSDVS, 24, 1);
}

static inline void clear_hs_intsuspendb(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSDVS, 25, 1, 0);
	return;
}

static inline unsigned char get_hs_intsuspendb(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSDVS, 25, 1);
}

static inline void clear_hs_intsof(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSDVS, 26, 1, 0);
	return;
}

static inline unsigned char get_hs_intsof(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSDVS, 26, 1);
}

static inline void clear_hs_intsetup(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSDVS, 27, 1, 0);
	return;
}

static inline unsigned char get_hs_intsetup(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSDVS, 27, 1);
}

static inline void clear_hs_intusbrste(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSDVS, 28, 1, 0);
	return;
}

static inline unsigned char get_hs_intusbrste(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSDVS, 28, 1);
}

static inline void clear_hs_intusbrstb(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSDVS, 29, 1, 0);
	return;
}

static inline unsigned char get_hs_intusbrstb(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSDVS, 29, 1);
}

static inline void clear_hs_intsetconf(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSDVS, 30, 1, 0);
	return;
}

static inline unsigned char get_hs_intsetconf(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSDVS, 30, 1);
}

static inline void clear_hs_interraticerr(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSDVS, 31, 1, 0);
	return;
}

static inline unsigned char get_hs_interraticerr(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSDVS, 31, 1);
}

static inline void set_hs_mskep(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPIC,
					 endpoint_channel, 1, mask);
	return;
}

	/* endpoint channel symbolic constant */
#define ENDPOINT0	0	/* endpoint 0 */
#define ENDPOINT1	1	/* endpoint 1 */
#define ENDPOINT2	2	/* endpoint 2 */
#define ENDPOINT3	3	/* endpoint 3 */
#define ENDPOINT4	4	/* endpoint 4 */
#define ENDPOINT5	5	/* endpoint 5 */

static inline unsigned char get_hs_mskep(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 F_USB30_REG_HSEPIC, endpoint_channel, 1);
}

static inline unsigned char get_hs_intep(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 F_USB30_REG_HSEPIS, endpoint_channel, 1);
}

static inline void set_hs_mskdmareq(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPDC,
					 endpoint_channel, 1, mask);
	return;
}

static inline void set_hs_dmamode(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char usage)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPDC,
					 endpoint_channel + 16, 1, usage);
	return;
}

static inline unsigned char get_hs_dmamode(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSEPDC,
						 endpoint_channel + 16, 1);
}

static inline unsigned char get_hs_dmareq(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 F_USB30_REG_HSEPDS, endpoint_channel, 1);
}

static inline unsigned short get_hs_timstamp(void __iomem *base_address)
{
	return (unsigned short) get_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSTSTAMP, 0, 11);
}

static inline void set_hs_tcselusb(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char usb)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPTCSEL,
					 endpoint_channel, 1, usb);
	return;
}

static inline void set_hs_tcnt(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned long count)
{
	set_f_usb30_register_bits(base_address,
				 F_USB30_REG_HSEPTC1 + endpoint_channel - 1,
				 0, 32, count);
	return;
}

static inline unsigned long get_hs_tcnt(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return get_f_usb30_register_bits(base_address,
			 F_USB30_REG_HSEPTC1 + endpoint_channel - 1, 0, 32);
}

static inline unsigned char get_hs_txrxsize0o(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSEPRS0, 0, 7);
}

static inline void set_hs_seltx0o(void __iomem *base_address,
	 unsigned char tx_size0o)
{
	set_f_usb30_register_bits(base_address,
				 F_USB30_REG_HSEPRS0, 7, 1, tx_size0o);
	return;
}

static inline unsigned char get_hs_txrxsize0i(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSEPRS0, 8, 7);
}

static inline void set_hs_seltx0i(void __iomem *base_address,
	 unsigned char tx_size0i)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPRS0,
					 15, 1, tx_size0i);
	return;
}

static inline unsigned short get_hs_txrxsize(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned short) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSEPRS0 +
						 endpoint_channel, 0, 11);
}

static inline void set_hs_seltx(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char tx_size)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPRS0 +
					 endpoint_channel, 15, 1, tx_size);
	return;
}

static inline void set_hs_cuscnt(void __iomem *base_address,
	 unsigned long cus_cnt)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSCUSTOMC,
					 0, 32, cus_cnt);
	return;
}

static inline unsigned long get_hs_cuscnt(void __iomem *base_address)
{
	return get_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSCUSTOMC, 0, 32);
}

static inline void set_hs_fscalib(void __iomem *base_address,
	 unsigned char calib)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSCALIB, 0, 3, calib);
	return;
}

static inline void set_hs_hscalib(void __iomem *base_address,
	 unsigned char calib)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSCALIB, 4, 3, calib);
	return;
}

static inline void set_hs_eplpbki0(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPLPBK,
					 0, 4, endpoint_channel);
	return;
}

static inline void set_hs_eplpbko0(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPLPBK,
					 4, 4, endpoint_channel);
	return;
}

static inline void set_hs_numaltintf(void __iomem *base_address,
	 unsigned char interface, unsigned char usage_number)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSINTFALTNUM,
					 interface * 4, 3, usage_number);
	return;
}

static inline void set_hs_numintf(void __iomem *base_address,
	 unsigned char usage_number)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSINTFALTNUM,
					 24, 4, usage_number);
	return;
}

static inline void set_hs_init0i(void __iomem *base_address)
{
	unsigned long counter;
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPC0, 0, 1, 1);
	for (counter = 0xffff;
		 ((counter) && (get_f_usb30_register_bits(base_address,
		 F_USB30_REG_HSEPC0, 0, 1))); counter--)
		;
	return;
}

static inline void set_hs_init0o(void __iomem *base_address)
{
	unsigned long counter;
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPC0, 1, 1, 1);
	for (counter = 0xffff;
		 ((counter) && (get_f_usb30_register_bits(base_address,
		 F_USB30_REG_HSEPC0, 1, 1))); counter--)
		;
	return;
}

static inline void set_hs_reqstall0(void __iomem *base_address,
	 unsigned char stall0)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPC0, 2, 1, stall0);
	return;
}

static inline unsigned char get_hs_reqstall0(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSEPC0, 2, 1);
}

static inline void set_hs_testmode0(void __iomem *base_address,
	 unsigned char test0)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPC0, 7, 1, test0);
	return;
}

static inline void set_hs_inififo0i(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPC0, 14, 1, 1);
	return;
}

static inline unsigned char get_hs_inififo0i(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSEPC0, 14, 1);
}

static inline void set_hs_inififo0o(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPC0, 15, 1, 1);
	return;
}

static inline unsigned char get_hs_inififo0o(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSEPC0, 15, 1);
}

static inline void set_hs_mskready0i(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPC0, 16, 1, mask);
	return;
}

static inline unsigned char get_hs_mskready0i(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSEPC0, 16, 1);
}

static inline void set_hs_mskready0o(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPC0, 17, 1, mask);
	return;
}

static inline unsigned char get_hs_mskready0o(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSEPC0, 17, 1);
}

static inline void set_hs_mskping0o(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPC0, 18, 1, mask);
	return;
}

static inline unsigned char get_hs_mskping0o(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSEPC0, 18, 1);
}

static inline void set_hs_mskstalled0(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPC0, 21, 1, mask);
	return;
}

static inline unsigned char get_hs_mskstalled0(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSEPC0, 21, 1);
}

static inline void set_hs_msknack0(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPC0, 22, 1, mask);
	return;
}

static inline unsigned char get_hs_msknack0(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSEPC0, 22, 1);
}

static inline void set_hs_mskclstall0(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPC0, 23, 1, mask);
	return;
}

static inline unsigned char get_hs_mskclstall0(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSEPC0, 23, 1);
}

static inline unsigned char get_hs_stalled0(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSEPS0, 2, 1);
}

static inline void enable_hs_ready0i(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPS0, 3, 1, 1);
	return;
}

static inline unsigned char get_hs_ready0i(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSEPS0, 3, 1);
}

static inline void enable_hs_ready0o(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPS0, 4, 1, 1);
	return;
}

static inline unsigned char get_hs_ready0o(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSEPS0, 4, 1);
}

static inline void clear_hs_intready0i(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPS0, 16, 1, 0);
	return;
}

static inline unsigned char get_hs_intready0i(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSEPS0, 16, 1);
}

static inline void clear_hs_intready0o(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPS0, 17, 1, 0);
	return;
}

static inline unsigned char get_hs_intready0o(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSEPS0, 17, 1);
}

static inline void clear_hs_intping0o(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPS0, 18, 1, 0);
	return;
}

static inline unsigned char get_hs_intping0o(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSEPS0, 18, 1);
}

static inline void clear_hs_intstalled0(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPS0, 21, 1, 0);
	return;
}

static inline unsigned char get_hs_intstalled0(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSEPS0, 21, 1);
}

static inline void clear_hs_intnack0(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPS0, 22, 1, 0);
	return;
}

static inline unsigned char get_hs_intnack0(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSEPS0, 22, 1);
}

static inline void clear_hs_intclstall0(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSEPS0, 23, 1, 0);
	return;
}

static inline unsigned char get_hs_intclstall0(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSEPS0, 23, 1);
}

static inline void set_hs_init(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	unsigned long counter;
	set_f_usb30_register_bits(base_address,
					 hsepc_register[endpoint_channel],
					 0, 1, 1);
	for (counter = 0xffff;
		 ((counter) && (get_f_usb30_register_bits(base_address,
		 hsepc_register[endpoint_channel], 0, 1))); counter--)
		;
	return;
}

static inline unsigned char get_hs_init(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
					hsepc_register[endpoint_channel], 0, 1);
}

static inline void set_hs_reqstall(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char stall)
{
	set_f_usb30_register_bits(base_address,
					 hsepc_register[endpoint_channel],
					 1, 1, stall);
	return;
}

static inline unsigned char get_hs_reqstall(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
					hsepc_register[endpoint_channel], 1, 1);
}

static inline void set_hs_initoggle(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	unsigned long counter;
	set_f_usb30_register_bits(base_address,
					 hsepc_register[endpoint_channel],
					 3, 1, 1);
	for (counter = 0xffff;
		 ((counter) && (get_f_usb30_register_bits(base_address,
		 hsepc_register[endpoint_channel], 3, 1)));
		 counter--)
		;
	return;
}

static inline void set_hs_inistall(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	unsigned long counter;
	set_f_usb30_register_bits(base_address,
					 hsepc_register[endpoint_channel],
					 4, 1, 1);
	for (counter = 0xffff;
		 ((counter) && (get_f_usb30_register_bits(base_address,
		 hsepc_register[endpoint_channel], 4, 1))); counter--)
		;
	return;
}

static inline void set_hs_toggledis(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char auto_disable)
{
	set_f_usb30_register_bits(base_address,
					 hsepc_register[endpoint_channel],
					 5, 1, auto_disable);
	return;
}

static inline void set_hs_stalldis(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char auto_disable)
{
	set_f_usb30_register_bits(base_address,
					 hsepc_register[endpoint_channel],
					 6, 1, auto_disable);
	return;
}

static inline void set_hs_nullresp(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char null)
{
	set_f_usb30_register_bits(base_address,
					 hsepc_register[endpoint_channel],
					 8, 1, null);
	return;
}

static inline void set_hs_nackresp(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char nack)
{
	set_f_usb30_register_bits(base_address,
					 hsepc_register[endpoint_channel],
					 9, 1, nack);
	return;
}

static inline void set_hs_enspr(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char usage)
{
	set_f_usb30_register_bits(base_address,
					 hsepc_register[endpoint_channel],
					 10, 1, usage);
	return;
}

static inline void set_hs_enspdd(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char usage)
{
	set_f_usb30_register_bits(base_address,
					 hsepc_register[endpoint_channel],
					 11, 1, usage);
	return;
}

static inline void set_hs_mskspr(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 hsepc_register[endpoint_channel],
					 13, 1, mask);
	return;
}

static inline unsigned char get_hs_mskspr(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hsepc_register[endpoint_channel], 13, 1);
}

static inline void set_hs_mskspdd(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 hsepc_register[endpoint_channel],
					 14, 1, mask);
	return;
}

static inline unsigned char get_hs_mskspdd(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hsepc_register[endpoint_channel], 14, 1);
}

static inline void set_hs_inififo(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	set_f_usb30_register_bits(base_address,
					 hsepc_register[endpoint_channel],
					 15, 1, 1);
	return;
}

static inline unsigned char get_hs_inififo(void __iomem *base_address,
		 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hsepc_register[endpoint_channel], 15, 1);
}

static inline void set_hs_mskready(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 hsepc_register[endpoint_channel],
					 16, 1, mask);
	return;
}

static inline unsigned char get_hs_mskready(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hsepc_register[endpoint_channel], 16, 1);
}

static inline void set_hs_mskping(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 hsepc_register[endpoint_channel],
					 17, 1, mask);
	return;
}

static inline unsigned char get_hs_mskping(void __iomem *base_address,
		 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hsepc_register[endpoint_channel], 17, 1);
}

static inline void set_hs_mskdend(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 hsepc_register[endpoint_channel],
					 19, 1, mask);
	return;
}

static inline unsigned char get_hs_mskdend(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hsepc_register[endpoint_channel], 19, 1);
}

static inline void set_hs_mskempty(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 hsepc_register[endpoint_channel],
					 20, 1, mask);
	return;
}

static inline unsigned char get_hs_mskempty(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hsepc_register[endpoint_channel], 20, 1);
}

static inline void set_hs_mskstalled(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 hsepc_register[endpoint_channel],
					 21, 1, mask);
	return;
}

static inline unsigned char get_hs_mskstalled(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hsepc_register[endpoint_channel], 21, 1);
}

static inline void set_hs_msknack(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 hsepc_register[endpoint_channel],
					 22, 1, mask);
	return;
}

static inline unsigned char get_hs_msknack(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hsepc_register[endpoint_channel], 22, 1);
}

static inline void set_hs_mskclstall(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 hsepc_register[endpoint_channel],
					 23, 1, mask);
	return;
}

static inline unsigned char get_hs_mskclstall(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hsepc_register[endpoint_channel], 23, 1);
}

static inline unsigned char get_hs_stalled(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hseps_register[endpoint_channel], 1, 1);
}

static inline void enable_hs_readyi(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	set_f_usb30_register_bits(base_address,
					 hseps_register[endpoint_channel],
					 2, 1, 1);
	return;
}

static inline unsigned char get_hs_readyi(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hseps_register[endpoint_channel], 2, 1);
}

static inline void enable_hs_readyo(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	set_f_usb30_register_bits(base_address,
					 hseps_register[endpoint_channel],
					 3, 1, 1);
	return;
}

static inline unsigned char get_hs_readyo(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hseps_register[endpoint_channel], 3, 1);
}

static inline unsigned char get_hs_empty(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hseps_register[endpoint_channel], 12, 1);
}

static inline void clear_hs_intspr(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	set_f_usb30_register_bits(base_address,
					 hseps_register[endpoint_channel],
					 13, 1, 0);
	return;
}

static inline unsigned char get_hs_intspr(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hseps_register[endpoint_channel], 13, 1);
}

static inline void clear_hs_intspdd(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	set_f_usb30_register_bits(base_address,
					 hseps_register[endpoint_channel],
					 14, 1, 0);
	return;
}

static inline unsigned char get_hs_intspdd(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hseps_register[endpoint_channel], 14, 1);
}

static inline void clear_hs_intready(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	set_f_usb30_register_bits(base_address,
					 hseps_register[endpoint_channel],
					 16, 1, 0);
	return;
}

static inline unsigned char get_hs_intready(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hseps_register[endpoint_channel], 16, 1);
}

static inline void clear_hs_intping(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	set_f_usb30_register_bits(base_address,
					 hseps_register[endpoint_channel],
					 17, 1, 0);
	return;
}

static inline unsigned char get_hs_intping(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hseps_register[endpoint_channel], 17, 1);
}

static inline void clear_hs_intdend(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	set_f_usb30_register_bits(base_address,
					 hseps_register[endpoint_channel],
					 19, 1, 0);
	return;
}

static inline unsigned char get_hs_intdend(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hseps_register[endpoint_channel], 19, 1);
}

static inline void clear_hs_intempty(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	set_f_usb30_register_bits(base_address,
					 hseps_register[endpoint_channel],
					 20, 1, 0);
	return;
}

static inline unsigned char get_hs_intempty(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hseps_register[endpoint_channel], 20, 1);
}

static inline void clear_hs_intstalled(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	set_f_usb30_register_bits(base_address,
					 hseps_register[endpoint_channel],
					 21, 1, 0);
	return;
}

static inline unsigned char get_hs_intstalled(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hseps_register[endpoint_channel], 21, 1);
}

static inline void clear_hs_intnack(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	set_f_usb30_register_bits(base_address,
					 hseps_register[endpoint_channel],
					 22, 1, 0);
	return;
}

static inline unsigned char get_hs_intnack(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hseps_register[endpoint_channel], 22, 1);
}

static inline void clear_hs_intclstall(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	set_f_usb30_register_bits(base_address,
					 hseps_register[endpoint_channel],
					 23, 1, 0);
	return;
}

static inline unsigned char get_hs_intclstall(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hseps_register[endpoint_channel], 23, 1);
}

static inline unsigned char get_hs_crtalt(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hseps_register[endpoint_channel], 24, 4);
}

static inline unsigned char get_hs_crtintf(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 hseps_register[endpoint_channel], 28, 4);
}

static inline void set_hs_mskachgif(void __iomem *base_address,
	 unsigned char interface_channel, unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSALTC,
					 interface_channel, 1, mask);
	return;
}

	/* interface channel symbolic constant */
#define INTERFACE0		0	/* interface 0 */
#define INTERFACE1		1	/* interface 1 */

static inline unsigned char get_hs_mskachgif(void __iomem *base_address,
	 unsigned char interface_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 F_USB30_REG_HSALTC, interface_channel, 1);
}

static inline void set_hs_testmodeif(void __iomem *base_address,
	 unsigned char test)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSALTC, 16, 1, test);
	return;
}

static inline void set_hs_testaltif(void __iomem *base_address,
	 unsigned char alternate)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSALTC,
					 24, 4, alternate);
	return;
}

static inline void set_hs_testintf(void __iomem *base_address,
	 unsigned char interface)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSALTC,
					 28, 4, interface);
	return;
}

static inline void clear_hs_intachgif(void __iomem *base_address,
	 unsigned char interface_channel)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_HSALTS,
					 interface_channel, 1, 0);
	return;
}

static inline unsigned char get_hs_intachgif(void __iomem *base_address,
	 unsigned char interface_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 F_USB30_REG_HSALTS, interface_channel, 1);
}

static inline unsigned char get_hs_curaif(void __iomem *base_address,
	 unsigned char interface_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_HSALTS,
						 16 + interface_channel * 2, 2);
}

static inline void set_hs_epinbuf(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned long data)
{
	switch (endpoint_channel) {
	case 0:
	case 1:
		__raw_writel(data, base_address +
				 f_usb30_register[F_USB30_REG_HSEPIB0 +
				 endpoint_channel].address_offset);
		break;
	case 3:
		__raw_writel(data, base_address +
				 f_usb30_register[F_USB30_REG_HSEPIB0 +
				 2].address_offset);
		break;
	case 5:
		__raw_writel(data, base_address +
				 f_usb30_register[F_USB30_REG_HSEPIB0 +
				 3].address_offset);
		break;
	default:
		break;
	}

	return;
}

static inline void set_hs_epinbuf_byte(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char byte, unsigned char data)
{
	switch (endpoint_channel) {
	case 0:
	case 1:
		__raw_writeb(data, base_address +
				 f_usb30_register[F_USB30_REG_HSEPIB0 +
				 endpoint_channel].address_offset + byte);
		break;
	case 3:
		__raw_writeb(data, base_address +
				 f_usb30_register[F_USB30_REG_HSEPIB0 +
				 2].address_offset + byte);
		break;
	case 5:
		__raw_writeb(data, base_address +
				 f_usb30_register[F_USB30_REG_HSEPIB0 +
				 3].address_offset + byte);
		break;
	default:
		break;
	}

	return;
}

static inline unsigned long get_hs_epinbuf_address(phys_addr_t base_address,
	 unsigned char endpoint_channel)
{
	unsigned char ep_channel = 0;

	switch (endpoint_channel) {
	case 0:
	case 1:
		ep_channel = endpoint_channel;
		break;
	case 3:
		ep_channel = endpoint_channel - 1;
		break;
	case 5:
		ep_channel = endpoint_channel - 2;
		break;
	default:
		break;
	}

	return (unsigned long) (base_address +
				 f_usb30_register[F_USB30_REG_HSEPIB0 +
				 ep_channel].address_offset);
}

static inline unsigned long get_hs_epoutbuf(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return __raw_readl(base_address +
				 f_usb30_register[F_USB30_REG_HSEPOB0 +
				 (endpoint_channel / 2)].address_offset);
}

static inline unsigned long get_hs_epoutbuf_address(phys_addr_t base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned long) (base_address +
				 f_usb30_register[F_USB30_REG_HSEPOB0 +
				 (endpoint_channel / 2)].address_offset);
}

static inline void set_hs_makeupdata(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_MAKEUP_DATA,
					 0, 32, 0x01200120);
	return;
}

static inline void set_hs_epnum(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char alternate_channel,
	 unsigned char number)
{
	set_f_usb30_makeup_register_bits(base_address,
						 F_USB30_REG_MAKEUP0 +
						 endpoint_channel,
						 alternate_channel,
						 0, 4, number);
	return;
}

static inline unsigned char get_hs_epnum(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char alternate_channel)
{
	return (unsigned char) get_f_usb30_makeup_register_bits(
				base_address, F_USB30_REG_MAKEUP0 +
				 endpoint_channel, alternate_channel, 0, 4);
}

static inline void set_hs_io(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char alternate_channel,
	 unsigned char in)
{
	set_f_usb30_makeup_register_bits(base_address,
						 F_USB30_REG_MAKEUP0 +
						 endpoint_channel,
						 alternate_channel, 4, 1, in);
	return;
}

static inline void set_hs_type(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char alternate_channel,
	 unsigned char type)
{
	set_f_usb30_makeup_register_bits(base_address,
						 F_USB30_REG_MAKEUP0 +
						 endpoint_channel,
						 alternate_channel, 5, 2, type);
	return;
}

/* transfer type symbolic constant */
#define TYPE_UNUSED		0	/* unused */
#define TYPE_CONTROL		0	/* control transfer */
#define TYPE_ISOCHRONOUS	1	/* isochronous transfer */
#define TYPE_BULK		2	/* bulk transfer */
#define TYPE_INTERRUPT		3	/* interrupt transfer */

static inline void set_hs_conf(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char alternate_channel,
	 unsigned char configure)
{
	set_f_usb30_makeup_register_bits(base_address,
						 F_USB30_REG_MAKEUP0 +
						 endpoint_channel,
						 alternate_channel,
						 7, 4, configure);
	return;
}

static inline void set_hs_intf(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char alternate_channel,
	 unsigned char interface)
{
	set_f_usb30_makeup_register_bits(base_address,
						 F_USB30_REG_MAKEUP0 +
						 endpoint_channel,
						 alternate_channel,
						 11, 4, interface);
	return;
}

static inline void set_hs_alt(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char alternate_channel,
	 unsigned char alternate)
{
	set_f_usb30_makeup_register_bits(base_address,
						 F_USB30_REG_MAKEUP0 +
						 endpoint_channel,
						 alternate_channel,
						 15, 4, alternate);
	return;
}

static inline void set_hs_size(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char alternate_channel,
	 unsigned short size)
{
	set_f_usb30_makeup_register_bits(base_address,
						 F_USB30_REG_MAKEUP0 +
						 endpoint_channel,
						 alternate_channel,
						 19, 11, size);
	return;
}

static inline unsigned short get_hs_size(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char alternate_channel)
{
	return (unsigned short) get_f_usb30_makeup_register_bits(
					base_address, F_USB30_REG_MAKEUP0 +
					 endpoint_channel,
					 alternate_channel, 19, 11);
}

static inline void set_hs_numtr(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char alternate_channel)
{
	set_f_usb30_makeup_register_bits(base_address,
						 F_USB30_REG_MAKEUP0 +
						 endpoint_channel,
						 alternate_channel, 30, 2, 0);
	return;
}

static inline void set_hsclkstp(void __iomem *base_address, unsigned char stop)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_CLKC, 0, 1, stop);
	return;
}

static inline unsigned char get_hsclkstp(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_CLKC, 0, 1);
}

static inline void set_ssclkstp(void __iomem *base_address, unsigned char stop)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_CLKC, 1, 1, stop);
	return;
}

static inline unsigned char get_ssclkstp(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_CLKC, 1, 1);
}

static inline void set_hsclkstpen(void __iomem *base_address,
	 unsigned char stop)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_CLKCE, 0, 1, stop);
	return;
}

static inline unsigned char get_hsclkstpen(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_CLKCE, 0, 1);
}

static inline void set_ssclkstpen(void __iomem *base_address,
	 unsigned char stop)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_CLKCE, 1, 1, stop);
	return;
}

static inline unsigned char get_ssclkstpen(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_CLKCE, 1, 1);
}

static inline void set_ss_softreset(void __iomem *base_address,
	 unsigned char reset)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSCPAC, 3, 1, reset);
	return;
}

static inline void set_ss_configwren(void __iomem *base_address,
	 unsigned char enable)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSCPAC, 16, 1, enable);
	return;
}

static inline void set_ss_ssovrden(void __iomem *base_address,
	 unsigned char enable)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSCPAC, 17, 1, enable);
	return;
}

static inline void set_ss_ltactive(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVC, 0, 1, 1);
	return;
}

static inline void set_ss_selfpw(void __iomem *base_address,
	 unsigned char self_power)
{
	set_f_usb30_register_bits(base_address,
					F_USB30_REG_SSDVC, 4, 1, self_power);
	return;
}

static inline void set_ss_disconnect(void __iomem *base_address,
	 unsigned char dis_connect)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVC,
					 5, 1, dis_connect);
	return;
}

static inline void set_ss_connect(void __iomem *base_address,
	 unsigned char connect)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVC, 6, 1, connect);
	return;
}

static inline void set_ss_rxdetonce(void __iomem *base_address,
	 unsigned char detection)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVC, 7, 1, detection);
	return;
}

static inline void set_ss_msksswrste(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVC, 12, 1, mask);
	return;
}

static inline unsigned char get_ss_msksswrste(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVC, 12, 1);
}

static inline void set_ss_msksswrstb(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVC, 13, 1, mask);
	return;
}

static inline unsigned char get_ss_msksswrstb(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVC, 13, 1);
}

static inline void set_ss_msksshrste(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVC, 14, 1, mask);
	return;
}

static inline unsigned char get_ss_msksshrste(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVC, 14, 1);
}

static inline void set_ss_msksshrstb(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVC, 15, 1, mask);
	return;
}

static inline unsigned char get_ss_msksshrstb(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVC, 15, 1);
}

static inline void set_ss_mskvdtest(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVC, 20, 1, mask);
	return;
}

static inline unsigned char get_ss_mskvdtest(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVC, 20, 1);
}

static inline void set_ss_mskssdisable(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVC, 21, 1, mask);
	return;
}

static inline unsigned char get_ss_mskssdisable(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVC, 21, 1);
}

static inline void set_ss_mskenterpoll(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVC, 22, 1, mask);
	return;
}

static inline unsigned char get_ss_mskenterpoll(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVC, 22, 1);
}

static inline void set_ss_mskpolltou0(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVC, 23, 1, mask);
	return;
}

static inline unsigned char get_ss_mskpolltou0(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVC, 23, 1);
}

static inline void set_ss_msksuspende(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVC, 24, 1, mask);
	return;
}

static inline unsigned char get_ss_msksuspende(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVC, 24, 1);
}

static inline void set_ss_msksuspendb(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVC, 25, 1, mask);
	return;
}

static inline unsigned char get_ss_msksuspendb(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVC, 25, 1);
}

static inline void set_ss_msksetintf(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVC, 26, 1, mask);
	return;
}

static inline unsigned char get_ss_msksetintf(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVC, 26, 1);
}

static inline void set_ss_msksetup(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVC, 27, 1, mask);
	return;
}

static inline unsigned char get_ss_msksetup(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVC, 27, 1);
}

static inline void set_ss_msku2inactto(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVC, 29, 1, mask);
	return;
}

static inline unsigned char get_ss_msku2inactto(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVC, 29, 1);
}

static inline void set_ss_msksetconf(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVC, 30, 1, mask);
	return;
}

static inline unsigned char get_ss_msksetconf(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVC, 30, 1);
}

static inline void set_ss_mskfncsusp(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVC, 31, 1, mask);
	return;
}

static inline unsigned char get_ss_mskfncsusp(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVC, 31, 1);
}

static inline unsigned char get_ss_u0up(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVS, 0, 1);
}

static inline unsigned char get_ss_ltstate(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVS, 4, 1);
}

static inline unsigned char get_ss_suspend(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVS, 8, 1);
}

static inline unsigned char get_ss_sswrst(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVS, 9, 1);
}

static inline unsigned char get_ss_sshrst(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVS, 10, 1);
}

static inline unsigned char get_ss_ssdiserr(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVS, 11, 1);
}

static inline unsigned char get_ss_intsswrste(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVS, 12, 1);
}

static inline void clear_ss_intsswrste(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVS, 12, 1, 0);
	return;
}

static inline unsigned char get_ss_intsswrstb(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVS, 13, 1);
}

static inline void clear_ss_intsswrstb(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVS, 13, 1, 0);
	return;
}

static inline unsigned char get_ss_intsshrste(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVS, 14, 1);
}

static inline void clear_ss_intsshrste(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVS, 14, 1, 0);
	return;
}

static inline unsigned char get_ss_intsshrstb(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVS, 15, 1);
}

static inline void clear_ss_intsshrstb(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
		 F_USB30_REG_SSDVS, 15, 1, 0);
	return;
}

static inline unsigned char get_ss_conf(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVS, 16, 4);
}

static inline unsigned char get_ss_intvdtest(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVS, 20, 1);
}

static inline void clear_ss_intvdtest(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVS, 20, 1, 0);
	return;
}

static inline unsigned char get_ss_intssdisable(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVS, 21, 1);
}

static inline void clear_ss_intssdisable(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVS, 21, 1, 0);
	return;
}

static inline unsigned char get_ss_intenterpoll(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVS, 22, 1);
}

static inline void clear_ss_intenterpoll(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVS, 22, 1, 0);
	return;
}

static inline unsigned char get_ss_intpolltou0(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVS, 23, 1);
}

static inline void clear_ss_intpolltou0(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVS, 23, 1, 0);
	return;
}

static inline unsigned char get_ss_intsuspende(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVS, 24, 1);
}

static inline void clear_ss_intsuspende(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVS, 24, 1, 0);
	return;
}

static inline unsigned char get_ss_intsuspendb(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVS, 25, 1);
}

static inline void clear_ss_intsuspendb(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVS, 25, 1, 0);
	return;
}

static inline unsigned char get_ss_intsetintf(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVS, 26, 1);
}

static inline void clear_ss_intsetintf(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVS, 26, 1, 0);
	return;
}

static inline unsigned char get_ss_intsetup(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVS, 27, 1);
}

static inline void clear_ss_intsetup(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVS, 27, 1, 0);
	return;
}

static inline unsigned char get_ss_intu2inactto(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVS, 29, 1);
}

static inline void clear_ss_intu2inactto(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVS, 29, 1, 0);
	return;
}

static inline unsigned char get_ss_intsetconf(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVS, 30, 1);
}

static inline void clear_ss_intsetconf(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVS, 30, 1, 0);
	return;
}

static inline unsigned char get_ss_intfncsusp(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDVS, 31, 1);
}

static inline void clear_ss_intfncsusp(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDVS, 31, 1, 0);
	return;
}

static inline void set_ss_mskep(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSIRQC,
					 endpoint_channel, 1, mask);
	return;
}

static inline unsigned char get_ss_mskep(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 F_USB30_REG_SSIRQC, endpoint_channel, 1);
}

static inline void set_ss_mskdev(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSIRQC, 16, 1, mask);
	return;
}

static inline unsigned char get_ss_mskdev(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSIRQC, 16, 1);
}

static inline unsigned char get_ss_intep(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 F_USB30_REG_SSIRQS, endpoint_channel, 1);
}

static inline unsigned char get_ss_intdev(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSIRQS, 16, 1);
}

static inline void set_ss_mskdmareq(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	if (endpoint_channel == ENDPOINT1 || endpoint_channel == ENDPOINT2)
		set_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPDC,
						 endpoint_channel, 1, mask);
	return;
}

static inline void set_ss_dmamode(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char usage)
{
	if (endpoint_channel == ENDPOINT1 || endpoint_channel == ENDPOINT2)
		set_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPDC,
						 endpoint_channel + 16,
						 1, usage);
	return;
}

static inline unsigned char get_ss_dmamode(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	if (endpoint_channel != ENDPOINT1 && endpoint_channel != ENDPOINT2)
		return 0;

	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPDC,
						 endpoint_channel + 16, 1);
}

static inline unsigned char get_ss_dmareq(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	if (endpoint_channel != ENDPOINT1 && endpoint_channel != ENDPOINT2)
		return 0;

	return (unsigned char) get_f_usb30_register_bits(base_address,
							 F_USB30_REG_SSEPDS,
							 endpoint_channel, 1);
}

static inline void set_ss_tcsel(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char usb)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPTCSEL,
					 endpoint_channel, 1, usb);
	return;
}

static inline void set_ss_tcnt(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned long count)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPTC1 +
					 endpoint_channel - 1, 0, 32, count);
	return;
}

static inline unsigned long get_ss_tcnt(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPTC1 +
						 endpoint_channel - 1, 0, 32);
}

static inline unsigned char get_ss_sizerd0i(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPSZ0I, 0, 11);
}

static inline unsigned char get_ss_sizewr0i(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPSZ0I, 16, 11);
}

static inline unsigned char get_ss_sizerd0o(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPSZ0O, 0, 11);
}

static inline unsigned char get_ss_sizewr0o(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPSZ0O, 16, 11);
}

static inline unsigned long get_ss_sizerdwr(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned long) get_f_usb30_register_bits(base_address,
					 ssepsz_register[endpoint_channel - 1],
					 16, 11);
}

static inline unsigned char get_ss_sizerd1i(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPSZ1I, 0, 11);
}

static inline unsigned char get_ss_sizewr1i(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPSZ1I, 16, 11);
}

static inline unsigned char get_ss_sizerd2o(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPSZ2O, 0, 11);
}

static inline unsigned char get_ss_sizewr2o(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPSZ2O, 16, 11);
}

static inline unsigned char get_ss_sizerd3i(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPSZ3I, 0, 11);
}

static inline unsigned char get_ss_sizewr3i(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPSZ3I, 16, 11);
}

static inline unsigned char get_ss_sizerd4o(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPSZ4O, 0, 11);
}

static inline unsigned char get_ss_sizewr4o(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPSZ4O, 16, 11);
}

static inline unsigned char get_ss_sizerd5i(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPSZ5I, 0, 11);
}

static inline unsigned char get_ss_sizewr5i(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPSZ5I, 16, 11);
}

static inline void set_ss_bulktest(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSCUSC, 0, 1, 1);
	return;
}

static inline unsigned long get_ss_bulktest(void __iomem *base_address)
{
	return get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSCUSC, 0, 1);
}

static inline void set_ss_setadd(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSCUSC, 16, 1, 1);
	return;
}

static inline unsigned long get_ss_setadd(void __iomem *base_address)
{
	return get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSCUSC, 16, 1);
}

static inline void set_ss_setconfig(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSCUSC, 17, 1, 1);
	return;
}

static inline unsigned long get_ss_setconfig(void __iomem *base_address)
{
	return get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSCUSC, 17, 1);
}

static inline void set_ss_tconf(void __iomem *base_address,
	 unsigned char config)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSCUSC, 20, 3, config);
	return;
}

static inline void set_ss_tadd(void __iomem *base_address,
	 unsigned char address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSCUSC, 24, 7, address);
	return;
}

static inline void set_ss_dlllpbk(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPLPBK, 0, 1, 1);
	return;
}

static inline void set_ss_numaintf(void __iomem *base_address,
	 unsigned char interface, unsigned char usage_number)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSINTFC,
					 interface * 4, 3, usage_number);
	return;
}

static inline void set_ss_firstintf(void __iomem *base_address,
	 unsigned char interface, unsigned char firstintf)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSINTFC,
					 interface * 4 + 3, 1, firstintf);
	return;
}

static inline unsigned long get_ss_firstintf(void __iomem *base_address,
	 unsigned char interface)
{
	return get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSINTFC,
						 interface * 4 + 3, 1);
}

static inline void set_ss_numintf(void __iomem *base_address,
	unsigned char usage_number)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSINTFC,
					 16, 3, usage_number);
	return;
}

static inline unsigned long get_ss_enfncwake(void __iomem *base_address,
	 unsigned char interface)
{
	return get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSINTFC,
						 interface + 24, 1);
}

static inline void set_ss_reqfncwake(void __iomem *base_address,
	 unsigned char interface, unsigned char enable)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSINTFC,
					 interface + 28, 1, enable);
	return;
}

static inline unsigned long get_ss_alt(void __iomem *base_address,
	 unsigned char interface)
{
	return get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSINTFS,
						 interface * 4, 3);
}

static inline unsigned long get_ss_achg(void __iomem *base_address,
	 unsigned char interface)
{
	return get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSINTFS,
						 interface * 4 + 3, 1);
}

static inline void clear_ss_achg(void __iomem *base_address,
	 unsigned char interface)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSINTFS,
					 interface * 4 + 3, 1, 0);
	return;
}

static inline unsigned long get_ss_fncsusp(void __iomem *base_address,
	 unsigned char interface)
{
	return get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSINTFS,
						 interface + 16, 1);
}

static inline void set_ss_init0i(void __iomem *base_address)
{
	unsigned long counter;
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPC0, 0, 1, 1);
	for (counter = 0xffff;
		 ((counter) && (get_f_usb30_register_bits(base_address,
		 F_USB30_REG_SSEPC0, 0, 1))); counter--)
		;
	return;
}

static inline void set_ss_init0o(void __iomem *base_address)
{
	unsigned long counter;
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPC0, 1, 1, 1);
	for (counter = 0xffff;
		 ((counter) && (get_f_usb30_register_bits(base_address,
		 F_USB30_REG_SSEPC0, 1, 1))); counter--)
		;
	return;
}

static inline void set_ss_reqstall0(void __iomem *base_address,
	 unsigned char stall0)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPC0, 2, 1, stall0);
	return;
}

static inline unsigned char get_ss_reqstall0(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPC0, 2, 1);
}

static inline void set_ss_rewdifo0(void __iomem *base_address,
	 unsigned char enable)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPC0, 3, 1, enable);
	return;
}

static inline void set_ss_nrdyresp0(void __iomem *base_address,
	 unsigned char enable)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPC0, 9, 1, enable);
	return;
}

static inline void set_ss_mskready0i(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPC0, 16, 1, mask);
	return;
}

static inline unsigned char get_ss_mskready0i(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPC0, 16, 1);
}

static inline void set_ss_mskready0o(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPC0, 17, 1, mask);
	return;
}

static inline unsigned char get_ss_mskready0o(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPC0, 17, 1);
}

static inline void set_ss_mskpktpnd0(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPC0, 18, 1, mask);
	return;
}

static inline void set_ss_mskstalled0(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPC0, 21, 1, mask);
	return;
}

static inline unsigned char get_ss_mskstalled0(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPC0, 21, 1);
}

static inline void set_ss_msknrdy0(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPC0, 22, 1, mask);
	return;
}

static inline void set_ss_mskclstall0(void __iomem *base_address,
	 unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPC0, 23, 1, mask);
	return;
}

static inline unsigned char get_ss_mskclstall0(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPC0, 23, 1);
}

static inline unsigned char get_ss_appactiv0(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPC0, 24, 1);
}

static inline void set_ss_ctldone0(void __iomem *base_address,
	 unsigned char enable)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPS0, 0, 1, enable);
	return;
}

static inline unsigned char get_ss_stalled0(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPS0, 2, 1);
}

static inline void enable_ss_ready0i(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPS0, 3, 1, 1);
	return;
}

static inline unsigned char get_ss_ready0i(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPS0, 3, 1);
}

static inline void enable_ss_ready0o(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPS0, 4, 1, 1);
	return;
}

static inline unsigned char get_ss_ready0o(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPS0, 4, 1);
}

static inline unsigned char get_ss_pktpnd0(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPS0, 8, 1);
}

static inline void clear_ss_intready0i(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPS0, 16, 1, 0);
	return;
}

static inline unsigned char get_ss_intready0i(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPS0, 16, 1);
}

static inline void clear_ss_intready0o(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPS0, 17, 1, 0);
	return;
}

static inline unsigned char get_ss_intready0o(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPS0, 17, 1);
}

static inline void clear_ss_intpktpnd0(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPS0, 18, 1, 0);
	return;
}

static inline unsigned char get_ss_intpktpnd0(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPS0, 18, 1);
}

static inline void clear_ss_intstalled0(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPS0, 21, 1, 0);
	return;
}

static inline unsigned char get_ss_intstalled0(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPS0, 21, 1);
}

static inline void clear_ss_intnrdy0(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPS0, 22, 1, 0);
	return;
}

static inline unsigned char get_ss_intnrdy0(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPS0, 22, 1);
}

static inline void clear_ss_intclstall0(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPS0, 23, 1, 0);
	return;
}

static inline unsigned char get_ss_intclstall0(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPS0, 23, 1);
}

static inline void set_ss_init(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	unsigned long counter;
	set_f_usb30_register_bits(base_address,
					 ssepc_register[endpoint_channel],
					 0, 1, 1);
	for (counter = 0xffff;
		 ((counter) && (get_f_usb30_register_bits(base_address,
		 ssepc_register[endpoint_channel], 0, 1))); counter--)
		;
	return;
}

static inline unsigned char get_ss_init(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 ssepc_register[endpoint_channel], 0, 1);
}

static inline void set_ss_reqstall(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char stall)
{
	set_f_usb30_register_bits(base_address,
					 ssepc_register[endpoint_channel],
					 1, 1, stall);
	return;
}

static inline unsigned char get_ss_reqstall(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 ssepc_register[endpoint_channel], 1, 1);
}

static inline void set_ss_rewdifo(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char rew)
{
	set_f_usb30_register_bits(base_address,
					 ssepc_register[endpoint_channel],
					 3, 1, rew);
	return;
}

static inline unsigned char get_ss_rewdifo(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 ssepc_register[endpoint_channel], 3, 1);
}

static inline void set_ss_inistall(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char stall)
{
	set_f_usb30_register_bits(base_address,
					 ssepc_register[endpoint_channel],
					 4, 1, stall);
	return;
}

static inline unsigned char get_ss_inistall(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 ssepc_register[endpoint_channel], 4, 1);
}

static inline void set_ss_stalldis(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char stall)
{
	set_f_usb30_register_bits(base_address,
					 ssepc_register[endpoint_channel],
					 6, 1, stall);
	return;
}

static inline unsigned char get_ss_stalldis(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 ssepc_register[endpoint_channel], 6, 1);
}

static inline void set_ss_autostreamclr(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char enable)
{
	set_f_usb30_register_bits(base_address,
					 ssepc_register[endpoint_channel],
					 7, 1, enable);
	return;
}

static inline unsigned char get_ss_autostreamclr(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 ssepc_register[endpoint_channel], 7, 1);
}

static inline void set_ss_enstream(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char enable)
{
	set_f_usb30_register_bits(base_address,
					 ssepc_register[endpoint_channel],
					 8, 1, enable);
	return;
}

static inline unsigned char get_ss_enstream(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 ssepc_register[endpoint_channel], 8, 1);
}

static inline void set_ss_nrdyresp(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char enable)
{
	set_f_usb30_register_bits(base_address,
					 ssepc_register[endpoint_channel],
					 9, 1, enable);
	return;
}

static inline unsigned char get_ss_nrdyresp(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 ssepc_register[endpoint_channel], 9, 1);
}

static inline void set_ss_mskclstream(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 ssepc_register[endpoint_channel],
					 15, 1, mask);
	return;
}

static inline unsigned char get_ss_mskclstream(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 ssepc_register[endpoint_channel], 15, 1);
}

static inline void set_ss_mskready(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 ssepc_register[endpoint_channel],
					 16, 1, mask);
	return;
}

static inline unsigned char get_ss_mskready(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 ssepc_register[endpoint_channel], 16, 1);
}

static inline void set_ss_mskpktpnd(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 ssepc_register[endpoint_channel],
					 17, 1, mask);
	return;
}

static inline unsigned char get_ss_mskpktpnd(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 ssepc_register[endpoint_channel], 17, 1);
}

static inline void set_ss_msksdend(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	if (endpoint_channel == ENDPOINT1 || endpoint_channel == ENDPOINT2)
		set_f_usb30_register_bits(base_address,
				 ssepc_register[endpoint_channel], 18, 1, mask);
	return;
}

static inline unsigned char get_ss_msksdend(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	if (endpoint_channel != ENDPOINT1 && endpoint_channel != ENDPOINT2)
		return 1;

	return (unsigned char) get_f_usb30_register_bits(base_address,
				 ssepc_register[endpoint_channel], 18, 1);
}

static inline void set_ss_mskdend(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	if (endpoint_channel == ENDPOINT1 || endpoint_channel == ENDPOINT2)
		set_f_usb30_register_bits(base_address,
				 ssepc_register[endpoint_channel], 19, 1, mask);
	return;
}

static inline unsigned char get_ss_mskdend(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	if (endpoint_channel != ENDPOINT1 && endpoint_channel != ENDPOINT2)
		return 1;

	return (unsigned char) get_f_usb30_register_bits(base_address,
				 ssepc_register[endpoint_channel], 19, 1);
}

static inline void set_ss_mskempty(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	if (endpoint_channel == ENDPOINT1 || endpoint_channel == ENDPOINT2)
		set_f_usb30_register_bits(base_address,
				 ssepc_register[endpoint_channel], 20, 1, mask);
	return;
}

static inline unsigned char get_ss_mskempty(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	if (endpoint_channel != ENDPOINT1 && endpoint_channel != ENDPOINT2)
		return 1;

	return (unsigned char) get_f_usb30_register_bits(base_address,
				 ssepc_register[endpoint_channel], 20, 1);
}

static inline void set_ss_mskstalled(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 ssepc_register[endpoint_channel],
					 21, 1, mask);
	return;
}

static inline unsigned char get_ss_mskstalled(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 ssepc_register[endpoint_channel], 21, 1);
}

static inline void set_ss_msknrdy(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 ssepc_register[endpoint_channel],
					 22, 1, mask);
	return;
}

static inline unsigned char get_ss_msknrdy(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 ssepc_register[endpoint_channel], 22, 1);
}

static inline void set_ss_mskclstall(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	set_f_usb30_register_bits(base_address,
					 ssepc_register[endpoint_channel],
					 23, 1, mask);
	return;
}

static inline unsigned char get_ss_mskclstall(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 ssepc_register[endpoint_channel], 23, 1);
}

static inline unsigned char get_ss_appactiv(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 ssepc_register[endpoint_channel], 24, 1);
}

static inline void set_ss_enspr(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char spr)
{
	if (endpoint_channel == ENDPOINT2)
		set_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPC2,
						 10, 1, spr);
	return;
}

static inline void set_ss_enspd(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char spd)
{
	if (endpoint_channel == ENDPOINT2)
		set_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPC2,
						 11, 1, spd);
	return;
}

static inline void set_ss_mskspr(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	if (endpoint_channel == ENDPOINT2)
		set_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPC2,
						 13, 1, mask);
	return;
}

static inline unsigned char get_ss_mskspr(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	if (endpoint_channel != ENDPOINT2)
		return 1;

	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPC2, 13, 1);
}

static inline void set_ss_mskspd(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mask)
{
	if (endpoint_channel == ENDPOINT2)
		set_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPC2,
						 14, 1, mask);
	return;
}

static inline unsigned char get_ss_mskspd(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	if (endpoint_channel != ENDPOINT2)
		return 1;

	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPC2, 14, 1);
}

static inline void set_ss_spunit(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char length)
{
	if (endpoint_channel == ENDPOINT2)
		set_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPC2,
						 28, 3, length);
	return;
}

static inline unsigned char get_ss_spunit(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	if (endpoint_channel != ENDPOINT2)
		return 0;

	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPC2, 28, 3);
}

static inline unsigned char get_ss_stalled(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 sseps_register[endpoint_channel], 1, 1);
}

static inline void enable_ss_ready(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	switch (endpoint_channel) {
	case 1:
	case 3:
	case 5:
		set_f_usb30_register_bits(base_address,
				 sseps_register[endpoint_channel], 2, 1, 1);
		break;
	case 2:
	case 4:
		set_f_usb30_register_bits(base_address,
				 sseps_register[endpoint_channel], 3, 1, 1);
		break;
	default:
		break;
	}

	return;
}

static inline unsigned char get_ss_ready(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	unsigned char flag = 0;
	switch (endpoint_channel) {
	case 1:
	case 3:
	case 5:
		flag = (unsigned char) get_f_usb30_register_bits(
					base_address, sseps_register[
					endpoint_channel], 2, 1);
		break;
	case 2:
	case 4:
		flag = (unsigned char) get_f_usb30_register_bits(
					base_address, sseps_register[
					endpoint_channel], 3, 1);
		break;
	default:
		break;
	}

	return flag;
}

static inline void set_ss_streamactive(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char active)
{
	set_f_usb30_register_bits(base_address,
					 sseps_register[endpoint_channel],
					 7, 1, active);
	return;
}

static inline unsigned char get_ss_streamactive(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 sseps_register[endpoint_channel], 7, 1);
}

static inline unsigned char get_ss_pktpnd(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 sseps_register[endpoint_channel], 8, 1);
}

static inline unsigned char get_ss_empty(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 sseps_register[endpoint_channel], 12, 1);
}

static inline unsigned char get_ss_intspr(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	if (endpoint_channel != ENDPOINT2)
		return 0;

	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPS2, 13, 1);
}

static inline void clear_ss_intspr(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	if (endpoint_channel == ENDPOINT2)
		set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPS2, 13, 1, 0);
	return;
}

static inline unsigned char get_ss_intspd(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	if (endpoint_channel != ENDPOINT2)
		return 0;

	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPS2, 14, 1);
}

static inline void clear_ss_intspd(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	if (endpoint_channel == ENDPOINT2)
		set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEPS2, 14, 1, 0);
	return;
}

static inline unsigned char get_ss_intclstream(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	unsigned char flag = 0;
	switch (endpoint_channel) {
	case 1:
	case 2:
	case 5:
		flag = (unsigned char) get_f_usb30_register_bits(
					base_address, sseps_register[
					endpoint_channel], 15, 1);
		break;
	default:
		break;
	}

	return flag;
}

static inline void clear_ss_intclstream(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	switch (endpoint_channel) {
	case 1:
	case 2:
	case 5:
		set_f_usb30_register_bits(base_address,
						 sseps_register[
						 endpoint_channel], 15, 1, 0);
		break;
	default:
		break;
	}

	return;
}

static inline unsigned char get_ss_intready(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 sseps_register[endpoint_channel], 16, 1);
}

static inline void clear_ss_intready(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	set_f_usb30_register_bits(base_address,
					 sseps_register[endpoint_channel],
					 16, 1, 0);
	return;
}

static inline unsigned char get_ss_intpktpnd(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 sseps_register[endpoint_channel], 17, 1);
}

static inline void clear_ss_intpktpnd(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	set_f_usb30_register_bits(base_address,
					 sseps_register[endpoint_channel],
					 17, 1, 0);
	return;
}

static inline unsigned char get_ss_intsdend(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	if (endpoint_channel != ENDPOINT1 && endpoint_channel != ENDPOINT2)
		return 0;

	return (unsigned char) get_f_usb30_register_bits(base_address,
				 sseps_register[endpoint_channel], 18, 1);
}

static inline void clear_ss_intsdend(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	if (endpoint_channel == ENDPOINT1 || endpoint_channel == ENDPOINT2)
		set_f_usb30_register_bits(base_address,
						 sseps_register[
						 endpoint_channel], 18, 1, 0);
	return;
}

static inline unsigned char get_ss_intdend(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	if (endpoint_channel != ENDPOINT1 && endpoint_channel != ENDPOINT2)
		return 0;

	return (unsigned char) get_f_usb30_register_bits(base_address,
				 sseps_register[endpoint_channel], 19, 1);
}

static inline void clear_ss_intdend(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	if (endpoint_channel == ENDPOINT1 || endpoint_channel == ENDPOINT2)
		set_f_usb30_register_bits(base_address,
						 sseps_register[
						 endpoint_channel], 19, 1, 0);
	return;
}

static inline unsigned char get_ss_intempty(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	if (endpoint_channel != ENDPOINT1 && endpoint_channel != ENDPOINT2)
		return 0;

	return (unsigned char) get_f_usb30_register_bits(base_address,
				 sseps_register[endpoint_channel], 20, 1);
}

static inline void clear_ss_intempty(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	if (endpoint_channel == ENDPOINT1 || endpoint_channel == ENDPOINT2)
		set_f_usb30_register_bits(base_address,
						 sseps_register[
						 endpoint_channel], 20, 1, 0);
	return;
}

static inline unsigned char get_ss_intstalled(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 sseps_register[endpoint_channel], 21, 1);
}

static inline void clear_ss_intstalled(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	set_f_usb30_register_bits(base_address,
					 sseps_register[endpoint_channel],
					 21, 1, 0);
	return;
}

static inline unsigned char get_ss_intnrdy(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 sseps_register[endpoint_channel], 22, 1);
}

static inline void clear_ss_intnrdy(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	set_f_usb30_register_bits(base_address,
					 sseps_register[endpoint_channel],
					 22, 1, 0);
	return;
}

static inline unsigned char get_ss_intclstall(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
				 sseps_register[endpoint_channel], 23, 1);
}

static inline void clear_ss_intclstall(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	set_f_usb30_register_bits(base_address,
					 sseps_register[endpoint_channel],
					 23, 1, 0);
	return;
}

static inline void set_ss_epinbuf(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned long data)
{
	switch (endpoint_channel) {
	case 0:
	case 1:
		__raw_writel(data, base_address +
				 f_usb30_register[F_USB30_REG_SSEPIB0 +
				 endpoint_channel].address_offset);
		break;
	case 3:
		__raw_writel(data, base_address +
				 f_usb30_register[F_USB30_REG_SSEPIB0 +
				 2].address_offset);
		break;
	case 5:
		__raw_writel(data, base_address +
				 f_usb30_register[F_USB30_REG_SSEPIB0 +
				 3].address_offset);
		break;
	default:
		break;
	}

	return;
}

static inline void set_ss_epinbuf_byte(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char byte, unsigned char data)
{
	switch (endpoint_channel) {
	case 0:
	case 1:
		__raw_writeb(data, base_address +
				 f_usb30_register[F_USB30_REG_SSEPIB0 +
				 endpoint_channel].address_offset + byte);
		break;
	case 3:
		__raw_writeb(data, base_address +
				 f_usb30_register[F_USB30_REG_SSEPIB0 +
				 2].address_offset + byte);
		break;
	case 5:
		__raw_writeb(data, base_address +
				 f_usb30_register[F_USB30_REG_SSEPIB0 +
				 3].address_offset + byte);
		break;
	default:
		break;
	}

	return;
}

static inline unsigned long get_ss_epinbuf_address(phys_addr_t base_address,
	 unsigned char endpoint_channel)
{
	unsigned char ep_channel = 0;

	switch (endpoint_channel) {
	case 0:
	case 1:
		ep_channel = endpoint_channel;
		break;
	case 3:
		ep_channel = endpoint_channel - 1;
		break;
	case 5:
		ep_channel = endpoint_channel - 2;
		break;
	default:
		break;
	}

	return (unsigned long) (base_address +
				 f_usb30_register[F_USB30_REG_SSEPIB0 +
				 ep_channel].address_offset);
}

static inline unsigned long get_ss_epoutbuf(void __iomem *base_address,
	 unsigned char endpoint_channel)
{
	return __raw_readl(base_address +
				 f_usb30_register[F_USB30_REG_SSEPOB0 +
				 (endpoint_channel / 2)].address_offset);
}

static inline unsigned long get_ss_epoutbuf_address(phys_addr_t base_address,
	 unsigned char endpoint_channel)
{
	return (unsigned long) (base_address +
				 f_usb30_register[F_USB30_REG_SSEPOB0 +
				 (endpoint_channel / 2)].address_offset);
}

static inline void set_ss_epnum(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char number)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSCFGEP1 +
					 endpoint_channel - 1, 0, 4, number);
	return;
}

static inline unsigned char get_ss_epnum(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSCFGEP1 +
						 endpoint_channel - 1, 0, 4);
}

static inline void set_ss_epconf(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char config)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSCFGEP1 +
					 endpoint_channel - 1, 7, 4, config);
	return;
}

static inline unsigned char get_ss_epconf(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSCFGEP1 +
						 endpoint_channel - 1, 7, 4);
}

static inline void set_ss_intf(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char interface)
{
	set_f_usb30_register_bits(base_address, F_USB30_REG_SSCFGEP1 +
				 endpoint_channel - 1, 11 + interface, 1, 1);
	return;
}

static inline unsigned char get_ss_intf(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSCFGEP1 +
						 endpoint_channel - 1, 11, 4);
}

static inline void set_ss_altmap(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char alternate)
{
	set_f_usb30_register_bits(base_address, F_USB30_REG_SSCFGEP1 +
					endpoint_channel - 1, 15, 4, alternate);
	return;
}

static inline unsigned char get_ss_altmap(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSCFGEP1 +
						 endpoint_channel - 1, 15, 4);
}

static inline void set_ss_capltm(void __iomem *base_address,
	 unsigned char ltm)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEXCFGDV, 0, 1, ltm);
	return;
}

static inline unsigned long get_ss_capltm(void __iomem *base_address)
{
	return get_f_usb30_register_bits(base_address,
						F_USB30_REG_SSEXCFGDV, 0, 1);
}

static inline void set_ss_capfncwake(void __iomem *base_address,
	 unsigned char interface, unsigned char wakeup)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSEXCFGDV,
					 interface + 3, 1, wakeup);
	return;
}

static inline unsigned long get_ss_capfncwake(void __iomem *base_address,
	 unsigned char interface)
{
	return get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEXCFGDV,
						 interface + 3, 1);
}

static inline void set_ss_maxburst(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char burst)
{
	switch (endpoint_channel) {
	case 1:
	case 2:
		set_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEXCFGEP1 +
						 endpoint_channel - 1,
						 0, 4, burst);
		break;
	default:
		break;
	}

	return;
}

static inline unsigned char get_ss_maxburst(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	unsigned char burst;
	switch (endpoint_channel) {
	case 1:
	case 2:
		burst = (unsigned char) get_f_usb30_register_bits(
					base_address, F_USB30_REG_SSEXCFGEP1
					 + endpoint_channel - 1, 0, 4);
	default:
		break;
	}

	return burst;
}

static inline void set_ss_diseob(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char disable)
{
	switch (endpoint_channel) {
	case 1:
		set_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEXCFGEP1,
						 8, 1, disable);
		 break;
	case 5:
		set_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEXCFGEP5,
						 8, 1, disable);
		 break;
	default:
		break;
	}

	return;
}

static inline unsigned char get_ss_diseob(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	unsigned char disable;
	switch (endpoint_channel) {
	case 1:
		disable = (unsigned char) get_f_usb30_register_bits(
						base_address,
						 F_USB30_REG_SSEXCFGEP1,
						 8, 1);
		break;
	case 5:
		disable = (unsigned char) get_f_usb30_register_bits(
						base_address,
						 F_USB30_REG_SSEXCFGEP5,
						 8, 1);
		break;
	default:
		break;
	}
	return disable;
}

static inline void set_ss_numpmode(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned char mode)
{
	switch (endpoint_channel) {
	case 1:
		set_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEXCFGEP1,
						 16, 2, mode);
		break;
	case 2:
		set_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEXCFGEP2,
						 16, 2, mode);
		break;
	case 4:
		set_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEXCFGEP4,
						 16, 2, mode);
		break;
	default:
		break;
	}

	return;
}

static inline unsigned char get_ss_numpmode(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	unsigned char mode;
	switch (endpoint_channel) {
	case 1:
		mode = (unsigned char) get_f_usb30_register_bits(
						base_address,
						 F_USB30_REG_SSEXCFGEP1,
						 16, 2);
		break;
	case 2:
		mode = (unsigned char) get_f_usb30_register_bits(
						base_address,
						 F_USB30_REG_SSEXCFGEP2,
						 16, 2);
		break;
	case 4:
		mode = (unsigned char) get_f_usb30_register_bits(
						base_address,
						 F_USB30_REG_SSEXCFGEP4,
						 16, 2);
		break;
	default:
		break;
	}

	return mode;
}

static inline void set_ss_streamid(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned int id)
{
	switch (endpoint_channel) {
	case 1:
		set_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSBSC1,
						 0, 16, id);
		break;
	case 2:
		set_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSBSC2,
						 0, 16, id);
		break;
	case 5:
		set_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSBSC5,
						 0, 16, id);
		break;
	default:
		break;
	}

	return;
}

static inline unsigned int get_ss_streamid(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	unsigned int id;
	switch (endpoint_channel) {
	case 1:
		id = (unsigned int) get_f_usb30_register_bits(
					base_address, F_USB30_REG_SSBSC1,
					 0, 16);
		break;
	case 2:
		id = (unsigned int) get_f_usb30_register_bits(
					base_address, F_USB30_REG_SSBSC2,
					 0, 16);
		break;
	case 5:
		id = (unsigned int) get_f_usb30_register_bits(
					base_address, F_USB30_REG_SSBSC5,
					 0, 16);
		break;
	default:
		break;
	}

	return id;
}

static inline void set_ss_stcnt(void __iomem *base_address,
	 unsigned char endpoint_channel, unsigned long count)
{
	switch (endpoint_channel) {
	case 1:
		set_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPSTC1,
						 0, 32, count);
		break;
	case 2:
		set_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSEPSTC2,
						 0, 32, count);
		break;
	default:
		break;
	}

	return;
}

static inline unsigned long get_ss_stcnt(void __iomem *base_address,
	unsigned char endpoint_channel)
{
	unsigned long count;
	switch (endpoint_channel) {
	case 1:
		count = (unsigned long) get_f_usb30_register_bits(
						base_address,
						 F_USB30_REG_SSEPSTC1,
						 0, 32);
		break;
	case 2:
		count = (unsigned long) get_f_usb30_register_bits(
						base_address,
						 F_USB30_REG_SSEPSTC2,
						 0, 32);
		break;
	default:
		break;
	}

	return count;
}

static inline void clear_ss_cldllerrcnt(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSERCC, 0, 1, 0);
	return;
}

static inline unsigned char get_ss_cldllerrcnt(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(
				base_address, F_USB30_REG_SSERCC, 0, 1);
}

static inline void clear_ss_clphyerrcnt(void __iomem *base_address)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSERCC, 1, 1, 0);
	return;
}

static inline unsigned char get_ss_clphyerrcnt(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(
				base_address, F_USB30_REG_SSERCC, 1, 1);
}

static inline unsigned char get_ss_derc_hbad(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(
				base_address, F_USB30_REG_SSDlECNT1, 0, 8);
}

static inline unsigned char get_ss_derc_hrcv(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(
				base_address, F_USB30_REG_SSDlECNT1, 8, 8);
}

static inline unsigned char get_ss_derc_drcv(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(
				base_address, F_USB30_REG_SSDlECNT1, 16, 8);
}

static inline unsigned char get_ss_derc_ssia(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(
				base_address, F_USB30_REG_SSDlECNT1, 24, 8);
}

static inline unsigned char get_ss_derc_lgod(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(
				base_address, F_USB30_REG_SSDlECNT2, 0, 8);
}

static inline unsigned char get_ss_derc_crdt(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(
				base_address, F_USB30_REG_SSDlECNT2, 8, 8);
}

static inline unsigned char get_ss_derc_fotr(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(
				base_address, F_USB30_REG_SSDlECNT2, 16, 8);
}

static inline unsigned char get_ss_derc_frer(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(
				base_address, F_USB30_REG_SSDlECNT2, 24, 8);
}

static inline unsigned char get_ss_derc_pdhp(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(
				base_address, F_USB30_REG_SSDlECNT3, 0, 8);
}

static inline unsigned char get_ss_derc_cdhp(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(
				base_address, F_USB30_REG_SSDlECNT3, 8, 8);
}

static inline unsigned char get_ss_derc_pmlc(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(
				base_address, F_USB30_REG_SSDlECNT3, 16, 8);
}

static inline unsigned char get_ss_derc_pent(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(
				base_address, F_USB30_REG_SSDlECNT3, 24, 8);
}

static inline unsigned char get_ss_derc_ldtm(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(
				base_address, F_USB30_REG_SSDlECNT4, 0, 8);
}

static inline unsigned int get_ss_perc_8b10ber(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(
				base_address, F_USB30_REG_SSPlECNT, 0, 16);
}

static inline unsigned char get_ss_perc_ltsmto(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(
				base_address, F_USB30_REG_SSPlECNT, 16, 8);
}

static inline unsigned char get_ss_perc_elbufof(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(
				base_address, F_USB30_REG_SSPlECNT, 24, 8);
}

static inline void set_ss_enu1(void __iomem *base_address,
	 unsigned char enable)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSLNC, 0, 1, enable);
	return;
}

static inline unsigned char get_ss_enu1(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSLNC, 0, 1);
}

static inline void set_ss_enu2(void __iomem *base_address,
	 unsigned char enable)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSLNC, 1, 1, enable);
	return;
}

static inline unsigned char get_ss_enu2(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSLNC, 1, 1);
}

static inline void set_ss_frclpma(void __iomem *base_address,
	 unsigned char assert)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSLNC, 2, 1, assert);
	return;
}

static inline unsigned char get_ss_frclpma(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSLNC, 2, 1);
}

static inline void set_ss_enltm(void __iomem *base_address,
	 unsigned char enable)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSLNC, 3, 1, enable);
	return;
}

static inline unsigned char get_ss_enltm(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSLNC, 3, 1);
}

static inline void set_ss_u2inactto(void __iomem *base_address,
	 unsigned char timeout)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSLNC, 8, 8, timeout);
	return;
}

static inline unsigned char get_ss_u2inactto(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSLNC, 8, 8);
}

static inline void set_ss_reqpkt(void __iomem *base_address,
	 unsigned char request)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSTRC1, 0, 1, request);
	return;
}

static inline unsigned char get_ss_reqpkt(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSTRC1, 0, 1);
}

static inline void set_ss_pktcode(void __iomem *base_address,
	 unsigned char code)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSTRC1, 1, 2, code);
	return;
}

static inline unsigned char get_ss_pktcode(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSTRC1, 1, 2);
}

static inline void set_ss_vsdtest(void __iomem *base_address,
	 unsigned char data)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSTRC1, 8, 8, data);
	return;
}

static inline unsigned char get_ss_vsdtest(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSTRC1, 8, 8);
}

static inline void set_ss_tnump(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSTRC1, 16, 5, value);
	return;
}

static inline unsigned char get_ss_tnump(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSTRC1, 16, 5);
}

static inline void set_ss_tseqnum(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSTRC1, 24, 5, value);
	return;
}

static inline unsigned char get_ss_tseqnum(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSTRC1, 24, 5);
}

static inline void set_ss_tretry(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSTRC1, 29, 1, value);
	return;
}

static inline unsigned char get_ss_tretry(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSTRC1, 29, 1);
}

static inline void set_ss_beltdef_value(void __iomem *base_address,
	 unsigned int value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSTRC2, 0, 10, value);
	return;
}

static inline unsigned int get_ss_beltdef_value(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSTRC2, 0, 10);
}

static inline void set_ss_beltdef_scale(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSTRC2, 10, 2, value);
	return;
}

static inline unsigned char get_ss_beltdef_scale(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSTRC2, 10, 2);
}

static inline void set_ss_beltmin_value(void __iomem *base_address,
	 unsigned int value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSTRC2, 16, 10, value);
	return;
}

static inline unsigned int get_ss_beltmin_value(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSTRC2, 16, 10);
}

static inline void set_ss_beltmin_scale(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSTRC2, 26, 2, value);
	return;
}

static inline unsigned char get_ss_beltmin_scale(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSTRC2, 26, 2);
}

static inline unsigned char get_ss_pcfgto(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSTRS1, 8, 1);
}

static inline unsigned char get_ss_rcvvsdtest(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSTRS1, 24, 8);
}

static inline unsigned char get_ss_tnxtseq(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSTRS2, 0, 5);
}

static inline unsigned char get_ss_tackdseq(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSTRS2, 8, 5);
}

static inline unsigned char get_ss_tlimseq(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSTRS2, 16, 5);
}

static inline void set_ss_pdhp_tov(void __iomem *base_address,
	 unsigned int value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDlC1, 0, 12, value);
	return;
}

static inline unsigned int get_ss_pdhp_tov(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDlC1, 0, 12);
}

static inline void set_ss_cdhp_tov(void __iomem *base_address,
	 unsigned int value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDlC1, 16, 12, value);
	return;
}

static inline unsigned int get_ss_cdhp_tov(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDlC1, 16, 12);
}

static inline void set_ss_pmlc_tov(void __iomem *base_address,
	 unsigned int value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDlC2, 0, 12, value);
	return;
}

static inline unsigned int get_ss_pmlc_tov(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDlC2, 0, 12);
}

static inline void set_ss_pmet_tov(void __iomem *base_address,
	 unsigned int value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDlC2, 16, 12, value);
	return;
}

static inline unsigned int get_ss_pmet_tov(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDlC2, 16, 12);
}

static inline void set_ss_ldtm_tov(void __iomem *base_address,
	 unsigned int value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDlC3, 0, 12, value);
	return;
}

static inline unsigned int get_ss_ldtm_tov(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDlC3, 0, 12);
}

static inline void set_ss_ldtm_cntv(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDlC3, 16, 8, value);
	return;
}

static inline unsigned char get_ss_ldtm_cntv(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDlC3, 16, 8);
}

static inline void set_ss_luptim(void __iomem *base_address,
	 unsigned int value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDlC4, 0, 12, value);
	return;
}

static inline unsigned int get_ss_luptim(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDlC4, 0, 12);
}

static inline void set_ss_skptim(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSDlC4, 16, 8, value);
	return;
}

static inline unsigned char get_ss_ldtm_skptim(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSDlC4, 16, 8);
}

static inline void set_ss_cnt_u1inactto(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPMC1, 0, 8, value);
	return;
}

static inline unsigned char get_ss_cnt_u1inactto(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPMC1, 0, 8);
}

static inline void set_ss_cnt_u2inactto(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPMC1, 8, 8, value);
	return;
}

static inline unsigned char get_ss_cnt_u2inactto(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPMC1, 8, 8);
}

static inline void set_ss_blki_u1inactto(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPMC1, 16, 8, value);
	return;
}

static inline unsigned char get_ss_blki_u1inactto(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPMC1, 16, 8);
}

static inline void set_ss_blki_u2inactto(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPMC1, 24, 8, value);
	return;
}

static inline unsigned char get_ss_blki_u2inactto(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPMC1, 24, 8);
}

static inline void set_ss_blko_u1inactto(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPMC2, 0, 8, value);
	return;
}

static inline unsigned char get_ss_blko_u1inactto(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPMC2, 0, 8);
}

static inline void set_ss_blko_u2inactto(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPMC2, 8, 8, value);
	return;
}

static inline unsigned char get_ss_blko_u2inactto(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPMC2, 8, 8);
}

static inline void set_ss_inti_u1inactto(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPMC2, 16, 8, value);
	return;
}

static inline unsigned char get_ss_inti_u1inactto(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPMC2, 16, 8);
}

static inline void set_ss_inti_u2inactto(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPMC2, 24, 8, value);
	return;
}

static inline unsigned char get_ss_inti_u2inactto(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPMC2, 24, 8);
}

static inline void set_ss_u1tou2inactto(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPMC3, 0, 8, value);
	return;
}

static inline unsigned char get_ss_u1tou2inactto(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPMC3, 0, 8);
}

static inline void set_ss_exitto(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPMC3, 16, 8, value);
	return;
}

static inline unsigned char get_ss_exitto(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPMC3, 16, 8);
}

static inline void set_ss_suspto(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPMC3, 24, 8, value);
	return;
}

static inline unsigned char get_ss_suspto(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPMC3, 24, 8);
}

static inline void set_ss_lt_to(void __iomem *base_address,
	 unsigned int value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPMC4, 0, 16, value);
	return;
}

static inline unsigned int get_ss_lt_to(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPMC4, 0, 16);
}

static inline void set_ss_preu2_to(void __iomem *base_address,
	 unsigned int value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPMC4, 16, 8, value);
	return;
}

static inline unsigned int get_ss_preu2_to(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPMC4, 16, 8);
}

static inline void set_ss_rjct_rcvu1(void __iomem *base_address,
	 unsigned char enable)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPMC4, 28, 1, enable);
	return;
}

static inline unsigned char get_ss_rjct_rcvu1(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPMC4, 28, 1);
}

static inline void set_ss_rjct_rcvu2(void __iomem *base_address,
	 unsigned char enable)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPMC4, 29, 1, enable);
	return;
}

static inline unsigned char get_ss_rjct_rcvu2(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPMC4, 29, 1);
}

static inline void set_ss_tim_256us(void __iomem *base_address,
	 unsigned int value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPMC5, 0, 10, value);
	return;
}

static inline unsigned int get_ss_tim_256us(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPMC5, 0, 10);
}

static inline void set_ss_vddata_l(void __iomem *base_address,
	 unsigned long value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSVDTC1, 0, 32, value);
	return;
}

static inline unsigned long get_ss_vddata_l(void __iomem *base_address)
{
	return (unsigned long) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSVDTC1, 0, 32);
}

static inline void set_ss_vddata_h(void __iomem *base_address,
	 unsigned long value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSVDTC2, 0, 32, value);
	return;
}

static inline unsigned long get_ss_vddata_h(void __iomem *base_address)
{
	return (unsigned long) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSVDTC2, 0, 32);
}

static inline unsigned long get_ss_rcvvddata_l(void __iomem *base_address)
{
	return (unsigned long) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSVDTS1, 0, 32);
}

static inline unsigned long get_ss_rcvvddata_h(void __iomem *base_address)
{
	return (unsigned long) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSVDTS2, 0, 32);
}

/* PLL divided mode select */
#define SS_PLL_CLOCK_20MHZ		0	/* 20Mhz */
#define SS_PLL_CLOCK_40MHZ		1	/* 40Mhz */
#define SS_PLL_CLOCK_25MHZ		8	/* 25Mhz */

static inline void set_ss_plfvdcnt(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC1, 0, 4, value);
	return;
}

static inline unsigned char get_ss_plfvdcnt(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC1, 0, 4);
}

static inline void set_ss_sscon(void __iomem *base_address,
	 unsigned char enable)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC1, 8, 1, enable);
	return;
}

static inline unsigned char get_ss_sscon(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC1, 8, 1);
}

static inline void set_ss_ssccen(void __iomem *base_address,
	 unsigned char spread)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC1, 9, 1, spread);
	return;
}

static inline unsigned char get_ss_ssccen(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC1, 9, 1);
}

static inline void set_ss_txmargin(void __iomem *base_address,
	 unsigned char level)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC1, 16, 3, level);
	return;
}

static inline unsigned char get_ss_txmargin(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC1, 16, 3);
}

static inline void set_ss_txswing(void __iomem *base_address,
	 unsigned char level)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC1, 24, 1, level);
	return;
}

static inline unsigned char get_ss_txswing(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC1, 24, 1);
}

static inline void set_ss_pll_locktim(void __iomem *base_address,
	 unsigned char time)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC2, 0, 4, time);
	return;
}

static inline unsigned char get_ss_pll_locktim(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC2, 0, 4);
}

static inline void set_ss_cdr_locktim(void __iomem *base_address,
	 unsigned char time)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC2, 4, 4, time);
	return;
}

static inline unsigned char get_ss_cdr_locktim(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC2, 4, 4);
}

static inline void set_ss_pll_rstentim(void __iomem *base_address,
	 unsigned char time)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC2, 8, 4, time);
	return;
}

static inline unsigned char get_ss_pll_rstentim(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC2, 8, 4);
}

static inline void set_ss_polen_in(void __iomem *base_address,
	 unsigned char assert)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC2, 12, 1, assert);
	return;
}

static inline unsigned char get_ss_polen_in(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC2, 12, 1);
}

static inline void set_ss_rxden_in(void __iomem *base_address,
	 unsigned char assert)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC2, 13, 1, assert);
	return;
}

static inline unsigned char get_ss_rxden_in(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC2, 13, 1);
}

static inline void set_ss_sslfsel(void __iomem *base_address,
	 unsigned char sel)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC2, 14, 1, sel);
	return;
}

static inline unsigned char get_ss_sslfsel(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC2, 14, 1);
}

static inline void set_ss_msklpbslf(void __iomem *base_address,
	 unsigned char sel)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC2, 15, 1, sel);
	return;
}

static inline unsigned char get_ss_msklpbslf(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC2, 15, 1);
}

static inline void set_ss_pollfp_p1(void __iomem *base_address,
	 unsigned char sel)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC2, 20, 1, sel);
	return;
}

static inline unsigned char get_ss_pollfp_p1(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC2, 20, 1);
}

static inline void set_ss_comp4_p1(void __iomem *base_address,
	 unsigned char sel)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC2, 21, 1, sel);
	return;
}

static inline unsigned char get_ss_comp4_p1(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC2, 21, 1);
}

static inline void set_ss_txei_pls(void __iomem *base_address,
	 unsigned char assert)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC2, 22, 1, assert);
	return;
}

static inline unsigned char get_ss_txei_pls(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC2, 22, 1);
}

static inline void set_ss_prsu12_p0(void __iomem *base_address,
	 unsigned char assert)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC2, 23, 1, assert);
	return;
}

static inline unsigned char get_ss_prsu12_p0(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC2, 23, 1);
}

static inline void set_ss_tts_func(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC2, 24, 4, value);
	return;
}

static inline unsigned char get_ss_tts_func(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC2, 24, 4);
}

static inline void set_ss_dir_lpb_ext(void __iomem *base_address,
	 unsigned char sel)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC2, 28, 1, sel);
	return;
}

static inline unsigned char get_ss_dir_lpb_ext(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC2, 28, 1);
}

static inline void set_ss_tim_tseqtr(void __iomem *base_address,
	 unsigned int time)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC3, 0, 10, time);
	return;
}

static inline unsigned int get_ss_tim_tseqtr(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC3, 0, 10);
}

static inline void set_ss_txopcnt0(void __iomem *base_address,
	 unsigned char code)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC3, 12, 4, code);
	return;
}

static inline unsigned char get_ss_txopcnt0(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC3, 12, 4);
}

static inline void set_ss_txodcnt0(void __iomem *base_address,
	 unsigned char code)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC3, 24, 4, code);
	return;
}

static inline unsigned char get_ss_txodcnt0(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC3, 24, 4);
}

static inline void set_ss_txdeemph(void __iomem *base_address,
	 unsigned char sel)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC3, 31, 1, sel);
	return;
}

static inline unsigned char get_ss_txdeemph(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC3, 31, 1);
}

static inline void set_ss_tim_2ms(void __iomem *base_address,
	 unsigned int time)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC4, 0, 10, time);
	return;
}

static inline unsigned int get_ss_tim_2ms(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC4, 0, 10);
}

static inline void set_ss_tim_6ms(void __iomem *base_address,
	 unsigned int time)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC4, 16, 10, time);
	return;
}

static inline unsigned int get_ss_tim_6ms(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC4, 16, 10);
}

static inline void set_ss_tim_10ms(void __iomem *base_address,
	 unsigned int time)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC5, 0, 10, time);
	return;
}

static inline unsigned int get_ss_tim_10ms(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC5, 0, 10);
}

static inline void set_ss_tim_12ms(void __iomem *base_address,
	 unsigned int time)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC5, 16, 10, time);
	return;
}

static inline unsigned int get_ss_tim_12ms(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC5, 16, 10);
}

static inline void set_ss_tim_100ms(void __iomem *base_address,
	 unsigned int time)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC6, 0, 10, time);
	return;
}

static inline unsigned int get_ss_tim_100ms(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC6, 0, 10);
}

static inline void set_ss_tim_300ms(void __iomem *base_address,
	 unsigned int time)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC6, 16, 10, time);
	return;
}

static inline unsigned int get_ss_tim_300ms(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC6, 16, 10);
}

static inline void set_ss_tim_360ms(void __iomem *base_address,
	 unsigned int time)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC7, 0, 10, time);
	return;
}

static inline unsigned int get_ss_tim_360ms(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC7, 0, 10);
}

static inline void set_ss_ux_ent_tim(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC7, 16, 5, value);
	return;
}

static inline unsigned char get_ss_ux_ent_tim(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC7, 16, 5);
}

static inline void set_ss_polf_rmin_tim(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC8, 0, 4, value);
	return;
}

static inline unsigned char get_ss_polf_rmin_tim(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC8, 0, 4);
}

static inline void set_ss_polf_rmax_tim(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC8, 8, 6, value);
	return;
}

static inline unsigned char get_ss_polf_rmax_tim(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC8, 8, 6);
}

static inline void set_ss_polf_bmin_tim(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC8, 16, 3, value);
	return;
}

static inline unsigned char get_ss_polf_bmin_tim(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC8, 16, 3);
}

static inline void set_ss_polf_bmax_tim(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC8, 24, 5, value);
	return;
}

static inline unsigned char get_ss_polf_bmax_tim(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC8, 24, 5);
}

static inline void set_ss_u1lftr_tim(void __iomem *base_address,
	 unsigned int time)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC9, 0, 15, time);
	return;
}

static inline unsigned int get_ss_u1lftr_tim(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC9, 0, 15);
}

static inline void set_ss_tim_wrstdet(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC9, 16, 7, value);
	return;
}

static inline unsigned char get_ss_tim_wrstdet(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC9, 16, 7);
}

static inline void set_ss_tim_lfpsdet(void __iomem *base_address,
	 unsigned char value)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC9, 24, 4, value);
	return;
}

static inline unsigned char get_ss_tim_lfpsdet(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC9, 24, 4);
}

static inline void set_ss_u3lftr_tim(void __iomem *base_address,
	 unsigned int time)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC10, 0, 10, time);
	return;
}

static inline unsigned int get_ss_u3lftr_tim(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC10, 0, 10);
}

static inline void set_ss_u2lftr_tim(void __iomem *base_address,
	 unsigned int time)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC10, 16, 10, time);
	return;
}

static inline unsigned int get_ss_u2lftr_tim(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						F_USB30_REG_SSPlC10, 16, 10);
}

static inline void set_ss_rxeqtre_tim(void __iomem *base_address,
	 unsigned int time)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC11, 0, 10, time);
	return;
}

static inline unsigned int get_ss_rxeqtre_tim(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC11, 0, 10);
}

static inline void set_ss_rxeqtrs_tim(void __iomem *base_address,
	 unsigned int time)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC11, 16, 10, time);
	return;
}

static inline unsigned int get_ss_rxeqtrs_tim(void __iomem *base_address)
{
	return (unsigned int) get_f_usb30_register_bits(base_address,
						F_USB30_REG_SSPlC11, 16, 10);
}

static inline void set_ss_polfeitr_tim(void __iomem *base_address,
	 unsigned char time)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC12, 0, 7, time);
	return;
}

static inline unsigned char get_ss_polfeitr_tim(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC12, 0, 7);
}

static inline void set_ss_polfbutr_tim(void __iomem *base_address,
	 unsigned char time)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC12, 8, 5, time);
	return;
}

static inline unsigned char get_ss_polfbutr_tim(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC12, 8, 5);
}

static inline void set_ss_pnglf_min_tim(void __iomem *base_address,
	 unsigned char time)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC12, 16, 7, time);
	return;
}

static inline unsigned char get_ss_pnglf_min_tim(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC12, 16, 7);
}

static inline void set_ss_pnglf_max_tim(void __iomem *base_address,
	 unsigned char time)
{
	set_f_usb30_register_bits(base_address,
					 F_USB30_REG_SSPlC12, 24, 7, time);
	return;
}

static inline unsigned char get_ss_pnglf_max_tim(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlC12, 24, 7);
}

static inline unsigned char get_ss_ltssm(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlS, 0, 6);
}

static inline unsigned char get_ss_rts_func(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlS, 8, 4);
}

static inline unsigned char get_ss_ltssm_to_st(void __iomem *base_address)
{
	return (unsigned char) get_f_usb30_register_bits(base_address,
						 F_USB30_REG_SSPlS, 16, 6);
}

#if defined(CONFIG_USB_GADGET_F_USB30_USED_DMA_TRANSFER)
/* F_USB30 DMA controller register ID Enumeration constant */
#define F_USB30_DMAC_REG_AXICFG1	0 /* DMA1 I/F AXI configuration */
#define F_USB30_DMAC_REG_AXIADR1	1
				/* DMA1 I/F AXI Master Read Start Address */
#define F_USB30_DMAC_REG_DMADS1		2 /* DMA1 I/F Data Size */
#define F_USB30_DMAC_REG_DMAC1		3 /* DMA1 I/F Control   */
#define F_USB30_DMAC_REG_DMAS1		4 /* DMA1 I/F Status    */
#define F_USB30_DMAC_REG_AXIDRC1	5
				/* AXI Master Read Data Receive Count   */
#define F_USB30_DMAC_REG_USBDTC1	6 /* USB Data In Tranfer Count  */
#define F_USB30_DMAC_REG_AXICFG2	7 /* DMA2 I/F AXI configuration */
#define F_USB30_DMAC_REG_AXIADR2	8
				/* DMA2 I/F AXI Master Write Start Address */
#define F_USB30_DMAC_REG_DMADS2		9 /* DMA2 I/F Data Size */
#define F_USB30_DMAC_REG_DMAC2		10 /* DMA2 I/F Control  */
#define F_USB30_DMAC_REG_DMAS2		11 /* DMA2 I/F Status   */
#define F_USB30_DMAC_REG_USBDRC2	12 /* USB Data Out Receive Count */
#define F_USB30_DMAC_REG_AXIDTC2	13
				/* AXI Master Write Data Transfer Count */
#define F_USB30_DMAC_REG_MAX		14 /* Max Value */

/* I/F AXI configuration register ID table array */
static const unsigned long axicfg_register[] = {
	F_USB30_DMAC_REG_AXICFG1,	/* DMA1 I/F AXI configuration */
	F_USB30_DMAC_REG_AXICFG2,	/* DMA2 I/F AXI configuration */
};

/* I/F AXI Master Read/write Start Address register ID table array */
static const unsigned long axiar_register[] = {
	F_USB30_DMAC_REG_AXIADR1,	/*
					 * DMA1 I/F AXI Master Read
					 * Start Address
					 */
	F_USB30_DMAC_REG_AXIADR2,	/*
					 * DMA2 I/F AXI Master Write
					 * Start Address
					 */
};

/* I/F Data Size register ID table array */
static const unsigned long dmads_register[] = {
	F_USB30_DMAC_REG_DMADS1,	/* DMA1 I/F Data Size */
	F_USB30_DMAC_REG_DMADS2,	/* DMA2 I/F Data Size */
};

/* I/F Control register ID table array */
static const unsigned long dmac_register[] = {
	F_USB30_DMAC_REG_DMAC1,		/* DMA1 I/F Control */
	F_USB30_DMAC_REG_DMAC2,		/* DMA2 I/F Control */
};

/* I/F Status register ID table array */
static const unsigned long dmas_register[] = {
	F_USB30_DMAC_REG_DMAS1,		/* DMA1 I/F Status */
	F_USB30_DMAC_REG_DMAS2,		/* DMA2 I/F Status */
};

/* AXI Master Read/write Data Receive Count register ID table array */
static const unsigned long axidrc_register[] = {
	F_USB30_DMAC_REG_AXIDRC1,	/* DMA1 I/F AXI Transferred Data Size */
	F_USB30_DMAC_REG_AXIDTC2,	/* DMA2 I/F AXI Received Data Size */
};

/* USB Data In/Out Tranfer Count register ID table array */
static const unsigned long usbdtc_register[] = {
	F_USB30_DMAC_REG_USBDTC1,	/* DMA1 USB Buffer Received Data Size */
	F_USB30_DMAC_REG_USBDRC2,	/*
					 * DMA2 USB Buffer Transferred
					 * Data Size
					 */
};

/* F_USB30 DMA controller register structures array */
static const struct {
	unsigned long address_offset;	/* register address offset */
	unsigned char readable;	/* register readable flag */
	unsigned char writable;	/* register writable flag */
} f_usb30_dma_register[F_USB30_DMAC_REG_MAX] = {
	{
	(unsigned long)
	(F_USB30_DMAC_REG_OFFSET + 0x0000), 1, 1 },
					/* F_USB30_DMAC_REG_AXICFG1 */
	{
	(unsigned long)
	(F_USB30_DMAC_REG_OFFSET + 0x0004), 1, 1 },
					/* F_USB30_DMAC_REG_AXIADR1 */
	{
	(unsigned long)
	(F_USB30_DMAC_REG_OFFSET + 0x0008), 1, 1 },
					/* F_USB30_DMAC_REG_DMADS1 */
	{
	(unsigned long)
	(F_USB30_DMAC_REG_OFFSET + 0x000c), 1, 1 },
					/* F_USB30_DMAC_REG_DMAC1 */
	{
	(unsigned long)
	(F_USB30_DMAC_REG_OFFSET + 0x0010), 1, 1 },
					/* F_USB30_DMAC_REG_DMAS1 */
	{
	(unsigned long)
	(F_USB30_DMAC_REG_OFFSET + 0x0014), 1, 1 },
					/* F_USB30_DMAC_REG_AXIDRC1 */
	{
	(unsigned long)
	(F_USB30_DMAC_REG_OFFSET + 0x0018), 1, 1 },
					/* F_USB30_DMAC_REG_USBDTC1 */
	{
	(unsigned long)
	(F_USB30_DMAC_REG_OFFSET + 0x0100), 1, 1 },
					/* F_USB30_DMAC_REG_AXICFG2 */
	{
	(unsigned long)
	(F_USB30_DMAC_REG_OFFSET + 0x0104), 1, 1 },
					/* F_USB30_DMAC_REG_AXIADR2 */
	{
	(unsigned long)
	(F_USB30_DMAC_REG_OFFSET + 0x0108), 1, 1 },
					/* F_USB30_DMAC_REG_DMADS2 */
	{
	(unsigned long)
	(F_USB30_DMAC_REG_OFFSET + 0x010c), 1, 1 },
					/* F_USB30_DMAC_REG_DMAC2 */
	{
	(unsigned long)
	(F_USB30_DMAC_REG_OFFSET + 0x0110), 1, 1 },
					/* F_USB30_DMAC_REG_DMAS2 */
	{
	(unsigned long)
	(F_USB30_DMAC_REG_OFFSET + 0x0114), 1, 1 },
					/* F_USB30_DMAC_REG_USBDRC2 */
	{
	(unsigned long)
	(F_USB30_DMAC_REG_OFFSET + 0x0118), 1, 1 },
					/* F_USB30_DMAC_REG_AXIDTC2 */
};

static inline void control_f_usb30_dmac_default_register_cache_bits(
	unsigned long *register_cache)
{
	return;
}

static inline void control_f_usb30_dmac_dmas1_register_cache_bits(
	unsigned long *register_cache)
{
	/* I/F Status register bit feild position */
#define F_USB30_DMAC_REG_DMAS_BIT_INTREMAINFIN1		16
#define F_USB30_DMAC_REG_DMAS_BIT_INTUSBFIN1		17
#define F_USB30_DMAC_REG_DMAS_BIT_INTNULL1		18
#define F_USB30_DMAC_REG_DMAS_BIT_INTUEXDREQ1		19
#define F_USB30_DMAC_REG_DMAS_BIT_INTDECERR1		24
#define F_USB30_DMAC_REG_DMAS_BIT_INTSLVERR1		25
#define F_USB30_DMAC_REG_DMAS_BIT_INTEXOKAY1		26
#define F_USB30_DMAC_REG_DMAS_BIT_INTAXIFIN1		27
#define F_USB30_DMAC_REG_DMAS_BIT_INTSTARTERR1		28

	*register_cache |=
	    (((unsigned long) 1 << F_USB30_DMAC_REG_DMAS_BIT_INTREMAINFIN1) |
	     ((unsigned long) 1 << F_USB30_DMAC_REG_DMAS_BIT_INTUSBFIN1) |
	     ((unsigned long) 1 << F_USB30_DMAC_REG_DMAS_BIT_INTNULL1) |
	     ((unsigned long) 1 << F_USB30_DMAC_REG_DMAS_BIT_INTUEXDREQ1) |
	     ((unsigned long) 1 << F_USB30_DMAC_REG_DMAS_BIT_INTDECERR1) |
	     ((unsigned long) 1 << F_USB30_DMAC_REG_DMAS_BIT_INTSLVERR1) |
	     ((unsigned long) 1 << F_USB30_DMAC_REG_DMAS_BIT_INTEXOKAY1) |
	     ((unsigned long) 1 << F_USB30_DMAC_REG_DMAS_BIT_INTAXIFIN1) |
	     ((unsigned long) 1 << F_USB30_DMAC_REG_DMAS_BIT_INTSTARTERR1));
	return;
}

static inline void control_f_usb30_dmac_dmas2_register_cache_bits(
	unsigned long *register_cache)
{
	/* I/F Status register bit feild position */
#define F_USB30_DMAC_REG_DMAS_BIT_INTREMAINFIN2		16
#define F_USB30_DMAC_REG_DMAS_BIT_INTUSBFIN2		17
#define F_USB30_DMAC_REG_DMAS_BIT_INTNULL2		18
#define F_USB30_DMAC_REG_DMAS_BIT_INTUEXDREQ2		19
#define F_USB30_DMAC_REG_DMAS_BIT_INTUSBUNDER2		20
#define F_USB30_DMAC_REG_DMAS_BIT_INTUSBOVER2		21
#define F_USB30_DMAC_REG_DMAS_BIT_INTDECERR2		24
#define F_USB30_DMAC_REG_DMAS_BIT_INTSLVERR2		25
#define F_USB30_DMAC_REG_DMAS_BIT_INTEXOKAY2		26
#define F_USB30_DMAC_REG_DMAS_BIT_INTAXIFIN2		27
#define F_USB30_DMAC_REG_DMAS_BIT_INTSTARTERR2		28

	*register_cache |=
	    (((unsigned long) 1 << F_USB30_DMAC_REG_DMAS_BIT_INTREMAINFIN2) |
	     ((unsigned long) 1 << F_USB30_DMAC_REG_DMAS_BIT_INTUSBFIN2) |
	     ((unsigned long) 1 << F_USB30_DMAC_REG_DMAS_BIT_INTNULL2) |
	     ((unsigned long) 1 << F_USB30_DMAC_REG_DMAS_BIT_INTUEXDREQ2) |
	     ((unsigned long) 1 << F_USB30_DMAC_REG_DMAS_BIT_INTUSBUNDER2) |
	     ((unsigned long) 1 << F_USB30_DMAC_REG_DMAS_BIT_INTUSBOVER2) |
	     ((unsigned long) 1 << F_USB30_DMAC_REG_DMAS_BIT_INTDECERR2) |
	     ((unsigned long) 1 << F_USB30_DMAC_REG_DMAS_BIT_INTSLVERR2) |
	     ((unsigned long) 1 << F_USB30_DMAC_REG_DMAS_BIT_INTEXOKAY2) |
	     ((unsigned long) 1 << F_USB30_DMAC_REG_DMAS_BIT_INTAXIFIN2) |
	     ((unsigned long) 1 << F_USB30_DMAC_REG_DMAS_BIT_INTSTARTERR2));
	return;
}

static inline void control_f_usb30_dmac_register_cache_bits(
	unsigned long register_id, unsigned long *register_cache)
{
	static const unsigned char register_cache_bits_control[] = {
		0,	/* DMA1 I/F AXI configuration */
		0,	/* DMA1 I/F AXI Master Read Start Address */
		0,	/* DMA1 I/F Data Size */
		0,	/* DMA1 I/F Control   */
		1,	/* DMA1 I/F Status    */
		0,	/* AXI Master Read Data Receive Count   */
		0,	/* USB Data In Tranfer Count  */
		0,	/* DMA2 I/F AXI configuration */
		0,	/* DMA2 I/F AXI Master Write Start Address */
		0,	/* DMA2 I/F Data Size */
		0,	/* DMA2 I/F Control  */
		2,	/* DMA2 I/F Status   */
		0,	/* USB Data Out Receive Count */
		0,	/* AXI Master Write Data Transfer Count */
	};

	static void (*const register_cache_bits_control_function[])
		(unsigned long *) = {
		control_f_usb30_dmac_default_register_cache_bits,
		control_f_usb30_dmac_dmas1_register_cache_bits,
		control_f_usb30_dmac_dmas2_register_cache_bits,};

	register_cache_bits_control_function[
		register_cache_bits_control[register_id]] (register_cache);
	return;
}

static inline void set_f_usb30_dmac_register_bits(
	void __iomem *base_address, unsigned long register_id,
	 unsigned char start_bit, unsigned char bit_length,
	 unsigned long value)
{
	unsigned long register_cache;
	unsigned long mask = (unsigned long) -1 >> (32 - bit_length);

	value &= mask;

	if (f_usb30_dma_register[register_id].readable)
		register_cache = __raw_readl(base_address +
			 f_usb30_dma_register[register_id].address_offset);

	control_f_usb30_dmac_register_cache_bits(register_id,
							 &register_cache);

	register_cache &= ~(mask << start_bit);
	register_cache |= (value << start_bit);

	if (f_usb30_dma_register[register_id].writable)
		__raw_writel(register_cache, base_address +
			 f_usb30_dma_register[register_id].address_offset);

	return;
}

static inline unsigned long get_f_usb30_dmac_register_bits(
	void __iomem *base_address, unsigned long register_id,
	 unsigned char start_bit, unsigned char bit_length)
{
	unsigned long register_cache;
	unsigned long mask = (unsigned long) -1 >> (32 - bit_length);

	if (f_usb30_dma_register[register_id].readable)
		register_cache = __raw_readl(base_address +
			 f_usb30_dma_register[register_id].address_offset);

	return register_cache >> start_bit & mask;
}

/* DMAC burst type select */
#define DMAC_BURST_FIXED		0	/* fixed burst */
#define DMAC_BURST_INCREMENT		1	/* increment burst */
#define DMAC_BURST_WRAPPING		2	/* wrapping burst */

static inline void set_bursttype(void __iomem *base_address,
	 unsigned char dma_channel, unsigned char type)
{
	set_f_usb30_dmac_register_bits(base_address,
		 axicfg_register[dma_channel], 0, 2, type);
	return;
}

/* DMAC max burst size select */
#define DMAC_MAX_BURST_4		3	/* 4 burst */
#define DMAC_MAX_BURST_8		7	/* 8 burst */
#define DMAC_MAX_BURST_16		15	/* 16 burst */

static inline void set_maxlen(void __iomem *base_address,
	 unsigned char dma_channel, unsigned char length)
{
	set_f_usb30_dmac_register_bits(base_address,
				 axicfg_register[dma_channel], 8, 4, length);
	return;
}

static inline void set_issue(void __iomem *base_address,
	 unsigned char dma_channel, unsigned char sel)
{
	set_f_usb30_dmac_register_bits(base_address,
				 axicfg_register[dma_channel], 24, 1, sel);
	return;
}

static inline void set_axiaddress(void __iomem *base_address,
	 unsigned char dma_channel, unsigned long address)
{
	set_f_usb30_dmac_register_bits(base_address,
				 axiar_register[dma_channel], 0, 32, address);
	return;
}

static inline void set_dmadatasize(void __iomem *base_address,
	 unsigned char dma_channel, unsigned long size)
{
	set_f_usb30_dmac_register_bits(base_address,
				 dmads_register[dma_channel], 0, 32, size);
	return;
}

static inline void set_dmastart(void __iomem *base_address,
	 unsigned char dma_channel, unsigned char enable)
{
	set_f_usb30_dmac_register_bits(base_address,
				 dmac_register[dma_channel], 0, 1, enable);
	return;
}

static inline void set_dmaabort(void __iomem *base_address,
	 unsigned char dma_channel, unsigned char enable)
{
	set_f_usb30_dmac_register_bits(base_address,
				 dmac_register[dma_channel], 1, 1, enable);
	return;
}

static inline void set_dmainit(void __iomem *base_address,
	 unsigned char dma_channel, unsigned char enable)
{
	set_f_usb30_dmac_register_bits(base_address,
				 dmac_register[dma_channel], 2, 1, enable);
	return;
}

static inline void set_abortcntl(void __iomem *base_address,
	 unsigned char dma_channel, unsigned char sel)
{
	set_f_usb30_dmac_register_bits(base_address,
				 dmac_register[dma_channel], 9, 1, sel);
	return;
}

static inline void set_nullcntl(void __iomem *base_address,
	 unsigned char dma_channel, unsigned char sel)
{
	if (dma_channel == F_USB30_IN_DMAC)
		set_f_usb30_dmac_register_bits(base_address,
					dmac_register[dma_channel], 10, 1, sel);
	return;
}

static inline void set_decerrcntl(void __iomem *base_address,
	 unsigned char dma_channel, unsigned char sel)
{
	set_f_usb30_dmac_register_bits(base_address,
				 dmac_register[dma_channel], 14, 1, sel);
	return;
}

static inline void set_slverrcntl(void __iomem *base_address,
	 unsigned char dma_channel, unsigned char sel)
{
	set_f_usb30_dmac_register_bits(base_address,
				 dmac_register[dma_channel], 15, 1, sel);
	return;
}

static inline void set_mskintremainfin(void __iomem *base_address,
	 unsigned char dma_channel, unsigned char mask)
{
	set_f_usb30_dmac_register_bits(base_address,
				 dmac_register[dma_channel], 16, 1, mask);
	return;
}

static inline unsigned char get_mskintremainfin(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmac_register[dma_channel], 16, 1);
}

static inline void set_mskintusbfin(void __iomem *base_address,
	 unsigned char dma_channel, unsigned char mask)
{
	set_f_usb30_dmac_register_bits(base_address,
				 dmac_register[dma_channel], 17, 1, mask);
	return;
}

static inline unsigned char get_mskintusbfin(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmac_register[dma_channel], 17, 1);
}

static inline void set_mskintnull(void __iomem *base_address,
	 unsigned char dma_channel, unsigned char mask)
{
	set_f_usb30_dmac_register_bits(base_address,
				 dmac_register[dma_channel], 18, 1, mask);
	return;
}

static inline unsigned char get_mskintnull(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmac_register[dma_channel], 18, 1);
}

static inline void set_mskintuexdreq(void __iomem *base_address,
	 unsigned char dma_channel, unsigned char mask)
{
	set_f_usb30_dmac_register_bits(base_address,
				 dmac_register[dma_channel], 19, 1, mask);
	return;
}

static inline unsigned char get_mskintuexdreq(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmac_register[dma_channel], 19, 1);
}

static inline void set_mskintusbunder(void __iomem *base_address,
	 unsigned char dma_channel, unsigned char mask)
{
	if (dma_channel == F_USB30_OUT_DMAC)
		set_f_usb30_dmac_register_bits(base_address,
				 dmac_register[dma_channel], 20, 1, mask);
	return;
}

static inline unsigned char get_mskintusbunder(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmac_register[dma_channel], 20, 1);
}

static inline void set_mskintusbover(void __iomem *base_address,
	 unsigned char dma_channel, unsigned char mask)
{
	if (dma_channel == F_USB30_OUT_DMAC)
		set_f_usb30_dmac_register_bits(base_address,
				 dmac_register[dma_channel], 21, 1, mask);
	return;
}

static inline unsigned char get_mskintusbover(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(
		base_address, dmac_register[dma_channel], 21, 1);
}

static inline void set_mskintdecerr(void __iomem *base_address,
	 unsigned char dma_channel, unsigned char mask)
{
	set_f_usb30_dmac_register_bits(base_address,
				 dmac_register[dma_channel], 24, 1, mask);
	return;
}

static inline unsigned char get_mskintdecerr(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmac_register[dma_channel], 24, 1);
}

static inline void set_mskintslverr(void __iomem *base_address,
	 unsigned char dma_channel, unsigned char mask)
{
	set_f_usb30_dmac_register_bits(base_address,
				 dmac_register[dma_channel], 25, 1, mask);
	return;
}

static inline unsigned char get_mskintslverr(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmac_register[dma_channel], 25, 1);
}

static inline void set_mskintexokay(void __iomem *base_address,
	 unsigned char dma_channel, unsigned char mask)
{
	set_f_usb30_dmac_register_bits(base_address,
				 dmac_register[dma_channel], 26, 1, mask);
	return;
}

static inline unsigned char get_mskintexokay(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmac_register[dma_channel], 26, 1);
}

static inline void set_mskintaxifin(void __iomem *base_address,
	 unsigned char dma_channel, unsigned char mask)
{
	set_f_usb30_dmac_register_bits(base_address,
				 dmac_register[dma_channel], 27, 1, mask);
	return;
}

static inline unsigned char get_mskintaxifin(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmac_register[dma_channel], 27, 1);
}

static inline void set_mskintstarterr(void __iomem *base_address,
	 unsigned char dma_channel, unsigned char mask)
{
	set_f_usb30_dmac_register_bits(base_address,
				 dmac_register[dma_channel], 28, 1, mask);
	return;
}

static inline unsigned char get_mskintstarterr(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmac_register[dma_channel], 28, 1);
}

static inline unsigned char get_dmaactive(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 0, 1);
}

static inline unsigned char get_abortactive(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 1, 1);
}

static inline unsigned char get_axiactive(void __iomem *base_address,
	 unsigned char dma_channel)
{
	if (dma_channel == F_USB30_OUT_DMAC)
		return 0;

	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 4, 1);
}

static inline unsigned char get_usbactive(void __iomem *base_address,
	 unsigned char dma_channel)
{
	if (dma_channel == F_USB30_IN_DMAC)
		return 0;

	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 3, 1);
}

static inline void clear_intremainfin(void __iomem *base_address,
	 unsigned char dma_channel)
{
	set_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 16, 1, 0);
	return;
}

static inline unsigned char get_intremainfin(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 16, 1);
}

static inline void clear_intusbfin(void __iomem *base_address,
	 unsigned char dma_channel)
{
	set_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 17, 1, 0);
	return;
}

static inline unsigned char get_intusbfin(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 17, 1);
}

static inline void clear_intnull(void __iomem *base_address,
	 unsigned char dma_channel)
{
	set_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 18, 1, 0);
	return;
}

static inline unsigned char get_intnull(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 18, 1);
}

static inline void clear_intuexdreq(void __iomem *base_address,
	 unsigned char dma_channel)
{
	set_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 19, 1, 0);
	return;
}

static inline unsigned char get_intuexdreq(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 19, 1);
}

static inline void clear_intusbunder(void __iomem *base_address,
	 unsigned char dma_channel)
{
	if (dma_channel == F_USB30_OUT_DMAC)
		set_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 20, 1, 0);
	return;
}

static inline unsigned char get_intusbunder(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 20, 1);
}

static inline void clear_intusbover(void __iomem *base_address,
	 unsigned char dma_channel)
{
	if (dma_channel == F_USB30_OUT_DMAC)
		set_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 21, 1, 0);
	return;
}

static inline unsigned char get_intusbover(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 21, 1);
}

static inline void clear_intdecerr(void __iomem *base_address,
	 unsigned char dma_channel)
{
	set_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 24, 1, 0);
	return;
}

static inline unsigned char get_intdecerr(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 24, 1);
}

static inline void clear_intslverr(void __iomem *base_address,
	 unsigned char dma_channel)
{
	set_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 25, 1, 0);
	return;
}

static inline unsigned char get_intslverr(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 25, 1);
}

static inline void clear_intexokay(void __iomem *base_address,
	 unsigned char dma_channel)
{
	set_f_usb30_dmac_register_bits(base_address,
					dmas_register[dma_channel], 26, 1, 0);
	return;
}

static inline unsigned char get_intexokay(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 26, 1);
}

static inline void clear_intaxifin(void __iomem *base_address,
	 unsigned char dma_channel)
{
	set_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 27, 1, 0);
	return;
}

static inline unsigned char get_intaxifin(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 27, 1);
}

static inline void clear_intstarterr(void __iomem *base_address,
	 unsigned char dma_channel)
{
	set_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 28, 1, 0);
	return;
}

static inline unsigned char get_intstarterr(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned char) get_f_usb30_dmac_register_bits(base_address,
					 dmas_register[dma_channel], 28, 1);
}

static inline unsigned long get_axicount(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned long) get_f_usb30_dmac_register_bits(base_address,
					 axidrc_register[dma_channel], 0, 32);
}

static inline unsigned long get_usbcount(void __iomem *base_address,
	 unsigned char dma_channel)
{
	return (unsigned long) get_f_usb30_dmac_register_bits(base_address,
					 usbdtc_register[dma_channel], 0, 32);
}
#endif

static inline void __iomem *remap_iomem_region(unsigned long base_address,
	unsigned long size)
{
	return (void __iomem *) ioremap(base_address, size);
}

static inline void unmap_iomem_region(void __iomem *base_address)
{
	iounmap(base_address);
	return;
}

#endif
