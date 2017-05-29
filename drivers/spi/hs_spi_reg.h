/*
 * linux/drivers/spi/hs_spi.h Register definitions for high speed SPI controller
 *
 * Copyright (C) 2010-2012 FUJITSU SEMICONDUCTOR LIMITED
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __HS_SPI_REG_H__
#define __HS_SPI_REG_H__

/*
 * HS_SPI register adress definitions
 */
#define	HS_SPI_REG_MCTRL	0x00
#define	HS_SPI_REG_PCC0		0x04
#define	HS_SPI_REG_PCC1		0x08
#define	HS_SPI_REG_PCC2		0x0C
#define	HS_SPI_REG_PCC3		0x10
#define	HS_SPI_REG_TXF		0x14
#define	HS_SPI_REG_TXE		0x18
#define	HS_SPI_REG_TXC		0x1C
#define	HS_SPI_REG_RXF		0x20
#define	HS_SPI_REG_RXE		0x24
#define	HS_SPI_REG_RXC		0x28
#define	HS_SPI_REG_FAULTF	0x2C
#define	HS_SPI_REG_FAULTC	0x30
#define	HS_SPI_REG_DMCFG	0x34
#define	HS_SPI_REG_DMDMAEN	0x34
#define	HS_SPI_REG_DMSTART	0x38
#define	HS_SPI_REG_DMSTOP	0x38
#define	HS_SPI_REG_DMPSEL	0x38
#define	HS_SPI_REG_DMTRP	0x38
#define	HS_SPI_REG_DMBCC	0x3C
#define	HS_SPI_REG_DMBCS	0x3C
#define	HS_SPI_REG_DMSTATUS	0x40
#define	HS_SPI_REG_TXBITCNT	0x44
#define	HS_SPI_REG_FIFOCFG	0x4C
#define	HS_SPI_REG_TXFIFO0	0x50
#define	HS_SPI_REG_TXFIFO1	0x54
#define	HS_SPI_REG_TXFIFO2	0x58
#define	HS_SPI_REG_TXFIFO3	0x5C
#define	HS_SPI_REG_TXFIFO4	0x60
#define	HS_SPI_REG_TXFIFO5	0x64
#define	HS_SPI_REG_TXFIFO6	0x68
#define	HS_SPI_REG_TXFIFO7	0x6C
#define	HS_SPI_REG_TXFIFO8	0x70
#define	HS_SPI_REG_TXFIFO9	0x74
#define	HS_SPI_REG_TXFIFO10	0x78
#define	HS_SPI_REG_TXFIFO11	0x7C
#define	HS_SPI_REG_TXFIFO12	0x80
#define	HS_SPI_REG_TXFIFO13	0x84
#define	HS_SPI_REG_TXFIFO14	0x88
#define	HS_SPI_REG_TXFIFO15	0x8C
#define	HS_SPI_REG_RXFIFO0	0x90
#define	HS_SPI_REG_RXFIFO1	0x94
#define	HS_SPI_REG_RXFIFO2	0x98
#define	HS_SPI_REG_RXFIFO3	0x9C
#define	HS_SPI_REG_RXFIFO4	0xA0
#define	HS_SPI_REG_RXFIFO5	0xA4
#define	HS_SPI_REG_RXFIFO6	0xA8
#define	HS_SPI_REG_RXFIFO7	0xAC
#define	HS_SPI_REG_RXFIFO8	0xB0
#define	HS_SPI_REG_RXFIFO9	0xB4
#define	HS_SPI_REG_RXFIFO10	0xB8
#define	HS_SPI_REG_RXFIFO11	0xBC
#define	HS_SPI_REG_RXFIFO12	0xC0
#define	HS_SPI_REG_RXFIFO13	0xC4
#define	HS_SPI_REG_RXFIFO14	0xC8
#define	HS_SPI_REG_RXFIFO15	0xCC
#define	HS_SPI_REG_CSCFG	0xD0
#define	HS_SPI_REG_CSITIME	0xD4
#define	HS_SPI_REG_CSAEXT	0xD8
#define	HS_SPI_REG_RDCSDC0	0xDC
#define	HS_SPI_REG_RDCSDC1	0xDE
#define	HS_SPI_REG_RDCSDC2	0xE0
#define	HS_SPI_REG_RDCSDC3	0xE2
#define	HS_SPI_REG_RDCSDC4	0xE4
#define	HS_SPI_REG_RDCSDC5	0xE6
#define	HS_SPI_REG_RDCSDC6	0xE8
#define	HS_SPI_REG_RDCSDC7	0xEA
#define	HS_SPI_REG_WRCSDC0	0xEC
#define	HS_SPI_REG_WRCSDC1	0xEE
#define	HS_SPI_REG_WRCSDC2	0xF0
#define	HS_SPI_REG_WRCSDC3	0xF2
#define	HS_SPI_REG_WRCSDC4	0xF4
#define	HS_SPI_REG_WRCSDC5	0xF6
#define	HS_SPI_REG_WRCSDC6	0xF8
#define	HS_SPI_REG_WRCSDC7	0xFA
#define	HS_SPI_REG_MID		0xFC

/*
 * HS_SPI register bit definitions
 */

/* HS_SPI Module Control Register */
#define	MCTRL_SYNCON_OFFSET	5
#define	MCTRL_SYNCON_MASK	1

#define	MCTRL_MES_OFFSET	4
#define	MCTRL_MES_MASK		1

#define	MCTRL_CDSS_OFFSET	3
#define	MCTRL_CDSS_MASK		1

#define	MCTRL_CSEN_OFFSET	1
#define	MCTRL_CSEN_MASK		1

#define	MCTRL_MEN_OFFSET	0
#define	MCTRL_MEN_MASK		1

/* HS_SPI Peripheral Communication Configuratio Register0-3 */
#define	PCC_RDDSEL_OFFSET	21
#define	PCC_RDDSEL_MASK		0x3

#define	PCC_WRDSEL_OFFSET	17
#define	PCC_WRDSEL_MASK		0xF

#define	PCC_ESYNC_OFFSET	16
#define	PCC_ESYNC_MASK		1

#define	PCC_CDRS_OFFSET		9
#define	PCC_CDRS_MASK		0x7F

#define	PCC_SENDIAN_OFFSET	8
#define	PCC_SENDIAN_MASK	1

#define	PCC_SDIR_OFFSET		7
#define	PCC_SDIR_MASK		1

#define	PCC_SS2CD_OFFSET	5
#define	PCC_SS2CD_MASK		3

#define	PCC_SSPOL_OFFSET	4
#define	PCC_SSPOL_MASK		1

#define	PCC_ACES_OFFSET		2
#define	PCC_ACES_MASK		1

#define	PCC_CPOL_OFFSET		1
#define	PCC_CPOL_MASK		1

#define	PCC_CPHA_OFFSET		0
#define	PCC_CPHA_MASK		1

/* HS_SPI TX Interrupt Flag Register */
#define	TXF_TSSRS_OFFSET	6
#define	TXF_TSSRS_MASK		1

#define	TXF_TFMTS_OFFSET	5
#define	TXF_TFMTS_MASK		1

#define	TXF_TFLETS_OFFSET	4
#define	TXF_TFLETS_MASK		1

#define	TXF_TFUS_OFFSET		3
#define	TXF_TFUS_MASK		1

#define	TXF_TFOS_OFFSET		2
#define	TXF_TFOS_MASK		1

#define	TXF_TFES_OFFSET		1
#define	TXF_TFES_MASK		1

#define	TXF_TFFS_OFFSET		0
#define	TXF_TFFS_MASK		1

/* HS_SPI TX Interrupt Enable Register */
#define	TXE_TSSRE_OFFSET	6
#define	TXE_TSSRE_MASK		1

#define	TXE_TFMTE_OFFSET	5
#define	TXE_TFMTE_MASK		1

#define	TXE_TFLETE_OFFSET	4
#define	TXE_TFLETE_MASK		1

#define	TXE_TFUE_OFFSET		3
#define	TXE_TFUE_MASK		1

#define	TXE_TFOE_OFFSET		2
#define	TXE_TFOE_MASK		1

#define	TXE_TFEE_OFFSET		1
#define	TXE_TFEE_MASK		1

#define	TXE_TFFE_OFFSET		0
#define	TXE_TFFE_MASK		1

/* HS_SPI TX Interrupt Clear Register */
#define	TXC_TSSRC_OFFSET	6
#define	TXC_TSSRC_MASK		1

#define	TXC_TFMTC_OFFSET	5
#define	TXC_TFMTC_MASK		1

#define	TXC_TFLETC_OFFSET	4
#define	TXC_TFLETC_MASK		1

#define	TXC_TFUC_OFFSET		3
#define	TXC_TFUC_MASK		1

#define	TXC_TFOC_OFFSET		2
#define	TXC_TFOC_MASK		1

#define	TXC_TFEC_OFFSET		1
#define	TXC_TFEC_MASK		1

#define	TXC_TFFC_OFFSET		0
#define	TXC_TFFC_MASK		1

/* HS_SPI RX Interrupt Flag Register */
#define	RXF_RSSRS_OFFSET	6
#define	RXF_RSSRS_MASK		1

#define	RXF_RFMTS_OFFSET	5
#define	RXF_RFMTS_MASK		1

#define	RXF_RFLETS_OFFSET	4
#define	RXF_RFLETS_MASK		1

#define	RXF_RFUS_OFFSET		3
#define	RXF_RFUS_MASK		1

#define	RXF_RFOS_OFFSET		2
#define	RXF_RFOS_MASK		1

#define	RXF_RFES_OFFSET		1
#define	RXF_RFES_MASK		1

#define	RXF_RFFS_OFFSET		0
#define	RXF_RFFS_MASK		1

/* HS_SPI RX Interrupt Enable Register */
#define	RXE_RSSRE_OFFSET	6
#define	RXE_RSSRE_MASK		1

#define	RXE_RFMTE_OFFSET	5
#define	RXE_RFMTE_MASK		1

#define	RXE_RFLETE_OFFSET	4
#define	RXE_RFLETE_MASK		1

#define	RXE_RFUE_OFFSET		3
#define	RXE_RFUE_MASK		1

#define	RXE_RFOE_OFFSET		2
#define	RXE_RFOE_MASK		1

#define	RXE_RFEE_OFFSET		1
#define	RXE_RFEE_MASK		1

#define	RXE_RFFE_OFFSET		0
#define	RXE_RFFE_MASK		1

/* HS_SPI RX Interrupt Clear Register */
#define	RXC_RSSRC_OFFSET	6
#define	RXC_RSSRC_MASK		1

#define	RXC_RFMTC_OFFSET	5
#define	RXC_RFMTC_MASK		1

#define	RXC_RFLETC_OFFSET	4
#define	RXC_RFLETC_MASK		1

#define	RXC_RFUC_OFFSET		3
#define	RXC_RFUC_MASK		1

#define	RXC_RFOC_OFFSET		2
#define	RXC_RFOC_MASK		1

#define	RXC_RFEC_OFFSET		1
#define	RXC_RFEC_MASK		1

#define	RXC_RFFC_OFFSET		0
#define	RXC_RFFC_MASK		1

/* HS_SPI Fault Interrupt Flag Register */
#define	FAULTF_DRCBSFS_OFFSET	4
#define	FAULTF_DRCBSFS_MASK	1

#define	FAULTF_DWCBSFS_OFFSET	3
#define	FAULTF_DWCBSFS_MASK	1

#define	FAULTF_PVFS_OFFSET	2
#define	FAULTF_PVFS_MASK	1

#define	FAULTF_WAFS_OFFSET	1
#define	FAULTF_WAFS_MASK	1

#define	FAULTF_UMAFS_OFFSET	0
#define	FAULTF_UMAFS_MASK	1

/* HS_SPI Fault Interrupt Flag Register */
#define	FAULTC_DRCBSFC_OFFSET	4
#define	FAULTC_DRCBSFC_MASK	1

#define	FAULTC_DWCBSFC_OFFSET	3
#define	FAULTC_DWCBSFC_MASK	1

#define	FAULTC_PVFC_OFFSET	2
#define	FAULTC_PVFC_MASK	1

#define	FAULTC_WAFC_OFFSET	1
#define	FAULTC_WAFC_MASK	1

#define	FAULTC_UMAFC_OFFSET	0
#define	FAULTC_UMAFC_MASK	1

/* HS_SPI Direct Mode Configuration Register */
#define	DMCFG_MSTARTEN_OFFSET	2
#define	DMCFG_MSTARTEN_MASK	1

#define	DMCFG_SSDC_OFFSET	1
#define	DMCFG_SSDC_MASK		1

/* HS_SPI Direct Mode DMA Enable Register */
#define	DMDMAEN_TXDMAEN_OFFSET	9
#define	DMDMAEN_TXDMAEN_MASK	1

#define	DMDMAEN_RXDMAEN_OFFSET	8
#define	DMDMAEN_RXDMAEN_MASK	1

/* HS_SPI Direct Mode Start Register */
#define	DMSTART_START_OFFSET	0
#define	DMSTART_START_MASK	1

/* HS_SPI Direct Mode Stop Register */
#define	DMSTOP_STOP_OFFSET	8
#define	DMSTOP_STOP_MASK	1

/* HS_SPI Direct Mode Peripheral Select Register */
#define	DMPSEL_PSEL_OFFSET	16
#define	DMPSEL_PSEL_MASK	3

/* HS_SPI Direct Mode Transfer Protocol Register */
#define	DMTRP_TRP_OFFSET	24
#define	DMTRP_TRP_MASK		0x0F

/* HS_SPI Direct Mode Byte Count Control Register */
#define	DMBCC_BCC_OFFSET	0
#define	DMBCC_BCC_MASK		0xFFFF

/* HS_SPI Direct Mode Byte Count Status Register */
#define	DMBCS_BCS_OFFSET	16
#define	DMBCS_BCS_MASK		0xFFFF

/* HS_SPI Direct Mode Status Register */
#define	DMSTATUS_TXFLEVEL_OFFSET	16
#define	DMSTATUS_TXFLEVEL_MASK		0x1F

#define	DMSTATUS_RXFLEVEL_OFFSET	8
#define	DMSTATUS_RXFLEVEL_MASK		0x1F

#define	DMSTATUS_TXACTIVE_OFFSET	1
#define	DMSTATUS_TXACTIVE_MASK		1

#define	DMSTATUS_RXACTIVE_OFFSET	0
#define	DMSTATUS_RXACTIVE_MASK		1

/* HS_SPI Transmit Bit Count Register */
#define	TXBITCNT_TXBITCNT_OFFSET	0
#define	TXBITCNT_TXBITCNT_MASK		0x3F

/* HS_SPI FIFO Configuration Register */
#define	FIFOCFG_TXFLSH_OFFSET	12
#define	FIFOCFG_TXFLSH_MASK	1

#define	FIFOCFG_RXFLSH_OFFSET	11
#define	FIFOCFG_RXFLSH_MASK	1

#define	FIFOCFG_TXCTRL_OFFSET	10
#define	FIFOCFG_TXCTRL_MASK	1

#define	FIFOCFG_FWIDTH_OFFSET	8
#define	FIFOCFG_FWIDTH_MASK	3

#define	FIFOCFG_TXFTH_OFFSET	4
#define	FIFOCFG_TXFTH_MASK	0x0F

#define	FIFOCFG_RXFTH_OFFSET	0
#define	FIFOCFG_RXFTH_MASK	0x0F

/* HS_SPI Command Sequencer Configuration Register */
#define	CSCFG_MSEL_OFFSET	16
#define	CSCFG_MSEL_MASK		0x0F

#define	CSCFG_SSELEN_OFFSET	8
#define	CSCFG_SSELEN_MASK	0xF

#define	CSCFG_BSEL_OFFSET	5
#define	CSCFG_BSEL_MASK	1

#define	CSCFG_BOOTEN_OFFSET	4
#define	CSCFG_BOOTEN_MASK	1

#define	CSCFG_SPICHNG_OFFSET	3
#define	CSCFG_SPICHNG_MASK	1

#define	CSCFG_MBM_OFFSET	1
#define	CSCFG_MBM_MASK		3

#define	CSCFG_SRAM_OFFSET	0
#define	CSCFG_SRAM_MASK		1

/* HS_SPI Command Sequencer Idle Time Register */
#define	CSITIME_ITIME_OFFSET	0
#define	CSITIME_ITIME_MASK	0xFFFF

/* HS_SPI Command Sequencer Address Extension Register */
#define	CSAEXT_AEXT_OFFSET	16
#define	CSAEXT_AEXT_MASK	0xFFFF

/* HS_SPI Read Command Sequence Data/Control Register0-7 */
#define	RDCSDC_RDCSDATA_OFFSET	8
#define	RDCSDC_RDCSDATA_MASK	0x0F

#define	RDCSDC_CONT_OFFSET	3
#define	RDCSDC_CONT_MASK	1

#define	RDCSDC_TRP_OFFSET	1
#define	RDCSDC_TRP_MASK		0x3

#define	RDCSDC_DEC_OFFSET	0
#define	RDCSDC_DEC_MASK		1

/* HS_SPI Write Command Sequence Data/Control Register0-7 */
#define	WRCSDC_WRCSDATA_OFFSET	8
#define	WRCSDC_WRCSDATA_MASK	0x0F

#define	WRCSDC_CONT_OFFSET	3
#define	WRCSDC_CONT_MASK	1

#define	WRCSDC_TRP_OFFSET	1
#define	WRCSDC_TRP_MASK		0x3

#define	WRCSDC_DEC_OFFSET	0
#define	WRCSDC_DEC_MASK		1

/* Bit manipulation macros */
#define HSSPI_BIT(bit) \
	(1 << bit##_OFFSET)

#define HSSPI_BITS(bits, val) \
	(((val) & bits##_MASK) << bits##_OFFSET)

#define HSSPI_BITS_GET(wid, bits, hs, reg) \
	((hs_spi_read##wid(hs, reg) >> bits##_OFFSET) & bits##_MASK)

#define HSSPI_BITS_SET(wid, bits, val, hs, reg) do { \
		hs_spi_read##wid(hs, reg); \
		hs_spi_write##wid(hs, reg, ((hs_spi_read##wid(hs, reg) & \
		  ~(bits##_MASK << bits##_OFFSET)) | HSSPI_BITS(bits, val))); \
	} while (0)

/* Register access macros */
#define hs_spi_fiforead(hs, regs) \
	__raw_readb((hs)->reg + HS_SPI_REG_##regs)

#define hs_spi_readb(hs, regs) \
	readb((hs)->reg + HS_SPI_REG_##regs)

#define hs_spi_readw(hs, regs) \
	readw((hs)->reg + HS_SPI_REG_##regs)

#define hs_spi_readl(hs, regs) \
	readl((hs)->reg + HS_SPI_REG_##regs)

#define hs_spi_writeb(hs, regs, val) \
	writeb((val), (hs)->reg + HS_SPI_REG_##regs)

#define hs_spi_writew(hs, regs, val) \
	writew((val), (hs)->reg + HS_SPI_REG_##regs)

#define hs_spi_writel(hs, regs, val) \
	writel((val), (hs)->reg + HS_SPI_REG_##regs)

#endif /* __HS_SPI_REG_H__ */
