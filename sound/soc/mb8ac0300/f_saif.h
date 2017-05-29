/*
 * linux/sound/soc/mb8ac0300/f_saif.h
 *
 * Copyright (C) 2011-2012 FUJITSU SEMICONDUCTOR LIMITED
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _F_SAIF_H
#define _F_SAIF_H

/* I2S register offsets */
#define F_SAIF_REG_RXFDAT		0x00
#define F_SAIF_REG_TXFDAT		0x04
#define F_SAIF_REG_CNTREG		0x08
#define F_SAIF_REG_MCR0			0x0c
#define F_SAIF_REG_MCR1			0x10
#define F_SAIF_REG_MCR2			0x14
#define F_SAIF_REG_OPR			0x18
#define F_SAIF_REG_SRST			0x1c
#define F_SAIF_REG_INTCNT		0x20
#define F_SAIF_REG_STATUS		0x24
#define F_SAIF_REG_DMAACT		0x28

/* I2SxCNTREG register */
/* CNTREG register CKRT bit offsets */
#define F_SAIF_CNTREG_CKRT_BIT_OFFSET	26
/* This sets output clock frequency dividing ratio at master operation */
#define F_SAIF_CNTREG_CKRT_MASK \
	(0x3F << F_SAIF_CNTREG_CKRT_BIT_OFFSET)
/*
 * Frame rate is able to be adjusted by inserting OVHD bit
 * following to valid data of the frame
 */
#define F_SAIF_CNTREG_OVHD		(0x3FF << 16)
/* Serial output data of invalid transmission frame is set */
#define F_SAIF_CNTREG_MSKB		(0x01 << 14)
/* Master and slave modes are set */
#define F_SAIF_CNTREG_MSMD		(0x01 << 13)
/* Sub frame construction (number of sub frame) of the frame is specified */
#define F_SAIF_CNTREG_SBFN		(0x01 << 12)
/* Whether word construction of FIFO is 1 or 2 words is set */
#define F_SAIF_CNTREG_RHLL		(0x01 << 11)
/* Clock frequency dividing is selected in the master mode */
#define F_SAIF_CNTREG_ECKM		(0x01 << 10)
/*
 * When reception word length is shorter than the word length of FIFO
 * extension mode of upper bit should be set
 */
#define F_SAIF_CNTREG_BEXT		(0x01 << 9)
/* Output mode of frame synchronous signal is set */
#define F_SAIF_CNTREG_FRUN		(0x01 << 8)
/* Word bit's shift order is set */
#define F_SAIF_CNTREG_MLSB		(0x01 << 7)
/* Transmitting function is enabled or disabled */
#define F_SAIF_CNTREG_TXDIS		(0x01 << 6)
/* Receiving function is enabled or disabled */
#define F_SAIF_CNTREG_RXDIS		(0x01 << 5)
/* Sampling point of the data is specified */
#define F_SAIF_CNTREG_SMPL		(0x01 << 4)
/* I2S_SCKx polarity which drives/samples serial data is specified */
#define F_SAIF_CNTREG_CPOL		(0x01 << 3)
/* Phase is specified to I2S_WSx frame datad */
#define F_SAIF_CNTREG_FSPH		(0x01 << 2)
/* Pulse width of I2Sx_WS is specified */
#define F_SAIF_CNTREG_FSLN		(0x01 << 1)
/* Polarity of I2Sx_WS pin is set */
#define F_SAIF_CNTREG_FSPL		(0x01 << 0)

/* I2SxMCR0REG register */
/* Number of channel of sub frame 1 is set */
#define F_SAIF_MCR0_S1CHN_MASK		(0x1F << 26)
/* Channel length of the channel constructing sub frame 1 is set */
#define F_SAIF_MCR0_S1CHL_MASK		(0x1F << 21)
/* Word length of the channel constructing sub frame 1 is set */
#define F_SAIF_MCR0_S1WDL_MASK		(0x1F << 16)
/* MCR0 register S0CHN bit offsets */
#define F_SAIF_MCR0_S0CHN_BIT_OFFSET	10
/* Number of channel of sub frame 0 is set up to 32 channels */
#define F_SAIF_MCR0_S0CHN_MASK \
	(0x1F << F_SAIF_MCR0_S0CHN_BIT_OFFSET)
/* MCR0 register S0CHL bit offsets */
#define F_SAIF_MCR0_S0CHL_BIT_OFFSET	5
/*
 * Channel length of the channel constructing sub frame 0
 * (bit length of channel) is set
 */
#define F_SAIF_MCR0_S0CHL_MASK \
	(0x1F << F_SAIF_MCR0_S0CHL_BIT_OFFSET)
/* MCR0 register S0WDL bit offsets */
#define F_SAIF_MCR0_S0WDL_BIT_OFFSET	0
/*
 * Word length of the channel constructing sub frame 0
 * (number of bit in channel) is set
 */
#define F_SAIF_MCR0_S0WDL_MASK \
	(0x1F << F_SAIF_MCR0_S0WDL_BIT_OFFSET)

#define F_SAIF_MCR1_MASK		0xFFFFFFFFUL
#define F_SAIF_MCR2_MASK		0xFFFFFFFFUL

/* I2SxOPRREG register */
/* Enable/Disable functions of receiving operation is set */
#define F_SAIF_OPR_RXENB		(0x01 << 24)
/* Enable/Disable functions of transmitting operation is set */
#define F_SAIF_OPR_TXENB		(0x01 << 16)
/* I2S is enabled/disabled */
#define F_SAIF_OPR_START		(0x01 << 0)
/* Enable functions of transmitting */
#define F_SAIF_OPR_TX_ENABLE \
	(F_SAIF_OPR_TXENB | F_SAIF_OPR_START)
/* Enable functions of receiving */
#define F_SAIF_OPR_RX_ENABLE \
	(F_SAIF_OPR_RXENB | F_SAIF_OPR_START)

/* I2SxSRST register */
/* Software reset is performed by writing "1" */
#define F_SAIF_SRST_SRST		(0x01 << 0)

/* I2SxINTCNT register */
/* Interrupt to CPU by TXUDR1 of STATUS register is masked */
#define F_SAIF_INTCNT_TXUD1M		(0x01 << 30)
/* Interrupt to CPU by TBERR of STATUS register is masked */
#define F_SAIF_INTCNT_TBERM		(0x01 << 29)
/* Interrupt to CPU by FERR of STATUS register is masked */
#define F_SAIF_INTCNT_FERRM		(0x01 << 28)
/* Interrupt to CPU by TXUDR0 of STATUS register is masked. */
#define F_SAIF_INTCNT_TXUD0M		(0x01 << 27)
/* Interrupt to CPU by TXOVM of STATUS register is masked */
#define F_SAIF_INTCNT_TXOVM		(0x01 << 26)
/* Interrupt to CPU by TXFI of STATUS register is masked */
#define F_SAIF_INTCNT_TXFIM		(0x01 << 24)
/* Interrupt to CPU by RBERR of STATUS register is masked */
#define F_SAIF_INTCNT_RBERM		(0x01 << 21)
/* Interrupt to CPU by RXUDR of STATUS register is masked */
#define F_SAIF_INTCNT_RXUDM		(0x01 << 20)
/* Interrupt to CPU by RXOVR of STATUS register is masked */
#define F_SAIF_INTCNT_RXOVM		(0x01 << 19)
/* Interrupt to CPU by EOPI of STATUS register is masked */
#define F_SAIF_INTCNT_EOPM		(0x01 << 18)
/* Interrupt to CPU by RXFI of STATUS register is masked */
#define F_SAIF_INTCNT_RXFIM		(0x01 << 16)
/* Threshold value of transmission FIFO is set */
#define F_SAIF_INTCNT_TFTH		(0x0F << 8)
/*
 * This is packet reception completion timer setting bit which sets time-out
 * value of the internal reception completion timer
 */
#define F_SAIF_INTCNT_RPTMR		(0x03 << 4)
/* Threshold value of reception FIFO is set */
#define F_SAIF_INTCNT_RFTH		(0x0F << 0)

/*
 * DMA transfer is not requested even reception data written to transmission
 * FIFO is threshold value or more
 */

/* I2SxDMAACT register */
/* Level-Sensitive mode */
#define F_SAIF_DMAACT_TL1E0		(0x01 << 24)
/* Transmission channel of DMAC (DMA controller) is activated */
#define F_SAIF_DMAACT_TDMACT		(0x01 << 16)
/* Level-Sensitive mode */
#define F_SAIF_DMAACT_RL1E0		(0x01 << 8)
/* The reception channel of DMAC (DMA controller) is activated */
#define F_SAIF_DMAACT_RDMACT		(0x01 << 0)
/* Transmission channel of DMAC (DMA controller) is activated */
#define F_SAIF_DMA_TX_REQ \
	(F_SAIF_DMAACT_TL1E0 | F_SAIF_DMAACT_TDMACT)
/* The reception channel of DMAC (DMA controller) is activated */
#define F_SAIF_DMA_RX_REQ \
	(F_SAIF_DMAACT_RL1E0 | F_SAIF_DMAACT_RDMACT)

/* I2SxSTATUS register */
/* Occurrence of frame error is indicated */
#define F_SAIF_STATUS_FERR		(0x01 << 29)
/*
 * When transmission FIFO underflows at the top of frame,
 * the value is set to "1"
 */
#define F_SAIF_STATUS_TXUDR1		(0x01 << 28)
/* When transmission FIFO underflows during frame transmission */
#define F_SAIF_STATUS_TXUDR0		(0x01 << 27)
/* When transmission FIFO overflows */
#define F_SAIF_STATUS_TXOVR		(0x01 << 26)
/* When reception FIFO underflows */
#define F_SAIF_STATUS_RXUDR		(0x01 << 25)
/* When reception FIFO overflows */
#define F_SAIF_STATUS_RXOVR		(0x01 << 24)
/* This is interrupt flag containing reception timer */
#define F_SAIF_STATUS_EOPI		(0x01 << 19)

#define F_SAIF_GET_RXFDAT(p) \
	(ioread32((p->regs) + F_SAIF_REG_RXFDAT))
#define F_SAIF_SET_TXFDAT(p, v) \
	(iowrite32((v), (p->regs) + F_SAIF_REG_TXFDAT))
#define F_SAIF_GET_CNTREG(p) \
	(ioread32((p->regs) + F_SAIF_REG_CNTREG))
#define F_SAIF_SET_CNTREG(p, v) \
	(iowrite32((v), (p->regs) + F_SAIF_REG_CNTREG))
#define F_SAIF_GET_MCR0(p) \
	(ioread32((p->regs) + F_SAIF_REG_MCR0))
#define F_SAIF_SET_MCR0(p, v) \
	(iowrite32((v), (p->regs) + F_SAIF_REG_MCR0))
#define F_SAIF_GET_MCR1(p) \
	(ioread32((p->regs) + F_SAIF_REG_MCR1))
#define F_SAIF_SET_MCR1(p, v) \
	(iowrite32((v), (p->regs) + F_SAIF_REG_MCR1))
#define F_SAIF_GET_MCR2(p) \
	(ioread32((p->regs) + F_SAIF_REG_MCR2))
#define F_SAIF_SET_MCR2(p, v) \
	(iowrite32((v), (p->regs) + F_SAIF_REG_MCR2))
#define F_SAIF_GET_OPR(p) \
	(ioread32((p->regs) + F_SAIF_REG_OPR))
#define F_SAIF_SET_OPR(p, v) \
	(iowrite32((v), (p->regs) + F_SAIF_REG_OPR))
#define F_SAIF_GET_SRST(p) \
	(ioread32((p->regs) + F_SAIF_REG_SRST))
#define F_SAIF_SET_SRST(p, v) \
	(iowrite32((v), (p->regs) + F_SAIF_REG_SRST))
#define F_SAIF_GET_INTCNT(p) \
	(ioread32((p->regs) + F_SAIF_REG_INTCNT))
#define F_SAIF_SET_INTCNT(p, v) \
	(iowrite32((v), (p->regs) + F_SAIF_REG_INTCNT))
#define F_SAIF_GET_STATUS(p) \
	(ioread32((p->regs) + F_SAIF_REG_STATUS))
#define F_SAIF_SET_STATUS(p, v) \
	(iowrite32((v), (p->regs) + F_SAIF_REG_STATUS))
#define F_SAIF_GET_DMAACT(p) \
	(ioread32((p->regs) + F_SAIF_REG_DMAACT))
#define F_SAIF_SET_DMAACT(p, v) \
	(iowrite32((v), (p->regs) + F_SAIF_REG_DMAACT))

/* max channels count of one frame */
#define F_SAIF_MAX_SOFT_CHANNELS	2
/* min channels count of one frame */
#define F_SAIF_MIN_SOFT_CHANNELS	1

/* disable i2s */
#define F_SAIF_DISABLE			0
/* enable i2s */
#define F_SAIF_ENABLE			1

#define F_SAIF_WORD_LENGTH_8BIT		8
#define F_SAIF_WORD_LENGTH_16BIT	16
#define F_SAIF_WORD_LENGTH_24BIT	24
#define F_SAIF_WORD_LENGTH_32BIT	32

#define F_SAIF_MIN_AHB_DIV		2
#define F_SAIF_MAX_AHB_DIV		63
#define F_SAIF_MIN_ECLK_DIV		0
#define F_SAIF_MAX_ECLK_DIV		63

enum {
	F_SAIF_SLAVE_MODE,	/* i2s slave mode */
	F_SAIF_MASTER_MODE,	/* i2s master mode */
};

/* support range of sampling rate */
#define F_SAIF_RATES			SNDRV_PCM_RATE_8000_192000

/* support range of data format */
#define F_SAIF_FMTS \
	(SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_U8 | SNDRV_PCM_FMTBIT_S16_LE |\
	SNDRV_PCM_FMTBIT_U16_LE | SNDRV_PCM_FMTBIT_S24_LE |\
	SNDRV_PCM_FMTBIT_U24_LE | SNDRV_PCM_FMTBIT_S32_LE |\
	SNDRV_PCM_FMTBIT_U32_LE)

enum {
	F_SAIF_CLKSRC_AHB,	/*i2s clock source:AHB clock*/
	F_SAIF_CLKSRC_ECLK,	/*i2s clock source:external clock*/
};

#include "mb8ac0300_pcm.h"
struct f_saif_info {
	struct device *dev;		/* device information */
	void __iomem *regs;		/* address of registers */
	phys_addr_t regs_phys;		/* phy address of register */
	unsigned int irq;		/*the irq*/
	struct clk *clk;		/* clock device */
	unsigned int clk_rate;		/* bit clock */
	unsigned int master;		/* master/slave mode */
	spinlock_t lock;		/* lock */
	unsigned int status;		/* 0=stop, 1=start */
	unsigned int rate;		/* sampling freqency (Hz) */
	unsigned int bits;		/* sampling bits (8 / 16 / 24 / 32) */
	unsigned int channels;		/* 1=mono, 2=stereo, */
	unsigned int format;		/* i2s data format */
	unsigned int i2s_cntreg;	/* value of CNTREG register */
	unsigned int i2s_mcr0;		/* value of MCR0 register */
	unsigned int i2s_mcr1;		/* value of MCR1 register */
	unsigned int i2s_mcr2;		/* value of MCR2 register */
	unsigned int i2s_intcnt;	/* value of INTCNT register */
	unsigned int i2s_oprreg;	/* value of OPRREG register */
	unsigned int i2s_dmaact;	/* value of DMAACT register */
};

struct f_saif_info *f_get_i2s(unsigned int id);
int f_hw_params(struct mb8ac0300_pcm_runtime_data *prtd,
		struct snd_pcm_hw_params *params, struct f_saif_info *i2s);
void f_saif_tx_enable(unsigned int id);
void f_saif_tx_disable(unsigned int id);
int f_saif_mclk_set(unsigned int id, unsigned int clk_rate);
int f_saif_params_set(unsigned int id,
			struct snd_pcm_hw_params *params);
#endif/* _F_SAIF_H */

