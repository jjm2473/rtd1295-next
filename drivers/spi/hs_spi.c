/*
 * linux/drivers/spi/hs_spi.c - high speed SPI controller driver
 *
 * Copyright (C) 2010-2012 FUJITSU SEMICONDUCTOR LIMITED
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

#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_data/dma-mb8ac0300-xdmac.h>
#include <linux/delay.h>

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#include <linux/platform_data/mb8ac0300-hs_spi.h>

#include "hs_spi_reg.h"

// #define DEBUG
#define	DRV_NAME "hs-spi"

/* HS_SPI all TX interrupts except Slave Select Released Interrupt */
#define	HS_SPI_TXINT_EXCEPT_TSSRC	(TXC_TFMTC_MASK << TXC_TFMTC_OFFSET |\
				TXC_TFLETC_MASK << TXC_TFLETC_OFFSET |\
				TXC_TFUC_MASK << TXC_TFUC_OFFSET |\
				TXC_TFOC_MASK << TXC_TFOC_OFFSET |\
				TXC_TFEC_MASK << TXC_TFEC_OFFSET |\
				TXC_TFFC_MASK << TXC_TFFC_OFFSET)
/* HS_SPI all RX interrupts except Slave Select Released Interrupt */
#define	HS_SPI_RXINT_EXCEPT_RSSRC	(RXC_RFMTC_MASK << RXC_RFMTC_OFFSET |\
				RXC_RFLETC_MASK << RXC_RFLETC_OFFSET |\
				RXC_RFUC_MASK << RXC_RFUC_OFFSET |\
				RXC_RFOC_MASK << RXC_RFOC_OFFSET |\
				RXC_RFEC_MASK << RXC_RFEC_OFFSET |\
				RXC_RFFC_MASK << RXC_RFFC_OFFSET)
/* HS_SPI all TX interrupts */
#define	HS_SPI_TX_ALL_INT	(TXC_TSSRC_MASK << TXC_TSSRC_OFFSET |\
				HS_SPI_TXINT_EXCEPT_TSSRC)
/* HS_SPI all RX interrupts */
#define	HS_SPI_RX_ALL_INT	(RXC_RSSRC_MASK << RXC_RSSRC_OFFSET |\
				HS_SPI_RXINT_EXCEPT_RSSRC)
/* HS_SPI all fault interrupts */
#define	HS_SPI_ALL_FAULT	(FAULTC_DRCBSFC_MASK << FAULTC_DRCBSFC_OFFSET |\
				FAULTC_DWCBSFC_MASK << FAULTC_DWCBSFC_OFFSET |\
				FAULTC_PVFC_MASK << FAULTC_PVFC_OFFSET |\
				FAULTC_WAFC_MASK << FAULTC_WAFC_OFFSET |\
				FAULTC_UMAFC_MASK << FAULTC_UMAFC_OFFSET)
/* HS_SPI mode bits mask value */
#define	HS_SPI_MODE_MASK	(PCC_CPHA_MASK << PCC_CPHA_OFFSET |\
				PCC_CPOL_MASK << PCC_CPOL_OFFSET |\
				PCC_SSPOL_MASK << PCC_SSPOL_OFFSET |\
				PCC_SDIR_MASK << PCC_SDIR_OFFSET)

/*
 * HS_SPI Controller state
 */
struct hs_spi_cs {
	unsigned char	mode;
	unsigned char	chip_select;
	unsigned int	speed_hz;
};

/*
 * Debugging macro and defines
 */
#ifdef CONFIG_HS_SPI_DEBUG
#define HS_SPI_DEBUG(n, args...) pr_info(args);
#else /* CONFIG_HS_SPI_DEBUG */
#define HS_SPI_DEBUG(n, args...)
#endif /* CONFIG_HS_SPI_DEBUG */

/*
 * hs_spi_write_tx_fifo - write datas into the transmit FIFO at direct mode
 * @hs:		HS SPI device platform data.
 *
 * No more than 16 byte datas can be write once.
 *
 * Returns write data size
 */
static int hs_spi_write_tx_fifo(struct hs_spi *hs)
{
	unsigned int	txflevel = HSSPI_BITS_GET(l, DMSTATUS_TXFLEVEL, hs,
			DMSTATUS);
	unsigned int	txbytes = min(HS_SPI_FIFO_LEN - txflevel,
			hs->len - hs->tx_cnt);
	unsigned int	i;
	unsigned char	txdata;

	for (i = 0; i < txbytes; i++) {
		txdata = hs->tx ? hs->tx[hs->tx_cnt + i] : 0xFF;
		hs_spi_writeb(hs, TXFIFO0, txdata);
	}

	return txbytes;
}

/*
 * hs_spi_read_rx_fifo - read datas from the receive FIFO direct mode
 * @hs:		HS SPI device platform data.
 *
 * No more then 16 byte datas can be read once.
 *
 * Returns read data size
 */
static int hs_spi_read_rx_fifo(struct hs_spi *hs)
{
	unsigned int	rxflevel = HSSPI_BITS_GET(l, DMSTATUS_RXFLEVEL, hs,
			DMSTATUS);
	unsigned int	rxbytes = min(rxflevel, hs->len - hs->rx_cnt);
	unsigned int	i;
	unsigned char	rxdata;

	for (i = 0; i < rxbytes; i++) {
		rxdata = hs_spi_fiforead(hs, RXFIFO0);
		hs->rx[hs->rx_cnt + i] = rxdata;
	}
	smp_rmb(); /* dunno why this is needed */

	return rxbytes;
}

/*
 * hs_spi_read_dummy - read dummy from the receive FIFO at direct mode
 * @hs:		HS SPI device platform data.
 *
 * When transfer protocol is TX_RX,
 * While TX-FIFO is transmitting data, RX-FIFO is also receiving dummy
 * at the same time.
 */
static void hs_spi_read_dummy(struct hs_spi *hs)
{
	unsigned int	rxbytes = HSSPI_BITS_GET(l, DMSTATUS_RXFLEVEL, hs,
			DMSTATUS);
	unsigned int	i;
	unsigned char	rxdata;

	for (i = 0; i < rxbytes; i++)
		rxdata = hs_spi_fiforead(hs, RXFIFO0);

	smp_rmb(); /* why? */
}

/*
 * hs_spi_set_speed - setup the clock frequency
 * @spi:	SPI device data.
 * @hz:		clock frequency to set.
 *
 * The clock frequency can be designated by spi devices or transfers.
 *
 * Returns 0 on success; negative errno on failure
 */
static int hs_spi_set_speed(struct spi_device *spi, unsigned int hz)
{
	struct hs_spi_cs	*cs = spi->controller_state;
	struct hs_spi		*hs = spi_master_get_devdata(spi->master);
	unsigned int		div;
	unsigned long		rate;
	unsigned char		safesync = 0;
	u32 csval = 0x230508;

	rate = clk_get_rate(hs->clk);

	div = DIV_ROUND_UP(rate, hz * 2);
	/*
	 * If the resulting divider doesn't fit into the
	 * register bitfield, we can't satisfy the constraint.
	 */
	if (div > 127) {
		dev_err(&spi->dev,
			"setup: %d Hz too slow, div %u; min %ld Hz\n",
			hz, div, rate / (2 * 127));
		return -EINVAL;
	}

	/* safesync bit */
	if (hs->pdata->mode == HS_SPI_DIRECT_MODE) {
		/* direct mode */
		if (((spi->rx_bitwidth == 4) ||
		     (spi->tx_bitwidth == 4)) &&
		     (div < 3))
			safesync = 1;
	} else
		/* cs mode */
		if (hs->pdata->clock == HS_SPI_PCLK)
			if (((spi->rx_bitwidth == 4) ||
				 (spi->tx_bitwidth == 4)) &&
			    (div < 3))
				safesync = 1;

	switch (cs->chip_select) {
	case 0:
		HSSPI_BITS_SET(l, PCC_CDRS, div, hs, PCC0);
		HSSPI_BITS_SET(l, PCC_ESYNC, safesync, hs, PCC0);
		HSSPI_BITS_SET(l, PCC_ACES, 0, hs, PCC0);
		HSSPI_BITS_SET(l, PCC_CPHA, 0, hs, PCC0);
		HS_SPI_DEBUG(&spi->dev, "Spi speed is set to %dHz, div:%u\n",
			hz, HSSPI_BITS_GET(l, PCC_CDRS, hs, PCC0));

		/* for now set them all to something good for mb86s70 */

		writel(csval, hs->reg + 4);
		writel(csval, hs->reg + 8);
		writel(csval, hs->reg + 0xc);
		writel(csval, hs->reg + 0x10);
		break;
	case 1:
		HSSPI_BITS_SET(l, PCC_CDRS, div, hs, PCC1);
		HSSPI_BITS_SET(l, PCC_ESYNC, safesync, hs, PCC1);
		HS_SPI_DEBUG(&spi->dev, "Spi speed is set to %dHz, div:%u\n",
			hz, HSSPI_BITS_GET(l, PCC_CDRS, hs, PCC1));
		/* for now set them all to something good for mb86s70 */

		writel(csval, hs->reg + 4);
		writel(csval, hs->reg + 8);
		writel(csval, hs->reg + 0xc);
		writel(csval, hs->reg + 0x10);
		break;
	case 2:
		HSSPI_BITS_SET(l, PCC_CDRS, div, hs, PCC2);
		HSSPI_BITS_SET(l, PCC_ESYNC, safesync, hs, PCC2);
		HS_SPI_DEBUG(&spi->dev, "Spi speed is set to %dHz, div:%u\n",
			hz, HSSPI_BITS_GET(l, PCC_CDRS, hs, PCC2));
		break;
	case 3:
		HSSPI_BITS_SET(l, PCC_CDRS, div, hs, PCC3);
		HSSPI_BITS_SET(l, PCC_ESYNC, safesync, hs, PCC3);
		HS_SPI_DEBUG(&spi->dev, "Spi speed is set to %dHz, div:%u\n",
			hz, HSSPI_BITS_GET(l, PCC_CDRS, hs, PCC3));
		break;
	default:
		dev_err(&spi->dev,
			"setup: invalid chipselect %u (%u defined)\n",
			cs->chip_select, spi->master->num_chipselect);
		return -EINVAL;
	}
	cs->speed_hz = hz;

	return 0;
}

/*
 * hs_spi_quad_enable - enable quad mode for serial flash device
 * @hs:		HS SPI device platform data.
 * @spi:	SPI device data.
 *
 * The Quad bit of Configuration Register must be set to
 * puts the device into Quad I/O mode.
 */
static void hs_spi_quad_enable(struct hs_spi *hs, struct spi_device *spi)
{
	unsigned char	cmd[3];

	/* Set up the write data buffer. */
	cmd[0] = HS_SPI_CMD_WRSR;
	cmd[1] = 0x00;
	cmd[2] = HS_SPI_CR_QUAD;

	/* select chip */
	HSSPI_BITS_SET(l, DMPSEL_PSEL, spi->chip_select, hs, DMPSEL);
	/* Flush RX and TX FIFO  */
	HSSPI_BITS_SET(l, FIFOCFG_TXFLSH, 1, hs, FIFOCFG);
	HSSPI_BITS_SET(l, FIFOCFG_RXFLSH, 1, hs, FIFOCFG);
	/* set transfer protocol */
	HSSPI_BITS_SET(l, DMTRP_TRP, HS_SPI_LEGACY_TX_ONLY, hs, DMTRP);
	/* set transfer size */
	HSSPI_BITS_SET(l, DMBCC_BCC, 3, hs, DMBCC);

	/* transmit command byte at single line */
	hs_spi_writeb(hs, TXFIFO0, cmd[0]);
	hs_spi_writeb(hs, TXFIFO0, cmd[1]);
	hs_spi_writeb(hs, TXFIFO0, cmd[2]);
	/* start transfer */
	HSSPI_BITS_SET(l, DMSTART_START, 1, hs, DMSTART);

	init_completion(&hs->done);
	hs_spi_writel(hs, TXE, TXE_TFEE_MASK << TXE_TFEE_OFFSET);
	wait_for_completion(&hs->done);
}

/*
 * hs_spi_read_sr - read the status register
 * @hs:		HS SPI device platform data.
 * @spi:	SPI device data.
 *
 * Returns the status register value.
 */
static unsigned char hs_spi_read_sr(struct hs_spi *hs, struct spi_device *spi)
{
	unsigned char	cmd;
	unsigned char	status = 0;
	int timeout = 100000;

	/* Set up the write data buffer. */
	cmd = HS_SPI_CMD_RDSR;

	/* select chip */
	HSSPI_BITS_SET(l, DMPSEL_PSEL, spi->chip_select, hs, DMPSEL);
	/* Flush RX and TX FIFO  */
	HSSPI_BITS_SET(l, FIFOCFG_TXFLSH, 1, hs, FIFOCFG);
	HSSPI_BITS_SET(l, FIFOCFG_RXFLSH, 1, hs, FIFOCFG);
	/* set transfer protocol */
	HSSPI_BITS_SET(l, DMTRP_TRP, HS_SPI_LEGACY_TX_ONLY, hs, DMTRP);
	/* set transfer size */
	HSSPI_BITS_SET(l, DMBCC_BCC, 2, hs, DMBCC);

	/* start transfer */
	HSSPI_BITS_SET(l, DMSTART_START, 1, hs, DMSTART);

	/* transmit command byte at single line */
	hs_spi_writeb(hs, TXFIFO0, cmd);

	init_completion(&hs->done);
	hs_spi_writel(hs, TXE, TXE_TFEE_MASK << TXE_TFEE_OFFSET);
	wait_for_completion(&hs->done);

	/* set transfer protocol */
	HSSPI_BITS_SET(l, DMTRP_TRP, HS_SPI_LEGACY_RX_ONLY, hs, DMTRP);

	while (--timeout && HSSPI_BITS_GET(l, DMSTATUS_RXACTIVE, hs, DMSTATUS))
		;

	if (!timeout) {
		dev_err(&spi->dev, "Timeout waiting for status\n");
		return -ETIME;
	}

	status = hs_spi_readb(hs, RXFIFO0);

	return status;
}

/*
 * hs_spi_write_enable - enable the device to be write
 * @hs:		HS SPI device platform data.
 * @spi:	SPI device data.
 *
 * The Write Enable command must be sent to enables the device to accept a
 * Write Status Register to enable Quad mode.
 */
static void hs_spi_write_enable(struct hs_spi *hs, struct spi_device *spi)
{
	unsigned char	cmd;
	/* Set up the write data buffer. */
	cmd = HS_SPI_CMD_WREN;

	/* select chip */
	HSSPI_BITS_SET(l, DMPSEL_PSEL, spi->chip_select, hs, DMPSEL);
	/* Flush RX and TX FIFO  */
	HSSPI_BITS_SET(l, FIFOCFG_TXFLSH, 1, hs, FIFOCFG);
	HSSPI_BITS_SET(l, FIFOCFG_RXFLSH, 1, hs, FIFOCFG);
	/* set transfer protocol */
	HSSPI_BITS_SET(l, DMTRP_TRP, HS_SPI_LEGACY_TX_ONLY, hs, DMTRP);
	/* set transfer size */
	HSSPI_BITS_SET(l, DMBCC_BCC, 1, hs, DMBCC);

	/* transmit command byte at single line */
	hs_spi_writeb(hs, TXFIFO0, cmd);
	/* start transfer */
	HSSPI_BITS_SET(l, DMSTART_START, 1, hs, DMSTART);

	init_completion(&hs->done);
	hs_spi_writel(hs, TXE, TXE_TFEE_MASK << TXE_TFEE_OFFSET);
	wait_for_completion(&hs->done);
}

/*
 * hs_spi_cs_initialize_device - initializ device for command sequencer mode
 * @hs:		HS SPI device platform data.
 * @spi:	SPI device data.
 *
 * In Command Sequencer mode, it is not possible to change between the
 * legacy, dual-bit or quad-bit modes when a transfer has started.
 * For this reason, for some of the newer Serial Flash devices - like
 * the memory devices from Winbond, the Command Sequencer can be enabled
 * only after the device has been initialized to work in the "Continuous
 * Read Mode".
 *
 * The memory device can be programmed in the Continuous Read
 * Mode, using the Direct Mode of Operation of HS_SPI.
 */
static int  hs_spi_cs_initialize_device(struct hs_spi *hs,
	struct spi_device *spi)
{
	int	i = 0, offset = (int)(hs->reg + HS_SPI_REG_RDCSDC0);
	int	*rdcsdc_p = NULL;
	int timeout = 100000;
	int	rdcsdc_int_data[3][8] = {
	{
#ifdef CONFIG_M25PXX_USE_FAST_READ
		/* FAST_READ */
		(HS_SPI_CMD_FAST_READ << 8) | HS_SPI_CS_1BIT,
		HS_SPI_DECODE_ADDR2 | HS_SPI_CS_1BIT,
		HS_SPI_DECODE_ADDR1 | HS_SPI_CS_1BIT,
		HS_SPI_DECODE_ADDR0 | HS_SPI_CS_1BIT,
		HS_SPI_HIGH_Z_BYTE | HS_SPI_CS_1BIT,
		HS_SPI_LIST_END,
		HS_SPI_LIST_END,
		HS_SPI_LIST_END,
#else
		/* NORM_READ */
		(HS_SPI_CMD_NORM_READ << 8) | HS_SPI_CS_1BIT,
		HS_SPI_DECODE_ADDR2 | HS_SPI_CS_1BIT,
		HS_SPI_DECODE_ADDR1 | HS_SPI_CS_1BIT,
		HS_SPI_DECODE_ADDR0 | HS_SPI_CS_1BIT,
		HS_SPI_LIST_END,
		HS_SPI_LIST_END,
		HS_SPI_LIST_END,
		HS_SPI_LIST_END,
#endif
	},
	{ /* DUAL_READ */
		(HS_SPI_CMD_DOR<<8) | HS_SPI_CS_1BIT,
		HS_SPI_DECODE_ADDR2 | HS_SPI_CS_1BIT,
		HS_SPI_DECODE_ADDR1 | HS_SPI_CS_1BIT,
		HS_SPI_DECODE_ADDR0 | HS_SPI_CS_1BIT,
		HS_SPI_HIGH_Z_BYTE | HS_SPI_CS_1BIT,
		HS_SPI_LIST_END,
		HS_SPI_LIST_END,
		HS_SPI_LIST_END,
	},
	{ /* QUAD_READ */
		(HS_SPI_CMD_QOR<<8) | HS_SPI_CS_1BIT,
		HS_SPI_DECODE_ADDR2 | HS_SPI_CS_1BIT,
		HS_SPI_DECODE_ADDR1 | HS_SPI_CS_1BIT,
		HS_SPI_DECODE_ADDR0 | HS_SPI_CS_1BIT,
		HS_SPI_HIGH_Z_BYTE | HS_SPI_CS_1BIT,
		HS_SPI_LIST_END,
		HS_SPI_LIST_END,
		HS_SPI_LIST_END,
	}
	};

#define SIZE_TO_FLAG(x)	({ \
			int curr_bank, curr_flag; \
			curr_bank = (x) / SZ_8K; \
			for (curr_flag = 0;; curr_flag++) { \
				if (curr_bank == 1) \
					break; \
				curr_bank >>= 1; \
			} \
			curr_flag; \
			})

	/* cs mode spi structure saved */
	hs->spi[spi->chip_select] = spi;
	if (spi->rx_bitwidth == 4) {
		hs_spi_write_enable(hs, spi);
		while (--timeout && hs_spi_read_sr(hs, spi)&HS_SPI_SR_WIP)
			;

		if (!timeout) {
			dev_err(hs->dev, "Timeout in init\n");
			return -ETIME;
		}
		hs_spi_quad_enable(hs, spi);

		HS_SPI_DEBUG(hs->dev, "WRR quad read init command\n");
		while (hs_spi_read_sr(hs, spi)&HS_SPI_SR_WIP)
			;
		while (!(HSSPI_BITS_GET(l, TXF_TSSRS, hs, TXF) |
			HSSPI_BITS_GET(l, RXF_RSSRS, hs, RXF)))
			;
	}
	/* disable module */
	HSSPI_BITS_SET(l, MCTRL_MEN, 0, hs, MCTRL);

	while (HSSPI_BITS_GET(l, MCTRL_MES, hs, MCTRL))
		;

	/* Disable BOOTEN */
	HSSPI_BITS_SET(l, CSCFG_BOOTEN, 0, hs, CSCFG);
	/* set operation mode to command sequencer */
	HSSPI_BITS_SET(l, MCTRL_CSEN, 1, hs, MCTRL);
	HSSPI_BITS_SET(l, CSCFG_MSEL, SIZE_TO_FLAG(hs->bank_size), hs, CSCFG);
	/* reset the address extension field */
	HSSPI_BITS_SET(l, CSAEXT_AEXT, 0, hs, CSAEXT);
	/* set the Command Sequencer Idle Time */
	HSSPI_BITS_SET(l, CSITIME_ITIME, HS_SPI_IDLE_TIME, hs, CSITIME);
	HSSPI_BITS_SET(l, CSCFG_SSELEN, 1 << spi->chip_select, hs, CSCFG);
	/* set Synchronizer ON bit */
	if (hs->pdata->clock == HS_SPI_HCLK) {
		HSSPI_BITS_SET(l, MCTRL_SYNCON, 1, hs, MCTRL);
	} else {
		if (hs->pdata->syncon)
			HSSPI_BITS_SET(l, MCTRL_SYNCON, 1, hs, MCTRL);
		else
			HSSPI_BITS_SET(l, MCTRL_SYNCON, 0, hs, MCTRL);
	}

	if (spi->rx_bitwidth == 1) {
		/* CS transfer mode */
		HSSPI_BITS_SET(l, CSCFG_MBM, HS_SPI_LEGACY_BIT, hs, CSCFG);
		rdcsdc_p = &rdcsdc_int_data[0][0];
	} else if (spi->rx_bitwidth == 2) {
		/* CS transfer mode */
		HSSPI_BITS_SET(l, CSCFG_MBM, HS_SPI_DUAL_BIT, hs, CSCFG);
		rdcsdc_p = &rdcsdc_int_data[1][0];
	} else if (spi->rx_bitwidth == 4) {
		/* CS transfer mode */
		HSSPI_BITS_SET(l, CSCFG_MBM, HS_SPI_QUAD_BIT, hs, CSCFG);
		rdcsdc_p = &rdcsdc_int_data[2][0];
	}

	for (i = 0; i < 8; i++) {
		writew(rdcsdc_p[i], (void __iomem *)(offset + i * 2));
		HS_SPI_DEBUG(hs->dev,
			"address = %#x,Command Sequence is %#x\n",
		offset + i * 2, readw((void __iomem *)(offset + i * 2)));
	}

	/* enable module */
	HSSPI_BITS_SET(l, MCTRL_MEN, 1, hs, MCTRL);
	while (!HSSPI_BITS_GET(l, MCTRL_MES, hs, MCTRL))
		;
	return 0;
}

/*
 * hs_spi_init_hw - initializ hardware registers
 * @hs:		HS SPI device platform data.
 */
static int  hs_spi_init_hw(struct hs_spi *hs)
{
	int timeout = 1000000;
	/* change to dm mode */
	HSSPI_BITS_SET(l, MCTRL_CSEN, 0, hs, MCTRL);

	HSSPI_BITS_SET(l, MCTRL_MEN, 0, hs, MCTRL);
	while (--timeout && HSSPI_BITS_GET(l, MCTRL_MES, hs, MCTRL))
		;

	if (!timeout)
		return -ETIME;

	/* disable interrupt */
	hs_spi_writel(hs, TXE, 0x00);
	hs_spi_writel(hs, RXE, 0x00);
	/* clear interrupt flag */
	hs_spi_writel(hs, TXC, HS_SPI_TXINT_EXCEPT_TSSRC);
	hs_spi_writel(hs, RXC, HS_SPI_RXINT_EXCEPT_RSSRC);

	/* read module ID */
	dev_dbg(hs->dev, "HS SPI module ID:%#4x\n", hs_spi_readl(hs, MID));

	/* Clock Division Source Select 0:AHBCLK 1:PCLK*/
	if (hs->pdata->clock == HS_SPI_HCLK)
		HSSPI_BITS_SET(l, MCTRL_CDSS, 0, hs, MCTRL);
	else
		HSSPI_BITS_SET(l, MCTRL_CDSS, 1, hs, MCTRL);

	if (hs->pdata->mode != HS_SPI_COMMAND_SEQUENCER)
		/* set to software flow control mode */
		HSSPI_BITS_SET(l, DMCFG_SSDC, 0, hs, DMCFG);
	else
		/* set to hardware flow control mode */
		HSSPI_BITS_SET(l, DMCFG_SSDC, 1, hs, DMCFG);

	/* configure the FIFO threshold levels and the FIFO width */
	hs_spi_writel(hs, FIFOCFG,
		HSSPI_BITS(FIFOCFG_FWIDTH, HS_SPI_FIFO_WIDTH) |
		HSSPI_BITS(FIFOCFG_TXFTH, HS_SPI_TX_FIFO_LEVEL) |
		HSSPI_BITS(FIFOCFG_RXFTH, HS_SPI_RX_FIFO_LEVEL));
	/* enable module */
	HSSPI_BITS_SET(l, MCTRL_MEN, 1, hs, MCTRL);
	timeout = 100000;
	while (--timeout && !HSSPI_BITS_GET(l, MCTRL_MES, hs, MCTRL))
		;

	if (!timeout)
		return -ETIME;

	return 0;
}

/*
 * hs_spi_tx_irq - deal with transmit interrupt request
 * @irq:	Interrupt request number.
 * @dev:	HS SPI device platform data.
 *
 * Returns IRQ_HANDLED on success
 */
static irqreturn_t hs_spi_tx_irq(int irq, void *dev)
{
	struct hs_spi		*hs = dev;
	int			txf;
	int			mode = hs->pdata->mode;

	txf = hs_spi_readl(hs, TXF);
	/* clear flags */
	hs_spi_writel(hs, TXC, HS_SPI_TXINT_EXCEPT_TSSRC);

	if (txf & HSSPI_BIT(TXF_TFLETS)) {
		dev_dbg(hs->dev, "TX-FIFO Fill Level <= Threshold\n");
		if ((mode == HS_SPI_DIRECT_MODE) && (hs->tx) &&
			(hs->tx_cnt < hs->len))
			hs->tx_cnt += hs_spi_write_tx_fifo(hs);
	}

	if (txf & HSSPI_BIT(TXF_TFES)) {
		dev_dbg(hs->dev, "TX-FIFO and Shift Register is Empty\n");
		if (mode == HS_SPI_DIRECT_MODE) {
			if (hs->tx_cnt >= hs->len) {
				hs_spi_writel(hs, TXE, 0x00);
				complete(&hs->done);
			}
		} else {
			/* for special cmd on cs mode (only) */
			hs_spi_writel(hs, TXE, 0x00);
			complete(&hs->done);
		}
	}

#ifdef CONFIG_HS_SPI_DEBUG
	if (txf & HSSPI_BIT(TXF_TFMTS))
		dev_dbg(hs->dev, "TX-FIFO Fill Level is More Than Threshold\n");

	if (txf & HSSPI_BIT(TXF_TFUS))
		dev_dbg(hs->dev, "TX-FIFO Underrun\n");

	if (txf & HSSPI_BIT(TXF_TFOS))
		dev_dbg(hs->dev, "TX-FIFO Overrun\n");

	if (txf & HSSPI_BIT(TXF_TFFS))
		dev_dbg(hs->dev, "TX-FIFO Full\n");
#endif /* CONFIG_HS_SPI_DEBUG */

	return IRQ_HANDLED;
}

/*
 * hs_spi_rx_irq - deal with receive interrupt request
 * @irq:	Interrupt request number.
 * @dev:	HS SPI device platform data.
 *
 * Returns IRQ_HANDLED on success
 */
static irqreturn_t hs_spi_rx_irq(int irq, void *dev)
{
	struct hs_spi		*hs = dev;
	int			rxf;

	rxf = hs_spi_readl(hs, RXF);
	/* clear flags */
	hs_spi_writel(hs, RXC, HS_SPI_RXINT_EXCEPT_RSSRC);

	if (rxf & HSSPI_BIT(RXF_RFMTS)) {
		dev_dbg(hs->dev, "RX-FIFO Fill Level is More Than Threshold\n");
		hs->rx_cnt += hs_spi_read_rx_fifo(hs);
		if (hs->rx_cnt >= hs->len) {
			hs_spi_writel(hs, RXE, 0x00);
			complete(&hs->done);
		}
	}

#ifdef CONFIG_HS_SPI_DEBUG
	if (rxf & HSSPI_BIT(RXF_RFLETS))
		dev_dbg(hs->dev, "RX-FIFO Fill Level is <= Threshold\n");

	if (rxf & HSSPI_BIT(RXF_RFUS))
		dev_dbg(hs->dev, "RX-FIFO Underrun\n");

	if (rxf & HSSPI_BIT(RXF_RFOS))
		dev_dbg(hs->dev, "RX-FIFO Overrun\n");

	if (rxf & HSSPI_BIT(RXF_RFES))
		dev_dbg(hs->dev, "RX-FIFO Empty\n");

	if (rxf & HSSPI_BIT(RXF_RFFS))
		dev_dbg(hs->dev, "RX-FIFO Full\n");
#endif /* CONFIG_HS_SPI_DEBUG */

	return IRQ_HANDLED;
}

/*
 * hs_spi_fault_irq - deal with fault interrupt request
 * @irq:	Interrupt request number.
 * @dev:	HS SPI device platform data.
 *
 * Returns IRQ_HANDLED on success
 */
static irqreturn_t hs_spi_fault_irq(int irq, void *dev)
{
	struct hs_spi		*hs = dev;
	unsigned int		faultf;

	faultf = hs_spi_readl(hs, FAULTF);
	/* clear flags */
	hs_spi_writel(hs, FAULTC, HS_SPI_ALL_FAULT);

#ifdef CONFIG_HS_SPI_DEBUG
	if (faultf & HSSPI_BIT(FAULTF_DRCBSFS))
		dev_err(hs->dev, "DMA Read Channel Block Size Fault\n");

	if (faultf & HSSPI_BIT(FAULTF_DWCBSFS))
		dev_err(hs->dev, "DMA Write Channel Block Size Fault\n");

	if (faultf & HSSPI_BIT(FAULTF_PVFS))
		dev_err(hs->dev, "Protection Violation Fault\n");

	if (faultf & HSSPI_BIT(FAULTF_WAFS))
		dev_err(hs->dev, "Write Access Fault\n");

	if (faultf & HSSPI_BIT(FAULTF_UMAFS))
		dev_err(hs->dev, "Unmapped Memory Access Fault\n");
#endif /* CONFIG_HS_SPI_DEBUG */

	hs->fault_flag = 1;
	hs_spi_writel(hs, TXE, 0x00);
	hs_spi_writel(hs, RXE, 0x00);
	complete(&hs->done);

	return IRQ_HANDLED;
}

/*
 * hs_spi_chipselect - select or release the chip
 * @spi:	SPI device data.
 * @value:	Active or inactive.
 */
static void hs_spi_chipselect(struct spi_device *spi, int value)
{
	struct hs_spi		*hs = spi_master_get_devdata(spi->master);

	if (value == BITBANG_CS_INACTIVE) {
		/* stop the transfer */
		HSSPI_BITS_SET(l, DMSTOP_STOP, 1, hs, DMSTOP);
		while (!HSSPI_BITS_GET(l, DMSTOP_STOP, hs, DMSTOP))
			;
		if (hs->rx)
			hs_spi_read_dummy(hs);

		while (!(HSSPI_BITS_GET(l, TXF_TSSRS, hs, TXF) |
			HSSPI_BITS_GET(l, RXF_RSSRS, hs, RXF)))
			;
	} else {
		/* select chip */
		HSSPI_BITS_SET(l, DMPSEL_PSEL, spi->chip_select, hs, DMPSEL);
		HSSPI_BITS_SET(l, DMSTOP_STOP, 0, hs, DMSTOP);
	}
}

/*
 * hs_spi_setup_transfer - setup the transfer attributes
 * @spi:	SPI device data.
 * @t:		Transfer data.
 *
 * Returns 0 on success; negative errno on failure
 */
static int hs_spi_setup_transfer(struct spi_device *spi,
	struct spi_transfer *t)
{
	struct hs_spi_cs	*cs = spi->controller_state;
	unsigned int		hz;
	struct hs_spi		*hs = spi_master_get_devdata(spi->master);

	if (hs->stop)
		return -ESHUTDOWN;
	hz  = t ? t->speed_hz : spi->max_speed_hz;

	/*
	 * Modify the configuration if the transfer overrides it.  Do not allow
	 * the transfer to overwrite the generic configuration with zeros.
	 */
	if ((hz) && (cs->speed_hz != hz)) {
		if (hz > spi->max_speed_hz) {
			dev_err(&spi->dev, "%s: %dHz but max is %dHz\n",
					__func__, hz, spi->max_speed_hz);
			return -EINVAL;
		}
		/* Set the new speed */
		return hs_spi_set_speed(spi, hz);
	}

	return 0;
}

/*
 * hs_spi_setup - first setup for new devices
 * @spi:	SPI device data.
 *
 * Returns 0 on success; negative errno on failure
 */
static int hs_spi_setup(struct spi_device *spi)
{
	struct hs_spi_cs	*cs = spi->controller_state;
	struct hs_spi		*hs = spi_master_get_devdata(spi->master);
	int			mode = hs->pdata->mode;
	unsigned int		cfg = 0;
	int			retval;

	/* allocate settings on the first call */
	if (!cs) {
		cs = kzalloc(sizeof(struct hs_spi_cs), GFP_KERNEL);
		if (!cs) {
			dev_err(&spi->dev, "no memory for controller state\n");
			return -ENOMEM;
		}
		spi->controller_state = cs;
	}

	cs->chip_select = spi->chip_select;

	if (cs->chip_select >= spi->master->num_chipselect) {
		dev_err(&spi->dev,
			"setup: invalid chipselect %u (%u defined)\n",
			cs->chip_select, spi->master->num_chipselect);
		return -EINVAL;
	}

	cs->mode = spi->mode;

	if (cs->mode & SPI_CPHA)
		cfg |= HSSPI_BIT(PCC_CPHA);

	if (cs->mode & SPI_CPOL)
		cfg |= HSSPI_BIT(PCC_CPOL);

	if (cs->mode & SPI_CS_HIGH)
		cfg |= HSSPI_BIT(PCC_SSPOL);

	if (cs->mode & SPI_LSB_FIRST)
		cfg |= HSSPI_BIT(PCC_SDIR);

	cfg |= HSSPI_BITS(PCC_SS2CD, HS_SPI_CS_DELAY);
	cfg |= HSSPI_BITS(PCC_SENDIAN, 1);

	switch (cs->chip_select) {
	case 0:
		hs_spi_writel(hs, PCC0, cfg);
		HS_SPI_DEBUG(&spi->dev, "Spi mode is set to %#x\n",
			hs_spi_readl(hs, PCC0) & HS_SPI_MODE_MASK);
		break;
	case 1:
		hs_spi_writel(hs, PCC1, cfg);
		HS_SPI_DEBUG(&spi->dev, "Spi mode is set to %#x\n",
			hs_spi_readl(hs, PCC1) & HS_SPI_MODE_MASK);
		break;
	case 2:
		hs_spi_writel(hs, PCC2, cfg);
		HS_SPI_DEBUG(&spi->dev, "Spi mode is set to %#x\n",
			hs_spi_readl(hs, PCC2) & HS_SPI_MODE_MASK);
		break;
	case 3:
		hs_spi_writel(hs, PCC3, cfg);
		HS_SPI_DEBUG(&spi->dev, "Spi mode is set to %#x\n",
			hs_spi_readl(hs, PCC3) & HS_SPI_MODE_MASK);
		break;
	default:
		dev_err(&spi->dev,
			"setup: invalid chipselect %u (%u defined)\n",
			cs->chip_select, spi->master->num_chipselect);
		return -EINVAL;
	}

	retval = hs_spi_setup_transfer(spi, NULL);
	if (retval < 0)
		return retval;

	if (mode == HS_SPI_COMMAND_SEQUENCER)
		return hs_spi_cs_initialize_device(hs, spi);

	return 0;
}

/*
 * hs_spi_setup - clean driver specific data
 * @spi:	SPI device data.
 *
 * callback for spi framework.
 */
static void hs_spi_cleanup(struct spi_device *spi)
{
	struct hs_spi	*hs = spi_master_get_devdata(spi->master);

	HSSPI_BITS_SET(l, CSCFG_SSELEN, 0, hs, CSCFG);

	kfree(spi->controller_state);
}

/*
 * hs_spi_txrx - transmit or receive data
 * @spi:	SPI device data.
 * @t:		Transfer data.
 *
 * Called by spi bitbang.
 *
 * Returns transmitted or received data size
 */
static int hs_spi_txrx(struct spi_device *spi, struct spi_transfer *t)
{
	struct hs_spi	*hs = spi_master_get_devdata(spi->master);
	int		mode = hs->pdata->mode;
	if (mode == HS_SPI_COMMAND_SEQUENCER) {
		dev_err(hs->dev, "invalid hs spi mode, need direct mode\n");
		return -EINVAL;
	}

	hs->fault_flag	= 0;
	hs->tx		= t->tx_buf;
	hs->rx		= t->rx_buf;
	hs->tx_cnt	= 0;
	hs->rx_cnt	= 0;
	hs->len		= t->len;
	hs->bitwidth = t->bits_per_word;

	init_completion(&hs->done);

	/* Flush RX and TX FIFO  */
	HSSPI_BITS_SET(l, FIFOCFG_TXFLSH, 1, hs, FIFOCFG);
	HSSPI_BITS_SET(l, FIFOCFG_RXFLSH, 1, hs, FIFOCFG);

	if (hs->tx) {
		/* set tx transfer protocol */
		if (hs->bitwidth == 2) {
			HSSPI_BITS_SET(l, DMTRP_TRP, HS_SPI_DUAL_TX_ONLY,
				hs, DMTRP);
		} else if (hs->bitwidth == 4) {
			HSSPI_BITS_SET(l, DMTRP_TRP, HS_SPI_QUAD_TX_ONLY,
				hs, DMTRP);
	} else {
			HSSPI_BITS_SET(l, DMTRP_TRP, HS_SPI_LEGACY_TX_ONLY,
				hs, DMTRP);
		}
		hs->tx_cnt += hs_spi_write_tx_fifo(hs);
	} else {
		/* set rx transfer protocol */
		if (hs->bitwidth == 2) {
			HSSPI_BITS_SET(l, DMTRP_TRP, HS_SPI_DUAL_RX_ONLY,
				hs, DMTRP);
		} else if (hs->bitwidth == 4) {
			HSSPI_BITS_SET(l, DMTRP_TRP, HS_SPI_QUAD_RX_ONLY,
				hs, DMTRP);
		} else {
			HSSPI_BITS_SET(l, DMTRP_TRP, HS_SPI_LEGACY_RX_ONLY,
				hs, DMTRP);
		}
	}
	hs_spi_writel(hs, TXC, HS_SPI_TX_ALL_INT);
	hs_spi_writel(hs, RXC, HS_SPI_RX_ALL_INT);
	/* start transfer */
	HSSPI_BITS_SET(l, DMSTART_START, 1, hs, DMSTART);
	if (hs->tx)
		hs_spi_writel(hs, TXE, TXE_TFLETE_MASK << TXE_TFLETE_OFFSET |
					TXE_TFEE_MASK << TXE_TFEE_OFFSET);
	else
		hs_spi_writel(hs, RXE, RXE_RFMTE_MASK << RXE_RFMTE_OFFSET);

	wait_for_completion(&hs->done);

	if (hs->fault_flag)
		return -EPERM;

	return hs->rx ? hs->rx_cnt : hs->tx_cnt;
}

#ifdef CONFIG_PM
static int hs_spi_suspend(struct device *dev)
{
	struct hs_spi	*hs = platform_get_drvdata(to_platform_device(dev));
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&(hs->bitbang.lock), flags);

	hs->stop = true;

	hs->pcc[0] = hs_spi_readl(hs, PCC0);
	hs->pcc[1] = hs_spi_readl(hs, PCC1);
	hs->pcc[2] = hs_spi_readl(hs, PCC2);
	hs->pcc[3] = hs_spi_readl(hs, PCC3);

	spin_unlock_irqrestore(&hs->bitbang.lock, flags);

	clk_disable_unprepare(hs->clk);

	return ret;
}

static int hs_spi_resume(struct device *dev)
{
	struct hs_spi	*hs =
		platform_get_drvdata(to_platform_device(dev));
	int i;

	clk_prepare_enable(hs->clk);
	/* setup any hardware we can */
	hs_spi_init_hw(hs);

	/* PCC register restore */
	hs_spi_writel(hs, PCC0, hs->pcc[0]);
	hs_spi_writel(hs, PCC1, hs->pcc[1]);
	hs_spi_writel(hs, PCC2, hs->pcc[2]);
	hs_spi_writel(hs, PCC3, hs->pcc[3]);

	/* CS mode init */
	if (hs->pdata->mode == HS_SPI_COMMAND_SEQUENCER) {
		for (i = 0; i < 4; i++) {
			if (hs->spi[i])
				hs_spi_cs_initialize_device(hs, hs->spi[i]);
		}

	}
	hs->stop = false;

	return 0;
}

static int hs_spi_freeze(struct device *dev)
{
	struct hs_spi	*hs =
		platform_get_drvdata(to_platform_device(dev));
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&(hs->bitbang.lock), flags);
	hs->stop = true;
	spin_unlock_irqrestore(&hs->bitbang.lock, flags);

	return ret;
}

static int hs_spi_thaw(struct device *dev)
{
	struct hs_spi	*hs =
		platform_get_drvdata(to_platform_device(dev));

	hs->stop = false;

	return 0;
}

static int hs_spi_restore(struct device *dev)
{
	struct hs_spi	*hs =
		platform_get_drvdata(to_platform_device(dev));

	hs->stop = false;

	return 0;
}

static const struct dev_pm_ops hs_spi_pm_ops = {
	.suspend = hs_spi_suspend,
	.resume = hs_spi_resume,
	.freeze = hs_spi_freeze,
	.thaw = hs_spi_thaw,
	.restore = hs_spi_restore,
};

#define HS_SPI_PM_OPS	(&hs_spi_pm_ops)
#else
#define HS_SPI_PM_OPS	NULL
#endif

/* needs to be adapted for any change to the platform data struct */

static const char * const plat_data_names[] = {
	"mode",
	"num_chipselect",
	"addr_width",
	"bank_size",
	"clock",
	"sync_on",
};

/*
 * hs_spi_probe - probe the HS SPI
 * @pdev:	Platform device data.
 *
 * Returns 0 on success; negative errno on failure
 */
static int hs_spi_probe(struct platform_device *pdev)
{
	struct hs_spi		*hs;
	struct spi_master	*master;
	struct resource		*reg_res, *csa_res;
	int			i, err = 0;
	u32 *m;
	const u32 *p;
	int n;

	dev_dbg(&pdev->dev, "probing\n");

	master = spi_alloc_master(&pdev->dev, sizeof(struct hs_spi));
	if (master == NULL) {
		dev_err(&pdev->dev, "No memory for spi_master\n");
		err = -ENOMEM;
		goto err_nomem;
	}

	hs = spi_master_get_devdata(master);
	memset(hs, 0, sizeof(struct hs_spi));

	hs->master = spi_master_get(master);

	if (pdev->dev.of_node) {
		hs->pdata = kzalloc(sizeof(*hs->pdata), GFP_KERNEL);
		if (hs->pdata == NULL) {
			dev_err(&pdev->dev, "Out of memory\n");
			err = -ENOMEM;
			goto err_no_pdata;
		}

		m = &hs->pdata->mode;
		for (n = 0; n < ARRAY_SIZE(plat_data_names); n++) {
			p = of_get_property(pdev->dev.of_node,
					plat_data_names[n], NULL);
			if (p)
				*m++ = be32_to_cpu(*p);
		}
	} else {
		hs->pdata = pdev->dev.platform_data;
		if (hs->pdata == NULL) {
			dev_err(&pdev->dev, "No platform data supplied\n");
			err = -ENOENT;
			goto err_no_pdata;
		}
	}
	if ((hs->pdata->mode != HS_SPI_DIRECT_MODE) &&
		(hs->pdata->mode != HS_SPI_COMMAND_SEQUENCER)) {
		dev_err(&pdev->dev, "invalid hs spi mode\n");
		err = -ENOENT;
		goto err_pdata;
	}

	hs->dev		= &pdev->dev;
	hs->bank_size	= hs->pdata->bank_size;
	hs->stop = false;
	for (i = 0; i < 4; i++)
		hs->spi[i] = NULL;

	platform_set_drvdata(pdev, hs);
	init_completion(&hs->done);

	/* setup the master state. */
	master->mode_bits	= SPI_CPOL | SPI_CPHA |
		SPI_CS_HIGH | SPI_LSB_FIRST;
	master->num_chipselect	= hs->pdata->num_chipselect;
	master->bus_num		= pdev->id == -1 ? 0 : pdev->id;

	/* setup the state for the bitbang driver */

	hs->bitbang.master		= hs->master;
	hs->bitbang.setup_transfer	= hs_spi_setup_transfer;
	hs->bitbang.chipselect		= hs_spi_chipselect;
	hs->bitbang.txrx_bufs		= hs_spi_txrx;

	hs->master->setup		= hs_spi_setup;
	hs->master->cleanup		= hs_spi_cleanup;

	master->dev.of_node = pdev->dev.of_node;

	reg_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (reg_res == NULL) {
		dev_err(&pdev->dev, "Cannot get register IORESOURCE_MEM\n");
		err = -ENOENT;
		goto err_no_reg_iores;
	}

	hs->reg = ioremap(reg_res->start, resource_size(reg_res));
	if (hs->reg == NULL) {
		dev_err(&pdev->dev, "Cannot map register IO\n");
		err = -ENXIO;
		goto err_no_reg_iomap;
	}

	csa_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (csa_res == NULL) {
		dev_err(&pdev->dev, "Cannot get cmd seq area resource\n");
		err = -ENOENT;
		goto err_no_csa_iores;
	}
	hs->csa_size = resource_size(csa_res);
	hs->csa = ioremap(csa_res->start, resource_size(csa_res));
	if (hs->csa == NULL) {
		dev_err(&pdev->dev, "Cannot map command sequence area IO\n");
		err = -ENXIO;
		goto err_no_csa_iomap;
	}

	if (hs->pdata->clock == HS_SPI_HCLK)
		hs->clk = clk_get(&pdev->dev, "iHCLK");
	else
		hs->clk = clk_get(&pdev->dev, "iPCLK");
	if (IS_ERR(hs->clk)) {
		dev_err(&pdev->dev, "No clock for device\n");
		err = PTR_ERR(hs->clk);
		goto err_no_clk;
	}
	/* for the moment, permanently enable the clock */
	clk_prepare_enable(hs->clk);

	dev_dbg(&pdev->dev, "pre init\n");
	/* setup any hardware we can */
	n = hs_spi_init_hw(hs);
	if (n)
		return n;
	dev_dbg(&pdev->dev, "post init\n");

	/* find and request our interrupt resources */
	hs->tx_irq = platform_get_irq(pdev, 0);
	if (hs->tx_irq < 0) {
		dev_err(&pdev->dev, "No TX IRQ specified\n");
		err = -ENOENT;
		goto err_no_tx_irq;
	}

	err = request_irq(hs->tx_irq, hs_spi_tx_irq, 0, pdev->name, hs);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim TX IRQ\n");
		goto err_no_tx_irq;
	}

	hs->rx_irq = platform_get_irq(pdev, 1);
	if (hs->rx_irq < 0) {
		dev_err(&pdev->dev, "No RX IRQ specified\n");
		err = -ENOENT;
		goto err_no_rx_irq;
	}

	err = request_irq(hs->rx_irq, hs_spi_rx_irq, 0, pdev->name, hs);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim RX IRQ\n");
		goto err_no_rx_irq;
	}

	hs->fault_irq = platform_get_irq(pdev, 2);
	if (hs->fault_irq < 0) {
		dev_err(&pdev->dev, "No FAULT IRQ specified\n");
		err = -ENOENT;
		goto err_no_fault_irq;
	}

	err = request_irq(hs->fault_irq, hs_spi_fault_irq, 0, pdev->name, hs);
	if (err) {
		dev_err(&pdev->dev, "Cannot claim FAULT IRQ\n");
		goto err_no_fault_irq;
	}

	/* register our spi controller */
	dev_dbg(hs->dev, "bitbang at %p\n", &hs->bitbang);
	err = spi_bitbang_start(&hs->bitbang);
	if (err) {
		dev_err(&pdev->dev, "Failed to register SPI master\n");
		goto err_register;
	}

	return 0;

 err_register:
	clk_put(hs->clk);
	free_irq(hs->fault_irq, hs);

 err_no_fault_irq:
	free_irq(hs->rx_irq, hs);

 err_no_rx_irq:
	free_irq(hs->tx_irq, hs);

 err_no_tx_irq:
	clk_disable_unprepare(hs->clk);
	clk_put(hs->clk);

err_no_clk:
	iounmap(hs->csa);

 err_no_csa_iomap:
 err_no_csa_iores:
	iounmap(hs->reg);

 err_no_reg_iomap:
 err_no_reg_iores:
 err_pdata:
	if (pdev->dev.of_node)
		kfree(hs->pdata);
 err_no_pdata:
	spi_master_put(hs->master);

 err_nomem:
	return err;
}

/*
 * hs_spi_remove - remove the HS SPI
 * @pdev:	Platform device data.
 *
 * Stop hardware and remove the driver.
 *
 * Returns 0 on success
 */
static int __exit hs_spi_remove(struct platform_device *pdev)
{
	struct hs_spi		*hs = platform_get_drvdata(pdev);

	if (!hs)
		return 0;

	/* Disconnect from the SPI framework */
	spi_bitbang_stop(&hs->bitbang);

	/* clear platform driver data */
	platform_set_drvdata(pdev, NULL);

	clk_disable_unprepare(hs->clk);
	clk_put(hs->clk);

	free_irq(hs->fault_irq, hs);
	free_irq(hs->rx_irq, hs);
	free_irq(hs->tx_irq, hs);
	iounmap(hs->reg);
	iounmap(hs->csa);

	spi_master_put(hs->master);
	if (pdev->dev.of_node)
		kfree(hs->pdata);

	return 0;
}

static const struct of_device_id f_hsspi_dt_ids[] = {
	{ .compatible = "fujitsu,mb86s7x-hsspi" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, f_hsspi_dt_ids);

/*
 * Structure for a device driver
 */
static struct platform_driver hs_spi_driver = {
	.probe		= hs_spi_probe,
	.remove		= __exit_p(hs_spi_remove),
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = f_hsspi_dt_ids,
		.pm = HS_SPI_PM_OPS,
	},
};

/*
 * hs_spi_init - initialize module
 *
 * Returns 0 on success; negative errno on failure
 */
static int __init hs_spi_init(void)
{
	return platform_driver_register(&hs_spi_driver);
}

/*
 * hs_spi_exit - exit module
 */
static void __exit hs_spi_exit(void)
{
	platform_driver_unregister(&hs_spi_driver);
}

module_init(hs_spi_init);
module_exit(hs_spi_exit);

MODULE_DESCRIPTION("Fujitsu Semiconductor HS_SPI Driver");
MODULE_AUTHOR("Fujitsu Semiconductor Limitd");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:hs-spi");
