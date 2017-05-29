/*
 *  linux/arch/arm/mach-mb8ac0300/include/mach/hs_spi.h
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

#ifndef __MACH_HS_SPI_H
#define __MACH_HS_SPI_H

#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>

#define	HS_SPI_CS_DELAY		3	/* Slave-Select to Clock Delay */
#define	HS_SPI_FIFO_WIDTH	0	/* FIFO Width (1byte) */
#define	HS_SPI_TX_FIFO_LEVEL	1	/* TX-FIFO Threshold Level */
#define	HS_SPI_RX_FIFO_LEVEL	0	/* RX-FIFO Threshold Level */
#define	HS_SPI_FIFO_LEN		16	/* FIFO Length */

#define	HS_SPI_MAX_TRANSFER_LEN	0xFFFF	/* Max numbers of transfer bytes */
/*
 * HS_SPI Direct Mode Transfer Protocol
 */
#define	HS_SPI_LEGACY_TX_RX	0x00	/* TX and RX, in legacy mode */
#define	HS_SPI_LEGACY_RX_ONLY	0x04	/* RX only, in legacy mode */
#define	HS_SPI_DUAL_RX_ONLY	0x05	/* RX only, in dual mode */
#define	HS_SPI_QUAD_RX_ONLY	0x06	/* RX only, in quad mode */
#define	HS_SPI_LEGACY_TX_ONLY	0x08	/* TX only, in legacy mode */
#define	HS_SPI_DUAL_TX_ONLY	0x09	/* TX only, in dual mode */
#define	HS_SPI_QUAD_TX_ONLY	0x0A	/* TX only, in quad mode */

/*
 * HS_SPI Command Sequencer Mode
 */
#define	HS_SPI_IDLE_TIME	0xFFFF	/* Command Sequencer Idle Time */
#define	HS_SPI_CS_CONT_BIT	0x08	/* Command Sequencer Continuous bit */
#define	HS_SPI_CS_1BIT		(3<<1)	/* Command Sequencer Protocol 1bit*/
#define	HS_SPI_CS_2BIT		(2<<1)	/* Command Sequencer Protocol 2bit*/
#define	HS_SPI_CS_4BIT		(1<<1)	/* Command Sequencer Protocol 4bit*/

#define	HS_SPI_DECODE_ADDR3	0x0301	/* Transmit address bits [31:24] */
#define	HS_SPI_DECODE_ADDR2	0x0201	/* Transmit address bits [23:16] */
#define	HS_SPI_DECODE_ADDR1	0x0101	/* Transmit address bits [15:08] */
#define	HS_SPI_DECODE_ADDR0	0x0001	/* Transmit address bits [07:00] */
#define	HS_SPI_HIGH_Z_BYTE	0x0401	/* High-Z byte for 1 byte time */
#define	HS_SPI_LIST_END		0x0701	/* Command Sequencer List End */
/*
 * HS_SPI Command Sequencer Multi Bit Mode
 */
#define	HS_SPI_LEGACY_BIT	0x00	/* use the legacy SPI protocol */
#define	HS_SPI_DUAL_BIT		0x01	/* use the dual-bit SPI protocol */
#define	HS_SPI_QUAD_BIT		0x02	/* use the quad-bit SPI protocol */
#define	HS_SPI_CR_QUAD		0x02	/* Quad Mode Flag */
#define	HS_SPI_SR_WIP		0x01	/* Write In Progress Flag */
#define	HS_SPI_CONTINUOUS_READ_MODE 0xA0 /* Continuous read mode bits value */

/*
 * Flash operation codes
 */
#define	HS_SPI_CMD_RDID		0x9f	/* Read JEDEC ID */
#define	HS_SPI_CMD_NORM_READ	0x03	/* Read data bytes (low frequency) */
#define	HS_SPI_CMD_FAST_READ	0x0b	/* Read data bytes (high frequency) */
#define	HS_SPI_CMD_DOR		0x3B	/* Dual Output Read */
#define	HS_SPI_CMD_QOR		0x6B	/* Quad Output Read */
#define	HS_SPI_CMD_DIOR		0xBB	/* Dual I/O High Performance Read */
#define	HS_SPI_CMD_QIOR		0xEB	/* Quad I/O High Performance Read */
#define	HS_SPI_CMD_WREN		0x06	/* Write enable */
#define	HS_SPI_CMD_WRDN		0x04	/* Write Disable */
#define	HS_SPI_CMD_CHIP_ERASE	0xc7	/* Erase whole flash chip */
#define	HS_SPI_CMD_SE		0xd8	/* Sector erase (usually 64KiB) */
#define	HS_SPI_CMD_BE_4K	0x20	/* Erase 4KiB block */
#define	HS_SPI_CMD_PP		0x02	/* Page program (up to 256 bytes) */
#define	HS_SPI_CMD_QPP		0x32	/* Quad Page Programming */
#define	HS_SPI_CMD_RDSR		0x05	/* Read status register */
#define	HS_SPI_CMD_WRSR		0x01	/* Write status register 1 byte */
#define	HS_SPI_CMD_RCR		0x35	/* Read Configuration Register (CFG) */


#define	HS_SPI_DIRECT_MODE		1
#define	HS_SPI_COMMAND_SEQUENCER	2

#define	HS_SPI_HCLK			0
#define	HS_SPI_PCLK			1

/**
 * struct hs_spi_pdata - HS SPI device platform data
 * @mode: Direct mode or command sequencer mode.
 * @read_operation: Read operation for direct mode and command sequencer mode.
 * @write_operation: Write operation only for direct mode.
 * @num_chipselect: Max numbers of slaves can be selected.
 * @addr_width: Address width (number of bytes) for slaves.
 * @bits_per_word: Data transfers involve one or more words; word sizes
 * @bank_size: Bank size of each external memory devices.
 *
 * A @hs_spi_pdata is used by hs spi driver.
 */
struct hs_spi_pdata {
	int	mode;
	int	num_chipselect;		/* total chipselects */
	int	bank_size;		/* bank size */
	int	clock;
	int	syncon;
};

/* HS_SPI Controller driver's data. */

/**
 * struct hs_spi - HS_SPI Controller driver's data.
 * @csa: Command sequencer access area.
 *
 * Other members of the struct are private.
 *
 * A @csa is used by flash driver hs_spi_flash.
 */
struct hs_spi {
	/* private  */

	/* bitbang has to be first */
	struct spi_bitbang	bitbang;
	struct completion	done;

	int			tx_irq;
	int			rx_irq;
	int			fault_irq;
	unsigned int		len;
	unsigned int		tx_cnt;
	unsigned int		rx_cnt;
	int			fault_flag;
	int			bank_size;

	int			stop;

	/* data buffers */
	const unsigned char	*tx;
	unsigned char		*rx;

	struct spi_master	*master;

	struct clk		*clk;
	struct device		*dev;
	struct hs_spi_pdata	*pdata;

	void __iomem		*reg;

	/* external */
	void __iomem		*csa;
	unsigned int		csa_size;

	struct spi_device	*spi[4];
	unsigned int		pcc[4];
	/* spi transfer protocol */
	unsigned int		bitwidth;
};

#endif /* __MACH_HS_SPI_H */
