/*
 * linux/drivers/i2c/busses/i2c-f_i2c.h
 *
 * Copyright (C) 2012 FUJITSU SEMICONDUCTOR LIMITED
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

#ifndef __I2C_F_I2C_H__
#define __I2C_F_I2C_H__

#define WAIT_PCLK(n, clkrate) ndelay((((1000000000 +  clkrate - 1) / \
						clkrate + n - 1) / n) + 10)
#define F_I2C_TIMEOUT(x) (msecs_to_jiffies(x))

/* I2C register adress definitions */
#define F_I2C_REG_BSR		(0x00 << 2) /* Bus Status Regster */
#define F_I2C_REG_BCR		(0x01 << 2) /* Bus Control Register */
#define F_I2C_REG_CCR		(0x02 << 2) /* Clock Control Register */
#define F_I2C_REG_ADR		(0x03 << 2) /* Address Register */
#define F_I2C_REG_DAR		(0x04 << 2) /* Data Register */
#define F_I2C_REG_CSR		(0x05 << 2) /* Expansion CS Register */
#define F_I2C_REG_FSR		(0x06 << 2) /* Bus Clock Frequency Register */
#define F_I2C_REG_BC2R		(0x07 << 2) /* Bus Control 2 Register */

/* I2C register bit definitions */
#define F_I2C_BSR_FBT		(1 << 0)  /* First Byte Transfer */
#define F_I2C_BSR_GCA		(1 << 1)  /* General Call Address */
#define F_I2C_BSR_AAS		(1 << 2)  /* Address as Slave */
#define F_I2C_BSR_TRX		(1 << 3)  /* Transfer/Receive */
#define F_I2C_BSR_LRB		(1 << 4)  /* Last Received Bit */
#define F_I2C_BSR_AL		(1 << 5)  /* Arbitration Lost */
#define F_I2C_BSR_RSC		(1 << 6)  /* Repeated Start Condition */
#define F_I2C_BSR_BB		(1 << 7)  /* Bus Busy */

#define F_I2C_BCR_INT		(1 << 0)  /* Interrupt */
#define F_I2C_BCR_INTE		(1 << 1)  /* Interrupt Enable */
#define F_I2C_BCR_GCAA		(1 << 2)  /* General Call Access Acknowledge */
#define F_I2C_BCR_ACK		(1 << 3)  /* Acknowledge */
#define F_I2C_BCR_MSS		(1 << 4)  /* Master Slave Select */
#define F_I2C_BCR_SCC		(1 << 5)  /* Start Condition Continue */
#define F_I2C_BCR_BEIE		(1 << 6)  /* Bus Error Interrupt Enable */
#define F_I2C_BCR_BER		(1 << 7)  /* Bus Error */

#define F_I2C_CCR_CS_MASK	(0x1f)  /* CCR Clock Period Select */
#define F_I2C_CCR_EN		(1 << 5)  /* Enable */
#define F_I2C_CCR_FM		(1 << 6)  /* Speed Mode Select */

#define F_I2C_CSR_CS_MASK	(0x3f)  /* CSR Clock Period Select */

#define F_I2C_BC2R_SCLL		(1 << 0)  /* SCL Low Drive */
#define F_I2C_BC2R_SDAL		(1 << 1)  /* SDA Low Drive */
#define F_I2C_BC2R_SCLS		(1 << 4)  /* SCL Status */
#define F_I2C_BC2R_SDAS		(1 << 5)  /* SDA Status */

/* PCLK frequency */
#define F_I2C_BUS_CLK_FR(clkrate)		((clkrate / 20000000) + 1)

/* STANDARD MODE frequency */
#define F_I2C_CLK_MASTER_STANDARD(clkrate) \
	DIV_ROUND_UP(DIV_ROUND_UP(clkrate, 100000) - 2, 2)
/* FAST MODE frequency */
#define F_I2C_CLK_MASTER_FAST(clkrate) \
	DIV_ROUND_UP((DIV_ROUND_UP(clkrate, 400000) - 2) * 2, 3)

/* (clkrate <= 18000000) */
/* calculate the value of CS bits in CCR register on standard mode */
#define F_I2C_CCR_CS_STANDARD_MAX_18M(clkrate) \
	   ((F_I2C_CLK_MASTER_STANDARD(clkrate) - 65) & F_I2C_CCR_CS_MASK)
/* calculate the value of CS bits in CSR register on standard mode */
#define F_I2C_CSR_CS_STANDARD_MAX_18M(clkrate)	0x00
/* calculate the value of CS bits in CCR register on fast mode */
#define F_I2C_CCR_CS_FAST_MAX_18M(clkrate) \
	   ((F_I2C_CLK_MASTER_FAST(clkrate) - 1)  & F_I2C_CCR_CS_MASK)
/* calculate the value of CS bits in CSR register on fast mode */
#define F_I2C_CSR_CS_FAST_MAX_18M(clkrate)	0x00

/* (clkrate > 18000000) */
/* calculate the value of CS bits in CCR register on standard mode */
#define F_I2C_CCR_CS_STANDARD_MIN_18M(clkrate) \
	   ((F_I2C_CLK_MASTER_STANDARD(clkrate) - 1) & F_I2C_CCR_CS_MASK)
/* calculate the value of CS bits in CSR register on standard mode */
#define F_I2C_CSR_CS_STANDARD_MIN_18M(clkrate) \
	   (((F_I2C_CLK_MASTER_STANDARD(clkrate) - 1) >> 5) & F_I2C_CSR_CS_MASK)
/* calculate the value of CS bits in CCR register on fast mode */
#define F_I2C_CCR_CS_FAST_MIN_18M(clkrate) \
	   ((F_I2C_CLK_MASTER_FAST(clkrate) - 1) & F_I2C_CCR_CS_MASK)
/* calculate the value of CS bits in CSR register on fast mode */
#define F_I2C_CSR_CS_FAST_MIN_18M(clkrate) \
	   (((F_I2C_CLK_MASTER_FAST(clkrate) - 1) >> 5) & F_I2C_CSR_CS_MASK)

/* min I2C clock frequency 14M */
#define F_I2C_MIN_CLK_RATE	(14 * 1000000)
/* max I2C clock frequency 200M */
#define F_I2C_MAX_CLK_RATE	(200 * 1000000)
/* I2C clock frequency 18M */
#define F_I2C_CLK_RATE_18M	(18 * 1000000)

#endif /* __I2C_F_I2C_H__ */
