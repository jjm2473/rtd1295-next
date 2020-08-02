// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek Digital Home Center SD
 *
 * Copyright (C) 2017 Realtek Ltd.
 * Copyright (c) 2020 Andreas Färber
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/timer.h>

#define REG_SYS_PLL_SD1		0x1e0
#define REG_SYS_PLL_SD2		0x1e4
#define REG_SYS_PLL_SD3		0x1e8
#define REG_SYS_PLL_SD4		0x1ec

#define SYS_PLL_SD1_BIAS_EN			BIT(0)
#define SYS_PLL_SD1_PHRT0			BIT(1)
#define SYS_PLL_SD1_PHRSTB_DLY_SEL		BIT(2)
#define SYS_PLL_SD1_PHSEL0			GENMASK(7, 3)
#define SYS_PLL_SD1_PHSEL1			GENMASK(12, 8)
#define RTD119X_SYS_PLL_SD1_REGPD_D3318		BIT(16)
#define RTD119X_SYS_PLL_SD1_REG_D3318POW	GENMASK(18, 17)
#define RTD119X_SYS_PLL_SD1_REG_TUNED3318	GENMASK(21, 19)
#define RTD129X_SYS_PLL_SD1_REG_SEL3318		GENMASK(14, 13)

#define SYS_PLL_SD2_SSCLDO_EN			BIT(0)
#define SYS_PLL_SD2_REG_TUNE11			GENMASK(2, 1)
#define SYS_PLL_SD2_SSCPLL_CS1			GENMASK(4, 3)
#define SYS_PLL_SD2_SSCPLL_ICP			GENMASK(9, 5)
#define SYS_PLL_SD2_SSCPLL_RS			GENMASK(12, 10)
#define SYS_PLL_SD2_SSC_DEPTH			GENMASK(15, 13)
#define SYS_PLL_SD2_SSC_8X_EN			BIT(16)
#define SYS_PLL_SD2_SSC_DIV_F_SEL		BIT(17)
#define SYS_PLL_SD2_SSC_DIV_EXT_F		GENMASK(25, 18)
#define SYS_PLL_SD2_EN_CPNEW			BIT(26)

#define SYS_PLL_SD3_SSC_TBASE			GENMASK(7, 0)
#define SYS_PLL_SD3_SSC_STEP_IN			GENMASK(14, 8)
#define SYS_PLL_SD3_SSC_DIV_N			GENMASK(25, 16)

#define SYS_PLL_SD4_SSC_RSTB			BIT(0)
#define SYS_PLL_SD4_SSC_PLL_RSTB		BIT(1)
#define SYS_PLL_SD4_SSC_PLL_POW			BIT(2)

#define REG_SB2_SYNC		0x020

#define REG_SD_DMA_CTL1			0x04
#define REG_SD_DMA_CTL2			0x08
#define REG_SD_DMA_CTL3			0x0c
#define REG_SD_DMA_RST			0x20
#define REG_SD_ISR			0x24
#define REG_SD_ISREN			0x28
#define RTD129X_REG_SD_DUMMY_SYS	0x2c
#define REG_SD_PAD_CTL			0x74
#define REG_SD_CKGEN_CTL		0x78

#define REG_SD_CR_CARD_STOP		0x103
#define REG_SD_CR_CARD_OE		0x104
#define REG_SD_CARD_SELECT		0x10e
#define REG_SD_CARD_EXIST		0x11f
#define REG_SD_CARD_CLOCK_EN_CTL	0x129
#define REG_SD_CONFIGURE1		0x180
#define REG_SD_CONFIGURE2		0x181
#define REG_SD_CONFIGURE3		0x182
#define REG_SD_STATUS2			0x184
#define REG_SD_BUS_STATUS		0x185
#define REG_SD_SAMPLE_POINT_CTL		0x187
#define REG_SD_PUSH_POINT_CTL		0x188
#define REG_SD_CMD0			0x189
#define REG_SD_CMD1			0x18a
#define REG_SD_CMD2			0x18b
#define REG_SD_CMD3			0x18c
#define REG_SD_CMD4			0x18d
#define REG_SD_CMD5			0x18e
#define REG_SD_BYTE_CNT_L		0x18f
#define REG_SD_BYTE_CNT_H		0x190
#define REG_SD_BLK_CNT_L		0x191
#define REG_SD_BLK_CNT_H		0x192
#define REG_SD_TRANSFER			0x193
#define REG_SD_BUS_TA_STATE		0x197

#define SD_DMA_CTL1_DRAM_SA		GENMASK(29, 0)

#define SD_DMA_CTL2_DMA_LEN		GENMASK(15, 0)

#define SD_DMA_CTL3_DMA_XFER		BIT(0)
#define SD_DMA_CTL3_DDR_WR		BIT(1)
#define SD_DMA_CTL3_RSP17_SEL		BIT(4)
#define SD_DMA_CTL3_DAT64_SEL		BIT(5)

#define SD_DMA_RST_DMA_RSTN			BIT(0)
#define SD_DMA_RST_L4_GATED_DISABLE		BIT(1)
#define SD_DMA_RST_DBUS_ENDIAN_SEL		BIT(2)
#define SD_DMA_RST_DBUS_ENDIAN_SEL_LITTLE	FIELD_PREP(SD_DMA_RST_DBUS_ENDIAN_SEL, 0)
#define SD_DMA_RST_DBUS_ENDIAN_SEL_BIG		FIELD_PREP(SD_DMA_RST_DBUS_ENDIAN_SEL, 1)

#define SD_PAD_CTL_TUNE3318		BIT(0)
#define SD_PAD_CTL_TUNE3318_18V		FIELD_PREP(SD_PAD_CTL_TUNE3318, 0)
#define SD_PAD_CTL_TUNE3318_33V		FIELD_PREP(SD_PAD_CTL_TUNE3318, 1)

#define SD_CKGEN_CTL_CLK_DIV				GENMASK(2, 0)
#define SD_CKGEN_CTL_CLK_DIV_DIV1			FIELD_PREP(SD_CKGEN_CTL_CLK_DIV, 0)
#define SD_CKGEN_CTL_CLK_DIV_DIV2			FIELD_PREP(SD_CKGEN_CTL_CLK_DIV, 1)
#define SD_CKGEN_CTL_CLK_DIV_DIV4			FIELD_PREP(SD_CKGEN_CTL_CLK_DIV, 2)
#define SD_CKGEN_CTL_CLK_DIV_DIV8			FIELD_PREP(SD_CKGEN_CTL_CLK_DIV, 3)
#define SD_CKGEN_CTL_CRC_CLK_SRC			GENMASK(5, 4)
#define SD_CKGEN_CTL_CRC_CLK_SRC_SSC_CLK		FIELD_PREP(SD_CKGEN_CTL_CRC_CLK_SRC, 0)
#define SD_CKGEN_CTL_CRC_CLK_SRC_SSC_CLK_VP0		FIELD_PREP(SD_CKGEN_CTL_CRC_CLK_SRC, 1)
#define SD_CKGEN_CTL_CRC_CLK_SRC_SSC_CLK_VP1		FIELD_PREP(SD_CKGEN_CTL_CRC_CLK_SRC, 2)
#define SD_CKGEN_CTL_SD30_PUSH_CLK_SRC			GENMASK(9, 8)
#define SD_CKGEN_CTL_SD30_PUSH_CLK_SRC_SSC_CLK		FIELD_PREP(SD_CKGEN_CTL_SD30_PUSH_CLK_SRC, 0)
#define SD_CKGEN_CTL_SD30_PUSH_CLK_SRC_SSC_CLK_VP0	FIELD_PREP(SD_CKGEN_CTL_SD30_PUSH_CLK_SRC, 1)
#define SD_CKGEN_CTL_SD30_PUSH_CLK_SRC_SSC_CLK_VP1	FIELD_PREP(SD_CKGEN_CTL_SD30_PUSH_CLK_SRC, 2)
#define SD_CKGEN_CTL_SD30_SAMPLE_CLK_SRC		GENMASK(13, 12)
#define SD_CKGEN_CTL_SD30_SAMPLE_CLK_SRC_SSC_CLK	FIELD_PREP(SD_CKGEN_CTL_SD30_SAMPLE_CLK_SRC, 0)
#define SD_CKGEN_CTL_SD30_SAMPLE_CLK_SRC_SSC_CLK_VP0	FIELD_PREP(SD_CKGEN_CTL_SD30_SAMPLE_CLK_SRC, 1)
#define SD_CKGEN_CTL_SD30_SAMPLE_CLK_SRC_SSC_CLK_VP1	FIELD_PREP(SD_CKGEN_CTL_SD30_SAMPLE_CLK_SRC, 2)
#define SD_CKGEN_CTL_CRC_CLK_CHANGE			BIT(16)
#define SD_CKGEN_CTL_CRC_CLK_CHANGE_SRC			FIELD_PREP(SD_CKGEN_CTL_CRC_CLK_CHANGE, 0)
#define SD_CKGEN_CTL_CRC_CLK_CHANGE_4M			FIELD_PREP(SD_CKGEN_CTL_CRC_CLK_CHANGE, 1)
#define SD_CKGEN_CTL_SD30_PUSH_CHANGE			BIT(17)
#define SD_CKGEN_CTL_SD30_PUSH_CHANGE_SRC		FIELD_PREP(SD_CKGEN_CTL_SD30_PUSH_CHANGE, 0)
#define SD_CKGEN_CTL_SD30_PUSH_CHANGE_4M		FIELD_PREP(SD_CKGEN_CTL_SD30_PUSH_CHANGE, 1)
#define SD_CKGEN_CTL_SD30_SAMPLE_CHANGE			BIT(18)
#define SD_CKGEN_CTL_SD30_SAMPLE_CHANGE_SRC		FIELD_PREP(SD_CKGEN_CTL_SD30_SAMPLE_CHANGE, 0)
#define SD_CKGEN_CTL_SD30_SAMPLE_CHANGE_4M		FIELD_PREP(SD_CKGEN_CTL_SD30_SAMPLE_CHANGE, 1)

#define SD_CARD_EXIST_EXISTENCE			BIT(2)
#define SD_CARD_EXIST_WRITE_PROTECT		BIT(5)

#define SD_CONFIGURE1_BUS_WIDTH			GENMASK(1, 0)
#define SD_CONFIGURE1_BUS_WIDTH_1		FIELD_PREP(SD_CONFIGURE1_BUS_WIDTH, 0)
#define SD_CONFIGURE1_BUS_WIDTH_4		FIELD_PREP(SD_CONFIGURE1_BUS_WIDTH, 1)
#define SD_CONFIGURE1_BUS_WIDTH_8		FIELD_PREP(SD_CONFIGURE1_BUS_WIDTH, 2)
#define SD_CONFIGURE1_MODE_SEL			GENMASK(3, 2)
#define SD_CONFIGURE1_MODE_SEL_SD20		FIELD_PREP(SD_CONFIGURE1_MODE_SEL, 0)
#define SD_CONFIGURE1_MODE_SEL_DDR		FIELD_PREP(SD_CONFIGURE1_MODE_SEL, 1)
#define SD_CONFIGURE1_MODE_SEL_SD30		FIELD_PREP(SD_CONFIGURE1_MODE_SEL, 2)
#define SD_CONFIGURE1_RST_RDWR_FIFO		BIT(4)
#define SD_CONFIGURE1_SD30_ASYNC_FIFO_RST	SD_CONFIGURE1_RST_RDWR_FIFO
#define SD_CONFIGURE1_SDCLK_DIV_256		BIT(6)
#define SD_CONFIGURE1_SDCLK_DIV			BIT(7)

#define SD_CONFIGURE2_RESP_TYPE			GENMASK(1, 0)
#define SD_CONFIGURE2_RESP_TYPE_NONE		FIELD_PREP(SD_CONFIGURE2_RESP_TYPE, 0)
#define SD_CONFIGURE2_RESP_TYPE_6B		FIELD_PREP(SD_CONFIGURE2_RESP_TYPE, 1)
#define SD_CONFIGURE2_RESP_TYPE_17B		FIELD_PREP(SD_CONFIGURE2_RESP_TYPE, 2)
#define SD_CONFIGURE2_CRC7_CHK_DIS		BIT(2)
#define SD_CONFIGURE2_WAIT_BUSY_EN		BIT(3)
#define SD_CONFIGURE2_CRC16_CAL_DIS		BIT(6)
#define SD_CONFIGURE2_CRC7_CAL_DIS		BIT(7)

#define SD_BUS_STATUS_CMD_LEVEL			BIT(0)
#define SD_BUS_STATUS_DAT3_0_LEVEL		GENMASK(4, 1)
#define SD_BUS_STATUS_CLK_TOG_STOP		BIT(6)
#define SD_BUS_STATUS_CLK_TOG_EN		BIT(7)

#define SD_TRANSFER_CMD_CODE			GENMASK(3, 0)
#define SD_TRANSFER_CMD_CODE_NORMALWRITE	FIELD_PREP(SD_TRANSFER_CMD_CODE, 0x0)
#define SD_TRANSFER_CMD_CODE_AUTOWRITE3		FIELD_PREP(SD_TRANSFER_CMD_CODE, 0x1)
#define SD_TRANSFER_CMD_CODE_AUTOWRITE4		FIELD_PREP(SD_TRANSFER_CMD_CODE, 0x2)
#define SD_TRANSFER_CMD_CODE_AUTOREAD3		FIELD_PREP(SD_TRANSFER_CMD_CODE, 0x5)
#define SD_TRANSFER_CMD_CODE_AUTOREAD4		FIELD_PREP(SD_TRANSFER_CMD_CODE, 0x6)
#define SD_TRANSFER_CMD_CODE_SENDCMDGETRSP	FIELD_PREP(SD_TRANSFER_CMD_CODE, 0x8)
#define SD_TRANSFER_CMD_CODE_AUTOWRITE1		FIELD_PREP(SD_TRANSFER_CMD_CODE, 0x9)
#define SD_TRANSFER_CMD_CODE_AUTOWRITE2		FIELD_PREP(SD_TRANSFER_CMD_CODE, 0xa)
#define SD_TRANSFER_CMD_CODE_NORMALREAD		FIELD_PREP(SD_TRANSFER_CMD_CODE, 0xc)
#define SD_TRANSFER_CMD_CODE_AUTOREAD1		FIELD_PREP(SD_TRANSFER_CMD_CODE, 0xd)
#define SD_TRANSFER_CMD_CODE_AUTOREAD2		FIELD_PREP(SD_TRANSFER_CMD_CODE, 0xe)
#define SD_TRANSFER_CMD_CODE_UNKNOWN		FIELD_PREP(SD_TRANSFER_CMD_CODE, 0xf)
#define SD_TRANSFER_ERR_STATUS			BIT(4)
#define SD_TRANSFER_IDLE_STATE			BIT(5)
#define SD_TRANSFER_END_STATE			BIT(6)
#define SD_TRANSFER_START_EN			BIT(7)

#define DHC_MMC_ERR_RMOVE	3
#define DHC_MMC_ERR_FAILED	4

#define RTD129X_A00_PLL_QUIRK			BIT(0)
#define RTD129X_NON_A00_PROLOG_EPILOG_QUIRK	BIT(1)

#define SD_ALLOC_LEN	2048

struct dhc_sdmmc_priv;

struct dhc_sdmmc_info {
	int (*pre_init)(struct dhc_sdmmc_priv *priv);
	int (*post_add)(struct dhc_sdmmc_priv *priv);
	int (*pre_remove)(struct dhc_sdmmc_priv *priv);
};

struct dhc_sdmmc_priv {
	struct device *dev;
	struct mmc_host *mmc;
	const struct dhc_sdmmc_info *info;
	void __iomem *base;
	struct clk *clk, *clk_ip;
	struct reset_control *rstc;
	struct pinctrl *pctrl;
	struct pinctrl_state *card_pctrl;
	struct regmap *crt, *sb2;
	unsigned int quirks;
	bool powered;
	bool card_detected;
	bool card_selected;
	bool mmc_detected;

	void *dma_mem;
	dma_addr_t dma_handle;

	struct completion sent;
	struct tasklet_struct req_end_tasklet;
	struct mmc_request *req;

	struct timer_list plug_timer;
	u8 last_card_exist;
};

static int dhc_sdmmc_sync(struct dhc_sdmmc_priv *priv)
{
	return regmap_write(priv->sb2, REG_SB2_SYNC, 0);
}

static inline void rtd129x_prolog(struct dhc_sdmmc_priv *priv)
{
	if (priv->quirks & RTD129X_NON_A00_PROLOG_EPILOG_QUIRK)
		writel(0x40000000, priv->base + RTD129X_REG_SD_DUMMY_SYS);
}

static inline void rtd129x_epilog(struct dhc_sdmmc_priv *priv)
{
	if (priv->quirks & RTD129X_NON_A00_PROLOG_EPILOG_QUIRK)
		writel(0x00000000, priv->base + RTD129X_REG_SD_DUMMY_SYS);
}

static void dhc_sdmmc_set_bus_width(struct dhc_sdmmc_priv *priv, unsigned int bits)
{
	u8 val;

	val = readb(priv->base + REG_SD_CONFIGURE1);
	val &= ~SD_CONFIGURE1_BUS_WIDTH;
	switch (bits) {
	case 4:
		val |= SD_CONFIGURE1_BUS_WIDTH_4;
		break;
	case 8:
		val |= SD_CONFIGURE1_BUS_WIDTH_8;
		break;
	case 1:
	default:
		val |= SD_CONFIGURE1_BUS_WIDTH_1;
		break;
	}
	writeb(val, priv->base + REG_SD_CONFIGURE1);
}

static void dhc_sdmmc_set_mode(struct dhc_sdmmc_priv *priv, u8 mode)
{
	u8 val;

	val = readb(priv->base + REG_SD_CONFIGURE1);
	val &= ~SD_CONFIGURE1_MODE_SEL;
	val |= mode;
	if (val == SD_CONFIGURE1_MODE_SEL_SD30)
		val |= SD_CONFIGURE1_SD30_ASYNC_FIFO_RST;
	writeb(val, priv->base + REG_SD_CONFIGURE1);
}

static int dhc_sdmmc_set_power(struct dhc_sdmmc_priv *priv, bool power)
{
	int ret;

	if (priv->powered == power)
		return 0;

	if (power) {
		//dev_info(priv->dev, "Selecting card state\n");
		ret = pinctrl_select_state(priv->pctrl, priv->card_pctrl);
		if (ret) {
			dev_warn(priv->dev, "Failed to select card state (%d)\n", ret);
			return ret;
		}
		mdelay(10);
	} else {
		//dev_info(priv->dev, "Selecting default state\n");
		ret = pinctrl_select_default_state(priv->dev);
		if (ret) {
			dev_warn(priv->dev, "Failed to select default state (%d)\n", ret);
			return ret;
		}
	}
	priv->powered = power;
	return 0;
}

static int dhc_sdmmc_set_speed(struct dhc_sdmmc_priv *priv, unsigned int speed)
{
	u32 val;
	u8 b;
	int ret;

	//dev_info(priv->dev, "Speed: %u\n", speed);

	if (speed == 50000000 && priv->mmc->ios.timing == MMC_TIMING_UHS_DDR50) {
		val = readl(priv->base + REG_SD_CKGEN_CTL);
		val |= SD_CKGEN_CTL_SD30_SAMPLE_CHANGE_4M |
		       SD_CKGEN_CTL_SD30_PUSH_CHANGE_4M |
		       SD_CKGEN_CTL_CRC_CLK_CHANGE_4M;
		writel(val, priv->base + REG_SD_CKGEN_CTL);

		rtd129x_prolog(priv);
		if (unlikely(priv->quirks & RTD129X_A00_PLL_QUIRK)) {
		}
		mdelay(100);
		ret = regmap_read(priv->crt, REG_SYS_PLL_SD4, &val);
		if (ret)
			return ret;
		val &= ~SYS_PLL_SD4_SSC_RSTB;
		ret = regmap_write(priv->crt, REG_SYS_PLL_SD4, val);
		if (ret)
			return ret;
		val = FIELD_PREP(SYS_PLL_SD3_SSC_DIV_N, 0x32) |
		      FIELD_PREP(SYS_PLL_SD3_SSC_STEP_IN, 0x43) |
		      FIELD_PREP(SYS_PLL_SD3_SSC_TBASE, 0x88);
		ret = regmap_write(priv->crt, REG_SYS_PLL_SD3, val);
		if (ret)
			return ret;
		mdelay(2);
		ret = regmap_read(priv->crt, REG_SYS_PLL_SD4, &val);
		if (ret)
			return ret;
		val |= SYS_PLL_SD4_SSC_RSTB;
		ret = regmap_write(priv->crt, REG_SYS_PLL_SD4, val);
		if (ret)
			return ret;
		udelay(200);
		rtd129x_epilog(priv);

		val = readl(priv->base + REG_SD_CKGEN_CTL);
		val &= ~(SD_CKGEN_CTL_SD30_SAMPLE_CHANGE |
			 SD_CKGEN_CTL_SD30_PUSH_CHANGE |
			 SD_CKGEN_CTL_CRC_CLK_CHANGE);
		val |= SD_CKGEN_CTL_SD30_SAMPLE_CHANGE_SRC |
		       SD_CKGEN_CTL_SD30_PUSH_CHANGE_SRC |
		       SD_CKGEN_CTL_CRC_CLK_CHANGE_SRC;
		writel(val, priv->base + REG_SD_CKGEN_CTL);

		b = readb(priv->base + REG_SD_CONFIGURE1);
		b &= ~SD_CONFIGURE1_RST_RDWR_FIFO;
		writeb(b, priv->base + REG_SD_CONFIGURE1);
	} else if (speed == 100000000) {
		// TODO pinctrl
	} else if (speed == 208000000) {
		// TODO pinctrl
		rtd129x_prolog(priv);
		ret = regmap_read(priv->crt, REG_SYS_PLL_SD4, &val);
		if (ret)
			return ret;
		val &= ~SYS_PLL_SD4_SSC_RSTB;
		ret = regmap_write(priv->crt, REG_SYS_PLL_SD4, val);
		if (ret)
			return ret;
		val = readl(priv->base + REG_SD_CKGEN_CTL);
		val |= SD_CKGEN_CTL_SD30_SAMPLE_CHANGE_4M |
		       SD_CKGEN_CTL_SD30_PUSH_CHANGE_4M |
		       SD_CKGEN_CTL_CRC_CLK_CHANGE_4M;
		writel(val, priv->base + REG_SD_CKGEN_CTL);
		mdelay(2);
		ret = regmap_read(priv->crt, REG_SYS_PLL_SD4, &val);
		if (ret)
			return ret;
		val |= SYS_PLL_SD4_SSC_RSTB;
		ret = regmap_write(priv->crt, REG_SYS_PLL_SD4, val);
		if (ret)
			return ret;
		udelay(200);
		rtd129x_epilog(priv);

		rtd129x_prolog(priv);
		if (unlikely(priv->quirks & RTD129X_A00_PLL_QUIRK)) {
		}
		mdelay(100);
		ret = regmap_read(priv->crt, REG_SYS_PLL_SD4, &val);
		if (ret)
			return ret;
		val &= ~SYS_PLL_SD4_SSC_RSTB;
		ret = regmap_write(priv->crt, REG_SYS_PLL_SD4, val);
		if (ret)
			return ret;
		val = FIELD_PREP(SYS_PLL_SD3_SSC_DIV_N, 0xb6) |
		      FIELD_PREP(SYS_PLL_SD3_SSC_STEP_IN, 0x43) |
		      FIELD_PREP(SYS_PLL_SD3_SSC_TBASE, 0x88);
		ret = regmap_write(priv->crt, REG_SYS_PLL_SD3, val);
		if (ret)
			return ret;
		mdelay(2);
		ret = regmap_read(priv->crt, REG_SYS_PLL_SD4, &val);
		if (ret)
			return ret;
		val |= SYS_PLL_SD4_SSC_RSTB;
		ret = regmap_write(priv->crt, REG_SYS_PLL_SD4, val);
		if (ret)
			return ret;
		udelay(200);
		rtd129x_epilog(priv);

		rtd129x_prolog(priv);
		ret = regmap_read(priv->crt, REG_SYS_PLL_SD4, &val);
		if (ret)
			return ret;
		val &= ~SYS_PLL_SD4_SSC_RSTB;
		ret = regmap_write(priv->crt, REG_SYS_PLL_SD4, val);
		if (ret)
			return ret;
		val = readl(priv->base + REG_SD_CKGEN_CTL);
		val &= ~(SD_CKGEN_CTL_SD30_SAMPLE_CHANGE |
			 SD_CKGEN_CTL_SD30_PUSH_CHANGE |
			 SD_CKGEN_CTL_CRC_CLK_CHANGE);
		val |= SD_CKGEN_CTL_SD30_SAMPLE_CHANGE_SRC |
		       SD_CKGEN_CTL_SD30_PUSH_CHANGE_SRC |
		       SD_CKGEN_CTL_CRC_CLK_CHANGE_SRC;
		writel(val, priv->base + REG_SD_CKGEN_CTL);
		mdelay(2);
		ret = regmap_read(priv->crt, REG_SYS_PLL_SD4, &val);
		if (ret)
			return ret;
		val |= SYS_PLL_SD4_SSC_RSTB;
		ret = regmap_write(priv->crt, REG_SYS_PLL_SD4, val);
		if (ret)
			return ret;
		udelay(200);
		rtd129x_epilog(priv);

		b = readb(priv->base + REG_SD_CONFIGURE1);
		b &= ~SD_CONFIGURE1_RST_RDWR_FIFO;
		writeb(b, priv->base + REG_SD_CONFIGURE1);
	}

	ret = dhc_sdmmc_sync(priv);
	if (ret)
		return ret;

	switch (speed) {
	case 6200000:
	case 25000000:
	case 50000000:
	case 100000000:
	case 208000000:
		b = readb(priv->base + REG_SD_CONFIGURE1);
		b &= ~(SD_CONFIGURE1_SDCLK_DIV | SD_CONFIGURE1_SDCLK_DIV_256);
		writeb(b, priv->base + REG_SD_CONFIGURE1);
		break;
	case 200000:
	case 400000:
	default:
		b = readb(priv->base + REG_SD_CONFIGURE1);
		b |= SD_CONFIGURE1_SDCLK_DIV | SD_CONFIGURE1_SDCLK_DIV_256;
		writeb(b, priv->base + REG_SD_CONFIGURE1);
		break;
	}

	ret = dhc_sdmmc_sync(priv);
	if (ret)
		return ret;

	val = readl(priv->base + REG_SD_CKGEN_CTL);
	val &= ~(SD_CKGEN_CTL_SD30_SAMPLE_CHANGE |
		 SD_CKGEN_CTL_SD30_PUSH_CHANGE |
		 SD_CKGEN_CTL_CRC_CLK_CHANGE |
		 SD_CKGEN_CTL_SD30_SAMPLE_CLK_SRC |
		 SD_CKGEN_CTL_SD30_PUSH_CLK_SRC |
		 SD_CKGEN_CTL_CRC_CLK_SRC |
		 SD_CKGEN_CTL_CLK_DIV);
	val |= SD_CKGEN_CTL_SD30_SAMPLE_CHANGE_SRC |
	       SD_CKGEN_CTL_SD30_PUSH_CHANGE_SRC |
	       SD_CKGEN_CTL_CRC_CLK_CHANGE_SRC |
	       SD_CKGEN_CTL_SD30_SAMPLE_CLK_SRC_SSC_CLK_VP1 |
	       SD_CKGEN_CTL_SD30_PUSH_CLK_SRC_SSC_CLK_VP0 |
	       SD_CKGEN_CTL_CRC_CLK_SRC_SSC_CLK;
	switch (speed) {
	case 200000:
		val |= SD_CKGEN_CTL_CLK_DIV_DIV2;
		break;
	case 6200000:
		val |= SD_CKGEN_CTL_CLK_DIV_DIV8;
		break;
	case 25000000:
		if (priv->mmc->ios.timing == MMC_TIMING_UHS_SDR12)
			val |= SD_CKGEN_CTL_CLK_DIV_DIV4;
		else
			val |= SD_CKGEN_CTL_CLK_DIV_DIV2;
		break;
	case 50000000:
		if (priv->mmc->ios.timing == MMC_TIMING_UHS_SDR25)
			val |= SD_CKGEN_CTL_CLK_DIV_DIV2;
		else
			val |= SD_CKGEN_CTL_CLK_DIV_DIV1;
		break;
	case 100000000:
		val |= SD_CKGEN_CTL_CLK_DIV_DIV1;
		break;
	case 208000000:
		val |= SD_CKGEN_CTL_CLK_DIV_DIV1;
		break;
	case 400000:
	default:
		val |= SD_CKGEN_CTL_CLK_DIV_DIV1;
		break;
	}
	writel(val, priv->base + REG_SD_CKGEN_CTL);

	return dhc_sdmmc_sync(priv);
}

static void dhc_sdmmc_reset(struct dhc_sdmmc_priv *priv)
{
	writel(0, priv->base + REG_SD_DMA_CTL3);
	writel(0x16, priv->base + REG_SD_ISREN);
	writel(0x7, priv->base + REG_SD_ISREN);

	writeb(0, priv->base + REG_SD_TRANSFER);
	writeb(0xff, priv->base + REG_SD_CR_CARD_STOP);
	writeb(0x00, priv->base + REG_SD_CR_CARD_STOP);
}

static u8 dhc_sdmmc_command_code(struct dhc_sdmmc_priv *priv, struct mmc_command *cmd)
{
	if (cmd->opcode == 8 && priv->mmc_detected)
		return SD_TRANSFER_CMD_CODE_NORMALREAD;

	if (cmd->data->flags & MMC_DATA_WRITE) {
		switch (cmd->opcode) {
		case 42:
			return SD_TRANSFER_CMD_CODE_NORMALWRITE;
		case 56:
			return SD_TRANSFER_CMD_CODE_AUTOWRITE2;
		default:
			break;
		}
	}

	switch (cmd->opcode) {
	case 6:
	case 13:
	case 22:
	case 30:
	case 31:
	case 42:
	case 51:
		return SD_TRANSFER_CMD_CODE_NORMALREAD;
	case 18:
		return SD_TRANSFER_CMD_CODE_AUTOREAD1;
	case 17:
	case 48:
	case 56:
	case 58:
		return SD_TRANSFER_CMD_CODE_AUTOREAD2;
	case 26:
	case 27:
		return SD_TRANSFER_CMD_CODE_NORMALWRITE;
	case 20:
	case 25: //XXX
		return SD_TRANSFER_CMD_CODE_AUTOWRITE1;
	case 24:
	//case 25:
	case 49:
	case 59:
		return SD_TRANSFER_CMD_CODE_AUTOWRITE2;
	default:
		return SD_TRANSFER_CMD_CODE_UNKNOWN;
	}
}

static u8 dhc_sdmmc_resp_type_to_configure2(struct mmc_command *cmd)
{
	unsigned int resp_type = mmc_resp_type(cmd);
	u8 val = 0;

	if (resp_type & MMC_RSP_136)
		return SD_CONFIGURE2_RESP_TYPE_17B;

	if (!(resp_type & MMC_RSP_CRC))
		val |= SD_CONFIGURE2_CRC7_CHK_DIS;

	if (resp_type & MMC_RSP_PRESENT)
		val |= SD_CONFIGURE2_RESP_TYPE_6B;
	else
		val |= SD_CONFIGURE2_RESP_TYPE_NONE;

	return val;
}

static int dhc_sdmmc_transfer(struct dhc_sdmmc_priv *priv, struct mmc_command *cmd,
	u32 dma_buffer, u8 cmdcode, u16 byte_count, u16 block_count)
{
	u32 val;
	u8 b;

	dev_info(priv->dev, "%s\n", __func__);

	writeb_relaxed(BIT(6) | cmd->opcode, priv->base + REG_SD_CMD0);
	writeb_relaxed(cmd->arg >> 24, priv->base + REG_SD_CMD1);
	writeb_relaxed(cmd->arg >> 16, priv->base + REG_SD_CMD2);
	writeb_relaxed(cmd->arg >> 8, priv->base + REG_SD_CMD3);
	writeb_relaxed(cmd->arg, priv->base + REG_SD_CMD4);
	writeb_relaxed(0x0, priv->base + REG_SD_CMD5);

	b = dhc_sdmmc_resp_type_to_configure2(cmd);
	writeb_relaxed(b, priv->base + REG_SD_CONFIGURE2);

	writeb_relaxed(byte_count,      priv->base + REG_SD_BYTE_CNT_L);
	writeb_relaxed(byte_count >> 8, priv->base + REG_SD_BYTE_CNT_H);
	writeb_relaxed(block_count,      priv->base + REG_SD_BLK_CNT_L);
	writeb_relaxed(block_count >> 8, priv->base + REG_SD_BLK_CNT_H);

	val = FIELD_PREP(SD_DMA_CTL1_DRAM_SA, dma_buffer >> 3);
	writel_relaxed(val, priv->base + REG_SD_DMA_CTL1);
	val = FIELD_PREP(SD_DMA_CTL2_DMA_LEN, block_count);
	writel_relaxed(val, priv->base + REG_SD_DMA_CTL2);
	val = 0;
	if ((byte_count == 17) || ((b & SD_CONFIGURE2_RESP_TYPE) == SD_CONFIGURE2_RESP_TYPE_17B))
		val |= SD_DMA_CTL3_RSP17_SEL;
	else if (byte_count == 64)
		val |= SD_DMA_CTL3_DAT64_SEL;
	if ((cmd->data && (cmd->data->flags & MMC_DATA_READ)) ||
	    (cmdcode == SD_TRANSFER_CMD_CODE_SENDCMDGETRSP))
		val |= SD_DMA_CTL3_DDR_WR;
	if ((cmd->data && (cmd->data->flags & (MMC_DATA_READ | MMC_DATA_WRITE))) ||
	    (cmdcode == SD_TRANSFER_CMD_CODE_SENDCMDGETRSP))
		val |= SD_DMA_CTL3_DMA_XFER;
	writel_relaxed(val, priv->base + REG_SD_DMA_CTL3);

	writel(0x16, priv->base + REG_SD_ISREN);
	reinit_completion(&priv->sent);
	writel(0x6 | 1, priv->base + REG_SD_ISREN);

	dhc_sdmmc_sync(priv);

	b = SD_TRANSFER_START_EN | (cmdcode & SD_TRANSFER_CMD_CODE);
	writeb(b, priv->base + REG_SD_TRANSFER);

	wait_for_completion_timeout(&priv->sent, msecs_to_jiffies(3000));

	b = readb(priv->base + REG_SD_TRANSFER);
	if (!((b & SD_TRANSFER_END_STATE) && (b & SD_TRANSFER_IDLE_STATE))) {
		dev_warn(priv->dev, "timeout!\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int dhc_sdmmc_stream(struct dhc_sdmmc_priv *priv, struct mmc_command *cmd, struct mmc_data *data)
{
	enum dma_data_direction dma_dir;
	unsigned int dma_addr, dma_len;
	struct scatterlist *sg;
	unsigned int arg;
	int i, ret = 0, nents;
	u16 byte_count, block_count;
	u8 cmdcode;

	dev_info(priv->dev, "%s\n", __func__);

	arg = cmd->arg;
	cmdcode = dhc_sdmmc_command_code(priv, cmd);

	dma_dir = (data->flags & MMC_DATA_READ) ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
	nents = dma_map_sg(priv->dev, data->sg, data->sg_len, dma_dir);
	data->bytes_xfered = 0;

	for (i = 0; i < nents; i++) {
		sg = &data->sg[i];
		dma_addr = sg_dma_address(sg);
		dma_len = sg_dma_len(sg);

		if ((cmd->opcode == SD_SWITCH && (cmd->flags & MMC_CMD_ADTC)) ||
		    (cmd->opcode == SD_APP_SD_STATUS && (cmd->flags & MMC_CMD_MASK) == MMC_CMD_ADTC)) {
			byte_count = 64;
			block_count = dma_len / 64;
		} else if (cmd->opcode == MMC_SEND_TUNING_BLOCK ||
			   cmd->opcode == SD_APP_SEND_SCR ||
			   cmd->opcode == SD_APP_SEND_NUM_WR_BLKS) {
			byte_count = 64;
			block_count = 1;
		} else if (cmd->opcode == MMC_ALL_SEND_CID ||
			   cmd->opcode == MMC_SEND_CSD ||
			   cmd->opcode == MMC_SEND_CID) {
			byte_count = 17;
			block_count = dma_len / 17;
		} else {
			byte_count = 512;
			block_count = dma_len / 512;
		}
		if (block_count == 0 && dma_len)
			block_count = 1;

		ret = dhc_sdmmc_transfer(priv, cmd, dma_addr, cmdcode, byte_count, block_count);
		if (ret) {
			cmd->arg = arg;
		} else {
			cmd->arg += dma_len;
			data->bytes_xfered += dma_len;
		}
	}

	dma_unmap_sg(priv->dev, data->sg, data->sg_len, dma_dir);

	return ret;
}

static void dhc_sdmmc_send_command(struct dhc_sdmmc_priv *priv, struct mmc_command *cmd)
{
	u32 dma_buffer;
	int ret;

	dev_info(priv->dev, "%s, opcode=%u\n", __func__, (u32)cmd->opcode);

	if (cmd->opcode == MMC_SEND_OP_COND)
		priv->mmc_detected = true;

	if (cmd->opcode == SD_SWITCH_VOLTAGE) {
		/* force clock disable after cmd11 */
		writeb(0x40, priv->base + REG_SD_BUS_STATUS);
	}

	if (cmd->data) {
		if (true)
			ret = dhc_sdmmc_stream(priv, cmd, cmd->data);
	} else {
		if (false) // XXX RTD119x
			dma_buffer = ((unsigned long)priv->dma_mem) & ~0xff;
		else
			dma_buffer = priv->dma_handle;

		ret = dhc_sdmmc_transfer(priv, cmd, dma_buffer, SD_TRANSFER_CMD_CODE_SENDCMDGETRSP, 512, 1);
	}

	if (cmd->opcode == MMC_SELECT_CARD) {
		priv->card_selected = true;
		dhc_sdmmc_set_speed(priv, 6200000);
	}

	if (ret) {
		if (cmd->opcode == 49 || cmd->opcode == 59 || cmd->opcode == 48 || cmd->opcode == 58) {
			dhc_sdmmc_reset(priv);
			cmd->error = -110;
		} else
			cmd->error = -DHC_MMC_ERR_FAILED;
	} else
		cmd->error = -DHC_MMC_ERR_FAILED;
}

static void dhc_sdmmc_request(struct mmc_host *host, struct mmc_request *req)
{
	struct dhc_sdmmc_priv *priv = mmc_priv(host);
	struct mmc_command *cmd = req->cmd;
	u8 card_cmd;

	WARN_ON(priv->req);
	priv->req = req;

	if (cmd->opcode == 48 || cmd->opcode == 49 ||
	    cmd->opcode == 58 || cmd->opcode == 59) {
		card_cmd = host->card->raw_scr[0] & 0xf;
		if (((cmd->opcode == 58 || cmd->opcode == 59) && !(card_cmd & BIT(3))) ||
		    ((cmd->opcode == 48 || cmd->opcode == 49) && !(card_cmd & BIT(3)) && !(card_cmd & BIT(2)))) {
			cmd->error = -110;
			goto out;
		}
	}
	if (!priv->card_detected) {
		cmd->error = -DHC_MMC_ERR_RMOVE;
		cmd->retries = 0;
		goto out;
	}

	dhc_sdmmc_send_command(priv, cmd);

out:
	tasklet_schedule(&priv->req_end_tasklet);
}

static void dhc_sdmmc_req_end(unsigned long data)
{
	struct dhc_sdmmc_priv *priv = (void *)data;
	struct mmc_request *req = priv->req;

	priv->req = NULL;
	mmc_request_done(priv->mmc, req);
}

static int dhc_sdmmc_get_cd(struct mmc_host *host)
{
	struct dhc_sdmmc_priv *priv = mmc_priv(host);
	u8 val;

	val = readb(priv->base + REG_SD_CARD_EXIST);
	return (val & SD_CARD_EXIST_EXISTENCE) ? 1 : 0;
}

static int dhc_sdmmc_get_ro(struct mmc_host *host)
{
	struct dhc_sdmmc_priv *priv = mmc_priv(host);
	u8 val;

	val = readb(priv->base + REG_SD_CARD_EXIST);
	return (val & SD_CARD_EXIST_WRITE_PROTECT) ? 1 : 0;
}

static void dhc_sdmmc_set_ios(struct mmc_host *host, struct mmc_ios *ios)
{
	struct dhc_sdmmc_priv *priv = mmc_priv(host);
	unsigned int speed = 400000, bits = 1;
	u8 mode = SD_CONFIGURE1_MODE_SEL_SD20;

	if (ios->bus_mode == MMC_BUSMODE_PUSHPULL) {
		if (ios->bus_width == MMC_BUS_WIDTH_8)
			bits = 8;
		else if (ios->bus_width == MMC_BUS_WIDTH_4)
			bits = 4;

		if (ios->clock >= UHS_SDR104_MAX_DTR && ios->timing == MMC_TIMING_UHS_SDR104) {
			mode = SD_CONFIGURE1_MODE_SEL_SD30;
			speed = 208000000;
		} else if (ios->clock >= UHS_SDR50_MAX_DTR && ios->timing == MMC_TIMING_UHS_SDR50) {
			mode = SD_CONFIGURE1_MODE_SEL_SD30;
			speed = 100000000;
		} else if (ios->clock >= UHS_SDR25_MAX_DTR && ios->timing == MMC_TIMING_UHS_SDR25) {
			mode = SD_CONFIGURE1_MODE_SEL_SD30;
			speed = 50000000;
		} else if (ios->clock >= UHS_SDR12_MAX_DTR && ios->timing == MMC_TIMING_UHS_SDR12) {
			mode = SD_CONFIGURE1_MODE_SEL_SD30;
			speed = 25000000;
		} else if (ios->clock >= UHS_DDR50_MAX_DTR && ios->timing == MMC_TIMING_UHS_DDR50) {
			mode = SD_CONFIGURE1_MODE_SEL_DDR;
			speed = 50000000;
		} else if (ios->clock >= HIGH_SPEED_MAX_DTR && ios->timing == MMC_TIMING_SD_HS)
			speed = 50000000;
		else if (ios->clock >= 25000000)
			speed = 25000000;
		else if (priv->card_selected)
			speed = 6200000;
	}

	dhc_sdmmc_set_bus_width(priv, bits);
	dhc_sdmmc_set_mode(priv, mode);
	dhc_sdmmc_set_speed(priv, speed);

	if (ios->power_mode == MMC_POWER_UP) {
		dev_info(mmc_dev(host), "Powering up\n");
		dhc_sdmmc_set_power(priv, true);
	} else if (ios->power_mode == MMC_POWER_OFF) {
		dev_info(mmc_dev(host), "Powering down\n");
		dhc_sdmmc_set_power(priv, false);
	}
}

static int dhc_sdmmc_wait_signal_voltage(struct dhc_sdmmc_priv *priv, bool level)
{
	u8 val, mask = SD_BUS_STATUS_DAT3_0_LEVEL | SD_BUS_STATUS_CMD_LEVEL;
	u8 expected = level ? mask : 0;
	int i = 0;

	do {
		val = readb(priv->base + REG_SD_BUS_STATUS);
		if ((val & mask) == expected)
			return 0;
		msleep(3);
	} while (i++ < 300);
	return -ETIMEDOUT;
}

static int dhc_sdmmc_start_signal_voltage_switch(struct mmc_host *host, struct mmc_ios *ios)
{
	struct dhc_sdmmc_priv *priv = mmc_priv(host);
	u32 val;
	int ret;

	dev_info(priv->dev, "%s\n", __func__);

	if (ios->signal_voltage != MMC_SIGNAL_VOLTAGE_330) {
		ret = dhc_sdmmc_wait_signal_voltage(priv, false);
		if (ret)
			return ret;

		writeb(0x3b, priv->base + REG_SD_CARD_CLOCK_EN_CTL);
		mdelay(10);

		// XXX
		writel(0, priv->base + REG_SD_PAD_CTL);

		ret = dhc_sdmmc_sync(priv);

		val = SYS_PLL_SD1_PHRT0 | SYS_PLL_SD1_BIAS_EN;
		if (false)
			val |= FIELD_PREP(RTD119X_SYS_PLL_SD1_REG_D3318POW, 0x3) |
			       FIELD_PREP(RTD119X_SYS_PLL_SD1_REG_TUNED3318, 0x1);
		ret = regmap_write(priv->crt, REG_SYS_PLL_SD1, val);
		if (true) {
			msleep(500);
			val |= FIELD_PREP(RTD129X_SYS_PLL_SD1_REG_SEL3318, 0x2);
			ret = regmap_write(priv->crt, REG_SYS_PLL_SD1, val);
		}
		msleep(10);

		writeb(0x3b | BIT(2), priv->base + REG_SD_CARD_CLOCK_EN_CTL);
		writeb(SD_BUS_STATUS_CLK_TOG_EN, priv->base + REG_SD_BUS_STATUS);

		ret = dhc_sdmmc_sync(priv);

		ret = dhc_sdmmc_wait_signal_voltage(priv, true);
		if (ret)
			return ret;
	}

	writeb(0, priv->base + REG_SD_BUS_STATUS);

	return 0;
}

static int dhc_sdmmc_init(struct dhc_sdmmc_priv *priv)
{
	unsigned int val;
	u8 b;
	int ret;

	ret = regmap_read(priv->crt, REG_SYS_PLL_SD4, &val);
	if (ret)
		return ret;

	val |= SYS_PLL_SD4_SSC_PLL_POW;
	ret = regmap_write(priv->crt, REG_SYS_PLL_SD4, val);
	if (ret)
		return ret;

	val |= SYS_PLL_SD4_SSC_PLL_RSTB | SYS_PLL_SD4_SSC_RSTB;
	ret = regmap_write(priv->crt, REG_SYS_PLL_SD4, val);
	if (ret)
		return ret;

	val = SYS_PLL_SD1_PHRT0 | SYS_PLL_SD1_BIAS_EN;
	if (false) // XXX RTD119x
		val |= FIELD_PREP(RTD119X_SYS_PLL_SD1_REG_D3318POW, 0x3) |
		       FIELD_PREP(RTD119X_SYS_PLL_SD1_REG_TUNED3318, 0x7);
	ret = regmap_write(priv->crt, REG_SYS_PLL_SD1, val);
	if (ret)
		return ret;

	mdelay(10);
	if (true) { // XXX !RTD119x
		val |= FIELD_PREP(RTD129X_SYS_PLL_SD1_REG_SEL3318, 1);
		ret = regmap_write(priv->crt, REG_SYS_PLL_SD1, val);
		if (ret)
			return ret;
	}

	rtd129x_prolog(priv);
	udelay(100);
	ret = regmap_read(priv->crt, REG_SYS_PLL_SD4, &val);
	if (ret)
		return ret;
	val &= ~SYS_PLL_SD4_SSC_RSTB;
	ret = regmap_write(priv->crt, REG_SYS_PLL_SD4, val);
	if (ret)
		return ret;
	val = readl(priv->base + REG_SD_CKGEN_CTL);
	val |= SD_CKGEN_CTL_SD30_SAMPLE_CHANGE_4M |
	       SD_CKGEN_CTL_SD30_PUSH_CHANGE_4M |
	       SD_CKGEN_CTL_CRC_CLK_CHANGE_4M;
	writel(val, priv->base + REG_SD_CKGEN_CTL);
	mdelay(2);
	ret = regmap_read(priv->crt, REG_SYS_PLL_SD4, &val);
	if (ret)
		return ret;
	val |= SYS_PLL_SD4_SSC_RSTB;
	ret = regmap_write(priv->crt, REG_SYS_PLL_SD4, val);
	if (ret)
		return ret;
	udelay(200);
	rtd129x_epilog(priv);

	rtd129x_prolog(priv);
	udelay(100);
	ret = regmap_read(priv->crt, REG_SYS_PLL_SD4, &val);
	if (ret)
		return ret;
	val &= ~SYS_PLL_SD4_SSC_RSTB;
	ret = regmap_write(priv->crt, REG_SYS_PLL_SD4, val);
	if (ret)
		return ret;
	val = SYS_PLL_SD2_EN_CPNEW |
	      FIELD_PREP(SYS_PLL_SD2_SSC_DIV_EXT_F, 0x14) |
	      SYS_PLL_SD2_SSC_8X_EN |
	      FIELD_PREP(SYS_PLL_SD2_SSC_DEPTH, 2) | /* 3 -> 2 "for passing the EMI" */
	      FIELD_PREP(SYS_PLL_SD2_SSCPLL_RS, 6) |
	      FIELD_PREP(SYS_PLL_SD2_SSCPLL_ICP, 4) |
	      FIELD_PREP(SYS_PLL_SD2_SSCPLL_CS1, 2) |
	      FIELD_PREP(SYS_PLL_SD2_REG_TUNE11, 1) |
	      SYS_PLL_SD2_SSCLDO_EN;
	ret = regmap_write(priv->crt, REG_SYS_PLL_SD2, val);
	if (ret)
		return ret;
	val = FIELD_PREP(SYS_PLL_SD3_SSC_DIV_N, 0x56) |
	      FIELD_PREP(SYS_PLL_SD3_SSC_STEP_IN, 0x43) |
	      FIELD_PREP(SYS_PLL_SD3_SSC_TBASE, 0x88);
	ret = regmap_write(priv->crt, REG_SYS_PLL_SD3, val); // 100 MHz
	if (ret)
		return ret;
	mdelay(2);
	ret = regmap_read(priv->crt, REG_SYS_PLL_SD4, &val);
	if (ret)
		return ret;
	val |= SYS_PLL_SD4_SSC_RSTB;
	ret = regmap_write(priv->crt, REG_SYS_PLL_SD4, val);
	if (ret)
		return ret;
	udelay(200);
	rtd129x_epilog(priv);

	if (true) { // XXX !RTD119x
		val = readl(priv->base + REG_SD_DMA_RST);
		val |= SD_DMA_RST_L4_GATED_DISABLE;
		writel(val, priv->base + REG_SD_DMA_RST);
	}
	writeb(0x3, priv->base + REG_SD_BUS_TA_STATE);
	if (true) { // XXX !RTD119x
		val = readl(priv->base + REG_SD_DMA_RST);
		val &= ~SD_DMA_RST_DMA_RSTN;
		writel(val, priv->base + REG_SD_DMA_RST);
		val |= SD_DMA_RST_DMA_RSTN;
		writel(val, priv->base + REG_SD_DMA_RST);
	}

	rtd129x_prolog(priv);
	udelay(100);
	ret = regmap_read(priv->crt, REG_SYS_PLL_SD4, &val);
	if (ret)
		return ret;
	val &= ~SYS_PLL_SD4_SSC_RSTB;
	ret = regmap_write(priv->crt, REG_SYS_PLL_SD4, val);
	if (ret)
		return ret;
	val = readl(priv->base + REG_SD_CKGEN_CTL);
	val &= ~(SD_CKGEN_CTL_SD30_SAMPLE_CHANGE |
		 SD_CKGEN_CTL_SD30_PUSH_CHANGE |
		 SD_CKGEN_CTL_CRC_CLK_CHANGE);
	val |= SD_CKGEN_CTL_SD30_SAMPLE_CHANGE_SRC |
	       SD_CKGEN_CTL_SD30_PUSH_CHANGE_SRC |
	       SD_CKGEN_CTL_CRC_CLK_CHANGE_SRC;
	writel(val, priv->base + REG_SD_CKGEN_CTL);
	udelay(100);
	ret = regmap_read(priv->crt, REG_SYS_PLL_SD4, &val);
	if (ret)
		return ret;
	val |= SYS_PLL_SD4_SSC_RSTB;
	ret = regmap_write(priv->crt, REG_SYS_PLL_SD4, val);
	if (ret)
		return ret;
	udelay(200);
	rtd129x_epilog(priv);

	b = readb(priv->base + REG_SD_CONFIGURE1);
	b &= ~SD_CONFIGURE1_RST_RDWR_FIFO;
	writeb(b, priv->base + REG_SD_CONFIGURE1);
	// XXX duplicate?
	ret = regmap_write(priv->crt, REG_SYS_PLL_SD4, val);
	if (ret)
		return ret;
	udelay(100);
	b = SD_CONFIGURE1_SDCLK_DIV | SD_CONFIGURE1_SDCLK_DIV_256 | SD_CONFIGURE1_RST_RDWR_FIFO;
	writeb(b, priv->base + REG_SD_CONFIGURE1);

	ret = dhc_sdmmc_set_speed(priv, 400000);
	if (ret)
		return ret;

	// XXX pinctrl

	val = readl(priv->base + REG_SD_PAD_CTL);
	val &= ~SD_PAD_CTL_TUNE3318;
	val |= SD_PAD_CTL_TUNE3318_18V;
	writel(val, priv->base + REG_SD_PAD_CTL);

	writel(0x16, priv->base + REG_SD_ISR);
	writel(0x16, priv->base + REG_SD_ISREN);
	writel(0x07, priv->base + REG_SD_ISREN);

	ret = dhc_sdmmc_sync(priv);
	if (ret)
		return ret;

	writeb(0x2, priv->base + REG_SD_CARD_SELECT);
	writeb(0x0, priv->base + REG_SD_SAMPLE_POINT_CTL);
	writeb(0x0, priv->base + REG_SD_PUSH_POINT_CTL);

	return 0;
}

static void dhc_sdmmc_hw_reset(struct mmc_host *host)
{
	struct dhc_sdmmc_priv *priv = mmc_priv(host);
	u8 b;
	int ret;

	dev_info(mmc_dev(host), "Resetting\n");

	priv->card_selected = false;

	ret = dhc_sdmmc_init(priv);

	writeb(0xff, priv->base + REG_SD_CR_CARD_STOP);
	writeb(0x00, priv->base + REG_SD_CR_CARD_STOP);
	writeb(0x02, priv->base + REG_SD_CARD_SELECT); // 2:0
	writeb(BIT(2), priv->base + REG_SD_CR_CARD_OE);
	writeb(BIT(2), priv->base + REG_SD_CARD_CLOCK_EN_CTL);
	b = SD_CONFIGURE1_SDCLK_DIV | SD_CONFIGURE1_SDCLK_DIV_256 | SD_CONFIGURE1_RST_RDWR_FIFO;
	writeb(b, priv->base + REG_SD_CONFIGURE1);
	writeb(0x00, priv->base + REG_SD_STATUS2);

	dhc_sdmmc_set_power(priv, false);
	dhc_sdmmc_set_power(priv, true);

	priv->mmc_detected = false;

	ret = dhc_sdmmc_sync(priv);
}

static const struct mmc_host_ops dhc_sdmmc_ops = {
	.request			= dhc_sdmmc_request,
	.get_cd				= dhc_sdmmc_get_cd,
	.get_ro				= dhc_sdmmc_get_ro,
	.set_ios			= dhc_sdmmc_set_ios,
	.hw_reset			= dhc_sdmmc_hw_reset,
	.start_signal_voltage_switch	= dhc_sdmmc_start_signal_voltage_switch,
};

static irqreturn_t dhc_sdmmc_irq(int irq, void *dev_id)
{
	struct dhc_sdmmc_priv *priv = dev_id;
	u32 val;

	dhc_sdmmc_sync(priv);
	val = readl(priv->base + REG_SD_ISR);
	dev_info(priv->dev, "%s ISR=%08x\n", __func__, val);
	complete(&priv->sent);
	writel(val, priv->base + REG_SD_ISR);
	return IRQ_HANDLED;
}

static void rtd119x_sdmmc_plug(struct timer_list *timer)
{
	struct dhc_sdmmc_priv *priv = from_timer(priv, timer, plug_timer);
	u8 val, tmp, last = priv->last_card_exist;

	val = readb(priv->base + REG_SD_CARD_EXIST);
	if ((val & SD_CARD_EXIST_EXISTENCE) ^ (last & SD_CARD_EXIST_EXISTENCE)) {
		priv->card_detected = !!(val & SD_CARD_EXIST_EXISTENCE);
		if (priv->card_detected) {
			dev_info(mmc_dev(priv->mmc), "card detected\n");
			dhc_sdmmc_set_power(priv, true);

			writel(0x0, priv->base + REG_SD_DMA_CTL3);
			writeb(0x00, priv->base + REG_SD_CR_CARD_STOP);
			writeb(0x02, priv->base + REG_SD_CARD_SELECT);
			writeb(BIT(2), priv->base + REG_SD_CR_CARD_OE);
			writeb(BIT(2), priv->base + REG_SD_CARD_CLOCK_EN_CTL);
			tmp = SD_CONFIGURE1_SDCLK_DIV | SD_CONFIGURE1_SDCLK_DIV_256 | SD_CONFIGURE1_RST_RDWR_FIFO;
			writeb(tmp, priv->base + REG_SD_CONFIGURE1);
			dhc_sdmmc_set_speed(priv, 400000);
			writeb(0x00, priv->base + REG_SD_STATUS2);
		} else {
			dev_info(mmc_dev(priv->mmc), "card gone\n");
			// TODO pinctrl
			if (false) { // RTD119x DMA reset quirk
			}
			writel(0x0, priv->base + REG_SD_DMA_CTL3);
			writeb(0xff, priv->base + REG_SD_CR_CARD_STOP);
			writeb(0x3b, priv->base + REG_SD_CARD_CLOCK_EN_CTL);

			dhc_sdmmc_set_power(priv, false);
		}
		dhc_sdmmc_sync(priv);
		priv->card_selected = false;
		mmc_detect_change(priv->mmc, msecs_to_jiffies(1));
	}
	priv->last_card_exist = val;

	mod_timer(&priv->plug_timer, jiffies + msecs_to_jiffies(100));
}

static int rtd119x_sdmmc_pre_init(struct dhc_sdmmc_priv *priv)
{
	timer_setup(&priv->plug_timer, rtd119x_sdmmc_plug, 0);
	priv->last_card_exist = ~(u8)SD_CARD_EXIST_EXISTENCE;

	return 0;
}

static int rtd119x_sdmmc_post_add(struct dhc_sdmmc_priv *priv)
{
	mod_timer(&priv->plug_timer, jiffies + 3 * HZ);

	return 0;
}

static int rtd119x_sdmmc_pre_remove(struct dhc_sdmmc_priv *priv)
{
	del_timer_sync(&priv->plug_timer);

	return 0;
}

static const struct dhc_sdmmc_info rtd119x_sdmmc_info = {
	.pre_init = rtd119x_sdmmc_pre_init,
	.post_add = rtd119x_sdmmc_post_add,
	.pre_remove = rtd119x_sdmmc_pre_remove,
};

static int rtd129x_sdmmc_pre_init(struct dhc_sdmmc_priv *priv)
{
	unsigned int val;
	int ret;

	ret = rtd119x_sdmmc_pre_init(priv);
	if (ret)
		return ret;

	ret = regmap_read(priv->sb2, 0x204, &val);
	if (ret)
		dev_warn(priv->dev, "could not read chip_rev\n");
	else if ((val >> 16) == 0)
		priv->quirks |= RTD129X_A00_PLL_QUIRK;
	else
		priv->quirks |= RTD129X_NON_A00_PROLOG_EPILOG_QUIRK;

	return 0;
}

static int rtd129x_sdmmc_post_add(struct dhc_sdmmc_priv *priv)
{
	return rtd119x_sdmmc_post_add(priv);
}

static int rtd129x_sdmmc_pre_remove(struct dhc_sdmmc_priv *priv)
{
	return rtd119x_sdmmc_pre_remove(priv);
}

static const struct dhc_sdmmc_info rtd129x_sdmmc_info = {
	.pre_init	= rtd129x_sdmmc_pre_init,
	.post_add	= rtd129x_sdmmc_post_add,
	.pre_remove	= rtd129x_sdmmc_pre_remove,
};

static int dhc_sdmmc_probe(struct platform_device *pdev)
{
	struct dhc_sdmmc_priv *priv;
	struct mmc_host *mmc;
	int ret;

	mmc = mmc_alloc_host(sizeof(*priv), &pdev->dev);
	if (!mmc)
		return -ENOMEM;

	priv = mmc_priv(mmc);
	priv->mmc = mmc;
	priv->dev = &pdev->dev;

	priv->info = device_get_match_data(&pdev->dev);
	if (!priv->info) {
		ret = -EINVAL;
		goto err_match;
	}

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (!priv->base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		goto err_ioremap;
	}

	priv->rstc = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (IS_ERR(priv->rstc)) {
		ret = PTR_ERR(priv->rstc);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "reset failed (%d)\n", ret);
		goto err_rst;
	}

	priv->clk = devm_clk_get(&pdev->dev, "clk_en_cr");
	if (IS_ERR(priv->clk)) {
		ret = PTR_ERR(priv->clk);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "clk_en_cr failed (%d)\n", ret);
		goto err_clk;
	}

	priv->clk_ip = devm_clk_get(&pdev->dev, "clk_en_sd_ip");
	if (IS_ERR(priv->clk_ip)) {
		ret = PTR_ERR(priv->clk_ip);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "clk_en_sd_ip failed (%d)\n", ret);
		goto err_clk_ip;
	}

	priv->pctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(priv->pctrl)) {
		ret = PTR_ERR(priv->pctrl);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "pinctrl failed (%d)\n", ret);
		goto err_pctrl;
	}

	priv->card_pctrl = pinctrl_lookup_state(priv->pctrl, "card");
	if (IS_ERR(priv->card_pctrl)) {
		ret = PTR_ERR(priv->card_pctrl);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "pinctrl 'card' failed (%d)\n", ret);
		goto err_pctrl_card;
	}

	priv->crt = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "crt-syscon");
	if (IS_ERR(priv->crt)) {
		ret = PTR_ERR(priv->crt);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "crt syscon failed (%d)\n", ret);
		goto err_crt;
	}

	priv->sb2 = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "sb2-syscon");
	if (IS_ERR(priv->sb2)) {
		ret = PTR_ERR(priv->sb2);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "sb2 syscon failed (%d)\n", ret);
		goto err_sb2;
	}

	ret = mmc_of_parse(mmc);
	if (ret) {
		dev_err(&pdev->dev, "parse failed (%d)\n", ret);
		goto err_parse;
	}

	ret = reset_control_deassert(priv->rstc);
	if (ret) {
		dev_err(&pdev->dev, "reset failed (%d)\n", ret);
		goto err_reset;
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(&pdev->dev, "clk_en_cr enable failed (%d)\n", ret);
		goto err_enable;
	}

	ret = clk_prepare_enable(priv->clk_ip);
	if (ret) {
		dev_err(&pdev->dev, "clk_en_sd_ip enable failed (%d)\n", ret);
		goto err_enable_ip;
	}

	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		goto err_irq_get;
	ret = devm_request_irq(&pdev->dev, ret, dhc_sdmmc_irq, 0, dev_name(&pdev->dev), priv);
	if (ret < 0)
		goto err_irq_req;

	init_completion(&priv->sent);
	priv->card_detected = false;
	priv->card_selected = false;
	priv->mmc_detected = false;
	priv->quirks = 0;

	if (priv->info->pre_init) {
		ret = priv->info->pre_init(priv);
		if (ret)
			goto err_init_pre;
	}
	if (priv->quirks & RTD129X_A00_PLL_QUIRK)
		dev_info(&pdev->dev, "RTD129x A00 detected\n");
	if (priv->quirks & RTD129X_NON_A00_PROLOG_EPILOG_QUIRK)
		dev_info(&pdev->dev, "RTD129x non-A00 detected\n");

	platform_set_drvdata(pdev, mmc);

	ret = dhc_sdmmc_init(priv);
	if (ret)
		goto err_init;

	mmc->ocr_avail = MMC_VDD_30_31 |
			 MMC_VDD_31_32 |
			 MMC_VDD_33_34 |
			 MMC_VDD_165_195;
	mmc->caps = MMC_CAP_4_BIT_DATA |
		    MMC_CAP_SD_HIGHSPEED |
		    MMC_CAP_MMC_HIGHSPEED |
		    MMC_CAP_HW_RESET |
		    MMC_CAP_UHS_SDR12 |
		    MMC_CAP_UHS_SDR25 |
		    MMC_CAP_UHS_SDR50 |
		    MMC_CAP_UHS_SDR104;
	mmc->caps2 = MMC_CAP2_NO_SDIO;
	mmc->f_min = 10000000 >> 8; /* 10 MHz / 256 */
	mmc->f_max = 208000000; /* 208 MHz */
	mmc->max_segs = 1;
	mmc->max_blk_size = 512;
	mmc->max_blk_count = 0x1000;
	mmc->max_seg_size = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;
	mmc->ops = &dhc_sdmmc_ops;

	tasklet_init(&priv->req_end_tasklet, dhc_sdmmc_req_end, (unsigned long)priv);

	ret = dhc_sdmmc_sync(priv);
	if (ret)
		goto err_sync;

	ret = mmc_add_host(mmc);
	if (ret)
		goto err_add;

	priv->dma_mem = dma_alloc_coherent(&pdev->dev, SD_ALLOC_LEN, &priv->dma_handle, GFP_KERNEL);
	if (!priv->dma_mem) {
		ret = -ENOMEM;
		goto err_dma;
	}

	if (priv->info->post_add) {
		ret = priv->info->post_add(priv);
		if (ret)
			goto err_add_post;
	}

	dev_info(&pdev->dev, "probed\n");

	return 0;

err_add_post:
	dma_free_coherent(&pdev->dev, SD_ALLOC_LEN, priv->dma_mem, priv->dma_handle);
err_dma:
	mmc_remove_host(mmc);
err_add:
err_sync:
err_init:
	platform_set_drvdata(pdev, NULL);
err_init_pre:
err_irq_req:
err_irq_get:
	clk_disable_unprepare(priv->clk_ip);
err_enable_ip:
	clk_disable_unprepare(priv->clk);
err_enable:
	reset_control_assert(priv->rstc);
err_reset:
err_parse:
err_sb2:
err_crt:
err_pctrl_card:
err_pctrl:
err_clk_ip:
err_clk:
err_rst:
err_ioremap:
err_match:
	mmc_free_host(mmc);
	return ret;
}

static int dhc_sdmmc_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct dhc_sdmmc_priv *priv = mmc_priv(mmc);

	if (priv->info->pre_remove)
		priv->info->pre_remove(priv);

	mmc_remove_host(mmc);

	dma_free_coherent(&pdev->dev, SD_ALLOC_LEN, priv->dma_mem, priv->dma_handle);

	clk_disable_unprepare(priv->clk_ip);
	clk_disable_unprepare(priv->clk);
	reset_control_assert(priv->rstc);

	mmc_free_host(mmc);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id dhc_sdmmc_dt_ids[] = {
	 { .compatible = "realtek,rtd1195-sdmmc", .data = &rtd119x_sdmmc_info },
	 { .compatible = "realtek,rtd1295-sdmmc", .data = &rtd129x_sdmmc_info },
	 { }
};
MODULE_DEVICE_TABLE(of, dhc_sdmmc_dt_ids);

static struct platform_driver dhc_sdmmc_driver = {
	.probe = dhc_sdmmc_probe,
	.remove = dhc_sdmmc_remove,
	.driver = {
		.name = "rtk-dhc-sdmmc",
		.of_match_table	= dhc_sdmmc_dt_ids,
	},
};
module_platform_driver(dhc_sdmmc_driver);

MODULE_DESCRIPTION("Realtek DHC SD/MMC driver");
MODULE_LICENSE("GPL v2");
