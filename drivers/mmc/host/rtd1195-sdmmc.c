// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek Digital Home Center SD
 *
 * Copyright (C) 2017 Realtek Ltd.
 * Copyright (c) 2020 Andreas Färber
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/mmc/host.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/slab.h>

#define REG_SYS_PLL_SD1		0x1e0
#define REG_SYS_PLL_SD2		0x1e4
#define REG_SYS_PLL_SD3		0x1e8
#define REG_SYS_PLL_SD4		0x1ec

#define SYS_PLL_SD1_BIAS_EN			BIT(0)
#define SYS_PLL_SD1_PHRT0			BIT(1)
#define RTD119X_SYS_PLL_SD1_REGPD_D3318		BIT(16)
#define RTD119X_SYS_PLL_SD1_REG_D3318POW	GENMASK(18, 17)
#define RTD119X_SYS_PLL_SD1_REG_TUNED3318	GENMASK(21, 19)
#define RTD129X_SYS_PLL_SD1_REG_SEL3318		BIT(13)

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

#define REG_SD_DMA_RST			0x20
#define REG_SD_ISR			0x24
#define REG_SD_ISREN			0x28
#define RTD129X_REG_SD_DUMMY_SYS	0x2c
#define REG_SD_PAD_CTL			0x74
#define REG_SD_CKGEN_CTL		0x78

#define REG_SD_CR_CARD_STOP		0x103
#define REG_SD_CR_CARD_OE		0x104
#define REG_SD_CARD_SELECT		0x10e
#define REG_SD_CARD_CLOCK_EN_CTL	0x129
#define REG_SD_CONFIGURE1		0x180
#define REG_SD_STATUS2			0x184
#define REG_SD_SAMPLE_POINT_CTL		0x187
#define REG_SD_PUSH_POINT_CTL		0x188
#define REG_SD_BUS_TA_STATE		0x197

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

#define SD_CONFIGURE1_RST_RDWR_FIFO	BIT(4)
#define SD_CONFIGURE1_SDCLK_DIV_256	BIT(6)
#define SD_CONFIGURE1_SDCLK_DIV		BIT(7)

struct dhc_sdmmc_priv {
	struct mmc_host *mmc;
	void __iomem *base;
	struct clk *clk, *clk_ip;
	struct reset_control *rstc;
	struct regmap *crt, *sb2;
};

static int dhc_sdmmc_sync(struct dhc_sdmmc_priv *priv)
{
	return regmap_write(priv->sb2, REG_SB2_SYNC, 0);
}

static void rtd129x_prolog(struct dhc_sdmmc_priv *priv)
{
	if (true) // XXX RTD129x chip_rev != 0
		writel(0x40000000, priv->base + RTD129X_REG_SD_DUMMY_SYS);
}

static void rtd129x_epilog(struct dhc_sdmmc_priv *priv)
{
	if (true) // XXX RTD129x chip_rev != 0
		writel(0x00000000, priv->base + RTD129X_REG_SD_DUMMY_SYS);
}

static int dhc_sdmmc_set_power(struct dhc_sdmmc_priv *priv, bool power)
{
	return 0;
}

static int dhc_sdmmc_set_speed(struct dhc_sdmmc_priv *priv, unsigned int speed)
{
	u32 val;
	u8 b;
	int ret;

	if (speed == 50000000 && priv->mmc->ios.timing == MMC_TIMING_UHS_DDR50) {
	} else if (speed == 100000000) {
	} else if (speed == 208000000) {
		// TODO pinctrl
		rtd129x_prolog(priv);
		rtd129x_epilog(priv);
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

static void dhc_sdmmc_set_ios(struct mmc_host *host, struct mmc_ios *ios)
{
	//struct dhc_sdmmc_priv *priv = mmc_priv(host);
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
	//
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

	ret = dhc_sdmmc_sync(priv);
}

static const struct mmc_host_ops dhc_sdmmc_ops = {
	.set_ios = dhc_sdmmc_set_ios,
	.hw_reset = dhc_sdmmc_hw_reset,
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

	platform_set_drvdata(pdev, mmc);

	ret = dhc_sdmmc_init(priv);
	if (ret)
		goto err_init;

	mmc->f_min = 10000000 >> 8; /* 10 MHz / 256 */
	mmc->f_max = 208000000; /* 208 MHz */
	mmc->max_segs = 1;
	mmc->max_blk_size = 512;
	mmc->max_blk_count = 0x1000;
	mmc->max_seg_size = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_req_size = mmc->max_blk_size * mmc->max_blk_count;
	mmc->ops = &dhc_sdmmc_ops;

	ret = dhc_sdmmc_sync(priv);
	if (ret)
		goto err_sync;

	ret = mmc_add_host(mmc);
	if (ret)
		goto err_add;

	dev_info(&pdev->dev, "probed\n");

	return 0;

err_add:
err_sync:
err_init:
	clk_disable_unprepare(priv->clk_ip);
err_enable_ip:
	clk_disable_unprepare(priv->clk);
err_enable:
	reset_control_assert(priv->rstc);
err_reset:
err_parse:
err_sb2:
err_crt:
err_clk_ip:
err_clk:
err_rst:
err_ioremap:
	mmc_free_host(mmc);
	return ret;
}

static int dhc_sdmmc_remove(struct platform_device *pdev)
{
	struct mmc_host *mmc = platform_get_drvdata(pdev);
	struct dhc_sdmmc_priv *priv = mmc_priv(mmc);

	mmc_remove_host(mmc);

	clk_disable_unprepare(priv->clk_ip);
	clk_disable_unprepare(priv->clk);
	reset_control_assert(priv->rstc);

	mmc_free_host(mmc);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id dhc_sdmmc_dt_ids[] = {
	 { .compatible = "realtek,rtd1195-sdmmc" },
	 { .compatible = "realtek,rtd1295-sdmmc" },
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
