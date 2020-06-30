// SPDX-License-Identifier: GPL-2.0-only
/*
 * Realtek Digital Home Center SDIO
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

#include "sdhci-pltfm.h"

#define REG_SYS_PLL_SDIO2	0x1a4
#define REG_SYS_PLL_SDIO3	0x1a8
#define REG_SYS_PLL_SDIO4	0x1ac

#define SYS_PLL_SDIO4_SSC_RSTB		BIT(0)
#define SYS_PLL_SDIO4_SSC_PLL_RSTB	BIT(1)
#define SYS_PLL_SDIO4_SSC_PLL_POW	BIT(2)

#define REG_SDIO_IP_CTL	0x10
#define REG_SDIO_ISREN	0x34

#define SDIO_IP_CTL_SUSPEND_N			BIT(0)
#define SDIO_IP_CTL_L4_GATED_DISABLE		BIT(1)
#define SDIO_IP_CTL_DBUS_ENDIAN_SEL		BIT(2)
#define SDIO_IP_CTL_DBUS_ENDIAN_SEL_LITTLE	FIELD_PREP(SDIO_IP_CTL_DBUS_ENDIAN_SEL, 0)
#define SDIO_IP_CTL_DBUS_ENDIAN_SEL_BIG		FIELD_PREP(SDIO_IP_CTL_DBUS_ENDIAN_SEL, 1)

#define SDIO_ISREN_WRITE_DATA		BIT(0)
#define SDIO_ISREN_WRITE_DATA_CLEAR	FIELD_PREP(SDIO_ISREN_WRITE_DATA, 0)
#define SDIO_ISREN_WRITE_DATA_SET	FIELD_PREP(SDIO_ISREN_WRITE_DATA, 1)
#define SDIO_ISREN_INT1EN		BIT(1)
#define SDIO_ISREN_INT3EN		BIT(3)
#define SDIO_ISREN_INT4EN		BIT(4)

struct dhc_sdio_priv;

#define RTD129X_NON_A00_PROLOG_EPILOG_QUIRK	0

struct dhc_sdio_info {
	int (*pre_init)(struct dhc_sdio_priv *priv);
};

struct dhc_sdio_priv {
	const struct dhc_sdio_info *info;
	struct clk *clk, *clk_ip;
	struct reset_control *rstc;
	void __iomem *base;
	struct regmap *crt;
	unsigned int quirks;
};

/*static void dhc_sdio_reset(struct sdhci_host *host, u8 mask)
{
	unsigned int timeout = 100;

	sdhci_writeb(host, mask, SDHCI_SOFTWARE_RESET);
	if (mask & SDHCI_RESET_ALL)
		host->clock = 0;

	while (sdhci_readb(host, SDHCI_SOFTWARE_RESET) & mask) {
		if (timeout == 0)
		timeout--;
	}
}*/

static const struct sdhci_ops dhc_sdio_ops = {
	.set_clock = sdhci_set_clock,
	.set_bus_width = sdhci_set_bus_width,
	.reset = /*dhc_sdio_reset*/sdhci_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
};

static const struct dhc_sdio_info rtd119x_sdio_info = {
};

static int rtd129x_sdio_pre_init(struct dhc_sdio_priv *priv)
{
	// XXX non-A00 only
	priv->quirks |= RTD129X_NON_A00_PROLOG_EPILOG_QUIRK;

	return 0;
}

static const struct dhc_sdio_info rtd129x_sdio_info = {
	.pre_init = rtd129x_sdio_pre_init,
};

static int dhc_sdio_probe(struct platform_device *pdev)
{
	struct sdhci_pltfm_data pdata = {};
	struct sdhci_host *host;
	struct sdhci_pltfm_host *phost;
	struct dhc_sdio_priv *priv;
	u32 val;
	int ret;

	pdata.ops = &dhc_sdio_ops;
	pdata.quirks = SDHCI_QUIRK_BROKEN_TIMEOUT_VAL | SDHCI_QUIRK_SINGLE_POWER_WRITE | SDHCI_QUIRK_NO_HISPD_BIT | SDHCI_QUIRK_BROKEN_CARD_DETECTION | SDHCI_QUIRK_BROKEN_ADMA_ZEROLEN_DESC;
	pdata.quirks2 = SDHCI_QUIRK2_BROKEN_DDR50 | SDHCI_QUIRK2_PRESET_VALUE_BROKEN;

	host = sdhci_pltfm_init(pdev, &pdata, sizeof(*priv));
	if (IS_ERR(host)) {
		ret = PTR_ERR(host);
		dev_err(&pdev->dev, "init failed (%d)\n", ret);
		return ret;
	}

	phost = sdhci_priv(host);
	priv = sdhci_pltfm_priv(phost);

	priv->info = device_get_match_data(&pdev->dev);
	if (!priv->info) {
		ret = -EINVAL;
		goto err_match;
	}

	priv->rstc = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (IS_ERR(priv->rstc)) {
		ret = PTR_ERR(priv->rstc);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "reset failed (%d)\n", ret);
		goto err_rst;
	}

	priv->clk = devm_clk_get(&pdev->dev, "clk_en_sdio");
	if (IS_ERR(priv->clk)) {
		ret = PTR_ERR(priv->clk);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "clk_en_sdio failed (%d)\n", ret);
		goto err_clk;
	}

	priv->clk_ip = devm_clk_get(&pdev->dev, "clk_en_sdio_ip");
	if (IS_ERR(priv->clk_ip)) {
		ret = PTR_ERR(priv->clk_ip);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "clk_en_sdio_ip failed (%d)\n", ret);
		goto err_clk_ip;
	}

	priv->base = devm_platform_ioremap_resource(pdev, 1);
	if (!priv->base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		goto err_ioremap;
	}

	priv->crt = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "crt-syscon");
	if (IS_ERR(priv->crt)) {
		ret = PTR_ERR(priv->crt);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "crt syscon failed (%d)\n", ret);
		goto err_syscon;
	}

	sdhci_get_of_property(pdev);

	ret = mmc_of_parse(host->mmc);
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
		dev_err(&pdev->dev, "clk_en_sdio enable failed (%d)\n", ret);
		goto err_enable;
	}

	ret = clk_prepare_enable(priv->clk_ip);
	if (ret) {
		dev_err(&pdev->dev, "clk_en_sdio_ip enable failed (%d)\n", ret);
		goto err_enable_ip;
	}

	priv->quirks = 0;
	if (priv->info->pre_init) {
		ret = priv->info->pre_init(priv);
		if (ret)
			goto err_init_pre;
	}

	if (priv->quirks & RTD129X_NON_A00_PROLOG_EPILOG_QUIRK)
		writel(0x40000000, priv->base + 0x58);

	regmap_write(priv->crt, REG_SYS_PLL_SDIO4, SYS_PLL_SDIO4_SSC_PLL_POW | SYS_PLL_SDIO4_SSC_PLL_RSTB);
	regmap_write(priv->crt, REG_SYS_PLL_SDIO2, 0x04517893);
	regmap_write(priv->crt, REG_SYS_PLL_SDIO3, 0x00ae4388);
	mdelay(2);

	regmap_read(priv->crt, REG_SYS_PLL_SDIO4, &val);
	val |= SYS_PLL_SDIO4_SSC_RSTB;
	regmap_write(priv->crt, REG_SYS_PLL_SDIO4, val);
	udelay(200);

	if (priv->quirks & RTD129X_NON_A00_PROLOG_EPILOG_QUIRK)
		writel(0x00000000, priv->base + 0x58);

	writel(SDIO_ISREN_INT4EN |
	       SDIO_ISREN_WRITE_DATA_SET, priv->base + REG_SDIO_ISREN);
	writel(SDIO_IP_CTL_DBUS_ENDIAN_SEL_LITTLE |
	       SDIO_IP_CTL_L4_GATED_DISABLE |
	       SDIO_IP_CTL_SUSPEND_N, priv->base + REG_SDIO_IP_CTL);

	host->caps = sdhci_readl(host, SDHCI_CAPABILITIES);
	host->caps1 = sdhci_readl(host, SDHCI_CAPABILITIES_1);
	//host->mmc->caps = MMC_CAP_NONREMOVABLE;
	host->quirks |= SDHCI_QUIRK_MISSING_CAPS;

	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(&pdev->dev, "add failed (%d)\n", ret);
		goto err_add;
	}

	dev_info(&pdev->dev, "probed\n");

	return 0;

err_add:
err_init_pre:
	clk_disable_unprepare(priv->clk_ip);
err_enable_ip:
	clk_disable_unprepare(priv->clk);
err_enable:
	reset_control_assert(priv->rstc);
err_reset:
err_parse:
err_syscon:
err_ioremap:
err_clk_ip:
err_clk:
err_rst:
err_match:
	sdhci_pltfm_free(pdev);
	return ret;
}

static const struct of_device_id dhc_sdio_dt_ids[] = {
	 { .compatible = "realtek,rtd1195-sdio", .data = &rtd119x_sdio_info },
	 { .compatible = "realtek,rtd1295-sdio", .data = &rtd129x_sdio_info },
	 { }
};

static struct platform_driver dhc_sdio_driver = {
	.probe = dhc_sdio_probe,
	.remove = sdhci_pltfm_unregister,
	.driver = {
		.name = "rtk-dhc-sdio",
		.of_match_table	= dhc_sdio_dt_ids,
	},
};
module_platform_driver(dhc_sdio_driver);

MODULE_DESCRIPTION("Realtek DHC SDIO driver");
MODULE_LICENSE("GPL v2");
