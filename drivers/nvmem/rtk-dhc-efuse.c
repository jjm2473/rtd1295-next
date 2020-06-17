// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Realtek Digital Home Center eFuse
 *
 * Copyright (c) 2020 Andreas Färber
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/nvmem-provider.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

struct dhc_efuse {
	struct device *dev;
	void __iomem *base;
	struct nvmem_device *nvmem;
};

static int dhc_efuse_reg_read(void *priv, unsigned int offset, void *val,
	size_t bytes)
{
	struct dhc_efuse *efuse = priv;
	u8 *buf = val;

	while (bytes--)
		*buf++ = readb_relaxed(efuse->base + offset++);

	return 0;
}

static int dhc_efuse_probe(struct platform_device *pdev)
{
	struct dhc_efuse *efuse;
	struct nvmem_config config = {};
	struct resource *res;

	efuse = devm_kzalloc(&pdev->dev, sizeof(*efuse), GFP_KERNEL);
	if (!efuse)
		return -ENOMEM;

	efuse->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(efuse->base))
		return PTR_ERR(efuse->base);

	efuse->dev = &pdev->dev;

	config.dev = &pdev->dev;
	config.name = "dhc-efuse";
	config.owner = THIS_MODULE;
	config.type = NVMEM_TYPE_OTP;
	config.read_only = true,
	config.word_size = 4;
	config.stride = 1;
	config.size = resource_size(res);
	config.reg_read = dhc_efuse_reg_read;
	config.priv = efuse;

	efuse->nvmem = devm_nvmem_register(&pdev->dev, &config);
	if (IS_ERR(efuse->nvmem)) {
		dev_err(&pdev->dev, "failed to register nvmem (%ld)\n",
			PTR_ERR(efuse->nvmem));
		return PTR_ERR(efuse->nvmem);
	}

	return 0;
}

static const struct of_device_id dhc_efuse_dt_ids[] = {
	 { .compatible = "realtek,rtd1195-efuse" },
	 { }
};

static struct platform_driver dhc_efuse_driver = {
	.probe = dhc_efuse_probe,
	.driver = {
		.name = "rtk-dhc-efuse",
		.of_match_table	= dhc_efuse_dt_ids,
	},
};
module_platform_driver(dhc_efuse_driver);

MODULE_DESCRIPTION("Realtek DHC eFuse driver");
MODULE_LICENSE("GPL");
