// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ST S2-LP transceiver
 *
 * Copyright (c) 2019 Andreas Färber
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>

#ifdef CONFIG_OF
static const struct of_device_id s2lp_dt_ids[] = {
	{ .compatible = "st,s2-lp" },
	{}
};
MODULE_DEVICE_TABLE(of, s2lp_dt_ids);
#endif

static int s2lp_probe(struct spi_device *spi)
{
	spi->bits_per_word = 8;
	spi_setup(spi);

	dev_info(&spi->dev, "probed\n");

	return 0;
}

static int s2lp_remove(struct spi_device *spi)
{
	dev_info(&spi->dev, "removed\n");

	return 0;
}

static struct spi_driver s2lp_spi_driver = {
	.driver = {
		.name = "s2-lp",
		.of_match_table = of_match_ptr(s2lp_dt_ids),
	},
	.probe = s2lp_probe,
	.remove = s2lp_remove,
};

module_spi_driver(s2lp_spi_driver);

MODULE_DESCRIPTION("ST S2-LP SPI driver");
MODULE_AUTHOR("Andreas Färber <afaerber@suse.de>");
MODULE_LICENSE("GPL");
