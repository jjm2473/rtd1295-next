// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * TI CC1120 transceiver
 *
 * Copyright (c) 2019 Andreas Färber
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>

#ifdef CONFIG_OF
static const struct of_device_id cc1120_dt_ids[] = {
	{ .compatible = "ti,cc1120" },
	{}
};
MODULE_DEVICE_TABLE(of, cc1120_dt_ids);
#endif

static int cc1120_probe(struct spi_device *spi)
{
	spi->bits_per_word = 8;
	spi_setup(spi);

	dev_info(&spi->dev, "probed\n");

	return 0;
}

static int cc1120_remove(struct spi_device *spi)
{
	dev_info(&spi->dev, "removed\n");

	return 0;
}

static struct spi_driver cc1120_spi_driver = {
	.driver = {
		.name = "cc1120",
		.of_match_table = of_match_ptr(cc1120_dt_ids),
	},
	.probe = cc1120_probe,
	.remove = cc1120_remove,
};

module_spi_driver(cc1120_spi_driver);

MODULE_DESCRIPTION("TI CC1120 SPI driver");
MODULE_AUTHOR("Andreas Färber <afaerber@suse.de>");
MODULE_LICENSE("GPL");
