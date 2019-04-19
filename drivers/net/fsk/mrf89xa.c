// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Microchip MRF89XA transceiver
 *
 * Copyright (c) 2019 Andreas Färber
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>

#ifdef CONFIG_OF
static const struct of_device_id mrf89xa_dt_ids[] = {
	{ .compatible = "microchip,mrf89xa" },
	{}
};
MODULE_DEVICE_TABLE(of, mrf89xa_dt_ids);
#endif

static int mrf89xa_probe(struct spi_device *spi)
{
	spi->bits_per_word = 8;
	spi_setup(spi);

	dev_info(&spi->dev, "probed\n");

	return 0;
}

static int mrf89xa_remove(struct spi_device *spi)
{
	dev_info(&spi->dev, "removed\n");

	return 0;
}

static struct spi_driver mrf89xa_spi_driver = {
	.driver = {
		.name = "mrf89xa",
		.of_match_table = of_match_ptr(mrf89xa_dt_ids),
	},
	.probe = mrf89xa_probe,
	.remove = mrf89xa_remove,
};

module_spi_driver(mrf89xa_spi_driver);

MODULE_DESCRIPTION("MRF89XA SPI driver");
MODULE_AUTHOR("Andreas Färber <afaerber@suse.de>");
MODULE_LICENSE("GPL");
