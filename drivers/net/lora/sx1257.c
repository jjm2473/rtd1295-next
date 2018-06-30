// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Semtech SX1255/SX1257 LoRa transceiver
 *
 * Copyright (c) 2018 Andreas Färber
 *
 * Based on SX1301 HAL code:
 * Copyright (c) 2013 Semtech-Cycleo
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>

static int sx1257_write(struct spi_device *spi, u8 reg, u8 val)
{
	u8 buf[2];

	buf[0] = reg | BIT(7);
	buf[1] = val;
	return spi_write(spi, buf, 2);
}

static int sx1257_read(struct spi_device *spi, u8 reg, u8 *val)
{
	u8 addr = reg & 0x7f;
	return spi_write_then_read(spi, &addr, 1, val, 1);
}

static int sx1257_probe(struct spi_device *spi)
{
	u8 val;
	int ret;

	if (true) {
		ret = sx1257_read(spi, 0x07, &val);
		if (ret) {
			dev_err(&spi->dev, "version read failed\n");
			return ret;
		}

		dev_info(&spi->dev, "SX125x version: %02x\n", (unsigned)val);
	}

	ret = sx1257_write(spi, 0x10, 1 /* + 2 */);
	if (ret) {
		dev_err(&spi->dev, "clk write failed\n");
		return ret;
	}

	dev_info(&spi->dev, "clk written\n");

	if (true) {
		ret = sx1257_write(spi, 0x26, 13 + 2 * 16);
		if (ret) {
			dev_err(&spi->dev, "xosc write failed\n");
			return ret;
		}
	}

	dev_info(&spi->dev, "SX1257 module probed\n");

	return 0;
}

static int sx1257_remove(struct spi_device *spi)
{
	dev_info(&spi->dev, "SX1257 module removed\n");

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id sx1257_dt_ids[] = {
	{ .compatible = "semtech,sx1255" },
	{ .compatible = "semtech,sx1257" },
	{}
};
MODULE_DEVICE_TABLE(of, sx1257_dt_ids);
#endif

static struct spi_driver sx1257_spi_driver = {
	.driver = {
		.name = "sx1257",
		.of_match_table = of_match_ptr(sx1257_dt_ids),
	},
	.probe = sx1257_probe,
	.remove = sx1257_remove,
};

module_spi_driver(sx1257_spi_driver);

MODULE_DESCRIPTION("SX1257 SPI driver");
MODULE_AUTHOR("Andreas Färber <afaerber@suse.de>");
MODULE_LICENSE("GPL");
