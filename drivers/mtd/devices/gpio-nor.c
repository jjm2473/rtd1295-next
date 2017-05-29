/*
 * drivers/mtd/devices/gpio-nor.c
 * (C) Copyright 2013 Linaro, Ltd
 * Andy Green <andy.green@linaro.org>
 *
 * This driver is a bit unusual... it's a bitbang gpio driver to access
 * a parallel NOR flash entirely by GPIO.
 *
 * based on -->
 *
 * Updated, and converted to generic GPIO based driver by Russell King.
 *
 * Written by Ben Dooks <ben@simtec.co.uk>
 *   Based on 2.4 version by Mark Whittaker
 *
 * © 2004 Simtec Electronics
 *
 * Device driver for NAND connected via GPIO
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/mtd/cfi.h>
#include <linux/of_platform.h>
#include <linux/err.h>

#define MAX_ADS_BITS 32
#define MAX_DATA_BITS 16

struct gpio_mtd {
	void __iomem		*io_sync;

	struct mtd_info fake_mtd_info;
	struct map_info map;

	int data_bits;
	int ads_bits;

	int	gpio_nce;
	int	gpio_nwe;
	int	gpio_noe;
	int	gpio_rdy;
	int	gpio_ads[MAX_ADS_BITS];
	int	gpio_data[MAX_DATA_BITS];

	struct mtd_partition *parts;
	unsigned int num_parts;
	unsigned int options;
};

#define gpio_nor_getpriv(x) \
	((struct gpio_mtd *)((struct map_info *)(x)->priv)->map_priv_1)
#define map_to_mtd_info(x) ((struct mtd_info *)(x)->map_priv_2)

static const struct of_device_id gpio_nor_id_table[] = {
	{ .compatible = "gpio-control-nor" },
	{}
};
MODULE_DEVICE_TABLE(of, gpio_nor_id_table);

static int gpio_nor_get_config_of(const struct device *dev, struct gpio_mtd *gm)
{
	u32 val;
	int n;
	int gpio = 1;
	int ret;

	gm->data_bits = 8;
	if (!of_property_read_u32(dev->of_node, "bank-width", &val)) {
		if (val == 2) {
			gm->options |= NAND_BUSWIDTH_16;
			gm->data_bits = 16;
		} else if (val != 1) {
			dev_err(dev, "invalid bank-width %u\n", val);
			return -EINVAL;
		}
	}

	/* first 4 are in this fixed order */
	gm->gpio_rdy = of_get_gpio(dev->of_node, 0);
	gm->gpio_nce = of_get_gpio(dev->of_node, 1);
	gm->gpio_noe = of_get_gpio(dev->of_node, 2);
	gm->gpio_nwe = of_get_gpio(dev->of_node, 3);

	if (gpio_is_valid(gm->gpio_rdy)) {
		ret = gpio_request(gm->gpio_rdy, "NOR_rdy");
		if (!ret)
			gpio_direction_input(gm->gpio_rdy);
	}

	ret = gpio_request(gm->gpio_nce, "NOR_nCE");
	if (ret) {
		dev_err(dev, "failed to request nCE gpio %d\n", gm->gpio_nce);
		return -EINVAL;
	}
	gpio_direction_output(gm->gpio_nce, 1);

	ret = gpio_request(gm->gpio_noe, "NOR_nOE");
	if (ret) {
		dev_err(dev, "failed to request nOE gpio\n");
		return -EINVAL;
	}
	gpio_direction_output(gm->gpio_noe, 1);

	ret = gpio_request(gm->gpio_nwe, "NOR_nWE");
	if (ret) {
		dev_err(dev, "failed to request nWE gpio\n");
		return -EINVAL;
	}
	gpio_direction_output(gm->gpio_nwe, 1);

	/* then the next 8 * bank-width are data */
	for (n = 0; n < gm->data_bits; n++) {
		gm->gpio_data[n] = of_get_gpio(dev->of_node, n + 4);
		if (IS_ERR((void *)gm->gpio_data[n])) {
			dev_err(dev, " invalid gpio %d\n", gm->gpio_data[n]);
			return -EINVAL;
		}
		ret = gpio_request(gm->gpio_data[n] , "NOR_data");
		if (ret) {
			dev_err(dev, "failed to request D%d gpio %d: %d\n",
						     n, gm->gpio_data[n], ret);
			return -EINVAL;
		}
	}

	/* and the remaining ones are address */
	gm->ads_bits = 0;
	while (!IS_ERR((void *)gpio)) {
		gpio = of_get_gpio(dev->of_node, gm->ads_bits +
							gm->data_bits + 4);
		if (IS_ERR((void *)gpio))
			continue;
		ret = gpio_request(gpio, "NOR_ads");
		if (ret) {
			dev_err(dev, "failed to request A%d gpio %d: %d\n",
						      gm->ads_bits, gpio, ret);
			return -EINVAL;
		}
		gm->gpio_ads[gm->ads_bits++] = gpio;
		gpio_direction_output(gpio, 0);
	}

	return 0;
}

static void gpio_nor_set_address(struct gpio_mtd *gm, unsigned long ads)
{
	int n;

	for (n = 0; n < gm->ads_bits; n++)
		gpio_set_value(gm->gpio_ads[n], !!(ads & (1 << n)));
}

static void gpio_nor_write_data(struct gpio_mtd *gm, unsigned int data)
{
	int n;

	for (n = 0; n < gm->data_bits; n++)
		gpio_set_value(gm->gpio_data[n], !!(data & (1 << n)));
}

static unsigned int gpio_nor_read_data(struct gpio_mtd *gm)
{
	int n;
	unsigned int data = 0;

	for (n = 0; n < gm->data_bits; n++)
		if (gpio_get_value(gm->gpio_data[n]))
			data |= 1 << n;

	return data;
}

static unsigned int gpio_nor_data_bus_write_mode(struct gpio_mtd *gm)
{
	int n;

	for (n = 0; n < gm->data_bits; n++)
		gpio_direction_output(gm->gpio_data[n], 0);

	return 0;
}

static unsigned int gpio_nor_data_bus_read_mode(struct gpio_mtd *gm)
{
	int n;

	for (n = 0; n < gm->data_bits; n++)
		gpio_direction_input(gm->gpio_data[n]);

	return 0;
}


static int gpio_nor_read(struct mtd_info *mtd, loff_t from, size_t len,
						   size_t *retlen, u_char *buf)
{
	struct gpio_mtd *gm = gpio_nor_getpriv(mtd);
	int val;

	*retlen = len;
	gpio_nor_data_bus_read_mode(gm);

	while (len) {

		gpio_nor_set_address(gm, from >> 1);

		gpio_set_value(gm->gpio_noe, 0);
		gpio_set_value(gm->gpio_nwe, 1);
		gpio_set_value(gm->gpio_nce, 0);

		val = gpio_nor_read_data(gm);
		*buf++ = val;
		len--;
		if (len) {
			*buf++ = val >> 8;
			len--;
		}

		gpio_set_value(gm->gpio_nce, 1);
		gpio_set_value(gm->gpio_nwe, 1);
		gpio_set_value(gm->gpio_nce, 1);

		from += 2;
	}

	return 0;
}

map_word gpio_nor_map_read(struct map_info *map, unsigned long ads)
{
	struct mtd_info *mtd_info = map_to_mtd_info(map);
	map_word result = { {0} };
	size_t len = map->bankwidth;

	gpio_nor_read(mtd_info, ads, len, &len, (char *)&result);

	return result;
}
void gpio_nor_copy_from(struct map_info *map, void *source, unsigned long ads,
								ssize_t len)
{
	struct mtd_info *mtd_info = map_to_mtd_info(map);

	gpio_nor_read(mtd_info, ads, len, &len, (char *)source);
}


static int gpio_nor_write(struct mtd_info *mtd, loff_t to, size_t len,
					     size_t *retlen, const u_char *buf)
{
	struct gpio_mtd *gm = gpio_nor_getpriv(mtd);
	int val;

	*retlen = len;

	if (gm->data_bits == 16)
		if (len & 1)
			len++;

	gpio_nor_data_bus_write_mode(gm);

	while (len) {

		gpio_nor_set_address(gm, to >> 1);
		gpio_set_value(gm->gpio_noe, 1);
		gpio_set_value(gm->gpio_nwe, 0);
		gpio_set_value(gm->gpio_nce, 0);

		val = *buf++;
		val |= (*buf++) << 8;

		gpio_nor_write_data(gm, val);

		gpio_set_value(gm->gpio_nce, 1);
		gpio_set_value(gm->gpio_nwe, 1);
		gpio_set_value(gm->gpio_nce, 1);

		len -= 2;
		to += 2;
	}

	return 0;
}

void gpio_nor_map_write(struct map_info *map, map_word data, unsigned long ads)
{
	struct mtd_info *mtd_info = map_to_mtd_info(map);
	size_t len = map->bankwidth;

	gpio_nor_write(mtd_info, ads, len, &len, (char *)&data);
}

static int gpio_nor_remove(struct platform_device *dev)
{
	struct gpio_mtd *gm = platform_get_drvdata(dev);
	struct mtd_info *mtd_info = map_to_mtd_info(&gm->map);
	int n;

	if (mtd_info) {
		mtd_device_unregister(mtd_info);
		dev_info(&dev->dev, "Removing\n");
		map_destroy(mtd_info);
	}

	if (gpio_is_valid(gm->gpio_nce))
		gpio_free(gm->gpio_nce);
	if (gpio_is_valid(gm->gpio_noe))
		gpio_free(gm->gpio_noe);
	if (gpio_is_valid(gm->gpio_nwe))
		gpio_free(gm->gpio_nwe);
	if (gpio_is_valid(gm->gpio_rdy))
		gpio_free(gm->gpio_rdy);

	for (n = 0; n < gm->data_bits; n++)
		if (gpio_is_valid(gm->gpio_data[n]))
			gpio_free(gm->gpio_data[n]);

	for (n = 0; n < gm->ads_bits; n++)
		if (gpio_is_valid(gm->gpio_ads[n]))
			gpio_free(gm->gpio_ads[n]);

	kfree(gm);

	return 0;
}

static int gpio_nor_probe(struct platform_device *dev)
{
	struct gpio_mtd *gm;
	struct mtd_info *mtd_info;
	struct mtd_part_parser_data ppdata = {};
	int ret = 0;

	if (!dev->dev.of_node)
		return -EINVAL;

	gm = devm_kzalloc(&dev->dev, sizeof(*gm), GFP_KERNEL);
	if (gm == NULL) {
		dev_err(&dev->dev, "failed to create NOR MTD\n");
		return -ENOMEM;
	}
	platform_set_drvdata(dev, gm);

	ret = gpio_nor_get_config_of(&dev->dev, gm);
	if (ret) {
		dev_err(&dev->dev, "failed get_config_of\n");
		goto bail;
	}

	/* the real mtd_info->priv also points to map_info */
	gm->fake_mtd_info.priv = &gm->map;

	gm->map.name = "gpio-nor";
	gm->map.map_priv_1 = (unsigned long)gm;
	gm->map.map_priv_2 = (unsigned long)&gm->fake_mtd_info;

	gm->map.bankwidth = gm->data_bits >> 3;
	gm->map.read = gpio_nor_map_read;
	gm->map.write = gpio_nor_map_write;
	gm->map.copy_from = gpio_nor_copy_from;

	/* there's no mapping region, limited only by ads bits x width */
	gm->map.size = 1 << (gm->ads_bits + gm->map.bankwidth - 1);

	mtd_info = do_map_probe("cfi_probe", &gm->map);
	if (!mtd_info) {
		dev_err(&dev->dev, "map probe failed\n");
		return -ENODEV;
	}

	mtd_info->dev.parent = &dev->dev;
	mtd_info->owner = THIS_MODULE;
	/* point to the real one */
	gm->map.map_priv_2 = (unsigned long)mtd_info;
	mtd_info->_read = gpio_nor_read;

	ppdata.of_node = dev->dev.of_node;
	ret = mtd_device_parse_register(mtd_info, NULL, &ppdata,
						     gm->parts, gm->num_parts);
	if (ret) {
		dev_err(&dev->dev, "mtd_device_parse_register: %d\n", ret);
		goto bail;
	}

	dev_info(&dev->dev, "Usable: %dMiB\n", 1 << (gm->ads_bits - 20));

	return 0;

bail:
	dev_err(&dev->dev, "failed to probe %d\n", ret);
	gpio_nor_remove(dev);

	return ret;
}

static struct platform_driver gpio_nor_driver = {
	.probe		= gpio_nor_probe,
	.remove		= gpio_nor_remove,
	.driver		= {
		.name	= "gpio-nor",
		.of_match_table = of_match_ptr(gpio_nor_id_table),
	},
};

module_platform_driver(gpio_nor_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andy Green <andy.green@linaro.org>");
MODULE_DESCRIPTION("GPIO NOR Driver");
