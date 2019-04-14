// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SiLabs Si4432 transceiver
 *
 * Copyright (c) 2019 Andreas Färber
 */

#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <net/cfgfsk.h>

#define REG_FO1		0x73
#define REG_FO2		0x74
#define REG_FO2_FO_MASK		GENMASK(1, 0)

#define REG_FBSEL	0x75
#define REG_FBSEL_FB_MASK	GENMASK(4, 0)

#define REG_FC1		0x76
#define REG_FC2		0x77
#define REG_FHCH	0x79
#define REG_FHS		0x7a

struct si443x_priv {
	struct regmap *regmap;
};

static int si443x_fsk_get_freq(struct fsk_phy *phy, u32 *val)
{
	struct regmap *regmap = dev_get_drvdata(phy->dev);
	unsigned int fo1, fo2, fbsel, fc1, fc2, fhch, fhs;
	u16 tmp, fc;
	s16 fo;
	u32 fcarrier, fb, hbsel;
	int ret;

	ret = regmap_read(regmap, REG_FO1, &fo1);
	if (ret)
		return ret;

	ret = regmap_read(regmap, REG_FO2, &fo2);
	if (ret)
		return ret;

	tmp = (((u16)fo2 & REG_FO2_FO_MASK) << 8) | fo1;
	if (tmp & BIT(9))
		/* sign-extend from 10-bit to 16-bit */
		tmp |= GENMASK(15, 10);
	fo = (s16)tmp;

	ret = regmap_read(regmap, REG_FBSEL, &fbsel);
	if (ret)
		return ret;
	fb = fbsel & REG_FBSEL_FB_MASK;
	hbsel = (fbsel >> 5) & 0x1;

	ret = regmap_read(regmap, REG_FC1, &fc1);
	if (ret)
		return ret;

	ret = regmap_read(regmap, REG_FC2, &fc2);
	if (ret)
		return ret;

	fc = ((u16)fc1 << 8) | fc2;

	ret = regmap_read(regmap, REG_FHCH, &fhch);
	if (ret)
		return ret;

	ret = regmap_read(regmap, REG_FHS, &fhs);
	if (ret)
		return ret;

	fcarrier = (fb + 24) * 10000 * (hbsel + 1);
	fcarrier += (((u32)fc + fo) * 10) / 64 * (hbsel + 1);
	fcarrier += ((u32)fhch * fhs * 10);

	*val = fcarrier * 1000;

	return 0;
}

static int si443x_fsk_set_freq(struct fsk_phy *phy, u32 val)
{
	struct regmap *regmap = dev_get_drvdata(phy->dev);
	unsigned int fbsel, fo2;
	u64 fb, fc;
	int ret;

	ret = regmap_read(regmap, REG_FBSEL, &fbsel);
	if (ret)
		return ret;
	if (val >= 480000000) {
		fbsel |= BIT(5);
		fb = val - 480000000;
		do_div(fb, 20000000);
		fc = val - (24 + fb) * 20000000;
		fc *= 2;
		do_div(fc, (156.25 * 2 * 2));
	} else {
		fbsel &= ~BIT(5);
		fb = val - 240000000;
		do_div(fb, 10000000);
		fc = val - (24 + fb) * 10000000;
		fc *= 4;
		do_div(fc, (156.25 * 4));
	}
	fbsel &= ~REG_FBSEL_FB_MASK;
	fbsel |= fb & REG_FBSEL_FB_MASK;
	ret = regmap_write(regmap, REG_FBSEL, fbsel);
	if (ret)
		return ret;

	ret = regmap_write(regmap, REG_FC1, fc >> 8);
	if (ret)
		return ret;

	ret = regmap_write(regmap, REG_FC2, fc & GENMASK(7, 0));
	if (ret)
		return ret;

	ret = regmap_write(regmap, REG_FO1, 0);
	if (ret)
		return ret;

	ret = regmap_read(regmap, REG_FO2, &fo2);
	if (ret)
		return ret;
	fo2 &= ~REG_FO2_FO_MASK;
	ret = regmap_write(regmap, REG_FO2, fo2);
	if (ret)
		return ret;

	ret = regmap_write(regmap, REG_FHCH, 0);
	if (ret)
		return ret;

	ret = regmap_write(regmap, REG_FHS, 0);
	if (ret)
		return ret;

	return 0;
}

static const struct cfgfsk_ops si443x_fsk_ops = {
	.get_freq	= si443x_fsk_get_freq,
	.set_freq	= si443x_fsk_set_freq,
};

static struct regmap_config si443x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.cache_type = REGCACHE_NONE,

	.read_flag_mask = 0,
	.write_flag_mask = BIT(7),

	.max_register = 0x7f,
};

#ifdef CONFIG_OF
static const struct of_device_id si443x_dt_ids[] = {
	{ .compatible = "silabs,si4432" },
	{}
};
MODULE_DEVICE_TABLE(of, si443x_dt_ids);
#endif

static int si443x_probe(struct spi_device *spi)
{
	struct regmap *regmap;
	struct fsk_phy *phy;
	unsigned int val;
	u32 freq;
	int ret;

	spi->bits_per_word = 8;
	spi_setup(spi);

	regmap = devm_regmap_init_spi(spi, &si443x_regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		return ret;
	}

	spi_set_drvdata(spi, regmap);

	ret = regmap_read(regmap, 0, &val);
	if (ret)
		return ret;

	dev_info(&spi->dev, "device type: 0x%02x\n", val);

	ret = regmap_read(regmap, 1, &val);
	if (ret)
		return ret;

	dev_info(&spi->dev, "device version: 0x%02x\n", val);

	ret = of_property_read_u32(spi->dev.of_node, "radio-frequency", &freq);
	if (ret) {
		dev_err(&spi->dev, "failed reading radio-frequency");
		return ret;
	}

	phy = devm_fsk_phy_new(&spi->dev, &si443x_fsk_ops, 0);
	if (!phy)
		return -ENOMEM;

	ret = si443x_fsk_set_freq(phy, freq);
	if (ret) {
		dev_err(&spi->dev, "failed to set frequency (%d)\n", ret);
		return ret;
	}

	ret = si443x_fsk_get_freq(phy, &freq);
	if (ret) {
		dev_err(&spi->dev, "failed to get frequency (%d)\n", ret);
		return ret;
	}
	dev_info(&spi->dev, "frequency: %u\n", freq);

	dev_info(&spi->dev, "probed\n");

	return 0;
}

static int si443x_remove(struct spi_device *spi)
{
	dev_info(&spi->dev, "removed\n");

	return 0;
}

static struct spi_driver si443x_spi_driver = {
	.driver = {
		.name = "si443x",
		.of_match_table = of_match_ptr(si443x_dt_ids),
	},
	.probe = si443x_probe,
	.remove = si443x_remove,
};

module_spi_driver(si443x_spi_driver);

MODULE_DESCRIPTION("Si443x SPI driver");
MODULE_AUTHOR("Andreas Färber <afaerber@suse.de>");
MODULE_LICENSE("GPL");
