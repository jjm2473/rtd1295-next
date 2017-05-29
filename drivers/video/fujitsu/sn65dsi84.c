/*
 * Copyright (C) 2015 Socionext Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <video/display_timing.h>

#include <video/sn65dsi84.h>

struct sn65dsi84_property {
	u8 lvds_clk;
	u8 clk_div;
	u8 dsi_lane;
	u8 dsi_clk;
	u8 de_pola;
	u8 hs_pola;
	u8 vs_pola;
	u8 lvds_link;
	u8 ch_a_24bpp;
	u8 ch_b_24bpp;
	u16 sdelay;
};

struct sn65dsi84_data {
	int en_gpio;
	struct sn65dsi84_property property;
};


#define SN65_MAX_REG 0xe5


static int sn65dsi84_i2c_write_register(struct i2c_client *client, u8 reg, u8 data)
{
	u8 buf[2] = { reg, data };
	int ret;

	ret = i2c_master_send(client, buf, 2);
	if (ret != 2) {
		dev_err(&client->dev, "i2c_master_send error(reg:%#x, ret:%d)\n", reg, ret);
	}
	else {
		ret = 0;
	}

	return ret;
}

static int sn65dsi84_i2c_read_register(struct i2c_client *client, u8 reg, u8 *data, int cnt)
{
	u8 buf[SN65_MAX_REG];
	int ret;

	if (cnt <= 0 || reg + cnt > SN65_MAX_REG) {
		return -EINVAL;
	}

	ret = i2c_master_recv(client, buf, SN65_MAX_REG);
	if (ret != SN65_MAX_REG) {
		dev_err(&client->dev, "i2c_master_recv error(reg:%#x, ret:%d)\n", reg, ret);
		return ret;
	}

	memcpy(data, &buf[reg], cnt);
	return 0;
}

static int sn65dsi84_i2c_read_dt_property(struct i2c_client *client, struct sn65dsi84_property *property)
{
	struct device *dev = &client->dev;
	int ret;
	u8 val;
	u16 val16;

	ret = of_property_read_u8(client->dev.of_node, "lvds-clk-range", &val);
	if (ret) {
		dev_err(dev, "%s: not found\n", "lvds-clk-range");
		goto fail;
	}

	property->lvds_clk = val;

	ret = of_property_read_u8(client->dev.of_node, "dsi-clk-divider", &val);
	if (ret) {
		dev_err(dev, "%s: not found\n", "dsi-clk-divider");
		goto fail;
	}

	property->clk_div = val;

	ret = of_property_read_u8(client->dev.of_node, "cha-dsi-lanes", &val);
	if (ret) {
		dev_err(dev, "%s: not found\n", "cha-dsi-lanes");
		goto fail;
	}

	property->dsi_lane = val;

	ret = of_property_read_u8(client->dev.of_node, "cha-dsi-clk-range", &val);
	if (ret) {
		dev_err(dev, "%s: not found\n", "cha-dsi-clk-range");
		goto fail;
	}

	property->dsi_clk = val;

	ret = of_property_read_u8(client->dev.of_node, "de-neg-polarity", &val);
	if (ret) {
		dev_err(dev, "%s: not found\n", "de-neg-polarity");
		goto fail;
	}

	property->de_pola = val;

	ret = of_property_read_u8(client->dev.of_node, "hs-neg-polarity", &val);
	if (ret) {
		dev_err(dev, "%s: not found\n", "hs-neg-polarity");
		goto fail;
	}

	property->hs_pola = val;

	ret = of_property_read_u8(client->dev.of_node, "vs-neg-polarity", &val);
	if (ret) {
		dev_err(dev, "%s: not found\n", "vs-neg-polarity");
		goto fail;
	}

	property->vs_pola = val;

	ret = of_property_read_u8(client->dev.of_node, "lvds-link-cfg", &val);
	if (ret) {
		dev_err(dev, "%s: not found\n", "lvds-link-cfg");
		goto fail;
	}

	property->lvds_link = val;

	ret = of_property_read_u8(client->dev.of_node, "channel-a-24bpp", &val);
	if (ret) {
		dev_err(dev, "%s: not found\n", "channel-a-24bpp");
		goto fail;
	}

	property->ch_a_24bpp = val;

	ret = of_property_read_u8(client->dev.of_node, "channel-b-24bpp", &val);
	if (ret) {
		dev_err(dev, "%s: not found\n", "channel-b-24bpp");
		goto fail;
	}

	property->ch_b_24bpp = val;

	ret = of_property_read_u16(client->dev.of_node, "sync-delay", &val16);
	if (ret) {
		dev_err(dev, "%s: not found\n", "sync-delay");
		goto fail;
	}

	property->sdelay = val16;

	dev_dbg(&client->dev, "lvds_clk:%#x, clk_div:%#x, dsi_lane:%#x, dsi_clk:%#x, "
					"de_pola:%#x, hs_pola:%#x, vs_pola:%#x, lvds_link:%#x, "
					"ch_a_24bpp:%#x, ch_b_24bpp:%#x, sdelay:%#x\n",
					 property->lvds_clk, property->clk_div, property->dsi_lane, property->dsi_clk,
					 property->de_pola, property->hs_pola, property->vs_pola,
					 property->lvds_link, property->ch_a_24bpp, property->ch_b_24bpp, property->sdelay);

fail:
	return ret;
}

static int sn65dsi84_i2c_set_property(struct i2c_client *client, struct sn65dsi84_data *priv)
{
	int ret, cnt;
	u8 val;
	struct sn65dsi84_property *property = &priv->property;

	if (!gpio_is_valid(priv->en_gpio)) {
		/*
		 * If EN signal is fixed to Vcc, we should set default values to CSRs.
		 * Because the only method to reset CSRs is to deassert EN signal.
		 * (SOFT_RESET does not affect CSRs value)
		 */

		struct csr_setting {
			u8 addr;
			u8 val;
		};

		const struct csr_setting sets[] = {
			{ .addr = 0x0d, .val = 0x00 }, /* PLL disabled */
			{ .addr = 0x11, .val = 0x00 },
			{ .addr = 0x19, .val = 0x05 },
			{ .addr = 0x1a, .val = 0x03 },
			{ .addr = 0x1b, .val = 0x00 },
# if 0
			/* TEST PATTERN GENERATION PURPOSE ONLY. */
			{ .addr = 0x24, .val = 0x00 },
			{ .addr = 0x25, .val = 0x00 },
			{ .addr = 0x36, .val = 0x00 },
			{ .addr = 0x38, .val = 0x00 },
			{ .addr = 0x3a, .val = 0x00 },
			{ .addr = 0x3c, .val = 0x10 }, /* enable test pattern */
#else
			{ .addr = 0x3c, .val = 0x00 },
#endif
			{ .addr = 0xe0, .val = 0x00 },
			{ .addr = 0xe1, .val = 0x00 },
			{ .addr = 0xe5, .val = 0xff }, /* write 1 to clear bit */
		};

		for (cnt = 0; cnt < ARRAY_SIZE(sets); cnt++) {
			ret = sn65dsi84_i2c_write_register(client, sets[cnt].addr, sets[cnt].val);
			if (ret) {
				goto end;
			}
		}
	}

	/*
	 * LVDS_CLK_RANGE -> Device Tree property
	 * HS_CLK_SRC -> MIPI D-PHY
	 */
	val = (((property->lvds_clk & 0x7) << 1) | 0x1);
	ret = sn65dsi84_i2c_write_register(client, 0xa, val);
	if (ret) {
		goto end;
	}

	/* DSI_CLK_DIVIDER -> Device Tree property */
	val = (property->clk_div & 0x1f) << 2;
	ret = sn65dsi84_i2c_write_register(client, 0xb, val);
	if (ret) {
		goto end;
	}

	/*
	 * CHA_DSI_LANES -> Device Tree property
	 * SOT_ERR_TOL_DIS -> Default value
	 */
	val = ((property->dsi_lane & 0x3) << 3 | (0x1 << 5));
	ret = sn65dsi84_i2c_write_register(client, 0x10, val);
	if (ret) {
		goto end;
	}

	/* CHA_DSI_CLK_RANGE -> Device Tree property */
	val = property->dsi_clk;
	ret = sn65dsi84_i2c_write_register(client, 0x12, val);
	if (ret) {
		goto end;
	}

	/*
	 * {DE|HS|VS}_NEG_POLARITY -> Device Tree property
	 * LVDS_LINK_CFG -> Device Tree property
	 * CH{A|B}_24BPP_MODE -> Device Tree property
	 */
	val = ((property->de_pola & 0x1) << 7) |
		((property->hs_pola & 0x1) << 6) |
		((property->vs_pola & 0x1) << 5) |
		((property->lvds_link & 0x1) << 4) |
		((property->ch_a_24bpp & 0x1) << 3) |
		((property->ch_b_24bpp & 0x1) << 2);
	ret = sn65dsi84_i2c_write_register(client, 0x18, val);

	/* CHA_SYNC_DELAY_LOW -> Device Tree property */
	val = (u8)(property->sdelay & 0xff);
	ret = sn65dsi84_i2c_write_register(client, 0x28, val);
	if (ret) {
		goto end;
	}

	/* CHA_SYNC_DELAY_HIGH -> Device Tree property */
	val = (u8)((property->sdelay >> 8) & 0xf);
	ret = sn65dsi84_i2c_write_register(client, 0x29, val);
	if (ret) {
		goto end;
	}

end:
	return ret;
}

static int sn65dsi84_i2c_prepare(struct i2c_client *client, struct sn65dsi84_data *priv)
{
	if (gpio_is_valid(priv->en_gpio)) {
		/* reset device */
		gpio_set_value(priv->en_gpio, 0);
		usleep_range(10000, 10000); /* 10msec */
		gpio_set_value(priv->en_gpio, 1);
		usleep_range(1000, 1000);   /* 1msec */
	}

	/* Set CSRs */
	return sn65dsi84_i2c_set_property(client, priv);
}

int sn65dsi84_i2c_set_timings(struct i2c_client *client, struct display_timing *dt)
{
	struct sn65dsi84_data *priv = i2c_get_clientdata(client);
	int ret;
	u8 val;

	dev_info(&client->dev, "%s\n", __func__);

	if (gpio_is_valid(priv->en_gpio) && !gpio_get_value(priv->en_gpio)) {
		usleep_range(10000, 10000); /* 10msec */
		gpio_set_value(priv->en_gpio, 1);
		usleep_range(1000, 1000);   /* 1msec */

		/* Set CSRs */
		ret = sn65dsi84_i2c_set_property(client, priv);
		if (ret) {
			goto end;
		}
	}

	/* CHA_ACTIVE_LINE_LENGTH_LOW */
	val = (u8)(dt->hactive.typ & 0xff);
	ret = sn65dsi84_i2c_write_register(client, 0x20, val);
	if (ret) {
		goto end;
	}

	/* CHA_ACTIVE_LINE_LENGTH_HIGH */
	val = (u8)((dt->hactive.typ >> 8) & 0xf);
	ret = sn65dsi84_i2c_write_register(client, 0x21, val);
	if (ret) {
		goto end;
	}

	/* CHA_HSYNC_PULSE_WIDTH_LOW */
	val = (u8)(dt->hsync_len.typ & 0xff);
	ret = sn65dsi84_i2c_write_register(client, 0x2c, val);
	if (ret) {
		goto end;
	}

	/* CHA_HSYNC_PULSE_WIDTH_HIGH */
	val = (u8)((dt->hsync_len.typ >> 8) & 0x3);
	ret = sn65dsi84_i2c_write_register(client, 0x2d, val);
	if (ret) {
		goto end;
	}

	/* CHA_VSYNC_PULSE_WIDTH_LOW */
	val = (u8)(dt->vsync_len.typ & 0xff);
	ret = sn65dsi84_i2c_write_register(client, 0x30, val);
	if (ret) {
		goto end;
	}

	/* CHA_VSYNC_PULSE_WIDTH_HIGH */
	val = (u8)((dt->vsync_len.typ >> 8) & 0x3);
	ret = sn65dsi84_i2c_write_register(client, 0x31, val);
	if (ret) {
		goto end;
	}

	/* CHA_HORIZONTAL_BACK_PORCH */
	val = (u8)dt->hback_porch.typ;
	ret = sn65dsi84_i2c_write_register(client, 0x34, val);
	if (ret) {
		goto end;
	}

end:
	dev_info(&client->dev, "%s done(%d).\n", __func__, ret);
	return ret;
}
EXPORT_SYMBOL(sn65dsi84_i2c_set_timings);

int sn65dsi84_i2c_start(struct i2c_client *client, bool exit_ulps)
{
	int ret, cnt;

	dev_info(&client->dev, "%s\n", __func__);

	if (!exit_ulps) {
		/* set PLL_EN */
		ret = sn65dsi84_i2c_write_register(client, 0xd, 1);
		if (ret) {
			goto end;
		}
	}

	/* wait for the PLL_LOCK bit to be set */
	cnt = 0;
	do {
		u8 val;

		usleep_range(1000, 1000);

		ret = sn65dsi84_i2c_read_register(client, 0xa, &val, 1);
		if (ret) {
			goto end;
		}

		if (val & 0x80) {
			break;
		}

		if (++cnt >= 100) {
			ret = -ETIMEDOUT;
			goto end;
		}
	} while(1);

	/* set SOFT_RESET bit */
	ret = sn65dsi84_i2c_write_register(client, 0x9, 0);

end:
	dev_info(&client->dev, "%s done(%d).\n", __func__, ret);
	return ret;
}
EXPORT_SYMBOL(sn65dsi84_i2c_start);

int sn65dsi84_i2c_stop(struct i2c_client *client)
{
	struct sn65dsi84_data *priv = i2c_get_clientdata(client);
	int ret = 0;

	dev_info(&client->dev, "%s\n", __func__);

	if (!gpio_is_valid(priv->en_gpio)) {
		/* clear PLL_EN */
		ret = sn65dsi84_i2c_write_register(client, 0xd, 0);
	}
	else {
		(void)sn65dsi84_i2c_write_register(client, 0xd, 0);
		gpio_set_value(priv->en_gpio, 0);
	}

	dev_info(&client->dev, "%s done(%d).\n", __func__, ret);
	return ret;
}
EXPORT_SYMBOL(sn65dsi84_i2c_stop);

#ifdef DEBUG
static void sn65dsi84_i2c_chk_device(struct i2c_client *client)
{
	int ret;
	u8 vals[9];
	const u8 magic[] = {0x35, 0x38, 0x49, 0x53, 0x44, 0x20, 0x20, 0x20, 0x01};

	ret = sn65dsi84_i2c_read_register(client, 0x0, vals, 9);
	if (ret) {
		return;
	}

	if (memcmp(vals, magic, 9) != 0) {
		/*
		 * XXX: If EN signal is fixed to Vcc, sometimes we read invalid
		 *      value from register 0x0~0x8.
		 *      So this function is only intended to be used for debugging.
		 */

		dev_info(&client->dev, "invalid i2c device?\n");
		dev_info(&client->dev,
					"device magic number: %#x %#x %#x %#x %#x %#x %#x %#x %#x\n",
					vals[0], vals[1], vals[2], vals[3], vals[4],
					vals[5], vals[6], vals[7], vals[8]);
	}

	return;
}
#else
#define sn65dsi84_i2c_chk_device(client)
#endif

static int sn65dsi84_i2c_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	int ret, en_gpio;
	struct sn65dsi84_data *priv;
	struct sn65dsi84_property *property;

	dev_info(dev, "%s\n", __func__);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto end;
	}

	property = &priv->property;
	ret = sn65dsi84_i2c_read_dt_property(client, property);
	if (ret) {
		dev_err(dev, "read dtb property failed.\n");
		goto end;
	}

	en_gpio = of_get_named_gpio(dev->of_node, "en-input-gpio", 0);
	if(gpio_is_valid(en_gpio)) {
		ret = devm_gpio_request_one(dev, en_gpio, GPIOF_OUT_INIT_LOW, "sn65dsi84-en-input");
		if (ret) {
			dev_err(dev, "gpio request error(gpio:%d, ret:%d)\n", en_gpio, ret);
			goto end;
		}
	}

	priv->en_gpio = en_gpio;
	i2c_set_clientdata(client, priv);

	ret = sn65dsi84_i2c_prepare(client, priv);
	if (ret) {
		goto end;
	}

	sn65dsi84_i2c_chk_device(client);
	dev_info(dev, "%s done.\n", __func__);

	return 0;

end:
	return ret;
}

static int sn65dsi84_i2c_remove(struct i2c_client *client)
{
	return 0;
}


static const struct i2c_device_id sn65dsi84_i2c_id[] = {
	{ "sn65dsi84", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sn65dsi84_i2c_id);

static const struct of_device_id sn65dsi84_dt_ids[] = {
	{ .compatible = "fujitsu,sn65dsi84" },
	{ /* sentinel */ }
};

static struct i2c_driver sn65dsi84_driver = {
	.driver = {
		.name = "sn65dsi84_i2c",
		.owner = THIS_MODULE,
		.of_match_table = sn65dsi84_dt_ids,
	},
	.id_table = sn65dsi84_i2c_id,
	.probe = sn65dsi84_i2c_probe,
	.remove = sn65dsi84_i2c_remove,
};

module_i2c_driver(sn65dsi84_driver);

MODULE_LICENSE("GPL v2");
