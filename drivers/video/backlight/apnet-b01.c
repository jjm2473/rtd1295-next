/*
 *  Backlight driver for Alpha project B01 panel
 *  (C) 2014 Andy Green <andy.green@linaro.org>
 *
 *  based on -->
 *  LCD / Backlight control code for Sharp SL-6000x (tosa)
 *
 *  Copyright (c) 2005		Dirk Opfer
 *  Copyright (c) 2007,2008	Dmitry Baryshkov
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <uapi/linux/input.h>
#if defined(CONFIG_ARCH_MB8AC0300)
#include <linux/irqchip/irq-mb8ac0300.h>
#endif


enum {
	REG_VERSION,
	REG_INT_STATUS,
	REG_INT_MASK,
	REG_BACKLIGHT,

	REG_MAX, /* always last */

	B01_IRQ_REASON_TP     = 1 << 0,
	B01_IRQ_REASON_SW1INT = 1 << 2,
	B01_IRQ_REASON_SW2INT = 1 << 3,
	B01_IRQ_REASON_SW3INT = 1 << 4,

	B01_IRQ_SW_MASK = B01_IRQ_REASON_SW1INT | B01_IRQ_REASON_SW2INT | B01_IRQ_REASON_SW3INT,
};

struct apnetb01_bl_priv {
	struct i2c_client *i2c;
	struct regmap *regmap;
	struct backlight_device *bl;
	struct input_dev *input_dev;
	int irq;
	unsigned short keymap[3];
	u8 old_state;
};


static int apnetb01_bl_update_status(struct backlight_device *dev)
{
	struct backlight_properties *props = &dev->props;
	struct apnetb01_bl_priv *priv = bl_get_data(dev);
	int power = max(props->power, props->fb_blank);
	int brightness = props->brightness;

	if (power)
		brightness = 0;

	regmap_write(priv->regmap, REG_BACKLIGHT, brightness);

	return 0;
}

static int apnetb01_bl_get_brightness(struct backlight_device *dev)
{
	struct apnetb01_bl_priv *priv = bl_get_data(dev);
	unsigned int reg_val;
	int ret;

	ret = regmap_read(priv->regmap, REG_BACKLIGHT, &reg_val);
	if (ret < 0)
		return ret;

	return reg_val;
}

static const struct backlight_ops bl_ops = {
	.get_brightness		= apnetb01_bl_get_brightness,
	.update_status		= apnetb01_bl_update_status,
};

static const struct regmap_config apnetb01_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_MAX,
};

static irqreturn_t apnetb01_isr(int irq, void *dev_id)
{
	struct apnetb01_bl_priv *priv = dev_id;
	u32 reg;
	int ret;


	ret = regmap_read(priv->regmap, REG_INT_STATUS, &reg);
	if (ret < 0) {
		dev_err(&priv->i2c->dev, "Unable to read irq status\n");
		goto bail;
	}

	if ((reg & B01_IRQ_SW_MASK) != priv->old_state) {

		input_report_key(priv->input_dev,
			     priv->keymap[0], !!(reg & B01_IRQ_REASON_SW1INT));
		input_report_key(priv->input_dev,
			     priv->keymap[1], !!(reg & B01_IRQ_REASON_SW2INT));
		input_report_key(priv->input_dev,
			     priv->keymap[2], !!(reg & B01_IRQ_REASON_SW3INT));

		input_sync(priv->input_dev);

		priv->old_state = reg & B01_IRQ_SW_MASK;
	}

	if (reg & B01_IRQ_REASON_TP)
		/* touchpanel handled by another driver */
		return IRQ_NONE;

bail:
	return IRQ_HANDLED;
}


static int apnetb01_bl_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct backlight_properties props;
	struct apnetb01_bl_priv *priv;
	int ret = 0;
	unsigned int reg_val;
	u32 key;

	priv = devm_kzalloc(&client->dev, sizeof(struct apnetb01_bl_priv),
				GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->input_dev = devm_input_allocate_device(&client->dev);
	if (!priv->input_dev)
		return -ENOMEM;

	priv->keymap[0] = KEY_POWER;
	if (!of_property_read_u32(client->dev.of_node, "key1", &key))
		priv->keymap[0] = key;
	priv->keymap[1] = KEY_HOME;
	if (!of_property_read_u32(client->dev.of_node, "key2", &key))
		priv->keymap[1] = key;
	priv->keymap[2] = KEY_ENTER;
	if (!of_property_read_u32(client->dev.of_node, "key3", &key))
		priv->keymap[2] = key;

	priv->input_dev->evbit[0] = BIT_MASK(EV_KEY);

	priv->input_dev->name = dev_name(&client->dev);
	priv->input_dev->phys = "input/input0";
	priv->input_dev->id.bustype = BUS_HOST;
	priv->input_dev->id.vendor = 0x0001;
	priv->input_dev->id.product = 0x0001;
	priv->input_dev->id.version = 0x0100;



	priv->input_dev->phys = "apnetb01-buttons/input0";
	priv->input_dev->dev.parent = &client->dev;
	__set_bit(INPUT_PROP_DIRECT, priv->input_dev->propbit);

	input_set_capability(priv->input_dev, EV_KEY, priv->keymap[0]);
	input_set_capability(priv->input_dev, EV_KEY, priv->keymap[1]);
	input_set_capability(priv->input_dev, EV_KEY, priv->keymap[2]);

	ret = input_register_device(priv->input_dev);
	if (ret) {
		dev_err(&client->dev, "Can't register keyboard: %d\n", ret);
		goto failed_ird;
	}

	priv->irq = client->irq;

	i2c_set_clientdata(client, priv);
	priv->i2c = client;

	priv->regmap = devm_regmap_init_i2c(client, &apnetb01_regmap);
	if (IS_ERR(priv->regmap)) {
		dev_err(&client->dev, "Regmap failed\n");
		goto failed_regmap;
	}

	ret = regmap_read(priv->regmap, REG_VERSION, &reg_val);
	if (ret < 0)
		goto failed_regmap;

#if defined(CONFIG_ARCH_MB8AC0300)
	ret = exiu_irq_set_type(client->irq, IRQ_TYPE_LEVEL_HIGH);
	if (ret) {
		dev_err(&client->dev, "Failed to set irq type to exiu\n");
		goto failed_regmap;
	}
#endif

	ret = devm_request_threaded_irq(
		&client->dev, priv->irq, NULL, apnetb01_isr, IRQF_TRIGGER_HIGH | IRQF_ONESHOT | IRQF_SHARED,
							    "apnet-b01", priv);
	if (ret < 0) {
		dev_err(&client->dev, "Unable to request irq %d\n", priv->irq);
		goto failed_regmap;
	}


	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = 100;
	priv->bl = backlight_device_register(
			"apnetb01-bl", &client->dev, priv, &bl_ops, &props);
	if (IS_ERR(priv->bl)) {
		ret = PTR_ERR(priv->bl);
		goto failed_reg;
	}

	priv->bl->props.brightness = 100;
	priv->bl->props.power = FB_BLANK_UNBLANK;

	backlight_update_status(priv->bl);

	dev_info(&client->dev, "Registered bl on panel ver: 0x%x\n", reg_val);

	return 0;

failed_reg:
	free_irq(priv->irq, priv);
failed_regmap:
	input_unregister_device(priv->input_dev);
	input_free_device(priv->input_dev);
failed_ird:
	kfree(priv);

	priv->bl = NULL;
	return ret;
}

static int apnetb01_bl_remove(struct i2c_client *client)
{
	struct apnetb01_bl_priv *priv = i2c_get_clientdata(client);

	backlight_device_unregister(priv->bl);
	free_irq(priv->irq, priv);
	priv->bl = NULL;
	input_free_device(priv->input_dev);
	kfree(priv);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int apnetb01_bl_suspend(struct device *dev)
{
	struct apnetb01_bl_priv *priv = dev_get_drvdata(dev);

	regmap_write(priv->regmap, REG_BACKLIGHT, 0);	

	return 0;
}

static int apnetb01_bl_resume(struct device *dev)
{
	struct apnetb01_bl_priv *priv = dev_get_drvdata(dev);

	backlight_update_status(priv->bl);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(apnetb01_bl_pm_ops,
				apnetb01_bl_suspend, apnetb01_bl_resume);

static const struct i2c_device_id apnetb01_bl_id[] = {
	{ "apnetb01-bl", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, apnetb01_bl_id);

static const struct of_device_id apnetb01_dt_ids[] = {
	{ .compatible = "apnet,b01-bl" },
	{ /* sentinel */ }
};

static struct i2c_driver apnetb01_bl_driver = {
	.driver = {
		.name		= "apnetb01-bl",
		.owner		= THIS_MODULE,
		.pm		= &apnetb01_bl_pm_ops,
		.of_match_table = apnetb01_dt_ids,
	},
	.probe		= apnetb01_bl_probe,
	.remove		= apnetb01_bl_remove,
	.id_table	= apnetb01_bl_id,
};

module_i2c_driver(apnetb01_bl_driver);

MODULE_AUTHOR("Andy Green");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("LCD/Backlight control for Alpha Project B01 Panel");
MODULE_ALIAS("*b01-bl*");
