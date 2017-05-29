/*
 * Copyright (C) 2015 Socionext Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/backlight.h>

static int bl_gpio_pin = -EINVAL;
static int bl_en;
static bool bl_sleep;

static void gpio_bl_set_intensity(int intensity);

static struct generic_bl_info gpio_bl_info = {
	.name = "gpio-backlight",
	.max_intensity = 0xff,
	.default_intensity = 0xff,
	.set_bl_intensity = gpio_bl_set_intensity,
};

static struct platform_device gpio_bl_dev = {
	.name = "generic-bl",
	.id = 1,
	.dev = {
		.platform_data	= &gpio_bl_info,
	},
};


static void gpio_bl_set_intensity(int intensity)
{
	int val = intensity ? bl_en : !bl_en;

	if (bl_sleep) {
		WARN_ON(in_interrupt());
		gpio_set_value_cansleep(bl_gpio_pin, val);
	}
	else {
		gpio_set_value(bl_gpio_pin, val);
	}
}

static int gpio_bl_probe(struct platform_device *pdev)
{
	int ret, pin;
	unsigned long out_init;
	enum of_gpio_flags flags;

	pin = of_get_named_gpio_flags(pdev->dev.of_node, "enable-gpio", 0, &flags);
	if (!gpio_is_valid(pin)) {
		if (pin != -EPROBE_DEFER) {
			dev_err(&pdev->dev, "valid gpio is missing(ret:%d)\n", pin);
		}

		ret = pin;
		goto end;
	}

	out_init = (flags & OF_GPIO_ACTIVE_LOW) ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW;
	ret = devm_gpio_request_one(&pdev->dev, pin, out_init, "gpio-backlight");
	if (ret) {
		if (ret != -EPROBE_DEFER) {
			dev_err(&pdev->dev, "request gpio error(ret:%d, gpio:%d)\n", ret, pin);
		}

		goto end;
	}

	bl_gpio_pin = pin;
	bl_en = (flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1;
	bl_sleep = gpio_cansleep(pin);

	ret = platform_device_register(&gpio_bl_dev);
	if (ret) {
		dev_err(&pdev->dev, "platform_device_register error(%d)", ret);
	}

end:
	return ret;
}

static int gpio_bl_remove(struct platform_device *pdev)
{
	platform_device_unregister(&gpio_bl_dev);
	return 0;
}


static const struct of_device_id gpio_bl_dt_ids[] = {
	{ .compatible = "generic,gpio-backlight" },
	{ /* sentinel */ }
};

static struct platform_driver gpio_bl_driver = {
	.probe = gpio_bl_probe,
	.remove = gpio_bl_remove,
	.driver = {
		.name = "gpio-backlight",
		.of_match_table = gpio_bl_dt_ids,
	},
};

MODULE_DEVICE_TABLE(of, gpio_bl_dt_ids);

static int __init gpio_bl_init(void)
{
	return platform_driver_register(&gpio_bl_driver);
}

static void __exit gpio_bl_exit(void)
{
	platform_driver_unregister(&gpio_bl_driver);
}

module_init(gpio_bl_init);
module_exit(gpio_bl_exit);

MODULE_LICENSE("GPL v2");
