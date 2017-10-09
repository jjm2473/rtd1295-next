/*
 * Copyright (c) 2017 Andreas Färber
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include "core.h"
#include "pinctrl-utils.h"

struct rtd119x_pin_group_desc {
	const char *name;
	const unsigned int *pins;
	unsigned int num_pins;
};

struct rtd119x_pin_func_desc {
	const char *name;
	const char * const *groups;
	unsigned int num_groups;
};

struct rtd119x_pin_mux_desc {
	const char *name;
	u32 mux_value;
};

struct rtd119x_pin_desc {
	const char *name;
	unsigned int mux_offset;
	u32 mux_mask;
	const struct rtd119x_pin_mux_desc *functions;
};

#define RTK_PIN_MUX(_name, _mux_off, _mux_mask, ...) \
	{ \
		.name = # _name, \
		.mux_offset = _mux_off, \
		.mux_mask = _mux_mask, \
		.functions = (const struct rtd119x_pin_mux_desc []) { \
			__VA_ARGS__, { } \
		}, \
	}

#define RTK_PIN_FUNC(_mux_val, _name) \
	{ \
		.name = _name, \
		.mux_value = _mux_val, \
	}

struct rtd119x_pinctrl_desc {
	const struct pinctrl_pin_desc *pins;
	unsigned int num_pins;
	const struct rtd119x_pin_group_desc *groups;
	unsigned int num_groups;
	const struct rtd119x_pin_func_desc *functions;
	unsigned int num_functions;
	const struct rtd119x_pin_desc *muxes;
	unsigned int num_muxes;
};

#include "pinctrl-rtd1295.h"

struct rtd119x_pinctrl {
	struct pinctrl_dev *pcdev;
	void __iomem *base;
	struct pinctrl_desc desc;
	const struct rtd119x_pinctrl_desc *info;
};

static int rtd119x_pinctrl_get_groups_count(struct pinctrl_dev *pcdev)
{
	struct rtd119x_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);

	return data->info->num_groups;
}

static const char *rtd119x_pinctrl_get_group_name(struct pinctrl_dev *pcdev,
		unsigned selector)
{
	struct rtd119x_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);

	return data->info->groups[selector].name;
}

static int rtd119x_pinctrl_get_group_pins(struct pinctrl_dev *pcdev,
		unsigned selector, const unsigned **pins, unsigned *num_pins)
{
	struct rtd119x_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);

	*pins		= data->info->groups[selector].pins;
	*num_pins	= data->info->groups[selector].num_pins;

	return 0;
}

static const struct pinctrl_ops rtd119x_pinctrl_ops = {
	.dt_node_to_map = pinconf_generic_dt_node_to_map_all,
	.dt_free_map = pinctrl_utils_free_map,
	.get_groups_count = rtd119x_pinctrl_get_groups_count,
	.get_group_name = rtd119x_pinctrl_get_group_name,
	.get_group_pins = rtd119x_pinctrl_get_group_pins,
};

static int rtd119x_pinctrl_get_functions_count(struct pinctrl_dev *pcdev)
{
	struct rtd119x_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);

	return data->info->num_functions;
}

static const char *rtd119x_pinctrl_get_function_name(struct pinctrl_dev *pcdev,
		unsigned selector)
{
	struct rtd119x_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);

	return data->info->functions[selector].name;
}

static int rtd119x_pinctrl_get_function_groups(struct pinctrl_dev *pcdev,
		unsigned selector, const char * const **groups,
		unsigned * const num_groups)
{
	struct rtd119x_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);

	*groups		= data->info->functions[selector].groups;
	*num_groups	= data->info->functions[selector].num_groups;

	return 0;
}

static const struct pinctrl_pin_desc *rtd119x_pinctrl_get_pin_by_number(struct rtd119x_pinctrl *data, int number)
{
	int i;

	for (i = 0; i < data->info->num_pins; i++) {
		if (data->info->pins[i].number == number)
			return &data->info->pins[i];
	}

	return NULL;
}

static const struct rtd119x_pin_desc *rtd119x_pinctrl_find_mux(struct rtd119x_pinctrl *data, const char *name)
{
	int i;

	for (i = 0; i < data->info->num_muxes; i++) {
		if (strcmp(data->info->muxes[i].name, name) == 0)
			return &data->info->muxes[i];
	}

	return NULL;
}

static int rtd119x_pinctrl_set_one_mux(struct pinctrl_dev *pcdev,
	unsigned int pin, const char *func_name)
{
	struct rtd119x_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);
	const struct pinctrl_pin_desc *pin_desc;
	const struct rtd119x_pin_desc *mux;
	const char *pin_name;
	u32 val;
	int i;

	pin_desc = rtd119x_pinctrl_get_pin_by_number(data, pin);
	if (!pin_desc)
		return -ENOTSUPP;

	pin_name = pin_desc->name;

	mux = rtd119x_pinctrl_find_mux(data, pin_name);
	if (!mux)
		return -ENOTSUPP;

	if (!mux->functions) {
		dev_err(pcdev->dev, "No functions available for pin %s\n", pin_name);
		return -ENOTSUPP;
	}

	for (i = 0; mux->functions[i].name; i++) {
		if (strcmp(mux->functions[i].name, func_name) != 0)
			continue;

		val = readl_relaxed(data->base + mux->mux_offset);
		val &= ~mux->mux_mask;
		val |= mux->functions[i].mux_value & mux->mux_mask;
		writel_relaxed(val, data->base + mux->mux_offset);
		return 0;
	}

	dev_err(pcdev->dev, "No function %s available for pin %s\n", func_name, pin_name);
	return -EINVAL;
}

static int rtd119x_pinctrl_set_mux(struct pinctrl_dev *pcdev,
		unsigned function, unsigned group)
{
	struct rtd119x_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);
	const unsigned int *pins;
	unsigned int num_pins;
	const char *func_name;
	const char *group_name;
	int i, ret;

	func_name = data->info->functions[function].name;
	group_name = data->info->groups[group].name;

	ret = rtd119x_pinctrl_get_group_pins(pcdev, group, &pins, &num_pins);
	if (ret) {
		dev_err(pcdev->dev, "Getting pins for group %s failed\n", group_name);
		return ret;
	}

	for (i = 0; i < num_pins; i++) {
		ret = rtd119x_pinctrl_set_one_mux(pcdev, pins[i], func_name);
		if (ret)
			return ret;
	}

	return 0;
}

static int rtd119x_pinctrl_gpio_request_enable(struct pinctrl_dev *pcdev,
	struct pinctrl_gpio_range *range, unsigned offset)
{
	return rtd119x_pinctrl_set_one_mux(pcdev, offset, "gpio");
}

static const struct pinmux_ops rtd119x_pinmux_ops = {
	.get_functions_count = rtd119x_pinctrl_get_functions_count,
	.get_function_name = rtd119x_pinctrl_get_function_name,
	.get_function_groups = rtd119x_pinctrl_get_function_groups,
	.set_mux = rtd119x_pinctrl_set_mux,
	.gpio_request_enable = rtd119x_pinctrl_gpio_request_enable,
};

static int rtd119x_pin_config_get(struct pinctrl_dev *pcdev, unsigned pinnr,
		unsigned long *config)
{
	//struct rtd119x_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);
	unsigned int param = pinconf_to_config_param(*config);
	unsigned int arg = 0;

	switch (param) {
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);
	return 0;
}

static int rtd119x_pin_config_set(struct pinctrl_dev *pcdev, unsigned pinnr,
		unsigned long *configs, unsigned num_configs)
{
	//struct rtd119x_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);

	return 0;
}

static const struct pinconf_ops rtd119x_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = rtd119x_pin_config_get,
	.pin_config_set = rtd119x_pin_config_set,
};

static void rtd119x_pinctrl_selftest(struct rtd119x_pinctrl *data)
{
	int i, j, k;

	for (i = 0; i < data->info->num_muxes; i++) {
		/* Check for pin */
		for (j = 0; j < data->info->num_pins; j++) {
			if (strcmp(data->info->pins[j].name, data->info->muxes[i].name) == 0)
				break;
		}
		if (j == data->info->num_pins)
			dev_warn(data->pcdev->dev, "Mux %s lacking matching pin\n",
				 data->info->muxes[i].name);

		/* Check for group */
		for (j = 0; j < data->info->num_groups; j++) {
			if (strcmp(data->info->groups[j].name, data->info->muxes[i].name) == 0)
				break;
		}
		if (j == data->info->num_groups)
			dev_warn(data->pcdev->dev, "Mux %s lacking matching group\n",
				 data->info->muxes[i].name);

		for (j = 0; data->info->muxes[i].functions[j].name; j++) {
			/* Check for function */
			for (k = 0; k < data->info->num_functions; k++) {
				if (strcmp(data->info->functions[k].name,
				           data->info->muxes[i].functions[j].name) == 0)
					break;
			}
			if (k == data->info->num_functions)
				dev_warn(data->pcdev->dev, "Mux %s lacking function %s\n",
					 data->info->muxes[i].name,
					 data->info->muxes[i].functions[j].name);

			/* Check for duplicate mux value - assumption: ascending order */
			if (j > 0 && data->info->muxes[i].functions[j].mux_value
			          <= data->info->muxes[i].functions[j - 1].mux_value)
				dev_warn(data->pcdev->dev, "Mux %s function %s has unexpected value\n",
					 data->info->muxes[i].name,
					 data->info->muxes[i].functions[j].name);
		}
	}
}

static const struct of_device_id rtd119x_pinctrl_dt_ids[] = {
	 { .compatible = "realtek,rtd1295-iso-pinctrl", .data = &rtd1295_iso_pinctrl_desc },
	 { .compatible = "realtek,rtd1295-sb2-pinctrl", .data = &rtd1295_sb2_pinctrl_desc },
	 { .compatible = "realtek,rtd1295-disp-pinctrl", .data = &rtd1295_disp_pinctrl_desc },
	 { .compatible = "realtek,rtd1295-cr-pinctrl", .data = &rtd1295_cr_pinctrl_desc },
	 { }
};

static int rtd119x_pinctrl_probe(struct platform_device *pdev)
{
	struct rtd119x_pinctrl *data;
	const struct of_device_id *match;

	match = of_match_node(rtd119x_pinctrl_dt_ids, pdev->dev.of_node);
	if (!match)
		return -EINVAL;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->base = of_iomap(pdev->dev.of_node, 0);
	if (IS_ERR(data->base))
		return PTR_ERR(data->base);

	data->info = match->data;
	data->desc.name = "foo";
	data->desc.pins = data->info->pins;
	data->desc.npins = data->info->num_pins;
	data->desc.pctlops = &rtd119x_pinctrl_ops;
	data->desc.pmxops = &rtd119x_pinmux_ops;
	data->desc.confops = &rtd119x_pinconf_ops;
	data->desc.custom_params = NULL;
	data->desc.num_custom_params = 0;
	data->desc.owner = THIS_MODULE;

	data->pcdev = pinctrl_register(&data->desc, &pdev->dev, data);
	if (!data->pcdev)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);

	rtd119x_pinctrl_selftest(data);

	dev_info(&pdev->dev, "probed\n");

	return 0;
}

static struct platform_driver rtd119x_pinctrl_driver = {
	.probe = rtd119x_pinctrl_probe,
	.driver = {
		.name = "rtd1295-pinctrl",
		.of_match_table	= rtd119x_pinctrl_dt_ids,
	},
};
builtin_platform_driver(rtd119x_pinctrl_driver);
