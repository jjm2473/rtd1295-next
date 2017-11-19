// SPDX-License-Identifier: GPL-2.0+
/*
 * Actions Semi "Owl"
 *
 * Copyright (c) 2017 Andreas Färber
 */

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

struct owl_pin_group_desc {
	const char *name;
	const unsigned int *pins;
	unsigned int num_pins;
};

struct owl_pin_func_desc {
	const char *name;
	const char * const *groups;
	unsigned int num_groups;
};

struct owl_pin_mux_desc {
	const char *name;
	u32 mux_value;
};

struct owl_pin_desc {
	const char *name;
	unsigned int mux_offset;
	u32 mux_mask;
	const struct owl_pin_mux_desc *functions;
};

struct owl_pinctrl_desc {
	const struct pinctrl_pin_desc *pins;
	unsigned int num_pins;
	const struct owl_pin_group_desc *groups;
	unsigned int num_groups;
	const struct owl_pin_func_desc *functions;
	unsigned int num_functions;
	const struct owl_pin_desc *muxes;
	unsigned int num_muxes;
};

#define OWL_PIN_MUX(_name, _mux_off, _mux_mask, ...) \
	{ \
		.name = # _name, \
		.mux_offset = _mux_off, \
		.mux_mask = _mux_mask, \
		.functions = (const struct owl_pin_mux_desc []) { \
			__VA_ARGS__, { } \
		}, \
	}

#define OWL_PIN_FUNC(_mux_val, _name) \
	{ \
		.name = _name, \
		.mux_value = _mux_val, \
	}

#include "pinctrl-owl-s500.h"

struct owl_pinctrl {
	struct pinctrl_dev *pcdev;
	void __iomem *mfp_base;
	struct pinctrl_desc desc;
	const struct owl_pinctrl_desc *info;
};

static int owl_pinctrl_get_groups_count(struct pinctrl_dev *pcdev)
{
	struct owl_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);

	return data->info->num_groups;
}

static const char *owl_pinctrl_get_group_name(struct pinctrl_dev *pcdev,
		unsigned selector)
{
	struct owl_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);

	return data->info->groups[selector].name;
}

static int owl_pinctrl_get_group_pins(struct pinctrl_dev *pcdev,
		unsigned selector, const unsigned **pins, unsigned *num_pins)
{
	struct owl_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);

	*pins		= data->info->groups[selector].pins;
	*num_pins	= data->info->groups[selector].num_pins;

	return 0;
}

static const struct pinctrl_ops owl_pinctrl_ops = {
	.dt_node_to_map = pinconf_generic_dt_node_to_map_all,
	.dt_free_map = pinctrl_utils_free_map,
	.get_groups_count = owl_pinctrl_get_groups_count,
	.get_group_name = owl_pinctrl_get_group_name,
	.get_group_pins = owl_pinctrl_get_group_pins,
};

static int owl_pinctrl_get_functions_count(struct pinctrl_dev *pcdev)
{
	struct owl_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);

	return data->info->num_functions;
}

static const char *owl_pinctrl_get_function_name(struct pinctrl_dev *pcdev,
		unsigned selector)
{
	struct owl_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);

	return data->info->functions[selector].name;
}

static int owl_pinctrl_get_function_groups(struct pinctrl_dev *pcdev,
		unsigned selector, const char * const **groups,
		unsigned * const num_groups)
{
	struct owl_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);

	*groups		= data->info->functions[selector].groups;
	*num_groups	= data->info->functions[selector].num_groups;

	return 0;
}

static const struct owl_pin_desc *owl_pinctrl_find_mux(struct owl_pinctrl *data,
		const char *name)
{
	int i;

	for (i = 0; i < data->info->num_muxes; i++) {
		if (strcmp(data->info->muxes[i].name, name) == 0)
			return &data->info->muxes[i];
	}

	return NULL;
}

static int owl_pinctrl_set_mux(struct pinctrl_dev *pcdev,
		unsigned function, unsigned group)
{
	struct owl_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);
	const struct owl_pin_desc *mux;
	const char *func_name;
	const char *group_name;
	u32 val;
	int i;

	func_name = data->info->functions[function].name;
	group_name = data->info->groups[group].name;

	mux = owl_pinctrl_find_mux(data, group_name);
	if (!mux)
		return -ENOTSUPP;

	if (!mux->functions) {
		dev_err(pcdev->dev, "No functions available for group %s\n", group_name);
		return -ENOTSUPP;
	}

	for (i = 0; mux->functions[i].name; i++) {
		if (strcmp(mux->functions[i].name, func_name) != 0)
			continue;

		if (!mux->mux_mask)
			return -ENOTSUPP;

		val = readl_relaxed(data->mfp_base + mux->mux_offset);
		val &= ~mux->mux_mask;
		val |= (mux->functions[i].mux_value << __ffs(mux->mux_mask)) & mux->mux_mask;
		writel_relaxed(val, data->mfp_base + mux->mux_offset);
		return 0;
	}

	dev_err(pcdev->dev, "No function %s available for group %s\n", func_name, group_name);
	return -EINVAL;
}

static const struct pinmux_ops owl_pinmux_ops = {
	.get_functions_count = owl_pinctrl_get_functions_count,
	.get_function_name = owl_pinctrl_get_function_name,
	.get_function_groups = owl_pinctrl_get_function_groups,
	.set_mux = owl_pinctrl_set_mux,
};

static int owl_pin_config_get(struct pinctrl_dev *pcdev, unsigned pinnr,
		unsigned long *config)
{
	//struct owl_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);
	unsigned int param = pinconf_to_config_param(*config);
	unsigned int arg = 0;

	switch (param) {
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);
	return 0;
}

static int owl_pin_config_set(struct pinctrl_dev *pcdev, unsigned pinnr,
		unsigned long *configs, unsigned num_configs)
{
	//struct owl_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);

	return 0;
}

static const struct pinconf_ops owl_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = owl_pin_config_get,
	.pin_config_set = owl_pin_config_set,
};

static const struct of_device_id owl_pinctrl_dt_ids[] = {
	 { .compatible = "actions,s500-pinctrl", .data = &s500_pinctrl_desc },
	 { .compatible = "actions,s700-pinctrl", .data = &s500_pinctrl_desc },
	 { .compatible = "actions,s900-pinctrl", .data = &s500_pinctrl_desc },
	 { }
};

static int owl_pinctrl_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct owl_pinctrl *data;
	struct resource *res;

	match = of_match_node(owl_pinctrl_dt_ids, pdev->dev.of_node);
	if (!match)
		return -EINVAL;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->mfp_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->mfp_base))
		return PTR_ERR(data->mfp_base);

	data->info = match->data;
	data->desc.name = "owl";
	data->desc.pins = data->info->pins;
	data->desc.npins = data->info->num_pins;
	data->desc.pctlops = &owl_pinctrl_ops;
	data->desc.pmxops = &owl_pinmux_ops;
	data->desc.confops = &owl_pinconf_ops;
	data->desc.custom_params = NULL;
	data->desc.num_custom_params = 0;
	data->desc.owner = THIS_MODULE;

	data->pcdev = pinctrl_register(&data->desc, &pdev->dev, data);
	if (!data->pcdev)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);

	dev_info(&pdev->dev, "probed\n");

	return 0;
}

static struct platform_driver owl_pinctrl_driver = {
	.probe = owl_pinctrl_probe,
	.driver = {
		.name = "owl-pinctrl",
		.of_match_table	= owl_pinctrl_dt_ids,
	},
};
builtin_platform_driver(owl_pinctrl_driver);
