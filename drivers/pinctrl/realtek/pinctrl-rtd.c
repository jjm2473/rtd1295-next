// SPDX-License-Identifier: GPL-2.0-or-later OR BSD-2-Clause

/*
 * Realtek DHC pin controller driver
 *
 * Copyright (c) 2019 Realtek Semiconductor Corp.
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
#include "../core.h"
#include "../pinctrl-utils.h"

struct rtd_pin_group_desc {
	const char *name;
	const unsigned int *pins;
	unsigned int num_pins;
};

struct rtd_pin_func_desc {
	const char *name;
	const char * const *groups;
	unsigned int num_groups;
};

struct rtd_pin_mux_desc {
	const char *name;
	u32 mux_value;
};

struct rtd_pin_sconfig_desc {
	const char *name;
	unsigned int reg_offset;
	unsigned int ndrive_offset;
	unsigned int ndrive_maskbits;
	unsigned int pdrive_offset;
	unsigned int pdrive_maskbits;
};

struct rtd_pin_config_desc {
	const char *name;
	unsigned int reg_offset;
	unsigned int base_bit;
	unsigned int pud_en_offset;
	unsigned int pud_sel_offset;
	unsigned int curr_offset;
	unsigned int smt_offset;
	unsigned int curr_type;
};

struct rtd_pin_desc {
	const char *name;
	unsigned int mux_offset;
	u32 mux_mask;
	const struct rtd_pin_mux_desc *functions;
};

#define RTK_PIN_SCONFIG(_name, _reg_off, _n_offset, _n_mask, _p_offset, _p_mask) \
		{ \
			.name = # _name, \
			.reg_offset = _reg_off, \
			.ndrive_offset = _n_offset, \
			.ndrive_maskbits = _n_mask, \
			.pdrive_offset = _p_offset, \
			.pdrive_maskbits = _p_mask, \
		}


struct rtd_pin_reg_list {
	unsigned int reg_offset;
	unsigned int val;
};


#define RTK_PIN_CONFIG(_name, _reg_off, _base_bit, _pud_en_off, \
		_pud_sel_off, _curr_off, _smt_off, _curr_type) \
	{ \
		.name = # _name, \
		.reg_offset = _reg_off, \
		.base_bit = _base_bit, \
		.pud_en_offset = _pud_en_off, \
		.pud_sel_offset = _pud_sel_off, \
		.curr_offset = _curr_off, \
		.smt_offset = _smt_off, \
		.curr_type = _curr_type, \
	}


#define RTK_PIN_MUX(_name, _mux_off, _mux_mask, ...) \
	{ \
		.name = # _name, \
		.mux_offset = _mux_off, \
		.mux_mask = _mux_mask, \
		.functions = (const struct rtd_pin_mux_desc []) { \
			__VA_ARGS__, { } \
		}, \
	}

#define RTK_PIN_FUNC(_mux_val, _name) \
	{ \
		.name = _name, \
		.mux_value = _mux_val, \
	}

struct rtd_pinctrl_desc {
	const struct pinctrl_pin_desc *pins;
	unsigned int num_pins;
	const struct rtd_pin_group_desc *groups;
	unsigned int num_groups;
	const struct rtd_pin_func_desc *functions;
	unsigned int num_functions;
	const struct rtd_pin_desc *muxes;
	unsigned int num_muxes;
	const struct rtd_pin_config_desc *configs;
	unsigned int num_configs;
	const struct rtd_pin_sconfig_desc *sconfigs;
	unsigned int num_sconfigs;
	struct rtd_pin_reg_list *lists;
	unsigned int num_regs;
};

#define PCONF_UNSUPP 0xffffffff
#define PADDRI_4_8 1
#define PADDRI_2_4 0


#include "pinctrl-rtd1195.h"
#include "pinctrl-rtd1295.h"

struct rtd_pinctrl {
	struct pinctrl_dev *pcdev;
	void __iomem *base;
	struct pinctrl_desc desc;
	const struct rtd_pinctrl_desc *info;
};

/* custom pinconf parameters */
#define RTK_P_DRIVE	(PIN_CONFIG_END + 1)
#define RTK_N_DRIVE	(PIN_CONFIG_END + 2)

static const struct pinconf_generic_params rtd_custom_bindings[] = {
	{"rtk,pdrive",  RTK_P_DRIVE,	0},
	{"rtk,ndrive",	RTK_N_DRIVE,	0},
};

static int rtd_pinctrl_get_groups_count(struct pinctrl_dev *pcdev)
{
	struct rtd_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);

	return data->info->num_groups;
}

static const char *rtd_pinctrl_get_group_name(struct pinctrl_dev *pcdev,
		unsigned int selector)
{
	struct rtd_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);

	return data->info->groups[selector].name;
}

static int rtd_pinctrl_get_group_pins(struct pinctrl_dev *pcdev,
		unsigned int selector, const unsigned int **pins,
		unsigned int *num_pins)
{
	struct rtd_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);

	*pins		= data->info->groups[selector].pins;
	*num_pins	= data->info->groups[selector].num_pins;

	return 0;
}

static const struct pinctrl_ops rtd_pinctrl_ops = {
	.dt_node_to_map = pinconf_generic_dt_node_to_map_all,
	.dt_free_map = pinctrl_utils_free_map,
	.get_groups_count = rtd_pinctrl_get_groups_count,
	.get_group_name = rtd_pinctrl_get_group_name,
	.get_group_pins = rtd_pinctrl_get_group_pins,
};

static int rtd_pinctrl_get_functions_count(struct pinctrl_dev *pcdev)
{
	struct rtd_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);

	return data->info->num_functions;
}

static const char *rtd_pinctrl_get_function_name(struct pinctrl_dev *pcdev,
		unsigned int selector)
{
	struct rtd_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);

	return data->info->functions[selector].name;
}

static int rtd_pinctrl_get_function_groups(struct pinctrl_dev *pcdev,
		unsigned int selector, const char * const **groups,
		unsigned int * const num_groups)
{
	struct rtd_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);

	*groups		= data->info->functions[selector].groups;
	*num_groups	= data->info->functions[selector].num_groups;

	return 0;
}

static const struct pinctrl_pin_desc *rtd_pinctrl_get_pin_by_number(
		struct rtd_pinctrl *data, int number)
{
	int i;

	for (i = 0; i < data->info->num_pins; i++) {
		if (data->info->pins[i].number == number)
			return &data->info->pins[i];
	}

	return NULL;
}

static const struct rtd_pin_desc *rtd_pinctrl_find_mux(
		struct rtd_pinctrl *data, const char *name)
{
	int i;

	for (i = 0; i < data->info->num_muxes; i++) {
		if (strcmp(data->info->muxes[i].name, name) == 0)
			return &data->info->muxes[i];
	}

	return NULL;
}

static const struct rtd_pin_config_desc *rtd_pinctrl_find_config(
		struct rtd_pinctrl *data, const char *name)
{
	int i;

	for (i = 0; i < data->info->num_configs; i++) {
		if (strcmp(data->info->configs[i].name, name) == 0)
			return &data->info->configs[i];
	}

	return NULL;
}

static const struct rtd_pin_sconfig_desc *rtd_pinctrl_find_sconfig(struct rtd_pinctrl *data, const char *name)
{
	int i;

	for (i = 0; i < data->info->num_configs; i++) {
		if (strcmp(data->info->sconfigs[i].name, name) == 0)
			return &data->info->sconfigs[i];
	}

	return NULL;
}



static int rtd_pinctrl_set_one_mux(struct pinctrl_dev *pcdev,
	unsigned int pin, const char *func_name)
{
	struct rtd_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);
	const struct pinctrl_pin_desc *pin_desc;
	const struct rtd_pin_desc *mux;
	const char *pin_name;
	u32 val;
	int i;

	pin_desc = rtd_pinctrl_get_pin_by_number(data, pin);
	if (!pin_desc)
		return -ENOTSUPP;

	pin_name = pin_desc->name;

	mux = rtd_pinctrl_find_mux(data, pin_name);
	if (!mux)
		return -ENOTSUPP;

	if (!mux->functions) {
		dev_err(pcdev->dev,
			"No functions available for pin %s\n", pin_name);
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

	dev_err(pcdev->dev, "No function %s available for pin %s\n",
			func_name, pin_name);
	return -EINVAL;
}

static int rtd_pinctrl_set_mux(struct pinctrl_dev *pcdev,
		unsigned int function, unsigned int group)
{
	struct rtd_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);
	const unsigned int *pins;
	unsigned int num_pins;
	const char *func_name;
	const char *group_name;
	int i, ret;

	func_name = data->info->functions[function].name;
	group_name = data->info->groups[group].name;

	ret = rtd_pinctrl_get_group_pins(pcdev, group, &pins, &num_pins);
	if (ret) {
		dev_err(pcdev->dev, "Getting pins for group %s failed\n",
			group_name);
		return ret;
	}

	for (i = 0; i < num_pins; i++) {
		ret = rtd_pinctrl_set_one_mux(pcdev, pins[i], func_name);
		if (ret)
			return ret;
	}

	return 0;
}

static int rtd_pinctrl_gpio_request_enable(struct pinctrl_dev *pcdev,
	struct pinctrl_gpio_range *range, unsigned int offset)
{
	return rtd_pinctrl_set_one_mux(pcdev, offset, "gpio");
}

static const struct pinmux_ops rtd_pinmux_ops = {
	.get_functions_count = rtd_pinctrl_get_functions_count,
	.get_function_name = rtd_pinctrl_get_function_name,
	.get_function_groups = rtd_pinctrl_get_function_groups,
	.set_mux = rtd_pinctrl_set_mux,
	.gpio_request_enable = rtd_pinctrl_gpio_request_enable,
};


static int rtd_pconf_parse_conf(struct rtd_pinctrl *data,
	const char *pin_name, enum pin_config_param param,
	enum pin_config_param arg)
{
	u8 set_val = 0;
	u16 strength;
	u32 val, mask;
	int pulsel_off, pulen_off, smt_off, curr_off;
	const struct rtd_pin_config_desc *config_desc;
	const struct rtd_pin_sconfig_desc *sconfig_desc;

	config_desc = rtd_pinctrl_find_config(data, pin_name);
	if (!config_desc)
		return -ENOTSUPP;

	smt_off = config_desc->base_bit + config_desc->smt_offset;
	curr_off = config_desc->base_bit + config_desc->curr_offset;
	pulsel_off = config_desc->base_bit + config_desc->pud_sel_offset;
	pulen_off = config_desc->base_bit + config_desc->pud_en_offset;

	switch ((u32)param) {
	case PIN_CONFIG_INPUT_SCHMITT:
	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
		set_val = arg;
		val = readl(data->base + config_desc->reg_offset);
		if (set_val)
			val |= BIT(smt_off);
		else
			val &= ~BIT(smt_off);
		writel(val, data->base + config_desc->reg_offset);
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		val = readl(data->base + config_desc->reg_offset);
		if (set_val)
			val |= BIT(pulen_off);
		else
			val &= ~BIT(pulen_off);
		writel(val, data->base + config_desc->reg_offset);
		break;
	case PIN_CONFIG_BIAS_DISABLE:
		val = readl(data->base + config_desc->reg_offset);
		val &= ~BIT(pulen_off);
		writel(val, data->base + config_desc->reg_offset);
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		val = readl(data->base + config_desc->reg_offset);
		val |= BIT(pulen_off) | BIT(pulsel_off);
		writel(val, data->base + config_desc->reg_offset);
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		val = readl(data->base + config_desc->reg_offset);
		val |= BIT(pulen_off);
		val &= ~BIT(pulsel_off);
		writel(val, data->base + config_desc->reg_offset);
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		strength = arg;
		val = readl(data->base + config_desc->reg_offset);
		switch (config_desc->curr_type) {
		case PADDRI_4_8:
			if (strength == 4)
				val &= ~BIT(curr_off);
			else if (strength == 8)
				val |= BIT(curr_off);
			else
				return -EINVAL;
			break;
		case PADDRI_2_4:
			if (strength == 2)
				val &= ~BIT(curr_off);
			else if (strength == 4)
				val |= BIT(curr_off);
			else
				return -EINVAL;
			break;
		case PCONF_UNSUPP:
			dev_err(data->pcdev->dev, "[%s] not support drive strength\n",
				config_desc->name);
			return -ENOTSUPP;
		default:
			return -EINVAL;
		}
		writel(val, data->base + config_desc->reg_offset);
		break;
	case RTK_P_DRIVE:
		sconfig_desc = rtd_pinctrl_find_sconfig(data, pin_name);
		if (!sconfig_desc)
			return -ENOTSUPP;
		set_val = arg;
		val = readl(data->base + sconfig_desc->reg_offset);
		mask = GENMASK(sconfig_desc->pdrive_offset +
				sconfig_desc->pdrive_maskbits - 1, sconfig_desc->pdrive_offset);
		val = (val & ~mask) | (set_val << sconfig_desc->pdrive_offset);
		writel(val, data->base + sconfig_desc->reg_offset);
		break;
	case RTK_N_DRIVE:
		sconfig_desc = rtd_pinctrl_find_sconfig(data, pin_name);
		if (!sconfig_desc)
			return -ENOTSUPP;
		set_val = arg;
		val = readl(data->base + sconfig_desc->reg_offset);
		mask = GENMASK(sconfig_desc->ndrive_offset +
				sconfig_desc->ndrive_maskbits - 1, sconfig_desc->ndrive_offset);
		val = (val & ~mask) | (set_val << sconfig_desc->ndrive_offset);
		writel(val, data->base + sconfig_desc->reg_offset);
		break;
	default:
		break;
	}

	return 0;
}

static int rtd_pin_config_get(struct pinctrl_dev *pcdev, unsigned int pinnr,
		unsigned long *config)
{
	unsigned int param = pinconf_to_config_param(*config);
	unsigned int arg = 0;

	switch (param) {
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);
	return 0;
}

static int rtd_pin_config_set(struct pinctrl_dev *pcdev, unsigned int pinnr,
		unsigned long *configs, unsigned int num_configs)
{
	struct rtd_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);
	const struct pinctrl_pin_desc *pin_desc;
	const char *pin_name;
	int i;
	int ret = 0;

	pin_desc = rtd_pinctrl_get_pin_by_number(data, pinnr);
	if (!pin_desc)
		return -ENOTSUPP;

	pin_name = pin_desc->name;

	for (i = 0; i < num_configs; i++) {
		ret = rtd_pconf_parse_conf(data, pin_name,
			pinconf_to_config_param(configs[i]),
			pinconf_to_config_argument(configs[i]));
		if (ret < 0)
			return ret;
	}

	return 0;
}


static int rtd_pin_config_group_set(struct pinctrl_dev *pcdev,
					unsigned int group, unsigned long *configs, unsigned int num_configs)
{
	struct rtd_pinctrl *data = pinctrl_dev_get_drvdata(pcdev);
	const unsigned int *pins;
	unsigned int num_pins;
	const char *group_name;
	int i, ret;

		group_name = data->info->groups[group].name;

	ret = rtd_pinctrl_get_group_pins(pcdev, group, &pins, &num_pins);
	if (ret) {
		dev_err(pcdev->dev, "Getting pins for group %s failed\n", group_name);
		return ret;
	}

	for (i = 0; i < num_pins; i++) {
		ret = rtd_pin_config_set(pcdev, pins[i], configs, num_configs);
		if (ret)
			return ret;
	}

	return 0;
}


static const struct pinconf_ops rtd_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = rtd_pin_config_get,
	.pin_config_set = rtd_pin_config_set,
	.pin_config_group_set = rtd_pin_config_group_set,
};

static void rtd_pinctrl_selftest(struct rtd_pinctrl *data)
{
	int i, j, k;
	const struct rtd_pinctrl_desc *info = data->info;

	for (i = 0; i < info->num_muxes; i++) {
		/* Check for pin */
		for (j = 0; j < info->num_pins; j++) {
			if (strcmp(info->pins[j].name,
				info->muxes[i].name) == 0)
				break;
		}
		if (j == info->num_pins)
			dev_warn(data->pcdev->dev, "Mux %s lacking matching pin\n",
				info->muxes[i].name);

		/* Check for group */
		for (j = 0; j < info->num_groups; j++) {
			if (strcmp(info->groups[j].name,
				info->muxes[i].name) == 0)
				break;
		}
		if (j == info->num_groups)
			dev_warn(data->pcdev->dev, "Mux %s lacking matching group\n",
				 info->muxes[i].name);

		for (j = 0; info->muxes[i].functions[j].name; j++) {
			/* Check for function */
			for (k = 0; k < info->num_functions; k++) {
				if (strcmp(info->functions[k].name,
					info->muxes[i].functions[j].name) == 0)
					break;
			}
			if (k == info->num_functions)
				dev_warn(data->pcdev->dev, "Mux %s lacking function %s\n",
					 info->muxes[i].name,
					 info->muxes[i].functions[j].name);

			/* Check for duplicate mux value
			 *- assumption: ascending order
			 */
			if (j > 0 && info->muxes[i].functions[j].mux_value
				< info->muxes[i].functions[j - 1].mux_value)
				dev_warn(data->pcdev->dev, "Mux %s function %s has unexpected value\n",
					 info->muxes[i].name,
					 info->muxes[i].functions[j].name);
		}
	}
}

static const struct of_device_id rtd_pinctrl_dt_ids[] = {
	 { .compatible = "realtek,rtd1195-iso-pinctrl",
		.data = &rtd1195_iso_pinctrl_desc },
	 { .compatible = "realtek,rtd1195-crt-pinctrl",
		.data = &rtd1195_crt_pinctrl_desc },
	 { .compatible = "realtek,rtd1295-iso-pinctrl",
		.data = &rtd1295_iso_pinctrl_desc },
	 { .compatible = "realtek,rtd1295-sb2-pinctrl",
		.data = &rtd1295_sb2_pinctrl_desc },
	 { .compatible = "realtek,rtd1295-disp-pinctrl",
		.data = &rtd1295_disp_pinctrl_desc },
	 { .compatible = "realtek,rtd1295-cr-pinctrl",
		.data = &rtd1295_cr_pinctrl_desc },
	 { }
};

static int rtd_pinctrl_probe(struct platform_device *pdev)
{
	struct rtd_pinctrl *data;
	const struct of_device_id *match;

	match = of_match_node(rtd_pinctrl_dt_ids, pdev->dev.of_node);
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
	data->desc.pctlops = &rtd_pinctrl_ops;
	data->desc.pmxops = &rtd_pinmux_ops;
	data->desc.confops = &rtd_pinconf_ops;
	data->desc.custom_params = rtd_custom_bindings;
	data->desc.num_custom_params = ARRAY_SIZE(rtd_custom_bindings);
	data->desc.owner = THIS_MODULE;

	data->pcdev = pinctrl_register(&data->desc, &pdev->dev, data);
	if (!data->pcdev)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);

	rtd_pinctrl_selftest(data);

	dev_info(&pdev->dev, "probed\n");

	return 0;
}

static int rtd_pinctrl_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct rtd_pinctrl *data = platform_get_drvdata(pdev);
	struct rtd_pin_reg_list *list;
	int i;

	for (i = 0; i < data->info->num_regs; i++) {
		list = &data->info->lists[i];
		list->val = readl(data->base + list->reg_offset);
	}

	return 0;
}

static int rtd_pinctrl_resume(struct platform_device *pdev)
{
	struct rtd_pinctrl *data = platform_get_drvdata(pdev);
	const struct rtd_pin_reg_list *list;
	int i;

	for (i = 0; i < data->info->num_regs; i++) {
		list = &data->info->lists[i];
		writel(list->val, data->base + list->reg_offset);
	}

	return 0;
}


static struct platform_driver rtd_pinctrl_driver = {
	.probe = rtd_pinctrl_probe,
	.suspend = rtd_pinctrl_suspend,
	.resume = rtd_pinctrl_resume,
	.driver = {
		.name = "rtd-pinctrl",
		.of_match_table	= rtd_pinctrl_dt_ids,
	},
};
builtin_platform_driver(rtd_pinctrl_driver);
MODULE_DESCRIPTION("rtd pinctrl driver");
MODULE_LICENSE("GPL");
