/*
 * XMC4x00 PORTS
 *
 * Copyright 2015-2016 Andreas Färber
 *
 * License: GPL-2.0+
 */

#include "pinctrl-xmc4000.h"
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/pinctrl/machine.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include "../pinctrl-utils.h"
#include "../core.h"

#define Pn_OMR		0x04
#define Pn_IOCR0	0x10
#define Pn_HWSEL	0x74

struct xmc4000_pinctrl_dev {
	struct pinctrl_dev *pcdev;
	void __iomem *base;
	const struct xmc4000_ports_data *data;
	struct pinctrl_desc desc;
};

static const char * const xmc4000_ports_functions[] = {
	"gpio", "alt1", "alt2", "alt3", "alt4"
};

static inline void __iomem *xmc4000_port_base(struct xmc4000_pinctrl_dev *pctrl, unsigned port)
{
	return pctrl->base + 0x100 * port;
}

static inline int xmc4000_gpio_get_port(unsigned offset)
{
	int port = offset / 16;
	return (port > 6) ? (port + 7) : port;
}

static void xmc4000_gpio_set(struct gpio_chip *gc, unsigned offset, int value)
{
	void __iomem *base = of_iomap(gc->of_node, 0) + Pn_OMR;
	int port = xmc4000_gpio_get_port(offset);
	int pin = offset % 16;

	if (value)
		writel_relaxed(1 << pin, base + port * 0x100);
	else
		writel_relaxed(1 << (16 + pin), base + port * 0x100);
}

static int xmc4000_gpio_direction_input(struct gpio_chip *gc, unsigned offset)
{
	return pinctrl_gpio_direction_input(gc->base + offset);
}

static int xmc4000_gpio_direction_output(struct gpio_chip *gc, unsigned offset, int value)
{
	xmc4000_gpio_set(gc, offset, value);
	return pinctrl_gpio_direction_output(gc->base + offset);
}

static int xmc4000_gpio_of_xlate(struct gpio_chip *gc, const struct of_phandle_args *gpiospec, u32 *flags)
{
	uint32_t port, pin;

	port = gpiospec->args[0];
	if ((port > 15) || (port > 6 && port < 14))
		return -EINVAL;

	pin = gpiospec->args[1];
	if (pin >= 16)
		return -EINVAL;

	if (flags)
		*flags = gpiospec->args[2];

	return ((port >= 14) ? (port - 7) : port) * 16 + pin;
}

static struct gpio_chip xmc4000_gpio_chip = {
	.label = "xmc4000",
	.owner = THIS_MODULE,
	.base = 0,
	.ngpio = (7 + 2) * 16,
	.set = xmc4000_gpio_set,
	.direction_input  = xmc4000_gpio_direction_input,
	.direction_output = xmc4000_gpio_direction_output,
	.of_gpio_n_cells = 3,
	.of_xlate = xmc4000_gpio_of_xlate,
};

static int xmc4000_gpiolib_register(struct platform_device *pdev)
{
	int ret;

	xmc4000_gpio_chip.parent = &pdev->dev;
	xmc4000_gpio_chip.of_node = pdev->dev.of_node;

	ret = gpiochip_add(&xmc4000_gpio_chip);
	if (ret) {
		dev_err(&pdev->dev, "could not add gpio chip\n");
		return ret;
	}

	ret = gpiochip_add_pin_range(&xmc4000_gpio_chip, dev_name(&pdev->dev), 0, 0, xmc4000_gpio_chip.ngpio);
	if (ret) {
		dev_err(&pdev->dev, "could not add gpio pin range\n");
		return ret;
	}

	return 0;
}

static int xmc4000_pinctrl_get_groups_count(struct pinctrl_dev *pcdev)
{
	struct xmc4000_pinctrl_dev *pctrl = pinctrl_dev_get_drvdata(pcdev);

	return pctrl->data->npins;
}

static const char *xmc4000_pinctrl_get_group_name(struct pinctrl_dev *pcdev,
		unsigned selector)
{
	struct xmc4000_pinctrl_dev *pctrl = pinctrl_dev_get_drvdata(pcdev);

	return pctrl->data->pin_names[selector];
}

static int xmc4000_pinctrl_get_group_pins(struct pinctrl_dev *pcdev,
		unsigned selector, const unsigned **pins, unsigned *num_pins)
{
	struct xmc4000_pinctrl_dev *pctrl = pinctrl_dev_get_drvdata(pcdev);

	*pins = pctrl->data->pins[selector].pins;

	return 0;
}

static const struct pinctrl_ops xmc4000_pinctrl_ops = {
	.dt_node_to_map = pinconf_generic_dt_node_to_map_all,
	.dt_free_map = pinctrl_utils_free_map,
	.get_groups_count = xmc4000_pinctrl_get_groups_count,
	.get_group_name = xmc4000_pinctrl_get_group_name,
	.get_group_pins = xmc4000_pinctrl_get_group_pins,
};

static int xmc4000_pinctrl_get_functions_count(struct pinctrl_dev *pcdev)
{
	return ARRAY_SIZE(xmc4000_ports_functions);
}

static const char *xmc4000_pinctrl_get_function_name(struct pinctrl_dev *pcdev,
		unsigned selector)
{
	return xmc4000_ports_functions[selector];
}

static int xmc4000_pinctrl_get_function_groups(struct pinctrl_dev *pcdev,
		unsigned selector, const char * const **groups,
		unsigned * const num_groups)
{
	struct xmc4000_pinctrl_dev *pctrl = pinctrl_dev_get_drvdata(pcdev);

	*groups = pctrl->data->pin_names;
	*num_groups = pctrl->data->npins;

	return 0;
}

static int xmc4000_pinctrl_set_mux(struct pinctrl_dev *pcdev,
		unsigned function, unsigned group)
{
	struct xmc4000_pinctrl_dev *pctrl = pinctrl_dev_get_drvdata(pcdev);
	int port = xmc4000_gpio_get_port(group);
	int pin = group % 16;
	void __iomem *base = xmc4000_port_base(pctrl, port);
	u8 pc;

	pc = readb_relaxed(base + Pn_IOCR0 + pin) >> 3;
	if (function == 0) {
		if (pc & BIT(4))
			pc &= 7;
	} else {
		/* default to push-pull */
		if (!(pc & BIT(4)))
			pc &= ~BIT(3);
		pc &= ~7;
		pc |= BIT(4) | (function & 7);
	}
	writeb_relaxed(pc << 3, base + Pn_IOCR0 + pin);

	return 0;
}

static int xmc4000_pinctrl_gpio_set_direction(struct pinctrl_dev *pcdev,
	struct pinctrl_gpio_range *range, unsigned offset, bool input)
{
	struct xmc4000_pinctrl_dev *pctrl = pinctrl_dev_get_drvdata(pcdev);
	int port = xmc4000_gpio_get_port(offset);
	int pin = offset % 16;
	void __iomem *base = xmc4000_port_base(pctrl, port);
	u8 pc;

	pc = readb_relaxed(base + Pn_IOCR0 + pin) >> 3;
	if (input)
		pc &= ~BIT(4);
	else {
		pc |= BIT(4);
		pc &= ~7;
	}
	writeb_relaxed(pc << 3, base + Pn_IOCR0 + pin);

	return 0;
}

static const struct pinmux_ops xmc4000_pinmux_ops = {
	.get_functions_count = xmc4000_pinctrl_get_functions_count,
	.get_function_name = xmc4000_pinctrl_get_function_name,
	.get_function_groups = xmc4000_pinctrl_get_function_groups,
	.set_mux = xmc4000_pinctrl_set_mux,
	.gpio_set_direction = xmc4000_pinctrl_gpio_set_direction,
};

enum xmc4000_hwsel {
	xmc4000_hwsel_software = 0,
	xmc4000_hwsel_hwi0_hwo0 = 1,
	xmc4000_hwsel_hwi1_hwo1 = 2,
};

enum xmc4000_pin_config_param {
	PIN_CONFIG_HWSEL = PIN_CONFIG_END + 1,
};

static const struct pinconf_generic_params xmc4000_custom_params[] = {
	{ "infineon,hwsel", PIN_CONFIG_HWSEL, xmc4000_hwsel_software },
};

static int xmc4000_pin_config_get(struct pinctrl_dev *pcdev, unsigned pinnr,
		unsigned long *config)
{
	struct xmc4000_pinctrl_dev *pctrl = pinctrl_dev_get_drvdata(pcdev);
	unsigned int param = pinconf_to_config_param(*config);
	unsigned int arg = 0;
	int port = xmc4000_gpio_get_port(pinnr);
	int pin = pinnr % 16;
	void __iomem *base = xmc4000_port_base(pctrl, port);
	u8 pc;

	pc = readb_relaxed(base + Pn_IOCR0 + pin) >> 3;

	switch (param) {
	case PIN_CONFIG_BIAS_PULL_UP:
		if (pc & BIT(4))
			return -EINVAL;
		arg = ((pc & 3) == 2) ? 1 : 0;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (pc & BIT(4))
			return -EINVAL;
		arg = ((pc & 3) == 1) ? 1 : 0;
		break;
	case PIN_CONFIG_HWSEL:
	{
		u32 hwsel = readl_relaxed(base + Pn_HWSEL);
		arg = (hwsel >> (pin * 2)) & 3;
		break;
	}
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);
	return 0;
}

static int xmc4000_pin_config_set(struct pinctrl_dev *pcdev, unsigned pinnr,
		unsigned long *configs, unsigned num_configs)
{
	struct xmc4000_pinctrl_dev *pctrl = pinctrl_dev_get_drvdata(pcdev);
	int port = xmc4000_gpio_get_port(pinnr);
	int pin = pinnr % 16;
	void __iomem *base = xmc4000_port_base(pctrl, port);
	u8 pc;
	u32 hwsel;
	int i;

	pc = readb_relaxed(base + Pn_IOCR0 + pin) >> 3;
	hwsel = readl_relaxed(base + Pn_HWSEL);

	for (i = 0; i < num_configs; i++) {
		unsigned int param = pinconf_to_config_param(configs[i]);
		unsigned int arg = pinconf_to_config_argument(configs[i]);

		switch (param) {
		case PIN_CONFIG_HWSEL:
			hwsel &= ~(3 << (pin * 2));
			hwsel |= (arg & 3) << (pin * 2);
			break;
		default:
			dev_warn(pcdev->dev, "unsupported configuration parameter %u\n", param);
			continue;
		}
	}

	writeb_relaxed(pc << 3, base + Pn_IOCR0 + pin);
	writel_relaxed(hwsel, base + Pn_HWSEL);

	return 0;
}

static const struct pinconf_ops xmc4000_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = xmc4000_pin_config_get,
	.pin_config_set = xmc4000_pin_config_set,
};

int xmc4000_ports_probe(struct platform_device *pdev)
{
	struct xmc4000_pinctrl_dev *pinctrl;
	const struct of_device_id *match;
	int ret;

	match = of_match_device(pdev->dev.driver->of_match_table, &pdev->dev);
	if (!match || !match->data)
		return -EINVAL;

	pinctrl = devm_kzalloc(&pdev->dev, sizeof(*pinctrl), GFP_KERNEL);
	if (!pinctrl)
		return -ENOMEM;

	pinctrl->base = of_iomap(pdev->dev.of_node, 0);
	pinctrl->data = match->data;
	pinctrl->desc.name = pinctrl->data->name;
	pinctrl->desc.pins = pinctrl->data->pin_desc;
	pinctrl->desc.npins = pinctrl->data->npins;
	pinctrl->desc.pctlops = &xmc4000_pinctrl_ops;
	pinctrl->desc.pmxops  = &xmc4000_pinmux_ops;
	pinctrl->desc.confops = &xmc4000_pinconf_ops;
	pinctrl->desc.custom_params = xmc4000_custom_params;
	pinctrl->desc.num_custom_params = ARRAY_SIZE(xmc4000_custom_params);
	pinctrl->desc.owner = THIS_MODULE;

	pinctrl->pcdev = pinctrl_register(&pinctrl->desc, &pdev->dev, pinctrl);
	if (!pinctrl->pcdev)
		return -ENOMEM;

	platform_set_drvdata(pdev, pinctrl);

	ret = xmc4000_gpiolib_register(pdev);
	if (ret) {
		pinctrl_unregister(pinctrl->pcdev);
		return ret;
	}

	dev_info(&pdev->dev, "XMC4000 pinctrl initialized\n");

	return 0;
}
