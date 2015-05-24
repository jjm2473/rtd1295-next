/*
 * XMC4500
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include "pinctrl-utils.h"
#include "core.h"

struct xmc4000_pinctrl_dev {
	struct pinctrl_dev *pcdev;
};

static int xmc4000_pinctrl_get_groups_count(struct pinctrl_dev *pcdev)
{
	return 0;
}

static const char *xmc4000_pinctrl_get_group_name(struct pinctrl_dev *pcdev,
		unsigned selector)
{
	return NULL;
}

static const struct pinctrl_ops xmc4000_pinctrl_ops = {
	.get_groups_count = xmc4000_pinctrl_get_groups_count,
	.get_group_name = xmc4000_pinctrl_get_group_name,
};

static int xmc4000_pinctrl_get_functions_count(struct pinctrl_dev *pcdev)
{
	return 0;
}

static const char *xmc4000_pinctrl_get_function_name(struct pinctrl_dev *pcdev,
		unsigned selector)
{
	return NULL;
}

static int xmc4000_pinctrl_get_function_groups(struct pinctrl_dev *pcdev,
		unsigned selector, const char * const **groups,
		unsigned * const num_groups)
{
	*groups = NULL;
	*num_groups = 0;

	return 0;
}

static int xmc4000_pinctrl_set_mux(struct pinctrl_dev *pcdev,
		unsigned function, unsigned group)
{
	return -ENOTSUPP;
}

static const struct pinmux_ops xmc4000_pinmux_ops = {
	.get_functions_count = xmc4000_pinctrl_get_functions_count,
	.get_function_name = xmc4000_pinctrl_get_function_name,
	.get_function_groups = xmc4000_pinctrl_get_function_groups,
	.set_mux = xmc4000_pinctrl_set_mux,
};

static int xmc4000_pin_config_get(struct pinctrl_dev *pcdev, unsigned pin,
		unsigned long *config)
{
	return -ENOTSUPP;
}

static int xmc4000_pin_config_set(struct pinctrl_dev *pcdev, unsigned pin,
		unsigned long *configs, unsigned num_configs)
{
	return -ENOTSUPP;
}

static const struct pinconf_ops xmc4000_pinconf_ops = {
	.is_generic = 1,
	.pin_config_get = xmc4000_pin_config_get,
	.pin_config_set = xmc4000_pin_config_set,
};

static struct pinctrl_desc xmc4000_pinctrl_desc = {
	.name = "xmc4000-pinctrl",
	.pctlops = &xmc4000_pinctrl_ops,
	.pmxops  = &xmc4000_pinmux_ops,
	.confops = &xmc4000_pinconf_ops,
	.owner = THIS_MODULE,
};

#define PIN_NAME_LEN (1 + 2 + 1 + 2 + 1)

static int xmc4000_pinctrl_probe(struct platform_device *pdev)
{
	struct xmc4000_pinctrl_dev *pinctrl;
	struct pinctrl_pin_desc *pins;
	char *pin_names;
	int i, j;

	pinctrl = devm_kzalloc(&pdev->dev, sizeof(*pinctrl), GFP_KERNEL);
	if (!pinctrl)
		return -ENOMEM;

	xmc4000_pinctrl_desc.npins = (7 + 2) * 16;
	pins = devm_kzalloc(&pdev->dev, sizeof(*xmc4000_pinctrl_desc.pins) * xmc4000_pinctrl_desc.npins, GFP_KERNEL);
	xmc4000_pinctrl_desc.pins = pins;
	if (!xmc4000_pinctrl_desc.pins)
		return -ENOMEM;

	pin_names = devm_kzalloc(&pdev->dev, sizeof(char) * PIN_NAME_LEN * xmc4000_pinctrl_desc.npins, GFP_KERNEL);
	if (!pin_names)
		return -ENOMEM;

	for (i = 0; i < 7 + 2; i++) {
		for (j = 0; j < 16; j++) {
			struct pinctrl_pin_desc *pin = &pins[i * 16 + j];
			char *pin_name = pin_names + (i * 16 + j) * PIN_NAME_LEN;
			int bank = i <= 6 ? i : i + 7;
			pin->number = i * 16 + j;
			snprintf(pin_name, PIN_NAME_LEN, "P%d.%d", bank, j);
			pin->name = pin_name;
		}
	}

	pinctrl->pcdev = pinctrl_register(&xmc4000_pinctrl_desc, &pdev->dev, pinctrl);
	if (!pinctrl->pcdev)
		return -ENOMEM;

	platform_set_drvdata(pdev, pinctrl);

	dev_info(&pdev->dev, "XMC4000 pinctrl initialized\n");

	return 0;
}

static const struct of_device_id xmc4000_pinctrl_of_match[] = {
	{ .compatible = "infineon,xmc4500-ports" },
	{ }
};

static struct platform_driver xmc4000_pinctrl_driver = {
	.probe = xmc4000_pinctrl_probe,
	.driver = {
		.name = "xmc4000-pinctrl",
		.owner = THIS_MODULE,
		.of_match_table = xmc4000_pinctrl_of_match,
	},
};

static int __init xmc4000_pinctrl_init(void)
{
	return platform_driver_register(&xmc4000_pinctrl_driver);
}
arch_initcall(xmc4000_pinctrl_init);
