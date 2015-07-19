/*
 * FM4
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

struct fm4_pinctrl_dev {
	struct pinctrl_dev *pcdev;
};

struct fm4_pinctrl_group {
	const char *name;
	const unsigned int *pins;
	unsigned npins;
};

struct fm4_pinmux_function {
	const char *name;
	const char * const *groups;
	unsigned int ngroups;
};

#define FM4_PINBANK(basenr, bank) \
	PINCTRL_PIN((basenr) + 0x0, "P" __stringify(bank) __stringify(0)), \
	PINCTRL_PIN((basenr) + 0x1, "P" __stringify(bank) __stringify(1)), \
	PINCTRL_PIN((basenr) + 0x2, "P" __stringify(bank) __stringify(2)), \
	PINCTRL_PIN((basenr) + 0x3, "P" __stringify(bank) __stringify(3)), \
	PINCTRL_PIN((basenr) + 0x4, "P" __stringify(bank) __stringify(4)), \
	PINCTRL_PIN((basenr) + 0x5, "P" __stringify(bank) __stringify(5)), \
	PINCTRL_PIN((basenr) + 0x6, "P" __stringify(bank) __stringify(6)), \
	PINCTRL_PIN((basenr) + 0x7, "P" __stringify(bank) __stringify(7)), \
	PINCTRL_PIN((basenr) + 0x8, "P" __stringify(bank) __stringify(8)), \
	PINCTRL_PIN((basenr) + 0x9, "P" __stringify(bank) __stringify(9)), \
	PINCTRL_PIN((basenr) + 0xa, "P" __stringify(bank) __stringify(A)), \
	PINCTRL_PIN((basenr) + 0xb, "P" __stringify(bank) __stringify(B)), \
	PINCTRL_PIN((basenr) + 0xc, "P" __stringify(bank) __stringify(C)), \
	PINCTRL_PIN((basenr) + 0xd, "P" __stringify(bank) __stringify(D)), \
	PINCTRL_PIN((basenr) + 0xe, "P" __stringify(bank) __stringify(E)), \
	PINCTRL_PIN((basenr) + 0xf, "P" __stringify(bank) __stringify(F))

static const struct pinctrl_pin_desc fm4_pins[] = {
	FM4_PINBANK(0x00, 0),
	FM4_PINBANK(0x10, 1),
	FM4_PINBANK(0x20, 2),
	FM4_PINBANK(0x30, 3),
	FM4_PINBANK(0x40, 4),
	FM4_PINBANK(0x50, 5),
	FM4_PINBANK(0x60, 6),
	FM4_PINBANK(0x70, 7),
	FM4_PINBANK(0x80, 8),
	FM4_PINBANK(0x90, 9),
	FM4_PINBANK(0xa0, A),
	FM4_PINBANK(0xb0, B),
	FM4_PINBANK(0xc0, C),
	FM4_PINBANK(0xd0, D),
	FM4_PINBANK(0xe0, E),
	FM4_PINBANK(0xf0, F),
};

static const unsigned int ethernet0_pins[] = {  };

#define FM4_PINCTRL_GRP(gname, epfr)				\
	{						\
		.name = __stringify(gname),		\
		.pins = gname ## _pins,			\
		.npins = ARRAY_SIZE(gname ## _pins),	\
	}

static const struct fm4_pinctrl_group fm4_pinctrl_groups[] = {
	FM4_PINCTRL_GRP(ethernet0, 14),
};

static const char * const ethernet0_groups[] = {
};

enum fm4_mux_functions {
	FM4_PINMUX_ethernet0,
	FM4_PINMUX_MAX_FUNCS
};

#define FM4_PINMUX_FUNCTION(fname)			\
	[FM4_PINMUX_ ## fname] = {			\
		.name = __stringify(fname),		\
		.groups = fname ## _groups,		\
		.ngroups = ARRAY_SIZE(fname ## _groups)	\
	}

static const struct fm4_pinmux_function fm4_pinmux_functions[] = {
	FM4_PINMUX_FUNCTION(ethernet0),
};

static int fm4_pinctrl_get_groups_count(struct pinctrl_dev *pcdev)
{
	return 0;
}

static const char *fm4_pinctrl_get_group_name(struct pinctrl_dev *pcdev,
		unsigned selector)
{
	return NULL;
}

static const struct pinctrl_ops fm4_pinctrl_ops = {
	.get_groups_count = fm4_pinctrl_get_groups_count,
	.get_group_name = fm4_pinctrl_get_group_name,
};

static int fm4_pinctrl_get_functions_count(struct pinctrl_dev *pcdev)
{
	return 0;
}

static const char *fm4_pinctrl_get_function_name(struct pinctrl_dev *pcdev,
		unsigned selector)
{
	return NULL;
}

static int fm4_pinctrl_get_function_groups(struct pinctrl_dev *pcdev,
		unsigned selector, const char * const **groups,
		unsigned * const num_groups)
{
	*groups = NULL;
	*num_groups = 0;

	return 0;
}

static int fm4_pinctrl_set_mux(struct pinctrl_dev *pcdev,
		unsigned function, unsigned group)
{
	return -ENOTSUPP;
}

static const struct pinmux_ops fm4_pinmux_ops = {
	.get_functions_count = fm4_pinctrl_get_functions_count,
	.get_function_name = fm4_pinctrl_get_function_name,
	.get_function_groups = fm4_pinctrl_get_function_groups,
	.set_mux = fm4_pinctrl_set_mux,
};

static int fm4_pin_config_get(struct pinctrl_dev *pcdev, unsigned pin,
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

static int fm4_pin_config_set(struct pinctrl_dev *pcdev, unsigned pin,
		unsigned long *configs, unsigned num_configs)
{
	return -ENOTSUPP;
}

static const struct pinconf_ops fm4_pinconf_ops = {
	.is_generic = 1,
	.pin_config_get = fm4_pin_config_get,
	.pin_config_set = fm4_pin_config_set,
};

static struct pinctrl_desc fm4_pinctrl_desc = {
	.name = "fm4-pinctrl",
	.pins = fm4_pins,
	.npins = ARRAY_SIZE(fm4_pins),
	.pctlops = &fm4_pinctrl_ops,
	.pmxops  = &fm4_pinmux_ops,
	.confops = &fm4_pinconf_ops,
	.owner = THIS_MODULE,
};

#define PIN_NAME_LEN (1 + 1 + 1 + 1)

static int fm4_pinctrl_probe(struct platform_device *pdev)
{
	struct fm4_pinctrl_dev *pinctrl;

	pinctrl = devm_kzalloc(&pdev->dev, sizeof(*pinctrl), GFP_KERNEL);
	if (!pinctrl)
		return -ENOMEM;

	pinctrl->pcdev = pinctrl_register(&fm4_pinctrl_desc, &pdev->dev, pinctrl);
	if (!pinctrl->pcdev)
		return -ENOMEM;

	platform_set_drvdata(pdev, pinctrl);

	dev_info(&pdev->dev, "FM4 pinctrl initialized\n");

	return 0;
}

static const struct of_device_id fm4_pinctrl_of_match[] = {
	{ .compatible = "cypress,fm4-gpio" },
	{ }
};

static struct platform_driver fm4_pinctrl_driver = {
	.probe = fm4_pinctrl_probe,
	.driver = {
		.name = "fm4-pinctrl",
		.owner = THIS_MODULE,
		.of_match_table = fm4_pinctrl_of_match,
	},
};

static int __init fm4_pinctrl_init(void)
{
	return platform_driver_register(&fm4_pinctrl_driver);
}
arch_initcall(fm4_pinctrl_init);
