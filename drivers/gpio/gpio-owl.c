// SPDX-License-Identifier: GPL-2.0+
#include <linux/bitops.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define OWL_GPIO_xOUTEN		0x0
#define OWL_GPIO_xINEN		0x4
#define OWL_GPIO_xDAT		0x8

struct owl_gpio {
	struct platform_device *pdev;
	void __iomem *base;
	struct gpio_chip gpio_chip;
	spinlock_t lock;
};

#define to_owl_gpio_chip(chip) container_of(chip, struct owl_gpio, gpio_chip)

static int owl_gpio_request(struct gpio_chip *chip, unsigned int offset)
{
	return pinctrl_gpio_request(chip->base + offset);
}

static void owl_gpio_free(struct gpio_chip *chip, unsigned int offset)
{
	pinctrl_gpio_free(chip->base + offset);
}

static int owl_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	struct owl_gpio *data = to_owl_gpio_chip(chip);
	unsigned long flags;
	u32 outen, inen;

	spin_lock_irqsave(&data->lock, flags);

	outen = readl_relaxed(data->base + OWL_GPIO_xOUTEN);
	outen &= BIT(offset % 32);

	inen = readl_relaxed(data->base + OWL_GPIO_xINEN);
	inen &= BIT(offset % 32);

	spin_unlock_irqrestore(&data->lock, flags);

	if (outen)
		return GPIOF_DIR_OUT;
	if (inen)
		return GPIOF_DIR_IN;

	return -EINVAL;
}

static int owl_gpio_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	struct owl_gpio *data = to_owl_gpio_chip(chip);
	unsigned long flags;
	u32 mask = BIT(offset % 32);
	u32 val;

	spin_lock_irqsave(&data->lock, flags);

	val = readl_relaxed(data->base + OWL_GPIO_xINEN);
	val |= mask;
	writel_relaxed(val, data->base + OWL_GPIO_xINEN);

	val = readl_relaxed(data->base + OWL_GPIO_xOUTEN);
	val &= ~mask;
	writel_relaxed(val, data->base + OWL_GPIO_xOUTEN);

	spin_unlock_irqrestore(&data->lock, flags);

	return 0;
}

static int owl_gpio_direction_output(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct owl_gpio *data = to_owl_gpio_chip(chip);
	unsigned long flags;
	u32 mask = BIT(offset % 32);
	u32 val;

	spin_lock_irqsave(&data->lock, flags);

	val = readl_relaxed(data->base + OWL_GPIO_xOUTEN);
	val |= mask;
	writel_relaxed(val, data->base + OWL_GPIO_xOUTEN);

	val = readl_relaxed(data->base + OWL_GPIO_xINEN);
	val &= ~mask;
	writel_relaxed(val, data->base + OWL_GPIO_xINEN);

	spin_unlock_irqrestore(&data->lock, flags);

	chip->set(chip, offset, value);

	return 0;
}

static void owl_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct owl_gpio *data = to_owl_gpio_chip(chip);
	unsigned long flags;
	u32 mask = BIT(offset % 32);
	u32 val;

	spin_lock_irqsave(&data->lock, flags);

	val = readl_relaxed(data->base + OWL_GPIO_xDAT);
	if (value)
		val |= mask;
	else
		val &= ~mask;
	writel_relaxed(val, data->base + OWL_GPIO_xDAT);

	spin_unlock_irqrestore(&data->lock, flags);
}

static int owl_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct owl_gpio *data = to_owl_gpio_chip(chip);
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&data->lock, flags);

	val = readl_relaxed(data->base + OWL_GPIO_xDAT);
	val >>= offset % 32;
	val &= 0x1;

	spin_unlock_irqrestore(&data->lock, flags);

	return val;
}

static const struct of_device_id owl_gpio_of_matches[] = {
	{ .compatible = "actions,s500-gpio" },
	{ .compatible = "actions,s700-gpio" },
	{ }
};

static int owl_gpio_probe(struct platform_device *pdev)
{
	struct owl_gpio *data;
	struct resource *res;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	spin_lock_init(&data->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->base))
		return PTR_ERR(data->base);

	data->gpio_chip.parent = &pdev->dev;
	data->gpio_chip.label = dev_name(&pdev->dev);
	data->gpio_chip.of_node = pdev->dev.of_node;
	data->gpio_chip.of_gpio_n_cells = 2;
	data->gpio_chip.of_xlate = of_gpio_simple_xlate;
	data->gpio_chip.base = -1;
	data->gpio_chip.ngpio = 32;
	data->gpio_chip.request = owl_gpio_request;
	data->gpio_chip.free = owl_gpio_free;
	data->gpio_chip.get_direction = owl_gpio_get_direction;
	data->gpio_chip.direction_input = owl_gpio_direction_input;
	data->gpio_chip.direction_output = owl_gpio_direction_output;
	data->gpio_chip.set = owl_gpio_set;
	data->gpio_chip.get = owl_gpio_get;

	ret = gpiochip_add(&data->gpio_chip);
	if (ret) {
		dev_err(&pdev->dev, "Adding GPIO chip failed (%d)\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, data);

	dev_info(&pdev->dev, "probed\n");

	return 0;
}

static struct platform_driver owl_gpio_platform_driver = {
	.driver = {
		.name = "gpio-owl",
		.of_match_table = owl_gpio_of_matches,
	},
	.probe = owl_gpio_probe,
};
builtin_platform_driver(owl_gpio_platform_driver);
