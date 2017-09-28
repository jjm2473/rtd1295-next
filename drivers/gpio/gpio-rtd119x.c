#include <linux/bitops.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

struct rtd119x_gpio_info {
	unsigned int gpio_base;
	unsigned int num_gpios;
	unsigned int (*dir_offset)(unsigned int offset);
	unsigned int (*dato_offset)(unsigned int offset);
	unsigned int (*dati_offset)(unsigned int offset);
};

struct rtd119x_gpio {
	struct platform_device *pdev;
	const struct rtd119x_gpio_info *info;
	void __iomem *base;
	struct gpio_chip gpio_chip;
	spinlock_t lock;
};

#define to_rtd119x_gpio_chip(chip) container_of(chip, struct rtd119x_gpio, gpio_chip)

static int rtd119x_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	return pinctrl_gpio_request(chip->base + offset);
}

static void rtd119x_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	pinctrl_gpio_free(chip->base + offset);
}

static int rtd119x_gpio_get_direction(struct gpio_chip *chip, unsigned offset)
{
	struct rtd119x_gpio *data = to_rtd119x_gpio_chip(chip);
	unsigned long flags;
	unsigned int reg_offset;
	u32 val;

	reg_offset = data->info->dir_offset(offset);

	spin_lock_irqsave(&data->lock, flags);

	val = readl_relaxed(data->base + reg_offset);
	val &= BIT(offset % 32);

	spin_unlock_irqrestore(&data->lock, flags);

	return (val) ? GPIOF_DIR_OUT : GPIOF_DIR_IN;
}

static int rtd119x_gpio_set_direction(struct gpio_chip *chip, unsigned offset, bool out)
{
	struct rtd119x_gpio *data = to_rtd119x_gpio_chip(chip);
	unsigned long flags;
	unsigned int reg_offset;
	u32 mask = BIT(offset % 32);
	u32 val;

	reg_offset = data->info->dir_offset(offset);

	spin_lock_irqsave(&data->lock, flags);

	val = readl_relaxed(data->base + reg_offset);
	if (out)
		val |= mask;
	else
		val &= ~mask;
	writel_relaxed(val, data->base + reg_offset);

	spin_unlock_irqrestore(&data->lock, flags);

	return 0;
}

static int rtd119x_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	return rtd119x_gpio_set_direction(chip, offset, false);
}

static int rtd119x_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	chip->set(chip, offset, value);
	return rtd119x_gpio_set_direction(chip, offset, true);
}

static void rtd119x_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct rtd119x_gpio *data = to_rtd119x_gpio_chip(chip);
	unsigned long flags;
	unsigned int dato_reg_offset;
	u32 mask = BIT(offset % 32);
	u32 val;

	dato_reg_offset = data->info->dato_offset(offset);

	spin_lock_irqsave(&data->lock, flags);

	val = readl_relaxed(data->base + dato_reg_offset);
	if (value)
		val |= mask;
	else
		val &= ~mask;
	writel_relaxed(val, data->base + dato_reg_offset);

	spin_unlock_irqrestore(&data->lock, flags);
}

static int rtd119x_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct rtd119x_gpio *data = to_rtd119x_gpio_chip(chip);
	unsigned long flags;
	unsigned int dir_reg_offset, dat_reg_offset;
	u32 val;

	dir_reg_offset = data->info->dir_offset(offset);

	spin_lock_irqsave(&data->lock, flags);

	val = readl_relaxed(data->base + dir_reg_offset);
	val &= BIT(offset % 32);
	dat_reg_offset = (val) ? data->info->dato_offset(offset) : data->info->dati_offset(offset);

	val = readl_relaxed(data->base + dat_reg_offset);
	val >>= offset % 32;
	val &= 0x1;

	spin_unlock_irqrestore(&data->lock, flags);

	return val;
}

static unsigned int rtd1295_misc_gpio_dir_offset(unsigned int offset)
{
	return 0x00 + (offset / 32) * 4;
}

static unsigned int rtd1295_misc_gpio_dato_offset(unsigned int offset)
{
	return 0x10 + (offset / 32) * 4;
}

static unsigned int rtd1295_misc_gpio_dati_offset(unsigned int offset)
{
	return 0x20 + (offset / 32) * 4;
}

static unsigned int rtd1295_iso_gpio_dir_offset(unsigned int offset)
{
	return 0x00 + (offset / 32) * 0x18;
}

static unsigned int rtd1295_iso_gpio_dato_offset(unsigned int offset)
{
	return 0x04 + (offset / 32) * 0x18;
}

static unsigned int rtd1295_iso_gpio_dati_offset(unsigned int offset)
{
	return 0x08 + (offset / 32) * 0x18;
}

static const struct rtd119x_gpio_info rtd1295_misc_gpio_info = {
	.gpio_base = 0,
	.num_gpios = 101,
	.dir_offset  = rtd1295_misc_gpio_dir_offset,
	.dato_offset = rtd1295_misc_gpio_dato_offset,
	.dati_offset = rtd1295_misc_gpio_dati_offset,
};

static const struct rtd119x_gpio_info rtd1295_iso_gpio_info = {
	.gpio_base = 101,
	.num_gpios = 35,
	.dir_offset  = rtd1295_iso_gpio_dir_offset,
	.dato_offset = rtd1295_iso_gpio_dato_offset,
	.dati_offset = rtd1295_iso_gpio_dati_offset,
};

static const struct of_device_id rtd119x_gpio_of_matches[] = {
	{ .compatible = "realtek,rtd1295-misc-gpio", .data = &rtd1295_misc_gpio_info },
	{ .compatible = "realtek,rtd1295-iso-gpio", .data = &rtd1295_iso_gpio_info },
	{ }
};

static int rtd119x_gpio_probe(struct platform_device *pdev)
{
	struct rtd119x_gpio *data;
	const struct of_device_id *match;
	struct resource *res;
	int ret;

	match = of_match_node(rtd119x_gpio_of_matches, pdev->dev.of_node);
	if (!match)
		return -EINVAL;
	if (!match->data)
		return -EINVAL;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->info = match->data;
	spin_lock_init(&data->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->base))
		return PTR_ERR(data->base);

	data->gpio_chip.parent = &pdev->dev;
	data->gpio_chip.label = dev_name(&pdev->dev);
	data->gpio_chip.of_node = pdev->dev.of_node;
	data->gpio_chip.of_gpio_n_cells = 2;
	//data->gpio_chip.of_xlate = of_gpio_simple_xlate;
	data->gpio_chip.base = data->info->gpio_base;
	data->gpio_chip.ngpio = data->info->num_gpios;
	data->gpio_chip.request = rtd119x_gpio_request;
	data->gpio_chip.free = rtd119x_gpio_free;
	data->gpio_chip.get_direction = rtd119x_gpio_get_direction;
	data->gpio_chip.direction_input = rtd119x_gpio_direction_input;
	data->gpio_chip.direction_output = rtd119x_gpio_direction_output;
	data->gpio_chip.set = rtd119x_gpio_set;
	data->gpio_chip.get = rtd119x_gpio_get;

	ret = gpiochip_add(&data->gpio_chip);
	if (ret) {
		dev_err(&pdev->dev, "Adding GPIO chip failed (%d)\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, data);

	dev_info(&pdev->dev, "probed\n");

	return 0;
}

static struct platform_driver rtd119x_gpio_platform_driver = {
	.driver = {
		.name = "gpio-rtd119x",
		.of_match_table = rtd119x_gpio_of_matches,
	},
	.probe = rtd119x_gpio_probe,
};
builtin_platform_driver(rtd119x_gpio_platform_driver);
