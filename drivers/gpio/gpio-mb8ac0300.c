/*
 *  linux/drivers/gpio/mb8ac0300_gpio.c
 *
 *  Copyright (C) 2011 FUJITSU SEMICONDUCTOR LIMITED
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/platform_data/mb8ac0300-gpio.h>

#define DRIVER_NAME		"gpio"
#define DRIVER_DESC		"MB8AC0300 GPIO Driver"

static int mb8ac0300_gpio_is_valid(struct gpio_chip *gc, unsigned offset)
{
#if 0
	struct mb8ac0300_gpio_chip *chip =
			container_of(gc, struct mb8ac0300_gpio_chip, gc);
	unsigned char func;
	unsigned char dir;
#endif
	/* check gpio pin offset */
	if (offset >= gc->ngpio) {
		dev_dbg(gc->dev, "offset is more than gpio pin numbers.\n");
		return 0;
	}
#if 0
	/* get gpio pin functions value */
	func = chip->functions[offset / MB8AC0300_GPIO_NR_PER_REG];
	func = (func >> (offset % MB8AC0300_GPIO_NR_PER_REG)) &
						MB8AC0300_GPIO_IS_PERIPHERAL;

	/* the gpio pin is used as peripheral port */
	if (func == MB8AC0300_GPIO_IS_PERIPHERAL) {
		/* get gpio pin directions value */
		dir = chip->directions[offset/MB8AC0300_GPIO_NR_PER_REG];
		dir = (dir >> (offset % MB8AC0300_GPIO_NR_PER_REG)) &
						MB8AC0300_GPIO_IS_OUTPUT;

		/* it is invalid if not used as external interrupt functions */
		if ((dir == MB8AC0300_GPIO_IS_OUTPUT) ||
			((offset < chip->pdata->irq_gpio_min) ||
			(offset > chip->pdata->irq_gpio_max))) {
			dev_dbg(gc->dev, "the gpio pin with no EXTINT\n");
			return 0;
		}
	}
#endif
	return 1;
}

/**
 * mb8ac0300_gpio_request - gpio pin request
 * @gc:		gpio chip
 * @offset:	offset of gpio pin number
 *
 * Returns 0 if no error, -EINVAL or other negative errno on failure
 */
static int mb8ac0300_gpio_request(struct gpio_chip *gc, unsigned offset)
{
	struct mb8ac0300_gpio_chip *chip =
			container_of(gc, struct mb8ac0300_gpio_chip, gc);
	unsigned long flags;
	unsigned char val;

	/* check whether gpio pin is valid */
	if (!mb8ac0300_gpio_is_valid(gc, offset)) {
		dev_dbg(gc->dev, "gpio pin is invalid.\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&chip->lock, flags);

	/* force it to not be in function mode then */
	val = readl(chip->base + MB8AC0300_GPIO_REG_PFR0 +
					MB8AC0300_GPIO_OFFSET_TO_REG(offset));
	val &= ~(1 << (offset % MB8AC0300_GPIO_NR_PER_REG));
	writel(val, chip->base + MB8AC0300_GPIO_REG_PFR0 +
					MB8AC0300_GPIO_OFFSET_TO_REG(offset));

	spin_unlock_irqrestore(&chip->lock, flags);

	return 0;
}

/**
 * mb8ac0300_gpio_direction_input - set gpio pin to input
 * @gc:		gpio chip
 * @offset:	offset of gpio pin number
 *
 * Returns 0
 */
static int mb8ac0300_gpio_direction_input(struct gpio_chip *gc, unsigned offset)
{
	struct mb8ac0300_gpio_chip *chip =
			container_of(gc, struct mb8ac0300_gpio_chip, gc);
	unsigned long flags;
	unsigned char val;

	/* get lock */
	spin_lock_irqsave(&chip->lock, flags);

	/* set gpio pin direction */
	val = readl(chip->base + MB8AC0300_GPIO_REG_DDR0 +
					MB8AC0300_GPIO_OFFSET_TO_REG(offset));
	val &= ~(1 << (offset % MB8AC0300_GPIO_NR_PER_REG));
	writel(val, chip->base + MB8AC0300_GPIO_REG_DDR0 +
					MB8AC0300_GPIO_OFFSET_TO_REG(offset));

	/* release lock */
	spin_unlock_irqrestore(&chip->lock, flags);

	return 0;
}

/**
 * mb8ac0300_gpio_direction_output - set gpio pin to output
 * @gc:		gpio chip
 * @offset:	offset of gpio pin number
 * @value:	output value of gpio pin
 *
 * Returns 0
 */
static int mb8ac0300_gpio_direction_output(struct gpio_chip *gc,
	 unsigned offset, int value)
{
	struct mb8ac0300_gpio_chip *chip =
			container_of(gc, struct mb8ac0300_gpio_chip, gc);
	unsigned long flags;
	unsigned char val;

	/* get lock */
	spin_lock_irqsave(&chip->lock, flags);

	/* set gpio pin value */
	val = readl(chip->base + MB8AC0300_GPIO_REG_PDR0 +
					MB8AC0300_GPIO_OFFSET_TO_REG(offset));
	if (value)
		val |= (1 << (offset % MB8AC0300_GPIO_NR_PER_REG));
	else
		val &= ~(1 << (offset % MB8AC0300_GPIO_NR_PER_REG));
	writel(val, chip->base + MB8AC0300_GPIO_REG_PDR0 +
					MB8AC0300_GPIO_OFFSET_TO_REG(offset));

	/* set gpio pin direction */
	val = readl(chip->base + MB8AC0300_GPIO_REG_DDR0 +
					MB8AC0300_GPIO_OFFSET_TO_REG(offset));
	val |= (1 << (offset % MB8AC0300_GPIO_NR_PER_REG));
	writel(val, chip->base + MB8AC0300_GPIO_REG_DDR0 +
					MB8AC0300_GPIO_OFFSET_TO_REG(offset));

	/* release lock */
	spin_unlock_irqrestore(&chip->lock, flags);

	return 0;
}

/**
 * mb8ac0300_gpio_get - get gpio pin value
 * @gc:		gpio chip
 * @offset:	offset of gpio pin number
 *
 * Returns 0 if low level,Returns 1 if high level
 */
static int mb8ac0300_gpio_get(struct gpio_chip *gc, unsigned offset)
{
	struct mb8ac0300_gpio_chip *chip =
			container_of(gc, struct mb8ac0300_gpio_chip, gc);
	unsigned char val;

	/* get gpio pin value */
	val = readl(chip->base + MB8AC0300_GPIO_REG_PDR0 +
					MB8AC0300_GPIO_OFFSET_TO_REG(offset));
	val &= (1 << (offset % MB8AC0300_GPIO_NR_PER_REG));

	return val ? 1 : 0;
}

/**
 * mb8ac0300_gpio_set - set gpio pin value
 * @gc:		gpio chip
 * @offset:	offset of gpio pin number
 * @value:	setup value
 *
 * no return value
 */
static void mb8ac0300_gpio_set(struct gpio_chip *gc, unsigned offset, int value)
{
	struct mb8ac0300_gpio_chip *chip =
			container_of(gc, struct mb8ac0300_gpio_chip, gc);
	unsigned long flags;
	unsigned char val;

	/* get lock */
	spin_lock_irqsave(&chip->lock, flags);

	/* set gpio pin value */
	val = readl(chip->base + MB8AC0300_GPIO_REG_PDR0 +
					MB8AC0300_GPIO_OFFSET_TO_REG(offset));
	if (value)
		val |= (1 << (offset % MB8AC0300_GPIO_NR_PER_REG));
	else
		val &= ~(1 << (offset % MB8AC0300_GPIO_NR_PER_REG));
	writel(val, chip->base + MB8AC0300_GPIO_REG_PDR0 +
					MB8AC0300_GPIO_OFFSET_TO_REG(offset));

	/* release lock */
	spin_unlock_irqrestore(&chip->lock, flags);

	return;
}

/**
 * mb8ac0300_gpio_to_irq - Convert the gpio pin number to irq number
 * @gc:		gpio chip
 * @offset:	offset of gpio pin number
 *
 * Returns the IRQ corresponding to a GPIO, -EINVAL or other negative errno on
 * failure
 */
static int mb8ac0300_gpio_to_irq(struct gpio_chip *gc, unsigned offset)
{
	struct mb8ac0300_gpio_chip *chip =
			container_of(gc, struct mb8ac0300_gpio_chip, gc);
	int num = offset + gc->base;

	/* check external interrupt gpio pin range */
	if (num < chip->pdata->irq_gpio_min ||
					num > chip->pdata->irq_gpio_max) {
		dev_dbg(gc->dev, "offset is over external interrupt range");
		return -EINVAL;
	}

	return chip->pdata->irq_base + (num - chip->pdata->irq_gpio_min);
}

/**
 * mb8ac0300_gpio_probe - Probes the gpio device
 * @pdev:	platform device
 *
 * Perform basic init : allocates memory, initialize
 *
 * Returns 0 if no error, -ENODEV -ENOMEM -EIO or other negative errno on
 * failure
 */
static int mb8ac0300_gpio_probe(struct platform_device *pdev)
{
	struct mb8ac0300_gpio_platform_data *pdata;
	struct mb8ac0300_gpio_chip *chip;
	struct resource *res;
	int counter;
	int ret = 0;
#ifdef CONFIG_OF
	const int *p;
	int len;
	int n;

	/* reserve memory space for gpio chip */
	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (chip == NULL) {
		dev_err(&pdev->dev, "can't allocate gpio chip data.\n");
		ret = -ENOMEM;
		goto done;
	}

	if (pdev->dev.of_node) {
		pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
		if (pdata == NULL) {
			dev_err(&pdev->dev, "Out of memory\n");
			ret = -ENOMEM;
			goto done;
		}

		p = of_get_property(pdev->dev.of_node, "gpio_base", NULL);
		if (p)
			pdata->gpio_base = be32_to_cpu(*p);
		p = of_get_property(pdev->dev.of_node, "irq_base", NULL);
		if (p)
			pdata->irq_base = be32_to_cpu(*p);
		p = of_get_property(pdev->dev.of_node, "irq_gpio_min", NULL);
		if (p)
			pdata->irq_gpio_min = be32_to_cpu(*p);
		p = of_get_property(pdev->dev.of_node, "irq_gpio_max", NULL);
		if (p)
			pdata->irq_gpio_max = be32_to_cpu(*p);
		p = of_get_property(pdev->dev.of_node, "directions", &len);
		if (p && len == (4 * sizeof(u32)))
			for (n = 0; n < 4; n++)
				chip->directions[n] = be32_to_cpu(*p++);
		p = of_get_property(pdev->dev.of_node, "values", &len);
		if (p && len == (4 * sizeof(u32)))
			for (n = 0; n < 4; n++)
				chip->values[n] = be32_to_cpu(*p++);
		p = of_get_property(pdev->dev.of_node, "functions", &len);
		if (p && len == (4 * sizeof(u32)))
			for (n = 0; n < 4; n++)
				chip->functions[n] = be32_to_cpu(*p++);

	} else
#endif
	{
		pdata = pdev->dev.platform_data;
		if (pdata == NULL) {
			dev_err(&pdev->dev, "No platform data supplied\n");
			ret = -ENOENT;
			goto done;
		}
	}

	/* gpio chip store in platform private data */
	platform_set_drvdata(pdev, chip);

	/* get IO resource */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	/* remap IO resource base address */
	chip->base = ioremap(res->start, res->end - res->start + 1);
	if (chip->base == NULL) {
		dev_err(&pdev->dev, "mapping IO resource is failed.\n");
		ret = -ENOMEM;
		goto free_mem;
	}

	/* get clock for gpio driver */
	chip->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(chip->clk)) {
		dev_err(&pdev->dev, "clock not found.\n");
		ret = PTR_ERR(chip->clk);
		goto iounmap;
	}

	/* enable clock */
	clk_prepare_enable(chip->clk);

	/* initiate lock */
	spin_lock_init(&chip->lock);

	/* initiate gpio chip structure */
	chip->gc.request = mb8ac0300_gpio_request;
	chip->gc.direction_input = mb8ac0300_gpio_direction_input;
	chip->gc.direction_output = mb8ac0300_gpio_direction_output;
	chip->gc.get = mb8ac0300_gpio_get;
	chip->gc.set = mb8ac0300_gpio_set;
	chip->gc.to_irq = mb8ac0300_gpio_to_irq;
	chip->gc.base = pdata->gpio_base;
	chip->gc.ngpio = MB8AC0300_GPIO_NR;
	chip->gc.label = dev_name(&pdev->dev);
	chip->gc.dev = &pdev->dev;
	chip->gc.owner = THIS_MODULE;
	chip->pdata = pdata;

	/* add gpio chip to gpiolib */
	ret = gpiochip_add(&chip->gc);
	if (ret) {
		dev_err(&pdev->dev, "gpiochip_add error %d\n", ret);
		goto disable_clk;
	}

	/* initiate gpio controller register */
	for (counter = 0; counter < MB8AC0300_GPIO_NR/MB8AC0300_GPIO_NR_PER_REG;
		 counter++) {
		/* initiate gpio function register */
		writel(chip->functions[counter],
			chip->base + MB8AC0300_GPIO_REG_PFR0 + counter * 4);

		/* initiate gpio value register */
		writel(chip->values[counter],
			chip->base + MB8AC0300_GPIO_REG_PDR0 + counter * 4);

		/* initiate gpio direction register */
		writel(chip->directions[counter],
			chip->base + MB8AC0300_GPIO_REG_DDR0 + counter * 4);
	}

	goto done;

disable_clk:
	/* disable clock */
	clk_disable_unprepare(chip->clk);
	clk_put(chip->clk);
iounmap:
	/* unmap gpio chip base address */
	iounmap(chip->base);
free_mem:
	/* free gpio chip memory space */
	kfree(chip);
done:
	if (ret && pdev->dev.of_node)
		kfree(pdata);
	return ret;
}

/**
 * mb8ac0300_gpio_remove - Removes the gpio device driver
 * @pdev:	platform device
 *
 * Returns 0
 */
static int mb8ac0300_gpio_remove(struct platform_device *pdev)
{
	struct mb8ac0300_gpio_chip *chip;

	/* get gpio chip from platform private data */
	chip = (struct mb8ac0300_gpio_chip *)platform_get_drvdata(pdev);

	if (chip) {
		/* unmap gpio chip base address */
		iounmap(chip->base);

		/* disable clock */
		clk_disable_unprepare(chip->clk);
		clk_put(chip->clk);

		if (pdev->dev.of_node)
			kfree(chip->pdata);

		/* free gpio chip memory space */
		kfree(chip);
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mb8ac0300_gpio_dt_ids[] = {
	{ .compatible = "fujitsu,mb8ac0300-gpio" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, mb8ac0300_gpio_dt_ids);
#else
#define mb8ac0300_gpio_dt_ids NULL
#endif

#ifdef CONFIG_PM
static inline void save_gpio_setting(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mb8ac0300_gpio_chip *chip = platform_get_drvdata(pdev);
	int n;

	for (n = 0; n < MB8AC0300_GPIO_NR/MB8AC0300_GPIO_NR_PER_REG; n++) {
		chip->directions[n] = (unsigned char)(readl(
			chip->base + MB8AC0300_GPIO_REG_DDR0 + n * 4));

		chip->values[n] = (unsigned char)(readb(
			chip->base + MB8AC0300_GPIO_REG_PDR0 + n * 4));
	}
}

static inline void restore_gpio_setting(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mb8ac0300_gpio_chip *chip = platform_get_drvdata(pdev);
	int n;

	for (n = 0; n < MB8AC0300_GPIO_NR/MB8AC0300_GPIO_NR_PER_REG; n++) {
		/* restore gpio direction register */
		writel((unsigned long)chip->directions[n],
			chip->base + MB8AC0300_GPIO_REG_DDR0 + n * 4);

		/* restore  gpio value register */
		writel(chip->values[n],
			chip->base + MB8AC0300_GPIO_REG_PDR0 + n * 4);
	}
}

static int mb8ac0300_gpio_suspend(struct device *dev)
{
	save_gpio_setting(dev);
	return 0;
}

static int mb8ac0300_gpio_resume(struct device *dev)
{
	restore_gpio_setting(dev);
	return 0;
}

static int mb8ac0300_gpio_freeze(struct device *dev)
{
	save_gpio_setting(dev);
	return 0;
}

static int mb8ac0300_gpio_thaw(struct device *dev)
{
	restore_gpio_setting(dev);
	return 0;
}

static int mb8ac0300_gpio_restore(struct device *dev)
{
	restore_gpio_setting(dev);
	return 0;
}

static int mb8ac0300_gpio_suspend_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mb8ac0300_gpio_chip *chip = platform_get_drvdata(pdev);

	clk_disable_unprepare(chip->clk);

	return 0;
}

static int mb8ac0300_gpio_resume_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mb8ac0300_gpio_chip *chip = platform_get_drvdata(pdev);
	int ret;

	ret = clk_prepare_enable(chip->clk);
	if (ret) {
		dev_err(dev, "%s: clk_enable failed\n", __func__);
		return ret;
	}

	return 0;
}


static const struct dev_pm_ops mb8ac0300_gpio_dev_pm_ops = {
	.suspend_noirq = mb8ac0300_gpio_suspend_noirq,
	.resume_noirq = mb8ac0300_gpio_resume_noirq,
	.suspend	= mb8ac0300_gpio_suspend,
	.resume		= mb8ac0300_gpio_resume,
	.freeze		= mb8ac0300_gpio_freeze,
	.thaw		= mb8ac0300_gpio_thaw,
	.restore	= mb8ac0300_gpio_restore,
};

#endif


/* GPIO device driver structure */
static struct platform_driver mb8ac0300_gpio_driver = {
	.probe     = mb8ac0300_gpio_probe,
	.remove    = mb8ac0300_gpio_remove,
	.driver    = {
		.name  = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mb8ac0300_gpio_dt_ids,
#ifdef CONFIG_PM
		.pm	= &mb8ac0300_gpio_dev_pm_ops,
#endif
	},
};

/**
 * mb8ac0300_gpio_init - initialize module
 *
 * Returns 0 if no error, negative errno on failure
 */
static int __init mb8ac0300_gpio_init(void)
{
	return platform_driver_register(&mb8ac0300_gpio_driver);
}
subsys_initcall(mb8ac0300_gpio_init);

/* GPIO device module definition */
MODULE_AUTHOR("Fujitsu Semiconductor Limited");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_LICENSE("GPL");
