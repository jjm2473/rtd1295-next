/*
 *  linux/drivers/gpio/gpio-mb86s7x.c
 *
 *  Copyright (C) 2015 Fujitsu Semiconductor Limited
 *  Copyright (C) 2015 Linaro Ltd.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/io.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/debugfs.h>

/*
 * Only first 8bits of a register correspond to each pin,
 * so there are 4 registers for 32 pins.
 */
#define PDR(x)	(0x0 + x / 8 * 4)
#define DDR(x)	(0x10 + x / 8 * 4)
#define PFR(x)	(0x20 + x / 8 * 4)

#define OFFSET(x)	BIT((x) % 8)
#define N_GPIO	32

struct mb86s70_gpio_chip {
	struct gpio_chip gc;
	void __iomem *base;
	struct clk *clk;
	spinlock_t lock;
	u32 *pdr_reg_save;
	u32 *ddr_reg_save;
	u32 *pfr_reg_save;
	int resume_dis_pin[N_GPIO + 1];
};

static inline struct mb86s70_gpio_chip *chip_to_mb86s70(struct gpio_chip *gc)
{
	return container_of(gc, struct mb86s70_gpio_chip, gc);
}

static int mb86s70_gpio_request(struct gpio_chip *gc, unsigned gpio)
{
	struct mb86s70_gpio_chip *gchip = chip_to_mb86s70(gc);
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&gchip->lock, flags);

	val = readl(gchip->base + PFR(gpio));
	val &= ~OFFSET(gpio);
	writel(val, gchip->base + PFR(gpio));

	spin_unlock_irqrestore(&gchip->lock, flags);

	return 0;
}

static void mb86s70_gpio_free(struct gpio_chip *gc, unsigned gpio)
{
	struct mb86s70_gpio_chip *gchip = chip_to_mb86s70(gc);
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&gchip->lock, flags);

	val = readl(gchip->base + PFR(gpio));
	val |= OFFSET(gpio);
	writel(val, gchip->base + PFR(gpio));

	spin_unlock_irqrestore(&gchip->lock, flags);
}

static int mb86s70_gpio_direction_input(struct gpio_chip *gc, unsigned gpio)
{
	struct mb86s70_gpio_chip *gchip = chip_to_mb86s70(gc);
	unsigned long flags;
	unsigned char val;

	spin_lock_irqsave(&gchip->lock, flags);

	val = readl(gchip->base + DDR(gpio));
	val &= ~OFFSET(gpio);
	writel(val, gchip->base + DDR(gpio));

	spin_unlock_irqrestore(&gchip->lock, flags);

	return 0;
}

static int mb86s70_gpio_direction_output(struct gpio_chip *gc,
					 unsigned gpio, int value)
{
	struct mb86s70_gpio_chip *gchip = chip_to_mb86s70(gc);
	unsigned long flags;
	unsigned char val;

	spin_lock_irqsave(&gchip->lock, flags);

	val = readl(gchip->base + PDR(gpio));
	if (value)
		val |= OFFSET(gpio);
	else
		val &= ~OFFSET(gpio);
	writel(val, gchip->base + PDR(gpio));

	val = readl(gchip->base + DDR(gpio));
	val |= OFFSET(gpio);
	writel(val, gchip->base + DDR(gpio));

	spin_unlock_irqrestore(&gchip->lock, flags);

	return 0;
}

static int mb86s70_gpio_get(struct gpio_chip *gc, unsigned gpio)
{
	struct mb86s70_gpio_chip *gchip = chip_to_mb86s70(gc);

	return !!(readl(gchip->base + PDR(gpio)) & OFFSET(gpio));
}

static void mb86s70_gpio_set(struct gpio_chip *gc, unsigned gpio, int value)
{
	struct mb86s70_gpio_chip *gchip = chip_to_mb86s70(gc);
	unsigned long flags;
	unsigned char val;

	spin_lock_irqsave(&gchip->lock, flags);

	val = readl(gchip->base + PDR(gpio));
	if (value)
		val |= OFFSET(gpio);
	else
		val &= ~OFFSET(gpio);
	writel(val, gchip->base + PDR(gpio));

	spin_unlock_irqrestore(&gchip->lock, flags);
}

#if defined(CONFIG_DEBUG_FS) && defined(CONFIG_DEBUG_GPIO)
int gpio_dbg_open(struct inode *inode, struct file *file)
{
	struct mb86s70_gpio_chip *gchip = inode->i_private;

	if (!gchip) {
		pr_err("%s gchip null\n", __func__);
		return -EINVAL;
	}
	file->private_data = gchip;
	return 0;
}

ssize_t gpio_dbg_read(struct file *file, char __user *user_buf,
	size_t count, loff_t *ppos)
{
	struct mb86s70_gpio_chip *gchip;
	struct device *dev;
	ssize_t ret = 0;
	u32 id = 0, cnt = 0, ngpio_gp;

	gchip = file->private_data;
	if (!gchip) {
		pr_err("%s gchip null\n", __func__);
		return -EINVAL;
	}

	dev = gchip->gc.dev;
	ngpio_gp = gchip->gc.ngpio / 8;

	while (cnt < ngpio_gp) {
		dev_info(dev,
			"PDR(0x%x):0x%x\n", cnt * 0x04, readb(gchip->base + PDR(id)));
		dev_info(dev,
			"DDR(0x%x):0x%x\n", cnt * 0x04, readb(gchip->base + DDR(id)));
		dev_info(dev,
			"PFR(0x%x):0x%x\n", cnt * 0x04, readb(gchip->base + PFR(id)));
		id = id + 8;
		cnt++;
	};

	return ret;
}

static const struct file_operations gpio_dbg_fops = {
	.open = gpio_dbg_open,
	.read = gpio_dbg_read,
};
#endif

static int mb86s70_gpio_probe(struct platform_device *pdev)
{
	struct mb86s70_gpio_chip *gchip;
	struct resource *res;
	int ret, base, r, index = 0;
#if defined(CONFIG_DEBUG_FS) && defined(CONFIG_DEBUG_GPIO)
	struct dentry *dbg_dir;
	struct dentry *dbg_file;
#endif

	gchip = devm_kzalloc(&pdev->dev, sizeof(*gchip), GFP_KERNEL);
	if (gchip == NULL)
		return -ENOMEM;

#ifdef CONFIG_PM_RUNTIME
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);
#endif

	platform_set_drvdata(pdev, gchip);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	gchip->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(gchip->base))
		return PTR_ERR(gchip->base);

	gchip->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(gchip->clk))
		return PTR_ERR(gchip->clk);

	clk_prepare_enable(gchip->clk);

	spin_lock_init(&gchip->lock);

	ret = of_property_read_u32_index(pdev->dev.of_node, "base", 0, &base);
	if (ret) {
		dev_info(&pdev->dev, "missing base in dt\n");
		base = -1;
	}

	gchip->gc.direction_output = mb86s70_gpio_direction_output;
	gchip->gc.direction_input = mb86s70_gpio_direction_input;
	gchip->gc.request = mb86s70_gpio_request;
	gchip->gc.free = mb86s70_gpio_free;
	gchip->gc.get = mb86s70_gpio_get;
	gchip->gc.set = mb86s70_gpio_set;
	gchip->gc.label = dev_name(&pdev->dev);
	gchip->gc.ngpio = N_GPIO;
	gchip->gc.owner = THIS_MODULE;
	gchip->gc.dev = &pdev->dev;
	gchip->gc.base = base;

	gchip->pdr_reg_save = devm_kzalloc(&pdev->dev,
		sizeof(u32) * gchip->gc.ngpio / 8, GFP_KERNEL);
	gchip->ddr_reg_save = devm_kzalloc(&pdev->dev,
		sizeof(u32) * gchip->gc.ngpio / 8, GFP_KERNEL);
	gchip->pfr_reg_save = devm_kzalloc(&pdev->dev,
		sizeof(u32) * gchip->gc.ngpio / 8, GFP_KERNEL);
	
	platform_set_drvdata(pdev, gchip);

	ret = gpiochip_add(&gchip->gc);
	if (ret) {
		dev_err(&pdev->dev, "couldn't register gpio driver\n");
		clk_disable_unprepare(gchip->clk);
	}

#if defined(CONFIG_DEBUG_FS) && defined(CONFIG_DEBUG_GPIO)
	dbg_dir = debugfs_create_dir(dev_name(&pdev->dev), NULL);
	if (!dbg_dir) {
		dev_err(&pdev->dev, "debugfs_create_dir fail\n");
		return -ENOMEM;
	}

	dbg_file = debugfs_create_file("mb86s7x_gpio_dbg",
		S_IWUGO, dbg_dir, gchip, &gpio_dbg_fops);
	if (!dbg_file) {
		dev_err(&pdev->dev, "debugfs_create_file fail\n");
		return -ENOMEM;
	}
#endif

	while(1) {
		r = of_property_read_u32_index(pdev->dev.of_node,
				"resume_function_enabled",
				index, &gchip->resume_dis_pin[index]);

		if (r || index > (N_GPIO - 1)) {
			gchip->resume_dis_pin[index] = -1;
			break;
		}
		index++;
	};

	return ret;
}

static int mb86s70_gpio_remove(struct platform_device *pdev)
{
	struct mb86s70_gpio_chip *gchip = platform_get_drvdata(pdev);
	int ret;

	ret = gpiochip_remove(&gchip->gc);
	if (ret)
		return ret;

	kfree(gchip->pdr_reg_save);
	kfree(gchip->ddr_reg_save);
	kfree(gchip->pfr_reg_save);

	clk_disable_unprepare(gchip->clk);
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
#endif
	return 0;
}

#ifdef CONFIG_PM
static int mb86s70_gpio_pm_suspend(struct device *dev)
{
	struct mb86s70_gpio_chip *gchip = dev_get_drvdata(dev);
	u32 id = 0, cnt = 0, ngpio_gp;

	if (!gchip)
		return 0;

	/* 8 pin of group */
	ngpio_gp = gchip->gc.ngpio / 8;

	while (cnt < ngpio_gp) {
		gchip->pdr_reg_save[cnt] = readb(gchip->base + PDR(id));
		gchip->ddr_reg_save[cnt] = readb(gchip->base + DDR(id));
		gchip->pfr_reg_save[cnt] = readb(gchip->base + PFR(id));
		id = id + 8;
		cnt++;
	};

	clk_disable_unprepare(gchip->clk);

	return 0;
}

static int mb86s70_gpio_pm_resume(struct device *dev)
{
	struct mb86s70_gpio_chip *gchip = dev_get_drvdata(dev);
	u32 id = 0, cnt = 0, ngpio_gp;
	u32 en_pin_mask[4] = {0}, rd_pfr, rd_pdr, rd_ddr;
	u32 pin_gp;

	if (!gchip)
		return 0;

	ngpio_gp = gchip->gc.ngpio / 8;

	clk_prepare_enable(gchip->clk);

	while (gchip->resume_dis_pin[cnt] >= 0) {
		if ( gchip->resume_dis_pin[cnt] > N_GPIO)
			gchip->resume_dis_pin[cnt] -= gchip->gc.base;

		pin_gp = gchip->resume_dis_pin[cnt] / 8;
		en_pin_mask[pin_gp] |= 1 << gchip->resume_dis_pin[cnt] % 8;
		cnt++;
	};

	cnt = 0;
	while (cnt < ngpio_gp) {
		/*
		 * read gpio current state,
		 * because SCB will update GPIO configure
		 */
		rd_pfr = readb(gchip->base + PFR(id));
		rd_pdr = readb(gchip->base + PDR(id));
		rd_ddr = readb(gchip->base + DDR(id));

		/* keep disable resume pin value */
		rd_pfr &= ~en_pin_mask[cnt];
		rd_pdr &= ~en_pin_mask[cnt];
		rd_ddr &= ~en_pin_mask[cnt];

		/* clear disable resume pin value from suspend reserved */
		gchip->pfr_reg_save[cnt] &= en_pin_mask[cnt];
		gchip->pdr_reg_save[cnt] &= en_pin_mask[cnt];
		gchip->ddr_reg_save[cnt] &= en_pin_mask[cnt];

		gchip->pfr_reg_save[cnt] |= rd_pfr;
		gchip->pdr_reg_save[cnt] |= rd_pdr;
		gchip->ddr_reg_save[cnt] |= rd_ddr;

		writeb(gchip->pfr_reg_save[cnt], gchip->base + PFR(id));
		writeb(gchip->pdr_reg_save[cnt], gchip->base + PDR(id));
		writeb(gchip->ddr_reg_save[cnt], gchip->base + DDR(id));

		id = id + 8;
		cnt++;
	};

	return 0;
}

static int mb86s70_gpio_runtime_suspend(struct device *dev)
{
	return mb86s70_gpio_pm_suspend(dev);
}

static int mb86s70_gpio_runtime_resume(struct device *dev)
{
	return mb86s70_gpio_pm_resume(dev);
}

static const struct dev_pm_ops mb86s70_gpio_pm_ops = {
	.resume = &mb86s70_gpio_pm_resume,
	.suspend = &mb86s70_gpio_pm_suspend,
#ifdef CONFIG_PM_RUNTIME
	SET_RUNTIME_PM_OPS(
	mb86s70_gpio_runtime_suspend,
	mb86s70_gpio_runtime_resume, NULL)
#endif
};
#endif

static const struct of_device_id mb86s70_gpio_dt_ids[] = {
	{ .compatible = "fujitsu,mb86s70-gpio" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mb86s70_gpio_dt_ids);

static struct platform_driver mb86s70_gpio_driver = {
	.driver = {
		.name = "mb86s70-gpio",
		.of_match_table = mb86s70_gpio_dt_ids,
#ifdef CONFIG_PM
		.pm = &mb86s70_gpio_pm_ops,
#endif
	},
	.probe = mb86s70_gpio_probe,
	.remove = mb86s70_gpio_remove,
};

static int __init mb86s70_gpio_init(void)
{
	return platform_driver_register(&mb86s70_gpio_driver);
}
subsys_initcall(mb86s70_gpio_init);

MODULE_DESCRIPTION("MB86S7x GPIO Driver");
MODULE_ALIAS("platform:mb86s70-gpio");
MODULE_LICENSE("GPL");
