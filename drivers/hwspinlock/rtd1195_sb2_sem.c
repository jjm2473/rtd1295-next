// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * RTD1195 SB2 hardware semaphore
 *
 * Copyright (c) 2019 Andreas Färber
 */

#include <linux/bitops.h>
#include <linux/hwspinlock.h>
#include <linux/idr.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#include "hwspinlock_internal.h"

struct rtd1195_sb2_sem {
	struct platform_device *pdev;
	void __iomem *base;
	int base_id;
	struct hwspinlock_device lockdev;
};

static DEFINE_IDR(rtd1195_sb2_sem_idr);

static int rtd1195_sb2_sem_trylock(struct hwspinlock *lock)
{
	void __iomem *reg = (void __iomem *)lock->priv;

	return readl_relaxed(reg) & BIT(0);
}

static void rtd1195_sb2_sem_unlock(struct hwspinlock *lock)
{
	void __iomem *reg = (void __iomem *)lock->priv;

	writel_relaxed(0, reg);
}

static const struct hwspinlock_ops rtd1195_sb2_sem_hwspinlock_ops = {
	.trylock	= rtd1195_sb2_sem_trylock,
	.unlock		= rtd1195_sb2_sem_unlock,
};

static int rtd1195_sb2_sem_probe(struct platform_device *pdev)
{
	struct rtd1195_sb2_sem *sem;
	struct hwspinlock *lock;
	struct resource *res;
	int i, num;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENOMEM;

	num = resource_size(res) / 4;

	sem = devm_kzalloc(&pdev->dev, sizeof(*sem) + num * sizeof(*lock),
			   GFP_KERNEL);
	if (!sem)
		return -ENOMEM;

	sem->pdev = pdev;

	sem->base = of_iomap(pdev->dev.of_node, 0);
	if (!sem->base)
		return -ENOMEM;

	for (i = 0; i < num; i++) {
		lock = &sem->lockdev.lock[i];
		lock->priv = sem->base + i * 4;
	}

	platform_set_drvdata(pdev, sem);

	sem->base_id = idr_alloc(&rtd1195_sb2_sem_idr, sem, 0, 0, GFP_KERNEL);

	return devm_hwspin_lock_register(&pdev->dev, &sem->lockdev,
		&rtd1195_sb2_sem_hwspinlock_ops, sem->base_id, num);
}

static const struct of_device_id rtd1195_sb2_sem_dt_ids[] = {
	{ .compatible = "realtek,rtd1195-sb2-sem" },
	{ }
};
MODULE_DEVICE_TABLE(of, rtd1195_sb2_sem_dt_ids);

static struct platform_driver rtd1195_sb2_sem_platform_driver = {
	.driver = {
		.name = "rtd1195-sb2-sem",
		.of_match_table = rtd1195_sb2_sem_dt_ids,
	},
	.probe = rtd1195_sb2_sem_probe,
};
module_platform_driver(rtd1195_sb2_sem_platform_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Hardware spinlock driver for Realtek RTD1195 SB2");
MODULE_AUTHOR("Andreas Färber <afaerber@suse.de>");
