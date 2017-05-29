/**
 * dwc3-mb86s70.c - Fujitsu mb86s70 DWC3 Specific Glue layer
 *
 * Copyright (c) 2013 FUJITSU SEMICONDUCTOR LIMITED
 *		http://jp.fujitsu.com/group/fsl
 *
 * based on dwc3-exynos.c
 *
 * Author: FUJITSU SEMICONDUCTOR
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/*#define DEBUG */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>

struct dwc3_mb86s70 {
	struct device		*dev;
	struct clk **clk_table;
	int irq;
	resource_size_t base_address;
};

/* return 0 means successful */
static int dwc3_mb86s70_clk_control(struct device *dev, bool on)
{
	int ret, i;
	struct clk *clk;

	dev_dbg(dev, "%s() is started (on:%d).\n", __func__, on);

	if (!on)
		goto clock_off;

	for (i = 0;; i++) {
		clk = of_clk_get(dev->of_node, i);
		if (IS_ERR(clk))
			break;

		ret = clk_prepare_enable(clk);
		if (ret) {
			dev_err(dev, "failed to enable clock[%d]\n", i);
			goto clock_off;
		}
		dev_info(dev, "enabled_clk_num[%d]\n", i+1);
	}
	dev_dbg(dev, "%s() is ended.\n", __func__);
	return 0;

clock_off:
	for (i = 0;; i++) {
		clk = of_clk_get(dev->of_node, i);
		if (IS_ERR(clk))
			break;

		clk_disable_unprepare(clk);
		dev_info(dev, "disabled_clk_num[%d]\n", i+1);
	}
	dev_dbg(dev, "%s() is ended.\n", __func__);
	return on;
}

static int dwc3_mb86s70_remove_child(struct device *dev, void *unused)
{
	struct platform_device *pdev = to_platform_device(dev);

	of_device_unregister(pdev);

	return 0;
}

static u64 dwc3_mb86s70_dma_mask = DMA_BIT_MASK(32);

static int dwc3_mb86s70_probe(struct platform_device *pdev)
{
	struct dwc3_mb86s70	*mb86s70;
	struct resource		*res;
	struct device		*dev = &pdev->dev;
	struct device_node	*node = dev->of_node;
	void            __iomem *kvaddr = NULL;

	int			ret;

	mb86s70 = devm_kzalloc(dev, sizeof(*mb86s70), GFP_KERNEL);
	if (!mb86s70) {
		dev_err(dev, "not enough memory\n");
		ret = -ENOMEM;
		goto err;
	}

	/*
	 * Right now device-tree probed devices don't get dma_mask set.
	 * Since shared usb code relies on it, set it here for now.
	 * Once we move to full device tree support this will vanish off.
	 */
	dev->dma_mask = &dwc3_mb86s70_dma_mask;

	platform_set_drvdata(pdev, mb86s70);

	mb86s70->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "missing memory base resource\n");
		ret = -EINVAL;
		goto irq_err;
	}
	kvaddr = ioremap_nocache((res->start), 4);
	if(!kvaddr) {
		dev_err(dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto irq_err;
	}
	iounmap((void __force __iomem *)kvaddr);
	mb86s70->base_address = res->start;

	/* resume driver for clock, power */
	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "get_sync failed with err %d\n", ret);
		goto irq_err;
	}

	if (node) {
		ret = of_platform_populate(node, NULL, NULL, dev);
		if (!ret)
			return 0;
		dev_err(dev, "failed to add dwc3 core\n");
	}
	dev_err(dev, "no device node, failed to add dwc3 core\n");
	ret = -ENODEV;
irq_err:
	kfree(mb86s70);
err:
	return ret;
}

static int dwc3_mb86s70_remove(struct platform_device *pdev)
{
	device_for_each_child(&pdev->dev, NULL, dwc3_mb86s70_remove_child);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}


static const struct of_device_id mb86s70_dwc3_match[] = {
	{ .compatible = "fujitsu,mb86s70-dwc3" },
	{},
};
MODULE_DEVICE_TABLE(of, mb86s70_dwc3_match);

#ifdef CONFIG_PM
#ifdef CONFIG_PM_RUNTIME
static int dwc3_mb86s70_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "%s() is started.\n", __func__);

	dwc3_mb86s70_clk_control(dev, false);

	dev_dbg(dev, "%s() is ended.\n", __func__);
	return 0;
}

static int  dwc3_mb86s70_runtime_resume(struct device *dev)
{
	struct dwc3_mb86s70	*mb86s70 = dev_get_drvdata(dev);
	void            __iomem *kvaddr = NULL;
	
	dev_dbg(dev, "%s() is started.\n", __func__);

	dwc3_mb86s70_clk_control(dev, true);

	kvaddr = ioremap_nocache((mb86s70->base_address+0xC12C), 4);
	if(!kvaddr) {
		dev_err(dev, "ioremap failed at %s\n", __func__);
	}
	else {
		*((unsigned long *)kvaddr) &= 0xFFFF7FFF;
		iounmap((void __force __iomem *)kvaddr);
	}

	dev_dbg(dev, "%s() is ended.\n", __func__);
	return 0;
}
#endif /* CONFIG_PM_RUNTIME */
static int dwc3_mb86s70_suspend(struct device *dev)
{
	if (pm_runtime_status_suspended(dev))
		return 0;

	return dwc3_mb86s70_runtime_suspend(dev);
}

static int dwc3_mb86s70_resume(struct device *dev)
{
	if (pm_runtime_status_suspended(dev))
		return 0;

	return dwc3_mb86s70_runtime_resume(dev);
}

static const struct dev_pm_ops dwc3_mb86s70_dev_pm_ops = {
	.suspend = dwc3_mb86s70_suspend,
	.resume = dwc3_mb86s70_resume,
	SET_RUNTIME_PM_OPS(
		dwc3_mb86s70_runtime_suspend,
		dwc3_mb86s70_runtime_resume, NULL)
};

#define DEV_PM_OPS	(&dwc3_mb86s70_dev_pm_ops)
#else
#define DEV_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */

static struct platform_driver dwc3_mb86s70_driver = {
	.probe		= dwc3_mb86s70_probe,
	.remove		= dwc3_mb86s70_remove,
	.driver		= {
		.name	= "mb86s70-dwc3",
		.of_match_table = of_match_ptr(mb86s70_dwc3_match),
		.pm	= DEV_PM_OPS,
	},
};

module_platform_driver(dwc3_mb86s70_driver);

MODULE_ALIAS("platform:mb86s70-dwc3");
MODULE_AUTHOR("FUJITSU SEMICONDUCTOR");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DesignWare USB3 mb86s70 Glue Layer");
