/**
 * f_usb20ho_lap.c - Fujitsu EHCI platform driver
 *
 * Copyright (c) 2013 FUJITSU SEMICONDUCTOR LIMITED
 *		http://jp.fujitsu.com/group/fsl
 *
 * based on bcma-hcd.c
 *
 * Author: FUJITSU SEMICONDUCTOR
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
/* #define DEBUG */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/usb/ehci_pdriver.h>
#include <linux/usb/ohci_pdriver.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include "f_usb20ho_hcd.h"

/* to be confirmed this unknow value */
#define F_EHCI_OFFSET	0x40000
#define F_EHCI_SIZE		0x1000
#define F_OHCI_OFFSET	0x41000
#define F_OHCI_SIZE		0x1000
#define F_OTHER_OFFSET	0x42000
#define F_OTHER_SIZE		0x1000

MODULE_AUTHOR("FUJITSU SEMICONDUCTOR");
MODULE_DESCRIPTION("USB platform driver for f_usb20ho_lap IP ");
MODULE_LICENSE("GPL");

static const struct usb_ehci_pdata ehci_pdata = {
/* TO-DO: power management callback might be useful */
};

static const struct usb_ohci_pdata ohci_pdata = {
/* TO-DO: power management callback might be useful */
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

static struct platform_device *f_usb20ho_hcd_create_pdev(
		struct platform_device *pdev, bool ohci)
{
	struct resource *resource;
	struct platform_device *hci_dev;
	struct resource hci_res[2];
	int ret = -ENOMEM;
	int irq;
	resource_size_t resource_size;

	dev_dbg(&pdev->dev, "%s() is started.\n", __func__);

	memset(hci_res, 0, sizeof(hci_res));

	/* get a resource for a F_USB20HO device */
	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!resource) {
		dev_err(&pdev->dev, "%s():platform_get_resource() failed.\n"
			, __func__);
		ret = -ENODEV;
		goto err_res;
	}
	resource_size = resource->end - resource->start + 1;

	/* get an IRQ for a F_USB20HO device */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev,
			"%s():platform_get_irq() for F_USB20HO failed at %d.\n",
			__func__, irq);
		ret = -ENODEV;
		goto err_res;
	}

	hci_res[0].start = ohci ? resource->start + F_OHCI_OFFSET :
		resource->start + F_EHCI_OFFSET;
	hci_res[0].end = ohci ? resource->start + F_OHCI_OFFSET
			+ F_OHCI_SIZE - 1 :
		resource->start + F_EHCI_OFFSET + F_EHCI_SIZE - 1;
	hci_res[0].flags = IORESOURCE_MEM;

	hci_res[1].start = irq;
	hci_res[1].flags = IORESOURCE_IRQ;

	hci_dev = platform_device_alloc(ohci ? "ohci-platform" :
					"ehci-platform" , 0);
	if (!hci_dev) {
		dev_err(&pdev->dev, "platform_device_alloc() failed.\n");
		ret = -ENODEV;
		goto err_res;
	}

	dma_set_coherent_mask(&hci_dev->dev, pdev->dev.coherent_dma_mask);
	hci_dev->dev.parent = &pdev->dev;
	hci_dev->dev.dma_mask = pdev->dev.dma_mask;

	ret = platform_device_add_resources(hci_dev, hci_res,
					    ARRAY_SIZE(hci_res));
	if (ret) {
		dev_err(&pdev->dev
			, "platform_device_add_resources() failed.\n");
		goto err_alloc;
	}

	ret = platform_device_add_data(hci_dev,
			ohci ? (void *)&ohci_pdata : (void *)&ehci_pdata,
			ohci ? sizeof(ohci_pdata) : sizeof(ehci_pdata));
	if (ret) {
		dev_err(&pdev->dev, "platform_device_add_data() failed.\n");
		goto err_alloc;
	}

	ret = platform_device_add(hci_dev);
	if (ret) {
		dev_err(&pdev->dev, "platform_device_add() failed.\n");
		goto err_alloc;
	}

	dev_info(&pdev->dev, "%s() is ended.\n", __func__);
	return hci_dev;

err_alloc:
	platform_device_put(hci_dev);
err_res:
	dev_dbg(&pdev->dev, "%s() is ended with error.\n", __func__);
	dev_err(&pdev->dev, "%s(): fail and return ERR_PTR.\n", __func__);
	return ERR_PTR(ret);
}

static u64 f_usb20ho_dma_mask =  DMA_BIT_MASK(32);

static int f_usb20ho_hcd_probe(struct platform_device *pdev)
{
	int err;
	struct f_usb20ho_hcd *usb_dev;
	struct device *dev = &pdev->dev;

	dev_dbg(&pdev->dev, "%s() is started.\n", __func__);

	dev->dma_mask = &f_usb20ho_dma_mask;
	if (!dev->coherent_dma_mask)
		dev->coherent_dma_mask = DMA_BIT_MASK(32);
	dev_info(&pdev->dev, "%s(): coherent_dma_mask is 0x%llx .\n"
		, __func__, dev->coherent_dma_mask);

	usb_dev = kzalloc(sizeof(struct f_usb20ho_hcd), GFP_KERNEL);
	if (!usb_dev) {
		dev_err(&pdev->dev, "kzalloc() failed.\n");
		return -ENOMEM;
	}
	usb_dev->dev = &pdev->dev;
	platform_set_drvdata(pdev, usb_dev);

	/* get an IRQ for a F_USB20HO device */
	usb_dev->irq = platform_get_irq(pdev, 0);
	if (usb_dev->irq < 0) {
		dev_err(&pdev->dev,
			"%s():platform_get_irq() for F_USB20HO failed at %d.\n",
			__func__, usb_dev->irq);
		err = -ENODEV;
		goto err_free_usb_dev;
	}
	disable_irq(usb_dev->irq);

	/* resume driver for clock, power, irq */
	pm_runtime_enable(&pdev->dev);
	err = pm_runtime_get_sync(&pdev->dev);
	if (err < 0) {
		dev_err(&pdev->dev, "get_sync failed with err %d\n", err);
		goto err_unregister_ohci_dev;
	}

	usb_dev->ehci_dev = f_usb20ho_hcd_create_pdev(pdev, false);
	if (IS_ERR(usb_dev->ehci_dev)) {
		dev_err(&pdev->dev, "failed to create EHCI driver.\n");
		err = -ENODEV;
		goto err_free_usb_dev;
	}

	usb_dev->ohci_dev = f_usb20ho_hcd_create_pdev(pdev, true);
	if (IS_ERR(usb_dev->ohci_dev)) {
		dev_err(&pdev->dev, "failed to create OHCI driver.\n");
		err = -ENODEV;
		goto err_unregister_ehci_dev;
	}

	device_set_wakeup_capable(&pdev->dev, 1);

	dev_dbg(&pdev->dev, "%s() is ended.\n", __func__);
	return 0;

err_unregister_ohci_dev:
	platform_device_unregister(usb_dev->ohci_dev);
err_unregister_ehci_dev:
	platform_device_unregister(usb_dev->ehci_dev);
	pm_runtime_put_sync(&pdev->dev);
err_free_usb_dev:
	kfree(usb_dev);
	dev_err(&pdev->dev, "%s() is ended with error %d.\n"
		, __func__, err);
	return err;
}

static int f_usb20ho_hcd_remove(struct platform_device *pdev)
{
	struct f_usb20ho_hcd *usb_dev = platform_get_drvdata(pdev);
	struct platform_device *ohci_dev = usb_dev->ohci_dev;
	struct platform_device *ehci_dev = usb_dev->ehci_dev;

	dev_dbg(&pdev->dev, "%s() is started.\n", __func__);

	if (ohci_dev)
		platform_device_unregister(ohci_dev);
	if (ehci_dev)
		platform_device_unregister(ehci_dev);

	/* disable power,clock,irq */
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	dev_dbg(&pdev->dev, "%s() is ended.\n", __func__);
	return 0;
}

#ifdef CONFIG_PM
#ifdef CONFIG_PM_RUNTIME
static int  f_usb20ho_runtime_suspend(struct device *dev)
{
	struct f_usb20ho_hcd *usb_dev = dev_get_drvdata(dev);

	dev_dbg(dev, "%s() is started.\n", __func__);

	disable_irq(usb_dev->irq);
	dwc3_mb86s70_clk_control(dev, false);

	dev_dbg(dev, "%s() is ended.\n", __func__);
	return 0;
}

static int  f_usb20ho_runtime_resume(struct device *dev)
{
	struct f_usb20ho_hcd *usb_dev = dev_get_drvdata(dev);

	dev_dbg(dev, "%s() is started.\n", __func__);

	dwc3_mb86s70_clk_control(dev, true);
	enable_irq(usb_dev->irq);

	dev_dbg(dev, "%s() is ended.\n", __func__);
	return 0;
}
#endif /* CONFIG_PM_RUNTIME */

static int f_usb20ho_hcd_suspend(struct device *dev)
{
	dev_dbg(dev, "%s() is started.\n", __func__);

	if (pm_runtime_status_suspended(dev))
		return 0;

	dev_dbg(dev, "%s() is ended.\n", __func__);
	return f_usb20ho_runtime_suspend(dev);
}

static int f_usb20ho_hcd_resume(struct device *dev)
{
	dev_dbg(dev, "%s() is started.\n", __func__);

	if (pm_runtime_status_suspended(dev))
		return 0;

	dev_dbg(dev, "%s() is ended.\n", __func__);
	return f_usb20ho_runtime_resume(dev);
}

static const struct dev_pm_ops f_usb20ho_hcd_ops = {
	.suspend =  f_usb20ho_hcd_suspend,
	.resume = f_usb20ho_hcd_resume,
	SET_RUNTIME_PM_OPS(f_usb20ho_runtime_suspend
		, f_usb20ho_runtime_resume, NULL)
};

#define DEV_PM (&f_usb20ho_hcd_ops)
#else /* !CONFIG_PM */
#define DEV_PM	NULL
#endif /* CONFIG_PM */

static void f_usb20ho_hcd_shutdown(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "%s() is started.\n", __func__);
#ifdef CONFIG_PM
	 f_usb20ho_hcd_suspend(&pdev->dev);
#endif
	dev_dbg(&pdev->dev, "%s() is started.\n", __func__);
}

static const struct of_device_id f_usb20ho_hcd_dt_ids[] = {
	{ .compatible = "fujitsu,f_usb20ho_hcd" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, f_usb20ho_hcd_dt_ids);

static struct platform_driver f_usb20ho_hcd_driver = {
	.probe		= f_usb20ho_hcd_probe,
	.remove		= __exit_p(f_usb20ho_hcd_remove),
	.shutdown	= f_usb20ho_hcd_shutdown,
	.driver = {
		.name = "f_usb20ho_hcd",
		.owner = THIS_MODULE,
		.pm = DEV_PM,
		.of_match_table = f_usb20ho_hcd_dt_ids,
	},
};

static int f_usb20ho_hcd_init(void)
{
	return platform_driver_register(&f_usb20ho_hcd_driver);
}
module_init(f_usb20ho_hcd_init);

static void __exit f_usb20ho_hcd_exit(void)
{
	platform_driver_unregister(&f_usb20ho_hcd_driver);
}
module_exit(f_usb20ho_hcd_exit);
