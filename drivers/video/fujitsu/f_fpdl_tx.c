/*
 * f_fpdl_tx FPD-Link TX driver
 *
 * based on f_mipidsi1_lp MIPI DSI PHY driver
 *
 * Copyright (C) 2013 Linaro, Ltd (Andy Green <andy.green@linaro.org>)
 * Copyright (C) 2013 Fujitsu Semiconductor, Ltd
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>

#include <video/fdb.h>
#include <video/f_fpdl_tx.h>

struct f_fpdl_tx {
	struct f_fdb_child fdb_child;
	struct device *dev;
	void __iomem *base;
	u32 padding_data_rsvd;
	u32 lvds_output_mode;

	u32 source_crtc_bitfield;

	struct f_fdb_child *bound; /* fdb fb we are bound to */
};

static int f_fpdl_tx_get_connector_type(struct f_fdb_child *fdb_child)
{
	return DRM_MODE_CONNECTOR_LVDS;
}

static bool f_fpdl_tx_detect(struct f_fdb_child *fdb_child)
{
	return true;
}

static u32 f_fpdl_tx_get_source_crtc_bitfield(struct f_fdb_child *fdb_child)
{
	struct f_fpdl_tx *priv = dev_get_drvdata(fdb_child->dev);

	return priv->source_crtc_bitfield;
}

static int f_fpdl_tx_bind_to_source(struct f_fdb_child *fdb_child,
						struct f_fdb_child *fdb_bound)
{
	struct f_fpdl_tx *priv = dev_get_drvdata(fdb_child->dev);

	priv->bound = fdb_bound;
	fdb_bound->reverse_binding = fdb_child;
	dev_info(fdb_child->dev, "binding to source %s\n",
					dev_name(fdb_bound->dev));
	return 0;
}

static int f_fpdl_tx_check_timings(struct f_fdb_child *fdb_child,
					struct fdb_video_timings *timings)
{
	return 0;
}

static void f_fpdl_tx_get_timings(struct f_fdb_child *fdb_child,
					struct fdb_video_timings *timings)
{
	struct f_fpdl_tx *priv = dev_get_drvdata(fdb_child->dev);

	pr_info("f_mipidsi_get_timings: source %p\n", priv->bound);

	if (!priv->bound)
		return;

	if (!priv->bound->ops->get_timings) {
		dev_err(priv->dev, "no get_timings in connector bind\n");
		return;
	}

	priv->bound->ops->get_timings(priv->bound, timings);
}

static int
f_fpdl_tx_enable(struct f_fdb_child *fdb_child)
{
	return 0;
}

static void
f_fpdl_tx_disable(struct f_fdb_child *fdb_child)
{
}

struct f_fdb_ops f_fpdl_fdb_ops = {
	.get_connector_type = f_fpdl_tx_get_connector_type,
	.check_timings = f_fpdl_tx_check_timings,
	.get_timings = f_fpdl_tx_get_timings,
	.get_source_crtc_bitfield = f_fpdl_tx_get_source_crtc_bitfield,
	.bind_to_source = f_fpdl_tx_bind_to_source,
	.detect = f_fpdl_tx_detect,
	.enable = f_fpdl_tx_enable,
	.disable = f_fpdl_tx_disable,

};

static int f_fpdl_tx_probe(struct platform_device *pdev)
{
	struct f_fpdl_tx *priv;
	struct resource *res;
	int ret = 0;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);
	priv->dev = &pdev->dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Missing resource\n");
		ret = -EINVAL;
		goto bail1;
	}

	priv->base = ioremap(res->start, res->end - res->start);
	if (!priv->base) {
		ret = -EINVAL;
		goto bail1;
	}

	if (of_property_read_u32(pdev->dev.of_node,
				"sources", &priv->source_crtc_bitfield)) {
		dev_err(&pdev->dev, "Missing sources bitfield\n");
		ret = -EINVAL;
		goto bail2;
	}

#ifdef CONFIG_PM_RUNTIME
	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to pm_runtime_get_sync: %d\n", ret);
	}
#endif

	/* register our fdb_child with f_fdb bus */
	ret = fdb_register(&pdev->dev, &priv->fdb_child, &f_fpdl_fdb_ops);
	if (ret < 0) {
		dev_err(&pdev->dev, "bus registration failed\n");
		goto bail2;
	}
#if 1
	/* set SUSP powerdown mode */
	fdb_setl(priv->base, SYSOC_FPDL_PWRDWNSET, SUSP, 1);

	/* set RSVD register */
	if (of_property_read_u32(pdev->dev.of_node,
				"rsvd", &priv->padding_data_rsvd)) {
		fdb_setl(priv->base, SYSOC_FPDL_PADSET, RSVD, 0);
	} else {
		if (priv->padding_data_rsvd == 0) {
			fdb_setl(priv->base, SYSOC_FPDL_PADSET, RSVD, 0);
		} else if (priv->padding_data_rsvd == 1) {
			fdb_setl(priv->base, SYSOC_FPDL_PADSET, RSVD, 1);
		} else {
			dev_err(&pdev->dev, "abnormal value was set in rsvd field\n");
			ret = -EINVAL;
			goto bail3;
		}
	}
#if 0
	/* set SEL_MAP register */
	if (of_property_read_u32(pdev->dev.of_node,
				"selmap", &priv->lvds_output_mode)) {
		fdb_setl(priv->base, SYSOC_FPDL_OUTSET, SELMAP, 0);
	} else {
		if (priv->lvds_output_mode == 0) {
			fdb_setl(priv->base, SYSOC_FPDL_OUTSET, SELMAP, 0);
		} else if (priv->lvds_output_mode == 1) {
			fdb_setl(priv->base, SYSOC_FPDL_OUTSET, SELMAP, 1);
		} else {
			dev_err(&pdev->dev, "abnormal value was set in selmap field\n");
			ret = -EINVAL;
			goto bail3;
		}
	}
#endif
	fdb_setl(priv->base, SYSOC_FPDL_OUTSET, SELMAP, 1);


	/* set SUSP normal mode */
	fdb_setl(priv->base, SYSOC_FPDL_PWRDWNSET, SUSP, 0);
#endif
	dev_info(&pdev->dev, "FPDL-TX driver\n");

	return 0;

bail3:
//	fdb_unregister(&pdev->dev, &priv->fdb_child);
bail2:
	iounmap(priv->base);
bail1:
	kfree(priv);

	return ret;
}

static int f_fpdl_tx_remove(struct platform_device *pdev)
{
	struct f_fpdl_tx *priv = platform_get_drvdata(pdev);

	fdb_unregister(&pdev->dev, &priv->fdb_child);
#if 1
	/* set powerdown mode */
	fdb_setl(priv->base, SYSOC_FPDL_PWRDWNSET, SUSP, 1);
#endif
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
#endif
	iounmap(priv->base);

	kfree(priv);

	return 0;
}

#ifdef CONFIG_PM_RUNTIME
static int f_fpdl_tx_runtime_suspend(struct device *dev)
{
	dev_info(dev, "FPDL-runtime_suspend\n");
	return 0;
}

static int f_fpdl_tx_runtime_resume(struct device *dev)
{
	dev_info(dev, "FPDL-runtime_resume\n");
	return 0;
}
#endif

#ifdef CONFIG_PM
static int f_fpdl_tx_suspend(struct device *dev)
{
#if 1
	struct f_fpdl_tx *priv = dev_get_drvdata(dev);

	/* set powerdown mode */
	fdb_setl(priv->base, SYSOC_FPDL_PWRDWNSET, SUSP, 1);
#endif
	return 0;
}

static int f_fpdl_tx_resume(struct device *dev)
{
#if 1
	struct f_fpdl_tx *priv = dev_get_drvdata(dev);

	fdb_setl(priv->base, SYSOC_FPDL_OUTSET, SELMAP, 1);

    /* set normal mode */
	fdb_setl(priv->base, SYSOC_FPDL_PWRDWNSET, SUSP, 0);
#endif
	return 0;
}

static const struct dev_pm_ops f_fpdl_tx_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(f_fpdl_tx_suspend, f_fpdl_tx_resume)
#ifdef CONFIG_PM_RUNTIME
	SET_RUNTIME_PM_OPS(f_fpdl_tx_runtime_suspend, f_fpdl_tx_runtime_resume,
			   NULL)
#endif
};
#endif /* CONFIG_PM */

static const struct of_device_id f_fpdl_tx_fb_dt_ids[] = {
	{ .compatible = "fujitsu,f_fpdl_tx" },
	{ /* sentinel */ }
};

static struct platform_driver f_fpdl_tx_driver = {
	.probe = f_fpdl_tx_probe,
	.remove = f_fpdl_tx_remove,
	.driver = {
		.name = "f_fpdl_tx",
		.of_match_table = f_fpdl_tx_fb_dt_ids,
#ifdef CONFIG_PM
		.pm = &f_fpdl_tx_pm_ops,
#endif
	},
};

MODULE_DEVICE_TABLE(of, f_fpdl_tx_fb_dt_ids);

static int __init f_fpdl_tx_init(void)
{
	return platform_driver_register(&f_fpdl_tx_driver);
}

static void __exit f_fpdl_tx_exit(void)
{
	platform_driver_unregister(&f_fpdl_tx_driver);
}

module_init(f_fpdl_tx_init);
module_exit(f_fpdl_tx_exit);

MODULE_LICENSE("GPL");
