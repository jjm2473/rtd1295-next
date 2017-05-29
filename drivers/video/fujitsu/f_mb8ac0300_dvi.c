/*
 * f-mb8ac0300-dvi stub dvi fdb head driver
 * Copyright (C) 2013 Linaro, Ltd for Fujitsu Semi
 * Author: Andy Green <andy.green@linaro.org>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>

#include <video/fdb.h>

struct f_mb8ac0300_dvi {
	struct f_fdb_child fdb_child;
	struct device *dev;
	void __iomem *base;
	int irq;
	u32 source_crtc_bitfield;

	struct f_fdb_child *bound;
};


static int mb8ac0300_dvi_get_connector_type(struct f_fdb_child *fdb_child)
{
	return DRM_MODE_CONNECTOR_DVID;
}

static int mb8ac0300_dvi_check_timings(struct f_fdb_child *fdb_child,
					struct fdb_video_timings *timings)
{
	struct f_mb8ac0300_dvi *priv = dev_get_drvdata(fdb_child->dev);

	if (!priv->bound)
		return 0;

	if (!priv->bound->ops->get_timings)
		return 0;

	return priv->bound->ops->check_timings(priv->bound, timings);
}

static void mb8ac0300_dvi_get_timings(struct f_fdb_child *fdb_child,
					struct fdb_video_timings *timings)
{
	struct f_mb8ac0300_dvi *priv = dev_get_drvdata(fdb_child->dev);

	pr_debug("mb8ac0300_dvi_get_timings: source %p\n", priv->bound);

	if (!priv->bound)
		return;

	if (!priv->bound->ops->get_timings) {
		dev_err(priv->dev, "no get_timings in connector bind\n");
		return;
	}

	priv->bound->ops->get_timings(priv->bound, timings);
}

static bool mb8ac0300_dvi_detect(struct f_fdb_child *fdb_child)
{
	return true;
}

static u32 mb8ac0300_dvi_get_source_crtc_bitfield(struct f_fdb_child *fdb_child)
{
	struct f_mb8ac0300_dvi *priv = dev_get_drvdata(fdb_child->dev);

	return priv->source_crtc_bitfield;
}

static int mb8ac0300_dvi_bind_to_source(struct f_fdb_child *fdb_child,
						struct f_fdb_child *fdb_bound)
{
	struct f_mb8ac0300_dvi *priv = dev_get_drvdata(fdb_child->dev);

	priv->bound = fdb_bound;
	dev_info(fdb_child->dev, "binding to source %s\n",
					dev_name(fdb_bound->dev));
	return 0;
}

struct f_fdb_ops mb8ac0300_dvi_fdb_ops = {
	.get_connector_type = mb8ac0300_dvi_get_connector_type,
	.check_timings = mb8ac0300_dvi_check_timings,
	.get_timings = mb8ac0300_dvi_get_timings,
	.detect = mb8ac0300_dvi_detect,
	.get_source_crtc_bitfield = mb8ac0300_dvi_get_source_crtc_bitfield,
	.bind_to_source = mb8ac0300_dvi_bind_to_source,
};
static int
f_mb8ac0300_dvi_probe(struct platform_device *pdev)
{
	struct f_mb8ac0300_dvi *priv;
	int ret = 0;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	/* register our fdb_child with f_fdb bus */
	ret = fdb_register(&pdev->dev, &priv->fdb_child, &mb8ac0300_dvi_fdb_ops);
	if (ret < 0) {
		dev_err(&pdev->dev, "framebuffer registration failed\n");
		goto bail;
	}

	if (of_property_read_u32(pdev->dev.of_node,
				"sources", &priv->source_crtc_bitfield)) {
		dev_err(&pdev->dev, "Missing sources bitfield\n");
		return -EINVAL;
	}

	dev_info(&pdev->dev, "fdb mb8ac0300 DVI head initialized\n");

	return 0;

bail:
	kfree(priv);

	return ret;
}

static int f_mb8ac0300_dvi_remove(struct platform_device *pdev)
{
	struct f_mb8ac0300_dvi *priv = platform_get_drvdata(pdev);

	fdb_unregister(&pdev->dev, &priv->fdb_child);
	kfree(priv);

	return 0;
}

#ifdef CONFIG_PM
static int f_mb8ac0300_dvi_suspend(struct platform_device *pdev, pm_message_t msg)
{
	/* suspend here */
	return 0;
}

static int f_mb8ac0300_dvi_resume(struct platform_device *pdev)
{
	/* resume here */
	return 0;
}
#else
#define f_mb8ac0300_dvi_suspend NULL
#define f_mb8ac0300_dvi_resume NULL
#endif /* CONFIG_PM */

static const struct of_device_id f_mb8ac0300_dvi_fb_dt_ids[] = {
	{ .compatible = "fujitsu,f_mb8ac0300_dvi" },
	{ /* sentinel */ }
};

static struct platform_driver f_mb8ac0300_dvi_driver = {
	.probe = f_mb8ac0300_dvi_probe,
	.remove = f_mb8ac0300_dvi_remove,
	.suspend = f_mb8ac0300_dvi_suspend,
	.resume = f_mb8ac0300_dvi_resume,
	.driver = {
		.name = "f_mb8ac0300_dvi",
		.of_match_table = f_mb8ac0300_dvi_fb_dt_ids,
	},
};

MODULE_DEVICE_TABLE(of, f_mb8ac0300_dvi_fb_dt_ids);

static int __init f_mb8ac0300_dvi_init(void)
{
	return platform_driver_register(&f_mb8ac0300_dvi_driver);
}

static void __exit f_mb8ac0300_dvi_exit(void)
{
	platform_driver_unregister(&f_mb8ac0300_dvi_driver);
}

module_init(f_mb8ac0300_dvi_init);
module_exit(f_mb8ac0300_dvi_exit);

MODULE_LICENSE("GPL");
