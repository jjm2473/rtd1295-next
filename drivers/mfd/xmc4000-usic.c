/*
 * XMC4500 USIC
 */
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#define USICx_ID	0x8
#define USICx_ID_MOD_NUMBER	0x00AA
#define USICx_ID_MOD_TYPE	0xC0

static int xmc4000_usic_probe(struct platform_device *pdev)
{
	struct reset_control *reset;
	void __iomem *base;
	u32 usic_id;

	reset = of_reset_control_get(pdev->dev.of_node, NULL);
	if (IS_ERR(reset))
		return PTR_ERR(reset);

	/* Don't reset an active serial console */
	if (reset_control_status(reset))
		reset_control_deassert(reset);
	reset_control_put(reset);

	base = of_iomap(pdev->dev.of_node, 0);

	usic_id = readl_relaxed(base + USICx_ID);
	if ((usic_id >> 16) != USICx_ID_MOD_NUMBER) {
		dev_warn(&pdev->dev, "found module number %04X, expected %04X\n",
			 usic_id >> 16, USICx_ID_MOD_NUMBER);
		return -EINVAL;
	}
	if (((usic_id >> 8) & 0xff) != USICx_ID_MOD_TYPE) {
		dev_warn(&pdev->dev, "found module type %02X, expected %02X\n",
			 (usic_id >> 8) & 0xff, USICx_ID_MOD_TYPE);
		return -EINVAL;
	}
	dev_info(&pdev->dev, "module revision %02X\n", usic_id & 0xff);

	return of_platform_populate(pdev->dev.of_node, NULL, NULL, &pdev->dev);
}

static int xmc4000_usic_remove(struct platform_device *pdev)
{
	of_platform_depopulate(&pdev->dev);

	return 0;
}

static const struct of_device_id xmc4000_usic_of_matches[] = {
	{ .compatible = "infineon,xmc4500-usic" },
	{ }
};
MODULE_DEVICE_TABLE(of, xmc4000_usic_of_matches);

static struct platform_driver xmc4000_usic_driver = {
	.probe = xmc4000_usic_probe,
	.remove = xmc4000_usic_remove,

	.driver = {
		.name = "xmc4000-usic",
		.of_match_table = xmc4000_usic_of_matches,
	},
};
module_platform_driver(xmc4000_usic_driver);
