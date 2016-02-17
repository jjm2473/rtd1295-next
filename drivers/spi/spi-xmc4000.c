/*
 * XMC4500 USIC SSC
 */
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/mfd/xmc4000-usic.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

static int xmc4000_usic_ssc_probe(struct platform_device *pdev)
{
	void __iomem *base;
	u32 mode, reg;
	int ret;

	ret = of_property_read_u32(pdev->dev.parent->of_node,
				   "infineon,usic-mode", &mode);
	if (ret)
		return ret;

	if (WARN_ON(mode != USICx_CHy_CCR_MODE_SSC))
		return -EINVAL;

	base = of_iomap(pdev->dev.parent->of_node, 0);

	reg = readl_relaxed(base + USICx_CHy_KSCFG);
	reg |= USICx_CHy_KSCFG_BPMODEN | USICx_CHy_KSCFG_MODEN;
	writel_relaxed(reg, base + USICx_CHy_KSCFG);

	reg = readl_relaxed(base + USICx_CHy_CCFG);
	if (!(reg & USICx_CHy_CCFG_SSC)) {
		dev_warn(&pdev->dev, "no support for SSC on this channel (%08X)\n", reg);
		return -EINVAL;
	}

	return 0;
}

static int xmc4000_usic_ssc_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id xmc4000_usic_ssc_of_matches[] = {
	{ .compatible = "infineon,xmc4500-usic-channel-ssc" },
	{ }
};
MODULE_DEVICE_TABLE(of, xmc4000_spi_of_matches);

static struct platform_driver xmc4000_usic_ssc_driver = {
	.probe = xmc4000_usic_ssc_probe,
	.remove = xmc4000_usic_ssc_remove,

	.driver = {
		.name = "xmc4000-usic-channel-ssc",
		.of_match_table = xmc4000_usic_ssc_of_matches,
	},
};
module_platform_driver(xmc4000_usic_ssc_driver);
