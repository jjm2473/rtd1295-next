#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

static const struct of_device_id rtd_powerctrl_dt_ids[] = {
	 { .compatible = "realtek,rtd1295-powerctrl-simple" },
	 { }
};

static int rtd_powerctrl_probe(struct platform_device *pdev)
{
	struct resource *res;
	void __iomem *base;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	dev_info(&pdev->dev, "0x%08x\n", readl(base));

	return 0;
}

static struct platform_driver rtd_powerctrl_driver = {
	.probe = rtd_powerctrl_probe,
	.driver = {
		.name = "rtd1295-powerctrl",
		.of_match_table	= rtd_powerctrl_dt_ids,
	},
};
builtin_platform_driver(rtd_powerctrl_driver);
