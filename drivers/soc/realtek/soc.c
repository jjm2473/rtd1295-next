#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sys_soc.h>

#define REG_CHIP_ID	0x0
#define REG_CHIP_REV	0x4

static const struct of_device_id rtd_soc_dt_ids[] = {
	 { .compatible = "realtek,rtd1195-chip" },
	 { }
};

static int rtd_soc_probe(struct platform_device *pdev)
{
	struct soc_device_attribute *soc_dev_attr;
	struct soc_device *soc_dev;
	struct device_node *node;
	struct resource *res;
	void __iomem *base;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr)
		return -ENODEV;

	soc_dev_attr->family = "Realtek RTD1x9x";

	node = of_find_node_by_path("/");
	of_property_read_string(node, "model", &soc_dev_attr->machine);
	of_node_put(node);

	soc_dev_attr->revision = kasprintf(GFP_KERNEL, "%u", readl(base + REG_CHIP_REV) >> 16);

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR(soc_dev)) {
		kfree(soc_dev_attr->revision);
		kfree(soc_dev_attr);
		return PTR_ERR(soc_dev);
	}

	dev_info(soc_device_to_device(soc_dev), "chipid  = 0x%08x\n", readl(base + REG_CHIP_ID));
	dev_info(soc_device_to_device(soc_dev), "chiprev = 0x%08x\n", readl(base + REG_CHIP_REV));

	return 0;
}

static struct platform_driver rtd_soc_driver = {
	.probe = rtd_soc_probe,
	.driver = {
		.name = "rtd1295-soc",
		.of_match_table	= rtd_soc_dt_ids,
	},
};
builtin_platform_driver(rtd_soc_driver);
