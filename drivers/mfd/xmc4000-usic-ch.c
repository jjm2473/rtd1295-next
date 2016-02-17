/*
 * XMC4500 USIC
 */
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/mfd/core.h>
#include <linux/mfd/xmc4000-usic.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

static const struct resource channel_res[] = {
	{
		.start = 0x000,
		.end   = 0x1ff,
		.flags = IORESOURCE_MEM,
	},
};

static const struct mfd_cell xmc4000_usic_cells[] = {
	[USICx_CHy_CCR_MODE_ASC] = {
		.name = "xmc4000-usic-channel-asc",
		.of_compatible = "infineon,xmc4500-usic-channel-asc",
		.resources = channel_res,
		.num_resources = ARRAY_SIZE(channel_res),
	},
	[USICx_CHy_CCR_MODE_SSC] = {
		.name = "xmc4000-usic-channel-ssc",
		.of_compatible = "infineon,xmc4500-usic-channel-ssc",
		.resources = channel_res,
		.num_resources = ARRAY_SIZE(channel_res),
	},
	[USICx_CHy_CCR_MODE_IIS] = {
		.name = "xmc4000-usic-channel-iis",
		.of_compatible = "infineon,xmc4500-usic-channel-iis",
		.resources = channel_res,
		.num_resources = ARRAY_SIZE(channel_res),
	},
	[USICx_CHy_CCR_MODE_IIC] = {
		.name = "xmc4000-usic-channel-iic",
		.of_compatible = "infineon,xmc4500-usic-channel-iic",
		.resources = channel_res,
		.num_resources = ARRAY_SIZE(channel_res),
	},
};

static int xmc4000_usic_probe(struct platform_device *pdev)
{
	struct resource *res;
	const struct mfd_cell *cell;
	u32 mode;
	int ret;

	ret = of_property_read_u32(pdev->dev.of_node, "infineon,usic-mode", &mode);
	if (ret)
		return ret;

	if (mode >= ARRAY_SIZE(xmc4000_usic_cells)) {
		dev_warn(&pdev->dev, "unsupported USIC mode %u\n", mode);
		return -EINVAL;
	}

	cell = &xmc4000_usic_cells[mode];
	if (cell->name) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res) {
			dev_err(&pdev->dev, "no USIC memory region provided\n");
			return -ENOENT;
		}

		ret = mfd_add_devices(&pdev->dev, res->start,
				      cell, 1, res, -1, NULL);
		if (ret) {
			dev_err(&pdev->dev, "failed to add MFD devices (%d)\n", ret);
			return ret;
		}
	}

	return 0;
}

static int xmc4000_usic_remove(struct platform_device *pdev)
{
	mfd_remove_devices(&pdev->dev);

	return 0;
}

static const struct of_device_id xmc4000_usic_of_matches[] = {
	{ .compatible = "infineon,xmc4500-usic-channel" },
	{ }
};
MODULE_DEVICE_TABLE(of, xmc4000_usic_of_matches);

static struct platform_driver xmc4000_usic_driver = {
	.probe = xmc4000_usic_probe,
	.remove = xmc4000_usic_remove,

	.driver = {
		.name = "xmc4000-usic-channel",
		.of_match_table = xmc4000_usic_of_matches,
	},
};
module_platform_driver(xmc4000_usic_driver);
