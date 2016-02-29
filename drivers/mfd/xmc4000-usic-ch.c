/*
 * XMC4500 USIC
 *
 * License: GPL-2.0+
 */
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/mfd/core.h>
#include <linux/mfd/xmc4000-usic.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

struct xmc4000_usic_channel {
	void __iomem *base;
	struct device *dev;
	struct clk_hw fd_hw;
	struct clk *fd;
	const char *pin_parents[4];
	struct clk *pin;
	struct clk *mclk;
	const char *ppp_parents[2];
	struct clk *ppp;
	struct clk *pdiv;
	struct clk *sclk;
	const char *ctqin_parents[4];
	struct clk *ctqin;
};

static unsigned long xmc4000_usic_fd_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct xmc4000_usic_channel *ch = container_of(hw, struct xmc4000_usic_channel, fd_hw);
	unsigned long rate;
	u32 fdr, step;

	fdr = readl_relaxed(ch->base + USICx_CHy_FDR);
	step = (fdr & USICx_CHy_FDR_STEP_MASK) >> USICx_CHy_FDR_STEP_OFFSET;
	switch (fdr & USICx_CHy_FDR_DM_MASK) {
	case USICx_CHy_FDR_DM_NORMAL:
		rate = parent_rate / (1024 - step);
		dev_info(ch->dev, "FD normal: %lu (parent %lu)\n", rate, parent_rate);
		break;
	case USICx_CHy_FDR_DM_FRAC:
		rate = parent_rate * step / 1024;
		dev_info(ch->dev, "FD fractional: %lu (parent %lu)\n", rate, parent_rate);
		break;
	default:
		rate = 0;
		dev_info(ch->dev, "FD off: %lu (parent %lu)\n", rate, parent_rate);
	}

	return rate;
}

static const struct clk_ops xmc4000_usic_fd_ops = {
	.recalc_rate = xmc4000_usic_fd_recalc_rate,
};

static const char * xmc4000_usic_fd_parents[] = {
	"PERIPH"
};

static const struct clk_init_data xmc4000_usic_fd_init = {
	.ops = &xmc4000_usic_fd_ops,
	.parent_names = xmc4000_usic_fd_parents,
	.num_parents = ARRAY_SIZE(xmc4000_usic_fd_parents),
};

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
	struct xmc4000_usic_channel *ch;
	struct resource *res;
	const struct mfd_cell *cell;
	struct clk_init_data fd_init;
	u32 mode;
	int ret;

	ret = of_property_read_u32(pdev->dev.of_node, "infineon,usic-mode", &mode);
	if (ret)
		return ret;

	if (mode >= ARRAY_SIZE(xmc4000_usic_cells)) {
		dev_warn(&pdev->dev, "unsupported USIC mode %u\n", mode);
		return -EINVAL;
	}

	ch = devm_kzalloc(&pdev->dev, sizeof(*ch), GFP_KERNEL);
	if (!ch)
		return -ENOMEM;

	ch->dev = &pdev->dev;
	ch->base = of_iomap(pdev->dev.of_node, 0);
	platform_set_drvdata(pdev, ch);

	fd_init = xmc4000_usic_fd_init;
	fd_init.name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "USIC.%p_FD", ch->base);
	ch->fd_hw.init = &fd_init;
	ch->fd = devm_clk_register(&pdev->dev, &ch->fd_hw);
	if (IS_ERR(ch->fd)) {
		dev_err(&pdev->dev, "failed to register FD clk (%ld)\n", PTR_ERR(ch->fd));
		return PTR_ERR(ch->fd);
	}

	ch->pin_parents[0] = __clk_get_name(ch->fd);
	ch->pin_parents[1] = NULL;
	ch->pin_parents[2] = NULL;
	ch->pin_parents[3] = NULL;
	ch->pin = clk_register_mux(&pdev->dev,
		devm_kasprintf(&pdev->dev, GFP_KERNEL, "USIC.%p_PIN", ch->base),
		ch->pin_parents, 1, 0, ch->base + USICx_CHy_BRG, 0, 2, 0, NULL);

	ch->mclk = clk_register_fixed_factor(&pdev->dev,
		devm_kasprintf(&pdev->dev, GFP_KERNEL, "USIC.%p_MCLK", ch->base),
		__clk_get_name(ch->pin), 0, 1, 2);

	ch->ppp_parents[0] = __clk_get_name(ch->pin);
	ch->ppp_parents[1] = __clk_get_name(ch->mclk);
	ch->ppp = clk_register_mux(&pdev->dev,
		devm_kasprintf(&pdev->dev, GFP_KERNEL, "USIC.%p_PPP", ch->base),
		ch->ppp_parents, 2, 0, ch->base + USICx_CHy_BRG, 0, 1, 0, NULL);

	ch->pdiv = clk_register_divider(&pdev->dev,
		devm_kasprintf(&pdev->dev, GFP_KERNEL, "USIC.%p_PDIV", ch->base),
		__clk_get_name(ch->ppp), 0, ch->base + USICx_CHy_BRG, 16, 10, 0, NULL);

	ch->sclk = clk_register_fixed_factor(&pdev->dev,
		devm_kasprintf(&pdev->dev, GFP_KERNEL, "USIC.%p_SCLK", ch->base),
		__clk_get_name(ch->pdiv), 0, 1, 2);

	ch->ctqin_parents[0] = __clk_get_name(ch->pdiv);
	ch->ctqin_parents[1] = __clk_get_name(ch->ppp);
	ch->ctqin_parents[2] = __clk_get_name(ch->sclk);
	ch->ctqin_parents[3] = __clk_get_name(ch->mclk);
	ch->ctqin = clk_register_mux(&pdev->dev,
		devm_kasprintf(&pdev->dev, GFP_KERNEL, "USIC.%p_CTQIN", ch->base),
		ch->ctqin_parents, 4, 0, ch->base + USICx_CHy_BRG, 0, 2, 0, NULL);

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
	struct xmc4000_usic_channel *ch = platform_get_drvdata(pdev);

	mfd_remove_devices(&pdev->dev);
	devm_clk_unregister(&pdev->dev, ch->fd);

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
