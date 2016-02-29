/*
 * XMC4500 USIC IIC
 *
 * License: GPL-2.0+
 */
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/mfd/xmc4000-usic.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#define USICx_CHy_PCR_IIC_SLAD_OFFSET	0
#define USICx_CHy_PCR_IIC_SLAD_MASK	(0xff << USICx_CHy_PCR_IIC_SLAD_OFFSET)

static inline u16 xmc4000_usic_iic_slad(u16 slave_addr)
{
	if ((slave_addr & 0x7c00) == 0x7800)
		return ((slave_addr << 1) & 0xfe00) | (slave_addr & 0xff);
	else
		return slave_addr << 8;
}

static int xmc4000_usic_iic_probe(struct platform_device *pdev)
{
	void __iomem *base;
	u32 mode, reg;
	int ret;

	ret = of_property_read_u32(pdev->dev.parent->of_node,
				   "infineon,usic-mode", &mode);
	if (ret)
		return ret;

	if (WARN_ON(mode != USICx_CHy_CCR_MODE_IIC))
		return -EINVAL;

	base = of_iomap(pdev->dev.parent->of_node, 0);

	reg = readl_relaxed(base + USICx_CHy_CCFG);
	if (!(reg & USICx_CHy_CCFG_IIC)) {
		dev_warn(&pdev->dev, "no support for IIC on this channel (%08X)\n", reg);
		ret = -EINVAL;
		goto err_ccfg_iic;
	}

	reg = readl_relaxed(base + USICx_CHy_KSCFG);
	reg |= USICx_CHy_KSCFG_BPMODEN | USICx_CHy_KSCFG_MODEN;
	writel_relaxed(reg, base + USICx_CHy_KSCFG);

	reg = readl_relaxed(base + USICx_CHy_CCR);
	reg &= ~USICx_CHy_CCR_MODE_MASK;
	writel_relaxed(reg, base + USICx_CHy_CCR);

	reg = (0x3 << USICx_CHy_SCTR_TRM_OFFSET) & USICx_CHy_SCTR_TRM_MASK;
	reg |= ((8 - 1) << USICx_CHy_SCTR_WLE_OFFSET) & USICx_CHy_SCTR_WLE_MASK;
	reg |= USICx_CHy_SCTR_FLE_MASK | USICx_CHy_SCTR_SDIR_MSB | USICx_CHy_SCTR_PDL_1;
	writel_relaxed(reg, base + USICx_CHy_SCTR);

	reg = (xmc4000_usic_iic_slad(0) << USICx_CHy_PCR_IIC_SLAD_OFFSET) & USICx_CHy_PCR_IIC_SLAD_MASK;
	writel_relaxed(reg, base + USICx_CHy_PCR);

	return 0;

err_ccfg_iic:
	return ret;
}

static int xmc4000_usic_iic_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id xmc4000_usic_iic_of_matches[] = {
	{ .compatible = "infineon,xmc4500-usic-channel-iic" },
	{ }
};
MODULE_DEVICE_TABLE(of, xmc4000_usic_iic_of_matches);

static struct platform_driver xmc4000_usic_iic_driver = {
	.probe = xmc4000_usic_iic_probe,
	.remove = xmc4000_usic_iic_remove,

	.driver = {
		.name = "xmc4000-usic-channel-iic",
		.of_match_table = xmc4000_usic_iic_of_matches,
	},
};
module_platform_driver(xmc4000_usic_iic_driver);
