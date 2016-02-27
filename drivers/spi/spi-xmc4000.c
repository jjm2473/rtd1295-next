/*
 * XMC4500 USIC SSC
 *
 * License: GPL-2.0+
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

#define USICx_CHy_PCR_SSC_MSLSEN	BIT(0)
#define USICx_CHy_PCR_SSC_SELCTR	BIT(1)
#define USICx_CHy_PCR_SSC_SELINV	BIT(2)
#define USICx_CHy_PCR_SSC_FEM		BIT(3)
#define USICx_CHy_PCR_SSC_MSLSIEN	BIT(14)

#define USICx_CHy_PCR_SSC_CTQSEL1_OFFSET	4
#define USICx_CHy_PCR_SSC_CTQSEL1_PDIV		(0x0 << USICx_CHy_PCR_SSC_CTQSEL1_OFFSET)
#define USICx_CHy_PCR_SSC_CTQSEL1_PPP		(0x1 << USICx_CHy_PCR_SSC_CTQSEL1_OFFSET)
#define USICx_CHy_PCR_SSC_CTQSEL1_SCLK		(0x2 << USICx_CHy_PCR_SSC_CTQSEL1_OFFSET)
#define USICx_CHy_PCR_SSC_CTQSEL1_MCLK		(0x3 << USICx_CHy_PCR_SSC_CTQSEL1_OFFSET)
#define USICx_CHy_PCR_SSC_CTQSEL1_MASK		(0x3 << USICx_CHy_PCR_SSC_CTQSEL1_OFFSET)

#define USICx_CHy_PCR_SSC_DCTQ1_OFFSET		8
#define USICx_CHy_PCR_SSC_DCTQ1_MASK		(0x1f << USICx_CHy_PCR_SSC_DCTQ1_OFFSET)

#define USICx_CHy_PCR_SSC_SELO_OFFSET		16
#define USICx_CHy_PCR_SSC_SELO_MASK		(0xff << USICx_CHy_PCR_SSC_SELO_OFFSET)

struct xmc4000_ssc {
	void __iomem *base;
};

static int xmc4000_usic_ssc_setup(struct spi_device *spi)
{
	if (spi->master->busy)
		return -EBUSY;
	return 0;
}

static void xmc4000_usic_ssc_set_cs(struct spi_device *spi, bool is_high)
{
}

static int xmc4000_usic_ssc_transfer_one(struct spi_master *master,
					 struct spi_device *spi,
					 struct spi_transfer *transfer)
{
	return 0;
}

static int xmc4000_usic_ssc_prepare_transfer_hardware(struct spi_master *master)
{
	return 0;
}

static int xmc4000_usic_ssc_unprepare_transfer_hardware(struct spi_master *master)
{
	return 0;
}

static int xmc4000_usic_ssc_probe(struct platform_device *pdev)
{
	struct xmc4000_ssc *ssc;
	struct spi_master *master;
	u32 mode, reg;
	int ret;

	ret = of_property_read_u32(pdev->dev.parent->of_node,
				   "infineon,usic-mode", &mode);
	if (ret)
		return ret;

	if (WARN_ON(mode != USICx_CHy_CCR_MODE_SSC))
		return -EINVAL;

	master = spi_alloc_master(&pdev->dev, sizeof(*ssc));
	if (!master)
		return -ENOMEM;

	master->dev.of_node = pdev->dev.parent->of_node;
	master->num_chipselect = 1;
	master->setup = xmc4000_usic_ssc_setup;
	master->set_cs = xmc4000_usic_ssc_set_cs;
	master->transfer_one = xmc4000_usic_ssc_transfer_one;
	master->prepare_transfer_hardware = xmc4000_usic_ssc_prepare_transfer_hardware;
	master->unprepare_transfer_hardware = xmc4000_usic_ssc_unprepare_transfer_hardware;
	master->max_speed_hz = 100000;
	master->bits_per_word_mask = SPI_BPW_MASK(8);
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_RX_DUAL | SPI_RX_QUAD |
			    SPI_TX_DUAL | SPI_TX_QUAD;
	platform_set_drvdata(pdev, master);

	ssc = spi_master_get_devdata(master);
	ssc->base = of_iomap(pdev->dev.parent->of_node, 0);

	reg = readl_relaxed(ssc->base + USICx_CHy_CCFG);
	if (!(reg & USICx_CHy_CCFG_SSC)) {
		dev_warn(&pdev->dev, "no support for SSC on this channel (%08X)\n", reg);
		ret = -EINVAL;
		goto err_ccfg_ssc;
	}

	reg = readl_relaxed(ssc->base + USICx_CHy_CCR);
	reg &= ~USICx_CHy_CCR_MODE_MASK;
	writel_relaxed(reg, ssc->base + USICx_CHy_CCR);

	reg = readl_relaxed(ssc->base + USICx_CHy_KSCFG);
	reg |= USICx_CHy_KSCFG_BPMODEN | USICx_CHy_KSCFG_MODEN;
	writel_relaxed(reg, ssc->base + USICx_CHy_KSCFG);

	// ratio = ((120000000.f / (2 * 1024)) * 500) / 100000.f = 292.96875
	// pdiv = 291.96875
	// step = ((((100000 * 1024) / 1000000) * 2) * (1 + pdiv)) / 120 = 500

	reg = readl_relaxed(ssc->base + USICx_CHy_FDR);
	reg &= ~USICx_CHy_FDR_DM_MASK;
	reg |= USICx_CHy_FDR_DM_FRAC;
	writel_relaxed(reg, ssc->base + USICx_CHy_FDR);

	reg |= (500 << USICx_CHy_FDR_STEP_OFFSET) & USICx_CHy_FDR_STEP_MASK;
	writel_relaxed(reg, ssc->base + USICx_CHy_FDR);

	reg = readl_relaxed(ssc->base + USICx_CHy_BRG);
	reg &= ~(USICx_CHy_BRG_SCLKCFG_MASK | USICx_CHy_BRG_PDIV_MASK | USICx_CHy_BRG_DCTQ_MASK | USICx_CHy_BRG_CTQSEL_MASK);
	reg |= (1 << USICx_CHy_BRG_DCTQ_OFFSET) & USICx_CHy_BRG_DCTQ_MASK;
	reg |= (291 << USICx_CHy_BRG_PDIV_OFFSET) & USICx_CHy_BRG_PDIV_MASK;
	reg |= USICx_CHy_BRG_SCLKCFG_PASSIVE_1 | USICx_CHy_BRG_CTQSEL_SCLK;
	writel_relaxed(reg, ssc->base + USICx_CHy_BRG);

	reg = readl_relaxed(ssc->base + USICx_CHy_SCTR);
	reg &= ~(USICx_CHy_SCTR_WLE_MASK | USICx_CHy_SCTR_FLE_MASK | USICx_CHy_SCTR_TRM_MASK);
	reg |= (0x1 << USICx_CHy_SCTR_TRM_OFFSET) & USICx_CHy_SCTR_TRM_MASK;
	reg |= ((8 - 1) << USICx_CHy_SCTR_FLE_OFFSET) & USICx_CHy_SCTR_FLE_MASK;
	reg |= ((8 - 1) << USICx_CHy_SCTR_WLE_OFFSET) & USICx_CHy_SCTR_WLE_MASK;
	reg |= USICx_CHy_SCTR_PDL_1 | USICx_CHy_SCTR_SDIR_MSB;
	writel_relaxed(reg, ssc->base + USICx_CHy_SCTR);

	/* begin quad mode */

	reg = readl_relaxed(ssc->base + USICx_CHy_CCR);
	reg &= ~USICx_CHy_CCR_HPCEN_MASK;
	reg |= (0x3 << USICx_CHy_CCR_HPCEN_OFFSET) & USICx_CHy_CCR_HPCEN_MASK;
	writel_relaxed(reg, ssc->base + USICx_CHy_CCR);

	reg = readl_relaxed(ssc->base + USICx_CHy_TCSR);
	reg |= USICx_CHy_TCSR_HPCMD;
	writel_relaxed(reg, ssc->base + USICx_CHy_TCSR);

	reg = readl_relaxed(ssc->base + USICx_CHy_DX0CR);
	reg &= ~(USICx_CHy_DXnCR_DSEL_MASK);
	reg |= (0x6 << USICx_CHy_DXnCR_DSEL_OFFSET) & USICx_CHy_DXnCR_DSEL_MASK;
	reg |= USICx_CHy_DXnCR_INSW;
	writel_relaxed(reg, ssc->base + USICx_CHy_DX0CR);

	reg = readl_relaxed(ssc->base + USICx_CHy_DX3CR);
	reg &= ~(USICx_CHy_DXnCR_DSEL_MASK);
	reg |= (0x6 << USICx_CHy_DXnCR_DSEL_OFFSET) & USICx_CHy_DXnCR_DSEL_MASK;
	reg |= USICx_CHy_DXnCR_INSW;
	writel_relaxed(reg, ssc->base + USICx_CHy_DX3CR);

	reg = readl_relaxed(ssc->base + USICx_CHy_DX4CR);
	reg &= ~(USICx_CHy_DXnCR_DSEL_MASK);
	reg |= (0x6 << USICx_CHy_DXnCR_DSEL_OFFSET) & USICx_CHy_DXnCR_DSEL_MASK;
	reg |= USICx_CHy_DXnCR_INSW;
	writel_relaxed(reg, ssc->base + USICx_CHy_DX4CR);

	reg = readl_relaxed(ssc->base + USICx_CHy_DX5CR);
	reg &= ~(USICx_CHy_DXnCR_DSEL_MASK);
	reg |= (0x6 << USICx_CHy_DXnCR_DSEL_OFFSET) & USICx_CHy_DXnCR_DSEL_MASK;
	reg |= USICx_CHy_DXnCR_INSW;
	writel_relaxed(reg, ssc->base + USICx_CHy_DX5CR);

	/* end quad mode */

	reg = readl_relaxed(ssc->base + USICx_CHy_TCSR);
	reg &= ~(USICx_CHy_TCSR_TDEN_MASK);
	reg |= (0x1 << USICx_CHy_TCSR_TDEN_OFFSET) & USICx_CHy_TCSR_TDEN_MASK;
	reg |= USICx_CHy_TCSR_TDSSM;
	writel_relaxed(reg, ssc->base + USICx_CHy_TCSR);

	reg = readl_relaxed(ssc->base + USICx_CHy_PCR);
	reg &= ~(USICx_CHy_PCR_SSC_SELO_MASK | USICx_CHy_PCR_SSC_MSLSIEN | USICx_CHy_PCR_SSC_DCTQ1_MASK | USICx_CHy_PCR_SSC_CTQSEL1_MASK);
	reg |= (0 << USICx_CHy_PCR_SSC_DCTQ1_OFFSET) & USICx_CHy_PCR_SSC_DCTQ1_MASK;
	reg |= (BIT(1) << USICx_CHy_PCR_SSC_SELO_OFFSET) & USICx_CHy_PCR_SSC_SELO_MASK;
	reg |= USICx_CHy_PCR_SSC_CTQSEL1_SCLK;
	reg |= USICx_CHy_PCR_SSC_FEM | USICx_CHy_PCR_SSC_SELINV | USICx_CHy_PCR_SSC_SELCTR | USICx_CHy_PCR_SSC_MSLSEN;
	writel_relaxed(reg, ssc->base + USICx_CHy_PCR);

	reg = readl_relaxed(ssc->base + USICx_CHy_TBCTR);
	reg &= ~(USICx_CHy_TBCTR_LIMIT_MASK);
	reg |= (1 << USICx_CHy_TBCTR_LIMIT_OFFSET) & USICx_CHy_TBCTR_LIMIT_MASK;
	writel_relaxed(reg, ssc->base + USICx_CHy_TBCTR);

	reg = readl_relaxed(ssc->base + USICx_CHy_RBCTR);
	reg &= ~(USICx_CHy_RBCTR_LIMIT_MASK);
	reg |= (1 << USICx_CHy_RBCTR_LIMIT_OFFSET) & USICx_CHy_RBCTR_LIMIT_MASK;
	writel_relaxed(reg, ssc->base + USICx_CHy_RBCTR);

	reg = readl_relaxed(ssc->base + USICx_CHy_CCR);
	reg &= ~(USICx_CHy_CCR_MODE_MASK);
	reg |= USICx_CHy_CCR_MODE_SSC;
	writel_relaxed(reg, ssc->base + USICx_CHy_CCR);

	ret = spi_register_master(master);
	if (ret)
		goto err_register_master;

	return 0;

err_register_master:
err_ccfg_ssc:
	spi_master_put(master);
	return ret;
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
