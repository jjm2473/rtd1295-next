/*
 * XMC4500
 */

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/reset.h>
#include "sdhci-pltfm.h"

static int xmc4000_sdhci_probe(struct platform_device *pdev)
{
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct clk *clk_xin;
	int ret;

	clk_xin = devm_clk_get(&pdev->dev, "clk_xin");
	if (IS_ERR(clk_xin)) {
		dev_err(&pdev->dev, "clk_xin clock not found.\n");
		ret = PTR_ERR(clk_xin);
		goto err_clk_xin_get;
	}

	ret = clk_prepare_enable(clk_xin);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable clk_xin.\n");
		goto err_clk_xin_enable;
	}

	ret = device_reset(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "Unable to reset.\n");
		goto err_reset;
	}

	host = sdhci_pltfm_init(pdev, NULL, 0);
	if (IS_ERR(host)) {
		ret = PTR_ERR(host);
		goto err_sdhci_init;
	}

	sdhci_get_of_property(pdev);

	pltfm_host = sdhci_priv(host);
	pltfm_host->clk = clk_xin;

	ret = mmc_of_parse(host->mmc);
	if (ret) {
		dev_err(&pdev->dev, "parsing dt failed (%d)\n", ret);
		goto err_mmc_parse;
	}

	ret = sdhci_add_host(host);
	if (ret)
		goto err_sdhci_add;

	return 0;

err_sdhci_add:
err_mmc_parse:
	sdhci_pltfm_free(pdev);
err_sdhci_init:
err_reset:
	clk_disable_unprepare(clk_xin);
err_clk_xin_enable:
	devm_clk_put(&pdev->dev, clk_xin);
err_clk_xin_get:
	return ret;
}

static const struct of_device_id xmc4000_sdhci_of_match[] = {
	{ .compatible = "infineon,xmc4500-sdmmc" },
	{ }
};
MODULE_DEVICE_TABLE(of, xmc4000_sdhci_of_match);

static struct platform_driver xmc4000_sdhci_driver = {
	.driver = {
		.name = "sdhci-xmc4000",
		.of_match_table = xmc4000_sdhci_of_match,
		.pm = SDHCI_PLTFM_PMOPS,
	},
	.probe = xmc4000_sdhci_probe,
};

module_platform_driver(xmc4000_sdhci_driver);

MODULE_LICENSE("GPL");
