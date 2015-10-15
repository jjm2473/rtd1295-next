/*
 * Spansion FM4 Ethernet
 *
 * License: GPL-2.0+
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/stmmac.h>

#include "stmmac_platform.h"

static int fm4_dwmac_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	int ret;

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	plat_dat = stmmac_probe_config_dt(pdev, &stmmac_res.mac);
	if (IS_ERR(plat_dat))
		return PTR_ERR(plat_dat);

	if (plat_dat->interface == PHY_INTERFACE_MODE_MII)
		;
	else if (plat_dat->interface == PHY_INTERFACE_MODE_RMII)
		;
	else {
		dev_err(&pdev->dev, "Only MII and RMII mode supported\n");
		return -EINVAL;
	}

	return stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
}

static const struct of_device_id fm4_dwmac_of_match[] = {
	{ .compatible = "cypress,fm4-dwmac" },
	{}
};
MODULE_DEVICE_TABLE(of, fm4_dwmac_of_match);

static struct platform_driver fm4_dwmac_driver = {
	.probe = fm4_dwmac_probe,
	.driver = {
		.name = "fm4-dwmac",
		.of_match_table = fm4_dwmac_of_match,
	},
};
module_platform_driver(fm4_dwmac_driver);

MODULE_LICENSE("GPL");
