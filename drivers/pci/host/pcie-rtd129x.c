/*
 * Realtek RTD1295 PCIe
 *
 * Copyright (c) 2017 Andreas Färber
 *
 * Authors:
 *   James Tai <james.tai@realtek.com>
 *   Andreas Färber
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

struct rtd129x_pcie_device {
	struct platform_device *pdev;
	void __iomem *ctrl_base;
	void __iomem *cfg_base;
	struct clk *clk;
	struct reset_control *pcie_stitch_reset;
	struct reset_control *pcie_reset;
	struct reset_control *pcie_core_reset;
	struct reset_control *pcie_power_reset;
	struct reset_control *pcie_nonstich_reset;
	struct reset_control *pcie_phy_reset;
	struct reset_control *pcie_phy_mdio_reset;
	u32 speed_mode;
};

static struct pci_ops rtd129x_pcie_ops = {
};

static int rtd129x_pcie_init(struct rtd129x_pcie_device *data)
{
	u32 val;
	int ret, timeout;

	reset_control_deassert(data->pcie_stitch_reset);
	reset_control_deassert(data->pcie_reset);
	reset_control_deassert(data->pcie_core_reset);
	reset_control_deassert(data->pcie_power_reset);
	reset_control_deassert(data->pcie_nonstich_reset);
	reset_control_deassert(data->pcie_phy_reset);
	reset_control_deassert(data->pcie_phy_mdio_reset);

	ret = clk_enable(data->clk);
	if (ret)
		return ret;

	writel_relaxed(0x00140010, data->ctrl_base + 0xc00);

	if (data->speed_mode == 0) {
		val = readl_relaxed(data->ctrl_base + 0x0a0);
		val &= ~0xf;
		val |= 0x1;
		writel_relaxed(val, data->ctrl_base + 0x0a0);
	}

	/* #Write soft reset */
	writel_relaxed(0x00000003, data->ctrl_base + 0xc1c);
	mdelay(1);
	writel_relaxed(0x27f10301, data->ctrl_base + 0xc1c);
	mdelay(1);

	/* #F code, close SSC */
	writel_relaxed(0x52f50401, data->ctrl_base + 0xc1c);
	mdelay(1);

	/* #modify N code */
	writel_relaxed(0xead70501, data->ctrl_base + 0xc1c);
	mdelay(1);

	/* #modify CMU ICP (TX jitter) */
	writel_relaxed(0x000c0601, data->ctrl_base + 0xc1c);
	mdelay(1);

	/* #modify CMU RS (TX jitter) */
	writel_relaxed(0xa6530a01, data->ctrl_base + 0xc1c);
	mdelay(1);

	/* #modify AMP */
	writel_relaxed(0xd4662001, data->ctrl_base + 0xc1c);
	mdelay(1);

	/* #modify Rx parameter */
	writel_relaxed(0xa84a0101, data->ctrl_base + 0xc1c);
	mdelay(1);

	/* #clk driving */
	writel_relaxed(0xb8032b01, data->ctrl_base + 0xc1c);
	mdelay(1);

	/* #EQ */
	writel_relaxed(0x27e94301, data->ctrl_base + 0xc1c);
	mdelay(1);

	/* #F code, close SSC */
	writel_relaxed(0x52f54401, data->ctrl_base + 0xc1c);
	mdelay(1);

	/* #modify N code */
	writel_relaxed(0xead74501, data->ctrl_base + 0xc1c);
	mdelay(1);

	/* #modify CMU ICP (TX jitter) */
	writel_relaxed(0x000c4601, data->ctrl_base + 0xc1c);
	mdelay(1);

	/* #modify CMU RS (TX jitter) */
	writel_relaxed(0xa6534a01, data->ctrl_base + 0xc1c);
	mdelay(1);

	/* #modify AMP */
	writel_relaxed(0xd4776001, data->ctrl_base + 0xc1c);
	mdelay(1);

	/* #modify Rx parameter */
	writel_relaxed(0xa84a4101, data->ctrl_base + 0xc1c);
	mdelay(1);

	/* #clk driving */
	writel_relaxed(0xa8036b01, data->ctrl_base + 0xc1c);
	mdelay(1);
	writel_relaxed(0x01225a01, data->ctrl_base + 0xc1c);
	mdelay(1);

	/* TODO after phy mdio reset */

	/* set to MMIO */
	writel_relaxed(true ? 0x00040012 : 0x001E0022, data->ctrl_base + 0xc00);
	msleep(50);

	/* #Link initial setting */
	writel_relaxed(0x00010120, data->ctrl_base + 0x710);

	val = readl_relaxed(data->ctrl_base + 0xcb4);
	for (timeout = 0; !(val & 0x800) && (timeout < 60); timeout++) {
		mdelay(1);
		val = readl_relaxed(data->ctrl_base + 0xcb4);
	}
	if (!(val & 0x800)) {
		dev_err(&data->pdev->dev, "link down\n");
		ret = -ENODEV;
		goto err_disable_clk;
	}

	return 0;

err_disable_clk:
	clk_disable(data->clk);
	return ret;
}

static const struct of_device_id rtd129x_pcie_dt_ids[] = {
	{ .compatible = "realtek,rtd1295-pcie" },
	{ }
};

static int rtd129x_pcie_probe(struct platform_device *pdev)
{
	struct rtd129x_pcie_device *data;
	struct pci_bus *bus;
	LIST_HEAD(bus_res);
	struct resource *res;
	resource_size_t iobase;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->pdev = pdev;
	data->speed_mode = 1;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->ctrl_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->ctrl_base))
		return PTR_ERR(data->ctrl_base);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	data->cfg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->cfg_base))
		return PTR_ERR(data->cfg_base);

	data->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(data->clk))
		return PTR_ERR(data->clk);

	data->pcie_stitch_reset = devm_reset_control_get_exclusive(&pdev->dev, "pcie_stitch");
	if (IS_ERR(data->pcie_stitch_reset))
		return PTR_ERR(data->pcie_stitch_reset);

	data->pcie_reset = devm_reset_control_get_exclusive(&pdev->dev, "pcie");
	if (IS_ERR(data->pcie_reset))
		return PTR_ERR(data->pcie_reset);

	data->pcie_core_reset = devm_reset_control_get_exclusive(&pdev->dev, "pcie_core");
	if (IS_ERR(data->pcie_core_reset))
		return PTR_ERR(data->pcie_core_reset);

	data->pcie_power_reset = devm_reset_control_get_exclusive(&pdev->dev, "pcie_power");
	if (IS_ERR(data->pcie_power_reset))
		return PTR_ERR(data->pcie_power_reset);

	data->pcie_nonstich_reset = devm_reset_control_get_exclusive(&pdev->dev, "pcie_nonstich");
	if (IS_ERR(data->pcie_nonstich_reset))
		return PTR_ERR(data->pcie_nonstich_reset);

	data->pcie_phy_reset = devm_reset_control_get_exclusive(&pdev->dev, "pcie_phy");
	if (IS_ERR(data->pcie_phy_reset))
		return PTR_ERR(data->pcie_phy_reset);

	data->pcie_phy_mdio_reset = devm_reset_control_get_exclusive(&pdev->dev, "pcie_phy_mdio");
	if (IS_ERR(data->pcie_phy_mdio_reset))
		return PTR_ERR(data->pcie_phy_mdio_reset);

	ret = clk_prepare(data->clk);
	if (ret)
		return ret;

	ret = rtd129x_pcie_init(data);
	if (ret) {
		clk_unprepare(data->clk);
		return ret;
	}

	ret = of_pci_get_host_bridge_resources(pdev->dev.of_node, 0x1, 0xff, &bus_res, &iobase);
	if (ret) {
		clk_unprepare(data->clk);
		return ret;
	}

	bus = pci_create_root_bus(&pdev->dev, 1, &rtd129x_pcie_ops, NULL, &bus_res);
	if (IS_ERR(bus))
		return PTR_ERR(bus);

	pci_scan_child_bus(bus);
	pci_assign_unassigned_bus_resources(bus);
	pci_bus_add_devices(bus);

	dev_info(&pdev->dev, "probed");

	return 0;
}

static struct platform_driver rtd129x_pcie_platform_driver = {
	.driver = {
		.name = "pcie-rtd129x",
		.of_match_table = rtd129x_pcie_dt_ids,
	},
	.probe = rtd129x_pcie_probe,
};
builtin_platform_driver(rtd129x_pcie_platform_driver);
