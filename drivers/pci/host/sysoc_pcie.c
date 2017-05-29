/*
 * Copyright (C) 2013-2014 Fujitsu Semiconductor Ltd
 *
 * HSIO System control functions for Fujitsu SoC
 *
 * Author: Slash Huang <slash.huang@tw.fujitsu.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/of_address.h>
#include <linux/pm_domain.h>
#include <linux/delay.h>
#include <asm/dma-iommu.h>
#include "pcie_f_pcie2_dme.h"

#define PCIE_PHY_RST			0x4
#define PCIE_PHY_RST_DIS		1
#define PCIE_PHY_RST_EN			0
#define SYSOC_PCIE_TYPE		1
#define SYSOC_GRGMX_SAT		(1 << 9)
#define BIFURC_ENABLE			2
#define PCIE0_RGS0				0X34
#define PCIE1_RGS0				0X38
#define PCIE1_RGS1				0X48

struct sysoc {
	struct list_head ports;
	struct device *dev;
	struct pcie_pro *port;
	struct device_node node;
	struct clk **clk;
	void __iomem *sysoc_reg;
	void __iomem *bifurc_reg;
	u32 pm_cap;
	u32 clk_num;
	const char *phy_reset;
};

static LIST_HEAD(port_list);
/* for link list */
spinlock_t port_lock;

struct sysoc *find_sysoc(struct device *dev)
{
	unsigned long flags;
	struct sysoc *tmp, *port;

	spin_lock_irqsave(&port_lock, flags);
	list_for_each_entry_safe(port, tmp, &port_list, ports) {
		if (dev->of_node == port->dev->of_node) {
			spin_unlock_irqrestore(&port_lock, flags);
			return port;
		}
	}
	spin_unlock_irqrestore(&port_lock, flags);
	return NULL;
}

#ifdef CONFIG_PM
static int f_sysoc_pcie_pm_suspend(struct device *dev)
{
	struct sysoc *port;
	int clk_id;

	port = find_sysoc(dev);

	if (!port)
		return 0;

	for (clk_id = 0; clk_id < port->clk_num; clk_id++)
		clk_disable_unprepare(port->clk[clk_id]);

	return 0;
}

static int f_sysoc_pcie_pm_resume(struct device *dev)
{
	struct sysoc *port;
	int clk_id;

	port = find_sysoc(dev);

	if (!port || !port->sysoc_reg || !port->sysoc_reg)
		return 0;

	/* assert phy reset */

	for (clk_id = 0; clk_id < port->clk_num; clk_id++)
		if (port->clk[clk_id])
			clk_prepare_enable(port->clk[clk_id]);

	if (port->phy_reset) {
		writel(PCIE_PHY_RST_EN, port->sysoc_reg + PCIE_PHY_RST);
		usleep_range(10, 20);
	}

	/* deassert phy reset */
	if (port->phy_reset) {
		writel(PCIE_PHY_RST_DIS, port->sysoc_reg + PCIE_PHY_RST);
		usleep_range(10, 20);
	}

	return 0;
}

static int f_sysoc_pcie_runtime_suspend(struct device *dev)
{
	return f_sysoc_pcie_pm_suspend(dev);
}

static int f_sysoc_pcie_runtime_resume(struct device *dev)
{
	return f_sysoc_pcie_pm_resume(dev);
}
#endif

int f_sysoc_pcie_pm_start(struct device *dev)
{
	return f_sysoc_pcie_pm_resume(dev);
}

int f_sysoc_pcie_pm_stop(struct device *dev)
{
	return f_sysoc_pcie_pm_suspend(dev);
}

static struct gpd_dev_ops gpd_dev_ops = {
	.start = f_sysoc_pcie_pm_start,
	.stop = f_sysoc_pcie_pm_stop,
};

static int sysoc_pcie_probe(struct platform_device *pdev)
{
	struct resource res;
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node, *np_pm;
	const char *clk_nam[10] = {0};
	struct pcie_pro *port;
	unsigned long flags;
	int ret = 0, phy_reset_sz, err, clk_id = 0;
	u32 clk_num, id;
	struct sysoc *_sysoc;

	spin_lock_init(&port_lock);
	_sysoc = kzalloc(sizeof(*_sysoc), GFP_KERNEL);
	if (!_sysoc) {
		dev_err(&pdev->dev, "kzalloc _sysoc fail\n");
		return -EINVAL;
	}
	_sysoc->dev = &pdev->dev;
	_sysoc->node = *dev->of_node;

	ret = of_property_read_u32_index(node, "id", 0, &id);
	if (ret) {
		dev_err(dev, "Missing id in dt\n");
		goto err0;
	}

	/* check Bifurcation enable or not */
	if (!of_address_to_resource(node, 1, &res)) {
		_sysoc->bifurc_reg = ioremap(res.start, resource_size(&res));

		if (_sysoc->bifurc_reg) {
			ret = readl(_sysoc->bifurc_reg) & 0x3;
			dev_info(&pdev->dev, "bifurc status 0x%x\n", ret);
			/* enable Bifurcation */
			writeb(BIFURC_ENABLE, _sysoc->bifurc_reg);
		}
	}

	if (of_property_read_u32(node, "clock_num", &clk_num))
		clk_num = 0;

	if (clk_num) {
		_sysoc->clk = kmalloc(clk_num * sizeof(struct clk *),
			GFP_KERNEL);
		if(!_sysoc->clk){
			dev_err(dev, "%s %d:_sysoc->clk NULL\n",
				__func__, __LINE__);
			goto err0;
		}
		memset(_sysoc->clk, 0, clk_num * sizeof(struct clk *));
	}
	_sysoc->clk_num = clk_num;

	while (clk_num) {
		of_property_read_string_index(node,
			"clock-names", clk_id, &clk_nam[clk_id]);

		if (clk_nam[clk_id] == NULL)
			break;

		_sysoc->clk[clk_id] = of_clk_get(node, clk_id);

		if (IS_ERR(_sysoc->clk[clk_id])) {
			dev_err(_sysoc->dev, "clock%d not found.\n", clk_id);
			clk_id--;
			goto err1;
		}

		err = clk_prepare_enable(_sysoc->clk[clk_id]);
		if (err) {
			clk_id--;
			dev_err(_sysoc->dev, "clk (%d) enable fail.\n", clk_id);
			goto err1;
		}

		clk_id++;
	}

	_sysoc->phy_reset = of_get_property(node, "phy_reset", &phy_reset_sz);

	if (of_address_to_resource(node, 0, &res)) {
		dev_err(&pdev->dev, "no regs resource defined\n");
		goto err1;
	}

	_sysoc->sysoc_reg = ioremap(res.start, resource_size(&res));
	if (!_sysoc->sysoc_reg) {
		dev_err(dev, "sysoc_reg fail\n");
		goto err1;
	}

#ifdef CONFIG_PM_RUNTIME
	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);
#endif

	_sysoc->port = kmalloc(sizeof(*_sysoc->port), GFP_KERNEL);
	if (!_sysoc->port) {
		dev_err(dev, "_sysoc->port fail\n");
		goto err2;
	}
	port = _sysoc->port;

	/* assert phy reset */
	if (_sysoc->phy_reset) {
		writel(PCIE_PHY_RST_EN, _sysoc->sysoc_reg + PCIE_PHY_RST);
		usleep_range(10, 20);
	}

	/* deassert phy reset */
	if (_sysoc->phy_reset) {
		writel(PCIE_PHY_RST_DIS, _sysoc->sysoc_reg + PCIE_PHY_RST);
		usleep_range(10, 20);
	}
	port->pcie_type = readl(_sysoc->sysoc_reg) & SYSOC_PCIE_TYPE;
	port->loop_coned = readl(_sysoc->sysoc_reg) & SYSOC_GRGMX_SAT;
	port->id = id;
	dev_set_drvdata(dev, port);

	_sysoc->pm_cap = 0;
	np_pm = of_parse_phandle(pdev->dev.of_node, "power-domains", 0);
	if (np_pm) {
		_sysoc->pm_cap = 1;
		ret = pm_genpd_add_callbacks(dev, &gpd_dev_ops, NULL);
		if (ret) {
			dev_err(dev, "pm_genpd_add_callbacks fail\n");
			goto err3;
		}
	}
#ifdef CONFIG_ARM_DMA_USE_IOMMU
	if (!port->pcie_type && port->loop_coned) {
		if (dev->archdata.mapping) {
			arm_iommu_detach_device(dev);
			arm_iommu_release_mapping(dev->archdata.mapping);
			dev->archdata.mapping = NULL;
		}
	}
#endif
	spin_lock_irqsave(&port_lock, flags);
	list_add(&_sysoc->ports, &port_list);
	spin_unlock_irqrestore(&port_lock, flags);
	if (node) {
		ret = of_platform_populate(node, NULL, NULL, dev);
		if (ret) {
			dev_err(dev, "failed to add pcie core\n");
			goto err3;
		}
	}
	return 0;

err3:
	if (_sysoc->port)
		kfree(_sysoc->port);
err2:
	if (_sysoc->sysoc_reg)
		iounmap(_sysoc->sysoc_reg);

err1:
	while (_sysoc->clk_num && clk_id >= 0) {
		if (_sysoc->clk[clk_id])
			clk_disable_unprepare(_sysoc->clk[clk_id]);
		clk_id--;
	};

	if (_sysoc->clk_num && _sysoc->clk)
		kfree(_sysoc->clk);

err0:
	if (_sysoc->bifurc_reg)
		iounmap(_sysoc->bifurc_reg);

	if (_sysoc)
		kfree(_sysoc);

	return -EINVAL;
}

static int sysoc_pcie_remove_child(struct device *dev, void *unused)
{
	struct platform_device *pdev = to_platform_device(dev);

	of_device_unregister(pdev);

	return 0;
}

static int sysoc_pcie_remove(struct platform_device *pdev)
{
	int r = 0;
	struct sysoc *pcie_sysoc;

	device_for_each_child(&pdev->dev, NULL, sysoc_pcie_remove_child);

	pcie_sysoc = find_sysoc(&pdev->dev);
	if (!pcie_sysoc)
		return 0;

	if (pcie_sysoc->pm_cap) {
		r = __pm_genpd_remove_callbacks(&pdev->dev, false);
		if (r)
			dev_err(&pdev->dev, "pm_genpd_remove fail\n");
	}

	if (pcie_sysoc->sysoc_reg)
		iounmap(pcie_sysoc->sysoc_reg);

	if (pcie_sysoc->bifurc_reg)
		iounmap(pcie_sysoc->bifurc_reg);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	list_del(&pcie_sysoc->ports);

	kfree(pcie_sysoc->clk);
	kfree(pcie_sysoc->port);
	kfree(pcie_sysoc);
	return 0;
}

static const struct of_device_id sysoc_pcie_match[] = {
	{
		.compatible = "fujitsu,sysoc-f_pcie_dmx"
	},
	{/* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sysoc_pcie_match);


#ifdef CONFIG_PM
static const struct dev_pm_ops f_sysoc_pm_ops = {
	.suspend_noirq = f_sysoc_pcie_pm_suspend,
	.resume_noirq = f_sysoc_pcie_pm_resume,

#ifdef CONFIG_PM_RUNTIME
	SET_RUNTIME_PM_OPS(
	f_sysoc_pcie_runtime_suspend,
	f_sysoc_pcie_runtime_resume, NULL)
#endif
};
#endif

static struct platform_driver sysoc_pcie_driver = {
	.probe		= sysoc_pcie_probe,
	.remove		= sysoc_pcie_remove,
	.driver		= {
		.name	= "sysoc_pcie_driver",
		.of_match_table	= of_match_ptr(sysoc_pcie_match),
#ifdef CONFIG_PM
		.pm = &f_sysoc_pm_ops,
#endif
	},
};

static int __init sysoc_pcie_init(void)
{
	return platform_driver_register(&sysoc_pcie_driver);
}

static void __exit sysoc_pcie_exit(void)
{
	platform_driver_unregister(&sysoc_pcie_driver);
}

module_init(sysoc_pcie_init);
module_exit(sysoc_pcie_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("HSIO system control driver");
MODULE_AUTHOR("slash.huang@tw.fujitsu.com");
