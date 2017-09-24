#include <linux/ahci_platform.h>
#include <linux/bitops.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include "ahci.h"

#define DRV_NAME "ahci_rtd129x"

static void writel_delay(unsigned int value, void __iomem *address)
{
	writel(value, address);
	mdelay(1);
}

static void rtd129x_ahci_phy_init(struct device *dev, void __iomem *base, int port)
{
	writel_delay(port, base + 0xF64);

	writel_delay(0x00001111, base + 0xF60);
	writel_delay(0x00005111, base + 0xF60);
	writel_delay(0x00009111, base + 0xF60);
#if 0
	writel_delay(0x538E0411, base + 0xF60);
	writel_delay(0x538E4411, base + 0xF60);
	writel_delay(0x538E8411, base + 0xF60);

	writel_delay(0x3b6a0511, base + 0xF60);
	writel_delay(0x3b6a4511, base + 0xF60);
	writel_delay(0x3b6a8511, base + 0xF60);

	writel_delay(0xE0500111, base + 0xF60);
	writel_delay(0xE0504111, base + 0xF60);
	writel_delay(0xE04C8111, base + 0xF60);

	writel_delay(0x00110611, base + 0xF60);
	writel_delay(0x00114611, base + 0xF60);
	writel_delay(0x00118611, base + 0xF60);

	writel_delay(0xA6000A11, base + 0xF60);
	writel_delay(0xA6004A11, base + 0xF60);
	writel_delay(0xA6008A11, base + 0xF60);

	writel_delay(0x27FD8211, base + 0xF60);
	writel_delay(0xA6408A11, base + 0xF60);
	writel_delay(0x041BA611, base + 0xF60);
#else
	if (true) {
		dev_info(dev, "disabling spread-spectrum\n");
		writel_delay(0x538E0411, base + 0xF60);
		writel_delay(0x538E4411, base + 0xF60);
		writel_delay(0x538E8411, base + 0xF60);
	} else {
		dev_info(dev, "enabling spread-spectrum\n");
		writel_delay(0x738E0411, base + 0xF60);
		writel_delay(0x738E4411, base + 0xF60);
		writel_delay(0x738E8411, base + 0xF60);

		writel_delay(0x35910811, base + 0xF60);
		writel_delay(0x35914811, base + 0xF60);
		writel_delay(0x35918811 , base + 0xF60);

		writel_delay(0x02342711, base + 0xF60);
		writel_delay(0x02346711, base + 0xF60);
		writel_delay(0x0234a711, base + 0xF60);
	}
	writel_delay(0x336a0511, base + 0xF60);
	writel_delay(0x336a4511, base + 0xF60);
	writel_delay(0x336a8511, base + 0xF60);

	writel_delay(0xE0700111, base + 0xF60);
	writel_delay(0xE05C4111, base + 0xF60);
	writel_delay(0xE04A8111, base + 0xF60);

	writel_delay(0x00150611, base + 0xF60);
	writel_delay(0x00154611, base + 0xF60);
	writel_delay(0x00158611, base + 0xF60);

	writel_delay(0xC6000A11, base + 0xF60);
	writel_delay(0xC6004A11, base + 0xF60);
	writel_delay(0xC6008A11, base + 0xF60);

	writel_delay(0x70000211, base + 0xF60);
	writel_delay(0x70004211, base + 0xF60);
	writel_delay(0x70008211, base + 0xF60);

	writel_delay(0xC6600A11, base + 0xF60);
	writel_delay(0xC6604A11, base + 0xF60);
	writel_delay(0xC6608A11, base + 0xF60);

	writel_delay(0x20041911, base + 0xF60);
	writel_delay(0x20045911, base + 0xF60);
	writel_delay(0x20049911, base + 0xF60);

	writel_delay(0x94aa2011, base + 0xF60);
	writel_delay(0x94aa6011, base + 0xF60);
	writel_delay(0x94aaa011, base + 0xF60);
#endif

	writel_delay(0x17171511, base + 0xF60);
	writel_delay(0x17175511, base + 0xF60);
	writel_delay(0x17179511, base + 0xF60);

	writel_delay(0x07701611, base + 0xF60);
	writel_delay(0x07705611, base + 0xF60);
	writel_delay(0x07709611, base + 0xF60);

// for rx sensitivity
	writel_delay(0x72100911, base + 0xF60);
	writel_delay(0x72104911, base + 0xF60);
	writel_delay(0x72108911, base + 0xF60);
/*	if(ahci_dev->port[port]->phy_status==0) {
		writel_delay(0x27640311, base + 0xF60);
		writel_delay(0x27644311, base + 0xF60);
		writel_delay(0x27648311, base + 0xF60);
	} else if(ahci_dev->port[port]->phy_status==2) {
		writel_delay(0x27710311, base + 0xF60);
		writel_delay(0x27714311, base + 0xF60);
		writel_delay(0x27718311, base + 0xF60);
	}*/
	writel_delay(0x27710311, base + 0xF60);
	writel_delay(0x27684311, base + 0xF60);
	writel_delay(0x27688311, base + 0xF60);

	writel_delay(0x29001011, base + 0xF60);
	writel_delay(0x29005011, base + 0xF60);
	writel_delay(0x29009011, base + 0xF60);

	if (false) {
		printk("[SATA] set tx-driving to L (level 2)\n");
		writel_delay(0x94a72011, base + 0xF60);
		writel_delay(0x94a76011, base + 0xF60);
		writel_delay(0x94a7a011, base + 0xF60);
		writel_delay(0x587a2111, base + 0xF60);
		writel_delay(0x587a6111, base + 0xF60);
		writel_delay(0x587aa111, base + 0xF60);
	} else if (of_machine_is_compatible("synology,ds418j")) { // for DS418j
		printk("[SATA] set tx-driving to L (level 8)\n");
		if(port==0) {
			writel_delay(0x94a82011, base + 0xF60);
			writel_delay(0x94a86011, base + 0xF60);
			writel_delay(0x94a8a011, base + 0xF60);
			writel_delay(0x588a2111, base + 0xF60);
			writel_delay(0x588a6111, base + 0xF60);
			writel_delay(0x588aa111, base + 0xF60);
		} else if(port==1) {
			writel_delay(0x94a82011, base + 0xF60);
			writel_delay(0x94a86011, base + 0xF60);
			writel_delay(0x94a8a011, base + 0xF60);
			writel_delay(0x58da2111, base + 0xF60);
			writel_delay(0x58da6111, base + 0xF60);
			writel_delay(0x58daa111, base + 0xF60);
		}
	} else if (of_machine_is_compatible("synology,ds418")) { // for DS418
		printk("[SATA] set tx-driving to L (level 6)\n");
		if(port==0) {
			writel_delay(0x94aa2011, base + 0xF60);
			writel_delay(0x94aa6011, base + 0xF60);
			writel_delay(0x94aaa011, base + 0xF60);
			writel_delay(0xa86a2111, base + 0xF60);
			writel_delay(0xa86a6111, base + 0xF60);
			writel_delay(0xa86aa111, base + 0xF60);
		} else if(port==1) {
			writel_delay(0x94a42011, base + 0xF60);
			writel_delay(0x94a46011, base + 0xF60);
			writel_delay(0x94a4a011, base + 0xF60);
			writel_delay(0x68ca2111, base + 0xF60);
			writel_delay(0x68ca6111, base + 0xF60);
			writel_delay(0x68caa111, base + 0xF60);
		}
	} else if (false) { // for DS218play
		printk("[SATA] set tx-driving to L (level 4)\n");
		if(port==0) {
			writel_delay(0x94a72011, base + 0xF60);
			writel_delay(0x94a76011, base + 0xF60);
			writel_delay(0x94a7a011, base + 0xF60);
			writel_delay(0x587a2111, base + 0xF60);
			writel_delay(0x587a6111, base + 0xF60);
			writel_delay(0x587aa111, base + 0xF60);
		} else if(port==1) {
			writel_delay(0x94a72011, base + 0xF60);
			writel_delay(0x94a76011, base + 0xF60);
			writel_delay(0x94a7a011, base + 0xF60);
			writel_delay(0x587a2111, base + 0xF60);
			writel_delay(0x587a6111, base + 0xF60);
			writel_delay(0x587aa111, base + 0xF60);
		}
	} else if (false) { // for DS118
		printk("[SATA] set tx-driving to L (level 10)\n");
		if(port==0) {
			writel_delay(0x94a72011, base + 0xF60);
			writel_delay(0x94a76011, base + 0xF60);
			writel_delay(0x94a7a011, base + 0xF60);
			writel_delay(0x383a2111, base + 0xF60);
			writel_delay(0x383a6111, base + 0xF60);
			writel_delay(0x383aa111, base + 0xF60);
		}
	}
	// RX power saving off
	writel_delay(0x40000C11, base + 0xF60);
	writel_delay(0x40004C11, base + 0xF60);
	writel_delay(0x40008C11, base + 0xF60);

	writel_delay(0x00271711, base + 0xF60);
	writel_delay(0x00275711, base + 0xF60);
	writel_delay(0x00279711, base + 0xF60);
}

static void rtd129x_ahci_mac_init(struct device *dev, void __iomem *base, int port)
{
	void __iomem *port_base = base + port * 0x80;
	u32 val;

	writel_delay(port, base + 0xF64);
	/* SATA MAC */
//	writel_delay(0x2, port_base + 0x144);
	writel_delay(0x6726ff81, base);
	val = readl(base);
	writel_delay(0x6737ff81, base);
	val = readl(base);

//	writel_delay(0x83090c15, base + 0xbc);
//	writel_delay(0x83090c15, base + 0xbc);

	writel_delay(0x80000001, base + 0x4);
	writel_delay(0x80000002, base + 0x4);

	val = readl(base + 0x14);
	writel_delay((val & ~0x1), base + 0x14);
	val = readl(base + 0xC);
	writel_delay((val | 0x3), base + 0xC);
	val = readl(base + 0x18);
	val |= port << 1;
	writel_delay(val, base + 0x18);

	writel_delay(0xffffffff, port_base + 0x114);
//	writel_delay(0x05040000, port_base + 0x100);
//	writel_delay(0x05040400, port_base + 0x108);

	val = readl(port_base + 0x170);
	writel_delay(0x88, port_base + 0x170);
	val = readl(port_base + 0x118);
	writel_delay(0x10, port_base + 0x118);
	val = readl(port_base + 0x118);
	writel_delay(0x4016, port_base + 0x118);
	val = readl(port_base + 0x140);
	writel_delay(0xf000, port_base + 0x140);

	writel_delay(0x3c300, base + 0xf20);

	writel_delay(0x700, base + 0xA4);
	//Set to Auto mode
	if (true)
		writel_delay(0xA, base + 0xF68);
	else if (false)
		writel_delay(0x5, base + 0xF68);
	else if (false)
		writel_delay(0x0, base + 0xF68);
}

static int send_oob(void __iomem *ukbase, unsigned int port)
{
	u32 val = 0;

	if (port == 0) {
		val = readl(ukbase + 0x80);
		val |= 0x115;
	} else if (port == 1) {
		val = readl(ukbase + 0x80);
		val |= 0x12A;
	}
	writel(val, ukbase + 0x80);

	return 0;
}

static const struct ata_port_info rtd129x_ahci_port_info = {
	.flags		= AHCI_FLAG_COMMON | ATA_FLAG_EM | ATA_FLAG_SW_ACTIVITY,
	.pio_mask	= ATA_PIO4,
	.udma_mask	= ATA_UDMA6,
	.port_ops	= &ahci_platform_ops,
};

static struct scsi_host_template rtd129x_ahci_scsi_host_template = {
	AHCI_SHT(DRV_NAME),
};

static const struct of_device_id rtd129x_ahci_dt_ids[] = {
	{ .compatible = "realtek,rtd1295-ahci" },
	{ }
};

static int rtd129x_ahci_probe(struct platform_device *pdev)
{
	struct ahci_host_priv *hpriv;
	void __iomem *ukbase;
	struct device_node *child;
	int rc;

	hpriv = ahci_platform_get_resources(pdev, 0);
	if (IS_ERR(hpriv))
		return PTR_ERR(hpriv);

	ukbase = of_iomap(pdev->dev.of_node, 1);
	if (!ukbase)
		return -ENOMEM;

	rc = ahci_platform_enable_resources(hpriv);
	if (rc)
		return rc;

	for_each_child_of_node(pdev->dev.of_node, child) {
		struct platform_device *port_pdev = NULL;
		struct device *port_dev;
		struct reset_control *sata_reset;
		struct reset_control *sata_func_reset;
		struct reset_control *phy_reset;
		struct reset_control *phy_pow_reset;
		u32 port;

		if (!of_device_is_available(child))
			continue;

		if (of_property_read_u32(child, "reg", &port)) {
			rc = -EINVAL;
			goto disable_resources;
		}

		if (port >= 2) {
			dev_warn(&pdev->dev, "invalid port number %d\n", port);
			continue;
		}

#ifdef CONFIG_OF_ADDRESS
		port_pdev = of_find_device_by_node(child);
		if (port_pdev) {
			port_dev = &port_pdev->dev;
			sata_reset = devm_reset_control_get_exclusive_by_index(port_dev, 0);
			sata_func_reset = devm_reset_control_get_exclusive_by_index(port_dev, 1);
			phy_reset = devm_reset_control_get_exclusive_by_index(port_dev, 2);
			phy_pow_reset = devm_reset_control_get_exclusive_by_index(port_dev, 3);
		} else
#endif
		{
			port_dev = &pdev->dev;
			sata_reset = of_reset_control_get_exclusive_by_index(child, 0);
			sata_func_reset = of_reset_control_get_exclusive_by_index(child, 1);
			phy_reset = of_reset_control_get_exclusive_by_index(child, 2);
			phy_pow_reset = of_reset_control_get_exclusive_by_index(child, 3);
		}

		if (sata_reset) {
			dev_info(port_dev, "resetting SATA for port %u", port);
			reset_control_deassert(sata_reset);
			if (!port_pdev)
				reset_control_put(sata_reset);
		}
		if (sata_func_reset) {
			dev_info(port_dev, "resetting SATA Func Exist for port %u", port);
			reset_control_deassert(sata_func_reset);
			if (!port_pdev)
				reset_control_put(sata_func_reset);
		}
		if (phy_reset) {
			dev_info(port_dev, "resetting PHY for port %u", port);
			reset_control_deassert(phy_reset);
			if (!port_pdev)
				reset_control_put(phy_reset);
		}

		rtd129x_ahci_mac_init(&pdev->dev, hpriv->mmio, port);
		rtd129x_ahci_phy_init(&pdev->dev, hpriv->mmio, port);

		if (phy_pow_reset) {
			dev_info(port_dev, "resetting PHY Pow for port %u", port);
			reset_control_deassert(phy_pow_reset);
			if (!port_pdev)
				reset_control_put(phy_pow_reset);
		}

		send_oob(ukbase, port);
	}

	rc = ahci_platform_init_host(pdev, hpriv, &rtd129x_ahci_port_info,
				     &rtd129x_ahci_scsi_host_template);
	if (rc)
		goto disable_resources;

	return 0;

disable_resources:
	ahci_platform_disable_resources(hpriv);
	return rc;
}

static struct platform_driver rtd129x_ahci_platform_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = rtd129x_ahci_dt_ids,
	},
	.probe = rtd129x_ahci_probe,
};
builtin_platform_driver(rtd129x_ahci_platform_driver);
