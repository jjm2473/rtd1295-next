#include <linux/ahci_platform.h>
#include <linux/bitfield.h>
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

#define REG_SATA_CTR	0xf20
#define REG_MDIO_CTR	0xf60
#define REG_MDIO_CTR1	0xf64
#define REG_SPD		0xf68

#define SATA_CTR_CKBUF_EN0			BIT(8)
#define SATA_CTR_CKBUF_EN1			BIT(9)
#define SATA_CTR_RX_ERROR_SEL			GENMASK(15, 14)
#define SATA_CTR_RX_ERROR_SEL_MAC_ORIG		(0x3 << 14)
#define SATA_CTR_RX_ERROR_CDR_SEL		GENMASK(17, 16)
#define SATA_CTR_RX_ERROR_CDR_SEL_MAC_ORIG	(0x3 << 16)

#define MDIO_CTR_MDIO_RDWR_WRITE	BIT(0)
#define MDIO_CTR_MDIO_SRST		BIT(1)
#define MDIO_CTR_MCLK_RATE_MASK		GENMASK(3, 2)
#define MDIO_CTR_MDIO_RDY		BIT(4)
#define MDIO_CTR_MDIO_ST_MASK		GENMASK(6, 5)
#define MDIO_CTR_MDIO_BUSY		BIT(7)
#define MDIO_CTR_PHY_REG_ADDR_MASK	GENMASK(13, 8)
#define MDIO_CTR_PHY_ADDR_MASK		GENMASK(15, 14)
#define MDIO_CTR_DATA_MASK		GENMASK(31, 16)

#define MDIO_CTR1_PHY_SEL_MASK	BIT(0)

static void writel_delay(unsigned int value, void __iomem *address)
{
	writel(value, address);
	mdelay(1);
}

static int rtd1295_ahci_mdio_wait_busy(void __iomem *base, u32 *pval)
{
	u32 val;
	int i = 0;

	val = readl_relaxed(base + REG_MDIO_CTR);
	while ((val & MDIO_CTR_MDIO_BUSY) && i < 10) {
		mdelay(1);
		i++;
		val = readl_relaxed(base + REG_MDIO_CTR);
	}

	if (val & MDIO_CTR_MDIO_BUSY)
		return -EBUSY;

	if (pval)
		*pval = val;

	return 0;
}

static int rtd1295_ahci_mdio_phy_select(void __iomem *base, unsigned int port)
{
	u32 val;
	int ret;

	if (port > 1)
		return -EINVAL;

	ret = rtd1295_ahci_mdio_wait_busy(base, NULL);
	if (ret)
		return ret;

	val = readl_relaxed(base + REG_MDIO_CTR1);
	val &= ~MDIO_CTR1_PHY_SEL_MASK;
	val |= FIELD_PREP(MDIO_CTR1_PHY_SEL_MASK, port);
	writel_relaxed(val, base + REG_MDIO_CTR1);

	return 0;
}

enum rtd1295_ahci_mdio_phy_addr {
	MDIO_CTR_PHY_ADDR_SATA1_0	= 0,
	MDIO_CTR_PHY_ADDR_SATA2_0	= 1,
	MDIO_CTR_PHY_ADDR_SATA3_0	= 2,
	MDIO_CTR_PHY_ADDR_SGMII		= 3,
};

static int rtd1295_ahci_mdio_phy_write(void __iomem *base,
	enum rtd1295_ahci_mdio_phy_addr addr, u8 reg, u16 data)
{
	u32 val;
	int ret;

	ret = rtd1295_ahci_mdio_wait_busy(base, &val);
	if (ret)
		return ret;

	val = (val & MDIO_CTR_MCLK_RATE_MASK) | (val & MDIO_CTR_MDIO_SRST);
	val |= FIELD_PREP(MDIO_CTR_DATA_MASK, data) |
		FIELD_PREP(MDIO_CTR_PHY_ADDR_MASK, addr) |
		FIELD_PREP(MDIO_CTR_PHY_REG_ADDR_MASK, reg) |
		MDIO_CTR_MDIO_RDWR_WRITE;
	//pr_info("%s: 0x%08x\n", __func__, val);
	writel_relaxed(val, base + REG_MDIO_CTR);

	return 0;
}

static inline int rtd1295_ahci_mdio_sata1_phy_write(void __iomem *base,
	u8 reg, u16 data)
{
	return rtd1295_ahci_mdio_phy_write(base, MDIO_CTR_PHY_ADDR_SATA1_0,
		reg, data);
}

static inline int rtd1295_ahci_mdio_sata2_phy_write(void __iomem *base,
	u8 reg, u16 data)
{
	return rtd1295_ahci_mdio_phy_write(base, MDIO_CTR_PHY_ADDR_SATA2_0,
		reg, data);
}

static inline int rtd1295_ahci_mdio_sata3_phy_write(void __iomem *base,
	u8 reg, u16 data)
{
	return rtd1295_ahci_mdio_phy_write(base, MDIO_CTR_PHY_ADDR_SATA3_0,
		reg, data);
}

static int rtd1295_ahci_mdio_sata_phy_write(void __iomem *base,
	u8 reg, u16 data)
{
	int ret;

	ret = rtd1295_ahci_mdio_sata1_phy_write(base, reg, data);
	if (ret)
		return ret;

	ret = rtd1295_ahci_mdio_sata2_phy_write(base, reg, data);
	if (ret)
		return ret;

	return rtd1295_ahci_mdio_sata3_phy_write(base, reg, data);
}

#define REG_PHY_ANA04		0x04
#define REG_PHY_ANA08		0x08
#define REG_PHY_BACR		0x11
#define REG_PHY_ANA27		0x27

static void rtd1295_ahci_phy_init(struct device *dev, void __iomem *base, int port)
{
	int ret;

	ret = rtd1295_ahci_mdio_phy_select(base, port);
	if (ret)
		return;

	rtd1295_ahci_mdio_sata_phy_write(base, REG_PHY_BACR, 0x0);

#if 0
	rtd1295_ahci_mdio_sata_phy_write(base, REG_PHY_ANA04, 0x538E);

	rtd1295_ahci_mdio_sata_phy_write(base, 0x05, 0x3b6a);

	rtd1295_ahci_mdio_sata1_phy_write(base, 0x01, 0xE050);
	rtd1295_ahci_mdio_sata2_phy_write(base, 0x01, 0xE050);
	rtd1295_ahci_mdio_sata3_phy_write(base, 0x01, 0xE04C);

	rtd1295_ahci_mdio_sata_phy_write(base, 0x06, 0x0011);

	rtd1295_ahci_mdio_sata_phy_write(base, 0x0a, 0xA600);

	rtd1295_ahci_mdio_sata3_phy_write(base, 0x02, 0x27FD);
	rtd1295_ahci_mdio_sata3_phy_write(base, 0x0a, 0xA640);
	rtd1295_ahci_mdio_sata3_phy_write(base, 0x26, 0x041B);
#else
	if (true) {
		dev_info(dev, "disabling spread-spectrum\n");
		rtd1295_ahci_mdio_sata_phy_write(base, REG_PHY_ANA04, 0x538E);
	} else {
		dev_info(dev, "enabling spread-spectrum\n");
		rtd1295_ahci_mdio_sata_phy_write(base, REG_PHY_ANA04, 0x738E);
		rtd1295_ahci_mdio_sata_phy_write(base, REG_PHY_ANA08, 0x3591);
		rtd1295_ahci_mdio_sata_phy_write(base, REG_PHY_ANA27, 0x0234);
	}
	rtd1295_ahci_mdio_sata_phy_write(base, 0x05, 0x336a);

	rtd1295_ahci_mdio_sata1_phy_write(base, 0x01, 0xE070);
	rtd1295_ahci_mdio_sata2_phy_write(base, 0x01, 0xE05C);
	rtd1295_ahci_mdio_sata3_phy_write(base, 0x01, 0xE04A);

	rtd1295_ahci_mdio_sata_phy_write(base, 0x06, 0x0015);

	rtd1295_ahci_mdio_sata_phy_write(base, 0x0a, 0xC600);

	rtd1295_ahci_mdio_sata_phy_write(base, 0x02, 0x7000);

	rtd1295_ahci_mdio_sata_phy_write(base, 0x0a, 0xC660);

	rtd1295_ahci_mdio_sata_phy_write(base, 0x19, 0x2004);

	rtd1295_ahci_mdio_sata_phy_write(base, 0x20, 0x94aa);
#endif

	rtd1295_ahci_mdio_sata_phy_write(base, 0x15, 0x1717);

	rtd1295_ahci_mdio_sata_phy_write(base, 0x16, 0x0770);

// for rx sensitivity
	rtd1295_ahci_mdio_sata_phy_write(base, 0x09, 0x7210);
/*	if(ahci_dev->port[port]->phy_status==0) {
		rtd1295_ahci_mdio_sata_phy_write(base, 0x03, 0x2764);
	} else if(ahci_dev->port[port]->phy_status==2) {
		rtd1295_ahci_mdio_sata_phy_write(base, 0x03, 0x2771);
	}*/
	rtd1295_ahci_mdio_sata1_phy_write(base, 0x03, 0x2771);
	rtd1295_ahci_mdio_sata2_phy_write(base, 0x03, 0x2768);
	rtd1295_ahci_mdio_sata3_phy_write(base, 0x03, 0x2768);

	rtd1295_ahci_mdio_sata_phy_write(base, 0x10, 0x2900);

	if (false) {
		printk("[SATA] set tx-driving to L (level 2)\n");
		rtd1295_ahci_mdio_sata_phy_write(base, 0x20, 0x94a7);
		rtd1295_ahci_mdio_sata_phy_write(base, 0x21, 0x587a);
	} else if (of_machine_is_compatible("synology,ds418j")) { // for DS418j
		printk("[SATA] set tx-driving to L (level 8)\n");
		if(port==0) {
			rtd1295_ahci_mdio_sata_phy_write(base, 0x20, 0x94a8);
			rtd1295_ahci_mdio_sata_phy_write(base, 0x21, 0x588a);
		} else if(port==1) {
			rtd1295_ahci_mdio_sata_phy_write(base, 0x20, 0x94a8);
			rtd1295_ahci_mdio_sata_phy_write(base, 0x21, 0x58da);
		}
	} else if (of_machine_is_compatible("synology,ds418")) { // for DS418
		printk("[SATA] set tx-driving to L (level 6)\n");
		if(port==0) {
			rtd1295_ahci_mdio_sata_phy_write(base, 0x20, 0x94aa);
			rtd1295_ahci_mdio_sata_phy_write(base, 0x21, 0xa86a);
		} else if(port==1) {
			rtd1295_ahci_mdio_sata_phy_write(base, 0x20, 0x94a4);
			rtd1295_ahci_mdio_sata_phy_write(base, 0x21, 0x68ca);
		}
	} else if (false) { // for DS218play
		printk("[SATA] set tx-driving to L (level 4)\n");
		if(port==0) {
			rtd1295_ahci_mdio_sata_phy_write(base, 0x20, 0x94a7);
			rtd1295_ahci_mdio_sata_phy_write(base, 0x21, 0x587a);
		} else if(port==1) {
			rtd1295_ahci_mdio_sata_phy_write(base, 0x20, 0x94a7);
			rtd1295_ahci_mdio_sata_phy_write(base, 0x21, 0x587a);
		}
	} else if (false) { // for DS118
		printk("[SATA] set tx-driving to L (level 10)\n");
		if(port==0) {
			rtd1295_ahci_mdio_sata_phy_write(base, 0x20, 0x94a7);
			rtd1295_ahci_mdio_sata_phy_write(base, 0x21, 0x383a);
		}
	}
	// RX power saving off
	rtd1295_ahci_mdio_sata_phy_write(base, 0x0c, 0x4000);

	rtd1295_ahci_mdio_sata_phy_write(base, 0x17, 0x0027);
}

static void rtd1295_ahci_mac_init(struct device *dev, void __iomem *base, int port)
{
	void __iomem *port_base = base + port * 0x80;
	u32 val;
	int ret;

	ret = rtd1295_ahci_mdio_phy_select(base, port);
	if (ret)
		return;

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

	val = SATA_CTR_RX_ERROR_CDR_SEL_MAC_ORIG |
		SATA_CTR_RX_ERROR_SEL_MAC_ORIG |
		SATA_CTR_CKBUF_EN1 | SATA_CTR_CKBUF_EN0;
	writel_delay(val, base + REG_SATA_CTR);

	writel_delay(0x700, base + 0xA4);
	//Set to Auto mode
	if (true)
		writel_delay(0xA, base + REG_SPD);
	else if (false)
		writel_delay(0x5, base + REG_SPD);
	else if (false)
		writel_delay(0x0, base + REG_SPD);
}

#define REG_SB2_SATA_PHY_CTRL	0x80

#define SB2_SATA_PHY_CTRL_SATA_BG_EN_0		BIT(0)
#define SB2_SATA_PHY_CTRL_SATA_BG_EN_1		BIT(1)
#define SB2_SATA_PHY_CTRL_SATA_MBIAS_EN_0	BIT(2)
#define SB2_SATA_PHY_CTRL_SATA_MBIAS_EN_1	BIT(3)
#define SB2_SATA_PHY_CTRL_SATA_RX50_LINK_0	BIT(4)
#define SB2_SATA_PHY_CTRL_SATA_RX50_LINK_1	BIT(5)
#define SB2_SATA_PHY_CTRL_ISOLATE_SATA_SGMII_0	BIT(6)
#define SB2_SATA_PHY_CTRL_ISOLATE_SATA_SGMII_1	BIT(7)
#define SB2_SATA_PHY_CTRL_SATA_SGMII_SEL	BIT(8)

static int send_oob(void __iomem *ukbase, unsigned int port)
{
	u32 val = 0;

	if (port == 0) {
		val = readl(ukbase + REG_SB2_SATA_PHY_CTRL);
		val |= SB2_SATA_PHY_CTRL_SATA_SGMII_SEL |
			SB2_SATA_PHY_CTRL_SATA_RX50_LINK_0 |
			SB2_SATA_PHY_CTRL_SATA_MBIAS_EN_0 |
			SB2_SATA_PHY_CTRL_SATA_BG_EN_0;
	} else if (port == 1) {
		val = readl(ukbase + REG_SB2_SATA_PHY_CTRL);
		val |= SB2_SATA_PHY_CTRL_SATA_SGMII_SEL |
			SB2_SATA_PHY_CTRL_SATA_RX50_LINK_1 |
			SB2_SATA_PHY_CTRL_SATA_MBIAS_EN_1 |
			SB2_SATA_PHY_CTRL_SATA_BG_EN_1;
	}
	writel(val, ukbase + REG_SB2_SATA_PHY_CTRL);

	return 0;
}

static const struct ata_port_info rtd1295_ahci_port_info = {
	.flags		= AHCI_FLAG_COMMON | ATA_FLAG_EM | ATA_FLAG_SW_ACTIVITY,
	.pio_mask	= ATA_PIO4,
	.udma_mask	= ATA_UDMA6,
	.port_ops	= &ahci_platform_ops,
};

static struct scsi_host_template rtd1295_ahci_scsi_host_template = {
	AHCI_SHT(DRV_NAME),
};

static const struct of_device_id rtd1295_ahci_dt_ids[] = {
	{ .compatible = "realtek,rtd1295-ahci" },
	{ }
};

static int rtd1295_ahci_probe(struct platform_device *pdev)
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

		rtd1295_ahci_mac_init(&pdev->dev, hpriv->mmio, port);
		rtd1295_ahci_phy_init(&pdev->dev, hpriv->mmio, port);

		if (phy_pow_reset) {
			dev_info(port_dev, "resetting PHY Pow for port %u", port);
			reset_control_deassert(phy_pow_reset);
			if (!port_pdev)
				reset_control_put(phy_pow_reset);
		}

		send_oob(ukbase, port);
	}

	rc = ahci_platform_init_host(pdev, hpriv, &rtd1295_ahci_port_info,
				     &rtd1295_ahci_scsi_host_template);
	if (rc)
		goto disable_resources;

	return 0;

disable_resources:
	ahci_platform_disable_resources(hpriv);
	return rc;
}

static struct platform_driver rtd1295_ahci_platform_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = rtd1295_ahci_dt_ids,
	},
	.probe = rtd1295_ahci_probe,
};
builtin_platform_driver(rtd1295_ahci_platform_driver);
