// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Realtek Digital Home Center System-on-Chip info
 *
 * Copyright (c) 2017-2020 Andreas Färber
 * Copyright (c) 2019 Realtek Semiconductor Corp.
 */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/sys_soc.h>

#define REG_SB2_CHIP_ID		0x200
#define REG_SB2_CHIP_INFO	0x204

#define SB2_CHIP_ID_CHIP_ID		GENMASK(15, 0)

#define SB2_CHIP_INFO_REVISE_ID		GENMASK(31, 16)

#define REG_ISO_CHIP_INFO1	0x028

#define REG_EFUSE_PACKAGE_ID	0x1d8

struct dhc_soc_revision {
	const char *name;
	u16 chip_rev;
};

static const struct dhc_soc_revision rtd1195_revisions[] = {
	{ "A", 0x0000 },
	{ "B", 0x0001 },
	{ "C", 0x0002 },
	{ "D", 0x0003 },
	{ }
};

static const struct dhc_soc_revision rtd1295_revisions[] = {
	{ "A00", 0x0000 },
	{ "A01", 0x0001 },
	{ "B00", 0x0002 },
	{ "B01", 0x0003 },
	{ }
};

static const struct dhc_soc_revision rtd1395_revisions[] = {
	{ "A00", 0x0000 },
	{ "A01", 0x0001 },
	{ "A02", 0x0002 },
	{ }
};

static const struct dhc_soc_revision rtd1619_revisions[] = {
	{ "A00", 0x0000 },
	{ "A01", 0x0001 },
	{ }
};

static const struct dhc_soc_revision rtd1319_revisions[] = {
	{ "A00", 0x0000 },
	{ "B00", 0x0001 },
	{ "B01", 0x0002 },
	{ "B02", 0x0003 },
	{ }
};

struct dhc_soc {
	u16 chip_id;
	const char *family;
	const char *(*get_name)(struct device *dev, const struct dhc_soc *s);
	const struct dhc_soc_revision *revisions;
	const char *codename;
};

static const char *default_name(struct device *dev, const struct dhc_soc *s)
{
	return s->family;
}

static const char *rtd1295_name(struct device *dev, const struct dhc_soc *s)
{
	struct regmap *regmap;
	unsigned int val;
	int ret;
	u8 b;

	ret = nvmem_cell_read_u8(dev, "efuse_package_id", &b);
	if (ret == -EPROBE_DEFER)
		return ERR_PTR(ret);
	else if (ret)
		dev_warn(dev, "Could not read efuse package_id (%d)\n", ret);
	else if (b == 0x1)
		return "RTD1294";

	regmap = syscon_regmap_lookup_by_phandle(dev->of_node, "iso-syscon");
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		if (ret == -EPROBE_DEFER)
			return ERR_PTR(ret);
		dev_warn(dev, "Could not check iso (%d)\n", ret);
	} else {
		ret = regmap_read(regmap, REG_ISO_CHIP_INFO1, &val);
		if (ret)
			dev_warn(dev, "Could not read chip_info1 (%d)\n", ret);
		else if (val & BIT(11)) {
			if (val & BIT(4))
				return "RTD1293";
			return "RTD1296";
		}
	}

	return "RTD1295";
}

static const char *rtd1395_name(struct device *dev, const struct dhc_soc *s)
{
	struct regmap *regmap;
	unsigned int val;
	int ret;

	regmap = syscon_regmap_lookup_by_phandle(dev->of_node, "iso-syscon");
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		if (ret == -EPROBE_DEFER)
			return ERR_PTR(ret);
		dev_warn(dev, "Could not check iso (%d)\n", ret);
	} else {
		ret = regmap_read(regmap, REG_ISO_CHIP_INFO1, &val);
		if (ret)
			dev_warn(dev, "Could not read chip_info1 (%d)\n", ret);
		else if (val & BIT(12))
			return "RTD1392";
	}

	return "RTD1395";
}

static const struct dhc_soc dhc_soc_families[] = {
	{ 0x6329, "RTD1195", default_name, rtd1195_revisions, "Phoenix" },
	{ 0x6421, "RTD1295", rtd1295_name, rtd1295_revisions, "Kylin" },
	{ 0x6481, "RTD1395", rtd1395_name, rtd1395_revisions, "Hercules" },
	{ 0x6525, "RTD1619", default_name, rtd1619_revisions, "Thor" },
	{ 0x6570, "RTD1319", default_name, rtd1319_revisions, "Hank" },
};

static const struct dhc_soc *dhc_soc_by_chip_id(u16 chip_id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dhc_soc_families); i++) {
		const struct dhc_soc *family = &dhc_soc_families[i];

		if (family->chip_id == chip_id)
			return family;
	}
	return NULL;
}

static const char *dhc_soc_rev(const struct dhc_soc *family, u16 chip_rev)
{
	if (family) {
		const struct dhc_soc_revision *rev = family->revisions;

		while (rev && rev->name) {
			if (rev->chip_rev == chip_rev)
				return rev->name;
			rev++;
		}
	}
	return "unknown";
}

static int dhc_soc_probe(struct platform_device *pdev)
{
	const struct dhc_soc *s;
	struct soc_device_attribute *soc_dev_attr;
	struct soc_device *soc_dev;
	struct device_node *node;
	struct resource *res;
	struct regmap *regmap;
	u16 chip_id, chip_rev;
	unsigned int val;
	int ret;

	regmap = syscon_node_to_regmap(pdev->dev.of_node->parent);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!devm_request_mem_region(&pdev->dev, res->start, resource_size(res), dev_name(&pdev->dev)))
		return -EBUSY;

	ret = regmap_read(regmap, REG_SB2_CHIP_ID, &val);
	if (ret)
		return ret;
	chip_id = FIELD_GET(SB2_CHIP_ID_CHIP_ID, val);

	ret = regmap_read(regmap, REG_SB2_CHIP_INFO, &val);
	if (ret)
		return ret;
	chip_rev = FIELD_GET(SB2_CHIP_INFO_REVISE_ID, val);

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr)
		return -ENOMEM;

	node = of_find_node_by_path("/");
	of_property_read_string(node, "model", &soc_dev_attr->machine);
	of_node_put(node);

	s = dhc_soc_by_chip_id(chip_id);

	if (likely(s && s->get_name)) {
		soc_dev_attr->soc_id = s->get_name(&pdev->dev, s);
		if (IS_ERR(soc_dev_attr->soc_id))
			return PTR_ERR(soc_dev_attr->soc_id);
	} else
		soc_dev_attr->soc_id = "unknown";

	soc_dev_attr->revision = dhc_soc_rev(s, chip_rev);

	soc_dev_attr->family = kasprintf(GFP_KERNEL, "Realtek %s",
		(s && s->codename) ? s->codename :
		((s && s->family) ? s->family : "Digital Home Center"));

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR(soc_dev)) {
		kfree(soc_dev_attr->family);
		kfree(soc_dev_attr);
		return PTR_ERR(soc_dev);
	}

	platform_set_drvdata(pdev, soc_dev);

	pr_info("%s %s (0x%04x) rev %s (0x%04x) detected\n",
		soc_dev_attr->family, soc_dev_attr->soc_id, (u32)chip_id,
		soc_dev_attr->revision, (u32)chip_rev);

	return 0;
}

static int dhc_soc_remove(struct platform_device *pdev)
{
	struct soc_device *soc_dev = platform_get_drvdata(pdev);

	soc_device_unregister(soc_dev);

	return 0;
}

static const struct of_device_id dhc_soc_dt_ids[] = {
	 { .compatible = "realtek,rtd1195-chip" },
	 { }
};

static struct platform_driver dhc_soc_driver = {
	.probe = dhc_soc_probe,
	.remove = dhc_soc_remove,
	.driver = {
		.name = "dhc-soc",
		.of_match_table	= dhc_soc_dt_ids,
	},
};
module_platform_driver(dhc_soc_driver);

MODULE_DESCRIPTION("Realtek DHC SoC identification driver");
MODULE_LICENSE("GPL");
