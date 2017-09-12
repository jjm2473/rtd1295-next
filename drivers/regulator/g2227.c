/*
 * GMT G2227
 *
 * Copyright (c) 2017 Andreas Färber
 *
 * Authors:
 *   Simon Hsu
 *   Andeas Färber
 *
 * SPDX-License-Identifier: GPL-2.0+
 */
#include <linux/bitops.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define REG_PWRKEY		0x02
#define REG_SYS			0x04
#define REG_DCDC_LDO_ONOFF	0x05
#define REG_DCDC2_NRMVOLT	0x10
#define REG_DCDC3_NRMVOLT	0x11
#define REG_DCDC5_NRMVOLT	0x12
#define REG_DCDC1_6_NRMVOLT	0x13
#define REG_LDO_NRMVOLT		0x14
#define REG_VERSION		0x20

#define REG_SYS_LDOFF_TO_DO	BIT(7)

struct g2227_dev {
	struct i2c_client *client;
	struct regmap *regmap;
};

static const struct regmap_config g2227_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x30,
};

static const struct regulator_ops g2227_regulator_ops = {
	.list_voltage = regulator_list_voltage_table,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
};

static const unsigned int g2227_dcdc1_voltages[4] = {
	3000000, 3100000, 3200000, 3300000
};

static const unsigned int g2227_dcdc2_voltages[32] = {
	800000, 812500, 825000, 837500, 850000, 862500, 875000, 887500,
	900000, 912500, 925000, 937500, 950000, 962500, 975000, 987500,
	1000000, 1012500, 1025000, 1037500, 1050000, 1062500, 1075000, 1087500,
	1100000, 1112500, 1125000, 1137500, 1150000, 1162500, 1175000, 1187500
};

static const unsigned int g2227_ldo_voltages[16] = {
	800000, 850000, 900000, 950000, 1000000, 1100000, 1200000, 1300000,
	1500000, 1600000, 1800000, 1900000, 2500000, 2600000, 3000000, 3100000
};

static struct regulator_desc g2227_dcdc1_regulator_desc = {
	.name = "dcdc1",
	.id = 0,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.ops = &g2227_regulator_ops,
	.volt_table = g2227_dcdc1_voltages,
	.n_voltages = ARRAY_SIZE(g2227_dcdc1_voltages),
	.vsel_reg = REG_DCDC1_6_NRMVOLT,
	.vsel_mask = GENMASK(7, 6),
	.enable_reg = REG_DCDC_LDO_ONOFF,
	.enable_mask = BIT(7),
	.supply_name = "vin1",
};

static struct regulator_desc g2227_dcdc2_regulator_desc = {
	.name = "dcdc2",
	.id = 1,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.ops = &g2227_regulator_ops,
	.volt_table = g2227_dcdc2_voltages,
	.n_voltages = ARRAY_SIZE(g2227_dcdc2_voltages),
	.vsel_reg = REG_DCDC2_NRMVOLT,
	.vsel_mask = GENMASK(4, 0),
	.enable_reg = REG_DCDC_LDO_ONOFF,
	.enable_mask = BIT(6),
	.supply_name = "vin2",
};

static struct regulator_desc g2227_dcdc3_regulator_desc = {
	.name = "dcdc3",
	.id = 2,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.ops = &g2227_regulator_ops,
	.volt_table = g2227_dcdc2_voltages,
	.n_voltages = ARRAY_SIZE(g2227_dcdc2_voltages),
	.vsel_reg = REG_DCDC3_NRMVOLT,
	.vsel_mask = GENMASK(4, 0),
	.enable_reg = REG_DCDC_LDO_ONOFF,
	.enable_mask = BIT(5),
	.supply_name = "vin3",
};

static struct regulator_desc g2227_dcdc5_regulator_desc = {
	.name = "dcdc5",
	.id = 4,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.ops = &g2227_regulator_ops,
	.volt_table = g2227_dcdc2_voltages,
	.n_voltages = ARRAY_SIZE(g2227_dcdc2_voltages),
	.vsel_reg = REG_DCDC5_NRMVOLT,
	.vsel_mask = GENMASK(4, 0),
	.enable_reg = REG_DCDC_LDO_ONOFF,
	.enable_mask = BIT(3),
	.supply_name = "vin5",
};

static struct regulator_desc g2227_dcdc6_regulator_desc = {
	.name = "dcdc6",
	.id = 5,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.ops = &g2227_regulator_ops,
	.volt_table = g2227_dcdc2_voltages,
	.n_voltages = ARRAY_SIZE(g2227_dcdc2_voltages),
	.vsel_reg = REG_DCDC1_6_NRMVOLT,
	.vsel_mask = GENMASK(4, 0),
	.enable_reg = REG_DCDC_LDO_ONOFF,
	.enable_mask = BIT(2),
	.supply_name = "vin6",
};

static struct regulator_desc g2227_rtcldo_regulator_desc = {
	.name = "rtcldo",
	.id = 6,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.ops = &g2227_regulator_ops,
	.volt_table = g2227_dcdc1_voltages,
	.n_voltages = ARRAY_SIZE(g2227_dcdc1_voltages),
	.vsel_reg = REG_SYS,
	.vsel_mask = GENMASK(1, 0),
};

static struct regulator_desc g2227_ldo2_regulator_desc = {
	.name = "ldo2",
	.id = 7,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.ops = &g2227_regulator_ops,
	.volt_table = g2227_ldo_voltages,
	.n_voltages = ARRAY_SIZE(g2227_ldo_voltages),
	.vsel_reg = REG_LDO_NRMVOLT,
	.vsel_mask = GENMASK(7, 4),
	.enable_reg = REG_DCDC_LDO_ONOFF,
	.enable_mask = BIT(1),
	.supply_name = "ldoin23",
};

static struct regulator_desc g2227_ldo3_regulator_desc = {
	.name = "ldo3",
	.id = 8,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.ops = &g2227_regulator_ops,
	.volt_table = g2227_ldo_voltages,
	.n_voltages = ARRAY_SIZE(g2227_ldo_voltages),
	.vsel_reg = REG_LDO_NRMVOLT,
	.vsel_mask = GENMASK(3, 0),
	.enable_reg = REG_DCDC_LDO_ONOFF,
	.enable_mask = BIT(0),
	.supply_name = "ldoin23",
};

static struct of_regulator_match g2227_matches[] = {
	{
		.name = "dcdc1",
		.desc = &g2227_dcdc1_regulator_desc,
	}, {
		.name = "dcdc2",
		.desc = &g2227_dcdc2_regulator_desc,
	}, {
		.name = "dcdc3",
		.desc = &g2227_dcdc3_regulator_desc,
	}, {
		.name = "dcdc5",
		.desc = &g2227_dcdc5_regulator_desc,
	}, {
		.name = "dcdc6",
		.desc = &g2227_dcdc6_regulator_desc,
	}, {
		.name = "ldo1",
		.desc = &g2227_rtcldo_regulator_desc,
	}, {
		.name = "ldo2",
		.desc = &g2227_ldo2_regulator_desc,
	}, {
		.name = "ldo3",
		.desc = &g2227_ldo3_regulator_desc,
	},
};

static int g2227_probe(struct i2c_client *client)
{
	struct g2227_dev *data;
	struct of_regulator_match *match;
	struct regulator_config cfg = {};
	struct regulator_dev *rdev;
	unsigned int val;
	int ret, i;

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;

	data->regmap = devm_regmap_init_i2c(client, &g2227_regmap_config);
	if (IS_ERR(data->regmap)) {
		dev_err(&client->dev, "regmap init failed\n");
		return PTR_ERR(data->regmap);
	}

	if (regmap_read(data->regmap, REG_VERSION, &val) >= 0)
		dev_info(&client->dev, "version = %u\n", val & 0x7);

	ret = of_regulator_match(&client->dev, client->dev.of_node, g2227_matches, ARRAY_SIZE(g2227_matches));
	if (ret < 0) {
		dev_err(&client->dev, "regulator match failed\n");
		return ret;
	}

	cfg.dev = &client->dev;
	cfg.driver_data = data;
	cfg.regmap = data->regmap;

	for (i = 0; i < ARRAY_SIZE(g2227_matches); i++) {
		match = &g2227_matches[i];
		cfg.init_data = match->init_data;
		cfg.of_node = of_get_child_by_name(client->dev.of_node, match->name);

		rdev = devm_regulator_register(&client->dev, match->desc, &cfg);
		if (IS_ERR(rdev)) {
			dev_err(&client->dev, "%s register failed\n", match->name);
			return PTR_ERR(rdev);
		}
	}

	i2c_set_clientdata(client, data);

	ret = regmap_read(data->regmap, REG_PWRKEY, &val);
	if (ret < 0) {
		dev_err(&client->dev, "regmap read failed\n");
		return ret;
	}
	val &= ~(REG_SYS_LDOFF_TO_DO | 0x3);
	val |= 0x2;
	ret = regmap_write(data->regmap, REG_PWRKEY, val);
	if (ret < 0) {
		dev_err(&client->dev, "regmap write failed\n");
		return ret;
	}

	dev_info(&client->dev, "probed\n");

	return 0;
}

static const struct of_device_id g2227_dt_matches[] = {
	{ .compatible = "gmt,g2227" },
	{ }
};

static struct i2c_driver g2227_driver = {
	.driver = {
		.name = "gmt-g2227",
		.of_match_table = g2227_dt_matches,
	},
	.probe_new = g2227_probe,
};

module_i2c_driver(g2227_driver);
