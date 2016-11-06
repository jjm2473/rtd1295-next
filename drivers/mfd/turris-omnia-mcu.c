#include <linux/i2c.h>
#include <linux/module.h>

static int turris_omnia_mcu_probe(struct i2c_client *i2c,
	const struct i2c_device_id *id)
{
	int ret;

	ret = i2c_smbus_read_byte_data(i2c, 0x9);
	dev_info(&i2c->dev, "CMD_GET_RESET = %d", ret);

	ret = i2c_smbus_read_byte_data(i2c, 0xd);
	dev_info(&i2c->dev, "CMD_WATCHDOG_STATE = %d", ret);

	return 0;
}

static const struct i2c_device_id turris_omnia_mcu_ids[] = {
	{ "turris-omnia-mcu", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, turris_omnia_mcu_ids);

static const struct of_device_id turris_omnia_mcu_of_matches[] = {
	{ .compatible = "cznic,turris-omnia-mcu" },
	{}
};
MODULE_DEVICE_TABLE(of, turris_omnia_mcu_of_matches);

static struct i2c_driver turris_omnia_i2c_driver = {
	.driver = {
		.name = "turris-omnia-mcu",
		.of_match_table = of_match_ptr(turris_omnia_mcu_of_matches),
	},
	.probe = turris_omnia_mcu_probe,
	.id_table = turris_omnia_mcu_ids,
};

module_i2c_driver(turris_omnia_i2c_driver);

MODULE_AUTHOR("Andreas Färber");
MODULE_DESCRIPTION("Turris Omnia MCU I2C driver");
MODULE_LICENSE("GPL");
