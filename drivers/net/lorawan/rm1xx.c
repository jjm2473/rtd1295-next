// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Laird RM186/RM191
 *
 * Copyright (c) 2019 Andreas Färber
 */

#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/lora.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/serdev.h>

struct rm1xx_device {
	struct serdev_device *serdev;
};

static int rm1xx_probe(struct serdev_device *sdev)
{
	struct rm1xx_device *rmdev;
	int ret;

	dev_info(&sdev->dev, "Probing");

	rmdev = devm_kzalloc(&sdev->dev, sizeof(*rmdev), GFP_KERNEL);
	if (!rmdev)
		return -ENOMEM;

	rmdev->serdev = sdev;
	serdev_device_set_drvdata(sdev, rmdev);

	ret = serdev_device_open(sdev);
	if (ret) {
		dev_err(&sdev->dev, "Failed to open (%d)\n", ret);
		return ret;
	}

	//serdev_device_set_baudrate(sdev, 115200);
	serdev_device_set_flow_control(sdev, false);
	//serdev_device_set_client_ops(sdev, &gmxlr1_serdev_client_ops);

	dev_info(&sdev->dev, "Done.\n");

	return 0;
}

static void rm1xx_remove(struct serdev_device *sdev)
{
	serdev_device_close(sdev);

	dev_info(&sdev->dev, "Removed\n");
}

static const struct of_device_id rm1xx_of_match[] = {
	{ .compatible = "laird,rm186" },
	{}
};
MODULE_DEVICE_TABLE(of, rm1xx_of_match);

static struct serdev_device_driver rm1xx_serdev_driver = {
	.probe = rm1xx_probe,
	.remove = rm1xx_remove,
	.driver = {
		.name = "rm1xx",
		.of_match_table = rm1xx_of_match,
	},
};
module_serdev_device_driver(rm1xx_serdev_driver);

MODULE_DESCRIPTION("Laird RM1xx serdev driver");
MODULE_AUTHOR("Andreas Färber <afaerber@suse.de>");
MODULE_LICENSE("GPL");
