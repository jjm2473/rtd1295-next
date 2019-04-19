// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Gimasi GMX-LR1
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

struct gmx_device {
	struct serdev_device *serdev;
};

static int gmxlr1_probe(struct serdev_device *sdev)
{
	struct gmx_device *gdev;
	int ret;

	dev_info(&sdev->dev, "Probing");

	gdev = devm_kzalloc(&sdev->dev, sizeof(*gdev), GFP_KERNEL);
	if (!gdev)
		return -ENOMEM;

	gdev->serdev = sdev;
	serdev_device_set_drvdata(sdev, gdev);

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

static void gmxlr1_remove(struct serdev_device *sdev)
{
	serdev_device_close(sdev);

	dev_info(&sdev->dev, "Removed\n");
}

static const struct of_device_id gmxlr1_of_match[] = {
	{ .compatible = "gimasi,gmx-lr1" },
	{}
};
MODULE_DEVICE_TABLE(of, gmxlr1_of_match);

static struct serdev_device_driver gmxlr1_serdev_driver = {
	.probe = gmxlr1_probe,
	.remove = gmxlr1_remove,
	.driver = {
		.name = "gmx-lr1",
		.of_match_table = gmxlr1_of_match,
	},
};
module_serdev_device_driver(gmxlr1_serdev_driver);

MODULE_DESCRIPTION("Gimasi GMX-LR1 serdev driver");
MODULE_AUTHOR("Andreas Färber <afaerber@suse.de>");
MODULE_LICENSE("GPL");
