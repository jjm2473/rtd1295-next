// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Z-Wave
 *
 * Copyright (c) 2019 Andreas Färber
 */

#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/rculist.h>
#include <linux/serdev.h>

struct zwave_msg_dispatcher {
	struct list_head list;
	u8 id;
	void (*dispatch)(const u8 *data, u8 len, struct zwave_msg_dispatcher *d);
};

struct zwave_device {
	struct serdev_device *serdev;

	struct completion ack_comp;
	struct list_head msg_dispatchers;

	struct zwave_msg_dispatcher node_list_disp;
};

static void zwave_add_dispatcher(struct zwave_device *zdev,
	struct zwave_msg_dispatcher *entry)
{
	list_add_tail_rcu(&entry->list, &zdev->msg_dispatchers);
}

static void zwave_remove_dispatcher(struct zwave_device *zdev,
	struct zwave_msg_dispatcher *entry)
{
	list_del_rcu(&entry->list);
}

static u8 zwave_msg_checksum_first(const u8 first, const u8 *data, int len)
{
	u8 chksum;
	int i;

	chksum = first;
	for (i = 0; i < len; i++) {
		chksum ^= data[i];
	}
	return ~chksum;
}

static u8 zwave_msg_checksum(const u8 *data, int len)
{
	return zwave_msg_checksum_first(data[0], data + 1, len - 1);
}

static int zwave_send_msg(struct zwave_device *zdev, const u8 *data, int data_len,
	unsigned long timeout)
{
	u8 chksum;
	u8 buf[2];
	int ret;

	reinit_completion(&zdev->ack_comp);

	buf[0] = 0x01;
	buf[1] = data_len + 1;

	chksum = zwave_msg_checksum_first(buf[1], data, data_len);
	dev_dbg(&zdev->serdev->dev, "checksum: 0x%02x\n", (unsigned int)chksum);

	ret = serdev_device_write(zdev->serdev, buf, 2, timeout);
	if (ret < 0)
		return ret;
	if (ret > 0 && ret < 2)
		return -EIO;

	ret = serdev_device_write(zdev->serdev, data, data_len, timeout);
	if (ret < 0)
		return ret;
	if (ret > 0 && ret < data_len)
		return -EIO;

	ret = serdev_device_write(zdev->serdev, &chksum, 1, timeout);
	if (ret < 0)
		return ret;

	timeout = wait_for_completion_timeout(&zdev->ack_comp, timeout);
	if (!timeout)
		return -ETIMEDOUT;

	return 0;
}

static void zwave_node_list_report(const u8 *data, u8 len,
	struct zwave_msg_dispatcher *d)
{
	struct zwave_device *zdev = container_of(d, struct zwave_device, node_list_disp);
	struct device *dev = &zdev->serdev->dev;
	int i, j;

	if (len != 36) {
		dev_err(dev, "node list report unexpected length (%u)\n", (unsigned int)len);
		return;
	};

	dev_info(dev, "node list report\n");
	for (i = 0; i < data[4]; i++) {
		u8 tmp = data[5 + i];
		for (j = 0; j < 8; j++) {
			if (tmp & BIT(j))
				dev_info(dev, "node list data %u: node id %u\n", i + 1, i * 8 + j + 1);
		}
	}
}

static int zwave_receive_buf(struct serdev_device *sdev, const u8 *data, size_t count)
{
	struct zwave_device *zdev = serdev_device_get_drvdata(sdev);
	struct zwave_msg_dispatcher *e;
	u8 buf[1] = { 0x06 };
	u8 msglen;
	u8 chksum;

	dev_dbg(&sdev->dev, "Receive (%zu)\n", count);

	if (data[0] == 0x06) {
		dev_info(&sdev->dev, "ACK received\n");
		complete(&zdev->ack_comp);
		return 1;
	} else {
		if (count < 2)
			return 0;
		msglen = data[1];
		if (count < 2 + msglen) {
			dev_dbg(&sdev->dev, "%s: received %zu, expecting %u\n", __func__, count, 2 + (unsigned int)msglen);
			return 0;
		}
		print_hex_dump_bytes("received: ", DUMP_PREFIX_OFFSET, data, 2 + msglen);
		if (msglen > 0) {
			chksum = zwave_msg_checksum(data + 1, msglen);
			if (data[1 + msglen] == chksum) {
				dev_info(&sdev->dev, "sending ACK\n");
				serdev_device_write_buf(sdev, buf, 1);
			} else {
				dev_warn(&sdev->dev, "checksum mismatch\n");
				return 2 + msglen;
			}
		}
		list_for_each_entry(e, &zdev->msg_dispatchers, list) {
			if (msglen > 2 && e->id == data[3])
				e->dispatch(data + 2, msglen ? msglen - 1 : 0, e);
		}
		return 2 + msglen;
	}
}

static const struct serdev_device_ops zwave_serdev_client_ops = {
	.receive_buf = zwave_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

static int zwave_probe(struct serdev_device *sdev)
{
	struct zwave_device *zdev;
	int ret;
	const u8 msg[] = { 0x00, 0x02 };
	//const u8 msg[] = { 0x00, 0x41, 1 };

	dev_dbg(&sdev->dev, "Probing");

	zdev = devm_kzalloc(&sdev->dev, sizeof(*zdev), GFP_KERNEL);
	if (!zdev)
		return -ENOMEM;

	zdev->serdev = sdev;
	init_completion(&zdev->ack_comp);
	INIT_LIST_HEAD(&zdev->msg_dispatchers);
	serdev_device_set_drvdata(sdev, zdev);

	ret = serdev_device_open(sdev);
	if (ret) {
		dev_err(&sdev->dev, "Failed to open (%d)\n", ret);
		return ret;
	}

	serdev_device_set_baudrate(sdev, 115200);
	serdev_device_set_flow_control(sdev, false);
	serdev_device_set_client_ops(sdev, &zwave_serdev_client_ops);

	zdev->node_list_disp.id = 0x02;
	zdev->node_list_disp.dispatch = zwave_node_list_report;
	zwave_add_dispatcher(zdev, &zdev->node_list_disp);

	ret = zwave_send_msg(zdev, msg, sizeof(msg), HZ);
	if (ret) {
		dev_warn(&sdev->dev, "Failed to send (%d)\n", ret);
	}

	dev_dbg(&sdev->dev, "Done.\n");

	return 0;
}

static void zwave_remove(struct serdev_device *sdev)
{
	struct zwave_device *zdev = serdev_device_get_drvdata(sdev);

	serdev_device_close(sdev);

	zwave_remove_dispatcher(zdev, &zdev->node_list_disp);

	dev_dbg(&sdev->dev, "Removed\n");
}

static const struct of_device_id zwave_of_match[] = {
	{ .compatible = "zwave,zwave" }, /* XXX */
	{}
};
MODULE_DEVICE_TABLE(of, zwave_of_match);

static struct serdev_device_driver zwave_serdev_driver = {
	.probe = zwave_probe,
	.remove = zwave_remove,
	.driver = {
		.name = "zwave",
		.of_match_table = zwave_of_match,
	},
};
module_serdev_device_driver(zwave_serdev_driver);

MODULE_DESCRIPTION("Z-Wave serdev driver");
MODULE_AUTHOR("Andreas Färber <afaerber@suse.de>");
MODULE_LICENSE("GPL");
