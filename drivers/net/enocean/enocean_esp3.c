// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * EnOcean Serial Protocol 3
 *
 * Copyright (c) 2019 Andreas Färber
 */

#include <asm/unaligned.h>
#include <linux/completion.h>
#include <linux/crc8.h>
#include <linux/rculist.h>
#include <linux/serdev.h>
#include <linux/slab.h>

#include "enocean_esp.h"

/* G(x) = x^8 + x^2 + x^1 + x^0 */
#define ESP3_CRC8_POLY_MSB 0x07

DECLARE_CRC8_TABLE(enocean_esp3_crc8_table);

void enocean_esp3_crc8_populate(void)
{
	crc8_populate_msb(enocean_esp3_crc8_table, ESP3_CRC8_POLY_MSB);
}

static inline u8 enocean_esp3_crc8(u8 *pdata, size_t nbytes)
{
	return crc8(enocean_esp3_crc8_table, pdata, nbytes, 0x00);
}

#define ESP3_SYNC_WORD 0x55

struct enocean_esp3_dispatcher {
	struct list_head list;
	u8 packet_type;
	void (*dispatch)(const u8 *data, u16 data_len, struct enocean_esp3_dispatcher *d);
};

static void enocean_add_esp3_dispatcher(struct enocean_device *edev,
	struct enocean_esp3_dispatcher *entry)
{
	list_add_tail_rcu(&entry->list, &edev->esp_dispatchers);
}

static void enocean_remove_esp3_dispatcher(struct enocean_device *edev,
	struct enocean_esp3_dispatcher *entry)
{
	list_del_rcu(&entry->list);
}

struct enocean_esp3_response {
	struct enocean_esp3_dispatcher disp;

	u8 code;
	void *data;
	u16 data_len;

	struct completion comp;
};

static void enocean_esp3_response_dispatch(const u8 *data, u16 data_len,
	struct enocean_esp3_dispatcher *d)
{
	struct enocean_esp3_response *resp =
		container_of(d, struct enocean_esp3_response, disp);

	if (completion_done(&resp->comp))
		return;

	if (data_len < 1)
		return;

	resp->code = data[0];
	if (data_len > 1) {
		resp->data = kzalloc(data_len - 1, GFP_KERNEL);
		if (resp->data)
			memcpy(resp->data, data + 1, data_len - 1);
		resp->data_len = data_len - 1;
	} else {
		resp->data = NULL;
		resp->data_len = 0;
	}

	complete(&resp->comp);
}

struct enocean_esp3_priv {
	struct enocean_device *edev;
	struct enocean_esp3_dispatcher radio_erp1_response;
};

static inline int enocean_esp3_packet_size(u16 data_len, u8 optional_len)
{
	return 1 + 4 + 1 + data_len + optional_len + 1;
}

static int enocean_send_esp3_packet(struct enocean_device *edev, u8 packet_type,
	const void *data, u16 data_len, const void *optional_data, u8 optional_len,
	unsigned long timeout)
{
	int len = enocean_esp3_packet_size(data_len, optional_len);
	u8 *buf;
	int ret;

	buf = kzalloc(len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf[0] = ESP3_SYNC_WORD;
	put_unaligned_be16(data_len, buf + 1);
	buf[3] = optional_len;
	buf[4] = packet_type;
	buf[5] = enocean_esp3_crc8(buf + 1, 4);
	dev_dbg(&edev->serdev->dev, "CRC8H = %02x\n", (unsigned int)buf[5]);
	memcpy(buf + 6, data, data_len);
	memcpy(buf + 6 + data_len, optional_data, optional_len);
	buf[6 + data_len + optional_len] = enocean_esp3_crc8(buf + 6, data_len + optional_len);
	dev_dbg(&edev->serdev->dev, "CRC8D = %02x\n", (unsigned int)buf[6 + data_len + optional_len]);

	ret = serdev_device_write(edev->serdev, buf, len, timeout);

	kfree(buf);

	if (ret < 0)
		return ret;
	if (ret > 0 && ret < len)
		return -EIO;
	return 0;
}

#define ESP3_RADIO_ERP1		0x1
#define ESP3_RESPONSE		0x2
#define ESP3_COMMON_COMMAND	0x5

static void enocean_esp3_radio_erp1_response_dispatch(const u8 *data, u16 data_len,
	struct enocean_esp3_dispatcher *d)
{
	struct enocean_esp3_priv *priv = container_of(d, struct enocean_esp3_priv, radio_erp1_response);
	struct enocean_device *edev = priv->edev;
	int ret;

	enocean_remove_esp3_dispatcher(edev, d);

	if (data_len < 1)
		return;

	switch (data[0]) {
	case 0:
		enocean_esp_tx_done(edev);
		break;
	case 2:
		ret = -ENOTSUPP;
		break;
	case 3:
		ret = -EINVAL;
		break;
	case 5:
		ret = -EIO;
		break;
	default:
		ret = -EIO;
		break;
	}
}

static int enocean_esp3_send_radio_erp1(struct enocean_device *edev, u32 dest,
	const u8 *data, int data_len, unsigned long timeout)
{
	struct enocean_esp3_priv *priv = edev->priv;
	struct esp3_radio_erp1_optional {
		u8 sub_tel_num;
		__be32 destination_id;
		u8 dbm;
		u8 security_level;
	} __packed opt = {
		.sub_tel_num = 3,
		.destination_id = cpu_to_be32(dest),
		.dbm = 0xff,
		.security_level = 0,
	};
	int ret;

	enocean_add_esp3_dispatcher(edev, &priv->radio_erp1_response);

	ret = enocean_send_esp3_packet(edev, ESP3_RADIO_ERP1, data, data_len, &opt, sizeof(opt), timeout);
	if (ret) {
		enocean_remove_esp3_dispatcher(edev, &priv->radio_erp1_response);
		return ret;
	}

	return 0;
}

static int enocean_esp3_reset(struct enocean_device *edev, unsigned long timeout)
{
	struct enocean_esp3_response resp;
	const u8 buf[1] = { 0x02 };
	int ret;

	init_completion(&resp.comp);
	resp.disp.packet_type = ESP3_RESPONSE;
	resp.disp.dispatch = enocean_esp3_response_dispatch;
	enocean_add_esp3_dispatcher(edev, &resp.disp);

	ret = enocean_send_esp3_packet(edev, ESP3_COMMON_COMMAND, buf, sizeof(buf), NULL, 0, timeout);
	if (ret) {
		enocean_remove_esp3_dispatcher(edev, &resp.disp);
		return ret;
	}

	timeout = wait_for_completion_timeout(&resp.comp, timeout);
	enocean_remove_esp3_dispatcher(edev, &resp.disp);
	if (!timeout)
		return -ETIMEDOUT;

	switch (resp.code) {
	case 0:
		return 0;
	case 1:
		return -EIO;
	case 2:
		return -ENOTSUPP;
	default:
		return -EIO;
	}
}

struct enocean_esp3_version {
	u8 app_version[4];
	u8 api_version[4];
	__be32 chip_id;
	__be32 chip_version;
	char app_desc[16];
} __packed;

static int enocean_esp3_read_version(struct enocean_device *edev, unsigned long timeout)
{
	struct enocean_esp3_response resp;
	struct enocean_esp3_version *ver;
	const u8 buf[1] = { 0x03 };
	int ret;

	init_completion(&resp.comp);
	resp.disp.packet_type = ESP3_RESPONSE;
	resp.disp.dispatch = enocean_esp3_response_dispatch;
	enocean_add_esp3_dispatcher(edev, &resp.disp);

	ret = enocean_send_esp3_packet(edev, ESP3_COMMON_COMMAND, buf, sizeof(buf), NULL, 0, timeout);
	if (ret) {
		enocean_remove_esp3_dispatcher(edev, &resp.disp);
		return ret;
	}

	timeout = wait_for_completion_timeout(&resp.comp, timeout);
	enocean_remove_esp3_dispatcher(edev, &resp.disp);
	if (!timeout)
		return -ETIMEDOUT;

	switch (resp.code) {
	case 0:
		if (!resp.data)
			return -ENOMEM;
		break;
	case 2:
		if (resp.data)
			kfree(resp.data);
		return -ENOTSUPP;
	default:
		if (resp.data)
			kfree(resp.data);
		return -EIO;
	}

	ver = resp.data;
	ver->app_desc[15] = '\0';
	dev_info(&edev->serdev->dev, "'%s'\n", ver->app_desc);
	kfree(resp.data);

	return 0;
}

static int enocean_esp3_receive_buf(struct serdev_device *sdev, const u8 *data, size_t count)
{
	struct enocean_device *edev = serdev_device_get_drvdata(sdev);
	struct enocean_esp3_dispatcher *e;
	u8 crc8h, crc8d, optional_len;
	u16 data_len;

	dev_dbg(&sdev->dev, "Receive (%zu)\n", count);

	if (data[0] != ESP3_SYNC_WORD) {
		dev_warn(&sdev->dev, "not Sync Word (found 0x%02x), skipping\n",
			(unsigned int)data[0]);
		return 1;
	}

	if (count < 6)
		return 0;

	crc8h = enocean_esp3_crc8((u8*)data + 1, 4);
	if (data[5] != crc8h) {
		dev_warn(&sdev->dev, "invalid CRC8H (expected 0x%02x, found 0x%02x), skipping\n",
			(unsigned int)crc8h, (unsigned int)data[5]);
		return 1;
	}

	data_len = be16_to_cpup((__be16 *)(data + 1));
	optional_len = data[3];
	if (count < enocean_esp3_packet_size(data_len, optional_len))
		return 0;

	crc8d = enocean_esp3_crc8((u8*)data + 6, data_len + optional_len);
	if (data[6 + data_len + optional_len] != crc8d) {
		dev_warn(&sdev->dev, "invalid CRC8D (expected 0x%02x, found 0x%02x), skipping\n",
			(unsigned int)crc8d, (unsigned int)data[6 + data_len + optional_len]);
		return 1;
	}

	print_hex_dump_debug("received: ", DUMP_PREFIX_NONE, 16, 1, data,
		enocean_esp3_packet_size(data_len, optional_len), false);

	list_for_each_entry_rcu(e, &edev->esp_dispatchers, list) {
		if (e->packet_type == data[4])
			e->dispatch(data + 6, data_len, e);
	}

	return enocean_esp3_packet_size(data_len, optional_len);
}

const struct serdev_device_ops enocean_esp3_serdev_client_ops = {
	.receive_buf = enocean_esp3_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

int enocean_esp3_send(struct enocean_device *edev, u32 dest, const void *data, int data_len)
{
	return enocean_esp3_send_radio_erp1(edev, dest, data, data_len, HZ / 10);
}

int enocean_esp3_init(struct enocean_device *edev)
{
	struct enocean_esp3_priv *priv;
	int ret;

	ret = enocean_esp3_reset(edev, HZ / 10);
	if (ret)
		return ret;

	msleep(100); /* XXX */

	ret = enocean_esp3_read_version(edev, HZ / 10);

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->edev = edev;
	edev->priv = priv;

	priv->radio_erp1_response.packet_type = ESP3_RESPONSE;
	priv->radio_erp1_response.dispatch = enocean_esp3_radio_erp1_response_dispatch;

	return 0;
}

void enocean_esp3_cleanup(struct enocean_device *edev)
{
	struct enocean_esp3_priv *priv = edev->priv;

	kfree(priv);
}
