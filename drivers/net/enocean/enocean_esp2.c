// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * EnOcean Serial Protocol 2
 *
 * Copyright (c) 2019 Andreas Färber
 */

#include <linux/serdev.h>

#include "enocean_esp.h"

#define ESP2_SYNC_BYTE1 0xA5
#define ESP2_SYNC_BYTE0 0x5A

struct enocean_esp2_packet {
	u8 sync[2];
#ifdef CONFIG_CPU_BIG_ENDIAN
	u8 h_seq:3;
	u8 length:5;
#else
	u8 length:5;
	u8 h_seq:3;
#endif
	u8 org;
	u8 data[4];
	u8 id[4];
	u8 status;
	u8 checksum;
} __packed;

#define ESP2_H_SEQ_TRT	0x3
#define ESP2_H_SEQ_RCT	0x4
#define ESP2_H_SEQ_TCT	0x5

#define ESP2_TELEGRAM_RESET	0x0A

#define ENOCEAN_ORG_RPS		0x05
#define ENOCEAN_ORG_1BS		0x06
#define ENOCEAN_ORG_4BS		0x07

/* Cf. EnOcean Equipment Profiles, Telegram Types (RORG) */
static inline u8 enocean_org_to_rorg(u8 org)
{
	switch (org) {
	case ENOCEAN_ORG_RPS:
		return ENOCEAN_RORG_RPS;
	case ENOCEAN_ORG_1BS:
		return ENOCEAN_RORG_1BS;
	case ENOCEAN_ORG_4BS:
		return ENOCEAN_RORG_4BS;
	default:
		return org;
	}
}

static u8 enocean_rorg_to_org(u8 rorg)
{
	switch (rorg) {
	case ENOCEAN_RORG_RPS:
		return ENOCEAN_ORG_RPS;
	case ENOCEAN_RORG_1BS:
		return ENOCEAN_ORG_1BS;
	case ENOCEAN_RORG_4BS:
		return ENOCEAN_ORG_4BS;
	default:
		return rorg;
	}
}

struct enocean_esp2_dispatcher {
	struct list_head list;
	u8 h_seq;
	void (*dispatch)(const u8 *data, u8 data_len, struct enocean_esp2_dispatcher *d);
};

static void enocean_add_esp2_dispatcher(struct enocean_device *edev,
	struct enocean_esp2_dispatcher *entry)
{
	list_add_tail_rcu(&entry->list, &edev->esp_dispatchers);
}

static void enocean_remove_esp2_dispatcher(struct enocean_device *edev,
	struct enocean_esp2_dispatcher *entry)
{
	list_del_rcu(&entry->list);
}

#define ESP2_RESPONSE_ERR_SYNTAX_H_SEQ	0x08
#define ESP2_RESPONSE_ERR_SYNTAX_LENGTH	0x09
#define ESP2_RESPONSE_ERR_SYNTAX_CHKSUM	0x0A
#define ESP2_RESPONSE_ERR_SYNTAX_ORG	0x0B
#define ESP2_RESPONSE_ERR		0x19
#define ESP2_RESPONSE_ERR_IDRANGE	0x1A
#define ESP2_RESPONSE_ERR_TX_IDRANGE	0x22
#define ESP2_RESPONSE_OK		0x58

struct enocean_esp2_priv {
	struct enocean_device *edev;
	struct enocean_esp2_dispatcher tx_telegram_response;
};

static u8 enocean_esp2_checksum(const u8 *data, int len)
{
	u8 chksum = 0;
	int i;

	for (i = 0; i < len; i++) {
		chksum += data[i];
	}
	return chksum;
}

static int enocean_esp2_send_telegram(struct enocean_device *edev, u8 h_seq, u8 org,
	const u8 *data, int data_len, unsigned long timeout)
{
	struct enocean_esp2_packet pkt;
	int ret;

	memset(&pkt, 0, sizeof(pkt));
	pkt.sync[0] = ESP2_SYNC_BYTE1;
	pkt.sync[1] = ESP2_SYNC_BYTE0;
	pkt.h_seq = h_seq;
	pkt.length = 11;
	dev_dbg(&edev->serdev->dev, "H_SEQ | LENGTH = %02x\n", (unsigned int)(((u8*)&pkt)[2]));
	pkt.org = org;
	if (data_len > 0)
		memcpy(pkt.data, data, min(data_len, 9));
	pkt.checksum = enocean_esp2_checksum(((u8 *)&pkt) + 2, sizeof(pkt) - 2 - 1);
	dev_dbg(&edev->serdev->dev, "checksum = %02x\n", (unsigned int)pkt.checksum);

	ret = serdev_device_write(edev->serdev, (const u8 *)&pkt, sizeof(pkt), timeout);
	if (ret < 0)
		return ret;
	if (ret > 0 && ret < sizeof(pkt))
		return -EIO;
	return 0;
}

static void enocean_esp2_tx_telegram_response_dispatch(const u8 *data, u8 data_len,
	struct enocean_esp2_dispatcher *d)
{
	struct enocean_esp2_priv *priv = container_of(d, struct enocean_esp2_priv, tx_telegram_response);
	struct enocean_device *edev = priv->edev;

	enocean_remove_esp2_dispatcher(edev, d);

	if (data_len < 1)
		return;

	switch (data[0]) {
	case ESP2_RESPONSE_OK:
		enocean_esp_tx_done(edev);
		break;
	case ESP2_RESPONSE_ERR:
	case ESP2_RESPONSE_ERR_TX_IDRANGE:
	default:
		break;
	}
}

static int enocean_esp2_tx_telegram(struct enocean_device *edev, u8 org,
	const u8 *data, int data_len, unsigned long timeout)
{
	struct enocean_esp2_priv *priv = edev->priv;
	int ret;

	enocean_add_esp2_dispatcher(edev, &priv->tx_telegram_response);

	ret = enocean_esp2_send_telegram(edev, ESP2_H_SEQ_TRT,
		org, data, data_len, timeout);
	if (ret) {
		enocean_remove_esp2_dispatcher(edev, &priv->tx_telegram_response);
		return ret;
	}

	return 0;
}

static int enocean_esp2_reset(struct enocean_device *edev, unsigned long timeout)
{
	return enocean_esp2_send_telegram(edev, ESP2_H_SEQ_TCT, ESP2_TELEGRAM_RESET, NULL, 0, timeout);
	/* no RCT */
}

static int enocean_esp2_receive_buf(struct serdev_device *sdev, const u8 *data, size_t count)
{
	struct enocean_device *edev = serdev_device_get_drvdata(sdev);
	struct enocean_esp2_dispatcher *e;
	u8 h_seq, length, chksum;

	dev_dbg(&sdev->dev, "Receive (%zu)\n", count);

	if (data[0] != ESP2_SYNC_BYTE1) {
		dev_warn(&sdev->dev, "not first Sync Byte (found 0x%02x), skipping\n",
			(unsigned int)data[0]);
		return 1;
	}

	if (count < 2)
		return 0;

	if (data[1] != ESP2_SYNC_BYTE0) {
		dev_warn(&sdev->dev, "not second Sync Byte (found 0x%02x 0x%02x), skipping\n",
			ESP2_SYNC_BYTE1, (unsigned int)data[1]);
		return 1;
	}

	if (count < 3)
		return 0;

	h_seq = data[2] >> 5;
	length = data[2] & 0x1f;

	if (count < 3 + length)
		return 0;

	chksum = enocean_esp2_checksum(data + 2, 1 + length - 1);
	if (data[3 + length - 1] != chksum) {
		dev_warn(&sdev->dev, "invalid checksum (expected 0x%02x, found %02x), skipping\n",
			(unsigned int)chksum, (unsigned int)data[3 + length - 1]);
		return 2; /* valid second Sync Byte is not a valid first Sync Byte */
	}

	print_hex_dump_bytes("received: ", DUMP_PREFIX_OFFSET, data, 3 + length);

	list_for_each_entry_rcu(e, &edev->esp_dispatchers, list) {
		if (e->h_seq == h_seq)
			e->dispatch(data + 3, length, e);
	}

	return 3 + length;
}

const struct serdev_device_ops enocean_esp2_serdev_client_ops = {
	.receive_buf = enocean_esp2_receive_buf,
	.write_wakeup = serdev_device_write_wakeup,
};

int enocean_esp2_send(struct enocean_device *edev, u32 dest, const void *data, int data_len)
{
	const u8 *buf = data;

	return enocean_esp2_tx_telegram(edev,
		enocean_rorg_to_org(buf[0]), buf + 1, data_len - 1, HZ);
}

int enocean_esp2_init(struct enocean_device *edev)
{
	struct enocean_esp2_priv *priv;
	int ret;

	ret = enocean_esp2_reset(edev, HZ);
	if (ret)
		return ret;

	msleep(100); /* XXX */

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->edev = edev;
	edev->priv = priv;

	priv->tx_telegram_response.h_seq = ESP2_H_SEQ_RCT;
	priv->tx_telegram_response.dispatch = enocean_esp2_tx_telegram_response_dispatch;

	return 0;
}

void enocean_esp2_cleanup(struct enocean_device *edev)
{
	struct enocean_esp2_priv *priv = edev->priv;

	kfree(priv);
}
