// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * EnOcean Serial Protocol
 *
 * Copyright (c) 2019 Andreas Färber
 */
#ifndef ENOCEAN_H
#define ENOCEAN_H

#include <linux/netdevice.h>
#include <linux/rculist.h>
#include <linux/serdev.h>

struct enocean_esp_version;

struct enocean_device {
	struct serdev_device *serdev;
	const struct enocean_esp_version *version;

	struct net_device *netdev;

	struct list_head esp_dispatchers;

	void *priv;
};

extern const struct serdev_device_ops enocean_esp3_serdev_client_ops;

void enocean_esp3_crc8_populate(void);

int enocean_esp3_init(struct enocean_device *edev);

int enocean_esp3_send(struct enocean_device *edev, u32 dest, const void *data, int data_len);
void enocean_esp_tx_done(struct enocean_device *edev);

void enocean_esp3_cleanup(struct enocean_device *edev);

#endif
