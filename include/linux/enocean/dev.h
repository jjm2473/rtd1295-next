/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * linux/enocean/dev.h
 *
 * Copyright (c) 2019 Andreas Färber
 */
#ifndef _ENOCEAN_DEV_H
#define _ENOCEAN_DEV_H

#include <linux/netdevice.h>

struct net_device *alloc_enocean_dev(size_t priv_size);
struct net_device *devm_alloc_enocean_dev(struct device *dev, size_t priv_size);
int register_enocean_dev(struct net_device *netdev);
void unregister_enocean_dev(struct net_device *netdev);
int open_enocean_dev(struct net_device *netdev);
void close_enocean_dev(struct net_device *netdev);

struct enocean_dev_priv {
	struct net_device *dev;
};

#endif
