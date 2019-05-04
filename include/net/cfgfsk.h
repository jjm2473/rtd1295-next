/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * FSK config interface
 *
 * Copyright (c) 2019 Andreas Färber
 */
#ifndef __NET_CFGFSK_H
#define __NET_CFGFSK_H

#include <linux/types.h>

struct fsk_phy;

struct cfgfsk_ops {
	int (*get_freq)(struct fsk_phy *phy, u32 *val);
	int (*set_freq)(struct fsk_phy *phy, u32 val);
	int (*get_freq_dev)(struct fsk_phy *phy, u32 *val);
	int (*set_freq_dev)(struct fsk_phy *phy, u32 val);
	int (*get_tx_power)(struct fsk_phy *phy, s32 *val);
	int (*set_tx_power)(struct fsk_phy *phy, s32 val);
};

struct fsk_phy {
	struct device *dev;
	struct net_device *netdev;

	/* keep this last */
	u8 priv[0] __aligned(NETDEV_ALIGN);
};

struct fsk_phy *fsk_phy_new(const struct cfgfsk_ops *ops, size_t priv_size);
struct fsk_phy *devm_fsk_phy_new(struct device *dev, const struct cfgfsk_ops *ops, size_t priv_size);
int fsk_phy_register(struct fsk_phy *phy);
void fsk_phy_unregister(struct fsk_phy *phy);
void fsk_phy_free(struct fsk_phy *phy);

#endif
