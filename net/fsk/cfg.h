/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * FSK config interface
 *
 * Copyright (c) 2019 Andreas Färber
 */
#ifndef FSK_CFGFSK_H
#define FSK_CFGFSK_H

struct cfgfsk_registered_phy {
	struct list_head list;

	const struct cfgfsk_ops *ops;

	/* keep this last */
	struct fsk_phy fsk_phy __aligned(NETDEV_ALIGN);
};

static inline struct cfgfsk_registered_phy *to_registered_phy(struct fsk_phy *phy)
{
	return container_of(phy, struct cfgfsk_registered_phy, fsk_phy);
}

struct cfgfsk_registered_phy *cfgfsk_get_phy_by_ifindex(int ifindex);

int __init nlfsk_init(void);
void __exit nlfsk_exit(void);

#endif
