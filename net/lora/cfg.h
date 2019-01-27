/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * LoRa config interface
 *
 * Copyright (c) 2019 Andreas Färber
 */
#ifndef LORA_CFGLORA_H
#define LORA_CFGLORA_H

struct cfglora_registered_phy {
	struct list_head list;

	const struct cfglora_ops *ops;

	/* keep this last */
	struct lora_phy lora_phy __aligned(NETDEV_ALIGN);
};

static inline struct cfglora_registered_phy *to_registered_phy(struct lora_phy *phy)
{
	return container_of(phy, struct cfglora_registered_phy, lora_phy);
}

struct cfglora_registered_phy *cfglora_get_phy_by_ifindex(int ifindex);

int __init nllora_init(void);
void __exit nllora_exit(void);

#endif
