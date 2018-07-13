// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2018-2019 Andreas Färber
 */

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/rculist.h>
#include <linux/slab.h>
#include <net/cfglora.h>
#include <net/rtnetlink.h>

#include "cfg.h"

static LIST_HEAD(cfglora_phy_list);

struct cfglora_registered_phy *cfglora_get_phy_by_ifindex(int ifindex)
{
	struct cfglora_registered_phy *rphy;
	struct net_device *netdev;

	list_for_each_entry(rphy, &cfglora_phy_list, list) {
		netdev = rphy->lora_phy.netdev;
		if (!netdev)
			continue;
		if (netdev->ifindex == ifindex)
			return rphy;
	}

	return NULL;
}

struct lora_phy *lora_phy_new(const struct cfglora_ops *ops, size_t priv_size)
{
	struct cfglora_registered_phy *rphy;

	rphy = kzalloc(sizeof(*rphy) + priv_size, GFP_KERNEL);
	if (!rphy)
		return NULL;

	rphy->ops = ops;

	return &rphy->lora_phy;
}
EXPORT_SYMBOL_GPL(lora_phy_new);

int lora_phy_register(struct lora_phy *phy)
{
	struct cfglora_registered_phy *rphy = to_registered_phy(phy);

	rtnl_lock();

	list_add_rcu(&rphy->list, &cfglora_phy_list);

	rtnl_unlock();

	return 0;
}
EXPORT_SYMBOL_GPL(lora_phy_register);

void lora_phy_unregister(struct lora_phy *phy)
{
	struct cfglora_registered_phy *rphy = to_registered_phy(phy);

	rtnl_lock();

	list_del_rcu(&rphy->list);
	synchronize_rcu();

	rtnl_unlock();
}
EXPORT_SYMBOL_GPL(lora_phy_unregister);

void lora_phy_free(struct lora_phy *phy)
{
	struct cfglora_registered_phy *rphy = to_registered_phy(phy);

	kfree(rphy);
}
EXPORT_SYMBOL_GPL(lora_phy_free);

static void devm_lora_phy_free(struct device *dev, void *res)
{
	struct lora_phy **phy = res;

	lora_phy_free(*phy);
}

struct lora_phy *devm_lora_phy_new(struct device *dev, const struct cfglora_ops *ops, size_t priv_size)
{
	struct lora_phy **ptr;
	struct lora_phy *phy;

	phy = lora_phy_new(ops, priv_size);
	if (!phy)
		return NULL;

	phy->dev = dev;

	ptr = devres_alloc(devm_lora_phy_free, sizeof(*ptr), GFP_KERNEL);
	if (!ptr) {
		lora_phy_free(phy);
		return NULL;
	}

	*ptr = phy;
	devres_add(dev, ptr);

	return phy;
}
EXPORT_SYMBOL_GPL(devm_lora_phy_new);

static int __init cfglora_init(void)
{
	return nllora_init();
}
subsys_initcall(cfglora_init);

static void __exit cfglora_exit(void)
{
	nllora_exit();
}
module_exit(cfglora_exit);

MODULE_DESCRIPTION("LoRa configuration interface");
MODULE_AUTHOR("Andreas Färber <afaerber@suse.de>");
MODULE_LICENSE("GPL");
