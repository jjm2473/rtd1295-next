// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2018-2019 Andreas Färber
 */

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/rculist.h>
#include <linux/slab.h>
#include <net/cfgfsk.h>
#include <net/rtnetlink.h>

#include "cfg.h"

static LIST_HEAD(cfgfsk_phy_list);

struct cfgfsk_registered_phy *cfgfsk_get_phy_by_ifindex(int ifindex)
{
	struct cfgfsk_registered_phy *rphy;
	struct net_device *netdev;

	list_for_each_entry(rphy, &cfgfsk_phy_list, list) {
		netdev = rphy->fsk_phy.netdev;
		if (!netdev)
			continue;
		if (netdev->ifindex == ifindex)
			return rphy;
	}

	return NULL;
}

struct fsk_phy *fsk_phy_new(const struct cfgfsk_ops *ops, size_t priv_size)
{
	struct cfgfsk_registered_phy *rphy;

	rphy = kzalloc(sizeof(*rphy) + priv_size, GFP_KERNEL);
	if (!rphy)
		return NULL;

	rphy->ops = ops;

	return &rphy->fsk_phy;
}
EXPORT_SYMBOL_GPL(fsk_phy_new);

int fsk_phy_register(struct fsk_phy *phy)
{
	struct cfgfsk_registered_phy *rphy = to_registered_phy(phy);

	rtnl_lock();

	list_add_rcu(&rphy->list, &cfgfsk_phy_list);

	rtnl_unlock();

	return 0;
}
EXPORT_SYMBOL_GPL(fsk_phy_register);

void fsk_phy_unregister(struct fsk_phy *phy)
{
	struct cfgfsk_registered_phy *rphy = to_registered_phy(phy);

	rtnl_lock();

	list_del_rcu(&rphy->list);
	synchronize_rcu();

	rtnl_unlock();
}
EXPORT_SYMBOL_GPL(fsk_phy_unregister);

void fsk_phy_free(struct fsk_phy *phy)
{
	struct cfgfsk_registered_phy *rphy = to_registered_phy(phy);

	kfree(rphy);
}
EXPORT_SYMBOL_GPL(fsk_phy_free);

static void devm_fsk_phy_free(struct device *dev, void *res)
{
	struct fsk_phy **phy = res;

	fsk_phy_free(*phy);
}

struct fsk_phy *devm_fsk_phy_new(struct device *dev, const struct cfgfsk_ops *ops, size_t priv_size)
{
	struct fsk_phy **ptr;
	struct fsk_phy *phy;

	phy = fsk_phy_new(ops, priv_size);
	if (!phy)
		return NULL;

	phy->dev = dev;

	ptr = devres_alloc(devm_fsk_phy_free, sizeof(*ptr), GFP_KERNEL);
	if (!ptr) {
		fsk_phy_free(phy);
		return NULL;
	}

	*ptr = phy;
	devres_add(dev, ptr);

	return phy;
}
EXPORT_SYMBOL_GPL(devm_fsk_phy_new);

static int __init cfgfsk_init(void)
{
	return nlfsk_init();
}
subsys_initcall(cfgfsk_init);

static void __exit cfgfsk_exit(void)
{
	nlfsk_exit();
}
module_exit(cfgfsk_exit);

MODULE_DESCRIPTION("FSK configuration interface");
MODULE_AUTHOR("Andreas Färber <afaerber@suse.de>");
MODULE_LICENSE("GPL");
