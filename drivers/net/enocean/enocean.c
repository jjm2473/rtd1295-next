// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * EnOcean net device
 *
 * Copyright (c) 2019 Andreas Färber
 */

#include <linux/enocean/dev.h>
#include <linux/if_arp.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <net/rtnetlink.h>

/* XXX for external module */
#ifndef ARPHRD_ENOCEAN
#define ARPHRD_ENOCEAN 832
#endif

int open_enocean_dev(struct net_device *dev)
{
	if (!netif_carrier_ok(dev))
		netif_carrier_on(dev);

	return 0;
}
EXPORT_SYMBOL_GPL(open_enocean_dev);

void close_enocean_dev(struct net_device *dev)
{
}
EXPORT_SYMBOL_GPL(close_enocean_dev);

static void enocean_setup(struct net_device *dev)
{
	dev->type = ARPHRD_ENOCEAN;
	dev->mtu = 255; /* XXX */
	dev->hard_header_len = 0;
	dev->addr_len = 0; /* XXX 4 */
	dev->tx_queue_len = 10;

	dev->flags = IFF_NOARP;
	dev->features = 0;
}

struct net_device *alloc_enocean_dev(size_t priv_size)
{
	struct enocean_dev_priv *priv;
	struct net_device *netdev;

	netdev = alloc_netdev(priv_size, "enocean%d", NET_NAME_UNKNOWN, enocean_setup);
	if (!netdev)
		return NULL;

	priv = netdev_priv(netdev);
	priv->dev = netdev;

	return netdev;
}
EXPORT_SYMBOL_GPL(alloc_enocean_dev);

void free_enocean_dev(struct net_device *netdev)
{
	free_netdev(netdev);
}
EXPORT_SYMBOL_GPL(free_enocean_dev);

static void devm_free_enocean_dev(struct device *dev, void *res)
{
	struct net_device **net = res;

	free_enocean_dev(*net);
}

struct net_device *devm_alloc_enocean_dev(struct device *dev, size_t priv)
{
	struct net_device **ptr;
	struct net_device *net;

	net = alloc_enocean_dev(priv);
	if (!net)
		return NULL;

	ptr = devres_alloc(devm_free_enocean_dev, sizeof(*ptr), GFP_KERNEL);
	if (!ptr) {
		free_enocean_dev(net);
		return NULL;
	}

	*ptr = net;
	devres_add(dev, ptr);

	return net;
}
EXPORT_SYMBOL_GPL(devm_alloc_enocean_dev);

static struct rtnl_link_ops enocean_link_ops __read_mostly = {
	.kind = "enocean",
	.setup = enocean_setup,
};

int register_enocean_dev(struct net_device *dev)
{
	dev->rtnl_link_ops = &enocean_link_ops;
	return register_netdev(dev);
}
EXPORT_SYMBOL_GPL(register_enocean_dev);

void unregister_enocean_dev(struct net_device *dev)
{
	unregister_netdev(dev);
}
EXPORT_SYMBOL_GPL(unregister_enocean_dev);

static int __init enocean_dev_init(void)
{
	return rtnl_link_register(&enocean_link_ops);
}
module_init(enocean_dev_init);

static void __exit enocean_dev_exit(void)
{
	rtnl_link_unregister(&enocean_link_ops);
}
module_exit(enocean_dev_exit);

MODULE_DESCRIPTION("EnOcean device driver interface");
MODULE_ALIAS_RTNL_LINK("enocean");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Andreas Färber");
