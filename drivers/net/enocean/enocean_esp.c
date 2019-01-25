// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * EnOcean Serial Protocol
 *
 * Copyright (c) 2019 Andreas Färber
 */

#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/enocean/dev.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/serdev.h>

#include "enocean_esp.h"

struct enocean_esp_version {
	int version;
	int baudrate;
	const struct serdev_device_ops *serdev_client_ops;
	int (*init)(struct enocean_device *edev);
	int (*send)(struct enocean_device *edev, u32 dest, const void *data, int data_len);
	void (*cleanup)(struct enocean_device *edev);
};

struct enocean_esp_priv {
	struct enocean_dev_priv priv;

	struct enocean_device *edev;

	struct sk_buff *tx_skb;
	int tx_len;

	struct workqueue_struct *wq;
	struct work_struct tx_work;
};

static netdev_tx_t enocean_dev_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	struct enocean_esp_priv *priv = netdev_priv(netdev);

	netdev_dbg(netdev, "%s\n", __func__);

	if (skb->protocol != htons(ETH_P_ERP1) &&
	    skb->protocol != htons(ETH_P_ERP2)) {
		kfree_skb(skb);
		netdev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	netif_stop_queue(netdev);
	priv->tx_skb = skb;
	queue_work(priv->wq, &priv->tx_work);

	return NETDEV_TX_OK;
}

static int enocean_esp_tx(struct enocean_device *edev, void *data, int data_len)
{
	if (!edev->version->send)
		return -ENOTSUPP;
	return edev->version->send(edev, 0xffffffff, data, data_len);
}

static void enocean_esp_tx_work_handler(struct work_struct *ws)
{
	struct enocean_esp_priv *priv = container_of(ws, struct enocean_esp_priv, tx_work);
	struct enocean_device *edev = priv->edev;
	struct net_device *netdev = edev->netdev;

	netdev_dbg(netdev, "%s\n", __func__);

	if (priv->tx_skb) {
		enocean_esp_tx(edev, priv->tx_skb->data, priv->tx_skb->len);
		priv->tx_len = 1 + priv->tx_skb->len;
		if (!(netdev->flags & IFF_ECHO) ||
			priv->tx_skb->pkt_type != PACKET_LOOPBACK ||
			(priv->tx_skb->protocol != htons(ETH_P_ERP1) &&
			 priv->tx_skb->protocol != htons(ETH_P_ERP2)))
			kfree_skb(priv->tx_skb);
		priv->tx_skb = NULL;
	}
}

void enocean_esp_tx_done(struct enocean_device *edev)
{
	struct net_device *netdev = edev->netdev;
	struct enocean_esp_priv *priv = netdev_priv(netdev);

	netdev_info(netdev, "TX done.\n");
	netdev->stats.tx_packets++;
	netdev->stats.tx_bytes += priv->tx_len - 1;
	priv->tx_len = 0;
	netif_wake_queue(netdev);
}

static int enocean_dev_open(struct net_device *netdev)
{
	struct enocean_esp_priv *priv = netdev_priv(netdev);
	int ret;

	netdev_dbg(netdev, "%s\n", __func__);

	ret = open_enocean_dev(netdev);
	if (ret)
		return ret;

	priv->tx_skb = NULL;
	priv->tx_len = 0;

	priv->wq = alloc_workqueue("enocean_esp_wq", WQ_FREEZABLE | WQ_MEM_RECLAIM, 0);
	INIT_WORK(&priv->tx_work, enocean_esp_tx_work_handler);

	netif_wake_queue(netdev);

	return 0;
}

static int enocean_dev_stop(struct net_device *netdev)
{
	struct enocean_esp_priv *priv = netdev_priv(netdev);

	netdev_dbg(netdev, "%s\n", __func__);

	close_enocean_dev(netdev);

	destroy_workqueue(priv->wq);
	priv->wq = NULL;

	if (priv->tx_skb || priv->tx_len)
		netdev->stats.tx_errors++;
	if (priv->tx_skb)
		dev_kfree_skb(priv->tx_skb);
	priv->tx_skb = NULL;
	priv->tx_len = 0;

	return 0;
}

static const struct net_device_ops enocean_esp_netdev_ops =  {
	.ndo_open = enocean_dev_open,
	.ndo_stop = enocean_dev_stop,
	.ndo_start_xmit = enocean_dev_start_xmit,
};

static void enocean_esp_cleanup(struct enocean_device *edev)
{
	if (edev->version->cleanup)
		edev->version->cleanup(edev);
}

static const struct enocean_esp_version enocean_esp3 = {
	.version = 3,
	.baudrate = 57600,
	.serdev_client_ops = &enocean_esp3_serdev_client_ops,
	.init = enocean_esp3_init,
	.send = enocean_esp3_send,
	.cleanup = enocean_esp3_cleanup,
};

static const struct of_device_id enocean_of_match[] = {
	{ .compatible = "enocean,esp3", .data = &enocean_esp3 },
	{}
};
MODULE_DEVICE_TABLE(of, enocean_of_match);

static int enocean_probe(struct serdev_device *sdev)
{
	struct enocean_esp_priv *priv;
	struct enocean_device *edev;
	int ret;

	dev_dbg(&sdev->dev, "Probing");

	edev = devm_kzalloc(&sdev->dev, sizeof(*edev), GFP_KERNEL);
	if (!edev)
		return -ENOMEM;

	edev->version = of_device_get_match_data(&sdev->dev);
	if (!edev->version)
		return -ENOTSUPP;

	dev_dbg(&sdev->dev, "ESP%d\n", edev->version->version);

	edev->serdev = sdev;
	INIT_LIST_HEAD(&edev->esp_dispatchers);
	serdev_device_set_drvdata(sdev, edev);

	ret = serdev_device_open(sdev);
	if (ret) {
		dev_err(&sdev->dev, "Failed to open (%d)\n", ret);
		return ret;
	}

	serdev_device_set_baudrate(sdev, edev->version->baudrate);
	serdev_device_set_flow_control(sdev, false);
	serdev_device_set_client_ops(sdev, edev->version->serdev_client_ops);

	if (edev->version->init) {
		ret = edev->version->init(edev);
		if (ret) {
			serdev_device_close(sdev);
			return ret;
		}
	}

	edev->netdev = devm_alloc_enocean_dev(&sdev->dev, sizeof(struct enocean_esp_priv));
	if (!edev->netdev) {
		enocean_esp_cleanup(edev);
		serdev_device_close(sdev);
		return -ENOMEM;
	}

	edev->netdev->netdev_ops = &enocean_esp_netdev_ops;
	edev->netdev->flags |= IFF_ECHO;
	SET_NETDEV_DEV(edev->netdev, &sdev->dev);

	priv = netdev_priv(edev->netdev);
	priv->edev = edev;

	ret = register_enocean_dev(edev->netdev);
	if (ret) {
		enocean_esp_cleanup(edev);
		serdev_device_close(sdev);
		return ret;
	}

	dev_dbg(&sdev->dev, "Done.\n");

	return 0;
}

static void enocean_remove(struct serdev_device *sdev)
{
	struct enocean_device *edev = serdev_device_get_drvdata(sdev);

	unregister_enocean_dev(edev->netdev);
	enocean_esp_cleanup(edev);
	serdev_device_close(sdev);

	dev_dbg(&sdev->dev, "Removed\n");
}

static struct serdev_device_driver enocean_serdev_driver = {
	.probe = enocean_probe,
	.remove = enocean_remove,
	.driver = {
		.name = "enocean-esp",
		.of_match_table = enocean_of_match,
	},
};

static int __init enocean_init(void)
{
	int ret;

	enocean_esp3_crc8_populate();

	ret = serdev_device_driver_register(&enocean_serdev_driver);
	if (ret)
		return ret;

	return 0;
}

static void __exit enocean_exit(void)
{
	serdev_device_driver_unregister(&enocean_serdev_driver);
}

module_init(enocean_init);
module_exit(enocean_exit);

MODULE_DESCRIPTION("EnOcean serdev driver");
MODULE_AUTHOR("Andreas Färber <afaerber@suse.de>");
MODULE_LICENSE("GPL");
