// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Semtech SX1280/SX1281 LoRa transceiver
 *
 * Copyright (c) 2018-2019 Andreas Färber
 *
 * Based on sx1276.c:
 * Copyright (c) 2016-2018 Andreas Färber
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/lora.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/lora/dev.h>
#include <net/cfglora.h>

#include "sx128x.h"

struct sx128x_priv {
	struct sx128x_device *sxdev;
	struct lora_phy *lora_phy;
};

static int sx128x_get_status(struct sx128x_device *sxdev, u8 *val)
{
	return sxdev->cmd_ops->send_command(sxdev, SX128X_CMD_GET_STATUS, 0, NULL, val, 1);
}

static int sx128x_write_regs(struct sx128x_device *sxdev, u16 addr, const u8 *val, size_t len)
{
	return sxdev->cmd_ops->send_addr_command(sxdev, SX128X_CMD_WRITE_REGISTER, addr, len, val, NULL, 0);
}

static inline int sx128x_write_reg(struct sx128x_device *sxdev, u16 addr, u8 val)
{
	return sx128x_write_regs(sxdev, addr, &val, 1);
}

static int sx128x_read_regs(struct sx128x_device *sxdev, u16 addr, u8 *val, size_t len)
{
	return sxdev->cmd_ops->send_addr_command(sxdev, SX128X_CMD_READ_REGISTER, addr, 0, NULL, val, len);
}

static inline int sx128x_read_reg(struct sx128x_device *sxdev, u16 addr, u8 *val)
{
	return sx128x_read_regs(sxdev, addr, val, 1);
}

static int sx128x_set_standby(struct sx128x_device *sxdev, u8 val)
{
	return sxdev->cmd_ops->send_command(sxdev, SX128X_CMD_SET_STANDBY, 1, &val, NULL, 0);
}

static int sx128x_set_packet_type(struct sx128x_device *sxdev, u8 val)
{
	return sxdev->cmd_ops->send_command(sxdev, SX128X_CMD_SET_PACKET_TYPE, 1, &val, NULL, 0);
}

static int sx128x_set_tx_params(struct sx128x_device *sxdev, u8 power, u8 ramp_time)
{
	u8 buf[2];

	buf[0] = power;
	buf[1] = ramp_time;

	return sxdev->cmd_ops->send_command(sxdev, SX128X_CMD_SET_TX_PARAMS, 2, buf, NULL, 0);
}

static int sx128x_set_regulator_mode(struct sx128x_device *sxdev, u8 val)
{
	return sxdev->cmd_ops->send_command(sxdev, SX128X_CMD_SET_REGULATOR_MODE, 1, &val, NULL, 0);
}

static int sx128x_get_silicon_version(struct sx128x_device *sxdev, u8 *val)
{
	return sxdev->cmd_ops->send_command(sxdev, SX128X_CMD_GET_SILICON_VERSION, 0, NULL, val, 1);
}

static void sx128x_reset(struct sx128x_device *sxdev)
{
	gpiod_set_value_cansleep(sxdev->rst, 0);
	msleep(50);
	gpiod_set_value_cansleep(sxdev->rst, 1);
	msleep(20);
}

static netdev_tx_t sx128x_loradev_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	//struct sx128x_priv *priv = netdev_priv(netdev);

	netdev_dbg(netdev, "%s\n", __func__);

	if (skb->protocol != htons(ETH_P_LORA) &&
	    skb->protocol != htons(ETH_P_FLRC)) {
		kfree_skb(skb);
		netdev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	netif_stop_queue(netdev);
	/* TODO */

	return NETDEV_TX_OK;
}

static int sx128x_loradev_open(struct net_device *netdev)
{
	//struct sx128x_priv *priv = netdev_priv(netdev);
	int ret;

	netdev_dbg(netdev, "%s\n", __func__);

	ret = open_loradev(netdev);
	if (ret)
		return ret;

	/* TODO */

	netif_wake_queue(netdev);

	return 0;
}

static int sx128x_loradev_stop(struct net_device *netdev)
{
	//struct sx128x_priv *priv = netdev_priv(netdev);

	netdev_dbg(netdev, "%s\n", __func__);

	close_loradev(netdev);

	/* TODO */

	return 0;
}

static const struct net_device_ops sx128x_netdev_ops =  {
	.ndo_open = sx128x_loradev_open,
	.ndo_stop = sx128x_loradev_stop,
	.ndo_start_xmit = sx128x_loradev_start_xmit,
};

static int sx128x_lora_set_tx_power(struct lora_phy *phy, s32 value)
{
	struct sx128x_device *sxdev = dev_get_drvdata(phy->dev);

	if (value < 0 || value > 255)
		return -EINVAL;

	return sx128x_set_tx_params(sxdev, value, SX128X_RADIO_RAMP_20_US);
}

static const struct cfglora_ops sx128x_lora_ops = {
	.set_tx_power	= sx128x_lora_set_tx_power,
};

int sx128x_probe(struct sx128x_device *sxdev)
{
	struct device *dev = sxdev->dev;
	struct net_device *netdev;
	struct sx128x_priv *priv;
	u8 val, status;
	int ret;

	sxdev->rst = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(sxdev->rst)) {
		ret = PTR_ERR(sxdev->rst);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to obtain reset GPIO (%d)\n", ret);
		return ret;
	}

	sxdev->busy_gpio = devm_gpiod_get_optional(dev, "busy", GPIOD_IN);
	if (IS_ERR(sxdev->busy_gpio)) {
		ret = PTR_ERR(sxdev->busy_gpio);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to obtain reset GPIO (%d)\n", ret);
		return ret;
	}

	sx128x_reset(sxdev);

	ret = sx128x_get_status(sxdev, &status);
	if (ret) {
		dev_err(dev, "GetStatus failed (%d)\n", ret);
		return ret;
	}

	if ((status & SX128X_STATUS_MODE_MASK) != SX128X_STATUS_MODE_STDBY_RC) {
		ret = sx128x_set_standby(sxdev, SX128X_STANDBY_CONFIG_STDBY_RC);
		if (ret) {
			dev_err(dev, "SetStandby STDBY_RC failed (%d)\n", ret);
			return ret;
		}
	}

	ret = sx128x_set_regulator_mode(sxdev, SX128X_REGULATOR_MODE_LDO);
	if (ret) {
		dev_err(dev, "SetRegulatorMode LDO failed (%d)\n", ret);
		return ret;
	}

	ret = sx128x_set_tx_params(sxdev, 31, SX128X_RADIO_RAMP_20_US);
	if (ret) {
		dev_err(dev, "SetTxParams failed (%d)\n", ret);
		return ret;
	}

	ret = sx128x_get_silicon_version(sxdev, &val);
	if (ret) {
		dev_err(dev, "GetSiliconVersion failed (%d)\n", ret);
		return ret;
	}
	dev_info(dev, "silicon version: 0x%02x\n", (unsigned int)val);

	ret = sx128x_set_packet_type(sxdev, SX128X_PACKET_TYPE_LORA);
	if (ret) {
		dev_err(dev, "SetPacketType LORA failed (%d)\n", ret);
		return ret;
	}

	ret = sx128x_read_reg(sxdev, 0x925, &val);
	if (ret) {
		dev_err(dev, "ReadRegister failed (%d)\n", ret);
		return ret;
	}
	dev_info(dev, "ReadRegister 0x925: 0x%02x\n", (unsigned int)val);

	netdev = devm_alloc_loradev(dev, sizeof(*priv));
	if (!netdev)
		return -ENOMEM;

	netdev->netdev_ops = &sx128x_netdev_ops;

	priv = netdev_priv(netdev);
	priv->sxdev = sxdev;

	sxdev->netdev = netdev;
	SET_NETDEV_DEV(netdev, dev);

	priv->lora_phy = devm_lora_phy_new(dev, &sx128x_lora_ops, 0);
	if (!priv->lora_phy)
		return -ENOMEM;

	priv->lora_phy->netdev = netdev;

	ret = lora_phy_register(priv->lora_phy);
	if (ret)
		return ret;

	ret = register_loradev(netdev);
	if (ret) {
		dev_err(dev, "registering loradev failed (%d)\n", ret);
		lora_phy_unregister(priv->lora_phy);
		return ret;
	}

	dev_info(dev, "probed\n");

	return 0;
}

int sx128x_remove(struct sx128x_device *sxdev)
{
	struct sx128x_priv *priv = netdev_priv(sxdev->netdev);

	unregister_loradev(sxdev->netdev);

	lora_phy_unregister(priv->lora_phy);

	dev_info(sxdev->dev, "removed\n");

	return 0;
}

#ifdef CONFIG_OF
const struct of_device_id sx128x_dt_ids[] = {
	{ .compatible = "semtech,sx1280" },
	{}
};
MODULE_DEVICE_TABLE(of, sx128x_dt_ids);
#endif

int sx128x_status_to_errno(struct sx128x_device *sxdev, u8 status)
{
	dev_dbg(sxdev->dev, "%s: 0x%02x\n", __func__, (unsigned int)status);

	switch (status & GENMASK(4, 2)) {
	case SX128X_STATUS_COMMAND_TIMEOUT:
		return -ETIMEDOUT;
	case SX128X_STATUS_COMMAND_PROCESSING_ERROR:
		return -EINVAL;
	case SX128X_STATUS_COMMAND_FAILURE_TO_EXECUTE:
		return -EOPNOTSUPP;
	default:
		return 0;
	}
}

int sx128x_busy_check_pre(struct sx128x_device *sxdev)
{
	int ret;

	if (!sxdev->busy_gpio)
		return 0;

	ret = gpiod_get_value_cansleep(sxdev->busy_gpio);
	if (ret < 0) {
		dev_err(sxdev->dev, "reading Busy GPIO failed (%d)\n", ret);
		return ret;
	}
	if (ret > 0) {
		dev_warn(sxdev->dev, "chip is busy!\n");
		return -EBUSY;
	}
	return 0;
}

int sx128x_busy_wait_post(struct sx128x_device *sxdev)
{
	int ret, i;

	if (!sxdev->busy_gpio)
		return 0;

	for (i = 10; i > 0; i--) {
		ret = gpiod_get_value_cansleep(sxdev->busy_gpio);
		if (ret == 0)
			return 0;
		else if (ret < 0) {
			dev_err(sxdev->dev, "reading Busy GPIO failed (%d)\n", ret);
			return ret;
		}
	}
	dev_dbg(sxdev->dev, "still busy\n");
	return 0;
}

static int __init sx128x_init(void)
{
	int ret = 0;

#ifdef CONFIG_LORA_SX128X_SPI
	ret = sx128x_spi_init();
	if (ret)
		return ret;
#endif
	return ret;
}

static void __exit sx128x_exit(void)
{
#ifdef CONFIG_LORA_SX128X_SPI
	sx128x_spi_exit();
#endif
}

module_init(sx128x_init);
module_exit(sx128x_exit);

MODULE_DESCRIPTION("SX1280 driver");
MODULE_AUTHOR("Andreas Färber <afaerber@suse.de>");
MODULE_LICENSE("GPL");
