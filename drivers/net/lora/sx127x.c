// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Semtech SX1272/SX1276 LoRa transceiver
 *
 * Copyright (c) 2016-2019 Andreas Färber
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/lora.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/lora/dev.h>
#include <linux/spi/spi.h>
#include <net/cfgfsk.h>
#include <net/cfglora.h>

#define REG_FIFO			0x00
#define REG_OPMODE			0x01
#define FSKOOK_REG_FDEV_MSB		0x04
#define FSKOOK_REG_FDEV_LSB		0x05
#define REG_FRF_MSB			0x06
#define REG_FRF_MID			0x07
#define REG_FRF_LSB			0x08
#define REG_PA_CONFIG			0x09
#define LORA_REG_FIFO_ADDR_PTR		0x0d
#define LORA_REG_FIFO_TX_BASE_ADDR	0x0e
#define LORA_REG_IRQ_FLAGS_MASK		0x11
#define LORA_REG_IRQ_FLAGS		0x12
#define LORA_REG_MODEM_CONFIG1		0x1d
#define LORA_REG_MODEM_CONFIG2		0x1e
#define LORA_REG_PAYLOAD_LENGTH		0x22
#define LORA_REG_SYNC_WORD		0x39
#define REG_DIO_MAPPING1		0x40
#define REG_DIO_MAPPING2		0x41
#define REG_VERSION			0x42
#define REG_PA_DAC			0x4d

#define REG_OPMODE_LONG_RANGE_MODE		BIT(7)
#define FSKOOK_REG_OPMODE_MODULATION_TYPE_SHIFT	5
#define FSKOOK_REG_OPMODE_MODULATION_TYPE_MASK	GENMASK(6, 5)
#define FSKOOK_REG_OPMODE_MODULATION_TYPE_FSK	(0x0 << 5)
#define FSKOOK_REG_OPMODE_MODULATION_TYPE_OOK	(0x1 << 5)
#define REG_OPMODE_LOW_FREQUENCY_MODE_ON	BIT(3)
#define REG_OPMODE_MODE_MASK			GENMASK(2, 0)
#define REG_OPMODE_MODE_SLEEP			(0x0 << 0)
#define REG_OPMODE_MODE_STDBY			(0x1 << 0)
#define REG_OPMODE_MODE_TX			(0x3 << 0)
#define REG_OPMODE_MODE_RXCONTINUOUS		(0x5 << 0)
#define REG_OPMODE_MODE_RXSINGLE		(0x6 << 0)

#define FSKOOK_REG_FDEV_MSB_MASK		GENMASK(5, 0)

#define REG_PA_CONFIG_PA_SELECT			BIT(7)
#define REG_PA_CONFIG_PA_SELECT_RFO		0
#define REG_PA_CONFIG_PA_SELECT_PA_BOOST	BIT(7)
#define SX1276_REG_PA_CONFIG_MAX_POWER_SHIFT	4
#define SX1276_REG_PA_CONFIG_MAX_POWER_MASK	GENMASK(6, 4)
#define REG_PA_CONFIG_OUTPUT_POWER_MASK		GENMASK(3, 0)

#define LORA_REG_IRQ_FLAGS_TX_DONE		BIT(3)
#define LORA_REG_IRQ_FLAGS_RX_DONE		BIT(6)

#define LORA_REG_MODEM_CONFIG1_BW_MASK		GENMASK(7, 4)
#define LORA_REG_MODEM_CONFIG1_BW_SHIFT		4
#define LORA_REG_MODEM_CONFIG1_CR_MASK		GENMASK(3, 1)
#define LORA_REG_MODEM_CONFIG1_CR_SHIFT		1

#define LORA_REG_MODEM_CONFIG2_SF_MASK		GENMASK(7, 4)
#define LORA_REG_MODEM_CONFIG2_SF_SHIFT		4

#define REG_DIO_MAPPING1_DIO0_MASK	GENMASK(7, 6)

#define SX1272_REG_VERSION_V2B	0x22
#define SX1276_REG_VERSION_V1B	0x12

#define REG_PA_DAC_PA_DAC_MASK		GENMASK(2, 0)
#define REG_PA_DAC_PA_DAC_DEFAULT	0x04
#define REG_PA_DAC_PA_DAC_20_DBM	0x07

struct sx127x_priv;

struct sx127x_model {
	unsigned int number;
	unsigned int version;
	int (*reset)(struct sx127x_priv *priv);
};

struct sx127x_priv {
	struct lora_dev_priv lora;
	struct lora_phy *lora_phy;
	struct fsk_phy *fsk_phy;
	struct spi_device *spi;
	struct regmap *regmap;
	struct gpio_desc *rst;
	struct gpio_desc *dio[6];
	const struct sx127x_model *model;

	size_t fifosize;

	struct mutex spi_lock;

	struct sk_buff *tx_skb;
	int tx_len;

	struct workqueue_struct *wq;
	struct work_struct tx_work;

	struct dentry *debugfs;
};

static int sx127x_get_xosc_freq(struct sx127x_priv *priv, u32 *val)
{
	struct spi_device *spi = priv->spi;

	return of_property_read_u32(spi->dev.of_node, "clock-frequency", val);
}

static bool sx127x_volatile_reg(struct device *dev, unsigned int reg)
{
	if (reg == REG_FIFO)
		return true;

	return false;
}

static bool sx127x_writeable_noinc_reg(struct device *dev, unsigned int reg)
{
	if (reg == REG_FIFO)
		return true;

	return false;
}

static bool sx127x_readable_noinc_reg(struct device *dev, unsigned int reg)
{
	if (reg == REG_FIFO)
		return true;

	return false;
}

static struct regmap_config sx127x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.cache_type = REGCACHE_NONE,

	.read_flag_mask = 0,
	.write_flag_mask = BIT(7),

	.volatile_reg = sx127x_volatile_reg,
	.writeable_noinc_reg = sx127x_writeable_noinc_reg,
	.readable_noinc_reg = sx127x_readable_noinc_reg,

	.max_register = 0xff,
};

static netdev_tx_t sx127x_loradev_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	struct sx127x_priv *priv = netdev_priv(netdev);

	netdev_dbg(netdev, "%s\n", __func__);

	if (priv->tx_skb || priv->tx_len) {
		netdev_warn(netdev, "TX busy\n");
		return NETDEV_TX_BUSY;
	}

	if (skb->protocol != htons(ETH_P_LORA)) {
		kfree_skb(skb);
		netdev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	netif_stop_queue(netdev);
	priv->tx_skb = skb;
	queue_work(priv->wq, &priv->tx_work);

	return NETDEV_TX_OK;
}

static int sx127x_start_rx(struct spi_device *spi)
{
	struct net_device *netdev = spi_get_drvdata(spi);
	struct sx127x_priv *priv = netdev_priv(netdev);
	unsigned int val;
	int ret;

	ret = regmap_read(priv->regmap, REG_OPMODE, &val);
	if (ret) {
		dev_err(&spi->dev, "Failed to read RegOpMode (%d)\n", ret);
		return ret;
	}
	dev_dbg(&spi->dev, "RegOpMode = 0x%02x\n", val);
	if (!(val & REG_OPMODE_LONG_RANGE_MODE))
		dev_err(&spi->dev, "LongRange Mode not active!\n");
	if ((val & REG_OPMODE_MODE_MASK) == REG_OPMODE_MODE_SLEEP)
		dev_err(&spi->dev, "Cannot access FIFO in Sleep Mode!\n");

	ret = regmap_read(priv->regmap, LORA_REG_IRQ_FLAGS, &val);
	if (ret) {
		dev_err(&spi->dev, "Failed to read RegIrqFlags (%d)\n", ret);
		return ret;
	}
	dev_dbg(&spi->dev, "RegIrqFlags = 0x%02x\n", val);

	ret = regmap_write(priv->regmap, LORA_REG_IRQ_FLAGS, LORA_REG_IRQ_FLAGS_RX_DONE);
	if (ret) {
		dev_err(&spi->dev, "Failed to write RegIrqFlags (%d)\n", ret);
		return ret;
	}

	ret = regmap_read(priv->regmap, LORA_REG_IRQ_FLAGS_MASK, &val);
	if (ret) {
		dev_err(&spi->dev, "Failed to read RegIrqFlagsMask (%d)\n", ret);
		return ret;
	}
	dev_dbg(&spi->dev, "RegIrqFlagsMask = 0x%02x\n", val);

	val &= ~LORA_REG_IRQ_FLAGS_RX_DONE;
	ret = regmap_write(priv->regmap, LORA_REG_IRQ_FLAGS_MASK, val);
	if (ret) {
		dev_err(&spi->dev, "Failed to write RegIrqFlagsMask (%d)\n", ret);
		return ret;
	}

	ret = regmap_read(priv->regmap, REG_DIO_MAPPING1, &val);
	if (ret) {
		dev_err(&spi->dev, "Failed to read RegDioMapping1 (%d)\n", ret);
		return ret;
	}

	val &= ~REG_DIO_MAPPING1_DIO0_MASK;
	val |= 0x1 << 6;
	ret = regmap_write(priv->regmap, REG_DIO_MAPPING1, val);
	if (ret) {
		dev_err(&spi->dev, "Failed to write RegDioMapping1 (%d)\n", ret);
		return ret;
	}

	ret = regmap_read(priv->regmap, REG_OPMODE, &val);
	if (ret) {
		dev_err(&spi->dev, "Failed to read RegOpMode (%d)\n", ret);
		return ret;
	}

	val &= ~REG_OPMODE_MODE_MASK;
	val |= REG_OPMODE_MODE_RXSINGLE;
	ret = regmap_write(priv->regmap, REG_OPMODE, val);
	if (ret) {
		dev_err(&spi->dev, "Failed to write RegOpMode (%d)\n", ret);
		return ret;
	}

	dev_dbg(&spi->dev, "%s: done\n", __func__);

	return 0;
}

static int sx127x_tx(struct spi_device *spi, void *data, int data_len)
{
	struct net_device *netdev = spi_get_drvdata(spi);
	struct sx127x_priv *priv = netdev_priv(netdev);
	unsigned int addr, val;
	int ret;

	dev_dbg(&spi->dev, "%s (data_len %d)\n", __func__, data_len);

	ret = regmap_read(priv->regmap, REG_OPMODE, &val);
	if (ret) {
		dev_err(&spi->dev, "Failed to read RegOpMode (%d)\n", ret);
		return ret;
	}
	dev_dbg(&spi->dev, "RegOpMode = 0x%02x\n", val);
	if (!(val & REG_OPMODE_LONG_RANGE_MODE))
		dev_err(&spi->dev, "LongRange Mode not active!\n");
	if ((val & REG_OPMODE_MODE_MASK) == REG_OPMODE_MODE_SLEEP)
		dev_err(&spi->dev, "Cannot access FIFO in Sleep Mode!\n");

	ret = regmap_read(priv->regmap, LORA_REG_FIFO_TX_BASE_ADDR, &addr);
	if (ret) {
		dev_err(&spi->dev, "Failed to read RegFifoTxBaseAddr (%d)\n", ret);
		return ret;
	}
	dev_dbg(&spi->dev, "RegFifoTxBaseAddr = 0x%02x\n", addr);

	ret = regmap_write(priv->regmap, LORA_REG_FIFO_ADDR_PTR, addr);
	if (ret) {
		dev_err(&spi->dev, "Failed to write RegFifoAddrPtr (%d)\n", ret);
		return ret;
	}

	ret = regmap_write(priv->regmap, LORA_REG_PAYLOAD_LENGTH, data_len);
	if (ret) {
		dev_err(&spi->dev, "Failed to write RegPayloadLength (%d)\n", ret);
		return ret;
	}

	ret = regmap_noinc_write(priv->regmap, REG_FIFO, data, data_len);
	if (ret) {
		dev_err(&spi->dev, "Failed to write into FIFO (%d)\n", ret);
		return ret;
	}

	ret = regmap_read(priv->regmap, LORA_REG_IRQ_FLAGS, &val);
	if (ret) {
		dev_err(&spi->dev, "Failed to read RegIrqFlags (%d)\n", ret);
		return ret;
	}
	dev_dbg(&spi->dev, "RegIrqFlags = 0x%02x\n", val);

	ret = regmap_write(priv->regmap, LORA_REG_IRQ_FLAGS, LORA_REG_IRQ_FLAGS_TX_DONE);
	if (ret) {
		dev_err(&spi->dev, "Failed to write RegIrqFlags (%d)\n", ret);
		return ret;
	}

	ret = regmap_read(priv->regmap, LORA_REG_IRQ_FLAGS_MASK, &val);
	if (ret) {
		dev_err(&spi->dev, "Failed to read RegIrqFlagsMask (%d)\n", ret);
		return ret;
	}
	dev_dbg(&spi->dev, "RegIrqFlagsMask = 0x%02x\n", val);

	val &= ~LORA_REG_IRQ_FLAGS_TX_DONE;
	ret = regmap_write(priv->regmap, LORA_REG_IRQ_FLAGS_MASK, val);
	if (ret) {
		dev_err(&spi->dev, "Failed to write RegIrqFlagsMask (%d)\n", ret);
		return ret;
	}

	ret = regmap_read(priv->regmap, REG_DIO_MAPPING1, &val);
	if (ret) {
		dev_err(&spi->dev, "Failed to read RegDioMapping1 (%d)\n", ret);
		return ret;
	}

	val &= ~REG_DIO_MAPPING1_DIO0_MASK;
	val |= 0x1 << 6;
	ret = regmap_write(priv->regmap, REG_DIO_MAPPING1, val);
	if (ret) {
		dev_err(&spi->dev, "Failed to write RegDioMapping1 (%d)\n", ret);
		return ret;
	}

	ret = regmap_read(priv->regmap, REG_OPMODE, &val);
	if (ret) {
		dev_err(&spi->dev, "Failed to read RegOpMode (%d)\n", ret);
		return ret;
	}

	val &= ~REG_OPMODE_MODE_MASK;
	val |= REG_OPMODE_MODE_TX;
	ret = regmap_write(priv->regmap, REG_OPMODE, val);
	if (ret) {
		dev_err(&spi->dev, "Failed to write RegOpMode (%d)\n", ret);
		return ret;
	}

	dev_dbg(&spi->dev, "%s: done\n", __func__);

	return 0;
}

static void sx127x_tx_work_handler(struct work_struct *ws)
{
	struct sx127x_priv *priv = container_of(ws, struct sx127x_priv, tx_work);
	struct spi_device *spi = priv->spi;
	struct net_device *netdev = spi_get_drvdata(spi);

	netdev_dbg(netdev, "%s\n", __func__);

	mutex_lock(&priv->spi_lock);

	if (priv->tx_skb) {
		sx127x_tx(spi, priv->tx_skb->data, priv->tx_skb->len);
		priv->tx_len = 1 + priv->tx_skb->len;
		if (!(netdev->flags & IFF_ECHO) ||
			priv->tx_skb->pkt_type != PACKET_LOOPBACK ||
			priv->tx_skb->protocol != htons(ETH_P_LORA))
			kfree_skb(priv->tx_skb);
		priv->tx_skb = NULL;
	}

	mutex_unlock(&priv->spi_lock);
}

static irqreturn_t sx127x_dio_interrupt(int irq, void *dev_id)
{
	struct net_device *netdev = dev_id;
	struct sx127x_priv *priv = netdev_priv(netdev);
	unsigned int val;
	int ret;

	netdev_dbg(netdev, "%s\n", __func__);

	mutex_lock(&priv->spi_lock);

	ret = regmap_read(priv->regmap, LORA_REG_IRQ_FLAGS, &val);
	if (ret) {
		netdev_warn(netdev, "Failed to read RegIrqFlags (%d)\n", ret);
		val = 0;
	}

	if (val & LORA_REG_IRQ_FLAGS_RX_DONE) {
		netdev_info(netdev, "RX done.\n");
		netdev->stats.rx_packets++;

		ret = regmap_write(priv->regmap, LORA_REG_IRQ_FLAGS, LORA_REG_IRQ_FLAGS_RX_DONE);
		if (ret)
			netdev_warn(netdev, "Failed to write RegIrqFlags (%d)\n", ret);
	}
	if (val & LORA_REG_IRQ_FLAGS_TX_DONE) {
		netdev_info(netdev, "TX done.\n");
		netdev->stats.tx_packets++;
		netdev->stats.tx_bytes += priv->tx_len - 1;
		priv->tx_len = 0;
		netif_wake_queue(netdev);

		ret = regmap_write(priv->regmap, LORA_REG_IRQ_FLAGS, LORA_REG_IRQ_FLAGS_TX_DONE);
		if (ret)
			netdev_warn(netdev, "Failed to write RegIrqFlags (%d)\n", ret);
	}
	if ((val & LORA_REG_IRQ_FLAGS_RX_DONE) ||
	    (val & LORA_REG_IRQ_FLAGS_TX_DONE)) {
		ret = sx127x_start_rx(priv->spi);
		if (ret)
			netdev_warn(netdev, "Failed to re-start RX (%d)\n", ret);
	}

	mutex_unlock(&priv->spi_lock);

	return IRQ_HANDLED;
}

static int sx127x_loradev_open(struct net_device *netdev)
{
	struct sx127x_priv *priv = netdev_priv(netdev);
	unsigned int val;
	int ret, irq;

	netdev_dbg(netdev, "%s\n", __func__);

	ret = open_loradev(netdev);
	if (ret)
		return ret;

	mutex_lock(&priv->spi_lock);

	ret = regmap_read(priv->regmap, REG_OPMODE, &val);
	if (ret) {
		netdev_err(netdev, "Failed to read RegOpMode (%d)\n", ret);
		goto err_opmode;
	}

	val &= ~REG_OPMODE_MODE_MASK;
	val |= REG_OPMODE_MODE_STDBY;
	ret = regmap_write(priv->regmap, REG_OPMODE, val);
	if (ret) {
		netdev_err(netdev, "Failed to write RegOpMode (%d)\n", ret);
		goto err_opmode;
	}

	priv->tx_skb = NULL;
	priv->tx_len = 0;

	priv->wq = alloc_workqueue("sx127x_wq", WQ_FREEZABLE | WQ_MEM_RECLAIM, 0);
	INIT_WORK(&priv->tx_work, sx127x_tx_work_handler);

	if (priv->dio[0]) {
		irq = gpiod_to_irq(priv->dio[0]);
		if (irq <= 0)
			netdev_warn(netdev, "Failed to obtain interrupt for DIO0 (%d)\n", irq);
		else {
			netdev_info(netdev, "Succeeded in obtaining interrupt for DIO0: %d\n", irq);
			ret = request_threaded_irq(irq, NULL, sx127x_dio_interrupt, IRQF_ONESHOT | IRQF_TRIGGER_RISING, netdev->name, netdev);
			if (ret) {
				netdev_err(netdev, "Failed to request interrupt for DIO0 (%d)\n", ret);
				goto err_irq;
			}
			ret = sx127x_start_rx(priv->spi);
			if (ret) {
				netdev_err(netdev, "Failed to start RX (%d)\n", ret);
				goto err_rx;
			}
		}
	}

	netif_wake_queue(netdev);

	mutex_unlock(&priv->spi_lock);

	return 0;

err_rx:
err_irq:
	destroy_workqueue(priv->wq);
	priv->wq = NULL;
err_opmode:
	close_loradev(netdev);
	mutex_unlock(&priv->spi_lock);
	return ret;
}

static int sx127x_loradev_stop(struct net_device *netdev)
{
	struct sx127x_priv *priv = netdev_priv(netdev);
	unsigned int val;
	int ret, irq;

	netdev_dbg(netdev, "%s\n", __func__);

	close_loradev(netdev);

	mutex_lock(&priv->spi_lock);

	ret = regmap_write(priv->regmap, LORA_REG_IRQ_FLAGS_MASK, 0xff);
	if (ret) {
		netdev_err(netdev, "Failed to write RegIrqFlagsMask (%d)\n", ret);
		goto err_irqmask;
	}

	ret = regmap_read(priv->regmap, REG_OPMODE, &val);
	if (ret) {
		netdev_err(netdev, "Failed to read RegOpMode (%d)\n", ret);
		goto err_opmode;
	}

	val &= ~REG_OPMODE_MODE_MASK;
	val |= REG_OPMODE_MODE_SLEEP;
	ret = regmap_write(priv->regmap, REG_OPMODE, val);
	if (ret) {
		netdev_err(netdev, "Failed to write RegOpMode (%d)\n", ret);
		goto err_opmode;
	}

	if (priv->dio[0]) {
		irq = gpiod_to_irq(priv->dio[0]);
		if (irq > 0) {
			netdev_dbg(netdev, "Freeing IRQ %d\n", irq);
			free_irq(irq, netdev);
		}
	}

	destroy_workqueue(priv->wq);
	priv->wq = NULL;

	if (priv->tx_skb || priv->tx_len)
		netdev->stats.tx_errors++;
	if (priv->tx_skb)
		dev_kfree_skb(priv->tx_skb);
	priv->tx_skb = NULL;
	priv->tx_len = 0;

	mutex_unlock(&priv->spi_lock);

	return 0;

err_opmode:
err_irqmask:
	mutex_unlock(&priv->spi_lock);
	return ret;
}

static const struct net_device_ops sx127x_netdev_ops =  {
	.ndo_open = sx127x_loradev_open,
	.ndo_stop = sx127x_loradev_stop,
	.ndo_start_xmit = sx127x_loradev_start_xmit,
};

static int sx127x_get_freq(struct sx127x_priv *priv, u32 *val)
{
	int ret;
	unsigned int msb, mid, lsb;
	u32 freq_xosc;
	unsigned long long frf;

	ret = sx127x_get_xosc_freq(priv, &freq_xosc);
	if (ret)
		return ret;

	mutex_lock(&priv->spi_lock);

	ret = regmap_read(priv->regmap, REG_FRF_MSB, &msb);
	if (!ret)
		ret = regmap_read(priv->regmap, REG_FRF_MID, &mid);
	if (!ret)
		ret = regmap_read(priv->regmap, REG_FRF_LSB, &lsb);

	mutex_unlock(&priv->spi_lock);

	if (ret)
		return ret;

	frf = freq_xosc;
	frf *= ((ulong)msb << 16) | ((ulong)mid << 8) | lsb;
	do_div(frf, 1 << 19);
	*val = frf;

	return 0;
}

static int sx127x_set_freq(struct sx127x_priv *priv, u32 freq)
{
	struct spi_device *spi = priv->spi;
	unsigned long long freq_rf;
	u32 freq_xosc;
	unsigned int val;
	int ret;

	ret = sx127x_get_xosc_freq(priv, &freq_xosc);
	if (ret)
		return ret;

	mutex_lock(&priv->spi_lock);

	ret = regmap_read(priv->regmap, REG_OPMODE, &val);
	if (ret) {
		dev_err(&spi->dev, "failed reading RegPaDac\n");
		mutex_unlock(&priv->spi_lock);
		return ret;
	}
	/* only write OpMode if necessary */
	if ((freq < 525000000) == !(val & REG_OPMODE_LOW_FREQUENCY_MODE_ON)) {
		if (freq < 525000000)
			val |= REG_OPMODE_LOW_FREQUENCY_MODE_ON;
		else
			val &= ~REG_OPMODE_LOW_FREQUENCY_MODE_ON;
		ret = regmap_write(priv->regmap, REG_OPMODE, val);
		if (ret) {
			dev_err(&spi->dev, "failed writing OpMode\n");
			mutex_unlock(&priv->spi_lock);
			return ret;
		}
		dev_info(&spi->dev, "wrote OpMode\n");
	} else
		dev_info(&spi->dev, "skipped writing OpMode\n");

	freq_rf = freq;
	freq_rf *= (1 << 19);
	do_div(freq_rf, freq_xosc);
	dev_dbg(&spi->dev, "Frf = %llu", freq_rf);

	ret = regmap_write(priv->regmap, REG_FRF_MSB, freq_rf >> 16);
	if (!ret)
		ret = regmap_write(priv->regmap, REG_FRF_MID, freq_rf >> 8);
	if (!ret)
		ret = regmap_write(priv->regmap, REG_FRF_LSB, freq_rf);

	mutex_unlock(&priv->spi_lock);

	return ret;
}

static int sx127x_get_tx_power(struct sx127x_priv *priv, s32 *power)
{
	struct spi_device *spi = priv->spi;
	unsigned int val, output_power;
	int ret;

	mutex_lock(&priv->spi_lock);

	ret = regmap_read(priv->regmap, REG_PA_CONFIG, &val);
	if (ret) {
		dev_err(&spi->dev, "failed reading RegPaConfig\n");
		goto out;
	}
	output_power = val & REG_PA_CONFIG_OUTPUT_POWER_MASK;
	switch (val & REG_PA_CONFIG_PA_SELECT) {
	case REG_PA_CONFIG_PA_SELECT_PA_BOOST:
		if (output_power == 0xf) {
			ret = regmap_read(priv->regmap, REG_PA_DAC, &val);
			if (ret) {
				dev_err(&spi->dev, "failed reading RegPaDac\n");
				goto out;
			}
			if ((val & REG_PA_DAC_PA_DAC_MASK) == REG_PA_DAC_PA_DAC_20_DBM) {
				*power = 20;
				ret = 0;
				goto out;
			}
		}
		if (priv->model->number == 1272)
			*power = 2 + output_power;
		else
			*power = 17 - (15 - output_power);
		break;
	case REG_PA_CONFIG_PA_SELECT_RFO:
		if (priv->model->number == 1272)
			*power = -1 + output_power;
		else {
			unsigned int max_power;
			u64 pmax;
			max_power = (val & SX1276_REG_PA_CONFIG_MAX_POWER_MASK);
			max_power >>= SX1276_REG_PA_CONFIG_MAX_POWER_SHIFT;
			pmax = 108 + 6 * max_power;
			do_div(pmax, 10);
			*power = pmax - (15 - output_power);
		}
		break;
	}

out:
	mutex_unlock(&priv->spi_lock);
	return ret;
}

static int sx127x_set_tx_power(struct sx127x_priv *priv, s32 power)
{
	struct spi_device *spi = priv->spi;
	unsigned int val;
	int ret;

	if (power > 20 || (power == 18 || power == 19))
		return -EINVAL;

	mutex_lock(&priv->spi_lock);

	ret = regmap_read(priv->regmap, REG_PA_CONFIG, &val);
	if (ret) {
		dev_err(&spi->dev, "failed reading RegPaConfig\n");
		goto out;
	}
	if ((power > 13 && priv->model->number == 1272) ||
	    (power > 14 && priv->model->number == 1276))
		val |= REG_PA_CONFIG_PA_SELECT_PA_BOOST;
	else
		val &= ~REG_PA_CONFIG_PA_SELECT;
	val &= ~REG_PA_CONFIG_OUTPUT_POWER_MASK;
	switch (val & REG_PA_CONFIG_PA_SELECT) {
	case REG_PA_CONFIG_PA_SELECT_PA_BOOST:
		if (power == 20)
			val |= 0xf;
		else
			val |= (power - 2) & REG_PA_CONFIG_OUTPUT_POWER_MASK;
		break;
	case REG_PA_CONFIG_PA_SELECT_RFO:
		if (priv->model->number == 1272)
			val |= (power + 1) & REG_PA_CONFIG_OUTPUT_POWER_MASK;
		else {
			unsigned int max_power = (power >= 0) ? 7 : 2;
			u64 pmax;
			val &= ~SX1276_REG_PA_CONFIG_MAX_POWER_MASK;
			val |= (max_power << SX1276_REG_PA_CONFIG_MAX_POWER_SHIFT) & SX1276_REG_PA_CONFIG_MAX_POWER_MASK;
			pmax = 108 + 6 * max_power;
			do_div(pmax, 10);
			val |= (u32)(power + 15 - pmax) & REG_PA_CONFIG_OUTPUT_POWER_MASK;
		}
		break;
	}
	ret = regmap_write(priv->regmap, REG_PA_CONFIG, val);
	if (ret) {
		dev_err(&spi->dev, "failed writing RegPaConfig\n");
		goto out;
	}

	ret = regmap_read(priv->regmap, REG_PA_DAC, &val);
	if (ret) {
		dev_err(&spi->dev, "failed reading RegPaDac\n");
		goto out;
	}
	val &= ~REG_PA_DAC_PA_DAC_MASK;
	val |= (power == 20) ? REG_PA_DAC_PA_DAC_20_DBM : REG_PA_DAC_PA_DAC_DEFAULT;
	ret = regmap_write(priv->regmap, REG_PA_DAC, val);
	if (ret) {
		dev_err(&spi->dev, "failed writing RegPaDac\n");
		goto out;
	}

out:
	mutex_unlock(&priv->spi_lock);
	return ret;
}

static int sx127x_is_lora(struct sx127x_priv *priv, bool *val)
{
	unsigned int opmode;
	int ret;

	mutex_lock(&priv->spi_lock);
	ret = regmap_read(priv->regmap, REG_OPMODE, &opmode);
	mutex_unlock(&priv->spi_lock);
	if (ret)
		return ret;

	*val = (opmode & REG_OPMODE_LONG_RANGE_MODE) ? true : false;
	return 0;
}

static int sx127x_lora_get_freq(struct lora_phy *phy, u32 *val)
{
	struct net_device *netdev = dev_get_drvdata(phy->dev);
	struct sx127x_priv *priv = netdev_priv(netdev);

	return sx127x_get_freq(priv, val);
}

static int sx127x_lora_set_freq(struct lora_phy *phy, u32 val)
{
	struct net_device *netdev = dev_get_drvdata(phy->dev);
	struct sx127x_priv *priv = netdev_priv(netdev);

	return sx127x_set_freq(priv, val);
}

static u32 sx127x_lora_to_bandwidth(unsigned int bw)
{
	switch (bw) {
	case 0: return   7800;
	case 1: return  10400;
	case 2: return  15600;
	case 3: return  20800;
	case 4: return  31250;
	case 5: return  41700;
	case 6: return  62500;
	case 7: return 125000;
	case 8: return 250000;
	case 9: return 500000;
	default: return 0;
	}
}

static int sx127x_lora_get_bandwidth(struct lora_phy *phy, u32 *val)
{
	struct net_device *netdev = dev_get_drvdata(phy->dev);
	struct sx127x_priv *priv = netdev_priv(netdev);
	unsigned int reg;
	bool lora;
	int ret;

	ret = sx127x_is_lora(priv, &lora);
	if (ret)
		return ret;
	if (!lora)
		return -EBUSY;

	mutex_lock(&priv->spi_lock);
	ret = regmap_read(priv->regmap, LORA_REG_MODEM_CONFIG1, &reg);
	mutex_unlock(&priv->spi_lock);
	if (ret)
		return ret;

	*val = sx127x_lora_to_bandwidth((reg & LORA_REG_MODEM_CONFIG1_BW_MASK) >> LORA_REG_MODEM_CONFIG1_BW_SHIFT);
	return 0;
}

static int sx127x_lora_set_bandwidth(struct lora_phy *phy, u32 val)
{
	struct net_device *netdev = dev_get_drvdata(phy->dev);
	struct sx127x_priv *priv = netdev_priv(netdev);
	unsigned int cfg, bw;
	bool lora;
	int ret;

	ret = sx127x_is_lora(priv, &lora);
	if (ret)
		return ret;
	if (!lora)
		return -EBUSY;

	switch (val) {
	case   7800: bw = 0; break;
	case  10400: bw = 1; break;
	case  15600: bw = 2; break;
	case  20800: bw = 3; break;
	case  31250: bw = 4; break;
	case  41700: bw = 5; break;
	case  62500: bw = 6; break;
	case 125000: bw = 7; break;
	case 250000: bw = 8; break;
	case 500000: bw = 9; break;
	default: return -EINVAL;
	}

	mutex_lock(&priv->spi_lock);

	ret = regmap_read(priv->regmap, LORA_REG_MODEM_CONFIG1, &cfg);
	if (ret)
		goto out;
	cfg &= ~LORA_REG_MODEM_CONFIG1_BW_MASK;
	cfg |= bw << LORA_REG_MODEM_CONFIG1_BW_SHIFT;
	ret = regmap_write(priv->regmap, LORA_REG_MODEM_CONFIG1, cfg);

out:
	mutex_unlock(&priv->spi_lock);
	return ret;
}

static int sx127x_lora_get_sf(struct lora_phy *phy, u8 *val)
{
	struct net_device *netdev = dev_get_drvdata(phy->dev);
	struct sx127x_priv *priv = netdev_priv(netdev);
	unsigned int cfg;
	bool lora;
	int ret;

	ret = sx127x_is_lora(priv, &lora);
	if (ret)
		return ret;
	if (!lora)
		return -EBUSY;

	mutex_lock(&priv->spi_lock);
	ret = regmap_read(priv->regmap, LORA_REG_MODEM_CONFIG2, &cfg);
	mutex_unlock(&priv->spi_lock);
	if (ret)
		return ret;

	*val = (cfg & LORA_REG_MODEM_CONFIG2_SF_MASK) >> LORA_REG_MODEM_CONFIG2_SF_SHIFT;
	return 0;
}

static int sx127x_lora_set_sf(struct lora_phy *phy, u8 val)
{
	struct net_device *netdev = dev_get_drvdata(phy->dev);
	struct sx127x_priv *priv = netdev_priv(netdev);
	unsigned int cfg;
	bool lora;
	int ret;

	ret = sx127x_is_lora(priv, &lora);
	if (ret)
		return ret;
	if (!lora)
		return -EBUSY;

	mutex_lock(&priv->spi_lock);

	ret = regmap_read(priv->regmap, LORA_REG_MODEM_CONFIG2, &cfg);
	if (ret)
		goto out;
	cfg &= ~LORA_REG_MODEM_CONFIG2_SF_MASK;
	cfg |= val << LORA_REG_MODEM_CONFIG2_SF_SHIFT;
	ret = regmap_write(priv->regmap, LORA_REG_MODEM_CONFIG2, cfg);

out:
	mutex_unlock(&priv->spi_lock);
	return ret;
}

static int sx127x_lora_get_cr(struct lora_phy *phy, u8 *val)
{
	struct net_device *netdev = dev_get_drvdata(phy->dev);
	struct sx127x_priv *priv = netdev_priv(netdev);
	unsigned int cfg;
	bool lora;
	int ret;

	ret = sx127x_is_lora(priv, &lora);
	if (ret)
		return ret;
	if (!lora)
		return -EBUSY;

	mutex_lock(&priv->spi_lock);
	ret = regmap_read(priv->regmap, LORA_REG_MODEM_CONFIG1, &cfg);
	mutex_unlock(&priv->spi_lock);
	if (ret)
		return ret;

	*val = (cfg & LORA_REG_MODEM_CONFIG1_CR_MASK) >> LORA_REG_MODEM_CONFIG1_CR_SHIFT;
	return 0;
}

static int sx127x_lora_set_cr(struct lora_phy *phy, u8 val)
{
	struct net_device *netdev = dev_get_drvdata(phy->dev);
	struct sx127x_priv *priv = netdev_priv(netdev);
	unsigned int cfg;
	bool lora;
	int ret;

	ret = sx127x_is_lora(priv, &lora);
	if (ret)
		return ret;
	if (!lora)
		return -EBUSY;

	mutex_lock(&priv->spi_lock);

	ret = regmap_read(priv->regmap, LORA_REG_MODEM_CONFIG1, &cfg);
	if (ret)
		goto out;
	cfg &= ~LORA_REG_MODEM_CONFIG1_CR_MASK;
	cfg |= val << LORA_REG_MODEM_CONFIG1_CR_SHIFT;
	ret = regmap_write(priv->regmap, LORA_REG_MODEM_CONFIG1, cfg);

out:
	mutex_unlock(&priv->spi_lock);
	return ret;
}

static int sx127x_lora_get_sync_word(struct lora_phy *phy, u8 *val)
{
	struct net_device *netdev = dev_get_drvdata(phy->dev);
	struct sx127x_priv *priv = netdev_priv(netdev);
	unsigned int sync_word;
	bool lora;
	int ret;

	ret = sx127x_is_lora(priv, &lora);
	if (ret)
		return ret;
	if (!lora)
		return -EBUSY;

	mutex_lock(&priv->spi_lock);
	ret = regmap_read(priv->regmap, LORA_REG_SYNC_WORD, &sync_word);
	mutex_unlock(&priv->spi_lock);
	if (ret)
		return ret;

	*val = sync_word;
	return 0;
}

static int sx127x_lora_set_sync_word(struct lora_phy *phy, u8 val)
{
	struct net_device *netdev = dev_get_drvdata(phy->dev);
	struct sx127x_priv *priv = netdev_priv(netdev);
	bool lora;
	int ret;

	ret = sx127x_is_lora(priv, &lora);
	if (ret)
		return ret;
	if (!lora)
		return -EBUSY;

	mutex_lock(&priv->spi_lock);
	ret = regmap_write(priv->regmap, LORA_REG_SYNC_WORD, val);
	mutex_unlock(&priv->spi_lock);

	return ret;
}

static int sx127x_lora_get_tx_power(struct lora_phy *phy, s32 *val)
{
	struct net_device *netdev = dev_get_drvdata(phy->dev);
	struct sx127x_priv *priv = netdev_priv(netdev);

	return sx127x_get_tx_power(priv, val);
}

static int sx127x_lora_set_tx_power(struct lora_phy *phy, s32 val)
{
	struct net_device *netdev = dev_get_drvdata(phy->dev);
	struct sx127x_priv *priv = netdev_priv(netdev);

	return sx127x_set_tx_power(priv, val);
}

static const struct cfglora_ops sx127x_lora_ops = {
	.get_freq	= sx127x_lora_get_freq,
	.set_freq	= sx127x_lora_set_freq,
	.get_bandwidth	= sx127x_lora_get_bandwidth,
	.set_bandwidth	= sx127x_lora_set_bandwidth,
	.get_sf		= sx127x_lora_get_sf,
	.set_sf		= sx127x_lora_set_sf,
	.get_cr		= sx127x_lora_get_cr,
	.set_cr		= sx127x_lora_set_cr,
	.get_sync_word	= sx127x_lora_get_sync_word,
	.set_sync_word	= sx127x_lora_set_sync_word,
	.get_tx_power	= sx127x_lora_get_tx_power,
	.set_tx_power	= sx127x_lora_set_tx_power,
};

static int sx127x_lora_init(struct lora_phy *phy)
{
	struct net_device *netdev = dev_get_drvdata(phy->dev);
	struct sx127x_priv *priv = netdev_priv(netdev);
	struct spi_device *spi = priv->spi;
	unsigned int val;
	u32 freq_band;
	int ret;

	ret = of_property_read_u32(spi->dev.of_node, "radio-frequency", &freq_band);
	if (ret) {
		dev_err(&spi->dev, "failed reading radio-frequency");
		return ret;
	}

	val = REG_OPMODE_LONG_RANGE_MODE | REG_OPMODE_MODE_SLEEP;
	if (freq_band < 525000000)
		val |= REG_OPMODE_LOW_FREQUENCY_MODE_ON;
	ret = regmap_write(priv->regmap, REG_OPMODE, val);
	if (ret) {
		dev_err(&spi->dev, "failed writing opmode");
		return ret;
	}

	ret = sx127x_lora_set_freq(priv->lora_phy, freq_band);
	if (ret) {
		dev_err(&spi->dev, "failed setting frequency (%d)", ret);
		return ret;
	}

	ret = sx127x_lora_set_tx_power(priv->lora_phy, 14);
	if (ret) {
		dev_err(&spi->dev, "failed setting TX power (%d)", ret);
		return ret;
	}

	return 0;
}

static int sx127x_is_fsk(struct sx127x_priv *priv, bool *val)
{
	unsigned int opmode;
	int ret;

	ret = regmap_read(priv->regmap, REG_OPMODE, &opmode);
	if (ret)
		return ret;

	if (opmode & REG_OPMODE_LONG_RANGE_MODE) {
		*val = false;
	} else {
		unsigned int modtype = opmode & FSKOOK_REG_OPMODE_MODULATION_TYPE_MASK;
		*val = (modtype == FSKOOK_REG_OPMODE_MODULATION_TYPE_FSK);
	}
	return 0;
}

static int sx127x_fsk_get_freq(struct fsk_phy *phy, u32 *val)
{
	struct net_device *netdev = dev_get_drvdata(phy->dev);
	struct sx127x_priv *priv = netdev_priv(netdev);

	return sx127x_get_freq(priv, val);
}

static int sx127x_fsk_set_freq(struct fsk_phy *phy, u32 val)
{
	struct net_device *netdev = dev_get_drvdata(phy->dev);
	struct sx127x_priv *priv = netdev_priv(netdev);

	return sx127x_set_freq(priv, val);
}

static int sx127x_fsk_get_freq_dev(struct fsk_phy *phy, u32 *val)
{
	struct net_device *netdev = dev_get_drvdata(phy->dev);
	struct sx127x_priv *priv = netdev_priv(netdev);
	unsigned long long fdev;
	unsigned int msb, lsb;
	u32 freq_xosc;
	bool fsk;
	int ret;

	ret = sx127x_is_fsk(priv, &fsk);
	if (ret)
		return ret;
	if (!fsk)
		return -EBUSY;

	ret = sx127x_get_xosc_freq(priv, &freq_xosc);
	if (ret)
		return ret;

	mutex_lock(&priv->spi_lock);

	ret = regmap_read(priv->regmap, FSKOOK_REG_FDEV_MSB, &msb);
	if (!ret)
		ret = regmap_read(priv->regmap, FSKOOK_REG_FDEV_LSB, &lsb);

	mutex_unlock(&priv->spi_lock);

	if (ret)
		return ret;

	fdev = (((ulong)msb & FSKOOK_REG_FDEV_MSB_MASK) << 8) | lsb;
	fdev *= freq_xosc;
	do_div(fdev, 1 << 19);
	*val = fdev;

	return 0;
}

static int sx127x_fsk_set_freq_dev(struct fsk_phy *phy, u32 val)
{
	struct net_device *netdev = dev_get_drvdata(phy->dev);
	struct sx127x_priv *priv = netdev_priv(netdev);
	unsigned long long fdev;
	u32 freq_xosc;
	bool fsk;
	int ret;

	ret = sx127x_is_fsk(priv, &fsk);
	if (ret)
		return ret;
	if (!fsk)
		return -EBUSY;

	ret = sx127x_get_xosc_freq(priv, &freq_xosc);
	if (ret)
		return ret;

	fdev = val;
	fdev *= 1 << 19;
	do_div(fdev, freq_xosc);

	mutex_lock(&priv->spi_lock);

	ret = regmap_write(priv->regmap, FSKOOK_REG_FDEV_MSB, (fdev >> 8) & FSKOOK_REG_FDEV_MSB_MASK);
	if (!ret)
		ret = regmap_write(priv->regmap, FSKOOK_REG_FDEV_LSB, fdev & 0xff);

	mutex_unlock(&priv->spi_lock);

	return ret;
}

static int sx127x_fsk_get_tx_power(struct fsk_phy *phy, s32 *val)
{
	struct net_device *netdev = dev_get_drvdata(phy->dev);
	struct sx127x_priv *priv = netdev_priv(netdev);

	return sx127x_get_tx_power(priv, val);
}

static int sx127x_fsk_set_tx_power(struct fsk_phy *phy, s32 val)
{
	struct net_device *netdev = dev_get_drvdata(phy->dev);
	struct sx127x_priv *priv = netdev_priv(netdev);

	return sx127x_set_tx_power(priv, val);
}

static const struct cfgfsk_ops sx127x_fsk_ops = {
	.get_freq	= sx127x_fsk_get_freq,
	.set_freq	= sx127x_fsk_set_freq,
	.get_freq_dev	= sx127x_fsk_get_freq_dev,
	.set_freq_dev	= sx127x_fsk_set_freq_dev,
	.get_tx_power	= sx127x_fsk_get_tx_power,
	.set_tx_power	= sx127x_fsk_set_tx_power,
};

static ssize_t sx127x_state_read(struct file *file, char __user *user_buf,
				 size_t count, loff_t *ppos)
{
	struct net_device *netdev = file->private_data;
	struct sx127x_priv *priv = netdev_priv(netdev);
	ssize_t size;
	char *buf;
	int len = 0;
	int ret;
	unsigned int val;
	bool lora_mode = true;
	const int max_len = 4096;

	buf = kzalloc(max_len, GFP_KERNEL);
	if (!buf)
		return 0;

	mutex_lock(&priv->spi_lock);

	ret = regmap_read(priv->regmap, REG_OPMODE, &val);
	if (!ret) {
		len += snprintf(buf + len, max_len - len, "RegOpMode = 0x%02x\n", val);
		lora_mode = (val & REG_OPMODE_LONG_RANGE_MODE) != 0;
	}

	ret = regmap_read(priv->regmap, REG_FRF_MSB, &val);
	if (!ret)
		len += snprintf(buf + len, max_len - len, "RegFrMsb = 0x%02x\n", val);
	ret = regmap_read(priv->regmap, REG_FRF_MID, &val);
	if (!ret)
		len += snprintf(buf + len, max_len - len, "RegFrMid = 0x%02x\n", val);
	ret = regmap_read(priv->regmap, REG_FRF_LSB, &val);
	if (!ret)
		len += snprintf(buf + len, max_len - len, "RegFrLsb = 0x%02x\n", val);

	ret = regmap_read(priv->regmap, REG_PA_CONFIG, &val);
	if (!ret)
		len += snprintf(buf + len, max_len - len, "RegPaConfig = 0x%02x\n", val);

	if (lora_mode) {
		ret = regmap_read(priv->regmap, LORA_REG_IRQ_FLAGS_MASK, &val);
		if (!ret)
			len += snprintf(buf + len, max_len - len, "RegIrqFlagsMask = 0x%02x\n", val);

		ret = regmap_read(priv->regmap, LORA_REG_IRQ_FLAGS, &val);
		if (!ret)
			len += snprintf(buf + len, max_len - len, "RegIrqFlags = 0x%02x\n", val);

		ret = regmap_read(priv->regmap, LORA_REG_SYNC_WORD, &val);
		if (!ret)
			len += snprintf(buf + len, max_len - len, "RegSyncWord = 0x%02x\n", val);
	}

	ret = regmap_read(priv->regmap, REG_PA_DAC, &val);
	if (!ret)
		len += snprintf(buf + len, max_len - len, "RegPaDac = 0x%02x\n", val);

	mutex_unlock(&priv->spi_lock);

	size = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);

	return size;
}

static const struct file_operations sx127x_state_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = sx127x_state_read,
};

static int sx1272_reset(struct sx127x_priv *priv)
{
	if (!priv->rst)
		return 0;

	gpiod_set_value_cansleep(priv->rst, 0);
	udelay(100);

	gpiod_set_value_cansleep(priv->rst, 1);
	msleep(5);

	return 0;
}

static int sx1276_reset(struct sx127x_priv *priv)
{
	if (!priv->rst)
		return 0;

	gpiod_set_value_cansleep(priv->rst, 1);
	udelay(100);

	gpiod_set_value_cansleep(priv->rst, 0);
	msleep(5);

	return 0;
}

static struct sx127x_model sx1272_model = {
	.number = 1272,
	.version = SX1272_REG_VERSION_V2B,
	.reset = sx1272_reset,
};

static struct sx127x_model sx1276_model = {
	.number = 1276,
	.version = SX1276_REG_VERSION_V1B,
	.reset = sx1276_reset,
};

#ifdef CONFIG_OF
static const struct of_device_id sx127x_dt_ids[] = {
	{ .compatible = "semtech,sx1272", .data = &sx1272_model },
	{ .compatible = "semtech,sx1276", .data = &sx1276_model },
	{}
};
MODULE_DEVICE_TABLE(of, sx127x_dt_ids);
#endif

static int sx127x_probe(struct spi_device *spi)
{
	struct net_device *netdev;
	struct sx127x_priv *priv;
	unsigned int val;
	int ret, i;

	netdev = devm_alloc_loradev(&spi->dev, sizeof(struct sx127x_priv));
	if (!netdev)
		return -ENOMEM;

	netdev->netdev_ops = &sx127x_netdev_ops;
	netdev->flags |= IFF_ECHO;

	priv = netdev_priv(netdev);
	priv->spi = spi;
	mutex_init(&priv->spi_lock);

	priv->regmap = devm_regmap_init_spi(spi, &sx127x_regmap_config);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err(&spi->dev, "regmap allocation failed (%d)\n", ret);
		return ret;
	}

	priv->rst = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_LOW);
	if (priv->rst == NULL)
		dev_warn(&spi->dev, "no reset GPIO available, ignoring");

	for (i = 0; i < 6; i++) {
		priv->dio[i] = devm_gpiod_get_index_optional(&spi->dev, "dio", i, GPIOD_IN);
		if (priv->dio[i] == NULL)
			dev_dbg(&spi->dev, "DIO%d not available, ignoring", i);
	}

	spi->bits_per_word = 8;
	spi_setup(spi);

	priv->model = of_device_get_match_data(&spi->dev);
	if (priv->model) {
		ret = priv->model->reset(priv);
		if (ret) {
			dev_err(&spi->dev, "reset failed (%d)\n", ret);
			return ret;
		}

		ret = regmap_read(priv->regmap, REG_VERSION, &val);
		if (ret) {
			dev_err(&spi->dev, "version read failed\n");
			return ret;
		}
		if (val != priv->model->version) {
			dev_err(&spi->dev, "unexpected version read: 0x%x\n", val);
			return -EINVAL;
		}
	} else {
		dev_info(&spi->dev, "auto-detecting model\n");
		ret = sx1272_reset(priv);
		if (ret) {
			dev_err(&spi->dev, "SX1272 reset failed (%d)\n", ret);
			return ret;
		}

		ret = regmap_read(priv->regmap, REG_VERSION, &val);
		if (ret) {
			dev_err(&spi->dev, "version read failed\n");
			return ret;
		}

		if (val == sx1272_model.version)
			priv->model = &sx1272_model;
		else {
			ret = sx1276_reset(priv);
			if (ret) {
				dev_err(&spi->dev, "SX1276 reset failed (%d)\n", ret);
				return ret;
			}

			ret = regmap_read(priv->regmap, REG_VERSION, &val);
			if (ret) {
				dev_err(&spi->dev, "version read failed\n");
				return ret;
			}

			if (val == sx1276_model.version)
				priv->model = &sx1276_model;
			else {
				dev_err(&spi->dev, "transceiver not recognized (RegVersion = 0x%02x)\n", val);
				return -EINVAL;
			}
		}
	}

	spi_set_drvdata(spi, netdev);
	SET_NETDEV_DEV(netdev, &spi->dev);

	priv->lora_phy = devm_lora_phy_new(&spi->dev, &sx127x_lora_ops, 0);
	if (!priv->lora_phy)
		return -ENOMEM;

	priv->lora_phy->netdev = netdev;

	if (IS_ENABLED(CONFIG_FSK)) {
		priv->fsk_phy = devm_fsk_phy_new(&spi->dev, &sx127x_fsk_ops, 0);
		if (!priv->fsk_phy)
			return -ENOMEM;

		priv->fsk_phy->netdev = netdev;
	}

	ret = sx127x_lora_init(priv->lora_phy);
	if (ret) {
		dev_err(&spi->dev, "failed LoRa init (%d)", ret);
		return ret;
	}

	ret = lora_phy_register(priv->lora_phy);
	if (ret)
		return ret;

	if (IS_ENABLED(CONFIG_FSK)) {
		ret = fsk_phy_register(priv->fsk_phy);
		if (ret) {
			lora_phy_unregister(priv->lora_phy);
			return ret;
		}
	}

	ret = register_loradev(netdev);
	if (ret) {
		if (IS_ENABLED(CONFIG_FSK))
			fsk_phy_unregister(priv->fsk_phy);
		lora_phy_unregister(priv->lora_phy);
		return ret;
	}

	priv->debugfs = debugfs_create_dir(dev_name(&spi->dev), NULL);
	debugfs_create_file("state", S_IRUGO, priv->debugfs, netdev, &sx127x_state_fops);

	dev_info(&spi->dev, "probed (SX%d)\n", priv->model->number);

	return 0;
}

static int sx127x_remove(struct spi_device *spi)
{
	struct net_device *netdev = spi_get_drvdata(spi);
	struct sx127x_priv *priv = netdev_priv(netdev);

	debugfs_remove_recursive(priv->debugfs);

	unregister_loradev(netdev);

	lora_phy_unregister(priv->lora_phy);

	if (IS_ENABLED(CONFIG_FSK))
		fsk_phy_unregister(priv->fsk_phy);

	dev_info(&spi->dev, "removed\n");

	return 0;
}

static struct spi_driver sx127x_spi_driver = {
	.driver = {
		.name = "sx127x",
		.of_match_table = of_match_ptr(sx127x_dt_ids),
	},
	.probe = sx127x_probe,
	.remove = sx127x_remove,
};

module_spi_driver(sx127x_spi_driver);

MODULE_DESCRIPTION("SX127x SPI driver");
MODULE_AUTHOR("Andreas Färber <afaerber@suse.de>");
MODULE_LICENSE("GPL");
