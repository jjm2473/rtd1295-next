/**
 * ogma_driver.c 
 *
 *  Copyright (c) 2013 Fujitsu Semiconductor Limited.
 *  All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *   
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *   
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * Main source file of OGMA Linux sample driver.
 *
 */

#include <linux/version.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/random.h>
#include <linux/pci.h>
#include <linux/ctype.h>
#include <linux/netdevice.h>
#include <linux/types.h>
#include <linux/bitops.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>                                                       
#include <linux/sizes.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <asm/io.h> 

#include "ogma_driver_global.h"
#include "ogma_internal.h" 

struct ogma_priv {
	void __iomem *base;
	int irq;
	int id;
	u8 mac[ETH_ALEN];
	struct net_device *netdev_p;
	ogma_handle_t handle;
	struct clk *clk[3];
	int gpio_phy_enable;
	int gpio_phy_nrst;
};

static unsigned short tx_desc_num = 128;
static unsigned short rx_desc_num = 128;
static int napi_weight = 64;
extern unsigned int flow_ctrl_start_threshold;
extern unsigned int flow_ctrl_stop_threshold;
extern unsigned short pause_time;

extern int ogma_netdev_open(struct net_device *);
extern void ogma_netdev_stop_sub(ogma_netdev_t *, ogma_bool);
extern void ogma_netdev_get_phy_link_status(
	ogma_netdev_t *ogma_netdev_p,
	unsigned int *link_status_flag_p,
	unsigned int *auto_nego_complete_flag_p,
	unsigned int *latched_link_down_flag_p,
	unsigned int *link_speed_p,
	unsigned int *half_duplex_flag_p
);

static ogma_param_t param;
static ogma_gmac_mode_t prb_gmac_mode;
bool ogma_napi_enable_flag;
bool ogma_gmac_start_flag;
bool susres_napi_enable_flag;
bool susres_gmac_start_flag;
unsigned int susres_link_speed;
unsigned int susres_half_duplex_flag;

static ogma_uint32 scb_top_irq_enable_value = 0;

static ogma_err_t ogma_hw_configure_to_normal(struct ogma_priv *priv)
{

    ogma_err_t ogma_err;
    ogma_normal_mode_hwconf_t ogma_normal_mode_hwconf = {0};

    ogma_normal_mode_hwconf.use_jumbo_pkt_flag = use_jumbo_pkt_flag;
    ogma_normal_mode_hwconf.tx_little_endian_flag = OGMA_TRUE;
    ogma_normal_mode_hwconf.rx_little_endian_flag = OGMA_TRUE;
    if ( ( ogma_err = ogma_configure_normal_mode(priv->handle,
                                                  &ogma_normal_mode_hwconf) )
         != OGMA_ERR_OK) {
        pr_err("ogma_configure_normal_mode() failed\n");
        return ogma_err;
    }
    if ( ( ogma_err = ogma_change_mode_to_normal(
               priv->handle) ) != OGMA_ERR_OK) {
        pr_err("ogma_change_mode_to_normal() failed\n");
        return ogma_err;
    }
    /* Wait Change mode Complete */
    msleep( 2);

    return ogma_err;
}

static ogma_err_t ogma_hw_configure_to_taiki(struct ogma_priv *priv)
{
	ogma_err_t ogma_err;

	ogma_err = ogma_change_mode_to_taiki(priv->handle);
	if (ogma_err != OGMA_ERR_OK) {
		pr_err("ogma_change_mode_to_taiki() failed with error status %d\n", ogma_err);
		return ogma_err;
	}

	/* Wait Change mode to Taiki Complete */
	pfdep_msleep(2);

	/* Clear mode change complete IRQ */
	ogma_err = ogma_clear_mode_trans_comp_irq_status(priv->handle,
		(OGMA_MODE_TRANS_COMP_IRQ_T2N | OGMA_MODE_TRANS_COMP_IRQ_N2T));

	if (ogma_err != OGMA_ERR_OK) {
		pr_err("ogma_clear_mode_trans_comp_irq_status() failed with error status %d\n", ogma_err);
		return ogma_err;
	}

	return ogma_err;
}

static irqreturn_t ogma_irq_handler(int irq, void *dev_id)
{
	ogma_uint32 status;
	struct net_device *netdev_p;
	ogma_netdev_t *ogma_netdev_p;

	netdev_p = (struct net_device *)dev_id;
	ogma_netdev_p = (ogma_netdev_t *)netdev_priv(netdev_p);

	dev_dbg(&netdev_p->dev, "ogma_irq_handler\n");

	status = ogma_get_top_irq_status_non_clear(
					ogma_netdev_p->ogma_handle, OGMA_TRUE);
	if (!status)
		return IRQ_NONE;

	if ((status & (OGMA_TOP_IRQ_REG_NRM_TX | OGMA_TOP_IRQ_REG_NRM_RX))) {
		ogma_disable_top_irq(ogma_netdev_p->ogma_handle,
			     OGMA_TOP_IRQ_REG_NRM_TX | OGMA_TOP_IRQ_REG_NRM_RX);
		napi_schedule(&ogma_netdev_p->napi);
	}

	return IRQ_HANDLED;
}

static int ogma_probe(struct platform_device *pdev)
{
	int err, i, n;
	ogma_err_t ogma_err;
	struct ogma_priv *priv;
	struct resource *res;
	int len;
	const u32 *p;
	const char *cp;
	ogma_ctrl_t *ctrl_p;

	ogma_napi_enable_flag = OGMA_FALSE;
	ogma_gmac_start_flag  = OGMA_FALSE;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	p = of_get_property(pdev->dev.of_node, "id", NULL);
	if (p)
		priv->id = be32_to_cpu(*p);

	memset(&param, 0, sizeof(param));

	param.use_gmac_flag = OGMA_TRUE;
	p = of_get_property(pdev->dev.of_node, "default_mac", &len);
	if (!p || len != (ETH_ALEN * sizeof(int))) {
		dev_err(&pdev->dev, "Missing Mac Address\n");
		goto bail1;
	}
	for (i = 0; i < ETH_ALEN; i++)
		priv->mac[i] = be32_to_cpu(p[i]);

	param.desc_ring_param[OGMA_DESC_RING_ID_NRM_TX].little_endian_flag = OGMA_TRUE;
	param.desc_ring_param[OGMA_DESC_RING_ID_NRM_TX].valid_flag = OGMA_TRUE;
	param.desc_ring_param[OGMA_DESC_RING_ID_NRM_TX].entry_num = tx_desc_num;

	param.desc_ring_param[OGMA_DESC_RING_ID_NRM_RX].little_endian_flag = OGMA_TRUE;
	param.desc_ring_param[OGMA_DESC_RING_ID_NRM_RX].valid_flag = OGMA_TRUE;
	param.desc_ring_param[OGMA_DESC_RING_ID_NRM_RX].entry_num = rx_desc_num;                                           

	param.gmac_config.phy_interface = OGMA_PHY_INTERFACE_RGMII;
	cp = of_get_property(pdev->dev.of_node, "phy-interface", NULL);
	if (cp) {
		if (!strcmp(cp, "GMII"))
			param.gmac_config.phy_interface = OGMA_PHY_INTERFACE_GMII;
		else
			if (!strcmp(cp, "RGMII"))
				param.gmac_config.phy_interface =
							OGMA_PHY_INTERFACE_RGMII;
			else
				if (!strcmp(cp, "RMII"))
					param.gmac_config.phy_interface =
							OGMA_PHY_INTERFACE_RMII;
				else {
					dev_err(&pdev->dev,
			    "phy-interface should be GMII, RGMII or RMII\n");
					goto bail1;
				}
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Missing base resource\n");
		goto bail1;
	}

	priv->base = ioremap_nocache(res->start, res->end - res->start + 1);
	if (!priv->base) {
		dev_err(&pdev->dev, "ioremap_nocache() failed\n");
		err = -EINVAL;
		goto bail1;
	}
	dev_info(&pdev->dev, "pa 0x%lx -> va 0x%p\n",
					(unsigned long)res->start, priv->base);

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	priv->clk[0] = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(priv->clk[0])) {
		dev_err(&pdev->dev, "Failed to get clock\n");
		goto bail2;
	}
	clk_prepare_enable(priv->clk[0]);
	priv->clk[1] = of_clk_get(pdev->dev.of_node, 1);
	if (IS_ERR(priv->clk[1])) {
		dev_err(&pdev->dev, "Failed to get clock\n");
		goto bail3;
	}
	clk_prepare_enable(priv->clk[1]);
	priv->clk[2] = of_clk_get(pdev->dev.of_node, 2);
	if (IS_ERR(priv->clk[2])) {
		dev_err(&pdev->dev, "Failed to get clock\n");
		goto bail3;
	}
	clk_prepare_enable(priv->clk[2]);

#if 0     /* left to SCB firmware to handle */

	p = of_get_property(pdev->dev.of_node, "gpio-phy-enable", &len);
	if (p) {
		priv->gpio_phy_enable = be32_to_cpu(*p);
		err = gpio_request(priv->gpio_phy_enable, "f_taiki-en");
		if (unlikely(err)) {
			dev_err(&pdev->dev, " gpio %d request failed ",
						priv->gpio_phy_enable);
			goto bail3a;
		}
		err = gpio_direction_output(priv->gpio_phy_enable, 1);
		if (unlikely(err)) {
			dev_err(&pdev->dev, "failed to set phy_enable gpio\n");
			goto bail3a;
		}
		msleep(50);
		dev_info(&pdev->dev, "phy-gpio-enable %d\n", priv->gpio_phy_enable);
	} else
		dev_warn(&pdev->dev, "no phy_enable gpio\n");

	p = of_get_property(pdev->dev.of_node, "gpio-phy-nrst", &len);
	if (p) {
		priv->gpio_phy_nrst = be32_to_cpu(*p);
		err = gpio_request(priv->gpio_phy_nrst, "f_taiki-nrst");
		if (unlikely(err)) {
			dev_err(&pdev->dev, " gpio %d request failed ",
						priv->gpio_phy_enable);
			goto bail3a;
		}

		err = gpio_direction_output(priv->gpio_phy_nrst, 0);
		msleep(10);
		if (unlikely(err)) {
			dev_err(&pdev->dev, "failed to set gpio\n");
			goto bail3a;
		}
		err = gpio_direction_output(priv->gpio_phy_nrst, 1);
		msleep(50);
		dev_info(&pdev->dev, "phy-gpio-nrst %d\n", priv->gpio_phy_nrst);
	} else
		dev_warn(&pdev->dev, "no reset gpio\n");
#endif
	p = of_get_property(pdev->dev.of_node, "phy-dev-addr", NULL);
	if (p)
		phy_dev_addr = be32_to_cpu(*p);

	p = of_get_property(pdev->dev.of_node, "phy-status-poll-interval-ms",
									NULL);
	if (p)
		phy_status_poll_interval_ms = be32_to_cpu(*p);

	param.use_jumbo_pkt_flag = 0;
	p = of_get_property(pdev->dev.of_node, "use-jumbo", NULL);
	if (p)
		param.use_jumbo_pkt_flag = !!be32_to_cpu(*p);

	ogma_err = ogma_init(priv->base, &pdev->dev, &param, &priv->handle);
	if (ogma_err != OGMA_ERR_OK) {
		dev_err(&pdev->dev, "ogma_init() failed: %d\n", ogma_err);
		switch (ogma_err) {
		case OGMA_ERR_ALLOC:
			err = -ENOMEM;
			break;
		case OGMA_ERR_NOTAVAIL:
			err = -ENODEV;
			break;
		case OGMA_ERR_BUSY:
			err = -EBUSY;
			break;
		default:
			err = -EINVAL;
		}
		goto bail3a;
	}

	ctrl_p = (ogma_ctrl_t *)priv->handle;
	ctrl_p->gmac_mode.flow_ctrl_start_threshold = flow_ctrl_start_threshold;
	ctrl_p->gmac_mode.flow_ctrl_stop_threshold = flow_ctrl_stop_threshold;
	ctrl_p->gmac_mode.pause_time = pause_time;

	ctrl_p->gmac_mode.half_duplex_flag = 0;
	p = of_get_property(pdev->dev.of_node, "half-duplex", NULL);
	if (p)
		ctrl_p->gmac_mode.half_duplex_flag = !!be32_to_cpu(*p);

	ctrl_p->gmac_mode.flow_ctrl_enable_flag = 0;
	p = of_get_property(pdev->dev.of_node, "flow-control", NULL);
	if (p)
		ctrl_p->gmac_mode.flow_ctrl_enable_flag = !!be32_to_cpu(*p);

	param.use_jumbo_pkt_flag = 0;
	p = of_get_property(pdev->dev.of_node, "use-jumbo", NULL);
	if (p)
		param.use_jumbo_pkt_flag = !!be32_to_cpu(*p);

	ctrl_p->gmac_mode.link_speed = OGMA_PHY_LINK_SPEED_100M;
	cp = of_get_property(pdev->dev.of_node, "link-speed", NULL);
	if (cp) {
		if (!strcmp(cp, "1G"))
			ctrl_p->gmac_mode.link_speed = OGMA_PHY_LINK_SPEED_1G;
		else
			if (!strcmp(cp, "100M"))
				ctrl_p->gmac_mode.link_speed = OGMA_PHY_LINK_SPEED_100M;
			else
				if (!strcmp(cp, "10M"))
					ctrl_p->gmac_mode.link_speed = OGMA_PHY_LINK_SPEED_10M;
				else {
					dev_err(&pdev->dev,
				    "link-speed should be 1G, 100M or 10M\n");
					goto bail4;
				}
	}
	memcpy ((void *)&prb_gmac_mode, (const void*)&ctrl_p->gmac_mode, sizeof(ogma_gmac_mode_t));
	scb_top_irq_enable_value = ogma_get_top_irq_enable(priv->handle);
	susres_link_speed = ctrl_p->gmac_mode.link_speed;
	susres_half_duplex_flag = ctrl_p->gmac_mode.half_duplex_flag;

	ogma_disable_top_irq(priv->handle, scb_top_irq_enable_value);

	if ( (ogma_err =  ogma_hw_configure_to_normal(priv) ) != OGMA_ERR_OK) {
		dev_err(&pdev->dev, "ogma_hw_configure_to_normal() failed with error code %d", ogma_err);
		err = -EINVAL;
		goto bail3b;
	}
	err = ogma_netdev_init(priv->handle, &pdev->dev,
			(const ogma_uint8 *)priv->mac,
			"eth%d", napi_weight,
			tx_desc_num, rx_desc_num, &priv->netdev_p);

	if (err) {
		dev_err(&pdev->dev, "ogma_netdev_init() failed\n");
		ogma_terminate(priv->handle);
		goto bail4a;
	}
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "Missing IRQ resource\n");
		goto bail5;
	}
	priv->irq = res->start;
	priv->netdev_p->irq = priv->irq;
	err = request_irq(priv->irq, ogma_irq_handler, IRQF_SHARED,
					  priv->netdev_p->name, priv->netdev_p);
	if (err) {
		dev_err(&pdev->dev, "request_irq() failed\n");
		goto bail5;
	}
	ogma_enable_top_irq(priv->handle, OGMA_TOP_IRQ_REG_NRM_TX |
						       OGMA_TOP_IRQ_REG_NRM_RX);

	platform_set_drvdata(pdev, priv);

	dev_info(&pdev->dev, "%s initialized\n", priv->netdev_p->name);

	return 0;

bail5:
	ogma_netdev_uninit(priv->netdev_p);
bail4a:
	ogma_terminate(priv->handle);

bail4:
    ogma_hw_configure_to_taiki(priv);

bail3b:
    ogma_enable_top_irq(priv->handle, scb_top_irq_enable_value);

    ogma_terminate(priv->handle);

bail3a:
	if (priv->gpio_phy_enable) {
		err = gpio_direction_output(priv->gpio_phy_enable, 0);
		gpio_free(priv->gpio_phy_enable);
	}
	if (priv->gpio_phy_nrst) {
		err = gpio_direction_output(priv->gpio_phy_nrst, 0);
		gpio_free(priv->gpio_phy_nrst);
	}
bail3:
	for (n = 0; n < 3; n++) {
		clk_disable_unprepare(priv->clk[n]);
		clk_put(priv->clk[n]);
	}
bail2:
	iounmap(priv->base);
bail1:
	kfree(priv);

	dev_err(&pdev->dev, "init failed\n");

	return -EINVAL;
}

static int ogma_remove(struct platform_device *pdev)
{
	struct ogma_priv *priv = platform_get_drvdata(pdev);
	int n;

	ogma_disable_top_irq(priv->handle, OGMA_TOP_IRQ_REG_NRM_TX |
					      OGMA_TOP_IRQ_REG_NRM_RX);

	BUG_ON(ogma_hw_configure_to_taiki(priv) != OGMA_ERR_OK);

	ogma_netdev_uninit(priv->netdev_p);

	synchronize_irq(priv->irq);

	free_irq(priv->irq, priv->netdev_p);

	ogma_terminate(priv->handle);

	iounmap(priv->base);

	if (priv->gpio_phy_enable) {
		gpio_direction_output(priv->gpio_phy_enable, 0);
		gpio_free(priv->gpio_phy_enable);
	}
	if (priv->gpio_phy_nrst) {
		gpio_direction_output(priv->gpio_phy_nrst, 0);
		gpio_free(priv->gpio_phy_nrst);
	}
	for (n = 0; n < 3; n++) {
		clk_disable_unprepare(priv->clk[n]);
		clk_put(priv->clk[n]);
	}
	kfree(priv);

	return 0;
}


static const struct of_device_id ogma_dt_ids[] = {
	{ .compatible = "fujitsu,f_taiki" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, ogma_dt_ids);

#ifdef CONFIG_PM

static int ogma_pm_suspend (struct device *dev)
{
	struct platform_device *pdev;
	struct ogma_priv *priv;
	struct net_device *ndev;
	ogma_netdev_t *ogma_netdev_p;
	int n;

	pdev = to_platform_device(dev);
	priv = platform_get_drvdata(pdev);
	ndev = priv->netdev_p;
	ogma_netdev_p = (ogma_netdev_t *)netdev_priv (ndev);

	susres_napi_enable_flag = ogma_napi_enable_flag;
	susres_gmac_start_flag  = ogma_gmac_start_flag;

	ogma_disable_top_irq( priv->handle, OGMA_TOP_IRQ_REG_NRM_TX | OGMA_TOP_IRQ_REG_NRM_RX);

	if (susres_napi_enable_flag == OGMA_TRUE) {
		/* Stop TX queue */
		netif_stop_queue (ndev);
		/* Stop All TX queue */
		netif_device_detach (ndev);
		set_bit (__LINK_STATE_PRESENT, &ndev->state);
		/* Carrier Off */
		netif_carrier_off (ndev);

		/*
		 * Tx & Rx DMA Stop
		 * PHY kthread stop : prev status flag is initialize
		 * Tx & Rx desc_ring stop
		 * napi disable
		 */
		if (susres_gmac_start_flag == OGMA_TRUE) {
			ogma_netdev_stop_sub (ogma_netdev_p, OGMA_TRUE);
		} else {
			ogma_netdev_stop_sub (ogma_netdev_p, OGMA_FALSE);
		}

	} else {
		/* nothing to do */
	}

	synchronize_irq (priv->irq);

	free_irq (priv->irq, priv->netdev_p);

	/* desc_ring/packet buff/dma/ogma_handle(ctrl_p) is free */
	ogma_terminate (priv->handle);

	for(n = 0; n < 3; n++) {
		clk_disable_unprepare (priv->clk[n]);
		clk_put (priv->clk[n]);
	}

	return 0;
}

static int ogma_pm_resume (struct device *dev)
{
	struct platform_device *pdev;
	int err;
	ogma_err_t ogma_err;
	struct ogma_priv *priv;
	ogma_ctrl_t *ctrl_p;
	unsigned int link_status_flag = 0;
	unsigned int auto_nego_complete_flag = 0;
	unsigned int latched_link_down_flag = 0;
	unsigned int link_speed = OGMA_PHY_LINK_SPEED_10M;
	unsigned int half_duplex_flag = 0;
	ogma_gmac_mode_t ogma_gmac_mode;
	struct net_device *ndev;
	ogma_netdev_t *ogma_netdev_p;
	struct resource *res;

	pdev = to_platform_device(dev);
	priv = platform_get_drvdata(pdev);
	ndev = priv->netdev_p;
	ogma_netdev_p = (ogma_netdev_t *)netdev_priv (ndev);

	if ((susres_napi_enable_flag == OGMA_FALSE) &&
		(susres_gmac_start_flag  == OGMA_TRUE)) {
		dev_err(&pdev->dev, "State ABNORMAL\n");
		goto err_route1;
	}

	/* clk_enable */
	priv->clk[0] = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(priv->clk[0])) {
		dev_err(&pdev->dev, "Failed to get clock\n");
		goto err_route1;
	}
	clk_prepare_enable(priv->clk[0]);

	priv->clk[1] = of_clk_get(pdev->dev.of_node, 1);
	if (IS_ERR(priv->clk[1])) {
		dev_err(&pdev->dev, "Failed to get clock\n");
		goto err_route1;
	}
	clk_prepare_enable(priv->clk[1]);

	priv->clk[2] = of_clk_get(pdev->dev.of_node, 2);
	if (IS_ERR(priv->clk[2])) {
		dev_err(&pdev->dev, "Failed to get clock\n");
		goto err_route1;
	}
	clk_prepare_enable(priv->clk[2]);

	/* allocate ctrl_p, desc_ring memory allocate */
	ogma_err = ogma_init(priv->base, &pdev->dev, &param, &priv->handle);
	if (ogma_err != OGMA_ERR_OK) {
		dev_err(&pdev->dev, "ogma_init() failed: %d\n", ogma_err);
		switch (ogma_err) {
		case OGMA_ERR_ALLOC:
			err = -ENOMEM;
			break;
		case OGMA_ERR_NOTAVAIL:
			err = -ENODEV;
			break;
		case OGMA_ERR_BUSY:
			err = -EBUSY;
			break;
		default:
			err = -EINVAL;
		}
		goto err_route1;
	}

	ctrl_p = (ogma_ctrl_t *)priv->handle;
	memcpy((void *)&ctrl_p->gmac_mode, (const void*)&prb_gmac_mode,sizeof(ogma_gmac_mode_t));
	ogma_netdev_p->ogma_handle = ctrl_p;

	ogma_disable_top_irq(priv->handle, scb_top_irq_enable_value);

	/* Get PHY Link Status */
	/* LinkUP & auto negotiation complete? */
	ogma_netdev_get_phy_link_status ( ogma_netdev_p,
										&link_status_flag,
										&auto_nego_complete_flag,
										&latched_link_down_flag,
										&link_speed,
										&half_duplex_flag );
	if ( (link_status_flag != 1) || (auto_nego_complete_flag != 1)) {
		link_speed = susres_link_speed;
		half_duplex_flag = susres_half_duplex_flag;
	}

	/* DMA Initialize */
	memset(&ogma_gmac_mode, 0, sizeof(ogma_gmac_mode_t));

	ogma_gmac_mode.link_speed = link_speed;
	ogma_gmac_mode.half_duplex_flag = (ogma_bool)half_duplex_flag;
	ogma_gmac_mode.flow_ctrl_enable_flag = (ogma_bool)flow_ctrl;
	ogma_gmac_mode.flow_ctrl_start_threshold = (ogma_uint16)flow_ctrl_start_threshold;
	ogma_gmac_mode.flow_ctrl_stop_threshold = (ogma_uint16)flow_ctrl_stop_threshold;

	ogma_err = ogma_set_gmac_mode (ogma_netdev_p->ogma_handle, &ogma_gmac_mode);
	if (ogma_err != OGMA_ERR_OK) {
		dev_err(&pdev->dev, "ogma_set_gmac() failed with error status %d\n", ogma_err);
		goto err_route1;
	}

	/* MAC Initialize sequence */
	ogma_err = ogma_hw_configure_to_normal (priv);
	if (ogma_err != OGMA_ERR_OK) {
		dev_err(&pdev->dev, "ogma_hw_configure_to_normal() failed with error code %d\n", ogma_err);
		err = -EINVAL;
		goto err_route1;
	}

	ogma_netdev_p->ogma_handle = ctrl_p;
	ogma_netdev_p->rx_cksum_offload_flag = OGMA_TRUE;
	spin_lock_init(&ogma_netdev_p->tx_queue_lock);
	ogma_netdev_p->tx_desc_num = tx_desc_num;
	ogma_netdev_p->rx_desc_num = rx_desc_num;

	/* Rx IRQ coalesce is disabled by default. */
	ogma_netdev_p->rxint_tmr_cnt_us = 0;
	ogma_netdev_p->rxint_pktcnt = 1;

	/* Tx_empty IRQ suppression is effective by default. */
	ogma_netdev_p->tx_empty_irq_activation_threshold = tx_desc_num - 2;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev, "Missing IRQ resource\n");
		goto err_route1;
	}

	priv->irq = res->start;
	priv->netdev_p->irq = priv->irq;
	err = request_irq(priv->irq, ogma_irq_handler, IRQF_SHARED,
						priv->netdev_p->name, priv->netdev_p);
	if (err) {
		dev_err(&pdev->dev, "request_irq() failed\n");
		goto err_route1;
	}

	if (susres_napi_enable_flag == OGMA_TRUE) {
		/*
		 * ogma_netdev_open
		 *  - napi enable
		 *  - start RX desc_ring
		 *  - set IRQ coalesce param
		 *  - start TX desc_ring
		 *  - TX desc_ring IRQ disable
		 */
		err = ogma_netdev_open (ndev);
		if (err != 0) {
			dev_err(&pdev->dev, "ogma_netdev_open() failed\n");
			goto err_route1;
		}

	} else {
		/* nothing to do */
	}

	ogma_enable_top_irq(priv->handle, OGMA_TOP_IRQ_REG_NRM_TX |
						OGMA_TOP_IRQ_REG_NRM_RX);

	return 0;

err_route1:
	dev_err(&pdev->dev, "resume process failed\n");
	return 0;
}

static int ogma_runtime_suspend (struct device *dev)
{
	return ogma_pm_suspend (dev);
}

static int ogma_runtime_resume (struct device *dev)
{
	return ogma_pm_resume (dev);
}

static const struct dev_pm_ops ogma_pm_ops = {
	.suspend = ogma_pm_suspend,
	.resume  = ogma_pm_resume,
#ifdef CONFIG_PM_RUNTIME
	SET_RUNTIME_PM_OPS(ogma_runtime_suspend,ogma_runtime_resume,NULL)
#endif/* ifdef CONFIG_PM_RUNTIME */
};
#endif/* ifdef CONFIG_PM */

static struct platform_driver ogma_driver = {
	.probe = ogma_probe,
	.remove = ogma_remove,
	.driver = {
		.name = "f_taiki",
		.of_match_table = ogma_dt_ids,
#ifdef CONFIG_PM
		.pm = &ogma_pm_ops,
#endif/* ifdef CONFIG_PM */
	},
};

static int __init ogma_module_init(void)
{
	return platform_driver_register(&ogma_driver);
}

static void __exit ogma_module_clean(void)
{
	platform_driver_unregister(&ogma_driver);
}


module_init(ogma_module_init);
module_exit(ogma_module_clean);

MODULE_AUTHOR("FUJITSU SEMICONDUCTOR LIMITED");
MODULE_DESCRIPTION("OGMA sample driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

MODULE_ALIAS("platform:f_taiki");


