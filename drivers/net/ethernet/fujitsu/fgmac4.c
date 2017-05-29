/*
 * linux/drivers/net/fgmac4.c
 *
 * Copyright (C) FUJITSU LIMITED 2007-2008
 * Copyright (C) FUJITSU ELECTRONICS INC. 2011. All rights reserved.
 * Copyright (C) 2011 FUJITSU SEMICONDUCTOR LIMITED
 *
 * Derived from f_mac3h driver.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#define DRIVER_NAME		"fgmac4"
#define DRIVER_VERSION		"1.3.0"

#define DEBUG 1
#undef DEBUG			/* enable dev_dbg() */

#define FGMAC4_DEBUG
#undef FGMAC4_DEBUG		/* verbose messages */

#define FGMAC4_DUMP
#undef FGMAC4_DUMP		/* verbose dump info */

/* debug phy registers */
#define FGMAC4_PHY_DEBUG
#undef FGMAC4_PHY_DEBUG		/* verbose phy messages */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/compiler.h>
#include <linux/rtnetlink.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/ip.h>
#include <linux/icmp.h>
#include <linux/udp.h>
#include <linux/tcp.h>
#include <linux/mdio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <asm/unaligned.h>

#include "fgmac4.h"

/* The driver uses a "RX_COPYBREAK" scheme. Rather than a fixed intermediate
 * receive buffer, this scheme allocates full-sized skbuffs as receive buffers.
 * The value RX_COPYBREAK is used as the copying breakpoint: it is chosen to
 * trade-off the memory wasted by passing the full-sized skbuff to the queue
 * layer for all frames vs. the copying cost of copying a frame to a
 * correctly-sized skbuff.
 *
 * ARM systems perform better by disregarding the bus-master transfer capability
 * of these cards.
 *
 * the explanation of RX_COPYBREAK stolen from 3c59x.c
 */

#ifdef CONFIG_FGMAC4_ANEG
#define RESET_B4_ANEG		1
#undef RESET_B4_ANEG	/* Comment out this line for reset before ANEG. */
#endif /* CONFIG_FGMAC4_ANEG */

/* If you do not want to use auto negotiation, choose
 * the speed(SPEED_1000/SPEED_100/SPEED_10)
 * and duplex mode(DUPLEX_FULL/DUPLEX_HALF) here
 */
#define FGMAC4_FORCE_SPEED	SPEED_100
#define FGMAC4_FORCE_DUPLEX	DUPLEX_FULL

#define RESET_AFTER_PHY_SETUP	1
#undef RESET_AFTER_PHY_SETUP /* Comment out line for reset after phy setup */

/* For some kind of phy some address should be masked.
 * Every bit correspons to an address.
 */
#define PHY_MASK_ADDR	0xFFFFFF01	/* mask bit31:8 & bit0 */

#define FGMAC4_ALLOC_RING 1
#define FGMAC4_FREE_RING FGMAC4_ALLOC_RING

#define FGMAC4_STAT_ADD32(base_addr, mmc_stat_member, reg) \
do {	u32 __val = fgmac4_reg_read(base_addr, reg); \
	mmc_stat_member += (u64)__val; \
} while (0)

#ifdef CONFIG_FGMAC4_NAPI
/* NAPI weight */
static int fgmac4_napi_weight = 32;
module_param(fgmac4_napi_weight, int, S_IRUGO);
MODULE_PARM_DESC(fgmac4_napi_weight,
		"Processing number of packets with NAPI poll function.");
#endif /* CONFIG_FGMAC4_NAPI */

static int eee;
module_param(eee, int, S_IRUGO);
MODULE_PARM_DESC(eee, "Enable/Disable EEE function");

#ifdef CONFIG_FGMAC4_MMC
static const char fgmac4_mmc_strings[][ETH_GSTRING_LEN] = {
	"txoctetcount_gb",
	"txframecount_gb",
	"txbroadcastframes_g",
	"txmulticastframes_g",
	"tx64octets_gb",
	"tx65to127octets_gb",
	"tx256to511octets_gb",
	"tx512to1023octets_gb",
	"tx1024tomaxoctets_gb",
	"txunicastframes_gb",
	"txmulticastframes_gb",
	"txbroadcastframes_gb",
	"txunderflowerror",
	"txsinglecol_g",
	"txmulticol_g",
	"txdeferred",
	"txlatecol",
	"txexesscol",
	"txcarriererrror",
	"txoctetcount_g",
	"txframecount_g",
	"txexecessdef",
	"txpauseframes",
	"txvlanframes_g",
	"rxframecount_gb",
	"rxoctetcount_gb",
	"rxoctetcount_g",
	"rxbroadcastframes_g",
	"rxmulticastframes_g",
	"rxcrcerror",
	"rxallignmenterror",
	"rxrunterror",
	"rxjabbererror",
	"rxundersize_g",
	"rxoversize_g",
	"rx64octets_gb",
	"rx65to127octets_gb",
	"rx128to255octets_gb",
	"rx256to511octets_gb",
	"rx512to1023octets_gb",
	"rx1024tomaxoctets_gb",
	"rxunicastframes_g",
	"rxlengtherror",
	"rxoutofrangetype",
	"rxpauseframes",
	"rxfifooverflow",
	"rxvlanframes_gb",
	"rxwatchdogerror",
	"rxipv4_gd_frms",
	"rxipv4_hdrerr_frms",
	"rxipv4_nopay_frms",
	"rxipv4_frag_frms",
	"rxipv4_udsbl_frms",
	"rxipv6_gd_frms",
	"rxipv6_hdrerr_frms",
	"rxipv6_nopay_frms",
	"rxudp_gd_frms",
	"rxudp_err_frms",
	"rxtcp_gd_frms",
	"rxtcp_err_frms",
	"rxicmp_gd_frms",
	"rxicmp_err_frms",
	"rxipv4_gd_octets",
	"rxipv4_hdrerr_octets",
	"rxipv4_nopay_octets",
	"rxipv4_frag_octets",
	"rxipv4_udsbl_octets",
	"rxipv6_gd_octets",
	"rxipv6_hdrerr_octets",
	"rxipv6_nopay_octets",
	"rxudp_gd_octets",
	"rxudp_err_octets",
	"rxtcp_gd_octets",
	"rxtcp_err_octets",
	"rxicmp_gd_octets",
	"rxicmp_err_octets",
};
#endif /* CONFIG_FGMAC4_MMC */

static u32 fgmac4_reg_read(void __iomem *base_addr, u32 reg)
{
	return __raw_readl(base_addr + reg);
}

static void fgmac4_reg_write(void __iomem *base_addr, u32 reg, u32 val)
{
	__raw_writel(val, base_addr + reg);
}

/**
 * fgmac4_phy_wait -- confirm that PHY is not busy
 * @base_addr: the base address of FGMAC4's registers space.
 *
 * Description: In this function, the driver will confirm whether PHY is
 * not busy.  If PHY does not become free in time, the driver will send
 * warning message to user and continue.
 * Returns 0 on success, negative value on failure
 */
static int fgmac4_phy_wait(void __iomem *base_addr)
{
	u32 wtime;		/* waiting time */
	u32 val;

	/* confirm that PHY is not busy befor write/read GAR and GDR.
	 * We set a waitingtime, if PHY does not become free in time,
	 * we will send warning message to user and continue.
	 */
	wtime = WAIT_TIME;
	while (wtime--) {
		val = __raw_readl(base_addr + FGMAC4_REG_GAR);
		if (!(val & FGMAC4_GAR_GB))
			return 0;
		udelay(DELAY_TIME);
	}

	return -EBUSY;
}

/**
 * fgmac4_phy_read -- read PHY's register
 * @mii_busp: target mii_bus structure
 * @phy_id: the id of phy. range:0-31
 * @reg: phy register address
 *
 * Description: read PHY's register by setting FGMAC4's GAR register and
 * GDR register.
 * Returns register's value on success, negative value on failure
 */

static int fgmac4_phy_read(struct mii_bus *mii_busp, int phy_id, int reg)
{
	struct net_device *ndev = mii_busp->priv;
	struct fgmac4_private *lp = netdev_priv(ndev);
	struct device *dev = &ndev->dev;
	int wval;
	int ret;		/* the return value */

	/* Set the GAR register */
	wval = (phy_id << FGMAC4_GAR_PA_SHIFT)
		| ((reg & FGMAC4_GAR_GR_MASK) << FGMAC4_GAR_GR_SHIFT)
		| FGMAC4_GAR_GW_R
		| ((lp->mdc_clk & FGMAC4_GAR_CR_MASK) << FGMAC4_GAR_CR_SHIFT)
		| FGMAC4_GAR_GB;

	/* MDIO */
	/* Wait until GMII/MII is not busy */
	ret = fgmac4_phy_wait(lp->base_addr);
	if (ret) {
		PMSG(dev, KERN_ERR, "Read PHY Error!\n");
		DBG_FUN_ERREND(dev);
		return ret;
	}

	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_GAR, wval);

	/* Wait until GMII/MII is not busy */
	ret = fgmac4_phy_wait(lp->base_addr);
	if (ret) {
		PMSG(dev, KERN_ERR, "Read PHY Error!\n");
		DBG_FUN_ERREND(dev);
		return ret;
	}

	return fgmac4_reg_read(lp->base_addr, FGMAC4_REG_GDR);
}

/**
 * fgmac4_phy_write -- write PHY's register
 * @mii_busp: target mii_bus structure
 * @phy_id: the id of phy. range:0-31
 * @reg: phy register address
 * @val: written value
 *
 * Description: write PHY's register by setting FGMAC4's GAR register and
 * GDR register.
 * Returns 0 on success, negative value on failure
 */
static int fgmac4_phy_write(struct mii_bus *mii_busp, int phy_id, int reg,
								u16 val)
{
	struct net_device *ndev = mii_busp->priv;
	struct fgmac4_private *lp = netdev_priv(ndev);
	struct device *dev = &ndev->dev;
	int ret;
	u32 wval;

	wval = (phy_id << FGMAC4_GAR_PA_SHIFT)
		| (reg & FGMAC4_GAR_GR_MASK) << FGMAC4_GAR_GR_SHIFT
		| FGMAC4_GAR_GW_W
		| ((lp->mdc_clk & FGMAC4_GAR_CR_MASK) << FGMAC4_GAR_CR_SHIFT)
		| FGMAC4_GAR_GB;

	/* Wait until GMII/MII is not busy */
	ret = fgmac4_phy_wait(lp->base_addr);
	if (ret) {
		PMSG(dev, KERN_ERR, "Write PHY Error!!!\n");
		DBG_FUN_ERREND(dev);
		return ret;
	}

	/* Set the value that is want to be written in PHY register */
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_GDR, val);
	/* Set PHY Register access information */
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_GAR, wval);

	/* Wait until GMII/MII is not busy */
	ret = fgmac4_phy_wait(lp->base_addr);
	if (ret) {
		PMSG(dev, KERN_ERR, "Write PHY Error!!!\n");
		DBG_FUN_ERREND(dev);
		return ret;
	}

	return 0;
}

/**
 * fgmac4_mii_read -- ethool use this function to read PHY's registers
 * @netdev: target net device structure
 * @phy_id: the id of phy. range:0-31
 * @reg: phy register address
 *
 * Description: ethool use this function to read PHY's registers.
 * Returns 0 and above on success, negative value on failure
 */
static int fgmac4_mii_read(struct net_device *netdev, int phy_id, int reg)
{
	int val;

	struct fgmac4_private *lp = netdev_priv(netdev);
	val = fgmac4_phy_read(lp->mdio_bus, phy_id, reg);

	return val;
}

/**
 * fgmac4_mii_write -- ethool use this function to write PHY's registers
 * @netdev: target net device structure
 * @phy_id: the id of phy. range:0-31
 * @reg: phy register address
 * @val: written value
 *
 * Description: ethool use this function to write PHY's registers.
 */
static void fgmac4_mii_write(struct net_device *netdev, int phy_id, int reg,
								int val)
{
	int ret;
	struct fgmac4_private *lp = netdev_priv(netdev);
	ret = fgmac4_phy_write(lp->mdio_bus, phy_id, reg, val);
}

/**
 * fgmac4_change_mtu -- change the MTU of system
 * @netdev: target net device structure
 * @new_mtu: new mtu value
 *
 * Description: change the MTU of system.
 * Returns 0 on success, negative value on failure
 */
static int fgmac4_change_mtu(struct net_device *netdev, int new_mtu)
{
	struct fgmac4_private *lp;
	int ret = 0;		/* the return value */
	int max_mtu = ETH_DATA_LEN;

	lp = netdev_priv(netdev);
	if (lp->rx_csum || (netdev->features & NETIF_F_ALL_CSUM))
		max_mtu = ETH_DATA_LEN;
	else
		max_mtu = ETH_JUMBO_DATA_LEN;
	/* official minimum mtu is 68 octets-rfc791 */
	if (new_mtu < 68 || new_mtu > max_mtu)
		return -EINVAL;

	netdev->mtu = new_mtu;

	return ret;
}

/**
 * fgmac4_set_multicast -- setup multicast function
 * @netdev: target net device structure
 *
 * Description: This function is called when the multicast address list or
 * IFF_XXX is changed.  Supported modes are : Promiscuous, Accept all multicast,
 * Reject all multicast, and multi-addresses filtering.
 */
static void fgmac4_set_multicast(struct net_device *netdev)
{
	unsigned long val;
	struct fgmac4_private *lp;
	unsigned long flags;

	lp = netdev_priv(netdev);

	spin_lock_irqsave(&lp->lock, flags);
	if (netdev->flags & IFF_PROMISC)
		/* Promiscuous mode. */
		val = FGMAC4_MFFR_PR;
	else if (netdev->flags & IFF_ALLMULTI)
		/* Accept all multicast. */
		val = FGMAC4_MFFR_PM;
	else if (netdev_mc_empty(netdev))
		/* Reject all multicast. */
		val = 0;
	else {
		/* Accept one or more multicast(s). */
		struct netdev_hw_addr *ha;
		u32 crc;
		u32 filterh, filterl;
		val = FGMAC4_MFFR_HMC;
		filterh = 0;
		filterl = 0;

		netdev_for_each_mc_addr(ha, netdev) {

			/* calculate the 32-bit CRC for the DA */
			crc = ether_crc(ETH_ALEN, ha->addr);

			/* bitwise reversal */
			crc = ~crc;
			/* Get the most significant 6 bits */
			crc >>= 26;

			/* The most significant bit determines the register to
			 * use (H/L) while the other 5 bits determine the bit
			 * within the register.
			 */

			/* Get the most significant bit(bit 5)
			 * 0x20 == 1 << 5
			 */
			if (crc & 0x20)
				filterh |= 1 << (crc & 0x1f);
			else
				filterl |= 1 << (crc & 0x1f);
		}
		fgmac4_reg_write(lp->base_addr, FGMAC4_REG_MHTRH, filterh);
		fgmac4_reg_write(lp->base_addr, FGMAC4_REG_MHTRL, filterl);
	}

	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_MFFR, val);
	spin_unlock_irqrestore(&lp->lock, flags);
}

/**
 * fgmac4_get_macaddr_locked -- get mac address stored in MAR
 * @netdev: target net device structure
 *
 * Description: get mac address stored in MAR (Not locked).
 */
static void fgmac4_get_macaddr_locked(struct net_device *netdev)
{
	u32 mac_addrh, mac_addrl;
	struct fgmac4_private *lp;

	lp = netdev_priv(netdev);
	mac_addrh = fgmac4_reg_read(lp->base_addr, FGMAC4_REG_MAR0H);
	mac_addrl = fgmac4_reg_read(lp->base_addr, FGMAC4_REG_MAR0L);

	netdev->dev_addr[0] =  mac_addrl & 0xff;
	netdev->dev_addr[1] = (mac_addrl >> 8) & 0xff;
	netdev->dev_addr[2] = (mac_addrl >> 16) & 0xff;
	netdev->dev_addr[3] = (mac_addrl >> 24) & 0xff;
	netdev->dev_addr[4] =  mac_addrh & 0xff;
	netdev->dev_addr[5] = (mac_addrh >> 8) & 0xff;
}

/**
 * fgmac4_set_macaddr_locked -- set mac address to MAR
 * @netdev: target net device structure
 * @mac_addr: mac address
 *
 * Description: set mac address to MAR (Not locked).
 */
static void fgmac4_set_macaddr_locked(struct net_device *netdev,
				unsigned char *mac_addr)
{
	u32 mac_addrh, mac_addrl;
	struct fgmac4_private *lp;

	/* set the generated MAC address in MAC address register */
	mac_addrh = (mac_addr[5] << 8)
		|    mac_addr[4];
	mac_addrl = (mac_addr[3] << 24)
		|   (mac_addr[2] << 16)
		|   (mac_addr[1] << 8)
		|    mac_addr[0];

	lp = netdev_priv(netdev);
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_MAR0H, mac_addrh);
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_MAR0L, mac_addrl);
}

/**
 * fgmac4_set_macaddr -- set mac address to MAR
 * @netdev: target net device structure
 * @addr: mac address
 *
 * Description: set mac address to MAR (locked).
 * Returns 0 on success, negative value on failure
 */
static int fgmac4_set_macaddr(struct net_device *netdev, void *p)
{
	struct fgmac4_private *lp;
	struct sockaddr *addr = p;
	unsigned long flags;

	if (netif_running(netdev))
		return -EBUSY;
	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);
	lp = netdev_priv(netdev);
	spin_lock_irqsave(&lp->lock, flags);
	fgmac4_set_macaddr_locked(netdev, netdev->dev_addr);
	spin_unlock_irqrestore(&lp->lock, flags);

	return 0;
}

/**
 * fgmac4_stop_rxtx -- stop rx engine and tx engine
 * @lp: target FGMAC4 private structure
 *
 * Description: In this function, the driver will stop RX/TX DMA by setting
 * OMR register. If RX/TX engines do not stop in time, the driver will send
 * warning message to user and continue.
 */
static void fgmac4_stop_rxtx(struct fgmac4_private *lp)
{
	u32 val;
	u32 wtime;		/* waiting time */
	u32 i;
	struct device *dev = &lp->dev->dev;

	DBG_FUN_START(dev);

	val = fgmac4_reg_read(lp->base_addr, FGMAC4_REG_OMR);

	/* Stop RX:set 0 to OMR[bit1]; Stop TX:set 0 to OMR[bit13] */
	val &= ~(FGMAC4_OMR_START_TX | FGMAC4_OMR_START_RX);

	/* Send stop command to TX and RX process */
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_OMR, val);

	/* TX/RX process will not stop immediately with stop command, until
	 * they finish the work which has been started.
	 * We set a waitingtime, if TX/RX process do not stop in time, we
	 * will send warning message to user and continue.
	 */
	wtime = WAIT_TIME;
	while (wtime--) {
		val = fgmac4_reg_read(lp->base_addr, FGMAC4_REG_OMR);
		if (!(val & (FGMAC4_OMR_START_TX | FGMAC4_OMR_START_RX))) {
			DBG_PRINT(dev, "TX/RX Stoped.\n");
			goto drop_packet;
		}
		udelay(DELAY_TIME);
	}

	PMSG(dev, KERN_CRIT, "Can not to stop DMA!!!\n");

drop_packet:
	DBG_DUMP(dev, lp->base_addr, REG_DUMP | DESC_DUMP, (int *) lp->tx_ring,
						(int *) lp->rx_ring, 0, 0);

	/* after stop TX/RX, the packets that has not been tx/rx will be
	 * dropped
	 */
	/* Check TX descriptors */
	for (i = 0; i < FGMAC4_TDESC_NUM; i++) {
		if (lp->tx_ring[i].opts1 & FGMAC4_TX_DESC_OWN_BIT) {
			PMSG(dev, KERN_WARNING,
			"The packet of TX descirptor(%d) will be dropped!!!\n",
				i);
			lp->net_stats.tx_dropped++;
		}
	}

	/* Check RX descriptors */
	for (i = 0; i < FGMAC4_RDESC_NUM; i++) {
		if (!(lp->rx_ring[i].opts1 & FGMAC4_RX_DESC_OWN_BIT)) {
			PMSG(dev, KERN_WARNING,
			"The packet of RX descirptor(%d) will be dropped!!!\n",
				i);
			lp->net_stats.rx_dropped++;
		}
	}

	DBG_FUN_END(dev);
}

static void fgmac4_disable_intr_all(struct fgmac4_private *lp)
{
	/* disable all interrupt */
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_IER, 0);
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_IMR,
		(FGMAC4_IMR_LPIIM | FGMAC4_IMR_TSIM | FGMAC4_IMR_RGIM));
	/* Mask all interrupt of MMC */
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_MMC_INTR_MASK_RX,
							0x00FFFFFF);
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_MMC_INTR_MASK_TX,
							0x01FFFFFF);
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_MMC_IPC_INTR_MASK_RX,
							0x3FFF3FFF);
}

static void fgmac4_switch_gmii(struct fgmac4_private *lp)
{
	u32 val;
	/* switch to GMII port */
	val = fgmac4_reg_read(lp->base_addr, FGMAC4_REG_MCR)
			& (~FGMAC4_MCR_MII_PORT);
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_MCR, val);
	udelay(10);
}

/**
 * fgmac4_get_stats_from_mfc -- get RX status by reading MFC register
 * @lp: target FGMAC4 private structure
 *
 * Description: In this function, the driver will read MFC register to fill
 *  net_stats structure.
 */
static void fgmac4_get_stats_from_mfc(struct fgmac4_private *lp)
{
	u32 val;
#ifdef FGMAC4_DEBUG
	struct device *dev = &lp->dev->dev;
#endif /* FGMAC4_DEBUG */

	DBG_FUN_START(dev);

	val = fgmac4_reg_read(lp->base_addr, FGMAC4_REG_MFC);
	/*
	 * Miss error = Number of missed frame by FGMAC4 (MFC[bit27:17])
	 *            + Number of missed frame by HOST (MFC[bit15:0])
	 */
	lp->net_stats.rx_missed_errors += ((val & 0x0000FFFF)	/* NMFH */
				+((val >> 17) & 0x000007FF));	/* NMFF */

	DBG_FUN_END(dev);
}

#ifdef FGMAC4_PHY_DEBUG
/**
 * print_phy_regs --  print PHY registers' value
 *
 * @lp: target FGMAC4 private structure
 * @mii: mii interface pointer
 * Description: In this function, the driver will read PHY registers
 * and print the corresponding values.
 */
static void print_phy_regs(struct fgmac4_private *lp, struct mii_if_info *mii)
{
#define PBUFSIZ 512
	char buf[PBUFSIZ];
	int i, rc, idx;
	u32 d = 0; /* 0 to supress warning */
	struct device *dev = &lp->dev->dev;
	static struct {
		char *name;
		int no;
	} regtbl[] = {
		{
		"Basic Control", 0x00}, {
		" Basic Status", 0x01}, {
		"     PHY ID 1", 0x02}, {
		"     PHY ID 2", 0x03}, {
		"     ANEG ADV", 0x04}, {
		"     ANEG LPA", 0x05}, {
		"     ANEG EXP", 0x06}, {
		"    ANEG NEXT", 0x07}, {
		"      LPN NPA", 0x08}, {
		"1000BT Cntrol", 0x09}, {
		"1000BT Status", 0x0a}, {
		"Extend Status", 0x0f}, {
		"PHY-sp Cntrol", 0x10}, {
		"PHY-sp Status", 0x11}, {
		"     RXER Cnt", 0x15}, {
		" INTR CTRL/ST", 0x1b}, {
		"       LinkMD", 0x1d}, {
		"   PHY CTRL 1", 0x1e}, {
		"   PHY CTRL 2", 0x1f}, {
		NULL, 0x20}
	};

	PMSG(dev, KERN_DEBUG, "PHY Regs:\n");
	for (i = 0; regtbl[i].no < 32; i++) {
		idx = 0;

		d = fgmac4_phy_read(lp->mdio_bus, mii->phy_id, regtbl[i].no);
		if (d < 0)
			PMSG(dev, KERN_ERR, "Failed to read PHY register!!!\n");

		rc = snprintf(buf, PBUFSIZ, "%s(%02x): 0x%04x",
				regtbl[i].name, regtbl[i].no, d);
		idx += rc;
		switch (regtbl[i].no) {
		case 0:	/* Basic Control */
			if (d & (1 << 13)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
							" 100Mbps");
				idx += rc;
			} else if (d & (1 << 6)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
							" 1000Mbps");
				idx += rc;
			} else {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
							"  10Mbps");
				idx += rc;
			}
			if (d & (1 << 8)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
							" Full-duplex");
				idx += rc;
			} else {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
							" Half-duplex");
				idx += rc;
			}
			break;

		case 1:
			if (d & (1 << 2)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
							" Link-up");
				idx += rc;
			} else {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
							" Link-down");
				idx += rc;
			}
			rc = snprintf(&buf[idx], PBUFSIZ - idx, " Capability:");
			idx += rc;
			if (d & (1 << 15)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
							" 100BASE-T4");
				idx += rc;
			}
			if (d & (1 << 14)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
							" 100BASE-TX_Full");
				idx += rc;
			}
			if (d & (1 << 13)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
							" 100BASE-TX_Half");
				idx += rc;
			}
			if (d & (1 << 12)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
							" 10BASE-T_Full");
				idx += rc;
			}
			if (d & (1 << 11)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
							" 10BASE-T_Half");
				idx += rc;
			}
			break;

		case 4:
			if (d & (1 << 9)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
							" 100BASE-T4");
				idx += rc;
			}
			rc = snprintf(&buf[idx], PBUFSIZ - idx,
						" Advertising:");
			idx += rc;
			if (d & (1 << 8)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
						" 100BASE-TX_Full");
				idx += rc;
			}
			if (d & (1 << 7)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
						" 100BASE-TX_Half");
				idx += rc;
			}
			if (d & (1 << 6)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
						" 10BASE-T_Full");
				idx += rc;
			}
			if (d & (1 << 5)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
						" 10BASE-T_Half");
				idx += rc;
			}
			break;

		case 5:
			if (d & (1 << 9)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
							" 100BASE-T4");
				idx += rc;
			}
			if (d & (1 << 8)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
						" 100BASE-TX_Full");
				idx += rc;
			}
			if (d & (1 << 7)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
						" 100BASE-TX_Half");
				idx += rc;
			}
			if (d & (1 << 6)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
						" 10BASE-T_Full");
				idx += rc;
			}
			if (d & (1 << 5)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
						" 10BASE-T_Half");
				idx += rc;
			}
			break;

		case 9:
			rc = snprintf(&buf[idx], PBUFSIZ - idx,
						" Advertising:");
			idx += rc;
			if (d & (1 << 9)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
						" 1000BASE-T_Full");
				idx += rc;
			}
			if (d & (1 << 8)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
						" 1000BASE-T_Half");
				idx += rc;
			}
			break;

		case 0xa:
			rc = snprintf(&buf[idx], PBUFSIZ - idx,
						" Capability:");
			idx += rc;
			if (d & (1 << 11)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
						" 1000BASE-T_Full");
				idx += rc;
			}
			if (d & (1 << 10)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
						" 1000BASE-T_Half");
				idx += rc;
			}
			break;

		case 0x1f:
			if (d & (1 << 7)) {
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
						" ANEG Completed.");
				idx += rc;
			}
			switch ((d >> 2) & 7) {
			case 1:
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
						" 10BASE-T Half-duplex");
				idx += rc;
				break;
			case 2:
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
						" 100BASE-TX Half-duplex");
				idx += rc;
				break;
			case 5:
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
						" 10BASE-T Full-duplex");
				idx += rc;
				break;
			case 6:
				rc = snprintf(&buf[idx], PBUFSIZ - idx,
						" 100BASE-TX Full-duplex");
				idx += rc;
				break;
			}
			break;
		}
		PMSG(dev, KERN_ERR, "%s\n", buf);
	}
}
#endif /* FGMAC4_PHY_DEBUG */

/**
 * fgmac4_phy_prepare -- prepare phy setup parameter
 * @mii: mii info pointer
 * @phy_info: phy info pointer
 *
 * Description:prepare phy setup parameter
 */
static void fgmac4_phy_prepare(struct mii_if_info *mii,
				struct fgmac4_phy_info *phy_info)
{
	mii->mdio_read = fgmac4_mii_read;
	mii->mdio_write = fgmac4_mii_write;
	mii->phy_id_mask = 0x1f;
	mii->reg_num_mask = 0x1f;
	/* setup phy address */
	phy_info->phy_addr = mii->phy_id;

#ifdef CONFIG_FGMAC4_ANEG
	mii->supports_gmii = 1;	/* support GMII */
	mii->force_media = 0;
	mii->advertising = ADVERTISE_10HALF | ADVERTISE_10FULL |
					ADVERTISE_100HALF | ADVERTISE_100FULL;
	mii->full_duplex = 1;
	phy_info->adv = ADVERTISED_10baseT_Half
					| ADVERTISED_10baseT_Full
					| ADVERTISED_100baseT_Half
					| ADVERTISED_100baseT_Full
					| ADVERTISED_1000baseT_Half
					| ADVERTISED_1000baseT_Full;
	phy_info->autoneg = AUTONEG_ENABLE;
#else
	mii->force_media = 1;
	mii->full_duplex = !!FGMAC4_FORCE_DUPLEX == DUPLEX_FULL;

	phy_info->adv = 0;
	mii->supports_gmii = !!FGMAC4_FORCE_SPEED == SPEED_1000;
	phy_info->autoneg = AUTONEG_DISABLE;
#endif /* CONFIG_FGMAC4_ANEG */
	phy_info->duplex = FGMAC4_FORCE_DUPLEX;
	phy_info->speed = FGMAC4_FORCE_SPEED;
}

/**
 * fgmac4_get_adv -- get the phy's advertising
 *    or link partner advertising register's info, but
 *    do not contains the advertising info about 1Gbits
 * @mii: mii info pointer
 * addr: phy register number
 *
 * Description:get the phy's setup of auto negotiation
 * Return the negotiation parameter
 */
static u32 fgmac4_get_adv(struct mii_if_info *mii, u16 addr)
{
	u32 result = 0;
	int advert;

	advert = mii->mdio_read(mii->dev, mii->phy_id, addr);
	if (advert & LPA_LPACK)
		result |= ADVERTISED_Autoneg;
	if (advert & ADVERTISE_10HALF)
		result |= ADVERTISED_10baseT_Half;
	if (advert & ADVERTISE_10FULL)
		result |= ADVERTISED_10baseT_Full;
	if (advert & ADVERTISE_100HALF)
		result |= ADVERTISED_100baseT_Half;
	if (advert & ADVERTISE_100FULL)
		result |= ADVERTISED_100baseT_Full;

	return result;
}

/**
 * fgmac4_update_phyinfo_by_ethtool - update phyinfo based on ethtool cmd
 * @lp: target FGMAC4 private structure
 * @cmd: ethtool cmd pointer
 *
 * Returns 0 for success, negative on error.
 */
static int fgmac4_update_phyinfo_by_ethtool(struct fgmac4_private *lp,
						struct ethtool_cmd *cmd)
{
	struct fgmac4_phy_info *phy_info = &lp->phy_info;
	if (cmd->autoneg != AUTONEG_DISABLE
				&& cmd->autoneg != AUTONEG_ENABLE)
		return -EINVAL;

	if (cmd->autoneg == AUTONEG_ENABLE) {
		if ((cmd->advertising & (ADVERTISED_10baseT_Half |
					ADVERTISED_10baseT_Full |
					ADVERTISED_100baseT_Half |
					ADVERTISED_100baseT_Full |
					ADVERTISED_1000baseT_Half |
					ADVERTISED_1000baseT_Full)) == 0)
			return -EINVAL;

		phy_info->adv = cmd->advertising
					& (ADVERTISED_10baseT_Half |
					ADVERTISED_10baseT_Full |
					ADVERTISED_100baseT_Half |
					ADVERTISED_100baseT_Full |
					ADVERTISED_1000baseT_Half |
					ADVERTISED_1000baseT_Full);
		phy_info->autoneg = AUTONEG_ENABLE;
	} else {
		if (cmd->speed != SPEED_10 &&
			cmd->speed != SPEED_100 &&
			cmd->speed != SPEED_1000)
			return -EINVAL;
		if (cmd->duplex != DUPLEX_HALF
			&& cmd->duplex != DUPLEX_FULL)
			return -EINVAL;
		phy_info->autoneg = AUTONEG_DISABLE;
		phy_info->speed = cmd->speed;
		phy_info->duplex = cmd->duplex;
	}
	return 0;
}

/**
 * fgmac4_update_phyinfo_by_ioctl - update phyinfo based on ioctl cmd
 * @lp: target FGMAC4 private structure
 * @mii_data: mii ioctl data pointer
 * @cmd: ioctl command
 *
 * Returns 0 for success, negative on error.
 */
static int fgmac4_update_phyinfo_by_ioctl(struct fgmac4_private *lp,
						struct mii_ioctl_data *mii_data,
						int cmd)
{
	int ret = 0;
	struct fgmac4_phy_info *phy_info = &lp->phy_info;
	struct mii_if_info *mii_if = &lp->mii;

	switch (cmd) {
	case SIOCGMIIPHY:
		/* fall through */
	case SIOCGMIIREG:
		break;

	case SIOCSMIIREG: {
		u16 val = mii_data->val_in;

		mii_data->phy_id &= mii_if->phy_id_mask;
		mii_data->reg_num &= mii_if->reg_num_mask;

		if (mii_data->phy_id == mii_if->phy_id) {
			switch (mii_data->reg_num) {
			case MII_BMCR: {

				if (val & (BMCR_RESET|BMCR_ANENABLE)) {
					phy_info->adv = ADVERTISED_Autoneg;
					phy_info->autoneg = AUTONEG_ENABLE;
					mii_if->force_media = 0;
				} else {
					phy_info->autoneg = AUTONEG_DISABLE;

					phy_info->speed =
						((val & BMCR_SPEED1000 &&
						(val & BMCR_SPEED100) == 0) ?
							SPEED_1000 :
						(val & BMCR_SPEED100) ?
							SPEED_100 : SPEED_10);
					phy_info->duplex =
						(val & BMCR_FULLDPLX) ?
						DUPLEX_FULL : DUPLEX_HALF;
					mii_if->force_media = 1;
				}
				break;
			}
			case MII_ADVERTISE:
				phy_info->adv &= ~(ADVERTISED_10baseT_Half
						| ADVERTISED_10baseT_Full
						| ADVERTISED_100baseT_Half
						| ADVERTISED_100baseT_Full);

				if (val & ADVERTISE_10HALF)
					phy_info->adv |=
						ADVERTISED_10baseT_Half;
				if (val & ADVERTISE_10FULL)
					phy_info->adv |=
						ADVERTISED_10baseT_Full;
				if (val & ADVERTISE_100HALF)
					phy_info->adv |=
						ADVERTISED_100baseT_Half;
				if (val & ADVERTISE_100FULL)
					phy_info->adv |=
						ADVERTISED_100baseT_Full;
				mii_if->advertising = val;
				break;
			case MII_CTRL1000:
				phy_info->adv &= ~(ADVERTISED_1000baseT_Half
						   | ADVERTISED_1000baseT_Full);
				if (val & ADVERTISE_1000HALF)
					phy_info->adv |=
						ADVERTISED_1000baseT_Half;
				if (val & ADVERTISE_1000FULL)
					phy_info->adv |=
						ADVERTISED_1000baseT_Full;
				break;
			default:
				/* do nothing */
				break;
			}
		}

		break;
	}

	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

/**
 * fgmac4_phy_get - get settings of phy
 * @mii: MII interface
 * @phy_info: requested phy info
 *
 * Returns 0 for success, negative on error.
 */
static int fgmac4_phy_get(struct mii_if_info *mii,
				unsigned int *speed_p, unsigned int *duplex_p)
{
	struct net_device *dev = mii->dev;
	u16 bmcr, bmsr, ctrl1000 = 0, stat1000 = 0;
	u32 nego, local_advertising = 0, lp_advertising = 0;

	bmcr = mii->mdio_read(dev, mii->phy_id, MII_BMCR);
	bmsr = mii->mdio_read(dev, mii->phy_id, MII_BMSR);
	if (mii->supports_gmii) {
		ctrl1000 = mii->mdio_read(dev, mii->phy_id, MII_CTRL1000);
		stat1000 = mii->mdio_read(dev, mii->phy_id, MII_STAT1000);
	}
	if (bmcr & BMCR_ANENABLE) {
		local_advertising |= fgmac4_get_adv(mii, MII_ADVERTISE);
		if (ctrl1000 & ADVERTISE_1000HALF)
			local_advertising |= ADVERTISED_1000baseT_Half;
		if (ctrl1000 & ADVERTISE_1000FULL)
			local_advertising |= ADVERTISED_1000baseT_Full;

		if (bmsr & BMSR_ANEGCOMPLETE) {
			lp_advertising = fgmac4_get_adv(mii, MII_LPA);
			if (stat1000 & LPA_1000HALF)
				lp_advertising |=
					ADVERTISED_1000baseT_Half;
			if (stat1000 & LPA_1000FULL)
				lp_advertising |=
					ADVERTISED_1000baseT_Full;
		} else
			lp_advertising = 0;

		nego = local_advertising & lp_advertising;

		if (nego & (ADVERTISED_1000baseT_Full |
				ADVERTISED_1000baseT_Half)) {
			*speed_p = SPEED_1000;
			*duplex_p = !!(nego & ADVERTISED_1000baseT_Full);
		} else if (nego & (ADVERTISED_100baseT_Full |
				   ADVERTISED_100baseT_Half)) {
			*speed_p = SPEED_100;
			*duplex_p = !!(nego & ADVERTISED_100baseT_Full);
		} else {
			*speed_p = SPEED_10;
			*duplex_p = !!(nego & ADVERTISED_10baseT_Full);
		}
	} else {
		*speed_p = ((bmcr & BMCR_SPEED1000 &&
				(bmcr & BMCR_SPEED100) == 0) ? SPEED_1000 :
				(bmcr & BMCR_SPEED100) ? SPEED_100 : SPEED_10);
		*duplex_p =
			(bmcr & BMCR_FULLDPLX) ? DUPLEX_FULL : DUPLEX_HALF;
	}
	mii->full_duplex = *duplex_p;

	return 0;
}

/**
 * fgmac4_phy_set - set settings that are specified in @phy_info
 * @mii: MII interface
 * @phy_info: requested fgmac4_phy_info
 *
 * Returns 0 for success, negative on error.
 */
static int fgmac4_phy_set(struct mii_if_info *mii,
				struct fgmac4_phy_info *phy_info)
{
	struct net_device *dev = mii->dev;

	if (phy_info->phy_addr != mii->phy_id)
		return -EINVAL;
	if (phy_info->autoneg != AUTONEG_DISABLE
				&& phy_info->autoneg != AUTONEG_ENABLE)
		return -EINVAL;

	if (phy_info->autoneg == AUTONEG_ENABLE) {
		u32 bmcr, advert, tmp;
		u32 advert2 = 0, tmp2 = 0;

		if ((phy_info->adv & (ADVERTISED_10baseT_Half |
					ADVERTISED_10baseT_Full |
					ADVERTISED_100baseT_Half |
					ADVERTISED_100baseT_Full |
					ADVERTISED_1000baseT_Half |
					ADVERTISED_1000baseT_Full)) == 0)
			return -EINVAL;

		/* advertise only what has been requested */ /* read */
		advert = mii->mdio_read(dev, mii->phy_id, MII_ADVERTISE);
		tmp = advert & ~(ADVERTISE_ALL | ADVERTISE_100BASE4);
		if (mii->supports_gmii) {
			advert2 = mii->mdio_read(dev,
						mii->phy_id, MII_CTRL1000);
			tmp2 = advert2
				& ~(ADVERTISE_1000HALF | ADVERTISE_1000FULL);
		}
		if (phy_info->adv & ADVERTISED_10baseT_Half)
			tmp |= ADVERTISE_10HALF;
		if (phy_info->adv & ADVERTISED_10baseT_Full)
			tmp |= ADVERTISE_10FULL;
		if (phy_info->adv & ADVERTISED_100baseT_Half)
			tmp |= ADVERTISE_100HALF;
		if (phy_info->adv & ADVERTISED_100baseT_Full)
			tmp |= ADVERTISE_100FULL;
		if (mii->supports_gmii) {
			if (phy_info->adv & ADVERTISED_1000baseT_Half)
				tmp2 |= ADVERTISE_1000HALF;
			if (phy_info->adv & ADVERTISED_1000baseT_Full)
				tmp2 |= ADVERTISE_1000FULL;
		}
		if (advert != tmp) {
			mii->mdio_write(dev, mii->phy_id, MII_ADVERTISE, tmp);
			mii->advertising = tmp;
		}
		if ((mii->supports_gmii) && (advert2 != tmp2))
			mii->mdio_write(dev, mii->phy_id, MII_CTRL1000, tmp2);

		/* turn on autonegotiation, and force a renegotiate */
		bmcr = mii->mdio_read(dev, mii->phy_id, MII_BMCR);
		bmcr |= (BMCR_ANENABLE | BMCR_ANRESTART);
		mii->mdio_write(dev, mii->phy_id, MII_BMCR, bmcr);

	} else {
		u32 bmcr, tmp;
		if (phy_info->speed != SPEED_10 &&
			phy_info->speed != SPEED_100 &&
			phy_info->speed != SPEED_1000)
			return -EINVAL;
		if (phy_info->duplex != DUPLEX_HALF
			&& phy_info->duplex != DUPLEX_FULL)
			return -EINVAL;
		if ((phy_info->speed == SPEED_1000)
			&& (!mii->supports_gmii))
			return -EINVAL;

		/* turn off auto negotiation, set speed and duplexity */
		bmcr = mii->mdio_read(dev, mii->phy_id, MII_BMCR);
		tmp = bmcr & ~(BMCR_ANENABLE | BMCR_SPEED100 |
				BMCR_SPEED1000 | BMCR_FULLDPLX);
		if (phy_info->speed == SPEED_1000)
			tmp |= BMCR_SPEED1000;
		else if (phy_info->speed == SPEED_100)
			tmp |= BMCR_SPEED100;
		if (phy_info->duplex == DUPLEX_FULL) {
			tmp |= BMCR_FULLDPLX;
			mii->full_duplex = 1;
		} else
			mii->full_duplex = 0;
		if (bmcr != tmp)
			mii->mdio_write(dev, mii->phy_id, MII_BMCR, tmp);
	}
	return 0;
}

#if defined(RESET_B4_ANEG) || defined(RESET_AFTER_PHY_SETUP)
/**
 * fgmac4_phy_reset - reset phy
 * @lp: target FGMAC4 private structure
 *
 * Returns 0 for success, negative on error.
 */
static int fgmac4_phy_reset(struct fgmac4_private *lp)
{
	struct mii_if_info *mii;
	struct device *dev = &lp->dev->dev;
	int ctl;
	u32 wtime;
	int ret;
	mii = &lp->mii;

	/* reset phy */
	ctl = fgmac4_phy_read(lp->mdio_bus, mii->phy_id, MII_BMCR);
	if (ctl < 0) {
		PMSG(dev, KERN_ERR, "Failed to read PHY register!!!\n");
		DBG_FUN_ERREND(dev);
		return -EACCES;
	}
	ctl |= BMCR_RESET;

	ret = fgmac4_phy_write(lp->mdio_bus, mii->phy_id, MII_BMCR, ctl);
	if (ret) {
		PMSG(dev, KERN_ERR, "Failed to write PHY register!!!\n");
		DBG_FUN_ERREND(dev);
		return -EACCES;
	}

	/* to sure reset complete in time */
	wtime = WAIT_TIME;
	DBG_PRINT(dev, "Wait start. Time(%d)\n", wtime);
	while (wtime--) {
		ctl = fgmac4_phy_read(lp->mdio_bus, mii->phy_id, MII_BMCR);
		if (ctl < 0) {
			PMSG(dev, KERN_ERR, "Failed to read PHY register!!!\n");
			DBG_FUN_ERREND(dev);
			return -EACCES;
		}
		if (!(ctl & BMCR_RESET)) {
			wtime = 0;
			break;
		}
		udelay(DELAY_TIME);
	}

	if (ctl & BMCR_RESET) {
		PMSG(dev, KERN_ERR, "Failed to setup autonegotiation!!!\n");
		DBG_FUN_ERREND(dev);
		return -EBUSY;
	}
#ifdef FGMAC4_PHY_DEBUG
	PMSG(dev, KERN_DEBUG, "PHY Reset:\n");
	print_phy_regs(lp, mii);
	PMSG(dev, KERN_DEBUG, "mii->advertising: 0x%08x\n", mii->advertising);
#endif /* FGMAC4_PHY_DEBUG */
	return 0;
}
#endif /* RESET_B4_ANEG || RESET_AFTER_PHY_SETUP */

/**
 * fgmac4_phy_aneg_wait - wait completion of phy auto negotiation
 * @lp: target FGMAC4 private structure
 *
 * Returns 0 for success, negative on error.
 */
#ifdef CONFIG_FGMAC4_ANEG
static int fgmac4_phy_aneg_wait(struct fgmac4_private *lp)
{
	struct mii_if_info *mii;
	struct device *dev = &lp->dev->dev;
	int ctl = 0;
	u32 wtime;
	mii = &lp->mii;

	/* sure autonegotiation success */
	wtime = ANEG_WAIT_TIME;
	while (wtime--) {
		ctl = fgmac4_phy_read(lp->mdio_bus, mii->phy_id, MII_BMSR);
		if (ctl < 0) {
			PMSG(dev, KERN_ERR, "Failed to read PHY register!!!\n");
			DBG_FUN_ERREND(dev);
			return -EACCES;
		}

		if ((ctl & BMSR_LSTATUS) && (ctl & BMSR_ANEGCOMPLETE)) {
			wtime = 0;

#ifdef FGMAC4_PHY_DEBUG
			PMSG(dev, KERN_ERR, "ANEG done:\n");
			print_phy_regs(lp, mii);
#endif /* FGMAC4_PHY_DEBUG */

			DBG_FUN_END(dev);
			return 0;
		}
		udelay(DELAY_TIME);
	}

	if (!((ctl & BMSR_LSTATUS) && (ctl & BMSR_ANEGCOMPLETE))) {
		PMSG(dev, KERN_ERR, "Setup Autonegotiation is failed!!!\n");
		DBG_PRINT(dev, "BMSR(0x%08x) Wish(0x%08x)\n", ctl,
					BMSR_LSTATUS | BMSR_ANEGCOMPLETE);
		DBG_FUN_ERREND(dev);
		return -EBUSY;
	}
	return 0;
}
#endif /* CONFIG_FGMAC4_ANEG */

/**
 * fgmac4_phy_setup - setup phy
 * @lp: target FGMAC4 private structure
 *
 * Returns 0 for success, negative on error.
 */
static int fgmac4_phy_setup(struct fgmac4_private *lp)
{
	struct mii_if_info *mii;
	struct device *dev = &lp->dev->dev;
	int ret;

	DBG_FUN_START(dev);

	mii = &lp->mii;

#ifdef RESET_B4_ANEG
	if (!mii->force_media) {
		ret = fgmac4_phy_reset(lp);
		if (unlikely(ret))
			return ret;
	}
#endif /* RESET_B4_ANEG */

	ret = fgmac4_phy_set(mii, &lp->phy_info);
	if (ret) {
		PMSG(dev, KERN_ERR, "Failed to read PHY register!!!\n");
		DBG_FUN_ERREND(dev);
		return ret;
	}

	/* set init_media to inited state*/
	lp->init_media = 1;

#ifdef CONFIG_FGMAC4_ANEG
	if (!mii->force_media) {
		/* wait for completion of auto negotiation */
		ret = fgmac4_phy_aneg_wait(lp);
		if (unlikely(ret))
			return ret;
	}
#endif /* CONFIG_FGMAC4_ANEG */
#ifdef RESET_AFTER_PHY_SETUP
	if (mii->force_media) {
		ret = fgmac4_phy_reset(lp);
		if (unlikely(ret))
			return ret;
	}
#endif /* RESET_AFTER_PHY_SETUP */
	DBG_FUN_END(dev);
	return 0;
}

/**
 * fgmac4_adjust_link - adjust link parameters base on the speed and duplex info
 * @lp: target FGMAC4 private structure
 * @speed: setup speed
 * @duplex: setup duplex
 *
 * Returns 0 for success, negative on error.
 */
static int fgmac4_adjust_link(struct fgmac4_private *lp,
			unsigned int speed, unsigned int duplex)
{
	u32 val = fgmac4_reg_read(lp->base_addr, FGMAC4_REG_MCR);
	if (duplex == DUPLEX_FULL) {
		val |= FGMAC4_MCR_FULL_DUPLEX;
		val &= ~FGMAC4_MCR_IFG_MASK;

		val |= FGMAC4_MCR_IFG_80;
		val &= ~FGMAC4_MCR_DCRS;
	} else {
		val &= ~FGMAC4_MCR_FULL_DUPLEX;
		val &= ~FGMAC4_MCR_IFG_MASK;
		val |= FGMAC4_MCR_IFG_64;
		val |= FGMAC4_MCR_DCRS;
	}
	if (speed == SPEED_1000)	/* GMII Port */
		/* MCR[bit15]: 0 = GMII Port; 1 = MII Port */
		val &= ~FGMAC4_MCR_MII_PORT;
	else	/* MII Port */
		val |= FGMAC4_MCR_MII_PORT;

	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_MCR, val);
	udelay(10);		/* wait 10us */
	return 0;
}

/**
 * fgmac4_phy_init_eee - init and check the EEE feature
 * @lp: target FGMAC4 private structure
 *
 * Description: init and check the Energy-Efficient Ethernet (EEE) feature.
 * Returns 0 on success, negative value on failure
 */
static int fgmac4_phy_init_eee(struct fgmac4_private *lp)
{
	struct mii_bus *bus = lp->mdio_bus;
	struct mii_if_info *mii = &lp->mii;
	unsigned int phy_addr = lp->phy_info.phy_addr;
	int speed, duplex;

	fgmac4_phy_get(mii, &speed, &duplex);

	return phy_init_eee_private(bus, phy_addr, speed, duplex, 1);
}

/**
 * fgmac4_init_eee - init FGMAC4's EEE capability
 * @lp: target FGMAC4 private structure
 * Description: If the phy can manage EEE, then enable the EEE capability,
 *  otherwise, disable it.
 * Returns 0 on success, negative value on failure
 */
static int fgmac4_init_eee(struct fgmac4_private *lp)
{
	int err;
	u32 val;

	/* FIXME: we may need to do something more if timer nego
	 * is useable
	 */
	err = fgmac4_phy_init_eee(lp);
	if (!err)
		lp->eee_capable = 1;
	else {
		val = fgmac4_reg_read(lp->base_addr, FGMAC4_REG_LPICSR);
		val &= ~(FGMAC4_LPICSR_LPITXA | FGMAC4_LPICSR_LPIEN);
		fgmac4_reg_write(lp->base_addr, FGMAC4_REG_LPICSR, val);
		lp->eee_capable = 0;
		lp->eee_enabled = 0;
	}
	return err;
}

/**
 * fgmac4_enable_eee_mode - enable FGMAC4's EEE mode
 * @lp: target FGMAC4 private structure
 * Description: If there is no TX requests and FGMAC4's EEE capability
 *  is useable, and the EEE mode hasn't been enabled, then enable the EEE mode.
 * No return value.
 */
static void fgmac4_enable_eee_mode(struct fgmac4_private *lp)
{
	u32 val;
	/* no TX request */
	if ((lp->tx_head == lp->tx_tail) &&
	    lp->eee_capable &&
	    (!lp->eee_enabled)) {
		val = fgmac4_reg_read(lp->base_addr, FGMAC4_REG_LPICSR);
		val |= FGMAC4_LPICSR_LPITXA
			| FGMAC4_LPICSR_LPIEN;
		fgmac4_reg_write(lp->base_addr, FGMAC4_REG_LPICSR, val);
		lp->eee_enabled = 1;
	}
}

/**
 * fgmac4_adjust_eee_link - modify PLS bit if link state changes
 * @lp: target FGMAC4 private structure
 * @link: link status
 * Description: modify PLS bit if link state changes.
 * No return value.
 */
static void fgmac4_adjust_eee_link(struct fgmac4_private *lp, u32 link)
{
	u32 val;
	if (lp->eee_capable) {
		val = fgmac4_reg_read(lp->base_addr, FGMAC4_REG_LPICSR);
		if (link) {
			val |= FGMAC4_LPICSR_PLS;
			fgmac4_reg_write(lp->base_addr, FGMAC4_REG_LPICSR, val);
		} else {
			val &= ~FGMAC4_LPICSR_PLS;
			fgmac4_reg_write(lp->base_addr, FGMAC4_REG_LPICSR, val);
		}
	}
}


#ifdef CONFIG_FGMAC4_MMC
/**
 * fgmac4_init_mmc - init mmc registers
 * @lp: target FGMAC4 private structure
 *
 * No return value
 */
static void fgmac4_init_mmc(struct fgmac4_private *lp)
{
	u32 val;
	/* Set MMC Register */
	val = FGMAC4_MMC_CNTL_RESET_ON_READ	/* Auto clear after is read */
		| FGMAC4_MMC_CNTL_CLEAR_ALL;	/* Clear all MMC register   */

	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_MMC_CNTL, val);

	/* wait until FGMAC4_MMC_CNTL_CLEAR_ALL bit is cleared */
	while (fgmac4_reg_read(lp->base_addr, FGMAC4_REG_MMC_CNTL)
						& FGMAC4_MMC_CNTL_CLEAR_ALL)
		;

	/* Enable all interrupt of MMC */
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_MMC_INTR_MASK_RX, 0);
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_MMC_INTR_MASK_TX, 0);
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_MMC_IPC_INTR_MASK_RX, 0);
}
#endif /* CONFIG_FGMAC4_MMC */

/**
 * fgmac4_init_hw -- initialize the registers of F_GMAC4
 * @netdev: net_device device structure
 *
 * Description: In this function, the driver will setup autonegotiation and
 * set registers of FGMAC4.
 * Returns 0 on success, negative value on failure
 */
static int fgmac4_init_hw(struct net_device *netdev)
{
	u32 val;
	int err;
	dma_addr_t rx_adr, tx_adr;
	struct fgmac4_private *lp;
	struct device *dev = &netdev->dev;
	unsigned int speed = 0;
	unsigned int duplex = 0;

	DBG_FUN_START(dev);

	lp = netdev_priv(netdev);

	DBG_DUMP(dev, lp->base_addr, REG_DUMP, NULL, NULL, 0, 0);

	/* 1.Set BMR(MDC Bus Mode Register) */
	val = (FGMAC4_BMR_XPBL & SET_1)	/* MAX burst is N-times of PBL       */
		|(FGMAC4_BMR_UPS & SET_1) /* RXburst is RPBL, TXburst is PBL */
		|(FGMAC4_BMR_FB & SET_1) /* AHB Master Burst:Single, INCR    */
#ifdef CONFIG_FGMAC4_ALT_DESC
		|(FGMAC4_BMR_ATDS & SET_1) /* Use alternate descriptor       */
#else
		|(FGMAC4_BMR_ATDS & SET_0) /* Not alternate descriptor       */
#endif
		|(FGMAC4_BMR_DA & SET_1) /* DMA Arbitration Scheme:RX First  */
		|FGMAC4_BMR_RPBL_16	/* RX Burst Length is 16 Bytes       */
		|FGMAC4_BMR_PBL_16	/* TX Burst Length is 16 Bytes       */
		|FGMAC4_BMR_DSL;	/* Descripter Skip Length is 0       */

	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_BMR, val);

	/* clear status register */
	val = fgmac4_reg_read(lp->base_addr, FGMAC4_REG_SR);
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_SR, val);

	/* 2.Set IER(MDC Interrupt Enable Register) */
		/* Eary Receive Int Disable         */
		/* Fatal Bus Error Int Enable       */
		/* Eary Transmit Int Disable        */
		/* RX Watchdog Timeout Int Disable  */
		/* Receive Process Stop Int Enable  */
		/* RX Buffer Unaballable Int Enable */
		/* Receive Int Enable               */
		/* Transmit Underflow Int Enable    */
		/* Receive Overlow Int Enable       */
		/* TX Jabber Timeout Int Disable    */
		/* TX Buffer Unaballable Int Enable */
		/* Transmit Process Stop Int Enable */
		/* Transmit Int Enable              */
		/* Normal Interrupt Summary Enable  */
		/* Abnormal Interrupt Summary       */
	val = (FGMAC4_IER_INT_EARY_RX & FGMAC4_IER_INT_MASK)
		|(FGMAC4_IER_INT_BUS_ERR & FGMAC4_IER_INT_ENABLE)
		|(FGMAC4_IER_INT_EARY_TX & FGMAC4_IER_INT_MASK)
		|(FGMAC4_IER_INT_RX_WT & FGMAC4_IER_INT_MASK)
		|(FGMAC4_IER_INT_RX_STOP & FGMAC4_IER_INT_ENABLE)
		|(FGMAC4_IER_INT_RXB_UNAV & FGMAC4_IER_INT_ENABLE)
		|(FGMAC4_IER_INT_RX_INT & FGMAC4_IER_INT_ENABLE)
		|(FGMAC4_IER_INT_TX_UFLOW & FGMAC4_IER_INT_ENABLE)
		|(FGMAC4_IER_INT_RX_OVERF & FGMAC4_IER_INT_ENABLE)
		|(FGMAC4_IER_INT_TX_JT & FGMAC4_IER_INT_MASK)
		|(FGMAC4_IER_INT_TXB_UNAV & FGMAC4_IER_INT_ENABLE)
		|(FGMAC4_IER_INT_TX_STOP & FGMAC4_IER_INT_ENABLE)
		|(FGMAC4_IER_INT_TX_INT & FGMAC4_IER_INT_ENABLE)
		|(FGMAC4_IER_INT_NORMAL & FGMAC4_IER_INT_ENABLE)
		|(FGMAC4_IER_INT_ABNORMAL & FGMAC4_IER_INT_ENABLE);
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_IER, val);

	/* Set PMTR(PMT Register) */
	/* Disable Wake Up Frame | Disable Magic Packet by default */
		/* Wake Up Frame Disable */
	val = (FGMAC4_PMTR_WFE & FGMAC4_IER_INT_MASK)
		/* Magic Packet Disable  */
		|(FGMAC4_PMTR_MPE & FGMAC4_IER_INT_MASK);

	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_PMTR, val);

	/* 3.Set MFFR(MAC Frame Filter Register)
	 *   With this set the LAN can do:
	 *   (1).Receive broadcast frames
	 *   (2).Receive packets that has the same unicast address as MARn
	 */

	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_MFFR, 0);

	/* 4.Set MHTRH(MAC Hash Table Register High)
	 *       MHTRL(MAC Hash Table Register Low)
	 */
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_MHTRH, 0);
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_MHTRL, 0);

	/* switch to GMII port */
	val = fgmac4_reg_read(lp->base_addr, FGMAC4_REG_MCR)
			& (~FGMAC4_MCR_MII_PORT);
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_MCR, val);
	udelay(10);

	/* 5.Setup PHY Autonegotiation */
	if (!lp->init_media) {
		err = fgmac4_phy_setup(lp);
		if (err) {
			PMSG(dev, KERN_ERR, "Failed to setup AutoNeg!!!\n");
			DBG_FUN_ERREND(dev);
			return err;
		}
	}

	/* 6.Set MCR(MAC Configuration Register) */
	val = (FGMAC4_MCR_WD & SET_1)	   /* Disable RX Watchdog timeout     */
		|(FGMAC4_MCR_JD & SET_1)   /* Disable TX Jabber timer         */
		|(FGMAC4_MCR_BE & SET_1)   /* Frame Burst Enable              */
		|(FGMAC4_MCR_JE & SET_1)   /* Jumbo Frame Enable              */
		|(FGMAC4_MCR_DCRS & SET_0) /* Enable Carrier During Trans     */
		|(FGMAC4_MCR_DO & SET_0)   /* Enable Receive Own              */
		|(FGMAC4_MCR_LM & SET_0)   /* Not Loop-back Mode              */

		|(FGMAC4_MCR_DR & SET_0)   /* Enable Retry                    */
		|(FGMAC4_MCR_ACS & SET_0)  /* Not Automatic Pad/CRC Stripping */
		|(FGMAC4_MCR_DC & SET_0)   /* Deferral Check                  */
		|(FGMAC4_MCR_TX_ENABLE & SET_1) /* Enable Transmitter         */
		|(FGMAC4_MCR_RX_ENABLE & SET_1) /* Enable Receiver            */
		|FGMAC4_MCR_BL_00;		/* Back-off Limit is setted 0 */

	/* add for rx checksum offload */
	if (lp->rx_csum)
		val |= (FGMAC4_MCR_IPC & SET_1); /* Checksum Offload Engine */
	else
		val |= (FGMAC4_MCR_IPC & SET_0);

	/* MCR[bit15,11] is seted based on PHY`s setting */
	err = fgmac4_phy_get(&(lp->mii), &speed, &duplex);
	if (err) {
		PMSG(dev, KERN_ERR, "Failed to get setting mode!!!\n");
		return err;
	}

	if (speed == SPEED_1000)
		val &= ~FGMAC4_MCR_MII_PORT;	/* GMII Port */
	else
		val |= FGMAC4_MCR_MII_PORT;	/* MII Port */

	wmb();
	/* Check duplex mode */
	if (duplex == DUPLEX_FULL) {
		val |= FGMAC4_MCR_FULL_DUPLEX;
		val &= ~FGMAC4_MCR_IFG_MASK;
		val |= FGMAC4_MCR_IFG_80;
		val &= ~FGMAC4_MCR_DCRS;
	} else {
		val &= ~FGMAC4_MCR_FULL_DUPLEX;
		val &= ~FGMAC4_MCR_IFG_MASK;
		val |= FGMAC4_MCR_IFG_64;
		val |= FGMAC4_MCR_DCRS;
	}

	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_MCR, val);

	mb();
	/* add wait time after setting MCR over 1.6us */
	udelay(10);		/* wait 10us */

	/* 7.Set RDLAR(MDC Receive Descriptor List Address Register)
	 *       TDLAR(MDC Transmit Descriptor List Address Register)
	 */
	rx_adr = lp->ring_dma;
	tx_adr = lp->ring_dma + (sizeof(struct fgmac4_desc) * FGMAC4_RDESC_NUM);
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_RDLAR, rx_adr);
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_TDLAR, tx_adr);

	/* 8.Set OMR to start RX and TX */
	val = FGMAC4_OMR_START_RX;
	/*
	 * checksum offload is valid
	 * only when tx/rx is on store and forward mode.
	 * Another limitation is that because the FIFO length
	 * of Fgmac4 is 2k only, so on store and forward mode,
	 * the frame size could not exceed this size (2k). That
	 * means checksum offload does not support jumbo
	 * frame.
	 */
	if (lp->rx_csum)
		val |= FGMAC4_OMR_RSF;
	if (netdev->features & NETIF_F_ALL_CSUM)
		val |= FGMAC4_OMR_TSF;

	val |= FGMAC4_OMR_TTC_256B;

	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_OMR, val);

	DBG_DUMP(dev, lp->base_addr, REG_DUMP, NULL, NULL, 0, 0);

	DBG_FUN_END(dev);
	return 0;
}

#ifdef CONFIG_FGMAC4_USE_BOUNCE_BUF
/**
 * fgmac4_setup_tx_descriptors - setup one tx descriptor
 * @lp: target FGMAC4 private structure
 *
 * No return value
 */
static void fgmac4_setup_tx_descriptors(struct fgmac4_private *lp)
{
	int index;

	/* alloc a transmit buffer for every transmit descriptor */
	for (index = 0; index < FGMAC4_TDESC_NUM; index++) {

		lp->tx_skb[index].data = &lp->txbuf_p[index * lp->tx_buf_sz];
		lp->tx_skb[index].mapping = lp->txbuf_dma + index *
								lp->tx_buf_sz;

		lp->tx_ring[index].addr1 = lp->tx_skb[index].mapping;
		if (lp->buff2_cnt)
			lp->tx_ring[index].addr2 = lp->tx_skb[index].mapping
						+ lp->buff1_cnt;
		else
			lp->tx_ring[index].addr2 = 0;

		if (index != FGMAC4_TDESC_NUM - 1)
				lp->tx_ring[index].opts1 =
						lp->tx_ring[index].opts2 = 0;
		else {
#ifdef CONFIG_FGMAC4_ALT_DESC
				lp->tx_ring[index].opts1 =
						FGMAC4_TX_DESC_END_RING;
				lp->tx_ring[index].opts2 = 0;
#else
				lp->tx_ring[FGMAC4_TDESC_NUM - 1].opts2 =
						FGMAC4_TX_DESC_END_RING;
				lp->tx_ring[index].opts1 = 0;
#endif
		}
	}
}

/**
 * fgmac4_alloc_bounce_bufs - alloc the bounce buffer
 * @lp: target FGMAC4 private structure
 *
 * Returns 0 on success, negative value on failure
 */
static int fgmac4_alloc_bounce_bufs(struct fgmac4_private *lp)
{
	int err = 0;
	struct device *dev = &lp->dev->dev;

	/* alloc all Rx buffer */
	lp->rxbuf_p = dma_alloc_coherent(NULL, lp->rx_buf_sz * FGMAC4_RDESC_NUM,
					&lp->rxbuf_dma,
					GFP_KERNEL);
	if (lp->rxbuf_p == NULL) {
		PMSG(dev, KERN_ERR,
			"Failed to alloc Rx buffer!!!\n");
		err = -ENOMEM;
		goto err_out;
	}

	/* alloc all Tx buffer */
	lp->txbuf_p = dma_alloc_coherent(NULL, lp->tx_buf_sz * FGMAC4_TDESC_NUM,
					&lp->txbuf_dma,
					GFP_KERNEL);
	if (lp->txbuf_p == NULL) {
		PMSG(dev, KERN_ERR,
			"Failed to alloc Tx buffer!!!\n");
		dma_free_coherent(NULL, lp->rx_buf_sz * FGMAC4_RDESC_NUM,
				lp->rxbuf_p, lp->rxbuf_dma);
		lp->rxbuf_p = NULL;
		err = -ENOMEM;
	}

err_out:
	return err;

}

/**
 * fgmac4_free_bounce_bufs - free the bounce buffer
 * @lp: target FGMAC4 private structure
 *
 * No return value
 */
static void fgmac4_free_bounce_bufs(struct fgmac4_private *lp)
{
	if (lp->rxbuf_p) {
		dma_free_coherent(NULL, lp->rx_buf_sz * FGMAC4_RDESC_NUM,
				lp->rxbuf_p, lp->rxbuf_dma);
		lp->rxbuf_p = NULL;
	}
	if (lp->txbuf_p) {
		dma_free_coherent(NULL, lp->tx_buf_sz * FGMAC4_TDESC_NUM,
				lp->txbuf_p, lp->txbuf_dma);
		lp->txbuf_p = NULL;
	}
}

#else /* !CONFIG_FGMAC4_USE_BOUNCE_BUF */

/**
 * fgmac4_free_rx_skbs - unmap and free all allocated sk buffers
 * @lp: target FGMAC4 private structure
 *
 * No return value
 */
static void fgmac4_free_rx_skbs(struct fgmac4_private *lp)
{
	int i;

	/* free all of receive skb */
	for (i = 0; i < FGMAC4_RDESC_NUM; i++) {
		if (lp->rx_skb[i].skb) {
			/* delete the mapping of skbuff */
			dma_unmap_single(NULL, lp->rx_skb[i].mapping,
					lp->rx_buf_sz, DMA_FROM_DEVICE);
			/* free buffer space */
			dev_kfree_skb(lp->rx_skb[i].skb);
			lp->rx_skb[i].skb = NULL;
		}
	}
}

static void fgmac4_free_tx_skbs(struct fgmac4_private *lp)
{
	int i;

	/* free the transmit skb that has not been freed */
	for (i = 0; i < FGMAC4_TDESC_NUM; i++) {
		if (lp->tx_skb[i].skb) {
			/* delete the mapping of skbuff */
			dma_unmap_single(NULL, lp->tx_skb[i].mapping,
					lp->tx_skb[i].skb->len, DMA_TO_DEVICE);
			/* free buffer space */
			dev_kfree_skb(lp->tx_skb[i].skb);
			lp->tx_skb[i].skb = NULL;
		}
	}
}

static int fgmac4_alloc_rx_skbs(struct fgmac4_private *lp)
{
	int i;
	struct sk_buff *skb;
	struct device *dev = &lp->dev->dev;

	for (i = 0; i < FGMAC4_RDESC_NUM; i++) {
		skb = dev_alloc_skb(lp->rx_buf_sz);  /* alloc receive buffer */
		if (!skb) {
			PMSG(dev, KERN_ERR,
				"Failed to alloc receive buffer!!!\n");
			/* free all allocated RX skbs */
			fgmac4_free_rx_skbs(lp);

			return -ENOMEM;
		}
		skb_reserve(skb, NET_IP_ALIGN);

		skb->dev = lp->dev;
		lp->rx_skb[i].skb = skb;

		/* set the alloced skb in receive skb array */
		lp->rx_skb[i].mapping = dma_map_single(NULL,
				lp->rx_skb[i].skb->data,
				lp->rx_buf_sz - NET_IP_ALIGN, DMA_FROM_DEVICE);

	}
	return 0;
}

static void fgmac4_free_all_skbs(struct fgmac4_private *lp)
{
	/* free all of receive skbs */
	fgmac4_free_rx_skbs(lp);

	/* free the transmit skbs that have not been freed */
	fgmac4_free_tx_skbs(lp);
}

#endif /* CONFIG_FGMAC4_USE_BOUNCE_BUF */

/**
 * fgmac4_free_ring --clear ring and free the buffer of every descriptor
 * @lp: target FGMAC4 private structure
 *
 * Description: Free the buffer which has not been freed and clear the
 * setting of all descriptors.
 * Returns 0 on success, negative value on failure
 */
static int fgmac4_free_ring(struct fgmac4_private *lp, int need_free)
{
#ifdef FGMAC4_DEBUG
	struct device *dev = &lp->dev->dev;
#endif
	DBG_FUN_START(dev);
	if (lp->rx_ring) {
		memset(lp->tx_ring, 0,
			sizeof(struct fgmac4_desc) * FGMAC4_TDESC_NUM);
#ifdef CONFIG_FGMAC4_ALT_DESC
		lp->tx_ring[FGMAC4_TDESC_NUM - 1].opts1 =
							FGMAC4_TX_DESC_END_RING;
#else
		lp->tx_ring[FGMAC4_TDESC_NUM - 1].opts2 =
							FGMAC4_TX_DESC_END_RING;
#endif
		memset(lp->rx_ring, 0,
			sizeof(struct fgmac4_desc) * FGMAC4_RDESC_NUM);
		lp->rx_ring[FGMAC4_RDESC_NUM - 1].opts2 =
							FGMAC4_RX_DESC_END_RING;

#ifdef CONFIG_FGMAC4_USE_BOUNCE_BUF
		if (need_free)
			fgmac4_free_bounce_bufs(lp);

#else /* !CONFIG_FGMAC4_USE_BOUNCE_BUF */

		if (need_free)
			fgmac4_free_all_skbs(lp);
#endif /* CONFIG_FGMAC4_USE_BOUNCE_BUF */

		/* delete the head and tail flag of descriptor ring */
		lp->tx_head = lp->tx_tail = 0;
		lp->rx_tail = 0;
	}
	DBG_FUN_END(dev);
	return 0;
}

static int fgmac4_setup_rx_descriptors(struct fgmac4_private *lp)
{
	int i;

	/* alloc a receive buffer for every receive descriptor */
	for (i = 0; i < FGMAC4_RDESC_NUM; i++) {

#ifdef CONFIG_FGMAC4_USE_BOUNCE_BUF
		lp->rx_skb[i].data = &lp->rxbuf_p[i * lp->rx_buf_sz];
		lp->rx_skb[i].mapping = lp->rxbuf_dma + i * lp->rx_buf_sz;
#endif /* CONFIG_FGMAC4_USE_BOUNCE_BUF */

		/* set receive descriptor */
		if (i == (FGMAC4_RDESC_NUM - 1)) {
			/* the last one in ring */
			lp->rx_ring[i].opts2 = (FGMAC4_RX_DESC_END_RING
						|(lp->buff2_cnt << TRBS2_SHIFT)
						| lp->buff1_cnt);
		} else {
			lp->rx_ring[i].opts2 = (lp->buff2_cnt << TRBS2_SHIFT)
						| lp->buff1_cnt;
	}
#ifdef CONFIG_FGMAC4_ALT_DESC
		lp->rx_ring[i].ex_status_rsv = 0;
#endif

		lp->rx_ring[i].addr1 = lp->rx_skb[i].mapping;
		if (lp->buff2_cnt)
			lp->rx_ring[i].addr2 = lp->rx_skb[i].mapping
						+ lp->buff1_cnt;
		else
			lp->rx_ring[i].addr2 = 0;
		lp->rx_ring[i].opts1 = FGMAC4_RX_DESC_OWN_BIT; /* set OWN bit */

	}

	return 0;
}

static int fgmac4_init_ring(struct fgmac4_private *lp, int need_alloc)
{
	int err = 0;
	struct device *dev = &lp->dev->dev;

	DBG_FUN_START(dev);

	if (need_alloc) {
#ifdef CONFIG_FGMAC4_USE_BOUNCE_BUF
		err = fgmac4_alloc_bounce_bufs(lp);
#else /* !CONFIG_FGMAC4_USE_BOUNCE_BUF */
		err = fgmac4_alloc_rx_skbs(lp);
#endif /* CONFIG_FGMAC4_USE_BOUNCE_BUF */
		if (unlikely(err)) {
			PMSG(dev, KERN_ERR,
				"Failed to alloc receive buffer!!!\n");
			return err;
		}
	}

#ifdef CONFIG_FGMAC4_USE_BOUNCE_BUF
	fgmac4_setup_tx_descriptors(lp);
#endif
	fgmac4_setup_rx_descriptors(lp);

	DBG_FUN_END(dev);

	return 0;
}

/**
 * fgmac4_ioctl -- performs interface-specific ioctl commands
 * @netdev: target net device structure
 * @rq: the rq pointer points to a kernel-space address that holds a copy
 *      of the structure passed by the user.
 * @cmd: ioctl command
 *
 * Description: In FGMAC4 driver, three commands are supported. SIOCGMIIPHY,
 * SIOCGMIIREG and SIOCSMIIREG.
 * Returns 0 on success, negative value on failure
 */
static int fgmac4_ioctl(struct net_device *netdev, struct ifreq *rq, int cmd)
{
	struct device *dev = &netdev->dev;
	struct fgmac4_private *lp;
	int ret = 0;

	int link_state = 1; /* link on */
	u16 val;
	unsigned long flags;

	DBG_FUN_START(dev);

	lp = netdev_priv(netdev);

	if (!netif_running(netdev)) {
		PMSG(dev, KERN_CRIT, "Device is not running!!!\n");
		DBG_FUN_ERREND(dev);
		return -EINVAL;
	}

	spin_lock_irqsave(&lp->lock, flags);

	switch (cmd) {
	case SIOCGMIIPHY:
	case SIOCGMIIREG:
		ret = generic_mii_ioctl(&lp->mii, if_mii(rq), cmd, NULL);
		break;
	case SIOCSMIIREG:

		if (if_mii(rq)->reg_num == MII_BMCR) {
#ifdef RESET_B4_ANEG
			if (if_mii(rq)->val_in & (BMCR_RESET|BMCR_ANENABLE)) {
				ret = fgmac4_phy_reset(lp);
				if (unlikely(ret)) {
					spin_unlock_irqrestore(&lp->lock,
									flags);
					return ret;
				}
			}
#endif /* RESET_B4_ANEG */
			if (!mii_link_ok(&lp->mii))
				link_state = 0; /* link off */

			val = if_mii(rq)->val_in;
			if (val & (BMCR_RESET|BMCR_ANENABLE))
				fgmac4_switch_gmii(lp);
			else {
				unsigned int tmp_speed;
				tmp_speed = ((val & BMCR_SPEED1000 &&
					(val & BMCR_SPEED100) == 0) ?
						SPEED_1000 :
						(val & BMCR_SPEED100) ?
							SPEED_100 : SPEED_10);
				if (lp->phy_info.speed != tmp_speed)
					fgmac4_switch_gmii(lp);
			}
		}
		fgmac4_update_phyinfo_by_ioctl(lp, if_mii(rq), cmd);

		ret = generic_mii_ioctl(&lp->mii, if_mii(rq), cmd, NULL);
		if (!ret) {
			switch (if_mii(rq)->reg_num) {
			unsigned int speed = 0;
			unsigned int duplex = 0;
			case MII_BMCR:

				/* set init_media to inited state*/
				lp->init_media = 1;

#ifdef CONFIG_FGMAC4_ANEG
				if (!lp->mii.force_media) {
					/* Do not wait if link off */
					if (!link_state) {
						spin_unlock_irqrestore(
							&lp->lock, flags);
						return ret;
					}
					/*
					 * wait for completion of
					 * auto negotiation
					 */
					ret = fgmac4_phy_aneg_wait(lp);
					if (unlikely(ret)) {
						spin_unlock_irqrestore(
							&lp->lock, flags);
						return ret;
					}
				}
#endif /* CONFIG_FGMAC4_ANEG */
#ifdef RESET_AFTER_PHY_SETUP
				if (lp->mii.force_media) {
					ret = fgmac4_phy_reset(lp);
					if (unlikely(ret)) {
						spin_unlock_irqrestore(
							&lp->lock, flags);
						return ret;
					}
				}
#endif /* RESET_AFTER_PHY_SETUP */
				ret = fgmac4_phy_get(&(lp->mii), &speed,
								    &duplex);
				if (ret) {
					PMSG(dev, KERN_ERR,
						"Failed to set device!!!");
					spin_unlock_irqrestore(&lp->lock,
									flags);
					return ret;
				}

				fgmac4_adjust_link(lp, speed, duplex);
				break;
			}
		}
		break;
	default:
		PMSG(dev, KERN_ERR, "Do not support the command(0x%04x)!!!\n",
								cmd);
		ret = -EOPNOTSUPP;
		break;
	}
	spin_unlock_irqrestore(&lp->lock, flags);

	DBG_FUN_END(dev);
	return ret;
}

/**
 * fgmac4_get_drvinfo -- report driver information
 * @netdev: target net device structure
 * @info: ethtool driver information structure
 *
 * Description: set driver information of structure ethtool_drvinfo.
 */
static void fgmac4_get_drvinfo(struct net_device *netdev,
					struct ethtool_drvinfo *info)
{
#ifdef FGMAC4_DEBUG
	struct device *dev = &netdev->dev;
#endif
	DBG_FUN_START(dev);

	strlcpy(info->driver, DRIVER_NAME, sizeof(info->driver));
	strlcpy(info->version, DRIVER_VERSION, sizeof(info->version));
	strlcpy(info->bus_info, dev_name(netdev->dev.parent),
					sizeof(info->bus_info));

	DBG_FUN_END(dev);
}

/**
 * fgmac4_get_settings -- get device-specific settings
 * @netdev: target net device structure
 * @cmd: ethtool command
 *
 * Description: fgmac4_get_settings is passed an &ethtool_cmd to fill in.
 * It returns an negative errno or zero.
 * Returns 0 on success, negative value on failure
 */
static int fgmac4_get_settings(struct net_device *netdev,
				struct ethtool_cmd *cmd)
{
#ifdef FGMAC4_DEBUG
	struct device *dev = &netdev->dev;
#endif
	struct fgmac4_private *lp;
	int ret;
	unsigned long flags;

	DBG_FUN_START(dev);


	lp = netdev_priv(netdev);
	spin_lock_irqsave(&lp->lock, flags);
	ret = mii_ethtool_gset(&lp->mii, cmd);
	spin_unlock_irqrestore(&lp->lock, flags);

	DBG_FUN_END(dev);
	return ret;
}

/**
 * fgmac4_set_settings -- Set device-specific settings
 * @netdev: target net device structure
 * @cmd: ethtool command
 *
 * Description: fgmac4_set_settings is passed an &ethtool_cmd and should
 * attempt to set all the settings this device supports. It returns an
 * error value if something goes wrong (otherwise 0).
 * Returns 0 on success, negative value on failure
 */
static int fgmac4_set_settings(struct net_device *netdev,
				struct ethtool_cmd *cmd)
{
	struct device *dev = &netdev->dev;
	struct fgmac4_private *lp;
	int ret;
	unsigned int speed = 0;
	unsigned int duplex = 0;

	int link_state = 1; /* link on */
	unsigned long flags;

	DBG_FUN_START(dev);

	lp = netdev_priv(netdev);

#ifdef FGMAC4_PHY_DEBUG
	PMSG(dev, KERN_DEBUG, "set_settings():\n");
	print_phy_regs(lp, &lp->mii);
#endif /* FGMAC4_PHY_DEBUG */

#ifndef CONFIG_FGMAC4_ANEG
	if (cmd->autoneg == AUTONEG_ENABLE)
		return -EINVAL;
#endif /* CONFIG_FGMAC4_ANEG */

	spin_lock_irqsave(&lp->lock, flags);
#ifdef CONFIG_FGMAC4_ANEG
	if (cmd->autoneg == AUTONEG_ENABLE) {
#ifdef RESET_B4_ANEG
		ret = fgmac4_phy_reset(lp);
		if (unlikely(ret)) {
			spin_unlock_irqrestore(&lp->lock, flags);
			return ret;
		}
#endif /* RESET_B4_ANEG */
	}
#endif /* CONFIG_FGMAC4_ANEG */

	if (!mii_link_ok(&lp->mii))
		link_state = 0; /* link off */

	if (cmd->autoneg == AUTONEG_ENABLE)
		fgmac4_switch_gmii(lp);
	else
		if (lp->phy_info.speed != cmd->speed)
			fgmac4_switch_gmii(lp);

	ret = fgmac4_update_phyinfo_by_ethtool(lp, cmd);
	if (unlikely(ret)) {
		spin_unlock_irqrestore(&lp->lock, flags);
		return ret;
	}

	ret = mii_ethtool_sset(&lp->mii, cmd);
	/* MCR[bit15,11] is seted based on PHY`s setting */
	if (!ret) {

		/* set init_media to inited state*/
		lp->init_media = 1;

#ifdef CONFIG_FGMAC4_ANEG
		if (!lp->mii.force_media) {
			/* Do not wait if link off */
			if (!link_state) {
				spin_unlock_irqrestore(&lp->lock, flags);
				return ret;
			}
			/* wait for completion of auto negotiation */
			ret = fgmac4_phy_aneg_wait(lp);
			if (unlikely(ret)) {
				spin_unlock_irqrestore(&lp->lock, flags);
				return ret;
			}
		}
#endif /* CONFIG_FGMAC4_ANEG */
#ifdef RESET_AFTER_PHY_SETUP
		if (lp->mii.force_media) {
			ret = fgmac4_phy_reset(lp);
			if (unlikely(ret)) {
				spin_unlock_irqrestore(&lp->lock, flags);
				return ret;
			}
		}
#endif /* RESET_AFTER_PHY_SETUP */

		ret = fgmac4_phy_get(&(lp->mii), &speed, &duplex);
		if (ret) {
			PMSG(dev, KERN_ERR, "Failed to set device!!!");
			spin_unlock_irqrestore(&lp->lock, flags);
			return ret;
		}

		fgmac4_adjust_link(lp, speed, duplex);
		if (eee)
			fgmac4_init_eee(lp);
	} else
		PMSG(dev, KERN_ERR, "Failed to set device!!!");

	spin_unlock_irqrestore(&lp->lock, flags);

#ifdef FGMAC4_PHY_DEBUG
	PMSG(dev, KERN_DEBUG, "set_settings() done:\n");
	print_phy_regs(lp, &lp->mii);
#endif /* FGMAC4_PHY_DEBUG */

	DBG_DUMP(dev, lp->base_addr, REG_DUMP, NULL, NULL, 0, 0);

	DBG_FUN_END(dev);
	return ret;
}

/**
 * fgmac4_nway_reset -- restart autonegotiation
 * @netdev: target net device structure
 *
 * Description: In this function, the driver will restart autonegotiation
 * and set the result of autonegotiation to MCR registers.
 * Returns 0 on success, negative value on failure
 */
#ifdef CONFIG_FGMAC4_ANEG
static int fgmac4_nway_reset(struct net_device *netdev)
{
	struct device *dev = &netdev->dev;
	struct fgmac4_private *lp;
	int ret = 0;
	unsigned int speed = 0;
	unsigned int duplex = 0;
	unsigned long flags;

	DBG_FUN_START(dev);

	lp = netdev_priv(netdev);

#ifdef FGMAC4_PHY_DEBUG
	PMSG(dev, KERN_DEBUG, "nway_reset():\n");
	print_phy_regs(lp, &lp->mii);
#endif /* FGMAC4_PHY_DEBUG */

	spin_lock_irqsave(&lp->lock, flags);

#ifdef RESET_B4_ANEG
	ret = fgmac4_phy_reset(lp);
	if (unlikely(ret)) {
		spin_unlock_irqrestore(&lp->lock, flags);
		return ret;
	}
#endif /* RESET_B4_ANEG */

	if (!mii_link_ok(&lp->mii)) {
		spin_unlock_irqrestore(&lp->lock, flags);
		return -ENOLINK;
	}

	if (!lp->mii.force_media)
		fgmac4_switch_gmii(lp);

	ret = mii_nway_restart(&lp->mii);
	/* MCR[bit15,11] is seted based on PHY`s setting */
	if (!ret) {
		if (!lp->mii.force_media) {
			/* wait for completion of auto negotiation */
			ret = fgmac4_phy_aneg_wait(lp);
			if (unlikely(ret)) {
				spin_unlock_irqrestore(&lp->lock, flags);
				return ret;
			}
		}

		ret = fgmac4_phy_get(&(lp->mii), &speed, &duplex);
		if (ret) {
			spin_unlock_irqrestore(&lp->lock, flags);
			PMSG(dev, KERN_ERR, "Failed to get setting mode!!!");
			return ret;
		}

		fgmac4_adjust_link(lp, speed, duplex);
	} else {
		PMSG(dev, KERN_ERR, "Failed to restart device!!!");
	}

	spin_unlock_irqrestore(&lp->lock, flags);

#ifdef FGMAC4_PHY_DEBUG
	PMSG(dev, KERN_DEBUG, "nway_reset() done:\n");
	print_phy_regs(lp, &lp->mii);
#endif /* FGMAC4_PHY_DEBUG */

	DBG_DUMP(dev, lp->base_addr, REG_DUMP, NULL, NULL, 0, 0);

	DBG_FUN_END(dev);
	return ret;
}
#endif /* CONFIG_FGMAC4_ANEG */

#ifdef CONFIG_FGMAC4_MMC
/**
 * fgmac4_get_sset_count -- get mmc string array size
 * @netdev: target net device structure
 * @sset: ethtool_stringset feature
 *
 * Returns 0 on success, negative value on failure
 */
static int fgmac4_get_sset_count(struct net_device *dev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return ARRAY_SIZE(fgmac4_mmc_strings);
	default:
		return -EOPNOTSUPP;
	}
}

/**
 * fgmac4_get_strings -- get mmc string data
 * @netdev: target net device structure
 * @stringset: ethtool_stringset feature
 * @data: point to memory address copy to
 *
 * No return value
 */
static void fgmac4_get_strings(struct net_device *dev, u32 stringset, u8 *data)
{
	switch (stringset) {
	case ETH_SS_STATS:
		memcpy(data, *fgmac4_mmc_strings, sizeof(fgmac4_mmc_strings));
		break;
	}
}

/**
 * fgmac4_update_mmc -- update mmc statistics info by reading mmc registers
 * @netdev: target net device structure
 *
 * Description: update mmc statistics info by reading mmc registers
 */
static void fgmac4_update_mmc(struct net_device *dev)
{
	struct fgmac4_private *lp = netdev_priv(dev);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.txoctetcount_gb,
		FGMAC4_REG_MMC_TXOTCETCOUNT_GB);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.txframecount_gb,
		FGMAC4_REG_MMC_TXFRAMECOUNT_GB);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.txbroadcastframes_g,
		FGMAC4_REG_MMC_TXBROADCASTFRAMES_G);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.txmulticastframes_g,
		FGMAC4_REG_MMC_TXMULTICASTFRAMES_G);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.tx64octets_gb,
		FGMAC4_REG_MMC_TX64OCTECS_GB);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.tx65to127octets_gb,
		FGMAC4_REG_MMC_TX65TO127OCTETS_GB);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.tx256to511octets_gb,
		FGMAC4_REG_MMC_TX256TO511OCTETS_GB);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.tx512to1023octets_gb,
		FGMAC4_REG_MMC_TX512TO1023OCTETS_GB);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.tx1024tomaxoctets_gb,
		FGMAC4_REG_MMC_TX1024TOMAXOCTETS_GB);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.txunicastframes_gb,
		FGMAC4_REG_MMC_TXUNICASTFRAMES_GB);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.txmulticastframes_gb,
		FGMAC4_REG_MMC_TXMULTICASTFRAMES_GB);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.txbroadcastframes_gb,
		FGMAC4_REG_MMC_TXBROADCASTFRAMES_GB);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.txunderflowerror,
		FGMAC4_REG_MMC_TXUNDERFLOWERROR);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.txsinglecol_g,
		FGMAC4_REG_MMC_TXSINGLECOL_G);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.txmulticol_g,
		FGMAC4_REG_MMC_TXMULTICOL_G);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.txdeferred,
		FGMAC4_REG_MMC_TXDEFERRED);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.txlatecol,
		FGMAC4_REG_MMC_TXLATECOL);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.txexesscol,
		FGMAC4_REG_MMC_TXEXESSCOL);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.txcarriererror,
		FGMAC4_REG_MMC_TXCARRIERERROR);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.txoctetcount_g,
		FGMAC4_REG_MMC_TXOCTETCOUNT_G);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.txframecount_g,
		FGMAC4_REG_MMC_TXFRAMECOUNT_G);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.txexecessdef,
		FGMAC4_REG_MMC_TXEXECESSDEF);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.txpauseframes,
		FGMAC4_REG_MMC_TXPAUSEFRAMES);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.txvlanframes_g,
		FGMAC4_REG_MMC_TXVLANFRAMES_G);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxframecount_gb,
		FGMAC4_REG_MMC_RXFRAMECOUNT_GB);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxoctetcount_gb,
		FGMAC4_REG_MMC_RXOCTETCOUNT_GB);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxoctetcount_g,
		FGMAC4_REG_MMC_RXOCTETCOUNT_G);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxbroadcastframes_g,
		FGMAC4_REG_MMC_RXBROADCASTFRAMES_G);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxmulticastframes_g,
		FGMAC4_REG_MMC_RXMULTICASTFRAMES_G);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxcrcerror,
		FGMAC4_REG_MMC_RXCRCERROR);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxalignmenterror,
		FGMAC4_REG_MMC_RXALIGNMENTERROR);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxrunterror,
		FGMAC4_REG_MMC_RXRUNTERROR);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxjabbererror,
		FGMAC4_REG_MMC_RXJABBERERROR);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxundersize_g,
		FGMAC4_REG_MMC_RXUNDERSIZE_G);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxoversize_g,
		FGMAC4_REG_MMC_RXOVERSIZE_G);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rx64octets_gb,
		FGMAC4_REG_MMC_RX64OCTETS_GB);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rx65to127octets_gb,
		FGMAC4_REG_MMC_RX65TO127OCTETS_GB);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rx128to255octets_gb,
		FGMAC4_REG_MMC_RX128TO255OCTETS_GB);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rx256to511octets_gb,
		FGMAC4_REG_MMC_RX256TO511OCTETS_GB);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rx512to1023octets_gb,
		FGMAC4_REG_MMC_RX512TO1023OCTETS_GB);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rx1024tomaxoctets_gb,
		FGMAC4_REG_MMC_RX1024TOMAXOCTETS_GB);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxunicastframes_g,
		FGMAC4_REG_MMC_RXUNICASTFRAMES_G);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxlengtherror,
		FGMAC4_REG_MMC_RXLENGTHERROR);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxoutofrangetype,
		FGMAC4_REG_MMC_RXOUTOFRANGETYPE);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxpauseframes,
		FGMAC4_REG_MMC_RXPAUSEFRAMES);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxfifooverflow,
		FGMAC4_REG_MMC_RXFIFOOVERFLOW);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxvlanframes_gb,
		FGMAC4_REG_MMC_RXVLANFRAMES_GB);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxwatchdogerror,
		FGMAC4_REG_MMC_RXWATCHDOGERROR);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxipv4_gd_frms,
		FGMAC4_REG_MMC_RXIPV4_GB_FRMS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxipv4_hdrerr_frms,
		FGMAC4_REG_MMC_RXIPV4_HDRERR_FRMS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxipv4_nopay_frms,
		FGMAC4_REG_MMC_RXIPV4_NOPAY_FRMS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxipv4_frag_frms,
		FGMAC4_REG_MMC_RXIPV4_FRAG_FRMS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxipv4_udsbl_frms,
		FGMAC4_REG_MMC_RXIPV4_UDSBL_FRMS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxipv6_gd_frms,
		FGMAC4_REG_MMC_RXIPV6_GB_FRMS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxipv6_hdrerr_frms,
		FGMAC4_REG_MMC_RXIPV6_HDRERR_FRMS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxipv6_nopay_frms,
		FGMAC4_REG_MMC_RXIPV6_NOPAY_FRMS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxudp_gd_frms,
		FGMAC4_REG_MMC_RXUDP_GB_FRMS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxudp_err_frms,
		FGMAC4_REG_MMC_RXUDP_ERR_FRMS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxtcp_gd_frms,
		FGMAC4_REG_MMC_RXTCP_GB_FRMS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxtcp_err_frms,
		FGMAC4_REG_MMC_RXTCP_ERR_FRMS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxicmp_gd_frms,
		FGMAC4_REG_MMC_RXICMP_GB_FRMS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxicmp_err_frms,
		FGMAC4_REG_MMC_RXICMP_ERR_FRMS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxipv4_gd_octets,
		FGMAC4_REG_MMC_RXIPV4_GB_OCTETS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxipv4_hdrerr_octets,
		FGMAC4_REG_MMC_RXIPV4_HDRERR_OCTETS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxipv4_nopay_octets,
		FGMAC4_REG_MMC_RXIPV4_NOPAY_OCTETS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxipv4_frag_octets,
		FGMAC4_REG_MMC_RXIPV4_FRAG_OCTETS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxipv4_udsbl_octets,
		FGMAC4_REG_MMC_RXIPV4_UDSBL_OCTETS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxipv6_gd_octets,
		FGMAC4_REG_MMC_RXIPV6_GB_OCTETS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxipv6_hdrerr_octets,
		FGMAC4_REG_MMC_RXIPV6_HDRERR_OCTETS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxipv6_nopay_octets,
		FGMAC4_REG_MMC_RXIPV6_NOPAY_OCTETS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxudp_gd_octets,
		FGMAC4_REG_MMC_RXUDP_GB_OCTETS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxudp_err_octets,
		FGMAC4_REG_MMC_RXUDP_ERR_OCTETS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxtcp_gd_octets,
		FGMAC4_REG_MMC_RXTCP_GB_OCTETS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxtcp_err_octets,
		FGMAC4_REG_MMC_RXTCP_ERR_OCTETS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxicmp_gd_octets,
		FGMAC4_REG_MMC_RXICMP_GB_OCTETS);
	FGMAC4_STAT_ADD32(lp->base_addr,
		lp->mmc_stats.rxicmp_err_octets,
		FGMAC4_REG_MMC_RXICMP_ERR_OCTETS);
}

/**
 * fgmac4_get_ethtool_stats -- get mmc statistics info
 * @netdev: target net device structure
 * @stats: point to NIC-specific statistics(unused)
 * @data: point to memory address copy to
 *
 * Description: copy mmc statistics info to data
 */
static void fgmac4_get_ethtool_stats(struct net_device *netdev,
				struct ethtool_stats *stats, u64 *data)
{
	struct fgmac4_private *lp = netdev_priv(netdev);
	unsigned long flags;

	spin_lock_irqsave(&lp->lock, flags);

	if (netif_running(netdev)) {
		/* update mmc */
		fgmac4_update_mmc(netdev);
	}
	memcpy(data, &lp->mmc_stats,
		sizeof(lp->mmc_stats));

	spin_unlock_irqrestore(&lp->lock, flags);
}
#endif /* CONFIG_FGMAC4_MMC */

/* Ethtool supported by F_GMAC4 */
static const struct ethtool_ops fgmac4_ethtool_ops = {
	.get_link = ethtool_op_get_link,
	.get_drvinfo = fgmac4_get_drvinfo,
	.get_settings = fgmac4_get_settings,
	.set_settings = fgmac4_set_settings,
#ifdef CONFIG_FGMAC4_ANEG
	.nway_reset = fgmac4_nway_reset,
#endif
#ifdef CONFIG_FGMAC4_MMC
	.get_strings		= fgmac4_get_strings,
	.get_sset_count		= fgmac4_get_sset_count,
	.get_ethtool_stats	= fgmac4_get_ethtool_stats,
#endif /* CONFIG_FGMAC4_MMC */
};

/**
 * fgmac4_check_carrier -- check the status of carrier
 * data: address of target net device structure
 *
 * Description: this function will be called every one second to check
 * the status of carrier.
 */
static void fgmac4_stop_hw(struct fgmac4_private *);
static void fgmac4_check_carrier(unsigned long data)
{
	struct device *dev;
	struct net_device *netdev;
	struct fgmac4_private *lp;
	u32 oldlink, newlink;
	int err;

	unsigned long flags;

	netdev = (struct net_device *) data;
	dev = &netdev->dev;

	lp = netdev_priv(netdev);

	spin_lock_irqsave(&lp->lock, flags);

	/* check the old link state of carrier
	 * the return value of netif_carrier_ok()
	 *  1: carrier on
	 *  0: carrier off
	 */
	oldlink = netif_carrier_ok(netdev);

	/* check the new link state of carrier
	 * the return value of mii_link_ok()
	 *  1: carrier on
	 *  0: carrier off
	 */
	newlink = (unsigned int) mii_link_ok(&lp->mii);

	/* update link state in dev->state */
	if (!oldlink && newlink) {
		/* Link OFF->ON */
		err = fgmac4_init_ring(lp, !FGMAC4_ALLOC_RING);
		if (err) {
			PMSG(dev, KERN_ERR,
				"Failed to initialize DMA descriptor!!!\n");
			DBG_FUN_ERREND(dev);
			goto out;
		}

		err = fgmac4_init_hw(netdev);
		if (err) {
			PMSG(dev, KERN_WARNING,
				"Failed to initialize device!!!\n");
			DBG_FUN_ERREND(dev);
			goto out;
		}

		if (eee) {
			fgmac4_init_eee(lp);
			fgmac4_adjust_eee_link(lp, newlink);
		}

#ifdef FGMAC4_PHY_DEBUG
		PMSG(dev, KERN_DEBUG, "PHY linkup:\n");
		print_phy_regs(lp, &lp->mii);
#endif /* FGMAC4_PHY_DEBUG */
		/* start queue to allow transmit */
		netif_carrier_on(netdev);
		netif_start_queue(netdev);

		PMSG(dev, KERN_NOTICE, "Link UP\n");
	} else if (oldlink && !newlink) {
		/* Link ON->OFF */
		/* switch to GMII port */
		fgmac4_switch_gmii(lp);

		fgmac4_stop_hw(lp);

		if (eee)
			fgmac4_adjust_eee_link(lp, newlink);

		netif_carrier_off(netdev);
		netif_stop_queue(netdev);

		PMSG(dev, KERN_NOTICE, "Link DOWN\n");
	}

	if (eee && newlink)
		fgmac4_enable_eee_mode(lp);

out:
	spin_unlock_irqrestore(&lp->lock, flags);

	/* set the next check */
	mod_timer(&lp->timer, CHECK_CARRIER_TIME);
}

/**
 * fgmac4_rx_csum_is_ok -- Judge whether the rx checksum status is ok
 * status: status that need to be judged
 *
 * Description: Judge whether the rx checksum status is ok. Return 1 if ok, 0
 * if failed.
 */
#ifdef CONFIG_FGMAC4_ALT_DESC
static inline int fgmac4_rx_csum_is_ok(struct fgmac4_desc *ldesc)
{
	if (ldesc == NULL)
		return 0;

	/* Extended Status Available */
	if (ldesc->opts1 & FGMAC4_RX_DESC_ESA) {
		if (!(ldesc->ex_status_rsv &
			((FGMAC4_RX_DESC_IPE_ERR | FGMAC4_RX_DESC_IPHE_ERR))))
			return 1;
		return 0;
	}
	return 1;
}

#else
static inline int fgmac4_rx_csum_is_ok(struct fgmac4_desc *ldesc)
{
	u32 status = ldesc->opts1;
	if ((status & (FGMAC4_RX_DESC_IPC_ERR | FGMAC4_RX_DESC_RMCE_ERR))
	    && (status & FGMAC4_RX_DESC_FT))
		return 0;
	else
		return 1;
}
#endif

/**
 * fgmac4_rx -- this function will be called when there is receive
 * interrupt(SR[bit6]) or receive buffer unavailable interrupt(SR[bit7])
 * or receive FIFO overflow interrupt(SR[bit4])
 * @netdev: target net device structure
 * @limit: processing limit number of packets
 *
 * Description: In this function, the driver will check rx descriptor and
 * post skb to next network layer.
 * Returns 0 on success, negative value on failure
 */
static int fgmac4_rx(struct net_device *netdev, int limit)
{
	struct device *dev = &netdev->dev;
	struct fgmac4_private *lp;
	u32 rxtail, desc_num;
	u32 fdesc, ldesc;
	int work_done = 0;

	DBG_FUN_START(dev);

	lp = netdev_priv(netdev);

	desc_num = FGMAC4_RDESC_NUM;	/* check the whole RX ring */
	rxtail = lp->rx_tail;	/* start check from this descriptor */

	DBG_DUMP(dev, lp->base_addr, DESC_DUMP, NULL,
			(int *) lp->rx_ring, 0, 0);

	/* Check every RxDesc.
	 * if its OWN bit is 0, post its skbuf to nex layer
	 */
	while (desc_num) {

		struct sk_buff *reskb; /* the skbuf passed to the upper layer */
		u32 rx_sta, datalen, buflen;
		u32 first_rx_sta;
		u32 bnum, i;
		u32 rx_err = 0;

		first_rx_sta = rx_sta = lp->rx_ring[rxtail].opts1;

		if (rx_sta & FGMAC4_RX_DESC_OWN_BIT) {	/* OWN bit is 1 */
			DBG_PRINT(dev, "RX Desc(%d) is not mine.\n", rxtail);
			break;
		}

		fdesc = rxtail;
		DBG_PRINT(dev, "First Desc(%d):0x%08x\n", fdesc, rx_sta);
		ldesc = rxtail;
		for (bnum = 1; bnum <= desc_num; bnum++) {
			rx_sta = lp->rx_ring[ldesc].opts1;

			if (rx_sta & FGMAC4_RX_DESC_OWN_BIT) {	/*OWN bit is 1*/
				DBG_PRINT(dev, "NO LAST ONE!!!\n");
				goto rx_end;
			}

			if (rx_sta & FGMAC4_RX_DESC_LS) {
				DBG_PRINT(dev, "Last Desc(%d):0x%08x\n",
							ldesc, rx_sta);
				break;
			} else
				DBG_PRINT(dev, "Middle Desc(%d):0x%08x\n",
							ldesc, rx_sta);

			if (fdesc == NEXT_RX(ldesc)) {
				DBG_PRINT(dev, "NO LAST ONE!!!\n");
				goto rx_end;
			} else
				ldesc = NEXT_RX(ldesc);
		}

		if (bnum > desc_num) {
			DBG_PRINT(dev, "NOT FIND LAST ONE!!!\n");
			goto rx_end;
		}

		DBG_PRINT(dev, "Frame's descriptor:%d\n", bnum);

		/* Now rx_sta contains the desc0's info of the last desc */
		if (rx_sta & FGMAC4_RX_DESC_VLAN)
			DBG_PRINT(dev, "Received VLAN Frame.\n");

		if (lp->rx_csum) {
#ifdef CONFIG_FGMAC4_ALT_DESC
			rx_err = !fgmac4_rx_csum_is_ok(&(lp->rx_ring[ldesc]))
				|| (rx_sta & FGMAC4_RX_DESC_ERR_IPC);
#else
			rx_err = !fgmac4_rx_csum_is_ok(&(lp->rx_ring[ldesc]))
				|| (rx_sta & FGMAC4_RX_DESC_ERR_IPC
				& ~(FGMAC4_RX_DESC_IPC_ERR
				| FGMAC4_RX_DESC_RMCE_ERR));
#endif
		} else
			rx_err = rx_sta & FGMAC4_RX_DESC_ERR;

		if (rx_err) {
			if (lp->rx_csum) {
#ifdef CONFIG_FGMAC4_ALT_DESC
				PMSG(dev, KERN_NOTICE, "%08x %08x\n",
					fgmac4_rx_csum_is_ok(
						&(lp->rx_ring[ldesc])),
					(rx_sta & FGMAC4_RX_DESC_ERR_IPC));
#else
				PMSG(dev, KERN_NOTICE, "%08x %08x\n",
				fgmac4_rx_csum_is_ok(&(lp->rx_ring[ldesc])),
					(rx_sta & FGMAC4_RX_DESC_ERR
						& ~(FGMAC4_RX_DESC_IPC_ERR
						| FGMAC4_RX_DESC_RMCE_ERR)));
#endif
			}
			PMSG(dev, KERN_NOTICE,
				"RX Desc(%d) has error status(0x%08x:0x%08x)\n",
				 rxtail, rx_sta,
				fgmac4_reg_read(lp->base_addr, FGMAC4_REG_SR));

			/* renew the value of error */
			lp->net_stats.rx_errors++;

			/* when descripter error happen,
			 * it means rx ring buffer overflow.
			 */
			if (rx_sta & FGMAC4_RX_DESC_DE_ERR)
				lp->net_stats.rx_over_errors++;

			if (rx_sta & FGMAC4_RX_DESC_LE_ERR)
				/* Length Error without Giant Frame setting */
				lp->net_stats.rx_length_errors++;

			if (rx_sta & FGMAC4_RX_DESC_CE_ERR)
				/* CRC Error */
				lp->net_stats.rx_crc_errors++;

			if (rx_sta & FGMAC4_RX_DESC_OE_ERR)
				/* Overflow Error */
				lp->net_stats.rx_fifo_errors++;

			goto rx_next;
		}

		/* Get data length from desc without 4bytes FCS */
		datalen = ((rx_sta >> FGMAC4_RX_DESC_FL_SHIFT)
				& FGMAC4_RX_DESC_FL_MASK) - 4;

#ifndef CONFIG_FGMAC4_USE_BOUNCE_BUF
		if (bnum == 1) {
			dma_unmap_single(NULL, lp->rx_skb[fdesc].mapping,
					 datalen, DMA_FROM_DEVICE);
			reskb = lp->rx_skb[fdesc].skb;

			/* alloc new receive buffer */
			lp->rx_skb[fdesc].skb = dev_alloc_skb(lp->rx_buf_sz);
			if (lp->rx_skb[fdesc].skb) {
				skb_reserve(lp->rx_skb[fdesc].skb,
					NET_IP_ALIGN);
				lp->rx_skb[fdesc].skb->dev = lp->dev;
				/* set the alloced skb in receive skb array */
				lp->rx_skb[fdesc].mapping = dma_map_single(NULL,
						lp->rx_skb[fdesc].skb->data,
						lp->rx_buf_sz - NET_IP_ALIGN,
						DMA_FROM_DEVICE);

				skb_put(reskb, datalen);
				goto rx_trans_skb;
			} else {
				/* renew status struct */
				lp->net_stats.rx_dropped++;
				/* set the alloced skb in receive skb array */
				lp->rx_skb[fdesc].mapping =
					dma_map_single(NULL, reskb->data,
						lp->rx_buf_sz - NET_IP_ALIGN,
						DMA_FROM_DEVICE);
				goto rx_next;
			}
		}
#endif /* CONFIG_FGMAC4_USE_BOUNCE_BUF */

		/* DestinationAddr(6)+SourceAddr(6)+Type(2)+NET_IP_ALIGN(2)
		 * =16Bytes
		 */
		buflen = datalen + NET_IP_ALIGN;
		DBG_PRINT(dev, "Receive Data Length:%d\n", buflen);

		/* alloc a properly sized skbuff */
		reskb = dev_alloc_skb(buflen);
		if (unlikely(reskb == NULL)) {
			PMSG(dev, KERN_WARNING,
			"Failed to alloc skb, Drop! RxDesc(%d)  DataLen(%d)!!!",
				rxtail, datalen);
			/* renew status struct */
			lp->net_stats.rx_dropped++;
			goto rx_next;
		}
		reskb->dev = netdev;

		/* insert 16 bytes befor ethernet header
		 * |pad 2B|desAddr 6B|souAddr 6B|type 2B|
		 */
		skb_reserve(reskb, NET_IP_ALIGN);

		i = bnum;
		while (i) {
			u32 temp_len;

			temp_len = (fdesc == ldesc) ? datalen : lp->rx_buf_sz;
			/* copy data from RX buffer to
			 * a properly sized skbuffer
			 */
#ifdef CONFIG_FGMAC4_USE_BOUNCE_BUF
			memcpy(skb_put(reskb, temp_len),
					lp->rx_skb[fdesc].data, temp_len);
			datalen -= temp_len;

#else /* !CONFIG_FGMAC4_USE_BOUNCE_BUF */

			dma_sync_single_for_cpu(NULL, lp->rx_skb[fdesc].mapping,
				temp_len, DMA_FROM_DEVICE);
			memcpy(skb_put(reskb, temp_len),
					lp->rx_skb[fdesc].skb->data, temp_len);
			datalen -= temp_len;
			dma_sync_single_for_device(NULL,
				lp->rx_skb[fdesc].mapping,
				temp_len, DMA_FROM_DEVICE);
#endif /* CONFIG_FGMAC4_USE_BOUNCE_BUF */

			fdesc = NEXT_RX(fdesc);
			i--;
		}
#ifndef CONFIG_FGMAC4_USE_BOUNCE_BUF
rx_trans_skb:
#endif /* CONFIG_FGMAC4_USE_BOUNCE_BUF */
		/*
		 * determine the packet type which will be used
		 * to identify net work
		 */

		reskb->protocol = eth_type_trans(reskb, netdev);

		/* checksum offload engine */
		if (lp->rx_csum)
			reskb->ip_summed = CHECKSUM_UNNECESSARY;
		else
			reskb->ip_summed = CHECKSUM_NONE;

		lp->net_stats.rx_packets++;
		lp->net_stats.rx_bytes += reskb->len;

		netdev->last_rx = jiffies;
		/* post skb to next network layer */
#ifdef CONFIG_FGMAC4_NAPI
		netif_receive_skb(reskb);
#else
		netif_rx(reskb);
#endif /* CONFIG_FGMAC4_NAPI */
		work_done++;

rx_next:
		for (i = 0; i < bnum; i++) {

			/* set receive descriptor */
			if (rxtail == (FGMAC4_RDESC_NUM - 1)) {
				/* the last one in ring */
				lp->rx_ring[rxtail].opts2 =
						(FGMAC4_RX_DESC_END_RING
						| (lp->buff2_cnt << TRBS2_SHIFT)
						| lp->buff1_cnt);
			} else {
				lp->rx_ring[rxtail].opts2 =
						(lp->buff2_cnt << TRBS2_SHIFT)
						| lp->buff1_cnt;
			}
#ifdef CONFIG_FGMAC4_ALT_DESC
			lp->rx_ring[rxtail].ex_status_rsv = 0;
#endif

			lp->rx_ring[rxtail].addr1 = lp->rx_skb[rxtail].mapping;
			if (lp->buff2_cnt)
				lp->rx_ring[rxtail].addr2 =
						lp->rx_skb[rxtail].mapping
						+ lp->buff1_cnt;
			else
				lp->rx_ring[rxtail].addr2 = 0;
			/*
			 * Make sure other field are setup correctly before
			 * setup OWN bit, so a wmb is needed.
			 */
			wmb();
			/*set OWN bit*/
			lp->rx_ring[rxtail].opts1 = FGMAC4_RX_DESC_OWN_BIT;

			/* get next descriptor */
			rxtail = NEXT_RX(rxtail);
		}

		desc_num -= bnum;

		if (limit <= work_done)
			break;
	}

	wmb();

rx_end:
	/* send RX POLL Demand to wake suspend receive process */
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_RPDR, 1);

	lp->rx_tail = rxtail;

	DBG_DUMP(dev, lp->base_addr, DESC_DUMP, NULL, (int *) lp->rx_ring,
								0, 0);

	DBG_FUN_END(dev);
	return work_done;
}

/**
 * fgmac4_tx -- this function will be called when there is transmit finish
 * interrupt(SR[bit0]) or transmit buffer unavailable interrupt(SR[bit2])
 * or transmit underflow interrupt(SR[bit5])
 * @netdev: target net device structure
 *
 * Description: In this function, the driver will check tx descriptor and
 * conditionally free the used skbuffer of tx descriptor.
 * Returns 0 on success, negative value on failure
 */
static int fgmac4_tx(struct net_device *netdev)
{
	struct device *dev = &netdev->dev;
	struct fgmac4_private *lp;
	int txtail;		/* the head of TX rings whose OWN bit is 1 */
	int txhead;		/* the head of TX ring whose OWN bit is 0 */

	DBG_FUN_START(dev);

	lp = netdev_priv(netdev);

	/* begin to check the TX descriptor that has done the job */
	txtail = lp->tx_tail;
	txhead = lp->tx_head;

	DBG_DUMP(dev, lp->base_addr, DESC_DUMP, (int *)lp->tx_ring, NULL, 0, 0);

	/* we start check from descriptor(tx_tail), because it first get job
	 * and must be first finish the job.
	 */
	while (txtail != txhead) {
		u32 dstatus;	/* the status of descriptor */

		dstatus = lp->tx_ring[txtail].opts1;
		if (dstatus & FGMAC4_TX_DESC_OWN_BIT) {
			DBG_PRINT(dev, "the deal of TX interrupt over\n");
			break;
		}
		/* The FS field of mb8ac0300 contains in opts1 */
		/* The FS field of ES2 contains in opts2 */
#ifdef CONFIG_FGMAC4_ALT_DESC
		if (!(lp->tx_ring[txtail].opts1 & FGMAC4_TX_DESC_LAST_SEG))
#else
		if (!(lp->tx_ring[txtail].opts2 & FGMAC4_TX_DESC_LAST_SEG))
#endif
			goto do_stats;
		if (dstatus & FGMAC4_TX_DESC_ERR) {
			DBG_PRINT(dev, "TX Desc ERROR 0x%08x\n",
							dstatus, txtail);

			/* renew the value of errors */
			lp->net_stats.tx_errors++;

			/* When carrier error, late collision error,
			 * Excessive collision error,
			 * underflow error happened,
			 * the packet is aborted.
			 */
			if (dstatus & (FGMAC4_TX_DESC_LC_ERR
						| FGMAC4_TX_DESC_NC_ERR
						| FGMAC4_TX_DESC_LCO_ERR
						| FGMAC4_TX_DESC_UF_ERR
						| FGMAC4_TX_DESC_EC_ERR)) {

				lp->net_stats.tx_aborted_errors++;

				if ((dstatus & FGMAC4_TX_DESC_LC_ERR) |
					   (dstatus & FGMAC4_TX_DESC_NC_ERR)) {
					/* Loss of Carrier | No Carrier */
					lp->net_stats.tx_carrier_errors++;
				}

				if (dstatus & FGMAC4_TX_DESC_UF_ERR)
					/* Undeflow Error */
					lp->net_stats.tx_fifo_errors++;
				else if (dstatus & FGMAC4_TX_DESC_LCO_ERR)
					/* Late Collision without
					 * Undeflow Error
					 */
					lp->net_stats.tx_window_errors++;

				/*renew the status of collisions count*/
				if ((dstatus & FGMAC4_TX_DESC_EC_ERR))
					lp->net_stats.collisions += 16;
				else {
					lp->net_stats.collisions +=
					 (dstatus >> FGMAC4_TX_DESC_CC_SHIFT) &
							 FGMAC4_TX_DESC_CC_MASK;
				}
			}

		}
		lp->net_stats.tx_packets++;
do_stats:
#ifdef CONFIG_FGMAC4_USE_BOUNCE_BUF
		lp->net_stats.tx_bytes += lp->tx_skb[txtail].len;
		if (unlikely(lp->tx_skb[txtail].data == NULL)) {
			PMSG(dev, KERN_WARNING,
				"TX Desc(%d) does not have buffer!!!\n",
				txtail);
		}

		/* clean desciptor status */
		lp->tx_ring[txtail].opts1 = 0;
		lp->tx_ring[txtail].opts2 = 0;

#else /* !CONFIG_FGMAC4_USE_BOUNCE_BUF */

		lp->net_stats.tx_bytes += lp->tx_skb[txtail].skb->len;

		if (unlikely(lp->tx_skb[txtail].skb == NULL)) {
			PMSG(dev, KERN_WARNING,
				"TX Desc(%d) does not have skbuffer!!!\n",
				txtail);
		} else {

			/* delete the mapping with skbuffer */
			dma_unmap_single(NULL, lp->tx_skb[txtail].mapping,
					lp->tx_skb[txtail].skb->len,
					DMA_TO_DEVICE);

			/* free the skbuffer */
			dev_kfree_skb_irq(lp->tx_skb[txtail].skb);
		}

		/* clean desciptor status */
		lp->tx_skb[txtail].skb = NULL;
		lp->tx_skb[txtail].mapping = 0;
		lp->tx_ring[txtail].opts1 = 0;
		lp->tx_ring[txtail].opts2 = 0;
		lp->tx_ring[txtail].addr1 = 0;
		lp->tx_ring[txtail].addr2 = 0;
#endif /* CONFIG_FGMAC4_USE_BOUNCE_BUF */

		/* Till now, we didn't use FGMAC4MT's time stamp function,
		 * so do not do any operation about time stamp.
		 */

		/* get next descriptor */
		txtail = NEXT_TX(txtail);
	}

	/* renew the flag of ring */
	lp->tx_tail = txtail;

	DBG_DUMP(dev, lp->base_addr, DESC_DUMP, (int *)lp->tx_ring, NULL, 0, 0);

	if (netif_carrier_ok(netdev) && netif_queue_stopped(netdev)
					&& (TX_RING_AVAIL(lp) > 0)) {
		/* wake queue */
		netif_wake_queue(netdev);
		DBG_PRINT(dev, "Wake up queue\n");
	}

	DBG_FUN_END(dev);
	return 0;
}

/**
 * fgmac4_interrupt -- this function will be called when there is interrupts
 * irq: the interrupt number being requested
 * @dev_id: target net device id
 *
 * Description: In this function, the driver will handle the device's
 * interrupts.
 * Return IRQ_HANDLED if the corresponding interrupt is handled, IRQ_NONE if not
 */
static irqreturn_t fgmac4_interrupt(int irq, void *dev_id)
{
	struct device *dev;
	struct net_device *netdev;
	struct fgmac4_private *lp;
	u32 ints;		/* recived interrupts */
	int ret;
#ifdef CONFIG_FGMAC4_NAPI
	u32 d;
#endif /* CONFIG_FGMAC4_NAPI */

	dev = &(((struct net_device *) dev_id)->dev);

	DBG_FUN_START(dev);

	DBG_PRINT(dev, "IRQ(%d)\n", irq);

	netdev = (struct net_device *) dev_id;

	lp = netdev_priv(netdev);

	/* get spin lock to refuse others */
	spin_lock(&lp->lock);

	DBG_DUMP(dev, lp->base_addr, REG_DUMP, NULL, NULL, 0, 0);

	/* get interrupts from SR register */
	ints = fgmac4_reg_read(lp->base_addr, FGMAC4_REG_SR);

	/* clear interrupt */
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_SR, ints);

	DBG_PRINT(dev, "interrupts 0x%08x\n", ints);

	if (!(ints & FGMAC4_CHECK_INT)) {	/* did not find any interrupt */

		spin_unlock(&lp->lock);
		DBG_FUN_END(dev);
		return IRQ_NONE;
	}

	/* RX stop interrupt. RPS(SR[bit8])
	 * OR Receive Buffer Unavailable interrupt. RU(SR[bit7])
	 * OR Receive FIFO overflow interrupt. OVF(SR[bit4])
	 */
	if (ints & (FGMAC4_SR_RPS
		| FGMAC4_SR_RU | FGMAC4_SR_OVF)) {
		if (ints & FGMAC4_SR_RPS) {
			/* find out where RX stop by RDLAR register */
			PMSG(dev, KERN_ALERT,
				"RX stop interrupt. Stop at (0x%08x)."
				" Need to restart LAN device!!!\n",
				fgmac4_reg_read(lp->base_addr,
						FGMAC4_REG_RDLAR));
		}

		if (ints & FGMAC4_SR_RU) {
			/* tell user what kind of interrupt */
			DBG_PRINT(dev,
				"Receive Buffer Unavailable interrupt\n");
		}

		if (ints & FGMAC4_SR_OVF) {
			/* tell user what kind of interrupt */
			DBG_PRINT(dev, "Receive FIFO overflow interrupt\n");
		}

		/* collect status */
		fgmac4_get_stats_from_mfc(lp);

		DBG_PRINT(dev, "The deal with RX error interrupt over\n");
	}

	/* Receive interrupt. RI(SR[bit6]) */
	if (ints & FGMAC4_SR_RI) {
		/* tell user what kind of interrupt */
		DBG_PRINT(dev, "Receive interrupt\n");

#ifdef CONFIG_FGMAC4_NAPI
		/* disable RX interrupt */
		d = fgmac4_reg_read(lp->base_addr, FGMAC4_REG_IER);
		d &= ~FGMAC4_IER_INT_RX_INT;
		fgmac4_reg_write(lp->base_addr, FGMAC4_REG_IER, d);
		/* start NAPI scheduling */

		napi_schedule(&lp->napi);
#else
		/* begin pass received packet and
		 * send RX poll demand to wake suspending RX
		 */
		ret = fgmac4_rx(netdev, FGMAC4_RDESC_NUM);
		if (ret < 0)
			DBG_PRINT(dev, "The deal with RX interrupt ERROR\n");
		else
			DBG_PRINT(dev, "The deal with RX interrupt OK\n");
#endif /* CONFIG_FGMAC4_NAPI */
	}

	/* TX stop interrupt. TPS(SR[bit1])
	 * OR Transmit buffer unavailable interrupt. TU(SR[bit2])
	 * OR Transmit underflow interrupt. UNF(SR[bit5])
	 */
	if (ints & (FGMAC4_SR_TU | FGMAC4_SR_UNF
		| FGMAC4_SR_TPS)) {
		if (ints & FGMAC4_SR_TPS) {
			/* find out where TX stop by TDLAR register */
			PMSG(dev, KERN_ALERT,
				"TX stop interrupt(0x%08x). stop at (0x%08x)."
				" Need to restart LAN device!!!\n",
				ints, fgmac4_reg_read(lp->base_addr,
							FGMAC4_REG_TDLAR));
		}

		if (ints & FGMAC4_SR_TU)
			/* tell user what kind of interrupt */
			DBG_PRINT(dev,
				"Transmit Buffer Unavailable interrupt\n");

		if (ints & FGMAC4_SR_UNF)
			/* tell user what kind of interrupt */
			DBG_PRINT(dev, "Transmit underflow interrupt\n");

		DBG_DUMP(dev, lp->base_addr, REG_DUMP | DESC_DUMP,
				(int *) lp->tx_ring, (int *)lp->rx_ring, 0, 0);

		DBG_PRINT(dev, "The deal with TX error interrupt over\n");
	}

	/* Transmit finish interrupt. TI(SR[bit0]) */
	if (ints & FGMAC4_SR_TI) {
		/* tell user what kind of interrupt */
		DBG_PRINT(dev, "Transmit finish interrupt\n");

		/* Check every TX descriptor's status and conditionally
		 * free skbuff
		 */
		ret = fgmac4_tx(netdev);
		if (ret)
			DBG_PRINT(dev, "The deal with TX interrupt ERROR\n");
		else
			DBG_PRINT(dev, "The deal with TX interrupt OK\n");
	}

	/* Fatal bus error interrupt. FBI(SR[bit13]) */
	if (ints & FGMAC4_SR_FBI) {
		/* tell user what kind of interrupt */
		DBG_PRINT(dev, "Fatal bus error interrupt\n");

		/* collect status */
		fgmac4_get_stats_from_mfc(lp);

		DBG_PRINT(dev, "The deal with Fatal bus error int over\n");
	}

#ifdef CONFIG_FGMAC4_MMC
	if (ints & FGMAC4_SR_GMI) {
		fgmac4_update_mmc(netdev);
		fgmac4_reg_read(lp->base_addr, FGMAC4_REG_MMC_INTR_RX);
		fgmac4_reg_read(lp->base_addr, FGMAC4_REG_MMC_INTR_TX);
		fgmac4_reg_read(lp->base_addr, FGMAC4_REG_MMC_IPC_INTR_RX);
	}
#endif /* CONFIG_FGMAC4_MMC */
	/* free spin lock */
	spin_unlock(&lp->lock);

	DBG_DUMP(dev, lp->base_addr, REG_DUMP, NULL, NULL, 0, 0);

	DBG_FUN_END(dev);
	return IRQ_HANDLED;
}

/**
 * fgmac4_timeout -- this function will be called when send engine timeout
 * @netdev: target net device structure
 *
 * Description: When send engine timeout this function will be called. In
 * this funtion, the driver will reset the whole device and initialize
 * send and receive engine.
 */
static void fgmac4_timeout(struct net_device *netdev)
{
	struct device *dev = &netdev->dev;
	struct fgmac4_private *lp;
	u32 val;
	int err;
	unsigned long flags;

	DBG_FUN_START(dev);

	lp = netdev_priv(netdev);

	/* get spin lock and mask interrupt */
	spin_lock_irqsave(&lp->lock, flags);

	DBG_DUMP(dev, lp->base_addr, REG_DUMP | DESC_DUMP,
		 (int *) lp->tx_ring, (int *)lp->rx_ring, 0, 0);

	val = fgmac4_reg_read(lp->base_addr, FGMAC4_REG_SR);
	PMSG(dev, KERN_NOTICE, "Transmit timed out, status 0x%08x\n", val);

	DBG_PRINT(dev, "Now TX Desc(0x%08x)\n",
			fgmac4_reg_read(lp->base_addr, FGMAC4_REG_TDLAR));
	DBG_PRINT(dev, "Now RX Desc(0x%08x)\n",
			fgmac4_reg_read(lp->base_addr, FGMAC4_REG_RDLAR));

	/* Watchdog Timesout will not happen unless there is something wrong
	 * in processer. Now we reset the processor to correct the error.
	 * Before begin reset, we stop TX/RX process with stop command.
	 */
	fgmac4_stop_rxtx(lp);

	/* collect the status of transfer befor reset */
	fgmac4_get_stats_from_mfc(lp);

	/* Reset the processer */
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_BMR,
			FGMAC4_BMR_SOFTWARE_RESET);
	/* have to wait morn than 1600ns before access the register */
	udelay(DELAY_TIME);

	/* free spin lock and enable interrupt */
	spin_unlock_irqrestore(&lp->lock, flags);

	fgmac4_init_ring(lp, !FGMAC4_ALLOC_RING);

	/* initialize the hardware */
	err = fgmac4_init_hw(netdev);
	if (err)
		PMSG(dev, KERN_WARNING, "Failed to initialize device!!!\n");

	DBG_DUMP(dev, lp->base_addr, DESC_DUMP,
		(int *) lp->tx_ring, (int *) lp->rx_ring, 0, 0);

	/* wake queue to start transfer */
	netif_wake_queue(netdev);

	DBG_FUN_END(dev);
}

/**
 * fgmac4_stats -- get the status of send and receive from mmc registers
 * @netdev: target net device structure
 *
 * Description: In this function, the driver collect the status of send and
 * receive engine in mmc registers.
 * Returns @net_stats on success, NULL on failure
 */
static struct net_device_stats *fgmac4_stats(struct net_device *netdev)
{
	struct device *dev = &netdev->dev;
	struct fgmac4_private *lp;
	unsigned long flags;

	DBG_FUN_START(dev);

	if (unlikely(netdev == NULL)) {
		PMSG(dev, KERN_CRIT, "Can not find proper device!!!\n");
		DBG_FUN_ERREND(dev);
		return NULL;
	}

	lp = netdev_priv(netdev);
	if (unlikely(lp == NULL)) {
		PMSG(dev, KERN_CRIT, "Can not get the device!!!\n");
		DBG_FUN_ERREND(dev);
		return NULL;
	}

	spin_lock_irqsave(&lp->lock, flags);

	if (netif_running(netdev)) {
		/* Get status from MFC registers */
		fgmac4_get_stats_from_mfc(lp);
		DBG_DUMP(dev, lp->base_addr, REG_DUMP | DESC_DUMP,
			(int *) lp->tx_ring, (int *) lp->rx_ring, 0, 0);
	}

	spin_unlock_irqrestore(&lp->lock, flags);

	DBG_FUN_END(dev);
	return &lp->net_stats;
}

/**
 * fgmac4_mdio_register -- init mdio bus
 * @netdev: target net device structure
 * @pdev: target platform device structure
 *
 * Description: init mdio bus and setup phy id.
 * Returns 0 on success, negative value on failure
 */
static int fgmac4_mdio_register(struct net_device *netdev,
				struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fgmac4_private *lp;
	int i, id;
	int ret;
	struct phy_device *phydev;

	DBG_FUN_START(dev);

	lp = netdev_priv(netdev);

	lp->mdio_bus = mdiobus_alloc();
	if (lp->mdio_bus == NULL) {
		PMSG(dev, KERN_NOTICE, "mdiobus_alloc NULL\n");
		return -ENOMEM;
	}

	lp->mdio_bus->name = "fgmac4 mdio bus";
	id = pdev->id == -1 ? 0 : pdev->id;
	snprintf(lp->mdio_bus->id, MII_BUS_ID_SIZE, "%x", id);

	lp->mdio_bus->priv     = netdev;

	lp->mdio_bus->parent   = lp->device;
	lp->mdio_bus->read     = &fgmac4_phy_read;
	lp->mdio_bus->write    = &fgmac4_phy_write;

	lp->mdio_bus->phy_mask = PHY_MASK_ADDR;

	lp->mdio_bus->irq      = &lp->mdio_irq[0];

	for (i = 0; i < PHY_MAX_ADDR; i++)
		lp->mdio_bus->irq[i] = PHY_POLL;

	ret = mdiobus_register(lp->mdio_bus);
	if (ret) {
		PMSG(dev, KERN_NOTICE, "%s: mdiobus_reg failed (0x%x)\n",
			lp->dev->name, ret);
		mdiobus_free(lp->mdio_bus);
		return ret;
	}
	/* After mdiobus_register, the mdio_bus->phy_map
	 * becomes not NULL
	 */

	lp->phy_map = 0;
	for (i = 0; i < PHY_MAX_ADDR; i++) {
		phydev = lp->mdio_bus->phy_map[i];
		if (phydev) {
			lp->phy_map |= 1 << i;

			PMSG(dev, KERN_NOTICE, "The phy id is [%d]\n", i);
		}
	}

	if (!lp->phy_map) {
		PMSG(dev, KERN_NOTICE, "Not got a valid phy id!\n");
		mdiobus_unregister(lp->mdio_bus);
		mdiobus_free(lp->mdio_bus);
		return -ENODEV;
	}

	DBG_PRINT(dev, "phy_map : 0x%08x\n", lp->phy_map);
	DBG_FUN_END(dev);
	return 0;
}

/**
 * fgmac4_mdio_unregister -- de-init mdio bus
 * @netdev: target net device structure
 *
 * Description: de-init mdio bus.
 */
static void fgmac4_mdio_unregister(struct net_device *netdev)
{
	struct fgmac4_private *lp = netdev_priv(netdev);

	mdiobus_unregister(lp->mdio_bus);
	mdiobus_free(lp->mdio_bus);
}

/**
 * fgmac4_stop_hw -- stop hardware
 * @lp: target FGMAC4 private structure
 *
 * Description: In this function, the driver will stop the TX/RX engine and
 * clear the status of hardware.
 */
static void fgmac4_stop_hw(struct fgmac4_private *lp)
{
	u32 val;
#ifdef FGMAC4_DEBUG
	struct device *dev = &lp->dev->dev;
#endif /* FGMAC4_DEBUG */

	DBG_FUN_START(dev);

	DBG_DUMP(dev, lp->base_addr, REG_DUMP, NULL, NULL, 0, 0);

	/* disable all interrupt */
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_IER, 0);

	/* stop TX and RX process */
	fgmac4_stop_rxtx(lp);

	/* clear status register */
	val = fgmac4_reg_read(lp->base_addr, FGMAC4_REG_SR);
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_SR, val);

	DBG_DUMP(dev, lp->base_addr, REG_DUMP, NULL, NULL, 0, 0);

	DBG_FUN_END(dev);
}

/**
 * fgmac4_hw_csum_need -- Judge if tx checksum offload setup is needed
 * @lp: target FGMAC4 private structure
 * @netdev: target net device structure
 *
 * Description: Judge if tx checksum offload setup is needed.
 * Returns 1 if needed, 0 if not
 */
static u32 fgmac4_hw_csum_need(struct sk_buff *skb, struct net_device *netdev)
{
	u32 hw_checksum_needed = 0;	/* 0 : not needed, 1 : needed */

	/* tx checksum offload is disabled */
	if (!(netdev->features & NETIF_F_ALL_CSUM))
		return hw_checksum_needed;

	/* Refer to skbuff.h for more detail about CHECKSUM_XXX macros.
	 * On transmit path, CHECKSUM_PARTIAL and CHECKSUM_NONE are
	 * used .
	 */

	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		/* CHECKSUM_PARTIAL */
		/* IP packet*/
		if (skb->protocol == htons(ETH_P_IP)) {
			if (ip_hdr(skb)->version == 0x4) {
				struct iphdr *iph = ip_hdr(skb);

				/* Till now, checksum offload only support
				 * tcp&udp&icmp
				 */
				if (likely((iph->protocol == IPPROTO_TCP)
					|| (iph->protocol == IPPROTO_UDP)
					|| (iph->protocol == IPPROTO_ICMP))) {

					/* enable checksum on tx path */
					hw_checksum_needed = 1;

					/* set checksum field of ipv4 to 0 */
					iph->check = 0;

					/* set checksum field of tcp to 0 */
					if (iph->protocol == IPPROTO_TCP) {
						struct tcphdr *tcph =
								   tcp_hdr(skb);
						tcph->check = 0;
					}

					/* set checksum field of udp to 0 */
					if (iph->protocol == IPPROTO_UDP) {
						struct udphdr *udph =
								   udp_hdr(skb);
						udph->check = 0;
					}

					/* For ICMP-over-IPv4 packets, the
					 * Checksum field in the ICMP packet
					 * must always be 16'h0000 in both
					 * modes, defined for such packets. If
					 * it does not equal 16'fh0000, an
					 * incorrect because pseudo-headers are
					 * not checksum may be inserted into the
					 * packet. For moredetail, refer to the
					 * data sheet.
					 */
					if (iph->protocol == IPPROTO_ICMP) {
						struct icmphdr *icmpv4hdr =
								icmp_hdr(skb);
						icmpv4hdr->checksum = 0;
					}
				}
			}
		}
	}
	return hw_checksum_needed;
}

/**
 * fgmac4_start_xmit -- this function will send the buffer
 * @skb: the socket buffer which contains the data to be sent
 * @netdev: target net device structure
 *
 * Description: This function is the send data engine of the lan device. In
 * this function, the driver will send the buffer by send DMA.
 * Returns 0 on success, negative value on failure
 */
static int fgmac4_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
#ifdef FGMAC4_DEBUG
	struct device *dev = &netdev->dev;
#endif
	struct fgmac4_private *lp = netdev_priv(netdev);
	struct fgmac4_desc *desp;	/* the pointer to descriptor */
	struct fgmac4_desc *first_desp;	/* the pointer to descriptor */
	u32 ring_num;		/* the number of free descriptors */
	u32 txhead;		/* The first descriptor of this transfer ring */
	u32 skblen, flag;
	u32 val;

#ifdef CONFIG_FGMAC4_USE_BOUNCE_BUF
	u32 tx_buff1_cnt = lp->buff1_cnt;

#else /* !CONFIG_FGMAC4_USE_BOUNCE_BUF */

	u32 tx_buff1_cnt = BUFFER1_MAX_CNT;
	dma_addr_t mapping;
#endif /* CONFIG_FGMAC4_USE_BOUNCE_BUF */
	u32 bnum, i;
	u32 temp_len;
	unsigned long flags;
	u32 hw_checksum_needed = 0;	/* 0 : not needed, 1 : needed */

	DBG_FUN_START(dev);

	/* we should use spin_lock_irqsave and spin_unlock_irqrestore
	 * instead of spin_lock_irq and spin_unlock_irq, because if not,
	 * there will be some core dump because the spin_unlock_irq
	 * enable the nic irq that should not be enabled(we disable it
	 * in ndo_poll_controller).
	 */
	spin_lock_irqsave(&lp->lock, flags);
	if (unlikely(skb->len < ETH_ZLEN)) {
		DBG_PRINT(dev, "skb_padto\n");

		if (skb_padto(skb, ETH_ZLEN)) {

			spin_unlock_irqrestore(&lp->lock, flags);

			DBG_FUN_END(dev);
			return FGMAC4_TX_OK;
		}
	}

	/* set up DMA mappings of sk buffer */
	skblen = ETH_ZLEN < skb->len ? skb->len : ETH_ZLEN;

	/* Use bnum descriptors to send one frame */
#ifdef CONFIG_FGMAC4_USE_BOUNCE_BUF
	if ((skblen % lp->tx_buf_sz) == 0)
		bnum = skblen / lp->tx_buf_sz;
	else
		bnum = skblen / lp->tx_buf_sz + 1;
#else /* !CONFIG_FGMAC4_USE_BOUNCE_BUF */
	bnum = 1;
#endif /* CONFIG_FGMAC4_USE_BOUNCE_BUF */

	DBG_PRINT(dev, "bnum:%d\n", bnum);

	/* get the number of free descriptor */
	ring_num = TX_RING_AVAIL(lp);
	if (ring_num < bnum) {
		netif_stop_queue(netdev);

		/* drop this packet */
		lp->net_stats.tx_dropped++;

		spin_unlock_irqrestore(&lp->lock, flags);

		DBG_FUN_ERREND(dev);
		return FGMAC4_TX_BUSY;
	}

	hw_checksum_needed = fgmac4_hw_csum_need(skb, netdev);

	txhead = lp->tx_head;
	first_desp = &lp->tx_ring[txhead];

	for (i = 1; i <= bnum; i++) {

		desp = &lp->tx_ring[txhead];
		DBG_PRINT(dev, "txhead(%d)\n", txhead);

		temp_len = (i == bnum) ? skblen : lp->tx_buf_sz;

#ifdef CONFIG_FGMAC4_USE_BOUNCE_BUF
		memcpy(lp->tx_skb[txhead].data, skb->data, temp_len);
		lp->tx_skb[txhead].len = temp_len;

		/* decrease the length of SKB */
		skblen -= temp_len;
		if (i != bnum)
			skb_pull(skb, temp_len);

#else /* !CONFIG_FGMAC4_USE_BOUNCE_BUF */

		/* skb is directly mapped and DMAed, so bnum should always 1,
		 * if for some reason, bnum is not equal to 1,
		 * it's probably a bug.
		 */

		BUG_ON(bnum != 1);
		mapping = dma_map_single(NULL, skb->data,
					temp_len, DMA_TO_DEVICE);
		desp->addr1 = mapping;
		if (tx_buff1_cnt < temp_len)
			desp->addr2 = mapping + tx_buff1_cnt;
		else
			desp->addr2 = 0;

		/* save the skb in TX skbuffer array, can be freed after TX */
		lp->tx_skb[txhead].skb = skb;
		lp->tx_skb[txhead].mapping = mapping;

#endif /* CONFIG_FGMAC4_USE_BOUNCE_BUF */

		/* the settings of descriptor */
#ifdef CONFIG_FGMAC4_ALT_DESC
		flag = FGMAC4_TX_DESC_INT_COMPLET;
#else
		if (tx_buff1_cnt < temp_len)
			flag = FGMAC4_TX_DESC_INT_COMPLET
				| (((temp_len - BUFFER1_MAX_CNT) << TRBS2_SHIFT)
				| BUFFER1_MAX_CNT);
		else
			flag = FGMAC4_TX_DESC_INT_COMPLET | temp_len;
#endif

		if (i == 1)	/* The first one of frame chain */
			flag |= FGMAC4_TX_DESC_FIRST_SEG;

		if (i == bnum)	/* The last one of frame chain  */
			flag |= FGMAC4_TX_DESC_LAST_SEG;

		if (unlikely(txhead == (FGMAC4_TDESC_NUM - 1))) {
			/* this descriptor is the end of ring */
			flag |= FGMAC4_TX_DESC_END_RING;
		}

		/* setup the checksum offload mode */

		if (hw_checksum_needed) {
			/* CIC[2b'11] */
			/* insert ipv4&tcp&udp&icmp/icmpv6 checksum */
			flag |= FGMAC4_TX_DESC_CIC3;
		} else {
			/* CIC[2b'00] */
			/* Bypass checksum offload */
			flag &= ~FGMAC4_TX_DESC_CIC_MASK;
		}
#ifdef CONFIG_FGMAC4_ALT_DESC
		desp->opts1 = flag;
		if (tx_buff1_cnt < temp_len)
			desp->opts2 = ((temp_len - BUFFER1_MAX_CNT)
						<< TRBS2_SHIFT)
					| BUFFER1_MAX_CNT;
		else
			desp->opts2 = temp_len;
#else
		desp->opts1 = 0;
		desp->opts2 = flag;
#endif

		/* set OWN bit */
		if (i != 1)
			desp->opts1 |= FGMAC4_TX_DESC_OWN_BIT;

		ring_num--;
		if (ring_num == 0) {
			DBG_PRINT(dev, KERN_NOTICE,
					"No more space in TX ring.\n");
			netif_stop_queue(netdev);
		}

		/* get the next descriptor */
		txhead = NEXT_TX(txhead);
	}
	/*
	 * Make sure other field are setup correctly before setup
	 * OWN bit, so a wmb is needed.
	 */
	wmb();

	/* set the first descriptor's owner bit, and start to trans */
	first_desp->opts1 |= FGMAC4_TX_DESC_OWN_BIT;
	wmb();
	/* renew the flag of TX ring */
	lp->tx_head = txhead;

	/* 9.Set OMR to start TX */
	val = fgmac4_reg_read(lp->base_addr, FGMAC4_REG_OMR);
	if (!(val & FGMAC4_OMR_START_TX)) {
		val |= FGMAC4_OMR_START_TX;	/* Start Transmission */
		fgmac4_reg_write(lp->base_addr, FGMAC4_REG_OMR, val);
	}

	/* clear the eee enable flag */
	if (eee && lp->eee_capable)
		lp->eee_enabled = 0;

	spin_unlock_irqrestore(&lp->lock, flags);

	/* send TX POLL Demand to wake suspend transmit process */
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_TPDR, 1);
	netdev->trans_start = jiffies;
#ifdef CONFIG_FGMAC4_USE_BOUNCE_BUF
	kfree_skb(skb);
#endif /* CONFIG_FGMAC4_USE_BOUNCE_BUF */

	DBG_FUN_END(dev);
	return FGMAC4_TX_OK;
}

/**
 * fgmac4_start_locked-- Prepare for normal tx/rx process
 * @netdev: net device pointer
 *
 * Description: Prepare for normal tx/rx process.
 * Returns 0 on success, negative value on failure
 */
static int fgmac4_start_locked(struct net_device *netdev)
{
	struct device *dev = &netdev->dev;
	struct fgmac4_private *lp;
	int err = 0, val;

	lp = netdev_priv(netdev);
	BUG_ON(!lp);

	fgmac4_set_macaddr_locked(netdev, netdev->dev_addr);
#ifdef CONFIG_FGMAC4_MMC
	/* init mmc */
	fgmac4_init_mmc(lp);
#endif /* CONFIG_FGMAC4_MMC */

	/* Initialize hardware */
	val = mii_link_ok(&lp->mii);
	if (!val) {
		PMSG(dev, KERN_NOTICE, "mii_link_ok says no carrier\n");
		return 0;
	}
	err = fgmac4_init_hw(netdev);
	if (err) {
		PMSG(dev, KERN_ERR,
			"Failed to initialize LAN's hardware!!!\n");
		return err;
	}

	return 0;
}

/**
 * fgmac4_open -- this function will be called when the device is started
 * @netdev: target net device structure
 *
 * Description: In this function, the driver will get the descriptor and
 * buffer space, set the device's registers, registe the interrupt handler
 * function and start the queue.
 * Returns 0 on success, negative value on failure
 */

static int fgmac4_open(struct net_device *netdev)
{

	struct device *dev = &netdev->dev;
#ifdef CONFIG_FGMAC4_USE_BOUNCE_BUF
	int alloc_ring = !FGMAC4_ALLOC_RING;
#else
	int alloc_ring = FGMAC4_ALLOC_RING;
#endif
	struct fgmac4_private *lp;
	int err = 0;
	unsigned long flags;

	DBG_FUN_START(dev);

	lp = netdev_priv(netdev);

	err = fgmac4_init_ring(lp, alloc_ring);
	if (err) {
		PMSG(dev, KERN_ERR, "Failed to initialize DMA descriptor!!!\n");
		return err;
	}
	DBG_DUMP(dev, lp->base_addr, DESC_DUMP, (int *) lp->tx_ring,
					(int *) lp->rx_ring, 0, 0);

#ifdef CONFIG_FGMAC4_NAPI
	napi_enable(&lp->napi);
#endif /* CONFIG_FGMAC4_NAPI */

	spin_lock_irqsave(&lp->lock, flags);

	/* set init_media flag to not init state */
	lp->init_media = 0;

	err = fgmac4_start_locked(netdev);
	if (err)
		goto err_start_lock;

	if (eee) {
		fgmac4_init_eee(lp);
		fgmac4_adjust_eee_link(lp, mii_link_ok(&lp->mii));
	}

	spin_unlock_irqrestore(&lp->lock, flags);

	if (mii_link_ok(&lp->mii)) {
		netif_carrier_on(netdev);
		/* start queue to allow transmit */
		netif_start_queue(netdev);
	}

	/* Initialize timer function which will check the state
	 * of carrier timely
	 */
	init_timer(&lp->timer);
	lp->timer.expires = CHECK_CARRIER_TIME;
	lp->timer.data = (unsigned long) netdev;
	lp->timer.function = fgmac4_check_carrier;
	add_timer(&lp->timer);

	DBG_FUN_ERREND(dev);

	return 0;

err_start_lock:
	spin_unlock_irqrestore(&lp->lock, flags);

#ifdef CONFIG_FGMAC4_NAPI
	napi_disable(&lp->napi);
#endif /* CONFIG_FGMAC4_NAPI */
	/* free descriptor and buffer space */
	fgmac4_free_ring(lp, alloc_ring);

	return err;
}

/**
 * fgmac4_stop_locked -- delete timer, stop hardware, stop netif, free rings
 * @netdev: net device pointer
 *
 * Description: delete timer, stop hardware, stop netif, free rings.
 * Returns 0 on success, negative value on failure
 */
static int fgmac4_stop_locked(struct net_device *netdev)
{
	struct fgmac4_private *lp;

	lp = netdev_priv(netdev);
	BUG_ON(!lp);

	/* stop hardware first */
	fgmac4_stop_hw(lp);
#ifdef CONFIG_FGMAC4_MMC
	/* update mmc */
	fgmac4_update_mmc(netdev);
#endif /* CONFIG_FGMAC4_MMC */

	return 0;
}

/**
 * fgmac4_close -- this function will be called when the device is stopped
 * @netdev: target net device structure
 *
 * Description: In this function, the driver will stop the device, remove
 * the interrupt handler that is registed in fgmac4_open function, free
 * all of spaces that are malloced in fgmac4_open function.
 * Returns 0 on success, negative value on failure
 */

static int fgmac4_close(struct net_device *netdev)
{
#ifdef FGMAC4_DEBUG
	struct device *dev = &netdev->dev;
#endif
#ifdef CONFIG_FGMAC4_USE_BOUNCE_BUF
	int need_free = !FGMAC4_FREE_RING;
#else
	int need_free = FGMAC4_FREE_RING;
#endif
	struct fgmac4_private *lp;
	unsigned long flags;

	DBG_FUN_START(dev);

	lp = netdev_priv(netdev);

	/* delete check carrier timer */
	del_timer_sync(&lp->timer);

	netif_stop_queue(netdev);
	netif_carrier_off(netdev);
#ifdef CONFIG_FGMAC4_NAPI
	napi_disable(&lp->napi);
#endif /* CONFIG_FGMAC4_NAPI */

	spin_lock_irqsave(&lp->lock, flags);

	fgmac4_stop_locked(netdev);

	spin_unlock_irqrestore(&lp->lock, flags);

	fgmac4_free_ring(lp, need_free);

	DBG_FUN_ERREND(dev);
	return 0;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
/**
 * fgmac4_netpoll -- this function will be called when using NET_CONSOLE
 * @dev: target net device structure
 *
 * Description: This function is used for net console, interrupt is disabled, so
 * it works on polling mode.
 */
static void fgmac4_netpoll(struct net_device *netdev)
{
	disable_irq_lockdep(netdev->irq);
	fgmac4_interrupt(netdev->irq, netdev);
	enable_irq_lockdep(netdev->irq);
}
#endif /* CONFIG_NET_POLL_CONTROLLER */

#ifdef CONFIG_FGMAC4_NAPI
/**
 * fgmac4_napipoll -- this function will be called when NAPI enbaled
 * @napi: napi structure
 * @budget: limit of processing packets
 *
 * Description: this function will be called when NAPI enbaled
 * Return the number of received packets
 */
static int fgmac4_napipoll(struct napi_struct *napi, int budget)
{
	struct fgmac4_private *lp = container_of(napi, struct fgmac4_private,
							napi);
	struct net_device *netdev = lp->dev;
	int work_done;
	u32 d;

	work_done = fgmac4_rx(netdev, budget);

	if (work_done < budget) {
		/* all pending packets processed */
		napi_complete(napi);
		d = fgmac4_reg_read(lp->base_addr, FGMAC4_REG_IER);
		d |= FGMAC4_IER_INT_RX_INT;
		fgmac4_reg_write(lp->base_addr, FGMAC4_REG_IER, d);
	}

	return work_done;
}
#endif	/* CONFIG_FGMAC4_NAPI */

static const struct net_device_ops fgmac4_netdev_ops = {
	.ndo_open		= fgmac4_open,
	.ndo_stop		= fgmac4_close,
	.ndo_start_xmit		= fgmac4_start_xmit,
	.ndo_get_stats		= fgmac4_stats,
	.ndo_do_ioctl		= fgmac4_ioctl,
	.ndo_tx_timeout		= fgmac4_timeout,
	.ndo_change_mtu		= fgmac4_change_mtu,
	.ndo_set_rx_mode	= fgmac4_set_multicast,
	.ndo_set_mac_address	= fgmac4_set_macaddr,
	.ndo_validate_addr	= eth_validate_addr,
#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller	= fgmac4_netpoll,
#endif /* CONFIG_NET_POLL_CONTROLLER */
};

static int fgmac4_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct net_device *netdev;
	struct fgmac4_private *lp;
	int i, err = 0;
	unsigned int clk_rate = 0;
	const u32 *p;
	u32 len;
	struct resource *res;

	DBG_FUN_START(dev);

	if (unlikely(pdev == NULL)) {
		PMSG(dev, KERN_CRIT,
			"Can not find proper platform device, aborting!!!\n");
		DBG_FUN_ERREND(dev);
		return -ENODEV;
	}

	/* allocate a new ethernet device */
	DBG_PRINT(dev, "alloc etherdev\n");
	netdev = alloc_etherdev(sizeof(struct fgmac4_private));
	if (!netdev) {
		PMSG(dev, KERN_ERR, "Etherdev alloc failed, aborting!!!\n");
		DBG_FUN_ERREND(dev);
		return -ENOMEM;
	}

	SET_NETDEV_DEV(netdev, &pdev->dev);

	/* get private struct's pointer */
	lp = netdev_priv(netdev);
	memset(lp, 0, sizeof(*lp));
	lp->dev = netdev;

	/* registe ethernet device */
	platform_set_drvdata(pdev, netdev);

	/* initialize spin lock */
	spin_lock_init(&lp->lock);

	/* fill structure net_device in defaults */
	ether_setup(netdev);
	/* fill structure net_device by FGMAC4 function */

	netdev->netdev_ops = &fgmac4_netdev_ops;
	netdev->ethtool_ops = &fgmac4_ethtool_ops;

#ifdef CONFIG_FGMAC4_NAPI
	netif_napi_add(netdev, &lp->napi, fgmac4_napipoll, fgmac4_napi_weight);
#endif /* CONFIG_FGMAC4_NAPI */
	netdev->watchdog_timeo = 5 * HZ;	/* 500jiffy = 5s */

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	netdev->irq = res->start;	/* set IRQ */

	/* Can not handle VLAN packets */

	/* by default, checksum offload is off */
	netdev->features = NETIF_F_VLAN_CHALLENGED;
	netdev->hw_features = NETIF_F_HW_CSUM;
	/* netdev->flags is seted in ether_setup() */
	/* MTU should be changed by system */

	/* By default, checksum offload is on */
	lp->rx_csum = 1;
	netdev->features |= NETIF_F_IP_CSUM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	/* map register address to virtual address space */
	lp->base_addr = ioremap(res->start, res->end - res->start + 1);
	if (lp->base_addr == NULL) {
		PMSG(dev, KERN_ERR,
			"Failed to ioremap ethernet registers!!!\n");
		err = -ENOMEM;
		goto err_out;
	}

	PMSG(dev, KERN_DEBUG, "ioremaped to 0x%08x\n", (unsigned)lp->base_addr);
	DBG_DUMP(dev, lp->base_addr, REG_DUMP, NULL, NULL, 0, 0);

	/* When not get the MAC address from register,
	 * generate random MAC address
	 */

	fgmac4_get_macaddr_locked(netdev);

	p = of_get_property(pdev->dev.of_node, "default_mac", &len);
	if (p && len == (ETH_ALEN * sizeof(int))) {
		for (i = 0; i < ETH_ALEN; i++)
			netdev->dev_addr[i] = be32_to_cpu(p[i]);

		fgmac4_set_macaddr_locked(netdev, netdev->dev_addr);
	}

	PMSG(dev, KERN_INFO,
		"Boot MAC address: %.2x:%.2x:%.2x:%.2x:%.2x:%.2x.\n",
		netdev->dev_addr[0], netdev->dev_addr[1],
		netdev->dev_addr[2], netdev->dev_addr[3],
		netdev->dev_addr[4], netdev->dev_addr[5]);

	if (!is_valid_ether_addr(netdev->dev_addr)) {
		/* generate MAC address */
		random_ether_addr(netdev->dev_addr);
		PMSG(dev, KERN_INFO,
			"Random MAC address: %.2x:%.2x:%.2x:%.2x:%.2x:%.2x.\n",
			netdev->dev_addr[0], netdev->dev_addr[1],
			netdev->dev_addr[2], netdev->dev_addr[3],
			netdev->dev_addr[4], netdev->dev_addr[5]);
	}
	lp->p_clk = clk_get(dev, "pclk");
	if (!IS_ERR(lp->p_clk))
		clk_prepare_enable(lp->p_clk);

	/* get MDC clock from SYS_CLOCK */
	lp->osc_clk = clk_get(dev, "osc_clk");
	if (!IS_ERR(lp->osc_clk))
		clk_prepare_enable(lp->osc_clk);

	lp->h_clk = clk_get(dev, "h_clk");
	if (unlikely(IS_ERR(lp->h_clk))) {
		PMSG(dev, KERN_ERR, "h_clk init Error!!!\n");
		err = -EINVAL;
		goto err_out;
	}
	clk_prepare_enable(lp->h_clk);
	clk_rate = clk_get_rate(lp->h_clk);
	if (clk_rate < 20000000) {
		PMSG(dev, KERN_ERR, "Unavailable SYS_CLK(%d), aborting!!!\n",
								clk_rate);
		err = -EINVAL;
		goto err_out;
	} else if (clk_rate < 35000000)
		lp->mdc_clk = 2;
	else if (clk_rate < 60000000)
		lp->mdc_clk = 3;
	else if (clk_rate < 100000000)
		lp->mdc_clk = 0;
	else if (clk_rate < 150000000)
		lp->mdc_clk = 1;
	/* Add for new */
	else if (clk_rate < 250000000)
		lp->mdc_clk = 4;
	else if (clk_rate < 300000000)
		lp->mdc_clk = 5;
	else {
		PMSG(dev, KERN_ERR, "Unavailable SYS_CLK(%d), aborting!!!\n",
								clk_rate);
		err = -EINVAL;
		goto err_out;
	}
	DBG_PRINT(dev, "SYS CLK:%d    MDC CLK:%d\n", clk_rate, lp->mdc_clk);

	p = of_get_property(pdev->dev.of_node, "gpio-phy-enable", &len);
	if (p) {
		lp->gpio_phy_enable = be32_to_cpu(*p);
		err = gpio_request(lp->gpio_phy_enable, "fgmac4-en");
		if (unlikely(err)) {
			dev_err(dev, " gpio %d request failed ",
						lp->gpio_phy_enable);
			return err;
		}
		err = gpio_direction_output(lp->gpio_phy_enable, 1);
		if (unlikely(err)) {
			dev_err(dev, "failed to set phy_enable gpio\n");
			return err;
		}
		msleep(50);
		dev_info(dev, "phy-gpio-enable %d\n", lp->gpio_phy_enable);
	} else
		dev_warn(dev, "no phy_enable gpio\n");

	p = of_get_property(pdev->dev.of_node, "gpio-phy-nrst", &len);
	if (p) {
		lp->gpio_phy_nrst = be32_to_cpu(*p);
		err = gpio_request(lp->gpio_phy_nrst, "fgmac4-nrst");
		if (unlikely(err)) {
			dev_err(dev, " gpio %d request failed ",
						lp->gpio_phy_enable);
			return err;
		}
		err = gpio_direction_output(lp->gpio_phy_nrst, 0);
		msleep(10);
		if (unlikely(err)) {
			dev_err(dev, "failed to set gpio\n");
			return err;
		}
		err = gpio_direction_output(lp->gpio_phy_nrst, 1);
		msleep(50);
		dev_info(dev, "phy-gpio-nrst %d\n", lp->gpio_phy_nrst);
	} else
		dev_warn(dev, "no reset gpio\n");

	lp->device = &(pdev->dev);
	/* get phy install information */
	err = fgmac4_mdio_register(netdev, pdev);
	if (err) {
		PMSG(dev, KERN_ERR, "Failed to find PHY, aborting!!!\n");
		goto err_out;
	}

	/* use the PHY who's ID is the minimum one of all */
	for (i = 1; i < PHY_MAX_ADDR; i++)
		if (lp->phy_map & (1 << i)) {
			lp->mii.phy_id = i;
			break;
		}
	lp->mii.dev = netdev;

	fgmac4_phy_prepare(&(lp->mii), &(lp->phy_info));

	/* Descriptor and Buffer initialize */
	lp->rx_ring = (struct fgmac4_desc *) dma_alloc_coherent(NULL,
				FGMAC4_DESC_BYTES, &lp->ring_dma, GFP_KERNEL);
	if (lp->rx_ring == NULL) {
		PMSG(dev, KERN_ERR,
			"Failed to alloc DMA descriptor memory!!!\n");
		err = -ENOMEM;
		goto err_out;
	}
	lp->tx_ring = &lp->rx_ring[FGMAC4_RDESC_NUM];
	memset(lp->rx_ring, 0, sizeof(struct fgmac4_desc) * FGMAC4_RDESC_NUM);
	memset(lp->tx_ring, 0, sizeof(struct fgmac4_desc) * FGMAC4_TDESC_NUM);
#ifdef CONFIG_FGMAC4_ALT_DESC
	lp->tx_ring[FGMAC4_TDESC_NUM - 1].opts1 = FGMAC4_TX_DESC_END_RING;
#else
	lp->tx_ring[FGMAC4_TDESC_NUM - 1].opts2 = FGMAC4_TX_DESC_END_RING;
#endif

	lp->rx_buf_sz = lp->tx_buf_sz = BUFFER_SIZE;

	if (lp->rx_buf_sz <= BUFFER1_MAX_CNT) {
		lp->buff1_cnt = lp->rx_buf_sz;
		lp->buff2_cnt = 0;
	} else {
		lp->buff1_cnt = BUFFER1_MAX_CNT;
		lp->buff2_cnt = lp->rx_buf_sz - BUFFER1_MAX_CNT;
	}
#ifdef CONFIG_FGMAC4_USE_BOUNCE_BUF
	err = fgmac4_init_ring(lp, FGMAC4_ALLOC_RING);
	if (err) {
		PMSG(dev, KERN_ERR, "Failed to initialize DMA descriptor!!!\n");
		goto err_out;
	}
#endif /* CONFIG_FGMAC4_USE_BOUNCE_BUF */

	fgmac4_disable_intr_all(lp);
	/* Install the interrupt handler */
	err = request_irq(netdev->irq, fgmac4_interrupt, 0, netdev->name,
								netdev);
	if (err) {
		PMSG(dev, KERN_CRIT, "Failed to register IRQ%d, ERRNO:%d!!!\n",
							netdev->irq, err);
		goto err_out;
	}

	err = register_netdev(netdev);
	if (err) {
		PMSG(dev, KERN_CRIT, "Failed to register netdev. ERRNO:%d!!!\n",
								 err);

		goto err_out_irq;
	}
#ifdef CONFIG_FGMAC4_NAPI
	PMSG(dev, KERN_INFO, "FGMAC4(NAPI/%d) Driver Loaded.\n",
					fgmac4_napi_weight);
#else
	PMSG(dev, KERN_INFO, "FGMAC4 Driver Loaded.\n");
#endif /* CONFIG_FGMAC4_NAPI */

	DBG_FUN_END(dev);
	return 0;

err_out_irq:
	free_irq(netdev->irq, netdev);
err_out:
	/* disable, hold in reset */

	if (lp->gpio_phy_enable)
		err = gpio_direction_output(lp->gpio_phy_enable, 0);
	if (lp->gpio_phy_nrst)
		err = gpio_direction_output(lp->gpio_phy_nrst, 0);

	if (!IS_ERR(lp->h_clk)) {
		clk_disable_unprepare(lp->h_clk);
		clk_put(lp->h_clk);
	}
	if (!IS_ERR(lp->osc_clk)) {
		clk_disable_unprepare(lp->osc_clk);
		clk_put(lp->osc_clk);
	}
	if (!IS_ERR(lp->p_clk)) {
		clk_disable_unprepare(lp->p_clk);
		clk_put(lp->p_clk);
	}

#ifdef CONFIG_FGMAC4_USE_BOUNCE_BUF
	fgmac4_free_ring(lp, FGMAC4_FREE_RING);
#endif /* CONFIG_FGMAC4_USE_BOUNCE_BUF */

	if (lp->rx_ring) {
		dma_free_coherent(NULL, FGMAC4_DESC_BYTES,
					lp->rx_ring, lp->ring_dma);
	}
	if (lp->base_addr)
		iounmap(lp->base_addr);

	if (netdev)
		free_netdev(netdev);
	platform_set_drvdata(pdev, NULL);
	DBG_FUN_ERREND(dev);
	return err;
}

/**
 * fgmac4_remove -- remove the device from the system
 * @pdev: target platform device structure
 *
 * Description: Free the resource that was requested in probe and open
 * process, if it has not been freed.
 * Returns 0 on success, negative value on failure
 */
static int fgmac4_remove(struct platform_device *pdev)
{
#ifdef FGMAC4_DEBUG
	struct device *dev = &pdev->dev;
#endif
	struct net_device *netdev;
	struct fgmac4_private *lp;

	DBG_FUN_START(dev);

	netdev = platform_get_drvdata(pdev);

	lp = netdev_priv(netdev);

	fgmac4_mdio_unregister(netdev);

	/* FIXME: can not disable clocks now */
#if 0
	if (!IS_ERR(lp->h_clk)) {
		clk_disable_unprepare(lp->h_clk);
		clk_put(lp->h_clk);
	}
	if (!IS_ERR(lp->osc_clk)) {
		clk_disable_unprepare(lp->osc_clk);
		clk_put(lp->osc_clk);
	}
	if (!IS_ERR(lp->p_clk)) {
		clk_disable_unprepare(lp->p_clk);
		clk_put(lp->p_clk);
	}

#endif

	/* remove the interrupt handler */
	free_irq(netdev->irq, netdev);

	/* Force Down */
	unregister_netdev(netdev);

	/* Unmap a ioremap()ed region */
	if (lp->base_addr != NULL)
		iounmap(lp->base_addr);

	/* leave it disabled and held in reset */
	gpio_direction_output(lp->gpio_phy_nrst, 0);
	gpio_free(lp->gpio_phy_nrst);
	gpio_direction_output(lp->gpio_phy_enable, 0);
	gpio_free(lp->gpio_phy_enable);

	/* delete driver data */
	platform_set_drvdata(pdev, NULL);

	/* free descriptor space */

#ifdef CONFIG_FGMAC4_USE_BOUNCE_BUF
	fgmac4_free_ring(lp, FGMAC4_FREE_RING);
#endif /* CONFIG_FGMAC4_USE_BOUNCE_BUF */

	dma_free_coherent(NULL, FGMAC4_DESC_BYTES, lp->rx_ring, lp->ring_dma);
	lp->rx_ring = NULL;
	lp->tx_ring = NULL;
	lp->ring_dma = 0;

	/* free the allocated net device */
	free_netdev(netdev);

	DBG_FUN_END(dev);
	return 0;
}

#ifdef FGMAC4_DUMP
/* Dump control */
#define REG_DUMP  0x01
#define DESC_DUMP 0x02
#define MEM_DUMP  0x04
#define WORD0(val) ((val) & 0x000000FF)
#define WORD1(val) (((val) & 0x0000FF00) >> 8)
#define WORD2(val) (((val) & 0x00FF0000) >> 16)
#define WORD3(val) (((val) & 0xFF000000) >> 24)
#define READ_REG(REG) __raw_readl(base_addr + (REG))
/**
 * debug_dump -- dump data in debug mode
 * @dev: net device pointer
 * @base_addr: the base address of FGMAC4's register space
 * @ctl: dump message control
 *       bit0:dump FGMAC4 register
 *       bit1:dump TX/RX descriptors
 *       bit2:dump memory from *startp to (*startp + size)
 * @txring: TX descriptor ring start address
 * @rxring: RX descriptor ring start address
 * @startp: the start address of dump memory.
 * @size: the size of dump meory.
 *
 * Description: This function will be effective when FGMAC4_DEBUG define
 * is enable.
 */
static void debug_dump(struct device *dev, void __iomem *base_addr, int ctl,
		int *txring, int *rxring, char *startp, unsigned int size)
{
	int *p;
	int i, idx, rc;
#define DUMP_BUFSIZ 512
	char buf[DUMP_BUFSIZ], *mp;

	/* Dump Register
	 */
	if (ctl & REG_DUMP) {
		dev_dbg(dev, DUMPH "FGMAC4 Register Dump --------\n");
		dev_dbg(dev, DUMPH
			"MCR(0x%04x)    : %08x     |     "
			"MFFR(0x%04x)   : %08x\n",
			FGMAC4_REG_MCR, READ_REG(FGMAC4_REG_MCR),
			FGMAC4_REG_MFFR, READ_REG(FGMAC4_REG_MFFR));
		dev_dbg(dev, DUMPH
			"MHTRH(0x%04x)  : %08x     |     "
			"MHTRL(0x%04x)  : %08x\n",
			FGMAC4_REG_MHTRH, READ_REG(FGMAC4_REG_MHTRH),
			FGMAC4_REG_MHTRL, READ_REG(FGMAC4_REG_MHTRL));
		dev_dbg(dev, DUMPH
			"FCR(0x%04x)    : %08x     |     "
			"VTR(0x%04x)    : %08x\n",
			FGMAC4_REG_FCR, READ_REG(FGMAC4_REG_FCR),
			FGMAC4_REG_VTR, READ_REG(FGMAC4_REG_VTR));
		dev_dbg(dev, DUMPH
			"RWFFR(0x%04x)  : %08x     |     "
			"PMTR(0x%04x)   : %08x\n",
			FGMAC4_REG_RWFFR, READ_REG(FGMAC4_REG_RWFFR),
			FGMAC4_REG_PMTR, READ_REG(FGMAC4_REG_PMTR));
		dev_dbg(dev, DUMPH
			"MAR0H(0x%04x)  : %08x     |     "
			"MAR0L(0x%04x)  : %08x\n",
			FGMAC4_REG_MAR0H, READ_REG(FGMAC4_REG_MAR0H),
			FGMAC4_REG_MAR0L, READ_REG(FGMAC4_REG_MAR0L));
		dev_dbg(dev, DUMPH
			"BMR(0x%04x)    : %08x     |     "
			"TPDR(0x%04x)   : %08x\n",
			FGMAC4_REG_BMR, READ_REG(FGMAC4_REG_BMR),
			FGMAC4_REG_TPDR, READ_REG(FGMAC4_REG_TPDR));
		dev_dbg(dev, DUMPH
			"RPDR(0x%04x)   : %08x     |     "
			"RDLAR(0x%04x)  : %08x\n",
			FGMAC4_REG_RPDR, READ_REG(FGMAC4_REG_RPDR),
			FGMAC4_REG_RDLAR, READ_REG(FGMAC4_REG_RDLAR));
		dev_dbg(dev, DUMPH
			"TDLAR(0x%04x)  : %08x     |     "
			"SR(0x%04x)     : %08x\n",
			FGMAC4_REG_TDLAR, READ_REG(FGMAC4_REG_TDLAR),
			FGMAC4_REG_SR, READ_REG(FGMAC4_REG_SR));
		dev_dbg(dev, DUMPH
			"OMR(0x%04x)    : %08x     |     "
			"IER(0x%04x)    : %08x\n",
			FGMAC4_REG_OMR, READ_REG(FGMAC4_REG_OMR),
			FGMAC4_REG_IER, READ_REG(FGMAC4_REG_IER));
		dev_dbg(dev, DUMPH
			"MFC(0x%04x)    : %08x     |     "
			"CHTDR(0x%04x)  : %08x\n",
			FGMAC4_REG_MFC, READ_REG(FGMAC4_REG_MFC),
			FGMAC4_REG_CHTDR, READ_REG(FGMAC4_REG_CHTDR));
		dev_dbg(dev, DUMPH
			"CHRDR(0x%04x)  : %08x     |     "
			"CHTBAR(0x%04x) : %08x\n",
			FGMAC4_REG_CHRDR, READ_REG(FGMAC4_REG_CHRDR),
			FGMAC4_REG_CHTBAR, READ_REG(FGMAC4_REG_CHTBAR));
		dev_dbg(dev, DUMPH "CHRBAR(0x%04x) : %08x\n",
			FGMAC4_REG_CHRBAR, READ_REG(FGMAC4_REG_CHRBAR));
	}

	/* Dump TX/RX Descriptor
	 */
	if (ctl & DESC_DUMP) {

		if (txring) {
			dev_dbg(dev, DUMPH "TX Descriptor Dump --------\n");

			p = txring;
			for (i = 0; i < FGMAC4_TDESC_NUM; i++, p += 4) {
				dev_dbg(dev, DUMPH "DESC%03d(0x%08x) :"
					" %02x%02x %02x%02x"
					" %02x%02x %02x%02x "
					" %02x%02x %02x%02x"
					" %02x%02x %02x%02x\n",
					i, (unsigned int) p,
					WORD3(*p), WORD2(*p),
					WORD1(*p), WORD0(*p),
					WORD3(*(p + 1)), WORD2(*(p + 1)),
					WORD1(*(p + 1)), WORD0(*(p + 1)),
					WORD3(*(p + 2)), WORD2(*(p + 2)),
					WORD1(*(p + 2)), WORD0(*(p + 2)),
					WORD3(*(p + 3)), WORD2(*(p + 3)),
					WORD1(*(p + 3)), WORD0(*(p + 3)));
			}
			dev_dbg(dev, "\n");
		}

		if (rxring) {
			dev_dbg(dev, DUMPH "RX Descriptor Dump --------\n");

			p = rxring;
			for (i = 0; i < FGMAC4_RDESC_NUM; i++, p += 4) {
				dev_dbg(dev, DUMPH "DESC%03d(0x%08x) :"
					" %02x%02x %02x%02x"
					" %02x%02x %02x%02x "
					" %02x%02x %02x%02x"
					" %02x%02x %02x%02x\n",
					i, (unsigned int) p,
					WORD3(*p), WORD2(*p),
					WORD1(*p), WORD0(*p),
					WORD3(*(p + 1)), WORD2(*(p + 1)),
					WORD1(*(p + 1)), WORD0(*(p + 1)),
					WORD3(*(p + 2)), WORD2(*(p + 2)),
					WORD1(*(p + 2)), WORD0(*(p + 2)),
					WORD3(*(p + 3)), WORD2(*(p + 3)),
					WORD1(*(p + 3)), WORD0(*(p + 3)));
			}
			dev_dbg(dev, "\n");
		}
	}

	/* Dump Memory
	 */
	if (ctl & MEM_DUMP) {
		if (size == 0) {
			dev_dbg(dev, DBGH "Passed SIZE is 0. No Dump!\n");
			return;
		}

		dev_dbg(dev, DUMPH "Memory Dump --------\n");


		mp = (char *) startp;
		idx = 0;
		for (i = 0; i < size; i++) {
			if ((i % 16) == 0) {
				idx = 0;
				dev_dbg(dev, DUMPH "0x%08x :",
						(unsigned int) mp);
			}

			rc = snprintf(&buf[idx], DUMP_BUFSIZ - idx, " %02x",
									*mp);
			idx += rc;

			if ((i + 1) == size) {	/* the last one */
				snprintf(&buf[idx], DUMP_BUFSIZ - idx, "\n");
				dev_dbg(dev, "%s", buf);
			} else {
				if ((i + 1) % 16 == 0) {
					snprintf(&buf[idx], DUMP_BUFSIZ - idx,
									"\n");
					dev_dbg(dev, "%s", buf);
				}
				mp++;
			}
		}
	}
}
#endif /* FGMAC4_DUMP */

static const struct of_device_id mb8ac0300_fgmac4_dt_ids[] = {
	{ .compatible = "fujitsu,fgmac4" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, mb8ac0300_fgmac4_dt_ids);

#ifdef CONFIG_PM

/**
 * __fgmac4_suspend -- this function will be called when entering pm state
 * @dev: target net device structure
 * @free_ring: buffers need to be freed
 *
 * Description: In this function, the driver will stop timer, queue, device,
 * store multicast setup registers and free buffers if needed.
 * Returns 0 on success, negative value on failure
 */
static int __fgmac4_suspend(struct device *dev, int free_ring)
{
	struct net_device *netdev = dev_get_drvdata(dev);
	struct fgmac4_private *lp = netdev_priv(netdev);
	unsigned long flags;

	if (!netif_running(netdev))
		return 0;

	del_timer_sync(&lp->timer);
	netif_carrier_off(netdev);
	netif_stop_queue(netdev);
#ifdef CONFIG_FGMAC4_NAPI
	napi_disable(&lp->napi);
#endif /* CONFIG_FGMAC4_NAPI */

	spin_lock_irqsave(&lp->lock, flags);
	/* multicast setting saved */
	lp->mhtrh = fgmac4_reg_read(lp->base_addr, FGMAC4_REG_MHTRH);
	lp->mhtrl = fgmac4_reg_read(lp->base_addr, FGMAC4_REG_MHTRL);
	lp->mffr = fgmac4_reg_read(lp->base_addr, FGMAC4_REG_MFFR);

	fgmac4_stop_locked(netdev);
	spin_unlock_irqrestore(&lp->lock, flags);

	/* free descriptor and buffer space */
	fgmac4_free_ring(lp, free_ring);

	return 0;
}

/**
 * __fgmac4_resume -- this function will be called when leaving pm state
 * @dev: target net device structure
 * @alloc_ring: buffers need to be allocated
 *
 * Description: In this function, the driver will init ring, start device,
 * restore multicast setup registers and start the queue and timer.
 * Returns 0 on success, negative value on failure
 */
static int __fgmac4_resume(struct device *dev, int alloc_ring)
{
	struct net_device *netdev = dev_get_drvdata(dev);
	struct fgmac4_private *lp = netdev_priv(netdev);
	int err;
	unsigned long flags;

	if (!netif_running(netdev))
		return 0;

	err = fgmac4_init_ring(lp, alloc_ring);
	if (err) {
		PMSG(dev, KERN_ERR, "Failed to initialize DMA descriptor!!!\n");
		return err;
	}
#ifdef CONFIG_FGMAC4_NAPI
	napi_enable(&lp->napi);
#endif /* CONFIG_FGMAC4_NAPI */

	spin_lock_irqsave(&lp->lock, flags);

	err = fgmac4_start_locked(netdev);
	if (err) {
		PMSG(dev, KERN_ERR, "Can not start fgmac4\n");
		goto err_start_lock;
	}

	/* multicast setting restore */
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_MHTRH, lp->mhtrh);
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_MHTRL, lp->mhtrl);
	fgmac4_reg_write(lp->base_addr, FGMAC4_REG_MFFR, lp->mffr);
	if (eee) {
		fgmac4_init_eee(lp);
		fgmac4_adjust_eee_link(lp, mii_link_ok(&lp->mii));
	}
	spin_unlock_irqrestore(&lp->lock, flags);

	if (mii_link_ok(&lp->mii)) {
		netif_carrier_on(netdev);
		/* start queue to allow transmit */
		netif_start_queue(netdev);
	}

	/* Initialize timer function which will check the state
	 * of carrier timely
	 */
	init_timer(&lp->timer);
	lp->timer.expires = CHECK_CARRIER_TIME;
	lp->timer.data = (unsigned long) netdev;
	lp->timer.function = fgmac4_check_carrier;
	add_timer(&lp->timer);
	return 0;

err_start_lock:
	spin_unlock_irqrestore(&lp->lock, flags);

#ifdef CONFIG_FGMAC4_NAPI
	napi_disable(&lp->napi);
#endif /* CONFIG_FGMAC4_NAPI */
	/* free descriptor and buffer space */
	fgmac4_free_ring(lp, alloc_ring);

	return err;
}

static int fgmac4_suspend(struct device *dev)
{
	return __fgmac4_suspend(dev, !FGMAC4_FREE_RING);
}

static int fgmac4_resume(struct device *dev)
{
	return __fgmac4_resume(dev, !FGMAC4_ALLOC_RING);
}

static int fgmac4_freeze(struct device *dev)
{
#ifdef CONFIG_FGMAC4_USE_BOUNCE_BUF
	return __fgmac4_suspend(dev, !FGMAC4_FREE_RING);
#else
	return __fgmac4_suspend(dev, FGMAC4_FREE_RING);
#endif
}

static int fgmac4_thaw(struct device *dev)
{
#ifdef CONFIG_FGMAC4_USE_BOUNCE_BUF
	return __fgmac4_resume(dev, !FGMAC4_ALLOC_RING);
#else
	return __fgmac4_resume(dev, FGMAC4_ALLOC_RING);
#endif
}

static int fgmac4_restore(struct device *dev)
{
	struct net_device *netdev = dev_get_drvdata(dev);
	struct fgmac4_private *lp = netdev_priv(netdev);

	/* set init_media flag to not init state */
	lp->init_media = 0;

#ifdef CONFIG_FGMAC4_USE_BOUNCE_BUF
	return __fgmac4_resume(dev, !FGMAC4_ALLOC_RING);
#else
	return __fgmac4_resume(dev, FGMAC4_ALLOC_RING);
#endif
}

static const struct dev_pm_ops fgmac4_pm_ops = {
	.suspend = fgmac4_suspend,
	.resume = fgmac4_resume,
	.freeze = fgmac4_freeze,
	.thaw = fgmac4_thaw,
	.restore = fgmac4_restore,
};

#define FGMAC4_PM_OPS (&fgmac4_pm_ops)
#else
#define FGMAC4_PM_OPS NULL
#endif

/* initialize the platform_driver structure */
static struct platform_driver fgmac4_driver = {
	.probe   = fgmac4_probe,
	.remove  = fgmac4_remove,
	.driver  = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = mb8ac0300_fgmac4_dt_ids,
		.pm = FGMAC4_PM_OPS,
	},
};

static int __init fgmac4_module_init(void)
{
	return platform_driver_register(&fgmac4_driver);
}

module_init(fgmac4_module_init)

static void __exit fgmac4_module_exit(void)
{
	platform_driver_unregister(&fgmac4_driver);
}

module_exit(fgmac4_module_exit)

MODULE_DESCRIPTION("F_GMAC4 Ethernet driver");
MODULE_AUTHOR("Fujitsu Semiconductor Limited");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:"DRIVER_NAME);

#ifdef CONFIG_FGMAC4_NAPI
/* NAPI weight */
module_param(fgmac4_napi_weight, int, 32);
MODULE_PARM_DESC(fgmac4_napi_weight,
		"Processing number of packets with NAPI poll function.");
#endif	/* CONFIG_FGMAC4_NAPI */
