/*
 * Copyright (C) 2013-2014 Fujitsu Semiconductor Ltd
 *
 * F_PCIE2_DME functions for Fujitsu SoC
 *
 * Author: Slash Huang <slash.huang@tw.fujitsu.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/skbuff.h>
#include <linux/platform_device.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>
#include <asm/dma-iommu.h>
#include <linux/sched.h>
#include <linux/msi.h>
#include <linux/iommu.h>
#include "pcie_f_pcie2_dme.h"
#include <linux/spinlock.h>

#define PCIE_TRANS_STAT				0x844
#define PCIE_TRANS_STAT_DL_ACT			(1 << 6)
#define PCIE_PWR_CSR				0x408
#define PCIE_PWR_CTL_PERST			1
#define BRIDGE_MODE				0x880
#define DL_WRITE_MODE				(1 << 3)
#define SLA_TRANS_EN				0x884
#define ICS_IF_ENABLE				(1 << 8)
#define TRS_IF_ENABLE				(1 << 0)
#define BAR_EN_REG				0x820
#define EN_BAR(x)				(1 << x)
#define SLA_CFG_BASE_ADDR			0x1100
#define SLA_IO_CFG_STA				0x100C
#define CFG_BZ					1
#define SLA_CONF_TAR_ID0			0x1110
#define SLA_CONF_TAR_ID(a)			(SLA_CONF_TAR_ID0 + a * 4)
#define CFG_TLP_TYPE(a)				(a << 16)
#define BUS_NUM(a)				(a << 8)
#define DEV_NUM(a)				(a << 3)
#define FUN_NUM(a)				(a << 0)
#define TYPE0_ISSUE				0
#define TYPE1_ISSUE				1
#define SLA_MEM_BAR(a)				(0x1820 + a * 16)
#define SLA_MEM_BAR_REMAP_SZ(a)			(0x1824 + a * 16)
#define SLA_MEM_REMAP_SZ			0xF0000000
#define SLA_MEM_BAR_REMAP_L(a)			(0x1828 + a * 16)
#define SLA_MEM_BAR_REMAP_U(a)			(0x182C + a * 16)
#define SLA_IO_BAR(a)				(0x1200 + a * 16)
#define SLA_IO_BAR_REMAP_SZ(a)			(0x1204 + a * 16)
#define SLA_IO_BAR_REMAP_ADD(a)			(0x1208 + a * 16)
#define SLA_IO_REMAP_SZ				0xF0000000
#define BAR_RES_SET(x)				(0x804 + x * 4)
#define SLA_MEM_INT_ST				0x1800
#define SLA_MEM_INT_MASK			0x1804
#define MEM_RD_COMPLE_ABORT			(1 << 0)
#define MEM_RD_UNSUPPORT_REQ			(1 << 1)
#define MEM_RD_COMPLE_TIMEOUT			(1 << 2)
#define MEM_RD_POISONED_TLP			(1 << 3)
#define MEM_RD_TRANS_REQ			(1 << 24)
#define SLA_IO_CFG_INT_ST			0x1000
#define SLA_IO_CFG_INT_MASK			0x1004
#define IO_CFG_INT_ST_CLR			0xffffffff
#define CFG_RD_COMPLETION_ABORT			(1 << 0)
#define CFG_RD_UNSUPPORTED_REQ			(1 << 1)
#define CFG_RD_COMPLETE_TIMEOUT			(1 << 2)
#define CFG_RD_POISONED_TLP			(1 << 3)
#define CFG_RD_CFG_RETRY_STATUS			(1 << 4)
#define CFG_WT_COMPLETION_ABORT			(1 << 5)
#define CFG_WT_UNSUPPORTED_REQ			(1 << 6)
#define CFG_WT_COMPLETE_TIMEOUT			(1 << 7)
#define CFG_WT_CFG_RETRY_STATUS			(1 << 8)
#define DT_CONF_SPACE				0
#define BAR_SET_PREFETCHABLE			(1 << 3)
#define BAR_SET_MEMORY_TYPE_32BIT		(0 << 1)
#define BAR_SET_MEMORY_TYPE_64BIT		(2 << 1)
#define BAR_SET_IO_SPACE			(1 << 0)
#define BAR_SET_MEM_SPACE			(0 << 0)
#define EROMR_SETTING				0x81C
#define FUNC_SEL_BAR_REMAP			0x08A0
#define AXI_MASTER_REMAP_ADDR(x)		(0x8A4 + x * 4)
#define EROM_EN					(1 << 6)
#define BAR_EN(x)				(1 << x)
#define PCI_SLAVE_BUFFER			(512 * SZ_1K)
#define MSG_INT_A_D(x)				(x)
#define SLA_MEM_INT				5
#define DMAC_INSTR				0xC00
#define DMA_STOP				(0 << 4)
#define DMA_START				(1 << 4)
#define DMA_INT_CLR				(2 << 4)
#define DMA_NUM(x)				(x)
#define DMA_MODE_CONL				0xC04
#define DMA_STATUS				0xC08
#define DMA_NORMAL				0x00
#define DMA_TRANS_BUSY				0x01
#define DMA_ABORT_ENDERR			0x10
#define DMA_INT_MASK				0xC0C
#define DEM_DMA_PCIE_L_ADDR_CH(x)		(0xE00 + x * 0x20)
#define DEM_DMA_PCIE_U_ADDR_CH(x)		(0xE04 + x * 0x20)
#define DEM_DMA_AXI_ADDR_CH(x)			(0xE08 + x * 0x20)
#define DEM_DMA_TRANS_SZ_CH(x)			(0xE0C + x * 0x20)
#define DEM_DMA_TRANS_SET_CH(x)			(0xE10 + x * 0x20)
#define MULTI_R_REQ_1_MULTI			(1 << 20)
#define TAB_ADDR(x)				(0xE14 + x * 0x20)
#define DEM_DMA_TRAN_SAT_CH(x)			(0xE18 + x * 0x20)
#define DMA_TRAN_SPL_SZ(x)			(x)
#define DMA_TRANS_ABORT_BY_USER			(1 << 7)
#define DMA_TRANS_ABORT_RESET_DET		(1 << 6)
#define DMA_TRANS_ABORT_POISO_DET		(1 << 5)
#define DMA_TRANS_ABORT_BY_PCIE			(0x111 << 2)
#define COMPLETION_TIMEOUT(x)			(x & 0x100)
#define UNSUPPORTED_REQ(x)			(x & 0x10)
#define COMPLETION_ABORT(x)			(x & 0x1)
#define AXI_ERROR_DETECT			(1 << 1)
#define TRANS_COMPLETE				(1 << 0)
/* (PCIe: Read, AXI: Write) */
#define DMA_TRANS_PCIE2AXI			(0 << 19)
 /* (AXI: Read, PCIe Write) */
#define DMA_TRANS_AXI2PCIE			(1 << 19)
#define DMA_TYPE_DEMAND				0
#define DMA_TYPE_DESCRIPT			1
#define FUNC_SEL_BAR_REMAP			0x08A0
#define AXI_MASTER_REMAP_ADDR(x)		(0x8A4 + x * 4)
#define AXI_MASTER_ERR_INT_MASK			0x904
#define WAIT_DMA_TIMEOUT			msecs_to_jiffies(100)
#define WAIT_MEM_TIMEOUT			usecs_to_jiffies(5)
#define SAT_DMA_WAIT				0
#define SAT_DMA_DONE				1
#define DMXH_DMA_CH				4
#define DES_DMA_LAST_DESC			31
#define DES_DMA_DESC_ID				16
#define DES_DMA_TB_L_ADDR			0x00
#define DES_DMA_TB_U_ADDR			0x01
#define DES_DMA_TB_AXI_ADDR			0x02
#define DES_DMA_TB_TRANS_SZ			0x03
#define DES_DMA_TB_TRANS_SET			0x04
#define DES_DMA_TB_ADDR_SET			0x05
#define DES_DMA_TB_LDID_SET			0x06
#define TEST_NUM				6
#define DMA_INT_NUM				4
#define ST_DMA_INT_ENB				0x28
#define EN_DMA_INT				0xffff
#define LINK_STATUS				0x92
#define LINK_SPEED_MASK				0x0F
#define LINK_WIDTH_MASK				0xF0
#define LINK_WIDTH_1				(1 << 0)
#define LINK_WIDTH_2				(1 << 1)
#define LINK_WIDTH_4				(1 << 2)
#define LINK_WIDTH_8				(1 << 3)
#define LINK_SPEED_2_5				1
#define LINK_SPEED_5_0				2
#define PCIE_WAP_LIG_INT			10
#define PCIE_MSIX_CTL				0xE2
#define PCIE_MSIX_TABLE				0xE4
#define PCIE_MSIX_PBA				0xE8
#define PCIE_MSI_CTL				0xC2
#define PCIE_MSI_ADDR_LO			0xC4
#define PCIE_MSI_ADDR_HI			0xC8
#define PCIE_MSI_DATA				0xCC
#define MSIX_ADDR_L(a)				(0x700 + a * 0x10)
#define MSIX_ADDR_H(a)				(0x704 + a * 0x10)
#define MSIX_DATA(a)				(0x708 + a * 0x10)
#define PCIE_LIG				0x1200
#define LIG_INT_REQ				0X00
#define LIG_INT_STS				0X04
#define LIG_INT_EN				0X08
#define LIG_INT_CLR				0X0C
#define SET_LIG_ENABLE				0xffffffff
#define INT_INTX				0
#define INT_MSI				1
#define INT_MSIX				2
#define MSI_HW_INT_N				16
#define EP_NAME_LEN				20

struct desc_tab {
	dma_addr_t dma_table_phys;
	dma_addr_t pcie_lower_addr;
	dma_addr_t pcie_upper_addr;
	dma_addr_t axi_addr;
	dma_addr_t next_tab_addr;
	u32 *dma_tab;
	u32 *pcie_buf;
	u32 *axi_buf;
	u32 traf_size;
	u32 traf_set;
	u32 ld_descrid;
	u32 descr_cont;
};

struct dme_ep {
	void __iomem *pcie_addr;
	void __iomem *regs;
	struct device *dev;
	struct clk **clk;
	struct msix_entry *msix_entries;
	int dma_irq[4];
	int dma_done;
	int index;
	dma_addr_t *remap_addr;
	dma_addr_t remap_addr_phys;
	u32 device_id;
	u32 vendor_id;
	u32 clk_num;
	u32 dma_type; /* Demand mode:0, Descript mode:1 */
	u32 remap_addr_len;
	u32 act_dma_number;
	wait_queue_head_t dma_wait;
	u32 dma_wait_timeout;
	u32 loop_coned;
	u32 inited;
	u32 msix_count;
	u32 int_type;
};

u32 test_patten[TEST_NUM] = {
	0xFFFFFFFF,
	0xF0F0F0F0,
	0x0F0F0F0F,
	0x55555555,
	0x50505050,
	0x05050505
};

struct pcie_port pcie_port = {
	.rc_cnt = 0,
	.dme_pcie = NULL,
	.pcie_por_num = 0,
	.pcie_por_num_total = 0,
	.pcie_por_num_release = 0,
};

static DEFINE_SPINLOCK(pcie_older_lock);


void f_pcie_dev_set_platdata(struct device *dev, void *data)
{
	dev->platform_data = data;
}

struct f_pcie_port *f_get_pcie_port(int index)
{
	struct f_pcie_port *port = NULL, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&pcie_port.lock, flags);
	list_for_each_entry_safe(port, tmp, &pcie_port.list, ports) {
		if (port->index == index) {
			spin_unlock_irqrestore(&pcie_port.lock, flags);
			return port;
		}
	}

	spin_unlock_irqrestore(&pcie_port.lock, flags);
	pr_info("can't find pcie port\n");
	return NULL;
}

struct f_pcie_port *f_get_pcie_ep_port(u32 device_id)
{
	struct f_pcie_port *port, *tmp;
	struct dme_ep *ep;
	unsigned long flags;

	spin_lock_irqsave(&pcie_port.lock, flags);
	list_for_each_entry_safe(port, tmp, &pcie_port.list, ports) {
		ep = port->ep;
		if (!ep)
			continue;

		if (ep->device_id == device_id) {
			spin_unlock_irqrestore(&pcie_port.lock, flags);
			return port;
		}
	}
	spin_unlock_irqrestore(&pcie_port.lock, flags);
	pr_info("can't find EP port (device : 0x%x)\n", device_id);

	return NULL;
}

int f_pcie_host_link_up(struct dme_rc *rc)
{
	int retries = 20, link_up;

	while (retries) {
		link_up = readb(rc->rc_cfg_base + PCIE_TRANS_STAT);
		link_up &= PCIE_TRANS_STAT_DL_ACT;

		if (!link_up)
			retries--;
		else
			break;

		usleep_range(10000, 20000);
	}

	rc->link_up = link_up;
	if (rc->link_up)
		dev_info(rc->dev, "PCIe port %d link up\n", rc->index);
	else
		dev_info(rc->dev, "PCIe port %d link down\n", rc->index);

	return 0;
}


void f_pcie_host_init_rc(struct dme_rc *rc)
{
	void __iomem *base = rc->rc_cfg_base;
	u32 dl_sat, i, res_low, res_hi, res_mask, msi_multi;
	u32 now_link_lan, now_link_sp, link_sat, addr;

	f_pcie_host_link_up(rc);
	if (!rc->link_up)
		return;

	/* device DL write enable */
	dl_sat = readl(base + BRIDGE_MODE);
	dl_sat |= DL_WRITE_MODE;
	writel(dl_sat, base + BRIDGE_MODE);

	/* init pcie vendor class number */
	writew(PCI_CLASS_BRIDGE_PCI, base + PCI_CLASS_DEVICE);

	/* init pcie vendor number */
	writew(PCI_VENDOR_ID_FUJITSU_ME, base + PCI_VENDOR_ID);

	/*
	 * The numbers possessed by MSI Table are indicated
	 * S73 supply (1-16)
	 * 000h: 1 / 100: 16
	 */
	if (of_property_read_u32(rc->dev->of_node, "msi-multi",
		&msi_multi)) {
		dev_warn(rc->dev, "Missing msi-multi in dt\n");
		msi_multi = 0;
	}

	res_mask = readb(base + PCIE_MSI_CTL);
	res_mask &= ~0x0e;
	res_mask |= (msi_multi << 1);
	writeb(res_mask, base + PCIE_MSI_CTL);

	/*
	 * MSI-X
	 * S73 supply (1-32)
	 * 0x0h: 1 / 0x1F: 32
	 * setting  MSIX table count 32
	 * setting  MSIX address table offset in 0x1000 of base 0
	 * setting  MSIX Pending table offset in 0x1008 of base 0
	 */
	res_mask = readb(base + PCIE_MSIX_CTL);
	res_mask &= ~(PCI_MSIX_FLAGS_QSIZE);
	res_mask |= 0x1f;
	writeb(res_mask, base + PCIE_MSIX_CTL);
	writel(0x1000, base + PCIE_MSIX_TABLE);
	writel(0x1008, base + PCIE_MSIX_PBA);

	/* 512K DDR */
	addr = 0xFFF80000;
	res_mask = addr | BAR_SET_MEMORY_TYPE_32BIT | BAR_SET_MEM_SPACE;

	/* BAR 0 enable  */
	writel(res_mask, base + BAR_RES_SET(0));

	/* device DL write disenable */
	dl_sat = dl_sat & ~DL_WRITE_MODE;
	writel(dl_sat, base + BRIDGE_MODE);

	writel(0, base + BAR_RES_SET(1));
	writel(1, base + BAR_EN_REG);

	/* Enable AXI Slave IO/Config Interrupt */
	writel(0, base + SLA_MEM_INT_MASK);

	/* Enable AXI Slave Memory Interrupt Status */
	writel(0, base + SLA_IO_CFG_INT_MASK);

	/* Configure space */
	writel(rc->cfg_res.start, base + SLA_CFG_BASE_ADDR);
	writel(0, base + SLA_CONF_TAR_ID0);

	/* I/O memory */
	for (i = 0; i <= rc->mem_num; i++) {
		res_low = (u32)rc->mem_res[i].start;
		writel(rc->mem_offset[i], base + SLA_IO_BAR(i));
		writel(SLA_IO_REMAP_SZ, base + SLA_IO_BAR_REMAP_SZ(i));
		writel(res_low, base + SLA_IO_BAR_REMAP_ADD(i));
	}

	/* slave memory */
	for (i = 0; i <= rc->mem_num; i++) {
		res_hi = (u64)rc->mem_res[i].start >> 32;
		res_low = (u32)rc->mem_res[i].start;
		res_mask = (u32)rc->mem_offset[i];
		writel(rc->mem_offset[i], base + SLA_MEM_BAR(i));
		writel(SLA_MEM_REMAP_SZ, base + SLA_MEM_BAR_REMAP_SZ(i));
		writel(res_low, base + SLA_MEM_BAR_REMAP_L(i));
		writel(res_hi, base + SLA_MEM_BAR_REMAP_U(i));
	}

	link_sat = readw(base + LINK_STATUS);
	now_link_lan = ((link_sat & LINK_WIDTH_MASK) >> 4);
	switch (now_link_lan) {
	case LINK_WIDTH_1:
		dev_info(rc->dev, "pcie Host:now link lan x1\n");
	break;
	case LINK_WIDTH_2:
		dev_info(rc->dev, "pcie Host:now link lan x2\n");
	break;
	case LINK_WIDTH_4:
		dev_info(rc->dev, "pcie Host:now link lan x4\n");
	break;
	case LINK_WIDTH_8:
		dev_info(rc->dev, "pcie Host:now link lan x8\n");
	break;
	default:
		dev_info(rc->dev, "pcie Host:now link lan not support!\n");
	break;
	}

	now_link_sp = (link_sat & LINK_SPEED_MASK);
	switch (now_link_sp) {
	case LINK_SPEED_2_5:
		dev_info(rc->dev, "now link speed 2.5GT/s\n");
	break;

	case LINK_SPEED_5_0:
		dev_info(rc->dev, "now link speed 5.0GT/s\n");
	break;

	default:
		dev_info(rc->dev, "not support!\n");
	break;
	}

	/* Enable ICS */
	writel(ICS_IF_ENABLE | TRS_IF_ENABLE, base + SLA_TRANS_EN);

	/* BAR , EXROM Enable */
	writeb(EN_BAR(0) | EN_BAR(1), base + BAR_EN_REG);
	rc->inited = 1;
}

void f_pcie_host_clear_reset(struct dme_rc *rc)
{
	void __iomem *base = rc->rc_cfg_base;
	int retry_power = 20, val;

	val = readl(base + PCIE_PWR_CSR) & ~PCIE_PWR_CTL_PERST;
	do {
		writel(val, base + PCIE_PWR_CSR);
		val = readl(base + PCIE_PWR_CSR) & PCIE_PWR_CTL_PERST;
		if (!val)
			return;

		usleep_range(100, 500);
		retry_power--;
	} while (retry_power > 0);

	dev_err(rc->dev, "can't clear PCIE_PWR_CTL_PERST bit\n");
}

void f_pcie_host_enable_cmd(void __iomem *base)
{
	u32 cmd;

	cmd = readw(base + PCI_COMMAND);
	cmd |= PCI_COMMAND_IO;
	cmd |= PCI_COMMAND_MEMORY;
	cmd |= PCI_COMMAND_MASTER;
	writew(cmd, base + PCI_COMMAND);
}

int f_pcie_host_rd_conf(struct dme_rc *rc, struct pci_bus *bus,
			u32 devfn, int where, int size, u32 *val)
{
	void __iomem *reg = NULL;
	int stat, addr, times = 50;

	/* root cfg */
	reg = rc->rc_cfg_base;

	/* device cfg */
	if (bus->number > rc->root_bus_nr) {
		reg = rc->ep_cfg_base;

		addr = BUS_NUM(bus->number) | DEV_NUM(PCI_SLOT(devfn)) |
				FUN_NUM(PCI_FUNC(devfn));

		if (bus->number == rc->root_bus_nr + 1)
			addr |= CFG_TLP_TYPE(TYPE0_ISSUE);
		else
			addr |= CFG_TLP_TYPE(TYPE1_ISSUE);

		writel(addr, rc->rc_cfg_base + SLA_CONF_TAR_ID(0));
	}

	if (bus->number != rc->root_bus_nr) {
		do {
			stat = readb(rc->rc_cfg_base + SLA_IO_CFG_STA) & CFG_BZ;
			if (!stat)
				break;
			times--;
			usleep_range(100, 500);
		} while (times >= 0);

		if (times < 0) {
			dev_err(rc->dev, "PCIBIOS_SET_FAILED:Timeout!\n");
			return PCIBIOS_SET_FAILED;
		}
	}

	switch (size) {
	case 1:
	*val = readb(reg + where);
	break;

	case 2:
	*val = readw(reg + where);
	break;

	case 4:
	*val = readl(reg + where);
	break;

	default:
		return PCIBIOS_FUNC_NOT_SUPPORTED;
	}

	stat = readl(rc->rc_cfg_base + SLA_IO_CFG_INT_ST);
	writel(IO_CFG_INT_ST_CLR, rc->rc_cfg_base + SLA_IO_CFG_INT_ST);

	if (stat & CFG_RD_UNSUPPORTED_REQ && where == PCI_VENDOR_ID)
		*val = 0;
	else if (stat & 0xff)
		return PCIBIOS_FUNC_NOT_SUPPORTED;

	return PCIBIOS_SUCCESSFUL;
}

int f_pcie_host_wr_conf(struct dme_rc *rc, struct pci_bus *bus,
			u32 devfn, int where, int size, u32 val)
{
	int stat, addr, ret = PCIBIOS_SUCCESSFUL, times = 20;
	void __iomem *reg = NULL;

	 /* root cfg */
	reg = rc->rc_cfg_base;

	 /* device cfg */
	if (bus->number > rc->root_bus_nr) {
		reg = rc->ep_cfg_base;

		addr = BUS_NUM(bus->number) | DEV_NUM(PCI_SLOT(devfn)) |
				FUN_NUM(PCI_FUNC(devfn));

		if (bus->number == rc->root_bus_nr + 1)
			addr |= CFG_TLP_TYPE(TYPE0_ISSUE);
		else
			addr |= CFG_TLP_TYPE(TYPE1_ISSUE);

		writel(addr, rc->rc_cfg_base + SLA_CONF_TAR_ID(0));
	}

	if (bus->number != rc->root_bus_nr) {
		do {
			stat = readb(rc->rc_cfg_base + SLA_IO_CFG_STA) & CFG_BZ;
			if (!stat)
				break;
			times--;
			usleep_range(100, 500);
		} while (times >= 0);

		if (times < 0) {
			dev_err(rc->dev, "PCIBIOS_SET_FAILED:Timeout!\n");
			return PCIBIOS_SET_FAILED;
		}
	}

	writel(IO_CFG_INT_ST_CLR, rc->rc_cfg_base + SLA_IO_CFG_INT_ST);

	switch (size) {
	case 1:
	writeb(val, reg + where);
	ret = PCIBIOS_SUCCESSFUL;
	break;

	case 2:
	writew(val, reg + where);
	ret = PCIBIOS_SUCCESSFUL;
	break;

	case 4:
	writel(val, reg + where);
	ret = PCIBIOS_SUCCESSFUL;
	break;

	default:
		return PCIBIOS_FUNC_NOT_SUPPORTED;
	}

	stat = readl(rc->rc_cfg_base + SLA_IO_CFG_INT_ST);
	writel(IO_CFG_INT_ST_CLR, rc->rc_cfg_base + SLA_IO_CFG_INT_ST);
	if (stat)
		return PCIBIOS_FUNC_NOT_SUPPORTED;

	return ret;
}

static void f_pci_process_bridge_of_ranges(struct dme_rc *rc,
			struct device_node *dev_node, int primary)
{
	struct resource res, *res_sav;
	struct of_pci_range_parser parser;
	struct of_pci_range range;
	int memno = 0, iono = 0;

	if (of_pci_range_parser_init(&parser, dev_node)) {
		dev_err(rc->dev, "missing \"ranges\" property\n");
		return;
	}

	for_each_of_pci_range(&parser, &range) {
		of_pci_range_to_resource(&range, dev_node, &res);

		switch (res.flags & IORESOURCE_TYPE_BITS) {
		case IORESOURCE_IO:
			if (iono >= IO_MEM_RCS_NUM)
				continue;

			rc->io_offset[iono] = range.cpu_addr;
			res_sav = &rc->io_res[iono];

			if (res_sav != NULL) {
				rc->io_num = iono++;
				memcpy(res_sav, &res, sizeof(res));
			}
			break;

		case IORESOURCE_MEM:
			if (memno >= IO_MEM_RCS_NUM)
				continue;

#ifndef CONFIG_ARM_LPAE
			if ((range.cpu_addr >> 32))
				continue;
#endif
			rc->mem_offset[memno] = range.pci_addr;
			res_sav = &rc->mem_res[memno];

			if (res_sav != NULL) {
				rc->mem_num = memno++;
				memcpy(res_sav, &res, sizeof(res));
			}
		break;

		case DT_CONF_SPACE:
			memcpy(&rc->cfg_res, &res, sizeof(res));

		break;
		default:
		break;
		}
	}
}

static int f_pcie_host_setup(int nr, struct pci_sys_data *sys)
{
	struct f_pcie_port *dme_port = NULL;
	struct dme_rc *rc = NULL;
	int memno = 0, port_num;

	for (port_num = 0; port_num < pcie_port.pcie_por_num; port_num++) {
		dme_port = f_get_pcie_port(port_num);
		if (!dme_port)
			continue;
		if (!dme_port->rc)
			continue;

		if (dme_port->rc->root_bus_nr < 0) {
			pcie_port.nr_port_map[nr] = port_num;
			break;
		}
	}

	if (!dme_port || !dme_port->rc) {
		pr_err("%s %d:dme_port or rc is NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	sys->private_data = dme_port->rc;
	rc = dme_port->rc;
	rc->root_bus_nr = sys->busnr;

	/* set_local_bus_nr */
	writeb(sys->busnr, rc->rc_cfg_base + PCI_PRIMARY_BUS);

	f_pcie_host_enable_cmd(rc->rc_cfg_base);

	while (memno <= rc->mem_num) {
		pci_add_resource_offset(&sys->resources,
			&rc->mem_res[memno], sys->mem_offset);
		memno++;
	}

	return 1;
}

static int f_pcie_valid_config(struct dme_rc *rc, struct pci_bus *bus, int dev)
{
	/*
	 * Don't go out when trying to access nonexisting devices
	 * on the local bus.
	 */
	if (bus->number == rc->root_bus_nr && dev > 0)
		return 0;

	if (bus->primary == rc->root_bus_nr && dev > 0)
		return 0;

	return 1;
}

static int f_pcie_rd_conf(struct pci_bus *bus,
			u32 devfn, int where, int size, u32 *val)
{
	struct dme_rc *rc;
	struct pci_sys_data *sys = bus->sysdata;
	unsigned long flags;
	int ret = 0;

	rc = (struct dme_rc *)sys->private_data;
	if (f_pcie_valid_config(rc, bus, PCI_SLOT(devfn)) == 0) {
		*val = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	if (rc->link_up) {
		spin_lock_irqsave(&rc->conf_lock, flags);
		ret = f_pcie_host_rd_conf(rc, bus, devfn, where, size, val);
		spin_unlock_irqrestore(&rc->conf_lock, flags);
	}
	return ret;
}

static int f_pcie_wr_conf(struct pci_bus *bus,
			u32 devfn, int where, int size, u32 val)
{
	struct dme_rc *rc;
	struct pci_sys_data *sys = bus->sysdata;
	unsigned long flags;
	int ret = 0;

	rc = (struct dme_rc *)sys->private_data;
	if (f_pcie_valid_config(rc, bus, PCI_SLOT(devfn)) == 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (rc->link_up) {
		spin_lock_irqsave(&rc->conf_lock, flags);
		ret = f_pcie_host_wr_conf(rc, bus, devfn, where, size, val);
		spin_unlock_irqrestore(&rc->conf_lock, flags);
	}
	return ret;
}

static struct pci_ops pcie_ops = {
	.read = f_pcie_rd_conf,
	.write = f_pcie_wr_conf,
};

struct pci_bus *f_pcie_host_scan_root_bus(struct device *parent,
			int bus, struct pci_ops *ops,
			void *sysdata, struct list_head *resources)
{
	struct pci_bus *b;
	int max, busnr = bus;

	b = pci_create_root_bus(parent, bus, ops, sysdata, resources);
	if (!b)
		return NULL;

	pci_bus_insert_busn_res(b, bus, 255);
	max = pci_scan_child_bus(b);
	if (max > (busnr + 1))
		pci_rescan_bus(b);

	pci_bus_update_busn_res_end(b, max);
	pci_bus_add_devices(b);

	return b;
}

static struct pci_bus *f_pcie_host_scan_bus(int nr,
			struct pci_sys_data *sys)
{
	struct pci_bus *bus;
	struct device *dev;
	u32 nr_port;

	nr_port = pcie_port.nr_port_map[nr];
	dev = pcie_port.dme_pcie[nr_port].rc->dev;
	if (nr_port >= pcie_port.pcie_por_num) {
		dev_err(dev, "%s %d BUG().\n", __func__, __LINE__);
		BUG();
	}

	bus = f_pcie_host_scan_root_bus(dev, sys->busnr, &pcie_ops,
			sys, &sys->resources);

	pcie_port.dme_pcie[nr_port].rc->root_pcibus = bus;
	return bus;
}

static int f_pcie_host_map_irq(const struct pci_dev *dev,
			u8 slot, u8 pin)
{
	struct pci_sys_data *sys = dev->bus->sysdata;
	struct dme_rc *rc;
	int irq;

	if (dev->msi_enabled)
		return dev->irq;

	rc = (struct dme_rc *)sys->private_data;

	if (pin < 1 || pin > 4) {
		dev_warn(rc->dev, "pin number pin=%d\n", pin);
		pin = 1;
	}

	irq = rc->msg_irq[pin - 1];
	return irq;
}

static struct hw_pci f_pcie_host_dme_pci_ops = {
	.setup		= f_pcie_host_setup,
	.scan		= f_pcie_host_scan_bus,
	.map_irq	= f_pcie_host_map_irq,
};

static irqreturn_t f_pcie_host_slave_mem_irq(int irqno, void *dev_id)
{
	struct dme_rc *rc = (struct dme_rc *)dev_id;
	u32 status, err_flg;

	status = readl(rc->rc_cfg_base + SLA_MEM_INT_ST);

	/* clear interrupt status */
	writel(status, rc->rc_cfg_base + SLA_MEM_INT_ST);

	err_flg = MEM_RD_COMPLE_ABORT | MEM_RD_UNSUPPORT_REQ |
		MEM_RD_COMPLE_TIMEOUT | MEM_RD_POISONED_TLP |
		MEM_RD_TRANS_REQ;

	if (status & err_flg)
		dev_dbg(rc->dev, "0x%x\n", status);

	return IRQ_HANDLED;
}

static irqreturn_t f_pcie_ep_intx_msi(int irq, void *dev_id)
{
	struct dme_ep *ep = dev_id;

	pr_info("%s irq (%d) PCIE_MSI_DATA (0x%x)\n",
		__func__, irq, readw(ep->regs + PCIE_MSI_DATA));

	return IRQ_HANDLED;
}

static irqreturn_t f_pcie_ep_interrupt(int irq, void *dev_id)
{
	struct dme_ep *ep = (struct dme_ep *)dev_id;
	u32 dem_s, dma_s, err_flg, act_dma;

	act_dma = ep->act_dma_number;

	dma_s = readl(ep->regs + DMA_STATUS);
	dma_s = ((dma_s >> (act_dma * 2)) & 0x03);

	dem_s = readl(ep->regs + DEM_DMA_TRAN_SAT_CH(act_dma));
	dem_s = dem_s & 0xFF;

	err_flg = DMA_TRANS_ABORT_BY_USER | DMA_TRANS_ABORT_RESET_DET |
			DMA_TRANS_ABORT_POISO_DET | DMA_TRANS_ABORT_BY_PCIE |
			AXI_ERROR_DETECT | DMA_ABORT_ENDERR;

	if (dem_s & err_flg)
		dev_dbg(ep->dev, "0x%x\n", dem_s);

	if (dma_s == DMA_NORMAL && dem_s == TRANS_COMPLETE) {
		ep->dma_done = SAT_DMA_DONE;
		wake_up_interruptible(&ep->dma_wait);
	}

	return IRQ_HANDLED;
}

int f_pcie_host_enable_msi(struct dme_rc *rc)
{
	int err = 0;
#ifdef CONFIG_IOMMU_API
	struct dma_iommu_mapping *mapping;
	unsigned long iova;
	int port = IOMMU_READ | IOMMU_WRITE | IOMMU_EXEC;
	phys_addr_t paddr;
#endif

	if (!rc->msi_lig)
		return -EINVAL;

	writel((u32)rc->msi_data, rc->rc_cfg_base + PCIE_MSI_ADDR_LO);
	writel(0, rc->rc_cfg_base + PCIE_MSI_ADDR_HI);

#ifdef CONFIG_IOMMU_API
	mapping = rc->dev->archdata.mapping;
	if (mapping) {
		paddr = (dma_addr_t)rc->msi_data & ~(SZ_4K - 1);
		iova = (unsigned long)rc->msi_data & ~(SZ_4K - 1);
		do {
			err = iommu_iova_to_phys(mapping->domain, iova);
			if (err <= 0) {
				/* no one mapping this address */
				break;
			} else {
				/* find next address */
				iova += SZ_1M;
				if (iova > SZ_1G) {
					dev_err(rc->dev,
						"can't mapping iova (0x%llx)of MSI data\n",
						(u64)iova);
					return -EINVAL;
				}
			}
		} while (err > 0);

		/* no one mapping this iova */
		err = iommu_map(mapping->domain, iova, paddr, SZ_4K, port);
		if (err < 0) {
			dev_err(rc->dev, "LIG address mapping fail\n");
			return err;
		}

		rc->msi_iova = iova;
		rc->msi_iova_sz = SZ_4K;
	}
#endif

	return err;
}

static void f_pcie_ep_dma_start(void __iomem *dma_reg, u32 dma_ch)
{
	u32 dma_set;

	dma_set = DMA_START | DMA_NUM(dma_ch);
	writeb(dma_set, dma_reg + DMAC_INSTR);
}

static void f_pcie_ep_dma_stop(void __iomem *dma_reg, u32 dma_ch)
{
	u32 dma_set;

	dma_set = DMA_STOP | DMA_INT_CLR | DMA_NUM(dma_ch);
	writel(dma_set, dma_reg + DMAC_INSTR);
}

static void f_pcie_ep_demand_dma_init(void __iomem *dma_reg,
		u16 dma_ch, u32 dma_read_write,
		dma_addr_t pcie_addr, dma_addr_t axi_addr, u32 dma_size)
{
	u32 dem_dma_set;

	/* DMA channel 0~15 as demand */
	writel(0, dma_reg + DMA_MODE_CONL);

	/* Set PCIe Address */
	writel(pcie_addr, dma_reg + DEM_DMA_PCIE_L_ADDR_CH(dma_ch));
	writel(0, dma_reg + DEM_DMA_PCIE_U_ADDR_CH(dma_ch));

	/* Set AXI Address */
	writel(axi_addr, dma_reg + DEM_DMA_AXI_ADDR_CH(dma_ch));

	/* Set TRANS Size */
	writel(dma_size, dma_reg + DEM_DMA_TRANS_SZ_CH(dma_ch));

	/* Demand DMA Transfer Setting */
	dem_dma_set = MULTI_R_REQ_1_MULTI | dma_read_write |
			DMA_TRAN_SPL_SZ(128);
	writel(dem_dma_set, dma_reg + DEM_DMA_TRANS_SET_CH(dma_ch));
}

void f_pcie_ep_init(struct dme_ep *ep)
{
	u32 dl_sat, addr, res_mask, msi_multi;

	/* device DL write enable */
	dl_sat = readl(ep->regs + BRIDGE_MODE) | DL_WRITE_MODE;
	writel(dl_sat, ep->regs + BRIDGE_MODE);

	/* init pcie device class code */
	writew(PCI_CLASS_MEMORY_RAM, ep->regs + PCI_CLASS_DEVICE);

	/*
	 * The numbers possessed by MSI Table are indicated
	 * S73 supply (1-16)
	 * 000h: 1 / 100: 16
	 */
	if (of_property_read_u32(ep->dev->of_node, "msi-multi",
		&msi_multi)) {
		dev_warn(ep->dev, "Missing msi-multi in dt\n");
		msi_multi = 0;
	}

	res_mask = readb(ep->regs + PCIE_MSI_CTL);
	res_mask &= ~0x0e;
	res_mask |= (msi_multi << 1);
	writeb(res_mask, ep->regs + PCIE_MSI_CTL);

	/*
	 * MSI-X table size
	 * 000h: 1
	 * 7FFh: 2048
	 */
	res_mask = (readw(ep->regs + PCIE_MSIX_CTL) & ~(0x7ff));
	writeb(res_mask, ep->regs + PCIE_MSIX_CTL);

	/*
	 * MSI-X
	 * setting  MSIX address table offset in 0x1000 of base 0
	 * setting  MSIX Pending table offset in 0x1008 of base 0
	 */
	writel(0x1000, ep->regs + PCIE_MSIX_TABLE);
	writel(0x1008, ep->regs + PCIE_MSIX_PBA);

	/* device DL write disenable */
	dl_sat = dl_sat & ~DL_WRITE_MODE;
	writel(dl_sat, ep->regs + BRIDGE_MODE);

	/* addr = 0xFFF80000; */ /* 512K DDR */
	/* addr = 0xFFFE0000; */ /* 128K DDR */
	/* addr = 0xFFFF0000; */ /* 64K DDR */
	addr = 0xfff80000;
	dl_sat = addr | BAR_SET_MEMORY_TYPE_32BIT | BAR_SET_MEM_SPACE;

	/* BAR 0 enable  */
	writel(dl_sat, ep->regs + BAR_RES_SET(0));

	/* Only enabled BAR0 */
	writel((EROM_EN | BAR_EN(0)), ep->regs + BAR_EN_REG);

	/* AXI Master Error Interrupt Mask Register */
	writel(0, ep->regs + AXI_MASTER_ERR_INT_MASK);
	writel(0, ep->regs + DMA_INT_MASK);

	/* AXI Slave Memory Interrupt Mask */
	writel(0xff, ep->regs + SLA_MEM_INT_MASK);
}

static void f_pcie_ep_init_pcie_device(struct dme_ep *ep)
{
	/* HSIO DDR RAM BAR0 remap */
	writel(ep->remap_addr_phys, ep->regs + AXI_MASTER_REMAP_ADDR(0));
	writel(0x0, ep->regs + AXI_MASTER_REMAP_ADDR(1));
	writel(0x0, ep->regs + AXI_MASTER_REMAP_ADDR(2));
	writel(0x0, ep->regs + AXI_MASTER_REMAP_ADDR(3));
	writel(0x0, ep->regs + AXI_MASTER_REMAP_ADDR(4));
	writel(0x0, ep->regs + AXI_MASTER_REMAP_ADDR(5));
}

void f_desc_table_release(struct device *dev, struct desc_tab *tab,
			u32 size)
{
	dma_free_coherent(dev, size, tab->pcie_buf, tab->pcie_lower_addr);
	dma_free_coherent(NULL, size, tab->axi_buf, tab->axi_addr);
	dma_free_coherent(NULL, size, tab->dma_tab, tab->dma_table_phys);
}

int f_desc_tab_init(struct dme_ep *ep, struct desc_tab *tab,
			struct desc_tab *next_tab, u32 size)
{
	dma_addr_t buf_phys, buf2_phys, table_phys;
	u32 *buf = NULL, *buf2 = NULL, *table = NULL;
	u32 traf_set;

	buf = dma_alloc_coherent(NULL, size, &buf_phys, GFP_KERNEL);
	if (!buf) {
		dev_err(ep->dev, "dma_alloc_coherent fail\n");
		return -ENOMEM;
	}

	buf2 = dma_alloc_coherent(ep->dev, size, &buf2_phys, GFP_KERNEL);
	if (!buf2) {
		dev_err(ep->dev, "dma_alloc_coherent fail\n");
		return -ENOMEM;
	}

	table = dma_alloc_coherent(NULL, size, &table_phys, GFP_KERNEL);
	if (!table) {
		dev_err(ep->dev, "dma_alloc_coherent fail\n");
		return -ENOMEM;
	}

	tab->pcie_lower_addr = buf2_phys;
	tab->pcie_upper_addr = 0;
	tab->pcie_buf = buf2;
	tab->axi_addr = buf_phys;
	tab->axi_buf = buf;
	tab->traf_size = size;

	traf_set = MULTI_R_REQ_1_MULTI | DMA_TRAN_SPL_SZ(128);

	tab->traf_set = traf_set;
	tab->dma_tab = table;
	tab->dma_table_phys = table_phys;
	tab->next_tab_addr = 0;
	tab->descr_cont = 0;

	/* Set PCIe Address */
	writel(tab->pcie_lower_addr, tab->dma_tab + DES_DMA_TB_L_ADDR);
	writel(tab->pcie_upper_addr, tab->dma_tab + DES_DMA_TB_U_ADDR);

	/* Set AXI Address */
	writel(tab->axi_addr, tab->dma_tab + DES_DMA_TB_AXI_ADDR);

	/* Set TRANS Size */
	writel(tab->traf_size, tab->dma_tab + DES_DMA_TB_TRANS_SZ);

	/* DMA Transfer Setting */
	traf_set = 0;
	writel(tab->traf_set, tab->dma_tab + DES_DMA_TB_TRANS_SET);

	/* Descriptor DMA Transfer Table Address */
	if (next_tab)
		writel(next_tab->dma_tab, tab->dma_tab + DES_DMA_TB_ADDR_SET);

	if (next_tab)
		traf_set = 0 << DES_DMA_LAST_DESC;
	else
		traf_set = 1 << DES_DMA_LAST_DESC;

	/* Last Descriptor & Descriptor ID */
	traf_set |= tab->descr_cont << DES_DMA_DESC_ID;
	writel(traf_set, tab->dma_tab + DES_DMA_TB_LDID_SET);

	return 0;
}

static int f_pcie_ep_descr_dma_test(
struct dme_ep *ep, u32 size)
{
	struct desc_tab descr_tab, descr_tab_next;
	u32 i, j, k, err, dma_ch = 0, test_err = 0;
	u32 axi_buf , pcie_buf, in_out_f;
	u32 *buf1, *buf2, dma_type_rw = 2;

	for (i = 0; i < dma_type_rw; i++) {
		for (k = 0; k < TEST_NUM; k++) {
			if (i == 0) {
				axi_buf = 0;
				in_out_f = DMA_TRANS_PCIE2AXI;
				pcie_buf = test_patten[k];
			} else {
				pcie_buf = 0;
				in_out_f = DMA_TRANS_AXI2PCIE;
				axi_buf = test_patten[k];
			}

			f_desc_tab_init(ep, &descr_tab_next, NULL, size);
			f_desc_tab_init(ep, &descr_tab, &descr_tab_next, size);

			/* Set DMA descript mode */
			writel(0xff, ep->regs + DMA_MODE_CONL);

			/* Set first DMA descript table */
			writel(descr_tab.dma_tab, ep->regs + TAB_ADDR(dma_ch));

			/* Init AXI buffer */
			memset(descr_tab.axi_buf, (u8)axi_buf, size);
			memset(descr_tab_next.axi_buf, (u8)axi_buf, size);

			/* Init PCIe buffer */
			memset(descr_tab.pcie_buf, (u8)pcie_buf, size);
			memset(descr_tab_next.pcie_buf, (u8)pcie_buf, size);

			/* init wait flag */
			ep->dma_done = SAT_DMA_WAIT;

			/* DMA ch start */
			f_pcie_ep_dma_start(ep->regs, dma_ch);

			/* Wait DMA Interrupt */
			err = wait_event_interruptible_timeout(ep->dma_wait,
					(ep->dma_done == SAT_DMA_DONE),
					ep->dma_wait_timeout);

			if (!err)
				dev_err(ep->dev,
					"Desc DAM ch %d timeout\n", dma_ch);

			/* DMA ch stop */
			f_pcie_ep_dma_stop(ep->regs, dma_ch);

			buf1 = descr_tab.pcie_buf;
			buf2 = descr_tab.axi_buf;
			for (j = 0; j < (size / 4); j++)
				if (buf1[j] != buf2[j])
					test_err++;

			buf1 = descr_tab_next.pcie_buf;
			buf2 = descr_tab_next.axi_buf;
			for (j = 0; j < (size / 4); j++)
				if (buf1[j] != buf2[j])
					test_err++;

			f_desc_table_release(ep->dev, &descr_tab, size);
			f_desc_table_release(ep->dev, &descr_tab_next, size);
		}
	}

	if (test_err > 0)
		return  -EINVAL;

	return 0;
}

static int
f_pcie_ep_demand_dma_test(struct dme_ep *ep, u32 size)
{
	dma_addr_t buf_phys, buf2_phys;
	u32 axi_buf , pcie_buf, in_out_f, dma_type_rw, type_data;
	u32 *buf = NULL, *buf2 = NULL, j, err, dma_ch, test_err = 0;

	buf = dma_alloc_coherent(NULL, size, &buf_phys, GFP_KERNEL);
	if (!buf) {
		dev_err(ep->dev, "dma_alloc fail\n");
		return -ENOMEM;
	}

	buf2 = dma_alloc_coherent(ep->dev, size, &buf2_phys, GFP_KERNEL);
	if (!buf2) {
		dev_err(ep->dev, "dma_alloc fail\n");
		return -ENOMEM;
	}

	for (dma_ch = 0; dma_ch < DMXH_DMA_CH; dma_ch++) {
		ep->act_dma_number = dma_ch;

		for (dma_type_rw = 0; dma_type_rw < 2; dma_type_rw++) {
			ep->dma_type = DMA_TYPE_DEMAND;

			if (dma_type_rw == 0)
				in_out_f = DMA_TRANS_PCIE2AXI;
			else
				in_out_f = DMA_TRANS_AXI2PCIE;

			f_pcie_ep_demand_dma_init(ep->regs, dma_ch, in_out_f,
						buf2_phys, buf_phys, size);

			for (type_data = 0; type_data < TEST_NUM; type_data++) {
				if (in_out_f == DMA_TRANS_PCIE2AXI) {
					pcie_buf = test_patten[type_data];
					axi_buf = 0;
				} else {
					pcie_buf = 0;
					axi_buf = test_patten[type_data];
				}

				/* init buf */
				memset(buf2, pcie_buf, size);
				memset(buf, axi_buf, size);
				ep->dma_done = SAT_DMA_WAIT;

				/* DMA ch start */
				f_pcie_ep_dma_start(ep->regs, dma_ch);

				err = wait_event_interruptible_timeout(ep->dma_wait,
						(ep->dma_done == SAT_DMA_DONE),
						ep->dma_wait_timeout);
				if (!err)
					dev_err(ep->dev,
						"DAM ch %d timeout\n", dma_ch);

				/* DMA ch stop */
				f_pcie_ep_dma_stop(ep->regs, dma_ch);

				/* data compare */
				for (j = 0; j < size / 4; j++) {
					pcie_buf = buf2[j];
					axi_buf = test_patten[type_data];

					if (axi_buf != pcie_buf)
						test_err++;
				}
			}
		}
	}

	/* DMA memory free */
	 dma_free_coherent(NULL, size, buf, buf_phys);
	 dma_free_coherent(ep->dev, size, buf2, buf2_phys);

	if (test_err > 0)
		return  -EINVAL;

	return 0;
}

static int f_pcie_ep_mem_test(struct dme_ep *pdme_ep, u32 size)
{
	u32 i, j, test_w, test_r, err_cont = 0;

	/* Memory read/write test */
	for (j = 0; j < 4; j++) {
		for (i = 0; i < size; i = i + 4) {
			switch (j) {
			case 0:
				test_w = 0x00005555;
			break;
			case 1:
				test_w = 0x55550000;
			break;
			case 2:
				test_w = 0x0000FFFF;
			break;
			case 3:
				test_w = 0xFFFF0000;
			break;
			default:
			break;
			}

			writel(test_w, pdme_ep->pcie_addr + i);
			schedule_timeout(WAIT_MEM_TIMEOUT);
			test_r = readl(pdme_ep->pcie_addr + i);

			if (test_w != test_r)
				err_cont++;
		}

		if (err_cont > 0)
			return  -EINVAL;

		err_cont = 0;
	}
	return 0;
}

static int f_pcie_ep_init_one(struct pci_dev *pdev,
			const struct pci_device_id *ent)
{
	struct dme_ep *ep;
	struct f_pcie_port *dme_port;
	void __iomem *msix_tb_io;
	int test_err = 0, err, i;
	resource_size_t phys_addr;
	u32 size, msi_cont, table_offset, bir;

	dme_port = f_get_pcie_ep_port(ent->device);
	if (!dme_port) {
		dev_err(&pdev->dev, "%s EP dme_port is null\n", __func__);
		return -EINVAL;
	}
	ep = dme_port->ep;
	ep->dev = &pdev->dev;

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "Cannot enable PCI device\n");
		goto err_out_disable_pdev;
	}

	err = pci_request_regions(pdev, "ep");
	if (err) {
		dev_err(&pdev->dev, "Cannot obtain PCI resources\n");
		goto err_out_iounmap;
	}

	init_waitqueue_head(&ep->dma_wait);
	ep->dma_wait_timeout = WAIT_DMA_TIMEOUT;
	ep->dma_done = SAT_DMA_WAIT;
	size = pci_resource_len(pdev, 0);

	ep->remap_addr = kmalloc(size, GFP_KERNEL);
	if (!ep->remap_addr) {
		dev_err(ep->dev, "EP remap_addr fail\n");
		goto err_out_free_res;
	}

	ep->remap_addr_phys = __pa(ep->remap_addr);
	ep->remap_addr_len = size;
	f_pcie_ep_init_pcie_device(ep);

	pci_set_master(pdev);

	ep->pcie_addr = pci_ioremap_bar(pdev, 0);
	if (!ep->pcie_addr) {
		dev_err(&pdev->dev, "Cannot map device registers\n");
		err = -ENOMEM;
		goto err_out_free_res;
	}

	/* Memory test */
	err = f_pcie_ep_mem_test(ep, pci_resource_len(pdev, 0));
	if (err < 0)
		test_err++;

	if (ep->int_type == INT_MSI) {
		/* MSI */
		err = pci_enable_msi(pdev);
		if (err)
			dev_warn(ep->dev, "failed to allocate MSI entry\n");

	} else if (ep->int_type == INT_MSIX) {
		/* MSI-X */
		msi_cont = readw(ep->regs + PCIE_MSIX_CTL);
		msi_cont = (msi_cont & PCI_MSIX_FLAGS_QSIZE) + 1;
		ep->msix_count = msi_cont;

		ep->msix_entries =
			kmalloc((sizeof(struct msix_entry)) * ep->msix_count,
					GFP_KERNEL);
		if (!ep->msix_entries) {
			dev_err(ep->dev, "Failed to allocate MSI-X entries\n");
			return -ENOMEM;
		}

		for (i = 0; i < ep->msix_count; i++) {
			ep->msix_entries[i].entry = i;
			ep->msix_entries[i].vector = 0;
		}

		err = pci_enable_msix(pdev, ep->msix_entries, ep->msix_count);
		if (err) {
			dev_err(ep->dev, "Failed to enable MSI-X\n");
			return -EINVAL;
		}

		pci_read_config_dword(pdev, pdev->msix_cap + PCI_MSIX_TABLE,
				&table_offset);

		bir = table_offset & PCI_MSIX_TABLE_BIR;
		table_offset &= PCI_MSIX_TABLE_OFFSET;
		phys_addr = pci_resource_start(pdev, bir) + table_offset;

		msix_tb_io = ioremap(phys_addr,
			ep->msix_count * PCI_MSIX_ENTRY_SIZE);
		if (!msix_tb_io) {
			dev_err(ep->dev, "ioremap msix_tb_io fail");
			return -EINVAL;
		}

		/* read content of MSI-X table */
		writel(readl(msix_tb_io + PCI_MSIX_ENTRY_LOWER_ADDR),
			ep->regs + MSIX_ADDR_L(0));
		writel(readl(msix_tb_io + PCI_MSIX_ENTRY_UPPER_ADDR),
			ep->regs + MSIX_ADDR_H(0));
		writel(readl(msix_tb_io + PCI_MSIX_ENTRY_DATA),
			ep->regs + MSIX_DATA(0));

		iounmap(msix_tb_io);

		for (i = 0; i < ep->msix_count; i++) {
			err = request_irq(ep->msix_entries[i].vector,
					f_pcie_ep_intx_msi, 0, "EP MSI-X", ep);
			if (err) {
				dev_err(ep->dev, "MSI-X (%d) request fail\n",
					ep->msix_entries[i].vector);
				pci_disable_msix(pdev);
				return err;
			}
		}
	}

	if (ep->int_type == INT_MSI || ep->int_type == INT_INTX) {
		err = request_irq(pdev->irq, f_pcie_ep_intx_msi, IRQF_SHARED,
				"EP INT", ep);
		if (err) {
			dev_err(ep->dev, "request EP INT %d failed\n", pdev->irq);
			return err;
		}
	}

	/* DMA TEST */
	err = f_pcie_ep_demand_dma_test(ep, pci_resource_len(pdev, 0));
	if (err < 0)
		test_err++;

	/* Descript DAM chain test */
	err = f_pcie_ep_descr_dma_test(ep, pci_resource_len(pdev, 0));
	if (err < 0)
		test_err++;

	if (test_err > 0)
		dev_info(ep->dev, "pcie EP TEST Err !\n");
	else
		dev_info(ep->dev, "pcie EP TEST PASS !\n");

	ep->dev->platform_data = ep;
	pci_set_drvdata(pdev, ep);

	return err;

err_out_iounmap:
	if (ep->pcie_addr)
		iounmap(ep->pcie_addr);

	if (ep->regs)
		iounmap(ep->regs);

	ep->pcie_addr = NULL;
	ep->regs = NULL;

err_out_free_res:
	pci_release_regions(pdev);

err_out_disable_pdev:
	if (pci_is_enabled(pdev))
		pci_disable_device(pdev);

	pci_set_drvdata(pdev, NULL);

	return err;
}

static void f_pcie_ep_remove_one(struct pci_dev *pdev)
{
	struct dme_ep *pdme_ep;
	int i;

	pdme_ep = pci_get_drvdata(pdev);

	if (pdme_ep->pcie_addr) {
		iounmap(pdme_ep->pcie_addr);
		pdme_ep->pcie_addr = NULL;
	}

	if (pdme_ep->int_type == INT_MSIX) {
		for (i = 0; i < pdme_ep->msix_count; i++) {
			disable_irq(pdme_ep->msix_entries[i].vector);
			free_irq(pdme_ep->msix_entries[i].vector, pdme_ep);
		}
		pci_disable_msix(pdev);
		kfree(pdme_ep->msix_entries);
	} else {
		disable_irq(pdev->irq);
		free_irq(pdev->irq, pdme_ep);
		if (pdme_ep->int_type == INT_MSI)
			pci_disable_msi(pdev);
	}

	kfree(pdme_ep->remap_addr);
	pci_set_drvdata(pdev, NULL);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

#ifdef CONFIG_PM
static int f_pcie_ep_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct dme_ep *pdme_ep;
	int i;

	pdme_ep = dev_get_platdata(dev);

	if (pdme_ep->int_type == INT_MSIX) {
		for (i = 0; i < pdme_ep->msix_count; i++)
			disable_irq(pdme_ep->msix_entries[i].vector);
	} else {
		disable_irq(pdev->irq);
	}

	return 0;
}

static int f_pcie_ep_resume(struct device *dev)
{
	struct dme_ep *pdme_ep;
	struct pci_dev *pdev = to_pci_dev(dev);
	int err, i;

	pdme_ep = dev_get_platdata(dev);
	f_pcie_ep_init_pcie_device(pdme_ep);

	/* Memory test */
	err = f_pcie_ep_mem_test(pdme_ep, pdme_ep->remap_addr_len);

	if (err < 0)
		dev_info(pdme_ep->dev, "pcie EP TEST Err\n");
	else
		dev_info(pdme_ep->dev, "pcie EP TEST PASS\n");

	pdme_ep = dev_get_drvdata(dev);

	if (pdme_ep->int_type == INT_MSIX) {
		for (i = 0; i < pdme_ep->msix_count; i++)
			enable_irq(pdme_ep->msix_entries[i].vector);
	} else {
		enable_irq(pdev->irq);
	}

	return 0;
}

static int f_pcie_ep_runtime_resume(struct device *dev)
{
	return f_pcie_ep_resume(dev);
}

static int f_pcie_ep_runtime_suspend(struct device *dev)
{
	return f_pcie_ep_suspend(dev);
}

static const struct dev_pm_ops f_pcie_ep_pm_ops = {
	.suspend = f_pcie_ep_suspend,
	.resume = f_pcie_ep_resume,
	.runtime_suspend = f_pcie_ep_runtime_suspend,
	.runtime_resume = f_pcie_ep_runtime_resume,
};

#define F_PCIE_EP_PM_OPS (&f_pcie_ep_pm_ops)

#else
#define F_PCIE_EP_PM_OPS NULL
#endif

DEFINE_PCI_DEVICE_TABLE(f_pcie_ep_pci_tbl) = {
	{PCI_DEVICE(0x10cf, 0x2042)},
	{PCI_DEVICE(0x10cf, 0x2046)},
	{PCI_DEVICE(0x10cf, 0x204f)},
	{PCI_DEVICE(0x10cf, 0x204c)},
	{PCI_DEVICE(0x10cf, 0x2051)},
	{}
};
MODULE_DEVICE_TABLE(pci, f_pcie_ep_pci_tbl);

static struct pci_driver f_pcie_ep_driver = {
	.id_table	= f_pcie_ep_pci_tbl,
	.probe		= f_pcie_ep_init_one,
	.remove		= f_pcie_ep_remove_one,
	.driver.pm	= F_PCIE_EP_PM_OPS,
};

#ifdef CONFIG_PM
static int f_pcie_pm_suspend(struct device *dev)
{
	struct f_pcie_port *dme_port;
	struct dme_rc *rc;
	struct dme_ep *ep;
	u32 i = 0, clk_id, dma_irq_cnt;

	dme_port = dev_get_platdata(dev);
	if (!dme_port)
		return 0;

	if (!dme_port->rc && !dme_port->ep) {
		dev_err(dev, "rc and ep are null\n");
		return -EINVAL;
	}

	/* ep mode */
	if (dme_port->ep && dme_port->ep->inited) {
		ep = dme_port->ep;
		dma_irq_cnt = sizeof(ep->dma_irq)/sizeof(int);
		while (i < dma_irq_cnt) {
			if (ep->dma_irq[i] > 0)
				disable_irq(ep->dma_irq[i]);
			i++;
		};
		for (clk_id = 0; clk_id < ep->clk_num; clk_id++)
			if (ep->clk[clk_id])
				clk_disable_unprepare(ep->clk[clk_id]);

		ep->inited = 0;
		return 0;
	}

	/* rc mode */
	if (dme_port->rc) {
		rc = dme_port->rc;
		for (clk_id = 0; clk_id < rc->clk_num; clk_id++)
			clk_disable_unprepare(rc->clk[clk_id]);

		disable_irq(rc->slav_mem_irq);
		if (rc->lig_irq)
			disable_irq(rc->lig_irq);
		rc->inited = 0;
	}

	return 0;
}

static int f_pcie_pm_resume(struct device *dev)
{
	struct f_pcie_port *dme_port;
	struct dme_rc *rc;
	struct dme_ep *ep = NULL;
	u32 i = 0, clk_id, dma_irq_cnt;

	dme_port = dev_get_platdata(dev);
	if (!dme_port)
		return 0;

	if (!dme_port->rc && !dme_port->ep) {
		dev_err(dev, "rc and ep are null\n");
		return -EINVAL;
	}

	/* ep mode */
	if (dme_port->ep && dme_port->ep->inited)
		return 0;

	if (dme_port->ep && !dme_port->ep->inited) {
		ep = dme_port->ep;

		if (dme_port->wrapper) {
			writew(EN_DMA_INT, dme_port->wrapper + ST_DMA_INT_ENB);
			usleep_range(10, 20);
		}

		for (clk_id = 0; clk_id < ep->clk_num; clk_id++)
			if (ep->clk[clk_id])
				clk_prepare_enable(ep->clk[clk_id]);

		dma_irq_cnt = sizeof(ep->dma_irq)/sizeof(int);

		while (i < dma_irq_cnt) {
			if (ep->dma_irq[i] > 0)
				enable_irq(ep->dma_irq[i]);
			i++;
		};

		/* init pcie device */
		if (ep->loop_coned > 0)
			f_pcie_ep_init(dme_port->ep);

		ep->inited = 1;

		return 0;
	}

	/* rc mode */
	if (dme_port->rc) {
		rc = dme_port->rc;

		if (dme_port->wrapper) {
			writew(EN_DMA_INT, dme_port->wrapper + ST_DMA_INT_ENB);
			usleep_range(10, 20);
		}

		if (!rc->inited) {
			for (clk_id = 0; clk_id < rc->clk_num; clk_id++)
				if (rc->clk[clk_id])
					clk_prepare_enable(rc->clk[clk_id]);

			f_pcie_host_clear_reset(rc);
			f_pcie_host_init_rc(rc);

#ifdef CONFIG_PCI_MSI
			if (!f_pcie_host_enable_msi(rc))
				writel(SET_LIG_ENABLE, rc->msi_lig + LIG_INT_EN);
#endif
			enable_irq(rc->slav_mem_irq);
			if (rc->lig_irq)
				enable_irq(rc->lig_irq);
		}
	}

	return 0;
}

int f_pcie_pm_start(struct device *dev)
{
	return f_pcie_pm_resume(dev);
}

int f_pcie_pm_stop(struct device *dev)
{
	return f_pcie_pm_suspend(dev);
}

static struct gpd_dev_ops gpd_dev_ops = {
	.start = f_pcie_pm_start,
	.stop = f_pcie_pm_stop,
};

static int f_pcie_runtime_suspend(struct device *dev)
{
	return f_pcie_pm_suspend(dev);
}

static int f_pcie_runtime_resume(struct device *dev)
{
	return f_pcie_pm_resume(dev);
}
#endif

static irqreturn_t msi_map_intr(struct dme_rc *rc)
{
	void __iomem *int_clr, *int_sts, *int_req;
	u32 i, virq, irq_id;

	int_req = rc->msi_lig + LIG_INT_REQ;
	int_clr = rc->msi_lig + LIG_INT_CLR;
	int_sts = rc->msi_lig + LIG_INT_STS;

	/* S7x support 16 INT for MSI */
	irq_id = readl(int_sts);
	for (i = 0; i < MSI_HW_INT_N; i++)
		if (irq_id & 1 << i)
			break;

	irq_id = i;
	writel((readl(int_sts)), int_clr);
	virq = irq_find_mapping(rc->irq_domain, irq_id);
	generic_handle_irq(virq);
	return IRQ_HANDLED;
}

static irqreturn_t f_pcie_lig_int(int irq, void *dev_id)
{
	struct dme_rc *rc = dev_id;
	irqreturn_t r;

	r = msi_map_intr(rc);
	return r;
}

#ifdef CONFIG_PCI_MSI
struct irq_chip f_dme_msi_irq_chip = {
	.name = "PCI-MSI",
	.irq_enable = unmask_msi_irq,
	.irq_disable = mask_msi_irq,
	.irq_mask = mask_msi_irq,
	.irq_unmask = unmask_msi_irq,
};

static int f_pcie_msi_map(struct irq_domain *domain, unsigned int irq,
			irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &f_dme_msi_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);
	set_irq_flags(irq, IRQF_VALID);
	return 0;
}

static const struct irq_domain_ops msi_domain_ops = {
	.map = f_pcie_msi_map,
};
#endif

static int f_pcie_host_probe(struct platform_device *pdev,
			struct device_node *host_dev_n, u32 id)
{
	struct resource res, *res_tmp;
	struct f_pcie_port *dme_port;
	struct dme_rc *rc;
	struct device_node *h_dev_n = host_dev_n;
	const char *clk_nam[20] = {0};
	int err;
	u32 clk_num , clk_id = 0;

	dme_port = &pcie_port.dme_pcie[id];
	if (!dme_port->rc) {
		dev_err(&pdev->dev, "%s dme_port->rc is null\n", __func__);
		return -EINVAL;
	}

	rc = dme_port->rc;
	rc->dev = &pdev->dev;
	rc->index = id;
	platform_set_drvdata(pdev, rc);
	spin_lock_init(&rc->conf_lock);
	spin_lock_init(&rc->msi_lock);
	memset(rc->virq, 0, sizeof(u32) * S7X_MSIX_VECT);

	if (of_address_to_resource(h_dev_n, 0, &res)) {
		dev_err(&pdev->dev, "no regs resource defined\n");
		return -ENXIO;
	}

	rc->rc_cfg_base = ioremap(res.start, resource_size(&res));
	for (id = MSG_INT_A_D(0); id <= MSG_INT_A_D(3); id++) {
		rc->msg_irq[id] = of_irq_to_resource(h_dev_n, id, NULL);
		if (rc->msg_irq[id] < 0) {
			dev_err(&pdev->dev, "can not find irq_%d\n", id);
			err = -ENODEV;
			goto err_unmap_regs;
		}
	}

	if (!of_address_to_resource(rc->dev->of_node, 2, &res)) {
		rc->msi_lig = ioremap(res.start, resource_size(&res));
		rc->msi_data = res.start;
	} else {
		dev_info(rc->dev, "MSI PCIE_LIG not supply\n");
	}

	rc->slav_mem_irq = of_irq_to_resource(h_dev_n, SLA_MEM_INT, NULL);
	if (rc->slav_mem_irq < 0) {
		dev_err(&pdev->dev, "no slav_mem_irq ID\n");
		err = -ENODEV;
		goto err_unmap_regs;
	}

	err = request_irq(rc->slav_mem_irq, f_pcie_host_slave_mem_irq,
				IRQF_TRIGGER_RISING, "PCIe slave_memory", rc);
	if (err != 0) {
		dev_err(&pdev->dev, "slav_mem_irq:request_irq fail\n");
		err = -ENODEV;
		goto err_unmap_regs;
	}

	if (rc->msi_lig) {
		rc->lig_irq = of_irq_to_resource(rc->dev->of_node,
				PCIE_WAP_LIG_INT, NULL);
		if (rc->lig_irq > 0) {
			err = request_irq(rc->lig_irq, f_pcie_lig_int,
				IRQF_TRIGGER_RISING, "PCIe LIG", rc);
			if (err)
				return -EINVAL;
		}
	}

	if (of_property_read_u32(h_dev_n, "clock_num", &clk_num)) {
		dev_err(&pdev->dev, "Missing \"clk_num\" in dt\n");
		return -EINVAL;
	}

	rc->clk = kmalloc(clk_num * sizeof(struct clk *), GFP_KERNEL);
	memset(rc->clk, 0, clk_num * sizeof(struct clk *));
	rc->clk_num = clk_num;

	while (1) {
		of_property_read_string_index(h_dev_n,
			"clock-names", clk_id, &clk_nam[clk_id]);

		if (clk_nam[clk_id] == NULL)
			break;

		rc->clk[clk_id] = of_clk_get(h_dev_n, clk_id);
		if (IS_ERR(rc->clk[clk_id])) {
			dev_err(&pdev->dev, "clock%d not found.\n", clk_id);
			err = PTR_ERR(rc->clk[clk_id]);
			goto err_unmap_regs;
		}

		err = clk_prepare_enable(rc->clk[clk_id]);
		if (err)
			goto err_fail;

		clk_id++;
	}

	rc->root_bus_nr = -1;
	memset(rc->mem_res, 0, sizeof(rc->mem_res));
	memset(rc->io_res, 0, sizeof(rc->io_res));

	f_pci_process_bridge_of_ranges(rc, h_dev_n, 1);
	res_tmp = &rc->cfg_res;
	rc->ep_cfg_base = ioremap(res_tmp->start, resource_size(res_tmp));

	/* PCIe will not link up until clears the PERSET# bit */
	f_pcie_host_clear_reset(rc);
	f_pcie_host_init_rc(rc);
	f_pcie_dev_set_platdata(&pdev->dev, dme_port);
	rc->nvect = 0;

#ifdef CONFIG_PCI_MSI
	err = f_pcie_host_enable_msi(rc);
	rc->irq_domain = irq_domain_add_linear(pdev->dev.of_node,
		S7X_MSIX_VECT, &msi_domain_ops, NULL);
	if (!rc->irq_domain) {
		dev_err(rc->dev, "Failed to get a MSI IRQ domain\n");
		return PTR_ERR(rc->irq_domain);
	}
	/* enable local interrupt for MSI */
	if (!err)
		writel(SET_LIG_ENABLE, rc->msi_lig + LIG_INT_EN);
#endif
	return 0;

err_fail:
	for (clk_id = 0; clk_id < clk_num; clk_id++)
		clk_put(rc->clk[clk_id]);

err_unmap_regs:
	iounmap(rc->rc_cfg_base);
	iounmap(rc->ep_cfg_base);
	kfree(rc->clk);
	return err;
}

static int f_pcie_ep_probe(struct platform_device *pdev,
			struct device_node *ep_dev_n, u32 index)
{
	struct resource res;
	struct f_pcie_port *dme_port;
	struct dme_ep *pdme_ep;
	const char *clk_nam[20] = {0};
	u32 i = 0, j = 0, err = 0, clk_num = 0, clk_id;
	int irq = 0, int_type = 0, dma_int_off;

	dme_port = &pcie_port.dme_pcie[index];
	if (!dme_port->ep) {
		dev_err(&pdev->dev, "dme_port->ep is null\n");
		return -EINVAL;
	}

	pdme_ep = dme_port->ep;
	pdme_ep->dev = &pdev->dev;
	pdme_ep->index = index;

	if (of_property_read_u32(ep_dev_n, "int_type", &int_type))
		int_type = 0;

	pdme_ep->int_type = int_type;

	/* vendor ID */
	if (of_property_read_u32_index(ep_dev_n, "ven_dev_id", 0,
				&pdme_ep->vendor_id)) {
		dev_err(&pdev->dev, "Missing vendor_id in dt\n");
		return -EINVAL;
	}

	/* device ID */
	if (of_property_read_u32_index(ep_dev_n, "ven_dev_id", 1,
				&pdme_ep->device_id)) {
		dev_err(&pdev->dev, "Missing device_id in dt\n");
		return -EINVAL;
	}

	/* clock */
	if (of_property_read_u32(ep_dev_n, "clock_num", &clk_num)) {
		dev_err(&pdev->dev, "Missing clk_num in dt\n");
		return -EINVAL;
	}

	pdme_ep->clk_num = clk_num;
	pdme_ep->clk = devm_kzalloc(&pdev->dev,
			clk_num * sizeof(struct clk *), GFP_KERNEL);
	if (pdme_ep->clk == NULL) {
		dev_err(&pdev->dev, "pdme_ep->clk alloc fail!\n");
		return -EINVAL;
	}

	clk_id = 0;
	while (1) {
		of_property_read_string_index(ep_dev_n,
			"clock-names", clk_id, &clk_nam[clk_id]);

		if (clk_nam[clk_id]) {
			pdme_ep->clk[clk_id] = of_clk_get(ep_dev_n, clk_id);

			if (IS_ERR(pdme_ep->clk[clk_id])) {
				dev_err(&pdev->dev,
					"clock%d not found.\n", clk_id);
				err = PTR_ERR(pdme_ep->clk[clk_id]);
				goto err_unmap_regs;
			}

			err = clk_prepare_enable(pdme_ep->clk[clk_id]);
			if (err)
				goto err_fail;

		} else {
			break;
		}

		clk_id++;
	}

	if (of_address_to_resource(ep_dev_n, 0, &res)) {
		dev_err(&pdev->dev, "no regs resource defined\n");
		return -ENXIO;
	}

	if (of_property_read_u32_index(ep_dev_n, "dma_int_off", 0,
				&dma_int_off)) {
		dev_err(&pdev->dev, "Missing dma_int_off in dt\n");
		return -EINVAL;
	}

	pdme_ep->regs = ioremap(res.start, resource_size(&res));
	i = dma_int_off;
	while (j < DMA_INT_NUM) {
		irq = of_irq_to_resource(ep_dev_n, i, NULL);
		if (irq) {
			pdme_ep->dma_irq[j] = irq;
			err = request_irq(pdme_ep->dma_irq[j],
					f_pcie_ep_interrupt,
					IRQF_TRIGGER_RISING,
					"PCIe Endpoint DMA IRQ",
					(struct dme_ep *)pdme_ep);
			if (err) {
				dev_err(&pdev->dev, "request_irq err\n");
				goto err_unmap_regs;
			}
		} else {
			break;
		}
		i++;
		j++;
	}

	/* init pcie device */
	if (pdme_ep->loop_coned)
		f_pcie_ep_init(dme_port->ep);

	pdme_ep->inited = 1;
	f_pcie_dev_set_platdata(&pdev->dev, dme_port);

	return 0;

err_unmap_regs:
	if (pdme_ep->regs) {
		iounmap(pdme_ep->regs);
		pdme_ep->regs = NULL;
	}

err_fail:
	if (pdme_ep->clk)
		devm_kfree(&pdev->dev, pdme_ep->clk);

	return err;
}

int pcie_init(void)
{
	struct f_pcie_port *dme_port;
	int err = 0, port_num = 0;
	char *devname = NULL;

	f_pcie_host_dme_pci_ops.nr_controllers = pcie_port.rc_cnt;
	pci_common_init(&f_pcie_host_dme_pci_ops);

	while (port_num < pcie_port.pcie_por_num) {
		dme_port = f_get_pcie_port(port_num);
		if (!dme_port) {
			port_num++;
			continue;
		}

		if (dme_port->ep && dme_port->ep->loop_coned) {

			devname = kmalloc((sizeof(char) * EP_NAME_LEN),
				GFP_KERNEL);
			if (!devname) {
				pr_err("%s devname is NULL\n", __func__);
				return -ENOMEM;;
			}

			snprintf(devname, EP_NAME_LEN, "%s-ep",
				dev_name(dme_port->ep->dev));
			f_pcie_ep_driver.name = devname;
			err = pci_register_driver(&f_pcie_ep_driver);
			if (err < 0) {
				dev_err(dme_port->ep->dev, "pci_register_driver err\n");
				kfree(f_pcie_ep_driver.name);
			}
		} else {
			if (dme_port->ep)
				dev_info(dme_port->ep->dev, "no loop cable!\n");
		}
		port_num++;
	};

	return 0;
}

void pcie_uninit(void)
{
	struct f_pcie_port *dme_port;
	int port_num = 0;

	while (port_num < pcie_port.pcie_por_num) {
		dme_port = f_get_pcie_port(port_num);
		if (!dme_port) {
			port_num++;
			continue;
		}

		if (dme_port->ep && dme_port->ep->loop_coned) {
			pci_unregister_driver(&f_pcie_ep_driver);
			kfree(f_pcie_ep_driver.name);
		}

		kfree(pcie_port.dme_pcie[port_num].ep);
		kfree(pcie_port.dme_pcie[port_num].rc);
		pcie_port.dme_pcie[port_num].ep = NULL;
		pcie_port.dme_pcie[port_num].rc = NULL;
		port_num++;
	};

	kfree(pcie_port.dme_pcie);
	kfree(pcie_port.nr_port_map);
	pcie_port.dme_pcie = NULL;
	pcie_port.pcie_por_num = 0;
	pcie_port.pcie_por_num_release = 0;
	pcie_port.pcie_por_num_total = 0;
	pcie_port.rc_cnt = 0;
}

static int f_pcie_probe(struct platform_device *pdev)
{
	struct resource wrap;
	struct f_pcie_port *dme_port= NULL;
	struct pcie_pro *sysoc_pcie;
	struct device_node *np_pm, *dev_np;
	struct device *dev = &pdev->dev;
	struct device *parent_dev;
	unsigned long flags;
	int r = 0;
	size_t size;
	u32 index;

	parent_dev = pdev->dev.parent;
	sysoc_pcie = dev_get_drvdata(parent_dev);
	if (!sysoc_pcie)
		return -EINVAL;

	spin_lock_irqsave(&pcie_older_lock, flags);
	dev_np = pdev->dev.of_node;
	index = sysoc_pcie->id;
	spin_unlock_irqrestore(&pcie_older_lock, flags);

	dme_port = pcie_port.dme_pcie;
	if (!dme_port) {
		size = sizeof(struct f_pcie_port) * (index + 1);
		dme_port = kmalloc(size, GFP_KERNEL);
		if (!dme_port) {
			dev_err(dev, "dme_port is null\n");
			goto fail3;
		}

		size = sizeof(u32) * (index + 1);
		pcie_port.nr_port_map = kmalloc(size, GFP_KERNEL);
		if (!pcie_port.nr_port_map) {
			dev_err(dev, "pcie_port.nr_port_map is null\n");
			goto fail2;
		}
		pcie_port.dme_pcie = dme_port;
		INIT_LIST_HEAD(&pcie_port.list);
		spin_lock_init(&pcie_port.lock);
		pcie_port.pcie_por_num_total = index + 1;
		pcie_port.pcie_por_num_release = index + 1;
	}

	dme_port->pcie_type = sysoc_pcie->pcie_type;
	dme_port[index].wrapper = NULL;
	dme_port[index].index = index;

#ifdef CONFIG_PM_RUNTIME
	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);
#endif

	/* optional pcie wrapper -- used on S73 power */
	r = of_address_to_resource(dev_np, 1, &wrap);
	if (!r) {
		void __iomem *wrap_io;

		wrap_io = ioremap(wrap.start, resource_size(&wrap));
		writew(EN_DMA_INT, wrap_io + ST_DMA_INT_ENB);
		dme_port[index].wrapper = wrap_io;
	}

	spin_lock_irqsave(&pcie_port.lock, flags);
	list_add_tail(&dme_port[index].ports, &pcie_port.list);
	pcie_port.pcie_por_num++;
	spin_unlock_irqrestore(&pcie_port.lock, flags);

	if (dme_port->pcie_type) {
		struct dme_rc *rc;

		dev_info(dev, "Root Complex Mode (%d)\n", index);
		rc = kzalloc(sizeof(struct dme_rc), GFP_KERNEL);
		dme_port[index].rc = rc;
		if (!dme_port[index].rc) {
			dev_err(dev, "dme_port[%d].rc nill\n", index);
			goto fail1;
		}

		dme_port[index].rc->dev = &pdev->dev;
		dme_port[index].ep = NULL;
		r = f_pcie_host_probe(pdev, pdev->dev.of_node, index);
		if (r < 0) {
			dev_err(dev, "f_pcie_host_probe fail\n");
			goto fail1;
		}
		pcie_port.rc_cnt++;
	} else {
		struct dme_ep *ep;

		dev_info(dev, "Endpoint Mode (%d)\n", index);
		ep = kzalloc(sizeof(struct dme_ep), GFP_KERNEL);
		dme_port[index].ep = ep;
		if (!dme_port[index].ep) {
			dev_err(dev, "dme_port[%d].ep nill\n", index);
			goto fail1;
		}

		dme_port[index].ep->loop_coned = sysoc_pcie->loop_coned;
		dme_port[index].ep->dev = &pdev->dev;
		dme_port[index].rc = NULL;
		
		r = f_pcie_ep_probe(pdev, pdev->dev.of_node, index);
		if (r < 0) {
			dev_err(dev, "f_pcie_ep_probe fail\n");
			goto fail1;
		}
	}

	pcie_port.dme_pcie[index].pm_cap = 0;
	np_pm = of_parse_phandle(dev->of_node, "power-domains", 0);
	if (np_pm) {
		pcie_port.dme_pcie[index].pm_cap = 1;
		r = pm_genpd_add_callbacks(dev, &gpd_dev_ops, NULL);
		if (r)
			dev_err(dev, "pm_genpd_add_callbacks fail\n");
	}

	if (pcie_port.pcie_por_num_total == pcie_port.pcie_por_num)
		pcie_init();

	return 0;
fail1:

	kfree(pcie_port.dme_pcie);

	if (dme_port[index].wrapper)
		iounmap(dme_port[index].wrapper);

	if (pcie_port.dme_pcie[index].rc)
		kfree(pcie_port.dme_pcie[index].rc);

	if (pcie_port.dme_pcie[index].ep)
		kfree(pcie_port.dme_pcie[index].ep);

fail2:
	if (dme_port)
		kfree(dme_port);

fail3:
	return -ENOMEM;
}

static int f_pcie_remove(struct platform_device *pdev)
{
	struct dme_rc *rc = NULL;
	struct dme_ep *ep = NULL;
	struct pci_bus *root_bus = NULL;
	struct pci_dev *root_pci = NULL;
	struct pci_sys_data *sys = NULL;
	struct f_pcie_port *dme_port = NULL;
	struct device *dev = &pdev->dev;
	int r, i = 0, domain = 0;
	u32 clk_id = 0, dma_irq_cnt = 0;

	dme_port = dev_get_platdata(&pdev->dev);
	if (!dme_port) {
		dev_err(dev, "dev->platform_data is null\n");
		return -EINVAL;
	}

	if (!dme_port->rc && !dme_port->ep) {
		dev_err(dev, "f_pcie_port is fail\n");
		dev_err(dev, "dme_port index %d\n", dme_port->index);
		return -EINVAL;
	}

	/* ep mode */
	if (dme_port->ep) {
		ep = dme_port->ep;
		for (clk_id = 0; clk_id < ep->clk_num; clk_id++)
			if (ep->clk[clk_id])
				clk_put(ep->clk[clk_id]);

		dma_irq_cnt = sizeof(ep->dma_irq)/sizeof(int);
		while (i < dma_irq_cnt) {
			if (ep->dma_irq[i])
				free_irq(ep->dma_irq[i], ep);
			i++;
		};

		if (ep->regs)
			iounmap(ep->regs);

		if (dme_port->wrapper)
			iounmap(dme_port->wrapper);

		if (dme_port->pm_cap) {
			r = __pm_genpd_remove_callbacks(&pdev->dev, false);
			if (r)
				dev_err(dev, "pm_genpd_remove fail\n");
		}
		pdev->dev.platform_data = NULL;
		pm_runtime_put_sync(&pdev->dev);
		pm_runtime_disable(&pdev->dev);
	}

	/* rc mode */
	if (dme_port->rc) {
		rc = dme_port->rc;
		root_bus = rc->root_pcibus;
		if (!root_bus) {
			dev_err(dev, "can't find root_bus\n");
			return -EINVAL;
		}
		if (rc->irq_domain)
			irq_domain_remove(rc->irq_domain);

		disable_irq(rc->slav_mem_irq);
		if (rc->lig_irq)
			disable_irq(rc->lig_irq);
		free_irq(rc->slav_mem_irq, rc);
		if (rc->lig_irq)
			free_irq(rc->lig_irq, rc);

		domain = pci_domain_nr(root_bus);
		root_pci = pci_get_domain_bus_and_slot(domain, 0, 0);
		if (root_pci)
			pci_stop_and_remove_bus_device(root_pci);

		sys = (struct pci_sys_data *)root_bus->sysdata;
		release_resource(&sys->io_res);
		pci_stop_root_bus(root_bus);
		pci_remove_root_bus(root_bus);

		for (clk_id = 0; clk_id < rc->clk_num; clk_id++)
			if (rc->clk[clk_id])
				clk_put(rc->clk[clk_id]);

		if (rc->rc_cfg_base)
			iounmap(rc->rc_cfg_base);

		if (rc->ep_cfg_base)
			iounmap(rc->ep_cfg_base);

		if (rc->msi_lig)
			iounmap(rc->msi_lig);

		kfree(rc->clk);
		rc->inited = 0;
		if (dme_port->pm_cap) {
			r = __pm_genpd_remove_callbacks(dev, false);
			if (r)
				dev_err(dev, "pm_genpd_remove fail\n");
		}
		pm_runtime_put_sync(&pdev->dev);
		pm_runtime_disable(&pdev->dev);
		pdev->dev.platform_data = NULL;
	}

	pcie_port.pcie_por_num_release--;
	if (pcie_port.pcie_por_num_release == 0)
		pcie_uninit();

	return 0;
}

static const struct of_device_id f_pcie_dt_ids[] = {
	{.compatible = "fujitsu,mb86s7x-pcie_dme-integration" },
	{/* sentinel */ }
};
MODULE_DEVICE_TABLE(of, f_pcie_dt_ids);

#ifdef CONFIG_PM
static const struct dev_pm_ops f_pcie_pm_ops = {
	.suspend_noirq = f_pcie_pm_suspend,
	.resume_noirq = f_pcie_pm_resume,

#ifdef CONFIG_PM_RUNTIME
	SET_RUNTIME_PM_OPS(f_pcie_runtime_suspend,
		f_pcie_runtime_resume, NULL)
#endif
};
#endif

static struct platform_driver f_pcie_driver = {
	.driver		= {
		.name	= "f_pcie_dme",
		.owner  = THIS_MODULE,
		.of_match_table = f_pcie_dt_ids,

#ifdef CONFIG_PM
		.pm = &f_pcie_pm_ops,
#endif

	},
	.probe		= f_pcie_probe,
	.remove		= f_pcie_remove,
};

static int __init f_pcie_init(void)
{
	return platform_driver_register(&f_pcie_driver);
}

static void __exit f_pcie_exit(void)
{
	platform_driver_unregister(&f_pcie_driver);
}

module_init(f_pcie_init);
module_exit(f_pcie_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("PCIE IP driver");
MODULE_AUTHOR("slash.huang@tw.fujitsu.com");
