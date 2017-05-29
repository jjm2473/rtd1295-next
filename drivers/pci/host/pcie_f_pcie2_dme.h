#ifndef _LINUX_PCIE_F_PCIE2_DME_H
#define _LINUX_PCIE_F_PCIE2_DME_H

#define IO_MEM_RCS_NUM				10
#define S7X_MSIX_VECT 				32
#define S7X_MSI_VECT				16

struct pcie_pro {
	u32 pcie_type;
	u32 loop_coned;
	u32 id;
};

struct dme_rc {
	struct pci_bus *root_pcibus;
	struct resource mem_res[IO_MEM_RCS_NUM];
	struct resource io_res[IO_MEM_RCS_NUM];
	struct clk **clk;
	struct resource cfg_res;
	struct device *dev;
	int slav_mem_irq;
	int msg_irq[4];
	int mem_num;
	int io_num;
	void __iomem *rc_cfg_base;
	void __iomem *ep_cfg_base;
	void __iomem *msi_lig;
	spinlock_t conf_lock; /* for config r/w */
	spinlock_t msi_lock; /* for msi */
	u32 inited;
	u32 clk_num;
	u32 index;
	u32 link_up;
	resource_size_t mem_offset[IO_MEM_RCS_NUM];
	resource_size_t io_offset[IO_MEM_RCS_NUM];
	int lig_irq;
	int root_bus_nr;
	unsigned long msi_data;
	unsigned long msi_iova;
	size_t msi_iova_sz;
	struct irq_domain *irq_domain;
	DECLARE_BITMAP(msi_irq_in_use, S7X_MSIX_VECT);
	u32 virq[S7X_MSIX_VECT];
	u32 nvect;
};

struct f_pcie_port {
	struct dme_rc *rc;
	struct dme_ep *ep;
	struct list_head ports;
	void __iomem *wrapper;
	int index;
	int pm_cap;
	u32 pcie_type;
};

struct pcie_port {
	struct list_head list;
	struct f_pcie_port *dme_pcie;
	u32 *nr_port_map;
	u32 pcie_por_num;
	u32 pcie_por_num_release;
	u32 pcie_por_num_total;
	u32 rc_cnt;
	int total_rc_cnt;
	spinlock_t lock; /* */
};

#endif
