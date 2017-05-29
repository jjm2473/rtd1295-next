/*
 * PCI MSI support for the F_PCIE2_DME
 *
 * Copyright (C) 2013-2014 Fujitsu Semiconductor Ltd
 *
 * F_PCIE2_DME functions for Fujitsu SoC
 *
 * Author: Slash Huang <slash.huang@tw.fujitsu.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 */

#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/msi.h>
#include <asm/mach/irq.h>
#include <asm/irq.h>
#include "pcie_f_pcie2_dme.h"

int plat_supply_msi;

static struct dme_rc *sys_to_pcie(struct pci_sys_data *sys)
{
	return sys->private_data;
}

int get_virq_pos(struct dme_rc *rc, u32 virq)
{
	u32 pos;

	for (pos = 0; pos < S7X_MSIX_VECT; pos++) {
		if (rc->virq[pos] == virq)
			break;
	}
	return pos;
}

void arch_teardown_msi_irq(unsigned int irq)
{
	struct msi_desc *desc = irq_get_msi_desc(irq);
	struct dme_rc *rc = sys_to_pcie(desc->dev->bus->sysdata);
	unsigned int pos;

	pos = get_virq_pos(rc, irq);
	irq_dispose_mapping(irq);
	rc->virq[pos] = 0;
	rc->nvect--;
	clear_bit(pos, rc->msi_irq_in_use);
}

int arch_msi_check_device(struct pci_dev *dev, int nvec, int type)
{
	if (type == PCI_CAP_ID_MSI && nvec > S7X_MSI_VECT)
		return -EINVAL;

	if (type == PCI_CAP_ID_MSIX && nvec > S7X_MSIX_VECT)
		return -EINVAL;

	return 0;
}

int arch_setup_msi_irq(struct pci_dev *pdev, struct msi_desc *desc)
{
	struct msi_msg msg;
	struct dme_rc *rc = sys_to_pcie(desc->dev->bus->sysdata);
	unsigned long flags;
	int pos, virq;

	/*
	 * S70 not supply "local interrupt", 
	 * so S70 also not supply MSI/MSI-X
	 */
	if (rc->lig_irq <= 0)
		return -EINVAL;

	spin_lock_irqsave(&rc->msi_lock, flags);
	pos = find_first_zero_bit(rc->msi_irq_in_use, S7X_MSIX_VECT);

	if (pos < S7X_MSIX_VECT) {
		set_bit(pos, rc->msi_irq_in_use);
	} else {
		dev_err(&pdev->dev,
			"pos (%d)over VECT (%d)\n", pos, S7X_MSIX_VECT);
		return -ENOSPC;
	}

	virq = irq_create_mapping(rc->irq_domain, pos);
	irq_set_msi_desc(virq, desc);

	msg.address_lo = virt_to_phys((void *)rc->msi_data);
	msg.address_hi = 0;
	msg.data = (1 << pos);
	write_msi_msg(virq, &msg);
	rc->nvect++;
	rc->virq[pos] = virq;
	spin_unlock_irqrestore(&rc->msi_lock, flags);

	return 0;
}
