// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Realtek RTD1195/RTD1295/RTD1395 IRQ mux
 *
 * Copyright (C) 2017 Realtek Semiconductor Corporation
 * Copyright (c) 2017-2019 Andreas Färber
 */

#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>

#define UMSK_ISR_WRITE_DATA	BIT(0)
#define ISR_WRITE_DATA		BIT(0)

struct rtd1195_irq_mux_info {
	const char	*name;
	unsigned int	isr_offset;
	unsigned int	umsk_isr_offset;
	unsigned int	scpu_int_en_offset;
	const u32	*isr_to_int_en_mask;
};

#define SCPU_INT_EN_RSV_MASK	0
#define SCPU_INT_EN_NMI_MASK	GENMASK(31, 0)

struct rtd1195_irq_mux_data {
	void __iomem				*reg_isr;
	void __iomem				*reg_umsk_isr;
	void __iomem				*reg_scpu_int_en;
	const struct rtd1195_irq_mux_info	*info;
	int					irq;
	u32					scpu_int_en;
	struct irq_chip				chip;
	struct irq_domain			*domain;
	raw_spinlock_t				lock;
};

static void rtd1195_mux_irq_handle(struct irq_desc *desc)
{
	struct rtd1195_irq_mux_data *mux = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u32 isr;
	int i;

	chained_irq_enter(chip, desc);

	isr = readl_relaxed(mux->reg_isr);

	while (isr) {
		i = __ffs(isr);
		isr &= ~BIT(i);

		generic_handle_irq(irq_find_mapping(mux->domain, i));
	}

	chained_irq_exit(chip, desc);
}

static void rtd1195_mux_ack_irq(struct irq_data *data)
{
	struct rtd1195_irq_mux_data *mux = irq_data_get_irq_chip_data(data);

	writel_relaxed(BIT(data->hwirq) & ~ISR_WRITE_DATA, mux->reg_isr);
}

static void rtd1195_mux_mask_irq(struct irq_data *data)
{
	struct rtd1195_irq_mux_data *mux = irq_data_get_irq_chip_data(data);
	u32 mask = mux->info->isr_to_int_en_mask[data->hwirq];
	unsigned long flags;

	raw_spin_lock_irqsave(&mux->lock, flags);

	mux->scpu_int_en &= ~mask;
	writel_relaxed(mux->scpu_int_en, mux->reg_scpu_int_en);

	raw_spin_unlock_irqrestore(&mux->lock, flags);
}

static void rtd1195_mux_unmask_irq(struct irq_data *data)
{
	struct rtd1195_irq_mux_data *mux = irq_data_get_irq_chip_data(data);
	u32 mask = mux->info->isr_to_int_en_mask[data->hwirq];
	unsigned long flags;

	raw_spin_lock_irqsave(&mux->lock, flags);

	mux->scpu_int_en |= mask;
	writel_relaxed(mux->scpu_int_en, mux->reg_scpu_int_en);

	raw_spin_unlock_irqrestore(&mux->lock, flags);
}

static int rtd1195_mux_get_irqchip_state(struct irq_data *data,
	enum irqchip_irq_state which, bool *state)
{
	struct rtd1195_irq_mux_data *mux = irq_data_get_irq_chip_data(data);
	u32 val;

	switch (which) {
	case IRQCHIP_STATE_PENDING:
		/*
		 * UMSK_ISR provides the unmasked pending interrupts,
		 * except UART and I2C.
		 */
		val = readl_relaxed(mux->reg_umsk_isr);
		*state = !!(val & BIT(data->hwirq));
		break;
	case IRQCHIP_STATE_ACTIVE:
		/*
		 * ISR provides the masked pending interrupts,
		 * including UART and I2C.
		 */
		val = readl_relaxed(mux->reg_isr);
		*state = !!(val & BIT(data->hwirq));
		break;
	case IRQCHIP_STATE_MASKED:
		val = mux->info->isr_to_int_en_mask[data->hwirq];
		*state = !(mux->scpu_int_en & val);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct irq_chip rtd1195_mux_irq_chip = {
	.irq_ack		= rtd1195_mux_ack_irq,
	.irq_mask		= rtd1195_mux_mask_irq,
	.irq_unmask		= rtd1195_mux_unmask_irq,
	.irq_get_irqchip_state	= rtd1195_mux_get_irqchip_state,
};

static int rtd1195_mux_irq_domain_map(struct irq_domain *d,
		unsigned int irq, irq_hw_number_t hw)
{
	struct rtd1195_irq_mux_data *mux = d->host_data;
	u32 mask;

	if (BIT(hw) == ISR_WRITE_DATA)
		return -EINVAL;

	mask = mux->info->isr_to_int_en_mask[hw];
	if (mask == SCPU_INT_EN_RSV_MASK)
		return -EINVAL;

	if (mask == SCPU_INT_EN_NMI_MASK)
		return -ENOTSUPP;

	irq_set_chip_and_handler(irq, &mux->chip, handle_level_irq);
	irq_set_chip_data(irq, mux);
	irq_set_probe(irq);

	return 0;
}

static const struct irq_domain_ops rtd1195_mux_irq_domain_ops = {
	.xlate	= irq_domain_xlate_onecell,
	.map	= rtd1195_mux_irq_domain_map,
};

enum rtd1195_iso_isr_bits {
	RTD1195_ISO_ISR_TC3_SHIFT		= 1,
	RTD1195_ISO_ISR_UR0_SHIFT		= 2,
	RTD1195_ISO_ISR_IRDA_SHIFT		= 5,
	RTD1195_ISO_ISR_WDOG_NMI_SHIFT		= 7,
	RTD1195_ISO_ISR_I2C0_SHIFT		= 8,
	RTD1195_ISO_ISR_TC4_SHIFT		= 9,
	RTD1195_ISO_ISR_I2C6_SHIFT		= 10,
	RTD1195_ISO_ISR_RTC_HSEC_SHIFT		= 12,
	RTD1195_ISO_ISR_RTC_ALARM_SHIFT		= 13,
	RTD1195_ISO_ISR_VFD_WDONE_SHIFT		= 14,
	RTD1195_ISO_ISR_VFD_ARDKPADA_SHIFT	= 15,
	RTD1195_ISO_ISR_VFD_ARDKPADDA_SHIFT	= 16,
	RTD1195_ISO_ISR_VFD_ARDSWA_SHIFT	= 17,
	RTD1195_ISO_ISR_VFD_ARDSWDA_SHIFT	= 18,
	RTD1195_ISO_ISR_GPIOA_SHIFT		= 19,
	RTD1195_ISO_ISR_GPIODA_SHIFT		= 20,
	RTD1195_ISO_ISR_CEC_SHIFT		= 22,
};

static const u32 rtd1195_iso_isr_to_scpu_int_en_mask[32] = {
	[RTD1195_ISO_ISR_UR0_SHIFT]		= BIT(2),
	[RTD1195_ISO_ISR_IRDA_SHIFT]		= BIT(5),
	[RTD1195_ISO_ISR_I2C0_SHIFT]		= BIT(8),
	[RTD1195_ISO_ISR_I2C6_SHIFT]		= BIT(10),
	[RTD1195_ISO_ISR_RTC_HSEC_SHIFT]	= BIT(12),
	[RTD1195_ISO_ISR_RTC_ALARM_SHIFT]	= BIT(13),
	[RTD1195_ISO_ISR_VFD_WDONE_SHIFT]	= BIT(14),
	[RTD1195_ISO_ISR_VFD_ARDKPADA_SHIFT]	= BIT(15),
	[RTD1195_ISO_ISR_VFD_ARDKPADDA_SHIFT]	= BIT(16),
	[RTD1195_ISO_ISR_VFD_ARDSWA_SHIFT]	= BIT(17),
	[RTD1195_ISO_ISR_VFD_ARDSWDA_SHIFT]	= BIT(18),
	[RTD1195_ISO_ISR_GPIOA_SHIFT]		= BIT(19),
	[RTD1195_ISO_ISR_GPIODA_SHIFT]		= BIT(20),
	[RTD1195_ISO_ISR_CEC_SHIFT]		= BIT(22),
};

enum rtd1195_misc_isr_bits {
	RTD1195_MIS_ISR_WDOG_NMI_SHIFT		= 2,
	RTD1195_MIS_ISR_UR1_SHIFT		= 3,
	RTD1195_MIS_ISR_I2C1_SHIFT		= 4,
	RTD1195_MIS_ISR_UR1_TO_SHIFT		= 5,
	RTD1195_MIS_ISR_TC0_SHIFT		= 6,
	RTD1195_MIS_ISR_TC1_SHIFT		= 7,
	RTD1195_MIS_ISR_RTC_HSEC_SHIFT		= 9,
	RTD1195_MIS_ISR_RTC_MIN_SHIFT		= 10,
	RTD1195_MIS_ISR_RTC_HOUR_SHIFT		= 11,
	RTD1195_MIS_ISR_RTC_DATE_SHIFT		= 12,
	RTD1195_MIS_ISR_I2C5_SHIFT		= 14,
	RTD1195_MIS_ISR_I2C4_SHIFT		= 15,
	RTD1195_MIS_ISR_GPIOA_SHIFT		= 19,
	RTD1195_MIS_ISR_GPIODA_SHIFT		= 20,
	RTD1195_MIS_ISR_LSADC_SHIFT		= 21,
	RTD1195_MIS_ISR_I2C3_SHIFT		= 23,
	RTD1195_MIS_ISR_I2C2_SHIFT		= 26,
	RTD1195_MIS_ISR_GSPI_SHIFT		= 27,
};

static const u32 rtd1195_misc_isr_to_scpu_int_en_mask[32] = {
	[RTD1195_MIS_ISR_UR1_SHIFT]		= BIT(3),
	[RTD1195_MIS_ISR_I2C1_SHIFT]		= BIT(4),
	[RTD1195_MIS_ISR_UR1_TO_SHIFT]		= BIT(5),
	[RTD1195_MIS_ISR_RTC_MIN_SHIFT]		= BIT(10),
	[RTD1195_MIS_ISR_RTC_HOUR_SHIFT]	= BIT(11),
	[RTD1195_MIS_ISR_RTC_DATE_SHIFT]	= BIT(12),
	[RTD1195_MIS_ISR_I2C5_SHIFT]		= BIT(14),
	[RTD1195_MIS_ISR_I2C4_SHIFT]		= BIT(15),
	[RTD1195_MIS_ISR_GPIOA_SHIFT]		= BIT(19),
	[RTD1195_MIS_ISR_GPIODA_SHIFT]		= BIT(20),
	[RTD1195_MIS_ISR_LSADC_SHIFT]		= BIT(21),
	[RTD1195_MIS_ISR_I2C2_SHIFT]		= BIT(26),
	[RTD1195_MIS_ISR_GSPI_SHIFT]		= BIT(27),
	[RTD1195_MIS_ISR_I2C3_SHIFT]		= BIT(28),
	[RTD1195_MIS_ISR_WDOG_NMI_SHIFT]	= SCPU_INT_EN_NMI_MASK,
};

enum rtd1295_iso_isr_bits {
	RTD1295_ISO_ISR_UR0_SHIFT		= 2,
	RTD1295_ISO_ISR_IRDA_SHIFT		= 5,
	RTD1295_ISO_ISR_I2C0_SHIFT		= 8,
	RTD1295_ISO_ISR_I2C1_SHIFT		= 11,
	RTD1295_ISO_ISR_RTC_HSEC_SHIFT		= 12,
	RTD1295_ISO_ISR_RTC_ALARM_SHIFT		= 13,
	RTD1295_ISO_ISR_GPIOA_SHIFT		= 19,
	RTD1295_ISO_ISR_GPIODA_SHIFT		= 20,
	RTD1295_ISO_ISR_GPHY_DV_SHIFT		= 29,
	RTD1295_ISO_ISR_GPHY_AV_SHIFT		= 30,
	RTD1295_ISO_ISR_I2C1_REQ_SHIFT		= 31,
};

static const u32 rtd1295_iso_isr_to_scpu_int_en_mask[32] = {
	[RTD1295_ISO_ISR_UR0_SHIFT]		= BIT(2),
	[RTD1295_ISO_ISR_IRDA_SHIFT]		= BIT(5),
	[RTD1295_ISO_ISR_I2C0_SHIFT]		= BIT(8),
	[RTD1295_ISO_ISR_I2C1_SHIFT]		= BIT(11),
	[RTD1295_ISO_ISR_RTC_HSEC_SHIFT]	= BIT(12),
	[RTD1295_ISO_ISR_RTC_ALARM_SHIFT]	= BIT(13),
	[RTD1295_ISO_ISR_GPIOA_SHIFT]		= BIT(19),
	[RTD1295_ISO_ISR_GPIODA_SHIFT]		= BIT(20),
	[RTD1295_ISO_ISR_GPHY_DV_SHIFT]		= BIT(29),
	[RTD1295_ISO_ISR_GPHY_AV_SHIFT]		= BIT(30),
	[RTD1295_ISO_ISR_I2C1_REQ_SHIFT]	= BIT(31),
};

enum rtd1295_misc_isr_bits {
	RTD1295_MIS_ISR_WDOG_NMI_SHIFT		= 2,
	RTD1295_MIS_ISR_UR1_SHIFT		= 3,
	RTD1295_MIS_ISR_UR1_TO_SHIFT		= 5,
	RTD1295_MIS_ISR_UR2_SHIFT		= 8,
	RTD1295_MIS_ISR_RTC_MIN_SHIFT		= 10,
	RTD1295_MIS_ISR_RTC_HOUR_SHIFT		= 11,
	RTD1295_MIS_ISR_RTC_DATA_SHIFT		= 12,
	RTD1295_MIS_ISR_UR2_TO_SHIFT		= 13,
	RTD1295_MIS_ISR_I2C5_SHIFT		= 14,
	RTD1295_MIS_ISR_I2C4_SHIFT		= 15,
	RTD1295_MIS_ISR_GPIOA_SHIFT		= 19,
	RTD1295_MIS_ISR_GPIODA_SHIFT		= 20,
	RTD1295_MIS_ISR_LSADC0_SHIFT		= 21,
	RTD1295_MIS_ISR_LSADC1_SHIFT		= 22,
	RTD1295_MIS_ISR_I2C3_SHIFT		= 23,
	RTD1295_MIS_ISR_SC0_SHIFT		= 24,
	RTD1295_MIS_ISR_I2C2_SHIFT		= 26,
	RTD1295_MIS_ISR_GSPI_SHIFT		= 27,
	RTD1295_MIS_ISR_FAN_SHIFT		= 29,
};

static const u32 rtd1295_misc_isr_to_scpu_int_en_mask[32] = {
	[RTD1295_MIS_ISR_UR1_SHIFT]		= BIT(3),
	[RTD1295_MIS_ISR_UR1_TO_SHIFT]		= BIT(5),
	[RTD1295_MIS_ISR_UR2_TO_SHIFT]		= BIT(6),
	[RTD1295_MIS_ISR_UR2_SHIFT]		= BIT(7),
	[RTD1295_MIS_ISR_RTC_MIN_SHIFT]		= BIT(10),
	[RTD1295_MIS_ISR_RTC_HOUR_SHIFT]	= BIT(11),
	[RTD1295_MIS_ISR_RTC_DATA_SHIFT]	= BIT(12),
	[RTD1295_MIS_ISR_I2C5_SHIFT]		= BIT(14),
	[RTD1295_MIS_ISR_I2C4_SHIFT]		= BIT(15),
	[RTD1295_MIS_ISR_GPIOA_SHIFT]		= BIT(19),
	[RTD1295_MIS_ISR_GPIODA_SHIFT]		= BIT(20),
	[RTD1295_MIS_ISR_LSADC0_SHIFT]		= BIT(21),
	[RTD1295_MIS_ISR_LSADC1_SHIFT]		= BIT(22),
	[RTD1295_MIS_ISR_SC0_SHIFT]		= BIT(24),
	[RTD1295_MIS_ISR_I2C2_SHIFT]		= BIT(26),
	[RTD1295_MIS_ISR_GSPI_SHIFT]		= BIT(27),
	[RTD1295_MIS_ISR_I2C3_SHIFT]		= BIT(28),
	[RTD1295_MIS_ISR_FAN_SHIFT]		= BIT(29),
	[RTD1295_MIS_ISR_WDOG_NMI_SHIFT]	= SCPU_INT_EN_NMI_MASK,
};

enum rtd1395_iso_isr_bits {
	RTD1395_ISO_ISR_UR0_SHIFT		= 2,
	RTD1395_ISO_ISR_IRDA_SHIFT		= 5,
	RTD1395_ISO_ISR_I2C0_SHIFT		= 8,
	RTD1395_ISO_ISR_I2C1_SHIFT		= 11,
	RTD1395_ISO_ISR_RTC_HSEC_SHIFT		= 12,
	RTD1395_ISO_ISR_RTC_ALARM_SHIFT		= 13,
	RTD1395_ISO_ISR_LSADC0_SHIFT		= 16,
	RTD1395_ISO_ISR_LSADC1_SHIFT		= 17,
	RTD1395_ISO_ISR_GPIOA_SHIFT		= 19,
	RTD1395_ISO_ISR_GPIODA_SHIFT		= 20,
	RTD1395_ISO_ISR_GPHY_HV_SHIFT		= 28,
	RTD1395_ISO_ISR_GPHY_DV_SHIFT		= 29,
	RTD1395_ISO_ISR_GPHY_AV_SHIFT		= 30,
	RTD1395_ISO_ISR_I2C1_REQ_SHIFT		= 31,
};

static const u32 rtd1395_iso_isr_to_scpu_int_en_mask[32] = {
	[RTD1395_ISO_ISR_UR0_SHIFT]		= BIT(2),
	[RTD1395_ISO_ISR_IRDA_SHIFT]		= BIT(5),
	[RTD1395_ISO_ISR_I2C0_SHIFT]		= BIT(8),
	[RTD1395_ISO_ISR_I2C1_SHIFT]		= BIT(11),
	[RTD1395_ISO_ISR_RTC_HSEC_SHIFT]	= BIT(12),
	[RTD1395_ISO_ISR_RTC_ALARM_SHIFT]	= BIT(13),
	[RTD1395_ISO_ISR_LSADC0_SHIFT]		= BIT(16),
	[RTD1395_ISO_ISR_LSADC1_SHIFT]		= BIT(17),
	[RTD1395_ISO_ISR_GPIOA_SHIFT]		= BIT(19),
	[RTD1395_ISO_ISR_GPIODA_SHIFT]		= BIT(20),
	[RTD1395_ISO_ISR_GPHY_HV_SHIFT]		= BIT(28),
	[RTD1395_ISO_ISR_GPHY_DV_SHIFT]		= BIT(29),
	[RTD1395_ISO_ISR_GPHY_AV_SHIFT]		= BIT(30),
	[RTD1395_ISO_ISR_I2C1_REQ_SHIFT]	= BIT(31),
};

enum rtd1395_misc_isr_bits {
	RTD1395_MIS_ISR_UR1_SHIFT		= 3,
	RTD1395_MIS_ISR_UR1_TO_SHIFT		= 5,
	RTD1395_MIS_ISR_UR2_SHIFT		= 8,
	RTD1395_MIS_ISR_UR2_TO_SHIFT		= 13,
	RTD1395_MIS_ISR_I2C5_SHIFT		= 14,
	RTD1395_MIS_ISR_SC0_SHIFT		= 24,
	RTD1395_MIS_ISR_SPI_SHIFT		= 27,
	RTD1395_MIS_ISR_FAN_SHIFT		= 29,
};

static const u32 rtd1395_misc_isr_to_scpu_int_en_mask[32] = {
	[RTD1395_MIS_ISR_UR1_SHIFT]		= BIT(3),
	[RTD1395_MIS_ISR_UR1_TO_SHIFT]		= BIT(5),
	[RTD1395_MIS_ISR_UR2_TO_SHIFT]		= BIT(6),
	[RTD1395_MIS_ISR_UR2_SHIFT]		= BIT(7),
	[RTD1395_MIS_ISR_I2C5_SHIFT]		= BIT(14),
	[RTD1395_MIS_ISR_SC0_SHIFT]		= BIT(24),
	[RTD1395_MIS_ISR_SPI_SHIFT]		= BIT(27),
	[RTD1395_MIS_ISR_FAN_SHIFT]		= BIT(29),
};

enum rtd1619_iso_isr_bits {
	RTD1619_ISO_ISR_UR0_SHIFT		= 2,
	RTD1619_ISO_ISR_LSADC0_SHIFT		= 3,
	RTD1619_ISO_ISR_LSADC1_SHIFT		= 4,
	RTD1619_ISO_ISR_IRDA_SHIFT		= 5,
	RTD1619_ISO_ISR_I2C0_SHIFT		= 8,
	RTD1619_ISO_ISR_I2C1_SHIFT		= 11,
	RTD1619_ISO_ISR_RTC_HSEC_SHIFT		= 12,
	RTD1619_ISO_ISR_RTC_ALARM_SHIFT		= 13,
	RTD1619_ISO_ISR_VFD_WDONE_SHIFT		= 14,
	RTD1619_ISO_ISR_VFD_ARDKPADA_SHIFT	= 15,
	RTD1619_ISO_ISR_VFD_ARDKPADDA_SHIFT	= 16,
	RTD1619_ISO_ISR_VFD_ARDSWA_SHIFT	= 17,
	RTD1619_ISO_ISR_VFD_ARDSWDA_SHIFT	= 18,
	RTD1619_ISO_ISR_GPIOA_SHIFT		= 19,
	RTD1619_ISO_ISR_GPIODA_SHIFT		= 20,
	RTD1619_ISO_ISR_GPHY_HV_SHIFT		= 28,
	RTD1619_ISO_ISR_GPHY_DV_SHIFT		= 29,
	RTD1619_ISO_ISR_GPHY_AV_SHIFT		= 30,
	RTD1619_ISO_ISR_I2C1_REQ_SHIFT		= 31,
};

static const u32 rtd1619_iso_isr_to_scpu_int_en_mask[32] = {
	[RTD1619_ISO_ISR_UR0_SHIFT]		= BIT(2),
	[RTD1619_ISO_ISR_LSADC0_SHIFT]		= BIT(3),
	[RTD1619_ISO_ISR_LSADC1_SHIFT]		= BIT(4),
	[RTD1619_ISO_ISR_IRDA_SHIFT]		= BIT(5),
	[RTD1619_ISO_ISR_I2C0_SHIFT]		= BIT(8),
	[RTD1619_ISO_ISR_I2C1_SHIFT]		= BIT(11),
	[RTD1619_ISO_ISR_RTC_HSEC_SHIFT]	= BIT(12),
	[RTD1619_ISO_ISR_RTC_ALARM_SHIFT]	= BIT(13),
	[RTD1619_ISO_ISR_VFD_WDONE_SHIFT]	= BIT(14),
	[RTD1619_ISO_ISR_VFD_ARDKPADA_SHIFT]	= BIT(15),
	[RTD1619_ISO_ISR_VFD_ARDKPADDA_SHIFT]	= BIT(16),
	[RTD1619_ISO_ISR_VFD_ARDSWA_SHIFT]	= BIT(17),
	[RTD1619_ISO_ISR_VFD_ARDSWDA_SHIFT]	= BIT(18),
	[RTD1619_ISO_ISR_GPIOA_SHIFT]		= BIT(19),
	[RTD1619_ISO_ISR_GPIODA_SHIFT]		= BIT(20),
	[RTD1619_ISO_ISR_GPHY_HV_SHIFT]		= BIT(28),
	[RTD1619_ISO_ISR_GPHY_DV_SHIFT]		= BIT(29),
	[RTD1619_ISO_ISR_GPHY_AV_SHIFT]		= BIT(30),
	[RTD1619_ISO_ISR_I2C1_REQ_SHIFT]	= BIT(31),
};

enum rtd1619_misc_isr_bits {
	RTD1619_MIS_ISR_UR1_SHIFT		= 3,
	RTD1619_MIS_ISR_UR1_TO_SHIFT		= 5,
	RTD1619_MIS_ISR_UR2_TO_SHIFT		= 6,
	RTD1619_MIS_ISR_UR2_SHIFT		= 7,
	RTD1619_MIS_ISR_RTC_MIN_SHIFT		= 10,
	RTD1619_MIS_ISR_RTC_HOUR_SHIFT		= 11,
	RTD1619_MIS_ISR_RTC_DATE_SHIFT		= 12,
	RTD1619_MIS_ISR_I2C5_SHIFT		= 14,
	RTD1619_MIS_ISR_I2C4_SHIFT		= 15,
	RTD1619_MIS_ISR_I2C3_SHIFT		= 23,
	RTD1619_MIS_ISR_SC0_SHIFT		= 24,
	RTD1619_MIS_ISR_SC1_SHIFT		= 25,
	RTD1619_MIS_ISR_SPI_SHIFT		= 27,
	RTD1619_MIS_ISR_FAN_SHIFT		= 29,
};

static const u32 rtd1619_misc_isr_to_scpu_int_en_mask[32] = {
	[RTD1619_MIS_ISR_UR1_SHIFT]		= BIT(3),
	[RTD1619_MIS_ISR_UR1_TO_SHIFT]		= BIT(5),
	[RTD1619_MIS_ISR_UR2_TO_SHIFT]		= BIT(6),
	[RTD1619_MIS_ISR_UR2_SHIFT]		= BIT(7),
	[RTD1619_MIS_ISR_RTC_MIN_SHIFT]		= BIT(10),
	[RTD1619_MIS_ISR_RTC_HOUR_SHIFT]	= BIT(11),
	[RTD1619_MIS_ISR_RTC_DATE_SHIFT]	= BIT(12),
	[RTD1619_MIS_ISR_I2C5_SHIFT]		= BIT(14),
	[RTD1619_MIS_ISR_I2C4_SHIFT]		= BIT(15),
	[RTD1619_MIS_ISR_SC0_SHIFT]		= BIT(24),
	[RTD1619_MIS_ISR_SC1_SHIFT]		= BIT(25),
	[RTD1619_MIS_ISR_SPI_SHIFT]		= BIT(27),
	[RTD1619_MIS_ISR_I2C3_SHIFT]		= BIT(28),
	[RTD1619_MIS_ISR_FAN_SHIFT]		= BIT(29),
};

static const struct rtd1195_irq_mux_info rtd1195_iso_irq_mux_info = {
	.name			= "iso",
	.isr_offset		= 0x0,
	.umsk_isr_offset	= 0x4,
	.scpu_int_en_offset	= 0x40,
	.isr_to_int_en_mask	= rtd1195_iso_isr_to_scpu_int_en_mask,
};

static const struct rtd1195_irq_mux_info rtd1295_iso_irq_mux_info = {
	.name			= "iso",
	.isr_offset		= 0x0,
	.umsk_isr_offset	= 0x4,
	.scpu_int_en_offset	= 0x40,
	.isr_to_int_en_mask	= rtd1295_iso_isr_to_scpu_int_en_mask,
};

static const struct rtd1195_irq_mux_info rtd1395_iso_irq_mux_info = {
	.name			= "iso",
	.isr_offset		= 0x0,
	.umsk_isr_offset	= 0x4,
	.scpu_int_en_offset	= 0x40,
	.isr_to_int_en_mask	= rtd1395_iso_isr_to_scpu_int_en_mask,
};

static const struct rtd1195_irq_mux_info rtd1619_iso_irq_mux_info = {
	.name			= "iso",
	.isr_offset		= 0x0,
	.umsk_isr_offset	= 0x4,
	.scpu_int_en_offset	= 0x40,
	.isr_to_int_en_mask	= rtd1619_iso_isr_to_scpu_int_en_mask,
};

static const struct rtd1195_irq_mux_info rtd1195_misc_irq_mux_info = {
	.name			= "misc",
	.umsk_isr_offset	= 0x8,
	.isr_offset		= 0xc,
	.scpu_int_en_offset	= 0x80,
	.isr_to_int_en_mask	= rtd1195_misc_isr_to_scpu_int_en_mask,
};

static const struct rtd1195_irq_mux_info rtd1295_misc_irq_mux_info = {
	.name			= "misc",
	.umsk_isr_offset	= 0x8,
	.isr_offset		= 0xc,
	.scpu_int_en_offset	= 0x80,
	.isr_to_int_en_mask	= rtd1295_misc_isr_to_scpu_int_en_mask,
};

static const struct rtd1195_irq_mux_info rtd1395_misc_irq_mux_info = {
	.name			= "misc",
	.umsk_isr_offset	= 0x8,
	.isr_offset		= 0xc,
	.scpu_int_en_offset	= 0x80,
	.isr_to_int_en_mask	= rtd1395_misc_isr_to_scpu_int_en_mask,
};

static const struct rtd1195_irq_mux_info rtd1619_misc_irq_mux_info = {
	.name			= "misc",
	.umsk_isr_offset	= 0x8,
	.isr_offset		= 0xc,
	.scpu_int_en_offset	= 0x80,
	.isr_to_int_en_mask	= rtd1619_misc_isr_to_scpu_int_en_mask,
};

static const struct of_device_id rtd1295_irq_mux_dt_matches[] = {
	{
		.compatible = "realtek,rtd1195-iso-irq-mux",
		.data = &rtd1195_iso_irq_mux_info,
	},
	{
		.compatible = "realtek,rtd1295-iso-irq-mux",
		.data = &rtd1295_iso_irq_mux_info,
	},
	{
		.compatible = "realtek,rtd1395-iso-irq-mux",
		.data = &rtd1395_iso_irq_mux_info,
	},
	{
		.compatible = "realtek,rtd1619-iso-irq-mux",
		.data = &rtd1619_iso_irq_mux_info,
	},
	{
		.compatible = "realtek,rtd1195-misc-irq-mux",
		.data = &rtd1195_misc_irq_mux_info,
	},
	{
		.compatible = "realtek,rtd1295-misc-irq-mux",
		.data = &rtd1295_misc_irq_mux_info,
	},
	{
		.compatible = "realtek,rtd1395-misc-irq-mux",
		.data = &rtd1395_misc_irq_mux_info,
	},
	{
		.compatible = "realtek,rtd1619-misc-irq-mux",
		.data = &rtd1619_misc_irq_mux_info,
	},
	{
	}
};

static int __init rtd1195_irq_mux_init(struct device_node *node,
				       struct device_node *parent)
{
	struct rtd1195_irq_mux_data *mux;
	const struct of_device_id *match;
	const struct rtd1195_irq_mux_info *info;
	void __iomem *base;

	match = of_match_node(rtd1295_irq_mux_dt_matches, node);
	if (!match)
		return -EINVAL;

	info = match->data;
	if (!info)
		return -EINVAL;

	base = of_iomap(node, 0);
	if (!base)
		return -EIO;

	mux = kzalloc(sizeof(*mux), GFP_KERNEL);
	if (!mux)
		return -ENOMEM;

	mux->info		= info;
	mux->reg_isr		= base + info->isr_offset;
	mux->reg_umsk_isr	= base + info->umsk_isr_offset;
	mux->reg_scpu_int_en	= base + info->scpu_int_en_offset;
	mux->chip		= rtd1195_mux_irq_chip;
	mux->chip.name		= info->name;

	mux->irq = irq_of_parse_and_map(node, 0);
	if (mux->irq <= 0) {
		kfree(mux);
		return -EINVAL;
	}

	raw_spin_lock_init(&mux->lock);

	/* Disable (mask) all interrupts */
	writel_relaxed(mux->scpu_int_en, mux->reg_scpu_int_en);

	/* Ack (clear) all interrupts - not all are in UMSK_ISR, so use ISR */
	writel_relaxed(~ISR_WRITE_DATA, mux->reg_isr);

	mux->domain = irq_domain_add_linear(node, 32,
				&rtd1195_mux_irq_domain_ops, mux);
	if (!mux->domain) {
		kfree(mux);
		return -ENOMEM;
	}

	irq_set_chained_handler_and_data(mux->irq, rtd1195_mux_irq_handle, mux);

	return 0;
}
IRQCHIP_DECLARE(rtd1195_iso_mux, "realtek,rtd1195-iso-irq-mux", rtd1195_irq_mux_init);
IRQCHIP_DECLARE(rtd1295_iso_mux, "realtek,rtd1295-iso-irq-mux", rtd1195_irq_mux_init);
IRQCHIP_DECLARE(rtd1395_iso_mux, "realtek,rtd1395-iso-irq-mux", rtd1195_irq_mux_init);
IRQCHIP_DECLARE(rtd1619_iso_mux, "realtek,rtd1619-iso-irq-mux", rtd1195_irq_mux_init);
IRQCHIP_DECLARE(rtd1195_misc_mux, "realtek,rtd1195-misc-irq-mux", rtd1195_irq_mux_init);
IRQCHIP_DECLARE(rtd1295_misc_mux, "realtek,rtd1295-misc-irq-mux", rtd1195_irq_mux_init);
IRQCHIP_DECLARE(rtd1395_misc_mux, "realtek,rtd1395-misc-irq-mux", rtd1195_irq_mux_init);
IRQCHIP_DECLARE(rtd1619_misc_mux, "realtek,rtd1619-misc-irq-mux", rtd1195_irq_mux_init);
