/* linux/arch/arm/mach-mb8ac0300/pm.c
 *
 * Copyright (C) 2012-2013 FUJITSU SEMICONDUCTOR LIMITED
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/suspend.h>
#include <linux/io.h>

#include <asm/cacheflush.h>

#include <linux/platform_data/mb8ac0300-irqs.h>
#include <linux/platform_data/mb8ac0300-iomap.h>
#include <linux/irqchip/irq-mb8ac0300.h>

#include "devices.h"

/**
 * mb8ac0300_pm_valid
 * @state: requested state
 *
 * Determine if we support the specified state
 */
static int mb8ac0300_pm_valid(suspend_state_t state)
{
	return (state == PM_SUSPEND_STANDBY) || (state == PM_SUSPEND_MEM);
}

static irqreturn_t mb8ac0300_wakeup_handler(int irq, void *arg)
{
	return IRQ_HANDLED;
}

/**
 * mb8ac0300_pm_prepare
 *
 * Prepare the wakeup irq.
 */
static int mb8ac0300_pm_prepare(void)
{
	int ret;
	/* set button A to wakeup */
	ret = exiu_irq_set_type(IRQ_EXTINT21, IRQ_TYPE_EDGE_RISING);
	if (ret)
		return ret;
	ret = request_irq(IRQ_EXTINT21, mb8ac0300_wakeup_handler,
		IRQF_TRIGGER_RISING, "external irq", NULL);
	if (ret)
		return ret;
	ret = irq_set_irq_wake(IRQ_EXTINT21, 1);
	return ret;
}

/**
  * mb8ac0300_pm_enter
  * @state: requested state
  */
static int mb8ac0300_pm_enter(suspend_state_t state)
{
	pr_debug("Entered %s state:%d\n", __func__, state);

	switch (state) {
	case PM_SUSPEND_STANDBY:
		/*
		 * STANDBY mode just goes into WFI state
		 */
		asm("wfi");		/* wait for interrupt */
		break;
	case PM_SUSPEND_MEM:
		/* virt to phys offset */
		mb8ac0300_cpu_save(PHYS_OFFSET - PAGE_OFFSET);
		break;
	}
	return 0;
}

static void mb8ac0300_pm_finish(void)
{
	free_irq(IRQ_EXTINT21, NULL);
}

static const struct platform_suspend_ops mb8ac0300_pm_ops = {
	.valid		= mb8ac0300_pm_valid,
	.prepare	= mb8ac0300_pm_prepare,
	.enter		= mb8ac0300_pm_enter,
	.finish		= mb8ac0300_pm_finish,
};

int __init mb8ac0300_pm_init(void)
{
	suspend_set_ops(&mb8ac0300_pm_ops);
#ifdef CONFIG_SUSPEND
	memcpy((u32 *)MB8AC0300_XSRAM_VIRT, (u32 *)mb8ac0300_cpu_sleep,
					mb8ac0300_cpu_sleep_size);
#endif
	flush_cache_all();

	return 0;
}
