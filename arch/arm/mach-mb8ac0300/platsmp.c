/*
 * linux/arch/arm/mach-mb8ac0300/platsmp.c
 *
 * Copyright (C) 2013 FUJITSU SEMICONDUCTOR LIMITED
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 */

#include <linux/init.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/of_address.h>
#include <linux/irqchip/arm-gic.h>

#include <asm/cacheflush.h>
#include <asm/mach-types.h>
#include <asm/smp_scu.h>
#include <asm/smp_plat.h>
#include <asm/smp.h>

#define MRBC_GPREG0		0x30

static DEFINE_SPINLOCK(boot_lock);

extern void mb8ac0300_secondary_startup(void);
extern void __iomem *mb8ac0300_scu_base;
static int ncores;

static void __cpuinit mb8ac0300_secondary_init(unsigned int cpu)
{
	/*
	 * Let the primary cpu know we're out of the pen.
	 */
	pen_release = -1;
	smp_wmb();

	/*
	 * Synchronise with the boot thread.
	 */
	spin_lock(&boot_lock);
	spin_unlock(&boot_lock);
}

static struct of_device_id mrbc_ids[]  = {
	{ .compatible = "fujitsu,mb8ac0300-mrbc" },
	{},
};

static int __cpuinit mb8ac0300_boot_secondary(unsigned int cpu,
		struct task_struct *idle)
{
	unsigned long boot_vector;
	void __iomem *mrbc_base;
	struct device_node *np;
	unsigned long timeout;

	np = of_find_matching_node(NULL, mrbc_ids);
	if (!np)
		return -ENODEV;

	mrbc_base = of_iomap(np, 0);
	if (!mrbc_base)
		return -ENOMEM;

	/*
	 * Set synchronisation state between this boot processor
	 * and the secondary one
	 */
	spin_lock(&boot_lock);

	pen_release = cpu_logical_map(cpu);

	flush_cache_all();

	/* set the reset vector to point to the secondary_startup routine */
	boot_vector = virt_to_phys(mb8ac0300_secondary_startup);

	/* set vetcor to SECOND_BOOT_ADD_REG for CPU1 to use */
	writel(boot_vector, mrbc_base + MRBC_GPREG0);

	smp_wmb();	/* barrier */

	/* soft irq */
	arch_send_wakeup_ipi_mask(cpumask_of(cpu));

	/*
	 * now the secondary core is starting up let it run its
	 * calibrations, then wait for it to finish
	 */

	timeout = jiffies + HZ;
	while (time_before(jiffies, timeout)) {
		smp_rmb();
		if (pen_release == -1)
			break;
		udelay(10);
	}

	spin_unlock(&boot_lock);

	return pen_release != -1 ? -ENOSYS : 0;
}

static void __init mb8ac0300_smp_init_cpus(void)
{
	int i;

	ncores = scu_get_core_count(mb8ac0300_scu_base);

	if (ncores > nr_cpu_ids) {
		pr_warn("%s(): ncores (%u) > configured (%u)\n",
					__func__, ncores, nr_cpu_ids);
		ncores = nr_cpu_ids;
	}

	for (i = 0; i < ncores; i++)
		set_cpu_possible(i, true);
}

static void __init mb8ac0300_smp_prepare_cpus(unsigned int maxcpus)
{
	int i;

	for (i = 0; i < maxcpus; i++)
		set_cpu_present(i, true);

	scu_enable(mb8ac0300_scu_base);
}

static void __cpuinitdata mb8ac0300_cpu_die(unsigned int cpu)
{
	/* we put the platform to just WFI */
	for (;;) {
		__asm__ __volatile__("dsb\n\t" "wfi\n\t"
			: : : "memory");
		if (pen_release == cpu_logical_map(cpu)) {
			/*
			 * OK, proper wakeup, we're done
			 */
			break;
		}
	}
}

struct smp_operations mb8ac0300_smp_ops __initdata = {
	.smp_init_cpus		= mb8ac0300_smp_init_cpus,
	.smp_prepare_cpus	= mb8ac0300_smp_prepare_cpus,
	.smp_secondary_init	= mb8ac0300_secondary_init,
	.smp_boot_secondary	= mb8ac0300_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_die		= mb8ac0300_cpu_die,
#endif
};
