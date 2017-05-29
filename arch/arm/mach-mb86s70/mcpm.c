/*
 * arch/arm/mach-mb86s70/mcpm.c
 *
 * Shameless copy of tc_pm.c
 * Created by:	Nicolas Pitre, October 2012
 * Copyright:	(C) 2012  Linaro Limited
 * Some portions of this file were originally written by Achin Gupta
 * Copyright:   (C) 2012  ARM Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/irqchip/arm-gic.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/suspend.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/cpu.h>
#include <linux/syscore_ops.h>

#include <asm/mcpm.h>
#include <asm/proc-fns.h>
#include <asm/cacheflush.h>
#include <asm/cputype.h>
#include <asm/cp15.h>
#include <asm/psci.h>
#include <asm/system_misc.h>
#include <asm/suspend.h>
#include <asm/idmap.h>
#include <asm/smp_plat.h>

#include <linux/arm-cci.h>
#include <linux/mailbox_controller.h>
#include <linux/scb_mhu_api.h>
#include <linux/reboot.h>
#include <linux/cpu_pm.h>
#include <linux/slab.h>
#include <asm/cacheflush.h>

#include <linux/platform_data/mb86s70-iomap.h>

extern int skip_mhu;

static arch_spinlock_t mb86s70_pm_lock = __ARCH_SPIN_LOCK_UNLOCKED;

static atomic_t mb86s70_pm_use_count[4][2];

static bool use_retention_mode;

struct cmd_scb_version version_cmd;

#define MB86S70_WFICOLOR_VIRT (MB86S70_ISRAM_VIRT + WFI_COLOR_REG_OFFSET)

const struct cmd_scb_version *mb86s7x_get_scb_version(void)
{
	if (version_cmd.payload_size != sizeof(version_cmd))
		return NULL;

	return &version_cmd;
}
EXPORT_SYMBOL_GPL(mb86s7x_get_scb_version);

static void mb86s70_set_wficolor(unsigned clstr, unsigned cpu, unsigned clr)
{
	u8 val;

	if (clr & ~AT_WFI_COLOR_MASK)
		return;

	val = __raw_readb(MB86S70_WFICOLOR_VIRT + clstr * 2 + cpu);
	val &= ~AT_WFI_COLOR_MASK;
	val |= clr;
	__raw_writeb(val, MB86S70_WFICOLOR_VIRT + clstr * 2 + cpu);
}

static int mb86s70_pm_power_up(unsigned int cpu, unsigned int cluster)
{
	struct cmd_cpu_control_gate cmd;
	struct completion got_rsp;
	int ret = 0;

	local_irq_disable();
	arch_spin_lock(&mb86s70_pm_lock);
	switch (atomic_inc_return(&mb86s70_pm_use_count[cpu][cluster])) {
	case 1:
		cmd.payload_size = sizeof(cmd);
		cmd.cluster_class = 0;
		cmd.cluster_id = cluster;
		cmd.cpu_id = cpu;
		cmd.cpu_state = SCB_CPU_STATE_ON;

		mbox_dbg("%s:%d CMD Pyld-%u Cl_Class-%u CL_ID-%u CPU_ID-%u STATE-%u}\n",
			__func__, __LINE__, cmd.payload_size, cmd.cluster_class,
			cmd.cluster_id,	cmd.cpu_id, cmd.cpu_state);

		init_completion(&got_rsp);
		if (skip_mhu) {
			ret = 0;
		} else {
			mb86s70_set_wficolor(cluster, cpu, AT_WFI_DO_NOTHING);
			arch_spin_unlock(&mb86s70_pm_lock);
			local_irq_enable();
			ret = mhu_send_packet(CMD_CPU_CLOCK_GATE_SET_REQ,
					&cmd, sizeof(cmd), &got_rsp);
		}
		if (ret < 0) {
			pr_err("%s:%d failed!\n", __func__, __LINE__);
			return ret;
		}

		if (ret)
			wait_for_completion(&got_rsp);

		mbox_dbg("%s:%d REP Pyld-%u Cl_Class-%u CL_ID-%u CPU_ID-%u STATE-%u}\n",
			__func__, __LINE__, cmd.payload_size, cmd.cluster_class,
			cmd.cluster_id,	cmd.cpu_id, cmd.cpu_state);
		break;
	case 2:
		/* This power up request has overtaken a power down request */
		arch_spin_unlock(&mb86s70_pm_lock);
		local_irq_enable();
		break;
	default:
		/*
		 * The only possible values are:
		 * 0 = CPU down
		 * 1 = CPU (still) up
		 * 2 = CPU requested to be up before it had a chance
		 *     to actually make itself down.
		 * Any other value is a bug.
		 */
		BUG();
	}

	return 0;
}

static void mb86s70_pm_suspend(u64 suspend)
{
	unsigned int mpidr, cpu, cluster;
	bool last_man = false, skip_wfi = false;

	mpidr = read_cpuid_mpidr();
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);

	__mcpm_cpu_going_down(cpu, cluster);

	BUG_ON(!irqs_disabled());
	arch_spin_lock(&mb86s70_pm_lock);
	BUG_ON(__mcpm_cluster_state(cluster) != CLUSTER_UP);

	switch (atomic_dec_return(&mb86s70_pm_use_count[cpu][cluster])) {
	case 1:
		/*
		 * Overtaken by a power up. Flush caches, exit coherency,
		 * return & fake a reset
		 */
		skip_wfi = true;
		break;
	case 0:
		/* If all other cores in the cluster were already down
		 * AND this cpu is going to be down for long enough
		 * then prepare for cluster power down.
		 */
		if (!atomic_read(&mb86s70_pm_use_count[0][cluster]) &&
		    !atomic_read(&mb86s70_pm_use_count[1][cluster]) &&
		    !atomic_read(&mb86s70_pm_use_count[2][cluster]) &&
		    !atomic_read(&mb86s70_pm_use_count[3][cluster]))
			last_man = true;
		break;
	default:
		BUG();
	}

	if (!skip_wfi)
		gic_cpu_if_down();

	if (last_man && __mcpm_outbound_enter_critical(cpu, cluster)) {

		arch_spin_unlock(&mb86s70_pm_lock);

		v7_exit_coherency_flush(all);

		cci_disable_port_by_cpu(mpidr);

		__mcpm_outbound_leave_critical(cluster, CLUSTER_DOWN);
	} else {
		arch_spin_unlock(&mb86s70_pm_lock);
		v7_exit_coherency_flush(louis);
	}

	__mcpm_cpu_down(cpu, cluster);

	/* Now we are prepared for power-down, do it: */
	if (!skip_wfi) {

		mb86s70_set_wficolor(cluster, cpu, AT_WFI_DO_POWEROFF);
		while (1) {
			wfi();
		}
	}

	/* Not dead at this point?  Let our caller cope. */
}

static void mb86s70_pm_power_down(void)
{
	mb86s70_pm_suspend(0);
}

static const struct mcpm_platform_ops mb86s70_pm_power_ops = {
	.power_up	= mb86s70_pm_power_up,
	.power_down	= mb86s70_pm_power_down,
	.suspend	= mb86s70_pm_suspend,
};

extern void cci_port_control(unsigned int port, int enable);
extern void mb86s70_pm_power_up_setup(unsigned int affinity_level);
extern void mb86s70_reboot(u32 delay);

static int mb86s70_die(unsigned long arg);

/*
 * Wait until all other cpus than myself shut-down using MHU call.
 *
 * Must be called with irqs disabled.
 */
static void mb86s70_wait_nonboot_cpus_off(void)
{

	int cpu;
	int this_cpu = smp_processor_id();

	BUG_ON(! irqs_disabled());

	for_each_present_cpu(cpu) {

		unsigned int mpidr, cpu_id, cluster_id;

		unsigned long timeout = 100;

		if (cpu == this_cpu)
			continue;

		mpidr = cpu_logical_map(cpu);
		cluster_id = MPIDR_AFFINITY_LEVEL(mpidr, 1);;
		cpu_id = MPIDR_AFFINITY_LEVEL(mpidr, 0);

		while (timeout-- > 0) {
			struct cmd_cpu_control_gate cmd;
			struct completion got_rsp;
			int ret;

			cmd.payload_size = sizeof(cmd);
			cmd.cluster_class = 0;
			cmd.cluster_id = cluster_id;
			cmd.cpu_id = cpu_id;
		
			init_completion(&got_rsp);
	
			ret = mhu_send_packet(CMD_CPU_CLOCK_GATE_GET_REQ,
								  &cmd, sizeof(cmd), &got_rsp);
	
			if (ret < 0) {
				pr_err("%s:%d failed!\n", __func__, __LINE__);
				break;
			}

			if (ret)
				wait_for_completion(&got_rsp);
	
			if (cmd.cpu_state == SCB_CPU_STATE_OFF) {
				break;
			}

			mdelay(10);
		}

		if (timeout == 0) {
			pr_err("%s:%d failed!\n", __func__, __LINE__);
		}

	}

}

void mb86s70_restart(char mode, const char *unused)
{

	pr_err("%s\n", __func__);

	BUG_ON(irqs_disabled());

	local_irq_disable();

	mb86s70_wait_nonboot_cpus_off();

	/* Reboot immediately */
	mb86s70_reboot(50);

	mb86s70_die(0);
}

static void mb86s70_poweroff(void)
{

	pr_err("%s\n", __func__);

	BUG_ON(irqs_disabled());

	local_irq_disable();

	mb86s70_wait_nonboot_cpus_off();

	/* Reboot never, remain dead */
	mb86s70_reboot(~0);

	mb86s70_die(0);
}

static int mb86s70_pm_valid(suspend_state_t state)
{
	return (state == PM_SUSPEND_STANDBY) || (state == PM_SUSPEND_MEM);
}

extern void mb86s70_primary_reboot(void);
typedef void (*phys_reset_t)(unsigned long);
static int mb86s70_die(unsigned long arg)
{
	/* MCPM works with HW CPU identifiers */
	unsigned int mpidr = read_cpuid_mpidr();
	unsigned int cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);
	unsigned int cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	phys_reset_t phys_reset;

	BUG_ON(!irqs_disabled());

	mcpm_set_entry_vector(cpu, cluster, cpu_resume);

	setup_mm_for_reboot();

	if (use_retention_mode) {

		__mcpm_cpu_going_down(cpu, cluster);
		arch_spin_lock(&mb86s70_pm_lock);
		gic_cpu_if_down();
		__mcpm_outbound_enter_critical(cpu, cluster);
		arch_spin_unlock(&mb86s70_pm_lock);

		v7_exit_coherency_flush(all);
		cci_disable_port_by_cpu(mpidr);
		__mcpm_outbound_leave_critical(cluster, CLUSTER_DOWN);
		__mcpm_cpu_down(cpu, cluster);

		/* Move WFI-color setting just before calling wfi() as a workaround for SCB firmware bug. */
		mb86s70_set_wficolor(cluster, cpu, AT_WFI_DO_SUSPEND);

		/* Return only to mb86s70_primary_reboot() via reset */
		while (1)
			asm("wfi");
	}

	asm("wfi");
	/* Boot just like a secondary */
	phys_reset = (phys_reset_t)(unsigned long)virt_to_phys(cpu_reset);
	phys_reset(virt_to_phys(mcpm_entry_point));

	return 0;
}

void set_secondary_entry(unsigned long secondary_entry);

static void force_mvbar(void)
{
	struct cmd_scb_version capabitity_cmd;
	struct completion got_rsp;
	int ret;

	capabitity_cmd.payload_size = sizeof(capabitity_cmd);

	init_completion(&got_rsp);
	
	ret = mhu_send_packet(CMD_SCB_CAPABILITY_GET_REQ,
				   &capabitity_cmd, sizeof(capabitity_cmd), &got_rsp);
	if (ret < 0)
		pr_err("%s:%d failed to get SCB version\n",
						__func__, __LINE__);
	if (ret)
		wait_for_completion(&got_rsp);

	/* force the MVBAR to VA of secure monitor vector */
	if (capabitity_cmd.capabilities[0] & (1 << 2)) {
		asm("ldr r0,	=0xfe100200\n" \
				"mcr	p15, 0, r0, c12, c0, 1\n"
				);
	}
}

static int mb86s70_pm_enter(suspend_state_t state)
{
	unsigned int mpidr, cpu, cluster;
	struct cmd_resume_entry_point_msg resume_entry_cmd;
	struct completion got_rsp;
	int ret;
	mpidr = read_cpuid_mpidr();
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);

	mb86s70_wait_nonboot_cpus_off();

	switch (state) {
	case PM_SUSPEND_STANDBY:
		pr_err("STANDBY\n");
		asm("wfi");
		break;
	case PM_SUSPEND_MEM:
		pr_err("SUSPEND\n");
		if (use_retention_mode) {

			force_mvbar();
			set_secondary_entry(0);

			/* Set the target ads for primary after 'resume' from STR */
			resume_entry_cmd.payload_size = sizeof(resume_entry_cmd);
			resume_entry_cmd.resume_entry_point = virt_to_phys(mb86s70_primary_reboot);
			resume_entry_cmd.resume_flag = 1;
			
			init_completion(&got_rsp);
			
			ret = mhu_send_packet(CMD_RESUME_ENTRY_POINT_SET_REQ,
				   &resume_entry_cmd, sizeof(resume_entry_cmd), &got_rsp);

			if (ret < 0) {
				pr_err("%s:%d failed to set resume entry point, \
					please update scb firmware.\n",
								__func__, __LINE__);
				BUG();
			}
			if (ret)
				wait_for_completion(&got_rsp);

		}

		cpu_pm_enter();
		cpu_suspend(0, mb86s70_die);
		cpu_pm_exit();
		break;
	}
	return 0;
}

static const struct platform_suspend_ops mb86s70_pm_ops = {
	.valid		= mb86s70_pm_valid,
	.enter		= mb86s70_pm_enter,
};


static int mb86s70_shutdown(struct notifier_block *this,
							unsigned long code, void *x)
{

	int err;

	pr_err("%s\n", __func__);

	err = disable_nonboot_cpus();
	if (err) {
		pr_err("Failed to shut down nonboot cpus");
	}

	return NOTIFY_DONE;
}
	
static struct notifier_block mb86s70_reboot_notifier = {
	.notifier_call  = mb86s70_shutdown,
};

void set_secondary_entry(unsigned long secondary_entry);

static int __init mb86s70_mcpm_init(void)
{
	struct completion got_rsp;
	unsigned int mpidr, cpu, cluster;
	struct device_node *np;
	int ret;

	np = of_find_compatible_node(NULL, NULL, "arm,cci-400");
	if (!np)
		return 0;

	if (of_find_property(np, "retention-suspend", NULL))
		use_retention_mode = true;

	arm_pm_restart = mb86s70_restart;
	pm_power_off = mb86s70_poweroff;

	mpidr = read_cpuid_mpidr();
	cpu = MPIDR_AFFINITY_LEVEL(mpidr, 0);
	cluster = MPIDR_AFFINITY_LEVEL(mpidr, 1);
	atomic_set(&mb86s70_pm_use_count[cpu][cluster], 1);
	/* reset the wfi 'color' for primary cpu */
	mb86s70_set_wficolor(cluster, cpu, AT_WFI_DO_NOTHING);
	pr_info("Booting on cpu_%u cluster_%u\n", cpu, cluster);
	boot_cpu_color_offset = cluster * 2 + cpu;

	ret = mcpm_platform_register(&mb86s70_pm_power_ops);
	if (!ret)
		ret = mcpm_sync_init(mb86s70_pm_power_up_setup);
	if (!ret) {
		phys_cpu_reset = (u32) virt_to_phys(cpu_reset);
		phys_mcpm_entry_point = (u32) virt_to_phys(mcpm_entry_point);
		
		/* use smc to set secondary entry address */
		set_secondary_entry(phys_mcpm_entry_point);
		flush_cache_all();
		outer_flush_range(0, 16);
		smp_wmb();
                                             
		version_cmd.payload_size = sizeof(version_cmd);
		version_cmd.version = 0;
		version_cmd.config_version = 0;

		init_completion(&got_rsp);
		if (skip_mhu)
			ret = 0;
		else
			ret = mhu_send_packet(CMD_SCB_CAPABILITY_GET_REQ,
				   &version_cmd, sizeof(version_cmd), &got_rsp);
		if (ret < 0)
			pr_err("%s:%d failed to get SCB version\n",
							__func__, __LINE__);
		if (ret)
			wait_for_completion(&got_rsp);

		pr_info("MB86S7x MCPM initialized: SCB version 0x%x:0x%x\n",
			       version_cmd.version, version_cmd.config_version);
	}

	register_reboot_notifier(&mb86s70_reboot_notifier);

	return ret;
}
early_initcall(mb86s70_mcpm_init);

extern int exiu_irq_set_type(unsigned long irq_num, unsigned long type);

#if 0
static irqreturn_t s70_wakeup_handler(int irq, void *arg)
{
	return IRQ_HANDLED;
}
#endif

static int mb86s70_evbpm_probe(struct platform_device *pdev)
{
	struct pinctrl_state *pins_default;
	struct pinctrl *pinctrl;
	struct resource *res;
	int ret, irq;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	irq = res->start;

#if 0
	/* set button to wakeup */
	ret = exiu_irq_set_type(irq, IRQ_TYPE_EDGE_RISING);
	if (ret) {
		pr_err("%s:%d failed!\n", __func__, __LINE__);
		return ret;
	}

	ret = request_irq(irq, s70_wakeup_handler,
				IRQF_ONESHOT, "WKUPirq", NULL);
	if (ret) {
		pr_err("%s:%d failed!\n", __func__, __LINE__);
		return ret;
	}

	ret = irq_set_irq_wake(irq, 1);
	if (ret) {
		pr_err("%s:%d failed!\n", __func__, __LINE__);
		return ret;
	}
#endif
	pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		pr_err("%s:%d failed!\n", __func__, __LINE__);
		return PTR_ERR(pinctrl);
	}

	pins_default = pinctrl_lookup_state(pinctrl, PINCTRL_STATE_DEFAULT);
	if (IS_ERR(pins_default)) {
		pr_err("%s:%d failed!\n", __func__, __LINE__);
		return PTR_ERR(pins_default);
	}

	ret = pinctrl_select_state(pinctrl, pins_default);
	if (ret) {
		pr_err("%s:%d failed!\n", __func__, __LINE__);
		return ret;
	}

	return 0;
}

static const struct of_device_id mb86s70_evbpm_dt_ids[] = {
	{.compatible = "mb86s70evb,pm_ops"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mb86s70_evbpm_dt_ids);

static struct platform_driver mb86s70_evbpm_driver = {
	.probe     = mb86s70_evbpm_probe,
	.driver    = {
		.name  = "mb86s70evb_pm_ops",
		.owner = THIS_MODULE,
		.of_match_table = mb86s70_evbpm_dt_ids,
	},
};

static int __init mb86s70evb_pm_init(void)
{
	if (of_machine_is_compatible("fujitsu,mb8aa0350eb")) {
		static const struct platform_device_info devinfo = {
			.name = "s70-cpufreq",
		};
		platform_device_register_full(&devinfo);
		pr_err("Populated s70-cpufreq\n");
	}

	suspend_set_ops(&mb86s70_pm_ops);

	return platform_driver_register(&mb86s70_evbpm_driver);
}
late_initcall(mb86s70evb_pm_init);
