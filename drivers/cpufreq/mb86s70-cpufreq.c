/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/cpufreq.h>
#include <linux/sched.h>
#include <linux/cpu.h>
#include <linux/smp.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>

#define CLSTR_CA15	0
#define CLSTR_CA7	1

static unsigned int cur_freq[4] = {1600000, 1600000, 800000, 800000};

static struct cpufreq_frequency_table freq_table_ca15[] = {
	{ 0, 1200000 },
	{ 1, 1600000 },
	{ 2, CPUFREQ_TABLE_END },
};

static struct cpufreq_frequency_table freq_table_ca7[] = {
	{ 0, 400000 },
	{ 1, 800000 },
	{ 2, CPUFREQ_TABLE_END },
};

extern void mhu_cluster_rate(int cluster, unsigned long *rate, int get);

static int s70_set_target(struct cpufreq_policy *policy,
		unsigned int target_freq, unsigned int relation)
{
	int cluster, ca15 = 0, cpu = policy->cpu;
	struct cpufreq_frequency_table *table;
	struct cpufreq_freqs freqs;
	cpumask_t cpus_allowed;
	unsigned long rate;
	unsigned int idx;

	if (cpu < 2) {
		table = &freq_table_ca15[0];
		cluster = CLSTR_CA15;
	} else {
		table = &freq_table_ca7[0];
		cluster = CLSTR_CA7;
	}

	freqs.old = cur_freq[cpu];
	cpufreq_frequency_table_target(policy, table, target_freq,
					relation, &idx);
	freqs.new = table[idx].frequency;

	/* Return if nothing to do */
	if (freqs.old == freqs.new)
		return 0;

	/* Save the CPUs allowed mask */
	cpus_allowed = current->cpus_allowed;

	/* Make sure we are running on CA7 if we want to change CA15 */
	if (cluster == CLSTR_CA15) {
		cpumask_t cpus_ca7;
		cpumask_clear(&cpus_ca7);
		cpumask_set_cpu(2, &cpus_ca7);
		cpumask_set_cpu(3, &cpus_ca7);

		/* Migrate to CA7 */
		set_cpus_allowed_ptr(current, &cpus_ca7);
		BUG_ON(smp_processor_id() < 2); /* Still on CA15?! */

		/* Put CA15 down */
		if (cpu_online(1)) {
			ca15 |= (1 << 1);
			cpu_down(1);
		}
		if (cpu_online(0)) {
			ca15 |= (1 << 0);
			cpu_down(0);
		}
	}

	cpufreq_notify_transition(policy, &freqs, CPUFREQ_PRECHANGE);

	rate = freqs.new * 1000;
	mhu_cluster_rate(cluster, &rate, 0);

	pr_err("%s:%d Cluster=%d Rate %lu/%u\n",
		__func__, __LINE__, cluster, rate, freqs.new * 1000);

	if (cluster == CLSTR_CA15) {
		cur_freq[0] = rate;
		cur_freq[1] = rate;
	} else {
		cur_freq[2] = rate;
		cur_freq[3] = rate;
	}

	cpufreq_notify_transition(policy, &freqs, CPUFREQ_POSTCHANGE);

	/* Put CA15 down */
	if (ca15 & (1 << 0))
		cpu_up(0);
	if (ca15 & (1 << 1))
		cpu_up(1);

	/* Restore the CPUs allowed mask */
	set_cpus_allowed_ptr(current, &cpus_allowed);

	return 0;
}

static unsigned int s70_get(unsigned int cpu)
{
	unsigned long rate;
	int cluster;

	if (cpu < 2)
		cluster = CLSTR_CA15;
	else
		cluster = CLSTR_CA7;

	mhu_cluster_rate(cluster, &rate, 1);

	pr_err("%s:%d cpu=%d freq=%lu\n",
			__func__, __LINE__, cpu, rate);
	return rate / 1000;
}

static int s70_cpufreq_init(struct cpufreq_policy *policy)
{
	struct cpufreq_frequency_table *table;
	unsigned int cpu = policy->cpu;

	pr_err("%s:%d cpu=%d\n", __func__, __LINE__, cpu);

	if (cpu < 2) {
		table = &freq_table_ca15[0];
		cpumask_set_cpu(0, policy->cpus);
		cpumask_set_cpu(1, policy->cpus);
	} else {
		table = &freq_table_ca7[0];
		cpumask_set_cpu(2, policy->cpus);
		cpumask_set_cpu(3, policy->cpus);
	}

	cpufreq_frequency_table_cpuinfo(policy, table);
	cpufreq_frequency_table_get_attr(table, policy->cpu);
	policy->cpuinfo.transition_latency = 500000;

	return 0;
}

static int s70_verify_speed(struct cpufreq_policy *policy)
{
	struct cpufreq_frequency_table *table;
	unsigned int cpu = policy->cpu;

	if (cpu < 2)
		table = &freq_table_ca15[0];
	else
		table = &freq_table_ca7[0];

	return cpufreq_frequency_table_verify(policy, table);
}

static struct freq_attr *s70_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver s70_driver = {
	.verify		= s70_verify_speed,
	.target		= s70_set_target,
	.get		= s70_get,
	.init		= s70_cpufreq_init,
	.name		= "mb86s70-cpufreq",
	.attr		= s70_cpufreq_attr,
};

static int s70_cpufreq_probe(struct platform_device *pdev)
{
	return cpufreq_register_driver(&s70_driver);
}

static int s70_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&s70_driver);

	return 0;
}

static struct platform_driver s70_cpufreq_driver = {
	.driver = {
		.name = "s70-cpufreq",
		.owner = THIS_MODULE,
	},
	.probe = s70_cpufreq_probe,
	.remove = s70_cpufreq_remove,
};
module_platform_driver(s70_cpufreq_driver);

MODULE_AUTHOR("Jassi");
MODULE_DESCRIPTION("cpufreq driver for Fujitsu S70");
MODULE_LICENSE("GPL");
