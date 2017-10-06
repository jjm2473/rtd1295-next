#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/smp.h>
#include <asm/cacheflush.h>
#include <asm/smp_plat.h>

#define REG_A7_WRAP	0x100

#define A7_WRAP_NCORERESET	GENMASK(5, 4)

static void __iomem *rtd119x_scpu_iomap(void)
{
	struct device_node *node;
	void __iomem *base;

	node = of_find_compatible_node(NULL, NULL, "realtek,rtd1195-scpu-wrapper");
	if (!node) {
		pr_err("%s: missing SCPU wrapper DT node\n", __func__);
		return NULL;
	}

	base = of_iomap(node, 0);
	of_node_put(node);
	return base;
}

static void rtd119x_scpu_core_reset_assert(unsigned int cpu)
{
	void __iomem *base;
	u32 val;

	if (cpu >= 2)
		return;

	base = rtd119x_scpu_iomap();
	if (!base) {
		pr_err("%s: could not map SCPU wrapper registers\n", __func__);
		return;
	}

	val = readl(base + REG_A7_WRAP);
	val &= ~FIELD_PREP(A7_WRAP_NCORERESET, BIT(cpu));
	writel(val, base + REG_A7_WRAP);

	iounmap(base);
}

static void rtd119x_scpu_core_reset_deassert(unsigned int cpu)
{
	void __iomem *base;
	u32 val;

	if (cpu >= 2)
		return;

	base = rtd119x_scpu_iomap();
	if (!base) {
		pr_err("%s: could not map SCPU wrapper registers\n", __func__);
		return;
	}

	val = readl(base + REG_A7_WRAP);
	val |= FIELD_PREP(A7_WRAP_NCORERESET, BIT(cpu));
	writel(val, base + REG_A7_WRAP);

	iounmap(base);
}

static int rtd119x_smp_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	unsigned long phys_cpu = cpu_logical_map(cpu);

	rtd119x_scpu_core_reset_deassert(phys_cpu);

	arch_send_wakeup_ipi_mask(cpumask_of(cpu));

	return 0;
}

static void __init rtd119x_smp_prepare_cpus(unsigned int max_cpus)
{
	struct device_node *node;
	void __iomem *base;

	rtd119x_scpu_core_reset_assert(1);

	node = of_find_compatible_node(NULL, NULL, "realtek,rtd1195-iso-irq-mux");
	if (!node) {
		pr_err("%s: missing iso irq mux\n", __func__);
		return;
	}

	base = of_iomap(node, 0);
	of_node_put(node);
	if (!base) {
		pr_err("%s: could not map iso irq mux registers\n", __func__);
		return;
	}

	writel(__pa_symbol(secondary_startup), base + 0x064);

	iounmap(base);
}

static const struct smp_operations rtd119x_smp_ops __initconst = {
	.smp_prepare_cpus	= rtd119x_smp_prepare_cpus,
	.smp_boot_secondary	= rtd119x_smp_boot_secondary,
};
CPU_METHOD_OF_DECLARE(rtd1195_smp, "realtek,rtd1195-smp", &rtd119x_smp_ops);
