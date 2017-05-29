#include <linux/device.h>
#include <linux/io.h>
#include <linux/smp.h>
#include <linux/init.h>
#include <linux/memblock.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/mtd/physmap.h>
#include <linux/clk-provider.h>
#include <linux/clocksource.h>
#include <linux/irqflags.h>

#include <linux/platform_data/mb86s70-iomap.h>

#include <asm/arch_timer.h>
#include <asm/mach-types.h>
#include <asm/sizes.h>
#include <asm/smp_twd.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/hardware/arm_timer.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/hardware/timer-sp.h>
#include <asm/mcpm.h>

#include "devices.h"

bool __init mb86s70_smp_init_ops(void)
{
	if (of_find_compatible_node(NULL, NULL, "arm,cci-400")) {
		mcpm_smp_set_ops();
		return true;
	}

	pr_err("%s:%d CCI DT node not found\n", __func__, __LINE__);
	return false;
}

#define IOMAP_DEV(name) { \
		.virtual = (unsigned long) MB86S70_##name##_VIRT, \
		.pfn = __phys_to_pfn(MB86S70_##name##_PHYS), \
		.length = MB86S70_##name##_SIZE, \
		.type = MT_DEVICE, \
	}
#define IOMAP_SO(name) { \
		.virtual = (unsigned long) MB86S70_##name##_VIRT, \
		.pfn = __phys_to_pfn(MB86S70_##name##_PHYS), \
		.length = MB86S70_##name##_SIZE, \
		.type = MT_MEMORY_SO, \
	}
#define IOMAP_MEM(name) { \
		.virtual = (unsigned long) MB86S70_##name##_VIRT, \
		.pfn = __phys_to_pfn(MB86S70_##name##_PHYS), \
		.length = MB86S70_##name##_SIZE, \
		.type = MT_MEMORY, \
	}

static struct map_desc mb86s70_io_desc[] = {
	IOMAP_DEV(UART0),
	IOMAP_DEV(UART1),
	IOMAP_DEV(MHU),
	IOMAP_SO(ISRAM),
	IOMAP_MEM(SSRAM),
};

void __init mb86s70_dt_map_io(void)
{
	iotable_init(mb86s70_io_desc, ARRAY_SIZE(mb86s70_io_desc));
}

void __init mb86s70_dt_timer_init(void)
{
	if (!irqs_disabled())
		printk("%s:%d\n", __func__, __LINE__);
	of_clk_init(NULL);
	if (!irqs_disabled())
		printk("%s:%d\n", __func__, __LINE__);
	clocksource_of_init();
	if (!irqs_disabled())
		printk("%s:%d\n", __func__, __LINE__);
/*	twd_local_timer_of_register(); */
}

//static struct platform_device *bL_cpufreq;

void __init mb86s70_dt_init(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);

	{
		struct platform_device *bL_cpufreq;
		bL_cpufreq = platform_device_alloc("arm-bL-cpufreq-dt", -1);
		platform_device_add(bL_cpufreq);
	}

}

