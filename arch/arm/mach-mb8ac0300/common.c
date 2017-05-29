/*
 * linux/arch/arm/mach-mb8ac0300/common.c
 *
 * Copyright (C) 2013 Linaro, LTD
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <linux/clocksource.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/irqchip.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/smp_scu.h>
#include <asm/smp_twd.h>
#include <asm/mach/map.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/cacheflush.h>
#include <asm/mach-types.h>
#include <asm/hardware/timer-sp.h>
#include <asm/hardware/arm_timer.h>
#include <asm/hardware/cache-l2x0.h>

#include <linux/platform_data/mb8ac0300-iomap.h>

#include "devices.h"

#define MB8AC0300_SYSOC_PHYS		0xfff70000
#define MB8AC0300_SYSOC_SIZE		SZ_4K

#define SYSOC_REG_AUXCTL		0x044
#define SYSOC_AUXCTL_HSSPI_HSEL		(0x1 << 9)

void __init mb8ac0300_init_timer(void)
{
	of_clk_init(NULL);
	clocksource_of_init();
}

void __init mb8ac0300_init(void)
{
	unsigned int val;
	void __iomem *sysoc =
			ioremap(MB8AC0300_SYSOC_PHYS, MB8AC0300_SYSOC_SIZE);

	/*
	 * l2x0: 16 ways, CACHE_ID 0x000000c0, AUX_CTRL 0x72450000
	 * Cache size: 524288 B
	 */
	l2x0_of_init((1 << L2X0_AUX_CTRL_EARLY_BRESP_SHIFT) |
		    (1 << L2X0_AUX_CTRL_DATA_PREFETCH_SHIFT) |
		    (1 << L2X0_AUX_CTRL_INSTR_PREFETCH_SHIFT) |
		    (1 << L2X0_AUX_CTRL_SHARE_OVERRIDE_SHIFT), 0xFFFFFFFF);
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);

	/* Init sysoc for HS_SPI
	 * After power HSSPIn_CSCFG.BOOTEN bit is 1,AUXCTL.HSEL bit is 0.
	 * At this setting only command sequencer mode can be used.
	 * We want to use both of command sequencer mode and direct mode,
	 * so we set:
	 *   HSSPIn_CSCFG.BOOTEN: 0
	 *   AUXCTL.HSEL        : 1
	 * BOOTEN bit will set in HSSPI driver, so we only set AUXCTL.HSEL here.
	 */
	val = readl(sysoc + SYSOC_REG_AUXCTL);
	val |= SYSOC_AUXCTL_HSSPI_HSEL;
	writel(val, sysoc + SYSOC_REG_AUXCTL);

	iounmap(sysoc);

#if defined(CONFIG_PM)
	mb8ac0300_pm_init();
#endif
}

static const char *mb8ac0300_dt_compat[] __initdata = {
	"fujitsu,mb8ac0300",
	"fujitsu,mb8ac0300eb",
	NULL
};

extern struct smp_operations mb8ac0300_smp_ops;

DT_MACHINE_START(MB8AC0300_DT, "Fujitsu Semiconductor MB8AC0300-EVB")
	.smp		= smp_ops(mb8ac0300_smp_ops),
	.map_io         = mb8ac0300_map_io,
	.init_time	= mb8ac0300_init_timer,
	.init_machine	= mb8ac0300_init,
	.dt_compat	= mb8ac0300_dt_compat,
MACHINE_END
