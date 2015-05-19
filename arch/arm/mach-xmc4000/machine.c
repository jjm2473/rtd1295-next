/*
 * Infineon XMC4000 machine
 *
 * Copyright (c) 2015 Andreas Färber
 */

#include <asm/mach/arch.h>
#include <asm/v7m.h>
#include <linux/kernel.h>

static const char *const xmc4000_compat[] __initconst = {
	"infineon,xmc4500",
	NULL
};

DT_MACHINE_START(XMC4000DT, "XMC4000 (Device Tree Support)")
	.dt_compat = xmc4000_compat,
	.restart = armv7m_restart,
MACHINE_END
