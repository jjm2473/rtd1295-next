/*
 * Spansion FM4 machine
 *
 * Copyright (c) 2015 Andreas Färber
 */

#include <asm/mach/arch.h>
#include <asm/v7m.h>
#include <linux/kernel.h>

static const char *const fm4_compat[] __initconst = {
	"cypress,fm4",
	NULL
};

DT_MACHINE_START(FM4DT, "FM4 (Device Tree Support)")
	.dt_compat = fm4_compat,
	.restart = armv7m_restart,
MACHINE_END
