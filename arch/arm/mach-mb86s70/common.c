/*
 * linux/arch/arm/mach-mb86s70/common.c
 *
 * Copyright (C) 2013 Linaro, LTD
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

#include <linux/init.h>
#include <linux/irqchip.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include "devices.h"

static const char * const mb86s70_dt_match[] __initconst = {
	"fujitsu,mb86s70",
	"fujitsu,mb86s73",
	"fujitsu,mb86s72",
	"fujitsu,mb86s71",
	NULL,
};

DT_MACHINE_START(MB86S70_DT, "Fujitsu MB86S70-based board")
	.dt_compat	= mb86s70_dt_match,
	.smp_init	= smp_init_ops(mb86s70_smp_init_ops),
	.map_io		= mb86s70_dt_map_io,
	.init_irq	= irqchip_init,
	.init_time	= mb86s70_dt_timer_init,
	.init_machine	= mb86s70_dt_init,
MACHINE_END
