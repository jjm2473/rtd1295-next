/*
 * linux/arch/arm/mach-mb8ac0300/hibernate.c
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

#include <linux/module.h>
#include <linux/mm.h>

#include "devices.h"

/* Used in hibernate_asm.S */
unsigned long saved_context_usr[10];	/* user r3 - r11, r13, r14 */
unsigned long saved_context_svc[2];	/* SVC r13, r14 */
unsigned long saved_cpsr;
unsigned long saved_spsr_svc;


/**
 * pfn_is_nosave - check if given pfn is in the 'nosave' section
 */
int pfn_is_nosave(unsigned long pfn)
{
	unsigned long nosave_begin_pfn = __phys_to_pfn(__pa(&__nosave_begin));
	unsigned long nosave_end_pfn = __phys_to_pfn(__pa(&__nosave_end));

	return (pfn >= nosave_begin_pfn) && (pfn < nosave_end_pfn);
}

void save_processor_state(void)
{
}

void restore_processor_state(void)
{
}
