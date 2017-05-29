/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/linkage.h>

#include <asm/memory.h>
#include <asm/assembler.h>
#include <asm/asm-offsets.h>
#include <asm/cache.h>
#include <linux/platform_data/mb86s70-iomap.h>

/* From scb_mhu_api.h */
#define AT_WFI_DO_NOTHING	0x0
#define AT_WFI_DO_SUSPEND	0x1
#define AT_WFI_DO_POWEROFF	0x2
#define AT_WFI_COLOR_MASK	0x3
#define WFI_COLOR_REG_OFFSET	0x3f00

#define MB86S70_WFICOLOR_PHYS (MB86S70_ISRAM_PHYS + WFI_COLOR_REG_OFFSET)
#define PHYS_TRMP_ADDR	(MB86S70_TRAMPOLINE_PHYS + SEC_RSTADDR_OFF)
#define OFFSET_FIX	(PAGE_OFFSET - 0x80000000 /* PHYS_OFFSET */)
.arch_extension sec

ENTRY(mb86s70_primary_reboot)
	ldr	r2, =OFFSET_FIX
	/* Restore reset-address for the secondaries */
	ldr	r0, =phys_mcpm_entry_point
	sub	r0, r0, r2
	ldr	r0, [r0]
	
	mov r1, r0
	ldr r0, =1
	mrc p15, 0, r3, c1, c0, 0
	mov r4, r3
	and r3, #0xbfffffff
	mcr p15, 0, r3, c1, c0, 0
	smc #0
	mcr p15, 0, r4, c1, c0, 0

	/* Reset wficolor for the secondaries */
	ldr	r0, =boot_cpu_color_offset
	sub	r0, r0, r2
	ldr	r1, [r0]
	ldr	r0, =MB86S70_WFICOLOR_PHYS
	add	r0, r1

	ldrb	r1, [r0]
	bic	r1, r1, #AT_WFI_COLOR_MASK
	orr	r1, r1, #AT_WFI_DO_NOTHING
	strb	r1, [r0]

	/* Do cpu_reset(mcpm_entry_point) */
	ldr	r0, =phys_mcpm_entry_point
	sub	r0, r0, r2
	ldr	r0, [r0]
	ldr	r1, =phys_cpu_reset
	sub	r1, r1, r2
	ldr	r1, [r1]
	mov	pc, r1

ENDPROC(mb86s70_primary_reboot)

        .align  4
	.globl phys_mcpm_entry_point
phys_mcpm_entry_point:
	.space	4
	.globl phys_cpu_reset
phys_cpu_reset:
	.space	4
	.globl boot_cpu_color_offset
boot_cpu_color_offset:
	.space	4
