/*
 * linux/arch/arm/mach-mb8ac0300/hibernate_asm.S
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

#include <linux/linkage.h>
#include <asm/ptrace.h>
#include <asm/cache.h>
#include <asm/assembler.h>
#include <asm/asm-offsets.h>

	.text
ENTRY(swsusp_arch_suspend)
	/*
	 * Save current program status register
	 */
	ldr     r3, .Lsaved_cpsr
	mrs     r0, cpsr
	str     r0, [r3]

	/*
	 * Mask cpsr mode
	 */
	mov     r1, r0
	mvn     r2, #MODE_MASK
	and	r1, r2

	/*
	 * Change to system(user) mode
	 */
	orr	r3, r1, #SYSTEM_MODE
	msr     cpsr_c, r3

	/*
	 * Save User context
	 */
	ldr     r3, .Lsaved_context_usr
	stmia   r3, {r4-r11, r13, r14}

	/*
	 * Change to SVC mode
	 */
	orr     r3, r1, #SVC_MODE
	msr     cpsr_c, r3

	/*
	 * Save SVC context
	 */
	ldr	r3, .Lsaved_context_svc
	stmia	r3, {r13 - r14}
	ldr	r3, .Lsaved_spsr_svc
	mrs	r2, spsr
	str	r2, [r3]

	/*
	 * restore original mode before suspending.
	 */
	msr	cpsr_c, r0
	b	swsusp_save
ENDPROC(swsusp_arch_suspend)

#define CP_COUNT (PAGE_SZ / (2 * L1_CACHE_BYTES) PLD( -1 ))

ENTRY(swsusp_arch_resume)
	/*
	 * The following code is an assembly version of:
	 *	struct pbe *pbe;
	 *	for (pbe = restore_pblist; pbe != NULL; pbe = pbe->next)
	 *		copy_page(pbe->orig_address, pbe->address);
	 */

	/*
	 * Restore_pblist is the starting point for loaded pages
	 */
	ldr	r0, .Lrestore_pblist
	ldr	r6, [r0]
pbe_copy_loop:
	ldr     r0, [r6, #4]	/* dst address (pbe->orig_address) */
	ldr	r1, [r6]	/* src address (pbe->address) */

	/* copy from arm/lib/copy_page.S */
	PLD(	pld	[r1, #0]			)
	PLD(	pld	[r1, #L1_CACHE_BYTES]		)
	mov	r2, #CP_COUNT
	ldmia	r1!, {r3, r4, ip, lr}
1:	PLD(	pld	[r1, #2 * L1_CACHE_BYTES]	)
	PLD(	pld	[r1, #3 * L1_CACHE_BYTES]	)
2:
	.rept   (2 * L1_CACHE_BYTES / 16 - 1)
		stmia	r0!, {r3, r4, ip, lr}
		ldmia	r1!, {r3, r4, ip, lr}
	.endr
	subs	r2, r2, #1
	stmia	r0!, {r3, r4, ip, lr}
	ldmgtia	r1!, {r3, r4, ip, lr}
	bgt	1b
	PLD(	ldmeqia r1!, {r3, r4, ip, lr}		)
	PLD(	beq	2b				)

	/* The last field of struct pbe is a pointer to the next pbe struct */
	ldr	r6, [r6, #8]	/* pbe->next */
	cmp	r6, #0
	bne	pbe_copy_loop

	/*
	 * Flush TLB (Invalidate unified TLB unlocked entries)
	 */
	mov	r1, #0
	mcr	p15, 0, r1, c8, c7, 0

	/*
	 * Restore CPU registers.
	 */
	mrs	r0, cpsr
	mov	r1, r0
	mvn	r2, #MODE_MASK
	and	r1, r2

	/*
	 * Change to system(user) mode
	 */
	orr	r3, r1, #SYSTEM_MODE
	msr	cpsr_c, r3

	/*
	 * Restore user mode context
	 */
	ldr	r3, .Lsaved_context_usr
	ldmia	r3, {r4-r11, r13, r14}
	ldr	r3, .Lsaved_cpsr
	ldr	r2, [r3]
	msr	cpsr_c, r2

	/*
	 * Change to SVC mode
	 */
	orr	r3, r1, #SVC_MODE
	msr	cpsr_c, r3

	/*
	 * Restore SVC context
	 */
	ldr	r3, .Lsaved_context_svc
	ldmia	r3, {r13-r14}
	ldr	r3, .Lsaved_spsr_svc
	ldr	r1, [r3]
	msr	spsr_cxsf, r1

	/* Set the return value */
	mov	r0, #0
	mov     pc, lr
ENDPROC(swsusp_arch_resume)
	.align	4
.Lsaved_context_usr:		.long	saved_context_usr
.Lsaved_context_svc:		.long	saved_context_svc
.Lsaved_cpsr:			.long	saved_cpsr
.Lsaved_spsr_svc:		.long   saved_spsr_svc
.Lrestore_pblist:		.long	restore_pblist
