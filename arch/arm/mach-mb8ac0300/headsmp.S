/*
 * linux/arch/arm/mach-mb8ac0300/headsmp.S
 *
 * Copyright (C) 2013 FUJITSU SEMICONDUCTOR LIMITED
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 */

#include <linux/linkage.h>
#include <linux/init.h>

	__CPUINIT	/* .txt */

/*
 * MB8AC0300 entry point for secondary CPU to jump from ROM code.
 */
ENTRY(mb8ac0300_secondary_startup)
	mrc	p15, 0, r0, c0, c0, 5	@ Read Multiprocessor ID register
	and	r0, r0, #15		@ get Core id
	adr	r4, 1f
	ldmia	r4, {r5, r6}
	sub	r4, r4, r5
	add	r6, r6, r4
pen:	ldr	r7, [r6]		@ check the cpu ID
	cmp	r7, r0			@ if released cpu is myself
	bne	pen			@ release form pen

	b	secondary_startup
ENDPROC(mb8ac0300_secondary_startup)

	.align
1:	.long	.
	.long	pen_release
