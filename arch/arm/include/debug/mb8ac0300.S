/*
 * Debugging macro include header
 *
 *  Copyright (C) 1994-1999 Russell King
 *  Moved from linux/arch/arm/kernel/debug.S by Ben Dooks
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#define UART0_PHYS		0xfff6b000
#define UART0_VIRT		0xfe06b000
#define UART0_LSR_OFFSET	(5 << 2)

		.macro	addruart, rp, rv, tmp
		ldr	\rp, =UART0_PHYS	@ physical
		ldr	\rv, =UART0_VIRT	@ virtual
		.endm

		.macro	senduart,rd,rx
		str	\rd, [\rx, #0]	@ TXDATA
		.endm

		.macro	waituart,rd,rx
		.endm

		.macro	busyuart,rd,rx
1002:		ldr	\rd, [\rx, #UART0_LSR_OFFSET]	@ get status register
		tst	\rd, #0x20		@
		beq	1002b			@ wait if FIFO is full
		.endm
