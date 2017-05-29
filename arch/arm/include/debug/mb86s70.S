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

#define UART1_PHYS		0x31040000
#define UART1_VIRT		0xfe000000
#define UART1_FLAGR_OFFSET	(5 << 2)
#define UART1_TXFF		0x20

		.macro	addruart, rp, rv, tmp
		ldr	\rp, =UART1_PHYS	@ physical
		ldr	\rv, =UART1_VIRT	@ virtual
		.endm

		.macro	senduart,rd,rx
		str	\rd, [\rx, #0]	@ TXDATA
		.endm

		.macro	waituart,rd,rx
		.endm

		.macro	busyuart,rd,rx
1002:		ldr	\rd, [\rx, #UART1_FLAGR_OFFSET]	@ get status register
		tst	\rd, #UART1_TXFF		@
		beq	1002b			@ wait if FIFO is full
		.endm
