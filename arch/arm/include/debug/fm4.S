/*
 * FM4
 *
 * Copyright (c) 2015 Andreas Färber
 */

#define FM4_MFS_SSR	0x04
#define FM4_MFS_SSR_TDRE	(1 << 9)

#define FM4_MFS_TDR	0x08

	.macro addruart, rx, tmp, tmp2
	ldr	\rx, =(CONFIG_DEBUG_UART_PHYS)
	.endm

	.macro senduart, rd, rx
	strb	\rd, [\rx, #FM4_MFS_TDR]
	.endm

	.macro busyuart, rd, rx
1001:	ldr	\rd, [\rx, #FM4_MFS_SSR]
	tst	\rd, #FM4_MFS_SSR_TDRE
	beq	1001b
	.endm

	.macro waituart, rd, rx
	.endm
