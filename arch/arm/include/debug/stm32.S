/*
 * STM32 USART low-level support
 *
 * Copyright (c) 2014-2015 Andreas Färber
 *
 * License: GPL-2.0+
 */

#define STM32_USART_SR	0
#define STM32_USART_DR	4

#define STM32_USART_SR_TXE	(1 << 7)


	.macro addruart, rx, tmp, tmp2
	ldr	\rx, =(CONFIG_DEBUG_UART_PHYS)
	.endm

	.macro senduart, rd, rx
	strb	\rd, [\rx, #STM32_USART_DR]
	.endm

	.macro	busyuart, rd, rx
1001:	ldr	\rd, [\rx, #STM32_USART_SR]
	tst	\rd, #STM32_USART_SR_TXE
	beq	1001b
	.endm

	.macro	waituart, rd, rx
	.endm
