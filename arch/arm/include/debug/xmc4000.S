/*
 * XMC4000
 *
 * Copyright (c) 2015 Andreas Färber
 */

#define XMC4500_USIC_CHx_TRBSR	0x114
#define XMC4500_USIC_CHx_IN0	0x180

#define XMC4500_USIC_CHx_TRBSR_TEMPTY	(1 << 11)
#define XMC4500_USIC_CHx_TRBSR_TFULL	(1 << 12)

	.macro addruart, rx, tmp, tmp2
	ldr	\rx, =(CONFIG_DEBUG_UART_PHYS)
	.endm

	.macro senduart, rd, rx
	strb	\rd, [\rx, #XMC4500_USIC_CHx_IN0]
	.endm

	.macro busyuart, rd, rx
1001:	ldr	\rd, [\rx, #XMC4500_USIC_CHx_TRBSR]
	tst	\rd, #XMC4500_USIC_CHx_TRBSR_TFULL
	bne	1001b
	.endm

	.macro waituart, rd, rx
1001:	ldr	\rd, [\rx, #XMC4500_USIC_CHx_TRBSR]
	tst	\rd, #XMC4500_USIC_CHx_TRBSR_TEMPTY
	beq	1001b
	.endm
