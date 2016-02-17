/*
 * XMC4x00 USIC
 *
 * Copyright 2015-2016 Andreas Färber
 *
 * License: GPL-2.0+
 */
#ifndef __MFD_XMC4000_H
#define __MFD_XMC4000_H

#define USICx_CHy_CCFG		0x04
#define USICx_CHy_KSCFG		0x0C
#define USICx_CHy_CCR		0x40

#define USICx_CHy_CCFG_SSC	BIT(0)
#define USICx_CHy_CCFG_ASC	BIT(1)
#define USICx_CHy_CCFG_IIC	BIT(2)
#define USICx_CHy_CCFG_IIS	BIT(3)

#define USICx_CHy_KSCFG_MODEN		BIT(0)
#define USICx_CHy_KSCFG_BPMODEN		BIT(1)

#define USICx_CHy_CCR_MODE_DISABLED	0x0
#define USICx_CHy_CCR_MODE_SSC		0x1
#define USICx_CHy_CCR_MODE_ASC		0x2
#define USICx_CHy_CCR_MODE_IIS		0x3
#define USICx_CHy_CCR_MODE_IIC		0x4

#endif
