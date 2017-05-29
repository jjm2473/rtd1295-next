/*
 * f_fpdl_tx FPD-Link TX driver
 *
 * based on f_mipidsi1_lp MIPI DSI PHY driver
 *
 * Copyright (C) 2013 Linaro, Ltd (Andy Green <andy.green@linaro.org>)
 * Copyright (C) 2013 Fujitsu Semiconductor, Ltd
 */

#ifndef ___LINUX_VIDEO_F_FPDL_TX_H__
#define ___LINUX_VIDEO_F_FPDL_TX_H__

enum {
	SYSOC_FPDL_PADSET_OFS = 0x2000,	/* PaddingData setting register */
	SYSOC_FPDL_PWRDWNSET_OFS = 0x2004, /* I/O PowerDown setting register */
	SYSOC_FPDL_OUTSET_OFS = 0x2008, /* LVDS OutputMode setting register */
};

enum {
	SYSOC_FPDL_PADSET__RSVD__SHIFT = 0,
	SYSOC_FPDL_PADSET__RSVD__MASK = 1,
	SYSOC_FPDL_PWRDWNSET__SUSP__SHIFT = 0,
	SYSOC_FPDL_PWRDWNSET__SUSP__MASK = 1,
	SYSOC_FPDL_OUTSET__SELMAP__SHIFT = 0,
	SYSOC_FPDL_OUTSET__SELMAP__MASK = 1,
};

#endif

