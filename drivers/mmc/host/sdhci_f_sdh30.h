/*
 * linux/drivers/mmc/host/sdhci_f_sdh30.h
 *
 * Copyright (C) 2013 FUJITSU SEMICONDUCTOR LIMITED
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 */

#ifndef F_SDH30_H
#define F_SDH30_H

/* F_SDH30 extended Controller registers */
#define F_SDH30_AHB_CONFIG		0x100
#define  F_SDH30_AHB_BIGED		0x00000040
#define  F_SDH30_BUSLOCK_DMA		0x00000020
#define  F_SDH30_BUSLOCK_EN		0x00000010
#define  F_SDH30_SIN			0x00000008
#define  F_SDH30_AHB_INCR_16		0x00000004
#define  F_SDH30_AHB_INCR_8		0x00000002
#define  F_SDH30_AHB_INCR_4		0x00000001

#define F_SDH30_TUNING_SETTING		0x108
#define  F_SDH30_CMD_CHK_DIS		0x00010000

#define F_SDH30_IO_CONTROL2		0x114
#define  F_SDH30_CRES_O_DN		0x00080000
#define  F_SDH30_MSEL_O_1_8		0x00040000

#define F_SDH30_ESD_CONTROL		0x124
#define	F_SDH30_CMD_DAT_DELAY		0x200

#define F_SDH30_MIN_CLOCK		400000

#endif
