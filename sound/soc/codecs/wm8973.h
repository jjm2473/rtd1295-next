/*
 * wm8973.h  --  audio driver for WM8973
 *
 * Copyright 2013 Fujitsu, Inc.
 *
 * Author: FMPI <>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#ifndef _WM8973_H
#define _WM8973_H

#define WM8973_LINVOL	0x00
#define WM8973_RINVOL	0x01
#define WM8973_LOUT1V	0x02
#define WM8973_ROUT1V	0x03
#define WM8973_ADCDAC	0x05
#define WM8973_IFACE	0x07
#define WM8973_SRATE	0x08
#define WM8973_LDAC	0x0a
#define WM8973_RDAC	0x0b
#define WM8973_BASS	0x0c
#define WM8973_TREBLE	0x0d
#define WM8973_RESET	0x0f
#define WM8973_3D	0x10
#define WM8973_ALC1	0x11
#define	WM8973_ALC2	0x12
#define	WM8973_ALC3	0x13
#define WM8973_NGATE	0x14
#define WM8973_LADC	0x15
#define WM8973_RADC	0x16
#define	WM8973_ADCTL1	0x17
#define	WM8973_ADCTL2	0x18
#define WM8973_PWR1	0x19
#define WM8973_PWR2	0x1a
#define	WM8973_ADCTL3	0x1b
#define WM8973_ADCIN	0x1f
#define	WM8973_LADCIN	0x20
#define	WM8973_RADCIN	0x21
#define WM8973_LOUTM1	0x22
#define WM8973_LOUTM2	0x23
#define WM8973_ROUTM1	0x24
#define WM8973_ROUTM2	0x25
#define WM8973_MOUTM1	0x26
#define WM8973_MOUTM2	0x27
#define WM8973_LOUT2V	0x28
#define WM8973_ROUT2V	0x29
#define WM8973_MOUTV	0x2A

#define WM8973_SYSCLK	0

#endif
