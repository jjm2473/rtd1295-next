/*
 * linux/drivers/usb/host/f_usb20hdc-hcd.h - F_USB20HDC USB
 * host controller driver
 *
 * Copyright (C) FUJITSU ELECTRONICS INC. 2011. All rights reserved.
 * Copyright (C) 2012 FUJITSU SEMICONDUCTOR LIMITED.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 */

#ifndef _F_USB20HDC_HCD_H
#define _F_USB20HDC_HCD_H

#include <linux/platform_data/f_usb20hdc.h>

#define F_USB20HDC_MODE_HOST			0
#define F_USB20HDC_MODE_DEVICE			1
#define F_USB20HDC_MODE_DUAL_ROLE		2

static inline void ctrl_reg_cache_bits(
	void __iomem *base_addr, u32 reg_id,
	u32 *reg_cache)
{
	if (reg_id == F_USB20HDC_REG_INTS) {
		ctrl_ints_reg_cache_bits(base_addr, reg_id, reg_cache);
	} else if (reg_id == F_USB20HDC_REG_PORTSC) {
		ctrl_portsc_reg_cache_bits(base_addr, reg_id, reg_cache);
	} else if (reg_id == F_USB20HDC_REG_PORTSTSC) {
		ctrl_portstsc_reg_cache_bits(base_addr, reg_id, reg_cache);
	} else if (reg_id == F_USB20HDC_REG_HOSTEVENTSRC) {
		ctrl_hosteventsrc_reg_cache_bits(base_addr, reg_id, reg_cache);
	} else if (reg_id == F_USB20HDC_REG_OTGSTSC) {
		ctrl_otgstsc_reg_cache_bits(base_addr, reg_id, reg_cache);
	} else if ((reg_id >= F_USB20HDC_REG_EPCMD0) &&
			(reg_id <= F_USB20HDC_REG_EPCMD7)) {
		/*OTG driver won't read this register*/
		ctrl_default_reg_cache_bits(base_addr, reg_id, reg_cache);
	} else {
		ctrl_default_reg_cache_bits(base_addr, reg_id, reg_cache);
	}
}
#if defined(CONFIG_USB_F_USB20HDC_OTG_HOST_ONLY) || \
	defined(CONFIG_USB_F_USB20HDC_OTG_DUAL_ROLE)
extern int f_usb20hdc_hcd_probe(struct f_usb20hdc_otg *f_otg);
extern int f_usb20hdc_hcd_remove(struct f_usb20hdc_otg *f_otg);
extern int f_usb20hdc_hcd_suspend(struct f_usb20hdc_otg *f_otg);
extern int f_usb20hdc_hcd_resume(struct f_usb20hdc_otg *f_otg);
#else
static int f_usb20hdc_hcd_probe(struct f_usb20hdc_otg *f_otg)
{ return 0; }
static int  f_usb20hdc_hcd_remove(struct f_usb20hdc_otg *f_otg)
{ return 0; }
static int  f_usb20hdc_hcd_suspend(struct f_usb20hdc_otg *f_otg)
{ return 0; }
static int  f_usb20hdc_hcd_resume(struct f_usb20hdc_otg *f_otg)
{ return 0; }
#endif

#if defined(CONFIG_USB_F_USB20HDC_OTG_GADGET_ONLY) || \
	defined(CONFIG_USB_F_USB20HDC_OTG_DUAL_ROLE)
extern int f_usb20hdc_udc_probe(struct f_usb20hdc_otg *f_otg);
extern int f_usb20hdc_udc_remove(struct f_usb20hdc_otg *f_otg);
extern int f_usb20hdc_udc_suspend(struct f_usb20hdc_otg *f_otg);
extern int f_usb20hdc_udc_resume(struct f_usb20hdc_otg *f_otg);
#else
static int f_usb20hdc_udc_probe(struct f_usb20hdc_otg *f_otg)
{ return 0; }
static int  f_usb20hdc_udc_remove(struct f_usb20hdc_otg *f_otg)
{ return 0; }
static int  f_usb20hdc_udc_suspend(struct f_usb20hdc_otg *f_otg)
{ return 0; }
static int  f_usb20hdc_udc_resume(struct f_usb20hdc_otg *f_otg)
{ return 0; }
#endif

#if defined(CONFIG_USB_F_USB20HDC_OTG_DUAL_ROLE)
int f_udc_otg_suspend(struct f_usb20hdc_otg *f_otg);
int f_udc_otg_resume(struct f_usb20hdc_otg *f_otg);
#else
static int f_udc_otg_suspend(struct f_usb20hdc_otg *f_otg)
{ return 0; }
static int  f_udc_otg_resume(struct f_usb20hdc_otg *f_otg)
{ return 0; }
#endif

#endif
