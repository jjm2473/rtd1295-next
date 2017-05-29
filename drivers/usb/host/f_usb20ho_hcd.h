/*
 * linux/drivers/usb/host/f_usb20ho_hcd.h - F_USB20HDC USB
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

#ifndef _F_USB20HO_HCD_H
#define _F_USB20HO_HCD_H

struct f_usb20ho_hcd {
	struct device *dev;
	struct platform_device *ehci_dev;
	struct platform_device *ohci_dev;
	int irq;
};
#endif
