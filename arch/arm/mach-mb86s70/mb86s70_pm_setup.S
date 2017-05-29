/*
 * arch/arm/mach-mb86s70/mb86s70_pm_setup.S
 *
 * Shameless copy of dcscb_setup.S
 * Created by:  Dave Martin, 2012-06-22
 * Copyright:   (C) 2012-2013  Linaro Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/linkage.h>

ENTRY(mb86s70_pm_power_up_setup)
	b cci_enable_port_for_self
	bx	lr
ENDPROC(mb86s70_pm_power_up_setup)
