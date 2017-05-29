/*
 *  linux/arch/arm/mach-mb8ac0300/include/mach/gpio.h
 *
 *  Copyright (C) 2011 FUJITSU SEMICONDUCTOR LIMITED
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __MACH_GPIO_H
#define __MACH_GPIO_H

#include <linux/spinlock.h>
#include <linux/clk.h>

#define MB8AC0300_GPIO_NR		32	/* gpio pin number for every
							gpio controller*/
#define MB8AC0300_GPIO_NR_PER_REG	8	/* gpio pin number for every
							register hold */

#define MB8AC0300_GPIO_REG_PDR0		0x00UL	/* offset of PDR register */
#define MB8AC0300_GPIO_REG_DDR0		0x10UL	/* offset of DDR register */
#define MB8AC0300_GPIO_REG_PFR0		0x20UL	/* offset of PFR register */

/* peripheral function is set */
#define MB8AC0300_GPIO_IS_PERIPHERAL		1
/* output direction is set */
#define MB8AC0300_GPIO_IS_OUTPUT		1

/* shift gpio pin number offset to register address offset */
#define MB8AC0300_GPIO_OFFSET_TO_REG(offset) \
				(((offset) / MB8AC0300_GPIO_NR_PER_REG) * 4)

/* platform data information structure of gpio driver */
struct mb8ac0300_gpio_platform_data {
	unsigned int	gpio_base;	/* first pin number of gpio chip      */
	unsigned int	irq_base;	/* first extint number of gpio chip   */
	unsigned int	irq_gpio_min;	/* first gpio pin number for extint   */
	unsigned int	irq_gpio_max;	/* last gpio pin number for extint    */
};

/* gpio chip structure for gpio driver use */
struct mb8ac0300_gpio_chip {
	spinlock_t	lock;		/* lock for protect register accesses */
	void		*base;		/* base virtual address of registers  */
	struct mb8ac0300_gpio_platform_data	*pdata;
					/* pointer of platform data           */
	struct gpio_chip gc;		/* generic gpio chip interface        */
	struct clk	*clk;		/* pointer of clock device            */
	unsigned char	directions[MB8AC0300_GPIO_NR/MB8AC0300_GPIO_NR_PER_REG];
					/* gpio direction register init value */
	unsigned char	values[MB8AC0300_GPIO_NR/MB8AC0300_GPIO_NR_PER_REG];
					/* gpio data register init value      */
	unsigned char	functions[MB8AC0300_GPIO_NR/MB8AC0300_GPIO_NR_PER_REG];
					/* gpio function register init value  */
};

#endif /* __MACH_GPIO_H */
