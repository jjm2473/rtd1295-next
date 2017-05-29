/*
 *  linux/arch/arm/mach-mb8ac0300/io.c
 *
 * Copyright (C) 2011 FUJITSU SEMICONDUCTOR LIMITED
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <linux/kernel.h>
#include <linux/module.h>

#include <asm/io.h>
#include <asm/memory.h>
#include <asm/mach/map.h>
#include <asm/tlb.h>

#include <linux/platform_data/mb8ac0300-iomap.h>

void __iomem *mb8ac0300_scu_base;


#define IOMAP_DEV(name) { \
		.virtual = (unsigned long) MB8AC0300_##name##_VIRT, \
		.pfn = __phys_to_pfn(MB8AC0300_##name##_PHYS), \
		.length = MB8AC0300_##name##_SIZE, \
		.type = MT_DEVICE, \
	}
#define IOMAP_MEM(name) { \
		.virtual = (unsigned long) MB8AC0300_##name##_VIRT, \
		.pfn = __phys_to_pfn(MB8AC0300_##name##_PHYS), \
		.length = MB8AC0300_##name##_SIZE, \
		.type = MT_MEMORY, \
	}
#define IOMAP_MEM_NONCACHED(name) { \
		.virtual = (unsigned long) MB8AC0300_##name##_VIRT, \
		.pfn = __phys_to_pfn(MB8AC0300_##name##_PHYS), \
		.length = MB8AC0300_##name##_SIZE, \
		.type = MT_MEMORY_NONCACHED, \
	}

static struct map_desc mb8ac0300_io_desc[] = {
	IOMAP_DEV(UART0),
	IOMAP_DEV(MRBC),
	IOMAP_DEV(CORE),
	IOMAP_DEV(CRG11_MAIN),
	IOMAP_DEV(CRG11_DMC),
	IOMAP_DEV(CRG11_GMAC),
	IOMAP_DEV(CRG11_LCD),
	IOMAP_DEV(DDRC),
	IOMAP_MEM_NONCACHED(XSRAM),
};

/**
 * mb8ac0300_map_io()
 * make static mappings for initial devices
 */
void __init mb8ac0300_map_io(void)
{
	debug_ll_io_init();

	mb8ac0300_scu_base = MB8AC0300_CORE_VIRT;

	iotable_init(&mb8ac0300_io_desc[0], ARRAY_SIZE(mb8ac0300_io_desc));
}
