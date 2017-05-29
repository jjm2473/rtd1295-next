/*
 * linux/arch/arm/mach-mb8ac0300/devices.h
 *
 * Copyright (C) 2013 Linaro, LTD
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

extern void __init mb8ac0300_map_io(void);
extern void __init mb8ac0300_init_irq(void);
extern void __init mb8ac0300_init_timer(void);
extern void __init mb8ac0300_init(void);
extern struct smp_operations mb8ac0300_smp_ops;
extern void mb8ac0300_restart(char mode, const char *cmd);
extern void crg11_restart(void);
extern void mb8ac0300_secondary_startup(void);
extern const int mb8ac0300_cpu_sleep_size;
extern void mb8ac0300_cpu_save(long);
extern void mb8ac0300_cpu_sleep(u32 *);
extern int __init mb8ac0300_pm_init(void);
extern const void __nosave_begin, __nosave_end;

