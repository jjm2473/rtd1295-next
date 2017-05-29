#ifndef __MACH_MB86S70_DEVICES_H__
#define __MACH_MB86S70_DEVICES_H__

bool __init mb86s70_smp_init_ops(void);
void __init mb86s70_dt_map_io(void);
void __init mb86s70_dt_timer_init(void);
void __init mb86s70_dt_init(void);
void __init mb86s704_secondary_startup(void);

#endif
