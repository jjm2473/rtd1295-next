/*
 * mipilli_api.h F_MIPILLI_LP Controller Driver
 * Copyright (C) 2013 Fujitsu Semi, Ltd
 * Author: Slash Huang <slash.huang@tw.fujitsu.com>
 *
 *   This code is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This code is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#ifndef MIPILLI_API_H_
#define MIPILLI_API_H_

#include <linux/delay.h>
#include "mipilli_registers.h"
#include "mipilli_mem.h"
#include "mipilli_mb86s70.h"


/* Main function */
int mipilli_master_mount(u8 ch_num);
int mipilli_slave_mount(u8 ch_num);
int mipilli_master_unmount(u8 ch_num);
int mipilli_config_update(u8 ch_num, u8 txlanes, u8 rxlanes);
int mipilli_change_memmap(u32 from_addr, u32 to_addr, u32 size);
int mipilli_write_signal(u32 v);
u32 mipilli_read_signal(void);


/* Help Function */
void set_lo_cfg(struct lli_cfg *p_config);
void set_timeout(struct lli_cfg *p_config);
void set_lo_timeout(struct lli_cfg *p_config);
void set_deskew_to(struct lli_cfg *p_config);
void local_set_deskew_to(struct lli_cfg *p_config);
void setup_error_detect(void);
void set_err_wacfg(struct lli_cfg *pcfg, bool askpam, bool pamdef);
void set_mphy_attr(struct lli_cfg *pcfg);
bool update_shadow_remote(struct lli_cfg *pcfg, u8 txlan, u8 rxlan);
bool phy_link_update(bool unmount, u8 ch_num);
u32 worst_prep_length_in_si(struct lli_cfg *p_config);
u32 worst_local_prep_length_in_si(struct lli_cfg *p_config);
u32 worst_min_sleep_stall(struct lli_cfg *p_config);
u32 worst_local_min_sleep_stall(struct lli_cfg *p_config);
void set_lo_timeout(struct lli_cfg *p_config);
u32 unit_conv_si_to_pclk(u32 sival, struct lli_cfg *p_config);
u32 get_sync_len(u8 sync_lencap);
u32 unit_conv_symbol_to_si(u32 symbolval);
u32 unit_conv_si_to_symbol(u32 sival);
u32 measure_nack_rtt(void);

void mphy_write_reg(u32 addr_base, u8 targ, u8 val,
u8 txlanes_s, u8 txlanes_d,
u8 rxlanes_s, u8 rxlanes_d);


bool phy_test_start(u8 test_lane, u8 ch_num);
bool phy_test_stop(void);
void roe_test(struct lli_cfg *p_config);
bool slave_main_loop(u8 ch_num);

/* Register access function */
void lli_lo_wt(u32 f, u32 v);
u32 lli_lo_rd(u32 v);
void lli_rm_wt(u32 f, u32 v);
u32 lli_rm_rd(u32 v);


#endif /* MIPILLI_API_H_ */
