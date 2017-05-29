/*
 * mipilli_test.c F_MIPILLI_LP Controller Driver
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
#include "mipilli_api.h"

bool phy_test_start(u8 test_lane, u8 ch_num)
{

	bool result = true;
	u32 txlane_bit, rxlane_bit;
	u32 master_cfg, slave_cfg;
	struct lli_data *the_lli;
	struct lli_cfg *pcfg;

	the_lli = &gp_mb86s70_lli_dev->chan_data[ch_num];
	pcfg = (struct lli_cfg *)the_lli->lli_defcfg;

	/* DBG_MSG("Master test start start\n"); */

	txlane_bit = ~(0xFFFFFFFF << test_lane);
	rxlane_bit = ~(0xFFFFFFFF << test_lane);

	master_cfg = (pcfg->pa_cfig.pa_phy_test_config_master |
	    (txlane_bit << 4) | (rxlane_bit << 16));

	slave_cfg = (pcfg->pa_cfig.pa_phy_test_config_slasve |
	    (rxlane_bit << 4) | (txlane_bit << 16));

	pcfg->pa_cfig.pa_phy_test_config_master = master_cfg;
	pcfg->pa_cfig.pa_phy_test_config_slasve = slave_cfg;

	mipilli_config_update(ACT_LLI_CH, test_lane, test_lane);

	lli_lo_wt(MIPI_LLI_PA_CSA_PA_SET, B_CSA_PA_TESTMODE);
	lli_rm_wt(MIPI_LLI_PA_CSA_PA_SET, B_CSA_PA_TESTMODE);


	/* DBG_MSG("Master test start done successfully\n"); */

	/*master_test_start_err:*/
	return result;
}


bool phy_test_stop(void)
{

	bool result = true;
	u8 r = 0;
	u32 retry_cnt = RETRY_COUNTER;

	/* DBG_MSG("Master test stop start\n"); */

	lli_rm_wt(MIPI_LLI_PA_CSA_PA_CLEAR, B_CSA_PA_TESTMODE);
	lli_lo_wt(MIPI_LLI_PA_CSA_PA_CLEAR, B_CSA_PA_TESTMODE);

	while (r == 0) {
		r = ~((lli_lo_rd(PA_CSA_PA_STATUS) & B_CSA_PA_TESTMODE));
		retry_cnt = retry_cnt - 1;
		if (retry_cnt <= 0)
			break;

		/* msleep(20); */
	}

	result = !(retry_cnt <= 0);

	/* DBG_MSG("Phy test stop done successfully\n"); */

	return result;
}


bool slave_main_loop(u8 ch_num)
{
	bool result = true;
	/* u32 intval; */
	u8 rx_lane, act_rx_cnt, r = 0;
	u32 rx_lane_bit = 0;
	u32 int_s = 0;
	struct lli_data *the_lli;
	struct lli_cfg *pcfg;

	the_lli = &gp_mb86s70_lli_dev->chan_data[ch_num];
	pcfg = (struct lli_cfg *)the_lli->lli_defcfg;

	/* loop until serious error detection */
	while (1) {
		r = wait_for_completion_timeout(&the_lli->msg_cmp, CMP_TIMEOUT);
		INIT_COMPLETION(the_lli->msg_cmp);
		int_s = the_lli->lli_interrupt_status;

		if (r > 0 && (int_s & B_CRSTAST)) {
			/* DBG_MSG("ColdReset Detected\n"); */

			/* Clear Reset */
			lli_lo_wt(MIPI_USER_CTRL_CLDRST, 0);

			/* ToDo: Check if this call can be here,
			 * or after MOUNT.
			*/

			setup_error_detect();
			set_lo_cfg(pcfg);
			set_err_wacfg(pcfg, false, false);

			/* DBG_MSG("Start Slave Mount Again\n"); */
			mipilli_slave_mount(ACT_LLI_CH);
		}

		if (r > 0 && (int_s & B_UNMNT)) {

			/* DBG_MSG("Slave unmounted,
			   wait for next mounting\n");
			*/

			rx_lane = lli_lo_rd(PA_ACT_RX_CNT);

			/* DBG_MSG("rx_lane: %02x\n",
			   rx_lane);
			 */

			/* Make Rx Hibernate ????? */
			act_rx_cnt = pcfg->pa_cfig.active_rx_count;
			mphy_write_reg(
				MIPI_MPHY0_RXENTHIBE8,
				MPHY_WRITE_TAG_LOCAL,
				ENTER, 0, 0, 0,
				act_rx_cnt);

			/* set PA config_update */
			/* DBG_MSG("Set PA config_update\n"); */
			rx_lane_bit = ~(0xFFF << rx_lane);

			lli_lo_wt(MIPI_LLI_PA_CONFIG_UPDATE, rx_lane_bit << 12);

			mipilli_slave_mount(ACT_LLI_CH);
		}
		/*
		if (intval & B_RSTONERRDET){
			DBG_MSG("ResetOnError Detected\n");
		}
		*/
		#if DBG_FLAG
		if (r == 0)
			DBG_MSG("slave_main_loop:int timeout\n");
		#endif
		msleep(20);
	}

	return result;
}


/* ROE : Reset On Error */
void roe_test(struct lli_cfg *pcfg)
{
	u32 int_s , try = RETRY_COUNTER;
	u8 r = 0;
	struct lli_data *the_lli;

	the_lli = &gp_mb86s70_lli_dev->chan_data[ACT_LLI_CH];

	/* Enable ROE */
	lli_lo_wt(MIPI_USER_CTRL_ROE, 1);

	while (try > 0) {

		r = wait_for_completion_timeout(&the_lli->msg_cmp, CMP_TIMEOUT);
		INIT_COMPLETION(the_lli->msg_cmp);
		int_s = the_lli->lli_interrupt_status;

		/* int_s = lli_lo_rd(MIPI_USER_CTRL_INTST);
		if (int_s)
			lli_lo_wt(MIPI_USER_CTRL_INTCLR, intval);
		*/

		if (r > 0 && (int_s & B_CRSTAST) > 0) {
			/* DBG_MSG("ColdReset Detected\n"); */
			/* DBG_MSG("Clear ColdReset Now\n"); */

			/* Clear Reset */
			lli_lo_wt(MIPI_USER_CTRL_CLDRST, 0);

			/* DBG_MSG("Start Mounting now\n"); */

			/* ToDo: Check if this call can be
			 * here, or after MOUNT.
			*/
			setup_error_detect();
			set_lo_cfg(pcfg);
			set_err_wacfg(pcfg, false, false);
			/* mipilli_master_mount(pcfg); */
			break;
		}
		#if DBG_FLAG
		if (r == 0)
			DBG_MSG("roe_test:int timeout\n");
		#endif
		msleep(20);
		try--;
	}
}
