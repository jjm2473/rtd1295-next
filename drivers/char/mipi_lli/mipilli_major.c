/*
 * mipilli_major.c F_MIPILLI_LP Controller Driver
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

#define master_mount() \
	lli_lo_wt(CTRL_CSASYSTEMSET,\
	(B_CSA_MASTER_NOT_SLAVE | B_CSA_LLI_MOUNT_CTRL));

#define master_unmount() \
	lli_lo_wt(CTRL_CSASYSTEMCLR,\
	B_CSA_LLI_MOUNT_CTRL);


#define if_mounted() \
	((lli_lo_rd(CTRL_CSASYSTEMSTAT) & \
	B_CSA_LLI_MOUNTED) ? true : false)

#define if_link_update() \
	((lli_lo_rd(PA_CSA_PA_STATUS) & \
	B_CSA_PA_LINKUPDATECONFIG) ? true : false)

#define dis_local_automode() \
	lli_lo_wt(MIPI_LLI_PA_CSA_PA_CLEAR,\
	B_CSA_PA_AUTO_MODE);

#define dis_remote_automode() \
	lli_rm_wt(MIPI_LLI_PA_CSA_PA_CLEAR,\
	B_CSA_PA_AUTO_MODE);

#define slave_unmount() \
	lli_rm_wt(CTRL_CSASYSTEMCLR, \
	B_CSA_LLI_MOUNT_CTRL);

#define slave_mount() \
do {\
	lli_lo_wt(CTRL_CSASYSTEMCLR, \
	B_CSA_MASTER_NOT_SLAVE); \
	lli_lo_wt(CTRL_CSASYSTEMSET, \
	B_CSA_LLI_MOUNT_CTRL);\
} while (0)

#define phy_link_upddate() \
	lli_lo_wt(MIPI_LLI_PA_CSA_PA_SET,\
	B_CSA_PA_LINKUPDATECONFIG);

#define make_tx_hib8(a) \
	mphy_write_reg(MIPI_MPHY0_TXHIBERN8CTRL,\
	MPHY_WRITE_TAG_LOCAL | MPHY_WRITE_TAG_REMOTE,\
	ENTER, 0,\
	a->pa_cfig.active_tx_count, 0, 0);

#define make_tx_exit_hib8(a) \
	mphy_write_reg(MIPI_MPHY0_TXHIBERN8CTRL,\
	MPHY_WRITE_TAG_LOCAL | MPHY_WRITE_TAG_REMOTE,\
	EXIT, 0, a->pa_cfig.active_tx_count, 0, 0);


#define make_rx_hib8(a) \
	mphy_write_reg(MIPI_MPHY0_RXENTHIBE8,\
	MPHY_WRITE_TAG_LOCAL, ENTER,\
	0, 0, 0, a->pa_cfig.active_rx_count);

#define make_rx_exit_hib8(a) \
	mphy_write_reg(MIPI_MPHY0_RXENTHIBE8,\
	MPHY_WRITE_TAG_LOCAL, EXIT, 0, 0, 0, a->pa_cfig.active_rx_count);


int mipilli_write_signal(u32 v)
{
	lli_rm_wt(MIPI_LLI_SIGNAL_SIGNALSET, v);
	return 0;
}

u32 mipilli_read_signal(void)
{
	u32 v = 0;

	v = lli_lo_rd(MIPI_LLI_SIGNAL_SIGNALSTATE);
	return v;
}


int mipilli_master_mount(u8 ch_num)
{
	u8 r = 0;
	u8 retry_cnt = RETRY_COUNTER;
	struct lli_data *the_lli;

	the_lli = &gp_mb86s70_lli_dev->chan_data[ch_num];
	#if DBG_M_MOUNT
	DBG_MSG("1.%s Master mount start\n",
		the_lli->lli_chan_name);
	#endif

	master_mount();

	#if DBG_M_MOUNT
	DBG_MSG("2.Master Wait for INT_MNT\n");
	#endif

	the_lli = &gp_mb86s70_lli_dev->chan_data[ch_num];
	r = wait_for_completion_timeout(&the_lli->msg_cmp, CMP_TIMEOUT);
	INIT_COMPLETION(the_lli->msg_cmp);

	if (r == 0) {
		ERR_MSG("Error->Master MNT_INT timeout\n");
		return MASTER_MOUNT_MNTINT_TIMEOUT;
	}

	if (the_lli->lli_interrupt_status & B_MNT) {

		DBG_MSG("3.Master detect MNT_INT\n");

		#if DBG_M_MOUNT
		while ((if_mounted() == false)) {
			if (retry_cnt <= 0) {
				ERR_MSG(
				"Error->Master no detect CSA:MOUNTED\n");
				r = MASTER_MOUNT_NO_CSA_MOUNTED;
				the_lli->mount_stat = false;
				ERR_MSG("Error->Master Mount done fail\n");
				return r;
			}
			retry_cnt--;
			/*msleep(20);*/
		}
		#endif

		the_lli->mount_stat = true;

		#ifdef DBG_M_MOUNT
		DBG_MSG("4.Master detect CSA:MOUNTED\n");
		DBG_MSG("5.Master Mount done successfully\n");
		#endif
		return MASTER_MOUNT_SUCCESS;

	}

	ERR_MSG("Error->Master no detect INT_MNT\n");
	r = MASTER_MOUNT_NO_MNTINT;
	the_lli->mount_stat = false;
	ERR_MSG("Error->Master Mount done fail\n");
	return r;
}


int mipilli_master_unmount(u8 ch_num)
{
	u32 r = 0;
	u32 retry_cnt = RETRY_COUNTER;
	u32 rx_lane = 0;
	struct lli_data *the_lli;
	struct lli_cfg *p_config;

	the_lli = &gp_mb86s70_lli_dev->chan_data[ch_num];
	p_config = (struct lli_cfg *)the_lli->lli_defcfg;

	#if DBG_M_UNMOUNT
	DBG_MSG("1.Master Unmount start\n");
	#endif

	/* Disable Auto mode*/
	#if DBG_M_UNMOUNT
	DBG_MSG("2.Master disable auto mode\n");
	#endif

	dis_local_automode();
	dis_remote_automode();

	/*DBG_MSG("Make Tx Hibernate(in shadow)\n");*/

	make_tx_hib8(p_config);

	#if DBG_M_UNMOUNT
	DBG_MSG("3.Set Un-mount\n");
	#endif

	#if DBG_M_UNMOUNT
	DBG_MSG("Set Slave Unmount\n");
	#endif

	slave_unmount();

	#if DBG_M_UNMOUNT
	DBG_MSG("Set Master Unmount\n");
	#endif

	master_unmount();

	#if DBG_M_UNMOUNT
	DBG_MSG("4.Master Wait UnMNT_INT\n");
	#endif

	r = wait_for_completion_timeout(&the_lli->msg_cmp, CMP_TIMEOUT);
	INIT_COMPLETION(the_lli->msg_cmp);
	if (!r) {
		ERR_MSG("Error->UnMNT INT Timeout\n");
		r = MASTER_UNMOUNT_UNMNTINT_TIMEOUT;
		return r;
	}

	if (the_lli->lli_interrupt_status & B_UNMNT) {

		DBG_MSG("5.Master detect UnMNT_INT\n");

		#if DBG_M_UNMOUNT
		while ((lli_lo_rd(CTRL_CSASYSTEMSTAT) |
				B_CSA_LLI_MOUNT_CTRL) > 0) {

			retry_cnt--;
			if (retry_cnt <= 0) {
				r = MASTER_UNMOUNT_CSA_MOUNTED;
				ERR_MSG(
				"Err->Master CSA:MOUNT_CTRL!=0\n");
				return r;
			}
			/*msleep(20);*/
		}
		#endif

		#if DBG_M_UNMOUNT
		DBG_MSG("6.Master no detect CSA:MOUNT_CTRL\n");
		DBG_MSG("7.Master make Rx Hibernate\n");
		#endif

		make_rx_hib8(p_config);

		/*set PA config_update*/
		/*DBG_MSG("Set PA config_update\n");*/
		rx_lane = ~(0xFFF << p_config->pa_cfig.active_rx_count);
		lli_lo_wt(MIPI_LLI_PA_CONFIG_UPDATE, rx_lane << 12);
	}

	r = MASTER_UNMOUNT_SUCCESS;
	the_lli->mount_stat = false;
	DBG_MSG("8.Master unmount successfully\n");
	return r;
}

/*8.4 LLI Mount Procedure*/
int mipilli_slave_mount(u8 ch_num)
{
	u8 r = 0;
	u32 retry_cnt = RETRY_COUNTER;
	u32 int_s = 0;
	struct lli_cfg *p_config;
	struct lli_data *the_lli;

	the_lli = &gp_mb86s70_lli_dev->chan_data[ch_num];
	p_config = (struct lli_cfg *)the_lli->lli_defcfg;

	DBG_MSG("1.%s Slave mount start\n", the_lli->lli_chan_name);
	DBG_MSG("2.Slave Wait for RXH8EXIT_INT\n");

	while (retry_cnt > 0) {

		r = wait_for_completion_timeout(&the_lli->msg_cmp, CMP_TIMEOUT);
		INIT_COMPLETION(the_lli->msg_cmp);

		int_s = the_lli->lli_interrupt_status & B_RXH8EXIT;
		/*
		if (!r) {
			DBG_MSG("Error->RXH8EXIT_INT timeout\n");
		}
		*/
		if ((r > 0) && (int_s > 0)) {
			#ifdef DBG_S_MOUNT
			DBG_MSG("3.Slave Get RXH8EXIT_INT\n");
			#endif

			retry_cnt = RETRY_COUNTER;
			while ((lli_lo_rd(CTRL_CSASYSTEMSTAT) &
				    B_CSA_PA_HIB8) > 0) {

				retry_cnt--;
				if (retry_cnt <= 0) {
					ERR_MSG(
					"Error->CSASYSTEMSTAT:HIB8=1\n");
					return SLAVE_MOUNT_CSA_HIB8EXIT;
				}
				/*msleep(20);*/
			}

			#if DBG_S_MOUNT
			DBG_MSG("4.CSASYSTEMSTAT:HIBERNATE==0\n");
			#endif
			slave_mount();
			/*goto lab_slave_mount;*/
			retry_cnt = RETRY_COUNTER;
			break;
		}
		/*msleep(20);*/
		retry_cnt--;
	}

	if (retry_cnt <= 0) {
		r = SLAVE_MOUNT_HIB8EXIT_INT_TIME_OUT;
		ERR_MSG("Error->Slave RXH8EXIT_INT timeout\n");
		/*goto slave_mount_err;*/
		the_lli->mount_stat = false;
		ERR_MSG("Error->Slave Mount fail\n");
		return r;
	}

/*lab_slave_mount:*/
	#if DBG_S_MOUNT
	DBG_MSG("5.Slave wait MNT_INT\n");
	#endif
	r = wait_for_completion_timeout(&the_lli->msg_cmp, CMP_TIMEOUT);
	INIT_COMPLETION(the_lli->msg_cmp);

	if (!r) {
		r = SLAVE_MOUNT_MNTINT_TIMEOUT;
		ERR_MSG("Error->Slave MNT_INT timerout!\n");
		/*goto slave_mount_err;*/
		the_lli->mount_stat = false;
		ERR_MSG("Error->Slave Mount fail\n");
		return r;
	}

	if (the_lli->lli_interrupt_status & B_MNT) {

		#if DBG_S_MOUNT
		DBG_MSG("6.Slave Get MNT_INT\n");
		#endif

		retry_cnt = RETRY_COUNTER;
		while (if_mounted() == false) {
			retry_cnt--;
			if (retry_cnt <= 0) {
				r = SLAVE_MOUNT_NO_CSA_MOUNTED;
				ERR_MSG(
				"Err->Slave MOUNTED not detected\n");
				/*goto slave_mount_err;*/
				the_lli->mount_stat = false;
				ERR_MSG("Error->Slave Mount fail\n");
				return r;
			}
			/*msleep(20);*/
		}

		#if DBG_S_MOUNT
		DBG_MSG("7.Slave MOUNTED detected\n");
		DBG_MSG("8.Slave Mount done successfully\n");
		#endif
		the_lli->mount_stat = true;
		return SLAVE_MOUNT_SUCCESS;
	}

	/*slave_mount_err:*/
	the_lli->mount_stat = false;
	ERR_MSG("Error->Slave Mount fail\n");
	return r;
}

/*
 * Configuration Update and Lane Count Update implementation.
 * Lane Count Update mipi_lli_spec 8.6
 * Configuration Change Procedure
 */
int mipilli_config_update(u8 ch_num, u8 txlanes, u8 rxlanes)
{
	u32 retry_cnt = RETRY_COUNTER;
	u32 r = 0;
	u32 prev_auto_mode = 0;
	u8 old_txlanes = 0;
	u8 old_rxlanes = 0;
	u32 act_tx_cnt = 0;
	u32 act_rx_cnt = 0;

	struct lli_data *the_lli;
	struct lli_cfg *p_config;

	the_lli = &gp_mb86s70_lli_dev->chan_data[ch_num];
	p_config = (struct lli_cfg *)the_lli->lli_defcfg;

	#if DBG_CONFIG_UPDATE
	DBG_MSG("1.ConfigUpdate start\n");
	#endif

	r = lli_lo_rd(PA_CSA_PA_STATUS);
	/* prev_auto_mode = ((r & B_CSA_PA_AUTO_MODE) ? true : false); */

	/* Disable Auto mode*/
	#if DBG_CONFIG_UPDATE
	DBG_MSG("2.Disable Local Auto Mode\n");
	#endif

	dis_local_automode();

	#if DBG_CONFIG_UPDATE
	DBG_MSG("3.Disable Remote Auto Mode\n");
	#endif

	dis_remote_automode();

	/* Save old TX/RX lanes*/
	old_txlanes = lli_lo_rd(MIPI_LLI_PA_ACTIVETXCOUNT);
	old_rxlanes = lli_lo_rd(PA_ACT_RX_CNT);

	/* for each shadow attributes except active lane counts.
	   set local value and then set remote value using SVC
	*/
	r = update_shadow_remote(p_config, txlanes, rxlanes);

	if (!r) {
		r = CONFIG_UPDATE_SHADOW_FAIL;
		/* goto config_update_err; */
		ERR_MSG("Error:ConfigUpdate done fail\n");
		return r;
	}

	if (old_txlanes < txlanes) {
		/* Set new lanes to Hibernate8 state
		 * Set previously unused lanes to go out of Hibernate8
		*/
		make_tx_exit_hib8(p_config);
		/*
		if (!result) {
			goto configUpdate_Err;
		}
		*/
	}

	/* update Active_lane count if required*/
	if (old_txlanes != txlanes) {
		/*DBG_MSG("Update Master TX active lane count\n");*/
		act_tx_cnt = p_config->pa_cfig.active_tx_count;
		lli_lo_wt(MIPI_LLI_PA_ACTIVETXCOUNT, act_tx_cnt);
		lli_rm_wt(PA_ACT_RX_CNT, act_tx_cnt);
	}

	if (old_rxlanes != rxlanes) {
		/*DBG_MSG("Update Master RX active lane count\n");*/
		act_rx_cnt = p_config->pa_cfig.active_rx_count;
		lli_lo_wt(PA_ACT_RX_CNT, act_rx_cnt);
		lli_rm_wt(MIPI_LLI_PA_ACTIVETXCOUNT, act_rx_cnt);
	}

	#if DBG_CONFIG_UPDATE
	DBG_MSG("4.Start PLU Update\n");
	#endif

	phy_link_upddate();

	#if DBG_CONFIG_UPDATE
	DBG_MSG("5.Wait PLU INT\n");
	#endif

	r = wait_for_completion_timeout(&the_lli->msg_cmp, CMP_TIMEOUT);
	INIT_COMPLETION(the_lli->msg_cmp);

	if (r == 0) {
		ERR_MSG("Error:PLU INT timeout\n");
		r = CONFIG_UPDATE_PLU_INT_TIMEOUT;
		ERR_MSG("Error:ConfigUpdate done fail\n");
		return r;
	}

	if (the_lli->lli_interrupt_status & B_PLUFIN) {

		DBG_MSG("6.PLU INT Detect\n");

		while (if_link_update() == true) {
			retry_cnt--;
			if (retry_cnt <= 0) {
				ERR_MSG(
				"Error:PLUFin CSA LinkUpdateConfig=1\n");
				r = CONFIG_UPDATE_CSA_LINKUPDATE;
				break;
			}
			/*msleep(20);*/
		}

		if (retry_cnt > 0) {

			#if DBG_CONFIG_UPDATE
			DBG_MSG(
			"7.PLUFin in CSA LinkUpdateConfig=0\n");
			DBG_MSG(
			"8.PLUFin in CSA LinkUpdateConfig success\n");
			#endif
			r = CONFIG_UPDATE_SUCCESS;
		}
	} else {
		ERR_MSG("Error:PLU INT failure (No detect)\n");
		r = CONFIG_UPDATE_NO_PLU_INT;
	}

	/* Set Auto mode if originally set*/
	if (prev_auto_mode) {
		/*DBG_MSG("restore Auto Mode\n");*/
		lli_lo_wt(MIPI_LLI_PA_CSA_PA_SET, B_CSA_PA_AUTO_MODE);
		lli_rm_wt(MIPI_LLI_PA_CSA_PA_SET, B_CSA_PA_AUTO_MODE);
	}


	if (r == CONFIG_UPDATE_SUCCESS) {
		#if DBG_CONFIG_UPDATE
		DBG_MSG("9.ConfigUpdate done successfully\n");
		#endif
		return r;
	} else {
		ERR_MSG("Error:ConfigUpdate done fail\n");
	}

	/* config_update_err: */
	ERR_MSG("Error:ConfigUpdate done fail\n");
	return r;
}

int mipilli_change_memmap(u32 from_addr, u32 to_addr, u32 size)
{
	u8 i;
	u32 size_adjust = 0x01;
	u32 addr_mask;

	/* check if size is 2^n (n >= 12)*/
	for (i = 0; i < 32; i++, size_adjust <<= 1) {
		if (size_adjust >= size)
			break;
	}

	if (size_adjust != size) {
		/*
		 * DBG_MSG(
		 * "Warning!! changeMemmap size
		 * must be 2^N(N>=12)!!\n");
		 *
		 */
		size = size_adjust;
		/*DBG_MSG("Adjust size, now it is %d Bytes\n", size);*/
	}

	addr_mask = size - 1;
	if (from_addr & addr_mask) {
		/*DBG_MSG(
		 "Warning!! changeMemmap
		 from_addr must be aligned to size!!\n");
		 */
		from_addr &= ~addr_mask;
		/*DBG_MSG("Adjust from_addr, now it is %08x\n", from_addr);
		 */
	}

	if (to_addr & addr_mask) {
		/*DBG_MSG(
		 * "Warning!! changeMemmap to_addr
		 * must be aligned to size!!\n");
		 */
		to_addr &= ~addr_mask;
		/*DBG_MSG("Adjust to_addr, now it is %08x\n", to_addr);*/
	}

	lli_lo_wt(MIPI_USER_CTRL_MADREMAPUM1, (~addr_mask) >> 12);
	lli_lo_wt(MIPI_USER_CTRL_MADREMAPFLT1, from_addr >> 12);
	lli_lo_wt(MIPI_USER_CTRL_MADREMAP1, to_addr >> 12);
	return 0;
}
