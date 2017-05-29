/*
 * mipilli_help.c F_MIPILLI_LP Controller Driver
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

#include <linux/io.h>
#include "mipilli_api.h"

void set_lo_cfg(struct lli_cfg *pcfg)
{
	u8 i = 0;

	/* DBG("set_lo_cfg\n"); */
	/*
	 * mphy config not set here, use PLU
	 * link config not set here, use PLU
	 * PA config not set here, use PLU
	*/

	/* user PA config. exc. shadow register not set here, use PLU */
	/* retry buf time */
	lli_lo_wt(MIPI_USER_PA_RBTC, pcfg->usr_pacfg.rbtc);

	/* NACKThreshold */
	lli_lo_wt(MIPI_USER_PA_NACKT, pcfg->usr_pacfg.nackt);
	lli_lo_wt(MIPI_USER_PA_DSKWTC, pcfg->usr_pacfg.dskwtc);
	lli_lo_wt(MIPI_USER_PA_PATXSV, pcfg->usr_pacfg.patxsv);

	/* LLI control config */
	lli_lo_wt(CTRL_LLTCDISABLE, pcfg->lk_ctlcfg.ll_tc_disable);
	lli_lo_wt(CTRL_BETCDISABLE, pcfg->lk_ctlcfg.be_tc_disable);
	lli_lo_wt(CTRL_TLADDRMODE, pcfg->lk_ctlcfg.tl_addr_mode);

	/* user control config */
	lli_lo_wt(MIPI_USER_CTRL_CLDRSTAUTO, pcfg->usr_ctlcfg.cldrst_auto);
	lli_lo_wt(MIPI_USER_CTRL_ROEAUTO, pcfg->usr_ctlcfg.roe_auto);
	lli_lo_wt(MIPI_USER_CTRL_INTUM, pcfg->usr_ctlcfg.intum);
	lli_lo_wt(MIPI_USER_CTRL_AXICFG, pcfg->usr_ctlcfg.axi_cofg);
	lli_lo_wt(MIPI_USER_CTRL_TRANARB, pcfg->usr_ctlcfg.tranarb);
	lli_lo_wt(MIPI_USER_CTRL_AXIAPBCTL, pcfg->usr_ctlcfg.axiapb_ctl);
	lli_lo_wt(MIPI_USER_CTRL_RSTR, pcfg->usr_ctlcfg.rstr);


	for (i = 0; i < 8; i++) {

		lli_lo_wt(MIPI_USER_CTRL_MADREMAP1 + i * 4,
		    pcfg->usr_ctlcfg.m_ad_remapConfig[i].remap);

		lli_lo_wt(MIPI_USER_CTRL_MADREMAPUM1 + i * 4,
		    pcfg->usr_ctlcfg.m_ad_remapConfig[i].unmask);

		lli_lo_wt(MIPI_USER_CTRL_MADREMAPFLT1 + i * 4,
		    pcfg->usr_ctlcfg.m_ad_remapConfig[i].filter);

		lli_lo_wt(MIPI_USER_CTRL_SADREMAP1 + i * 4,
		    pcfg->usr_ctlcfg.s_ad_remapConfig[i].remap);

		lli_lo_wt(MIPI_USER_CTRL_SADREMAPUM1 + i * 4,
		    pcfg->usr_ctlcfg.s_ad_remapConfig[i].unmask);

		lli_lo_wt(MIPI_USER_CTRL_SADREMAPFLT1 + i * 4,
		    pcfg->usr_ctlcfg.s_ad_remapConfig[i].filter);

		lli_lo_wt(MIPI_USER_CTRL_SADPSTWUNM1 + i * 4,
		    pcfg->usr_ctlcfg.s_ad_postwConfig[i].unmask);

		lli_lo_wt(MIPI_USER_CTRL_SADPSTWFLT1 + i * 4,
		    pcfg->usr_ctlcfg.s_ad_postwConfig[i].filter);
	}


	lli_lo_wt(MIPI_USER_CTRL_MAWUSRSW7_0,
	    (pcfg->usr_ctlcfg.ll_m_awuserSwap[7] << 28) |
	    (pcfg->usr_ctlcfg.ll_m_awuserSwap[6] << 24) |
	    (pcfg->usr_ctlcfg.ll_m_awuserSwap[5] << 20) |
	    (pcfg->usr_ctlcfg.ll_m_awuserSwap[4] << 16) |
	    (pcfg->usr_ctlcfg.ll_m_awuserSwap[3] << 12) |
	    (pcfg->usr_ctlcfg.ll_m_awuserSwap[2] << 8)  |
	    (pcfg->usr_ctlcfg.ll_m_awuserSwap[1] << 4)  |
	    (pcfg->usr_ctlcfg.ll_m_awuserSwap[0] << 0));

	lli_lo_wt(MIPI_USER_CTRL_MAWUSRSW8,
	    pcfg->usr_ctlcfg.ll_m_awuserSwap[8]);

	lli_lo_wt(MIPI_USER_CTRL_MARUSRSW7_0,
	    (pcfg->usr_ctlcfg.ll_m_aruserSwap[7] << 28) |
	    (pcfg->usr_ctlcfg.ll_m_aruserSwap[6] << 24) |
	    (pcfg->usr_ctlcfg.ll_m_aruserSwap[5] << 20) |
	    (pcfg->usr_ctlcfg.ll_m_aruserSwap[4] << 16) |
	    (pcfg->usr_ctlcfg.ll_m_aruserSwap[3] << 12) |
	    (pcfg->usr_ctlcfg.ll_m_aruserSwap[2] << 8)  |
	    (pcfg->usr_ctlcfg.ll_m_aruserSwap[1] << 4)  |
	    (pcfg->usr_ctlcfg.ll_m_aruserSwap[0] << 0));

	lli_lo_wt(MIPI_USER_CTRL_MARUSRSW8,
	  pcfg->usr_ctlcfg.ll_m_aruserSwap[8]);

	lli_lo_wt(MIPI_USER_CTRL_SAWUSRSW7_0,
	    (pcfg->usr_ctlcfg.ll_s_awuserSwap[7] << 28) |
	    (pcfg->usr_ctlcfg.ll_s_awuserSwap[6] << 24) |
	    (pcfg->usr_ctlcfg.ll_s_awuserSwap[5] << 20) |
	    (pcfg->usr_ctlcfg.ll_s_awuserSwap[4] << 16) |
	    (pcfg->usr_ctlcfg.ll_s_awuserSwap[3] << 12) |
	    (pcfg->usr_ctlcfg.ll_s_awuserSwap[2] << 8)  |
	    (pcfg->usr_ctlcfg.ll_s_awuserSwap[1] << 4)  |
	    (pcfg->usr_ctlcfg.ll_s_awuserSwap[0] << 0));

	lli_lo_wt(MIPI_USER_CTRL_SAWUSRSW8,
	  pcfg->usr_ctlcfg.ll_s_awuserSwap[8]);

	lli_lo_wt(MIPI_USER_CTRL_SARUSRSW7_0,
	    (pcfg->usr_ctlcfg.ll_s_aruserSwap[7] << 28) |
	    (pcfg->usr_ctlcfg.ll_s_aruserSwap[6] << 24) |
	    (pcfg->usr_ctlcfg.ll_s_aruserSwap[5] << 20) |
	    (pcfg->usr_ctlcfg.ll_s_aruserSwap[4] << 16) |
	    (pcfg->usr_ctlcfg.ll_s_aruserSwap[3] << 12) |
	    (pcfg->usr_ctlcfg.ll_s_aruserSwap[2] << 8)  |
	    (pcfg->usr_ctlcfg.ll_s_aruserSwap[1] << 4)  |
	    (pcfg->usr_ctlcfg.ll_s_aruserSwap[0] << 0));

	lli_lo_wt(MIPI_USER_CTRL_SARUSRSW8,
	    pcfg->usr_ctlcfg.ll_s_aruserSwap[8]);

	lli_lo_wt(MIPI_USER_CTRL_MAWUSREN,
	    pcfg->usr_ctlcfg.mawusr_en);
	lli_lo_wt(MIPI_USER_CTRL_MARUSREN,
	    pcfg->usr_ctlcfg.marusr_en);
	lli_lo_wt(MIPI_USER_CTRL_SAWUSREN,
	    pcfg->usr_ctlcfg.sawusr_en);
	lli_lo_wt(MIPI_USER_CTRL_SARUSREN,
	    pcfg->usr_ctlcfg.sarusr_en);

	for (i = 0; i < 5; i++) {
		lli_lo_wt(MIPI_USER_CTRL_LSSTSSW5_0 + i * 4,
		    (pcfg->usr_ctlcfg.sig_stat_sw[5 + i * 6] << 25) |
		    (pcfg->usr_ctlcfg.sig_stat_sw[4 + i * 6] << 20) |
		    (pcfg->usr_ctlcfg.sig_stat_sw[3 + i * 6] << 15) |
		    (pcfg->usr_ctlcfg.sig_stat_sw[2 + i * 6] << 10) |
		    (pcfg->usr_ctlcfg.sig_stat_sw[1 + i * 6] << 5) |
		    (pcfg->usr_ctlcfg.sig_stat_sw[0 + i * 6] << 0));

		lli_lo_wt(MIPI_USER_CTRL_LSSETSW5_0 + i * 4,
		    (pcfg->usr_ctlcfg.sig_set_sw[5 + i * 6] << 25) |
		    (pcfg->usr_ctlcfg.sig_set_sw[4 + i * 6] << 20) |
		    (pcfg->usr_ctlcfg.sig_set_sw[3 + i * 6] << 15) |
		    (pcfg->usr_ctlcfg.sig_set_sw[2 + i * 6] << 10) |
		    (pcfg->usr_ctlcfg.sig_set_sw[1 + i * 6] << 5) |
		    (pcfg->usr_ctlcfg.sig_set_sw[0 + i * 6] << 0));
	}


	lli_lo_wt(MIPI_USER_CTRL_LSSTSSW5_0 + i * 4,
	    (pcfg->usr_ctlcfg.sig_set_sw[1 + i * 6] << 5) |
	    (pcfg->usr_ctlcfg.sig_set_sw[0 + i * 6] << 0));

	lli_lo_wt(MIPI_USER_CTRL_LSSETSW5_0 + i * 4,
	    (pcfg->usr_ctlcfg.sig_set_sw[1 + i * 6] << 5) |
	    (pcfg->usr_ctlcfg.sig_set_sw[0 + i * 6] << 0));

	lli_lo_wt(MIPI_USER_CTRL_LSSTSEN, pcfg->usr_ctlcfg.lssts_en);
	lli_lo_wt(MIPI_USER_CTRL_LSSTSOR, pcfg->usr_ctlcfg.lssts_or);
	lli_lo_wt(MIPI_USER_CTRL_LSSETEN, pcfg->usr_ctlcfg.lsset_en);
	lli_lo_wt(MIPI_USER_CTRL_LSSETUM, pcfg->usr_ctlcfg.lsset_um);
}



void set_mphy_attr(struct lli_cfg *pcfg)
{
	u32 op_mode = 0;
	u32 pwm_gear = 0;

	pcfg->tx_cfg.amplitude = LARGE_AMPLITUDE;

	if ((pcfg->tx_cfg.op_mode == LS_MODE) &&
		(pcfg->tx_cfg.pwm_gear <= PWM_G4)) {
		pcfg->tx_cfg.slew_rate = 0xC0;

	} else {
		pcfg->tx_cfg.slew_rate = 0x01;
	}

	pcfg->tx_cfg.sync_range = COARSE;
	pcfg->tx_cfg.sync_length = 0x05;

	if (pcfg->tx_cfg.op_mode == HS_MODE) {
		if (pcfg->tx_cfg.hs_gear == HS_G2)
			pcfg->tx_cfg.sync_length = 0x06;
		else if (pcfg->tx_cfg.hs_gear == HS_G3)
			pcfg->tx_cfg.sync_length = 0x07;
	}

	pcfg->tx_cfg.hs_prepare_length = 0x0F;
	pcfg->tx_cfg.ls_prepare_length = 0x04;
	pcfg->tx_cfg.lcc_enable = false;
	pcfg->tx_cfg.hs_unterminated = 0x20;
	pcfg->tx_cfg.hs_unterminated = false;

	if ((pcfg->tx_cfg.op_mode == LS_MODE) &&
		(pcfg->tx_cfg.pwm_gear >= PWM_G5)) {
		pcfg->tx_cfg.ls_terminated = true;
	} else {
		pcfg->tx_cfg.ls_terminated = false;
	}

	pcfg->tx_cfg.min_activate_time = 0x08;
	pcfg->tx_cfg.lsg6_sync_range = COARSE;
	pcfg->tx_cfg.lsg6_sync_length = 0x01;

	op_mode = pcfg->tx_cfg.op_mode;
	pwm_gear = pcfg->tx_cfg.pwm_gear;
	if ((op_mode == LS_MODE) && (pwm_gear >= PWM_G5))
		pcfg->rx_cfg.ls_terminated = true;
	else
		pcfg->rx_cfg.ls_terminated = false;

	pcfg->rx_cfg.hs_unterminated = false;
}


/*
 * M-Phy can not
 * communicate in PWM_G0 ~ PWM_G4 modes.
 * Also, PWM_G5 has CDC(asynchronous) problem.
 * Now, ignore PWM!!
 * Therefore, setting HS-G1/G2
 * must be done BEFORE MOUNTING.
 * Both of Master/Slave have the same value.
 * ToDo: In future, this bug will be fixed.
 * These code must be removed
 * when it is fixed.
*/
void set_err_wacfg(struct lli_cfg *pcfg,
		bool askpam, bool pamdef)
{

	/* Only lane0 must be set here */
	u32 updated_lanes = 0x1001;


	lli_lo_wt(MIPI_MPHY0_TXMODE, pcfg->tx_cfg.op_mode);

	lli_lo_wt(MIPI_MPHY0_TXHSRATE_SERIES, pcfg->tx_cfg.rate_series);

	lli_lo_wt(MIPI_MPHY0_TXHSGEAR, pcfg->tx_cfg.hs_gear);

	lli_lo_wt(MIPI_MPHY0_TXPWMGEAR, pcfg->tx_cfg.pwm_gear);

	lli_lo_wt(MIPI_MPHY0_TXAMPLITUDE, pcfg->tx_cfg.amplitude);

	lli_lo_wt(MIPI_MPHY0_TXHSSLEWRATE, pcfg->tx_cfg.slew_rate);

	lli_lo_wt(MIPI_MPHY0_TXHSSYNC_LENGTH,
	((pcfg->tx_cfg.sync_range << 6) | pcfg->tx_cfg.sync_length));

	lli_lo_wt(MIPI_MPHY0_TXHSPREPLENGTH, pcfg->tx_cfg.hs_prepare_length);

	lli_lo_wt(MIPI_MPHY0_TXLSPREPLENGTH, pcfg->tx_cfg.ls_prepare_length);

	lli_lo_wt(MIPI_MPHY0_TXLCCENABLE, pcfg->tx_cfg.lcc_enable);

	lli_lo_wt(MIPI_MPHY0_TXPWMCLOSEXT, pcfg->tx_cfg.hs_unterminated);

	lli_lo_wt(MIPI_MPHY0_TXHSUNTERM, pcfg->tx_cfg.hs_unterminated);

	lli_lo_wt(MIPI_MPHY0_TXLSTERM, pcfg->tx_cfg.ls_terminated);

	lli_lo_wt(MIPI_MPHY0_TXMINACTIVATTIME, pcfg->tx_cfg.min_activate_time);

	lli_lo_wt(MIPI_MPHY0_TXPWMG6G7SYNCLENGTH,
	    (pcfg->tx_cfg.lsg6_sync_range << 6) |
	    pcfg->tx_cfg.lsg6_sync_length);

	lli_lo_wt(MIPI_MPHY0_RXMODE, pcfg->rx_cfg.op_mode);

	lli_lo_wt(MIPI_MPHY0_RXHSRATE_SERIES, pcfg->rx_cfg.rate_series);

	lli_lo_wt(MIPI_MPHY0_RXHSGEAR, pcfg->rx_cfg.hs_gear);

	lli_lo_wt(MIPI_MPHY0_RXPWMGEAR, pcfg->rx_cfg.pwm_gear);

	lli_lo_wt(MIPI_MPHY0_RXLSTERM, pcfg->rx_cfg.ls_terminated);

	lli_lo_wt(MIPI_MPHY0_RXHSUNTERM, pcfg->rx_cfg.hs_unterminated);

	/* Set LLI PA and User PA shadow here */

	/* LLI PA config use PLU */
	/* DBG_MSG("LLI PA configuration for WA\n"); */
	lli_lo_wt(MIPI_LLI_PA_WTSTARTVAL, pcfg->pa_cfig.wt_start_value);

	lli_lo_wt(MIPI_LLI_PA_MIN_SAVE_CONFIG,
	    pcfg->pa_cfig.pa_min_save_config);

	lli_lo_wt(MIPI_LLI_PA_WORSTCASE_RTT,
	    pcfg->pa_cfig.pa_worst_case_rtt);

	lli_lo_wt(MIPI_LLI_PA_DRV_TACTV_DUR,
	    pcfg->pa_cfig.drive_tactive_duration);

	lli_lo_wt(MIPI_LLI_PA_MK0INS_ENABLE,
	    pcfg->pa_cfig.marker0_insertion);

	lli_lo_wt(MIPI_LLI_PA_PHITRCVCOUNTEN,
	    pcfg->pa_cfig.phit_rec_cont_en);

	lli_lo_wt(MIPI_LLI_PA_PHITERRCOUNTEN,
	    pcfg->pa_cfig.phit_err_cout_en);

	lli_lo_wt(MIPI_LLI_PA_WORSTCASE_RTT,
	    pcfg->pa_cfig.pa_worst_case_rtt);

	lli_lo_wt(MIPI_USER_PA_NACKT,
	    pcfg->usr_pacfg.nackt);

	/* user PA config. shadow register use PLU */
	/*DBG_MSG("User PA configuration for WA\n");*/
	lli_lo_wt(MIPI_USER_PA_AGPRM, pcfg->usr_pacfg.aging_parameter);
	/* ToDo: This value must be updated by PLU
	 * To overcome this, debug register must be used!!
	*/
	set_lo_timeout(pcfg);
	local_set_deskew_to(pcfg);

	/* set PA config_update
	 * Config_Update[11:0] shall control the M-TX [11:0].
	 * Config_Update[23:12] shall control the M-RX [11:0].
	 * DBG_MSG("Set PA config_update\n");
	*/
	lli_lo_wt(MIPI_LLI_PA_CONFIG_UPDATE, updated_lanes);
}

void mphy_write_reg(u32 addr_base, u8 targ, u8 val,
	u8 txlanes_s, u8 txlanes_d,
	u8 rxlanes_s, u8 rxlanes_d)
{

	s8 i = 0;

	lli_lo_wt(MIPI_MPHY0_TXHIBERN8CTRL + i * 0x400, EXIT);
	lli_rm_wt(MIPI_MPHY0_TXHIBERN8CTRL + i * 0x400, EXIT);

	if (targ == MPHY_WRITE_TAG_LOCAL) {
		for (i = txlanes_s; i < txlanes_d; i++)
			lli_lo_wt(addr_base + i * 0x400, val);

		for (i = rxlanes_s; i < rxlanes_d; i++)
			lli_lo_wt(addr_base + i * 0x400, val);

	} else if (targ == MPHY_WRITE_TAG_REMOTE) {
		for (i = txlanes_s; i < txlanes_d; i++)
			lli_rm_wt(addr_base + i * 0x400, val);

		for (i = rxlanes_s; i < txlanes_d; i++)
			lli_rm_wt(addr_base + i * 0x400, val);
	} else {
		for (i = txlanes_s; i < txlanes_d; i++) {
			lli_rm_wt(addr_base + i * 0x400, val);
			lli_lo_wt(addr_base + i * 0x400, val);
		}

		for (i = rxlanes_s; i < rxlanes_d; i++) {
			lli_rm_wt(addr_base + i * 0x400, val);
			lli_lo_wt(addr_base + i * 0x400, val);
		}
	}
}


bool
update_shadow_remote(struct lli_cfg *pcfg, u8 txlan, u8 rxlan)
{
	bool result = true;

	DBG_MSG("updateShadowAndRemote start\n");


	/* DBG_MSG("Mphy TX configuration\n"); */
	/* ---Mphy TX configuration --- */

	mphy_write_reg(MIPI_MPHY0_TXMODE,
	    MPHY_WRITE_TAG_LOCAL|
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->tx_cfg.op_mode,
	    0, txlan, 0, 0);

	mphy_write_reg(MIPI_MPHY0_TXHSRATE_SERIES,
	    MPHY_WRITE_TAG_LOCAL|
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->tx_cfg.rate_series,
	    0, txlan, 0, 0);

	mphy_write_reg(MIPI_MPHY0_TXHSGEAR,
	    MPHY_WRITE_TAG_LOCAL|
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->tx_cfg.hs_gear,
	    0, txlan, 0, 0);

	mphy_write_reg(MIPI_MPHY0_TXPWMGEAR,
	    MPHY_WRITE_TAG_LOCAL|
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->tx_cfg.pwm_gear,
	    0, txlan, 0, 0);

	mphy_write_reg(MIPI_MPHY0_TXAMPLITUDE,
	    MPHY_WRITE_TAG_LOCAL|
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->tx_cfg.amplitude,
	    0, txlan, 0, 0);

	mphy_write_reg(MIPI_MPHY0_TXHSSLEWRATE,
	    MPHY_WRITE_TAG_LOCAL|
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->tx_cfg.slew_rate,
	    0, txlan, 0, 0);

	mphy_write_reg(MIPI_MPHY0_TXSYNCSOURCE,
	    MPHY_WRITE_TAG_LOCAL|
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->tx_cfg.sync_source,
	    0, txlan, 0, 0);


	mphy_write_reg(MIPI_MPHY0_TXHSSYNC_LENGTH,
	    MPHY_WRITE_TAG_LOCAL|
	    MPHY_WRITE_TAG_REMOTE,
	    (pcfg->tx_cfg.sync_range << 6)|
	    pcfg->tx_cfg.sync_length,
	    0, txlan, 0, 0);

	mphy_write_reg(MIPI_MPHY0_TXHSPREPLENGTH,
	  MPHY_WRITE_TAG_LOCAL|
	  MPHY_WRITE_TAG_REMOTE,
	  pcfg->tx_cfg.hs_prepare_length,
	  0, txlan, 0, 0);

	mphy_write_reg(MIPI_MPHY0_TXLSPREPLENGTH,
	    MPHY_WRITE_TAG_LOCAL|
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->tx_cfg.ls_prepare_length,
	    0, txlan, 0, 0);

	mphy_write_reg(MIPI_MPHY0_TXLCCENABLE,
	    MPHY_WRITE_TAG_LOCAL|
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->tx_cfg.lcc_enable,
	    0, txlan, 0, 0);

	mphy_write_reg(MIPI_MPHY0_TXPWMCLOSEXT,
	    MPHY_WRITE_TAG_LOCAL|
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->tx_cfg.hs_unterminated,
	    0, txlan, 0, 0);

	mphy_write_reg(MIPI_MPHY0_TXBYPASS8B10B,
	    MPHY_WRITE_TAG_LOCAL|
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->tx_cfg.bypass8B10B,
	    0, txlan, 0, 0);

	mphy_write_reg(MIPI_MPHY0_TXPOLARITY,
	    MPHY_WRITE_TAG_LOCAL|
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->tx_cfg.polarity,
	    0, txlan, 0, 0);

	mphy_write_reg(MIPI_MPHY0_TXHSUNTERM,
	    MPHY_WRITE_TAG_LOCAL|
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->tx_cfg.hs_unterminated,
	    0, txlan, 0, 0);

	mphy_write_reg(MIPI_MPHY0_TXLSTERM,
	    MPHY_WRITE_TAG_LOCAL|
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->tx_cfg.ls_terminated,
	    0, txlan, 0, 0);

	mphy_write_reg(MIPI_MPHY0_TXMINACTIVATTIME,
	    MPHY_WRITE_TAG_LOCAL|
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->tx_cfg.min_activate_time,
	    0, txlan, 0, 0);

	mphy_write_reg(MIPI_MPHY0_TXPWMG6G7SYNCLENGTH,
	    MPHY_WRITE_TAG_LOCAL |
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->tx_cfg.lsg6_sync_length,
	    0, txlan, 0, 0);


	/* --Mphy RX configuration-- */

	/* DBG_MSG("Mphy RX configuration\n"); */

	mphy_write_reg(MIPI_MPHY0_RXMODE,
	    MPHY_WRITE_TAG_LOCAL|
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->rx_cfg.op_mode,
	    0, 0, 0, rxlan);

	mphy_write_reg(MIPI_MPHY0_RXHSRATE_SERIES,
	    MPHY_WRITE_TAG_LOCAL|
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->rx_cfg.rate_series,
	    0, 0, 0, rxlan);

	mphy_write_reg(MIPI_MPHY0_RXHSGEAR,
	    MPHY_WRITE_TAG_LOCAL|
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->rx_cfg.hs_gear,
	    0, 0, 0, rxlan);

	mphy_write_reg(MIPI_MPHY0_RXPWMGEAR,
	    MPHY_WRITE_TAG_LOCAL|
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->rx_cfg.pwm_gear,
	    0, 0, 0, rxlan);

	mphy_write_reg(MIPI_MPHY0_RXLSTERM,
	    MPHY_WRITE_TAG_LOCAL |
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->rx_cfg.ls_terminated,
	    0, 0, 0, rxlan);

	mphy_write_reg(MIPI_MPHY0_RXHSUNTERM,
	    MPHY_WRITE_TAG_LOCAL|
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->rx_cfg.hs_unterminated,
	    0, 0, 0, rxlan);

	mphy_write_reg(MIPI_MPHY0_RXBYPASS8B10B,
	    MPHY_WRITE_TAG_LOCAL|
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->rx_cfg.bypass8B10B,
	    0, 0, 0, rxlan);

	mphy_write_reg(MIPI_MPHY0_RXTERMFORCEEN,
	    MPHY_WRITE_TAG_LOCAL|
	    MPHY_WRITE_TAG_REMOTE,
	    pcfg->rx_cfg.force_term,
	    0, 0, 0, rxlan);



	/* LLI PA config use PLU */
	/* DBG_MSG("LLI PA configuration\n"); */
	lli_lo_wt(MIPI_LLI_PA_MK0INS_ENABLE, pcfg->pa_cfig.marker0_insertion);
	lli_rm_wt(MIPI_LLI_PA_MK0INS_ENABLE, pcfg->pa_cfig.marker0_insertion);
	lli_lo_wt(MIPI_LLI_PA_WTSTARTVAL, pcfg->pa_cfig.wt_start_value);
	lli_rm_wt(MIPI_LLI_PA_WTSTARTVAL, pcfg->pa_cfig.wt_start_value);
	lli_lo_wt(MIPI_LLI_PA_PHITRCVCOUNTEN, pcfg->pa_cfig.phit_rec_cont_en);
	lli_rm_wt(MIPI_LLI_PA_PHITRCVCOUNTEN, pcfg->pa_cfig.phit_rec_cont_en);
	lli_lo_wt(MIPI_LLI_PA_PHITERRCOUNTEN, pcfg->pa_cfig.phit_err_cout_en);
	lli_rm_wt(MIPI_LLI_PA_PHITERRCOUNTEN, pcfg->pa_cfig.phit_err_cout_en);

	lli_lo_wt(MIPI_LLI_PA_MIN_SAVE_CONFIG,
	    pcfg->pa_cfig.pa_min_save_config);
	lli_rm_wt(MIPI_LLI_PA_MIN_SAVE_CONFIG,
	    pcfg->pa_cfig.pa_min_save_config);

	lli_lo_wt(MIPI_LLI_PA_WORSTCASE_RTT, pcfg->pa_cfig.pa_worst_case_rtt);
	lli_rm_wt(MIPI_LLI_PA_WORSTCASE_RTT, pcfg->pa_cfig.pa_worst_case_rtt);

	lli_lo_wt(MIPI_LLI_PA_DRV_TACTV_DUR,
	    pcfg->pa_cfig.drive_tactive_duration);
	lli_rm_wt(MIPI_LLI_PA_DRV_TACTV_DUR,
	    pcfg->pa_cfig.drive_tactive_duration);

	lli_lo_wt(MIPI_LLI_PA_PHYTESTCONFIG,
	    pcfg->pa_cfig.pa_phy_test_config_master);
	lli_rm_wt(MIPI_LLI_PA_PHYTESTCONFIG,
	    pcfg->pa_cfig.pa_phy_test_config_slasve);

	/* user PA config. shadow register use PLU */
	/* DBG_MSG("User PA configuration\n"); */
	lli_lo_wt(MIPI_USER_PA_AGPRM, pcfg->usr_pacfg.aging_parameter);
	lli_rm_wt(MIPI_USER_PA_AGPRM, pcfg->usr_pacfg.aging_parameter);
	/* ToDo, check if Aging param may be set by SVC!! */

	DBG_MSG("updateShadowAndRemote done successfully\n");

	return result;
}



bool phy_link_update(bool unmount, u8 ch_num)
{

	/* bool result = true; */
	u32 ret = 0, retry_cnt = RETRY_COUNTER;
	u8 r = 0 , r1 = 0;

	struct lli_data *the_lli;
	/* DBG_MSG("PLU_start\n"); */

	the_lli = &gp_mb86s70_lli_dev->chan_data[ch_num];
	/* set PA config_update */
	/* DBG_MSG("Set PA config_update\n"); */

	/* Update Configuration Now */
	if (!unmount)
		lli_lo_wt(MIPI_LLI_PA_CSA_PA_SET, B_CSA_PA_LINKUPDATECONFIG);
	else
		lli_lo_wt(CTRL_CSASYSTEMCLR, B_CSA_LLI_MOUNT_CTRL);

	ret = wait_for_completion_timeout(&the_lli->msg_cmp, CMP_TIMEOUT);
	INIT_COMPLETION(the_lli->msg_cmp);

	if (!ret) {
		/* DBG_MSG("PLU(Physical Link Update) failure(timeout)\n"); */
		/* result = false; */
		/* return result; */
		return false;
	}

	if (the_lli->lli_interrupt_status & B_PLUFIN) {

		/* DBG_MSG("Master Mount done successfully\n"); */

		while (!r) {
			r1 = lli_lo_rd(PA_CSA_PA_STATUS);
			r = (r1 & B_CSA_PA_LINKUPDATECONFIG);
			retry_cnt = retry_cnt - 1;
			if (retry_cnt <= 0)
				break;
			/* msleep(20); */
		}
	}

	/* result = retry_cnt > 0; */
	/* DBG_MSG("PLU done successfully\n");*/
	/* return result; */
	return retry_cnt > 0;
}


/* return time conversion result,
 * from SI to Tx/Rx_Symbolclk
 */
u32 unit_conv_si_to_symbol(u32 sival)
{
	u32 val = sival / 2;
	/* round-up */
	if ((val * 2) < sival)
		val++;

	return val;
}

/* return time coversion result,
 * from Tx/Rx_Symbolclk to SI
 */
u32 unit_conv_symbol_to_si(u32 symbolval)
{
	u32 val = symbolval * 2;
	return val;
}

/*
 * return time conversion result, from SI to PCLK
 * PCLK is assumed to be 40 MHz.
 * Warning!! Master side can use this function.
*/
u32 unit_conv_si_to_pclk(u32 sival, struct lli_cfg *pcfg)
{
	static
	u32 mode_freq_in_khz[14] = { /* Mode/Gear indexed */
		10, 3000, 6000, 12000,/* LS-G0,1,2,3 */
		24000, 48000, 96000, 192000,/* LS-G4,5,6,7 */
		1248000, 1457600, 2496000, 2915200,/* HS-G1A, G1B, G2A, G2B */
		4992000, 5830400,/* HS-G3A, G3B */
	};

	u8 index = 0;
	enum MPHY_OP_MODE mode = pcfg->tx_cfg.op_mode;
	u32 khz = 0;
	u32 val = 0;

	if (mode == HS_MODE) {
		enum MPHY_HS_GEAR gear = pcfg->tx_cfg.hs_gear;
		enum MPHY_HS_RATE_SERIES rate = pcfg->tx_cfg.rate_series;
		index |= 0x08;
		index |= (gear - 1) << 1;

		if (rate == B)
			index |= 0x01;

	} else { /* LS */
		enum MPHY_PWM_GEAR gear = pcfg->tx_cfg.pwm_gear;
		index |= gear;
	}

	khz = mode_freq_in_khz[index];

	#if 0
	/* PCLK 40 MHz assumed */
	u32 ratio = (khz / 10) / 40 * 1000;
	u32 val = sival / ratio;
	/* round-up */
	if ((val * ratio) < sival)
		val++;
	#endif

	val = (sival * (40 * 1000) * 10) / khz;
	/* round-up */
	if ((val * khz) < (sival * (40 * 1000) * 10))
		val++;

	return val;

}


u32 get_sync_len(u8 sync_lencap)
{
	u32 val = 0;

	if ((sync_lencap & 0xC0) == 0x00)
		val = sync_lencap & 0x0F;
	else {

		if ((sync_lencap & 0x0F) > 14)
			val = 1 << 14;
		else
			val = 1 << (sync_lencap & 0x0F);

	}
	return val;
}


/*
 * Plz ref mipi-m-phy :
 * Table 6 PREPARE and SYNC Attribute and
 * Dependent Parameter Values
 * return sync_length in SI using config value and
 * SVC returned value.
 * Warning!! Only MOUNTED Master side can use this
 * function.
*/
u32 worstsync_lengthInSI(struct lli_cfg *pcfg)
{
	u32 val = 0;
	u8 rxsynccap = 0;
	u8 rxsynccaplocal = 0;
	u32 sync_len = 0, sync_lenLocal = 0;

	if (pcfg->tx_cfg.op_mode == LS_MODE) {
		if (pcfg->tx_cfg.pwm_gear < PWM_G6)
			val = 0;
		else {
			rxsynccap = lli_rm_rd(MIPI_MPHY0_RXLSG67sync_len_CAP);
			rxsynccaplocal =
			  lli_lo_rd(MIPI_MPHY0_RXLSG67sync_len_CAP);

			sync_len = get_sync_len(rxsynccap);
			sync_lenLocal = get_sync_len(rxsynccaplocal);

			if (sync_len > sync_lenLocal)
				val = sync_len;
			else
				val = sync_lenLocal;
		}
	} else { /* HS */
		switch (pcfg->tx_cfg.hs_gear) {
		case HS_G1:
			rxsynccap =
			    lli_rm_rd(MIPI_MPHY0_RXHSG1sync_len_CAP);
			rxsynccaplocal =
			    lli_lo_rd(MIPI_MPHY0_RXHSG1sync_len_CAP);
			break;
		case HS_G2:
			rxsynccap = lli_rm_rd(MIPI_MPHY0_RXHSG2sync_len_CAP);
			rxsynccaplocal =
			    lli_lo_rd(MIPI_MPHY0_RXHSG2sync_len_CAP);
			break;
		/* case HS_G3: */
		default:
			rxsynccap =
			    lli_rm_rd(MIPI_MPHY0_RXHSG3sync_len_CAP);
			rxsynccaplocal =
			    lli_lo_rd(MIPI_MPHY0_RXHSG3sync_len_CAP);
			break;
		}
		sync_len = get_sync_len(rxsynccap);
		sync_lenLocal = get_sync_len(rxsynccaplocal);

		if (sync_len > sync_lenLocal)
			val = sync_len;
		else
			val = sync_lenLocal;

	}
	return val;
}

static u32 worstLocalsync_lengthInSI(struct lli_cfg *pcfg)
{
	u32 val = 0;
	u8 rxsynccap = 0;
	u32 sync_len = 0;

	if (pcfg->tx_cfg.op_mode == LS_MODE) {
		if (pcfg->tx_cfg.pwm_gear < PWM_G6)
			val = 0;
		else {
			rxsynccap = lli_lo_rd(MIPI_MPHY0_RXLSG67sync_len_CAP);
			sync_len = get_sync_len(rxsynccap);
			val = sync_len;
		}
	} else { /* HS */
		switch (pcfg->tx_cfg.hs_gear) {
		case HS_G1:
			rxsynccap =
			    lli_lo_rd(MIPI_MPHY0_RXHSG1sync_len_CAP);
			break;
		case HS_G2:
			rxsynccap =
			    lli_lo_rd(MIPI_MPHY0_RXHSG2sync_len_CAP);
			break;
		/* case HS_G3: */
		default:
			rxsynccap = lli_lo_rd(MIPI_MPHY0_RXHSG3sync_len_CAP);
			break;
		}
		sync_len = get_sync_len(rxsynccap);
		val = sync_len;
	}

	return val;
}


/*
 * plz ref mipi-m-phy : Table 6 PREPARE
 * and SYNC Attribute and Dependent Parameter Values
*/
u32 lspreplencaptosi(u8 cap, struct lli_cfg *pcfg)
{
	u32 val = 0;

	if (cap + pcfg->tx_cfg.pwm_gear >= 7)
		val = 1 << (cap + pcfg->tx_cfg.pwm_gear - 7);
	else
		val = 1;

	return val;
}

/*
 * Table 6 PREPARE and SYNC Attribute
 * and Dependent Parameter Values
*/
u32 hspreplencaptosi(u8 cap, struct lli_cfg *pcfg)
{
	u32 val = 0;
	val = cap * (1 << (pcfg->tx_cfg.hs_gear - 1));

	return val;
}

/*
 * return PrepareLength in SI using config value
 * and SVC remote value.
 * Warning!! Only MOUNTED Master side
 * can use this function.
*/
u32 worst_prep_length_in_si(struct lli_cfg *pcfg)
{
	u32 val = 0;
	u8 rxprepcap = 0, rxprepcaplocal = 0;

	if (pcfg->tx_cfg.op_mode == LS_MODE) {
		rxprepcap = lli_rm_rd(MIPI_MPHY0_RXLSPREPLEN_CAP);

		rxprepcaplocal = lli_lo_rd(MIPI_MPHY0_RXLSPREPLEN_CAP);

		if (rxprepcap < rxprepcaplocal)
			rxprepcap = rxprepcaplocal;

		val = lspreplencaptosi(rxprepcap, pcfg);
	} else { /* HS */
		switch (pcfg->tx_cfg.hs_gear) {
		case HS_G1:
			rxprepcap =
			    lli_rm_rd(MIPI_MPHY0_RXHSG1PREPLEN_CAP);
			rxprepcaplocal =
			    lli_lo_rd(MIPI_MPHY0_RXHSG1PREPLEN_CAP);
			break;

		case HS_G2:
			rxprepcap = lli_rm_rd(MIPI_MPHY0_RXHSG2PREPLEN_CAP);
			rxprepcaplocal =
			    lli_lo_rd(MIPI_MPHY0_RXHSG2PREPLEN_CAP);
			break;

		/* case HS_G3: */
		default:
			rxprepcap =
			    lli_rm_rd(MIPI_MPHY0_RXHSG3PREPLEN_CAP);
			rxprepcaplocal =
			    lli_lo_rd(MIPI_MPHY0_RXHSG3PREPLEN_CAP);
			break;
		}

		if (rxprepcap < rxprepcaplocal)
			rxprepcap = rxprepcaplocal;

		val = hspreplencaptosi(rxprepcap, pcfg);
	}
	return val;
}

u32 worst_local_prep_length_in_si(struct lli_cfg *pcfg)
{
	u32 val = 0;
	u8 rxprepcap = 0;

	if (pcfg->tx_cfg.op_mode == LS_MODE) {
		rxprepcap =
		    lli_lo_rd(MIPI_MPHY0_RXLSPREPLEN_CAP);
		val = lspreplencaptosi(rxprepcap, pcfg);
	} else { /* HS */
		switch (pcfg->tx_cfg.hs_gear) {
		case HS_G1:
			rxprepcap =
			    lli_lo_rd(MIPI_MPHY0_RXHSG1PREPLEN_CAP);
			break;
		case HS_G2:
			rxprepcap =
			    lli_lo_rd(MIPI_MPHY0_RXHSG2PREPLEN_CAP);
			break;
		/* case HS_G3: */
		default:
			rxprepcap =
			    lli_lo_rd(MIPI_MPHY0_RXHSG3PREPLEN_CAP);
			break;
		}
		val = hspreplencaptosi(rxprepcap, pcfg);
	}
	return val;
}

/*
 * SI:Symbol Interval (PHY layer)
 * return Sleep/Stall_NoConfig in SI using config
 * value and SVC remote value.
 * Warning!! Only MOUNTED Master side can use this function.
*/
u32 worst_min_sleep_stall(struct lli_cfg *pcfg)
{
	u32 val = 0, val_next = 0;

	/* Check Tx MinSleep */
	val = lli_rm_rd(MIPI_MPHY0_TXMINSLEEP_CAP);
	val_next = lli_lo_rd(MIPI_MPHY0_TXMINSLEEP_CAP);

	if (val_next > val)
		val = val_next;

	/* Check Tx MinStall */
	val_next = lli_rm_rd(MIPI_MPHY0_TXMINSTALL_CAP);
	if (val_next > val)
		val = val_next;

	val_next = lli_lo_rd(MIPI_MPHY0_TXMINSTALL_CAP);
	if (val_next > val)
		val = val_next;

	/* Check Rx MinSleep */
	val_next = lli_rm_rd(MIPI_MPHY0_RXMINSLEEP_CAP);
	if (val_next > val)
		val = val_next;

	val_next = lli_lo_rd(MIPI_MPHY0_RXMINSLEEP_CAP);
	if (val_next > val)
		val = val_next;

	/* Check Rx MinStall */
	val_next = lli_rm_rd(MIPI_MPHY0_RXMINSTALL_CAP);
	if (val_next > val)
		val = val_next;

	val_next = lli_lo_rd(MIPI_MPHY0_RXMINSTALL_CAP);
	if (val_next > val)
		val = val_next;

	return val;
}


u32 worst_local_min_sleep_stall(struct lli_cfg *pcfg)
{
	u32 val = 0, val_next = 0;

	/* Check Minute Tx MinSleep */
	val = lli_lo_rd(MIPI_MPHY0_TXMINSLEEP_CAP);

	/* Check Minute Tx MinStall */
	val_next = lli_lo_rd(MIPI_MPHY0_TXMINSTALL_CAP);
	if (val_next > val)
		val = val_next;

	/* Check Minute Rx MinSleep */
	val_next = lli_lo_rd(MIPI_MPHY0_RXMINSLEEP_CAP);
	if (val_next > val)
		val = val_next;

	/* Check Minute Rx MinStall */
	val_next = lli_lo_rd(MIPI_MPHY0_RXMINSTALL_CAP);
	if (val_next > val)
		val = val_next;

	return val;
}

void set_lo_timeout(struct lli_cfg *pcfg)
{
	u32 retry_time_si = 0;
	u32 retry_time = 0;
	u16 patxsvin_si = 0, nextpatxsv = 0;
	u16 patxsv = 0;
	u32 cldrsttim = 0;
	u32 rval = 0;
	u32 one_us = 0;
	u32 svctime = 0;
	u32 aging = 0;
	u32 wtstartinsymbol = 0, wtstartinsi = 0;

	retry_time_si =
	    worst_local_min_sleep_stall(pcfg) +
	    worst_local_prep_length_in_si(pcfg) +
	    worstLocalsync_lengthInSI(pcfg) + (64 * 6) * 5;

	retry_time = unit_conv_si_to_symbol(retry_time_si);
	pcfg->usr_pacfg.rbtc = retry_time;
	lli_lo_wt(MIPI_USER_PA_RBTC, retry_time);

	if (pcfg->tx_cfg.op_mode == LS_MODE) {
		patxsvin_si = lli_lo_rd(MIPI_MPHY0_TXMINSLEEP_CAP);
		nextpatxsv = lli_lo_rd(MIPI_MPHY0_RXMINSLEEP_CAP);
		if (nextpatxsv > patxsvin_si)
			patxsvin_si = nextpatxsv;

	} else { /* HS_MODE */
		patxsvin_si = lli_lo_rd(MIPI_MPHY0_TXMINSTALL_CAP);
		nextpatxsv = lli_lo_rd(MIPI_MPHY0_RXMINSTALL_CAP);
		if (nextpatxsv > patxsvin_si)
			patxsvin_si = nextpatxsv;

	}
	patxsv = unit_conv_si_to_symbol(patxsvin_si);
	pcfg->usr_pacfg.patxsv = patxsv;
	lli_lo_wt(MIPI_USER_PA_PATXSV, (u32)patxsv);

	/* Local ColdResetTimer */
	cldrsttim = (pcfg->tx_cfg.min_activate_time * 100) + 3500;

	one_us = 40; /* 40 MHz PCLK */

	rval = lli_lo_rd(MIPI_USER_CTRL_CLDRSTAUTO);
	rval = (rval & 0xFF) | (cldrsttim << 16) | (one_us << 8);

	lli_lo_wt(MIPI_USER_CTRL_CLDRSTAUTO, rval);
	svctime = 0xFFFF;

	pcfg->usr_ctlcfg.rstr = svctime;
	lli_lo_wt(MIPI_USER_CTRL_RSTR, svctime);

	aging = 0x96;
	wtstartinsymbol = aging * 2 + 600;
	wtstartinsi = unit_conv_symbol_to_si(wtstartinsymbol);

	pcfg->pa_cfig.wt_start_value = wtstartinsi;
	lli_lo_wt(MIPI_LLI_PA_WTSTARTVAL, wtstartinsi);
}

/*
 * Retry time , ColdResetTimer , svctime ,
 * Aging Time , Window timer
 * set timeout values.
 * Warning!! Only MOUNTED Master side can call this.
 * Warning!! PLU required after this call,
 * as AgingParam is shadow
*/
void set_timeout(struct lli_cfg *pcfg)
{
	u32 retry_time_si = 0;
	u32 retry_time = 0;
	u16 patxsvin_si = 0, nextpatxsv = 0;
	u16 patxsv = 0;
	u32 cldrsttim = 0;
	u32 rval = 0;
	u32 one_us = 0;
	u32 svctimeinsi = 0;
	u32 svctime = 0;
	u32 rttinsymbol = 0, rttin_si = 0;
	u32 aging = 0;
	u32 wtstartinsymbol = 0, wtstartinsi = 0;
	u32 r0 = 0, r1 = 0, r2 = 0;

	/* Retry time configure */
	r0 = worst_min_sleep_stall(pcfg);
	r1 = worst_prep_length_in_si(pcfg);
	r2 = worstsync_lengthInSI(pcfg) + (64 * 6) * 5;

	retry_time_si = r0 + r1 + r2;
	retry_time = unit_conv_si_to_symbol(retry_time_si);

	/* DBG_MSG("retry_time is now %08x\n", retry_time); */
	pcfg->usr_pacfg.rbtc = retry_time;

	lli_rm_wt(MIPI_USER_PA_RBTC, retry_time);
	lli_lo_wt(MIPI_USER_PA_RBTC, retry_time);

	/* Minimum time that should stay in SAVE configure */
	if (pcfg->tx_cfg.op_mode == LS_MODE) {

		patxsvin_si = lli_lo_rd(MIPI_MPHY0_TXMINSLEEP_CAP);
		nextpatxsv = lli_rm_rd(MIPI_MPHY0_RXMINSLEEP_CAP);

		if (nextpatxsv > patxsvin_si)
			patxsvin_si = nextpatxsv;

	} else { /* HS_MODE*/
		patxsvin_si = lli_lo_rd(MIPI_MPHY0_TXMINSTALL_CAP);
		nextpatxsv = lli_rm_rd(MIPI_MPHY0_RXMINSTALL_CAP);

		if (nextpatxsv > patxsvin_si)
			patxsvin_si = nextpatxsv;
	}
	patxsv = unit_conv_si_to_symbol(patxsvin_si);
	/* DBG_MSG("patxsv is now %04x\n", patxsv); */
	pcfg->usr_pacfg.patxsv = patxsv;

	lli_rm_wt(MIPI_USER_PA_PATXSV, patxsv);
	lli_lo_wt(MIPI_USER_PA_PATXSV , patxsv);

	/* Remote ColdResetTimer */
	cldrsttim = (lli_rm_rd(MIPI_MPHY0_TXMINACTIVATTIME) * 100) + 3500;
	one_us = 40; /* 40 MHz PCLK*/
	rval = lli_rm_rd(MIPI_USER_CTRL_CLDRSTAUTO);
	rval = (rval & 0xFF) | (cldrsttim << 16) | (one_us << 8);

	/* DBG_MSG("Remote CldRstAuto is now %08x\n", rval); */
	pcfg->usr_ctlcfg.cldrst = rval;

	lli_rm_wt(MIPI_USER_CTRL_CLDRSTAUTO, rval);

	/* Local ColdResetTimer */

	r0 = lli_lo_rd(MIPI_MPHY0_TXMINACTIVATTIME);
	cldrsttim = (r0 * 100) + 3500;
	/* 40 MHz PCLK */
	one_us = 40;
	rval = lli_lo_rd(MIPI_USER_CTRL_CLDRSTAUTO);
	rval = (rval & 0xFF) | (cldrsttim << 16) | (one_us << 8);
	/* DBG_MSG("Local CldRstAuto is now %08x\n", rval); */
	lli_lo_wt(MIPI_USER_CTRL_CLDRSTAUTO, rval);

	/* Round Trip Time */
	rttinsymbol = measure_nack_rtt();
	rttin_si = unit_conv_symbol_to_si(rttinsymbol);

	r0 = worst_min_sleep_stall(pcfg);
	r1 = worst_prep_length_in_si(pcfg);
	r2 = worstsync_lengthInSI(pcfg);
	svctimeinsi = (r0 + r1 + r2 + rttin_si) * 5;

	/* Remote SVC Timeout Configure */
	svctime = unit_conv_si_to_pclk(svctimeinsi, pcfg);
	/* DBG_MSG("SVCTime is now %08x\n", svctime); */
	pcfg->usr_ctlcfg.rstr = svctime;
	lli_rm_wt(MIPI_USER_CTRL_RSTR, svctime);
	lli_lo_wt(MIPI_USER_CTRL_RSTR, svctime);

	/* Aging Param Configure */
	aging = rttinsymbol + 10;
	 /* value is too large for the 24bit register*/
	if (aging & 0xFF000000)
		aging = 0xFFFFFF;

	/* DBG_MSG("AgingParam is now %08x\n", aging); */

	pcfg->usr_pacfg.aging_parameter = aging;

	lli_rm_wt(MIPI_USER_PA_AGPRM, aging);
	lli_lo_wt(MIPI_USER_PA_AGPRM, aging);

	wtstartinsymbol = aging * 2 + 600;
	wtstartinsi = unit_conv_symbol_to_si(wtstartinsymbol);
	/* DBG_MSG("WT_Start_value is now %08x\n", wtstartinsi); */
	pcfg->pa_cfig.wt_start_value = wtstartinsi;
	lli_rm_wt(MIPI_LLI_PA_WTSTARTVAL, wtstartinsi);
	lli_lo_wt(MIPI_LLI_PA_WTSTARTVAL, wtstartinsi);
}

/* used in pre-mount procedure */
void local_set_deskew_to(struct lli_cfg *pcfg)
{

	u32 deskew_time_si = 0;
	u32 deskew_time = 0;
	u32 r0 = 0, r1 = 0, r2 = 0;

	if (pcfg->rx_cfg.op_mode == LS_MODE) {
		if (pcfg->rx_cfg.pwm_gear < PWM_G6) { /* G0-G5 */
			r0 = lli_lo_rd(MIPI_MPHY0_RXLSPREPLEN_CAP);
			deskew_time_si = lspreplencaptosi(r0, pcfg) + 100;
		} else { /* G6, G7 */
			r0 = lli_lo_rd(MIPI_MPHY0_RXLSPREPLEN_CAP);
			r1 = lli_lo_rd(MIPI_MPHY0_RXLSG67sync_len_CAP);
			deskew_time_si =
			    lspreplencaptosi(r0, pcfg) +
			    get_sync_len(r1) + 100;
		}
	} else { /* HS_MODE */
		switch (pcfg->rx_cfg.hs_gear) {
		case HS_G1:
			r0 = lli_lo_rd(MIPI_MPHY0_RXHSG1PREPLEN_CAP);
			r1 = lli_lo_rd(MIPI_MPHY0_RXHSG1sync_len_CAP);
			deskew_time_si = hspreplencaptosi(r0, pcfg) +
			    get_sync_len(r1) + 100;
			break;

		case HS_G2:
			r0 = lli_lo_rd(MIPI_MPHY0_RXHSG2PREPLEN_CAP);
			r1 = hspreplencaptosi(r0, pcfg) + 100;
			r2 = lli_lo_rd(MIPI_MPHY0_RXHSG2sync_len_CAP);
			deskew_time_si = r1 + get_sync_len(r2);
			break;

		/* case HS_G3: */
		default:
			r0 = lli_lo_rd(MIPI_MPHY0_RXHSG3PREPLEN_CAP);
			r1 = lli_lo_rd(MIPI_MPHY0_RXHSG3sync_len_CAP);
			deskew_time_si = 100 + hspreplencaptosi(r0, pcfg) +
			get_sync_len(r1);
			break;
		}
	}

	deskew_time = unit_conv_si_to_symbol(deskew_time_si);
	/* DBG_MSG("deskew_time(local) is now %08x\n", deskew_time); */
	pcfg->usr_pacfg.dskwtc = deskew_time;
	lli_lo_wt(MIPI_USER_PA_DSKWTC, deskew_time);
}

void set_deskew_to(struct lli_cfg *pcfg)
{
	u32 deskew_time_si = 0;
	u32 deskew_time = 0;
	u32 r0 = 0, r1 = 0;

	if (pcfg->rx_cfg.op_mode == LS_MODE) {
		r0 = lli_lo_rd(MIPI_MPHY0_RXLSPREPLEN_CAP);
		r1 = lli_lo_rd(MIPI_MPHY0_RXLSG67sync_len_CAP);
		/* G0-G5 */
		if (pcfg->rx_cfg.pwm_gear < PWM_G6)
			deskew_time_si = 100 + lspreplencaptosi(r0, pcfg);
		else { /* G6, G7 */
			deskew_time_si = 100 +
			lspreplencaptosi(r0, pcfg) + get_sync_len(r1);
		}
	} else { /* HS_MODE */

		switch (pcfg->rx_cfg.hs_gear) {
		case HS_G1:
			r0 = lli_lo_rd(MIPI_MPHY0_RXHSG1PREPLEN_CAP);
			r1 = lli_lo_rd(MIPI_MPHY0_RXHSG1sync_len_CAP);
			deskew_time_si = hspreplencaptosi(r0, pcfg) +
			    get_sync_len(r1) + 100;
			break;
		case HS_G2:
			r0 = lli_lo_rd(MIPI_MPHY0_RXHSG2PREPLEN_CAP);
			r1 = lli_lo_rd(MIPI_MPHY0_RXHSG2sync_len_CAP);
			deskew_time_si = hspreplencaptosi(r0, pcfg) +
			    get_sync_len(r1) + 100;
			break;
		/* case HS_G3: */
		default:
			r0 = lli_lo_rd(MIPI_MPHY0_RXHSG3PREPLEN_CAP);
			r1 = lli_lo_rd(MIPI_MPHY0_RXHSG3sync_len_CAP);
			deskew_time_si =
			    hspreplencaptosi(r0, pcfg) + get_sync_len(r1)
			    + 100;
			break;
		}
	}

	deskew_time = unit_conv_si_to_symbol(deskew_time_si);
	/*DBG_MSG("deskew_time is now %08x\n", deskew_time);*/
	pcfg->usr_pacfg.dskwtc = deskew_time;
	lli_rm_wt(MIPI_USER_PA_DSKWTC, deskew_time);
	lli_lo_wt(MIPI_USER_PA_DSKWTC, deskew_time);
}


u32 measure_nack_rtt(void)
{
	u32 val = 0, intval = 0;
	/* tries 10 times for precise measure*/
	/* u32 measure = 10; */
	u32 measure = 1;
	u32 i = 0;
	u32 max_val = 0;
	u32 try = 100;

	for (i = 0; i < measure; i++) {

		lli_lo_wt(MIPI_USER_PA_NACKRTTMSR, NACK_RTTMSR_EN);
		while (try > 0) {

			intval = lli_lo_rd(MIPI_USER_CTRL_INTST);

			if (intval)
				lli_lo_wt(MIPI_USER_CTRL_INTCLR, intval);

			val = lli_lo_rd(MIPI_USER_PA_NACKRTTMSR);

			if (val & NACK_RTTMSR_DONE) {
				val = lli_lo_rd(MIPI_USER_PA_NACKRTTMSRRSLT);
				/* DBG_MSG("Measured Val: %08x\n", val); */
				if (max_val < val)
					max_val = val;
				break;
			}
			try--;
		}
	}

	val = max_val;
	/* DBG_MSG("Maximum measured val: %08x\n", val); */

	return val;
}




void setup_error_detect(void)
{
	u32 v = 0;

	v = B_RETRYTIMEN | B_NACKCNTEN | B_DSKWTIMEN;
	lli_lo_wt(MIPI_USER_PA_CNTCTL, v);
}

void lli_lo_wt(u32 f, u32 v)
{
	struct lli_dev *dev_lli;
	struct lli_data *lli_ch;

	dev_lli = gp_mb86s70_lli_dev;
	lli_ch = &dev_lli->chan_data[ACT_LLI_CH];
	writel(v, lli_ch->regs + (f | LOCAL_ADDR_BIT));
}

u32 lli_lo_rd(u32 v)
{
	struct lli_dev *dev_lli;
	struct lli_data *lli_ch;

	dev_lli = gp_mb86s70_lli_dev;
	lli_ch = &dev_lli->chan_data[ACT_LLI_CH];
	return readl(lli_ch->regs + (v | LOCAL_ADDR_BIT));
}


void lli_rm_wt(u32 f, u32 v)
{
	struct lli_dev *dev_lli;
	struct lli_data *lli_ch;

	dev_lli = gp_mb86s70_lli_dev;
	lli_ch = &dev_lli->chan_data[ACT_LLI_CH];
	writel(v, lli_ch->regs + (f & (~LOCAL_ADDR_BIT)));
}

u32 lli_rm_rd(u32 v)
{
	struct lli_dev *dev_lli;
	struct lli_data *lli_ch;

	dev_lli = gp_mb86s70_lli_dev;
	lli_ch = &dev_lli->chan_data[ACT_LLI_CH];
	return readl(lli_ch->regs + (v & (~LOCAL_ADDR_BIT)));
}
