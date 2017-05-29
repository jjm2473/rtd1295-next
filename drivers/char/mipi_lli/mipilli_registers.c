/*
 * mipilli_registers.c F_MIPILLI_LP Controller Driver
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

#include "mipilli_registers.h"

struct lli_cfg lli_def_cfg = {
	/* ----------- M-PHY ----------- */
	.tx_cfg = {
		.op_mode            = LS_MODE,  /* TX/RX */
		.rate_series        = A,        /* TX/RX */
		.hs_gear            = HS_G1,    /* TX/RX HS-only*/
		.pwm_gear           = PWM_G1,   /* TX/RX LS-only*/
		.amplitude          = LARGE_AMPLITUDE,  /* TX */
		.slew_rate          = 0,        /* TX HS-only */
		.sync_source        = INTERNAL_SYNC,/* TX */
		.sync_range         = COARSE,   /* TX HS-only */
		.sync_length        = 15,       /* TX HS-only */
		.hs_prepare_length  = 15,       /* TX HS-only */
		.ls_prepare_length  = 10,       /* TX LS-only */
		.lcc_enable         = true,     /* TX */
		.hs_unterminated    = 32,       /* TX LS-only */
		.bypass8B10B        = false,    /* TX/RX */
		.polarity           = NORMAL,   /* TX */
		.hs_unterminated    = false,    /* TX/RX HS-only*/
		.ls_terminated      = false,    /* TX/RX LS-only*/
		.min_activate_time  = 15,       /* TX */
		.lsg6_sync_range    = COARSE,   /* TX LS-only */
		.lsg6_sync_length   = 15,       /* TX LS-only */
	},
	.rx_cfg = {
		.op_mode            = LS_MODE,  /* TX/RX */
		.rate_series        = A,        /* TX/RX */
		.hs_gear            = HS_G1,    /* TX/RX HS-only */
		.pwm_gear           = PWM_G1,   /* TX/RX LS-only */
		.hs_unterminated    = false,    /* TX/RX HS-only */
		.ls_terminated      = false,    /* TX/RX LS-only */
		.bypass8B10B        = false,    /* TX/RX */
		.force_term         = false,    /* RX */
	},

	/* ------ M-PHY ------- */
	.pa_cfig = {
		.marker0_insertion          = false,
		.wt_start_value             = 0,
		.phit_rec_cont_en           = false,
		.phit_err_cout_en           = false,
		.active_tx_count            = 1,
		.active_rx_count            = 1,
		.pa_min_save_config         = 250,
		.pa_worst_case_rtt          = 4096,
		.drive_tactive_duration     = 10,
		.csa_pa_set                 = 0,
		.pa_phy_test_config_master  = 0,
		.pa_phy_test_config_slasve  = 0,
	},
	.usr_pacfg = {
		.aging_parameter = 0x96,
		.rbtc   = 0,
		.nackt  = 0,
		.dskwtc = 0,
		.patxsv = 3,
	},
	.lk_ctlcfg = {
		.ll_tc_disable  = false,/* enable LL TC*/
		.be_tc_disable  = false,/* disable BE TC*/
		.tl_addr_mode   = false,/* 0: 36bit 1:40 bit*/
	},
	.usr_ctlcfg = {
		/* .cldrst_auto = 0 << CLD_RST_TIM | 0 << CRST_ONE_US_CNT |
			0 << CRST_AUTO_EN_2 | 0 << CRST_AUTO_EN,
		*/
		.cldrst_auto = 0,
		.roe_auto   = 0,
		.intum      = 0,
		.axi_cofg   = 0,
		.tranarb    = 0,
		.axiapb_ctl = 0,
		.rstr       = 0,

		.m_ad_remapConfig = {
			{/* [0]*/
				.remap  = 0,
				.unmask = 0,
				.filter = 0,
			},
			{/* [1]*/
				.remap  = 0,
				.unmask = 0,
				.filter = 0,
			},
			{/* [2]*/
				.remap  = 0,
				.unmask = 0,
				.filter = 0,
			},
			{/* [3]*/
				.remap  = 0,
				.unmask = 0,
				.filter = 0,
			},
			{/* [4]*/
				.remap  = 0,
				.unmask = 0,
				.filter = 0,
			},
			{/* [5]*/
				.remap  = 0,
				.unmask = 0,
				.filter = 0,
			},
			{/* [6]*/
				.remap  = 0,
				.unmask = 0,
				.filter = 0,
			},
			{/* [7]*/
				.remap  = 0,
				.unmask = 0,
				.filter = 0,
			},
		},
		.s_ad_remapConfig = {
			{/* [0]*/
				.remap = 0,
				.unmask = 0,
				.filter = 0,
			},
			{/* [1]*/
				.remap = 0,
				.unmask = 0,
				.filter = 0,
			},
			{/* [2]*/
				.remap = 0,
				.unmask = 0,
				.filter = 0,
			},
			{/* [3]*/
				.remap = 0,
				.unmask = 0,
				.filter = 0,
			},
			{/* [4]*/
				.remap = 0,
				.unmask = 0,
				.filter = 0,
			},
			{/* [5]*/
				.remap = 0,
				.unmask = 0,
				.filter = 0,
			},
			{/* [6]*/
				.remap = 0,
				.unmask = 0,
				.filter = 0,
			},
			{/* [7]*/
				.remap = 0,
				.unmask = 0,
				.filter = 0,
			},
		},
		.s_ad_postwConfig = {
			{/* [0]*/
				.unmask = 0,
				.filter = 0,
			},
			{/* [1]*/
				.unmask = 0,
				.filter = 0,
			},
			{/* [2]*/
				.unmask = 0,
				.filter = 0,
			},
			{/* [3]*/
				.unmask = 0,
				.filter = 0,
			},
			{/* [4]*/
				.unmask = 0,
				.filter = 0,
			},
			{/* [5]*/
				.unmask = 0,
				.filter = 0,
			},
			{/* [6]*/
				.unmask = 0,
				.filter = 0,
			},
			{/* [7] */
				.unmask = 0,
				.filter = 0,
			},
		},
		.ll_m_awuserSwap = {
			0x00, 0x01, 0x02, 0x03,
			0x04, 0x05, 0x06, 0x07,
			0x08,
		},
		.ll_m_aruserSwap = {
			0x00, 0x01, 0x02, 0x03,
			0x04, 0x05, 0x06, 0x07,
			0x08,
		},
		.ll_s_awuserSwap = {
			0x00, 0x01, 0x02, 0x03,
			0x04, 0x05, 0x06, 0x07,
			0x08,
		},
		.ll_s_aruserSwap = {
			0x00, 0x01, 0x02, 0x03,
			0x04, 0x05, 0x06, 0x07,
			0x08,
		},
		.mawusr_en = 0,
		.marusr_en = 0,
		.sawusr_en = 0,
		.sarusr_en = 0,

		.sig_stat_sw = {
			0x00, 0x01, 0x02, 0x03,
			0x04, 0x05, 0x06, 0x07,
			0x08, 0x09, 0x0A, 0x0B,
			0x0C, 0x0D, 0x0E, 0x0F,
			0x10, 0x11, 0x12, 0x13,
			0x14, 0x15, 0x16, 0x17,
			0x18, 0x19, 0x1A, 0x1B,
			0x1C, 0x1D, 0x1E, 0x1F,
		},

		.sig_set_sw = {
			0x00, 0x01, 0x02, 0x03,
			0x04, 0x05, 0x06, 0x07,
			0x08, 0x09, 0x0A, 0x0B,
			0x0C, 0x0D, 0x0E, 0x0F,
			0x10, 0x11, 0x12, 0x13,
			0x14, 0x15, 0x16, 0x17,
			0x18, 0x19, 0x1A, 0x1B,
			0x1C, 0x1D, 0x1E, 0x1F,
		},
		.lssts_en = 0xffffffff,
		.lssts_or = 0,
		.lsset_en = 0xffffffff,
		.lsset_um = 0xffffffff,
	},
};

