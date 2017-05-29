/*
 * mipilli_registers.h F_MIPILLI_LP Controller Driver
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

/*
 * Address[14:0] Name
 * 0x0000-0x3FFF PHY (see 6.4)
 * 0x4000-0x43FF LLI PA Configuration (see 6.1.1)
 * 0x4400-0x47FF User PA Configuration (*see 6.1.2)
 * 0x4800-0x4FFF LLI Control (see 6.1.1)
 * 0x5000-0x5FFF User Control (*see 6.1.3)
 * 0x6000-0x63FF LLI Signaling Configuration (see 6.1.4)
 * 0x6400-0x67FF User Signaling Configuration (Reserved)
 * 0x6800-0x6BFF LLI Debug Signaling Configuration (Reserved)
 * 0x6C00-0x6FFF User Debug Signaling Configuration (Reserved)
 * 0x7000-0x7BFF Reserved
 * 0x7C00-0x7FFF DDB(Not support) (Reserved)
*/


#ifndef MIPILLI_REGISTERS_H_
#define MIPILLI_REGISTERS_H_


#include <linux/module.h>


enum MIPILLI_EVALPAT {
	EVAL_WRITE,
	EVAL_READ,
	EVAL_WRITE_POST,
	EVAL_UMOUNT,
	EVAL_TEST,
};

enum MIPILLI_EVALMODE {
	EVAL_HSG1A,
	EVAL_HSG1B,
	EVAL_HSG2A,
	EVAL_HSG2B,
};

enum MIPILLI_MASTERSLAVE {
	EVAL_MASTER,
	EVAL_SLAVE,
};

enum MPHY_HS_GEAR_CAP {
	HS_G1_ONLY = 1,
	HS_G1_TO_G2 = 2,
	HS_G1_TO_G3 = 3,
};

enum MPHY_PWM_GEAR_CAP {
	PWM_G1_ONLY = 1,
	PWM_G1_TO_G2 = 2,
	PWM_G1_TO_G3 = 3,
	PWM_G1_TO_G4 = 4,
	PWM_G1_TO_G5 = 5,
	PWM_G1_TO_G6 = 6,
	PWM_G1_TO_G7 = 7,
};

enum MPHY_AMPLITUDE_CAP {
	SMALL_AMPLITUDE_ONLY = 1,
	LARGE_AMPLITUDE_ONLY = 2,
	LARGE_AND_SMALL_AMPLITUDE = 3,
};

enum MPHY_OP_MODE {
	LS_MODE = 1,
	HS_MODE = 2,
};

enum MPHY_HS_RATE_SERIES {
	A = 1,
	B = 2,
};

enum MPHY_HS_GEAR {
	HS_G1 = 1,
	HS_G2 = 2,
	HS_G3 = 3,
};

enum MPHY_PWM_GEAR {
	PWM_G0 = 0,
	PWM_G1 = 1,
	PWM_G2 = 2,
	PWM_G3 = 3,
	PWM_G4 = 4,
	PWM_G5 = 5,
	PWM_G6 = 6,
	PWM_G7 = 7,
};

enum MPHY_AMPLITUDE {
	SMALL_AMPLITUDE = 1,
	LARGE_AMPLITUDE = 2,
};

enum MPHY_SYNC_SOURCE {
	INTERNAL_SYNC = 0,
	EXTERNAL_SYNC = 1,
};

enum MPHY_SYNC_RANGE {
	FINE = 0,
	COARSE = 1,
};

enum MPHY_HIBERN8_CONTROL {
	EXIT = 0,
	ENTER = 1,
};

enum MPHY_POLARITY {
	NORMAL = 0,
	INVERTED = 1,
};

enum MPHY_STATE {
	DISABLED = 0,
	HIBERN8 = 1,
	SLEEP = 2,
	STALL = 3,
	LS_BURST = 4,
	HS_BURST = 5,
	LINE_CFG = 6,
};

/* LLI attributes' enumerated value definitions */
enum LLI_PA_TPVSTATE {
	WAITING_FOR_NAK0 = 0,
	PATTERN_COMPARISON_ON_GOING = 1,
	END_OF_PATTERN_COMPARISON = 2,
};

#define CLD_RST_TIM     16
#define CRST_ONE_US_CNT 8
#define CRST_AUTO_EN_2  1
#define CRST_AUTO_EN    0

/* CSA System bit fields*/
#define B_CSA_LLI_MOUNTED       0x01
#define B_CSA_MASTER_NOT_SLAVE  0x02
#define B_CSA_LLI_MOUNT_CTRL    0x04
#define B_RESET_ON_ERROR_DETECT 0x08


/* CSA PA bit fields */
#define B_CSA_PA_AUTO_MODE          0x01
#define B_CSA_PA_LINKUPDATECONFIG   0x02
#define B_CSA_PA_TESTMODE           0x04
#define B_CSA_PA_HIB8               0x08
#define B_CSA_PA_PLU_RECEIVED       0x10

/* INT fields */
/* LLI INTERRUPT SHIT */
#define B_RMTSVCTO          (1 << 16)
#define B_SIGSET            (1 << 15)
#define B_SEQERR            (1 << 14)
#define B_CRCERR            (1 << 13)
#define B_PHYSYMERR         (1 << 12)
#define B_NACKRCVD          (1 << 11)
#define B_RETRYBUFTO        (1 << 10)
#define B_FIFOOVF           (1 << 9)
#define B_DSKWTO            (1 << 8)
#define B_CRSTAST           (1 << 7)
#define B_RSTONERRDET   (1 << 6)
#define B_PLUFIN        (1 << 5)
#define B_PLURCV        (1 << 4)
#define B_MNTFTlERR     (1 << 3)
#define B_UNMNT         (1 << 2)
#define B_MNT           (1 << 1)
#define B_RXH8EXIT      (1 << 0)

/* Test Configuration fields */
#define B_PARAMSEL_CRPAT 0x01
#define B_TESTRESULT_VEN 0x02
#define B_TESTBURST_MODE 0x04
#define B_SEQID_CHK_DIS  0x08

/* AXI/APB Control Register fields */
#define B_RMTSVCTOEN 0x100


/* Counter Control Register fields */
#define B_RETRYTIMEN 0x1
#define B_NACKCNTEN 0x2
#define B_DSKWTIMEN 0x4



/*
 * M-PHY configuration values 0x0000-0x3FFF PHY (see 6.4)
 */
struct mipilli_mphy_cfg {
	enum MPHY_OP_MODE           op_mode;    /* TX/RX */
	enum MPHY_HS_RATE_SERIES   rate_series;/* TX/RX */
	enum MPHY_HS_GEAR           hs_gear;    /* TX/RX HS-only */
	enum MPHY_PWM_GEAR          pwm_gear;   /* TX/RX LS-only */
	enum MPHY_AMPLITUDE         amplitude;  /* TX */
	enum MPHY_SYNC_SOURCE       sync_source;/* TX */
	enum MPHY_SYNC_RANGE        sync_range; /* TX HS-only */
	enum MPHY_POLARITY          polarity;   /* TX */
	enum MPHY_SYNC_RANGE        lsg6_sync_range;/* TX LS-only */

	u8 slew_rate;           /* TX HS-only */
	u8 sync_length;         /* TX HS-only */
	u8 hs_prepare_length;   /* TX HS-only */
	u8 ls_prepare_length;   /* TX LS-only */
	bool lcc_enable;        /* TX */
	u8 burst_closureExt;    /* TX LS-only */
	bool bypass8B10B;       /* TX/RX */
	bool hs_unterminated;   /* TX/RX HS-only */
	bool ls_terminated;     /* TX/RX LS-only */
	u8 min_activate_time;   /* TX */
	u8 lsg6_sync_length;    /* TX LS-only */
	bool force_term;        /* RX */
};


/*
 * 0x4000-0x43FF LLI PA Configuration (see 6.1.1)
 * LLI PA configuration values
 */

struct mipilli_lk_pacfg {
	u8 tx_count;    /* ro */
	u8 rx_count;    /* ro */
	bool marker0_insertion;
	u16 wt_start_value;
	bool phit_err_cont;     /* ro */
	bool phit_rec_cont_en;
	bool phit_err_cout_en;
	u32 phit_rec_cont_lsb;  /* ro */
	u32 phit_rec_cont_msb;  /* ro */
	bool phit_clr_cont;
	u8 active_tx_count;
	u8 active_rx_count;
	u32 config_update;
	u8 pa_min_save_config;
	u16 pa_worst_case_rtt;
	u8 drive_tactive_duration;  /* default : 0x0a */
	u8 csa_pa_status;
	u8 csa_pa_set;
	u8 csa_pa_clr;
	u32 pa_phy_test_config_master;
	u32 pa_phy_test_config_slasve;
	u32 pa_phy_test_result0;    /* ro */
	u32 pa_phy_test_result1;    /* ro */
	u32 pa_phy_test_result2;    /* ro */
	u32 pa_phy_test_result3;    /* ro */
};




/*
 * 0x4400-0x47FF User PA Configuration (*see 6.1.2)
 */
struct mipilli_lk_ur_pacfg {
	u32 aging_parameter;
	u8 nack_rtt_msr;        /* default :0x00 */
	u32 nack_rtt_msr_rslt;  /* default :0x00 */
	u8 cnt_ctl;     /* default :0x00 */
	u32 rbtc;       /* Retry Buffer Timeout */
	u32 nackt;      /* NACKThreshold */
	u32 dskwtc;     /* DeskewTimeout */
	u16 patxsv;     /* PATXSave */
	u32 mphyrb;     /* M-PHY Register Bank default:0x00 */
};


/*
 * 0x4800-0x4FFF LLI Control (see 6.1.1)
 */
struct mipilli_lk_conl {
	bool ll_initiator_present;  /* ro */
	bool ii_target_present;     /* ro */
	bool be_initiator_present;  /* ro */
	bool be_Target_present;     /* ro */
	bool svc_target_present;    /* ro */
	bool ll_tc_disable;
	bool be_tc_disable;
	u8 csa_system_status;   /* ro */
	u8 csa_system_set;      /* default : x */
	u8 csa_system_clear;    /* default : x */
	u8 tl_addr_mode;
};





/* 0x5000-0x5FFF User Control (*see 6.1.3) */

struct mipilli_swcfg {
	u32 remap;
	u32 unmask;
	u32 filter;
};

struct mipilli_lk_ur_conl {
	bool cldrst;    /* default */
	u32 cldrst_auto;
	u8 roe;         /* default 0x00 */
	u8 roe_auto;
	u32 intst;      /* ro */
	u32 intum;
	u32 intclr;
	u8 axi_cofg;
	u8 tranarb;
	u32 axiapb_ctl;
	u32 rstr;       /* (Remote SVC Timeout) */

	struct mipilli_swcfg m_ad_remapConfig[8];
	struct mipilli_swcfg s_ad_remapConfig[8];
	struct mipilli_swcfg s_ad_postwConfig[8];

	u8 ll_m_awuserSwap[9];
	u8 ll_m_aruserSwap[9];
	u8 ll_s_awuserSwap[9];
	u8 ll_s_aruserSwap[9];

	u16 mawusr_en;
	u16 marusr_en;
	u16 sawusr_en;
	u16 sarusr_en;

	u8 sig_stat_sw[32];
	u8 sig_set_sw[32];
	u32 lssts_en;
	u32 lssts_or;
	u32 lsset_mr;   /* ro */
	u32 lsset_en;
	u32 lsset_um;

};


/* 0x6000-0x63FF LLI Signaling Configuration (see 6.1.4) */

struct mipilli_lk_sigcfg {
	u16 sig_reg_num;    /* ro */
	u32 signal_status;  /* ro */
	u32 signal_set;
	u32 signal_clear;
};


struct lli_cfg {
	/* M-PHY */
	struct mipilli_mphy_cfg tx_cfg;
	struct mipilli_mphy_cfg rx_cfg;
	/* 6.5.LLI PA Configuration Attributes */
	struct mipilli_lk_pacfg pa_cfig;
	struct mipilli_lk_ur_pacfg  usr_pacfg;
	struct mipilli_lk_conl      lk_ctlcfg;
	struct mipilli_lk_ur_conl   usr_ctlcfg;
	struct mipilli_lk_sigcfg    signal_config;
};

extern struct lli_cfg lli_def_cfg;

/*
 * 0x6400-0x67FF User Signaling Configuration (Reserved)
 * 0x6800-0x6BFF LLI Debug Signaling Configuration (Reserved)
 * 0x6C00-0x6FFF User Debug Signaling Configuration (Reserved)
 * 0x7000-0x7BFF Reserved
 * 0x7C00-0x7FFF DDB(Not support) (Reserved)
*/
#endif
