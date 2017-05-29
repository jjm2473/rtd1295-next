#ifndef __SCB_MHU_API_H
#define __SCB_MHU_API_H

#define CMD_MASK    0x7f    /* 128 possible commands */
#define RESP_BIT    (1 << 7) /* If it's a response */

#define ENC_CMD(c)  ((c) & CMD_MASK)
#define DEC_CMD(v)  (((v) & ~CMD_MASK) ? CMD_INVALID : ((v) & CMD_MASK))

#define ENC_REP(r)  (((r) & CMD_MASK) | RESP_BIT)

/* If v is the reply to command c */
#define IS_A_REP(v, c)  (((v) & RESP_BIT) && (((v) & CMD_MASK) == (c)))

enum {
	CMD_INVALID = 0,
	/* I2C PassThrough
	 con_id, addr, speed, flags  must be same in REQ and REP
	 If SUCCESS on SCB, REP_len == REQ_len
	 If ERROR on SCB REP_len == actual number of bytes read/written (< REP_len)
	*/
	CMD_PERI_POWER_SET_REQ = 2,
	/*
	peri_id  must be same in REQ and REP;
	If SUCCESS on SCB,  REP_en == current status{0,1} of gate
	If ERROR on SCB(only means clk doesn't exist),  REP_en == 0, that is, clk is disabled/gated
	 */
	CMD_PERI_CLOCK_GATE_SET_REQ = 3,
	/*
	 cntrlr, domain, port  must be same in REQ and REP;
	 If SUCCESS on SCB,  REP_en == current status{0,1} of gate
	 If ERROR on SCB(only means clk doesn't exist),  REP_en == 0, that is, clk is disabled/gated
	 freqency  ignored in REQ and REP;
	*/
	CMD_PERI_CLOCK_GATE_GET_REQ = 4,
	/*
	 cntrlr, domain, port must be same in REQ and REP;
	 If SUCCESS on SCB, REP_frequency == REQ_frequency
	 If ERROR on SCB,  REP_frequency == original/old rate of clock
	 'en' is ignored in REQ and REP;
	*/
	CMD_PERI_CLOCK_RATE_SET_REQ = 5,
	/*
	 cntrlr, domain, port must be same in REQ and REP;
	 If SUCCESS on SCB, REP_frequency == current rate of clock
	 If ERROR on SCB(only means clk doesn't exist),  REP_frequency == 0
	 'en' is ignored in REQ and REP;
	*/
	CMD_PERI_CLOCK_RATE_GET_REQ = 6,
	CMD_CPU_CLOCK_GATE_SET_REQ = 7,
	CMD_CPU_CLOCK_GATE_GET_REQ = 8,
	CMD_CPU_CLOCK_RATE_SET_REQ = 9,
	CMD_CPU_CLOCK_RATE_GET_REQ = 0xa,
	CMD_CLUSTER_OPP_GET_REQ = 0xb,
	CMD_CLOCK_DSI_PIXEL_REQ = 0xc,
	CMD_SCB_CAPABILITY_GET_REQ = 0xd,
	CMD_SYS_RESET_CAUSE_GET_REQ = 0xe,
	CMD_SYS_SPECIFIC_INFO_GET_REQ = 0xf,
	CMD_REBOOT_AP_AFTER_REQ = 0x10,

	CMD_TAIKI_REQ = 0x11,
	CMD_TAIKI_ASYNC_MSG_REQ = 0x12,
	CMD_GET_WORD_REQ = 0x13,
	CMD_HARD_RESET_REQ = 0x14,
	
	CMD_MAINTENANCE_MODE_REQ = 0x15,

	CMD_MEMORY_LAYOUT_GET_REQ = 0x19,

	CMD_POWERDOMAIN_GET_REQ = 0x1a,
	CMD_POWERDOMAIN_SET_REQ = 0x1b,

	CMD_THERMAL_INFO_REQ = 0x1d,
	CMD_RESUME_ENTRY_POINT_SET_REQ = 0x1e,
	CMD_RESUME_ENTRY_POINT_GET_REQ = 0x1f,

	/* Do NOT add new commands below this line */
	MHU_NUM_CMDS,
};

#define CMD_PERI_POWER_SET_REP	ENC_REP(CMD_PERI_POWER_SET_REQ)
#define CMD_PERI_CLOCK_GATE_SET_REP	ENC_REP(CMD_PERI_CLOCK_GATE_SET_REQ)
#define CMD_PERI_CLOCK_GATE_GET_REP	ENC_REP(CMD_PERI_CLOCK_GATE_GET_REQ)
#define CMD_PERI_CLOCK_RATE_SET_REP	ENC_REP(CMD_PERI_CLOCK_RATE_SET_REQ)
#define CMD_PERI_CLOCK_RATE_GET_REP	ENC_REP(CMD_PERI_CLOCK_RATE_GET_REQ)
#define CMD_CPU_CLOCK_GATE_SET_REP	ENC_REP(CMD_CPU_CLOCK_GATE_SET_REQ)
#define CMD_CPU_CLOCK_GATE_GET_REP	ENC_REP(CMD_CPU_CLOCK_GATE_GET_REQ)
#define CMD_CPU_CLOCK_RATE_SET_REP	ENC_REP(CMD_CPU_CLOCK_RATE_SET_REQ)
#define CMD_CPU_CLOCK_RATE_GET_REP	ENC_REP(CMD_CPU_CLOCK_RATE_GET_REQ)
#define CMD_CLUSTER_OPP_GET_REP	ENC_REP(CMD_CLUSTER_OPP_GET_REQ)
#define CMD_CLOCK_DSI_PIXEL_REP	ENC_REP(CMD_CLOCK_DSI_PIXEL_REQ)
#define CMD_SCB_CAPABILITY_GET_REP	ENC_REP(CMD_SCB_CAPABILITY_GET_REQ)
#define CMD_SYS_RESET_CAUSE_GET_REP	ENC_REP(CMD_SYS_RESET_CAUSE_GET_REQ)
#define CMD_SYS_SPECIFIC_INFO_GET_REP	ENC_REP(CMD_SYS_SPECIFIC_INFO_GET_REQ)
#define CMD_GET_WORD_REP	ENC_REP(CMD_GET_WORD_REQ)
#define CMD_REBOOT_AP_AFTER_REP	ENC_REP(CMD_REBOOT_AP_AFTER_REQ)
#define CMD_TAIKI_REP			ENC_REP(CMD_TAIKI_REQ)
#define CMD_TAIKI_ASYNC_MSG_REP		ENC_REP(CMD_TAIKI_ASYNC_MSG_REQ)
#define CMD_HARD_RESET_REP		ENC_REP(CMD_HARD_RESET_REQ)
#define CMD_MAINTENANCE_MODE_REP	ENC_RSP(CMD_MAINTENANCE_MODE_REQ)
#define CMD_MEMORY_LAYOUT_GET_REP	ENC_REP(CMD_MEMORY_LAYOUT_GET_REQ)
#define CMD_POWERDOMAIN_GET_REP		ENC_REP(CMD_POWERDOMAIN_GET_REQ)
#define CMD_POWERDOMAIN_SET_REP		ENC_REP(CMD_POWERDOMAIN_SET_REQ)
#define CMD_THERMAL_INFO_REP		ENC_REP(CMD_THERMAL_INFO_REQ)
#define CMD_RESUME_ENTRY_POINT_SET_REP		ENC_REP(CMD_RESUME_ENTRY_POINT_SET_REQ)
#define CMD_RESUME_ENTRY_POINT_GET_REP		ENC_REP(CMD_RESUME_ENTRY_POINT_GET_REQ)

struct cmd_periclk_control {
	u32 payload_size;
	u32 cntrlr;
	u32 domain;
	u32 port;
	u32 en;
	u64 freqency;
} __packed;

struct cmd_peripower_control {
	u32 payload_size;
	u32 peri_id;
	u32 en;
} __packed;

#define AT_WFI_DO_NOTHING	0x0
#define AT_WFI_DO_SUSPEND	0x1
#define AT_WFI_DO_POWEROFF	0x2
#define AT_WFI_COLOR_MASK	0x3

/*
 * Offset of 4-bytes wfi 'color' register from ISRAM Start
 * The 4-bytes store 'color' for each cpu core as
 *   A15_0, A15_1, A7_0, A7_1
 */
#define WFI_COLOR_REG_OFFSET	0x3f00

/**
 * struct CMD_CPU_CLOCK_GATE_SET_REQ
 *        CMD_CPU_CLOCK_GATE_SET_REP
 *        CMD_CPU_CLOCK_GATE_GET_REQ
 *        CMD_CPU_CLOCK_GATE_GET_REP
 *
 * @cluster_class : 0 fixed
 * @cluster_id    : 0x0:A15, 0x1:A7
 * @cpu_id        : 0x0:CPU0, 0x1:CPU1, 0x2:CPU2
 * @cpu_state     : 0x0:OFF, 0x1:ON, 0x2:SLEEP0,
 * 		    0x3:SLEEP1, 0xffffffff:CPU does not exist
 *
 * This packet format is for the commands
 * CMD_CPU_CLOCK_GATE_SET_REQ and CMD_CPU_CLOCK_GATE_SET_REP
 * CMD_CPU_CLOCK_GATE_GET_REQ and CMD_CPU_CLOCK_GATE_GET_REP
 * AP populates it as a command. SCB populates it as a response.
 *
 * SCB's response should always show the final status via 'cpu_control'
 */
struct cmd_cpu_control_gate {
	u32 payload_size;
	u32 cluster_class;
	u32 cluster_id;
	u32 cpu_id;
#define SCB_CPU_STATE_OFF	0x0
#define SCB_CPU_STATE_ON	0x1
#define SCB_CPU_STATE_SUSP	0x2
	u32 cpu_state;
} __packed;


/**
 * struct CMD_CPU_CLOCK_RATE_SET_REQ
 *        CMD_CPU_CLOCK_RATE_SET_REP
 *        CMD_CPU_CLOCK_RATE_GET_REQ
 *        CMD_CPU_CLOCK_RATE_GET_REP
 *
 * @cluster_class : 0 fixed
 * @cluster_id    : 0x0:A15, 0x1:A7
 * @freqency      : (mHz)
 *
 * This packet format is for the commands
 * CMD_CPU_CLOCK_RATE_SET_REQ and CMD_CPU_CLOCK_RATE_SET_REP
 * CMD_CPU_CLOCK_RATE_GET_REQ and CMD_CPU_CLOCK_RATE_GET_REP.
 * AP populates it as a command. SCB populates it as a response.
 *
 * SCB's response should always show the final status via 'cpu_control'
 */
struct cmd_cpu_control_rate {
	u32 payload_size;
	u32 cluster_class;
	u32 cluster_id;
	u32 cpu_id;
	u64 freqency;
} __packed;


/**
 * @cluster_opp_voltage  : (mV)
 * @cluster_opp_freqency : (mHz)
 */
struct cluster_opp {
	u32 voltage;
	u64 freqency;
} __packed;

/**
 * struct CMD_CLUSTER_OPP_GET_REQ
 *        CMD_CLUSTER_OPP_GET_REP
 *
 * @cmd_status_id       : Parameters for notifying Error Information of the response command
 * @cluster_class       : always fixed the value to be 0
 * @cluster_id          : 0x0:A15, 0x1:A7
 * @opp_num             : elements in the operating points array 'opp_param'
 * @opp_param       : array of OPPs
 *
 * This packet format is for the commands CMD_CLUSTER_OPP_GET_REQ and CMD_CLUSTER_OPP_GET_REP.
 * AP populates it as a command. SCB populates it as a response.
 */
struct cmd_cluster_opp_get {
	u32 payload_size;
	u32 cluster_class;
	u32 cluster_id;
	u32 opp_num;
	struct cluster_opp opp[0];
} __packed;

/* Payload for CMD_THERMAL_INFO_REQ from SCB to AP */
struct mb86s7x_thermal_info {
	u32 payload_size;
	s32 temp[16];
};

/**
 * struct CMD_CLOCK_DSI_PIXEL_REQ
 *        CMD_CLOCK_DSI_PIXEL_REP
 *
 * @mode          : T.B.D
 *
 * This packet format is for the commands CMD_CLOCK_DSI_PIXEL_REQ and CMD_CLOCK_DSI_PIXEL_REP.
 * AP populates it as a command. SCB populates it as a response.
 *
 * SCB's response should always show the final status via 'mode'
 */
struct cmd_clk_dsi_pixel {
	u32 payload_size;
	u32 mode;
} __packed;


/**
 * struct CMD_SCB_CAPABILITY_GET_REQ
 *        CMD_SCB_CAPABILITY_GET_REP
 *
 * @version    : Protocol version
 *
 * This packet format is for the commands CMD_SCB_CAPABILITY_GET_REQ and CMD_SCB_CAPABILITY_GET_REP.
 * AP populates it as a command. SCB populates it as a response.
 *
 * SCB's response should always show the final status via 'specific_info'
 */
struct cmd_scb_version {
	u32 payload_size;
	u32 version;
	u32 config_version;
#define S73_SCB_CAPABILITY0_STR_ENABLE    (1 << 0)
#define S73_SCB_CAPABILITY0_HSSPI_ACCESS  (1 << 1)
#define S7X_SCB_CAPABILITY0_SECURE_AP     (1 << 2)
	u32 capabilities[2];

} __packed;


/**
 * struct CMD_SYS_RESET_CAUSE_GET_REQ
 *        CMD_SYS_RESET_CAUSE_GET_REP
 *
 * @factor : Parameters to notify the reset source
 *
 * This packet format is for the commands CMD_SYS_RESET_CAUSE_GET_REQ and CMD_SYS_RESET_CAUSE_GET_REP.
 * AP populates it as a command. SCB populates it as a response.
 *
 * SCB's response should always show the final status via 'reason'
 */
struct cmd_sys_reset_cause {
	u32 payload_size;
#define RESET_BY_COLDBOOT	0x0
#define RESET_BY_SW		0x1
#define RESET_BY_WDOG		0x2
#define RESET_BY_xxyz		0x3
	u32 cause;
} __packed;


/**
 * struct CMD_SYS_SPECIFIC_INFO_GET_REQ
 *        CMD_SYS_SPECIFIC_INFO_GET_REP
 *
 * @version    : Protocol version
 *
 * This packet format is for the commands CMD_SYS_SPECIFIC_INFO_GET_REQ and CMD_SYS_SPECIFIC_INFO_GET_REP.
 * AP populates it as a command. SCB populates it as a response.
 *
 * SCB's response should always show the final status via 'specific_info'
 */
struct cmd_sys_specific_info {
	u32 payload_size;
	u64 specific_info;
} __packed;

/* Command structure for CMD_REBOOT_AP_AFTER_REQ */
struct cmd_reboot_ap {
	u32 payload_size;
	u32 delay; /* Reboot the AP after 'delay' milliseconds */
} __packed;

/* Used with CMD_GET_WORD_REQ for debugging purposes */
struct cmd_get_word {
	u32 payload_size;
#define REG_TOP_STATUS_CFG_CTL		0x44010000
#define REG_TOP_STATUS_CRG11_ALW	0xF1000000
#define REG_TOP_STATUS_CRG11_DDR3	0xF1010000
#define REG_TOP_STATUS_CRG11_MAIN	0xF1020000
#define REG_TOP_STATUS_CRG11_HDMI	0xF1040000
#define REG_TOP_STATUS_CRG11_DPHY	0xF1050000
#define REG_TOP_STATUS_CRGSYSOC		0xF1080000
#define REG_TOP_STATUS_PRMUX		0xCA4D0000
	u32 address;
	u32 data;
} __packed;

/**
 * struct CMD_TAIKI_REQ
 *        CMD_TAIKI_REP
 *
 * @payload_data : Data address
 *
 * This packet format is for the command CMD_TAIKI_REQ and CMD_TAIKI_REP.
 *
 */
struct cmd_taiki {
	u32	payload_data[64];
} __packed;

/**
 * struct CMD_TAIKI_ASYNC_MSG_REQ
 *        CMD_TAIKI_ASYNC_MSG_REP
 *
 * @payload_data : Data address
 *
 * This packet format is for the command CMD_TAIKI_ASYNC_MSG_REQ and CMD_TAIKI_ASYNC_MSG_REP.
 *
 */
struct cmd_taiki_async_msg {
	u32	payload_data[64];
} __packed;

/**
 * struct CMD_HARD_RESET_REQ
 *        CMD_HARD_RESET_REP
 */
struct cmd_hard_reset {
    u32 payload_size;
    u32 delay;         /* hard-reset after 'delay' ms */
} __attribute__ ((packed));



/**
 * struct CMD_MEMORY_LAYOUT_GET_REQ
 *        CMD_MEMORY_LAYOUT_GET_REP
 */

struct memory_layout_region {
	u64 start;
	u64 length; /* currently only 32-bit region length used by DT */
}  __attribute__ ((packed));

struct cmd_memory_layout {
	u32 payload_size;
	u32 count_regions; /* filled in by SCB */
	struct memory_layout_region regions[0]; /* filled in by SCB */
} __attribute__ ((packed));

/**
 * CMD_POWERDOMAIN_GET_REQ
 * CMD_POWERDOMAIN_GET_REP
 * CMD_POWERDOMAIN_SET_REQ
 * CMD_POWERDOMAIN_SET_REP
 */

struct cmd_powerdomain {
	u32 payload_size;
	u32 powerdomain_index;
	u32 state; /* 0 = OFF, 1 = ON; AP fills for SET, SCB fills for GET */
} __attribute__ ((packed));

/**
 * struct CMD_RESUME_ENTRY_POINT_SET_REQ
 *        CMD_RESUME_ENTRY_POINT_SET_REP
 *        CMD_RESUME_ENTRY_POINT_GET_REQ
 *        CMD_RESUME_ENTRY_POINT_GET_REP
 *
 * @resume_flag : 1 - AP is resumed from retention state.
 *                0 - AP is coldboot'ed.
 *     Valid only in CMD_RESUME_ENTRY_POINT_GET_REP.
 *
 * @resume_entry_point : Primary core's resume entry physical address from retention state.
 *     Valid only in CMD_RESUME_ENTRY_POINT_SET_REQ or in CMD_RESUME_ENTRY_POINT_GET_REP.
 *
 * Tell/Ask SCB of AP's resume entry point
 *
 */
struct cmd_resume_entry_point_msg {
	uint32_t payload_size;
	uint32_t resume_flag;
	uint32_t resume_entry_point;
} __attribute__ ((packed));

#define MB86S70_VERSION_SUPPORT_BLOCKDEV 0x509


#if 0
# At any time, there may be many clients wanting to send a command
   but at most one that could send a reply.
# A reply from remote is always routed to the client whose command
   was the last sent.
# A command from remote is matched against list of register handlers
   and routed to the one for the command if it exists.
#endif

/*
 cmd : Command code
 buf : pointer to command packet
 len : size of buffer = max(sizeof(cmd_packet), sizeof(reply_packet))
 c   : Client never init's the completion.
       If mhu_send_packet() returns 1, client waits on it.
       If mhu_send_packet() returns 0, client assumes 'buf'
        already contains reply and doesn't have to wait on 'c'.
       If mhu_send_packet() returns < 0, submission failed.
       'c == NULL' implies client is in atomic context.
 */
int mhu_send_packet(int cmd, void *buf, int len, struct completion *c);

/* This function _must_ not sleep */
typedef void (*mhu_handler_t)(u32 cmd, u8 rcbuf[]);
int mhu_hndlr_set(u32 cmd, mhu_handler_t);
void mhu_hndlr_clr(u32 cmd, mhu_handler_t);

extern int skip_mhu;
extern const struct cmd_scb_version *mb86s7x_get_scb_version(void);
extern void mb86s70_pm_power_up_setup(unsigned int affinity_level);
extern void mb86s70_reboot(u32 delay);
extern u32 phys_mcpm_entry_point, phys_cpu_reset, boot_cpu_color_offset;

#endif /* __SCB_MHU_API_H */
