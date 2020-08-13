/* SPDX-License-Identifier: GPL-2.0-or-later OR BSD-2-Clause */

/*
 * Realtek RTD1195 pin controller driver
 *
 * Copyright (c) 2019 Realtek Semiconductor Corp.
 */

#ifndef PINCTRL_RTD1195_H
#define PINCTRL_RTD1195_H

enum rtd1195_iso_pins {
	RTD1195_ISO_GPIO_0 = 0,
	RTD1195_ISO_GPIO_1,
	RTD1195_ISO_USB0,
	RTD1195_ISO_USB1,
	RTD1195_ISO_VFD_CS_N,
	RTD1195_ISO_VFD_CLK,
	RTD1195_ISO_VFD_D,
	RTD1195_ISO_IR_RX,
	RTD1195_ISO_IR_TX,
	RTD1195_ISO_UR0_RX,
	RTD1195_ISO_UR0_TX,
	RTD1195_ISO_UR1_RX,
	RTD1195_ISO_UR1_TX,
	RTD1195_ISO_UR1_CTS_N,
	RTD1195_ISO_UR1_RTS_N,
	RTD1195_ISO_I2C_SCL_0,
	RTD1195_ISO_I2C_SDA_0,
	RTD1195_ISO_ETN_LED_LINK,
	RTD1195_ISO_ETN_LED_RXTX,
	RTD1195_ISO_I2C_SCL_6,
	RTD1195_ISO_I2C_SDA_6,
	RTD1195_ISO_AI_LOC,
	RTD1195_ISO_EJTAG_AVCPU_LOC,
	RTD1195_ISO_UR1_LOC,
	RTD1195_ISO_PWM_01_OPEN_DRAIN,
	RTD1195_ISO_PWM_23_OPEN_DRAIN,
};

static const struct pinctrl_pin_desc rtd1195_iso_pins[] = {
	PINCTRL_PIN(RTD1195_ISO_GPIO_0, "iso_gpio_0"),
	PINCTRL_PIN(RTD1195_ISO_GPIO_1, "iso_gpio_1"),
	PINCTRL_PIN(RTD1195_ISO_USB0, "usb0"),
	PINCTRL_PIN(RTD1195_ISO_USB1, "usb1"),
	PINCTRL_PIN(RTD1195_ISO_VFD_CS_N, "vfd_cs_n"),
	PINCTRL_PIN(RTD1195_ISO_VFD_CLK, "vfd_clk"),
	PINCTRL_PIN(RTD1195_ISO_VFD_D, "vfd_d"),
	PINCTRL_PIN(RTD1195_ISO_IR_RX, "ir_rx"),
	PINCTRL_PIN(RTD1195_ISO_IR_TX, "ir_tx"),
	PINCTRL_PIN(RTD1195_ISO_UR0_RX, "ur0_rx"),
	PINCTRL_PIN(RTD1195_ISO_UR0_TX, "ur0_tx"),
	PINCTRL_PIN(RTD1195_ISO_UR1_RX, "ur1_rx"),
	PINCTRL_PIN(RTD1195_ISO_UR1_TX, "ur1_tx"),
	PINCTRL_PIN(RTD1195_ISO_UR1_CTS_N, "ur1_cts_n"),
	PINCTRL_PIN(RTD1195_ISO_UR1_RTS_N, "ur1_rts_n"),
	PINCTRL_PIN(RTD1195_ISO_I2C_SCL_0, "i2c_scl_0"),
	PINCTRL_PIN(RTD1195_ISO_I2C_SDA_0, "i2c_sda_0"),
	PINCTRL_PIN(RTD1195_ISO_ETN_LED_LINK, "etn_led_link"),
	PINCTRL_PIN(RTD1195_ISO_ETN_LED_RXTX, "etn_led_rxtx"),
	PINCTRL_PIN(RTD1195_ISO_I2C_SCL_6, "i2c_scl_6"),
	PINCTRL_PIN(RTD1195_ISO_I2C_SDA_6, "i2c_sda_6"),
	PINCTRL_PIN(RTD1195_ISO_AI_LOC, "ai_loc"),
	PINCTRL_PIN(RTD1195_ISO_EJTAG_AVCPU_LOC, "ejtag_avcpu_loc"),
	PINCTRL_PIN(RTD1195_ISO_UR1_LOC, "ur1_loc"),
	PINCTRL_PIN(RTD1195_ISO_PWM_01_OPEN_DRAIN, "pwm_01_open_drain"),
	PINCTRL_PIN(RTD1195_ISO_PWM_23_OPEN_DRAIN, "pwm_23_open_drain"),
};

static const unsigned int rtd1195_iso_gpio_0_pins[] = { RTD1195_ISO_GPIO_0 };
static const unsigned int rtd1195_iso_gpio_1_pins[] = { RTD1195_ISO_GPIO_1 };
static const unsigned int rtd1195_usb0_pins[] = { RTD1195_ISO_USB0 };
static const unsigned int rtd1195_usb1_pins[] = { RTD1195_ISO_USB1 };
static const unsigned int rtd1195_vfd_cs_n_pins[] = { RTD1195_ISO_VFD_CS_N };
static const unsigned int rtd1195_vfd_clk_pins[] = { RTD1195_ISO_VFD_CLK };
static const unsigned int rtd1195_vfd_d_pins[] = { RTD1195_ISO_VFD_D };
static const unsigned int rtd1195_ir_rx_pins[] = { RTD1195_ISO_IR_RX };
static const unsigned int rtd1195_ir_tx_pins[] = { RTD1195_ISO_IR_TX };
static const unsigned int rtd1195_ur0_rx_pins[] = { RTD1195_ISO_UR0_RX };
static const unsigned int rtd1195_ur0_tx_pins[] = { RTD1195_ISO_UR0_TX };
static const unsigned int rtd1195_ur1_rx_pins[] = { RTD1195_ISO_UR1_RX };
static const unsigned int rtd1195_ur1_tx_pins[] = { RTD1195_ISO_UR1_TX };
static const unsigned int rtd1195_ur1_cts_n_pins[] = { RTD1195_ISO_UR1_CTS_N };
static const unsigned int rtd1195_ur1_rts_n_pins[] = { RTD1195_ISO_UR1_RTS_N };
static const unsigned int rtd1195_i2c_scl_0_pins[] = { RTD1195_ISO_I2C_SCL_0 };
static const unsigned int rtd1195_i2c_sda_0_pins[] = { RTD1195_ISO_I2C_SDA_0 };
static const unsigned int rtd1195_etn_led_link_pins[] = {
				RTD1195_ISO_ETN_LED_LINK };
static const unsigned int rtd1195_etn_led_rxtx_pins[] = {
				RTD1195_ISO_ETN_LED_RXTX };
static const unsigned int rtd1195_i2c_scl_6_pins[] = { RTD1195_ISO_I2C_SCL_6 };
static const unsigned int rtd1195_i2c_sda_6_pins[] = { RTD1195_ISO_I2C_SDA_6 };
static const unsigned int rtd1195_ai_loc_pins[] = { RTD1195_ISO_AI_LOC };
static const unsigned int rtd1195_ejtag_avcpu_loc_pins[] = {
				RTD1195_ISO_EJTAG_AVCPU_LOC };
static const unsigned int rtd1195_ur1_loc_pins[] = { RTD1195_ISO_UR1_LOC };
static const unsigned int rtd1195_pwm_01_open_drain_pins[] = {
				RTD1195_ISO_PWM_01_OPEN_DRAIN };
static const unsigned int rtd1195_pwm_23_open_drain_pins[] = {
				RTD1195_ISO_PWM_23_OPEN_DRAIN };


#define RTD1195_GROUP(_name) \
	{ \
		.name = # _name, \
		.pins = rtd1195_ ## _name ## _pins, \
		.num_pins = ARRAY_SIZE(rtd1195_ ## _name ## _pins), \
	}

static const struct rtd_pin_group_desc rtd1195_iso_pin_groups[] = {
	RTD1195_GROUP(iso_gpio_0),
	RTD1195_GROUP(iso_gpio_1),
	RTD1195_GROUP(usb0),
	RTD1195_GROUP(usb1),
	RTD1195_GROUP(vfd_cs_n),
	RTD1195_GROUP(vfd_clk),
	RTD1195_GROUP(vfd_d),
	RTD1195_GROUP(ir_rx),
	RTD1195_GROUP(ir_tx),
	RTD1195_GROUP(ur0_rx),
	RTD1195_GROUP(ur0_tx),
	RTD1195_GROUP(ur1_rx),
	RTD1195_GROUP(ur1_tx),
	RTD1195_GROUP(ur1_cts_n),
	RTD1195_GROUP(ur1_rts_n),
	RTD1195_GROUP(i2c_scl_0),
	RTD1195_GROUP(i2c_sda_0),
	RTD1195_GROUP(etn_led_link),
	RTD1195_GROUP(etn_led_rxtx),
	RTD1195_GROUP(i2c_scl_6),
	RTD1195_GROUP(i2c_sda_6),
	RTD1195_GROUP(ai_loc),
	RTD1195_GROUP(ejtag_avcpu_loc),
	RTD1195_GROUP(ur1_loc),
	RTD1195_GROUP(pwm_01_open_drain),
	RTD1195_GROUP(pwm_23_open_drain),
};

static const char * const rtd1195_iso_gpio_groups[] = {
	"usb0", "usb1",
	"vfd_cs_n", "vfd_clk", "vfd_d",
	"ir_rx", "ir_tx",
	"ur0_rx", "ur0_tx",
	"ur1_rx", "ur1_tx", "ur1_cts_n", "ur1_rts_n",
	"i2c_scl_0", "i2c_sda_0",
	"etn_led_link", "etn_led_rxtx",
	"i2c_scl_6", "i2c_sda_6",
};
static const char * const rtd1195_iso_ai_ur1_groups[] = {
	"ur1_rx", "ur1_tx", "ur1_cts_n", "ur1_rts_n", "ai_loc"
};

static const char * const rtd1195_iso_ai_vfd_groups[] = {
	"usb0", "usb1", "vfd_cs_n", "vfd_clk", "ai_loc"
};

static const char * const rtd1195_iso_avcpu_ejtag_iso_groups[] = {
	"vfd_cs_n", "vfd_clk", "vfd_d", "usb0", "usb1", "ejtag_avcpu_loc"
};
static const char * const rtd1195_iso_avcpu_ejtag_misc_loc_groups[] = {
	"ejtag_avcpu_loc"
};

static const char * const rtd1195_iso_etn_led_groups[] = {
	"etn_led_link", "etn_led_rxtx"
};
static const char * const rtd1195_iso_i2c0_groups[] = {
	"i2c_scl_0", "i2c_sda_0"
};
static const char * const rtd1195_iso_i2c2_groups[] = { "vfd_d" };
static const char * const rtd1195_iso_i2c3_groups[] = { "ir_tx" };
static const char * const rtd1195_iso_i2c6_groups[] = {
	"i2c_scl_6", "i2c_sda_6"
};
static const char * const rtd1195_iso_ir_rx_groups[] = { "ir_rx" };
static const char * const rtd1195_iso_ir_tx_groups[] = { "ir_tx" };
static const char * const rtd1195_iso_pwm_groups[] = {
	"ur0_rx", "ur0_tx", "ur1_rx", "ur1_tx", "etn_led_link", "etn_led_rxtx"
};
static const char * const rtd1195_iso_standby_dbg_groups[] = {
	"ir_rx", "usb0", "usb1"
};
static const char * const rtd1195_iso_uart0_groups[] = { "ur0_rx", "ur0_tx" };
static const char * const rtd1195_iso_uart1_groups[] = {
	"ur1_rx", "ur1_tx", "ur1_cts_n", "ur1_rts_n", "ur1_loc"
};
static const char * const rtd1195_iso_ur1_misc_groups[] = { "ur1_loc" };
static const char * const rtd1195_iso_vfd_groups[] = {
	"vfd_cs_n", "vfd_clk", "vfd_d"
};
static const char * const rtd1195_iso_pwm_01_normal_groups[] = {
	"pwm_01_open_drain"
};
static const char * const rtd1195_iso_pwm_23_normal_groups[] = {
	"pwm_23_open_drain"
};
static const char * const rtd1195_iso_pwm_01_open_drain_groups[] = {
	"pwm_01_open_drain"
};
static const char * const rtd1195_iso_pwm_23_open_drain_groups[] = {
	"pwm_23_open_drain"

};




#define RTD1195_FUNC(_name) \
	{ \
		.name = # _name, \
		.groups = rtd1195_iso_ ## _name ## _groups, \
		.num_groups = ARRAY_SIZE(rtd1195_iso_ ## _name ## _groups), \
	}

static const struct rtd_pin_func_desc rtd1195_iso_pin_functions[] = {
	RTD1195_FUNC(gpio),
	RTD1195_FUNC(ai_ur1),
	RTD1195_FUNC(ai_vfd),
	RTD1195_FUNC(avcpu_ejtag_iso),
	RTD1195_FUNC(avcpu_ejtag_misc_loc),
	RTD1195_FUNC(etn_led),
	RTD1195_FUNC(i2c0),
	RTD1195_FUNC(i2c2),
	RTD1195_FUNC(i2c3),
	RTD1195_FUNC(i2c6),
	RTD1195_FUNC(ir_rx),
	RTD1195_FUNC(ir_tx),
	RTD1195_FUNC(pwm),
	RTD1195_FUNC(standby_dbg),
	RTD1195_FUNC(uart0),
	RTD1195_FUNC(uart1),
	RTD1195_FUNC(ur1_misc),
	RTD1195_FUNC(vfd),
	RTD1195_FUNC(pwm_01_normal),
	RTD1195_FUNC(pwm_23_normal),
	RTD1195_FUNC(pwm_01_open_drain),
	RTD1195_FUNC(pwm_23_open_drain),
};

#undef RTD1195_FUNC

static const struct rtd_pin_desc rtd1195_iso_muxes[] = {
	RTK_PIN_MUX(vfd_d, 0x10, GENMASK(1, 0),
		RTK_PIN_FUNC(0x0 << 0, "gpio"),
		RTK_PIN_FUNC(0x1 << 0, "vfd"),
		RTK_PIN_FUNC(0x2 << 0, "i2c2"),
		RTK_PIN_FUNC(0x3 << 0, "avcpu_ejtag_iso")),
	RTK_PIN_MUX(vfd_clk, 0x10, GENMASK(3, 2),
		RTK_PIN_FUNC(0x0 << 2, "gpio"),
		RTK_PIN_FUNC(0x1 << 2, "vfd"),
		RTK_PIN_FUNC(0x2 << 2, "ai_vfd"),
		RTK_PIN_FUNC(0x3 << 2, "avcpu_ejtag_iso")),
	RTK_PIN_MUX(vfd_cs_n, 0x10, GENMASK(5, 4),
		RTK_PIN_FUNC(0x0 << 4, "gpio"),
		RTK_PIN_FUNC(0x1 << 4, "vfd"),
		RTK_PIN_FUNC(0x2 << 4, "ai_vfd"),
		RTK_PIN_FUNC(0x3 << 4, "avcpu_ejtag_iso")),
	RTK_PIN_MUX(ir_rx, 0x10, GENMASK(7, 6),
		RTK_PIN_FUNC(0x0 << 6, "gpio"),
		RTK_PIN_FUNC(0x1 << 6, "ir_rx"),
		RTK_PIN_FUNC(0x2 << 6, "standby_dbg")),
	RTK_PIN_MUX(usb0, 0x10, GENMASK(9, 8),
		RTK_PIN_FUNC(0x0 << 8, "gpio"),
		RTK_PIN_FUNC(0x1 << 8, "standby_dbg"),
		RTK_PIN_FUNC(0x2 << 8, "ai_vfd"),
		RTK_PIN_FUNC(0x3 << 8, "avcpu_ejtag_iso")),
	RTK_PIN_MUX(usb1, 0x10, GENMASK(11, 10),
		RTK_PIN_FUNC(0x0 << 10, "gpio"),
		RTK_PIN_FUNC(0x1 << 10, "standby_dbg"),
		RTK_PIN_FUNC(0x2 << 10, "ai_vfd"),
		RTK_PIN_FUNC(0x3 << 10, "avcpu_ejtag_iso")),
	RTK_PIN_MUX(ur1_rx, 0x10, GENMASK(13, 12),
		RTK_PIN_FUNC(0x0 << 12, "gpio"),
		RTK_PIN_FUNC(0x1 << 12, "uart1"),
		RTK_PIN_FUNC(0x2 << 12, "ai_ur1"),
		RTK_PIN_FUNC(0x3 << 12, "pwm")),
	RTK_PIN_MUX(ur1_tx, 0x10, GENMASK(15, 14),
		RTK_PIN_FUNC(0x0 << 14, "gpio"),
		RTK_PIN_FUNC(0x1 << 14, "uart1"),
		RTK_PIN_FUNC(0x2 << 14, "ai_ur1"),
		RTK_PIN_FUNC(0x3 << 14, "pwm")),
	RTK_PIN_MUX(ur1_rts_n, 0x10, GENMASK(17, 16),
		RTK_PIN_FUNC(0x0 << 16, "gpio"),
		RTK_PIN_FUNC(0x1 << 16, "uart1"),
		RTK_PIN_FUNC(0x2 << 16, "ai_ur1")),
	RTK_PIN_MUX(ur1_cts_n, 0x10, GENMASK(19, 18),
		RTK_PIN_FUNC(0x0 << 18, "gpio"),
		RTK_PIN_FUNC(0x1 << 18, "uart1"),
		RTK_PIN_FUNC(0x2 << 18, "ai_ur1")),
	RTK_PIN_MUX(ur0_rx, 0x10, GENMASK(21, 20),
		RTK_PIN_FUNC(0x0 << 20, "gpio"),
		RTK_PIN_FUNC(0x1 << 20, "uart0"),
		RTK_PIN_FUNC(0x2 << 20, "pwm")),
	RTK_PIN_MUX(ur0_tx, 0x10, GENMASK(23, 22),
		RTK_PIN_FUNC(0x0 << 22, "gpio"),
		RTK_PIN_FUNC(0x1 << 22, "uart0"),
		RTK_PIN_FUNC(0x2 << 22, "pwm")),
	RTK_PIN_MUX(i2c_scl_0, 0x10, GENMASK(25, 24),
		RTK_PIN_FUNC(0x0 << 24, "gpio"),
		RTK_PIN_FUNC(0x1 << 24, "i2c0")),
	RTK_PIN_MUX(i2c_sda_0, 0x10, GENMASK(27, 26),
		RTK_PIN_FUNC(0x0 << 26, "gpio"),
		RTK_PIN_FUNC(0x1 << 26, "i2c0")),
	RTK_PIN_MUX(etn_led_link, 0x10, GENMASK(29, 28),
		RTK_PIN_FUNC(0x0 << 28, "gpio"),
		RTK_PIN_FUNC(0x1 << 28, "etn_led"),
		RTK_PIN_FUNC(0x2 << 28, "pwm")),
	RTK_PIN_MUX(etn_led_rxtx, 0x10, GENMASK(31, 30),
		RTK_PIN_FUNC(0x0 << 30, "gpio"),
		RTK_PIN_FUNC(0x1 << 30, "etn_led"),
		RTK_PIN_FUNC(0x2 << 30, "pwm")),

	RTK_PIN_MUX(i2c_scl_6, 0x14, GENMASK(1, 0),
		RTK_PIN_FUNC(0x0 << 0, "gpio"),
		RTK_PIN_FUNC(0x1 << 0, "i2c6")),
	RTK_PIN_MUX(i2c_sda_6, 0x14, GENMASK(3, 2),
		RTK_PIN_FUNC(0x0 << 2, "gpio"),
		RTK_PIN_FUNC(0x1 << 2, "i2c6")),
	RTK_PIN_MUX(ir_tx, 0x14, GENMASK(5, 4),
		RTK_PIN_FUNC(0x0 << 4, "gpio"),
		RTK_PIN_FUNC(0x1 << 4, "ir_tx"),
		RTK_PIN_FUNC(0x2 << 4, "i2c3")),
	RTK_PIN_MUX(pwm_23_open_drain, 0x14, GENMASK(24, 24),
		RTK_PIN_FUNC(0x0 << 24, "pwm_23_normal"),
		RTK_PIN_FUNC(0x1 << 24, "pwm_23_open_drain")),
	RTK_PIN_MUX(pwm_01_open_drain, 0x14, GENMASK(25, 25),
		RTK_PIN_FUNC(0x0 << 25, "pwm_01_normal"),
		RTK_PIN_FUNC(0x1 << 25, "pwm_01_open_drain")),
	RTK_PIN_MUX(ur1_loc, 0x14, GENMASK(27, 26),
		RTK_PIN_FUNC(0x1 << 26, "uart1"),
		RTK_PIN_FUNC(0x2 << 26, "ur1_misc")),
	RTK_PIN_MUX(ejtag_avcpu_loc, 0x14, GENMASK(29, 28),
		RTK_PIN_FUNC(0x1 << 28, "avcpu_ejtag_iso"),
		RTK_PIN_FUNC(0x2 << 28, "avcpu_ejtag_misc_loc")),
	RTK_PIN_MUX(ai_loc, 0x14, GENMASK(31, 30),
		RTK_PIN_FUNC(0x1 << 30, "ai_ur1"),
		RTK_PIN_FUNC(0x2 << 30, "ai_vfd")),
};

static const struct rtd_pin_config_desc rtd1195_iso_configs[] = {
	RTK_PIN_CONFIG(ir_rx, 0x0, 8, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(vfd_cs_n, 0x0, 12, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(vfd_clk, 0x0, 16, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(vfd_d, 0x0, 20, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(i2c_sda_0, 0x0, 24, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(i2c_scl_0, 0x0, 28, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(ur1_rx, 0x4, 0, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(ur1_tx, 0x4, 4, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(ur1_rts_n, 0x4, 8, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(ur1_cts_n, 0x4, 12, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(ur0_rx, 0x4, 16, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(ur0_tx, 0x4, 20, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(etn_led_link, 0x4, 24, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(etn_led_rxtx, 0x4, 28, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(i2c_sda_6, 0x8, 12, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(i2c_scl_6, 0x8, 16, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(ir_tx, 0x8, 20, 1, 0, 2, 3, PADDRI_2_4),
};


static struct rtd_pin_reg_list rtd1195_iso_reg_lists[] = {
	{.reg_offset = 0x0},
	{.reg_offset = 0x4},
	{.reg_offset = 0x8},
	{.reg_offset = 0xc},
	{.reg_offset = 0x10},
	{.reg_offset = 0x14},
};


static const struct rtd_pinctrl_desc rtd1195_iso_pinctrl_desc = {
	.pins = rtd1195_iso_pins,
	.num_pins = ARRAY_SIZE(rtd1195_iso_pins),
	.groups = rtd1195_iso_pin_groups,
	.num_groups = ARRAY_SIZE(rtd1195_iso_pin_groups),
	.functions = rtd1195_iso_pin_functions,
	.num_functions = ARRAY_SIZE(rtd1195_iso_pin_functions),
	.muxes = rtd1195_iso_muxes,
	.num_muxes = ARRAY_SIZE(rtd1195_iso_muxes),
	.configs = rtd1195_iso_configs,
	.num_configs = ARRAY_SIZE(rtd1195_iso_configs),
	.lists = rtd1195_iso_reg_lists,
	.num_regs = ARRAY_SIZE(rtd1195_iso_reg_lists),
};

/* CRT */

enum rtd1195_crt_pins {
	RTD1195_GPIO_0 = 0,
	RTD1195_GPIO_1,
	RTD1195_GPIO_2,
	RTD1195_GPIO_3,
	RTD1195_GPIO_4,
	RTD1195_GPIO_5,
	RTD1195_GPIO_6,
	RTD1195_GPIO_7,
	RTD1195_GPIO_8,
	RTD1195_NF_DD_0,
	RTD1195_NF_DD_1,
	RTD1195_NF_DD_2,
	RTD1195_NF_DD_3,
	RTD1195_NF_DD_4,
	RTD1195_NF_DD_5,
	RTD1195_NF_DD_6,
	RTD1195_NF_DD_7,
	RTD1195_NF_RDY,
	RTD1195_NF_RD_N,
	RTD1195_NF_WR_N,
	RTD1195_NF_ALE,
	RTD1195_NF_CLE,
	RTD1195_NF_CE_N_0,
	RTD1195_NF_CE_N_1,
	RTD1195_MMC_DATA_0,
	RTD1195_MMC_DATA_1,
	RTD1195_MMC_DATA_2,
	RTD1195_MMC_DATA_3,
	RTD1195_MMC_CLK,
	RTD1195_MMC_CMD,
	RTD1195_MMC_WP,
	RTD1195_MMC_CD,
	RTD1195_SDIO_CLK,
	RTD1195_SDIO_DATA_0,
	RTD1195_SDIO_DATA_1,
	RTD1195_SDIO_DATA_2,
	RTD1195_SDIO_DATA_3,
	RTD1195_SDIO_CMD,
	RTD1195_I2C_SCL_5,
	RTD1195_I2C_SDA_5,
	RTD1195_TP1_DATA,
	RTD1195_TP1_CLK,
	RTD1195_TP1_VALID,
	RTD1195_TP1_SYNC,
	RTD1195_TP0_DATA,
	RTD1195_TP0_CLK,
	RTD1195_TP0_VALID,
	RTD1195_TP0_SYNC,
	RTD1195_USB_ID,
	RTD1195_HDMI_HPD,
	RTD1195_SPDIF,
	RTD1195_I2C_SCL_1,
	RTD1195_I2C_SDA_1,
	RTD1195_I2C_SCL_4,
	RTD1195_I2C_SDA_4,
	RTD1195_SENSOR_CKO_0,
	RTD1195_SENSOR_CKO_1,
	RTD1195_SENSOR_RST,
	RTD1195_SENSOR_STB_0,
	RTD1195_SENSOR_STB_1,
	RTD1195_EJTAG_SCPU_LOC,
	RTD1195_HI_LOC,
	RTD1195_AO_LOC,
};

static const struct pinctrl_pin_desc rtd1195_crt_pins[] = {
	PINCTRL_PIN(RTD1195_GPIO_0, "gpio_0"),
	PINCTRL_PIN(RTD1195_GPIO_1, "gpio_1"),
	PINCTRL_PIN(RTD1195_GPIO_2, "gpio_2"),
	PINCTRL_PIN(RTD1195_GPIO_3, "gpio_3"),
	PINCTRL_PIN(RTD1195_GPIO_4, "gpio_4"),
	PINCTRL_PIN(RTD1195_GPIO_5, "gpio_5"),
	PINCTRL_PIN(RTD1195_GPIO_6, "gpio_6"),
	PINCTRL_PIN(RTD1195_GPIO_7, "gpio_7"),
	PINCTRL_PIN(RTD1195_GPIO_8, "gpio_8"),
	PINCTRL_PIN(RTD1195_NF_DD_0, "nf_dd_0"),
	PINCTRL_PIN(RTD1195_NF_DD_1, "nf_dd_1"),
	PINCTRL_PIN(RTD1195_NF_DD_2, "nf_dd_2"),
	PINCTRL_PIN(RTD1195_NF_DD_3, "nf_dd_3"),
	PINCTRL_PIN(RTD1195_NF_DD_4, "nf_dd_4"),
	PINCTRL_PIN(RTD1195_NF_DD_5, "nf_dd_5"),
	PINCTRL_PIN(RTD1195_NF_DD_6, "nf_dd_6"),
	PINCTRL_PIN(RTD1195_NF_DD_7, "nf_dd_7"),
	PINCTRL_PIN(RTD1195_NF_RDY, "nf_rdy"),
	PINCTRL_PIN(RTD1195_NF_RD_N, "nf_rd_n"),
	PINCTRL_PIN(RTD1195_NF_WR_N, "nf_wr_n"),
	PINCTRL_PIN(RTD1195_NF_ALE, "nf_ale"),
	PINCTRL_PIN(RTD1195_NF_CLE, "nf_cle"),
	PINCTRL_PIN(RTD1195_NF_CE_N_0, "nf_ce_n_0"),
	PINCTRL_PIN(RTD1195_NF_CE_N_1, "nf_ce_n_1"),
	PINCTRL_PIN(RTD1195_MMC_DATA_0, "mmc_data_0"),
	PINCTRL_PIN(RTD1195_MMC_DATA_1, "mmc_data_1"),
	PINCTRL_PIN(RTD1195_MMC_DATA_2, "mmc_data_2"),
	PINCTRL_PIN(RTD1195_MMC_DATA_3, "mmc_data_3"),
	PINCTRL_PIN(RTD1195_MMC_CLK, "mmc_clk"),
	PINCTRL_PIN(RTD1195_MMC_CMD, "mmc_cmd"),
	PINCTRL_PIN(RTD1195_MMC_WP, "mmc_wp"),
	PINCTRL_PIN(RTD1195_MMC_CD, "mmc_cd"),
	PINCTRL_PIN(RTD1195_SDIO_CLK, "sdio_clk"),
	PINCTRL_PIN(RTD1195_SDIO_DATA_0, "sdio_data_0"),
	PINCTRL_PIN(RTD1195_SDIO_DATA_1, "sdio_data_1"),
	PINCTRL_PIN(RTD1195_SDIO_DATA_2, "sdio_data_2"),
	PINCTRL_PIN(RTD1195_SDIO_DATA_3, "sdio_data_3"),
	PINCTRL_PIN(RTD1195_SDIO_CMD, "sdio_cmd"),
	PINCTRL_PIN(RTD1195_I2C_SCL_5, "i2c_scl_5"),
	PINCTRL_PIN(RTD1195_I2C_SDA_5, "i2c_sda_5"),
	PINCTRL_PIN(RTD1195_TP1_DATA, "tp1_data"),
	PINCTRL_PIN(RTD1195_TP1_CLK, "tp1_clk"),
	PINCTRL_PIN(RTD1195_TP1_VALID, "tp1_valid"),
	PINCTRL_PIN(RTD1195_TP1_SYNC, "tp1_sync"),
	PINCTRL_PIN(RTD1195_TP0_DATA, "tp0_data"),
	PINCTRL_PIN(RTD1195_TP0_CLK, "tp0_clk"),
	PINCTRL_PIN(RTD1195_TP0_VALID, "tp0_valid"),
	PINCTRL_PIN(RTD1195_TP0_SYNC, "tp0_sync"),
	PINCTRL_PIN(RTD1195_USB_ID, "usb_id"),
	PINCTRL_PIN(RTD1195_HDMI_HPD, "hdmi_hpd"),
	PINCTRL_PIN(RTD1195_SPDIF, "spdif"),
	PINCTRL_PIN(RTD1195_I2C_SCL_1, "i2c_scl_1"),
	PINCTRL_PIN(RTD1195_I2C_SDA_1, "i2c_sda_1"),
	PINCTRL_PIN(RTD1195_I2C_SCL_4, "i2c_scl_4"),
	PINCTRL_PIN(RTD1195_I2C_SDA_4, "i2c_sda_4"),
	PINCTRL_PIN(RTD1195_SENSOR_CKO_0, "sensor_cko_0"),
	PINCTRL_PIN(RTD1195_SENSOR_CKO_1, "sensor_cko_1"),
	PINCTRL_PIN(RTD1195_SENSOR_RST, "sensor_rst"),
	PINCTRL_PIN(RTD1195_SENSOR_STB_0, "sensor_stb_0"),
	PINCTRL_PIN(RTD1195_SENSOR_STB_1, "sensor_stb_1"),
	PINCTRL_PIN(RTD1195_EJTAG_SCPU_LOC, "ejtag_scpu_loc"),
	PINCTRL_PIN(RTD1195_HI_LOC, "hif_loc"),
	PINCTRL_PIN(RTD1195_AO_LOC, "ao_loc"),
};

static const unsigned int rtd1195_gpio_0_pins[] = { RTD1195_GPIO_0 };
static const unsigned int rtd1195_gpio_1_pins[] = { RTD1195_GPIO_1 };
static const unsigned int rtd1195_gpio_2_pins[] = { RTD1195_GPIO_2 };
static const unsigned int rtd1195_gpio_3_pins[] = { RTD1195_GPIO_3 };
static const unsigned int rtd1195_gpio_4_pins[] = { RTD1195_GPIO_4 };
static const unsigned int rtd1195_gpio_5_pins[] = { RTD1195_GPIO_5 };
static const unsigned int rtd1195_gpio_6_pins[] = { RTD1195_GPIO_6 };
static const unsigned int rtd1195_gpio_7_pins[] = { RTD1195_GPIO_7 };
static const unsigned int rtd1195_gpio_8_pins[] = { RTD1195_GPIO_8 };
static const unsigned int rtd1195_nf_dd_0_pins[] = { RTD1195_NF_DD_0 };
static const unsigned int rtd1195_nf_dd_1_pins[] = { RTD1195_NF_DD_1 };
static const unsigned int rtd1195_nf_dd_2_pins[] = { RTD1195_NF_DD_2 };
static const unsigned int rtd1195_nf_dd_3_pins[] = { RTD1195_NF_DD_3 };
static const unsigned int rtd1195_nf_dd_4_pins[] = { RTD1195_NF_DD_4 };
static const unsigned int rtd1195_nf_dd_5_pins[] = { RTD1195_NF_DD_5 };
static const unsigned int rtd1195_nf_dd_6_pins[] = { RTD1195_NF_DD_6 };
static const unsigned int rtd1195_nf_dd_7_pins[] = { RTD1195_NF_DD_7 };
static const unsigned int rtd1195_nf_rdy_pins[] = { RTD1195_NF_RDY };
static const unsigned int rtd1195_nf_rd_n_pins[] = { RTD1195_NF_RD_N };
static const unsigned int rtd1195_nf_wr_n_pins[] = { RTD1195_NF_WR_N };
static const unsigned int rtd1195_nf_ale_pins[] = { RTD1195_NF_ALE };
static const unsigned int rtd1195_nf_cle_pins[] = { RTD1195_NF_CLE };
static const unsigned int rtd1195_nf_ce_n_0_pins[] = { RTD1195_NF_CE_N_0 };
static const unsigned int rtd1195_nf_ce_n_1_pins[] = { RTD1195_NF_CE_N_1 };
static const unsigned int rtd1195_mmc_data_0_pins[] = { RTD1195_MMC_DATA_0 };
static const unsigned int rtd1195_mmc_data_1_pins[] = { RTD1195_MMC_DATA_1 };
static const unsigned int rtd1195_mmc_data_2_pins[] = { RTD1195_MMC_DATA_2 };
static const unsigned int rtd1195_mmc_data_3_pins[] = { RTD1195_MMC_DATA_3 };
static const unsigned int rtd1195_mmc_clk_pins[] = { RTD1195_MMC_CLK };
static const unsigned int rtd1195_mmc_cmd_pins[] = { RTD1195_MMC_CMD };
static const unsigned int rtd1195_mmc_wp_pins[] = { RTD1195_MMC_WP };
static const unsigned int rtd1195_mmc_cd_pins[] = { RTD1195_MMC_CD };
static const unsigned int rtd1195_sdio_clk_pins[] = { RTD1195_SDIO_CLK };
static const unsigned int rtd1195_sdio_data_0_pins[] = { RTD1195_SDIO_DATA_0 };
static const unsigned int rtd1195_sdio_data_1_pins[] = { RTD1195_SDIO_DATA_1 };
static const unsigned int rtd1195_sdio_data_2_pins[] = { RTD1195_SDIO_DATA_2 };
static const unsigned int rtd1195_sdio_data_3_pins[] = { RTD1195_SDIO_DATA_3 };
static const unsigned int rtd1195_sdio_cmd_pins[] = { RTD1195_SDIO_CMD };
static const unsigned int rtd1195_i2c_scl_5_pins[] = { RTD1195_I2C_SCL_5 };
static const unsigned int rtd1195_i2c_sda_5_pins[] = { RTD1195_I2C_SDA_5 };
static const unsigned int rtd1195_tp1_data_pins[] = { RTD1195_TP1_DATA };
static const unsigned int rtd1195_tp1_clk_pins[] = { RTD1195_TP1_CLK };
static const unsigned int rtd1195_tp1_valid_pins[] = { RTD1195_TP1_VALID };
static const unsigned int rtd1195_tp1_sync_pins[] = { RTD1195_TP1_SYNC };
static const unsigned int rtd1195_tp0_data_pins[] = { RTD1195_TP0_DATA };
static const unsigned int rtd1195_tp0_clk_pins[] = { RTD1195_TP0_CLK };
static const unsigned int rtd1195_tp0_valid_pins[] = { RTD1195_TP0_VALID };
static const unsigned int rtd1195_tp0_sync_pins[] = { RTD1195_TP0_SYNC };
static const unsigned int rtd1195_usb_id_pins[] = { RTD1195_USB_ID };
static const unsigned int rtd1195_hdmi_hpd_pins[] = { RTD1195_HDMI_HPD };
static const unsigned int rtd1195_spdif_pins[] = { RTD1195_SPDIF };
static const unsigned int rtd1195_i2c_scl_1_pins[] = { RTD1195_I2C_SCL_1 };
static const unsigned int rtd1195_i2c_sda_1_pins[] = { RTD1195_I2C_SDA_1 };
static const unsigned int rtd1195_i2c_scl_4_pins[] = { RTD1195_I2C_SCL_4 };
static const unsigned int rtd1195_i2c_sda_4_pins[] = { RTD1195_I2C_SDA_4 };
static const unsigned int rtd1195_sensor_cko_0_pins[] = {
					RTD1195_SENSOR_CKO_0 };
static const unsigned int rtd1195_sensor_cko_1_pins[] = {
					RTD1195_SENSOR_CKO_1 };
static const unsigned int rtd1195_sensor_rst_pins[] = { RTD1195_SENSOR_RST };
static const unsigned int rtd1195_sensor_stb_0_pins[] = {
					RTD1195_SENSOR_STB_0 };
static const unsigned int rtd1195_sensor_stb_1_pins[] = {
					RTD1195_SENSOR_STB_1 };
static const unsigned int rtd1195_ejtag_scpu_loc_pins[] = {
					RTD1195_EJTAG_SCPU_LOC };
static const unsigned int rtd1195_hif_loc_pins[] = { RTD1195_HI_LOC };
static const unsigned int rtd1195_ao_loc_pins[] = { RTD1195_AO_LOC };






#define RTD1195_GROUP(_name) \
	{ \
		.name = # _name, \
		.pins = rtd1195_ ## _name ## _pins, \
		.num_pins = ARRAY_SIZE(rtd1195_ ## _name ## _pins), \
	}

static const struct rtd_pin_group_desc rtd1195_crt_pin_groups[] = {
	RTD1195_GROUP(gpio_0),
	RTD1195_GROUP(gpio_1),
	RTD1195_GROUP(gpio_2),
	RTD1195_GROUP(gpio_3),
	RTD1195_GROUP(gpio_4),
	RTD1195_GROUP(gpio_5),
	RTD1195_GROUP(gpio_6),
	RTD1195_GROUP(gpio_7),
	RTD1195_GROUP(gpio_8),
	RTD1195_GROUP(nf_dd_0),
	RTD1195_GROUP(nf_dd_1),
	RTD1195_GROUP(nf_dd_2),
	RTD1195_GROUP(nf_dd_3),
	RTD1195_GROUP(nf_dd_4),
	RTD1195_GROUP(nf_dd_5),
	RTD1195_GROUP(nf_dd_6),
	RTD1195_GROUP(nf_dd_7),
	RTD1195_GROUP(nf_rdy),
	RTD1195_GROUP(nf_rd_n),
	RTD1195_GROUP(nf_wr_n),
	RTD1195_GROUP(nf_ale),
	RTD1195_GROUP(nf_cle),
	RTD1195_GROUP(nf_ce_n_0),
	RTD1195_GROUP(nf_ce_n_1),
	RTD1195_GROUP(mmc_data_0),
	RTD1195_GROUP(mmc_data_1),
	RTD1195_GROUP(mmc_data_2),
	RTD1195_GROUP(mmc_data_3),
	RTD1195_GROUP(mmc_clk),
	RTD1195_GROUP(mmc_cmd),
	RTD1195_GROUP(mmc_wp),
	RTD1195_GROUP(mmc_cd),
	RTD1195_GROUP(sdio_clk),
	RTD1195_GROUP(sdio_data_0),
	RTD1195_GROUP(sdio_data_1),
	RTD1195_GROUP(sdio_data_2),
	RTD1195_GROUP(sdio_data_3),
	RTD1195_GROUP(sdio_cmd),
	RTD1195_GROUP(i2c_scl_5),
	RTD1195_GROUP(i2c_sda_5),
	RTD1195_GROUP(tp1_data),
	RTD1195_GROUP(tp1_clk),
	RTD1195_GROUP(tp1_valid),
	RTD1195_GROUP(tp1_sync),
	RTD1195_GROUP(tp0_data),
	RTD1195_GROUP(tp0_clk),
	RTD1195_GROUP(tp0_valid),
	RTD1195_GROUP(tp0_sync),
	RTD1195_GROUP(usb_id),
	RTD1195_GROUP(hdmi_hpd),
	RTD1195_GROUP(spdif),
	RTD1195_GROUP(i2c_scl_1),
	RTD1195_GROUP(i2c_sda_1),
	RTD1195_GROUP(i2c_scl_4),
	RTD1195_GROUP(i2c_sda_4),
	RTD1195_GROUP(sensor_cko_0),
	RTD1195_GROUP(sensor_cko_1),
	RTD1195_GROUP(sensor_rst),
	RTD1195_GROUP(sensor_stb_0),
	RTD1195_GROUP(sensor_stb_1),
	RTD1195_GROUP(ejtag_scpu_loc),
	RTD1195_GROUP(hif_loc),
	RTD1195_GROUP(ao_loc),
};

static const char * const rtd1195_crt_gpio_groups[] = {
	"gpio_0", "gpio_1", "gpio_2", "gpio_3",
	"gpio_4", "gpio_5", "gpio_6", "gpio_7", "gpio_8",
	"nf_dd_0", "nf_dd_1", "nf_dd_2", "nf_dd_3",
	"nf_dd_4", "nf_dd_5", "nf_dd_6", "nf_dd_7",
	"nf_rdy", "nf_rd_n", "nf_wr_n", "nf_ale",
	"nf_cle", "nf_ce_n_0", "nf_ce_n_1",
	"mmc_data_0", "mmc_data_1", "mmc_data_2", "mmc_data_3",
	"mmc_clk", "mmc_cmd", "mmc_wp", "mmc_cd",
	"sdio_clk", "sdio_data_0", "sdio_data_1",
	"sdio_data_2", "sdio_data_3", "sdio_cmd",
	"i2c_scl_5", "i2c_sda_5",
	"tp1_data", "tp1_clk", "tp1_valid", "tp1_sync",
	"tp0_data", "tp0_clk", "tp0_valid", "tp0_sync",
	"usb_id",
	"hdmi_hpd",
	"spdif",
	"i2c_scl_1", "i2c_sda_1",
	"i2c_scl_4", "i2c_sda_4",
	"sensor_cko_0", "sensor_cko_1",
	"sensor_rst",
	"sensor_stb_0", "sensor_stb_1",
};

static const char * const rtd1195_crt_ao_tp0_groups[] = {
	"tp0_data", "tp0_sync", "tp0_valid", "tp0_clk", "ao_loc"
};

static const char * const rtd1195_crt_ao_gpio_groups[] = {
	"gpio_4", "gpio_5", "gpio_6", "gpio_7", "ao_loc"
};


static const char * const rtd1195_crt_avcpu_ejtag_misc_groups[] = {
	"nf_rdy", "nf_rd_n", "nf_dd_5", "nf_dd_6", "nf_dd_7"
};
static const char * const rtd1195_crt_cpu_loop_groups[] = { "usb_id" };
static const char * const rtd1195_crt_emmc_groups[] = {
	"nf_dd_0", "nf_dd_1", "nf_dd_2", "nf_dd_3",
	"nf_dd_4", "nf_dd_5", "nf_dd_6", "nf_dd_7",
	"nf_rdy", "nf_rd_n", "nf_wr_n", "nf_ale", "nf_cle",
};
static const char * const rtd1195_crt_gspi_groups[] = {
	"gpio_0", "gpio_1", "gpio_2", "gpio_3",
};
static const char * const rtd1195_crt_hif_nf_groups[] = {
	"nf_dd_4", "nf_wr_n", "nf_ale", "nf_cle", "hif_loc"
};

static const char * const rtd1195_crt_hif_misc_groups[] = {
	"gpio_0", "gpio_1", "gpio_2", "gpio_3", "hif_loc"
};


static const char * const rtd1195_crt_i2c1_groups[] = {
	"i2c_scl_1", "i2c_sda_1"
};
static const char * const rtd1195_crt_i2c2_groups[] = {
	"tp1_sync", "tp1_clk"
};
static const char * const rtd1195_crt_i2c3_groups[] = {
	"tp1_data", "tp1_valid" };
static const char * const rtd1195_crt_i2c4_groups[] = {
	"i2c_scl_4", "i2c_sda_4"
};
static const char * const rtd1195_crt_i2c5_groups[] = {
	"i2c_scl_5", "i2c_sda_5"
};
static const char * const rtd1195_crt_mmc_groups[] = {
	"mmc_data_0", "mmc_data_1", "mmc_data_2", "mmc_data_3",
	"mmc_clk", "mmc_cmd", "mmc_wp", "mmc_cd",
};
static const char * const rtd1195_crt_nand_groups[] = {
	"nf_dd_0", "nf_dd_1", "nf_dd_2", "nf_dd_3",
	"nf_dd_4", "nf_dd_5", "nf_dd_6", "nf_dd_7",
	"nf_rdy", "nf_rd_n", "nf_wr_n", "nf_ale",
	"nf_cle", "nf_ce_n_0", "nf_ce_n_1",
};
static const char * const rtd1195_crt_scpu_ejtag_gpio_groups[] = {
	"gpio_4", "gpio_5", "gpio_6", "gpio_7", "gpio_8", "ejtag_scpu_loc"
};
static const char * const rtd1195_crt_scpu_ejtag_cr_groups[] = {
	"mmc_data_0", "mmc_data_3", "mmc_clk",
	"mmc_cmd", "mmc_wp", "ejtag_scpu_loc"
};

static const char * const rtd1195_crt_sdio_groups[] = {
	"sdio_clk", "sdio_data_0", "sdio_data_1",
	"sdio_data_2", "sdio_data_3", "sdio_cmd",
};
static const char * const rtd1195_crt_sensor_groups[] = {
	"sensor_cko_0", "sensor_cko_1"
};
static const char * const rtd1195_crt_spdif_groups[] = { "spdif" };
static const char * const rtd1195_crt_tp0_groups[] = {
	"tp0_data", "tp0_clk", "tp0_valid", "tp0_sync",
};
static const char * const rtd1195_crt_tp1_groups[] = {
	"tp1_data", "tp1_clk", "tp1_valid", "tp1_sync",
};
static const char * const rtd1195_crt_uart1_groups[] = {
	"gpio_0", "gpio_1", "gpio_2", "gpio_3"
};
static const char * const rtd1195_crt_usb_groups[] = { "sensor_cko_1" };

#define RTD1195_FUNC(_name) \
	{ \
		.name = # _name, \
		.groups = rtd1195_crt_ ## _name ## _groups, \
		.num_groups = ARRAY_SIZE(rtd1195_crt_ ## _name ## _groups), \
	}

static const struct rtd_pin_func_desc rtd1195_crt_pin_functions[] = {
	RTD1195_FUNC(gpio),
	RTD1195_FUNC(ao_tp0),
	RTD1195_FUNC(ao_gpio),
	RTD1195_FUNC(avcpu_ejtag_misc),
	RTD1195_FUNC(cpu_loop),
	RTD1195_FUNC(emmc),
	RTD1195_FUNC(gspi),
	RTD1195_FUNC(hif_misc),
	RTD1195_FUNC(hif_nf),
	RTD1195_FUNC(i2c1),
	RTD1195_FUNC(i2c2),
	RTD1195_FUNC(i2c3),
	RTD1195_FUNC(i2c4),
	RTD1195_FUNC(i2c5),
	RTD1195_FUNC(mmc),
	RTD1195_FUNC(nand),
	RTD1195_FUNC(scpu_ejtag_gpio),
	RTD1195_FUNC(scpu_ejtag_cr),
	RTD1195_FUNC(sdio),
	RTD1195_FUNC(sensor),
	RTD1195_FUNC(spdif),
	RTD1195_FUNC(tp0),
	RTD1195_FUNC(tp1),
	RTD1195_FUNC(uart1),
	RTD1195_FUNC(usb),
};

#undef RTD1195_FUNC

static const struct rtd_pin_desc rtd1195_crt_muxes[] = {
	RTK_PIN_MUX(nf_rdy, 0x60, GENMASK(3, 2),
		RTK_PIN_FUNC(0x0 << 2, "gpio"),
		RTK_PIN_FUNC(0x1 << 2, "nand"),
		RTK_PIN_FUNC(0x2 << 2, "emmc"),
		RTK_PIN_FUNC(0x3 << 2, "avcpu_ejtag_misc")),
	RTK_PIN_MUX(nf_rd_n, 0x60, GENMASK(5, 4),
		RTK_PIN_FUNC(0x0 << 4, "gpio"),
		RTK_PIN_FUNC(0x1 << 4, "nand"),
		RTK_PIN_FUNC(0x2 << 4, "emmc"),
		RTK_PIN_FUNC(0x3 << 4, "avcpu_ejtag_misc")),
	RTK_PIN_MUX(nf_wr_n, 0x60, GENMASK(7, 6),
		RTK_PIN_FUNC(0x0 << 6, "gpio"),
		RTK_PIN_FUNC(0x1 << 6, "nand"),
		RTK_PIN_FUNC(0x2 << 6, "emmc"),
		RTK_PIN_FUNC(0x3 << 6, "hif_nf")),
	RTK_PIN_MUX(nf_ale, 0x60, GENMASK(9, 8),
		RTK_PIN_FUNC(0x0 << 8, "gpio"),
		RTK_PIN_FUNC(0x1 << 8, "nand"),
		RTK_PIN_FUNC(0x2 << 8, "emmc"),
		RTK_PIN_FUNC(0x3 << 8, "hif_nf")),
	RTK_PIN_MUX(nf_cle, 0x60, GENMASK(11, 10),
		RTK_PIN_FUNC(0x0 << 10, "gpio"),
		RTK_PIN_FUNC(0x1 << 10, "nand"),
		RTK_PIN_FUNC(0x2 << 10, "emmc"),
		RTK_PIN_FUNC(0x3 << 10, "hif_nf")),
	RTK_PIN_MUX(nf_ce_n_0, 0x60, GENMASK(13, 12),
		RTK_PIN_FUNC(0x0 << 12, "gpio"),
		RTK_PIN_FUNC(0x1 << 12, "nand")),
	RTK_PIN_MUX(nf_ce_n_1, 0x60, GENMASK(15, 14),
		RTK_PIN_FUNC(0x0 << 14, "gpio"),
		RTK_PIN_FUNC(0x1 << 14, "nand")),
	RTK_PIN_MUX(nf_dd_0, 0x60, GENMASK(17, 16),
		RTK_PIN_FUNC(0x0 << 16, "gpio"),
		RTK_PIN_FUNC(0x1 << 16, "nand"),
		RTK_PIN_FUNC(0x2 << 16, "emmc")),
	RTK_PIN_MUX(nf_dd_1, 0x60, GENMASK(19, 18),
		RTK_PIN_FUNC(0x0 << 18, "gpio"),
		RTK_PIN_FUNC(0x1 << 18, "nand"),
		RTK_PIN_FUNC(0x2 << 18, "emmc")),
	RTK_PIN_MUX(nf_dd_2, 0x60, GENMASK(21, 20),
		RTK_PIN_FUNC(0x0 << 20, "gpio"),
		RTK_PIN_FUNC(0x1 << 20, "nand"),
		RTK_PIN_FUNC(0x2 << 20, "emmc")),
	RTK_PIN_MUX(nf_dd_3, 0x60, GENMASK(23, 22),
		RTK_PIN_FUNC(0x0 << 22, "gpio"),
		RTK_PIN_FUNC(0x1 << 22, "nand"),
		RTK_PIN_FUNC(0x2 << 22, "emmc")),
	RTK_PIN_MUX(nf_dd_4, 0x60, GENMASK(25, 24),
		RTK_PIN_FUNC(0x0 << 24, "gpio"),
		RTK_PIN_FUNC(0x1 << 24, "nand"),
		RTK_PIN_FUNC(0x2 << 24, "emmc"),
		RTK_PIN_FUNC(0x3 << 24, "hif_nf")),
	RTK_PIN_MUX(nf_dd_5, 0x60, GENMASK(27, 26),
		RTK_PIN_FUNC(0x0 << 26, "gpio"),
		RTK_PIN_FUNC(0x1 << 26, "nand"),
		RTK_PIN_FUNC(0x2 << 26, "emmc"),
		RTK_PIN_FUNC(0x3 << 26, "avcpu_ejtag_misc")),
	RTK_PIN_MUX(nf_dd_6, 0x60, GENMASK(29, 28),
		RTK_PIN_FUNC(0x0 << 28, "gpio"),
		RTK_PIN_FUNC(0x1 << 28, "nand"),
		RTK_PIN_FUNC(0x2 << 28, "emmc"),
		RTK_PIN_FUNC(0x3 << 28, "avcpu_ejtag_misc")),
	RTK_PIN_MUX(nf_dd_7, 0x60, GENMASK(31, 30),
		RTK_PIN_FUNC(0x0 << 30, "gpio"),
		RTK_PIN_FUNC(0x1 << 30, "nand"),
		RTK_PIN_FUNC(0x2 << 30, "emmc"),
		RTK_PIN_FUNC(0x3 << 30, "avcpu_ejtag_misc")),

	RTK_PIN_MUX(sdio_cmd, 0x64, GENMASK(1, 0),
		RTK_PIN_FUNC(0x0 << 0, "gpio"),
		RTK_PIN_FUNC(0x1 << 0, "sdio")),
	RTK_PIN_MUX(sdio_clk, 0x64, GENMASK(3, 2),
		RTK_PIN_FUNC(0x0 << 2, "gpio"),
		RTK_PIN_FUNC(0x1 << 2, "sdio")),
	RTK_PIN_MUX(sdio_data_0, 0x64, GENMASK(5, 4),
		RTK_PIN_FUNC(0x0 << 4, "gpio"),
		RTK_PIN_FUNC(0x1 << 4, "sdio")),
	RTK_PIN_MUX(sdio_data_1, 0x64, GENMASK(7, 6),
		RTK_PIN_FUNC(0x0 << 6, "gpio"),
		RTK_PIN_FUNC(0x1 << 6, "sdio")),
	RTK_PIN_MUX(sdio_data_2, 0x64, GENMASK(9, 8),
		RTK_PIN_FUNC(0x0 << 8, "gpio"),
		RTK_PIN_FUNC(0x1 << 8, "sdio")),
	RTK_PIN_MUX(sdio_data_3, 0x64, GENMASK(11, 10),
		RTK_PIN_FUNC(0x0 << 10, "gpio"),
		RTK_PIN_FUNC(0x1 << 10, "sdio")),

	RTK_PIN_MUX(mmc_cmd, 0x64, GENMASK(17, 16),
		RTK_PIN_FUNC(0x0 << 16, "gpio"),
		RTK_PIN_FUNC(0x1 << 16, "mmc"),
		RTK_PIN_FUNC(0x3 << 16, "scpu_ejtag_cr")),
	RTK_PIN_MUX(mmc_clk, 0x64, GENMASK(19, 18),
		RTK_PIN_FUNC(0x0 << 18, "gpio"),
		RTK_PIN_FUNC(0x1 << 18, "mmc"),
		RTK_PIN_FUNC(0x3 << 18, "scpu_ejtag_cr")),
	RTK_PIN_MUX(mmc_wp, 0x64, GENMASK(21, 20),
		RTK_PIN_FUNC(0x0 << 20, "gpio"),
		RTK_PIN_FUNC(0x1 << 20, "mmc"),
		RTK_PIN_FUNC(0x3 << 20, "scpu_ejtag_cr")),
	RTK_PIN_MUX(mmc_cd, 0x64, GENMASK(23, 22),
		RTK_PIN_FUNC(0x0 << 22, "gpio"),
		RTK_PIN_FUNC(0x1 << 22, "mmc")),
	RTK_PIN_MUX(mmc_data_0, 0x64, GENMASK(25, 24),
		RTK_PIN_FUNC(0x0 << 24, "gpio"),
		RTK_PIN_FUNC(0x1 << 24, "mmc"),
		RTK_PIN_FUNC(0x3 << 24, "scpu_ejtag_cr")),
	RTK_PIN_MUX(mmc_data_1, 0x64, GENMASK(27, 26),
		RTK_PIN_FUNC(0x0 << 26, "gpio"),
		RTK_PIN_FUNC(0x1 << 26, "mmc")),
	RTK_PIN_MUX(mmc_data_2, 0x64, GENMASK(29, 28),
		RTK_PIN_FUNC(0x0 << 28, "gpio"),
		RTK_PIN_FUNC(0x1 << 28, "mmc")),
	RTK_PIN_MUX(mmc_data_3, 0x64, GENMASK(31, 30),
		RTK_PIN_FUNC(0x0 << 30, "gpio"),
		RTK_PIN_FUNC(0x1 << 30, "mmc"),
		RTK_PIN_FUNC(0x3 << 30, "scpu_ejtag_cr")),

	RTK_PIN_MUX(tp0_data, 0x68, GENMASK(1, 0),
		RTK_PIN_FUNC(0x0 << 0, "tp0"),
		RTK_PIN_FUNC(0x1 << 0, "tp1"),
		RTK_PIN_FUNC(0x2 << 0, "gpio"),
		RTK_PIN_FUNC(0x3 << 0, "ao_tp0")),
	RTK_PIN_MUX(tp0_sync, 0x68, GENMASK(3, 2),
		RTK_PIN_FUNC(0x0 << 2, "tp0"),
		RTK_PIN_FUNC(0x1 << 2, "tp1"),
		RTK_PIN_FUNC(0x2 << 2, "gpio"),
		RTK_PIN_FUNC(0x3 << 2, "ao_tp0")),
	RTK_PIN_MUX(tp0_valid, 0x68, GENMASK(5, 4),
		RTK_PIN_FUNC(0x0 << 4, "tp0"),
		RTK_PIN_FUNC(0x1 << 4, "tp1"),
		RTK_PIN_FUNC(0x2 << 4, "gpio"),
		RTK_PIN_FUNC(0x3 << 4, "ao_tp0")),
	RTK_PIN_MUX(tp0_clk, 0x68, GENMASK(7, 6),
		RTK_PIN_FUNC(0x0 << 6, "tp0"),
		RTK_PIN_FUNC(0x1 << 6, "tp1"),
		RTK_PIN_FUNC(0x2 << 6, "gpio"),
		RTK_PIN_FUNC(0x3 << 6, "ao_tp0")),
	RTK_PIN_MUX(tp1_data, 0x68, GENMASK(17, 16),
		RTK_PIN_FUNC(0x0 << 16, "tp1"),
		RTK_PIN_FUNC(0x1 << 16, "tp0"),
		RTK_PIN_FUNC(0x2 << 16, "gpio"),
		RTK_PIN_FUNC(0x3 << 16, "i2c3")),
	RTK_PIN_MUX(tp1_sync, 0x68, GENMASK(19, 18),
		RTK_PIN_FUNC(0x0 << 18, "tp1"),
		RTK_PIN_FUNC(0x1 << 18, "tp0"),
		RTK_PIN_FUNC(0x2 << 18, "gpio"),
		RTK_PIN_FUNC(0x3 << 18, "i2c2")),
	RTK_PIN_MUX(tp1_valid, 0x68, GENMASK(21, 20),
		RTK_PIN_FUNC(0x0 << 20, "tp1"),
		RTK_PIN_FUNC(0x1 << 20, "tp0"),
		RTK_PIN_FUNC(0x2 << 20, "gpio"),
		RTK_PIN_FUNC(0x3 << 20, "i2c3")),
	RTK_PIN_MUX(tp1_clk, 0x68, GENMASK(23, 22),
		RTK_PIN_FUNC(0x0 << 22, "tp1"),
		RTK_PIN_FUNC(0x1 << 22, "tp0"),
		RTK_PIN_FUNC(0x2 << 22, "gpio"),
		RTK_PIN_FUNC(0x3 << 22, "i2c2")),

	RTK_PIN_MUX(i2c_sda_1, 0x6c, GENMASK(1, 0),
		RTK_PIN_FUNC(0x0 << 0, "gpio"),
		RTK_PIN_FUNC(0x1 << 0, "i2c1")),
	RTK_PIN_MUX(i2c_scl_1, 0x6c, GENMASK(3, 2),
		RTK_PIN_FUNC(0x0 << 2, "gpio"),
		RTK_PIN_FUNC(0x1 << 2, "i2c1")),
	RTK_PIN_MUX(i2c_sda_4, 0x6c, GENMASK(5, 4),
		RTK_PIN_FUNC(0x0 << 4, "gpio"),
		RTK_PIN_FUNC(0x1 << 4, "i2c4")),
	RTK_PIN_MUX(i2c_scl_4, 0x6c, GENMASK(7, 6),
		RTK_PIN_FUNC(0x0 << 6, "gpio"),
		RTK_PIN_FUNC(0x1 << 6, "i2c4")),
	RTK_PIN_MUX(i2c_sda_5, 0x6c, GENMASK(9, 8),
		RTK_PIN_FUNC(0x0 << 8, "gpio"),
		RTK_PIN_FUNC(0x1 << 8, "i2c5"),
		RTK_PIN_FUNC(0x3 << 8, "nand")),
	RTK_PIN_MUX(i2c_scl_5, 0x6c, GENMASK(11, 10),
		RTK_PIN_FUNC(0x0 << 10, "gpio"),
		RTK_PIN_FUNC(0x1 << 10, "i2c5"),
		RTK_PIN_FUNC(0x3 << 10, "nand")),
	RTK_PIN_MUX(spdif, 0x6c, GENMASK(13, 12),
		RTK_PIN_FUNC(0x0 << 12, "gpio"),
		RTK_PIN_FUNC(0x1 << 12, "spdif")),
	RTK_PIN_MUX(hdmi_hpd, 0x6c, GENMASK(15, 14),
		RTK_PIN_FUNC(0x1 << 14, "gpio")),
	RTK_PIN_MUX(usb_id, 0x6c, GENMASK(17, 16),
		RTK_PIN_FUNC(0x1 << 16, "gpio"),
		RTK_PIN_FUNC(0x2 << 16, "cpu_loop")),
	RTK_PIN_MUX(hif_loc, 0x6c, GENMASK(19, 18),
		RTK_PIN_FUNC(0x1 << 18, "hif_misc"),
		RTK_PIN_FUNC(0x2 << 18, "hif_nf")),
	RTK_PIN_MUX(ejtag_scpu_loc, 0x6c, GENMASK(21, 20),
		RTK_PIN_FUNC(0x1 << 20, "scpu_ejtag_gpio"),
		RTK_PIN_FUNC(0x2 << 20, "scpu_ejtag_cr")),
	RTK_PIN_MUX(sensor_stb_1, 0x6c, GENMASK(23, 22),
		RTK_PIN_FUNC(0x0 << 22, "gpio")),
	RTK_PIN_MUX(sensor_stb_0, 0x6c, GENMASK(25, 24),
		RTK_PIN_FUNC(0x0 << 24, "gpio")),
	RTK_PIN_MUX(sensor_rst, 0x6c, GENMASK(28, 26),
		RTK_PIN_FUNC(0x0 << 26, "gpio")),
	RTK_PIN_MUX(sensor_cko_1, 0x6c, GENMASK(29, 28),
		RTK_PIN_FUNC(0x0 << 28, "gpio"),
		RTK_PIN_FUNC(0x1 << 28, "sensor"),
		RTK_PIN_FUNC(0x3 << 28, "usb")),
	RTK_PIN_MUX(sensor_cko_0, 0x6c, GENMASK(31, 30),
		RTK_PIN_FUNC(0x0 << 30, "gpio"),
		RTK_PIN_FUNC(0x1 << 30, "sensor")),

	RTK_PIN_MUX(gpio_0, 0x70, GENMASK(2, 0),
		RTK_PIN_FUNC(0x1 << 0, "gpio"),
		RTK_PIN_FUNC(0x2 << 0, "uart1"),
		RTK_PIN_FUNC(0x3 << 0, "hif_misc"),
		RTK_PIN_FUNC(0x4 << 0, "gspi")),
	RTK_PIN_MUX(gpio_1, 0x70, GENMASK(5, 3),
		RTK_PIN_FUNC(0x1 << 3, "gpio"),
		RTK_PIN_FUNC(0x2 << 3, "uart1"),
		RTK_PIN_FUNC(0x3 << 3, "hif_misc"),
		RTK_PIN_FUNC(0x4 << 3, "gspi")),
	RTK_PIN_MUX(gpio_2, 0x70, GENMASK(8, 6),
		RTK_PIN_FUNC(0x1 << 6, "gpio"),
		RTK_PIN_FUNC(0x2 << 6, "uart1"),
		RTK_PIN_FUNC(0x3 << 6, "hif_misc"),
		RTK_PIN_FUNC(0x4 << 6, "gspi")),
	RTK_PIN_MUX(gpio_3, 0x70, GENMASK(11, 9),
		RTK_PIN_FUNC(0x1 << 9, "gpio"),
		RTK_PIN_FUNC(0x2 << 9, "uart1"),
		RTK_PIN_FUNC(0x3 << 9, "hif_misc"),
		RTK_PIN_FUNC(0x4 << 9, "gspi")),
	RTK_PIN_MUX(gpio_4, 0x70, GENMASK(13, 12),
		RTK_PIN_FUNC(0x1 << 12, "gpio"),
		RTK_PIN_FUNC(0x2 << 12, "scpu_ejtag_gpio"),
		RTK_PIN_FUNC(0x3 << 12, "ao_gpio")),
	RTK_PIN_MUX(gpio_5, 0x70, GENMASK(15, 14),
		RTK_PIN_FUNC(0x1 << 14, "gpio"),
		RTK_PIN_FUNC(0x2 << 14, "scpu_ejtag_gpio"),
		RTK_PIN_FUNC(0x3 << 14, "ao_gpio")),
	RTK_PIN_MUX(gpio_6, 0x70, GENMASK(17, 16),
		RTK_PIN_FUNC(0x1 << 16, "gpio"),
		RTK_PIN_FUNC(0x2 << 16, "scpu_ejtag_gpio"),
		RTK_PIN_FUNC(0x3 << 16, "ao_gpio")),
	RTK_PIN_MUX(gpio_7, 0x70, GENMASK(19, 18),
		RTK_PIN_FUNC(0x1 << 18, "gpio"),
		RTK_PIN_FUNC(0x2 << 18, "scpu_ejtag_gpio"),
		RTK_PIN_FUNC(0x3 << 18, "ao_gpio")),
	RTK_PIN_MUX(gpio_8, 0x70, GENMASK(21, 20),
		RTK_PIN_FUNC(0x1 << 20, "gpio"),
		RTK_PIN_FUNC(0x2 << 20, "scpu_ejtag_gpio")),
	RTK_PIN_MUX(ao_loc, 0x74, GENMASK(3, 2),
		RTK_PIN_FUNC(0x1 << 2, "ao_tp0"),
		RTK_PIN_FUNC(0x2 << 2, "ao_gpio")),
};

static const struct rtd_pin_config_desc rtd1195_crt_configs[] = {
	RTK_PIN_CONFIG(nf_ale, 0x78, 4, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(nf_ce_n_0, 0x78, 8, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(nf_ce_n_1, 0x78, 12, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(nf_rdy, 0x78, 16, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(nf_rd_n, 0x78, 20, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(nf_wr_n, 0x78, 24, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(nf_cle, 0x78, 28, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(nf_dd_0, 0x7c, 0, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(nf_dd_1, 0x7c, 4, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(nf_dd_2, 0x7c, 8, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(nf_dd_3, 0x7c, 12, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(nf_dd_4, 0x7c, 16, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(nf_dd_5, 0x7c, 20, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(nf_dd_6, 0x7c, 24, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(nf_dd_7, 0x7c, 28, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(mmc_cmd, 0x80, 0, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(mmc_clk, 0x80, 4, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(mmc_wp, 0x80, 8, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(mmc_cd, 0x80, 12, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(mmc_data_0, 0x80, 16, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(mmc_data_1, 0x80, 20, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(mmc_data_2, 0x80, 24, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(mmc_data_3, 0x80, 28, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(sdio_cmd, 0x84, 0, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(sdio_clk, 0x84, 4, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(sdio_wp, 0x84, 8, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(sdio_cd, 0x84, 12, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(sdio_data_0, 0x84, 16, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(sdio_data_1, 0x84, 20, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(sdio_data_2, 0x84, 24, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(sdio_data_3, 0x84, 28, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(spdif, 0x88, 0, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(tp0_clk, 0x8c, 0, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(tp0_sync, 0x8c, 4, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(tp0_valid, 0x8c, 8, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(tp0_data, 0x8c, 12, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(tp1_clk, 0x90, 0, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(tp1_sync, 0x90, 4, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(tp1_valid, 0x90, 8, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(tp1_data, 0x90, 12, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(i2c_sda_1, 0x94, 0, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(i2c_scl_1, 0x94, 4, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(i2c_sda_4, 0x94, 8, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(i2c_scl_4, 0x94, 12, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(i2c_sda_5, 0x94, 16, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(i2c_scl_5, 0x94, 20, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(gpio_0, 0x98, 0, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(gpio_1, 0x98, 4, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(gpio_2, 0x98, 8, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(gpio_3, 0x98, 12, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(gpio_4, 0x98, 16, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(gpio_5, 0x98, 20, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(gpio_6, 0x98, 24, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(gpio_7, 0x98, 28, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(gpio_8, 0x9c, 0, 1, 0, 2, 3, PADDRI_4_8),
	RTK_PIN_CONFIG(hdmi_hpd, 0x9c, 4, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(usb_id, 0x9c, 8, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(sensor_stb_0, 0x9c, 12, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(sensor_stb_1, 0x9c, 16, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(sensor_rst, 0x9c, 20, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(sensor_cko_0, 0x9c, 24, 1, 0, 2, 3, PADDRI_2_4),
	RTK_PIN_CONFIG(sensor_cko_1, 0x9c, 28, 1, 0, 2, 3, PADDRI_2_4),
};

static struct rtd_pin_reg_list rtd1195_crt_reg_lists[] = {
	{.reg_offset = 0x60},
	{.reg_offset = 0x64},
	{.reg_offset = 0x68},
	{.reg_offset = 0x6c},
	{.reg_offset = 0x70},
	{.reg_offset = 0x74},
	{.reg_offset = 0x78},
	{.reg_offset = 0x7c},
	{.reg_offset = 0x80},
	{.reg_offset = 0x84},
	{.reg_offset = 0x88},
	{.reg_offset = 0x8c},
	{.reg_offset = 0x90},
	{.reg_offset = 0x94},
	{.reg_offset = 0x98},
	{.reg_offset = 0x9c},
};


static const struct rtd_pinctrl_desc rtd1195_crt_pinctrl_desc = {
	.pins = rtd1195_crt_pins,
	.num_pins = ARRAY_SIZE(rtd1195_crt_pins),
	.groups = rtd1195_crt_pin_groups,
	.num_groups = ARRAY_SIZE(rtd1195_crt_pin_groups),
	.functions = rtd1195_crt_pin_functions,
	.num_functions = ARRAY_SIZE(rtd1195_crt_pin_functions),
	.muxes = rtd1195_crt_muxes,
	.num_muxes = ARRAY_SIZE(rtd1195_crt_muxes),
	.configs = rtd1195_crt_configs,
	.num_configs = ARRAY_SIZE(rtd1195_crt_configs),
	.lists = rtd1195_crt_reg_lists,
	.num_regs = ARRAY_SIZE(rtd1195_crt_reg_lists),
};

#endif
