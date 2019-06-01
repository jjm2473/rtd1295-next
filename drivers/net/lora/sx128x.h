/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Semtech SX1280/SX1281
 *
 * Copyright (c) 2019 Andreas Färber
 */
#ifndef __LORA_SX128X_H__
#define __LORA_SX128X_H__

struct sx128x_device;

struct sx128x_ops {
	int (*send_command)(struct sx128x_device *sxdev, u8 opcode, u8 argc, const u8 *argv, u8 *buf, size_t buf_len);
	int (*send_addr_command)(struct sx128x_device *sxdev, u8 opcode, u16 addr, u8 argc, const u8 *argv, u8 *buf, size_t buf_len);
};

struct sx128x_device {
	struct device *dev;
	struct gpio_desc *rst;
	struct gpio_desc *busy_gpio;

	const struct sx128x_ops *cmd_ops;

	struct net_device *netdev;
};

extern const struct of_device_id sx128x_dt_ids[];

int sx128x_status_to_errno(struct sx128x_device *sxdev, u8 status);
int sx128x_busy_check_pre(struct sx128x_device *sxdev);
int sx128x_busy_wait_post(struct sx128x_device *sxdev);

int sx128x_probe(struct sx128x_device *sxdev);
int sx128x_remove(struct sx128x_device *sxdev);

int __init sx128x_spi_init(void);
void __exit sx128x_spi_exit(void);

#define SX128X_CMD_GET_SILICON_VERSION		0x14
#define SX128X_CMD_WRITE_REGISTER		0x18
#define SX128X_CMD_READ_REGISTER		0x19
#define SX128X_CMD_SET_STANDBY			0x80
#define SX128X_CMD_SET_PACKET_TYPE		0x8a
#define SX128X_CMD_SET_TX_PARAMS		0x8e
#define SX128X_CMD_SET_REGULATOR_MODE		0x96
#define SX128X_CMD_GET_STATUS			0xc0

#define SX128X_STATUS_COMMAND_MASK			GENMASK(4, 2)
#define SX128X_STATUS_COMMAND_TIMEOUT			(0x3 << 2)
#define SX128X_STATUS_COMMAND_PROCESSING_ERROR		(0x4 << 2)
#define SX128X_STATUS_COMMAND_FAILURE_TO_EXECUTE	(0x5 << 2)

#define SX128X_STATUS_MODE_MASK				GENMASK(7, 5)
#define SX128X_STATUS_MODE_STDBY_RC			(0x2 << 5)
#define SX128X_STATUS_MODE_STDBY_XOSC			(0x3 << 5)

#define SX128X_STANDBY_CONFIG_STDBY_RC		0
#define SX128X_STANDBY_CONFIG_STDBY_XOSC	1

#define SX128X_PACKET_TYPE_GFSK		0x00
#define SX128X_PACKET_TYPE_LORA		0x01

#define SX128X_RADIO_RAMP_20_US		0xe0

#define SX128X_REGULATOR_MODE_LDO	0
#define SX128X_REGULATOR_MODE_DCDC	1

#endif
