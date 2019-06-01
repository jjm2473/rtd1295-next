// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Semtech SX1280/SX1281 SPI
 *
 * Copyright (c) 2018-2019 Andreas Färber
 */

#include <linux/of.h>
#include <linux/spi/spi.h>

#include "sx128x.h"

static int sx128x_spi_send_command(struct sx128x_device *sxdev, u8 opcode, u8 argc, const u8 *argv, u8 *buf, size_t buf_len)
{
	struct spi_device *spi = to_spi_device(sxdev->dev);
	u8 status;
	struct spi_transfer xfers[] = {
		{
			.tx_buf = &opcode,
			.len = 1,
		},
		{
			.tx_buf = argv,
			.len = (argc > 0) ? argc - 1 : 0,
		},
		{
			.tx_buf = argv ? argv + (argc - 1) : NULL,
			.rx_buf = &status,
			.len = 1,
		},
		{
			.rx_buf = buf,
			.len = (opcode == SX128X_CMD_GET_STATUS) ? 0 : buf_len,
		},
	};
	int ret;

	ret = sx128x_busy_check_pre(sxdev);
	if (ret)
		return ret;

	ret = spi_sync_transfer(spi, xfers, ARRAY_SIZE(xfers));
	if (ret)
		return ret;

	if (buf && opcode == SX128X_CMD_GET_STATUS)
		*buf = status;

	sx128x_busy_wait_post(sxdev);

	return sx128x_status_to_errno(sxdev, status);
}

static int sx128x_spi_send_addr_command(struct sx128x_device *sxdev, u8 opcode, u16 addr, u8 argc, const u8 *argv, u8 *buf, size_t buf_len)
{
	struct spi_device *spi = to_spi_device(sxdev->dev);
	u8 addr_buf[2];
	u8 status;
	struct spi_transfer xfers[] = {
		{
			.tx_buf = &opcode,
			.len = 1,
		},
		{
			.tx_buf = addr_buf,
			.len = 2,
		},
		{
			.tx_buf = argv,
			.len = (argc > 0) ? argc - 1 : 0,
		},
		{
			.tx_buf = argv ? argv + (argc - 1) : NULL,
			.rx_buf = &status,
			.len = 1,
		},
		{
			.rx_buf = buf,
			.len = buf_len,
		},
	};
	int ret;

	addr_buf[0] = addr >> 8;
	addr_buf[1] = addr;

	ret = sx128x_busy_check_pre(sxdev);
	if (ret)
		return ret;

	ret = spi_sync_transfer(spi, xfers, ARRAY_SIZE(xfers));
	if (ret)
		return ret;

	sx128x_busy_wait_post(sxdev);

	return sx128x_status_to_errno(sxdev, status);
}

static const struct sx128x_ops sx128x_spi_cmd_ops = {
	.send_command = sx128x_spi_send_command,
	.send_addr_command = sx128x_spi_send_addr_command,
};

static int sx128x_spi_probe(struct spi_device *spi)
{
	struct sx128x_device *sxdev;

	sxdev = devm_kzalloc(&spi->dev, sizeof(*sxdev), GFP_KERNEL);
	if (!sxdev)
		return -ENOMEM;

	sxdev->dev = &spi->dev;
	sxdev->cmd_ops = &sx128x_spi_cmd_ops;

	spi_set_drvdata(spi, sxdev);

	spi->bits_per_word = 8;
	spi_setup(spi);

	return sx128x_probe(sxdev);
}

static int sx128x_spi_remove(struct spi_device *spi)
{
	struct sx128x_device *sxdev = spi_get_drvdata(spi);

	return sx128x_remove(sxdev);
}

static struct spi_driver sx128x_spi_driver = {
	.driver = {
		.name = "sx128x-spi",
		.of_match_table = of_match_ptr(sx128x_dt_ids),
	},
	.probe = sx128x_spi_probe,
	.remove = sx128x_spi_remove,
};

int __init sx128x_spi_init(void)
{
	return spi_register_driver(&sx128x_spi_driver);
}

void __exit sx128x_spi_exit(void)
{
	spi_unregister_driver(&sx128x_spi_driver);
}
