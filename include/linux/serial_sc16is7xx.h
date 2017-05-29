/*
 * Copyright (C) 2015 Socionext Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef ___LINUX_SC16IS7XX_H__
#define ___LINUX_SC16IS7XX_H__

int sc16is7xx_startup_not_tty(struct device *sc16_dev, int port, struct ktermios *termios,
		void (*cb_handler)(const void *data, size_t len, void *arg), void *cb_arg);
void sc16is7xx_shutdown_not_tty(struct device *sc16_dev, int port);
#ifdef CONFIG_SERIAL_SC16IS7XX_I2C
int sc16is7xx_i2c_send_async(struct i2c_client *client, int port, const void *buf, int count);
#endif

#endif

