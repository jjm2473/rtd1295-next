/*
 * Copyright (C) 2015 Socionext Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef ___LINUX_VIDEO_SN65DSI84_H__
#define ___LINUX_VIDEO_SN65DSI84_H__

#include <linux/i2c.h>
#include <video/display_timing.h>

int sn65dsi84_i2c_set_timings(struct i2c_client *client, struct display_timing *dt);
int sn65dsi84_i2c_start(struct i2c_client *client, bool exit_ulps);
int sn65dsi84_i2c_stop(struct i2c_client *client);

#endif

