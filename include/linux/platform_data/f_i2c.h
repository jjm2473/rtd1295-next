/*
 * linux/include/linux/plaform_data/f_i2c.h
 *
 * Copyright (C) 2012 FUJITSU SEMICONDUCTOR LIMITED
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __INCLUDE_LINUX_PLATFORM_DATA_F_I2C_H__
#define __INCLUDE_LINUX_PLATFORM_DATA_F_I2C_H__

/*
 * i2c platform devices data
 */
struct f_i2c_platform_data {
	unsigned int speed_khz;
};

#define F_I2C_SPEED_FM	400	/* Fast Mode */
#define F_I2C_SPEED_SM	100	/* Standard Mode */

#endif /* __INCLUDE_LINUX_PLATFORM_DATA_F_I2C_H__ */

