/*
 * LIS331DLH accelerometer driver
 *
 * Copyright (C) 2011 TI India
 * Author: Pankaj Bharadiya <pankaj.bharadiya@ti.com>
 *
 *  Based on the code by Carmine Iascone (carmine.iascone@st.com)
 *  and Matteo Dameno (matteo.dameno@st.com)
 *   
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __LIS331DLH_H__
#define __LIS331DLH_H__

#include <linux/types.h>

/************************************************/
/*	Accelerometer section defines		*/
/************************************************/

/* Accelerometer Sensor Full Scale */
#define LIS331DLH_G_2G			0x00
#define LIS331DLH_G_4G			0x10
#define LIS331DLH_G_8G			0x30

/* Accelerometer Sensor Operating Mode */
#define LIS331DLH_PM_OFF		0x00
#define LIS331DLH_PM_NORMAL		0x20
#define LIS331DLH_ENABLE_ALL_AXES	0x07

/* Accelerometer output data rate  */
#define LIS331DLH_ODRHALF		0x40	/* 0.5Hz output data rate */
#define LIS331DLH_ODR1			0x60	/* 1Hz output data rate */
#define LIS331DLH_ODR2			0x80	/* 2Hz output data rate */
#define LIS331DLH_ODR5			0xA0	/* 5Hz output data rate */
#define LIS331DLH_ODR10			0xC0	/* 10Hz output data rate */
#define LIS331DLH_ODR50			0x00	/* 50Hz output data rate */
#define LIS331DLH_ODR100		0x08	/* 100Hz output data rate */
#define LIS331DLH_ODR400		0x10	/* 400Hz output data rate */
#define LIS331DLH_ODR1000		0x18	/* 1000Hz output data rate */

#ifdef __KERNEL__
struct lis331dlh_platform_data {

	int poll_interval;
	int min_interval;

	u8 g_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */

#endif  /* __LIS331DLH_H__ */


