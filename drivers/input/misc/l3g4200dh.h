/*
 * L3G4200Dh gyroscope driver
 *
 * Copyright (C) 2011 TI India
 * Author: Pankaj Bharadiya <pankaj.bharadiya@ti.com>
 *
 *  Based on the code by kalle.viironen@digia.com
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

#ifndef __L3G4200DH_DRIVER__
#define __L3G4200DH_DRIVER__

#include <linux/platform_device.h>
#include <linux/i2c/l3g4200dh.h>

enum l3g4200dh_reg {
	WHO_AM_I	= 0x0F,
	CTRL_REG1	= 0x20,
	CTRL_REG2	= 0x21,
	CTRL_REG3	= 0x22,
	CTRL_REG4	= 0x23,
	CTRL_REG5	= 0x24,
	REFERENCE	= 0x25,
	OUT_TEMP	= 0x26,
	STATUS_REG	= 0x27,
	OUTX_L		= 0x28,
	OUTX_H		= 0x29,
	OUTX		= 0x29,
	OUTY_L		= 0x2A,
	OUTY_H		= 0x2B,
	OUTY		= 0x2B,
	OUTZ_L		= 0x2C,
	OUTZ_H		= 0x2D,
	OUTZ		= 0x2D,
	FIFO_CTRL_REG	= 0x2E,
	FIFO_SRC_REG	= 0x2F,
	INT1_CFG	= 0x30,
	INT1_SRC	= 0x31,
	INT1_TSH_XH	= 0x32,
	INT1_TSH_XL	= 0x33,
	INT1_TSH_YH	= 0x34,
	INT1_TSH_YL	= 0x35,
	INT1_TSH_ZH	= 0x36,
	INT1_TSH_ZL	= 0x37,
	INT1_DURATION	= 0x38,
};

enum l3g4200dh_ctrl1 {
	CTRL1_Xen	= 0x01,
	CTRL1_Yen	= 0x02,
	CTRL1_Zen	= 0x04,
	CTRL1_PD	= 0x08,
	CTRL1_BW0	= 0x10,
	CTRL1_BW1	= 0x20,
	CTRL1_DR0	= 0x40,
	CTRL1_DR1	= 0x80,
};

enum l3g4200dh_ctrl2 {
	CTRL2_HPCF0	= 0x01,
	CTRL2_HPCF1	= 0x02,
	CTRL2_HPCF2	= 0x04,
	CTRL2_HPCF3	= 0x08,
	CTRL2_HPM0	= 0x10,
	CTRL2_HPM1	= 0x20,
};

enum l3g4200dh_ctrl3 {
	CTRL3_I2_EMPTY	= 0x01,
	CTRL3_I2_ORUN	= 0x02,
	CTRL3_I2_WTM	= 0x04,
	CTRL3_I2_DRDY	= 0x08,
	CTRL3_PP_OD	= 0x10,
	CTRL3_H_LACTIVE	= 0x20,
	CTRL3_I1_BOOT	= 0x40,
	CTRL3_I1_INT1	= 0x80,
};

enum l3g4200dh_ctrl4 {
	CTRL4_SIM	= 0x01,
	CTRL4_ST0	= 0x02,
	CTRL4_ST1	= 0x04,
	CTRL4_FS0	= 0x10,
	CTRL4_FS1	= 0x20,
	CTRL4_BLE	= 0x40,
	CTRL4_BDU	= 0x80,
};

enum l3g4200dh_ctrl5 {
	CTRL5_OUT_SEL0	= 0x01,
	CTRL5_OUT_SEL1	= 0x02,
	CTRL5_INT1_SEL0	= 0x04,
	CTRL5_INT1_SEL1	= 0x08,
	CTRL5_HPen	= 0x10,
	CTRL5_FIFO_EN	= 0x40,
	CTRL5_BOOT	= 0x80,
};

enum l3g4200dh_status_reg {
	STATUS_XDA	= 0x01,
	STATUS_YDA	= 0x02,
	STATUS_ZDA	= 0x04,
	STATUS_ZYXDA	= 0x08,
	STATUS_XOR	= 0x10,
	STATUS_YOR	= 0x20,
	STATUS_ZOR	= 0x40,
	STATUS_ZYXOR	= 0x80,
};

enum l3g4200dh_fifo_ctrl {
	FIFO_CTRL_WTM0	= 0x01,
	FIFO_CTRL_WTM1	= 0x02,
	FIFO_CTRL_WTM2	= 0x04,
	FIFO_CTRL_WTM3	= 0x08,
	FIFO_CTRL_WTM4	= 0x10,
	FIFO_CTRL_FM0	= 0x20,
	FIFO_CTRL_FM1	= 0x40,
	FIFO_CTRL_FM2	= 0x80,
	FIFO_CTRL_WTM_MASK = 0x1f,
};

enum l3g4200dh_fifo_src {
	FIFO_SRC_FSS0	= 0x01,
	FIFO_SRC_FSS1	= 0x02,
	FIFO_SRC_FSS2	= 0x04,
	FIFO_SRC_FSS3	= 0x08,
	FIFO_SRC_FSS4	= 0x10,
	FIFO_SRC_EMPTY	= 0x20,
	FIFO_SRC_OVRN	= 0x40,
	FIFO_SRC_WTM	= 0x80,
};

enum l3g4200dh_int1_cfg {
	INT1_CFG_XLIE	= 0x01,
	INT1_CFG_XHIE	= 0x02,
	INT1_CFG_YLIE	= 0x04,
	INT1_CFG_YHIE	= 0x08,
	INT1_CFG_ZLIE	= 0x10,
	INT1_CFG_ZHIE	= 0x20,
	INT1_CFG_LIR	= 0x40,
	INT1_CFG_AND_OR	= 0x80,
};

enum l3g4200dh_int1_src {
	INT1_SRC_XL	= 0x01,
	INT1_SRC_XH	= 0x02,
	INT1_SRC_YL	= 0x04,
	INT1_SRC_YH	= 0x08,
	INT1_SRC_ZL	= 0x10,
	INT1_SRC_ZH	= 0x20,
	INT1_SRC_IA	= 0x40,
};

enum l3g4200dh_int1_duration {
	INT1_DUR_D0	= 0x01,
	INT1_DUR_D1	= 0x02,
	INT1_DUR_D2	= 0x04,
	INT1_DUR_D3	= 0x08,
	INT1_DUR_D4	= 0x10,
	INT1_DUR_D5	= 0x20,
	INT1_DUR_D6	= 0x40,
	INT1_DUR_WAIT	= 0x80,
};

struct axis_conversion {
	s8	x;
	s8	y;
	s8	z;
};

#define L3G4200DH_ID		0xD3

#define	L3G4200DH_REG_OFF	0x00
#define	L3G4200DH_REG_ON	0x01

struct l3g4200dh {
	int (*init) (struct l3g4200dh *gyro);
	int (*write) (struct l3g4200dh *gyro, int reg, u8 val);
	int (*read) (struct l3g4200dh *gyro, int reg, u8 *ret);
	int (*blkread) (struct l3g4200dh *gyro, int reg, int len, u8 *ret);
	int (*reg_ctrl) (struct l3g4200dh *gyro, bool state);

	int                     *odrs;     /* Supported output data rates */
	u8                      odr_mask;  /* ODR bit mask */
	u8			whoami;
	u8			sysfs_active;
	int			users;
	struct delayed_work	power_work; /* power off timer for sysfs */

	int			pwron_delay;
	int                     scale;
	u8			depth;	   /* fifo depth */

	struct i2c_client	*i2cdev;   /* i2c client device */
	struct input_dev	*idev;     /* input device */
	struct platform_device	*pdev;     /* platform device */
	struct axis_conversion	ac;        /* hw -> logical axis */

	u32			irq;       /* IRQ number */
	struct l3g4200dh_platform_data *pdata;/* for passing board config */
	struct mutex		mutex;     /* Serialize poll and selftest */
	atomic_t enabled;
};

int l3g4200dh_init_device(struct l3g4200dh *gyro);
void l3g4200dh_poweroff(struct l3g4200dh *gyro);
void l3g4200dh_poweron(struct l3g4200dh *gyro);
int l3g4200dh_remove_fs(struct l3g4200dh *gyro);

extern struct l3g4200dh l3g4200dh_dev;

#endif
