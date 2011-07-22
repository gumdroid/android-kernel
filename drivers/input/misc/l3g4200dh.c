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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/dmi.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/freezer.h>
#include <linux/uaccess.h>
#include <asm/atomic.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include "l3g4200dh.h"

#define DRIVER_NAME "l3g4200dh"

#define SYSFS_POWERDOWN_DELAY	(5 * HZ)

/* something like this more correct?
   gyro->pwron_delay / l3g4200dh_get_odr()); */
#define L3G4200DH_PWRON_DELAY		(5)
/* should be scaled by rate, but set to worst case here
 * for now.  fifo size / odr (+ some buffer?)
 * buffer seems to be larger than i'd expect */
#define L3G4200DH_EMPTY_FIFO_DELAY_MS	(400)

#define SELFTEST_OK		0
#define SELFTEST_FAIL		-1
#define SELFTEST_IRQ		-2
#define SELFTEST_LIMIT_ERR	-3

#define FS_MASK			0x30

#define FIFO_LENGTH		32
#define FIFO_NUM_AXIS		3
#define FIFO_DATA_SET_LENGTH	(FIFO_NUM_AXIS * sizeof(s16))
#define FIFO_BUFF_SET_LENGTH	(FIFO_NUM_AXIS * sizeof(s32))
#define WTM_LEVEL		25

struct l3g4200dh l3g4200dh_dev;
EXPORT_SYMBOL_GPL(l3g4200dh_dev);

/* global just because the are alloc'd */
/* fifo data used only locally when getting the fifo, fifo_buffer
** is passed around.  Should be able to remove fifo_data. */
s32 *fifo_buffer;
s16 *fifo_data;

/* conversion btw full scale and the register values */
static int l3g4200dh_scales[4] = { 250, 500, 2000, 2000 };

/* conversion btw sampling rate and the register values */
static int l3g4200dh_rates[4] = { 100, 200, 400, 800 };

static int l3g4200dh_get_scale(void);
void read_reg ();

static s8 l3g4200dh_read(int reg)
{
	u8 byte;
	if (l3g4200dh_dev.read(&l3g4200dh_dev, reg, &byte) < 0)
		return 0;

	return byte;
}

static s16 l3g4200dh_read_data(int reg)
{
	u8 lo, hi;
	s16 ret;

	if (l3g4200dh_dev.read(&l3g4200dh_dev, reg - 1, &lo) < 0)
		return 0;

	if (l3g4200dh_dev.read(&l3g4200dh_dev, reg, &hi) < 0)
		return 0;

	ret = ((hi << 8) | lo);
	return ret;
}

static inline void l3g4200dh_scale_reading(s32 *data, int count)
{
	int i;
	s32 tmpdata;

	for (i = 0; i < count; i++) {
		tmpdata = data[i];
		switch (l3g4200dh_dev.scale) {
		case 250:
			tmpdata *= 875;
			tmpdata /= 100;
			break;
		case 500:
			tmpdata *= 175;
			tmpdata /= 10;
			break;
		case 2000:
			tmpdata *= 70;
			break;
		}
		data[i] = tmpdata;
	}
}

/**
 * l3g4200dh_get_xyz - Get X, Y and Z axis values from the gyroscope
 * @x:    where to store the X axis value
 * @y:    where to store the Y axis value
 * @z:    where to store the Z axis value
 */
static void l3g4200dh_get_xyz(int *x, int *y, int *z)
{
	s16 data[3];
	s32 buffer[3];
	int i;
	u8 status;

	l3g4200dh_dev.read(&l3g4200dh_dev, STATUS_REG, &status);
	if (!(status & STATUS_ZYXDA)) {
		/* should be relative to rate */
		msleep(10);
	}

	if (l3g4200dh_dev.blkread) {
		l3g4200dh_dev.blkread(&l3g4200dh_dev, OUTX_L, 6, (u8 *)data);
	} else {
		data[0] = l3g4200dh_read_data(OUTX);
		data[1] = l3g4200dh_read_data(OUTY);
		data[2] = l3g4200dh_read_data(OUTZ);
	}

	for (i = 0; i < 3; i++)
		buffer[i] = (s16)le16_to_cpu(data[i]);

	l3g4200dh_scale_reading(buffer, 3);

	*x = buffer[0];
	*y = buffer[1];
	*z = buffer[2];
}

static int l3g4200dh_get_fifo(void)
{
	int i = 0;
	u8 depth;

	memset(fifo_buffer, 0, FIFO_LENGTH*FIFO_BUFF_SET_LENGTH);
	memset(fifo_data, 0, FIFO_LENGTH*FIFO_DATA_SET_LENGTH);

	l3g4200dh_dev.read(&l3g4200dh_dev, FIFO_SRC_REG, &depth);
	depth &= (FIFO_SRC_FSS0|FIFO_SRC_FSS1|FIFO_SRC_FSS2|
		  FIFO_SRC_FSS3|FIFO_SRC_FSS4);

	if (l3g4200dh_dev.blkread) {
		/* currently i2c at least will only block read up to 32 bytes,
		** so we HAVE to adjust the start register based on the
		** data received */
		while (i < (depth*FIFO_DATA_SET_LENGTH)) {
			i += l3g4200dh_dev.blkread(&l3g4200dh_dev,
					OUTX_L+(i%FIFO_DATA_SET_LENGTH),
					depth*FIFO_DATA_SET_LENGTH-i,
					(u8 *)(fifo_data)+i);
		}
	} else {
		for (i = 0; i < (depth*FIFO_NUM_AXIS); i++) {
			fifo_data[i++] = l3g4200dh_read_data(OUTX);
			fifo_data[i++] = l3g4200dh_read_data(OUTY);
			fifo_data[i] = l3g4200dh_read_data(OUTZ);
		}
	}

	for (i = 0; i < depth*FIFO_NUM_AXIS; i++)
		fifo_buffer[i] = (s16)le16_to_cpu(fifo_data[i]);

	l3g4200dh_scale_reading(fifo_buffer, depth*FIFO_NUM_AXIS);

	return depth;
}

/* ODR is Output Data Rate */
static int l3g4200dh_get_odr(void)
{
	u8 ctrl;
	int shift;

	l3g4200dh_dev.read(&l3g4200dh_dev, CTRL_REG1, &ctrl);
	ctrl &= l3g4200dh_dev.odr_mask;
	shift = ffs(l3g4200dh_dev.odr_mask) - 1;
	return l3g4200dh_dev.odrs[(ctrl >> shift)];
}

static int l3g4200dh_set_odr(int rate)
{
	u8 ctrl;
	int i, shift;

	l3g4200dh_dev.read(&l3g4200dh_dev, CTRL_REG1, &ctrl);
	ctrl &= ~l3g4200dh_dev.odr_mask;
	shift = ffs(l3g4200dh_dev.odr_mask) - 1;

	for (i = 0; i < ARRAY_SIZE(l3g4200dh_rates); i++)
		if (l3g4200dh_dev.odrs[i] == rate) {
			l3g4200dh_dev.write(&l3g4200dh_dev, CTRL_REG1,
					ctrl | (i << shift));
			return 0;
		}
	return -EINVAL;
}

/* conversion btw full scale and the register values */
static int l3g4200dh_get_scale(void)
{
	u8 reg;
	int shift;

	l3g4200dh_dev.read(&l3g4200dh_dev, CTRL_REG4, &reg);

	reg &= FS_MASK;
	shift = ffs(FS_MASK) - 1;

	return l3g4200dh_scales[(reg >> shift)];
}

void l3g4200dh_report_values (struct l3g4200dh *l3g4200dh_data)
{
	int x, y, z;
	l3g4200dh_get_xyz(&x, &y, &z);
	input_report_rel(l3g4200dh_dev.idev, REL_RX, x);
	input_report_rel(l3g4200dh_dev.idev, REL_RY, y);
	input_report_rel(l3g4200dh_dev.idev, REL_RZ, z);
	input_sync(l3g4200dh_dev.idev);
}

/* interrupt interface  */
static irqreturn_t l3g4200dh_interrupt_thread2(int irq, void *data)
{
	u8 depth;
	struct l3g4200dh *l3g4200dh_data  = (struct l3g4200dh *)data;

	mutex_lock(&l3g4200dh_dev.mutex);

	depth = l3g4200dh_get_fifo();
	if (depth) {
		l3g4200dh_dev.depth = depth;
		l3g4200dh_report_values (l3g4200dh_data);
	}

	mutex_unlock(&l3g4200dh_dev.mutex);

	return IRQ_HANDLED;
}

static int l3g4200dh_enable_irq(void)
{
	l3g4200dh_dev.write(&l3g4200dh_dev, CTRL_REG3, CTRL3_I2_WTM);
	return 0;
}

static void l3g4200dh_stop_irq(void)
{
	l3g4200dh_dev.write(&l3g4200dh_dev, CTRL_REG3, 0);
	l3g4200dh_dev.depth = 0;
	return;
}

void l3g4200dh_poweroff(struct l3g4200dh *gyro)
{
	u8 reg;

	/* disable X,Y,Z axis and power down */
	gyro->read(gyro, CTRL_REG1, &reg);
	reg &= ~(CTRL1_PD | CTRL1_Xen | CTRL1_Yen | CTRL1_Zen);
	gyro->write(gyro, CTRL_REG1, reg);
	if (gyro->reg_ctrl)
		gyro->reg_ctrl(gyro, L3G4200DH_REG_OFF);
	l3g4200dh_stop_irq();
}
EXPORT_SYMBOL_GPL(l3g4200dh_poweroff);

void l3g4200dh_poweron(struct l3g4200dh *gyro)
{
	u8 reg;

	gyro->init(gyro);

	gyro->read(gyro, CTRL_REG5, &reg);
	gyro->write(gyro, CTRL_REG5, reg | CTRL5_BOOT);
	gyro->write(gyro, CTRL_REG5, reg);
	msleep(L3G4200DH_PWRON_DELAY);

	gyro->read(gyro, CTRL_REG1, &reg);
	reg |= CTRL1_PD | CTRL1_Xen | CTRL1_Yen | CTRL1_Zen;
	gyro->write(gyro, CTRL_REG1, reg);
	msleep(L3G4200DH_PWRON_DELAY);
	l3g4200dh_enable_irq ();
}
EXPORT_SYMBOL_GPL(l3g4200dh_poweron);

static ssize_t l3g4200dh_angular_rate_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int x, y, z;

	mutex_lock(&l3g4200dh_dev.mutex);
	l3g4200dh_get_xyz(&x, &y, &z);
	mutex_unlock(&l3g4200dh_dev.mutex);
	return sprintf(buf, "%d %d %d\n", x, y, z);
}

void read_reg ()
{
	int i;
	s8 read_val;
	for (i=0; i< 0x3F;i++)
	{
		read_val = l3g4200dh_read (i);
	}
}

int l3g4200dh_inputdev_init(struct l3g4200dh *gyro)
{

        struct input_dev *input_dev;
	int err = 0;

	/* allocate gyro input_device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		err = -ENOMEM;
		goto err_input_allocate_device;
	}

	l3g4200dh_dev.idev = input_dev;
	input_dev->name = "gyro";
	/* X */
	input_set_capability(input_dev, EV_REL, REL_RX);
	input_set_abs_params(input_dev, REL_RX, -2048, 2047, 0, 0);
	/* Y */
	input_set_capability(input_dev, EV_REL, REL_RY);
	input_set_abs_params(input_dev, REL_RY, -2048, 2047, 0, 0);
	/* Z */
	input_set_capability(input_dev, EV_REL, REL_RZ);
	input_set_abs_params(input_dev, REL_RZ, -2048, 2047, 0, 0);

	err = input_register_device(input_dev);
	if (err < 0) {
		pr_err("%s: could not register input device\n", __func__);
		input_free_device(input_dev);
	}

err_input_allocate_device:
	return err;
}

int l3g4200dh_init_device(struct l3g4200dh *dev)
{
	u8 test = 0;
	int irq_flags;
	int err = 0;

	dev_dbg(&dev->idev->dev, "l3g4200dh_init_device\n");

	dev->pwron_delay = L3G4200DH_PWRON_DELAY;
	dev->odrs = l3g4200dh_rates;
	dev->odr_mask = CTRL1_DR0 | CTRL1_DR1;

	if (!dev->irq) {
		dev_err(&dev->idev->dev,
				"No IRQ. Input device will not work.\n");
		goto out;
	}

	dev->scale = l3g4200dh_get_scale();

	mutex_init(&dev->mutex);

	fifo_buffer = kcalloc(sizeof(s32), FIFO_LENGTH*FIFO_NUM_AXIS,
				GFP_KERNEL);
	if ( fifo_buffer == NULL )
	{
		dev_err(&dev->idev->dev,"L3G4220DH: Failed to allocate buffer\n");
		goto out;
	}

	fifo_data = kcalloc(sizeof(s16), FIFO_LENGTH*FIFO_NUM_AXIS,
				GFP_KERNEL);
	if ( fifo_data == NULL )
	{
		dev_err(&dev->idev->dev,"L3G4220DH: Failed to allocate buffer\n");
		goto out;
	}

	err = l3g4200dh_inputdev_init(dev);
	if ( err != 0 )
	{
		goto out;
	}

	dev->read(dev, WHO_AM_I, &dev->whoami);
	printk (KERN_ALERT "%s: L3G4220DH : WHO AM I : %x \n", __FUNCTION__, dev->whoami);
	/* Set BDU bit => output registers not updated until MSB and LSB
	** read */
	dev->write(dev, CTRL_REG4, CTRL4_BDU);
	dev->read(dev, CTRL_REG4, &test);

	dev->write(dev, CTRL_REG5, CTRL5_FIFO_EN);
	dev->read(dev, CTRL_REG5, &test);

	/* set to streaming mode, and the default watermark */
	dev->write(dev, FIFO_CTRL_REG, FIFO_CTRL_FM1 |
				       (WTM_LEVEL & FIFO_CTRL_WTM_MASK));

	dev->read(dev, FIFO_CTRL_REG, &test);
	dev->read(dev, CTRL_REG5, &test);

	irq_flags = IRQF_TRIGGER_RISING;
	err = request_threaded_irq(l3g4200dh_dev.irq,
			NULL,
			l3g4200dh_interrupt_thread2,
			irq_flags,
			DRIVER_NAME, &l3g4200dh_dev);

	l3g4200dh_poweron(dev);

	return err;

out:
	return -1;
}
EXPORT_SYMBOL_GPL(l3g4200dh_init_device);

MODULE_DESCRIPTION("L3G4200DH three-axis digital gyroscope driver");
MODULE_AUTHOR("");
MODULE_LICENSE("GPL");
