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

#include <linux/i2c/lis331dlh.h>

#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/input-polldev.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>


#define LIS331DLH_DEV_NAME	"lis331dlh"

/** Maximum polled-device-reported g value */
#define G_MAX			8000

#define SHIFT_ADJ_2G		4
#define SHIFT_ADJ_4G		3
#define SHIFT_ADJ_8G		2

#define AXISDATA_REG		0x28

/* ctrl 1: pm2 pm1 pm0 dr1 dr0 zenable yenable zenable */
#define CTRL_REG1		0x20	/* power control reg */
#define CTRL_REG2		0x21	/* power control reg */
#define CTRL_REG3		0x22	/* power control reg */
#define CTRL_REG4		0x23	/* interrupt control reg */
#define INT2_CFG		0x34	/*int2 cfg */
#define INT1_CFG		0x30	/*int2 cfg */

#define FUZZ			0
#define FLAT			0
#define I2C_RETRY_DELAY		5
#define I2C_RETRIES		5
#define AUTO_INCREMENT		0x80

#define LIS331DLH_INT2_GPIO 	42

static struct {
	unsigned int cutoff;
	unsigned int mask;
} odr_table[] = {
	{
	3,	LIS331DLH_PM_NORMAL | LIS331DLH_ODR1000}, {
	10,	LIS331DLH_PM_NORMAL | LIS331DLH_ODR400}, {
	20,	LIS331DLH_PM_NORMAL | LIS331DLH_ODR100}, {
	100,	LIS331DLH_PM_NORMAL | LIS331DLH_ODR50}, {
	200,	LIS331DLH_ODR1000 | LIS331DLH_ODR10}, {
	500,	LIS331DLH_ODR1000 | LIS331DLH_ODR5}, {
	1000,	LIS331DLH_ODR1000 | LIS331DLH_ODR2}, {
	2000,	LIS331DLH_ODR1000 | LIS331DLH_ODR1}, {
	0,	LIS331DLH_ODR1000 | LIS331DLH_ODRHALF},};

struct lis331dlh_data {
	struct i2c_client *client;
	struct lis331dlh_platform_data *pdata;

	struct mutex lock;

	struct input_dev *input_dev;

	int hw_initialized;
	atomic_t enabled;
	int on_before_suspend;

	u8 reg_addr;

	u8 shift_adj;
	u8 resume_state[5];
};



static int lis331dlh_enable_irq(struct lis331dlh_data *acc);
void lis331dlh_config_irq_reg (struct lis331dlh_data *acc);


/*
 * Because misc devices can not carry a pointer from driver register to
 * open, we keep this global.  This limits the driver to a single instance.
 */
struct lis331dlh_data *lis331dlh_misc_data;
u32			lis331dlh_irq;       /* IRQ number */

static int lis331dlh_i2c_read(struct lis331dlh_data *acc,
				  u8 *buf, int len)
{
	int err;

	struct i2c_msg msgs[] = {
		{
		 .addr = acc->client->addr,
		 .flags = acc->client->flags & I2C_M_TEN,
		 .len = 1,
		 .buf = buf,
		 },
		{
		 .addr = acc->client->addr,
		 .flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
		 .len = len,
		 .buf = buf,
		 },
	};

	err = i2c_transfer(acc->client->adapter, msgs, 2);

	if (err != 2) {
		dev_err(&acc->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lis331dlh_i2c_write(struct lis331dlh_data *acc,
				   u8 *buf, int len)
{
	int err;

	struct i2c_msg msgs[] = {
		{
		 .addr = acc->client->addr,
		 .flags = acc->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	err = i2c_transfer(acc->client->adapter, msgs, 1);

	if (err != 1) {
		dev_err(&acc->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}


static int dump_reg (struct lis331dlh_data *acc)
{
	
	u8 acc_data;
	u8 i;
	int err;
	for (i= 0; i< 0x3f; i++)
	{
		acc_data = i;
		err = lis331dlh_i2c_read(acc, &acc_data, 1);
		if (err < 0)
			return err;
	}
		printk (KERN_ALERT " LIS331DLH: Addr : value ::  0x%0x : 0x%0x ", i, acc_data);
	return err;
}

static int get_lis331dlh_id (struct lis331dlh_data *acc)
{

#define WHO_AM_I 	0x0F
	int err;
	u8 acc_data[2];
        acc_data[0] = WHO_AM_I;
        err = lis331dlh_i2c_read(acc, acc_data, 1);
	if (err < 0)
		return err;
	printk(KERN_ALERT "%s: LIS331DLH: Chip ID is 0x%x : 0x%x \n", __FUNCTION__, acc_data[0], acc_data[1]);

	return err;


}


static int lis331dlh_hw_init(struct lis331dlh_data *acc)
{
	int err = -1;
	u8 buf[6];

	buf[0] = (AUTO_INCREMENT | CTRL_REG1);
	buf[1] = acc->resume_state[0];
	buf[2] = acc->resume_state[1];
	buf[3] = acc->resume_state[2];
	buf[4] = acc->resume_state[3];
	buf[5] = acc->resume_state[4];
	err = lis331dlh_i2c_write(acc, buf, 5);
	if (err < 0)
		return err;

	acc->hw_initialized = 1;

	return 0;
}

static void lis331dlh_device_power_off(struct lis331dlh_data *acc)
{
	int err;
	u8 buf[2] = { CTRL_REG1,
		      LIS331DLH_PM_OFF | LIS331DLH_ENABLE_ALL_AXES };

	err = lis331dlh_i2c_write(acc, buf, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "soft power off failed\n");

	if (acc->pdata->power_off)
		acc->pdata->power_off();

	acc->hw_initialized = 0;

}

static int lis331dlh_device_power_on(struct lis331dlh_data *acc)
{
	int err;

	if (acc->pdata->power_on) {
		err = acc->pdata->power_on();
		if (err < 0)
			return err;
	}

	if (!acc->hw_initialized) {
		err = lis331dlh_hw_init(acc);
		if (err < 0) {
			lis331dlh_device_power_off(acc);
			return err;
		}
	}

	lis331dlh_config_irq_reg (acc);
	return 0;
}

int lis331dlh_update_g_range(struct lis331dlh_data *acc, u8 new_g_range)
{
	int err;
	u8 shift;
	u8 buf[2];
	switch (new_g_range) {
	case LIS331DLH_G_2G:
		shift = SHIFT_ADJ_2G;
		break;
	case LIS331DLH_G_4G:
		shift = SHIFT_ADJ_4G;
		break;
	case LIS331DLH_G_8G:
		shift = SHIFT_ADJ_8G;
		break;
	default:
		return -EINVAL;
	}

	if (atomic_read(&acc->enabled)) {
		/* Set configuration register 4, which contains g range setting
		 *  NOTE: this is a straight overwrite because this driver does
		 *  not use any of the other configuration bits in this
		 *  register.  Should this become untrue, we will have to read
		 *  out the value and only change the relevant bits --XX----
		 *  (marked by X) */
		buf[0] = CTRL_REG4;
		buf[1] = new_g_range;
		err = lis331dlh_i2c_write(acc, buf, 1);
		if (err < 0)
			return err;
	}

	acc->resume_state[3] = new_g_range;
	acc->shift_adj = shift;

	return 0;
}

int lis331dlh_update_odr(struct lis331dlh_data *acc, int poll_interval)
{
	int err = -1;
	int i;
	u8 config[2];

	/* Convert the poll interval into an output data rate configuration
	 *  that is as low as possible.  The ordering of these checks must be
	 *  maintained due to the cascading cut off values - poll intervals are
	 *  checked from shortest to longest.  At each check, if the next lower
	 *  ODR cannot support the current poll interval, we stop searching */
	for (i = 0; i < ARRAY_SIZE(odr_table); i++) {
		config[1] = odr_table[i].mask;
		if (poll_interval < odr_table[i].cutoff)
			break;
	}

	config[1] |= LIS331DLH_ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&acc->enabled)) {
		config[0] = CTRL_REG1;
		err = lis331dlh_i2c_write(acc, config, 1);
		if (err < 0)
			return err;
	}

	acc->resume_state[0] = config[1];

	return 0;
}

static int decode(u8 *p, int adj)
{
	s16 v = p[0] | (p[1] << 8);
	return (int) v >> adj;
}

static int lis331dlh_get_data(struct lis331dlh_data *acc,
					       int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	int hw_d[3] = { 0 };

	acc_data[0] = (AUTO_INCREMENT | AXISDATA_REG);
	err = lis331dlh_i2c_read(acc, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = decode(acc_data, acc->shift_adj);
	hw_d[1] = decode(acc_data + 2, acc->shift_adj);
	hw_d[2] = decode(acc_data + 4, acc->shift_adj);

	xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		  : (hw_d[acc->pdata->axis_map_x]));
	xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		  : (hw_d[acc->pdata->axis_map_y]));
	xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		  : (hw_d[acc->pdata->axis_map_z]));

	return err;
}

static void lis331dlh_report_values(struct lis331dlh_data *acc,
					int *xyz)
{
	struct input_dev *input = acc->input_dev;

	input_report_abs(input, ABS_X, xyz[0]);
	input_report_abs(input, ABS_Y, xyz[1]);
	input_report_abs(input, ABS_Z, xyz[2]);
	input_sync(input);
}

static int lis331dlh_enable(struct lis331dlh_data *acc)
{
	int err;

	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		
		err = lis331dlh_device_power_on(acc);
		if (err < 0) {
			atomic_set(&acc->enabled, 0);
			return err;
		}
	}

	return 0;
}

static int lis331dlh_disable(struct lis331dlh_data *acc)
{
	if (atomic_cmpxchg(&acc->enabled, 1, 0))
	{
		lis331dlh_device_power_off(acc);
	}
	return 0;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct lis331dlh_data *acc = dev_get_drvdata(dev);
	int val;
	mutex_lock(&acc->lock);
	val = acc->pdata->poll_interval;
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct lis331dlh_data *acc = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->pdata->poll_interval = interval_ms;
	lis331dlh_update_odr(acc, interval_ms);
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_range(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lis331dlh_data *acc = dev_get_drvdata(dev);
	char range;
	char val;
	mutex_lock(&acc->lock);
	val = acc->pdata->g_range ;
	switch (val) {
	case LIS331DLH_G_2G:
		range = 2;
		break;
	case LIS331DLH_G_4G:
		range = 4;
		break;
	case LIS331DLH_G_8G:
		range = 8;
		break;
	default:
		range = 2;
		break;
	}
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct lis331dlh_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->pdata->g_range = val;
	lis331dlh_update_g_range(acc, val);
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lis331dlh_data *acc = dev_get_drvdata(dev);
	int val = atomic_read(&acc->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lis331dlh_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val)
		lis331dlh_enable(acc);
	else
		lis331dlh_disable(acc);

	return size;
}

static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct lis331dlh_data *acc = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	x[0] = acc->reg_addr;
	mutex_unlock(&acc->lock);
	x[1] = val;
	rc = lis331dlh_i2c_write(acc, x, 1);
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct lis331dlh_data *acc = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&acc->lock);
	data = acc->reg_addr;
	mutex_unlock(&acc->lock);
	rc = lis331dlh_i2c_read(acc, &data, 1);
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lis331dlh_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->reg_addr = val;
	mutex_unlock(&acc->lock);
	return size;
}

static struct device_attribute attributes[] = {
	__ATTR(pollrate_ms, 0666, attr_get_polling_rate, attr_set_polling_rate),
	__ATTR(range, 0666, attr_get_range, attr_set_range),
	__ATTR(enable, 0666, attr_get_enable, attr_set_enable),
	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}


static irqreturn_t lis331dlh_interrupt_thread2 (int irq, void *data)
{
	struct lis331dlh_data *acc = (struct lis331dlh_data *) data;

	int xyz[3] = { 0 };
	int err;


	mutex_lock(&acc->lock);
	err = lis331dlh_get_data(acc, xyz);
	if (err < 0)
		dev_err(&acc->client->dev, "get_acceleration_data failed\n");
	else
		lis331dlh_report_values(acc, xyz);

	mutex_unlock(&acc->lock);
	return IRQ_HANDLED;
}

int lis331dlh_input_open(struct input_dev *input)
{
//	struct lis331dlh_data *acc = input_get_drvdata(input);
//	return lis331dlh_enable(acc);
	return 0; 
}

void lis331dlh_input_close(struct input_dev *dev)
{
/*	struct lis331dlh_data *acc = input_get_drvdata(dev);
	lis331dlh_disable(acc);
*/
	return ; 
}

static int lis331dlh_validate_pdata(struct lis331dlh_data *acc)
{
	acc->pdata->poll_interval = max(acc->pdata->poll_interval,
					acc->pdata->min_interval);

	if (acc->pdata->axis_map_x > 2 ||
	    acc->pdata->axis_map_y > 2 || acc->pdata->axis_map_z > 2) {
		dev_err(&acc->client->dev,
			"invalid axis_map value x:%u y:%u z%u\n",
			acc->pdata->axis_map_x, acc->pdata->axis_map_y,
			acc->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (acc->pdata->negate_x > 1 || acc->pdata->negate_y > 1 ||
	    acc->pdata->negate_z > 1) {
		dev_err(&acc->client->dev,
			"invalid negate value x:%u y:%u z:%u\n",
			acc->pdata->negate_x, acc->pdata->negate_y,
			acc->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (acc->pdata->poll_interval < acc->pdata->min_interval) {
		dev_err(&acc->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lis331dlh_input_init(struct lis331dlh_data *acc)
{
	int err;
	struct input_dev *input;

	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		dev_err(&acc->client->dev, "input device allocate failed\n");
		goto err0;
	}


	input = acc->input_dev;

	input->open = lis331dlh_input_open;
	input->close = lis331dlh_input_close;
	input->name = LIS331DLH_DEV_NAME;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &acc->client->dev;

	input_set_drvdata(acc->input_dev, acc);

	set_bit(EV_ABS, input->evbit);

	input_set_abs_params(input, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

	input->name = "accelerometer";

	err = input_register_device(acc->input_dev);
	if (err) {
		dev_err(&acc->client->dev,
			"unable to register input device %s\n",
			acc->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(acc->input_dev);
err0:
	return err;
}

static void lis331dlh_input_cleanup(struct lis331dlh_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

static int lis331dlh_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct lis331dlh_data *acc;
	int err = -1;

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto err0;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto err0;
	}

	acc = kzalloc(sizeof(*acc), GFP_KERNEL);
	if (acc == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto err0;
	}

	mutex_init(&acc->lock);
	mutex_lock(&acc->lock);
	acc->client = client;

	acc->pdata = kmalloc(sizeof(*acc->pdata), GFP_KERNEL);
	if (acc->pdata == NULL)
		goto err1;

	memcpy(acc->pdata, client->dev.platform_data, sizeof(*acc->pdata));

	err = lis331dlh_validate_pdata(acc);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto err1_1;
	}

	i2c_set_clientdata(client, acc);

	if (acc->pdata->init) {
		err = acc->pdata->init();
		if (err < 0)
			goto err1_1;
	}

	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));

	acc->resume_state[0] = 0x27;
	acc->resume_state[1] = 0x00;
	acc->resume_state[2] = 0x00;
	acc->resume_state[3] = 0x00;
	acc->resume_state[4] = 0x00;

	err = lis331dlh_device_power_on(acc);
	if (err < 0)
		goto err2;

	atomic_set(&acc->enabled, 1);

	acc->pdata->g_range =  LIS331DLH_G_8G;
	acc->pdata->poll_interval =  150;
	acc->pdata->negate_x =  0;
	acc->pdata->negate_y =  0;
	acc->pdata-> negate_z =  0;
	err = lis331dlh_update_g_range(acc, acc->pdata->g_range);
	if (err < 0) {
		dev_err(&client->dev, "update_g_range failed\n");
		goto err2;
	}

	err = lis331dlh_update_odr(acc, acc->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto err2;
	}

	err = lis331dlh_input_init(acc);
	if (err < 0)
		goto err3;

	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev, "lsm_acc_device register failed\n");
		goto err4;
	}

//	lis331dlh_device_power_off(acc);

	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);

	mutex_unlock(&acc->lock);

	get_lis331dlh_id (acc);
	lis331dlh_enable_irq(acc);
	dump_reg (acc);
	dev_info(&client->dev, "lis331dlh probed\n");

	return 0;

err4:
	lis331dlh_input_cleanup(acc);
err3:
	lis331dlh_device_power_off(acc);
err2:
	if (acc->pdata->exit)
		acc->pdata->exit();
err1_1:
	mutex_unlock(&acc->lock);
	kfree(acc->pdata);
err1:
	kfree(acc);
err0:
	return err;
}

static int __devexit lis331dlh_remove(struct i2c_client *client)
{
	/* TODO: revisit ordering here once _probe order is finalized */
	struct lis331dlh_data *acc = i2c_get_clientdata(client);
	lis331dlh_input_cleanup(acc);
	lis331dlh_device_power_off(acc);
	remove_sysfs_interfaces(&client->dev);
	if (acc->pdata->exit)
		acc->pdata->exit();
	kfree(acc->pdata);
	kfree(acc);
	gpio_free(LIS331DLH_INT2_GPIO);
	free_irq(lis331dlh_irq, acc);

	return 0;
}

static int lis331dlh_resume(struct i2c_client *client)
{
	struct lis331dlh_data *acc = i2c_get_clientdata(client);

	if (acc->on_before_suspend)
		return lis331dlh_enable(acc);
	return 0;
}

static int lis331dlh_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lis331dlh_data *acc = i2c_get_clientdata(client);

	acc->on_before_suspend = atomic_read(&acc->enabled);
	return lis331dlh_disable(acc);
}

static const struct i2c_device_id lis331dlh_id[] = {
	{LIS331DLH_DEV_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, lis331dlh_id);

static struct i2c_driver lis331dlh_driver = {
	.driver = {
		.name = LIS331DLH_DEV_NAME,
		.owner = THIS_MODULE,
	},
	.probe = lis331dlh_probe,
	.remove = __devexit_p(lis331dlh_remove),
//	.resume = lis331dlh_resume,
//	.suspend = lis331dlh_suspend,
	.id_table = lis331dlh_id,
};

void lis331dlh_config_irq_reg (struct lis331dlh_data *acc)
{
	u8 buf[2];

	buf[0]= CTRL_REG3;
	buf[1] = 0xB6;
	lis331dlh_i2c_write (acc, buf, 1);

	buf [0] = INT1_CFG;
	buf[1] = 0x3f;
	lis331dlh_i2c_write (acc, buf, 1);



}

static int lis331dlh_enable_irq(struct lis331dlh_data *acc)
{
        int irq_flags;
        int err;
        int ret;
	u8 buf[2];
	
        ret = gpio_request(LIS331DLH_INT2_GPIO , "is331dlh irq");
        if (ret < 0)
                goto fail;

        ret = gpio_direction_input(LIS331DLH_INT2_GPIO);
        if (ret < 0)
                goto fail_irq;

        lis331dlh_irq = gpio_to_irq(LIS331DLH_INT2_GPIO);
        if ( lis331dlh_irq < 0)
                goto fail_irq;

        //irq_flags = IRQF_TRIGGER_RISING;
        irq_flags = IRQF_TRIGGER_FALLING;
        err = request_threaded_irq(lis331dlh_irq,
                        NULL,
                        lis331dlh_interrupt_thread2,
                        irq_flags,
                        "lis331dlh", acc);

	lis331dlh_config_irq_reg (acc);

        return err;
fail_irq:
        gpio_free(LIS331DLH_INT2_GPIO);
fail:
        printk(KERN_ERR "lis331dlh initialisation failed\n");
        return -1;
}




static int __init lis331dlh_init(void)
{
	pr_debug("LIS331DLH driver for the accelerometer part\n");
	printk (KERN_ALERT "LIS331DLH driver for the accelerometer part\n");
	return i2c_add_driver(&lis331dlh_driver);
}

static void __exit lis331dlh_exit(void)
{
	i2c_del_driver(&lis331dlh_driver);
	return;
}

module_init(lis331dlh_init);
module_exit(lis331dlh_exit);

MODULE_DESCRIPTION("lis331dlh accelerometer driver");
MODULE_AUTHOR("Texas Instruments");
MODULE_LICENSE("GPL");

