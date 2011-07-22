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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#include "l3g4200dh.h"

#define DRV_NAME	"l3g4200dh_i2c"


static int l3g4200dh_enable(struct l3g4200dh *gyro)
{
        if (!atomic_cmpxchg(&gyro->enabled, 0, 1)) {
		l3g4200dh_poweron(gyro);
        }
	printk (KERN_ALERT "L3G4200DH: %s :implemented\n",__FUNCTION__ );
        return 0;
}

static int l3g4200dh_disable(struct l3g4200dh *gyro)
{
        if (atomic_cmpxchg(&gyro->enabled, 1, 0))
        {
		l3g4200dh_poweroff(gyro);
        }
	printk (KERN_ALERT "L3G4200DH: %s :implemented\n",__FUNCTION__ );
        return 0;
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	printk (KERN_ALERT "L3G4200DH: %s :not implemented\n",__FUNCTION__ );
	return 0;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	printk (KERN_ALERT "L3G4200DH: %s :not implemented\n",__FUNCTION__ );
	return 0;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct l3g4200dh *gyro = dev_get_drvdata(dev);
        int val = atomic_read(&gyro->enabled);
        return sprintf(buf, "%d\n", val);

//	printk (KERN_ALERT "L3G4200DH: %s :not implemented\n",__FUNCTION__ );
	return 0;
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct l3g4200dh *gyro = dev_get_drvdata(dev);
        unsigned long val;

        if (strict_strtoul(buf, 10, &val))
                return -EINVAL;
        if (val)
                l3g4200dh_enable(gyro);
        else
                l3g4200dh_disable(gyro);

        return size;

//	printk (KERN_ALERT "L3G4200DH: %s :not implemented\n",__FUNCTION__ );
	return 0;
}

static struct device_attribute attributes[] = {

	__ATTR(enable, 0666, attr_get_enable, attr_set_enable),
	__ATTR(pollrate_ms, 0666, attr_get_polling_rate, attr_set_polling_rate),
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


static inline s32 l3g4200dh_i2c_write(struct l3g4200dh *gyro, int reg, u8 value)
{
	struct i2c_client *c = gyro->i2cdev;
	return i2c_smbus_write_byte_data(c, reg, value);
}


static inline s32 l3g4200dh_i2c_read(struct l3g4200dh *gyro, int reg, u8 *value)
{
	struct i2c_client *client = gyro->i2cdev;
	*value = i2c_smbus_read_byte_data(client, reg);
	return 0;
}

static inline s32 l3g4200dh_i2c_blockread(struct l3g4200dh *gyro, int reg,
					  int len, u8 *v)
{
	struct i2c_client *c = gyro->i2cdev;
	reg |= (1 << 7); /* 7th bit enables address auto incrementation */
	return i2c_smbus_read_i2c_block_data(c, reg, len, v);
}



static int l3g4200dh_i2c_init(struct l3g4200dh *gyro)
{
	u8 reg;
	int ret;

	ret = gyro->read(gyro, WHO_AM_I, &reg);
	if (gyro->whoami != 0 && reg != gyro->whoami)
		dev_err(&gyro->i2cdev->dev, "l3g4200dh: incorrect device id\n");

	return ret;
}

/* Default axis mapping but it can be overwritten by platform data */
static struct axis_conversion l3g4200dh_axis_map = { 1, 2, 3
						/*L3G4200DH_DEV_X,
						  L3G4200DH_DEV_Y,
						  L3G4200DH_DEV_Z*/ };

static int __devinit l3g4200dh_i2c_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	int ret = 0;
	struct l3g4200dh_platform_data *pdata = client->dev.platform_data;

	pdata->dev = &client->dev;

/* kll.  passing client ONLY for debug output.  is there a better
 * way here? Note the lis3lv* guy passes a void pointer containing
 * plat data.  Prob better, but need to stash client in it i guess */

	if (pdata->setup_resources)
		ret = pdata->setup_resources(pdata);

	if (ret) {
		dev_err(&client->dev, "in %s. setup fail\n",
			__func__);
		goto fail;
	}

	l3g4200dh_dev.pdata	 = pdata;

	l3g4200dh_dev.i2cdev	= client;
	l3g4200dh_dev.init	= l3g4200dh_i2c_init;
	l3g4200dh_dev.read	= l3g4200dh_i2c_read;
	l3g4200dh_dev.blkread   = l3g4200dh_i2c_blockread;
	l3g4200dh_dev.write	= l3g4200dh_i2c_write;
	l3g4200dh_dev.irq	= client->irq;
	l3g4200dh_dev.ac	= l3g4200dh_axis_map;

	i2c_set_clientdata(client, &l3g4200dh_dev);

	ret = create_sysfs_interfaces(&client->dev);
	if (ret < 0) {
		dev_err(&client->dev, "device sysfs register failed\n");
		goto fail;
	}

	ret = l3g4200dh_init_device(&l3g4200dh_dev);
	atomic_set(&l3g4200dh_dev.enabled, 1);

fail:
	dev_dbg(&client->dev, "in %s. return %d\n", __func__, ret);
	return ret;
}

static int __devexit l3g4200dh_i2c_remove(struct i2c_client *client)
{
	struct l3g4200dh_platform_data *pdata = client->dev.platform_data;

	remove_sysfs_interfaces(&client->dev);
	if (pdata && pdata->release_resources)
		pdata->release_resources(pdata);

	return 0;
}



#ifdef CONFIG_PM
static int l3g4200dh_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct l3g4200dh *gyro = i2c_get_clientdata(client);

	mutex_lock(&gyro->mutex);
	/* if (!gyro->pdata->wakeup_flags && gyro->users)
	// TODO users counting */
	l3g4200dh_poweroff(gyro);
	mutex_unlock(&gyro->mutex);
	return 0;
}

static int l3g4200dh_i2c_resume(struct i2c_client *client)
{
	struct l3g4200dh *gyro = i2c_get_clientdata(client);

	mutex_lock(&gyro->mutex);
	/*if (!gyro->pdata->wakeup_flags && gyro->users)
	//TODO user counting */
	l3g4200dh_poweron(gyro);
	mutex_unlock(&gyro->mutex);
	return 0;
}

static void l3g4200dh_i2c_shutdown(struct i2c_client *client)
{
	l3g4200dh_i2c_suspend(client, PMSG_SUSPEND);
}
#else
#define l3g4200dh_i2c_suspend	NULL
#define l3g4200dh_i2c_resume	NULL
#define l3g4200dh_i2c_shutdown	NULL
#endif


static const struct i2c_device_id l3g4200dh_id[] = {
	{"l3g4200dh_i2c", 0 },
	{"l3g4200", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, l3g4200dh_id);

static struct i2c_driver l3g4200dh_i2c_driver = {
	.driver	 = {
		.name   = DRV_NAME,
		.owner  = THIS_MODULE,
	},
	//	.suspend  = l3g4200dh_i2c_suspend,
	//	.shutdown = l3g4200dh_i2c_shutdown,
	//	.resume	  = l3g4200dh_i2c_resume,
	.probe    = l3g4200dh_i2c_probe,
	.remove	  = __devexit_p(l3g4200dh_i2c_remove),
	.id_table = l3g4200dh_id,
};

static int __init l3g4200dh_init(void)
{
	return i2c_add_driver(&l3g4200dh_i2c_driver);
}

static void __exit l3g4200dh_exit(void)
{
	i2c_del_driver(&l3g4200dh_i2c_driver);
}

MODULE_AUTHOR("TI");
MODULE_DESCRIPTION("l3g4200dh I2C interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:" DRV_NAME);

module_init(l3g4200dh_init);
module_exit(l3g4200dh_exit);
