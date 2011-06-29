/*
 * LCD panel driver for Samsung LMS700KF23
 *
 * Copyright (C) 2011 TI India
 * Author: Goutam Kumar <goutam.kumar@ti.com>
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
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/err.h>
#include <linux/slab.h>

#include <plat/display.h>

struct samsung_lms_data {
	struct backlight_device *bl;
};

static struct omap_video_timings samsung_lms_timings = {
	.x_res = 800,
	.y_res = 480,

	.pixel_clock	= 24500,

	.hsw		= 1,
	.hfp		= 8,
	.hbp		= 16,

	.vsw		= 1,
	.vfp		= 5,
	.vbp		= 8,
};

#ifndef CONFIG_MACH_FLASHBOARD
static int samsung_lms_bl_update_status(struct backlight_device *bl)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&bl->dev);
	int level;

	if (!dssdev->set_backlight)
		return -EINVAL;

	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
			bl->props.power == FB_BLANK_UNBLANK)
		level = bl->props.brightness;
	else
		level = 0;

	return dssdev->set_backlight(dssdev, level);
}

static int samsung_lms_bl_get_brightness(struct backlight_device *bl)
{
	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
			bl->props.power == FB_BLANK_UNBLANK)
		return bl->props.brightness;

	return 0;
}

static const struct backlight_ops samsung_lms_bl_ops = {
	.get_brightness = samsung_lms_bl_get_brightness,
	.update_status  = samsung_lms_bl_update_status,
};
#endif


static int samsung_lms_panel_probe(struct omap_dss_device *dssdev)
{
#ifndef CONFIG_MACH_FLASHBOARD
	struct backlight_properties props;
	struct backlight_device *bl;
	struct samsung_lms_data *sd;
	int r;
#endif

	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |
		OMAP_DSS_LCD_IHS;
	dssdev->panel.acb = 0x28;
	dssdev->panel.timings = samsung_lms_timings;

#ifndef CONFIG_MACH_FLASHBOARD
	sd = kzalloc(sizeof(*sd), GFP_KERNEL);
	if (!sd)
		return -ENOMEM;

	dev_set_drvdata(&dssdev->dev, sd);
	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = dssdev->max_backlight_level;

	bl = backlight_device_register("samsung-lms", &dssdev->dev, dssdev,
			&samsung_lms_bl_ops, &props);
	if (IS_ERR(bl)) {
		r = PTR_ERR(bl);
		kfree(sd);
		return r;
	}
	sd->bl = bl;

	bl->props.fb_blank = FB_BLANK_UNBLANK;
	bl->props.power = FB_BLANK_UNBLANK;
	bl->props.brightness = dssdev->max_backlight_level;
	r = samsung_lms_bl_update_status(bl);
	if (r < 0)
		dev_err(&dssdev->dev, "failed to set lcd brightness\n");
#endif

	return 0;
}

static void samsung_lms_panel_remove(struct omap_dss_device *dssdev)
{
#ifndef CONFIG_MACH_FLASHBOARD
	struct samsung_lms_data *sd = dev_get_drvdata(&dssdev->dev);
	struct backlight_device *bl = sd->bl;

	bl->props.power = FB_BLANK_POWERDOWN;
	samsung_lms_bl_update_status(bl);
	backlight_device_unregister(bl);
	kfree(sd);
#endif
}

static int samsung_lms_power_on(struct omap_dss_device *dssdev)
{
	int r = 0;

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	r = omapdss_dpi_display_enable(dssdev);
	if (r)
		goto err0;

	/* wait couple of vsyncs until enabling the LCD */
	msleep(50);

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			goto err1;
	}

	return 0;
err1:
	omapdss_dpi_display_disable(dssdev);
err0:
	return r;
}

static void samsung_lms_power_off(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	/* wait at least 5 vsyncs after disabling the LCD */

	msleep(100);

	omapdss_dpi_display_disable(dssdev);
}

static int samsung_lms_panel_enable(struct omap_dss_device *dssdev)
{
	int r;
	r = samsung_lms_power_on(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	return r;
}

static void samsung_lms_panel_disable(struct omap_dss_device *dssdev)
{
	samsung_lms_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int samsung_lms_panel_suspend(struct omap_dss_device *dssdev)
{
	samsung_lms_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
	return 0;
}

static int samsung_lms_panel_resume(struct omap_dss_device *dssdev)
{
	int r;
	r = samsung_lms_power_on(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;
	return r;
}

static struct omap_dss_driver samsung_lms_driver = {
	.probe		= samsung_lms_panel_probe,
	.remove		= samsung_lms_panel_remove,

	.enable		= samsung_lms_panel_enable,
	.disable	= samsung_lms_panel_disable,
	.suspend	= samsung_lms_panel_suspend,
	.resume		= samsung_lms_panel_resume,

	.driver         = {
		.name   = "samsung_lms_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init samsung_lms_panel_drv_init(void)
{
	return omap_dss_register_driver(&samsung_lms_driver);
}

static void __exit samsung_lms_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&samsung_lms_driver);
}

module_init(samsung_lms_panel_drv_init);
module_exit(samsung_lms_panel_drv_exit);
MODULE_LICENSE("GPL");
