/*
 * Board file for Gumstix Pepper Single Board Computer.
 *
 * Copyright (C) 2013 Gumstix, Inc. - http://ww.gumstix.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/phy.h>
#include <linux/micrel_phy.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/mfd/tps65217.h>
#include <linux/input/ti_tsc.h>
#include <linux/platform_data/ti_adc.h>
#include <linux/mfd/ti_tscadc.h>
#include <linux/reboot.h>
#include <linux/pwm/pwm.h>
#include <linux/rtc/rtc-omap.h>
#include <linux/opp.h>
#include <linux/lis3lv02d.h>
#include <linux/regulator/fixed.h>

#include <video/da8xx-fb.h>

#include <mach/hardware.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hardware/asp.h>

#include <plat/omap_device.h>
#include <plat/omap-pm.h>
#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/lcdc.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/emif.h>

#include <sound/tlv320aic3x.h>

#include "cpuidle33xx.h"
#include "mux.h"
#include "devices.h"
#include "hsmmc.h"
#include "common.h"

/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

/* Pin Mux */
#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	AM33XX_MUX(I2C0_SDA, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW | AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(I2C0_SCL, OMAP_MUX_MODE0 | AM33XX_SLEWCTRL_SLOW | AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

struct pinmux_config {
	const char *string_name; /* signal name format */
	int val; /* Options for the mux register value */
};

static void setup_pin_mux(struct pinmux_config *pin_mux)
{
	int i;

	for (i = 0; pin_mux->string_name != NULL; pin_mux++)
		omap_mux_init_signal(pin_mux->string_name, pin_mux->val);
}


/* Dummy function */
int am335x_evm_get_id(void)
{
        return 0;
}
EXPORT_SYMBOL(am335x_evm_get_id);

/* LCD */
#define GPIO_LCD_ENABLE GPIO_TO_PIN(1, 27)

static const struct display_panel disp_panel = {
	WVGA,
	32,
	32,
	COLOR_ACTIVE,
};

static struct lcd_ctrl_config lcd_cfg = {
	&disp_panel,
	.ac_bias		= 255,
	.ac_bias_intrpt		= 0,
	.dma_burst_sz		= 16,
	.bpp			= 32,
	.fdd			= 0x80,
	.tft_alt_mode		= 0,
	.stn_565_mode		= 0,
	.mono_8bit_mode		= 0,
	.invert_line_clock	= 1,
	.invert_frm_clock	= 1,
	.sync_edge		= 0,
	.sync_ctrl		= 1,
	.raster_order		= 0,
};

static void pepper_panel_power_ctrl(int val)
{
	gpio_set_value(GPIO_LCD_ENABLE, val);
}

struct da8xx_lcdc_platform_data samsung43_pdata = {
	.manu_name		= "Samsung",
	.controller_data	= &lcd_cfg,
	.type			= "Samsung_LMS430",
	.get_context_loss_count	= omap_pm_get_dev_context_loss_count,
	.panel_power_ctrl       = pepper_panel_power_ctrl,
};

struct da8xx_lcdc_platform_data newhaven43_pdata = {
	.manu_name		= "Newhaven",
	.controller_data	= &lcd_cfg,
	.type			= "NHD-4.3-ATXI#-T-1",
	.get_context_loss_count	= omap_pm_get_dev_context_loss_count,
	.panel_power_ctrl       = pepper_panel_power_ctrl,
};

/* LCD Pin Mux */
static struct pinmux_config lcdc_pin_mux[] = {
	{"lcd_data0.lcd_data0",			OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA},
	{"lcd_data1.lcd_data1",			OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA},
	{"lcd_data2.lcd_data2",			OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA},
	{"lcd_data3.lcd_data3",			OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA},
	{"lcd_data4.lcd_data4",			OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA},
	{"lcd_data5.lcd_data5",			OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA},
	{"lcd_data6.lcd_data6",			OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA},
	{"lcd_data7.lcd_data7",			OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA},
	{"lcd_data8.lcd_data8",			OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA},
	{"lcd_data9.lcd_data9",			OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA},
	{"lcd_data10.lcd_data10",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA},
	{"lcd_data11.lcd_data11",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA},
	{"lcd_data12.lcd_data12",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA},
	{"lcd_data13.lcd_data13",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA},
	{"lcd_data14.lcd_data14",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA},
	{"lcd_data15.lcd_data15",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT | AM33XX_PULL_DISA},
	{"gpmc_ad8.lcd_data16",			OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad9.lcd_data17",			OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad10.lcd_data18",		OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad11.lcd_data19",		OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad12.lcd_data20",		OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad13.lcd_data21",		OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad14.lcd_data22",		OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"gpmc_ad15.lcd_data23",		OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"lcd_vsync.lcd_vsync",			OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_hsync.lcd_hsync",			OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_pclk.lcd_pclk",			OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_ac_bias_en.lcd_ac_bias_en",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	/* Display Enable */
	{"gpmc_a11.gpio1_27",			OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

static int __init conf_disp_pll(int rate)
{
	struct clk *disp_pll;
	int ret = -EINVAL;

	disp_pll = clk_get(NULL, "dpll_disp_ck");
	if (IS_ERR(disp_pll)) {
		pr_err("Cannot clk_get disp_pll\n");
		goto out;
	}

	ret = clk_set_rate(disp_pll, rate);
	clk_put(disp_pll);
out:
	return ret;
}

static void pepper_lcd_init(void)
{
	int err;

	setup_pin_mux(lcdc_pin_mux);
    if (conf_disp_pll(18400000)) {
		pr_info("Failed to configure display PLL\n");
		return;
	}

	if (am33xx_register_lcdc(&samsung43_pdata)) {
		pr_info("Failed to register LCD display\n");
		return;
	}

	err = gpio_request_one(GPIO_LCD_ENABLE, GPIOF_OUT_INIT_LOW, "LCD Enable");
	if (err < 0) {
		pr_info("Failed to configure display enable\n");
		return;
	}
	gpio_export(GPIO_LCD_ENABLE, false);
	gpio_set_value(GPIO_LCD_ENABLE, 1);
}

/* Touchscreen & ADC  controller */
static struct tsc_data pepper_touchscreen_data  = {
	.wires			= 4,
        .x = {
                .min = 0xCB,
                .max = 0xF9B,
                .inverted = 0,
        },
        .y = {
                .min = 0xC8,
                .max = 0xE93,
                .inverted = 1,
        },
	.x_plate_resistance	= 200,
	.steps_to_configure	= 5,
};

static struct adc_data pepper_adc_data = {
	.adc_channels = 4,
};

static struct mfd_tscadc_board tscadc = {
        .tsc_init = &pepper_touchscreen_data,
        .adc_init = &pepper_adc_data,
};


static void pepper_tsc_init(void)
{
	if (am33xx_register_mfd_tscadc(&tscadc))
		pr_err("Failed to register Touchscreen & ADC device\n");
}

/* Audio Codec */
static u8 pepper_iis_serializer_direction[] = {
	TX_MODE,	RX_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
};

static struct snd_platform_data pepper_snd_data = {
	.tx_dma_offset	= 0x46000000,	/* McASP0 */
	.rx_dma_offset	= 0x46000000,
	.op_mode	= DAVINCI_MCASP_IIS_MODE,
	.num_serializer	= ARRAY_SIZE(pepper_iis_serializer_direction),
	.tdm_slots	= 2,
	.serial_dir	= pepper_iis_serializer_direction,
	.asp_chan_q	= EVENTQ_2,
	.version	= MCASP_VERSION_3,
	.txnumevt	= 1,
	.rxnumevt	= 1,
	.get_context_loss_count = omap_pm_get_dev_context_loss_count,
};

/* Audio Pin Mux */
static struct pinmux_config mcasp0_pin_mux[] = {
	{"mcasp0_fsx.mcasp0_fsx",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mcasp0_aclkx.mcasp0_aclkx",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mcasp0_axr0.mcasp0_axr0",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mcasp0_axr1.mcasp0_axr1",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	/* Audio nReset */
	{"gpmc_a0.gpio1_16",		OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

static struct aic3x_pdata pepper_aic3x_data __initdata = {
	.gpio_reset = GPIO_TO_PIN(1, 16),
};

static void __init pepper_audio_init(void)
{
	setup_pin_mux(mcasp0_pin_mux);
	am335x_register_mcasp(&pepper_snd_data, 0);
}

/* Ethernet */
static int ksz9021rn_phy_fixup(struct phy_device *phydev)
{
	/* max rx/tx clock delay, min rx/tx control delay */
	phy_write(phydev, 0x0b, 0x8104);
	phy_write(phydev, 0x0c, 0xa0a0);

	/* min rx data delay */
	phy_write(phydev, 0x0b, 0x8105);
	phy_write(phydev, 0x0c, 0x0000);

	/* min tx data delay */
	phy_write(phydev, 0x0b, 0x8106);
	phy_write(phydev, 0x0c, 0x0000);

	return 0;
}

/* Pin mux for Ethernet in RGMII mode */
static struct pinmux_config rgmii1_pin_mux[] = {
	{"mii1_txen.rgmii1_tctl",	OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_rxdv.rgmii1_rctl",	OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_txd3.rgmii1_td3",	OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_txd2.rgmii1_td2",	OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_txd1.rgmii1_td1",	OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_txd0.rgmii1_td0",	OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_txclk.rgmii1_tclk",	OMAP_MUX_MODE2 | AM33XX_PIN_OUTPUT},
	{"mii1_rxclk.rgmii1_rclk",	OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd3.rgmii1_rd3",	OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd2.rgmii1_rd2",	OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd1.rgmii1_rd1",	OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd0.rgmii1_rd0",	OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mdio_data.mdio_data",		OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mdio_clk.mdio_clk",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
	/* ethernet interrupt */
	{"rmii1_refclk.gpio0_29",	OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	/* PHY nReset */
	{"mii1_col.gpio3_0",		OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

void static __init pepper_ethernet_init(void)
{
	setup_pin_mux(rgmii1_pin_mux);
	if (IS_ENABLED(CONFIG_PHYLIB))
		phy_register_fixup_for_uid(PHY_ID_KSZ9021, MICREL_PHY_ID_MASK, ksz9021rn_phy_fixup);
	am33xx_cpsw_init(AM33XX_CPSW_MODE_RGMII, NULL, NULL);
}


/* USB */
static struct omap_musb_board_data musb_board_data = {
	.interface_type	= MUSB_INTERFACE_ULPI,
	/*
	 * mode[0:3] = USB0PORT's mode
	 * mode[4:7] = USB1PORT's mode
	 * Pepper has USB0 in OTG mode and USB1 in host mode.
	 */
	.mode           = (MUSB_HOST << 4) | MUSB_OTG,
	.power		= 500,
	.instances	= 1,
};

/* USB Pin Mux */
static struct pinmux_config usb_pin_mux[] = {
	{"usb0_drvvbus.usb0_drvvbus",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	/* USB0 Over-Current (active low) */
	{"gpmc_a9.gpio1_25",		OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"usb1_drvvbus.usb1_drvvbus",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	/* USB1 Over-Current (active low) */
	{"gpmc_a10.gpio1_26",		OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{NULL, 0},
};

void static __init pepper_usb_init(void)
{
	setup_pin_mux(usb_pin_mux);
	usb_musb_init(&musb_board_data);
}

/* MMC/SDIO Devices */
static struct omap2_hsmmc_info pepper_mmc[] __initdata = {
	{
		.mmc            = 1, /* SD card */
		.caps           = MMC_CAP_4_BIT_DATA,
		.gpio_cd        = GPIO_TO_PIN(0, 6),
		.gpio_wp	= -EINVAL,
		.ocr_mask       = MMC_VDD_32_33 | MMC_VDD_33_34, /* 3.3V */
	},
	{
		.mmc            = 2, /* eMMC */
		.caps           = MMC_CAP_8_BIT_DATA,
		.nonremovable	= true,
		.gpio_cd        = -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask       = MMC_VDD_32_33 | MMC_VDD_33_34, /* 3.3V */
	},
	{
		.mmc            = 3, /* Bluetooth & Wifi */
		.caps           = MMC_CAP_4_BIT_DATA,
		.nonremovable	= true,
		.gpio_cd        = -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask       = MMC_VDD_32_33 | MMC_VDD_33_34, /* 3.3V */
	},
	{}      /* Terminator */
};

/* Module pin mux for mmc0 */
static struct pinmux_config mmc0_pin_mux[] = {
	{"mmc0_dat3.mmc0_dat3",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat2.mmc0_dat2",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat1.mmc0_dat1",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat0.mmc0_dat0",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_clk.mmc0_clk",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_cmd.mmc0_cmd",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"spi0_cs1.mmc0_sdcd",	OMAP_MUX_MODE5 | AM33XX_PIN_INPUT},
	{NULL, 0},
};

/* eMMC/MMC1 pin mux */
static struct pinmux_config mmc1_pin_mux[] = {
        {"gpmc_ad3.mmc1_dat3",  OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
        {"gpmc_ad2.mmc1_dat2",  OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
        {"gpmc_ad1.mmc1_dat1",  OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
        {"gpmc_ad0.mmc1_dat0",  OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
        {"gpmc_csn1.mmc1_clk",  OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
        {"gpmc_csn2.mmc1_cmd",  OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
        {"gpmc_ad7.mmc1_dat7",  OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
        {"gpmc_ad6.mmc1_dat6",  OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
        {"gpmc_ad5.mmc1_dat5",  OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
        {"gpmc_ad4.mmc1_dat4",  OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLUP},
	/* eMMC nReset */
        {"gpmc_wpn.gpio0_31",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
        {NULL, 0},
};

static void __init pepper_mmc_init(void)
{
	setup_pin_mux(mmc0_pin_mux);
	setup_pin_mux(mmc1_pin_mux);
	omap2_hsmmc_init(pepper_mmc);
}

/* Wifi/Bluetooth Pin Mux */
static struct pinmux_config wlan_pin_mux[] = {
	{"gpmc_a1.mmc2_dat0",	OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_a2.mmc2_dat1",	OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_a3.mmc2_dat2",	OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ben1.mmc2_dat3",	OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn3.mmc2_cmd",	OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_clk.mmc2_clk",	OMAP_MUX_MODE3 | AM33XX_PIN_INPUT_PULLUP},
	/* WLAN nReset */
        {"gpmc_a8.gpio1_24",	OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	/* WLAN nPower down */
        {"gpmc_wait0.gpio0_30",	OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct regulator_consumer_supply pepper_vmmc3_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.2"),
};

static struct regulator_init_data pepper_vmmc3 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies = ARRAY_SIZE(pepper_vmmc3_supply),
	.consumer_supplies = pepper_vmmc3_supply,
};

static struct fixed_voltage_config pepper_wlan = {
	.supply_name		= "vwlan",
	.microvolts		= 3300000, /* 3.3V */
	.gpio			= -EINVAL,
	.startup_delay		= 0,
	.init_data		= &pepper_vmmc3,
};

static struct platform_device pepper_wlan_device = {
	.name		= "reg-fixed-voltage",
	.id		= 1,
	.dev = {
		.platform_data = &pepper_wlan,
	},
};

static void __init pepper_wlan_init(void)
{
	setup_pin_mux(wlan_pin_mux);
	platform_device_register(&pepper_wlan_device);
}

/* Setup User Buttons as keys */
static struct gpio_keys_button gpio_buttons[] = {
	{
		.code		= KEY_BACK,
		.gpio		= GPIO_TO_PIN(1, 22),
		.active_low	= true,
		.desc		= "home",
		.type		= EV_KEY,
		.wakeup		= 1,
	},
	{
		.code		= KEY_MENU,
		.gpio		= GPIO_TO_PIN(1, 23),
		.active_low	= true,
		.desc		= "menu",
		.type		= EV_KEY,
		.wakeup		= 1,
	},
	{
		.code		= KEY_POWER,
		.gpio		= GPIO_TO_PIN(0, 7),
		.active_low	= true,
		.desc		= "power",
		.type		= EV_KEY,
		.wakeup		= 1,
	},
};

static struct gpio_keys_platform_data pepper_button_info = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};

static struct platform_device pepper_user_keys = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &pepper_button_info,
	},
};

/* Pin mux for User Buttons */
static struct pinmux_config user_button_mux[] = {
	{"gpmc_a6.gpio1_22",		OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_a7.gpio1_23",		OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{"ecap0_in_pwm0_out.gpio0_7",	OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static void user_keys_init(void)
{
	int err;

	setup_pin_mux(user_button_mux);
	err = platform_device_register(&pepper_user_keys);
	if (err)
		pr_err("Failed to register user keys\n");
}

/* Setup User LEDs */
static struct gpio_led gpio_leds[] = {
	{
		.name	= "pepper:user0:blue",
		.gpio	= GPIO_TO_PIN(1, 20),
	},
	{
		.name	= "pepper:user1:red",
		.gpio	= GPIO_TO_PIN(1, 21),
	},
};

static struct gpio_led_platform_data pepper_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device pepper_user_leds = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &pepper_led_info,
	},
};

/* Pin mux for User LEDs */
static struct pinmux_config user_led_mux[] = {
	{"gpmc_a4.gpio1_20",	OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{"gpmc_a5.gpio1_21",	OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static void user_leds_init(void)
{
	int err;

	setup_pin_mux(user_led_mux);
	err = platform_device_register(&pepper_user_leds);
	if (err)
		pr_err("Failed to register user LEDs\n");
}

/* RAM via EMIF */
void __iomem *pepper_emif_base;

void __iomem * __init pepper_get_mem_ctlr(void)
{
	pepper_emif_base = ioremap(AM33XX_EMIF0_BASE, SZ_32K);

	if (!pepper_emif_base)
		pr_warning("%s: Unable to map DDR2 controller",	__func__);

	return pepper_emif_base;
}

void __iomem *am33xx_get_ram_base(void)
{
	return pepper_emif_base;
}

/* For power management? */
void __iomem *am33xx_gpio0_base;

void __iomem *am33xx_get_gpio0_base(void)
{
        am33xx_gpio0_base = ioremap(AM33XX_GPIO0_BASE, SZ_4K);

        return am33xx_gpio0_base;
}

/* CPU Idle configuration */
static struct resource am33xx_cpuidle_resources[] = {
        {
                .start          = AM33XX_EMIF0_BASE,
                .end            = AM33XX_EMIF0_BASE + SZ_32K - 1,
                .flags          = IORESOURCE_MEM,
        },
};

static struct am33xx_cpuidle_config am33xx_cpuidle_pdata = {
};

static struct platform_device am33xx_cpuidle_device = {
        .name                   = "cpuidle-am33xx",
        .num_resources          = ARRAY_SIZE(am33xx_cpuidle_resources),
        .resource               = am33xx_cpuidle_resources,
        .dev = {
                .platform_data  = &am33xx_cpuidle_pdata,
        },
};

static void __init am33xx_cpuidle_init(void)
{
        int ret;

        am33xx_cpuidle_pdata.emif_base = pepper_get_mem_ctlr();
        ret = platform_device_register(&am33xx_cpuidle_device);
        if (ret)
                pr_warning("AM33XX cpuidle registration failed\n");
}

/* OMAP board config */
static struct omap_board_config_kernel pepper_config[] __initdata = {
};

/* Configure TPS65217B PMIC */
/* 1.8V */
static struct regulator_consumer_supply tps65217_dcdc1_consumers[] = {
	{
		.supply = "vdds_osc",
	},
	{
		.supply = "vdds_pll_ddr",
	},
	{
		.supply = "vdds_pll_mpu",
	},
	{
		.supply = "vdds_pll_core_lcd",
	},
	{
		.supply = "vdds_sram_mpu_bb",
	},
	{
		.supply = "vdds_sram_core_bg",
	},
	{
		.supply = "vdda_usb0_1p8v",
	},
	{
		.supply = "vdds_ddr",
	},
	{
		.supply = "vdds",
	},
	{
		.supply = "vdds_hvx_1p8v",
	},
	{
		.supply = "vdda_adc",
	},
	{
		.supply = "ddr2",
	},
	{
		.supply = "DVDD",
	},
};

/* Roughly 1.1V */
static struct regulator_consumer_supply tps65217_dcdc2_consumers[] = {
	{
		.supply = "vdd_core",
	},
};

/* Roughly 1.1V */
static struct regulator_consumer_supply tps65217_dcdc3_consumers[] = {
	{
		.supply = "vdd_mpu",
	},
};

/* 1.8V LDO */
static struct regulator_consumer_supply tps65217_ldo1_consumers[] = {
	{
		.supply = "vdds_rtc",
	},
};

/* 3.3V LDO */
static struct regulator_consumer_supply tps65217_ldo2_consumers[] = {
	{
		.supply = "vdds_any_pn",
	},
};

/* 3.3V LDO */
static struct regulator_consumer_supply tps65217_ldo3_consumers[] = {
	{
		.supply = "vdds_hvx_ldo3_3p3v",
	},
	{
		.supply = "vdda_usb0_3p3v",
	},
	{
		.supply = "AVDD",
	},
	{
		.supply = "IOVDD",
	},
	{
		.supply = "DRVDD",
	},
};

/* 3.3V LDO */
static struct regulator_consumer_supply tps65217_ldo4_consumers[] = {
	{
		.supply = "vdds_hvx_ldo4_3p3v",
	},
};

static struct regulator_init_data tps65217_regulator_data[] = {
	/* DCDC #1 */
	{
		.constraints = {
			.min_uV = 900000,
			.max_uV = 1800000,
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_dcdc1_consumers),
		.consumer_supplies = tps65217_dcdc1_consumers,
	},
	/* DCDC #2 */
	{
		.constraints = {
			.min_uV = 900000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_dcdc2_consumers),
		.consumer_supplies = tps65217_dcdc2_consumers,
	},
	/* DCDC #3 */
	{
		.constraints = {
			.min_uV = 900000,
			.max_uV = 1500000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_dcdc3_consumers),
		.consumer_supplies = tps65217_dcdc3_consumers,
	},
	/* LDO #1 */
	{
		.constraints = {
			.min_uV = 1000000,
			.max_uV = 3300000,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_ldo1_consumers),
		.consumer_supplies = tps65217_ldo1_consumers,
	},
	/* LDO #2 */
	{
		.constraints = {
			.min_uV = 900000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_ldo2_consumers),
		.consumer_supplies = tps65217_ldo2_consumers,
	},

	/* LDO #3 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_ldo3_consumers),
		.consumer_supplies = tps65217_ldo3_consumers,
	},

	/* LDO #4 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 3300000,
			.valid_ops_mask = (REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS),
			.boot_on = 1,
			.always_on = 1,
		},
		.num_consumer_supplies = ARRAY_SIZE(tps65217_ldo4_consumers),
		.consumer_supplies = tps65217_ldo4_consumers,
	},
};

struct tps65217_bl_pdata pepper_backlight_pdata[] = {
	{
		.isel	= TPS65217_BL_ISET1,
		.fdim	= TPS65217_BL_FDIM_200HZ,
	}
};

static struct tps65217_board pepper_tps65217_info = {
	.tps65217_init_data	= &tps65217_regulator_data[0],
	.bl_pdata		= pepper_backlight_pdata,
	.status_off		= true,
};

/* Setup LIS33DE Accelerometer */
static struct lis3lv02d_platform_data accel_pdata = {
	.wakeup_flags = LIS3_WAKEUP_X_LO | LIS3_WAKEUP_X_HI | LIS3_WAKEUP_Y_LO | LIS3_WAKEUP_Y_HI | LIS3_WAKEUP_Z_LO | LIS3_WAKEUP_Z_HI,
	.irq_cfg = LIS3_IRQ1_FF_WU_1,
	.g_range = 2,
	/* setup landscape convention */
	.axis_x = 1,
	.axis_y = 0,
	/* 1 LSB = 4.6; min = 3LSB, max = 32LSB */ 
	.st_min_limits[0] = -92,
	.st_min_limits[1] = 14,
	.st_min_limits[2] = -92,
	.st_max_limits[0] = -14,
	.st_max_limits[1] = 92,
	.st_max_limits[2] = -14,
};

static struct pinmux_config accel_pin_mux[] = {
	{"gpmc_wen.gpio2_4",	OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{NULL, 0},
};

/* Setup I2C Buses */
static void __iomem *am33xx_i2c0_base;

int am33xx_map_i2c0(void)
{
	am33xx_i2c0_base = ioremap(AM33XX_I2C0_BASE, SZ_4K);

	if (!am33xx_i2c0_base)
		return -ENOMEM;

	return 0;
}

void __iomem *am33xx_get_i2c0_base(void)
{
	return am33xx_i2c0_base;
}

static struct i2c_board_info pepper_i2c0_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps65217", TPS65217_I2C_ID),
		.platform_data  = &pepper_tps65217_info,
	},
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x1b),
		.platform_data = &pepper_aic3x_data,
	},
	{
		I2C_BOARD_INFO("lis331dlh", 0x1d),
		.platform_data = &accel_pdata,
		.irq = OMAP_GPIO_IRQ(GPIO_TO_PIN(2, 4)),
	},
};

static struct pinmux_config i2c1_pin_mux[] = {
	{"mii1_rxerr.i2c1_scl",	OMAP_MUX_MODE3 | AM33XX_SLEWCTRL_SLOW | AM33XX_PULL_ENBL | AM33XX_INPUT_EN},
	{"mii1_crs.i2c1_sda",	OMAP_MUX_MODE3 | AM33XX_SLEWCTRL_SLOW | AM33XX_PULL_ENBL | AM33XX_INPUT_EN},
	{NULL, 0},
};

static void pepper_i2c_init(void)
{
	setup_pin_mux(accel_pin_mux);
	setup_pin_mux(i2c1_pin_mux);
	omap_register_i2c_bus(1, 100, pepper_i2c0_boardinfo, ARRAY_SIZE(pepper_i2c0_boardinfo));
}

/* SPI */
static struct spi_board_info pepper_spi_slave_info[] = {
};

static struct pinmux_config spi0_pin_mux[] = {
	{"spi0_sclk.spi0_sclk",		OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"spi0_cs0.spi0_cs0",		OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"spi0_d0.spi0_d0",		OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"spi0_d1.spi0_d1",		OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static void pepper_spi_init(void)
{
	setup_pin_mux(spi0_pin_mux);
	spi_register_board_info(pepper_spi_slave_info, ARRAY_SIZE(pepper_spi_slave_info));
}

/* 32KHz Clock */
static struct pinmux_config clkout2_pin_mux[] = {
	{"xdma_event_intr1.clkout2",	OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

static void __init clkout2_enable(void)
{
	struct clk *ck_32;

	ck_32 = clk_get(NULL, "clkout2_ck");
	if (IS_ERR(ck_32)) {
		pr_err("Cannot clk_get ck_32\n");
		return;
	}

	clk_enable(ck_32);
	setup_pin_mux(clkout2_pin_mux);
}

/* Real-time Clock */
static struct omap_rtc_pdata am335x_rtc_info = {
	.pm_off		= false,
	.wakeup_capable	= 0,
};

static void am335x_rtc_init(void)
{
	void __iomem *base;
	struct clk *clk;
	struct omap_hwmod *oh;
	struct platform_device *pdev;
	char *dev_name = "am33xx-rtc";

	clk = clk_get(NULL, "rtc_fck");
	if (IS_ERR(clk)) {
		pr_err("rtc : Failed to get RTC clock\n");
		return;
	}

	if (clk_enable(clk)) {
		pr_err("rtc: Clock Enable Failed\n");
		return;
	}

	base = ioremap(AM33XX_RTC_BASE, SZ_4K);

	if (WARN_ON(!base))
		return;

	/* Unlock the RTC's registers */
	writel(0x83e70b13, base + 0x6c);
	writel(0x95a4f1e0, base + 0x70);

	/*
	 * Enable the 32K OSc
	 * TODO: Need a better way to handle this
	 * Since we want the clock to be running before mmc init
	 * we need to do it before the rtc probe happens
	 */
	writel(0x48, base + 0x54);
	iounmap(base);
	clk_disable(clk);
	clk_put(clk);
	if (omap_rev() >= AM335X_REV_ES2_0)
		am335x_rtc_info.wakeup_capable = 1;

	oh = omap_hwmod_lookup("rtc");
	if (!oh) {
		pr_err("could not look up %s\n", "rtc");
		return;
	}

	pdev = omap_device_build(dev_name, -1, oh, &am335x_rtc_info, sizeof(struct omap_rtc_pdata), NULL, 0, 0);
	WARN(IS_ERR(pdev), "Can't build omap_device for %s:%s.\n", dev_name, oh->name);
}

/* Initialize Pepper */
static void __init pepper_init(void)
{
	am33xx_cpuidle_init();
	am33xx_mux_init(board_mux);
	omap_serial_init();
	am335x_rtc_init();
	clkout2_enable();
	pepper_i2c_init();
	omap_sdrc_init(NULL, NULL);
	omap_board_config = pepper_config;
	omap_board_config_size = ARRAY_SIZE(pepper_config);
	pepper_mmc_init();
	pepper_lcd_init();
	pepper_tsc_init();
	pepper_audio_init();
	pepper_ethernet_init();
	pepper_spi_init();
	pepper_usb_init();
	pepper_wlan_init();
	user_leds_init();
	user_keys_init();
	if (omap3_has_sgx())
		am33xx_gpu_init();

        /* Create an alias for icss clock */
        if (clk_add_alias("pruss", NULL, "pruss_uart_gclk", NULL))
                pr_warn("failed to create an alias: icss_uart_gclk --> pruss\n");
        /* Create an alias for gfx/sgx clock */
        if (clk_add_alias("sgx_ck", NULL, "gfx_fclk", NULL))
                pr_warn("failed to create an alias: gfx_fclk --> sgx_ck\n");
}

static void __init pepper_map_io(void)
{
	omap2_set_globals_am33xx();
	omapam33xx_map_common_io();
}

MACHINE_START(PEPPER, "pepper")
	/* Maintainer: Ash Charles (ash@gumstix.com) */
	.atag_offset	= 0x100,
	.map_io		= pepper_map_io,
	.init_early	= am33xx_init_early,
	.init_irq	= ti81xx_init_irq,
	.handle_irq     = omap3_intc_handle_irq,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= pepper_init,
MACHINE_END
