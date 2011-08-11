/*
 * Support for AM3517/05 Craneboard
 * http://www.mistralsolutions.com/products/craneboard.php
 *
 * Copyright (C) 2010 Mistral Solutions Pvt Ltd. <www.mistralsolutions.com>
 * Author: R.Srinath <srinath@mistralsolutions.com>
 *
 * Based on mach-omap2/board-am3517evm.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as  published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/can/platform/ti_hecc.h>
#include <linux/davinci_emac.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/mmc/host.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>

#include <mach/hardware.h>
#include <mach/am35xx.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/display.h>
#include <plat/gpmc.h>
#include <plat/nand.h>

#include <linux/usb/android_composite.h>

#include "mux.h"
#include "control.h"
#include "hsmmc.h"
#include "board-flash.h"

#define GPIO_USB_POWER          35
#define GPIO_USB_NRESET         38

#define NAND_BLOCK_SIZE        SZ_128K

#ifdef CONFIG_USB_ANDROID

#define GOOGLE_VENDOR_ID                0x18d1
#define GOOGLE_PRODUCT_ID               0x9018
#define GOOGLE_ADB_PRODUCT_ID           0x9015

static char *usb_functions_adb[] = {
	"adb",
};

static char *usb_functions_mass_storage[] = {
	"usb_mass_storage",
};
static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_all[] = {
	"adb", "usb_mass_storage",
	"adb",
};

static struct android_usb_product usb_products[] = {
	{
		.product_id     = GOOGLE_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_adb),
		.functions      = usb_functions_adb,
	},
	{
		.product_id	= GOOGLE_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_mass_storage),
		.functions	= usb_functions_mass_storage,
	},
	{
		.product_id	= GOOGLE_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "rowboat",
	.product	= "rowboat gadget",
	.release	= 0x100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id      = GOOGLE_VENDOR_ID,
	.product_id     = GOOGLE_PRODUCT_ID,
	.functions      = usb_functions_all,
	.num_products	= ARRAY_SIZE(usb_products),
	.products       = usb_products,
	.version        = 0x0100,
	.product_name   = "rowboat gadget",
	.manufacturer_name      = "rowboat",
	.serial_number  = "20100720",
	.num_functions  = ARRAY_SIZE(usb_functions_all),
};

static struct platform_device androidusb_device = {
	.name   = "android_usb",
	.id     = -1,
	.dev    = {
		.platform_data = &android_usb_pdata,
	},
};

static void omap3evm_android_gadget_init(void)
{
	platform_device_register(&androidusb_device);
}

#endif

#if 0

static struct mtd_partition am3517crane_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name           = "xloader-nand",
		.offset         = 0,
		.size           = 4*(SZ_128K),
		.mask_flags     = MTD_WRITEABLE
	},
	{
		.name           = "uboot-nand",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 28*(SZ_128K),
		.mask_flags     = MTD_WRITEABLE
	},
	{
		.name           = "params-nand",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 4*(SZ_128K)
	},
	{
		.name           = "linux-nand",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 80*(SZ_128K)
	},
	{
		.name           = "jffs2-nand",
		.size           = MTDPART_SIZ_FULL,
		.offset         = MTDPART_OFS_APPEND,
	},
};
#endif

static struct emac_platform_data am3517_crane_emac_pdata = {
	.rmii_en	= 1,
};

static struct resource am3517_emac_resources[] = {
	{
		.start  = AM35XX_IPSS_EMAC_BASE,
		.end    = AM35XX_IPSS_EMAC_BASE + 0x2FFFF,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.end    = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device am3517_emac_device = {
	.name		= "davinci_emac",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(am3517_emac_resources),
	.resource	= am3517_emac_resources,
};

static void am3517_enable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
			AM35XX_CPGMAC_C0_TX_PULSE_CLR |
			AM35XX_CPGMAC_C0_MISC_PULSE_CLR |
			AM35XX_CPGMAC_C0_RX_THRESH_CLR);
	omap_ctrl_writel(regval, AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

static void am3517_disable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
			AM35XX_CPGMAC_C0_TX_PULSE_CLR);
	omap_ctrl_writel(regval, AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

static void am3517_crane_ethernet_init(struct emac_platform_data *pdata)
{
	u32 regval, mac_lo, mac_hi;

	mac_lo = omap_ctrl_readl(AM35XX_CONTROL_FUSE_EMAC_LSB);
	mac_hi = omap_ctrl_readl(AM35XX_CONTROL_FUSE_EMAC_MSB);

	pdata->mac_addr[0] = (u_int8_t)((mac_hi & 0xFF0000) >> 16);
	pdata->mac_addr[1] = (u_int8_t)((mac_hi & 0xFF00) >> 8);
	pdata->mac_addr[2] = (u_int8_t)((mac_hi & 0xFF) >> 0);
	pdata->mac_addr[3] = (u_int8_t)((mac_lo & 0xFF0000) >> 16);
	pdata->mac_addr[4] = (u_int8_t)((mac_lo & 0xFF00) >> 8);
	pdata->mac_addr[5] = (u_int8_t)((mac_lo & 0xFF) >> 0);

	pdata->ctrl_reg_offset		= AM35XX_EMAC_CNTRL_OFFSET;
	pdata->ctrl_mod_reg_offset	= AM35XX_EMAC_CNTRL_MOD_OFFSET;
	pdata->ctrl_ram_offset		= AM35XX_EMAC_CNTRL_RAM_OFFSET;
	pdata->ctrl_ram_size		= AM35XX_EMAC_CNTRL_RAM_SIZE;
	pdata->version			= EMAC_VERSION_2;
	pdata->hw_ram_addr		= AM35XX_EMAC_HW_RAM_ADDR;
	pdata->interrupt_enable		= am3517_enable_ethernet_int;
	pdata->interrupt_disable	= am3517_disable_ethernet_int;
	am3517_emac_device.dev.platform_data	= pdata;
	platform_device_register(&am3517_emac_device);

	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);
	regval = regval & (~(AM35XX_CPGMACSS_SW_RST));
	omap_ctrl_writel(regval, AM35XX_CONTROL_IP_SW_RESET);
	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);

	return ;
}


static void __init am3517_crane_display_init(void)
{
	omap_mux_init_gpio(52, OMAP_PIN_OUTPUT);
	gpio_request(52, "dvi_enable");
	gpio_direction_output(52, 1);
}


static int am3517_crane_panel_enable_dvi(struct omap_dss_device *dssdev)
{
	gpio_set_value(52, 1);
	return 0;
}

static void am3517_crane_panel_disable_dvi(struct omap_dss_device *dssdev)
{
	gpio_set_value(52, 0);
}

static struct omap_dss_device am3517_crane_dvi_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "dvi",
	.driver_name		= "generic_panel",
	.phy.dpi.data_lines	= 24,
	.platform_enable	= am3517_crane_panel_enable_dvi,
	.platform_disable	= am3517_crane_panel_disable_dvi,
};

static struct omap_dss_device *am3517_crane_dss_devices[] = {
	&am3517_crane_dvi_device,
};

static struct omap_dss_board_info am3517_crane_dss_data = {
	.num_devices	= ARRAY_SIZE(am3517_crane_dss_devices),
	.devices	= am3517_crane_dss_devices,
	.default_device	= &am3517_crane_dvi_device,
};

static struct platform_device am3517_crane_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data	= &am3517_crane_dss_data,
	},
};

/*
 * Board initialization
 */
static struct omap_board_config_kernel am3517_crane_config[] __initdata = {
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux       NULL
#endif


static struct platform_device *am3517_crane_devices[] __initdata = {
	&am3517_crane_dss_device,
	&usb_mass_storage_device,
};

static void __init am3517_crane_init_irq(void)
{
	omap_board_config = am3517_crane_config;
	omap_board_config_size = ARRAY_SIZE(am3517_crane_config);

	omap2_init_common_infrastructure();
	omap2_init_common_devices(NULL, NULL);
	omap_init_irq();
}


static struct omap_musb_board_data musb_board_data = {
	.interface_type         = MUSB_INTERFACE_ULPI,
	.mode                   = MUSB_OTG,
	.power                  = 250,
};


static __init void am3517_crane_musb_init(void)
{
	u32 devconf2;

	/*
	 * Set up USB clock/mode in the DEVCONF2 register.
	 */
	devconf2 = omap_ctrl_readl(AM35XX_CONTROL_DEVCONF2);

	/* USB2.0 PHY reference clock is 13 MHz */
	devconf2 &= ~(CONF2_REFFREQ | CONF2_OTGMODE | CONF2_PHY_GPIOMODE);
	devconf2 |=  CONF2_REFFREQ_13MHZ | CONF2_SESENDEN | CONF2_VBDTCTEN
		| CONF2_DATPOL;

	omap_ctrl_writel(devconf2, AM35XX_CONTROL_DEVCONF2);

	usb_musb_init(&musb_board_data);
}


static struct ehci_hcd_omap_platform_data ehci_pdata __initdata = {
	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = GPIO_USB_NRESET,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};


static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= 41,
		.gpio_wp	= 40,
	},
	{}      /* Terminator */
};


static void __init am3517_crane_init(void)
{
	int ret;

	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);

	platform_add_devices(am3517_crane_devices,
			ARRAY_SIZE(am3517_crane_devices));

	omap_serial_init();

	/* Configure GPIO for EHCI port */
	if (omap_mux_init_gpio(GPIO_USB_NRESET, OMAP_PIN_OUTPUT)) {
		pr_err("Can not configure mux for GPIO_USB_NRESET %d\n",
				GPIO_USB_NRESET);
		return;
	}

	if (omap_mux_init_gpio(GPIO_USB_POWER, OMAP_PIN_OUTPUT)) {
		pr_err("Can not configure mux for GPIO_USB_POWER %d\n",
				GPIO_USB_POWER);
		return;
	}

	ret = gpio_request(GPIO_USB_POWER, "usb_ehci_enable");
	if (ret < 0) {
		pr_err("Can not request GPIO %d\n", GPIO_USB_POWER);
		return;
	}

	ret = gpio_direction_output(GPIO_USB_POWER, 1);
	if (ret < 0) {
		gpio_free(GPIO_USB_POWER);
		pr_err("Unable to initialize EHCI power\n");
		return;
	}

	usb_ehci_init(&ehci_pdata);

	/* DSS */
	am3517_crane_display_init();
	/*Ethernet*/
	am3517_crane_ethernet_init(&am3517_crane_emac_pdata);

	/* MUSB */
	am3517_crane_musb_init();

	/* MMC init function */
	omap2_hsmmc_init(mmc);
#ifdef CONFIG_USB_ANDROID
	omap3evm_android_gadget_init();
#endif
}

MACHINE_START(CRANEBOARD, "AM3517/05 CRANEBOARD")
	.boot_params    = 0x80000100,
	.map_io         = omap3_map_io,
	.reserve        = omap_reserve,
	.init_irq       = am3517_crane_init_irq,
	.init_machine   = am3517_crane_init,
	.timer          = &omap_timer,
MACHINE_END
