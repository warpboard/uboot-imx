/*
 * Copyright (C) 2013-2014 Freescale Semiconductor, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * Copyright (C) 2014 O.S. Systems Software LTDA.
 *
 * Author: Otavio Salvador <otavio@ossystems.com.br>
 *
 * Copyright (C) 2014 Kynetics, LLC
 *
 * Backport to 2013.04: Diego Rondini
 *
 * Copyright (C) 2014 Revolution Robotics, Inc.
 *
 * Author: Jacob Postman
 *
 * SPDX-License-Identifier:    GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/io.h>
#include <asm/sizes.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <max77696.h>

// I2C Dependencies
#include <asm/imx-common/mxc_i2c.h>
#include <i2c.h>
#include "warp_bbi2c.h"

#ifdef CONFIG_FASTBOOT
#include <fastboot.h>
#ifdef CONFIG_ANDROID_RECOVERY
#include <recovery.h>
#endif
#endif /*CONFIG_FASTBOOT*/

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS | \
	PAD_CTL_LVE )

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS | \
	PAD_CTL_LVE)

#define I2C_PAD_CTRL   (PAD_CTL_PUS_100K_UP |                  \
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS |   \
	PAD_CTL_ODE | PAD_CTL_SRE_FAST | PAD_CTL_LVE)

#define LCD_PAD_CTRL   ( PAD_CTL_SPEED_HIGH | PAD_CTL_DSE_80ohm |      \
	PAD_CTL_SRE_FAST | PAD_CTL_LVE)

#define BBI2C_PAD_CTRL   (PAD_CTL_PUS_22K_UP |  PAD_CTL_PUE | PAD_CTL_PKE |  \
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_80ohm | PAD_CTL_HYS |   \
	PAD_CTL_ODE | PAD_CTL_SRE_FAST | PAD_CTL_LVE)

#define BBI2C_ADDR0	IMX_GPIO_NR(4, 14) //ECSPI2_MISO
#define BBI2C_ADDR1	IMX_GPIO_NR(4, 15) //ECSPI2_SS0

#define FXOS8700_I2C_ADDR 0x1E

#define BBI2C_WRITE 	0x00
#define BBI2C_READ  	0x01

int dram_init(void)
{
	gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);

	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_UART1_TXD__UART1_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RXD__UART1_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_SD2_CLK__USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_CMD__USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_RST__USDHC2_RST | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT0__USDHC2_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT1__USDHC2_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT2__USDHC2_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT3__USDHC2_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT4__USDHC2_DAT4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT5__USDHC2_DAT5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT6__USDHC2_DAT6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT7__USDHC2_DAT7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

struct i2c_pads_info i2c_pad_info0 = {
       .scl = {
               .i2c_mode =  MX6_PAD_I2C1_SCL__I2C1_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
               .gpio_mode = MX6_PAD_I2C1_SCL__GPIO_3_12 | MUX_PAD_CTRL(I2C_PAD_CTRL),
               .gp = IMX_GPIO_NR(3, 12)
       },
       .sda = {
               .i2c_mode = MX6_PAD_I2C1_SDA__I2C1_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
               .gpio_mode = MX6_PAD_I2C1_SDA__GPIO_3_13 | MUX_PAD_CTRL(I2C_PAD_CTRL),
               .gp = IMX_GPIO_NR(3, 13)
       }
};


#ifdef CONFIG_FSL_ESDHC

#define USDHC2_CD_GPIO	IMX_GPIO_NR(5, 0)

static struct fsl_esdhc_cfg usdhc_cfg[1] = {
	{USDHC2_BASE_ADDR},
};

int mmc_get_env_devno(void)
{
	u32 soc_sbmr = readl(SRC_BASE_ADDR + 0x4);
	u32 dev_no;

	/* BOOT_CFG2[3] and BOOT_CFG2[4] */
	dev_no = (soc_sbmr & 0x00001800) >> 11;

	return dev_no;
}

int board_mmc_getcd(struct mmc *mmc)
{
	return 1;       /* Assume boot SD always present */
}

int board_mmc_init(bd_t *bis)
{
	imx_iomux_v3_setup_multiple_pads(usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
	gpio_direction_input(USDHC2_CD_GPIO);
	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);

	if (fsl_esdhc_initialize(bis, &usdhc_cfg[0]))
		printf("Warning: failed to initialize mmc dev\n");

	return 0;
}

void board_late_mmc_env_init(void)
{
	char cmd[32];
	u32 dev_no = mmc_get_env_devno();

	setenv_ulong("mmcdev", dev_no);

	sprintf(cmd, "mmc dev %d", dev_no);
	run_command(cmd, 0);
}
#endif

static iomux_v3_cfg_t const bbi2c_pads[] = {
	MX6_PAD_ECSPI2_SCLK__GPIO_4_12 | MUX_PAD_CTRL(BBI2C_PAD_CTRL),
	MX6_PAD_ECSPI2_MOSI__GPIO_4_13 | MUX_PAD_CTRL(BBI2C_PAD_CTRL),
	MX6_PAD_ECSPI2_MISO__GPIO_4_14 | MUX_PAD_CTRL(BBI2C_PAD_CTRL),
	MX6_PAD_ECSPI2_SS0__GPIO_4_15 | MUX_PAD_CTRL(BBI2C_PAD_CTRL),
	MX6_PAD_AUD_TXFS__GPIO_1_4  | MUX_PAD_CTRL(BBI2C_PAD_CTRL),
	MX6_PAD_KEY_ROW2__GPIO_3_29  | MUX_PAD_CTRL(BBI2C_PAD_CTRL),
};

static iomux_v3_cfg_t const bbi2c_uncfg_addr[] = {
	MX6_PAD_ECSPI2_MISO__GPIO_4_14,
	MX6_PAD_ECSPI2_SS0__GPIO_4_15,
};

int board_early_init_f(void)
{
	// Configure FXOS8700 addresss pads
	imx_iomux_v3_setup_multiple_pads(bbi2c_pads, ARRAY_SIZE(bbi2c_pads));
	gpio_direction_output(BBI2C_ADDR0, 0);
	gpio_direction_output(BBI2C_ADDR1, 0);

	setup_iomux_uart();
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);

	return 0;
}

/******************************************************
 * On the development rev of WaRP FXOS8700
 * comes up in I2C mode by default. This workaround
 * sets the mode configuration pads as necessary and
 * bit-bangs the FXOS8700 I2C interface to initiate
 * a soft reset. This results in the FXOS8700 coming
 * up in the intended SPI mode which is then available
 * to interface with from the Linux/Android kernel.
 ******************************************************/
void fxos8700_init(void){

	unsigned char ret = 0;

	unsigned char tryAddr = 0;

	BBI2C_Init();

	// SEND RESET SEQUENCE ****************************
	// Send I2C Start Sequence
	imx_iomux_v3_setup_multiple_pads(bbi2c_uncfg_addr, ARRAY_SIZE(bbi2c_uncfg_addr));
	gpio_direction_input(BBI2C_ADDR0);
	gpio_direction_input(BBI2C_ADDR1);

	BBI2C_Start();

	ret = BBI2C_TransmitByte((FXOS8700_I2C_ADDR << 1) | (BBI2C_WRITE));
	if(ret){
		printf("Failed sending device address to FXOS\n");
		goto stop_seq;
	}
	ret = BBI2C_TransmitByte(0x2B);
	if(ret){
		printf("Failed sending register address to FXOS\n");
		goto stop_seq;
	}

	ret = BBI2C_TransmitByte(0x40);
	if(ret){
		// Doesn't actually fail - device just resets immediately.
		printf("Failed sending data to FXOS\n");
	}
	stop_seq:
		BBI2C_Stop();

	// END RESET SEQUENCE ***********************

}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	// Set manual reset button hold time to 2 seconds (min value)
	i2c_reg_write(MAX77696_PMIC_ADDR, GLBLCNFG1, 0x22);

	// Initialize the FXOS8700 into SPI mode using I2C Bitbang
	fxos8700_init();

	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	// 8 bit bus width
	{"sd2",	 MAKE_CFGVAL(0x40, 0x2A, 0x00, 0x00)},
	{"emmc8bitddr", MAKE_CFGVAL(0x60, 0xCA, 0x00, 0x00)},
	{"emmc8bit", MAKE_CFGVAL(0x60, 0x4A, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
	int ret = 0;

#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

	return 0;
}

u32 get_board_rev(void)
{
	return get_cpu_rev();
}

#ifdef CONFIG_FASTBOOT

void board_fastboot_setup(void)
{
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	if (!getenv("fastboot_dev"))
		setenv("fastboot_dev", "mmc0");
	if (!getenv("bootcmd"))
		setenv("bootcmd", "booti mmc0");
#else
	printf("unsupported boot devices\n");
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
}

#ifdef CONFIG_ANDROID_RECOVERY
int check_recovery_cmd_file(void)
{
    return recovery_check_and_clean_flag();
}

void board_recovery_setup(void)
{
	int bootdev = get_boot_device();

	/*current uboot BSP only supports USDHC2*/
	switch (bootdev) {
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD1_BOOT:
	case MMC1_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
					"booti mmc0 recovery");
		break;
	case SD2_BOOT:
	case MMC2_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
					"booti mmc1 recovery");
		break;
	case SD3_BOOT:
	case MMC3_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
					"booti mmc2 recovery");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
	default:
		printf("Unsupported bootup device for recovery: dev: %d\n",
			bootdev);
		return;
	}

	printf("setup env for recovery..\n");
	setenv("bootcmd", "run bootcmd_android_recovery");
}

#endif /*CONFIG_ANDROID_RECOVERY*/

#endif /*CONFIG_FASTBOOT*/

int checkboard(void)
{
	puts("Board: WaRP Board\n");

	return 0;
}

#ifdef CONFIG_IMX_UDC
iomux_v3_cfg_t const otg_udc_pads[] = {
	(MX6_PAD_EPDC_PWRCOM__ANATOP_USBOTG1_ID | MUX_PAD_CTRL(NO_PAD_CTRL)),
};
void udc_pins_setting(void)
{
	imx_iomux_v3_setup_multiple_pads(otg_udc_pads,
			ARRAY_SIZE(otg_udc_pads));
}
#endif /*CONFIG_IMX_UDC*/

#ifdef CONFIG_USB_EHCI_MX6
iomux_v3_cfg_t const usb_otg1_pads[] = {
	MX6_PAD_KEY_COL4__USB_USBOTG1_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_EPDC_PWRCOM__ANATOP_USBOTG1_ID | MUX_PAD_CTRL(NO_PAD_CTRL)
};

iomux_v3_cfg_t const usb_otg2_pads[] = {
	MX6_PAD_KEY_COL5__USB_USBOTG2_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
};

int board_ehci_hcd_init(int port)
{
	switch (port) {
	case 0:
		imx_iomux_v3_setup_multiple_pads(usb_otg1_pads,
			ARRAY_SIZE(usb_otg1_pads));
		break;
	case 1:
		imx_iomux_v3_setup_multiple_pads(usb_otg2_pads,
			ARRAY_SIZE(usb_otg2_pads));
		break;
	default:
		printf("MXC USB port %d not yet supported\n", port);
		return 1;
	}
	return 0;
}
#endif
