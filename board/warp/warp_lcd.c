/*
 * WaRP MX6SL ELCDIF/SSD2805 MIPI bridge driver
 *
 * Copyright (C) 2014 Revolution Robotics, Inc.
 *
 * Author: Jacob Postman <jacob@revolution-robotics.com>
 *
 * Portions of this work are derived from SSD2805 MIPI controller driver
 *
 * Copyright (C) 2011 Si14 SpA
 *
 * Author: Luca Burelli <luca.burelli@si14.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/gpio.h>
#include <asm/arch/mx6-pins.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/io.h>
#include <asm/errno.h>

#include "warp_common.h"
#include "warp_lcd.h"
#include "ssd2805.h"
#include "lh154.h"
#include "splash.hex"

//#define DEBUG_SSD2805

#  define LCDIF_SLOW_FREQDIV	250
#  define LCDIF_FAST_FREQDIV	20
#  define LCDIF_BUS_WIDTH	8
#  define SSD2805_PLL_CLK_FREQ	200000000

static int pll_configured = 0;

// Avoid nested calls to request_irq
static int irq_requested = 1;

// Delay count before notifying the panel is enabled
static int panel_init_delay = 2;

#define CMD_ONLY(c) { cmd: c, data: NULL, len: 0 }
#define CMD_ENTRY(c, d) { cmd: c, data: d, len: sizeof(d)-1 }
#define DELAY_ENTRY(t) { cmd: 0, data: NULL, len: -t }
static const struct {
	uint8_t cmd;
	char *data;
	int len;
} PANEL_INIT[] = {
	// [DCS] Sleep Out
	CMD_ONLY(0x11),
	DELAY_ENTRY(200),
	// [DCS] MADCTL - set RGB (not BGR) and rotation
	CMD_ENTRY(0x36, "\x68"),
	// [DCS] COLMOD - set 24bit color depth
	CMD_ENTRY(0x3a, "\x07"),
	// [DCS] Display On
	CMD_ONLY(0x29),
	DELAY_ENTRY(0),
	//CMD_ENTRY(0xbe, "\x//e0\x//01"), PSCR3 := 0x1e0
};

static inline void mxc_lcdif_setdiv_system(int div) {
	__raw_writel(BF_LCDIF_TIMING_CMD_HOLD(div) |
				 BF_LCDIF_TIMING_CMD_SETUP(div) |
				 BF_LCDIF_TIMING_DATA_HOLD(div) |
				 BF_LCDIF_TIMING_DATA_SETUP(div),
				 REGS_LCDIF_BASE + HW_LCDIF_TIMING);
}

static char display_buffer[240*240*3];

#ifdef DEBUG_SSD2805
static void __maybe_unused ssd2805_dump_registers(void)
{
#define SSD2805_DUMP_REG(reg, desc)	printf("%7s [%02x]: %04x | %s\n", #reg, SSD2805_REG_ ## reg, ssd2805_read_reg(SSD2805_REG_ ## reg), desc)
	SSD2805_DUMP_REG(DIR,	"Device Identification Register");
	SSD2805_DUMP_REG(VICR1,	"RGB Interface Control Register 1");
	SSD2805_DUMP_REG(VICR2,	"RGB Interface Control Register 2");
	SSD2805_DUMP_REG(VICR3,	"RGB Interface Control Register 3");
	SSD2805_DUMP_REG(VICR4,	"RGB Interface Control Register 4");
	SSD2805_DUMP_REG(VICR5,	"RGB Interface Control Register 5");
	SSD2805_DUMP_REG(VICR6,	"RGB Interface Control Register 6");
	SSD2805_DUMP_REG(CFGR,	"Configuration Register");
	SSD2805_DUMP_REG(VCR,	"VC Control Register");
	SSD2805_DUMP_REG(PCR,	"PLL Control Register");
	SSD2805_DUMP_REG(PLCR,	"PLL Configuration Register");
	SSD2805_DUMP_REG(CCR,	"Clock Control Register");
	SSD2805_DUMP_REG(PSCR1,	"Packet Size Control Register 1");
	SSD2805_DUMP_REG(PSCR2,	"Packet Size Control Register 2");
	SSD2805_DUMP_REG(PSCR3,	"Packet Size Control Register 3");
	SSD2805_DUMP_REG(GPDR,	"Generic Packet Drop Register");
	SSD2805_DUMP_REG(OCR,	"Operation Control Register");
	SSD2805_DUMP_REG(MRSR,	"Maximum Return Size Register");
	SSD2805_DUMP_REG(RDCR,	"Return Data Count Register");
	SSD2805_DUMP_REG(ARSR,	"ACK Response Register");
	SSD2805_DUMP_REG(LCR,	"Line Control Register");
	SSD2805_DUMP_REG(ICR,	"Interrupt Control Register");
	SSD2805_DUMP_REG(ISR,	"Interrupt Status Register");
	SSD2805_DUMP_REG(ESR,	"Error Status Register");
	SSD2805_DUMP_REG(DAR1,	"Delay Adjustment Register 1");
	SSD2805_DUMP_REG(DAR2,	"Delay Adjustment Register 2");
	SSD2805_DUMP_REG(DAR3,	"Delay Adjustment Register 3");
	SSD2805_DUMP_REG(DAR4,	"Delay Adjustment Register 4");
	SSD2805_DUMP_REG(DAR5,	"Delay Adjustment Register 5");
	SSD2805_DUMP_REG(DAR6,	"Delay Adjustment Register 6");
	SSD2805_DUMP_REG(HTTR1,	"HS TX Timer Register 1");
	SSD2805_DUMP_REG(HTTR2,	"HS TX Timer Register 2");
	SSD2805_DUMP_REG(LRTR1,	"LP RX Timer Register 1");
	SSD2805_DUMP_REG(LRTR2,	"LP RX Timer Register 2");
	SSD2805_DUMP_REG(TSR,	"TE Status Register");
	SSD2805_DUMP_REG(LRR,	"SPI Read Register");
	SSD2805_DUMP_REG(TR,	"Test Register");
	SSD2805_DUMP_REG(RR,	"Read Register");
#undef	SSD2805_DUMP_REG
}
#else
static void ssd2805_dump_registers(void) { }
#endif

// Adapted from mxs_init_lcdif() in lcdif.c for mxs in ImWatch kernel.
void lcdif_init(void){
	//int reg = 0;
	__raw_writel(BM_LCDIF_CTRL_CLKGATE, REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);

	/* Reset controller */
	__raw_writel(BM_LCDIF_CTRL_SFTRST, REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);

	/* Take controller out of reset */
	__raw_writel(BM_LCDIF_CTRL_SFTRST | BM_LCDIF_CTRL_CLKGATE, REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);

	/* Setup the bus protocol */
	__raw_writel(BM_LCDIF_CTRL1_MODE86, REGS_LCDIF_BASE + HW_LCDIF_CTRL1_CLR);
	mxc_lcdif_setdiv_system(LCDIF_SLOW_FREQDIV);
}

static void setup_system_panel(u16 h_active, u16 v_active, u8 bus_width)
{
	/* To prevent the LCDIF from mangling the pixel contents, we have to
	 * lie to the hardware by setting pixel width == bus width (8/16 bits),
	 * so set RGB565 mode and horizontal pixel count to h_active*3 for 8
	 * bits, or 3/2 for 16 bits.
	 */

	u32 val;
	u16 bw;
	u16 h_active_hw;
	switch (bus_width) {
	case 8:
		bw = BV_LCDIF_CTRL_LCD_DATABUS_WIDTH__8_BIT;
		h_active_hw = h_active * 3;
		break;

	case 16:
		bw = BV_LCDIF_CTRL_LCD_DATABUS_WIDTH__16_BIT;
		h_active_hw = h_active * 24/16;
		break;

	default:
		panic("Unknown lcdif bus width specified");
	}

	__raw_writel(BM_LCDIF_CTRL1_BYTE_PACKING_FORMAT,
				 REGS_LCDIF_BASE + HW_LCDIF_CTRL1_CLR);
	__raw_writel(BF_LCDIF_CTRL1_BYTE_PACKING_FORMAT(0xF),
				 REGS_LCDIF_BASE + HW_LCDIF_CTRL1_SET);

	val = BF_LCDIF_TRANSFER_COUNT_H_COUNT(h_active_hw) |
	BF_LCDIF_TRANSFER_COUNT_V_COUNT(v_active);
	__raw_writel(val, REGS_LCDIF_BASE + HW_LCDIF_TRANSFER_COUNT);

	// Set lcdif to SYSTEM mode
	__raw_writel(BM_LCDIF_CTRL_DVI_MODE,
				 REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
	__raw_writel(BM_LCDIF_CTRL_VSYNC_MODE,
				 REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
	__raw_writel(BM_LCDIF_CTRL_DOTCLK_MODE,
				 REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);

	__raw_writel(BM_LCDIF_CTRL_WORD_LENGTH |
				 BM_LCDIF_CTRL_INPUT_DATA_SWIZZLE |
				 BM_LCDIF_CTRL_CSC_DATA_SWIZZLE |
				 BM_LCDIF_CTRL_DATA_FORMAT_16_BIT | // force RGB565 in case of 16bit
				 BM_LCDIF_CTRL_DATA_SELECT | // command mode
				 BM_LCDIF_CTRL_LCD_DATABUS_WIDTH,
				 REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
	__raw_writel(BF_LCDIF_CTRL_WORD_LENGTH(bw) |	// 8/16 bits
				 BM_LCDIF_CTRL_DATA_SELECT |	// data mode
				 BF_LCDIF_CTRL_INPUT_DATA_SWIZZLE(0) |	// no swap
				 BF_LCDIF_CTRL_CSC_DATA_SWIZZLE(0) |	// no swap
				 BF_LCDIF_CTRL_LCD_DATABUS_WIDTH(bw),	// 8/16 bits
				 REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);


	// Disabled ELCDIF interrupts
	__raw_writel(BM_LCDIF_CTRL1_OVERFLOW_IRQ_EN |
				 BM_LCDIF_CTRL1_UNDERFLOW_IRQ_EN |
				 BM_LCDIF_CTRL1_CUR_FRAME_DONE_EN |
				 BM_LCDIF_CTRL1_VSYNC_EDGE_IRQ_EN,
				 REGS_LCDIF_BASE + HW_LCDIF_CTRL1_CLR);

	// LCDIF_SYSTEM
	__raw_writel(BM_LCDIF_CTRL_BYPASS_COUNT,
				 REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
}

static void lcdif_write(void *ptr, unsigned len, int is_data)
{
	uint8_t *buf = ptr;

	while(__raw_readl(REGS_LCDIF_BASE + HW_LCDIF_CTRL) & BM_LCDIF_CTRL_RUN);

	if (pll_configured)
		mxc_lcdif_setdiv_system(LCDIF_FAST_FREQDIV);
	else
		mxc_lcdif_setdiv_system(LCDIF_SLOW_FREQDIV);

	__raw_writel(BM_LCDIF_CTRL_LCDIF_MASTER, REGS_LCDIF_BASE + HW_LCDIF_CTRL_CLR);
	udelay(10);
	gpio_direction_output(PINID_LCD_RS, is_data?1:0); 	// LCD_DCX / LCD_RS
	while (len > 0)
	{
		unsigned burst, count;

#if (LCDIF_BUS_WIDTH == 8)
		count = burst = min(len, 0xffffu);
#elif (LCDIF_BUS_WIDTH == 16)
		burst = min(len, 0x1fffeu);
		count = (burst+1) / 2;
#endif
		len -= burst;

		__raw_writel(BF_LCDIF_TRANSFER_COUNT_V_COUNT(1) | BF_LCDIF_TRANSFER_COUNT_H_COUNT(1),
					 REGS_LCDIF_BASE + HW_LCDIF_TRANSFER_COUNT);
		do {
			// make sure the LFIFO is not full
			while (__raw_readl(REGS_LCDIF_BASE + HW_LCDIF_STAT) & BM_LCDIF_STAT_LFIFO_FULL);
			while (__raw_readl(REGS_LCDIF_BASE + HW_LCDIF_CTRL) & BM_LCDIF_CTRL_RUN);

			__raw_writel(BM_LCDIF_CTRL_RUN, REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);
#if (LCDIF_BUS_WIDTH == 8)
			// write 8 bits
			__raw_writeb(*(uint8_t*)buf, REGS_LCDIF_BASE + HW_LCDIF_DATA);
			buf += 1;
#elif (LCDIF_BUS_WIDTH == 16)
			// write 16 bits anyway, MIPI hardware will discard higher 8
			__raw_writew(*(uint16_t*)buf, REGS_LCDIF_BASE + HW_LCDIF_DATA);
			buf += 2;
#endif
		} while(--count > 0);

		while (__raw_readl(REGS_LCDIF_BASE + HW_LCDIF_CTRL) & BM_LCDIF_CTRL_RUN);
	}
}

void set_lcd_pads(void){
	// Configure iMX6SL pads
	imx_iomux_v3_setup_multiple_pads(mcu8080display_pads,ARRAY_SIZE(mcu8080display_pads));
	gpio_direction_input(PINID_MIPI_TE); 		// MIPI_TE
	gpio_direction_output(PINID_LCD_RSTN, 1);  	// LCD_RSTn
	gpio_direction_input(PINID_LCD_INTN); 		// LCD_INTn
	gpio_direction_input(PINID_MIPI_BSYNC);  	// MIPI_B_SYNC
	gpio_direction_output(PINID_MIPI_RSTN, 1); 	// MIPI_RSTn
	gpio_direction_output(PINID_LCD_RD, 1); 	// LCD_RDX
	gpio_direction_output(PINID_LCD_RS, 1); 	// LCD_DCX(ssd)/LCD_RS(mx6)
}

static void set_lcd_dat_gpio_out(void){
	gpio_direction_output(PINID_LCD_DAT0,1); 	// LCD_DAT0
	gpio_direction_output(PINID_LCD_DAT1,1); 	// LCD_DAT1
	gpio_direction_output(PINID_LCD_DAT2,1); 	// LCD_DAT2
	gpio_direction_output(PINID_LCD_DAT3,1); 	// LCD_DAT3
	gpio_direction_output(PINID_LCD_DAT4,1); 	// LCD_DAT4
	gpio_direction_output(PINID_LCD_DAT5,1); 	// LCD_DAT5
	gpio_direction_output(PINID_LCD_DAT6,1); 	// LCD_DAT6
	gpio_direction_output(PINID_LCD_DAT7,1); 	// LCD_DAT7
#if (LCDIF_BUS_WIDTH == 16)
	gpio_direction_output(PINID_LCD_DAT8,1); 	// LCD_DAT8
	gpio_direction_output(PINID_LCD_DAT9,1); 	// LCD_DAT9
	gpio_direction_output(PINID_LCD_DAT10,1); 	// LCD_DAT10
	gpio_direction_output(PINID_LCD_DAT11,1); 	// LCD_DAT11
	gpio_direction_output(PINID_LCD_DAT12,1); 	// LCD_DAT12
	gpio_direction_output(PINID_LCD_DAT13,1); 	// LCD_DAT13
	gpio_direction_output(PINID_LCD_DAT14,1); 	// LCD_DAT14
	gpio_direction_output(PINID_LCD_DAT15,1); 	// LCD_DAT15
#endif
	gpio_direction_output(PINID_LCD_CS,1); 		// LCD_CS
	gpio_direction_output(PINID_LCD_RS,1); 		// LCD_RS
	gpio_direction_output(PINID_LCD_WR,1); 		// LCD_WR
	gpio_direction_output(PINID_LCD_RD,1); 		// LCD_RD
}

static void set_lcd_dat_gpio_in(void){
	gpio_direction_input(PINID_LCD_DAT0); 	// LCD_DAT0
	gpio_direction_input(PINID_LCD_DAT1); 	// LCD_DAT1
	gpio_direction_input(PINID_LCD_DAT2); 	// LCD_DAT2
	gpio_direction_input(PINID_LCD_DAT3); 	// LCD_DAT3
	gpio_direction_input(PINID_LCD_DAT4); 	// LCD_DAT4
	gpio_direction_input(PINID_LCD_DAT5); 	// LCD_DAT5
	gpio_direction_input(PINID_LCD_DAT6); 	// LCD_DAT6
	gpio_direction_input(PINID_LCD_DAT7); 	// LCD_DAT7
#if (LCDIF_BUS_WIDTH == 16)
	gpio_direction_input(PINID_LCD_DAT8); 	// LCD_DAT8
	gpio_direction_input(PINID_LCD_DAT9); 	// LCD_DAT9
	gpio_direction_input(PINID_LCD_DAT10); 	// LCD_DAT10
	gpio_direction_input(PINID_LCD_DAT11); 	// LCD_DAT11
	gpio_direction_input(PINID_LCD_DAT12); 	// LCD_DAT12
	gpio_direction_input(PINID_LCD_DAT13); 	// LCD_DAT13
	gpio_direction_input(PINID_LCD_DAT14); 	// LCD_DAT14
	gpio_direction_input(PINID_LCD_DAT15); 	// LCD_DAT15
#endif
}

static iomux_v3_cfg_t const mcu8080display_gpio_pads[] = {
	MX6_PAD_LCD_DAT0__GPIO_2_20 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DAT1__GPIO_2_21 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DAT2__GPIO_2_22 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DAT3__GPIO_2_23 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DAT4__GPIO_2_24 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DAT5__GPIO_2_25 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DAT6__GPIO_2_26 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DAT7__GPIO_2_27 | MUX_PAD_CTRL(LCD_PAD_CTRL),
#if (LCDIF_BUS_WIDTH == 16)
	MX6_PAD_LCD_DAT8__GPIO_2_28 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DAT9__GPIO_2_29 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DAT10__GPIO_2_30 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DAT11__GPIO_2_31 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DAT12__GPIO_3_0 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DAT13__GPIO_3_1 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DAT14__GPIO_3_2 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_LCD_DAT15__GPIO_3_3 | MUX_PAD_CTRL(LCD_PAD_CTRL),
#endif
	MX6_PAD_LCD_DAT23__GPIO_3_11 | MUX_PAD_CTRL(LCD_PAD_CTRL),	// LCD_RDX(SSD)/LCD_RD_E(MX6_Signal)
	MX6_PAD_LCD_CLK__GPIO_2_15 | MUX_PAD_CTRL(LCD_PAD_CTRL),	// LCD_WRX(SSD)/LCD_WR_RWn(MX6_Signal)/
	MX6_PAD_LCD_HSYNC__GPIO_2_17 | MUX_PAD_CTRL(LCD_PAD_CTRL), 	// LCD_CSX(SSD)/LCD_CS(MX6_Signal)
	MX6_PAD_LCD_RESET__GPIO_2_19 | MUX_PAD_CTRL(LCD_PAD_CTRL), 	// LCD_DCX(SSD) LCD_RS(IMX)
	//	MX6_PAD_AUD_TXFS__PWM3_PWMO | MUX_PAD_CTRL(LCD_PAD_CTRL),	// temp pwm output test to pad
	//	MX6_PAD_AUD_TXFS__GPIO_1_4 | MUX_PAD_CTRL(LCD_PAD_CTRL),
	MX6_PAD_AUD_MCLK__PWM4_PWMO | MUX_PAD_CTRL(LCD_PAD_CTRL),	// LCD_CLK
	MX6_PAD_LCD_DAT16__GPIO_3_4 | MUX_PAD_CTRL(LCD_PAD_CTRL),	// MIPI_TE
	MX6_PAD_LCD_DAT19__GPIO_3_7 | MUX_PAD_CTRL(LCD_PAD_CTRL),	// LCD_RSTn
	MX6_PAD_LCD_DAT20__GPIO_3_8 | MUX_PAD_CTRL(LCD_PAD_CTRL),	// LCD_INTn
	MX6_PAD_LCD_DAT21__GPIO_3_9 | MUX_PAD_CTRL(LCD_PAD_CTRL),	// MIPI_B_SYNC
	MX6_PAD_LCD_DAT22__GPIO_3_10 | MUX_PAD_CTRL(LCD_PAD_CTRL),	// MIPI_RSTn
};

static void set_lcd_pads_gpio(void){
	// Configure iMX6SL pads
	imx_iomux_v3_setup_multiple_pads(mcu8080display_gpio_pads,ARRAY_SIZE(mcu8080display_gpio_pads));
	gpio_direction_input(PINID_LCD_DAT0); 	// LCD_DAT0
	gpio_direction_input(PINID_LCD_DAT1); 	// LCD_DAT1
	gpio_direction_input(PINID_LCD_DAT2); 	// LCD_DAT2
	gpio_direction_input(PINID_LCD_DAT3); 	// LCD_DAT3
	gpio_direction_input(PINID_LCD_DAT4); 	// LCD_DAT4
	gpio_direction_input(PINID_LCD_DAT5); 	// LCD_DAT5
	gpio_direction_input(PINID_LCD_DAT6); 	// LCD_DAT6
	gpio_direction_input(PINID_LCD_DAT7); 	// LCD_DAT7
#if (LCDIF_BUS_WIDTH == 16)
	gpio_direction_input(PINID_LCD_DAT8); 	// LCD_DAT8
	gpio_direction_input(PINID_LCD_DAT9); 	// LCD_DAT9
	gpio_direction_input(PINID_LCD_DAT10); 	// LCD_DAT10
	gpio_direction_input(PINID_LCD_DAT11); 	// LCD_DAT11
	gpio_direction_input(PINID_LCD_DAT12); 	// LCD_DAT12
	gpio_direction_input(PINID_LCD_DAT13); 	// LCD_DAT13
	gpio_direction_input(PINID_LCD_DAT14); 	// LCD_DAT14
	gpio_direction_input(PINID_LCD_DAT15); 	// LCD_DAT15
#endif
	gpio_direction_input(PINID_MIPI_TE); 		// MIPI_TE
	gpio_direction_output(PINID_LCD_RSTN, 1);  	// LCD_RSTn
	gpio_direction_input(PINID_LCD_INTN); 		// LCD_INTn
	gpio_direction_input(PINID_MIPI_BSYNC);  	// MIPI_B_SYNC
	gpio_direction_output(PINID_MIPI_RSTN	, 1); 	// MIPI_RSTn
	gpio_direction_output(PINID_LCD_RD, 1); 	// LCD_RDX
	gpio_direction_output(PINID_LCD_RS, 1); 	// LCD_DCX
}

static int lcdif_read(u16 reg, u16 *rbuf, int rlen)
{
	unsigned int currGPIO = 0;
	while(__raw_readl(REGS_LCDIF_BASE + HW_LCDIF_CTRL) & BM_LCDIF_CTRL_RUN);
	// set pins as GPIO outputs, all ones
	set_lcd_pads_gpio();
	set_lcd_dat_gpio_out();

	// write command cycle
	udelay(1000);
	gpio_direction_output(PINID_LCD_RS, 0);
	udelay(1000);
	gpio_direction_output(PINID_LCD_CS, 0);
	udelay(1000);
#if (LCDIF_BUS_WIDTH == 8)
	currGPIO =  __raw_readl(GPIO2_BASE_ADDR);
	currGPIO = (reg << 20) | (currGPIO & 0xf00fffff); 	// LCD_DAT[11:0] go to GPIO2[31:20]
	__raw_writel(currGPIO, GPIO2_BASE_ADDR);
#elif (LCDIF_BUS_WIDTH == 16)
	currGPIO =  readl(GPIO3_BASE_ADDR);
	currGPIO = (reg >> 12) | (currGPIO & 0xfffffff0); 	// LCD_DAT[15:12] go to GPIO3[3:0]
	__raw_writel(currGPIO, GPIO3_BASE_ADDR);

	currGPIO =  readl(GPIO2_BASE_ADDR);
	currGPIO = (reg << 20) | (currGPIO & 0x000fffff); 	// LCD_DAT[11:0] go to GPIO2[31:20]
	__raw_writel(currGPIO, GPIO2_BASE_ADDR);
#endif
	gpio_direction_output(PINID_LCD_WR, 0);
	udelay(1000);
	gpio_direction_output(PINID_LCD_WR, 1);
	udelay(1000);

	// switch data bus to read mode
	set_lcd_dat_gpio_in();

	// read data cycle(s)
	gpio_direction_output(PINID_LCD_RS, 1);
	udelay(1000);
	while (rlen > 0) {
#if (LCDIF_BUS_WIDTH == 8)
		gpio_direction_output(PINID_LCD_RD, 0);
		udelay(1000);

		currGPIO =  __raw_readl(GPIO2_BASE_ADDR);
		reg = ((currGPIO >> 20) & 0x000000ff); 	// LCD_DAT[11:0] come from GPIO2[31:20]

		gpio_direction_output(PINID_LCD_RD, 1);
		udelay(1000);
		gpio_direction_output(PINID_LCD_RD, 0);
		udelay(1000);

		currGPIO =  __raw_readl(GPIO2_BASE_ADDR);
		reg |= ((currGPIO >> 12) & 0x0000ff00); 	// LCD_DAT[11:0] come from GPIO2[31:20]
#elif (LCDIF_BUS_WIDTH == 16)
		gpio_direction_output(PINID_LCD_RD, 0);
		udelay(1000);
		currGPIO =  __raw_readl(GPIO3_BASE_ADDR);
		reg = ((currGPIO << 12) & 0x0000f000); 	// LCD_DAT[15:12] go to GPIO3[3:0]

		currGPIO =  __raw_readl(GPIO2_BASE_ADDR);
		reg |= ((currGPIO >> 20) & 0x00000fff); 	// LCD_DAT[11:0] come from GPIO2[31:20]
#endif
		gpio_direction_output(PINID_LCD_RD, 1);
		udelay(1000);
		rlen -= 2;
		*rbuf++ = (u16) reg;
	}

	// set pins as everyone expects, also releases CS and RS
	set_lcd_dat_gpio_out();

	// set pads for LCD MCU 8080 interface
	set_lcd_pads();

	return 0;
}

int ssd2805_write_reg(int reg, int value)
{
	lcdif_write(&reg, 1, false);
	lcdif_write(&value, 2, true);
	return 0;
}
int ssd2805_read_reg(int reg)
{
	u16 value=0;
	int ret=0;
	ret = lcdif_read((u16) reg, &value, 2);
	if (ret < 0)
		return ret;
	return value;
}

int ssd2805_command(int cmd)
{
	lcdif_write(&cmd, 1, false);
	return 0;
}

int ssd2805_setup_pll(int pll_freq)
{
	/* fPRE = fREF / N, fPRE > 5MHz
	 * fVCO = fPRE * M, 500 > fVCO > 225MHz
	 * fOUT = fVCO / P
	 */
	int N, M, P;
	int N_best, M_best, P_best, error;

	/* disable PLL */
	pll_configured = 0;
	ssd2805_write_reg(SSD2805_REG_PCR, 0);
	if (!pll_freq) return 0;

	error = pll_freq;
	N_best = M_best = P_best = -1;

	for (N=1; N<=16; ++N) {
		int M_min, M_max;
		int fPRE = MIPI_CLK_FREQ/N;
		if (fPRE < 5000000) continue;

		M_min=(225000000+fPRE-1)/fPRE;	M_min=max(2, min(256, M_min));
		M_max=(500000000)/fPRE;		M_max=max(2, min(256, M_max));
		for (M=M_min; M<M_max; ++M) {
			int fVCO = fPRE * M;
			int P_opt = fVCO/pll_freq;
			int P_min = max(P_opt-1, 1);
			int P_max = min(P_opt+1, 16);

			for (P=P_min; P<=P_max; ++P) {
				int fOUT = fVCO / P;
				if (abs(fOUT-pll_freq) < error) {
					error = abs(fOUT-pll_freq);
					N_best = N;
					M_best = M;
					P_best = P;
					if (error == 0) goto found;
				}
			}
		}
	}

	if (N_best < 0)
		return -EINVAL;

found:
	//printf("Setting up PLL for %iM->%iM with error %ik (N=%i, M=%i, P=%i)\n", MIPI_CLK_FREQ/1000000, pll_freq/1000000, error/1000, N_best, M_best, P_best);

	/* update values and enable PLL */
	ssd2805_write_reg(SSD2805_REG_PLCR, SSD2805_PLCR_DIV(N_best-1) | SSD2805_PLCR_MUL(M_best-1) | SSD2805_PLCR_PDIV(P_best-1));
	ssd2805_write_reg(SSD2805_REG_PCR, SSD2805_PCR_PEN);

	// Wait for PLL_LOCK bit; this requires readback from MIPI
	error = 1000;
	do {
		P = ssd2805_read_reg(SSD2805_REG_ISR);
		if (P<0) {
			// read status failed, assume OK after delay
			mdelay(50);
			break;
		}
		--error;
	} while (!(P & SSD2805_ISR_PLS) && error);
	if (!(P & SSD2805_ISR_PLS)) {
		printf("SSD2805 failed to lock PLL at %i, sreg %04x", pll_freq, P);
		return -EIO;
	}
	pll_configured = 1;
	return 0;
}

int ssd2805_setup_commands(int pll_freq)
{
	int r;

	r = ssd2805_setup_pll(pll_freq);
	if (r < 0) return r;

	/* disable all virtual circuit IDs */
	ssd2805_write_reg(SSD2805_REG_CCR, 0x0005);
	ssd2805_write_reg(SSD2805_REG_VCR, 0);

	/* setup initial CFGR */
	ssd2805_write_reg(SSD2805_REG_CFGR, SSD2805_CFGR_EOT | SSD2805_CFGR_DCS | SSD2805_CFGR_HCLK);

	return 0;
}

int ssd2805_setup_host(void)
{
	// Enable high speed (HS) communication
	ssd2805_write_reg(SSD2805_REG_CFGR, SSD2805_CFGR_EOT | SSD2805_CFGR_DCS | SSD2805_CFGR_CKE | SSD2805_CFGR_HS);

	// Biggest possible (optimal) transfer size
	ssd2805_write_reg(SSD2805_REG_PSCR3, 0x400);

	// Set a low HS TX timer value to fix MIPI transmission problems.
	// Without this, some packets are completely discarded, resulting in
	// partial image updates.
	ssd2805_write_reg(SSD2805_REG_HTTR1, 0x000f);
	ssd2805_write_reg(SSD2805_REG_HTTR2, 0x0);

	// Use PO IRQ as BUSY indication.
	ssd2805_write_reg(SSD2805_REG_ICR, SSD2805_ICR_POE);

	// Use RGB, instead of BGR
	ssd2805_write_reg(SSD2805_REG_TR, 0x5);

	return 0;
}

int mipi_dcs_command(int cmd)
{
	// TDC is OUTGOING DATA payload excluding the command
	ssd2805_write_reg(SSD2805_REG_PSCR1, SSD2805_PSCR1_TDCL(0));
	ssd2805_write_reg(SSD2805_REG_PSCR2, SSD2805_PSCR2_TDCH(0));

	return ssd2805_command(cmd);
}

int mipi_dcs_read(int cmd, u8 *rbuf, int rlen)
{
	return lcdif_read((u16)cmd, (u16*) rbuf, rlen);
}

int mipi_dcs_write(int cmd, char *wbuf, int wlen)
{
	uint16_t value;

	ssd2805_write_reg(SSD2805_REG_PSCR1, SSD2805_PSCR1_TDCL(wlen & 0xffff));
	ssd2805_write_reg(SSD2805_REG_PSCR2, SSD2805_PSCR2_TDCH(wlen >> 16));

	// Make sure REN is not set
	value = ssd2805_read_reg(SSD2805_REG_CFGR);
	ssd2805_write_reg(SSD2805_REG_CFGR, value & ~SSD2805_CFGR_REN);

	// Send command
	lcdif_write(&cmd, 1, false);

	// Send data
	lcdif_write(wbuf, wlen, true);
	return 0;
}

int lh154_init_panel(void)
{
	int ret = 0;
	int i;

	setup_system_panel(240, 240, LCDIF_BUS_WIDTH);
	printf("Configuring display bridge\n");
	ret = ssd2805_setup_commands(SSD2805_PLL_CLK_FREQ);
	ssd2805_setup_host();
	if (ret) goto out1;

	irq_requested = 1;
	panel_init_delay = 2;

	ssd2805_dump_registers();

	// Send commands per init sequence
	i=0;
	while (PANEL_INIT[i].cmd || PANEL_INIT[i].len) {
		if (PANEL_INIT[i].len < 0){
			mdelay(-PANEL_INIT[i].len);
		}
		else if (PANEL_INIT[i].cmd && !PANEL_INIT[i].len){
			mipi_dcs_command(PANEL_INIT[i].cmd);
		}
		else{
			mipi_dcs_write(PANEL_INIT[i].cmd, PANEL_INIT[i].data, PANEL_INIT[i].len);
		}
		++i;
	}

	mipi_dcs_command(LH154_CMD_DISPON);

	// BEGIN BARE METAL SPLASH SCREEN UPDATE
	// Copy splash image to simple frame buffer array
	memcpy(display_buffer, splash_image.pixel_data, sizeof(display_buffer));

	// Blank out simple frame buffer array
	//memset(display_buffer, 0, sizeof(display_buffer));

	__raw_writel(&display_buffer, REGS_LCDIF_BASE + HW_LCDIF_CUR_BUF);
	__raw_writel(&display_buffer, REGS_LCDIF_BASE + HW_LCDIF_NEXT_BUF);

	// performing: lh154_update_panel
	ssd2805_write_reg(SSD2805_REG_PSCR1, SSD2805_PSCR1_TDCL(sizeof(display_buffer) & 0xffff));
	ssd2805_write_reg(SSD2805_REG_PSCR2, SSD2805_PSCR2_TDCH(sizeof(display_buffer) >> 16));

	ssd2805_command(LH154_CMD_RAMWR);

#if (LCDIF_BUS_WIDTH == 8)
	__raw_writel(BF_LCDIF_TRANSFER_COUNT_H_COUNT(240*3) |
				 BF_LCDIF_TRANSFER_COUNT_V_COUNT(240), REGS_LCDIF_BASE + HW_LCDIF_TRANSFER_COUNT);
#elif (LCDIF_BUS_WIDTH == 16)
	__raw_writel(BF_LCDIF_TRANSFER_COUNT_H_COUNT(240*3*8/16) |
				 BF_LCDIF_TRANSFER_COUNT_V_COUNT(240), REGS_LCDIF_BASE + HW_LCDIF_TRANSFER_COUNT);
#endif

	__raw_writel(BM_LCDIF_CTRL_LCDIF_MASTER,
		REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);
	gpio_direction_output(PINID_LCD_RS, 1); // set data/command select to data via gpio

	__raw_writel(&display_buffer, REGS_LCDIF_BASE + HW_LCDIF_CUR_BUF);
	__raw_writel(&display_buffer, REGS_LCDIF_BASE + HW_LCDIF_NEXT_BUF);

	__raw_writel(BM_LCDIF_CTRL_LCDIF_MASTER, REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);
	__raw_writel(BM_LCDIF_CTRL_RUN, REGS_LCDIF_BASE + HW_LCDIF_CTRL_SET);
	// END BARE METAL SPLASH SCREEN UPDATE

	return 0;
out1:
	return ret;
}
