/*
 * Copyright (C) 2014 Revolution Robotics, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/io.h>
#include "warp_bbi2c.h"
#include "warp_common.h"

// Example read register sequence
	/*	BBI2C_Start();
	ret = BBI2C_TransmitByte((FXOS8700_I2C_ADDR << 1) | (BBI2C_WRITE));
	if(ret){
		printf("Failed sending device address write to FXOS\n");
		goto stop_seq;
	}
	ret = BBI2C_TransmitByte(0x0D);
	if(ret){
		printf("Failed sending register address to FXOS\n");
		goto stop_seq;
	}

	BBI2C_Start(); // Perform repeated start for read

	ret = BBI2C_TransmitByte((FXOS8700_I2C_ADDR << 1) | (BBI2C_READ));
	if(ret){
		printf("Failed sending device address read to FXOS\n");
		goto stop_seq;
	}else{
		ret = BBI2C_Read(0x1);
		BBI2C_Stop();
		printf("Read from FXOS: 0x%02X\n", ret);
	}
	//return;
	//stop_seq:
	//	BBI2C_Stop();
	// End example read register sequence */

void BBI2C_CLK_HI(void)
{
	gpio_direction_output(BBI2C_CLK,1);
	udelay(BB_I2C_SETTLE_US);
}

void BBI2C_WriteBit(unsigned char data)
{
	gpio_direction_output(BBI2C_DAT, data);
	udelay(BB_I2C_SETTLE_US);

	BBI2C_CLK_HI();

	gpio_direction_output(BBI2C_CLK,0);
	udelay(BB_I2C_SETTLE_US);
}

unsigned char BBI2C_ReadBit()
{
	unsigned char res = 0;

	gpio_direction_output(BBI2C_DAT,1);

	BBI2C_CLK_HI();
	gpio_direction_output(BBI2C_CLK,0);
	udelay(BB_I2C_SETTLE_US);
	gpio_direction_input(BBI2C_DAT);
	udelay(BB_I2C_SETTLE_US);

	res = (unsigned char)gpio_get_value(BBI2C_DAT);

	gpio_direction_output(BBI2C_DAT,1);

	return res;
}

// SETUP IO according to the bitbanging pin mux io
void BBI2C_Init(void)
{
	//later this could be made generic by passing in a config struct.
	//set clk/dat lines as output and set value to high
	gpio_direction_output(BBI2C_DAT, 1);
	udelay(BB_I2C_SETTLE_US);
	BBI2C_CLK_HI();
}

// Send a START Condition
void BBI2C_Start(void)
{
	gpio_direction_output(BBI2C_DAT,1);
	udelay(BB_I2C_SETTLE_US);
	BBI2C_CLK_HI();

	gpio_direction_output(BBI2C_DAT,0);
	udelay(BB_I2C_SETTLE_US);

	gpio_direction_output(BBI2C_CLK,0);
	udelay(BB_I2C_SETTLE_US);
}

// Send a STOP Condition
void BBI2C_Stop(void)
{
	gpio_direction_output(BBI2C_DAT,0);
	udelay(BB_I2C_SETTLE_US);
	gpio_direction_output(BBI2C_CLK,1);
	udelay(BB_I2C_SETTLE_US);
	gpio_direction_output(BBI2C_DAT,1);
	udelay(BB_I2C_SETTLE_US);
}

//Write I2C byte
unsigned char BBI2C_TransmitByte(unsigned char data)
{
	char i;
	for (i = 0; i < 8; ++i)
	{
		if(data & 0x80)
			gpio_direction_output(BBI2C_DAT, 1);
		else
			gpio_direction_output(BBI2C_DAT, 0);

		udelay(BB_I2C_SETTLE_US);
		gpio_direction_output(BBI2C_CLK,1);
		udelay(BB_I2C_SETTLE_US);
		gpio_direction_output(BBI2C_CLK,0);
		udelay(BB_I2C_SETTLE_US);
		data = (data << 1);
	}
	gpio_direction_output(BBI2C_DAT, 1);
	udelay(BB_I2C_SETTLE_US);
	gpio_direction_output(BBI2C_CLK,1);
	udelay(BB_I2C_SETTLE_US);
	gpio_direction_input(BBI2C_DAT);
	udelay(BB_I2C_SETTLE_US);
	i = gpio_get_value(BBI2C_DAT);
	gpio_direction_output(BBI2C_CLK,0);
	udelay(BB_I2C_SETTLE_US);
	return i;
}

//Read I2C byte
unsigned char BBI2C_Read(unsigned char ack)
{
	unsigned char res = 0;
	unsigned char i;
	int pad;
	gpio_direction_output(BBI2C_DAT, 1); // Master releases data line
	udelay(BB_I2C_SETTLE_US);
	gpio_direction_input(BBI2C_DAT);
	udelay(BB_I2C_SETTLE_US);

	for (i = 0; i < 8; i++)
	{
		res = res << 1;
		//
		gpio_direction_output(BBI2C_CLK,1);
		udelay(BB_I2C_SETTLE_US);
		pad = gpio_get_value(BBI2C_DAT);
		if(pad)
			res = res | 0x01;

		gpio_direction_output(BBI2C_CLK,0);
		udelay(BB_I2C_SETTLE_US);
	}

	if (ack > 0)
		gpio_direction_output(BBI2C_DAT, 0);
	else
		gpio_direction_output(BBI2C_DAT, 1);

	udelay(BB_I2C_SETTLE_US);
	gpio_direction_output(BBI2C_CLK,1);
	udelay(BB_I2C_SETTLE_US);
	gpio_direction_output(BBI2C_CLK,0);
	udelay(BB_I2C_SETTLE_US);
	gpio_direction_output(BBI2C_DAT, 1);

	return res;
}
