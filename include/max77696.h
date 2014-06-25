/*
 * Copyright (C) 2014 Revolution Robotics, Inc.
 *
 * Created on: May 7, 2014
 *     Author: Jacob Postman
 *
 * SPDX-License-Identifier:    GPL-2.0+
 */

#ifndef MAX77696_H_
#define MAX77696_H_

// MAX77696 I2C Addresses
#define MAX77696_RTC_EH_ADDR	0b1101000  // W: 0xD0 and R: 0xD1 in MAX datasheet
#define MAX77696_UIC_ADDR	0b0110101  // W: 0x6A and R: 0x6B in MAX datasheet
#define MAX77696_FUEL_ADDR	0b0110100  // W: 0x68 and R: 0x69 in MAX datasheet
#define MAX77696_PMIC_ADDR	0b0111100  // W: 0x78 and R: 0x79 in MAX datasheet

// MAX77696 Register Addresses


#define VOUT6		0x38

#define GLBLCNFG1	0x01


#define	L01_CNFG1	0x43
#define	L01_CNFG2	0x44
#define	L02_CNFG1	0x45
#define	L02_CNFG2	0x46
#define	L03_CNFG1	0x47
#define	L03_CNFG2	0x48
#define	L04_CNFG1	0x49
#define	L04_CNFG2	0x4A
#define	L05_CNFG1	0x4B
#define	L05_CNFG2	0x4C
#define	L06_CNFG1	0x4D
#define	L06_CNFG2	0x4E
#define	L07_CNFG1	0x4F
#define	L07_CNFG2	0x50
#define	L08_CNFG1	0x51
#define	L08_CNFG2	0x52
#define	L09_CNFG1	0x53
#define	L09_CNFG2	0x54
#define	L10_CNFG1	0x55
#define	L10_CNFG2	0x56
#define	LDO_INT1	0x57
#define	LDO_INT2	0x58
#define	LDO_INT1M	0x59
#define	LDO_INT2M	0x5A
#define	LDO_CNFG3	0x5B

#define LEDBST_CNTRL_1	0x6C
#define LED1CURRENT_1 	0x6D
#define LED1CURRENT_2 	0x6E


#define DOx_SHIFT		3
#define DIx_SHIFT		2
#define DIRx_SHIFT		1
#define PPRDRVx_SHIFT	0

#define	CNFG_GPIO0		0x75
#define	CNFG_GPIO1		0x76
#define	CNFG_GPIO2		0x77
#define	CNFG_GPIO3		0x78
#define	CNFG_GPIO4		0x79
#define	PUE_GPIO		0x7A
#define	PDE_GPIO		0x7B
#define	AME_GPIO		0x7C
#define	IRQ_LVL2_GPIO	0x7D


#endif /* MAX77696_H_ */
