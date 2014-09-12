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

#ifndef WARP_MX6SL_PWM
#define WARP_MX6SL_PWM

#define PWM3_PWMCR		0x02088000
#define PWM3_PWMSR		0x02088004
#define PWM3_PWMIR		0x02088008
#define PWM3_PWMSAR		0x0208800C
#define PWM3_PWMPR		0x02088010
#define PWM3_PWMCNR		0x02088014

#define PWM4_PWMCR		0x0208C000
#define PWM4_PWMSR		0x0208C004
#define PWM4_PWMIR		0x0208C008
#define PWM4_PWMSAR		0x0208C00C
#define PWM4_PWMPR		0x0208C010
#define PWM4_PWMCNR		0x0208C014

#define PWMCR_STOPEN		(1 << 25)
#define PWMCR_DOZEEN		(1 << 24)
#define PWMCR_WAITEN		(1 << 23)
#define PWMCR_DBGEN		(1 << 22)
#define PWMCR_CLKSRC(x)		((x) << 16)
#define PWMCR_PRESCALER(x) 	((x-1) & 0xFFF << 4) // 000-fff
#define PWMCR_ENABLE 		(1 << 0)

#endif // WARP_MX6SL_PWM
