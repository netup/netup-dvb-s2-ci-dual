/*
 * stv6110.h
 *
 * Driver for ST STV6110 satellite tuner IC.
 *
 * Copyright (C) 2009 NetUP Inc.
 * Copyright (C) 2009 Igor M. Liplianin <liplianin@netup.ru>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __DVB_STV6110_H__
#define __DVB_STV6110_H__

#include "driver.h"

/* registers */
#define RSTV6110_CTRL1		0
#define RSTV6110_CTRL2		1
#define RSTV6110_TUNING1	2
#define RSTV6110_TUNING2	3
#define RSTV6110_CTRL3		4
#define RSTV6110_STAT1		5
#define RSTV6110_STAT2		6
#define RSTV6110_STAT3		7

struct stv6110_config
{
	UCHAR i2c_address;
	ULONG mclk;
	UCHAR gain;
	UCHAR clk_div;	/* divisor value for the output clock */
};

struct stv6110_tuner
{
	ULONG no;
	PVOID tuner_priv;
};

NTSTATUS STV6110_Init(PKSDEVICE device);
ULONG STV6110_GetFrequency(PKSDEVICE device, ULONG channel);
NTSTATUS STV6110_SetFrequency(PKSDEVICE device, ULONG demod, ULONG frequency);
NTSTATUS STV6110_SetBandwidth(PKSDEVICE device, ULONG demod, ULONG bandwidth);


#endif
