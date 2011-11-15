/*
 *  i2c.h - I2C bus functions
 *
 *  WDM driver for NetUP Dual DVB-S2 CI card
 *
 *  Copyright (C) 2011 NetUP Inc.
 *  Copyright (C) 2011 Sergey Kozlov <serjk@netup.ru>
 *
 *  Based on code:
 *
 *  Copyright (c) 2006 Steven Toth <stoth@linuxtv.org>
 *  Copyright (C) 1995-99 Simon G. Vogl
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __NETUP_I2C_H__
#define __NETUP_I2C_H__

#include "driver.h"

struct i2c_msg {
	USHORT addr;
	USHORT flags;
	ULONG len;
	PUCHAR buf;
};

/* read bytes */
#define I2C_M_RD 1

LONG i2c_transfer(PKSDEVICE device, ULONG bus_no, i2c_msg * msg, ULONG len);

#endif
