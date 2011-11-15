/*
 *  lnbh24.h
 *
 *  WDM driver for NetUP Dual DVB-S2 CI card
 *
 *  Copyright (C) 2011 NetUP Inc.
 *  Copyright (C) 2011 Sergey Kozlov <serjk@netup.ru>
 *
 *  Driver for lnb supply and control ic lnbp21
 *
 *  Copyright (C) 2006, 2009 Oliver Endriss <o.endriss@gmx.de>
 *  Copyright (C) 2009 Igor M. Liplianin <liplianin@netup.ru>
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

#ifndef __NETUP_LNBH24_H__
#define __NETUP_LNBH24_H__

#include "driver.h"
#include "sec.h"

/* system register bits */
/* [RO] 0=OK; 1=over current limit flag */
#define LNBP21_OLF	0x01
/* [RO] 0=OK; 1=over temperature flag (150 C) */
#define LNBP21_OTF	0x02
/* [RW] 0=disable LNB power, enable loopthrough
	1=enable LNB power, disable loopthrough */
#define LNBP21_EN	0x04
/* [RW] 0=low voltage (13/14V, vert pol)
	1=high voltage (18/19V,horiz pol) */
#define LNBP21_VSEL	0x08
/* [RW] increase LNB voltage by 1V:
	0=13/18V; 1=14/19V */
#define LNBP21_LLC	0x10
/* [RW] 0=tone controlled by DSQIN pin
	1=tone enable, disable DSQIN */
#define LNBP21_TEN	0x20
/* [RW] current limit select:
	0:Iout=500-650mA Isc=300mA
	1:Iout=400-550mA Isc=200mA */
#define LNBP21_ISEL	0x40
/* [RW] short-circuit protect:
	0=pulsed (dynamic) curr limiting
	1=static curr limiting */
#define LNBP21_PCL	0x80

/* system register bits */
#define LNBH24_OLF	0x01
#define LNBH24_OTF	0x02
#define LNBH24_EN	0x04
#define LNBH24_VSEL	0x08
#define LNBH24_LLC	0x10
#define LNBH24_TEN	0x20
#define LNBH24_TTX	0x40
#define LNBH24_PCL	0x80

NTSTATUS LNBH24_Init(PKSDEVICE device);
NTSTATUS LNBH24_SetVoltage(PKSDEVICE device, ULONG channel, BOOLEAN enableHighVoltage);

#endif // __NETUP_LNBH24_H__