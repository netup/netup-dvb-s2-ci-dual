/*
 * driver.h
 *
 *  WDM driver for NetUP Dual DVB-S2 CI card
 *
 * Copyright (C) 2011 NetUP Inc.
 * Copyright (C) 2011 Sergey Kozlov <serjk@netup.ru>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.				
 */

#ifndef __NETUP_DRIVER_H__
#define __NETUP_DRIVER_H__
extern "C" {
#include <wdm.h>
}
#include <windef.h>
#include <stdio.h>
#include <stdlib.h>
#include <windef.h>
#include <unknown.h>
#include <uuids.h>
#include <ks.h>
#include <ksmedia.h>
#include <kcom.h>
#include <bdatypes.h>
#include <bdamedia.h>
#include <bdasup.h>
#define LOG_PREFIX "NetUP-CX23885: "
#include <ksdebug.h>

#define CLOCKS_PER_SEC	10000000

VOID DelayMilliseconds(ULONG delay);
VOID DelayMicroseconds(ULONG delay);
VOID RawDump(PUCHAR buf, ULONG size);

#endif // __NETUP_DRIVER_H__
