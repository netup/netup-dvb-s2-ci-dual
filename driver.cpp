/*
 * driver.cpp
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

#include "driver.h"
#include "device.h"
#include <ntstrsafe.h>

VOID DelayMilliseconds(ULONG delay)
{
	KeStallExecutionProcessor(delay*1000);
}

VOID DelayMicroseconds(ULONG delay)
{
	KeStallExecutionProcessor(delay);
}

VOID RawDump(PUCHAR buf, ULONG size)
{
	KdPrint(("-- Raw data (%d bytes) --", size));
	PUCHAR ptr = buf;
	CHAR str[16*3+1];
	for(ULONG i = 0; i < size/16; i++)
	{
		for(ULONG j = 0; j < 16; j++)
		{
			RtlStringCbPrintfA(str + j*3, sizeof(str)-j*3, "%02x ", (ULONG)(*ptr++));
		}
		KdPrint(("%s", str));
	}
	if((size % 16) > 0)
	{
		for(ULONG j = 0; j < (size % 16); j++)
		{
			RtlStringCbPrintfA(str + j*3, sizeof(str)-j*3, "%02x ", (ULONG)(*ptr++));
		}
		KdPrint(("%s", str));
	}
	KdPrint(("-- Raw data end --"));
}

extern "C"
{

DRIVER_INITIALIZE DriverEntry;

NTSTATUS DriverEntry(__in struct _DRIVER_OBJECT  *DriverObject, __in PUNICODE_STRING  RegistryPath)
{
	KdPrint((LOG_PREFIX "*** DriverEntry ***"));
	return KsInitializeDriver(DriverObject, RegistryPath, &NetupDeviceDescriptor);
}

}
