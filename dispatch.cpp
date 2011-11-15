/*
 * dispatch.cpp
 *
 *  WDM driver for NetUP Dual DVB-S2 CI card
 *
 * Copyright (C) 2011 NetUP Inc.
 * Copyright (C) 2011 Sergey Kozlov <serjk@netup.ru>
 *
 * Hauppauge Nova-T BDA driver
 *
 * Copyright (C) 2003 Colin Munro (colin@mice-software.com)
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

#include "dispatch.h"
#include "device.h"
#include "filter.h"
#include "cx23885.h"

NTSTATUS NetupAntennaCreate(IN PKSPIN Pin,IN PIRP Irp)
{
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	return STATUS_SUCCESS;
}

NTSTATUS NetupAntennaClose(IN PKSPIN Pin,IN PIRP Irp)
{
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	return STATUS_SUCCESS;
}

NTSTATUS NetupAntennaSetDeviceState(IN PKSPIN Pin,IN KSSTATE ToState,IN KSSTATE FromState)
{
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	return STATUS_SUCCESS;
}

NTSTATUS NetupOutputCreate(IN PKSPIN Pin,IN PIRP Irp)
{
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));

	Pin->Context=KsPinGetDevice(Pin);
    if((Pin->Context==NULL)||(((PKSDEVICE)Pin->Context)->Context==NULL)) {
        KdPrint((LOG_PREFIX "NetupOutputCreate: STATUS_DEVICE_NOT_CONNECTED"));
        return STATUS_DEVICE_NOT_CONNECTED;
    }
	PKSFILTER Filter = KsPinGetParentFilter(Pin);
	if(Filter == NULL || Filter->Context == NULL)
	{
		KdPrint((LOG_PREFIX "NetupOutputCreate: filter for PIN not set"));
		return STATUS_DEVICE_NOT_CONNECTED;
	}
	ULONG nr = ((FilterContext *)(Filter->Context))->nr;
	ASSERT(nr < 2);
    GETCONTEXT(Pin->Context)->output_pin[nr] =Pin;
    //GETCONTEXT(Pin->Context)->cur_state=KSSTATE_STOP;

	return STATUS_SUCCESS;
}

NTSTATUS NetupOutputClose(IN PKSPIN Pin,IN PIRP Irp)
{
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));

	PKSFILTER Filter = KsPinGetParentFilter(Pin);
	if(Filter == NULL || Filter->Context == NULL)
	{
		KdPrint((LOG_PREFIX "%s: filter for PIN not set", __FUNCTION__));
		return STATUS_DEVICE_NOT_CONNECTED;
	}
	ULONG nr = ((FilterContext *)(Filter->Context))->nr;

	if(Pin->Context)
	{
		GETCONTEXT(Pin->Context)->output_pin[nr] = NULL;
	}

	return STATUS_SUCCESS;
}

NTSTATUS NetupOutputSetDeviceState(IN PKSPIN Pin,IN KSSTATE ToState,IN KSSTATE FromState)
{
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));

	PKSFILTER Filter = KsPinGetParentFilter(Pin);
	if(Filter == NULL || Filter->Context == NULL)
	{
		KdPrint((LOG_PREFIX "%s: filter for PIN not set", __FUNCTION__));
		return STATUS_DEVICE_NOT_CONNECTED;
	}
	ULONG nr = ((FilterContext *)(Filter->Context))->nr;
	ASSERT(nr < 2);
	PKSDEVICE device = ((FilterContext *)(Filter->Context))->device;
	ASSERT(device != NULL);
	cx23885_tsport * port = NULL;
	switch(nr)
	{
	case 0:
		port = (cx23885_tsport *)GETCONTEXT(device)->ts1;
		break;
	case 1:
		port = (cx23885_tsport *)GETCONTEXT(device)->ts2;
		break;
	}
	ASSERT(port != NULL);

	if((ToState != KSSTATE_STOP) && (FromState == KSSTATE_STOP))
	{
		CX23885_Start_DMA(device, port);
	}

	if((ToState == KSSTATE_STOP) && (FromState != KSSTATE_STOP))
	{
		CX23885_Stop_DMA(device, port);
	}

	return STATUS_SUCCESS;
}

NTSTATUS NetupOutputProcess(IN PKSPIN Pin)
{
	ASSERT(Pin != NULL);
	PKSFILTER Filter = KsPinGetParentFilter(Pin);
	if(Filter == NULL || Filter->Context == NULL)
	{
		KdPrint((LOG_PREFIX "%s: no filter associated with pin", __FUNCTION__));
		return STATUS_DEVICE_NOT_CONNECTED;
	}
	PKSDEVICE Device = ((FilterContext *)(Filter->Context))->device;
	if(Device == NULL)
	{
		KdPrint((LOG_PREFIX "%s: no device associated with filter", __FUNCTION__));
		return STATUS_DEVICE_NOT_CONNECTED;
	}
	ULONG Nr = ((FilterContext *)(Filter->Context))->nr;
	ASSERT(Nr < 2);
	PVOID Port = NULL;
	switch(Nr)
	{
	case 0:
		Port = GETCONTEXT(Device)->ts1;
		break;
	case 1:
		Port = GETCONTEXT(Device)->ts2;
		break;
	default:
		ASSERT(0);
	}
	CX23885_OutputProcess(Device, Pin, Port);
	return STATUS_PENDING;
}
