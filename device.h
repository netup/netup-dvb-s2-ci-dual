/*
 * device.h
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

#ifndef __NETUP_DEVICE_H__
#define __NETUP_DEVICE_H__

#include "driver.h"

extern const KSDEVICE_DESCRIPTOR NetupDeviceDescriptor;

/* BDA tuner parameters */
struct NetupTunerData
{
	ULONG lnbLowFreq;		// low oscillator freq
	ULONG lnbHighFreq;		// high oscillator freq
	ULONG switchFreq;		// LNB switch freq

	ULONG polarisation;		// signal polarisation

	ULONG freq;				// tune freq
	ULONG freqMult;			// realFreq = freq * freqMult

	ULONG symbolRate;		// transponder symbol rate
};

/* BDA tuner signal info */
struct NetupSignalInfo
{
	BOOL locked;
	ULONG fec;
	ULONG modulation;
	ULONG pilot;
	ULONG spectrum;
	ULONG rolloff;
};

/* Device general context */
struct DEVICE_CONTEXT
{
	// Device access
	PVOID pci_memory;
	ULONG pci_size;
	PKINTERRUPT pci_interrupt;
	PDMA_ADAPTER dma;
	KDPC ci_dpc;

	// Tuner data
	NetupTunerData tuner[2];
	NetupSignalInfo signal[2];

	// Filters
	PKSFILTER filter[2];
	PKSPIN output_pin[2];

	// CX23558 PCIe bridge
	ULONG clk_freq;
	PVOID sram_channels;
	ULONG pci_irqmask;
	KSPIN_LOCK pci_irqmask_lock;
	PVOID ts1, ts2;
	PVOID i2c_bus;
	// Other components
	PVOID stv0900_context;
	PVOID stv6110_context[2];
	PVOID lnbh24_context[2];
	PVOID cimax2_context[2];
	PVOID en50221_context[2];
	
};

#define GETCONTEXT(x)	((DEVICE_CONTEXT*)(((PKSDEVICE)(x))->Context))

#endif // __NETUP_DEVICE_H__
