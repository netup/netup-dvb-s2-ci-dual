/*
 *  lnbh24.cpp
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

#include "lnbh24.h"
#include "device.h"
#include "i2c.h"

struct lnbp21 {
	PKSDEVICE	device;
	ULONG		demod;
	UCHAR		config;
	UCHAR		override_or;
	UCHAR		override_and;
	ULONG		i2c;
	UCHAR		i2c_addr;
};

static LONG lnbp21_set_voltage(struct lnbp21 * lnbp21,
					fe_sec_voltage_t voltage)
{
	struct i2c_msg msg;
	msg.addr = lnbp21->i2c_addr;
	msg.flags = 0;
	msg.buf = &lnbp21->config;
	msg.len = sizeof(lnbp21->config);

	lnbp21->config &= ~(LNBP21_VSEL | LNBP21_EN);

	switch(voltage) {
	case SEC_VOLTAGE_OFF:
		break;
	case SEC_VOLTAGE_13:
		lnbp21->config |= LNBP21_EN;
		break;
	case SEC_VOLTAGE_18:
		lnbp21->config |= (LNBP21_EN | LNBP21_VSEL);
		break;
	default:
		return -1;
	};

	lnbp21->config |= lnbp21->override_or;
	lnbp21->config &= lnbp21->override_and;

	KdPrint((LOG_PREFIX "lnbp21: config %d", (ULONG)(lnbp21->config)));

	return (i2c_transfer(lnbp21->device, lnbp21->i2c, &msg, 1) == 1) ? 0 : -2;
}

static LONG lnbp21_enable_high_lnb_voltage(struct lnbp21 * lnbp21, LONG arg)
{
	struct i2c_msg msg;
	msg.addr = lnbp21->i2c_addr;
	msg.flags = 0;
	msg.buf = &lnbp21->config;
	msg.len = sizeof(lnbp21->config);

	if (arg)
		lnbp21->config |= LNBP21_LLC;
	else
		lnbp21->config &= ~LNBP21_LLC;

	lnbp21->config |= lnbp21->override_or;
	lnbp21->config &= lnbp21->override_and;

	KdPrint((LOG_PREFIX "lnbp21: config %d", (ULONG)(lnbp21->config)));

	return (i2c_transfer(lnbp21->device, lnbp21->i2c, &msg, 1) == 1) ? 0 : -1;
}

static LONG lnbp21_set_tone(struct lnbp21 * lnbp21,
				fe_sec_tone_mode_t tone)
{
	struct i2c_msg msg;
	msg.addr = lnbp21->i2c_addr;
	msg.flags = 0;
	msg.buf = &lnbp21->config;
	msg.len = sizeof(lnbp21->config);

	switch (tone) {
	case SEC_TONE_OFF:
		lnbp21->config &= ~LNBP21_TEN;
		break;
	case SEC_TONE_ON:
		lnbp21->config |= LNBP21_TEN;
		break;
	default:
		return -1;
	};

	lnbp21->config |= lnbp21->override_or;
	lnbp21->config &= lnbp21->override_and;

	KdPrint((LOG_PREFIX "lnbp21: config %d", (ULONG)(lnbp21->config)));

	return (i2c_transfer(lnbp21->device, lnbp21->i2c, &msg, 1) == 1) ? 0 : -2;
}

static void lnbp21_release(struct lnbp21 * lnbp)
{
	/* LNBP power off */
	lnbp21_set_voltage(lnbp, SEC_VOLTAGE_OFF);

	/* free data */
	ExFreePool(lnbp);
}

PVOID lnbx2x_attach(PKSDEVICE device, ULONG i2c, ULONG demod, UCHAR override_set,
				UCHAR override_clear, UCHAR i2c_addr, UCHAR config)
{
	struct lnbp21 *lnbp21 = (struct lnbp21 *)ExAllocatePoolWithTag(NonPagedPool, sizeof(struct lnbp21), '2BNL');
	if (!lnbp21)
	{
		KdPrint((LOG_PREFIX "unable to allocate lnbp21 context"));
		return NULL;
	}

	/* default configuration */
	lnbp21->device = device;
	lnbp21->demod = demod;
	lnbp21->config = config;
	lnbp21->i2c = i2c;
	lnbp21->i2c_addr = i2c_addr;

	/* bits which should be forced to '1' */
	lnbp21->override_or = override_set;

	/* bits which should be forced to '0' */
	lnbp21->override_and = ~override_clear;

	/* detect if it is present or not */
	if (lnbp21_set_voltage(lnbp21, SEC_VOLTAGE_OFF)) {
		KdPrint((LOG_PREFIX "lnbx2x_attach: LNB not present"));
		ExFreePool(lnbp21);
		return NULL;
	}

	lnbp21_set_voltage(lnbp21, SEC_VOLTAGE_18);
	lnbp21_set_tone(lnbp21, SEC_TONE_OFF);

	DbgPrint(LOG_PREFIX "LNBx2x (demod %d) attached on addr=%x", demod, lnbp21->i2c_addr);

	return lnbp21;
}


PVOID LNBH24_Attach(PKSDEVICE device, ULONG i2c, ULONG demod, UCHAR override_set, UCHAR override_clear, UCHAR i2c_addr)
{
	ASSERT(demod < 2);
	return lnbx2x_attach(device, i2c, demod, override_set, override_clear,
							i2c_addr, LNBH24_TTX);
}

NTSTATUS LNBH24_Init(PKSDEVICE device)
{
	KdPrint((LOG_PREFIX "*** LNBH24_Init ***"));
	PVOID lnb0 = LNBH24_Attach(device, 0, 0, LNBH24_PCL | LNBH24_TTX, LNBH24_TEN, 0x09);
	if(!lnb0)
	{
		KdPrint((LOG_PREFIX "LNBH24_Init failed on demod #0"));
		return STATUS_INSUFFICIENT_RESOURCES;
	}
	PVOID lnb1 = LNBH24_Attach(device, 0, 1, LNBH24_PCL | LNBH24_TTX, LNBH24_TEN, 0x0a);
	if(!lnb1)
	{
		KdPrint((LOG_PREFIX "LNBH24_Init failed on demod #1"));
		return STATUS_INSUFFICIENT_RESOURCES;
	}
	GETCONTEXT(device)->lnbh24_context[0] = lnb0;
	GETCONTEXT(device)->lnbh24_context[1] = lnb1;
	KdPrint((LOG_PREFIX "*** LNBH24_Init done ***"));
	return STATUS_SUCCESS;
}

NTSTATUS LNBH24_SetVoltage(PKSDEVICE device, ULONG channel, BOOLEAN enableHighVoltage)
{
	ASSERT(channel < 2);
	struct lnbp21 *lnbp21 = (struct lnbp21 *)(GETCONTEXT(device)->lnbh24_context[channel]);
	if(lnbp21 == NULL)
	{
		KdPrint((LOG_PREFIX "%s: context not initialized for channed %d", __FUNCTION__, channel));
		return STATUS_UNSUCCESSFUL;
	}
	if(enableHighVoltage)
	{
		lnbp21_set_voltage(lnbp21, SEC_VOLTAGE_18);
	}
	else
	{
		lnbp21_set_voltage(lnbp21, SEC_VOLTAGE_13);
	}
	return STATUS_SUCCESS;
}
