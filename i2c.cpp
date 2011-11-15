/*
 *  i2c.cpp - I2C bus functions
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

#include "i2c.h"
#include "cx23885.h"
#include "device.h"

#define I2C_WAIT_DELAY 32
#define I2C_WAIT_RETRY 64

#define I2C_EXTEND  (1 << 3)
#define I2C_NOSTOP  (1 << 4)

#define GETBUS(device, bus_no) ((cx23885_i2c *)(GETCONTEXT(device)->i2c_bus) + bus_no)

static BOOL i2c_debug = FALSE;

static inline ULONG i2c_slave_did_ack(PKSDEVICE device, ULONG bus_no)
{
	if(i2c_debug)
	{
		KdPrint((LOG_PREFIX " i2c_slave_did_ack: bus %d reg_stat 0x%x", bus_no, GETBUS(device, bus_no)->reg_stat));
	}
	return cx_read(GETBUS(device, bus_no)->reg_stat) & 0x01;
}

static inline ULONG i2c_is_busy(PKSDEVICE device, ULONG bus_no)
{
	if(i2c_debug)
	{
		KdPrint((LOG_PREFIX " i2c_is_busy: bus %d reg_stat 0x%x", bus_no, GETBUS(device, bus_no)->reg_stat));
	}
	return (cx_read(GETBUS(device, bus_no)->reg_stat) & 0x02) ? 1 : 0;
}

static ULONG i2c_wait_done(PKSDEVICE device, ULONG bus_no)
{
	ULONG count;

	for (count = 0; count < I2C_WAIT_RETRY; count++) {
		if (!i2c_is_busy(device, bus_no))
			break;
		KeStallExecutionProcessor(I2C_WAIT_DELAY);
	}

	if (I2C_WAIT_RETRY == count)
	{
		if(i2c_debug)
		{
			KdPrint((LOG_PREFIX "i2c_wait_done: timeout"));
		}
		return 0;
	}

	return 1;
}

static LONG i2c_sendbytes(PKSDEVICE device, ULONG bus_no, const struct i2c_msg *msg, LONG joined_rlen)
{
	struct cx23885_i2c *bus = GETBUS(device, bus_no);
	ULONG wdata, addr, ctrl, cnt;
	LONG retval;

	if(i2c_debug)
	{
		if (joined_rlen)
			KdPrint((LOG_PREFIX "i2c_sendbytes (msg->wlen=%d, nextmsg->rlen=%d)", msg->len, joined_rlen));
		else
			KdPrint((LOG_PREFIX "i2c_sendbytes (msg->len=%d)",msg->len));
	}

	/* Deal with i2c probe functions with zero payload */
	if (msg->len == 0) {
		if(i2c_debug)
		{
			KdPrint((LOG_PREFIX "i2c_sendbytes: msg->len == 0"));
		}
		cx_write(bus->reg_addr, msg->addr << 25);
		cx_write(bus->reg_ctrl, bus->i2c_period | (1 << 2));
		if (!i2c_wait_done(device, bus_no))
			return -1;
		if (!i2c_slave_did_ack(device, bus_no))
			return -2;

		if(i2c_debug)
		{
			KdPrint((LOG_PREFIX "i2c_sendbytes returns 0"));
		}
		return 0;
	}

	/* dev, reg + first byte */
	addr = (msg->addr << 25) | msg->buf[0];
	wdata = msg->buf[0];
	ctrl = bus->i2c_period | (1 << 12) | (1 << 2);

	if (msg->len > 1)
		ctrl |= I2C_NOSTOP | I2C_EXTEND;
	else if (joined_rlen)
		ctrl |= I2C_NOSTOP;

	cx_write(bus->reg_addr, addr);
	cx_write(bus->reg_wdata, wdata);
	cx_write(bus->reg_ctrl, ctrl);

	if (!i2c_wait_done(device, bus_no))
		goto eio;
	if (i2c_debug)
	{
		KdPrint((LOG_PREFIX "<W %02x %02x>", msg->addr << 1, msg->buf[0]));
	}

	for (cnt = 1; cnt < msg->len; cnt++) {
		/* following bytes */
		wdata = msg->buf[cnt];
		ctrl = bus->i2c_period | (1 << 12) | (1 << 2);

		if (cnt < msg->len - 1)
			ctrl |= I2C_NOSTOP | I2C_EXTEND;
		else if (joined_rlen)
			ctrl |= I2C_NOSTOP;

		cx_write(bus->reg_addr, addr);
		cx_write(bus->reg_wdata, wdata);
		cx_write(bus->reg_ctrl, ctrl);

		if (!i2c_wait_done(device, bus_no))
			goto eio;
		if (i2c_debug)
		{
			KdPrint((LOG_PREFIX "<W %02x>", msg->buf[cnt]));
		}
	}
	if(i2c_debug)
	{
		KdPrint((LOG_PREFIX "i2c_sendbytes: %d sent", msg->len));
	}
	return msg->len;

 eio:
	retval = -1;
	KdPrint((LOG_PREFIX "i2c_sendbytes failed"));
	return retval;
}

static LONG i2c_readbytes(PKSDEVICE device, ULONG bus_no, const struct i2c_msg *msg, int joined)
{
	struct cx23885_i2c *bus = GETBUS(device, bus_no);
	ULONG ctrl, cnt;
	LONG retval;

	if (i2c_debug)
	{
		if(!joined)
			KdPrint((LOG_PREFIX "i2c_readbytes (msg->len=%d)", msg->len));
		else
			KdPrint((LOG_PREFIX "i2c_readbytes (msg->len=%d) joined", msg->len));
	}

	/* Deal with i2c probe functions with zero payload */
	if (msg->len == 0) {
		if(i2c_debug)
		{
			KdPrint((LOG_PREFIX "i2c_readbytes: msg->len == 0"));
		}
		cx_write(bus->reg_addr, msg->addr << 25);
		cx_write(bus->reg_ctrl, bus->i2c_period | (1 << 2) | 1);
		if (!i2c_wait_done(device, bus_no))
			return -1;
		if (!i2c_slave_did_ack(device, bus_no))
			return -2;
		if(i2c_debug)
		{
			KdPrint((LOG_PREFIX "i2c_readbytes returns 0"));
		}
		return 0;
	}

	if (i2c_debug) {
		if (!joined)
		{
			KdPrint((LOG_PREFIX "<R %02x>", (msg->addr << 1) + 1));
		}
	}

	for (cnt = 0; cnt < msg->len; cnt++) {

		ctrl = bus->i2c_period | (1 << 12) | (1 << 2) | 1;

		if (cnt < msg->len - 1)
			ctrl |= I2C_NOSTOP | I2C_EXTEND;

		cx_write(bus->reg_addr, msg->addr << 25);
		cx_write(bus->reg_ctrl, ctrl);

		if (!i2c_wait_done(device, bus_no))
			goto eio;
		msg->buf[cnt] = cx_read(bus->reg_rdata) & 0xff;
		if (i2c_debug)
		{
			KdPrint((LOG_PREFIX "<R %02x>", msg->buf[cnt]));
		}
	}
	return msg->len;

 eio:
	retval = -1;
	if (i2c_debug)
	{
		KdPrint((LOG_PREFIX "i2c_readbytes failed"));
	}
	return retval;
}

LONG i2c_transfer(PKSDEVICE device, ULONG bus_no, i2c_msg *msgs, ULONG num)
{
	ASSERT(bus_no < 3);
	struct cx23885_i2c *bus = GETBUS(device, bus_no);
	ULONG i;
	LONG retval = 0;

	KIRQL oldIrql;
	KeAcquireSpinLock(&bus->lock, &oldIrql);

	for (i = 0 ; i < num; i++) {
		if(i2c_debug)
		{
			KdPrint((LOG_PREFIX "i2c_transfer(bus_id=%d msg_num = %d) addr = 0x%02x  len = 0x%x", bus_no, num, msgs[i].addr, msgs[i].len));
		}
		if (msgs[i].flags & I2C_M_RD) {
			/* read */
			retval = i2c_readbytes(device, bus_no, &msgs[i], 0);
		} 
		else if (i + 1 < num && (msgs[i + 1].flags & I2C_M_RD) &&
			   msgs[i].addr == msgs[i + 1].addr) {
			/* write then read from same address */
			retval = i2c_sendbytes(device, bus_no, &msgs[i],
					       msgs[i + 1].len);
			if (retval < 0)
				goto err;
			i++;
			retval = i2c_readbytes(device, bus_no, &msgs[i], 1);

		}
		else {
			/* write */
			retval = i2c_sendbytes(device, bus_no, &msgs[i], 0);
		}
		if (retval < 0)
			goto err;
	}
	KeReleaseSpinLock(&bus->lock, oldIrql);
	return num;

err:
	KeReleaseSpinLock(&bus->lock, oldIrql);
	return retval;
}