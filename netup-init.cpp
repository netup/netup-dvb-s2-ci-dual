/*
 * netup-init.cpp
 *
 * WDM driver for NetUP Dual DVB-S2 CI
 *
 * Copyright (C) 2011 NetUP Inc.
 * Copyright (C) 2011 Sergey Kozlov <serjk@netup.ru>
 * Copyright (C) 2009 Igor M. Liplianin <liplianin@netup.ru>
 * Copyright (C) 2009 Abylay Ospan <aospan@netup.ru>
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

#include "cx23885.h"
#include "i2c.h"

void i2c_av_write(PKSDEVICE device, USHORT reg, UCHAR val)
{
	LONG ret;
	UCHAR buf[3];

	struct i2c_msg msg;
	msg.addr	= 0x88 >> 1;
	msg.flags	= 0;
	msg.buf	= buf;
	msg.len	= 3;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val;

	ret = i2c_transfer(device, 2, &msg, 1);

	if (ret != 1)
	{
		KdPrint(("i2c_av_write: i2c write error"));
	}
}

static void i2c_av_write4(PKSDEVICE device, USHORT reg, ULONG val)
{
	LONG ret;
	UCHAR buf[6];

	struct i2c_msg msg;
	msg.addr	= 0x88 >> 1;
	msg.flags	= 0;
	msg.buf	= buf;
	msg.len	= 6;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val & 0xff;
	buf[3] = (val >> 8) & 0xff;
	buf[4] = (val >> 16) & 0xff;
	buf[5] = val >> 24;

	ret = i2c_transfer(device, 2, &msg, 1);

	if (ret != 1)
	{
		KdPrint(("i2c_av_write4: i2c write error"));
	}
}

UCHAR i2c_av_read(PKSDEVICE device, USHORT reg)
{
	LONG ret;
	UCHAR buf[2];
	struct i2c_msg msg;
	msg.addr	= 0x88 >> 1;
	msg.flags	= 0;
	msg.buf	= buf;
	msg.len	= 2;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	ret = i2c_transfer(device, 2, &msg, 1);

	if (ret != 1)
	{
		KdPrint(("i2c_av_read: i2c write error"));
	}

	msg.flags = I2C_M_RD;
	msg.len = 1;

	ret = i2c_transfer(device, 2, &msg, 1);

	if (ret != 1)
	{
		KdPrint(("i2c_av_read: i2c read error"));
	}

	return buf[0];
}

static void i2c_av_and_or(PKSDEVICE device, USHORT reg, UCHAR and_mask, UCHAR or_value)
{
	i2c_av_write(device, reg, (i2c_av_read(device, reg) & and_mask) | or_value);
}

#define CX25840_VID_INT_STAT_REG 0x410
#define CX25840_AUD_INT_CTRL_REG 0x812
#define CX25840_AUD_INT_STAT_REG 0x813

/* set 27MHz on AUX_CLK */
void NetupCardInit(PKSDEVICE device)
{

	/*
     * Come out of digital power down
     * The CX23888, at least, needs this, otherwise registers aside from
     * 0x0-0x2 can't be read or written.
     */
	i2c_av_write(device, 0x0, 0x0);
	/* Internal Reset */
	i2c_av_and_or(device, 0x102, ~0x01, 0x01);
	i2c_av_and_or(device, 0x102, ~0x01, 0x00);

	/* Stop microcontroller */
    i2c_av_and_or(device, 0x803, ~0x10, 0x00);

    /* DIF in reset? */
    i2c_av_write(device, 0x398, 0);

	 /*
     * Trust the default xtal, no division
     * '885: 28.636363... MHz
     * '887: 25.000000 MHz
     * '888: 50.000000 MHz
     */
    i2c_av_write(device, 0x2, 0x76);

    /* Power up all the PLL's and DLL */
    i2c_av_write(device, 0x1, 0x40);

	/*
     * 28.636363 MHz * (0x14 + 0x0/0x2000000)/4 = 5 * 28.636363 MHz
     * 572.73 MHz before post divide
     */
    i2c_av_write4(device, 0x11c, 0x00000000);
    i2c_av_write4(device, 0x118, 0x00000414);
	
	/* Disable DIF bypass */
    i2c_av_write4(device, 0x33c, 0x00000001);

    /* DIF Src phase inc */
    i2c_av_write4(device, 0x340, 0x0df7df83);

	/*
     * Vid PLL
     * Setup for a BT.656 pixel clock of 13.5 Mpixels/second
     *
     * 28.636363 MHz * (0xf + 0x02be2c9/0x2000000)/4 = 8 * 13.5 MHz
     * 432.0 MHz before post divide
     */
    i2c_av_write4(device, 0x10c, 0x002be2c9);
    i2c_av_write4(device, 0x108, 0x0000040f);

    /* Luma */
    i2c_av_write4(device, 0x414, 0x00107d12);

    /* Chroma */
    i2c_av_write4(device, 0x420, 0x3d008282);

#if 0
	/*
         * 28.636363 MHz * (0xc + 0x1bf0c9e/0x2000000)/3 = 122.88 MHz
         * 368.64 MHz before post divide
         * 122.88 MHz / 0xa = 12.288 MHz
         */
    i2c_av_write4(device, 0x114, 0x01bf0c9e);
    i2c_av_write4(device, 0x110, 0x000a030c);
#endif

	/* Aux PLL frac for 27 MHz */
	i2c_av_write4(device, 0x114, 0xea0eb3);

	/* Aux PLL int for 27 MHz */
	i2c_av_write4(device, 0x110, 0x090319);


	/* ADC2 input select */
    i2c_av_write(device, 0x102, 0x10);

    /* VIN1 & VIN5 */
    i2c_av_write(device, 0x103, 0x11);

    /* Enable format auto detect */
    i2c_av_write(device, 0x400, 0);
    /* Fast subchroma lock */
    /* White crush, Chroma AGC & Chroma Killer enabled */
    i2c_av_write(device, 0x401, 0xe8);

    /* Select AFE clock pad output source */
    i2c_av_write(device, 0x144, 0x05);

    /* Drive GPIO2 direction and values for HVR1700
     * where an onboard mux selects the output of demodulator
     * vs the 417. Failure to set this results in no DTV.
     * It's safe to set this across all Hauppauge boards
     * currently, regardless of the board type.
     */
    i2c_av_write(device, 0x160, 0x1d);
    i2c_av_write(device, 0x164, 0x00);

	CX23885_Load_FW(device);

	/* start microcontroller */
    i2c_av_and_or(device, 0x803, ~0x10, 0x10);

	/* Disable and clear video interrupts - we don't use them */
    i2c_av_write4(device, CX25840_VID_INT_STAT_REG, 0xffffffff);

    /* Disable and clear audio interrupts - we don't use them */
    i2c_av_write(device, CX25840_AUD_INT_CTRL_REG, 0xff);
    i2c_av_write(device, CX25840_AUD_INT_STAT_REG, 0xff);

}

#define EEPROM_I2C_ADDR 0x50

UCHAR Netup_EEPROM_Read(PKSDEVICE device, ULONG i2c_adap, UCHAR addr)
{
	LONG ret;
	UCHAR buf[2];

	/* Read from EEPROM */
	struct i2c_msg msg[2];
	
	msg[0].addr	= EEPROM_I2C_ADDR;
	msg[0].flags	= 0;
	msg[0].buf	= &buf[0];
	msg[0].len	= 1;
	
	msg[1].addr	= EEPROM_I2C_ADDR;
	msg[1].flags	= I2C_M_RD;
	msg[1].buf	= &buf[1];
	msg[1].len	= 1;

	buf[0] = addr;
	buf[1] = 0x0;

	ret = i2c_transfer(device, i2c_adap, msg, 2);

	if (ret != 2)
	{
		KdPrint((LOG_PREFIX "EEPROM read failed: %d", ret));
		return 0xff;
	}

	return buf[1];
};

BOOLEAN Netup_EEPROM_Write(PKSDEVICE device, ULONG i2c_adap, UCHAR addr, UCHAR data)
{
	LONG ret;
	UCHAR bufw[2];

	/* Write into EEPROM */
	struct i2c_msg msg;
	
	msg.addr	= EEPROM_I2C_ADDR;
	msg.flags	= 0;
	msg.buf	= &bufw[0];
	msg.len	= 2;

	bufw[0] = addr;
	bufw[1] = data;

	ret = i2c_transfer(device, i2c_adap, &msg, 1);

	if (ret != 1)
		return FALSE;

	DelayMilliseconds(10); /* prophylactic delay, datasheet write cycle time = 5 ms */
	return TRUE;
};

void Netup_Get_Card_Info(PKSDEVICE device, ULONG i2c_adap, struct netup_card_info *cinfo)
{
	ULONG i, j;

	cinfo->rev = Netup_EEPROM_Read(device, i2c_adap, 63);

	for (i = 64, j = 0; i < 70; i++, j++)
		cinfo->mac1[j] =  Netup_EEPROM_Read(device, i2c_adap, i & 0xff);

	for (i = 70, j = 0; i < 76; i++, j++)
		cinfo->mac2[j] =  Netup_EEPROM_Read(device, i2c_adap, i & 0xff);
}