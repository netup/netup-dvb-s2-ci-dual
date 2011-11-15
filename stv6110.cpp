/*
 * stv6110.cpp
 *
 * Driver for ST STV6110 satellite tuner IC.
 *
 * Copyright (C) 2011 NetUP Inc.
 * Copyright (C) 2011 Sergey Kozlov <serjk@netup.ru>
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

#include "stv6110.h"
#include "stv0900.h"
#include "device.h"
#include "i2c.h"

#define CMD_MAX_LEN 256

struct stv6110_priv {
	/* parent ptr */
	PKSDEVICE device;
	/* i2c bus number */
	ULONG i2c;
	/* i2c device address */
	USHORT i2c_address;

	ULONG mclk;
	UCHAR clk_div;
	UCHAR gain;
	UCHAR regs[8];
};

static LONG abssub(LONG a, LONG b)
{
	if (a > b)
		return a - b;
	else
		return b - a;
};

static LONG stv6110_write_regs(stv6110_tuner *fe, UCHAR buf[],
							ULONG start, ULONG len)
{
	stv6110_priv *priv = (stv6110_priv *)(fe->tuner_priv);
	LONG rc;
	UCHAR cmdbuf[CMD_MAX_LEN];
	ASSERT(len + 1 <= CMD_MAX_LEN);
	struct i2c_msg msg;
	msg.addr = priv->i2c_address;
	msg.flags = 0;
	msg.buf	= cmdbuf;
	msg.len	= len + 1;

	KdPrint((LOG_PREFIX "stv6110_write_regs: len=%d", len));

	if (start + len > 8)
		return -1;

	RtlCopyMemory(&cmdbuf[1], buf, len);
	cmdbuf[0] = (UCHAR)start;

	STV0900_I2C_Gate_Ctrl(priv->device, fe->no, 1);

	rc = i2c_transfer(priv->device, priv->i2c, &msg, 1);
	if (rc != 1)
	{
		KdPrint((LOG_PREFIX "stv6110_write_regs: i2c error"));
	}

	STV0900_I2C_Gate_Ctrl(priv->device, fe->no, 0);

	return 0;
}



static LONG stv6110_read_regs(stv6110_tuner *fe, UCHAR regs[],
							ULONG start, ULONG len)
{
	stv6110_priv *priv = (stv6110_priv *)(fe->tuner_priv);
	LONG rc;
	UCHAR reg[] = { (UCHAR)start };
	struct i2c_msg msg[2];
	msg[0].addr	= priv->i2c_address;
	msg[0].flags	= 0;
	msg[0].buf	= reg;
	msg[0].len	= 1;

	msg[1].addr	= priv->i2c_address;
	msg[1].flags	= I2C_M_RD;
	msg[1].buf	= regs;
	msg[1].len	= len;

	STV0900_I2C_Gate_Ctrl(priv->device, fe->no, 1);

	rc = i2c_transfer(priv->device, priv->i2c, msg, 2);
	if (rc != 2)
	{
		KdPrint((LOG_PREFIX "stv6110_write_regs: i2c error"));
	}

	STV0900_I2C_Gate_Ctrl(priv->device, fe->no, 0);

	RtlCopyMemory(&priv->regs[start], regs, len);

	return 0;
}


static ULONG stv6110_read_reg(stv6110_tuner *fe, ULONG start)
{
	UCHAR buf[] = { 0 };
	stv6110_read_regs(fe, buf, start, 1);

	return buf[0];
}


static int stv6110_sleep(struct stv6110_tuner *fe)
{
	UCHAR reg[] = { 0 };
	stv6110_write_regs(fe, reg, 0, 1);

	return 0;
}

#if 0
static ULONG carrier_width(ULONG symbol_rate, fe_rolloff_t rolloff)
{
	ULONG rlf;

	switch (rolloff) {
	case ROLLOFF_20:
		rlf = 20;
		break;
	case ROLLOFF_25:
		rlf = 25;
		break;
	default:
		rlf = 35;
		break;
	}

	return symbol_rate  + ((symbol_rate * rlf) / 100);
}
#endif

static LONG stv6110_set_bandwidth(struct stv6110_tuner *fe, ULONG bandwidth)
{
	struct stv6110_priv *priv = (stv6110_priv *)(fe->tuner_priv);
	UCHAR r8, ret = 0x04;
	ULONG i;

	if ((bandwidth / 2) > 36000000) /*BW/2 max=31+5=36 mhz for r8=31*/
		r8 = 31;
	else if ((bandwidth / 2) < 5000000) /* BW/2 min=5Mhz for F=0 */
		r8 = 0;
	else /*if 5 < BW/2 < 36*/
		r8 = (UCHAR)((bandwidth / 2) / 1000000 - 5);

	/* ctrl3, RCCLKOFF = 0 Activate the calibration Clock */
	/* ctrl3, CF = r8 Set the LPF value */
	priv->regs[RSTV6110_CTRL3] &= ~((1 << 6) | 0x1f);
	priv->regs[RSTV6110_CTRL3] |= (r8 & 0x1f);
	stv6110_write_regs(fe, &priv->regs[RSTV6110_CTRL3], RSTV6110_CTRL3, 1);
	/* stat1, CALRCSTRT = 1 Start LPF auto calibration*/
	priv->regs[RSTV6110_STAT1] |= 0x02;
	stv6110_write_regs(fe, &priv->regs[RSTV6110_STAT1], RSTV6110_STAT1, 1);

	i = 0;
	/* Wait for CALRCSTRT == 0 */
	while ((i < 10) && (ret != 0)) {
		ret = ((stv6110_read_reg(fe, RSTV6110_STAT1)) & 0x02);
		KeStallExecutionProcessor(1);	/* wait for LPF auto calibration */
		i++;
	}

	/* RCCLKOFF = 1 calibration done, desactivate the calibration Clock */
	priv->regs[RSTV6110_CTRL3] |= (1 << 6);
	stv6110_write_regs(fe, &priv->regs[RSTV6110_CTRL3], RSTV6110_CTRL3, 1);
	return 0;
}


static int stv6110_init(struct stv6110_tuner *fe)
{
	struct stv6110_priv *priv = (stv6110_priv *)(fe->tuner_priv);
	UCHAR buf0[] = { 0x07, 0x11, 0xdc, 0x85, 0x17, 0x01, 0xe6, 0x1e };

	RtlCopyMemory(priv->regs, buf0, 8);
	/* K = (Reference / 1000000) - 16 */
	priv->regs[RSTV6110_CTRL1] &= ~(0x1f << 3);
	priv->regs[RSTV6110_CTRL1] |=
				((((priv->mclk / 1000000) - 16) & 0x1f) << 3);

	/* divisor value for the output clock */
	priv->regs[RSTV6110_CTRL2] &= ~0xc0;
	priv->regs[RSTV6110_CTRL2] |= (priv->clk_div << 6);

	stv6110_write_regs(fe, &priv->regs[RSTV6110_CTRL1], RSTV6110_CTRL1, 8);
	KeStallExecutionProcessor(1);
	stv6110_set_bandwidth(fe, 72000000);

	return 0;
}


static LONG stv6110_get_frequency(struct stv6110_tuner *fe, ULONG *frequency)
{
	struct stv6110_priv *priv = (stv6110_priv *)(fe->tuner_priv);
	ULONG nbsteps, divider, psd2, freq;
	UCHAR regs[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

	stv6110_read_regs(fe, regs, 0, 8);
	/*N*/
	divider = (regs[RSTV6110_TUNING2] & 0x0f) << 8;
	divider += regs[RSTV6110_TUNING1];

	/*R*/
	nbsteps  = (regs[RSTV6110_TUNING2] >> 6) & 3;
	/*p*/
	psd2  = (regs[RSTV6110_TUNING2] >> 4) & 1;

	freq = divider * (priv->mclk / 1000);
	freq /= (1 << (nbsteps + psd2));
	freq /= 4;

	*frequency = freq;

	return 0;
}

static LONG stv6110_set_frequency(struct stv6110_tuner *fe, ULONG frequency)
{
	struct stv6110_priv *priv = (stv6110_priv *)(fe->tuner_priv);
	UCHAR ret = 0x04;
	ULONG divider, ref, p, presc, i, result_freq, vco_freq;
	LONG p_calc, p_calc_opt = 1000, r_div, r_div_opt = 0, p_val;

	KdPrint((LOG_PREFIX "stv6110_set_frequency: freq=%d kHz, mclk=%d Hz", frequency, priv->mclk));

	/* K = (Reference / 1000000) - 16 */
	priv->regs[RSTV6110_CTRL1] &= ~(0x1f << 3);
	priv->regs[RSTV6110_CTRL1] |=
				((((priv->mclk / 1000000) - 16) & 0x1f) << 3);

	priv->regs[RSTV6110_CTRL2] &= ~0x0f;
	priv->regs[RSTV6110_CTRL2] |= (priv->gain & 0x0f);

	if (frequency <= 1023000) {
		p = 1;
		presc = 0;
	} else if (frequency <= 1300000) {
		p = 1;
		presc = 1;
	} else if (frequency <= 2046000) {
		p = 0;
		presc = 0;
	} else {
		p = 0;
		presc = 1;
	}
	/* DIV4SEL = p*/
	priv->regs[RSTV6110_TUNING2] &= ~(1 << 4);
	priv->regs[RSTV6110_TUNING2] |= (p << 4);

	/* PRESC32ON = presc */
	priv->regs[RSTV6110_TUNING2] &= ~(1 << 5);
	priv->regs[RSTV6110_TUNING2] |= (presc << 5);

	p_val = (int)(1 << (p + 1)) * 10;/* P = 2 or P = 4 */
	for (r_div = 0; r_div <= 3; r_div++) {
		p_calc = (priv->mclk / 100000);
		p_calc /= (1 << (r_div + 1));
		if ((abssub(p_calc, p_val)) < (abssub(p_calc_opt, p_val)))
			r_div_opt = r_div;

		p_calc_opt = (priv->mclk / 100000);
		p_calc_opt /= (1 << (r_div_opt + 1));
	}

	ref = priv->mclk / ((1 << (r_div_opt + 1))  * (1 << (p + 1)));
	divider = (((frequency * 1000) + (ref >> 1)) / ref);

	/* RDIV = r_div_opt */
	priv->regs[RSTV6110_TUNING2] &= ~(3 << 6);
	priv->regs[RSTV6110_TUNING2] |= (((r_div_opt) & 3) << 6);

	/* NDIV_MSB = MSB(divider) */
	priv->regs[RSTV6110_TUNING2] &= ~0x0f;
	priv->regs[RSTV6110_TUNING2] |= (((divider) >> 8) & 0x0f);

	/* NDIV_LSB, LSB(divider) */
	priv->regs[RSTV6110_TUNING1] = (divider & 0xff);

	/* CALVCOSTRT = 1 VCO Auto Calibration */
	priv->regs[RSTV6110_STAT1] |= 0x04;
	stv6110_write_regs(fe, &priv->regs[RSTV6110_CTRL1],
						RSTV6110_CTRL1, 8);

	i = 0;
	/* Wait for CALVCOSTRT == 0 */
	while ((i < 10) && (ret != 0)) {
		ret = ((stv6110_read_reg(fe, RSTV6110_STAT1)) & 0x04);
		KeStallExecutionProcessor(1); /* wait for VCO auto calibration */
		i++;
	}

	ret = (UCHAR)stv6110_read_reg(fe, RSTV6110_STAT1);
	stv6110_get_frequency(fe, &result_freq);

	vco_freq = divider * ((priv->mclk / 1000) / ((1 << (r_div_opt + 1))));
	KdPrint((LOG_PREFIX "stv6110_set_frequency: stat1=%x, lo_freq=%d kHz, vco_frec=%d kHz", ret, result_freq, vco_freq));

	return 0;
}

#if 0
static int stv6110_set_params(struct dvb_frontend *fe,
			      struct dvb_frontend_parameters *params)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	u32 bandwidth = carrier_width(c->symbol_rate, c->rolloff);

	stv6110_set_frequency(fe, c->frequency);
	stv6110_set_bandwidth(fe, bandwidth);

	return 0;
}
#endif


static LONG stv6110_get_bandwidth(struct stv6110_tuner *fe, ULONG *bandwidth)
{
	struct stv6110_priv *priv = (stv6110_priv *)(fe->tuner_priv);
	UCHAR r8 = 0;
	UCHAR regs[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	stv6110_read_regs(fe, regs, 0, 8);

	/* CF */
	r8 = priv->regs[RSTV6110_CTRL3] & 0x1f;
	*bandwidth = (r8 + 5) * 2000000;/* x2 for ZIF tuner BW/2 = F+5 Mhz */

	return 0;
}

ULONG STV6110_GetFrequency(PKSDEVICE device, ULONG channel)
{
	ASSERT(device != NULL);
	ASSERT(channel < 2);
	ASSERT(GETCONTEXT(device)->stv6110_context[channel] != NULL);
	struct stv6110_tuner * fe = (struct stv6110_tuner *)(GETCONTEXT(device)->stv6110_context[channel]);
	ULONG frequency = 0;
	stv6110_get_frequency(fe, &frequency);
	return frequency;
}

NTSTATUS STV6110_SetFrequency(PKSDEVICE device, ULONG channel, ULONG frequency)
{
	ASSERT(device != NULL);
	ASSERT(channel < 2);
	ASSERT(GETCONTEXT(device)->stv6110_context[channel] != NULL);
	struct stv6110_tuner * fe = (struct stv6110_tuner *)(GETCONTEXT(device)->stv6110_context[channel]);
	if(!stv6110_set_frequency(fe, frequency))
		return STATUS_SUCCESS;
	else
		return STATUS_UNSUCCESSFUL;
}

NTSTATUS STV6110_SetBandwidth(PKSDEVICE device, ULONG channel, ULONG bandwidth)
{
	ASSERT(device != NULL);
	ASSERT(channel < 2);
	ASSERT(GETCONTEXT(device)->stv6110_context[channel] != NULL);
	struct stv6110_tuner * fe = (struct stv6110_tuner *)(GETCONTEXT(device)->stv6110_context[channel]);
	if(!stv6110_set_bandwidth(fe, bandwidth))
		return STATUS_SUCCESS;
	else
		return STATUS_UNSUCCESSFUL;
}

struct stv6110_tuner * STV6110_Attach(PKSDEVICE device, ULONG i2c_bus, ULONG channel, const struct stv6110_config *config)
{
	KdPrint((LOG_PREFIX "STV6110_Attach channel #%d", channel));
	ASSERT(channel < 2);

	struct stv6110_tuner * fe = NULL;
	struct stv6110_priv *priv = NULL;
#if 0
	UCHAR reg0[] = { 0x00, 0x07, 0x11, 0xdc, 0x85, 0x17, 0x01, 0xe6, 0x1e };

	struct i2c_msg msg;
	msg.addr = config->i2c_address;
	msg.flags = 0;
	msg.buf = reg0;
	msg.len = 9;
	LONG ret;

	/* divisor value for the output clock */
	reg0[2] &= ~0xc0;
	reg0[2] |= (config->clk_div << 6);

	STV0900_I2C_Gate_Ctrl(device, channel, 0);

	ret = i2c_transfer(device, i2c_bus, &msg, 1);

	STV0900_I2C_Gate_Ctrl(device, channel, 1);

	if (ret != 1)
	{
		KdPrint((LOG_PREFIX "STV6110_Attach: I2C init failed"));
		return NULL;
	}

#endif
	fe = (stv6110_tuner *)ExAllocatePoolWithTag(NonPagedPool, sizeof(stv6110_tuner), 'CVTS');
	priv = (stv6110_priv *)ExAllocatePoolWithTag(NonPagedPool, sizeof(struct stv6110_priv), 'IVTS');
	RtlZeroMemory(fe, sizeof(stv6110_tuner));
	RtlZeroMemory(priv, sizeof(stv6110_priv));
	if (fe == NULL || priv == NULL)
	{
		KdPrint((LOG_PREFIX "STV6110_Attach: unable to allocate internal structure"));
		return NULL;
	}

	priv->device = device;
	priv->i2c_address = config->i2c_address;
	priv->i2c = i2c_bus;
	priv->mclk = config->mclk;
	priv->clk_div = config->clk_div;
	priv->gain = config->gain;

#if 0
	RtlCopyMemory(&priv->regs, &reg0[1], 8);
#endif

	fe->no = channel;
	fe->tuner_priv = priv;
	stv6110_init(fe);
	DbgPrint(LOG_PREFIX "STV6110 #%d attached on addr=%x!", channel, priv->i2c_address);

	return fe;
}

NTSTATUS STV6110_Init(PKSDEVICE device)
{
	KdPrint((LOG_PREFIX "*** STV6110_Init ***"));
	struct stv6110_config netup_stv6110_tunerconfig_a;
	netup_stv6110_tunerconfig_a.i2c_address = 0x60;
	netup_stv6110_tunerconfig_a.mclk = 16000000;
	netup_stv6110_tunerconfig_a.clk_div = 1;
	netup_stv6110_tunerconfig_a.gain = 8; /* +16 dB  - maximum gain */

	struct stv6110_config netup_stv6110_tunerconfig_b;
	netup_stv6110_tunerconfig_b.i2c_address = 0x63;
	netup_stv6110_tunerconfig_b.mclk = 16000000;
	netup_stv6110_tunerconfig_b.clk_div = 1;
	netup_stv6110_tunerconfig_b.gain = 8; /* +16 dB  - maximum gain */

	stv6110_tuner * tuner0 = STV6110_Attach(device, 0, 0, &netup_stv6110_tunerconfig_a);
	if(!tuner0)
	{
		KdPrint((LOG_PREFIX " unable to attach tuner #0"));
		return STATUS_INSUFFICIENT_RESOURCES;
	}
	stv6110_tuner * tuner1 = STV6110_Attach(device, 0, 1, &netup_stv6110_tunerconfig_b);
	if(!tuner1)
	{
		KdPrint((LOG_PREFIX " unable to attach tuner #1"));
		return STATUS_INSUFFICIENT_RESOURCES;
	}
	
	GETCONTEXT(device)->stv6110_context[0] = tuner0;
	GETCONTEXT(device)->stv6110_context[1] = tuner1;

	KdPrint((LOG_PREFIX "*** STV6110_Init done ***"));

	return STATUS_SUCCESS;
};