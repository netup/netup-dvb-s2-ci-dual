/*
 * stv0900.cpp
 *
 * Driver for ST STV0900 satellite demodulator IC.
 *
 * Copyright (C) ST Microelectronics.
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include "stv0900.h"
#include "stv0900_reg.h"
#include "stv0900_init.h"
#include "i2c.h"
#include "driver.h"
#include "device.h"
#include "stv6110.h"

#undef KdPrint
#define KdPrint(x)

LONG shiftx(LONG x, LONG demod, LONG shift)
{
	if (demod == 1)
		return x - shift;
	return x;
}

LONG ge2comp(LONG a, LONG width)
{
	if (width == 32)
		return a;
	else
		return (a >= (1 << (width - 1))) ? (a - (1 << width)) : a;
}

void stv0900_write_reg(struct stv0900_internal *intp, ULONG reg_addr, UCHAR reg_data)
{
	ASSERT(!(reg_addr & 0xffff0000));
#if 0
	KdPrint((LOG_PREFIX "stv0900_write_reg 0x%x value 0x%02x", reg_addr, reg_data));
#endif

	UCHAR data[3];
	LONG ret;
	struct i2c_msg i2cmsg;
	i2cmsg.addr  = intp->i2c_addr;
	i2cmsg.flags = 0;
	i2cmsg.len   = 3;
	i2cmsg.buf   = data;

	data[0] = MSB(reg_addr);
	data[1] = LSB(reg_addr);
	data[2] = reg_data;

	ret = i2c_transfer(intp->device, intp->i2c_adap, &i2cmsg, 1);
	if (ret != 1)
	{
		KdPrint((LOG_PREFIX "stv0900_write_reg: i2c error %d", ret));
	}
}

UCHAR stv0900_read_reg(struct stv0900_internal *intp, ULONG reg)
{
	ASSERT(!(reg & 0xffff0000));
	LONG ret;
	UCHAR b0[] = { MSB(reg), LSB(reg) };
	UCHAR buf = 0;
	struct i2c_msg msg[2];
	msg[0].addr	= intp->i2c_addr;
	msg[0].flags = 0;
	msg[0].buf = b0;
	msg[0].len = 2;
	msg[1].addr	= intp->i2c_addr;
	msg[1].flags	= I2C_M_RD;
	msg[1].buf = &buf;
	msg[1].len = 1;

	ret = i2c_transfer(intp->device, intp->i2c_adap, msg, 2);
	if (ret != 2)
	{
		KdPrint((LOG_PREFIX "stv0900_read_reg: i2c error %d, reg[0x%02x]", ret, reg));
	}

#if 0
	KdPrint((LOG_PREFIX "stv0900_read_reg 0x%x result 0x%02x", reg, buf));
#endif

	return buf;
}

static void extract_mask_pos(ULONG label, UCHAR *mask, UCHAR *pos)
{
	UCHAR position = 0, i = 0;

	(*mask) = label & 0xff;

	while ((position == 0) && (i < 8)) {
		position = ((*mask) >> i) & 0x01;
		i++;
	}

	(*pos) = (i - 1);
}

void stv0900_write_bits(stv0900_internal *intp, ULONG label, UCHAR val)
{
	UCHAR reg, mask, pos;

	reg = stv0900_read_reg(intp, (label >> 16) & 0xffff);
	extract_mask_pos(label, &mask, &pos);

	val = mask & (val << pos);

	reg = (reg & (~mask)) | val;
	stv0900_write_reg(intp, (label >> 16) & 0xffff, reg);

}

UCHAR stv0900_get_bits(struct stv0900_internal *intp, ULONG label)
{
	UCHAR val = 0xff;
	UCHAR mask, pos;

	extract_mask_pos(label, &mask, &pos);

	val = stv0900_read_reg(intp, label >> 16);
	val = (val & mask) >> pos;

	return val;
}

static ULONG stv0900_get_mclk_freq(struct stv0900_internal *intp, ULONG ext_clk)
{
	ULONG mclk = 90000000, div = 0, ad_div = 0;

	div = stv0900_get_bits(intp, F0900_M_DIV);
	ad_div = ((stv0900_get_bits(intp, F0900_SELX1RATIO) == 1) ? 4 : 6);

	mclk = (div + 1) * ext_clk / ad_div;

	KdPrint((LOG_PREFIX "stv0900_get_mclk_freq: Calculated Mclk = %d",mclk));

	return mclk;
}

static fe_stv0900_error stv0900_set_mclk(struct stv0900_internal *intp, ULONG mclk)
{
	ULONG m_div, clk_sel;

	KdPrint((LOG_PREFIX "stv0900_set_mclk: Mclk set to %d, Quartz = %d", mclk, intp->quartz));

	if (intp == NULL)
		return STV0900_INVALID_HANDLE;

	if (intp->errs)
		return STV0900_I2C_ERROR;

	clk_sel = ((stv0900_get_bits(intp, F0900_SELX1RATIO) == 1) ? 4 : 6);
	m_div = ((clk_sel * mclk) / intp->quartz) - 1;
	stv0900_write_bits(intp, F0900_M_DIV, (UCHAR)m_div);
	intp->mclk = stv0900_get_mclk_freq(intp,
					intp->quartz);

	/*Set the DiseqC frequency to 22KHz */
	/*
		Formula:
		DiseqC_TX_Freq= MasterClock/(32*F22TX_Reg)
		DiseqC_RX_Freq= MasterClock/(32*F22RX_Reg)
	*/
	m_div = intp->mclk / 704000;
	stv0900_write_reg(intp, R0900_P1_F22TX, (UCHAR)m_div);
	stv0900_write_reg(intp, R0900_P1_F22RX, (UCHAR)m_div);

	stv0900_write_reg(intp, R0900_P2_F22TX, (UCHAR)m_div);
	stv0900_write_reg(intp, R0900_P2_F22RX, (UCHAR)m_div);

	if ((intp->errs))
		return STV0900_I2C_ERROR;

	return STV0900_NO_ERROR;
}

static void stv0900_set_tone(struct stv0900_state *state, enum fe_stv0900_demod_num demod, fe_sec_tone_mode_t toneoff)
{
	struct stv0900_internal *intp = state->internal;

	KdPrint(("%s: %s", __FUNCTION__, ((toneoff == 0) ? "On" : "Off")));

	switch (toneoff) {
	case SEC_TONE_ON:
		/*Set the DiseqC mode to 22Khz _continues_ tone*/
		stv0900_write_bits(intp, DISTX_MODE, 0);
		stv0900_write_bits(intp, DISEQC_RESET, 1);
		/*release DiseqC reset to enable the 22KHz tone*/
		stv0900_write_bits(intp, DISEQC_RESET, 0);
		break;
	case SEC_TONE_OFF:
		/*return diseqc mode to config->diseqc_mode.
		Usually it's without _continues_ tone */
		stv0900_write_bits(intp, DISTX_MODE,
				state->config->diseqc_mode);
		/*maintain the DiseqC reset to disable the 22KHz tone*/
		stv0900_write_bits(intp, DISEQC_RESET, 1);
		stv0900_write_bits(intp, DISEQC_RESET, 0);
		break;
	}
}

void stv0900_stop_all_s2_modcod(struct stv0900_internal *intp, fe_stv0900_demod_num demod)
{
	USHORT regflist,i;

	KdPrint((LOG_PREFIX "stv0900_stop_all_s2_modcod()"));

	regflist = (USHORT)(MODCODLST0);

	for (i = 0; i < 16; i++)
		stv0900_write_reg(intp, regflist + i, 0xff);
}

void stv0900_activate_s2_modcod(struct stv0900_internal *intp,
				enum fe_stv0900_demod_num demod)
{
	ULONG matype,
		mod_code,
		fmod,
		reg_index,
		field_index;

	KdPrint((LOG_PREFIX "%s", __FUNCTION__));

	if (intp->chip_id <= 0x11) {
		DelayMilliseconds(5);

		mod_code = stv0900_read_reg(intp, PLHMODCOD);
		matype = mod_code & 0x3;
		mod_code = (mod_code & 0x7f) >> 2;

		reg_index = MODCODLSTF - mod_code / 2;
		field_index = mod_code % 2;

		switch (matype) {
		case 0:
		default:
			fmod = 14;
			break;
		case 1:
			fmod = 13;
			break;
		case 2:
			fmod = 11;
			break;
		case 3:
			fmod = 7;
			break;
		}

		if ((INRANGE(STV0900_QPSK_12, mod_code, STV0900_8PSK_910))
						&& (matype <= 1)) {
			if (field_index == 0)
				stv0900_write_reg(intp, reg_index,
							(UCHAR)(0xf0 | fmod));
			else
				stv0900_write_reg(intp, reg_index,
							(UCHAR)((fmod << 4) | 0xf));
		}

	} else if (intp->chip_id >= 0x12) {
		for (reg_index = 0; reg_index < 7; reg_index++)
			stv0900_write_reg(intp, MODCODLST0 + reg_index, 0xff);

		stv0900_write_reg(intp, MODCODLSTE, 0xff);
		stv0900_write_reg(intp, MODCODLSTF, 0xcf);
		for (reg_index = 0; reg_index < 8; reg_index++)
			stv0900_write_reg(intp, MODCODLST7 + reg_index, 0xcc);


	}
}

void stv0900_activate_s2_modcod_single(struct stv0900_internal *intp,
					fe_stv0900_demod_num demod)
{
	USHORT reg_index;

	KdPrint((LOG_PREFIX "stv0900_activate_s2_modcod_single()"));

	stv0900_write_reg(intp, (USHORT)MODCODLST0, 0xff);
	stv0900_write_reg(intp, (USHORT)MODCODLST1, 0xf0);
	stv0900_write_reg(intp, (USHORT)MODCODLSTF, 0x0f);
	for (reg_index = 0; reg_index < 13; reg_index++)
		stv0900_write_reg(intp, (USHORT)MODCODLST2 + reg_index, 0);
}

static fe_stv0900_error stv0900_initialize(stv0900_internal *intp)
{
	LONG i;

	if (intp == NULL)
		return STV0900_INVALID_HANDLE;

	intp->chip_id = stv0900_read_reg(intp, R0900_MID);

	if (intp->errs != STV0900_NO_ERROR)
		return intp->errs;

	/*Startup sequence*/
	stv0900_write_reg(intp, R0900_P1_DMDISTATE, 0x5c);
	stv0900_write_reg(intp, R0900_P2_DMDISTATE, 0x5c);
	DelayMicroseconds(3);
	stv0900_write_reg(intp, R0900_P1_TNRCFG, 0x6c);
	stv0900_write_reg(intp, R0900_P2_TNRCFG, 0x6f);
	stv0900_write_reg(intp, R0900_P1_I2CRPT, 0x20);
	stv0900_write_reg(intp, R0900_P2_I2CRPT, 0x20);
	stv0900_write_reg(intp, R0900_NCOARSE, 0x13);
	DelayMicroseconds(3);
	stv0900_write_reg(intp, R0900_I2CCFG, 0x08);

	switch (intp->clkmode) {
	case 0:
	case 2:
		stv0900_write_reg(intp, R0900_SYNTCTRL, 0x20
				| intp->clkmode);
		break;
	default:
		/* preserve SELOSCI bit */
		i = 0x02 & stv0900_read_reg(intp, R0900_SYNTCTRL);
		stv0900_write_reg(intp, R0900_SYNTCTRL, 0x20 | (UCHAR)i);
		break;
	}

	DelayMicroseconds(3);
	for (i = 0; i < 181; i++)
		stv0900_write_reg(intp, STV0900_InitVal[i][0],
				(UCHAR)(STV0900_InitVal[i][1]));

	if (stv0900_read_reg(intp, R0900_MID) >= 0x20) {
		stv0900_write_reg(intp, R0900_TSGENERAL, 0x0c);
		for (i = 0; i < 32; i++)
			stv0900_write_reg(intp, STV0900_Cut20_AddOnVal[i][0],
					(UCHAR)(STV0900_Cut20_AddOnVal[i][1]));
	}

	stv0900_write_reg(intp, R0900_P1_FSPYCFG, 0x6c);
	stv0900_write_reg(intp, R0900_P2_FSPYCFG, 0x6c);

	stv0900_write_reg(intp, R0900_P1_PDELCTRL2, 0x01);
	stv0900_write_reg(intp, R0900_P2_PDELCTRL2, 0x21);

	stv0900_write_reg(intp, R0900_P1_PDELCTRL3, 0x20);
	stv0900_write_reg(intp, R0900_P2_PDELCTRL3, 0x20);

	stv0900_write_reg(intp, R0900_TSTRES0, 0x80);
	stv0900_write_reg(intp, R0900_TSTRES0, 0x00);

	return STV0900_NO_ERROR;
}

static void stv0900_set_ts_parallel_serial(struct stv0900_internal *intp,
					fe_stv0900_clock_type path1_ts,
					fe_stv0900_clock_type path2_ts)
{

	KdPrint((LOG_PREFIX "stv0900_set_ts_parallel_serial()"));

	if (intp->chip_id >= 0x20) {
		switch (path1_ts) {
		case STV0900_PARALLEL_PUNCT_CLOCK:
		case STV0900_DVBCI_CLOCK:
			switch (path2_ts) {
			case STV0900_SERIAL_PUNCT_CLOCK:
			case STV0900_SERIAL_CONT_CLOCK:
			default:
				stv0900_write_reg(intp, R0900_TSGENERAL,
							0x00);
				break;
			case STV0900_PARALLEL_PUNCT_CLOCK:
			case STV0900_DVBCI_CLOCK:
				stv0900_write_reg(intp, R0900_TSGENERAL,
							0x06);
				stv0900_write_bits(intp,
						F0900_P1_TSFIFO_MANSPEED, 3);
				stv0900_write_bits(intp,
						F0900_P2_TSFIFO_MANSPEED, 0);
				stv0900_write_reg(intp,
						R0900_P1_TSSPEED, 0x14);
				stv0900_write_reg(intp,
						R0900_P2_TSSPEED, 0x28);
				break;
			}
			break;
		case STV0900_SERIAL_PUNCT_CLOCK:
		case STV0900_SERIAL_CONT_CLOCK:
		default:
			switch (path2_ts) {
			case STV0900_SERIAL_PUNCT_CLOCK:
			case STV0900_SERIAL_CONT_CLOCK:
			default:
				stv0900_write_reg(intp,
						R0900_TSGENERAL, 0x0C);
				break;
			case STV0900_PARALLEL_PUNCT_CLOCK:
			case STV0900_DVBCI_CLOCK:
				stv0900_write_reg(intp,
						R0900_TSGENERAL, 0x0A);
				KdPrint((LOG_PREFIX "stv0900_set_ts_parallel_serial: 0x0a"));
				break;
			}
			break;
		}
	} else {
		switch (path1_ts) {
		case STV0900_PARALLEL_PUNCT_CLOCK:
		case STV0900_DVBCI_CLOCK:
			switch (path2_ts) {
			case STV0900_SERIAL_PUNCT_CLOCK:
			case STV0900_SERIAL_CONT_CLOCK:
			default:
				stv0900_write_reg(intp, R0900_TSGENERAL1X,
							0x10);
				break;
			case STV0900_PARALLEL_PUNCT_CLOCK:
			case STV0900_DVBCI_CLOCK:
				stv0900_write_reg(intp, R0900_TSGENERAL1X,
							0x16);
				stv0900_write_bits(intp,
						F0900_P1_TSFIFO_MANSPEED, 3);
				stv0900_write_bits(intp,
						F0900_P2_TSFIFO_MANSPEED, 0);
				stv0900_write_reg(intp, R0900_P1_TSSPEED,
							0x14);
				stv0900_write_reg(intp, R0900_P2_TSSPEED,
							0x28);
				break;
			}

			break;
		case STV0900_SERIAL_PUNCT_CLOCK:
		case STV0900_SERIAL_CONT_CLOCK:
		default:
			switch (path2_ts) {
			case STV0900_SERIAL_PUNCT_CLOCK:
			case STV0900_SERIAL_CONT_CLOCK:
			default:
				stv0900_write_reg(intp, R0900_TSGENERAL1X,
							0x14);
				break;
			case STV0900_PARALLEL_PUNCT_CLOCK:
			case STV0900_DVBCI_CLOCK:
				stv0900_write_reg(intp, R0900_TSGENERAL1X,
							0x12);
				KdPrint((LOG_PREFIX "stv0900_set_ts_parallel_serial: 0x12"));
				break;
			}

			break;
		}
	}

	switch (path1_ts) {
	case STV0900_PARALLEL_PUNCT_CLOCK:
		stv0900_write_bits(intp, F0900_P1_TSFIFO_SERIAL, 0x00);
		stv0900_write_bits(intp, F0900_P1_TSFIFO_DVBCI, 0x00);
		break;
	case STV0900_DVBCI_CLOCK:
		stv0900_write_bits(intp, F0900_P1_TSFIFO_SERIAL, 0x00);
		stv0900_write_bits(intp, F0900_P1_TSFIFO_DVBCI, 0x01);
		break;
	case STV0900_SERIAL_PUNCT_CLOCK:
		stv0900_write_bits(intp, F0900_P1_TSFIFO_SERIAL, 0x01);
		stv0900_write_bits(intp, F0900_P1_TSFIFO_DVBCI, 0x00);
		break;
	case STV0900_SERIAL_CONT_CLOCK:
		stv0900_write_bits(intp, F0900_P1_TSFIFO_SERIAL, 0x01);
		stv0900_write_bits(intp, F0900_P1_TSFIFO_DVBCI, 0x01);
		break;
	default:
		break;
	}

	switch (path2_ts) {
	case STV0900_PARALLEL_PUNCT_CLOCK:
		stv0900_write_bits(intp, F0900_P2_TSFIFO_SERIAL, 0x00);
		stv0900_write_bits(intp, F0900_P2_TSFIFO_DVBCI, 0x00);
		break;
	case STV0900_DVBCI_CLOCK:
		stv0900_write_bits(intp, F0900_P2_TSFIFO_SERIAL, 0x00);
		stv0900_write_bits(intp, F0900_P2_TSFIFO_DVBCI, 0x01);
		break;
	case STV0900_SERIAL_PUNCT_CLOCK:
		stv0900_write_bits(intp, F0900_P2_TSFIFO_SERIAL, 0x01);
		stv0900_write_bits(intp, F0900_P2_TSFIFO_DVBCI, 0x00);
		break;
	case STV0900_SERIAL_CONT_CLOCK:
		stv0900_write_bits(intp, F0900_P2_TSFIFO_SERIAL, 0x01);
		stv0900_write_bits(intp, F0900_P2_TSFIFO_DVBCI, 0x01);
		break;
	default:
		break;
	}

	stv0900_write_bits(intp, F0900_P2_RST_HWARE, 1);
	stv0900_write_bits(intp, F0900_P2_RST_HWARE, 0);
	stv0900_write_bits(intp, F0900_P1_RST_HWARE, 1);
	stv0900_write_bits(intp, F0900_P1_RST_HWARE, 0);
}

static fe_stv0900_error stv0900_st_dvbs2_single(stv0900_internal *intp, fe_stv0900_demod_mode LDPC_Mode, fe_stv0900_demod_num demod)
{
	fe_stv0900_error error = STV0900_NO_ERROR;
	LONG reg_ind;

	KdPrint((LOG_PREFIX "stv0900_st_dvbs2_single()"));

	switch (LDPC_Mode) {
	case STV0900_DUAL:
	default:
		if ((intp->demod_mode != STV0900_DUAL)
			|| (stv0900_get_bits(intp, F0900_DDEMOD) != 1)) {
			stv0900_write_reg(intp, R0900_GENCFG, 0x1d);

			intp->demod_mode = STV0900_DUAL;

			stv0900_write_bits(intp, F0900_FRESFEC, 1);
			stv0900_write_bits(intp, F0900_FRESFEC, 0);

			for (reg_ind = 0; reg_ind < 7; reg_ind++)
				stv0900_write_reg(intp,
						R0900_P1_MODCODLST0 + (USHORT)reg_ind,
						0xff);
			for (reg_ind = 0; reg_ind < 8; reg_ind++)
				stv0900_write_reg(intp,
						R0900_P1_MODCODLST7 + (USHORT)reg_ind,
						0xcc);

			stv0900_write_reg(intp, R0900_P1_MODCODLSTE, 0xff);
			stv0900_write_reg(intp, R0900_P1_MODCODLSTF, 0xcf);

			for (reg_ind = 0; reg_ind < 7; reg_ind++)
				stv0900_write_reg(intp,
						R0900_P2_MODCODLST0 + (USHORT)reg_ind,
						0xff);
			for (reg_ind = 0; reg_ind < 8; reg_ind++)
				stv0900_write_reg(intp,
						R0900_P2_MODCODLST7 + (USHORT)reg_ind,
						0xcc);

			stv0900_write_reg(intp, R0900_P2_MODCODLSTE, 0xff);
			stv0900_write_reg(intp, R0900_P2_MODCODLSTF, 0xcf);
		}

		break;
	case STV0900_SINGLE:
		if (demod == STV0900_DEMOD_2) {
			stv0900_stop_all_s2_modcod(intp, STV0900_DEMOD_1);
			stv0900_activate_s2_modcod_single(intp,
							STV0900_DEMOD_2);
			stv0900_write_reg(intp, R0900_GENCFG, 0x06);
		} else {
			stv0900_stop_all_s2_modcod(intp, STV0900_DEMOD_2);
			stv0900_activate_s2_modcod_single(intp,
							STV0900_DEMOD_1);
			stv0900_write_reg(intp, R0900_GENCFG, 0x04);
		}

		intp->demod_mode = STV0900_SINGLE;

		stv0900_write_bits(intp, F0900_FRESFEC, 1);
		stv0900_write_bits(intp, F0900_FRESFEC, 0);
		stv0900_write_bits(intp, F0900_P1_ALGOSWRST, 1);
		stv0900_write_bits(intp, F0900_P1_ALGOSWRST, 0);
		stv0900_write_bits(intp, F0900_P2_ALGOSWRST, 1);
		stv0900_write_bits(intp, F0900_P2_ALGOSWRST, 0);
		break;
	}

	return error;
}

static fe_stv0900_error stv0900_init_internal(stv0900_state * state, stv0900_init_params *p_init)
{
	ASSERT(state != NULL);
	ASSERT(p_init != NULL);
	ASSERT(p_init->demod_mode == STV0900_DUAL);

	fe_stv0900_error error = STV0900_NO_ERROR;
	fe_stv0900_error demodError = STV0900_NO_ERROR;
	stv0900_internal *intp = NULL;
	LONG selosci, i;

	if (state->internal) {
		ASSERT(state->internal->dmds_used == 1);
		(state->internal->dmds_used)++;
		KdPrint((LOG_PREFIX "STV0900: Find Internal Structure!"));
		return STV0900_NO_ERROR;
	} else {
		state->internal = (stv0900_internal *)ExAllocatePoolWithTag(NonPagedPool, sizeof(stv0900_internal), 'IVTS');
		if (state->internal == NULL)
			return STV0900_INVALID_HANDLE;
		RtlZeroMemory(state->internal, sizeof(stv0900_internal));
		state->internal->device = state->device;
		state->internal->dmds_used = 1;
		state->internal->i2c_adap = state->i2c_adap;
		state->internal->i2c_addr = state->config->demod_address;
		state->internal->clkmode = state->config->clkmode;
		state->internal->errs = STV0900_NO_ERROR;
		KdPrint((LOG_PREFIX "STV0900: create New Internal Structure"));
	}

	if (state->internal == NULL) {
		error = STV0900_INVALID_HANDLE;
		return error;
	}

	demodError = stv0900_initialize(state->internal);
	if (demodError == STV0900_NO_ERROR) {
			error = STV0900_NO_ERROR;
	} else {
		if (demodError == STV0900_INVALID_HANDLE)
			error = STV0900_INVALID_HANDLE;
		else
			error = STV0900_I2C_ERROR;

		return error;
	}

	intp = state->internal;

	intp->demod_mode = p_init->demod_mode;
	stv0900_st_dvbs2_single(intp, intp->demod_mode,	STV0900_DEMOD_1);
	intp->chip_id = stv0900_read_reg(intp, R0900_MID);
	intp->rolloff = p_init->rolloff;
	intp->quartz = p_init->dmd_ref_clk;

	stv0900_write_bits(intp, F0900_P1_ROLLOFF_CONTROL, p_init->rolloff);
	stv0900_write_bits(intp, F0900_P2_ROLLOFF_CONTROL, p_init->rolloff);

	intp->ts_config = p_init->ts_config;
	if (intp->ts_config == NULL)
		stv0900_set_ts_parallel_serial(intp,
				p_init->path1_ts_clock,
				p_init->path2_ts_clock);
	else {
		for (i = 0; intp->ts_config[i].addr != 0xffff; i++)
		{
			KdPrint((LOG_PREFIX "stv0900_config: addr=0x%x value=0x%x", intp->ts_config[i].addr, intp->ts_config[i].val ));
			stv0900_write_reg(intp,
					intp->ts_config[i].addr,
					intp->ts_config[i].val);
		}

		stv0900_write_bits(intp, F0900_P2_RST_HWARE, 1);
		stv0900_write_bits(intp, F0900_P2_RST_HWARE, 0);
		stv0900_write_bits(intp, F0900_P1_RST_HWARE, 1);
		stv0900_write_bits(intp, F0900_P1_RST_HWARE, 0);
	}

	intp->tuner_type[0] = p_init->tuner1_type;
	intp->tuner_type[1] = p_init->tuner2_type;
	/* tuner init */
	switch (p_init->tuner1_type) {
	case 3: /*FE_AUTO_STB6100:*/
		stv0900_write_reg(intp, R0900_P1_TNRCFG, 0x3c);
		stv0900_write_reg(intp, R0900_P1_TNRCFG2, 0x86);
		stv0900_write_reg(intp, R0900_P1_TNRCFG3, 0x18);
		stv0900_write_reg(intp, R0900_P1_TNRXTAL, 27); /* 27MHz */
		stv0900_write_reg(intp, R0900_P1_TNRSTEPS, 0x05);
		stv0900_write_reg(intp, R0900_P1_TNRGAIN, 0x17);
		stv0900_write_reg(intp, R0900_P1_TNRADJ, 0x1f);
		stv0900_write_reg(intp, R0900_P1_TNRCTL2, 0x0);
		stv0900_write_bits(intp, F0900_P1_TUN_TYPE, 3);
		break;
	/* case FE_SW_TUNER: */
	default:
		KdPrint((LOG_PREFIX "tuner type: default"));
		stv0900_write_bits(intp, F0900_P1_TUN_TYPE, 6);
		break;
	}

	stv0900_write_bits(intp, F0900_P1_TUN_MADDRESS, p_init->tun1_maddress);
	switch (p_init->tuner1_adc) {
	case 1:
		stv0900_write_reg(intp, R0900_TSTTNR1, 0x26);
		break;
	default:
		break;
	}

	stv0900_write_reg(intp, R0900_P1_TNRLD, 1); /* hw tuner */

	/* tuner init */
	switch (p_init->tuner2_type) {
	case 3: /*FE_AUTO_STB6100:*/
		stv0900_write_reg(intp, R0900_P2_TNRCFG, 0x3c);
		stv0900_write_reg(intp, R0900_P2_TNRCFG2, 0x86);
		stv0900_write_reg(intp, R0900_P2_TNRCFG3, 0x18);
		stv0900_write_reg(intp, R0900_P2_TNRXTAL, 27); /* 27MHz */
		stv0900_write_reg(intp, R0900_P2_TNRSTEPS, 0x05);
		stv0900_write_reg(intp, R0900_P2_TNRGAIN, 0x17);
		stv0900_write_reg(intp, R0900_P2_TNRADJ, 0x1f);
		stv0900_write_reg(intp, R0900_P2_TNRCTL2, 0x0);
		stv0900_write_bits(intp, F0900_P2_TUN_TYPE, 3);
		break;
	/* case FE_SW_TUNER: */
	default:
		stv0900_write_bits(intp, F0900_P2_TUN_TYPE, 6);
		break;
	}

	stv0900_write_bits(intp, F0900_P2_TUN_MADDRESS, p_init->tun2_maddress);
	switch (p_init->tuner2_adc) {
	case 1:
		stv0900_write_reg(intp, R0900_TSTTNR3, 0x26);
		break;
	default:
		break;
	}

	stv0900_write_reg(intp, R0900_P2_TNRLD, 1); /* hw tuner */

	stv0900_write_bits(intp, F0900_P1_TUN_IQSWAP, p_init->tun1_iq_inv);
	stv0900_write_bits(intp, F0900_P2_TUN_IQSWAP, p_init->tun2_iq_inv);
	stv0900_set_mclk(intp, 135000000);
	DelayMicroseconds(3);

	switch (intp->clkmode) {
	case 0:
	case 2:
		stv0900_write_reg(intp, R0900_SYNTCTRL, 0x20 | intp->clkmode);
		break;
	default:
		selosci = 0x02 & stv0900_read_reg(intp, R0900_SYNTCTRL);
		stv0900_write_reg(intp, R0900_SYNTCTRL, 0x20 | (USHORT)selosci);
		break;
	}
	DelayMicroseconds(3);

	intp->mclk = stv0900_get_mclk_freq(intp, intp->quartz);
	if (intp->errs)
		error = STV0900_I2C_ERROR;

	stv0900_set_tone(state, STV0900_DEMOD_1, SEC_TONE_OFF);

	return error;
}

static void stv0900_get_lock_timeout(LONG *demod_timeout, LONG *fec_timeout,
					LONG srate,
					enum fe_stv0900_search_algo algo)
{
	switch (algo) {
	case STV0900_BLIND_SEARCH:
		if (srate <= 1500000) {
			(*demod_timeout) = 1500;
			(*fec_timeout) = 400;
		} else if (srate <= 5000000) {
			(*demod_timeout) = 1000;
			(*fec_timeout) = 300;
		} else {
			(*demod_timeout) = 700;
			(*fec_timeout) = 100;
		}

		break;
	case STV0900_COLD_START:
	case STV0900_WARM_START:
	default:
		if (srate <= 1000000) {
			(*demod_timeout) = 3000;
			(*fec_timeout) = 1700;
		} else if (srate <= 2000000) {
			(*demod_timeout) = 2500;
			(*fec_timeout) = 1100;
		} else if (srate <= 5000000) {
			(*demod_timeout) = 1000;
			(*fec_timeout) = 550;
		} else if (srate <= 10000000) {
			(*demod_timeout) = 700;
			(*fec_timeout) = 250;
		} else if (srate <= 20000000) {
			(*demod_timeout) = 400;
			(*fec_timeout) = 130;
		} else {
			(*demod_timeout) = 300;
			(*fec_timeout) = 100;
		}

		break;

	}

	if (algo == STV0900_WARM_START)
		(*demod_timeout) /= 2;
}

static ULONG stv0900_get_symbol_rate(struct stv0900_internal *intp,
					ULONG mclk,
					enum fe_stv0900_demod_num demod)
{
	ULONG	rem1, rem2, intval1, intval2, srate;

	srate = (stv0900_get_bits(intp, SYMB_FREQ3) << 24) +
		(stv0900_get_bits(intp, SYMB_FREQ2) << 16) +
		(stv0900_get_bits(intp, SYMB_FREQ1) << 8) +
		(stv0900_get_bits(intp, SYMB_FREQ0));
	KdPrint((LOG_PREFIX "lock: srate=%d r0=0x%x r1=0x%x r2=0x%x r3=0x%x ",
		srate, stv0900_get_bits(intp, SYMB_FREQ0),
		stv0900_get_bits(intp, SYMB_FREQ1),
		stv0900_get_bits(intp, SYMB_FREQ2),
		stv0900_get_bits(intp, SYMB_FREQ3)));

	intval1 = (mclk) >> 16;
	intval2 = (srate) >> 16;

	rem1 = (mclk) % 0x10000;
	rem2 = (srate) % 0x10000;
	srate =	(intval1 * intval2) +
		((intval1 * rem2) >> 16) +
		((intval2 * rem1) >> 16);

	return srate;
}

static void stv0900_set_symbol_rate(struct stv0900_internal *intp,
					ULONG mclk, ULONG srate,
					enum fe_stv0900_demod_num demod)
{
	ULONG symb;

	KdPrint((LOG_PREFIX "%s Mclk %d, SR %d, Dmd %d\n", __FUNCTION__, mclk,
							srate, demod));

	if (srate > 60000000) {
		symb = srate << 4;
		symb /= (mclk >> 12);
	} else if (srate > 6000000) {
		symb = srate << 6;
		symb /= (mclk >> 10);
	} else {
		symb = srate << 9;
		symb /= (mclk >> 7);
	}

	stv0900_write_reg(intp, SFRINIT1, (symb >> 8) & 0x7f);
	stv0900_write_reg(intp, SFRINIT1 + 1, (symb & 0xff));
}

static void stv0900_set_max_symbol_rate(struct stv0900_internal *intp,
					ULONG mclk, ULONG srate,
					enum fe_stv0900_demod_num demod)
{
	ULONG symb;

	srate = 105 * (srate / 100);

	if (srate > 60000000) {
		symb = srate << 4;
		symb /= (mclk >> 12);
	} else if (srate > 6000000) {
		symb = srate << 6;
		symb /= (mclk >> 10);
	} else {
		symb = srate << 9;
		symb /= (mclk >> 7);
	}

	if (symb < 0x7fff) {
		stv0900_write_reg(intp, SFRUP1, (symb >> 8) & 0x7f);
		stv0900_write_reg(intp, SFRUP1 + 1, (symb & 0xff));
	} else {
		stv0900_write_reg(intp, SFRUP1, 0x7f);
		stv0900_write_reg(intp, SFRUP1 + 1, 0xff);
	}
}

static void stv0900_set_min_symbol_rate(struct stv0900_internal *intp,
					ULONG mclk, ULONG srate,
					enum fe_stv0900_demod_num demod)
{
	ULONG	symb;

	srate = 95 * (srate / 100);
	if (srate > 60000000) {
		symb = srate << 4;
		symb /= (mclk >> 12);

	} else if (srate > 6000000) {
		symb = srate << 6;
		symb /= (mclk >> 10);

	} else {
		symb = srate << 9;
		symb /= (mclk >> 7);
	}

	stv0900_write_reg(intp, SFRLOW1, (symb >> 8) & 0xff);
	stv0900_write_reg(intp, SFRLOW1 + 1, (symb & 0xff));
}

static LONG stv0900_get_timing_offst(struct stv0900_internal *intp,
					LONG srate,
					enum fe_stv0900_demod_num demod)
{
	LONG timingoffset;


	timingoffset = (stv0900_read_reg(intp, TMGREG2) << 16) +
		       (stv0900_read_reg(intp, TMGREG2 + 1) << 8) +
		       (stv0900_read_reg(intp, TMGREG2 + 2));

	timingoffset = ge2comp(timingoffset, 24);


	if (timingoffset == 0)
		timingoffset = 1;

	timingoffset = ((LONG)srate * 10) / ((LONG)0x1000000 / timingoffset);
	timingoffset /= 320;

	return timingoffset;
}

static ULONG stv0900_carrier_width(ULONG srate, enum fe_stv0900_rolloff ro)
{
	ULONG rolloff;

	switch (ro) {
	case STV0900_20:
		rolloff = 20;
		break;
	case STV0900_25:
		rolloff = 25;
		break;
	case STV0900_35:
	default:
		rolloff = 35;
		break;
	}

	return srate  + (srate * rolloff) / 100;
}

static void stv0900_set_dvbs1_track_car_loop(struct stv0900_internal *intp,
					enum fe_stv0900_demod_num demod,
					ULONG srate)
{
	if (intp->chip_id >= 0x30) {
		if (srate >= 15000000) {
			stv0900_write_reg(intp, ACLC, 0x2b);
			stv0900_write_reg(intp, BCLC, 0x1a);
		} else if ((srate >= 7000000) && (15000000 > srate)) {
			stv0900_write_reg(intp, ACLC, 0x0c);
			stv0900_write_reg(intp, BCLC, 0x1b);
		} else if (srate < 7000000) {
			stv0900_write_reg(intp, ACLC, 0x2c);
			stv0900_write_reg(intp, BCLC, 0x1c);
		}

	} else { /*cut 2.0 and 1.x*/
		stv0900_write_reg(intp, ACLC, 0x1a);
		stv0900_write_reg(intp, BCLC, 0x09);
	}

}

static void stv0900_set_viterbi_acq(struct stv0900_internal *intp,
					enum fe_stv0900_demod_num demod)
{
	LONG vth_reg = VTH12;

	KdPrint((LOG_PREFIX "%s", __FUNCTION__));

	stv0900_write_reg(intp, vth_reg++, 0x96);
	stv0900_write_reg(intp, vth_reg++, 0x64);
	stv0900_write_reg(intp, vth_reg++, 0x36);
	stv0900_write_reg(intp, vth_reg++, 0x23);
	stv0900_write_reg(intp, vth_reg++, 0x1e);
	stv0900_write_reg(intp, vth_reg++, 0x19);
}

static void stv0900_set_viterbi_standard(struct stv0900_internal *intp,
				   enum fe_stv0900_search_standard standard,
				   enum fe_stv0900_fec fec,
				   enum fe_stv0900_demod_num demod)
{
	KdPrint((LOG_PREFIX "%s: ViterbiStandard = ", __FUNCTION__));

	switch (standard) {
	case STV0900_AUTO_SEARCH:
		KdPrint((LOG_PREFIX "Auto"));
		stv0900_write_reg(intp, FECM, 0x10);
		stv0900_write_reg(intp, PRVIT, 0x3f);
		break;
	case STV0900_SEARCH_DVBS1:
		KdPrint((LOG_PREFIX "DVBS1"));
		stv0900_write_reg(intp, FECM, 0x00);
		switch (fec) {
		case STV0900_FEC_UNKNOWN:
		default:
			stv0900_write_reg(intp, PRVIT, 0x2f);
			break;
		case STV0900_FEC_1_2:
			stv0900_write_reg(intp, PRVIT, 0x01);
			break;
		case STV0900_FEC_2_3:
			stv0900_write_reg(intp, PRVIT, 0x02);
			break;
		case STV0900_FEC_3_4:
			stv0900_write_reg(intp, PRVIT, 0x04);
			break;
		case STV0900_FEC_5_6:
			stv0900_write_reg(intp, PRVIT, 0x08);
			break;
		case STV0900_FEC_7_8:
			stv0900_write_reg(intp, PRVIT, 0x20);
			break;
		}

		break;
	case STV0900_SEARCH_DSS:
		KdPrint((LOG_PREFIX "DSS"));
		stv0900_write_reg(intp, FECM, 0x80);
		switch (fec) {
		case STV0900_FEC_UNKNOWN:
		default:
			stv0900_write_reg(intp, PRVIT, 0x13);
			break;
		case STV0900_FEC_1_2:
			stv0900_write_reg(intp, PRVIT, 0x01);
			break;
		case STV0900_FEC_2_3:
			stv0900_write_reg(intp, PRVIT, 0x02);
			break;
		case STV0900_FEC_6_7:
			stv0900_write_reg(intp, PRVIT, 0x10);
			break;
		}
		break;
	default:
		break;
	}
}

static void stv0900_set_viterbi_tracq(struct stv0900_internal *intp,
					enum fe_stv0900_demod_num demod)
{

	LONG vth_reg = VTH12;

	KdPrint((LOG_PREFIX "%s", __FUNCTION__));

	stv0900_write_reg(intp, vth_reg++, 0xd0);
	stv0900_write_reg(intp, vth_reg++, 0x7d);
	stv0900_write_reg(intp, vth_reg++, 0x53);
	stv0900_write_reg(intp, vth_reg++, 0x2f);
	stv0900_write_reg(intp, vth_reg++, 0x24);
	stv0900_write_reg(intp, vth_reg++, 0x1f);
}

static void stv0900_set_search_standard(struct stv0900_internal *intp,
					enum fe_stv0900_demod_num demod)
{

	KdPrint(("%s", __FUNCTION__));

	switch (intp->srch_standard[demod]) {
	case STV0900_SEARCH_DVBS1:
		KdPrint((LOG_PREFIX "Search Standard = DVBS1"));
		break;
	case STV0900_SEARCH_DSS:
		KdPrint((LOG_PREFIX "Search Standard = DSS"));
	case STV0900_SEARCH_DVBS2:
		break;
		KdPrint((LOG_PREFIX "Search Standard = DVBS2"));
	case STV0900_AUTO_SEARCH:
	default:
		KdPrint((LOG_PREFIX "Search Standard = AUTO"));
		break;
	}

	switch (intp->srch_standard[demod]) {
	case STV0900_SEARCH_DVBS1:
	case STV0900_SEARCH_DSS:
		stv0900_write_bits(intp, DVBS1_ENABLE, 1);
		stv0900_write_bits(intp, DVBS2_ENABLE, 0);
		stv0900_write_bits(intp, STOP_CLKVIT, 0);
		stv0900_set_dvbs1_track_car_loop(intp,
						demod,
						intp->symbol_rate[demod]);
		stv0900_write_reg(intp, CAR2CFG, 0x22);

		stv0900_set_viterbi_acq(intp, demod);
		stv0900_set_viterbi_standard(intp,
					intp->srch_standard[demod],
					intp->fec[demod], demod);

		break;
	case STV0900_SEARCH_DVBS2:
		stv0900_write_bits(intp, DVBS1_ENABLE, 0);
		stv0900_write_bits(intp, DVBS2_ENABLE, 1);
		stv0900_write_bits(intp, STOP_CLKVIT, 1);
		stv0900_write_reg(intp, ACLC, 0x1a);
		stv0900_write_reg(intp, BCLC, 0x09);
		if (intp->chip_id <= 0x20) /*cut 1.x and 2.0*/
			stv0900_write_reg(intp, CAR2CFG, 0x26);
		else
			stv0900_write_reg(intp, CAR2CFG, 0x66);

		if (intp->demod_mode != STV0900_SINGLE) {
			if (intp->chip_id <= 0x11)
				stv0900_stop_all_s2_modcod(intp, demod);
			else
				stv0900_activate_s2_modcod(intp, demod);

		} else
			stv0900_activate_s2_modcod_single(intp, demod);

		stv0900_set_viterbi_tracq(intp, demod);

		break;
	case STV0900_AUTO_SEARCH:
	default:
		stv0900_write_bits(intp, DVBS1_ENABLE, 1);
		stv0900_write_bits(intp, DVBS2_ENABLE, 1);
		stv0900_write_bits(intp, STOP_CLKVIT, 0);
		stv0900_write_reg(intp, ACLC, 0x1a);
		stv0900_write_reg(intp, BCLC, 0x09);
		stv0900_set_dvbs1_track_car_loop(intp,
						demod,
						intp->symbol_rate[demod]);
		if (intp->chip_id <= 0x20) /*cut 1.x and 2.0*/
			stv0900_write_reg(intp, CAR2CFG, 0x26);
		else
			stv0900_write_reg(intp, CAR2CFG, 0x66);

		if (intp->demod_mode != STV0900_SINGLE) {
			if (intp->chip_id <= 0x11)
				stv0900_stop_all_s2_modcod(intp, demod);
			else
				stv0900_activate_s2_modcod(intp, demod);

		} else
			stv0900_activate_s2_modcod_single(intp, demod);

		stv0900_set_viterbi_tracq(intp, demod);
		stv0900_set_viterbi_standard(intp,
						intp->srch_standard[demod],
						intp->fec[demod], demod);

		break;
	}
}

LONG stv0900_get_demod_lock(struct stv0900_internal *intp,
			enum fe_stv0900_demod_num demod, LONG time_out)
{
	LONG timer = 0,
		lock = 0;

	enum fe_stv0900_search_state	dmd_state;

	while ((timer < time_out) && (lock == 0)) {
		dmd_state = static_cast<fe_stv0900_search_state>(stv0900_get_bits(intp, HEADER_MODE));
		KdPrint((LOG_PREFIX "Demod State = %d", dmd_state));
		switch (dmd_state) {
		case STV0900_SEARCH:
		case STV0900_PLH_DETECTED:
		default:
			lock = 0;
			break;
		case STV0900_DVBS2_FOUND:
		case STV0900_DVBS_FOUND:
			lock = stv0900_get_bits(intp, LOCK_DEFINITIF);
			break;
		}

		if (lock == 0)
			DelayMilliseconds(10);

		timer += 10;
	}

	if (lock)
		KdPrint((LOG_PREFIX "DEMOD LOCK OK"));
	else
		KdPrint((LOG_PREFIX "DEMOD LOCK FAIL"));

	return lock;
}

static LONG stv0900_get_fec_lock(struct stv0900_internal *intp,
				enum fe_stv0900_demod_num demod, LONG time_out)
{
	LONG timer = 0, lock = 0;

	enum fe_stv0900_search_state dmd_state;

	KdPrint((LOG_PREFIX "%s", __FUNCTION__));

	dmd_state = static_cast<fe_stv0900_search_state>(stv0900_get_bits(intp, HEADER_MODE));

	while ((timer < time_out) && (lock == 0)) {
		switch (dmd_state) {
		case STV0900_SEARCH:
		case STV0900_PLH_DETECTED:
		default:
			lock = 0;
			break;
		case STV0900_DVBS2_FOUND:
			lock = stv0900_get_bits(intp, PKTDELIN_LOCK);
			break;
		case STV0900_DVBS_FOUND:
			lock = stv0900_get_bits(intp, LOCKEDVIT);
			break;
		}

		if (lock == 0) {
			DelayMilliseconds(10);
			timer += 10;
		}
	}

	if (lock)
		KdPrint((LOG_PREFIX "%s: DEMOD FEC LOCK OK", __FUNCTION__));
	else
		KdPrint((LOG_PREFIX "%s: DEMOD FEC LOCK FAIL", __FUNCTION__));

	return lock;
}

static LONG stv0900_wait_for_lock(struct stv0900_internal *intp,
				enum fe_stv0900_demod_num demod,
				LONG dmd_timeout, LONG fec_timeout)
{

	LONG timer = 0, lock = 0;

	KdPrint((LOG_PREFIX "%s", __FUNCTION__));

	lock = stv0900_get_demod_lock(intp, demod, dmd_timeout);

	if (lock)
		lock = lock && stv0900_get_fec_lock(intp, demod, fec_timeout);

	if (lock) {
		lock = 0;

		KdPrint((LOG_PREFIX "%s: Timer = %d, time_out = %d\n",
				__FUNCTION__, timer, fec_timeout));

		while ((timer < fec_timeout) && (lock == 0)) {
			lock = stv0900_get_bits(intp, TSFIFO_LINEOK);
			DelayMilliseconds(1);
			timer++;
		}
	}

	if (lock)
		KdPrint((LOG_PREFIX "%s: DEMOD LOCK OK", __FUNCTION__));
	else
		KdPrint((LOG_PREFIX "%s: DEMOD LOCK FAIL", __FUNCTION__));

	if (lock)
		return TRUE;
	else
		return FALSE;
}

enum fe_stv0900_tracking_standard stv0900_get_standard(struct stv0900_state *state,
						enum fe_stv0900_demod_num demod)
{
	struct stv0900_internal *intp = state->internal;
	enum fe_stv0900_tracking_standard fnd_standard;

	int hdr_mode = stv0900_get_bits(intp, HEADER_MODE);

	switch (hdr_mode) {
	case 2:
		fnd_standard = STV0900_DVBS2_STANDARD;
		break;
	case 3:
		if (stv0900_get_bits(intp, DSS_DVB) == 1)
			fnd_standard = STV0900_DSS_STANDARD;
		else
			fnd_standard = STV0900_DVBS1_STANDARD;

		break;
	default:
		fnd_standard = STV0900_UNKNOWN_STANDARD;
	}

	KdPrint((LOG_PREFIX "%s: standard %d", __FUNCTION__, fnd_standard));

	return fnd_standard;
}

static ULONG stv0900_get_tuner_freq(struct stv0900_state *state, enum fe_stv0900_demod_num demod)
{
	ULONG freq = STV6110_GetFrequency(state->device, demod);
	KdPrint(("%s: Frequency=%d", __FUNCTION__, freq));
	return freq;
}


void stv0900_set_bandwidth(struct stv0900_state *state, enum fe_stv0900_demod_num demod, ULONG bandwidth)
{
	KdPrint((LOG_PREFIX "%s: Bandwidth=%d", __FUNCTION__, bandwidth));
	STV6110_SetBandwidth(state->device, demod, bandwidth);
}


ULONG stv0900_get_freq_auto(struct stv0900_internal *intp, int demod)
{
	ULONG freq, round;
	/*	Formulat :
	Tuner_Frequency(MHz)	= Regs / 64
	Tuner_granularity(MHz)	= Regs / 2048
	real_Tuner_Frequency	= Tuner_Frequency(MHz) - Tuner_granularity(MHz)
	*/
	freq = (stv0900_get_bits(intp, TUN_RFFREQ2) << 10) +
		(stv0900_get_bits(intp, TUN_RFFREQ1) << 2) +
		stv0900_get_bits(intp, TUN_RFFREQ0);

	freq = (freq * 1000) / 64;

	round = (stv0900_get_bits(intp, TUN_RFRESTE1) >> 2) +
		stv0900_get_bits(intp, TUN_RFRESTE0);

	round = (round * 1000) / 2048;

	return freq + round;
}

void stv0900_set_tuner_auto(struct stv0900_internal *intp, ULONG Frequency,
						ULONG Bandwidth, LONG demod)
{
	ULONG tunerFrequency;
	/* Formulat:
	Tuner_frequency_reg= Frequency(MHz)*64
	*/
	tunerFrequency = (Frequency * 64) / 1000;

	stv0900_write_bits(intp, TUN_RFFREQ2, (tunerFrequency >> 10) & 0xff);
	stv0900_write_bits(intp, TUN_RFFREQ1, (tunerFrequency >> 2) & 0xff);
	stv0900_write_bits(intp, TUN_RFFREQ0, (tunerFrequency & 0x03));
	/* Low Pass Filter = BW /2 (MHz)*/
	stv0900_write_bits(intp, TUN_BW, (Bandwidth / 2000000) & 0xff);
	/* Tuner Write trig */
	stv0900_write_reg(intp, TNRLD, 1);
}

void stv0900_set_tuner(struct stv0900_state * state, fe_stv0900_demod_num demod, ULONG frequency,
							ULONG bandwidth)
{
	STV6110_SetFrequency(state->device, demod, frequency);
	KdPrint((LOG_PREFIX "%s: Frequency=%d", __FUNCTION__, frequency));
	STV6110_SetBandwidth(state->device, demod, bandwidth);
	KdPrint((LOG_PREFIX "%s: Bandwidth=%d", __FUNCTION__, bandwidth));
}

static LONG stv0900_get_carr_freq(struct stv0900_internal *intp, ULONG mclk,
					enum fe_stv0900_demod_num demod)
{
	LONG	derot,
		rem1,
		rem2,
		intval1,
		intval2;

	derot = (stv0900_get_bits(intp, CAR_FREQ2) << 16) +
		(stv0900_get_bits(intp, CAR_FREQ1) << 8) +
		(stv0900_get_bits(intp, CAR_FREQ0));

	derot = ge2comp(derot, 24);
	intval1 = mclk >> 12;
	intval2 = derot >> 12;
	rem1 = mclk % 0x1000;
	rem2 = derot % 0x1000;
	derot = (intval1 * intval2) +
		((intval1 * rem2) >> 12) +
		((intval2 * rem1) >> 12);

	return derot;
}

static enum fe_stv0900_fec stv0900_get_vit_fec(struct stv0900_internal *intp,
						enum fe_stv0900_demod_num demod)
{
	enum fe_stv0900_fec prate;
	LONG rate_fld = stv0900_get_bits(intp, VIT_CURPUN);

	switch (rate_fld) {
	case 13:
		prate = STV0900_FEC_1_2;
		break;
	case 18:
		prate = STV0900_FEC_2_3;
		break;
	case 21:
		prate = STV0900_FEC_3_4;
		break;
	case 24:
		prate = STV0900_FEC_5_6;
		break;
	case 25:
		prate = STV0900_FEC_6_7;
		break;
	case 26:
		prate = STV0900_FEC_7_8;
		break;
	default:
		prate = STV0900_FEC_UNKNOWN;
		break;
	}

	return prate;
}

static enum
fe_stv0900_signal_type stv0900_get_signal_params(struct stv0900_state *state, enum fe_stv0900_demod_num demod)
{
	struct stv0900_internal *intp = state->internal;
	enum fe_stv0900_signal_type range = STV0900_OUTOFRANGE;
	struct stv0900_signal_info *result = &intp->result[demod];
	LONG	offsetFreq,
		srate_offset;
	LONG	i = 0,
		d = demod;

	UCHAR timing;

	DelayMilliseconds(5);
	if (intp->srch_algo[d] == STV0900_BLIND_SEARCH) {
		timing = stv0900_read_reg(intp, TMGREG2);
		i = 0;
		stv0900_write_reg(intp, SFRSTEP, 0x5c);

		while ((i <= 50) && (timing != 0) && (timing != 0xff)) {
			timing = stv0900_read_reg(intp, TMGREG2);
			DelayMilliseconds(5);
			i += 5;
		}
	}

	result->standard = stv0900_get_standard(state, static_cast<fe_stv0900_demod_num>(d));
	if (intp->tuner_type[demod] == 3)
		result->frequency = stv0900_get_freq_auto(intp, d);
	else
		result->frequency = stv0900_get_tuner_freq(state, static_cast<fe_stv0900_demod_num>(d));

	offsetFreq = stv0900_get_carr_freq(intp, intp->mclk, static_cast<fe_stv0900_demod_num>(d)) / 1000;
	result->frequency += offsetFreq;
	result->symbol_rate = stv0900_get_symbol_rate(intp, intp->mclk, static_cast<fe_stv0900_demod_num>(d));
	srate_offset = stv0900_get_timing_offst(intp, result->symbol_rate, static_cast<fe_stv0900_demod_num>(d));
	result->symbol_rate += srate_offset;
	result->fec = stv0900_get_vit_fec(intp, static_cast<fe_stv0900_demod_num>(d));
	result->modcode = static_cast<fe_stv0900_modcode>(stv0900_get_bits(intp, DEMOD_MODCOD));
	result->pilot = static_cast<fe_stv0900_pilot>(stv0900_get_bits(intp, DEMOD_TYPE) & 0x01);
	result->frame_len = static_cast<fe_stv0900_frame_length>(((ULONG)stv0900_get_bits(intp, DEMOD_TYPE)) >> 1);
	result->rolloff = static_cast<fe_stv0900_rolloff>(stv0900_get_bits(intp, ROLLOFF_STATUS));

	KdPrint((LOG_PREFIX "%s: modcode=0x%x", __FUNCTION__, result->modcode));

	switch (result->standard) {
	case STV0900_DVBS2_STANDARD:
		result->spectrum = static_cast<stv0900_iq_inversion>(stv0900_get_bits(intp, SPECINV_DEMOD));
		if (result->modcode <= STV0900_QPSK_910)
			result->modulation = STV0900_QPSK;
		else if (result->modcode <= STV0900_8PSK_910)
			result->modulation = STV0900_8PSK;
		else if (result->modcode <= STV0900_16APSK_910)
			result->modulation = STV0900_16APSK;
		else if (result->modcode <= STV0900_32APSK_910)
			result->modulation = STV0900_32APSK;
		else
			result->modulation = STV0900_UNKNOWN;
		break;
	case STV0900_DVBS1_STANDARD:
	case STV0900_DSS_STANDARD:
		result->spectrum = static_cast<stv0900_iq_inversion>(stv0900_get_bits(intp, IQINV));
		result->modulation = STV0900_QPSK;
		break;
	default:
		break;
	}

	if ((intp->srch_algo[d] == STV0900_BLIND_SEARCH) ||
				(intp->symbol_rate[d] < 10000000)) {
		offsetFreq = result->frequency - intp->freq[d];
		if (intp->tuner_type[demod] == 3)
			intp->freq[d] = stv0900_get_freq_auto(intp, d);
		else
			intp->freq[d] = stv0900_get_tuner_freq(state, static_cast<fe_stv0900_demod_num>(d));

		if ((ULONG)ABS(offsetFreq) <= ((intp->srch_range[d] / 2000) + 500))
			range = STV0900_RANGEOK;
		else if ((ULONG)(ABS(offsetFreq)) <=
				(stv0900_carrier_width(result->symbol_rate,
						result->rolloff) / 2000))
			range = STV0900_RANGEOK;

	} else if ((ULONG)ABS(offsetFreq) <= ((intp->srch_range[d] / 2000) + 500))
		range = STV0900_RANGEOK;

	KdPrint((LOG_PREFIX "%s: range %d", __FUNCTION__, range));

	return range;
}

UCHAR stv0900_get_optim_carr_loop(LONG srate, enum fe_stv0900_modcode modcode,
							LONG pilot, UCHAR chip_id)
{
	UCHAR aclc_value = 0x29;
	LONG i;
	const struct stv0900_car_loop_optim *cls2, *cllqs2, *cllas2;

	KdPrint((LOG_PREFIX "%s", __FUNCTION__));

	if (chip_id <= 0x12) {
		cls2 = FE_STV0900_S2CarLoop;
		cllqs2 = FE_STV0900_S2LowQPCarLoopCut30;
		cllas2 = FE_STV0900_S2APSKCarLoopCut30;
	} else if (chip_id == 0x20) {
		cls2 = FE_STV0900_S2CarLoopCut20;
		cllqs2 = FE_STV0900_S2LowQPCarLoopCut20;
		cllas2 = FE_STV0900_S2APSKCarLoopCut20;
	} else {
		cls2 = FE_STV0900_S2CarLoopCut30;
		cllqs2 = FE_STV0900_S2LowQPCarLoopCut30;
		cllas2 = FE_STV0900_S2APSKCarLoopCut30;
	}

	if (modcode < STV0900_QPSK_12) {
		i = 0;
		while ((i < 3) && (modcode != cllqs2[i].modcode))
			i++;

		if (i >= 3)
			i = 2;
	} else {
		i = 0;
		while ((i < 14) && (modcode != cls2[i].modcode))
			i++;

		if (i >= 14) {
			i = 0;
			while ((i < 11) && (modcode != cllas2[i].modcode))
				i++;

			if (i >= 11)
				i = 10;
		}
	}

	if (modcode <= STV0900_QPSK_25) {
		if (pilot) {
			if (srate <= 3000000)
				aclc_value = cllqs2[i].car_loop_pilots_on_2;
			else if (srate <= 7000000)
				aclc_value = cllqs2[i].car_loop_pilots_on_5;
			else if (srate <= 15000000)
				aclc_value = cllqs2[i].car_loop_pilots_on_10;
			else if (srate <= 25000000)
				aclc_value = cllqs2[i].car_loop_pilots_on_20;
			else
				aclc_value = cllqs2[i].car_loop_pilots_on_30;
		} else {
			if (srate <= 3000000)
				aclc_value = cllqs2[i].car_loop_pilots_off_2;
			else if (srate <= 7000000)
				aclc_value = cllqs2[i].car_loop_pilots_off_5;
			else if (srate <= 15000000)
				aclc_value = cllqs2[i].car_loop_pilots_off_10;
			else if (srate <= 25000000)
				aclc_value = cllqs2[i].car_loop_pilots_off_20;
			else
				aclc_value = cllqs2[i].car_loop_pilots_off_30;
		}

	} else if (modcode <= STV0900_8PSK_910) {
		if (pilot) {
			if (srate <= 3000000)
				aclc_value = cls2[i].car_loop_pilots_on_2;
			else if (srate <= 7000000)
				aclc_value = cls2[i].car_loop_pilots_on_5;
			else if (srate <= 15000000)
				aclc_value = cls2[i].car_loop_pilots_on_10;
			else if (srate <= 25000000)
				aclc_value = cls2[i].car_loop_pilots_on_20;
			else
				aclc_value = cls2[i].car_loop_pilots_on_30;
		} else {
			if (srate <= 3000000)
				aclc_value = cls2[i].car_loop_pilots_off_2;
			else if (srate <= 7000000)
				aclc_value = cls2[i].car_loop_pilots_off_5;
			else if (srate <= 15000000)
				aclc_value = cls2[i].car_loop_pilots_off_10;
			else if (srate <= 25000000)
				aclc_value = cls2[i].car_loop_pilots_off_20;
			else
				aclc_value = cls2[i].car_loop_pilots_off_30;
		}

	} else {
		if (srate <= 3000000)
			aclc_value = cllas2[i].car_loop_pilots_on_2;
		else if (srate <= 7000000)
			aclc_value = cllas2[i].car_loop_pilots_on_5;
		else if (srate <= 15000000)
			aclc_value = cllas2[i].car_loop_pilots_on_10;
		else if (srate <= 25000000)
			aclc_value = cllas2[i].car_loop_pilots_on_20;
		else
			aclc_value = cllas2[i].car_loop_pilots_on_30;
	}

	return aclc_value;
}

UCHAR stv0900_get_optim_short_carr_loop(LONG srate,
				enum fe_stv0900_modulation modulation,
				UCHAR chip_id)
{
	const struct stv0900_short_frames_car_loop_optim *s2scl;
	const struct stv0900_short_frames_car_loop_optim_vs_mod *s2sclc30;
	LONG mod_index = 0;
	UCHAR aclc_value = 0x0b;

	KdPrint((LOG_PREFIX "%s", __FUNCTION__));

	s2scl = FE_STV0900_S2ShortCarLoop;
	s2sclc30 = FE_STV0900_S2ShortCarLoopCut30;

	switch (modulation) {
	case STV0900_QPSK:
	default:
		mod_index = 0;
		break;
	case STV0900_8PSK:
		mod_index = 1;
		break;
	case STV0900_16APSK:
		mod_index = 2;
		break;
	case STV0900_32APSK:
		mod_index = 3;
		break;
	}

	if (chip_id >= 0x30) {
		if (srate <= 3000000)
			aclc_value = s2sclc30[mod_index].car_loop_2;
		else if (srate <= 7000000)
			aclc_value = s2sclc30[mod_index].car_loop_5;
		else if (srate <= 15000000)
			aclc_value = s2sclc30[mod_index].car_loop_10;
		else if (srate <= 25000000)
			aclc_value = s2sclc30[mod_index].car_loop_20;
		else
			aclc_value = s2sclc30[mod_index].car_loop_30;

	} else if (chip_id >= 0x20) {
		if (srate <= 3000000)
			aclc_value = s2scl[mod_index].car_loop_cut20_2;
		else if (srate <= 7000000)
			aclc_value = s2scl[mod_index].car_loop_cut20_5;
		else if (srate <= 15000000)
			aclc_value = s2scl[mod_index].car_loop_cut20_10;
		else if (srate <= 25000000)
			aclc_value = s2scl[mod_index].car_loop_cut20_20;
		else
			aclc_value = s2scl[mod_index].car_loop_cut20_30;

	} else {
		if (srate <= 3000000)
			aclc_value = s2scl[mod_index].car_loop_cut12_2;
		else if (srate <= 7000000)
			aclc_value = s2scl[mod_index].car_loop_cut12_5;
		else if (srate <= 15000000)
			aclc_value = s2scl[mod_index].car_loop_cut12_10;
		else if (srate <= 25000000)
			aclc_value = s2scl[mod_index].car_loop_cut12_20;
		else
			aclc_value = s2scl[mod_index].car_loop_cut12_30;

	}

	return aclc_value;
}

static void stv0900_track_optimization(struct stv0900_state *state, enum fe_stv0900_demod_num demod)
{
	struct stv0900_internal *intp = state->internal;	

	LONG srate,
		pilots,
		i = 0,
		timed,
		timef,
		blind_tun_sw = 0,
		modulation;
	UCHAR aclc, freq0, freq1;

	enum fe_stv0900_rolloff rolloff;
	enum fe_stv0900_modcode foundModcod;

	KdPrint((LOG_PREFIX "%s", __FUNCTION__));

	srate = stv0900_get_symbol_rate(intp, intp->mclk, demod);
	srate += stv0900_get_timing_offst(intp, srate, demod);

	switch (intp->result[demod].standard) {
	case STV0900_DVBS1_STANDARD:
	case STV0900_DSS_STANDARD:
		KdPrint((LOG_PREFIX "%s: found DVB-S or DSS", __FUNCTION__));
		if (intp->srch_standard[demod] == STV0900_AUTO_SEARCH) {
			stv0900_write_bits(intp, DVBS1_ENABLE, 1);
			stv0900_write_bits(intp, DVBS2_ENABLE, 0);
		}

		stv0900_write_bits(intp, ROLLOFF_CONTROL, intp->rolloff);
		stv0900_write_bits(intp, MANUALSX_ROLLOFF, 1);

		if (intp->chip_id < 0x30) {
			stv0900_write_reg(intp, ERRCTRL1, 0x75);
			break;
		}

		if (stv0900_get_vit_fec(intp, demod) == STV0900_FEC_1_2) {
			stv0900_write_reg(intp, GAUSSR0, 0x98);
			stv0900_write_reg(intp, CCIR0, 0x18);
		} else {
			stv0900_write_reg(intp, GAUSSR0, 0x18);
			stv0900_write_reg(intp, CCIR0, 0x18);
		}

		stv0900_write_reg(intp, ERRCTRL1, 0x75);
		break;
	case STV0900_DVBS2_STANDARD:
		KdPrint((LOG_PREFIX "%s: found DVB-S2", __FUNCTION__));
		stv0900_write_bits(intp, DVBS1_ENABLE, 0);
		stv0900_write_bits(intp, DVBS2_ENABLE, 1);
		stv0900_write_reg(intp, ACLC, 0);
		stv0900_write_reg(intp, BCLC, 0);
		if (intp->result[demod].frame_len == STV0900_LONG_FRAME) {
			foundModcod = static_cast<fe_stv0900_modcode>(stv0900_get_bits(intp, DEMOD_MODCOD));
			pilots = stv0900_get_bits(intp, DEMOD_TYPE) & 0x01;
			aclc = stv0900_get_optim_carr_loop(srate,
							foundModcod,
							pilots,
							intp->chip_id);
			if (foundModcod <= STV0900_QPSK_910)
				stv0900_write_reg(intp, ACLC2S2Q, aclc & 0xff);
			else if (foundModcod <= STV0900_8PSK_910) {
				stv0900_write_reg(intp, ACLC2S2Q, 0x2a);
				stv0900_write_reg(intp, ACLC2S28, aclc & 0xff);
			}

			if ((intp->demod_mode == STV0900_SINGLE) &&
					(foundModcod > STV0900_8PSK_910)) {
				if (foundModcod <= STV0900_16APSK_910) {
					stv0900_write_reg(intp, ACLC2S2Q, 0x2a);
					stv0900_write_reg(intp, ACLC2S216A,
									aclc & 0xff);
				} else if (foundModcod <= STV0900_32APSK_910) {
					stv0900_write_reg(intp, ACLC2S2Q, 0x2a);
					stv0900_write_reg(intp,	ACLC2S232A,
									aclc & 0xff);
				}
			}

		} else {
			modulation = intp->result[demod].modulation;
			aclc = stv0900_get_optim_short_carr_loop(srate,
					static_cast<fe_stv0900_modulation>(modulation), intp->chip_id);
			if (modulation == STV0900_QPSK)
				stv0900_write_reg(intp, ACLC2S2Q, aclc & 0xff);
			else if (modulation == STV0900_8PSK) {
				stv0900_write_reg(intp, ACLC2S2Q, 0x2a);
				stv0900_write_reg(intp, ACLC2S28, aclc & 0xff);
			} else if (modulation == STV0900_16APSK) {
				stv0900_write_reg(intp, ACLC2S2Q, 0x2a);
				stv0900_write_reg(intp, ACLC2S216A, aclc & 0xff);
			} else if (modulation == STV0900_32APSK) {
				stv0900_write_reg(intp, ACLC2S2Q, 0x2a);
				stv0900_write_reg(intp, ACLC2S232A, aclc & 0xff);
			}

		}

		if (intp->chip_id <= 0x11) {
			if (intp->demod_mode != STV0900_SINGLE)
				stv0900_activate_s2_modcod(intp, demod);

		}

		stv0900_write_reg(intp, ERRCTRL1, 0x67);
		break;
	case STV0900_UNKNOWN_STANDARD:
	default:
		KdPrint((LOG_PREFIX "%s: found unknown standard", __FUNCTION__));
		stv0900_write_bits(intp, DVBS1_ENABLE, 1);
		stv0900_write_bits(intp, DVBS2_ENABLE, 1);
		break;
	}

	freq1 = stv0900_read_reg(intp, CFR2);
	freq0 = stv0900_read_reg(intp, CFR1);
	rolloff = static_cast<fe_stv0900_rolloff>(stv0900_get_bits(intp, ROLLOFF_STATUS));
	if (intp->srch_algo[demod] == STV0900_BLIND_SEARCH) {
		stv0900_write_reg(intp, SFRSTEP, 0x00);
		stv0900_write_bits(intp, SCAN_ENABLE, 0);
		stv0900_write_bits(intp, CFR_AUTOSCAN, 0);
		stv0900_write_reg(intp, TMGCFG2, 0xc1);
		stv0900_set_symbol_rate(intp, intp->mclk, srate, demod);
		blind_tun_sw = 1;
		if (intp->result[demod].standard != STV0900_DVBS2_STANDARD)
			stv0900_set_dvbs1_track_car_loop(intp, demod, srate);

	}

	if (intp->chip_id >= 0x20) {
		if ((intp->srch_standard[demod] == STV0900_SEARCH_DVBS1) ||
				(intp->srch_standard[demod] ==
							STV0900_SEARCH_DSS) ||
				(intp->srch_standard[demod] ==
							STV0900_AUTO_SEARCH)) {
			stv0900_write_reg(intp, VAVSRVIT, 0x0a);
			stv0900_write_reg(intp, VITSCALE, 0x0);
		}
	}

	if (intp->chip_id < 0x20)
		stv0900_write_reg(intp, CARHDR, 0x08);

	if (intp->chip_id == 0x10)
		stv0900_write_reg(intp, CORRELEXP, 0x0a);

	stv0900_write_reg(intp, AGC2REF, 0x38);

	if ((intp->chip_id >= 0x20) ||
			(blind_tun_sw == 1) ||
			(intp->symbol_rate[demod] < 10000000)) {
		stv0900_write_reg(intp, CFRINIT1, freq1);
		stv0900_write_reg(intp, CFRINIT0, freq0);
		intp->bw[demod] = stv0900_carrier_width(srate,
					intp->rolloff) + 10000000;

		if ((intp->chip_id >= 0x20) || (blind_tun_sw == 1)) {
			if (intp->srch_algo[demod] != STV0900_WARM_START) {
				if (intp->tuner_type[demod] == 3)
					stv0900_set_tuner_auto(intp,
							intp->freq[demod],
							intp->bw[demod],
							demod);
				else
					stv0900_set_bandwidth(state, demod,
							intp->bw[demod]);
			}
		}

		if ((intp->srch_algo[demod] == STV0900_BLIND_SEARCH) ||
				(intp->symbol_rate[demod] < 10000000))
			DelayMilliseconds(50);
		else
			DelayMilliseconds(5);

		stv0900_get_lock_timeout(&timed, &timef, srate,
						STV0900_WARM_START);

		if (stv0900_get_demod_lock(intp, demod, timed / 2) == FALSE) {
			stv0900_write_reg(intp, DMDISTATE, 0x1f);
			stv0900_write_reg(intp, CFRINIT1, freq1);
			stv0900_write_reg(intp, CFRINIT0, freq0);
			stv0900_write_reg(intp, DMDISTATE, 0x18);
			i = 0;
			while ((stv0900_get_demod_lock(intp,
							demod,
							timed / 2) == FALSE) &&
						(i <= 2)) {
				stv0900_write_reg(intp, DMDISTATE, 0x1f);
				stv0900_write_reg(intp, CFRINIT1, freq1);
				stv0900_write_reg(intp, CFRINIT0, freq0);
				stv0900_write_reg(intp, DMDISTATE, 0x18);
				i++;
			}
		}

	}

	if (intp->chip_id >= 0x20)
		stv0900_write_reg(intp, CARFREQ, 0x49);

	if ((intp->result[demod].standard == STV0900_DVBS1_STANDARD) ||
			(intp->result[demod].standard == STV0900_DSS_STANDARD))
		stv0900_set_viterbi_tracq(intp, demod);

}

static enum
fe_stv0900_signal_type stv0900_dvbs1_acq_workaround(struct stv0900_state *state, enum fe_stv0900_demod_num demod)
{
	struct stv0900_internal *intp = state->internal;
	enum fe_stv0900_signal_type signal_type = STV0900_NODATA;

	LONG	srate,
		demod_timeout,
		fec_timeout;
	UCHAR freq1, freq0;

	intp->result[demod].locked = FALSE;

	if (stv0900_get_bits(intp, HEADER_MODE) == STV0900_DVBS_FOUND) {
		srate = stv0900_get_symbol_rate(intp, intp->mclk, demod);
		srate += stv0900_get_timing_offst(intp, srate, demod);
		if (intp->srch_algo[demod] == STV0900_BLIND_SEARCH)
			stv0900_set_symbol_rate(intp, intp->mclk, srate, demod);

		stv0900_get_lock_timeout(&demod_timeout, &fec_timeout,
					srate, STV0900_WARM_START);
		freq1 = stv0900_read_reg(intp, CFR2);
		freq0 = stv0900_read_reg(intp, CFR1);
		stv0900_write_bits(intp, CFR_AUTOSCAN, 0);
		stv0900_write_bits(intp, SPECINV_CONTROL,
					STV0900_IQ_FORCE_SWAPPED);
		stv0900_write_reg(intp, DMDISTATE, 0x1c);
		stv0900_write_reg(intp, CFRINIT1, freq1);
		stv0900_write_reg(intp, CFRINIT0, freq0);
		stv0900_write_reg(intp, DMDISTATE, 0x18);
		if (stv0900_wait_for_lock(intp, demod,
				demod_timeout, fec_timeout) == TRUE) {
			intp->result[demod].locked = TRUE;
			signal_type = stv0900_get_signal_params(state, demod);
			stv0900_track_optimization(state, demod);
		} else {
			stv0900_write_bits(intp, SPECINV_CONTROL,
					STV0900_IQ_FORCE_NORMAL);
			stv0900_write_reg(intp, DMDISTATE, 0x1c);
			stv0900_write_reg(intp, CFRINIT1, freq1);
			stv0900_write_reg(intp, CFRINIT0, freq0);
			stv0900_write_reg(intp, DMDISTATE, 0x18);
			if (stv0900_wait_for_lock(intp, demod,
					demod_timeout, fec_timeout) == TRUE) {
				intp->result[demod].locked = TRUE;
				signal_type = stv0900_get_signal_params(state, demod);
				stv0900_track_optimization(state, demod);
			}

		}

	} else
		intp->result[demod].locked = FALSE;

	return signal_type;
}

static USHORT stv0900_blind_check_agc2_min_level(struct stv0900_internal *intp,
					enum fe_stv0900_demod_num demod)
{
	ULONG minagc2level = 0xffff,
		agc2level,
		init_freq, freq_step;

	LONG i, j, nb_steps, direction;

	KdPrint(("%s", __FUNCTION__));

	stv0900_write_reg(intp, AGC2REF, 0x38);
	stv0900_write_bits(intp, SCAN_ENABLE, 0);
	stv0900_write_bits(intp, CFR_AUTOSCAN, 0);

	stv0900_write_bits(intp, AUTO_GUP, 1);
	stv0900_write_bits(intp, AUTO_GLOW, 1);

	stv0900_write_reg(intp, DMDT0M, 0x0);

	stv0900_set_symbol_rate(intp, intp->mclk, 1000000, demod);
	nb_steps = -1 + (intp->srch_range[demod] / 1000000);
	nb_steps /= 2;
	nb_steps = (2 * nb_steps) + 1;

	if (nb_steps < 0)
		nb_steps = 1;

	direction = 1;

	freq_step = (1000000 << 8) / (intp->mclk >> 8);

	init_freq = 0;

	for (i = 0; i < nb_steps; i++) {
		if (direction > 0)
			init_freq = init_freq + (freq_step * i);
		else
			init_freq = init_freq - (freq_step * i);

		direction *= -1;
		stv0900_write_reg(intp, DMDISTATE, 0x5C);
		stv0900_write_reg(intp, CFRINIT1, (init_freq >> 8) & 0xff);
		stv0900_write_reg(intp, CFRINIT0, init_freq  & 0xff);
		stv0900_write_reg(intp, DMDISTATE, 0x58);
		DelayMilliseconds(10);
		agc2level = 0;

		for (j = 0; j < 10; j++)
			agc2level += (stv0900_read_reg(intp, AGC2I1) << 8)
					| stv0900_read_reg(intp, AGC2I0);

		agc2level /= 10;

		if (agc2level < minagc2level)
			minagc2level = agc2level;

	}

	return (USHORT)minagc2level;
}

static ULONG stv0900_search_srate_coarse(struct stv0900_state *state, enum fe_stv0900_demod_num demod)
{
	struct stv0900_internal *intp = state->internal;
	LONG timing_lck = FALSE;
	LONG i, timingcpt = 0,
		direction = 1,
		nb_steps,
		current_step = 0,
		tuner_freq;
	ULONG agc2_th,
		coarse_srate = 0,
		agc2_integr = 0,
		currier_step = 1200;

	if (intp->chip_id >= 0x30)
		agc2_th = 0x2e00;
	else
		agc2_th = 0x1f00;

	stv0900_write_bits(intp, DEMOD_MODE, 0x1f);
	stv0900_write_reg(intp, TMGCFG, 0x12);
	stv0900_write_reg(intp, TMGTHRISE, 0xf0);
	stv0900_write_reg(intp, TMGTHFALL, 0xe0);
	stv0900_write_bits(intp, SCAN_ENABLE, 1);
	stv0900_write_bits(intp, CFR_AUTOSCAN, 1);
	stv0900_write_reg(intp, SFRUP1, 0x83);
	stv0900_write_reg(intp, SFRUP0, 0xc0);
	stv0900_write_reg(intp, SFRLOW1, 0x82);
	stv0900_write_reg(intp, SFRLOW0, 0xa0);
	stv0900_write_reg(intp, DMDT0M, 0x0);
	stv0900_write_reg(intp, AGC2REF, 0x50);

	if (intp->chip_id >= 0x30) {
		stv0900_write_reg(intp, CARFREQ, 0x99);
		stv0900_write_reg(intp, SFRSTEP, 0x98);
	} else if (intp->chip_id >= 0x20) {
		stv0900_write_reg(intp, CARFREQ, 0x6a);
		stv0900_write_reg(intp, SFRSTEP, 0x95);
	} else {
		stv0900_write_reg(intp, CARFREQ, 0xed);
		stv0900_write_reg(intp, SFRSTEP, 0x73);
	}

	if (intp->symbol_rate[demod] <= 2000000)
		currier_step = 1000;
	else if (intp->symbol_rate[demod] <= 5000000)
		currier_step = 2000;
	else if (intp->symbol_rate[demod] <= 12000000)
		currier_step = 3000;
	else
			currier_step = 5000;

	nb_steps = -1 + ((intp->srch_range[demod] / 1000) / currier_step);
	nb_steps /= 2;
	nb_steps = (2 * nb_steps) + 1;

	if (nb_steps < 0)
		nb_steps = 1;
	else if (nb_steps > 10) {
		nb_steps = 11;
		currier_step = (intp->srch_range[demod] / 1000) / 10;
	}

	current_step = 0;
	direction = 1;

	tuner_freq = intp->freq[demod];

	while ((timing_lck == FALSE) && (current_step < nb_steps)) {
		stv0900_write_reg(intp, DMDISTATE, 0x5f);
		stv0900_write_bits(intp, DEMOD_MODE, 0);

		DelayMilliseconds(50);

		for (i = 0; i < 10; i++) {
			if (stv0900_get_bits(intp, TMGLOCK_QUALITY) >= 2)
				timingcpt++;

			agc2_integr += (stv0900_read_reg(intp, AGC2I1) << 8) |
					stv0900_read_reg(intp, AGC2I0);
		}

		agc2_integr /= 10;
		coarse_srate = stv0900_get_symbol_rate(intp, intp->mclk, demod);
		current_step++;
		direction *= -1;

		KdPrint((LOG_PREFIX "lock: I2C_DEMOD_MODE_FIELD =0. Search started."
			" tuner freq=%d agc2=0x%x agc2_th=0x%x srate_coarse=%d tmg_cpt=%d",
			tuner_freq, agc2_integr, agc2_th, coarse_srate, timingcpt));

		if ((timingcpt >= 5) &&
				(agc2_integr < agc2_th) &&
				(coarse_srate < 55000000) &&
				(coarse_srate > 850000))
		{
			KdPrint((LOG_PREFIX "timing_lck = TRUE"));
			timing_lck = TRUE;
		}
		else if (current_step < nb_steps) {
			if (direction > 0)
				tuner_freq += (current_step * currier_step);
			else
				tuner_freq -= (current_step * currier_step);

			if (intp->tuner_type[demod] == 3)
				stv0900_set_tuner_auto(intp, tuner_freq,
						intp->bw[demod], demod);
			else
				stv0900_set_tuner(state, demod, tuner_freq,
						intp->bw[demod]);
		}
	}

	if (timing_lck == FALSE)
		coarse_srate = 0;
	else
		coarse_srate = stv0900_get_symbol_rate(intp, intp->mclk, demod);

	return coarse_srate;
}

static ULONG stv0900_search_srate_fine(struct stv0900_state *state, enum fe_stv0900_demod_num demod)
{
	struct stv0900_internal *intp = state->internal;
	ULONG	coarse_srate,
		coarse_freq,
		symb,
		symbmax,
		symbmin,
		symbcomp;

	coarse_srate = stv0900_get_symbol_rate(intp, intp->mclk, demod);

	if (coarse_srate > 3000000) {
		symbmax = 13 * (coarse_srate / 10);
		symbmax = (symbmax / 1000) * 65536;
		symbmax /= (intp->mclk / 1000);

		symbmin = 10 * (coarse_srate / 13);
		symbmin = (symbmin / 1000)*65536;
		symbmin /= (intp->mclk / 1000);

		symb = (coarse_srate / 1000) * 65536;
		symb /= (intp->mclk / 1000);
	} else {
		symbmax = 13 * (coarse_srate / 10);
		symbmax = (symbmax / 100) * 65536;
		symbmax /= (intp->mclk / 100);

		symbmin = 10 * (coarse_srate / 14);
		symbmin = (symbmin / 100) * 65536;
		symbmin /= (intp->mclk / 100);

		symb = (coarse_srate / 100) * 65536;
		symb /= (intp->mclk / 100);
	}

	symbcomp = 13 * (coarse_srate / 10);
		coarse_freq = (stv0900_read_reg(intp, CFR2) << 8)
					| stv0900_read_reg(intp, CFR1);

	KdPrint((LOG_PREFIX "symbcomp %d symbol_rate %d demod %d", symbcomp, (intp->symbol_rate[demod]), (ULONG)demod));
	if (symbcomp < intp->symbol_rate[demod])
		coarse_srate = 0;
	else {
		stv0900_write_reg(intp, DMDISTATE, 0x1f);
		stv0900_write_reg(intp, TMGCFG2, 0xc1);
		stv0900_write_reg(intp, TMGTHRISE, 0x20);
		stv0900_write_reg(intp, TMGTHFALL, 0x00);
		stv0900_write_reg(intp, TMGCFG, 0xd2);
		stv0900_write_bits(intp, CFR_AUTOSCAN, 0);
		stv0900_write_reg(intp, AGC2REF, 0x38);

		if (intp->chip_id >= 0x30)
			stv0900_write_reg(intp, CARFREQ, 0x79);
		else if (intp->chip_id >= 0x20)
			stv0900_write_reg(intp, CARFREQ, 0x49);
		else
			stv0900_write_reg(intp, CARFREQ, 0xed);

		stv0900_write_reg(intp, SFRUP1, (symbmax >> 8) & 0x7f);
		stv0900_write_reg(intp, SFRUP0, (symbmax & 0xff));

		stv0900_write_reg(intp, SFRLOW1, (symbmin >> 8) & 0x7f);
		stv0900_write_reg(intp, SFRLOW0, (symbmin & 0xff));

		stv0900_write_reg(intp, SFRINIT1, (symb >> 8) & 0xff);
		stv0900_write_reg(intp, SFRINIT0, (symb & 0xff));

		stv0900_write_reg(intp, DMDT0M, 0x20);
		stv0900_write_reg(intp, CFRINIT1, (coarse_freq >> 8) & 0xff);
		stv0900_write_reg(intp, CFRINIT0, coarse_freq  & 0xff);
		stv0900_write_reg(intp, DMDISTATE, 0x15);
	}

	return coarse_srate;
}

static LONG stv0900_blind_search_algo(struct stv0900_state *state, enum fe_stv0900_demod_num demod)
{
	struct stv0900_internal *intp = state->internal;
	UCHAR	k_ref_tmg,
		k_ref_tmg_max,
		k_ref_tmg_min;
	ULONG	coarse_srate,
		agc2_th;
	LONG	lock = FALSE,
		coarse_fail = FALSE;
	LONG	demod_timeout = 500,
		fec_timeout = 50,
		fail_cpt,
		i,
		agc2_overflow;
	USHORT	agc2_int;
	UCHAR	dstatus2;

	KdPrint((LOG_PREFIX "%s", __FUNCTION__));

	if (intp->chip_id < 0x20) {
		k_ref_tmg_max = 233;
		k_ref_tmg_min = 143;
	} else {
		k_ref_tmg_max = 110;
		k_ref_tmg_min = 10;
	}

	if (intp->chip_id <= 0x20)
		agc2_th = STV0900_BLIND_SEARCH_AGC2_TH;
	else
		agc2_th = STV0900_BLIND_SEARCH_AGC2_TH_CUT30;

	agc2_int = stv0900_blind_check_agc2_min_level(intp, demod);

	KdPrint((LOG_PREFIX "%s agc2_int=%d agc2_th=%d", __FUNCTION__, agc2_int, agc2_th));
	if (agc2_int > agc2_th)
		return FALSE;

	if (intp->chip_id == 0x10)
		stv0900_write_reg(intp, CORRELEXP, 0xaa);

	if (intp->chip_id < 0x20)
		stv0900_write_reg(intp, CARHDR, 0x55);
	else
		stv0900_write_reg(intp, CARHDR, 0x20);

	if (intp->chip_id <= 0x20)
		stv0900_write_reg(intp, CARCFG, 0xc4);
	else
		stv0900_write_reg(intp, CARCFG, 0x6);

	stv0900_write_reg(intp, RTCS2, 0x44);

	if (intp->chip_id >= 0x20) {
		stv0900_write_reg(intp, EQUALCFG, 0x41);
		stv0900_write_reg(intp, FFECFG, 0x41);
		stv0900_write_reg(intp, VITSCALE, 0x82);
		stv0900_write_reg(intp, VAVSRVIT, 0x0);
	}

	k_ref_tmg = k_ref_tmg_max;

	do {
		stv0900_write_reg(intp, KREFTMG, k_ref_tmg);
		if (stv0900_search_srate_coarse(state, demod) != 0) {
			KdPrint((LOG_PREFIX "stv0900_search_srate_coarse(state, demod) != 0"));
			coarse_srate = stv0900_search_srate_fine(state, demod);
			KdPrint((LOG_PREFIX "coarse_srate=%d", coarse_srate));
			if (coarse_srate != 0) {
				stv0900_get_lock_timeout(&demod_timeout,
							&fec_timeout,
							coarse_srate,
							STV0900_BLIND_SEARCH);
				lock = stv0900_get_demod_lock(intp,
							demod,
							demod_timeout);
				KdPrint((LOG_PREFIX "lock=%d", lock));
			} else
				lock = FALSE;
		} else {
			fail_cpt = 0;
			agc2_overflow = 0;

			for (i = 0; i < 10; i++) {
				agc2_int = (stv0900_read_reg(intp, AGC2I1) << 8)
					| stv0900_read_reg(intp, AGC2I0);

				if (agc2_int >= 0xff00)
					agc2_overflow++;

				dstatus2 = stv0900_read_reg(intp, DSTATUS2);

				if (((dstatus2 & 0x1) == 0x1) &&
						((dstatus2 >> 7) == 1))
					fail_cpt++;
			}

			if ((fail_cpt > 7) || (agc2_overflow > 7))
				coarse_fail = TRUE;

			lock = FALSE;
		}
		k_ref_tmg -= 30;
	} while ((k_ref_tmg >= k_ref_tmg_min) &&
				(lock == FALSE) &&
				(coarse_fail == FALSE));

	return lock;
}

void stv0900_start_search(struct stv0900_internal *intp,
				enum fe_stv0900_demod_num demod)
{
	ULONG freq;
	SHORT freq_s16 ;

	stv0900_write_bits(intp, DEMOD_MODE, 0x1f);
	if (intp->chip_id == 0x10)
		stv0900_write_reg(intp, CORRELEXP, 0xaa);

	if (intp->chip_id < 0x20)
		stv0900_write_reg(intp, CARHDR, 0x55);

	if (intp->chip_id <= 0x20) {
		if (intp->symbol_rate[0] <= 5000000) {
			stv0900_write_reg(intp, CARCFG, 0x44);
			stv0900_write_reg(intp, CFRUP1, 0x0f);
			stv0900_write_reg(intp, CFRUP0, 0xff);
			stv0900_write_reg(intp, CFRLOW1, 0xf0);
			stv0900_write_reg(intp, CFRLOW0, 0x00);
			stv0900_write_reg(intp, RTCS2, 0x68);
		} else {
			stv0900_write_reg(intp, CARCFG, 0xc4);
			stv0900_write_reg(intp, RTCS2, 0x44);
		}

	} else { /*cut 3.0 above*/
		if (intp->symbol_rate[demod] <= 5000000)
			stv0900_write_reg(intp, RTCS2, 0x68);
		else
			stv0900_write_reg(intp, RTCS2, 0x44);

		stv0900_write_reg(intp, CARCFG, 0x46);
		if (intp->srch_algo[demod] == STV0900_WARM_START) {
			freq = 1000 << 16;
			freq /= (intp->mclk / 1000);
			freq_s16 = (SHORT)freq;
		} else {
			freq = (intp->srch_range[demod] / 2000);
			if (intp->symbol_rate[demod] <= 5000000)
				freq += 80;
			else
				freq += 600;

			freq = freq << 16;
			freq /= (intp->mclk / 1000);
			freq_s16 = (SHORT)freq;
		}

		stv0900_write_bits(intp, CFR_UP1, MSB(freq_s16));
		stv0900_write_bits(intp, CFR_UP0, LSB(freq_s16));
		freq_s16 *= (-1);
		stv0900_write_bits(intp, CFR_LOW1, MSB(freq_s16));
		stv0900_write_bits(intp, CFR_LOW0, LSB(freq_s16));
	}

	stv0900_write_reg(intp, CFRINIT1, 0);
	stv0900_write_reg(intp, CFRINIT0, 0);

	if (intp->chip_id >= 0x20) {
		stv0900_write_reg(intp, EQUALCFG, 0x41);
		stv0900_write_reg(intp, FFECFG, 0x41);

		if ((intp->srch_standard[demod] == STV0900_SEARCH_DVBS1) ||
			(intp->srch_standard[demod] == STV0900_SEARCH_DSS) ||
			(intp->srch_standard[demod] == STV0900_AUTO_SEARCH)) {
			stv0900_write_reg(intp, VITSCALE,
								0x82);
			stv0900_write_reg(intp, VAVSRVIT, 0x0);
		}
	}

	stv0900_write_reg(intp, SFRSTEP, 0x00);
	stv0900_write_reg(intp, TMGTHRISE, 0xe0);
	stv0900_write_reg(intp, TMGTHFALL, 0xc0);
	stv0900_write_bits(intp, SCAN_ENABLE, 0);
	stv0900_write_bits(intp, CFR_AUTOSCAN, 0);
	stv0900_write_bits(intp, S1S2_SEQUENTIAL, 0);
	stv0900_write_reg(intp, RTC, 0x88);
	if (intp->chip_id >= 0x20) {
		if (intp->symbol_rate[demod] < 2000000) {
			if (intp->chip_id <= 0x20)
				stv0900_write_reg(intp, CARFREQ, 0x39);
			else  /*cut 3.0*/
				stv0900_write_reg(intp, CARFREQ, 0x89);

			stv0900_write_reg(intp, CARHDR, 0x40);
		} else if (intp->symbol_rate[demod] < 10000000) {
			stv0900_write_reg(intp, CARFREQ, 0x4c);
			stv0900_write_reg(intp, CARHDR, 0x20);
		} else {
			stv0900_write_reg(intp, CARFREQ, 0x4b);
			stv0900_write_reg(intp, CARHDR, 0x20);
		}

	} else {
		if (intp->symbol_rate[demod] < 10000000)
			stv0900_write_reg(intp, CARFREQ, 0xef);
		else
			stv0900_write_reg(intp, CARFREQ, 0xed);
	}

	switch (intp->srch_algo[demod]) {
	case STV0900_WARM_START:
		stv0900_write_reg(intp, DMDISTATE, 0x1f);
		stv0900_write_reg(intp, DMDISTATE, 0x18);
		break;
	case STV0900_COLD_START:
		stv0900_write_reg(intp, DMDISTATE, 0x1f);
		stv0900_write_reg(intp, DMDISTATE, 0x15);
		break;
	default:
		break;
	}
}

static int stv0900_check_timing_lock(struct stv0900_internal *intp,
				enum fe_stv0900_demod_num demod)
{
	LONG timingLock = FALSE;
	LONG	i,
		timingcpt = 0;
	UCHAR	car_freq,
		tmg_th_high,
		tmg_th_low;

	car_freq = stv0900_read_reg(intp, CARFREQ);
	tmg_th_high = stv0900_read_reg(intp, TMGTHRISE);
	tmg_th_low = stv0900_read_reg(intp, TMGTHFALL);
	stv0900_write_reg(intp, TMGTHRISE, 0x20);
	stv0900_write_reg(intp, TMGTHFALL, 0x0);
	stv0900_write_bits(intp, CFR_AUTOSCAN, 0);
	stv0900_write_reg(intp, RTC, 0x80);
	stv0900_write_reg(intp, RTCS2, 0x40);
	stv0900_write_reg(intp, CARFREQ, 0x0);
	stv0900_write_reg(intp, CFRINIT1, 0x0);
	stv0900_write_reg(intp, CFRINIT0, 0x0);
	stv0900_write_reg(intp, AGC2REF, 0x65);
	stv0900_write_reg(intp, DMDISTATE, 0x18);
	DelayMilliseconds(7);

	for (i = 0; i < 10; i++) {
		if (stv0900_get_bits(intp, TMGLOCK_QUALITY) >= 2)
			timingcpt++;

		DelayMilliseconds(1);
	}

	if (timingcpt >= 3)
		timingLock = TRUE;

	stv0900_write_reg(intp, AGC2REF, 0x38);
	stv0900_write_reg(intp, RTC, 0x88);
	stv0900_write_reg(intp, RTCS2, 0x68);
	stv0900_write_reg(intp, CARFREQ, car_freq);
	stv0900_write_reg(intp, TMGTHRISE, tmg_th_high);
	stv0900_write_reg(intp, TMGTHFALL, tmg_th_low);

	return	timingLock;
}

static LONG stv0900_get_demod_cold_lock(struct stv0900_state *state, enum fe_stv0900_demod_num demod,
					LONG demod_timeout)
{
	struct stv0900_internal *intp = state->internal;
	LONG	lock = FALSE,
		d = demod;
	LONG	srate,
		search_range,
		locktimeout,
		currier_step,
		nb_steps,
		current_step,
		direction,
		tuner_freq,
		timeout,
		freq;

	srate = intp->symbol_rate[d];
	search_range = intp->srch_range[d];

	if (srate >= 10000000)
		locktimeout = demod_timeout / 3;
	else
		locktimeout = demod_timeout / 2;

	lock = stv0900_get_demod_lock(intp, static_cast<fe_stv0900_demod_num>(d), locktimeout);

	if (lock != FALSE)
		return lock;

	if (srate >= 10000000) {
		if (stv0900_check_timing_lock(intp, demod) == TRUE) {
			stv0900_write_reg(intp, DMDISTATE, 0x1f);
			stv0900_write_reg(intp, DMDISTATE, 0x15);
			lock = stv0900_get_demod_lock(intp, demod, demod_timeout);
		} else
			lock = FALSE;

		return lock;
	}

	if (intp->chip_id <= 0x20) {
		if (srate <= 1000000)
			currier_step = 500;
		else if (srate <= 4000000)
			currier_step = 1000;
		else if (srate <= 7000000)
			currier_step = 2000;
		else if (srate <= 10000000)
			currier_step = 3000;
		else
			currier_step = 5000;

		if (srate >= 2000000) {
			timeout = (demod_timeout / 3);
			if (timeout > 1000)
				timeout = 1000;
		} else
			timeout = (demod_timeout / 2);
	} else {
		/*cut 3.0 */
		currier_step = srate / 4000;
		timeout = (demod_timeout * 3) / 4;
	}

	nb_steps = ((search_range / 1000) / currier_step);

	if ((nb_steps % 2) != 0)
		nb_steps += 1;

	if (nb_steps <= 0)
		nb_steps = 2;
	else if (nb_steps > 12)
		nb_steps = 12;

	current_step = 1;
	direction = 1;

	if (intp->chip_id <= 0x20) {
		tuner_freq = intp->freq[d];
		intp->bw[d] = stv0900_carrier_width(intp->symbol_rate[d],
				intp->rolloff) + intp->symbol_rate[d];
	} else
		tuner_freq = 0;

	while ((current_step <= nb_steps) && (lock == FALSE)) {
		if (direction > 0)
			tuner_freq += (current_step * currier_step);
		else
			tuner_freq -= (current_step * currier_step);

		if (intp->chip_id <= 0x20) {
			if (intp->tuner_type[d] == 3)
				stv0900_set_tuner_auto(intp, tuner_freq,
						intp->bw[d], demod);
			else
				stv0900_set_tuner(state, demod, tuner_freq, intp->bw[d]);

			stv0900_write_reg(intp, DMDISTATE, 0x1c);
			stv0900_write_reg(intp, CFRINIT1, 0);
			stv0900_write_reg(intp, CFRINIT0, 0);
			stv0900_write_reg(intp, DMDISTATE, 0x1f);
			stv0900_write_reg(intp, DMDISTATE, 0x15);
		} else {
			stv0900_write_reg(intp, DMDISTATE, 0x1c);
			freq = (tuner_freq * 65536) / (intp->mclk / 1000);
			stv0900_write_bits(intp, CFR_INIT1, MSB(freq));
			stv0900_write_bits(intp, CFR_INIT0, LSB(freq));
			stv0900_write_reg(intp, DMDISTATE, 0x1f);
			stv0900_write_reg(intp, DMDISTATE, 0x05);
		}

		lock = stv0900_get_demod_lock(intp, demod, timeout);
		direction *= -1;
		current_step++;
	}

	return	lock;
}

LONG stv0900_check_signal_presence(struct stv0900_internal *intp,
					enum fe_stv0900_demod_num demod)
{
	LONG	carr_offset,
		agc2_integr,
		max_carrier;

	LONG no_signal = FALSE;

	carr_offset = (stv0900_read_reg(intp, CFR2) << 8)
					| stv0900_read_reg(intp, CFR1);
	carr_offset = ge2comp(carr_offset, 16);
	agc2_integr = (stv0900_read_reg(intp, AGC2I1) << 8)
					| stv0900_read_reg(intp, AGC2I0);
	max_carrier = intp->srch_range[demod] / 1000;

	max_carrier += (max_carrier / 10);
	max_carrier = 65536 * (max_carrier / 2);
	max_carrier /= intp->mclk / 1000;
	if (max_carrier > 0x4000)
		max_carrier = 0x4000;

	if ((agc2_integr > 0x2000)
			|| (carr_offset > (2 * max_carrier))
			|| (carr_offset < (-2 * max_carrier)))
		no_signal = TRUE;

	return no_signal;
}

static void stv0900_set_dvbs2_rolloff(struct stv0900_internal *intp,
					enum fe_stv0900_demod_num demod)
{
	UCHAR rolloff;

	if (intp->chip_id == 0x10) {
		stv0900_write_bits(intp, MANUALSX_ROLLOFF, 1);
		rolloff = stv0900_read_reg(intp, MATSTR1) & 0x03;
		stv0900_write_bits(intp, ROLLOFF_CONTROL, rolloff);
	} else if (intp->chip_id <= 0x20)
		stv0900_write_bits(intp, MANUALSX_ROLLOFF, 0);
	else /* cut 3.0 */
		stv0900_write_bits(intp, MANUALS2_ROLLOFF, 0);
}

static LONG stv0900_search_carr_sw_loop(struct stv0900_internal *intp,
				LONG FreqIncr, LONG Timeout, int zigzag,
				LONG MaxStep, enum fe_stv0900_demod_num demod)
{
	LONG	no_signal,
		lock = FALSE;
	LONG	stepCpt,
		freqOffset,
		max_carrier;

	max_carrier = intp->srch_range[demod] / 1000;
	max_carrier += (max_carrier / 10);

	max_carrier = 65536 * (max_carrier / 2);
	max_carrier /= intp->mclk / 1000;

	if (max_carrier > 0x4000)
		max_carrier = 0x4000;

	if (zigzag == TRUE)
		freqOffset = 0;
	else
		freqOffset = -max_carrier + FreqIncr;

	stepCpt = 0;

	do {
		stv0900_write_reg(intp, DMDISTATE, 0x1c);
		stv0900_write_reg(intp, CFRINIT1, (freqOffset / 256) & 0xff);
		stv0900_write_reg(intp, CFRINIT0, freqOffset & 0xff);
		stv0900_write_reg(intp, DMDISTATE, 0x18);
		stv0900_write_bits(intp, ALGOSWRST, 1);

		if (intp->chip_id == 0x12) {
			stv0900_write_bits(intp, RST_HWARE, 1);
			stv0900_write_bits(intp, RST_HWARE, 0);
		}

		if (zigzag == TRUE) {
			if (freqOffset >= 0)
				freqOffset = -freqOffset - 2 * FreqIncr;
			else
				freqOffset = -freqOffset;
		} else
			freqOffset += + 2 * FreqIncr;

		stepCpt++;
		lock = stv0900_get_demod_lock(intp, demod, Timeout);
		no_signal = stv0900_check_signal_presence(intp, demod);

	} while ((lock == FALSE)
			&& (no_signal == FALSE)
			&& ((freqOffset - FreqIncr) <  max_carrier)
			&& ((freqOffset + FreqIncr) > -max_carrier)
			&& (stepCpt < MaxStep));

	stv0900_write_bits(intp, ALGOSWRST, 0);

	return lock;
}

static void stv0900_get_sw_loop_params(struct stv0900_internal *intp,
				LONG *frequency_inc, LONG *sw_timeout,
				LONG *steps,
				enum fe_stv0900_demod_num demod)
{
	LONG timeout, freq_inc, max_steps, srate, max_carrier;

	enum fe_stv0900_search_standard	standard;

	srate = intp->symbol_rate[demod];
	max_carrier = intp->srch_range[demod] / 1000;
	max_carrier += max_carrier / 10;
	standard = intp->srch_standard[demod];

	max_carrier = 65536 * (max_carrier / 2);
	max_carrier /= intp->mclk / 1000;

	if (max_carrier > 0x4000)
		max_carrier = 0x4000;

	freq_inc = srate;
	freq_inc /= intp->mclk >> 10;
	freq_inc = freq_inc << 6;

	switch (standard) {
	case STV0900_SEARCH_DVBS1:
	case STV0900_SEARCH_DSS:
		freq_inc *= 3;
		timeout = 20;
		break;
	case STV0900_SEARCH_DVBS2:
		freq_inc *= 4;
		timeout = 25;
		break;
	case STV0900_AUTO_SEARCH:
	default:
		freq_inc *= 3;
		timeout = 25;
		break;
	}

	freq_inc /= 100;

	if ((freq_inc > max_carrier) || (freq_inc < 0))
		freq_inc = max_carrier / 2;

	timeout *= 27500;

	if (srate > 0)
		timeout /= srate / 1000;

	if ((timeout > 100) || (timeout < 0))
		timeout = 100;

	max_steps = (max_carrier / freq_inc) + 1;

	if ((max_steps > 100) || (max_steps < 0)) {
		max_steps =  100;
		freq_inc = max_carrier / max_steps;
	}

	*frequency_inc = freq_inc;
	*sw_timeout = timeout;
	*steps = max_steps;

}

static LONG stv0900_sw_algo(struct stv0900_internal *intp,
				enum fe_stv0900_demod_num demod)
{
	LONG	lock = FALSE,
		no_signal,
		zigzag;
	LONG	s2fw,
		fqc_inc,
		sft_stp_tout,
		trial_cntr,
		max_steps;

	stv0900_get_sw_loop_params(intp, &fqc_inc, &sft_stp_tout,
					&max_steps, demod);
	switch (intp->srch_standard[demod]) {
	case STV0900_SEARCH_DVBS1:
	case STV0900_SEARCH_DSS:
		if (intp->chip_id >= 0x20)
			stv0900_write_reg(intp, CARFREQ, 0x3b);
		else
			stv0900_write_reg(intp, CARFREQ, 0xef);

		stv0900_write_reg(intp, DMDCFGMD, 0x49);
		zigzag = FALSE;
		break;
	case STV0900_SEARCH_DVBS2:
		if (intp->chip_id >= 0x20)
			stv0900_write_reg(intp, CORRELABS, 0x79);
		else
			stv0900_write_reg(intp, CORRELABS, 0x68);

		stv0900_write_reg(intp, DMDCFGMD, 0x89);

		zigzag = TRUE;
		break;
	case STV0900_AUTO_SEARCH:
	default:
		if (intp->chip_id >= 0x20) {
			stv0900_write_reg(intp, CARFREQ, 0x3b);
			stv0900_write_reg(intp, CORRELABS, 0x79);
		} else {
			stv0900_write_reg(intp, CARFREQ, 0xef);
			stv0900_write_reg(intp, CORRELABS, 0x68);
		}

		stv0900_write_reg(intp, DMDCFGMD, 0xc9);
		zigzag = FALSE;
		break;
	}

	trial_cntr = 0;
	do {
		lock = stv0900_search_carr_sw_loop(intp,
						fqc_inc,
						sft_stp_tout,
						zigzag,
						max_steps,
						demod);
		no_signal = stv0900_check_signal_presence(intp, demod);
		trial_cntr++;
		if ((lock == TRUE)
				|| (no_signal == TRUE)
				|| (trial_cntr == 2)) {

			if (intp->chip_id >= 0x20) {
				stv0900_write_reg(intp, CARFREQ, 0x49);
				stv0900_write_reg(intp, CORRELABS, 0x9e);
			} else {
				stv0900_write_reg(intp, CARFREQ, 0xed);
				stv0900_write_reg(intp, CORRELABS, 0x88);
			}

			if ((stv0900_get_bits(intp, HEADER_MODE) ==
						STV0900_DVBS2_FOUND) &&
							(lock == TRUE)) {
				DelayMilliseconds(sft_stp_tout);
				s2fw = stv0900_get_bits(intp, FLYWHEEL_CPT);

				if (s2fw < 0xd) {
					DelayMilliseconds(sft_stp_tout);
					s2fw = stv0900_get_bits(intp,
								FLYWHEEL_CPT);
				}

				if (s2fw < 0xd) {
					lock = FALSE;

					if (trial_cntr < 2) {
						if (intp->chip_id >= 0x20)
							stv0900_write_reg(intp,
								CORRELABS,
								0x79);
						else
							stv0900_write_reg(intp,
								CORRELABS,
								0x68);

						stv0900_write_reg(intp,
								DMDCFGMD,
								0x89);
					}
				}
			}
		}

	} while ((lock == FALSE)
		&& (trial_cntr < 2)
		&& (no_signal == FALSE));

	return lock;
}

enum fe_stv0900_signal_type stv0900_algo(struct stv0900_state *state, enum fe_stv0900_demod_num demod)
{
	struct stv0900_internal *intp = state->internal;

	LONG demod_timeout = 500, fec_timeout = 50;
	LONG aq_power, agc1_power, i;

	LONG lock = FALSE, low_sr = FALSE;

	enum fe_stv0900_signal_type signal_type = STV0900_NOCARRIER;
	enum fe_stv0900_search_algo algo;
	LONG no_signal = FALSE;

	KdPrint((LOG_PREFIX "%s", __FUNCTION__));

	algo = intp->srch_algo[demod];
	stv0900_write_bits(intp, RST_HWARE, 1);
	stv0900_write_reg(intp, DMDISTATE, 0x5c);
	if (intp->chip_id >= 0x20) {
		if (intp->symbol_rate[demod] > 5000000)
			stv0900_write_reg(intp, CORRELABS, 0x9e);
		else
			stv0900_write_reg(intp, CORRELABS, 0x82);
	} else
		stv0900_write_reg(intp, CORRELABS, 0x88);

	stv0900_get_lock_timeout(&demod_timeout, &fec_timeout,
				intp->symbol_rate[demod],
				intp->srch_algo[demod]);
	KdPrint((LOG_PREFIX "demod_timeout=%d fec_timeout=%d", demod_timeout, fec_timeout));

	if (intp->srch_algo[demod] == STV0900_BLIND_SEARCH) {
		intp->bw[demod] = 2 * 36000000;

		stv0900_write_reg(intp, TMGCFG2, 0xc0);
		stv0900_write_reg(intp, CORRELMANT, 0x70);

		stv0900_set_symbol_rate(intp, intp->mclk, 1000000, demod);
	} else {
		stv0900_write_reg(intp, DMDT0M, 0x20);
		stv0900_write_reg(intp, TMGCFG, 0xd2);

		if (intp->symbol_rate[demod] < 2000000)
			stv0900_write_reg(intp, CORRELMANT, 0x63);
		else
			stv0900_write_reg(intp, CORRELMANT, 0x70);

		stv0900_write_reg(intp, AGC2REF, 0x38);

		intp->bw[demod] =
				stv0900_carrier_width(intp->symbol_rate[demod],
								intp->rolloff);
		if (intp->chip_id >= 0x20) {
			stv0900_write_reg(intp, KREFTMG, 0x5a);

			if (intp->srch_algo[demod] == STV0900_COLD_START) {
				intp->bw[demod] += 10000000;
				intp->bw[demod] *= 15;
				intp->bw[demod] /= 10;
			} else if (intp->srch_algo[demod] == STV0900_WARM_START)
				intp->bw[demod] += 10000000;

		} else {
			stv0900_write_reg(intp, KREFTMG, 0xc1);
			intp->bw[demod] += 10000000;
			intp->bw[demod] *= 15;
			intp->bw[demod] /= 10;
		}

		stv0900_write_reg(intp, TMGCFG2, 0xc1);

		stv0900_set_symbol_rate(intp, intp->mclk,
					intp->symbol_rate[demod], demod);
		stv0900_set_max_symbol_rate(intp, intp->mclk,
					intp->symbol_rate[demod], demod);
		stv0900_set_min_symbol_rate(intp, intp->mclk,
					intp->symbol_rate[demod], demod);
		if (intp->symbol_rate[demod] >= 10000000)
			low_sr = FALSE;
		else
			low_sr = TRUE;

	}

	stv0900_set_tuner(state, demod, intp->freq[demod], intp->bw[demod]);

	agc1_power = MAKEWORD(stv0900_get_bits(intp, AGCIQ_VALUE1),
				stv0900_get_bits(intp, AGCIQ_VALUE0));

	aq_power = 0;

	if (agc1_power == 0) {
		for (i = 0; i < 5; i++)
			aq_power += (stv0900_get_bits(intp, POWER_I) +
					stv0900_get_bits(intp, POWER_Q)) / 2;

		aq_power /= 5;
	}

	if ((agc1_power == 0) && (aq_power < IQPOWER_THRESHOLD)) {
		intp->result[demod].locked = FALSE;
		signal_type = STV0900_NOAGC1;
		KdPrint(("%s: NO AGC1, POWERI, POWERQ", __FUNCTION__));
	} else {
		stv0900_write_bits(intp, SPECINV_CONTROL,
					intp->srch_iq_inv[demod]);
		if (intp->chip_id <= 0x20) /*cut 2.0*/
			stv0900_write_bits(intp, MANUALSX_ROLLOFF, 1);
		else /*cut 3.0*/
			stv0900_write_bits(intp, MANUALS2_ROLLOFF, 1);

		stv0900_set_search_standard(intp, demod);

		if (intp->srch_algo[demod] != STV0900_BLIND_SEARCH)
			stv0900_start_search(intp, demod);
	}

	if (signal_type == STV0900_NOAGC1)
		return signal_type;

	if (intp->chip_id == 0x12) {
		stv0900_write_bits(intp, RST_HWARE, 0);
		DelayMilliseconds(3);
		stv0900_write_bits(intp, RST_HWARE, 1);
		stv0900_write_bits(intp, RST_HWARE, 0);
	}

	if (algo == STV0900_BLIND_SEARCH)
		lock = stv0900_blind_search_algo(state, demod);
	else if (algo == STV0900_COLD_START)
		lock = stv0900_get_demod_cold_lock(state, demod, demod_timeout);
	else if (algo == STV0900_WARM_START)
		lock = stv0900_get_demod_lock(intp, demod, demod_timeout);

	if ((lock == FALSE) && (algo == STV0900_COLD_START)) {
		if (low_sr == FALSE) {
			if (stv0900_check_timing_lock(intp, demod) == TRUE)
				lock = stv0900_sw_algo(intp, demod);
		}
	}

	if (lock == TRUE)
		signal_type = stv0900_get_signal_params(state, demod);

	if ((lock == TRUE) && (signal_type == STV0900_RANGEOK)) {
		stv0900_track_optimization(state, demod);
		if (intp->chip_id <= 0x11) {
			if ((stv0900_get_standard(state, static_cast<fe_stv0900_demod_num>(0)) ==
						STV0900_DVBS1_STANDARD) &&
			   (stv0900_get_standard(state, static_cast<fe_stv0900_demod_num>(1)) ==
						STV0900_DVBS1_STANDARD)) {
				DelayMilliseconds(20);
				stv0900_write_bits(intp, RST_HWARE, 0);
			} else {
				stv0900_write_bits(intp, RST_HWARE, 0);
				DelayMilliseconds(3);
				stv0900_write_bits(intp, RST_HWARE, 1);
				stv0900_write_bits(intp, RST_HWARE, 0);
			}

		} else if (intp->chip_id >= 0x20) {
			stv0900_write_bits(intp, RST_HWARE, 0);
			DelayMilliseconds(3);
			stv0900_write_bits(intp, RST_HWARE, 1);
			stv0900_write_bits(intp, RST_HWARE, 0);
		}

		if (stv0900_wait_for_lock(intp, demod,
					fec_timeout, fec_timeout) == TRUE) {
			lock = TRUE;
			intp->result[demod].locked = TRUE;
			if (intp->result[demod].standard ==
						STV0900_DVBS2_STANDARD) {
				stv0900_set_dvbs2_rolloff(intp, demod);
				stv0900_write_bits(intp, RESET_UPKO_COUNT, 1);
				stv0900_write_bits(intp, RESET_UPKO_COUNT, 0);
				stv0900_write_reg(intp, ERRCTRL1, 0x67);
			} else {
				stv0900_write_reg(intp, ERRCTRL1, 0x75);
			}

			stv0900_write_reg(intp, FBERCPT4, 0);
			stv0900_write_reg(intp, ERRCTRL2, 0xc1);
		} else {
			lock = FALSE;
			signal_type = STV0900_NODATA;
			no_signal = stv0900_check_signal_presence(intp, demod);

				intp->result[demod].locked = FALSE;
		}
	}

	if ((signal_type != STV0900_NODATA) || (no_signal != FALSE))
		return signal_type;

	if (intp->chip_id > 0x11) {
		intp->result[demod].locked = FALSE;
		return signal_type;
	}

	if ((stv0900_get_bits(intp, HEADER_MODE) == STV0900_DVBS_FOUND) &&
	   (intp->srch_iq_inv[demod] <= STV0900_IQ_AUTO_NORMAL_FIRST))
		signal_type = stv0900_dvbs1_acq_workaround(state, demod);

	return signal_type;
}

static LONG stv0900_carr_get_quality(struct stv0900_state *state, enum fe_stv0900_demod_num demod,
					const struct stv0900_table *lookup)
{
	struct stv0900_internal *intp = state->internal;

	LONG	c_n = -100,
		regval,
		imin,
		imax,
		i,
		noise_field1,
		noise_field0;

	if (stv0900_get_standard(state, demod) == STV0900_DVBS2_STANDARD) {
		noise_field1 = NOSPLHT_NORMED1;
		noise_field0 = NOSPLHT_NORMED0;
	} else {
		noise_field1 = NOSDATAT_NORMED1;
		noise_field0 = NOSDATAT_NORMED0;
	}

	if (stv0900_get_bits(intp, LOCK_DEFINITIF)) {
		if ((lookup != NULL) && lookup->size) {
			regval = 0;
			DelayMilliseconds(5);
			for (i = 0; i < 16; i++) {
				regval += MAKEWORD(stv0900_get_bits(intp,
								noise_field1),
						stv0900_get_bits(intp,
								noise_field0));
				DelayMilliseconds(1);
			}

			regval /= 16;
			imin = 0;
			imax = lookup->size - 1;
			if (INRANGE(lookup->table[imin].regval,
					regval,
					lookup->table[imax].regval)) {
				while ((imax - imin) > 1) {
					i = (imax + imin) >> 1;
					if (INRANGE(lookup->table[imin].regval,
						    regval,
						    lookup->table[i].regval))
						imax = i;
					else
						imin = i;
				}

				c_n = ((regval - lookup->table[imin].regval)
						* (lookup->table[imax].realval
						- lookup->table[imin].realval)
						/ (lookup->table[imax].regval
						- lookup->table[imin].regval))
						+ lookup->table[imin].realval;
			} else if (regval < lookup->table[imin].regval)
				c_n = 1000;
		}
	}

	return c_n;
}

static LONG stv0900_get_rf_level(struct stv0900_internal *intp,
				const struct stv0900_table *lookup,
				enum fe_stv0900_demod_num demod)
{
	LONG agc_gain = 0,
		imin,
		imax,
		i,
		rf_lvl = 0;

	if ((lookup == NULL) || (lookup->size <= 0))
		return 0;

	agc_gain = MAKEWORD(stv0900_get_bits(intp, AGCIQ_VALUE1),
				stv0900_get_bits(intp, AGCIQ_VALUE0));

	imin = 0;
	imax = lookup->size - 1;
	if (INRANGE(lookup->table[imin].regval, agc_gain,
					lookup->table[imax].regval)) {
		while ((imax - imin) > 1) {
			i = (imax + imin) >> 1;

			if (INRANGE(lookup->table[imin].regval,
					agc_gain,
					lookup->table[i].regval))
				imax = i;
			else
				imin = i;
		}

		rf_lvl = (LONG)agc_gain - lookup->table[imin].regval;
		rf_lvl *= (lookup->table[imax].realval -
				lookup->table[imin].realval);
		rf_lvl /= (lookup->table[imax].regval -
				lookup->table[imin].regval);
		rf_lvl += lookup->table[imin].realval;
	} else if (agc_gain > lookup->table[0].regval)
		rf_lvl = 5;
	else if (agc_gain < lookup->table[lookup->size-1].regval)
		rf_lvl = -100;

	KdPrint((LOG_PREFIX "%s: RFLevel = %d", __FUNCTION__, rf_lvl));

	return rf_lvl;
}

static ULONG stv0900_get_err_count(struct stv0900_internal *intp, ULONG cntr,
					enum fe_stv0900_demod_num demod)
{
	ULONG lsb, msb, hsb, err_val;

	switch (cntr) {
	case 0:
	default:
		hsb = stv0900_get_bits(intp, ERR_CNT12);
		msb = stv0900_get_bits(intp, ERR_CNT11);
		lsb = stv0900_get_bits(intp, ERR_CNT10);
		break;
	case 1:
		hsb = stv0900_get_bits(intp, ERR_CNT22);
		msb = stv0900_get_bits(intp, ERR_CNT21);
		lsb = stv0900_get_bits(intp, ERR_CNT20);
		break;
	}

	err_val = (hsb << 16) + (msb << 8) + (lsb);

	return err_val;
}

static void stv0900_status(struct stv0900_internal *intp,
					enum fe_stv0900_demod_num demod)
{
	enum fe_stv0900_search_state demod_state;
	BOOL locked = FALSE;
	UCHAR tsbitrate0_val, tsbitrate1_val;
	LONG bitrate;

	demod_state = static_cast<fe_stv0900_search_state>(stv0900_get_bits(intp, HEADER_MODE));
	switch (demod_state) {
	case STV0900_SEARCH:
	case STV0900_PLH_DETECTED:
	default:
		locked = FALSE;
		break;
	case STV0900_DVBS2_FOUND:
		locked = stv0900_get_bits(intp, LOCK_DEFINITIF) &&
				stv0900_get_bits(intp, PKTDELIN_LOCK) &&
				stv0900_get_bits(intp, TSFIFO_LINEOK);
		break;
	case STV0900_DVBS_FOUND:
		locked = stv0900_get_bits(intp, LOCK_DEFINITIF) &&
				stv0900_get_bits(intp, LOCKEDVIT) &&
				stv0900_get_bits(intp, TSFIFO_LINEOK);
		break;
	}

	KdPrint((LOG_PREFIX "%s: locked = %d", __FUNCTION__, locked));

	/* Print TS bitrate */
	tsbitrate0_val = stv0900_read_reg(intp, TSBITRATE0);
	tsbitrate1_val = stv0900_read_reg(intp, TSBITRATE1);
	/* Formula Bit rate = Mclk * px_tsfifo_bitrate / 16384 */
	bitrate = (stv0900_get_mclk_freq(intp, intp->quartz)/1000000)
		* (tsbitrate1_val << 8 | tsbitrate0_val);
	bitrate /= 16384;
	

	ULONG ber = 0, i;
	for (i = 0; i < 5; i++) {
		DelayMilliseconds(5);
		ber += stv0900_get_err_count(intp, 0, demod);
	}

	ber /= 5;
	if (stv0900_get_bits(intp, PRFVIT)) {
		ber *= 9766;
		ber = ber >> 13;
	}

	KdPrint((LOG_PREFIX "TS bitrate = %d Mbit/sec, BER = %d", bitrate, ber));
}

BOOL STV0900_HasLocked(PKSDEVICE device, LONG demod)
{
	ASSERT(device != NULL);
	ASSERT(demod >= 0 && demod < 2);
	stv0900_state * state = (stv0900_state *)(GETCONTEXT(device)->stv0900_context);
	struct stv0900_internal *intp = state->internal;

	if (stv0900_get_bits(intp, LOCK_DEFINITIF))
	{
		KdPrint((LOG_PREFIX "signal locked"));
		stv0900_status(intp, static_cast<fe_stv0900_demod_num>(demod));
		return TRUE;
	}
	else
	{
		KdPrint((LOG_PREFIX "signal NOT locked"));
		return FALSE;
	}
}

#define DEC_ONE (1 << 14)

ULONG STV0900_SignalQualityGet(PKSDEVICE device, LONG demod)
{
	ASSERT(device != NULL);
	ASSERT(demod >= 0 && demod < 2);
	stv0900_state * state = (stv0900_state *)(GETCONTEXT(device)->stv0900_context);
	struct stv0900_internal *intp = state->internal;

	ULONG i, noise_field1, noise_field0, regval;

	if (stv0900_get_standard(state, static_cast<fe_stv0900_demod_num>(demod)) == STV0900_DVBS2_STANDARD) {
		noise_field1 = NOSPLHT_NORMED1;
		noise_field0 = NOSPLHT_NORMED0;
	} else {
		noise_field1 = NOSDATAT_NORMED1;
		noise_field0 = NOSDATAT_NORMED0;
	}

	if (stv0900_get_bits(intp, LOCK_DEFINITIF))
	{
		regval = 0;
		DelayMilliseconds(5);
		for (i = 0; i < 16; i++) {
			regval += MAKEWORD(stv0900_get_bits(intp, noise_field1), stv0900_get_bits(intp, noise_field0));
			DelayMilliseconds(1);
		}
		regval /= 16;
		if(regval > DEC_ONE)
		{
			KdPrint((LOG_PREFIX "regval=0x%x", regval));
			//return 0;
		}
		if(regval == 0)
			regval = DEC_ONE;
		return 100 - 100/(DEC_ONE/regval + 1);
	}
	else
	{
		return 0;
	}
}

ULONG STV0900_SignalStrengthGet(PKSDEVICE device, LONG demod)
{
	ASSERT(device != NULL);
	ASSERT(demod >= 0 && demod < 2);
	stv0900_state * state = (stv0900_state *)(GETCONTEXT(device)->stv0900_context);
	struct stv0900_internal *intp = state->internal;

	if (stv0900_get_bits(intp, LOCK_DEFINITIF))
	{
		ULONG agc_gain = 0, i;
		for(i = 0; i < 16; i++)
		{
			agc_gain += MAKEWORD(stv0900_get_bits(intp, AGCIQ_VALUE1),
					stv0900_get_bits(intp, AGCIQ_VALUE0));
			DelayMilliseconds(1);
		}
		agc_gain /= 16;
		ULONG strength = 100*agc_gain/65535;
		KdPrint((LOG_PREFIX "demod #%d signal strength=%d%%", demod, strength));
		return strength;
	}
	else
	{
		KdPrint((LOG_PREFIX "demod #%d not locked, strength=0", demod));
		return 0;
	}
	
}

NTSTATUS STV0900_LockSignal(PKSDEVICE device, LONG demod, ULONG freq, ULONG srate)
{
	ASSERT(device != NULL);
	ASSERT(demod >= 0 && demod < 2);
	ASSERT(srate >= 100 && srate <= 70000);
	DbgPrint(LOG_PREFIX "%s (demod %d): freq=%d srate=%d", __FUNCTION__, demod, freq, srate);
	stv0900_state * state = (stv0900_state *)(GETCONTEXT(device)->stv0900_context);
	struct stv0900_internal *intp = state->internal;
	intp->srch_algo[demod] = STV0900_BLIND_SEARCH;
	intp->symbol_rate[demod] = srate * 1000;
	intp->freq[demod] = freq;
	intp->srch_iq_inv[demod] = STV0900_IQ_AUTO;
	intp->srch_standard[demod] = STV0900_AUTO_SEARCH;
	intp->srch_range[demod] = 10000000;

	RtlZeroMemory(&(intp->result), sizeof(intp->result));
	LONG signal_type = stv0900_algo(state, static_cast<fe_stv0900_demod_num>(demod));
	KdPrint((LOG_PREFIX "stv0900_algo result %d", signal_type));
	if(intp->result[demod].locked == TRUE)
	{
		DbgPrint(LOG_PREFIX "%s (demod %d): signal locked", __FUNCTION__, demod);
		return STATUS_SUCCESS;
	} else
	{
		DbgPrint(LOG_PREFIX "%s (demod %d): signal lock failed", __FUNCTION__, demod);
		return STATUS_UNSUCCESSFUL;
	}
}

VOID STV0900_GetSignalInfo(PKSDEVICE device, LONG demod, NetupSignalInfo & result)
{
	ASSERT(device != NULL);
	ASSERT(demod >= 0 && demod < 2);
	stv0900_state * state = (stv0900_state *)(GETCONTEXT(device)->stv0900_context);
	struct stv0900_internal *intp = state->internal;
	RtlZeroMemory(&result, sizeof(result));
	result.locked = intp->result[demod].locked;
	if(result.locked == TRUE)
	{
		switch(intp->result[demod].modulation)
		{
			case STV0900_QPSK:
				result.modulation = BDA_MOD_QPSK;
				break;
			case STV0900_8PSK:
				result.modulation = BDA_MOD_8PSK;
				break;
			case STV0900_16APSK:
				result.modulation = BDA_MOD_16APSK;
				break;
			case STV0900_32APSK:
				result.modulation = BDA_MOD_32APSK;
				break;
			default:
				result.modulation = BDA_MOD_NOT_DEFINED;
				break;
		}
		switch(intp->result[demod].fec)
		{
			case STV0900_FEC_1_2:
				result.fec = BDA_BCC_RATE_1_2;
				break;
			case STV0900_FEC_2_3:
				result.fec = BDA_BCC_RATE_2_3;
				break;
			case STV0900_FEC_3_4:
				result.fec = BDA_BCC_RATE_3_4;
				break;
			case STV0900_FEC_4_5:
				result.fec = BDA_BCC_RATE_4_5;
				break;
			case STV0900_FEC_5_6:
				result.fec = BDA_BCC_RATE_5_6;
				break;
			case STV0900_FEC_6_7:
				result.fec = BDA_BCC_RATE_6_7;
				break;
			case STV0900_FEC_7_8:
				result.fec = BDA_BCC_RATE_7_8;
				break;
			case STV0900_FEC_8_9:
				result.fec = BDA_BCC_RATE_8_9;
				break;
			default:
				result.fec = BDA_BCC_RATE_NOT_DEFINED;
				break;
		}
		switch(intp->result[demod].spectrum)
		{
			case STV0900_IQ_NORMAL:
				result.spectrum = BDA_SPECTRAL_INVERSION_NORMAL;
				break;
			case STV0900_IQ_SWAPPED:
				result.spectrum = BDA_SPECTRAL_INVERSION_INVERTED;
				break;
			default:
				result.spectrum = BDA_SPECTRAL_INVERSION_NOT_DEFINED;
				break;
		}
		switch(intp->result[demod].pilot)
		{
			case STV0900_PILOTS_OFF:
				result.pilot = BDA_PILOT_OFF;
				break;
			case STV0900_PILOTS_ON:
				result.pilot = BDA_PILOT_ON;
				break;
			default:
				result.pilot = BDA_PILOT_NOT_DEFINED;
				break;
		}
		switch(intp->result[demod].rolloff)
		{
			case STV0900_35:
				result.rolloff = BDA_ROLL_OFF_35;
				break;
			case STV0900_25:
				result.rolloff = BDA_ROLL_OFF_25;
				break;
			case STV0900_20:
				result.rolloff = BDA_ROLL_OFF_20;
				break;
			default:
				result.rolloff = BDA_ROLL_OFF_NOT_DEFINED;
				break;
		}
	}
}

LONG STV0900_DiSEqC_Write(PKSDEVICE device, LONG demod, PUCHAR data, ULONG NbData)
{
	ASSERT(device != NULL);
	ASSERT(demod >= 0 && demod < 2);
	stv0900_state * state = (stv0900_state *)(GETCONTEXT(device)->stv0900_context);
	struct stv0900_internal *intp = state->internal;

	KdPrint((LOG_PREFIX "%s", __FUNCTION__));

	ULONG i = 0;

	stv0900_write_bits(intp, DIS_PRECHARGE, 1);
	while (i < NbData) {
		while (stv0900_get_bits(intp, FIFO_FULL))
			;/* checkpatch complains */
		stv0900_write_reg(intp, DISTXDATA, data[i]);
		KdPrint((LOG_PREFIX "DiSEqC cmd data <%02x>", (ULONG)data[i]));
		i++;
	}

	stv0900_write_bits(intp, DIS_PRECHARGE, 0);
	i = 0;
	while ((stv0900_get_bits(intp, TX_IDLE) != 1) && (i < 10)) {
		DelayMilliseconds(10);
		i++;
	}
	return 0;
}

VOID STV0900_SetTone(PKSDEVICE device, LONG demod, BOOLEAN toneEnable)
{
	ASSERT(device != NULL);
	ASSERT(demod >= 0 && demod < 2);
	stv0900_state * state = (stv0900_state *)(GETCONTEXT(device)->stv0900_context);
	stv0900_set_tone(state, (fe_stv0900_demod_num)demod, toneEnable ? SEC_TONE_ON : SEC_TONE_OFF);
}

NTSTATUS STV0900_Attach(PKSDEVICE device, ULONG i2c, const stv0900_config *config, LONG demod)
{
	stv0900_state *state = NULL;
	stv0900_init_params init_params;
	fe_stv0900_error err_stv0900;


	if(GETCONTEXT(device)->stv0900_context != NULL)
	{
		KdPrint((LOG_PREFIX "STV0900: using old context"));
		state = (stv0900_state *)(GETCONTEXT(device)->stv0900_context);
	}
	else
	{
		KdPrint((LOG_PREFIX "STV0900: alloctaing new context"));
		state = (stv0900_state *)ExAllocatePoolWithTag(NonPagedPool, sizeof(stv0900_state), 'SVTS');
		RtlZeroMemory(state, sizeof(stv0900_state));
		if(!state)
		{
			KdPrint((LOG_PREFIX "STV0900 attach: unable to allocate context"));
			return STATUS_INSUFFICIENT_RESOURCES;
		}
		GETCONTEXT(device)->stv0900_context = state;
	}

	state->demod		= demod;
	state->config		= config;
	state->i2c_adap		= i2c;
	state->device = device;

	switch (demod) {
	case 0:
	case 1:
		init_params.dmd_ref_clk  	= config->xtal;
		init_params.demod_mode		= (fe_stv0900_demod_mode)(config->demod_mode);
		init_params.rolloff		= STV0900_35;
		init_params.path1_ts_clock	= (fe_stv0900_clock_type)(config->path1_mode);
		init_params.tun1_maddress	= config->tun1_maddress;
		init_params.tun1_iq_inv		= STV0900_IQ_NORMAL;
		init_params.tuner1_adc		= config->tun1_adc;
		init_params.tuner1_type		= config->tun1_type;
		init_params.path2_ts_clock	= (fe_stv0900_clock_type)(config->path2_mode);
		init_params.ts_config		= config->ts_config_regs;
		init_params.tun2_maddress	= config->tun2_maddress;
		init_params.tuner2_adc		= config->tun2_adc;
		init_params.tuner2_type		= config->tun2_type;
		init_params.tun2_iq_inv		= STV0900_IQ_SWAPPED;

		err_stv0900 = stv0900_init_internal(state, &init_params);

		if (err_stv0900)
			goto error;
		break;

	default:
		goto error;
		break;
	}

	DbgPrint("Attaching STV0900 demodulator(%d)", demod);
	return STATUS_SUCCESS;
	
error:
	DbgPrint("Failed to attach STV0900 demodulator(%d)", demod);
	ExFreePool(state);
	return STATUS_INSUFFICIENT_RESOURCES;
}

void STV0900_I2C_Gate_Ctrl(PKSDEVICE device, ULONG demod, ULONG enable)
{
	ASSERT(demod < 2);
	stv0900_write_bits(((stv0900_state *)(GETCONTEXT(device)->stv0900_context))->internal, I2CT_ON, enable & 0xff);
}

static struct stv0900_reg stv0900_ts_regs[] = {
	{ R0900_TSGENERAL, 0x00 },
	{ R0900_P1_TSSPEED, 0x40 },
	{ R0900_P2_TSSPEED, 0x40 },
	{ R0900_P1_TSCFGM, 0xc0 },
	{ R0900_P2_TSCFGM, 0xc0 },
	{ R0900_P1_TSCFGH, 0xe0 },
	{ R0900_P2_TSCFGH, 0xe0 },
	{ R0900_P1_TSCFGL, 0x20 },
	{ R0900_P2_TSCFGL, 0x20 },
	{ 0xffff, 0xff }, /* terminate */
};

static struct stv0900_config netup_stv0900_config;

NTSTATUS STV0900_Init(PKSDEVICE device)
{
	netup_stv0900_config.demod_address = 0x68;
	netup_stv0900_config.demod_mode = 1; /* dual */
	netup_stv0900_config.xtal = 8000000; 
	netup_stv0900_config.clkmode = 3; /* 0-CLKI, 2-XTALI, else AUTO */
	netup_stv0900_config.diseqc_mode = 2; /* 2/3 PWM */
	netup_stv0900_config.ts_config_regs = stv0900_ts_regs;
	netup_stv0900_config.tun1_maddress = 0; /* 0x60 */
	netup_stv0900_config.tun2_maddress = 3; /* 0x63 */
	netup_stv0900_config.tun1_adc = 1; /* 1 Vpp */
	netup_stv0900_config.tun2_adc = 1; /* 1 Vpp */

	ULONG i2c_bus = 0;
	STV0900_Attach(device, i2c_bus, &netup_stv0900_config, 0);
	STV0900_Attach(device, i2c_bus, &netup_stv0900_config, 1);
	return STATUS_SUCCESS;
}