/*
 * stv0900.h
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

#ifndef __NETUP_STV0900_H__
#define __NETUP_STV0900_H__

#include "driver.h"
#include "device.h"
#include "sec.h"

/* STV0900 config structures */

struct stv0900_reg {
	USHORT addr;
	UCHAR  val;
};

struct stv0900_config {
	UCHAR demod_address;
	UCHAR demod_mode;
	ULONG xtal;
	UCHAR clkmode;/* 0 for CLKI,  2 for XTALI */

	UCHAR diseqc_mode;

	UCHAR path1_mode;
	UCHAR path2_mode;
	stv0900_reg *ts_config_regs;
	UCHAR tun1_maddress; /* 0, 1, 2, 3 for 0xc0, 0xc2, 0xc4, 0xc6 */
	UCHAR tun2_maddress;
	UCHAR tun1_adc; /* 1 for stv6110, 2 for stb6100 */
	UCHAR tun2_adc;
	UCHAR tun1_type; /* for now 3 for stb6100 auto, else - software */
	UCHAR tun2_type;
};

/* STV0900 internal stuff */

#define ABS(X) ((X) < 0 ? (-1 * (X)) : (X))
#define INRANGE(X, Y, Z) ((((X) <= (Y)) && ((Y) <= (Z))) \
		|| (((Z) <= (Y)) && ((Y) <= (X))) ? 1 : 0)

#ifndef MAKEWORD
#define MAKEWORD(X, Y) (((X) << 8) + (Y))
#endif

#define LSB(X) (((X) & 0xFF))
#define MSB(Y) (((Y) >> 8) & 0xFF)

#define STV0900_MAXLOOKUPSIZE 500
#define STV0900_BLIND_SEARCH_AGC2_TH 700
#define STV0900_BLIND_SEARCH_AGC2_TH_CUT30 1400
#define IQPOWER_THRESHOLD  30

/* One point of the lookup table */
struct stv000_lookpoint {
	LONG realval;/* real value */
	LONG regval;/* binary value */
};

/* Lookup table definition */
struct stv0900_table{
	LONG size;/* Size of the lookup table */
	stv000_lookpoint table[STV0900_MAXLOOKUPSIZE];/* Lookup table */
};

enum fe_stv0900_error {
	STV0900_NO_ERROR = 0,
	STV0900_INVALID_HANDLE,
	STV0900_BAD_PARAMETER,
	STV0900_I2C_ERROR,
	STV0900_SEARCH_FAILED,
};

enum fe_stv0900_clock_type {
	STV0900_USE_REGISTERS_DEFAULT,
	STV0900_SERIAL_PUNCT_CLOCK,/*Serial punctured clock */
	STV0900_SERIAL_CONT_CLOCK,/*Serial continues clock */
	STV0900_PARALLEL_PUNCT_CLOCK,/*Parallel punctured clock */
	STV0900_DVBCI_CLOCK/*Parallel continues clock : DVBCI */
};

enum fe_stv0900_search_state {
	STV0900_SEARCH = 0,
	STV0900_PLH_DETECTED,
	STV0900_DVBS2_FOUND,
	STV0900_DVBS_FOUND

};

enum fe_stv0900_ldpc_state {
	STV0900_PATH1_OFF_PATH2_OFF = 0,
	STV0900_PATH1_ON_PATH2_OFF = 1,
	STV0900_PATH1_OFF_PATH2_ON = 2,
	STV0900_PATH1_ON_PATH2_ON = 3
};

enum fe_stv0900_signal_type {
	STV0900_NOAGC1 = 0,
	STV0900_AGC1OK,
	STV0900_NOTIMING,
	STV0900_ANALOGCARRIER,
	STV0900_TIMINGOK,
	STV0900_NOAGC2,
	STV0900_AGC2OK,
	STV0900_NOCARRIER,
	STV0900_CARRIEROK,
	STV0900_NODATA,
	STV0900_DATAOK,
	STV0900_OUTOFRANGE,
	STV0900_RANGEOK
};



enum fe_stv0900_demod_num {
	STV0900_DEMOD_1,
	STV0900_DEMOD_2
};

enum fe_stv0900_tracking_standard {
	STV0900_DVBS1_STANDARD,/* Found Standard*/
	STV0900_DVBS2_STANDARD,
	STV0900_DSS_STANDARD,
	STV0900_TURBOCODE_STANDARD,
	STV0900_UNKNOWN_STANDARD
};

enum fe_stv0900_search_standard {
	STV0900_AUTO_SEARCH,
	STV0900_SEARCH_DVBS1,/* Search Standard*/
	STV0900_SEARCH_DVBS2,
	STV0900_SEARCH_DSS,
	STV0900_SEARCH_TURBOCODE
};

enum fe_stv0900_search_algo {
	STV0900_BLIND_SEARCH,/* offset freq and SR are Unknown */
	STV0900_COLD_START,/* only the SR is known */
	STV0900_WARM_START/* offset freq and SR are known */
};

enum fe_stv0900_modulation {
	STV0900_QPSK,
	STV0900_8PSK,
	STV0900_16APSK,
	STV0900_32APSK,
	STV0900_UNKNOWN
};

enum fe_stv0900_modcode {
	STV0900_DUMMY_PLF,
	STV0900_QPSK_14,
	STV0900_QPSK_13,
	STV0900_QPSK_25,
	STV0900_QPSK_12,
	STV0900_QPSK_35,
	STV0900_QPSK_23,
	STV0900_QPSK_34,
	STV0900_QPSK_45,
	STV0900_QPSK_56,
	STV0900_QPSK_89,
	STV0900_QPSK_910,
	STV0900_8PSK_35,
	STV0900_8PSK_23,
	STV0900_8PSK_34,
	STV0900_8PSK_56,
	STV0900_8PSK_89,
	STV0900_8PSK_910,
	STV0900_16APSK_23,
	STV0900_16APSK_34,
	STV0900_16APSK_45,
	STV0900_16APSK_56,
	STV0900_16APSK_89,
	STV0900_16APSK_910,
	STV0900_32APSK_34,
	STV0900_32APSK_45,
	STV0900_32APSK_56,
	STV0900_32APSK_89,
	STV0900_32APSK_910,
	STV0900_MODCODE_UNKNOWN
};

enum fe_stv0900_fec {/*DVBS1, DSS and turbo code puncture rate*/
	STV0900_FEC_1_2 = 0,
	STV0900_FEC_2_3,
	STV0900_FEC_3_4,
	STV0900_FEC_4_5,/*for turbo code only*/
	STV0900_FEC_5_6,
	STV0900_FEC_6_7,/*for DSS only */
	STV0900_FEC_7_8,
	STV0900_FEC_8_9,/*for turbo code only*/
	STV0900_FEC_UNKNOWN
};

enum fe_stv0900_frame_length {
	STV0900_LONG_FRAME,
	STV0900_SHORT_FRAME
};

enum fe_stv0900_pilot {
	STV0900_PILOTS_OFF,
	STV0900_PILOTS_ON
};

enum fe_stv0900_rolloff {
	STV0900_35,
	STV0900_25,
	STV0900_20
};

enum fe_stv0900_search_iq {
	STV0900_IQ_AUTO,
	STV0900_IQ_AUTO_NORMAL_FIRST,
	STV0900_IQ_FORCE_NORMAL,
	STV0900_IQ_FORCE_SWAPPED
};

enum stv0900_iq_inversion {
	STV0900_IQ_NORMAL,
	STV0900_IQ_SWAPPED
};

enum fe_stv0900_diseqc_mode {
	STV0900_22KHZ_Continues = 0,
	STV0900_DISEQC_2_3_PWM = 2,
	STV0900_DISEQC_3_3_PWM = 3,
	STV0900_DISEQC_2_3_ENVELOP = 4,
	STV0900_DISEQC_3_3_ENVELOP = 5
};

enum fe_stv0900_demod_mode {
	STV0900_SINGLE = 0,
	STV0900_DUAL
};

struct stv0900_init_params{
	LONG	dmd_ref_clk;/* Reference,Input clock for the demod in Hz */

	/* Demodulator Type (single demod or dual demod) */
	fe_stv0900_demod_mode	demod_mode;
	fe_stv0900_rolloff		rolloff;
	fe_stv0900_clock_type	path1_ts_clock;

	UCHAR	tun1_maddress;
	LONG	tuner1_adc;
	LONG 	tuner1_type;

	/* IQ from the tuner1 to the demod */
	stv0900_iq_inversion	tun1_iq_inv;
	fe_stv0900_clock_type	path2_ts_clock;

	UCHAR	tun2_maddress;
	LONG	tuner2_adc;
	LONG	tuner2_type;

	/* IQ from the tuner2 to the demod */
	stv0900_iq_inversion	tun2_iq_inv;
	stv0900_reg		*ts_config;
};

struct stv0900_search_params {
	fe_stv0900_demod_num	path;/* Path Used demod1 or 2 */

	ULONG	frequency;/* Transponder frequency (in KHz) */
	ULONG	symbol_rate;/* Transponder symbol rate  (in bds)*/
	ULONG	search_range;/* Range of the search (in Hz) */

	fe_stv0900_search_standard	standard;
	fe_stv0900_modulation	modulation;
	fe_stv0900_fec		fec;
	fe_stv0900_modcode		modcode;
	fe_stv0900_search_iq	iq_inversion;
	fe_stv0900_search_algo	search_algo;

};

struct stv0900_signal_info {
	BOOL	locked;/* Transponder locked */
	ULONG	frequency;/* Transponder frequency (in KHz) */
	ULONG	symbol_rate;/* Transponder symbol rate  (in Mbds) */

	fe_stv0900_tracking_standard	standard;
	fe_stv0900_fec			fec;
	fe_stv0900_modcode			modcode;
	fe_stv0900_modulation		modulation;
	fe_stv0900_pilot			pilot;
	fe_stv0900_frame_length		frame_len;
	stv0900_iq_inversion		spectrum;
	fe_stv0900_rolloff			rolloff;

	LONG Power;/* Power of the RF signal (dBm) */
	LONG C_N;/* Carrier to noise ratio (dB x10)*/
	LONG BER;/* Bit error rate (x10^7) */

};

struct stv0900_internal{
	PKSDEVICE device;
	LONG	quartz;
	LONG	mclk;
	/* manual RollOff for DVBS1/DSS only */
	fe_stv0900_rolloff		rolloff;
	/* Demodulator use for single demod or for dual demod) */
	fe_stv0900_demod_mode	demod_mode;

	/*Demods */
	ULONG	freq[2];
	ULONG	bw[2];
	ULONG	symbol_rate[2];
	ULONG	srch_range[2];
	/* for software/auto tuner */
	ULONG	tuner_type[2];

	/* algorithm for search Blind, Cold or Warm*/
	fe_stv0900_search_algo	srch_algo[2];
	/* search standard: Auto, DVBS1/DSS only or DVBS2 only*/
	fe_stv0900_search_standard	srch_standard[2];
	/* inversion search : auto, auto norma first, normal or inverted */
	fe_stv0900_search_iq	srch_iq_inv[2];
	fe_stv0900_modcode		modcode[2];
	fe_stv0900_modulation	modulation[2];
	fe_stv0900_fec		fec[2];

	stv0900_signal_info	result[2];
	fe_stv0900_error		err[2];


	ULONG i2c_adap;
	UCHAR			i2c_addr;
	UCHAR			clkmode;/* 0 for CLKI, 2 for XTALI */
	UCHAR			chip_id;
	stv0900_reg	*ts_config;
	fe_stv0900_error	errs;
	LONG dmds_used;
};

/* state for each demod */
struct stv0900_state {
	PKSDEVICE device;
	/* pointer for internal params, one for each pair of demods */
	stv0900_internal		*internal;
	ULONG i2c_adap;
	const stv0900_config	*config;
	LONG demod;
};

void STV0900_I2C_Gate_Ctrl(PKSDEVICE device, ULONG demod, ULONG enable);
NTSTATUS STV0900_LockSignal(PKSDEVICE device, LONG demod, ULONG freq, ULONG srate);
ULONG STV0900_SignalQualityGet(PKSDEVICE device, LONG demod);
ULONG STV0900_SignalStrengthGet(PKSDEVICE device, LONG demod);
NTSTATUS STV0900_Init(PKSDEVICE device);
BOOL STV0900_HasLocked(PKSDEVICE device, LONG demod);
LONG STV0900_DiSEqC_Write(PKSDEVICE device, LONG demod, PUCHAR buff, ULONG count);
VOID STV0900_GetSignalInfo(PKSDEVICE device, LONG demod, NetupSignalInfo & result);
VOID STV0900_SetTone(PKSDEVICE device, LONG demod, BOOLEAN toneEnable);

#endif // __NETUP_STV0900_H__