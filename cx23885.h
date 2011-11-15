/*
 *  cx23885.h
 *
 *  WDM driver for NetUP Dual DVB-S2 CI card
 *
 *  Copyright (C) 2011 NetUP Inc.
 *  Copyright (C) 2011 Sergey Kozlov <serjk@netup.ru>
 *
 *  Driver for the Conexant CX23885 PCIe bridge
 *
 *  Copyright (c) 2006 Steven Toth <stoth@linuxtv.org>
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

#ifndef __NETUP_CX23885_H__
#define __NETUP_CX23885_H__

#include "driver.h"

#define cpu_to_le32(x) (x)

struct netup_card_info
{
	UCHAR rev;
	UCHAR mac1[6];
	UCHAR mac2[6];
};

// Functions to be called by driver
NTSTATUS CX23885_Init(PKSDEVICE device);
void CX23885_Close(PKSDEVICE device);
BOOLEAN CX23885_Interrupt(PKSDEVICE device);
void CX23885_DPCRoutine(PKDPC Dpc,PVOID DeferredContext,PVOID SystemArgument1,PVOID SystemArgument2);
void Netup_CI_DPC_Routine(PKDPC Dpc,PVOID DeferredContext,PVOID SystemArgument1,PVOID SystemArgument2);
void NetupCardInit(PKSDEVICE device);
void Netup_Get_Card_Info(PKSDEVICE device, ULONG i2c_adap, struct netup_card_info *cinfo);
BOOLEAN Netup_CI_Init(PKSDEVICE device, struct cx23885_tsport *port);
void CX23885_Test_DMA(PKSDEVICE device, ULONG nr);
void CX23885_Load_FW(PKSDEVICE device);
void CX23885_Stop_DMA(PKSDEVICE device, cx23885_tsport * port);
void CX23885_Start_DMA(PKSDEVICE device, cx23885_tsport *port);
LONG Netup_CI_Slot_Status(PKSDEVICE device, ULONG pci_status);
VOID Netup_CI_Stop(PKSDEVICE device);
VOID CX23885_OutputProcess(PKSDEVICE device, PKSPIN pin, PVOID port);

#define CX23885_SRC_SEL_PARALLEL_MPEG_VIDEO 1

#define SRAM_CH01  0 /* Video A */
#define SRAM_CH02  1 /* VBI A */
#define SRAM_CH03  2 /* Video B */
#define SRAM_CH04  3 /* Transport via B */
#define SRAM_CH05  4 /* VBI B */
#define SRAM_CH06  5 /* Video C */
#define SRAM_CH07  6 /* Transport via C */
#define SRAM_CH08  7 /* Audio Internal A */
#define SRAM_CH09  8 /* Audio Internal B */
#define SRAM_CH10  9 /* Audio External */
#define SRAM_CH11 10 /* COMB_3D_N */
#define SRAM_CH12 11 /* Comb 3D N1 */
#define SRAM_CH13 12 /* Comb 3D N2 */
#define SRAM_CH14 13 /* MOE Vid */
#define SRAM_CH15 14 /* MOE RSLT */

#define MPEG_QUEUE_SIZE	64

struct sram_channel {
	char *name;
	ULONG  cmds_start;
	ULONG  ctrl_start;
	ULONG  cdt;
	ULONG  fifo_start;
	ULONG  fifo_size;
	ULONG  ptr1_reg;
	ULONG  ptr2_reg;
	ULONG  cnt1_reg;
	ULONG  cnt2_reg;
	ULONG  jumponly;
};

struct cx23885_i2c {
	int                        nr;

	/* 885 registers used for raw addess */
	ULONG                        i2c_period;
	ULONG                        reg_ctrl;
	ULONG                        reg_stat;
	ULONG                        reg_addr;
	ULONG                        reg_rdata;
	ULONG                        reg_wdata;

	KSPIN_LOCK lock;
};

struct CX_RISC_MEM
{
	PULONG cpu;
	PULONG jmp;
	ULONG size;
	ULONG dma;
	PHYSICAL_ADDRESS phys;
};

struct cx23885_buffer {
	CX_RISC_MEM risc;
	CX_RISC_MEM data;
	ULONG count;
	LIST_ENTRY link;
};

struct cx23885_dmaqueue {
	CX_RISC_MEM starter;
	CX_RISC_MEM stopper;
	LIST_ENTRY active;
	LIST_ENTRY queued;
	ULONG count;
};

struct cx23885_tsport {
	int                        nr;
	int                        sram_chno;

	KDPC dpc;

	BOOLEAN running;

	/* dma queues */
	struct cx23885_dmaqueue    mpegq;
	ULONG                      ts_packet_size;
	ULONG                      ts_packet_count;

	KSPIN_LOCK                 slock;

	/* registers */
	ULONG                        reg_gpcnt;
	ULONG                        reg_gpcnt_ctl;
	ULONG                        reg_dma_ctl;
	ULONG                        reg_lngth;
	ULONG                        reg_hw_sop_ctrl;
	ULONG                        reg_gen_ctrl;
	ULONG                        reg_bd_pkt_status;
	ULONG                        reg_sop_status;
	ULONG                        reg_fifo_ovfl_stat;
	ULONG                        reg_vld_misc;
	ULONG                        reg_ts_clk_en;
	ULONG                        reg_ts_int_msk;
	ULONG                        reg_ts_int_stat;
	ULONG                        reg_src_sel;

	/* Default register vals */
	ULONG                        pci_irqmask;
	ULONG                        dma_ctl_val;
	ULONG                        ts_int_msk_val;
	ULONG                        gen_ctrl_val;
	ULONG                        ts_clk_en_val;
	ULONG                        src_sel_val;
	ULONG                        vld_misc_val;
	ULONG                        hw_sop_ctrl_val;
};

#define cx_read(reg)             READ_REGISTER_ULONG((PULONG)(GETCONTEXT(device)->pci_memory) + ((reg)>>2))
#define cx_write(reg, value)     WRITE_REGISTER_ULONG((PULONG)(GETCONTEXT(device)->pci_memory) + ((reg)>>2), (value))

#define cx_andor(reg, mask, value) \
  WRITE_REGISTER_ULONG((PULONG)(GETCONTEXT(device)->pci_memory) + ((reg)>>2), \
  (READ_REGISTER_ULONG((PULONG)(GETCONTEXT(device)->pci_memory) + ((reg)>>2)) & ~(mask)) | ((value) & (mask)))

#define cx_set(reg, bit)          cx_andor((reg), (bit), (bit))
#define cx_clear(reg, bit)        cx_andor((reg), (bit), 0)

#endif
