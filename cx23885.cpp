/*
 *  cx23885.cpp
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

#include "cx23885.h"
#include "cx23885-reg.h"
#include "device.h"
#include "stv0900.h"
#include "stv6110.h"
#include "lnbh24.h"
#include "en50221.h"
#include <ntstrsafe.h>

static sram_channel cx23885_sram_channels[15] = {
	{
		"VID A",
		0x10000,
		0x10380,
		0x104c0,
		0x40,
		0x2800,
		DMA1_PTR1,
		DMA1_PTR2,
		DMA1_CNT1,
		DMA1_CNT2,
	},
	{
		"ch2",
		0x0,
		0x0,
		0x0,
		0x0,
		0x0,
		DMA2_PTR1,
		DMA2_PTR2,
		DMA2_CNT1,
		DMA2_CNT2,
	},
	{
		"TS1 B",
		0x100A0,
		0x10400,
		0x10580,
		0x5000,
		0x1000,
		DMA3_PTR1,
		DMA3_PTR2,
		DMA3_CNT1,
		DMA3_CNT2,
	},
	{
		"ch4",
		0x0,
		0x0,
		0x0,
		0x0,
		0x0,
		DMA4_PTR1,
		DMA4_PTR2,
		DMA4_CNT1,
		DMA4_CNT2,
	},
	{
		"ch5",
		0x0,
		0x0,
		0x0,
		0x0,
		0x0,
		DMA5_PTR1,
		DMA5_PTR2,
		DMA5_CNT1,
		DMA5_CNT2,
	},
	{
		"TS2 C",
		0x10140,
		0x10440,
		0x105e0,
		0x6000,
		0x1000,
		DMA5_PTR1,
		DMA5_PTR2,
		DMA5_CNT1,
		DMA5_CNT2,
	},
	{
		"ch7",
		0x0,
		0x0,
		0x0,
		0x0,
		0x0,
		DMA6_PTR1,
		DMA6_PTR2,
		DMA6_CNT1,
		DMA6_CNT2,
	},
	{
		"ch8",
		0x0,
		0x0,
		0x0,
		0x0,
		0x0,
		DMA7_PTR1,
		DMA7_PTR2,
		DMA7_CNT1,
		DMA7_CNT2,
	},
	{
		"ch9",
		0x0,
		0x0,
		0x0,
		0x0,
		0x0,
		DMA8_PTR1,
		DMA8_PTR2,
		DMA8_CNT1,
		DMA8_CNT2,
	},
};

static BOOLEAN pmt_sent = FALSE;

void CX_RISC_Free(PKSDEVICE device, CX_RISC_MEM * risc)
{
	if(risc->cpu == NULL)
	{
		return;
	}
	GETCONTEXT(device)->dma->DmaOperations->FreeCommonBuffer(GETCONTEXT(device)->dma, risc->size, risc->phys, risc->cpu, TRUE);
	RtlZeroMemory(risc, sizeof(*risc));
}

BOOL CX_RISC_Alloc(PKSDEVICE device, CX_RISC_MEM * risc, ULONG size)
{
	if(risc->cpu != NULL && risc->size < size)
	{
		CX_RISC_Free(device, risc);
	}
	PHYSICAL_ADDRESS dma;
	PULONG cpu = (PULONG)(GETCONTEXT(device)->dma->DmaOperations->AllocateCommonBuffer(GETCONTEXT(device)->dma, size, &dma, TRUE));
	if(cpu != NULL)
	{
		risc->cpu = cpu;
		risc->phys = dma;
		risc->dma = dma.LowPart;
		risc->size = size;
		RtlZeroMemory(risc->cpu, risc->size);
		return TRUE;
	}
	else
	{
		KdPrint((LOG_PREFIX "%s: failed.", __FUNCTION__));
		return FALSE;
	}
}

void CX23885_IRQ_Add(PKSDEVICE device, ULONG mask)
{
	KIRQL oldIrql;
	KeAcquireSpinLock(&(GETCONTEXT(device)->pci_irqmask_lock), &oldIrql);
	GETCONTEXT(device)->pci_irqmask |= mask;
	KeReleaseSpinLock(&(GETCONTEXT(device)->pci_irqmask_lock), oldIrql);
}

void CX23885_IRQ_Disable(PKSDEVICE device, ULONG mask)
{
	KIRQL oldIrql;
	KeAcquireSpinLock(&(GETCONTEXT(device)->pci_irqmask_lock), &oldIrql);
	cx_clear(PCI_INT_MSK, mask);
	KdPrint((LOG_PREFIX "PCI_INT_MSK CLEAR mask 0x%x", mask));
	KeReleaseSpinLock(&(GETCONTEXT(device)->pci_irqmask_lock), oldIrql);
}

void CX23885_IRQ_Enable(PKSDEVICE device, ULONG mask)
{
	ULONG v;
	KIRQL oldIrql;
	KeAcquireSpinLock(&(GETCONTEXT(device)->pci_irqmask_lock), &oldIrql);

	v = mask & (GETCONTEXT(device)->pci_irqmask);
	if (v)
		cx_set(PCI_INT_MSK, v);
	KdPrint((LOG_PREFIX "PCI_INT_MSK SET mask 0x%x", v));

	KeReleaseSpinLock(&(GETCONTEXT(device)->pci_irqmask_lock), oldIrql);
}

void CX23885_IRQ_Add_Enable(PKSDEVICE device, ULONG mask)
{
	KIRQL oldIrql;
	KeAcquireSpinLock(&(GETCONTEXT(device)->pci_irqmask_lock), &oldIrql);
	(GETCONTEXT(device)->pci_irqmask) |= mask;
	cx_set(PCI_INT_MSK, mask);
	KdPrint((LOG_PREFIX "PCI_INT_MSK SET mask 0x%x", mask));
	KeReleaseSpinLock(&(GETCONTEXT(device)->pci_irqmask_lock), oldIrql);
}

static inline void CX23885_IRQ_Disable_All(PKSDEVICE device)
{
	CX23885_IRQ_Disable(device, 0xffffffff);
}

static inline void CX23885_IRQ_Enable_All(PKSDEVICE device)
{
	CX23885_IRQ_Enable(device, 0xffffffff);
}

static ULONG CX23885_IRQ_Get_Mask(PKSDEVICE device)
{
	ULONG v;
	KIRQL oldIrql;
	KeAcquireSpinLock(&(GETCONTEXT(device)->pci_irqmask_lock), &oldIrql);
	v = cx_read(PCI_INT_MSK);
	KeReleaseSpinLock(&(GETCONTEXT(device)->pci_irqmask_lock), oldIrql);
	return v;
}

void CX23885_SRAM_Channel_Setup(PKSDEVICE device, sram_channel *ch, ULONG bpl, ULONG risc)
{
	ULONG i, lines;
	ULONG cdt;

	if (ch->cmds_start == 0) {
		KdPrint((LOG_PREFIX "Erasing SRAM channel [%s]", ch->name));
		cx_write(ch->ptr1_reg, 0);
		cx_write(ch->ptr2_reg, 0);
		cx_write(ch->cnt2_reg, 0);
		cx_write(ch->cnt1_reg, 0);
		return;
	} else {
		KdPrint((LOG_PREFIX "Configuring channel [%s]", ch->name));
	}

	bpl   = (bpl + 7) & ~7; /* alignment */
	cdt   = ch->cdt;
	lines = ch->fifo_size / bpl;
	if (lines > 6)
		lines = 6;
	ASSERT(lines >= 2);

	cx_write(8 + 0, RISC_JUMP | RISC_IRQ1 | RISC_CNT_INC);
	cx_write(8 + 4, 8);
	cx_write(8 + 8, 0);

	/* write CDT */
	for (i = 0; i < lines; i++) {
		KdPrint((LOG_PREFIX "0x%08x <- 0x%08x",cdt + 16*i, ch->fifo_start + bpl*i));
		cx_write(cdt + 16*i, ch->fifo_start + bpl*i);
		cx_write(cdt + 16*i +  4, 0);
		cx_write(cdt + 16*i +  8, 0);
		cx_write(cdt + 16*i + 12, 0);
	}

	/* write CMDS */
	if (ch->jumponly)
		cx_write(ch->cmds_start + 0, 8);
	else
		cx_write(ch->cmds_start + 0, risc);
	cx_write(ch->cmds_start +  4, 0); /* 64 bits 63-32 */
	cx_write(ch->cmds_start +  8, cdt);
	cx_write(ch->cmds_start + 12, (lines*16) >> 3);
	cx_write(ch->cmds_start + 16, ch->ctrl_start);
	if (ch->jumponly)
		cx_write(ch->cmds_start + 20, 0x80000000 | (64 >> 2));
	else
		cx_write(ch->cmds_start + 20, 64 >> 2);
	for (i = 24; i < 80; i += 4)
		cx_write(ch->cmds_start + i, 0);

	/* fill registers */
	cx_write(ch->ptr1_reg, ch->fifo_start);
	cx_write(ch->ptr2_reg, cdt);
	cx_write(ch->cnt2_reg, (lines*16) >> 3);
	cx_write(ch->cnt1_reg, (bpl >> 3) - 1);

	KdPrint((LOG_PREFIX "[bridge 23885] SRAM setup %s: bpl=%d lines=%d", ch->name, bpl, lines));
}

void CX23885_GPIO_Setup(PKSDEVICE device)
{
	/* GPIO-0 INTA from CiMax1
	   GPIO-1 INTB from CiMax2
	   GPIO-2 reset chips
	   GPIO-3 to GPIO-10 data/addr for CA
	   GPIO-11 ~CS0 to CiMax1
	   GPIO-12 ~CS1 to CiMax2
	   GPIO-13 ADL0 load LSB addr
	   GPIO-14 ADL1 load MSB addr
	   GPIO-15 ~RDY from CiMax
	   GPIO-17 ~RD to CiMax
	   GPIO-18 ~WR to CiMax
	 */
	cx_set(GP0_IO, 0x00040000); /* GPIO as out */
	/* GPIO1 and GPIO2 as INTA and INTB from CiMaxes, reset low */
	cx_clear(GP0_IO, 0x00030004);
	DelayMilliseconds(100);/* reset delay */
	cx_set(GP0_IO, 0x00040004); /* GPIO as out, reset high */
	cx_write(MC417_CTL, 0x00000037);/* enable GPIO3-18 pins */
	/* GPIO-15 IN as ~ACK, rest as OUT */
	cx_write(MC417_OEN, 0x00001000);
	/* ~RD, ~WR high; ADL0, ADL1 low; ~CS0, ~CS1 high */
	cx_write(MC417_RWD, 0x0000c300);
	/* enable irq */
	cx_write(GPIO_ISM, 0x00000000);/* INTERRUPTS active low*/
}

static void CX23885_Shutdown(PKSDEVICE device)
{
	/* disable RISC controller */
	cx_write(DEV_CNTRL2, 0);

	/* Disable all IR activity */
	cx_write(IR_CNTRL_REG, 0);

	/* Disable Video A/B activity */
	cx_write(VID_A_DMA_CTL, 0);
	cx_write(VID_B_DMA_CTL, 0);
	cx_write(VID_C_DMA_CTL, 0);

	/* Disable Audio activity */
	cx_write(AUD_INT_DMA_CTL, 0);
	cx_write(AUD_EXT_DMA_CTL, 0);

	/* Disable Serial port */
	cx_write(UART_CTL, 0);

	/* Disable Interrupts */
	CX23885_IRQ_Disable_All(device);
	cx_write(VID_A_INT_MSK, 0);
	cx_write(VID_B_INT_MSK, 0);
	cx_write(VID_C_INT_MSK, 0);
	cx_write(AUDIO_INT_INT_MSK, 0);
	cx_write(AUDIO_EXT_INT_MSK, 0);
}

static void CX23885_Reset(PKSDEVICE device)
{
	KdPrint((LOG_PREFIX "CX23885 Reset"));

	CX23885_Shutdown(device);

	cx_write(PCI_INT_STAT, 0xffffffff);
	cx_write(VID_A_INT_STAT, 0xffffffff);
	cx_write(VID_B_INT_STAT, 0xffffffff);
	cx_write(VID_C_INT_STAT, 0xffffffff);
	cx_write(AUDIO_INT_INT_STAT, 0xffffffff);
	cx_write(AUDIO_EXT_INT_STAT, 0xffffffff);
	cx_write(CLK_DELAY, cx_read(CLK_DELAY) & 0x80000000);
	cx_write(PAD_CTRL, 0x00500300);

	DelayMilliseconds(100);

	CX23885_SRAM_Channel_Setup(device, &((sram_channel *)(GETCONTEXT(device)->sram_channels))[SRAM_CH01], 720*4, 0);
	CX23885_SRAM_Channel_Setup(device, &((sram_channel *)(GETCONTEXT(device)->sram_channels))[SRAM_CH02], 128, 0);
	CX23885_SRAM_Channel_Setup(device, &((sram_channel *)(GETCONTEXT(device)->sram_channels))[SRAM_CH03], 188*4, 0);
	CX23885_SRAM_Channel_Setup(device, &((sram_channel *)(GETCONTEXT(device)->sram_channels))[SRAM_CH04], 128, 0);
	CX23885_SRAM_Channel_Setup(device, &((sram_channel *)(GETCONTEXT(device)->sram_channels))[SRAM_CH05], 128, 0);
	CX23885_SRAM_Channel_Setup(device, &((sram_channel *)(GETCONTEXT(device)->sram_channels))[SRAM_CH06], 188*4, 0);
	CX23885_SRAM_Channel_Setup(device, &((sram_channel *)(GETCONTEXT(device)->sram_channels))[SRAM_CH07], 128, 0);
	CX23885_SRAM_Channel_Setup(device, &((sram_channel *)(GETCONTEXT(device)->sram_channels))[SRAM_CH08], 128, 0);
	CX23885_SRAM_Channel_Setup(device, &((sram_channel *)(GETCONTEXT(device)->sram_channels))[SRAM_CH09], 128, 0);

	CX23885_GPIO_Setup(device);
}


static void CX23885_PCI_Quirks(PKSDEVICE device)
{
	KdPrint(("CX23885_PCI_Quirks"));

	/* The cx23885 bridge has a weird bug which causes NMI to be asserted
	 * when DMA begins if RDR_TLCTL0 bit4 is not cleared. It does not
	 * occur on the cx23887 bridge.
	 */
	cx_clear(RDR_TLCTL0, 1 << 4);
}

static PULONG CX23885_RISC_Field(PULONG rp, CX_RISC_MEM * mem, ULONG bpl, ULONG lines)
{
	ASSERT(lines * bpl <= mem->size);
	ULONG line;
	for(line = 0; line < lines; line++)
	{
		*(rp++) = cpu_to_le32(RISC_WRITE|RISC_SOL|RISC_EOL|bpl);
		*(rp++) = cpu_to_le32(mem->dma+line*bpl);
		*(rp++) = cpu_to_le32(0);
	}
	return rp;
}

static cx23885_buffer * CX23885_Buffer_Prepare(PKSDEVICE device, cx23885_tsport *port)
{
	cx23885_buffer * buf = (cx23885_buffer *)ExAllocatePoolWithTag(NonPagedPool, sizeof(*buf), 'FBXC');
	if(!buf)
	{
		KdPrint((LOG_PREFIX "Unable to allocate data buffer"));
		return NULL;
	}
	RtlZeroMemory(buf, sizeof(*buf));

	ULONG bpl = port->ts_packet_size, lines = port->ts_packet_count, instructions;
	PULONG rp;

	/* estimate risc mem: worst case is one write per page border +
	   one write per scan line + syncs + jump (all 2 dwords).  Here
	   there is no padding and no sync.  First DMA region may be smaller
	   than PAGE_SIZE */
	/* Jump and write need an extra dword */
	instructions  = 1 + (bpl * lines) / PAGE_SIZE + lines;
	instructions += 1;

	CX_RISC_MEM * risc = &(buf->risc);
	CX_RISC_MEM * data = &(buf->data);

	if(!CX_RISC_Alloc(device, risc, instructions*12))
	{
		KdPrint((LOG_PREFIX "%s: ERROR: unable to allocate RISC code space (%d bytes)", __FUNCTION__, instructions * 12));
		return NULL;
	}
	
	if(!CX_RISC_Alloc(device, data, bpl * lines))
	{
		KdPrint((LOG_PREFIX "%s: ERROR: unable to allocate data buffer space (%d bytes)", __FUNCTION__, bpl * lines));
		return NULL;
	}

#ifdef NETUP_EXTRA_DBG
	KdPrint((LOG_PREFIX "RISC IO space DMA 0x%08x Ptr 0x%08x", risc->dma, risc->cpu));
	KdPrint((LOG_PREFIX "Data IO space DMA 0x%08x Ptr 0x%08x", data->dma, data->cpu));
	KdPrint((LOG_PREFIX "Stop IO space DMA 0x%08x Ptr 0x%08x", port->mpegq.stopper.dma, port->mpegq.stopper.cpu));
#endif

	/* write risc instructions */
	rp = risc->cpu;

	rp = CX23885_RISC_Field(rp, data, bpl, lines);

	/* save pointer to jmp instruction address */
	risc->jmp = rp;

	//ASSERT((risc->jmp - risc->cpu + 2) * sizeof(*risc->cpu) <= risc->size);

	/* add jump to stopper */
	risc->jmp[0] = cpu_to_le32(RISC_JUMP | RISC_IRQ1 | RISC_CNT_INC);
	risc->jmp[1] = cpu_to_le32(port->mpegq.stopper.dma);
	risc->jmp[2] = cpu_to_le32(0); /* bits 63-32 */

	return buf;
}

void CX23885_RISC_Starter(PKSDEVICE device, struct CX_RISC_MEM *risc, ULONG reg, ULONG mask, ULONG value)
{
	PULONG rp;
	BOOL rc;

	rc = CX_RISC_Alloc(device, risc, 4*16);
	if (!rc)
	{
		KdPrint((LOG_PREFIX "CX23885_RISC_Starter: CX_RISC_Alloc failed"));
		return;
	}

	/* write risc instructions */
	rp = risc->cpu;

	*(rp++) = cpu_to_le32(RISC_WRITECR);
	*(rp++) = cpu_to_le32(reg);
	*(rp++) = cpu_to_le32(value);
	*(rp++) = cpu_to_le32(mask);
	risc->jmp = rp;
	*(rp++) = cpu_to_le32(RISC_JUMP | RISC_CNT_RESET);
	*(rp++) = cpu_to_le32(risc->dma);
	*(rp++) = cpu_to_le32(0);
}

void CX23885_RISC_Stopper(PKSDEVICE device, struct CX_RISC_MEM *risc,
				ULONG reg, ULONG mask, ULONG value)
{
	PULONG rp;
	BOOL rc;

	rc = CX_RISC_Alloc(device, risc, 4*16);
	if (!rc)
	{
		KdPrint((LOG_PREFIX "CX23885_RISC_Stopper: CX_RISC_Alloc failed"));
		return;
	}

	/* write risc instructions */
	rp = risc->cpu;
	*(rp++) = cpu_to_le32(RISC_WRITECR | RISC_IRQ2);
	*(rp++) = cpu_to_le32(reg);
	*(rp++) = cpu_to_le32(value);
	*(rp++) = cpu_to_le32(mask);
	*(rp++) = cpu_to_le32(RISC_JUMP);
	*(rp++) = cpu_to_le32(risc->dma);
	*(rp++) = cpu_to_le32(0); /* bits 63-32 */
}

static void cx23885_tsport_reg_dump(PKSDEVICE device, struct cx23885_tsport *port)
{

	KdPrint((LOG_PREFIX "%s: Register Dump\n", __FUNCTION__));
	KdPrint((LOG_PREFIX "%s() DEV_CNTRL2               0x%08X\n", __FUNCTION__,
		cx_read(DEV_CNTRL2)));
	KdPrint((LOG_PREFIX "%s() PCI_INT_MSK              0x%08X\n", __FUNCTION__,
		CX23885_IRQ_Get_Mask(device)));
	KdPrint((LOG_PREFIX "%s() AUD_INT_INT_MSK          0x%08X\n", __FUNCTION__,
		cx_read(AUDIO_INT_INT_MSK)));
	KdPrint((LOG_PREFIX "%s() AUD_INT_DMA_CTL          0x%08X\n", __FUNCTION__,
		cx_read(AUD_INT_DMA_CTL)));
	KdPrint((LOG_PREFIX "%s() AUD_EXT_INT_MSK          0x%08X\n", __FUNCTION__,
		cx_read(AUDIO_EXT_INT_MSK)));
	KdPrint((LOG_PREFIX "%s() AUD_EXT_DMA_CTL          0x%08X\n", __FUNCTION__,
		cx_read(AUD_EXT_DMA_CTL)));
	KdPrint((LOG_PREFIX "%s() PAD_CTRL                 0x%08X\n", __FUNCTION__,
		cx_read(PAD_CTRL)));
	KdPrint((LOG_PREFIX "%s() ALT_PIN_OUT_SEL          0x%08X\n", __FUNCTION__,
		cx_read(ALT_PIN_OUT_SEL)));
	KdPrint((LOG_PREFIX "%s() GPIO2                    0x%08X\n", __FUNCTION__,
		cx_read(GPIO2)));
	//cx_write(GPIO2, 0x05A4ABD4);
	KdPrint((LOG_PREFIX "%s() gpcnt(0x%08X)          0x%08X\n", __FUNCTION__,
		port->reg_gpcnt, cx_read(port->reg_gpcnt)));
	KdPrint((LOG_PREFIX "%s() gpcnt_ctl(0x%08X)      0x%08x\n", __FUNCTION__,
		port->reg_gpcnt_ctl, cx_read(port->reg_gpcnt_ctl)));
	KdPrint((LOG_PREFIX "%s() dma_ctl(0x%08X)        0x%08x\n", __FUNCTION__,
		port->reg_dma_ctl, cx_read(port->reg_dma_ctl)));
	if (port->reg_src_sel)
	{
		KdPrint((LOG_PREFIX "%s() src_sel(0x%08X)        0x%08x\n", __FUNCTION__,
			port->reg_src_sel, cx_read(port->reg_src_sel)));
	}
	KdPrint((LOG_PREFIX "%s() lngth(0x%08X)          0x%08x\n", __FUNCTION__,
		port->reg_lngth, cx_read(port->reg_lngth)));
	KdPrint((LOG_PREFIX "%s() hw_sop_ctrl(0x%08X)    0x%08x\n", __FUNCTION__,
		port->reg_hw_sop_ctrl, cx_read(port->reg_hw_sop_ctrl)));
	KdPrint((LOG_PREFIX "%s() gen_ctrl(0x%08X)       0x%08x\n", __FUNCTION__,
		port->reg_gen_ctrl, cx_read(port->reg_gen_ctrl)));
	KdPrint((LOG_PREFIX "%s() bd_pkt_status(0x%08X)  0x%08x\n", __FUNCTION__,
		port->reg_bd_pkt_status, cx_read(port->reg_bd_pkt_status)));
	KdPrint((LOG_PREFIX "%s() sop_status(0x%08X)     0x%08x\n", __FUNCTION__,
		port->reg_sop_status, cx_read(port->reg_sop_status)));
	KdPrint((LOG_PREFIX "%s() fifo_ovfl_stat(0x%08X) 0x%08x\n", __FUNCTION__,
		port->reg_fifo_ovfl_stat, cx_read(port->reg_fifo_ovfl_stat)));
	KdPrint((LOG_PREFIX "%s() vld_misc(0x%08X)       0x%08x\n", __FUNCTION__,
		port->reg_vld_misc, cx_read(port->reg_vld_misc)));
	KdPrint((LOG_PREFIX "%s() ts_clk_en(0x%08X)      0x%08x\n", __FUNCTION__,
		port->reg_ts_clk_en, cx_read(port->reg_ts_clk_en)));
	KdPrint((LOG_PREFIX "%s() ts_int_msk(0x%08X)     0x%08x\n", __FUNCTION__,
		port->reg_ts_int_msk, cx_read(port->reg_ts_int_msk)));
}

static CHAR risc_cmd_str[256];

static ULONG cx23885_risc_decode(ULONG risc)
{
	static CHAR *instr[16] = { 0, 0, 0, 0,
							   0, 0, 0, 0,
							   0, 0, 0, 0,
							   0, 0, 0, 0 };
	instr[RISC_SYNC    >> 28] = "sync";
	instr[RISC_WRITE   >> 28] = "write";
	instr[RISC_WRITEC  >> 28] = "writec";
	instr[RISC_READ    >> 28] = "read";
	instr[RISC_READC   >> 28] = "readc";
	instr[RISC_JUMP    >> 28] = "jump";
	instr[RISC_SKIP    >> 28] = "skip";
	instr[RISC_WRITERM >> 28] = "writerm";
	instr[RISC_WRITECM >> 28] = "writecm";
	instr[RISC_WRITECR >> 28] = "writecr";

	static ULONG incr[16] =  { 0, 0, 0, 0,
							   0, 0, 0, 0,
							   0, 0, 0, 0,
							   0, 0, 0, 0 };
	incr[RISC_WRITE   >> 28] = 3;
	incr[RISC_JUMP    >> 28] = 3;
	incr[RISC_SKIP    >> 28] = 1;
	incr[RISC_SYNC    >> 28] = 1;
	incr[RISC_WRITERM >> 28] = 3;
	incr[RISC_WRITECM >> 28] = 3;
	incr[RISC_WRITECR >> 28] = 4;

	static CHAR *bits[16] = {
		"12",   "13",   "14",   "resync",
		"cnt0", "cnt1", "18",   "19",
		"20",   "21",   "22",   "23",
		"irq1", "irq2", "eol",  "sol",
	};

	LONG i;
	size_t ptr = 0;

	RtlStringCbPrintfA(risc_cmd_str, sizeof(risc_cmd_str), "0x%08x [ %s", risc,
	       instr[risc >> 28] ? instr[risc >> 28] : "INVALID");
	RtlStringCbLengthA(risc_cmd_str, sizeof(risc_cmd_str), &ptr);
	for (i = 16 - 1; i >= 0; i--)
		if (risc & (1 << (i + 12))) {
			RtlStringCbPrintfA(risc_cmd_str + ptr, sizeof(risc_cmd_str) - ptr, " %s", bits[i]);
			RtlStringCbLengthA(risc_cmd_str, sizeof(risc_cmd_str), &ptr);
		}
	RtlStringCbPrintfA(risc_cmd_str + ptr, sizeof(risc_cmd_str) - ptr, " count=%d ]", risc & 0xfff);
	KdPrint((LOG_PREFIX "%s", risc_cmd_str));
	return incr[risc >> 28] ? incr[risc >> 28] : 1;
}

void cx23885_sram_channel_dump(PKSDEVICE device,
				      struct sram_channel *ch)
{
	static char *name[] = {
		"init risc lo",
		"init risc hi",
		"cdt base",
		"cdt size",
		"iq base",
		"iq size",
		"risc pc lo",
		"risc pc hi",
		"iq wr ptr",
		"iq rd ptr",
		"cdt current",
		"pci target lo",
		"pci target hi",
		"line / byte",
	};
	ULONG risc;
	ULONG i, j, n;

	KdPrint((LOG_PREFIX "%s - dma channel status dump\n", ch->name));
	for (i = 0; i < 14; i++)
		KdPrint((LOG_PREFIX "cmds: %-15s: 0x%08x",
		       name[i],
		       cx_read(ch->cmds_start + 4*i)));

	for (i = 0; i < 4; i++) {
		risc = cx_read(ch->cmds_start + 4 * (i + 14));
		KdPrint((LOG_PREFIX "   risc%d: ", i));
		cx23885_risc_decode(risc);
	}
	for (i = 0; i < (64 >> 2); i += n) {
		risc = cx_read(ch->ctrl_start + 4 * i);
		/* No consideration for bits 63-32 */

		KdPrint((LOG_PREFIX "   (0x%08x) iq %x: ",
		       ch->ctrl_start + 4 * i, i));
		n = cx23885_risc_decode(risc);
		for (j = 1; j < n; j++) {
			risc = cx_read(ch->ctrl_start + 4 * (i + j));
			KdPrint((LOG_PREFIX "   iq %x: 0x%08x [ arg #%d ]", i+j, risc, j));
		}
	}

	KdPrint((LOG_PREFIX " fifo: 0x%08x -> 0x%x",
	       ch->fifo_start, ch->fifo_start+ch->fifo_size));
	KdPrint((LOG_PREFIX " ctrl: 0x%08x -> 0x%x",
	       ch->ctrl_start, ch->ctrl_start + 6*16));
	KdPrint((LOG_PREFIX "   ptr1_reg: 0x%08x",
	       cx_read(ch->ptr1_reg)));
	KdPrint((LOG_PREFIX "   ptr2_reg: 0x%08x",
	       cx_read(ch->ptr2_reg)));
	KdPrint((LOG_PREFIX "   cnt1_reg: 0x%08x",
	       cx_read(ch->cnt1_reg)));
	KdPrint((LOG_PREFIX "   cnt2_reg: 0x%08x",
	       cx_read(ch->cnt2_reg)));
}

static void cx23885_risc_disasm(PKSDEVICE device, struct cx23885_tsport *port,
				struct CX_RISC_MEM *risc)
{
	ULONG i, j, n;

	KdPrint((LOG_PREFIX " risc disasm: %p [dma=0x%08lx]",
	       risc->cpu, risc->dma));
	for (i = 0; i < (risc->size >> 2); i += n) {
		KdPrint((LOG_PREFIX "   %04d: ", i));
		n = cx23885_risc_decode(risc->cpu[i]);
		for (j = 1; j < n; j++)
			KdPrint((LOG_PREFIX "   %04d: 0x%08x [ arg #%d ]",
			       i + j, risc->cpu[i + j], j));
		if (risc->cpu[i] == cpu_to_le32(RISC_JUMP))
			break;
	}
}


void CX23885_Start_DMA(PKSDEVICE device, cx23885_tsport *port)
{
	KIRQL oldIrql;
	KeAcquireSpinLock(&port->slock, &oldIrql);
	/* Stop the fifo and risc engine for this port */
	cx_clear(port->reg_ts_int_msk, port->ts_int_msk_val);
	cx_clear(port->reg_dma_ctl, port->dma_ctl_val);

	/* Reset DMA queue */
	cx23885_dmaqueue * q = &port->mpegq;
	q->count = 1;
	if(IsListEmpty(&q->queued) && IsListEmpty(&q->active))
	{
		KdPrint((LOG_PREFIX "Start_DMA: MPEG queue is empty"));
		return;
	}
	LIST_ENTRY * entry;
	while(!IsListEmpty(&q->active))
	{
		entry = RemoveHeadList(&q->active);
		InsertTailList(&q->queued, entry);
	}
	cx23885_buffer * buff = NULL, * prev = NULL;
	while(!IsListEmpty(&q->queued))
	{
		entry = RemoveHeadList(&q->queued);
		buff = CONTAINING_RECORD(entry, cx23885_buffer, link);
		buff->count = q->count++;
		if(prev)
		{
			prev->risc.jmp[1] = buff->risc.dma;
		}
		InsertTailList(&q->active, &buff->link);
		prev = buff;
	}
	if(prev)
	{
		prev->risc.jmp[1] = q->stopper.dma;
	}
	
	cx23885_buffer * buf = CONTAINING_RECORD(q->active.Flink, cx23885_buffer, link);
	q->starter.jmp[1] = buf->risc.dma;

	/* setup fifo + format */
	CX23885_SRAM_Channel_Setup(device,
				   &((sram_channel *)(GETCONTEXT(device)->sram_channels))[port->sram_chno],
				   port->ts_packet_size, q->starter.dma);

#ifdef NETUP_EXTRA_DBG
	cx23885_tsport_reg_dump(device, port);
#endif

	/* write TS length to chip */
	cx_write(port->reg_lngth, port->ts_packet_size);

	DelayMicroseconds(100);

	/* If the port supports SRC SELECT, configure it */
	if (port->reg_src_sel)
		cx_write(port->reg_src_sel, port->src_sel_val);

	cx_write(port->reg_hw_sop_ctrl, port->hw_sop_ctrl_val);
	cx_write(port->reg_ts_clk_en, port->ts_clk_en_val);
	cx_write(port->reg_vld_misc, port->vld_misc_val);
	cx_write(port->reg_gen_ctrl, port->gen_ctrl_val);
	cx_write(port->reg_bd_pkt_status, 0);
	cx_write(port->reg_fifo_ovfl_stat, 0);

	DelayMicroseconds(100);

	/* NOTE: this is 2 (reserved) for portb, does it matter? */
	/* reset counter to zero */
	cx_write(port->reg_gpcnt_ctl, 3);

	/* Set VIDB pins to input */
	
	ULONG reg;

	reg = cx_read(PAD_CTRL);
	reg &= ~0x3; /* Clear TS1_OE & TS1_SOP_OE */
	cx_write(PAD_CTRL, reg);

	/* Set VIDC pins to input */
	reg = cx_read(PAD_CTRL);
	reg &= ~0x4; /* Clear TS2_SOP_OE */
	cx_write(PAD_CTRL, reg);

	/* enable irqs */
	KdPrint((LOG_PREFIX "%s() enabling TS int's and DMA\n", __FUNCTION__));
	port->running = TRUE;
	reg = cx_read(port->reg_ts_int_stat);
	cx_clear(port->reg_ts_int_stat, reg);
	cx_set(port->reg_ts_int_msk, port->ts_int_msk_val);
	cx_set(port->reg_dma_ctl, 0x10);
	CX23885_IRQ_Add(device, port->pci_irqmask);
	CX23885_IRQ_Enable_All(device);
	cx_set(DEV_CNTRL2, (1<<5)); /* Enable RISC controller */

	// XXX
	pmt_sent = FALSE;

	KeReleaseSpinLock(&port->slock, oldIrql);
}

void CX23885_Stop_DMA(PKSDEVICE device, cx23885_tsport * port)
{
	KIRQL oldIrql;
	KeAcquireSpinLock(&port->slock, &oldIrql);
	KdPrint((LOG_PREFIX "%s: (potno=%d)", __FUNCTION__, port->nr));
	KeRemoveQueueDpc(&port->dpc);
	cx_clear(port->reg_ts_int_msk, port->ts_int_msk_val);
	cx_clear(port->reg_dma_ctl, port->dma_ctl_val);
	port->running = FALSE;
	KeReleaseSpinLock(&port->slock, oldIrql);
}

static void CX23885_Init_TSport(PKSDEVICE device, cx23885_tsport *port, int portno)
{
	KdPrint((LOG_PREFIX "Init_TSport (portno=%d)", portno));

	KeInitializeDpc(&port->dpc,CX23885_DPCRoutine,device);
	KeSetImportanceDpc(&port->dpc, HighImportance);
	KeInitializeSpinLock(&port->slock);

	/* Transport bus init dma queue  - Common settings */
	port->dma_ctl_val        = 0x11; /* Enable RISC controller and Fifo */
	port->ts_int_msk_val     = 0x1111; /* TS port bits for RISC */
	port->vld_misc_val       = 0x0;
	port->hw_sop_ctrl_val    = (0x47 << 16 | 188 << 4);
	port->nr = portno;

	switch (portno) {
	case 1:
		port->reg_gpcnt          = VID_B_GPCNT;
		port->reg_gpcnt_ctl      = VID_B_GPCNT_CTL;
		port->reg_dma_ctl        = VID_B_DMA_CTL;
		port->reg_lngth          = VID_B_LNGTH;
		port->reg_hw_sop_ctrl    = VID_B_HW_SOP_CTL;
		port->reg_gen_ctrl       = VID_B_GEN_CTL;
		port->reg_bd_pkt_status  = VID_B_BD_PKT_STATUS;
		port->reg_sop_status     = VID_B_SOP_STATUS;
		port->reg_fifo_ovfl_stat = VID_B_FIFO_OVFL_STAT;
		port->reg_vld_misc       = VID_B_VLD_MISC;
		port->reg_ts_clk_en      = VID_B_TS_CLK_EN;
		port->reg_src_sel        = VID_B_SRC_SEL;
		port->reg_ts_int_msk     = VID_B_INT_MSK;
		port->reg_ts_int_stat    = VID_B_INT_STAT;
		port->sram_chno          = SRAM_CH03; /* VID_B */
		port->pci_irqmask        = 0x02; /* VID_B bit1 */
		break;
	case 2:
		port->reg_gpcnt          = VID_C_GPCNT;
		port->reg_gpcnt_ctl      = VID_C_GPCNT_CTL;
		port->reg_dma_ctl        = VID_C_DMA_CTL;
		port->reg_lngth          = VID_C_LNGTH;
		port->reg_hw_sop_ctrl    = VID_C_HW_SOP_CTL;
		port->reg_gen_ctrl       = VID_C_GEN_CTL;
		port->reg_bd_pkt_status  = VID_C_BD_PKT_STATUS;
		port->reg_sop_status     = VID_C_SOP_STATUS;
		port->reg_fifo_ovfl_stat = VID_C_FIFO_OVFL_STAT;
		port->reg_vld_misc       = VID_C_VLD_MISC;
		port->reg_ts_clk_en      = VID_C_TS_CLK_EN;
		port->reg_src_sel        = 0;
		port->reg_ts_int_msk     = VID_C_INT_MSK;
		port->reg_ts_int_stat    = VID_C_INT_STAT;
		port->sram_chno          = SRAM_CH06; /* VID_C */
		port->pci_irqmask        = 0x04; /* VID_C bit2 */
		break;
	default:
		ASSERT(0);
	}

	// initialize queue
	InitializeListHead(&port->mpegq.active);
	InitializeListHead(&port->mpegq.queued);
	port->mpegq.count = 1;
	
	// initialize tool programs
	CX23885_RISC_Starter(device, &port->mpegq.starter,
		     port->reg_dma_ctl, 0x01, 0x01);
	CX23885_RISC_Stopper(device, &port->mpegq.stopper,
		     port->reg_dma_ctl, port->dma_ctl_val, 0x00);

	port->ts_packet_size = 188 * 4;
	port->ts_packet_count = 32;

	// prepare buffers
	for(ULONG i = 0; i < MPEG_QUEUE_SIZE; i++)
	{
		cx23885_buffer * buff = CX23885_Buffer_Prepare(device, port);
		ASSERT(buff != NULL);
		InsertTailList(&(port->mpegq.queued), &buff->link);
	}
}

void CX23885_Card_Setup(PKSDEVICE device)
{
	cx23885_tsport *ts1 = (cx23885_tsport *)(GETCONTEXT(device)->ts1);
	cx23885_tsport *ts2 = (cx23885_tsport *)(GETCONTEXT(device)->ts2);

	ts1->gen_ctrl_val  = 0xc; /* Serial bus + punctured clock */
	ts1->ts_clk_en_val = 0x1; /* Enable TS_CLK */
	ts1->src_sel_val   = CX23885_SRC_SEL_PARALLEL_MPEG_VIDEO;

	ts2->gen_ctrl_val  = 0xc; /* Serial bus + punctured clock */
	ts2->ts_clk_en_val = 0x1; /* Enable TS_CLK */
	ts2->src_sel_val   = CX23885_SRC_SEL_PARALLEL_MPEG_VIDEO;

	NetupCardInit(device);
}

BOOLEAN CX23885_DVB_Init(PKSDEVICE device)
{
	BOOLEAN ret;
	KdPrint((LOG_PREFIX "DVB_Init: *** attaching DVB frontends ***"));
	ret = (STV0900_Init(device) == STATUS_SUCCESS);
	ret &= (STV6110_Init(device) == STATUS_SUCCESS);
	ret &= (LNBH24_Init(device) == STATUS_SUCCESS);

	static struct netup_card_info cinfo;

	Netup_Get_Card_Info(device, 0, &cinfo);
	DbgPrint(LOG_PREFIX "NetUP Dual DVB-S2 CI card rev %d", cinfo.rev);
	DbgPrint(LOG_PREFIX "MAC1=%02x:%02x:%02x:%02x:%02x:%02x", 
		cinfo.mac1[0],cinfo.mac1[1],cinfo.mac1[2],
		cinfo.mac1[3],cinfo.mac1[4],cinfo.mac1[5]);
	DbgPrint(LOG_PREFIX "MAC2=%02x:%02x:%02x:%02x:%02x:%02x",
		cinfo.mac2[0],cinfo.mac2[1],cinfo.mac2[2],
		cinfo.mac2[3],cinfo.mac2[4],cinfo.mac2[5]);


	ret &= Netup_CI_Init(device, (cx23885_tsport *)(GETCONTEXT(device)->ts1));
	ret &= Netup_CI_Init(device, (cx23885_tsport *)(GETCONTEXT(device)->ts2));

	DbgPrint(LOG_PREFIX "DVB_Init: %s", ((ret == TRUE) ? "OK" : "failed"));

	return ret;
}

NTSTATUS CX23885_Init(PKSDEVICE device)
{
	NTSTATUS result = STATUS_SUCCESS;
	cx23885_tsport * ts1, * ts2;
	cx23885_i2c * i2c_bus;
	ts1 = (cx23885_tsport *)ExAllocatePoolWithTag(NonPagedPool, sizeof(cx23885_tsport), '1PST');
	if(!ts1)
	{
		KdPrint((LOG_PREFIX "Unable to allocate TSPort1"));
		return STATUS_INSUFFICIENT_RESOURCES;
	}
	RtlZeroMemory(ts1, sizeof(*ts1));
	ts2 = (cx23885_tsport *)ExAllocatePoolWithTag(NonPagedPool, sizeof(cx23885_tsport), '2PST');
	if(!ts2)
	{
		ExFreePool(ts1);
		KdPrint((LOG_PREFIX "Unable to allocate TSPort2"));
		return STATUS_INSUFFICIENT_RESOURCES;
	}
	RtlZeroMemory(ts2, sizeof(*ts2));
	i2c_bus = (cx23885_i2c *)ExAllocatePoolWithTag(NonPagedPool, sizeof(cx23885_i2c)*3, 'BC2I');
	if(!i2c_bus)
	{
		ExFreePool(ts1);
		ExFreePool(ts2);
		KdPrint((LOG_PREFIX "Unable to allocate I2C bus structures"));
		return STATUS_INSUFFICIENT_RESOURCES;
	}
	RtlZeroMemory(i2c_bus, 3*sizeof(*i2c_bus));

	KeInitializeSpinLock(&(GETCONTEXT(device)->pci_irqmask_lock));

	GETCONTEXT(device)->clk_freq = 28000000;
	GETCONTEXT(device)->sram_channels = cx23885_sram_channels;
	GETCONTEXT(device)->ts1 = ts1;
	GETCONTEXT(device)->ts2 = ts2;
	GETCONTEXT(device)->i2c_bus = i2c_bus;
	CX23885_IRQ_Add(device, 0x001f00);

	/* External Master 1 Bus */
	i2c_bus[0].nr = 0;
	i2c_bus[0].reg_stat  = I2C1_STAT;
	i2c_bus[0].reg_ctrl  = I2C1_CTRL;
	i2c_bus[0].reg_addr  = I2C1_ADDR;
	i2c_bus[0].reg_rdata = I2C1_RDATA;
	i2c_bus[0].reg_wdata = I2C1_WDATA;
	i2c_bus[0].i2c_period = (0x9d << 24); /* 100kHz */
	KeInitializeSpinLock(&i2c_bus[0].lock);

	/* External Master 2 Bus */
	i2c_bus[1].nr = 1;
	i2c_bus[1].reg_stat  = I2C2_STAT;
	i2c_bus[1].reg_ctrl  = I2C2_CTRL;
	i2c_bus[1].reg_addr  = I2C2_ADDR;
	i2c_bus[1].reg_rdata = I2C2_RDATA;
	i2c_bus[1].reg_wdata = I2C2_WDATA;
	i2c_bus[1].i2c_period = (0x9d << 24); /* 100kHz */
	KeInitializeSpinLock(&i2c_bus[1].lock);

	/* Internal Master 3 Bus */
	i2c_bus[2].nr = 2;
	i2c_bus[2].reg_stat  = I2C3_STAT;
	i2c_bus[2].reg_ctrl  = I2C3_CTRL;
	i2c_bus[2].reg_addr  = I2C3_ADDR;
	i2c_bus[2].reg_rdata = I2C3_RDATA;
	i2c_bus[2].reg_wdata = I2C3_WDATA;
	i2c_bus[2].i2c_period = (0x07 << 24); /* 1.95MHz */
	KeInitializeSpinLock(&i2c_bus[2].lock);

	CX23885_Init_TSport(device, ts1, 1);
	CX23885_Init_TSport(device, ts2, 2);
	CX23885_PCI_Quirks(device);
	CX23885_Reset(device);
	CX23885_Card_Setup(device);

	if(!CX23885_DVB_Init(device))
	{
		return STATUS_DEVICE_NOT_CONNECTED;
	}

	/* disable MSI for NetUP cards, otherwise CI is not working */
	cx_clear(RDR_RDRCTL1, 1 << 8);

	CX23885_IRQ_Add_Enable(device, PCI_MSK_GPIO1 | PCI_MSK_GPIO0);

	return result;
}

void CX23885_Close(PKSDEVICE device)
{
	CX23885_Shutdown(device);
}

static void cx23885_tsport_dump(PKSDEVICE device, struct cx23885_tsport *port)
{
	cx23885_tsport_reg_dump(device, port);
	cx23885_sram_channel_dump(device, &((sram_channel *)(GETCONTEXT(device)->sram_channels))[port->sram_chno]);
}


/* 

// PAT parser data
static ULONG program_count = 0;
static USHORT pmt_pids[64];


static VOID PMT_Parser(PUCHAR buf, ULONG size, USHORT pid)
{
	if(buf[0] != 2)
	{
		KdPrint(("PMT: invalid table_id %d", (ULONG)buf[0]));
		return;
	}
	USHORT section_length = (((USHORT)buf[1] & 0xf) << 8) | (USHORT)buf[2];
	USHORT program_number = ((USHORT)buf[3] << 8) | (USHORT)buf[4];
	USHORT pcr_pid = (((USHORT)buf[8] << 8) | (USHORT)buf[9]) & 0x1fff;
	USHORT prog_info_length = (((USHORT)buf[10] << 8) | (USHORT)buf[11]) & 0xfff;
	KdPrint(("PMT PID 0x%04x section length: %d", (ULONG)pid, section_length));
	KdPrint(("Program number 0x%04x", (ULONG)program_number));
	KdPrint(("PCR PID 0x%04x", (ULONG)pcr_pid));
	KdPrint(("Program Info length=%d dump:", (ULONG)prog_info_length));
	RawDump(buf + 12, prog_info_length);
	PUCHAR ptr = buf + 12 + prog_info_length;
	USHORT len = section_length - (prog_info_length + 4 + 9);
	USHORT start = 0;
	while(start < len)
	{
		UCHAR type = ptr[start];
		USHORT stream_pid = (((USHORT)ptr[start + 1] << 8) | (USHORT)ptr[start + 2]) & 0x1fff;
		USHORT es_info_len = (((USHORT)ptr[start + 3] << 8) | (USHORT)ptr[start + 4]) & 0xfff;
		KdPrint(("Stream type 0x%02x PID 0x%04x ES_info_len %d Data:", (ULONG)type, (ULONG)stream_pid, (ULONG)es_info_len));
		RawDump(&ptr[start + 5], es_info_len);
		start += es_info_len + 5;
	}
}

static VOID PAT_Parser(PUCHAR buf, ULONG size)
{
	if(buf[0] != 0)
	{
		KdPrint(("PAT: invalid table_id %d", (ULONG)buf[0]));
		return;
	}
	ULONG section_length = (((ULONG)buf[1] & 0xf) << 8) | (ULONG)buf[2];
	KdPrint(("PAT section length: %d", section_length));
	program_count = (section_length - 9)/4;
	PUCHAR tmp = buf + 8;
	for(ULONG i = 0; i < program_count; i++)
	{
		USHORT program_id = ((USHORT)buf[i*4] << 8) | (USHORT)buf[i*4 + 1];
		USHORT pid = (((USHORT)buf[i*4+2] << 8) | (USHORT)buf[i*4 + 3]) & 0x1fff;
		KdPrint((LOG_PREFIX "program ID 0x%04x PID 0x%04x", (ULONG)program_id, (ULONG)pid));
		pmt_pids[i] = pid;
	}
}

static VOID Table_Filter(PKSDEVICE device, PUCHAR buf, ULONG size)
{
	if(pmt_sent)
	{
		return;
	}
	LONG start = -1;
	for(ULONG i = 0; i < size; i++)
	{
		if(buf[i] == 0x47)
		{
			start = i;
			break;
		}
	}
	if(start == -1)
		return;
	ULONG count = 0;
	while(start + 188 <= size)
	{
		if(buf[start] != 0x47)
		{
			KdPrint((LOG_PREFIX "%s: bad signature at %d", __FUNCTION__, count));
			break;
		}
		USHORT pid = ((buf[start + 1]) & 0x1f) << 8 | buf[start + 2];
		if((pid == 0 && program_count == 0) || pid == 0x8c)
		{
			UCHAR adaptation_field_ctl = (buf[start + 3] >> 4) & 0x03;
			UCHAR continuity_cnt = buf[start + 3] & 0x0f;
			ULONG adaptation_field_size = 0;
			if(adaptation_field_ctl == 0x02)
			{
				adaptation_field_size = buf[start + 4];
				if(adaptation_field_size != 183)
				{
					KdPrint((LOG_PREFIX "%s: adaptation_field_ctl == 0x02, adaptation_field_size %d != 183", __FUNCTION__, adaptation_field_size));
					return;
				}
			}
			else if(adaptation_field_ctl == 0x03)
			{
				adaptation_field_size = buf[start + 4];
				if(adaptation_field_size > 182)
				{
					KdPrint((LOG_PREFIX "%s: adaptation_field_ctl == 0x03, adaptation_field_size %d > 182", __FUNCTION__, adaptation_field_size));
					return;
				}
			}
			ULONG payload_size = 188 - (adaptation_field_size + 5);
			KdPrint(("----- Table (PAT/PMT) %d dump (cnt %d) -----", pid, (ULONG)continuity_cnt));
			RawDump(&buf[start + 5 + adaptation_field_size], payload_size);
			if(pid == 0)
			{
				PAT_Parser(&buf[start + 5 + adaptation_field_size], payload_size);
			}
			else
			{
				EN50221_APP_CA_PMT_Process(GETCONTEXT(device)->en50221_context[1], &buf[start + 5 + adaptation_field_size], payload_size);
				pmt_sent = TRUE;
			}
		}
		start += 188;
		count++;
	}
}

static ULONG counter = 0;

*/

static void CX23885_Submit_Buffers(PKSDEVICE device, PKSPIN pin, cx23885_tsport * port, LIST_ENTRY * buffers)
{
	if(pin == NULL)
		return;
	LIST_ENTRY * entry = buffers->Flink;
	cx23885_buffer * buff;
	PKSSTREAM_POINTER stream_pointer;
	while(entry != buffers)
	{
		buff = CONTAINING_RECORD(entry, cx23885_buffer, link);
		stream_pointer=KsPinGetLeadingEdgeStreamPointer(pin, KSSTREAM_POINTER_STATE_LOCKED);
		if(stream_pointer != NULL)
		{
			RtlCopyMemory(stream_pointer->OffsetOut.Data, buff->data.cpu, buff->data.size);
			KsStreamPointerAdvanceOffsetsAndUnlock(stream_pointer,0,buff->data.size,FALSE);
		}
		entry = entry->Flink;
	}
}

static BOOLEAN CX23885_GetQueuedBuffers(PVOID context)
{
	LIST_ENTRY * queued = (LIST_ENTRY *)(((PVOID *)context)[0]);
	LIST_ENTRY * result = (LIST_ENTRY *)(((PVOID *)context)[1]);
	LIST_ENTRY * current;
	while(!IsListEmpty(queued))
	{
		current = RemoveHeadList(queued);
		InsertTailList(result, current);
	}
	return TRUE;
}

static BOOLEAN CX23885_PutQueuedBuffers(PVOID context)
{
	cx23885_dmaqueue * q = (cx23885_dmaqueue *)(((PVOID *)context)[0]);
	LIST_ENTRY * buffers = (LIST_ENTRY *)(((PVOID *)context)[1]);
	LIST_ENTRY * curr;
	if(IsListEmpty(&q->active))
	{
		KdPrint((LOG_PREFIX "%s: RISC engine stopped", __FUNCTION__));
		while(!IsListEmpty(buffers))
		{
			curr = RemoveHeadList(buffers);
			InsertTailList(&q->queued, curr);
		}
		(((PVOID *)context)[2]) = (PVOID)(-1);
	}
	else
	{
		cx23885_buffer * buff, * prev;
		while(!IsListEmpty(buffers))
		{
			curr = RemoveHeadList(buffers);
			buff = CONTAINING_RECORD(curr, cx23885_buffer, link);
			ULONG old_cnt = buff->count;
			buff->count = q->count++;
			buff->risc.jmp[1] = q->stopper.dma;
			ASSERT(!IsListEmpty(&q->active));
			prev = CONTAINING_RECORD(q->active.Blink, cx23885_buffer, link);
			prev->risc.jmp[1] = buff->risc.dma;
			//KdPrint((LOG_PREFIX "adding buff (old_cnt=%d, new_cnt=%d, prev_cnt=%d) into active", old_cnt, buff->count, prev->count));
			InsertTailList(&q->active, &buff->link);
		}
	}
	return TRUE;
}

VOID CX23885_OutputProcess(PKSDEVICE device, PKSPIN pin, PVOID portPtr)
{
	cx23885_tsport * port = (cx23885_tsport *)portPtr;

	LIST_ENTRY result;
	InitializeListHead(&result);
	PVOID context[3];

	// move queued buffers into tmp list
	context[0] = &(port->mpegq.queued);
	context[1] = &result;
	context[2] = NULL;
	KeSynchronizeExecution(GETCONTEXT(device)->pci_interrupt, CX23885_GetQueuedBuffers, &context[0]);
	// process buffers here
	CX23885_Submit_Buffers(device, pin, port, &result);
	// move queued buffers into active queue
	context[0] = &(port->mpegq);
	KeSynchronizeExecution(GETCONTEXT(device)->pci_interrupt, CX23885_PutQueuedBuffers, &context[0]);
	if(context[2] != NULL && port->running)
	{
		KdPrint((LOG_PREFIX "restarting queue"));
		CX23885_Start_DMA(device, port);
	}
}

void CX23885_DPCRoutine(PKDPC Dpc,PVOID DeferredContext,PVOID SystemArgument1,PVOID SystemArgument2)
{
	PKSDEVICE device = (PKSDEVICE)DeferredContext;
	cx23885_tsport * port = (cx23885_tsport *)SystemArgument1;
	PKSPIN pin = GETCONTEXT(device)->output_pin[port->nr - 1];
	if(pin == NULL)
	{
		return;
	}
	KsPinAttemptProcessing(pin, TRUE);
}

static BOOL CX23885_ProcessQueue(PKSDEVICE device, cx23885_dmaqueue * q, ULONG gpcount)
{
	ULONG done = 0;
	while(!IsListEmpty(&q->active))
	{
		cx23885_buffer * buff = CONTAINING_RECORD(q->active.Flink, cx23885_buffer, link);
		if((SHORT)(gpcount - buff->count) < 0)
			break;
		RemoveHeadList(&q->active);
		InsertTailList(&q->queued, &buff->link);
		done++;
		
	}
	if(IsListEmpty(&q->active))
	{
		KdPrint((LOG_PREFIX "%s: last buffer queued, gpcnt=%d", __FUNCTION__, gpcount));
	}
	return (done > 0);
}

static BOOL CX23885_IRQ_TS(PKSDEVICE device, struct cx23885_tsport *port, ULONG ts_status)
{
	ULONG ts_int_msk_val = cx_read(port->reg_ts_int_msk);
	cx_write(port->reg_ts_int_msk, 0);

	if (ts_status & (VID_BC_MSK_OPC_ERR | VID_BC_MSK_BAD_PKT | VID_BC_MSK_SYNC | VID_BC_MSK_OF))
	{
		KdPrint((LOG_PREFIX "MPEG RISC program error:"));

		if (ts_status & VID_BC_MSK_OPC_ERR)
		{
			KdPrint((LOG_PREFIX " (VID_BC_MSK_OPC_ERR 0x%08x)",
				VID_BC_MSK_OPC_ERR));
		}

		if (ts_status & VID_BC_MSK_BAD_PKT)
		{
			KdPrint((LOG_PREFIX " (VID_BC_MSK_BAD_PKT 0x%08x)",
				VID_BC_MSK_BAD_PKT));
		}

		if (ts_status & VID_BC_MSK_SYNC)
		{
			KdPrint((LOG_PREFIX " (VID_BC_MSK_SYNC    0x%08x)",
				VID_BC_MSK_SYNC));
		}

		if (ts_status & VID_BC_MSK_OF)
		{
			KdPrint((LOG_PREFIX " (VID_BC_MSK_OF      0x%08x)",
				VID_BC_MSK_OF));
		}

		/* stop RISC engine & FIFO */
		cx_clear(port->reg_dma_ctl, port->dma_ctl_val);
		cx23885_tsport_dump(device, port);
	}
	else
	{
		ULONG gpcount = cx_read(port->reg_gpcnt);
		if(ts_status & (VID_BC_MSK_RISCI1 | VID_BC_MSK_RISCI2))
		{
			if(CX23885_ProcessQueue(device, &port->mpegq, gpcount))
			{
				// process queued buffers in DPC
				KeInsertQueueDpc(&port->dpc, port, NULL);
			}
		}
		if(ts_status & VID_BC_MSK_RISCI2)
		{
			KdPrint((LOG_PREFIX "MPEG program done, gpcount=%d", gpcount));
		}
	}

	cx_write(port->reg_ts_int_msk, ts_int_msk_val);
	cx_write(port->reg_ts_int_stat, ts_status);

	return TRUE;
}

BOOLEAN CX23885_Interrupt(PKSDEVICE device)
{
	ULONG pci_status = cx_read(PCI_INT_STAT);
	if(pci_status == 0)
	{
		return FALSE;
	}

	ULONG handled = 0;

	if(pci_status & (1 << 16))
	{
		ULONG state = cx_read(I2C1_STAT);
		handled++;
#if NETUP_EXTA_DBG
		KdPrint((LOG_PREFIX "Got I2C 1 Int"));
		KdPrint((LOG_PREFIX "i2c1_stat=%x", state));
#endif
	}
	if(pci_status & (1 << 18))
	{
		ULONG state = cx_read(I2C2_STAT);
		handled++;
#if NETUP_EXTRA_DBG
		KdPrint((LOG_PREFIX "Got I2C 2 Int"));
		KdPrint((LOG_PREFIX "i2c2_stat=%x", state));
#endif
	}
	if(pci_status & (1 << 20))
	{
		ULONG state = cx_read(I2C3_STAT);
		handled++;
#if NETUP_EXTRA_DBG
		KdPrint((LOG_PREFIX "Got I2C 3 Int"));
		KdPrint((LOG_PREFIX "i2c3_stat=%x", state));
#endif
	}

	ULONG ts_status;
	cx23885_tsport * port;

	if(pci_status & PCI_MSK_VID_B)
	{
		ts_status = cx_read(VID_B_INT_STAT);
		if(ts_status != 0)
		{
			port = (cx23885_tsport *)(GETCONTEXT(device)->ts1);
			handled += CX23885_IRQ_TS(device, port, ts_status);
		}
	}
	if(pci_status & PCI_MSK_VID_C)
	{
		ts_status = cx_read(VID_C_INT_STAT);
		if(ts_status != 0)
		{
			port = (cx23885_tsport *)(GETCONTEXT(device)->ts2);
			handled += CX23885_IRQ_TS(device, port, ts_status);
		}
	}

#ifdef NETUP_EXTRA_DBG
	if (pci_status & (PCI_MSK_RISC_RD | PCI_MSK_RISC_WR |
			  PCI_MSK_AL_RD   | PCI_MSK_AL_WR   | PCI_MSK_APB_DMA |
			  PCI_MSK_VID_C   | PCI_MSK_VID_B   | PCI_MSK_VID_A   |
			  PCI_MSK_AUD_INT | PCI_MSK_AUD_EXT |
			  PCI_MSK_GPIO0   | PCI_MSK_GPIO1   |
			  PCI_MSK_AV_CORE | PCI_MSK_IR)) {

		if (pci_status & PCI_MSK_RISC_RD)
			KdPrint((LOG_PREFIX " (PCI_MSK_RISC_RD   0x%08x)",
				PCI_MSK_RISC_RD));

		if (pci_status & PCI_MSK_RISC_WR)
			KdPrint((LOG_PREFIX " (PCI_MSK_RISC_WR   0x%08x)",
				PCI_MSK_RISC_WR));

		if (pci_status & PCI_MSK_AL_RD)
			KdPrint((LOG_PREFIX " (PCI_MSK_AL_RD     0x%08x)",
				PCI_MSK_AL_RD));

		if (pci_status & PCI_MSK_AL_WR)
			KdPrint((LOG_PREFIX " (PCI_MSK_AL_WR     0x%08x)",
				PCI_MSK_AL_WR));

		if (pci_status & PCI_MSK_APB_DMA)
			KdPrint((LOG_PREFIX " (PCI_MSK_APB_DMA   0x%08x)",
				PCI_MSK_APB_DMA));

		if (pci_status & PCI_MSK_VID_C)
			KdPrint((LOG_PREFIX " (PCI_MSK_VID_C     0x%08x)",
				PCI_MSK_VID_C));

		if (pci_status & PCI_MSK_VID_B)
			KdPrint((LOG_PREFIX " (PCI_MSK_VID_B     0x%08x)",
				PCI_MSK_VID_B));

		if (pci_status & PCI_MSK_VID_A)
			KdPrint((LOG_PREFIX " (PCI_MSK_VID_A     0x%08x)",
				PCI_MSK_VID_A));

		if (pci_status & PCI_MSK_AUD_INT)
			KdPrint((LOG_PREFIX " (PCI_MSK_AUD_INT   0x%08x)",
				PCI_MSK_AUD_INT));

		if (pci_status & PCI_MSK_AUD_EXT)
			KdPrint((LOG_PREFIX " (PCI_MSK_AUD_EXT   0x%08x)",
				PCI_MSK_AUD_EXT));

		if (pci_status & PCI_MSK_GPIO0)
			KdPrint((LOG_PREFIX " (PCI_MSK_GPIO0     0x%08x)",
				PCI_MSK_GPIO0));

		if (pci_status & PCI_MSK_GPIO1)
			KdPrint((LOG_PREFIX " (PCI_MSK_GPIO1     0x%08x)",
				PCI_MSK_GPIO1));

		if (pci_status & PCI_MSK_AV_CORE)
			KdPrint((LOG_PREFIX " (PCI_MSK_AV_CORE   0x%08x)",
				PCI_MSK_AV_CORE));

		if (pci_status & PCI_MSK_IR)
			KdPrint((LOG_PREFIX " (PCI_MSK_IR        0x%08x)",
				PCI_MSK_IR));
	}
#endif

	if(pci_status & (PCI_MSK_GPIO1 | PCI_MSK_GPIO0))
	{
		handled += Netup_CI_Slot_Status(device, pci_status);
	}

	if (pci_status & PCI_MSK_IR)
	{
		KdPrint((LOG_PREFIX "warn: IR port IRQ"));
	}

	if (pci_status & PCI_MSK_AV_CORE)
	{
		CX23885_IRQ_Disable(device, PCI_MSK_AV_CORE);
		KdPrint((LOG_PREFIX "warn: AV-core IRQ"));
	}

	if (handled > 0)
	{
		cx_write(PCI_INT_STAT, pci_status);
		return TRUE;
	}

	return FALSE;
}


