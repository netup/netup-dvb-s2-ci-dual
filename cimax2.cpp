/*
 * cimax2.cpp
 *
 * CIMax2(R) SP2 driver in conjunction with NetUp Dual DVB-S2 CI card
 *
 * Copyright (C) 2011 NetUP Inc.
 * Copyright (C) 2011 Sergey Kozlov <serjk@netup.ru>
 * Copyright (C) 2009 Igor M. Liplianin <liplianin@netup.ru>
 * Copyright (C) 2009 Abylay Ospan <aospan@netup.ru>
 *
 * Parts of this file were based on sources as follows:
 * Copyright (C) 2004 Andrew de Quincey
 * Copyright (C) 2003 Ralph Metzler <rjkm@metzlerbros.de>
 * Copyright (C) 1999-2002 Ralph  Metzler
 *                       & Marcus Metzler for convergence integrated media GmbH
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

#include "device.h"
#include "cx23885.h"
#include "cx23885-reg.h"
#include "i2c.h"
#include "en50221.h"

/**** Bit definitions for MC417_RWD and MC417_OEN registers  ***
  bits 31-16
+-----------+
| Reserved  |
+-----------+
  bit 15  bit 14  bit 13 bit 12  bit 11  bit 10  bit 9   bit 8
+-------+-------+-------+-------+-------+-------+-------+-------+
|  WR#  |  RD#  |       |  ACK# |  ADHI |  ADLO |  CS1# |  CS0# |
+-------+-------+-------+-------+-------+-------+-------+-------+
 bit 7   bit 6   bit 5   bit 4   bit 3   bit 2   bit 1   bit 0
+-------+-------+-------+-------+-------+-------+-------+-------+
|  DATA7|  DATA6|  DATA5|  DATA4|  DATA3|  DATA2|  DATA1|  DATA0|
+-------+-------+-------+-------+-------+-------+-------+-------+
***/
/* MC417 */
#define NETUP_DATA		0x000000ff
#define NETUP_WR		0x00008000
#define NETUP_RD		0x00004000
#define NETUP_ACK		0x00001000
#define NETUP_ADHI		0x00000800
#define NETUP_ADLO		0x00000400
#define NETUP_CS1		0x00000200
#define NETUP_CS0		0x00000100
#define NETUP_EN_ALL		0x00001000
#define NETUP_CTRL_OFF		(NETUP_CS1 | NETUP_CS0 | NETUP_WR | NETUP_RD)
#define NETUP_CI_CTL		0x04
#define NETUP_CI_RD		1

#define NETUP_IRQ_DETAM 	0x1
#define NETUP_IRQ_IRQAM		0x4

#define DVB_CA_SLOTSTATE_NONE           0
#define DVB_CA_SLOTSTATE_UNINITIALISED  1
#define DVB_CA_SLOTSTATE_RUNNING        2
#define DVB_CA_SLOTSTATE_INVALID        3
#define DVB_CA_SLOTSTATE_WAITREADY      4
#define DVB_CA_SLOTSTATE_VALIDATE       5
#define DVB_CA_SLOTSTATE_WAITFR         6
#define DVB_CA_SLOTSTATE_LINKINIT       7
#define DVB_CA_SLOTSTATE_SHUTDOWN		8

#define CTRLIF_DATA      0
#define CTRLIF_COMMAND   1
#define CTRLIF_STATUS    1
#define CTRLIF_SIZE_LOW  2
#define CTRLIF_SIZE_HIGH 3

#define CMDREG_HC        1	/* Host control */
#define CMDREG_SW        2	/* Size write */
#define CMDREG_SR        4	/* Size read */
#define CMDREG_RS        8	/* Reset interface */
#define CMDREG_FRIE   0x40	/* Enable FR interrupt */
#define CMDREG_DAIE   0x80	/* Enable DA interrupt */
#define IRQEN (CMDREG_DAIE)

#define STATUSREG_RE     1	/* read error */
#define STATUSREG_WE     2	/* write error */
#define STATUSREG_FR  0x40	/* module free */
#define STATUSREG_DA  0x80	/* data available */
#define STATUSREG_TXERR (STATUSREG_RE|STATUSREG_WE)	/* general transfer error */

#define INIT_TIMEOUT_SECS 10
#define READ_TIMEOUT_SECS 10

#define HOST_LINK_BUF_SIZE 0x200

#define CAM_IO_ERROR		-1
#define CAM_INVALID_STATE	-2
#define CAM_RETRY			-10

struct CAM_Packet
{
	PUCHAR data;
	ULONG size;
	LIST_ENTRY link;
};

struct netup_ci_state {
	PKSDEVICE device;
	cx23885_tsport * port;

	KEVENT event_kill;
	PKTHREAD thread;
	LARGE_INTEGER timeout;
	LARGE_INTEGER delay;
	LIST_ENTRY rx_buffer;

	// CI properties
	ULONG i2c_adap;
	UCHAR ci_i2c_addr;
	LONG state;
	PVOID priv;
	UCHAR current_irq_mode;
	LONG current_ci_flag;
	ULONG next_status_checked_time;
	ULONG cam_poll_state;
	ULONG link_buf_size;

	// CAM slot properties
	ULONG config_option;
	ULONG config_base;
};

static LONG dvb_ca_en50221_parse_attributes(netup_ci_state * ci);
static LONG dvb_ca_en50221_set_configoption(netup_ci_state * ci);
static LONG dvb_ca_en50221_link_init(netup_ci_state * ci);
static LONG dvb_ca_en50221_write_data(netup_ci_state * ci, PUCHAR buf, ULONG bytes_write);
static LONG dvb_ca_en50221_read_data(netup_ci_state *ci, PUCHAR ebuf, ULONG ecount);
VOID Test_CAM(netup_ci_state * ci);

static UCHAR ci_irq_flags() { return 0; }

BOOLEAN netup_read_i2c(PKSDEVICE device, ULONG i2c_adap, UCHAR addr, UCHAR reg, PUCHAR buf, ULONG len)
{
	LONG ret;
	struct i2c_msg msg[2];

	msg[0].addr	= addr;
	msg[0].flags	= 0;
	msg[0].buf	= &reg;
	msg[0].len	= 1;

	msg[1].addr	= addr;
	msg[1].flags	= I2C_M_RD;
	msg[1].buf	= buf;
	msg[1].len	= len;

	ret = i2c_transfer(device, i2c_adap, msg, 2);

	if (ret != 2)
	{
		return FALSE;
	}

	return TRUE;
}

BOOLEAN netup_write_i2c(PKSDEVICE device, ULONG i2c_adap, UCHAR addr, UCHAR reg, PUCHAR buf, ULONG len)
{
	LONG ret;
	PUCHAR buffer = (PUCHAR)ExAllocatePoolWithTag(NonPagedPool, len + 1, 'BC2I');
	if(!buffer)
		return FALSE;

	struct i2c_msg msg;
	msg.addr	= addr;
	msg.flags	= 0;
	msg.buf	= &buffer[0];
	msg.len	= len + 1;

	buffer[0] = reg;
	RtlCopyMemory(&buffer[1], buf, len);

	ret = i2c_transfer(device, i2c_adap, &msg, 1);

	ExFreePool(buffer);

	if (ret != 1)
	{
		return FALSE;
	}

	return TRUE;
}

LONG netup_ci_get_mem(PKSDEVICE device)
{
	LONG mem;
	ULONG timeout = 0;

	for (;;) {
		mem = cx_read(MC417_RWD);
		if ((mem & NETUP_ACK) == 0)
			break;
		if(timeout > 1000)
			break;
		DelayMicroseconds(1);
	}

	cx_set(MC417_RWD, NETUP_CTRL_OFF);

	return mem & 0xff;
}

LONG netup_ci_op_cam(PKSDEVICE device, cx23885_tsport * port, netup_ci_state * state, LONG slot,
				UCHAR flag, UCHAR read, LONG addr, UCHAR data)
{

	UCHAR store;
	LONG mem;
	LONG ret;

	if (0 != slot)
	{
		KdPrint((LOG_PREFIX "%s: slot != 0", __FUNCTION__));
		return -1;
	}

	if (state->current_ci_flag != flag)
	{
		if(!netup_read_i2c(device, state->i2c_adap, state->ci_i2c_addr,
				0, &store, 1))
		{
			KdPrint((LOG_PREFIX "%s: i2c read error", __FUNCTION__));
			return -1;
		}

		store &= ~0x0c;
		store |= flag;

		if(!netup_write_i2c(device, state->i2c_adap, state->ci_i2c_addr,
				0, &store, 1))
		{
			KdPrint((LOG_PREFIX "%s: i2c write error", __FUNCTION__));
			return -1;
		}
	}
	state->current_ci_flag = flag;

	//mutex_lock(&dev->gpio_lock);

	/* write addr */
	cx_write(MC417_OEN, NETUP_EN_ALL);
	cx_write(MC417_RWD, NETUP_CTRL_OFF |
				NETUP_ADLO | (0xff & addr));
	cx_clear(MC417_RWD, NETUP_ADLO);
	cx_write(MC417_RWD, NETUP_CTRL_OFF |
				NETUP_ADHI | (0xff & (addr >> 8)));
	cx_clear(MC417_RWD, NETUP_ADHI);

	if (read) { /* data in */
		cx_write(MC417_OEN, NETUP_EN_ALL | NETUP_DATA);
	} else /* data out */
		cx_write(MC417_RWD, NETUP_CTRL_OFF | data);

	/* choose chip */
	cx_clear(MC417_RWD,
			(state->ci_i2c_addr == 0x40) ? NETUP_CS0 : NETUP_CS1);
	/* read/write */
	cx_clear(MC417_RWD, (read) ? NETUP_RD : NETUP_WR);
	mem = netup_ci_get_mem(device);

	//mutex_unlock(&dev->gpio_lock);

	if (!read)
		if (mem < 0)
		{
			KdPrint((LOG_PREFIX "%s: netup_ci_get_mem result < 0", __FUNCTION__));
			return -1;
		}

#if 0
	KdPrint((LOG_PREFIX "%s: %s: chipaddr=[0x%x] addr=[0x%02x], %s=%x\n", __FUNCTION__,
			(read) ? "read" : "write", state->ci_i2c_addr, addr,
			(flag == NETUP_CI_CTL) ? "ctl" : "mem",
			(read) ? mem : data));
#endif

	if (read)
		return mem;

	return 0;
}

BOOLEAN netup_ci_slot_reset(netup_ci_state * ci)
{
	UCHAR buf =  0x80;

	DelayMicroseconds(500);
	if(!netup_write_i2c(ci->device, ci->i2c_adap, ci->ci_i2c_addr,
							0, &buf, 1))
	{
		KdPrint((LOG_PREFIX "%s: i2c write failed", __FUNCTION__));
		return FALSE;
	}

	DelayMicroseconds(500);

	buf = 0x00;
	if(!netup_write_i2c(ci->device, ci->i2c_adap, ci->ci_i2c_addr,
							0, &buf, 1))
	{
		KdPrint((LOG_PREFIX "%s: i2c write failed", __FUNCTION__));
		return FALSE;
	}

	LARGE_INTEGER to;
	to.QuadPart = -10000000;
	KeDelayExecutionThread(KernelMode, FALSE, &to);

	return TRUE;
}

LONG netup_ci_read_attribute_mem(netup_ci_state * state, LONG addr)
{
	return netup_ci_op_cam(state->device, state->port, state, 0, 0, NETUP_CI_RD, addr, 0);
}

LONG netup_ci_write_attribute_mem(netup_ci_state * state, LONG addr, UCHAR data)
{
	return netup_ci_op_cam(state->device, state->port, state, 0, 0, 0, addr, data);
}

LONG netup_ci_read_cam_ctl(netup_ci_state * state, UCHAR addr)
{
	return netup_ci_op_cam(state->device, state->port, state, 0, NETUP_CI_CTL, NETUP_CI_RD, addr, 0);
}

LONG netup_ci_write_cam_ctl(netup_ci_state * state, UCHAR addr, UCHAR data)
{
	return netup_ci_op_cam(state->device, state->port, state, 0, NETUP_CI_CTL, 0, addr, data);
}

LONG netup_ci_slot_ts_ctl(netup_ci_state * state)
{
	UCHAR buf;

	netup_read_i2c(state->device, state->i2c_adap, state->ci_i2c_addr,
			0, &buf, 1);
	buf |= 0x60;

	return netup_write_i2c(state->device, state->i2c_adap, state->ci_i2c_addr,
							0, &buf, 1);
}


static BOOLEAN set_ci_state(netup_ci_state * ci, ULONG state)
{
	const char * st[] = {
		"DVB_CA_SLOTSTATE_NONE",
		"DVB_CA_SLOTSTATE_UNINITIALISED",
		"DVB_CA_SLOTSTATE_RUNNING",
		"DVB_CA_SLOTSTATE_INVALID",
		"DVB_CA_SLOTSTATE_WAITREADY",
		"DVB_CA_SLOTSTATE_VALIDATE",
		"DVB_CA_SLOTSTATE_WAITFR",
		"DVB_CA_SLOTSTATE_LINKINIT"
	};
	LONG old_state = ci->state;
	if(old_state != state)
	{
		ci->state = state;
		KdPrint((LOG_PREFIX "CI #%d slot state changed from %s to %s", ci->port->nr, st[old_state], st[state]));
		return TRUE;
	}
	else
	{
		//KdPrint((LOG_PREFIX "CI #%d slot state %s unchanged", ci->port->nr, st[old_state]));
		return FALSE;
	}
}

/* work handler */
static BOOLEAN netup_read_ci_status(netup_ci_state * ci)
{
	BOOLEAN result = FALSE;

	if(ci->cam_poll_state & 1)
	{
		switch(ci->state)
		{
			case DVB_CA_SLOTSTATE_NONE:
				return set_ci_state(ci, DVB_CA_SLOTSTATE_UNINITIALISED);
			case DVB_CA_SLOTSTATE_WAITREADY:
				return set_ci_state(ci, DVB_CA_SLOTSTATE_VALIDATE);
			default:
				return FALSE;
		}
	}
	else
	{
		return set_ci_state(ci, DVB_CA_SLOTSTATE_NONE);
	}
}

/* CI irq handler */
LONG Netup_CI_Slot_Status(PKSDEVICE device, ULONG pci_status)
{
	struct cx23885_tsport *port = NULL;
	struct netup_ci_state *state = NULL;

	if (0 == (pci_status & (PCI_MSK_GPIO0 | PCI_MSK_GPIO1)))
		return 0;

	if (pci_status & PCI_MSK_GPIO0) {
		port = (cx23885_tsport *)(GETCONTEXT(device)->ts1);
		state = (netup_ci_state *)(GETCONTEXT(device)->cimax2_context[0]);
		KeInsertQueueDpc(&GETCONTEXT(device)->ci_dpc, port, state);
	}

	if (pci_status & PCI_MSK_GPIO1) {
		port = (cx23885_tsport *)(GETCONTEXT(device)->ts2);
		state = (netup_ci_state *)(GETCONTEXT(device)->cimax2_context[1]);
		KeInsertQueueDpc(&GETCONTEXT(device)->ci_dpc, port, state);
	}

	return 1;
}

static VOID set_ci_delay(netup_ci_state * ci)
{
	ci->delay.QuadPart = -CLOCKS_PER_SEC/10;
}

static VOID Netup_CI_Worker_Thread(PVOID StartContext)
{
	netup_ci_state * ci = (netup_ci_state *)StartContext;
	KdPrint((LOG_PREFIX "%s: slot %d thread started", __FUNCTION__, ci->port->nr));
	NTSTATUS ntstat;
	LONG flags;
	LARGE_INTEGER curr_time;
	set_ci_delay(ci);
	for(;;)
	{
		ntstat = KeWaitForSingleObject(&ci->event_kill, Executive, KernelMode, FALSE, &ci->delay);
		if(ntstat == STATUS_SUCCESS)
			break;
		KeQuerySystemTime(&curr_time);
		if(netup_read_ci_status(ci))
		{
			set_ci_delay(ci);
		}
		switch(ci->state) {
			case DVB_CA_SLOTSTATE_NONE:
			case DVB_CA_SLOTSTATE_INVALID:
				break;
			case DVB_CA_SLOTSTATE_UNINITIALISED:
				set_ci_state(ci, DVB_CA_SLOTSTATE_WAITREADY);
				ci->timeout.QuadPart = curr_time.QuadPart + 10000000 * INIT_TIMEOUT_SECS;
				netup_ci_slot_reset(ci);
				break;
			case DVB_CA_SLOTSTATE_WAITREADY:
				if(curr_time.QuadPart >= ci->timeout.QuadPart)
				{
					KdPrint((LOG_PREFIX "CI #%d slot reset timeout", ci->port->nr));
					set_ci_state(ci, DVB_CA_SLOTSTATE_INVALID);
					set_ci_delay(ci);
				}
				break;
			case DVB_CA_SLOTSTATE_VALIDATE:
				if(dvb_ca_en50221_parse_attributes(ci) != 0)
				{
					KdPrint((LOG_PREFIX "CI #%d slot: Invalid PC card inserted", ci->port->nr));
					set_ci_state(ci, DVB_CA_SLOTSTATE_INVALID);
					set_ci_delay(ci);
					break;
				}
				if(dvb_ca_en50221_set_configoption(ci) != 0)
				{
					KdPrint((LOG_PREFIX "CI #%d slot: Unable to initialize CAM", ci->port->nr));
					set_ci_state(ci, DVB_CA_SLOTSTATE_INVALID);
					set_ci_delay(ci);
					break;
				}
				if(netup_ci_write_cam_ctl(ci, CTRLIF_COMMAND, CMDREG_RS) != 0)
				{
					KdPrint((LOG_PREFIX "CI #%d slot: Unable to reset CAM IF", ci->port->nr));
					set_ci_state(ci, DVB_CA_SLOTSTATE_INVALID);
					set_ci_delay(ci);
					break;
				}
				DbgPrint(LOG_PREFIX " CI #%d slot DVB CAM validated", ci->port->nr);
				set_ci_state(ci, DVB_CA_SLOTSTATE_WAITFR);
				set_ci_delay(ci);
				ci->timeout.QuadPart = curr_time.QuadPart + 10000000 * INIT_TIMEOUT_SECS;
				break;
			case DVB_CA_SLOTSTATE_WAITFR:
				if(curr_time.QuadPart >= ci->timeout.QuadPart)
				{
					KdPrint((LOG_PREFIX "CI #%d slot: DVB CAM did not respond", ci->port->nr));
					set_ci_state(ci, DVB_CA_SLOTSTATE_INVALID);
					set_ci_delay(ci);
					break;
				}
				flags = netup_ci_read_cam_ctl(ci, CTRLIF_STATUS);
				if (flags & STATUSREG_FR)
				{
					set_ci_state(ci, DVB_CA_SLOTSTATE_LINKINIT);
					set_ci_delay(ci);
				}
				break;
			case DVB_CA_SLOTSTATE_LINKINIT:
				if (dvb_ca_en50221_link_init(ci) != 0)
				{
					KdPrint((LOG_PREFIX "CI #%d slot initialization failed", ci->port->nr));
					set_ci_state(ci, DVB_CA_SLOTSTATE_INVALID);
					set_ci_delay(ci);
					break;
				}
				netup_ci_slot_ts_ctl(ci);
				set_ci_state(ci, DVB_CA_SLOTSTATE_RUNNING);
				set_ci_delay(ci);
				DbgPrint(LOG_PREFIX "CI %d slot: DVB CAM detected and initialised successfully", ci->port->nr);

				Test_CAM(ci);

				break;
			case DVB_CA_SLOTSTATE_RUNNING:
				break;
		}
	}
	KdPrint((LOG_PREFIX "%s: slot %d thread done", __FUNCTION__, ci->port->nr));
	PsTerminateSystemThread(STATUS_SUCCESS);
}

VOID Netup_CI_Stop(PKSDEVICE device)
{
	netup_ci_state * ci;
	ci = (netup_ci_state *)(GETCONTEXT(device)->cimax2_context[0]);
	if(ci != NULL)
	{
		KeSetEvent(&ci->event_kill, 0, FALSE);
		KeWaitForSingleObject(ci->thread, Executive, KernelMode, FALSE, NULL);
		ObDereferenceObject(ci->thread);
	}
	ci = (netup_ci_state *)(GETCONTEXT(device)->cimax2_context[1]);
	if(ci != NULL)
	{
		ci->state = DVB_CA_SLOTSTATE_SHUTDOWN;
		KeSetEvent(&ci->event_kill, 0, FALSE);
		KeWaitForSingleObject(ci->thread, Executive, KernelMode, FALSE, NULL);
		ObDereferenceObject(ci->thread);
	}
}

BOOLEAN Netup_CI_Init(PKSDEVICE device, struct cx23885_tsport *port)
{
	struct netup_ci_state *state = (netup_ci_state *)ExAllocatePoolWithTag(NonPagedPool, sizeof(*state), '2XMC');
	if(!state)
	{
		KdPrint((LOG_PREFIX "Unable to allocte CIMAX2 context"));
		return FALSE;
	}
	RtlZeroMemory(state, sizeof(*state));
	InitializeListHead(&state->rx_buffer);

	UCHAR cimax_init[34] = {
		0x00, /* module A control*/
		0x00, /* auto select mask high A */
		0x00, /* auto select mask low A */
		0x00, /* auto select pattern high A */
		0x00, /* auto select pattern low A */
		0x44, /* memory access time A */
		0x00, /* invert input A */
		0x00, /* RFU */
		0x00, /* RFU */
		0x00, /* module B control*/
		0x00, /* auto select mask high B */
		0x00, /* auto select mask low B */
		0x00, /* auto select pattern high B */
		0x00, /* auto select pattern low B */
		0x44, /* memory access time B */
		0x00, /* invert input B */
		0x00, /* RFU */
		0x00, /* RFU */
		0x00, /* auto select mask high Ext */
		0x00, /* auto select mask low Ext */
		0x00, /* auto select pattern high Ext */
		0x00, /* auto select pattern low Ext */
		0x00, /* RFU */
		0x02, /* destination - module A */
		0x01, /* power on (use it like store place) */
		0x00, /* RFU */
		0x00, /* int status read only */
		ci_irq_flags() | NETUP_IRQ_DETAM, /* DETAM, IRQAM unmasked */
		0x05, /* EXTINT=active-high, INT=push-pull */
		0x00, /* USCG1 */
		0x04, /* ack active low */
		0x00, /* LOCK = 0 */
		0x33, /* serial mode, rising in, rising out, MSB first*/
		0x31, /* synchronization */
	};

	switch (port->nr) {
	case 1:
		state->ci_i2c_addr = 0x40;
		break;
	case 2:
		state->ci_i2c_addr = 0x41;
		break;
	default:
		KdPrint((LOG_PREFIX "%s: Invalid TSport %d", __FUNCTION__, port->nr));
		ExFreePool(state);
		return FALSE;
	}

	if(GETCONTEXT(device)->cimax2_context[port->nr-1] != NULL)
	{
		KdPrint((LOG_PREFIX "%s: TSport %d already initialized.", __FUNCTION__, port->nr));
		ExFreePool(state);
		return FALSE;
	}

	state->device = device;
	state->port = port;
	state->i2c_adap = 0;

	state->current_irq_mode = ci_irq_flags() | NETUP_IRQ_DETAM;

	BOOLEAN ret;

	ret = netup_write_i2c(state->device, state->i2c_adap, state->ci_i2c_addr,
						0, &cimax_init[0], 34);
	/* lock registers */
	ret &= netup_write_i2c(state->device, state->i2c_adap, state->ci_i2c_addr,
						0x1f, &cimax_init[0x18], 1);
	/* power on slots */
	ret &= netup_write_i2c(state->device, state->i2c_adap, state->ci_i2c_addr,
						0x18, &cimax_init[0x18], 1);

	KeInsertQueueDpc(&GETCONTEXT(device)->ci_dpc, state->port, state);

	if(ret)
	{
		GETCONTEXT(device)->cimax2_context[port->nr-1] = state;
		OBJECT_ATTRIBUTES obj_attrs;
		NTSTATUS ntstat;
		HANDLE handle;
		InitializeObjectAttributes(&obj_attrs, NULL, OBJ_KERNEL_HANDLE, NULL, NULL);
		KeInitializeEvent(&state->event_kill, SynchronizationEvent, FALSE);
		ntstat = PsCreateSystemThread(&handle, THREAD_ALL_ACCESS, &obj_attrs, NULL, NULL, Netup_CI_Worker_Thread, state);
		if(!NT_SUCCESS(ntstat))
		{
			DbgPrint(LOG_PREFIX "unable to create CI worker thread");
			return FALSE;
		}
		ObReferenceObjectByHandle(handle, THREAD_ALL_ACCESS, NULL, KernelMode, (PVOID *)(&state->thread), NULL);
		ZwClose(handle);

		GETCONTEXT(device)->en50221_context[port->nr - 1 ] = EN50221_Init(state);
		if(GETCONTEXT(device)->en50221_context[port->nr - 1 ] == NULL)
		{
			DbgPrint(LOG_PREFIX "EN50221 setup failed for CI #%d", port->nr);
			return FALSE;
		}

		DbgPrint(LOG_PREFIX "Port %d: CI initialized!", port->nr);
		return TRUE;
	}
	else
	{
		DbgPrint(LOG_PREFIX "Port %d: Cannot initialize CI!", port->nr);
		ExFreePool(state);
		return FALSE;
	}
}

void Netup_CI_DPC_Routine(PKDPC Dpc,PVOID DeferredContext,PVOID SystemArgument1,PVOID SystemArgument2)
{

#if 0
	PKSDEVICE device = (PKSDEVICE)DeferredContext;
	cx23885_tsport * port = (cx23885_tsport *)SystemArgument1;
#endif
	netup_ci_state * state = (netup_ci_state *)SystemArgument2;

	UCHAR buf[33];
	LONG ret;

	/* CAM module INSERT/REMOVE processing. slow operation because of i2c
	 * transfers */
	
	if(!netup_read_i2c(state->device, state->i2c_adap, state->ci_i2c_addr,
			0, &buf[0], 33))
	{
		KdPrint((LOG_PREFIX "%s: i2c read error", __FUNCTION__));
		return;
	}

#if 0
	KdPrint((LOG_PREFIX "%s: Slot Status Addr=[0x%04x], "
				"Reg=[0x%02x], data=%02x, "
				"TS config = %02x", __FUNCTION__,
				state->ci_i2c_addr, 0, buf[0],
				buf[0]));
#endif

	state->cam_poll_state = buf[0];
	KdPrint((LOG_PREFIX "%s: polling CI slot #%d result %d", __FUNCTION__, state->port->nr, (ULONG)buf[0]));
}

BOOLEAN Netup_CAM_Running(netup_ci_state * ci)
{
	return (ci->state == DVB_CA_SLOTSTATE_RUNNING);
}

LONG Netup_CAM_Read(netup_ci_state * ci, PUCHAR connection_id, PUCHAR * buffer, PLARGE_INTEGER timeout)
{
	KdPrint((LOG_PREFIX "%s", __FUNCTION__, connection_id));
	if(ci->state != DVB_CA_SLOTSTATE_RUNNING)
	{
		KdPrint((LOG_PREFIX "%s: state != RUNNING", __FUNCTION__));
		return -1;
	}
	LARGE_INTEGER default_timeout;
	default_timeout.QuadPart = -1 * READ_TIMEOUT_SECS * CLOCKS_PER_SEC;
	if(timeout == NULL)
	{
		KdPrint((LOG_PREFIX "%s: default timeout %d sec", READ_TIMEOUT_SECS));
		timeout = &default_timeout;
	}
	UCHAR buf[HOST_LINK_BUF_SIZE];
	UCHAR conn_id = 0;
	PUCHAR curr_buffer, next_buffer;
	ULONG curr_buffer_size = ci->link_buf_size * 4, count = 0;
	curr_buffer = (PUCHAR)ExAllocatePoolWithTag(NonPagedPool, curr_buffer_size, 'ZSBC');
	if(!curr_buffer)
	{
		KdPrint((LOG_PREFIX "%s: unable to allocate buffer", __FUNCTION__));
		return CAM_IO_ERROR;
	}
	LARGE_INTEGER curr_time, to, delay;
	KeQuerySystemTime(&curr_time);
	if(timeout->QuadPart > 0)
	{
		to.QuadPart = timeout->QuadPart;
	}
	else if(timeout->QuadPart < 0)
	{
		to.QuadPart = curr_time.QuadPart - 1*timeout->QuadPart;
	}
	delay.QuadPart = CLOCKS_PER_SEC / 1000;
	LONG status;
	for(;;)
	{
		if(ci->state != DVB_CA_SLOTSTATE_RUNNING)
		{
			KdPrint((LOG_PREFIX "%s: RUNNING state lost", __FUNCTION__));
			return CAM_INVALID_STATE;
		}
		if(timeout->QuadPart != 0 && curr_time.QuadPart >= to.QuadPart)
		{
			KdPrint((LOG_PREFIX "CAM read timeout"));
			return CAM_IO_ERROR;
		}
		if(curr_buffer_size - count < ci->link_buf_size)
		{
			next_buffer = (PUCHAR)ExAllocatePoolWithTag(NonPagedPool, curr_buffer_size * 2, 'ZSBC');
			if(!next_buffer)
			{
				KdPrint((LOG_PREFIX "%s: unable to realloc buffer, size=%d", __FUNCTION__, curr_buffer_size * 2));
				return CAM_IO_ERROR;
			}
			RtlCopyMemory(next_buffer, curr_buffer, curr_buffer_size);
			curr_buffer_size *= 2;
			ExFreePool(curr_buffer);
			curr_buffer = next_buffer;
		}
		status = dvb_ca_en50221_read_data(ci, buf, HOST_LINK_BUF_SIZE);
		if(status > 0)
		{
			if(status < 2)
			{
				KdPrint((LOG_PREFIX "CAM reply too short"));
				return CAM_IO_ERROR;
			}
			RtlCopyMemory(curr_buffer + count, buf + 2, status - 2);
			count += (status - 2);
			if(conn_id == 0)
			{
				KdPrint((LOG_PREFIX "first frag conn_id 0x%x", (ULONG)buf[0]));
				conn_id = buf[0];
			}
			else
			{
				if(conn_id != buf[0])
				{
					DbgPrint(LOG_PREFIX "new frag conn_id 0x%x != current conn_id 0x%x", (ULONG)buf[0], (ULONG)conn_id);
					return CAM_IO_ERROR;
				}
			}
			if((buf[1] & 0x80) == 0x00)
			{
				KdPrint((LOG_PREFIX "%s: got last frag, conn_id 0x%x, total length %d", (ULONG)count));
				*buffer = curr_buffer;
				*connection_id = conn_id;
				return count;
			}
		}
		else if(status == 0)
		{
			//KeDelayExecutionThread(KernelMode, FALSE, &delay);
			DelayMilliseconds(1);
		}
		else
		{
			KdPrint((LOG_PREFIX "CAM read error: %d", status));
			return status;
		}
	}
}

LONG Netup_CAM_Write(netup_ci_state * ci, UCHAR connection_id, PUCHAR buffer, ULONG size)
{
	KdPrint((LOG_PREFIX "%s(conn_id=%d,size=%d)", __FUNCTION__, connection_id, size));
	if(ci->state != DVB_CA_SLOTSTATE_RUNNING)
	{
		KdPrint((LOG_PREFIX "%s: state != RUNNING", __FUNCTION__));
		return CAM_INVALID_STATE;
	}
	UCHAR buf[HOST_LINK_BUF_SIZE];
	ULONG count = 0, part;
	LARGE_INTEGER timeout, curr_time, delay;
	LONG status;
	delay.QuadPart = CLOCKS_PER_SEC / 1000;
	while(size - count > 0)
	{
		part = ((size - count) > (ci->link_buf_size - 2)) ? ci->link_buf_size - 2 : (size - count);
		buf[0] = connection_id;
		buf[1] = (size - (count + part) > 0) ? 0x80 : 0x00;
		RtlCopyMemory(buf + 2, buffer + count, part);
		KeQuerySystemTime(&curr_time);
		timeout.QuadPart = curr_time.QuadPart + CLOCKS_PER_SEC / 2;
		while(curr_time.QuadPart < timeout.QuadPart)
		{
			if(ci->state != DVB_CA_SLOTSTATE_RUNNING)
			{
				KdPrint((LOG_PREFIX "%s: RUNNING state lost", __FUNCTION__));
				return CAM_INVALID_STATE;
			}
			status = dvb_ca_en50221_write_data(ci, buf, part + 2);
			if(status == (part + 2))
			{
				break;
			}
			else if(status == CAM_RETRY)
			{
				KeDelayExecutionThread(KernelMode, FALSE, &delay);
			}
			else
			{
				KdPrint((LOG_PREFIX "CAM write error: %d", status));
				return status;
			}
			KeQuerySystemTime(&curr_time);
		}
		if(curr_time.QuadPart >= timeout.QuadPart)
		{
			KdPrint((LOG_PREFIX "CAM write timeout"));
			return CAM_IO_ERROR;
		}
		count += part;
	}
	return count;
}

#define ST_OPEN_SESSION_REQ     0x91    // h<--m
#define ST_OPEN_SESSION_RES     0x92    // h-->m
#define ST_CREATE_SESSION       0x93    // h-->m
#define ST_CREATE_SESSION_RES   0x94    // h<--m
#define ST_CLOSE_SESSION_REQ    0x95    // h<->m
#define ST_CLOSE_SESSION_RES    0x96    // h<->m
#define ST_SESSION_NUMBER       0x90    // h<->m

/**
 * Make a host-endian uint32_t formatted resource id.
 *
 * @param CLASS Class of resource.
 * @param TYPE Type of resource.
 * @param VERSION Version of resource.
 * @return Formatted resource id.
 */
#define MKRID(CLASS, TYPE, VERSION) ((((CLASS)&0xffff)<<16) | (((TYPE)&0x3ff)<<6) | ((VERSION)&0x3f))
#define EN50221_APP_CA_RESOURCEID MKRID(3,1,1)

VOID Test_CAM(netup_ci_state * ci)
{
	PVOID ctx = GETCONTEXT(ci->device)->en50221_context[ci->port->nr - 1];
	EN50221_Reset(ctx);
	EN50221_Main(ctx);

#if 0
	ULONG conn_id = EN50221_CreateTransportConnection(ctx);
	if(conn_id <= 0)
	{
		KdPrint((LOG_PREFIX "failed to create connection"));
		return;
	}
	ULONG resource_id = EN50221_APP_CA_RESOURCEID;
	USHORT session_id = 1;
	UCHAR hdr[8];
	hdr[0] = ST_CREATE_SESSION;
	hdr[1] = 6;
	hdr[2] = (resource_id >> 24) & 0xff;
	hdr[3] = (resource_id >> 16) & 0xff;
	hdr[4] = (resource_id >> 8) & 0xff;
	hdr[5] = resource_id & 0xff;
	hdr[6] = (session_id >> 8) & 0xff;
	hdr[7] = session_id & 0xff;
	EN50221_TransportConnectionWrite(ctx, conn_id & 0xff, hdr, sizeof(hdr));
	EN50221_DeleteTransportConnection(ctx, conn_id & 0xff);
#endif
}

/* DVB CA support */

/**
 * Read a tuple from attribute memory.
 *
 * @param ca CA instance.
 * @param slot Slot id.
 * @param address Address to read from. Updated.
 * @param tupleType Tuple id byte. Updated.
 * @param tupleLength Tuple length. Updated.
 * @param tuple Dest buffer for tuple (must be 256 bytes). Updated.
 *
 * @return 0 on success, nonzero on error.
 */
static LONG dvb_ca_en50221_read_tuple(netup_ci_state * status, PLONG address, PLONG tupleType, PLONG tupleLength, PUCHAR tuple)
{
	LONG i;
	LONG _tupleType;
	LONG _tupleLength;
	LONG _address = *address;

	/* grab the next tuple length and type */
	if ((_tupleType = netup_ci_read_attribute_mem(status, _address)) < 0)
		return _tupleType;
	if (_tupleType == 0xff) {
		KdPrint((LOG_PREFIX "END OF CHAIN TUPLE type:0x%x", _tupleType));
		*address += 2;
		*tupleType = _tupleType;
		*tupleLength = 0;
		return 0;
	}
	if ((_tupleLength = netup_ci_read_attribute_mem(status, _address + 2)) < 0)
		return _tupleLength;
	_address += 4;

	KdPrint((LOG_PREFIX "TUPLE type:0x%x length:%i", _tupleType, _tupleLength));

	/* read in the whole tuple */
	for (i = 0; i < _tupleLength; i++) {
		tuple[i] = netup_ci_read_attribute_mem(status, _address + (i * 2)) & 0xff;
		KdPrint((LOG_PREFIX "  0x%02x: 0x%02x %c",
			i, tuple[i] & 0xff,
			((tuple[i] > 31) && (tuple[i] < 127)) ? tuple[i] : '.'));
	}
	_address += (_tupleLength * 2);

	// success
	*tupleType = _tupleType;
	*tupleLength = _tupleLength;
	*address = _address;
	return 0;
}

/**
 * Safely find needle in haystack.
 *
 * @param haystack Buffer to look in.
 * @param hlen Number of bytes in haystack.
 * @param needle Buffer to find.
 * @param nlen Number of bytes in needle.
 * @return Pointer into haystack needle was found at, or NULL if not found.
 */
static PCHAR findstr(PCHAR haystack, ULONG hlen, PCHAR needle, ULONG nlen)
{
	ULONG i;

	if (hlen < nlen)
		return NULL;

	for (i = 0; i <= hlen - nlen; i++) {
		if (!strncmp(haystack + i, needle, nlen))
			return haystack + i;
	}

	return NULL;
}

/**
 * Parse attribute memory of a CAM module, extracting Config register, and checking
 * it is a DVB CAM module.
 *
 * @return 0 on success, <0 on failure.
 */
static LONG dvb_ca_en50221_parse_attributes(netup_ci_state * ci)
{
	LONG address = 0;
	LONG tupleLength;
	LONG tupleType;
	UCHAR tuple[257];
	PCHAR dvb_str;
	LONG rasz;
	LONG status;
	LONG got_cftableentry = 0;
	LONG end_chain = 0;
	LONG i;
	USHORT manfid = 0;
	USHORT devid = 0;


	// CISTPL_DEVICE_0A
	if ((status =
	     dvb_ca_en50221_read_tuple(ci, &address, &tupleType, &tupleLength, tuple)) < 0)
		return status;
	if (tupleType != 0x1D)
		return -1;



	// CISTPL_DEVICE_0C
	if ((status =
	     dvb_ca_en50221_read_tuple(ci, &address, &tupleType, &tupleLength, tuple)) < 0)
		return status;
	if (tupleType != 0x1C)
		return -1;



	// CISTPL_VERS_1
	if ((status =
	     dvb_ca_en50221_read_tuple(ci, &address, &tupleType, &tupleLength, tuple)) < 0)
		return status;
	if (tupleType != 0x15)
		return -1;



	// CISTPL_MANFID
	if ((status = dvb_ca_en50221_read_tuple(ci, &address, &tupleType,
						&tupleLength, tuple)) < 0)
		return status;
	if (tupleType != 0x20)
		return -1;
	if (tupleLength != 4)
		return -1;
	manfid = (tuple[1] << 8) | tuple[0];
	devid = (tuple[3] << 8) | tuple[2];



	// CISTPL_CONFIG
	if ((status = dvb_ca_en50221_read_tuple(ci, &address, &tupleType,
						&tupleLength, tuple)) < 0)
		return status;
	if (tupleType != 0x1A)
		return -1;
	if (tupleLength < 3)
		return -1;

	/* extract the configbase */
	rasz = tuple[0] & 3;
	if (tupleLength < (3 + rasz + 14))
		return -1;
	ci->config_base = 0;
	for (i = 0; i < rasz + 1; i++) {
		ci->config_base |= (tuple[2 + i] << (8 * i));
	}

	/* check it contains the correct DVB string */
	dvb_str = findstr((char *)tuple, tupleLength, "DVB_CI_V", 8);
	if (dvb_str == NULL)
		return -1;
	if (tupleLength < ((dvb_str - (char *) tuple) + 12))
		return -1;

	/* is it a version we support? */
	if (strncmp(dvb_str + 8, "1.00", 4)) {
		KdPrint((LOG_PREFIX "dvb_ca adapter %d: Unsupported DVB CAM module version %c%c%c%c\n",
		       ci->port->nr, dvb_str[8], dvb_str[9], dvb_str[10], dvb_str[11]));
		return -1;
	}

	/* process the CFTABLE_ENTRY tuples, and any after those */
	while ((!end_chain) && (address < 0x1000)) {
		if ((status = dvb_ca_en50221_read_tuple(ci, &address, &tupleType,
							&tupleLength, tuple)) < 0)
			return status;
		switch (tupleType) {
		case 0x1B:	// CISTPL_CFTABLE_ENTRY
			if (tupleLength < (2 + 11 + 17))
				break;

			/* if we've already parsed one, just use it */
			if (got_cftableentry)
				break;

			/* get the config option */
			ci->config_option = tuple[0] & 0x3f;

			/* OK, check it contains the correct strings */
			if ((findstr((char *)tuple, tupleLength, "DVB_HOST", 8) == NULL) ||
			    (findstr((char *)tuple, tupleLength, "DVB_CI_MODULE", 13) == NULL))
				break;

			got_cftableentry = 1;
			break;

		case 0x14:	// CISTPL_NO_LINK
			break;

		case 0xFF:	// CISTPL_END
			end_chain = 1;
			break;

		default:	/* Unknown tuple type - just skip this tuple and move to the next one */
			KdPrint((LOG_PREFIX "Skipping unknown tuple type:0x%x length:0x%x", tupleType,
				tupleLength));
			break;
		}
	}

	if ((address > 0x1000) || (!got_cftableentry))
		return -1;

	KdPrint((LOG_PREFIX "Valid DVB CAM detected MANID:%x DEVID:%x CONFIGBASE:0x%x CONFIGOPTION:0x%x\n",
		manfid, devid, ci->config_base, ci->config_option));

	// success!
	return 0;
}

/**
 * Set CAM's configoption correctly.
 *
 * @param ci CA instance.
 */
static LONG dvb_ca_en50221_set_configoption(netup_ci_state * ci)
{
	LONG configoption;

	/* set the config option */
	netup_ci_write_attribute_mem(ci,
				     ci->config_base,
				     ci->config_option & 0xff);

	/* check it */
	configoption = netup_ci_read_attribute_mem(ci, ci->config_base);
	KdPrint((LOG_PREFIX "Set configoption 0x%x, read configoption 0x%x",
		ci->config_option, configoption & 0x3f));

	/* fine! */
	return 0;
}


/**
 * This function talks to an EN50221 CAM control interface. It reads a buffer of
 * data from the CAM. The data can either be stored in a supplied buffer, or
 * automatically be added to the slot's rx_buffer.
 *
 * @param ca CA instance.
 * @param slot Slot to read from.
 * @param ebuf If non-NULL, the data will be written to this buffer. If NULL,
 * the data will be added into the buffering system as a normal fragment.
 * @param ecount Size of ebuf. Ignored if ebuf is NULL.
 *
 * @return Number of bytes read, or < 0 on error
 */
static LONG dvb_ca_en50221_read_data(netup_ci_state *ci, PUCHAR ebuf, ULONG ecount)
{
	ULONG bytes_read;
	LONG status;
	UCHAR buf[HOST_LINK_BUF_SIZE];
	ULONG i;

	/* check if there is data available */
	if ((status = netup_ci_read_cam_ctl(ci, CTRLIF_STATUS)) < 0)
		goto exit;
	if (!(status & STATUSREG_DA)) {
		/* no data */
		status = 0;
		goto exit;
	}

	/* read the amount of data */
	if ((status = netup_ci_read_cam_ctl(ci, CTRLIF_SIZE_HIGH)) < 0)
		goto exit;
	bytes_read = (status  & 0xff) << 8;
	if ((status = netup_ci_read_cam_ctl(ci, CTRLIF_SIZE_LOW)) < 0)
		goto exit;
	bytes_read |= (status & 0xff);

	/* check it will fit */
	if (ebuf == NULL) {
		if (bytes_read > ci->link_buf_size) {
			DbgPrint("dvb_ca adapter CI #%d: CAM tried to send a buffer larger than the link buffer size (%i > %i)!",
			       ci->port->nr, bytes_read, ci->link_buf_size);
			set_ci_state(ci, DVB_CA_SLOTSTATE_LINKINIT);
			status = CAM_IO_ERROR;
			goto exit;
		}
		if (bytes_read < 2) {
			DbgPrint(LOG_PREFIX "dvb_ca adapter %d: CAM sent a buffer that was less than 2 bytes!",
			       ci->port->nr);
			set_ci_state(ci, DVB_CA_SLOTSTATE_LINKINIT);
			status = CAM_IO_ERROR;
			goto exit;
		}
	} else {
		if (bytes_read > ecount) {
			DbgPrint(LOG_PREFIX "dvb_ca adapter %d: CAM tried to send a buffer larger than the ecount size (%d > %d)!",
			       ci->port->nr, ecount, bytes_read);
			status = CAM_IO_ERROR;
			goto exit;
		}
	}

	/* fill the buffer */
	for (i = 0; i < bytes_read; i++) {
		/* read byte and check */
		if ((status = netup_ci_read_cam_ctl(ci, CTRLIF_DATA)) < 0)
			goto exit;

		/* OK, store it in the buffer */
		buf[i] = status & 0xff;
	}

	/* check for read error (RE should now be 0) */
	if ((status = netup_ci_read_cam_ctl(ci, CTRLIF_STATUS)) < 0)
		goto exit;
	if (status & STATUSREG_RE) {
		set_ci_state(ci, DVB_CA_SLOTSTATE_LINKINIT);
		status = -1;
		goto exit;
	}

	/* OK, add it to the receive buffer, or copy into external buffer if supplied */
	if (ebuf == NULL) {
		CAM_Packet * pkt = (CAM_Packet *)ExAllocatePoolWithTag(NonPagedPool, sizeof(*pkt), 'PMAC');
		if(!pkt)
		{
			status = CAM_IO_ERROR;
			goto exit;
		}
		pkt->data = (PUCHAR)ExAllocatePoolWithTag(NonPagedPool, bytes_read, 'BMAC');
		if(!pkt->data)
		{
			ExFreePool(pkt);
			status = CAM_IO_ERROR;
			goto exit;
		}
		pkt->size = bytes_read;
		RtlCopyMemory(pkt->data, buf, bytes_read);
		InsertTailList(&ci->rx_buffer, &pkt->link);
	} else {
		RtlCopyMemory(ebuf, buf, bytes_read);
	}

	KdPrint((LOG_PREFIX "Received CA packet for CI #%d connection id 0x%x last_frag:%i size:0x%x", ci->port->nr,
		buf[0], (buf[1] & 0x80) == 0, bytes_read));

	/* wake up readers when a last_fragment is received */
	if ((buf[1] & 0x80) == 0x00) {
		//wake_up_interruptible(&ca->wait_queue);
	}
	status = bytes_read;

exit:
	return status;
}


/**
 * This function talks to an EN50221 CAM control interface. It writes a buffer of data
 * to a CAM.
 *
 * @param ca CA instance.
 * @param slot Slot to write to.
 * @param ebuf The data in this buffer is treated as a complete link-level packet to
 * be written.
 * @param count Size of ebuf.
 *
 * @return Number of bytes written, or < 0 on error.
 */
static LONG dvb_ca_en50221_write_data(netup_ci_state * ci, PUCHAR buf, ULONG bytes_write)
{
	LONG status;
	ULONG i;

	KdPrint((LOG_PREFIX, "%s", __FUNCTION__));

	/* sanity check */
	if (bytes_write > ci->link_buf_size)
		return CAM_IO_ERROR;

	/* it is possible we are dealing with a single buffer implementation,
	   thus if there is data available for read or if there is even a read
	   already in progress, we do nothing but awake the kernel thread to
	   process the data if necessary. */
	if ((status = netup_ci_read_cam_ctl(ci, CTRLIF_STATUS)) < 0)
		goto exitnowrite;
	if (status & (STATUSREG_DA | STATUSREG_RE)) {
		if (status & STATUSREG_DA)
		{
			KdPrint((LOG_PREFIX "%s: STATUSREG_DA set, status=EAGAIN", __FUNCTION__));
			//dvb_ca_en50221_thread_wakeup(ca);
		}

		status = CAM_RETRY;
		goto exitnowrite;
	}

	/* OK, set HC bit */
	if ((status = netup_ci_write_cam_ctl(ci, CTRLIF_COMMAND,
						 IRQEN | CMDREG_HC)) != 0)
		goto exit;

	/* check if interface is still free */
	if ((status = netup_ci_read_cam_ctl(ci, CTRLIF_STATUS)) < 0)
		goto exit;
	if (!(status & STATUSREG_FR)) {
		/* it wasn't free => try again later */
		KdPrint((LOG_PREFIX "%s: STATUSREG_FR not set, status=EAGAIN", __FUNCTION__));
		status = CAM_RETRY;
		goto exit;
	}

	/* send the amount of data */
	if ((status = netup_ci_write_cam_ctl(ci, CTRLIF_SIZE_HIGH, (bytes_write >> 8) & 0xff)) != 0)
		goto exit;
	if ((status = netup_ci_write_cam_ctl(ci, CTRLIF_SIZE_LOW,
						 bytes_write & 0xff)) != 0)
		goto exit;

	/* send the buffer */
	for (i = 0; i < bytes_write; i++) {
		if ((status = netup_ci_write_cam_ctl(ci, CTRLIF_DATA, buf[i])) != 0)
			goto exit;
	}

	/* check for write error (WE should now be 0) */
	if ((status = netup_ci_read_cam_ctl(ci, CTRLIF_STATUS)) < 0)
		goto exit;
	if (status & STATUSREG_WE) {
		set_ci_state(ci, DVB_CA_SLOTSTATE_LINKINIT);
		status = CAM_IO_ERROR;
		goto exit;
	}
	status = bytes_write;

	KdPrint((LOG_PREFIX "Wrote CA packet for slot %i, connection id 0x%x last_frag:%i size:0x%x", ci->port->nr,
		buf[0], (buf[1] & 0x80) == 0, bytes_write));

exit:
	netup_ci_write_cam_ctl(ci, CTRLIF_COMMAND, IRQEN);

exitnowrite:
	return status;
}

/**
 * Wait for flags to become set on the STATUS register on a CAM interface,
 * checking for errors and timeout.
 *
 * @param ca CA instance.
 * @param slot Slot on interface.
 * @param waitfor Flags to wait for.
 * @param timeout_ms Timeout in milliseconds.
 *
 * @return 0 on success, nonzero on error.
 */
static LONG dvb_ca_en50221_wait_if_status(netup_ci_state * ci, UCHAR waitfor, ULONG timeout_msec)
{
	ULONG i;

	/* loop until timeout elapsed */
	for(i = 0; i < timeout_msec; i++) {
		if(ci->state == DVB_CA_SLOTSTATE_SHUTDOWN)
		{
			KdPrint((LOG_PREFIX "CI #%d slot shutting down", ci->port->nr));
			return -3;
		}
		/* read the status and check for error */
		LONG res = netup_ci_read_cam_ctl(ci, CTRLIF_STATUS);
		if (res < 0)
			return -1;

		/* if we got the flags, it was successful! */
		if ((res & 0xff) & waitfor) {
			KdPrint((LOG_PREFIX "%s succeeded timeout", __FUNCTION__));
			return 0;
		}

		/* wait for a bit */
		DelayMilliseconds(1);
	}

	KdPrint((LOG_PREFIX "%s failed timeout", __FUNCTION__));

	/* if we get here, we've timed out */
	return -2;
}


/**
 * Initialise the link layer connection to a CAM.
 *
 * @param ca CA instance.
 * @param slot Slot id.
 *
 * @return 0 on success, nonzero on failure.
 */
static LONG dvb_ca_en50221_link_init(netup_ci_state * ci)
{
	LONG ret;
	ULONG buf_size;
	UCHAR buf[2];

	/* we'll be determining these during this function */
	//ca->slot_info[slot].da_irq_supported = 0;

	/* set the host link buffer size temporarily. it will be overwritten with the
	 * real negotiated size later. */
	ci->link_buf_size = 2;

	/* read the buffer size from the CAM */
	if ((ret = netup_ci_write_cam_ctl(ci, CTRLIF_COMMAND, IRQEN | CMDREG_SR)) != 0)
		return ret;
	if ((ret = dvb_ca_en50221_wait_if_status(ci, STATUSREG_DA, 100)) != 0)
		return ret;
	if ((ret = dvb_ca_en50221_read_data(ci, buf, 2)) != 2)
	{
		KdPrint((LOG_PREFIX "dvb_ca_en50221_read_data(ci, buf, 2) failed"));
		return -1;
	}
	if ((ret = netup_ci_write_cam_ctl(ci, CTRLIF_COMMAND, IRQEN)) != 0)
		return ret;

	/* store it, and choose the minimum of our buffer and the CAM's buffer size */
	buf_size = (buf[0] << 8) | buf[1];
	if (buf_size > HOST_LINK_BUF_SIZE)
		buf_size = HOST_LINK_BUF_SIZE;
	ci->link_buf_size = buf_size;
	buf[0] = (buf_size >> 8) & 0xff;
	buf[1] = buf_size & 0xff;
	KdPrint((LOG_PREFIX "Chosen link buffer size of %d", buf_size));

	/* write the buffer size to the CAM */
	if ((ret = netup_ci_write_cam_ctl(ci, CTRLIF_COMMAND, IRQEN | CMDREG_SW)) != 0)
		return ret;
	if ((ret = dvb_ca_en50221_wait_if_status(ci, STATUSREG_FR, 1000)) != 0)
		return ret;
	if ((ret = dvb_ca_en50221_write_data(ci, buf, 2)) != 2)
		return -1;
	if ((ret = netup_ci_write_cam_ctl(ci, CTRLIF_COMMAND, IRQEN)) != 0)
		return ret;

	/* success */
	return 0;
}