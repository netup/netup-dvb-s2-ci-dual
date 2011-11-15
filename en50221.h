/*
 * en50221.h - EN50221 protocol implementation
 *
 *  WDM driver for NetUP Dual DVB-S2 CI card
 *
 *  Copyright (C) 2011 NetUP Inc.
 *  Copyright (C) 2011 Sergey Kozlov <serjk@netup.ru>
 *
 *  ASN.1 routines, implementation for libdvben50221
 *   an implementation for the High Level Common Interface
 *
 *  Copyright (C) 2004, 2005 Manu Abraham <abraham.manu@gmail.com>
 *  Copyright (C) 2006 Andrew de Quincey (adq_dvb@lidskialf.net)
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

#ifndef __NETUP_EN50221_H__
#define __NETUP_EN50221_H__

#include "driver.h"

/* EN50221 support routines error code */

#define EN50221_IO_ERROR			-1
#define EN50221_TIMEOUT				-2
#define EN50221_INVALID_REPLY		-3
#define EN50221_RESOURCE_BUSY		-4
#define EN50221_NOMEM				-5
#define EN50221_INVALID_ARGUMENT	-6
#define EN50221_GENERAL_ERROR		-7

struct netup_ci_state;

LONG Netup_CAM_Read(netup_ci_state * ci, PUCHAR connection_id, PUCHAR * buffer, PLARGE_INTEGER timeout);
LONG Netup_CAM_Write(netup_ci_state * ci, UCHAR connection_id, PUCHAR buffer, ULONG size);
BOOLEAN Netup_CAM_Running(netup_ci_state * ci);

PVOID EN50221_Init(netup_ci_state * ci);
VOID EN50221_Reset(PVOID ctx);
VOID EN50221_Main(PVOID ctx);
LONG EN50221_APP_CA_PMT_ListChange(PVOID ctx, UCHAR type, UCHAR cmd, PVOID inBuffer, ULONG inBufferSize, PVOID outBuffer, ULONG outBufferSize);
LONG EN50221_APP_CAM_EnterMenu(PVOID ctx);
LONG EN50221_APP_CAM_GetMenu(PVOID ctx, PVOID outBuffer, ULONG outBufferSize);
LONG EN50221_APP_CAM_Status(PVOID ctx, PVOID outBuffer, ULONG outBufferSize);
LONG EN50221_APP_CAM_AnswerMenu(PVOID ctx, UCHAR answ);
LONG EN50221_APP_CAM_CloseMenu(PVOID ctx);


#endif // __NETUP_EN50221_H__