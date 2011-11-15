/*
 * dispatch.h
 *
 *  WDM driver for NetUP Dual DVB-S2 CI card
 *
 * Copyright (C) 2011 NetUP Inc.
 * Copyright (C) 2011 Sergey Kozlov <serjk@netup.ru>
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

#ifndef __NETUP_DISPATCH_H__
#define __NETUP_DISPATCH_H__

#include "driver.h"

NTSTATUS NetupAntennaCreate(IN PKSPIN Pin,IN PIRP Irp);
NTSTATUS NetupAntennaClose(IN PKSPIN Pin,IN PIRP Irp);
NTSTATUS NetupAntennaSetDeviceState(IN PKSPIN Pin,IN KSSTATE ToState,IN KSSTATE FromState);
NTSTATUS NetupOutputCreate(IN PKSPIN Pin,IN PIRP Irp);
NTSTATUS NetupOutputClose(IN PKSPIN Pin,IN PIRP Irp);
NTSTATUS NetupOutputSetDeviceState(IN PKSPIN Pin,IN KSSTATE ToState,IN KSSTATE FromState);
NTSTATUS NetupOutputProcess(IN PKSPIN Pin);

#endif
