/*
 * filter.h - BDA filter code
 *
 *  WDM driver for NetUP Dual DVB-S2 CI card
 *
 * Copyright (C) 2011 NetUP Inc.
 * Copyright (C) 2011 Sergey Kozlov <serjk@netup.ru>
 *
 * Hauppauge Nova-T BDA driver
 *
 * Copyright (C) 2003 Colin Munro (colin@mice-software.com)
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

#ifndef __NETUP_FILTER_H__
#define __NETUP_FILTER_H__

#include "driver.h"

#define TRANSPORT_PACKET_SIZE	188
#define TRANSPORT_PACKET_COUNT	2048

extern const KSFILTER_DESCRIPTOR NetupFilter0Descriptor;
extern const KSFILTER_DESCRIPTOR NetupFilter1Descriptor;
extern const BDA_FILTER_TEMPLATE NetupFilter0Template;
extern const BDA_FILTER_TEMPLATE NetupFilter1Template;

struct FilterContext
{
	ULONG nr;
	PKSDEVICE device;
};

#endif
