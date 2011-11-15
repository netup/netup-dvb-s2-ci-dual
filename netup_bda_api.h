#ifndef __NETUP_BDA_API_H__
#define __NETUP_BDA_API_H__

/*
 * netup_bda_api.h
 *
 * NetUp Dual DVB-S2 CI card BDA interface extensions
 *
 * Copyright (C) 2011 NetUP Inc.
 * Copyright (C) 2011 Sergey Kozlov <serjk@netup.ru>
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

// {5AA642F2-BF94-4199-A98C-C2222091E3C3}
#define STATIC_KSPROPSETID_NetupExtProperties \
	0x5aa642f2, 0xbf94, 0x4199, 0xa9, 0x8c, 0xc2, 0x22, 0x20, 0x91, 0xe3, 0xc3

DEFINE_GUIDSTRUCT("5AA642F2-BF94-4199-A98C-C2222091E3C3", KSPROPSETID_NetupExtProperties);
#define KSPROPSETID_NetupExtProperties DEFINE_GUIDNAMED(KSPROPSETID_NetupExtProperties)

typedef enum {
    KSPROPERTY_BDA_NETUP_IOCTL = 0,
} KSPROPERTY_BDA_NETUP_EXTENSION;

struct NETUP_BDA_EXT_CMD
{
	DWORD dwCmd;
	LPVOID lpInputBuffer;
	DWORD dwInputBufferLength;
	LPVOID lpOutputBuffer;
	DWORD dwOutputBufferLength;
	LPDWORD lpOutputLength;
};

// specify the sizeof the actual property to retrieve here 
#define DEFINE_KSPROPERTY_ITEM_BDA_NETUP_IOCTL(SetHandler)\
    DEFINE_KSPROPERTY_ITEM(\
        KSPROPERTY_BDA_NETUP_IOCTL,\
        NULL,\
        sizeof(KSPROPERTY),\
        sizeof(NETUP_BDA_EXT_CMD),\
        (SetHandler),\
        NULL, 0, NULL, NULL, 0)

#define NETUP_IOCTL_CMD(cmd, arg1, arg2)	(((cmd & 0xff)) << 16 | ((arg1 & 0xff) << 8) | (arg2 & 0xff))
#define NETUP_IOCTL_MAX_BUFFER_SIZE			65536

/* Supported IOCTLs */
typedef enum
{
	NETUP_IOCTL_DISEQC_WRITE = 0x10,
	NETUP_IOCTL_CI_STATUS = 0x20,
	NETUP_IOCTL_CI_MMI_ENTER_MENU = 0x30,
	NETUP_IOCTL_CI_MMI_GET_MENU = 0x31,
	NETUP_IOCTL_CI_MMI_ANSWER_MENU = 0x32,
	NETUP_IOCTL_CI_MMI_CLOSE = 0x33,
	NETUP_IOCTL_CI_PMT_LIST_CHANGE = 0x40,
};

/* PMT_LIST_CHANGE first argument */
typedef enum
{
	PMT_LIST_MORE = 0,
	PMT_LIST_FIRST = 1,
	PMT_LIST_LAST = 2,
	PMT_LIST_ONLY = 3
};

/* PMT_LIST_CHANGE second argument */
typedef enum
{
	PMT_OK_DESCRAMBLING = 1,
	PMT_OK_MMI = 2,
	PMT_QUERY = 3
};

/* NETUP_CAM_STATUS::dwCamStatus bits */
typedef enum
{
	NETUP_CAM_PRESENT = 1,
	NETUP_CAM_MMI_DATA_READY = 2,
};

/* NETUP_IOCTL_CI_STATUS reply format */
struct NETUP_CAM_STATUS
{
	DWORD dwCamStatus;
	WORD wCamVendor;
	WORD wCamDevice;
	CHAR cCamString[256];
};

/* MMI menu limitations */
enum
{
	MENU_MAX_ITEM_LENGTH = 256,
	MENU_MAX_ITEM_COUNT = 64
};

/* NETUP_IOCTL_CI_MMI_GET_MENU reply format */
struct NETUP_CAM_MENU
{
	BOOL bIsMenu;
	CHAR cTitle[MENU_MAX_ITEM_LENGTH];
	CHAR cSubTitle[MENU_MAX_ITEM_LENGTH];
	CHAR cBottomText[MENU_MAX_ITEM_LENGTH];
	CHAR cItems[MENU_MAX_ITEM_COUNT][MENU_MAX_ITEM_LENGTH];
	DWORD dwItemCount;
};

#endif

