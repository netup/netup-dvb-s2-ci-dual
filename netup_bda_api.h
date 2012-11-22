#ifndef __NETUP_BDA_API_H__
#define __NETUP_BDA_API_H__

/*
 * netup_bda_api.h
 *
 * NetUp Dual DVB-S2 CI card BDA interface extensions
 *
 * Copyright (C) 2011,2012 NetUP Inc.
 * Copyright (C) 2011,2012 Sergey Kozlov <serjk@netup.ru>
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

#pragma pack(push, 1)
struct NETUP_BDA_EXT_CMD
{
	DWORD64 dwCmd;
	LPVOID lpInputBuffer;
#ifndef _WIN64
	DWORD dwReserved1;
#endif
	DWORD64 dwInputBufferLength;
	LPVOID lpOutputBuffer;
#ifndef _WIN64
	DWORD dwReserved2;
#endif
	DWORD64 dwOutputBufferLength;
	LPVOID lpOutputLength;
#ifndef _WIN64
	DWORD dwReserved3;
#endif
};
#pragma pack(pop)

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
	NETUP_IOCTL_CI_APPLICATION_INFO = 0x21,
	NETUP_IOCTL_CI_CONDITIONAL_ACCESS_INFO = 0x22,
	NETUP_IOCTL_CI_RESET = 0x23,
	NETUP_IOCTL_CI_MMI_ENTER_MENU = 0x30,
	NETUP_IOCTL_CI_MMI_GET_MENU = 0x31,
	NETUP_IOCTL_CI_MMI_ANSWER_MENU = 0x32,
	NETUP_IOCTL_CI_MMI_CLOSE = 0x33,
	NETUP_IOCTL_CI_MMI_GET_ENQUIRY = 0x34,
	NETUP_IOCTL_CI_MMI_PUT_ANSWER = 0x35,
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

/* MMI_PUT_ANSWER first argument */
typedef enum
{
	MMI_ANSWER_CANCEL = 0,
	MMI_ANSWER_OK = 1,
};

/* NETUP_CAM_STATUS::dwCamStatus bits */
typedef enum
{
	NETUP_CAM_PRESENT = 1,
	NETUP_CAM_MMI_DATA_READY = 2,
	NETUP_CAM_MMI_ENQ_READY = 4,
};

/* MMI menu limitations */
enum
{
	NETUP_MAX_STRING_LENGTH = 256,
	NETUP_MAX_CA_ID_COUNT = 256,
	MENU_MAX_ITEM_LENGTH = NETUP_MAX_STRING_LENGTH,
	MENU_MAX_ITEM_COUNT = 64,
};

#pragma pack(push, 1)
/* NETUP_IOCTL_CI_STATUS reply format */
struct NETUP_CAM_STATUS
{
	DWORD64 dwCamStatus;
	WORD wCamVendor;
	WORD wCamDevice;
	CHAR cCamString[NETUP_MAX_STRING_LENGTH];
};

struct NETUP_CAM_APPLICATION_INFO
{
	BYTE bAppType;
	WORD wAppVendor;
	WORD wAppCode;
	CHAR cAppString[NETUP_MAX_STRING_LENGTH];
};

struct NETUP_CAM_INFO
{
	DWORD64 dwSize;
	WORD wCaSystemIdList[NETUP_MAX_CA_ID_COUNT];
};

/* NETUP_IOCTL_CI_MMI_GET_MENU reply format */
struct NETUP_CAM_MENU
{
	BOOL bIsMenu;
	CHAR cTitle[MENU_MAX_ITEM_LENGTH];
	CHAR cSubTitle[MENU_MAX_ITEM_LENGTH];
	CHAR cBottomText[MENU_MAX_ITEM_LENGTH];
	CHAR cItems[MENU_MAX_ITEM_COUNT][MENU_MAX_ITEM_LENGTH];
	DWORD64 dwItemCount;
};

struct NETUP_CAM_MMI_ENQUIRY
{
	BOOL bBlindAnswer;
	BYTE bAnswerLength;
	CHAR cString[MENU_MAX_ITEM_LENGTH];
};

struct NETUP_CAM_MMI_ANSWER
{
	BYTE bAnswerLength;
	CHAR cAnswer[MENU_MAX_ITEM_LENGTH];
};
#pragma pack(pop)

#endif

