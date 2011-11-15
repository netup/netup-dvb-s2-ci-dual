/*
 * en50221.cpp - EN50221 protocol implementation
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

#include "en50221.h"
#include "netup_bda_api.h"

#define EN50221_BUF_SIZE	512

// EN50221 Transport Layer constants
#define T_SB                0x80    // sb                           primitive   h<--m
#define T_RCV               0x81    // receive                      primitive   h-->m
#define T_CREATE_T_C        0x82    // create transport connection  primitive   h-->m
#define T_C_T_C_REPLY       0x83    // ctc reply                    primitive   h<--m
#define T_DELETE_T_C        0x84    // delete tc                    primitive   h<->m
#define T_D_T_C_REPLY       0x85    // dtc reply                    primitive   h<->m
#define T_REQUEST_T_C       0x86    // request transport connection primitive   h<--m
#define T_NEW_T_C           0x87    // new tc / reply to t_request  primitive   h-->m
#define T_T_C_ERROR         0x77    // error creating tc            primitive   h-->m
#define T_DATA_LAST         0xA0    // convey data from higher      constructed h<->m
#define T_DATA_MORE         0xA1    // convey data from higher      constructed h<->m

// EN50221 Session Layer constants
#define ST_OPEN_SESSION_REQ     0x91	// h<--m
#define ST_OPEN_SESSION_RES     0x92    // h-->m
#define ST_CREATE_SESSION       0x93    // h-->m
#define ST_CREATE_SESSION_RES   0x94    // h<--m
#define ST_CLOSE_SESSION_REQ    0x95    // h<->m
#define ST_CLOSE_SESSION_RES    0x96    // h<->m
#define ST_SESSION_NUMBER       0x90    // h<->m

// EN50221 implementation limits
#define EN50221_MAX_TC			2
#define EN50221_MAX_SESSION		5

// Transport Connection state
#define EN50221_TC_STATE_NONE		0
#define EN50221_TC_STATE_WAIT_OPEN	1
#define EN50221_TC_STATE_OPEN		2
#define EN50221_TC_STATE_WAIT_CLOSE	3
#define EN50221_TC_STATE_CLOSE		4
#define EN50221_TC_STATE_ERROR		5
#define EN50221_TC_STATE_MAX		EN50221_TC_STATE_ERROR

// CAM module polling delay
#define EN50221_TC_POLL_DELAY		CLOCKS_PER_SEC * 2

// Resource Manager resource ID
#define CAM_RESOURCE_RM			0x00010041
// Application Information resource ID
#define CAM_RESOURCE_AI			0x00020041
// Conditional Access resource ID
#define CAM_RESOURCE_CA			0x00030041
// MMI
#define CAM_RESOURCE_MMI		0x00400041

// Resource Manager tags
#define TAG_PROFILE_ENQUIRY		0x009f8010
#define TAG_PROFILE_REPLY		0x009f8011
#define TAG_PROFILE_CHANGE		0x009f8012

// Application Info tags
#define TAG_APP_INFO_ENQUIRY	0x009f8020
#define TAG_APP_INFO			0x009f8021
#define TAG_ENTER_MENU			0x009f8022

// Conditional Access
#define TAG_CA_INFO_ENQUIRY     0x009f8030
#define TAG_CA_INFO             0x009f8031
#define TAG_CA_PMT              0x009f8032
#define TAG_CA_PMT_REPLY        0x009f8033

// MMI
#define TAG_CLOSE_MMI			0x009f8800
#define TAG_DISPLAY_CONTROL		0x009f8801
#define TAG_DISPLAY_REPLY       0x009f8802
#define TAG_TEXT_LAST			0x009f8803
#define TAG_LIST_LAST			0x009f880c
#define TAG_MENU_LAST			0x009f8809
#define TAG_MENU_ANSW			0x009f880b


/**
  * CA PMT structures
  */
struct CA_PMT_Descriptor
{
	ULONG size;
	PUCHAR data;
};

struct CA_PMT_Stream
{
	UCHAR type;
	USHORT pid;
	CA_PMT_Descriptor * descriptors;
	ULONG descriptor_count;
};

struct CA_PMT
{
	UCHAR ca_pmt_list_management;
	UCHAR ca_pmt_cmd_id;
	USHORT program_number;
	UCHAR version_number;
	UCHAR current_next_indicator;
	CA_PMT_Descriptor * descriptors;
	ULONG descriptor_count;
	CA_PMT_Stream * streams;
	ULONG stream_count;
	LIST_ENTRY link;
};

/*
 * MMI menu structure
 */ 
struct MMI_Menu
{
	LIST_ENTRY link;
	NETUP_CAM_MENU menu;
};


/**
  * EN50221 internal structures
  */
struct EN50221_MessageData
{
	PUCHAR data;
	ULONG size;
	UCHAR type;
	LIST_ENTRY link;
};

struct EN50221_Session
{
	/* EN50221 Session ID */
	USHORT id;
	/* EN50221 Resource ID  associated with session */
	ULONG resource_id;
	/* Optional resource-specific context */
	PVOID context;
};

struct EN50221_TransportConnection
{
	/* EN50221 Transport Connection state */
	ULONG state;
	/* EN50221 Transport Connection ID */
	UCHAR conn_id;
	/* CI Hardware access */
	netup_ci_state * ci;
	/* EN50221 Session slots */
	EN50221_Session sessions[EN50221_MAX_SESSION];
	/* Incoming packets buffer */
	LIST_ENTRY rx_buffer;
};

struct EN50221_Context
{
	/* EN50221 Transport Connection slots */
	EN50221_TransportConnection connections[EN50221_MAX_TC];
	/* CI Hardware access */
	netup_ci_state * ci;
	/* Application info fields */
	USHORT camVendor;
	USHORT camDevice;
	CHAR camString[256];
	/* CA PMT list management fields */
	FAST_MUTEX caPmtMutex;
	LIST_ENTRY caPmtList;
	/* MMI fields */
	FAST_MUTEX menuMutex;
	LIST_ENTRY menuList;
	BOOLEAN menuEnter;
	BOOLEAN menuClose;
	UCHAR menuAnsw;
};

static VOID EN50221_Parse_R_TPU(EN50221_TransportConnection * tc, PUCHAR buf, ULONG size);
static LONG EN50221_APP_CA_PMT_Build(const CA_PMT * pmt, const PUSHORT ca_id_list, PUCHAR * pmt_data);
static VOID EN50221_APP_CA_PMT_Free(CA_PMT * pmt);

static LONG ASN_1_Encode(USHORT length, PUCHAR asn_1_array, ULONG asn_1_array_len)
{
    if (length < 0x80) {
        if (asn_1_array_len < 1)
            return -1;

        asn_1_array[0] = length & 0x7f;
        return 1;
    } else if (length < 0x100) {
        if (asn_1_array_len < 2)
            return -1;
        asn_1_array[0] = 0x81;
        asn_1_array[1] = length & 0xff;
        return 2;
    } else {
        if (asn_1_array_len < 3)
            return -1;

        asn_1_array[0] = 0x82;
        asn_1_array[1] = length >> 8;
        asn_1_array[2] = length & 0xff;
        return 3;
    }
    // never reached
}

static LONG ASN_1_Decode(PUSHORT length, PUCHAR asn_1_array, ULONG asn_1_array_len)
{
    USHORT length_field;

    if (asn_1_array_len < 1)
        return -1;
    length_field = asn_1_array[0];

    if (length_field < 0x80) {
        // there is only one word
        *length = length_field & 0x7f;
        return 1;
    } else if (length_field == 0x81) {
        if (asn_1_array_len < 2)
            return -1;

        *length = asn_1_array[1];
        return 2;
    } else if (length_field == 0x82) {
        if (asn_1_array_len < 3)
            return -1;
        *length = (asn_1_array[1] << 8) | asn_1_array[2];
        return 3;
    }
    return -1;
}

static VOID EN50221_SetConnectionState(EN50221_TransportConnection * tc, ULONG state)
{
	static const char * names[] = {
		"EN50221_TC_STATE_NONE",
		"EN50221_TC_STATE_WAIT_OPEN",
		"EN50221_TC_STATE_OPEN",
		"EN50221_TC_STATE_WAIT_CLOSE",
		"EN50221_TC_STATE_CLOSE",
		"EN50221_TC_STATE_ERROR"
	};
	ASSERT(tc != NULL && state <= EN50221_TC_STATE_MAX);
	ULONG old_state = tc->state;
	tc->state = state;
	if(old_state != state)
	{
		KdPrint((LOG_PREFIX "EN50221 connection state changed from %s to %s", names[old_state], names[state]));
	}
	else
	{
		KdPrint((LOG_PREFIX "EN50221 connection state %s unchanged", names[old_state]));
	}
}

static VOID EN50221_Process_T_SB(EN50221_TransportConnection * tc, PUCHAR buf, ULONG size)
{
	ASSERT(tc != NULL && buf != NULL && tc->conn_id > 0);
	if(tc->state != EN50221_TC_STATE_OPEN)
	{
		KdPrint((LOG_PREFIX "invalid connection state, EN50221_TC_STATE_WAIT_CLOSE expected"));
		goto set_error;
	}
	if(size < 2)
	{
		KdPrint((LOG_PREFIX "invalid T_SB PDO size"));
		goto set_error;
	}
	if(buf[1] & 0x80)
	{
		UCHAR hdr[3];
		hdr[0] = T_RCV;
		hdr[1] = 1;
		hdr[2] = tc->conn_id;
		if(Netup_CAM_Write(tc->ci, tc->conn_id, hdr, sizeof(hdr)) != sizeof(hdr))
		{
			KdPrint((LOG_PREFIX "unable to sent T_RCV"));
			goto set_error;
		}
		PUCHAR reply;
		LONG reply_size;
		UCHAR reply_conn_id;
		reply_size = Netup_CAM_Read(tc->ci, &reply_conn_id, &reply, NULL);
		if(reply_size < 0)
		{
			KdPrint((LOG_PREFIX "CAM read failed: %d", reply_size));
			goto set_error;
		}
		else if(reply_size > 0)
		{
			EN50221_Parse_R_TPU(tc, reply, reply_size);
			ExFreePool(reply);
		}
		else
		{
			KdPrint((LOG_PREFIX "hmm.. CAM reply is empty"));
		}
	}
	return;

set_error:
	EN50221_SetConnectionState(tc, EN50221_TC_STATE_ERROR);
}

static VOID EN50221_Parse_R_TPU(EN50221_TransportConnection * tc, PUCHAR buf, ULONG size)
{
	ASSERT(tc != NULL);
	if(size < 2)
	{
		KdPrint((LOG_PREFIX "R_TPU too short"));
		goto set_error;
	}
	PUCHAR ptr = buf;
	UCHAR code = ptr[0];
	USHORT pdo_len;
	LONG size_field_length = ASN_1_Decode(&pdo_len, ptr + 1, size - 1);
	if(size_field_length < 0)
	{
		KdPrint((LOG_PREFIX "unable to decode ASN.1 data"));
		goto set_error;
	}
	
	if(pdo_len + (ULONG)size_field_length + 1 > size)
	{
		KdPrint((LOG_PREFIX "TPO size > R_TPU size"));
		goto set_error;
	}
	switch(code) {
		case T_C_T_C_REPLY:
		{
			if(tc->state != EN50221_TC_STATE_WAIT_OPEN)
			{
				KdPrint((LOG_PREFIX "invalid connection state, EN50221_TC_STATE_WAIT_OPEN expected"));
				goto set_error;
			}
			EN50221_SetConnectionState(tc, EN50221_TC_STATE_OPEN);
			break;
		}
		case T_D_T_C_REPLY:
		{
			if(tc->state != EN50221_TC_STATE_WAIT_CLOSE)
			{
				KdPrint((LOG_PREFIX "invalid connection state, EN50221_TC_STATE_WAIT_CLOSE expected"));
				goto set_error;
			}
			EN50221_SetConnectionState(tc, EN50221_TC_STATE_CLOSE);
			break;
		}
		case T_SB:
		{
			EN50221_Process_T_SB(tc, ptr + size_field_length + 1, pdo_len);
			if(tc->state == EN50221_TC_STATE_ERROR)
				return;
			break;
		}
		case T_DATA_MORE:
			KdPrint((LOG_PREFIX "T_DATA_MORE not implemented"));
			break;
		case T_DATA_LAST:
			{
				if(pdo_len < 2)
				{
					KdPrint((LOG_PREFIX "got emptry T_DATA_LAST"));
					break;
				}
				EN50221_MessageData * msg = (EN50221_MessageData *)ExAllocatePoolWithTag(NonPagedPool, sizeof(*msg), 'DGSM');
				if(msg)
				{
					msg->type = T_DATA_LAST;
					msg->size = pdo_len - 1;
					msg->data = (PUCHAR)ExAllocatePoolWithTag(NonPagedPool, pdo_len - 1, 'DGSM');
					if(!msg->data)
					{
						ExFreePool(msg);
						KdPrint((LOG_PREFIX "unable to allocate %d bytes of EN50221_MessageData", pdo_len - 1));
					}
					else
					{
						KdPrint((LOG_PREFIX "got T_DATA_LAST data size %d", pdo_len - 1));
						RtlCopyMemory(msg->data, ptr + size_field_length + 2, pdo_len - 1);
						InsertTailList(&tc->rx_buffer, &msg->link);
					}
				}
				else
				{
					KdPrint((LOG_PREFIX "unable to allocate EN50221_MessageData structure"));
				}
				break;
			}
		case T_T_C_ERROR:
		{
			KdPrint((LOG_PREFIX "got T_T_C_ERROR"));
			goto set_error;
		}
		default:
		{
			KdPrint((LOG_PREFIX "unimplemented PDO code 0x%02x", code));
			break;
		}
	}
	if(size - (pdo_len + size_field_length + 1) > 0)
	{
		if(size - (pdo_len + size_field_length + 1) < 2)
		{
			KdPrint((LOG_PREFIX "second PDO too short"));
			goto set_error;
		}
		ptr += pdo_len + size_field_length + 1;
		if(ptr[0] != T_SB)
		{
			KdPrint((LOG_PREFIX "second PDO (0x%02x) != T_SB", (ULONG)ptr[0]));
			goto set_error;
		}
		if(code == T_SB)
		{
			KdPrint((LOG_PREFIX "got second T_SB in single R_TPU"));
			goto set_error;
		}
		EN50221_Process_T_SB(tc, ptr + 2, ptr[1]);
	}
	return;

set_error:
	EN50221_SetConnectionState(tc, EN50221_TC_STATE_ERROR);
}


PVOID EN50221_Init(netup_ci_state * ci)
{
	ASSERT(ci != NULL);
	EN50221_Context * en50221 = (EN50221_Context *)ExAllocatePoolWithTag(NonPagedPool, sizeof(*en50221), 'TCNE');
	if(!en50221)
	{
		KdPrint((LOG_PREFIX "unable to allocate EN50221 context"));
		return NULL;
	}
	RtlZeroMemory(en50221, sizeof(*en50221));
	InitializeListHead(&en50221->caPmtList);
	ExInitializeFastMutex(&en50221->caPmtMutex);
	InitializeListHead(&en50221->menuList);
	ExInitializeFastMutex(&en50221->menuMutex);
	en50221->menuAnsw = 0xff;
	en50221->ci = ci;
	return en50221;
}

VOID EN50221_Reset(PVOID ctx)
{
	EN50221_Context * context = (EN50221_Context *)ctx;
	for(ULONG i = 0; i < EN50221_MAX_TC; i++)
	{
		if(context->connections[i].state != EN50221_TC_STATE_NONE)
		{
			EN50221_TransportConnection * tc = &context->connections[i];
			tc->state = EN50221_TC_STATE_NONE;
			while(!IsListEmpty(&tc->rx_buffer))
			{
				LIST_ENTRY * entry = RemoveTailList(&tc->rx_buffer);
				EN50221_MessageData * msg = CONTAINING_RECORD(entry, EN50221_MessageData, link);
				if(msg->data)
					ExFreePool(msg->data);
				ExFreePool(msg);
			}
			RtlZeroMemory(&tc->sessions, sizeof(tc->sessions));
		}
	}
	if(!IsListEmpty(&context->menuList))
	{
		LIST_ENTRY * tmp = RemoveHeadList(&context->menuList);
		MMI_Menu * m = CONTAINING_RECORD(tmp, MMI_Menu, link);
		ExFreePool(m);
	}
	context->menuEnter = FALSE;
	context->menuClose = FALSE;
	context->menuAnsw = 0xff;
	if(!IsListEmpty(&context->caPmtList))
	{
		LIST_ENTRY * tmp = RemoveHeadList(&context->caPmtList);
		CA_PMT * pmt = CONTAINING_RECORD(tmp, CA_PMT, link);
		EN50221_APP_CA_PMT_Free(pmt);
	}
}

LONG EN50221_TL_CreateConnection(PVOID ctx)
{
	KdPrint((LOG_PREFIX "%s", __FUNCTION__));
	EN50221_Context * en50221 = (EN50221_Context *)ctx;
	ASSERT(en50221 != NULL && en50221->ci != NULL);
	ULONG i;
	EN50221_TransportConnection * tc = NULL;
	for(i = 0; i < EN50221_MAX_TC; i++)
	{
		if(en50221->connections[i].state == EN50221_TC_STATE_NONE)
		{
			KdPrint((LOG_PREFIX "%s: new connection #%d", __FUNCTION__, i + 1));
			tc = &en50221->connections[i];
			EN50221_SetConnectionState(tc, EN50221_TC_STATE_WAIT_OPEN);
			tc->conn_id = (i + 1) & 0xff;
			tc->ci = en50221->ci;
			InitializeListHead(&tc->rx_buffer);
			break;
		}
	}
	if(tc == NULL)
	{
		KdPrint((LOG_PREFIX "%s: free connection pool is empty", __FUNCTION__));
		return EN50221_RESOURCE_BUSY;
	}
	UCHAR buf[3];
	buf[0] = T_CREATE_T_C;
	buf[1] = 1;
	buf[2] = tc->conn_id;
	if(Netup_CAM_Write(tc->ci, tc->conn_id, buf, sizeof(buf)) != sizeof(buf))
	{
		KdPrint((LOG_PREFIX "%s: failed to send T_CREATE_T_C", __FUNCTION__));
		return EN50221_IO_ERROR;
	}
	PUCHAR reply;
	LONG size;
	UCHAR reply_conn_id;
	size = Netup_CAM_Read(tc->ci, &reply_conn_id, &reply, NULL);
	if(size <= 0)
	{
		KdPrint((LOG_PREFIX "%s: CAM not responding: %d", __FUNCTION__, size));
		return EN50221_TIMEOUT;
	}
	EN50221_Parse_R_TPU(tc, reply, size);
	ExFreePool(reply);
	if(tc->state != EN50221_TC_STATE_OPEN)
	{
		EN50221_SetConnectionState(tc, EN50221_TC_STATE_NONE);
		return EN50221_INVALID_REPLY;
	}
	return tc->conn_id;
}

LONG EN50221_TL_DeleteConnection(PVOID ctx, UCHAR conn_id)
{
	KdPrint((LOG_PREFIX "%s", __FUNCTION__));
	EN50221_Context * en50221 = (EN50221_Context *)ctx;
	ASSERT(en50221 != NULL && en50221->ci != NULL);
	if(conn_id == 0 || conn_id > EN50221_MAX_TC)
	{
		KdPrint((LOG_PREFIX "%s: invalid conn_id %d", __FUNCTION__, conn_id));
		return EN50221_IO_ERROR;
	}
	EN50221_TransportConnection * tc = &en50221->connections[conn_id - 1];
	if(tc->state != EN50221_TC_STATE_OPEN)
	{
		KdPrint((LOG_PREFIX "%s: inactive connection", __FUNCTION__));
		return EN50221_IO_ERROR;
	}
	UCHAR buf[3];
	buf[0] = T_DELETE_T_C;
	buf[1] = 1;
	buf[2] = conn_id;
	if(Netup_CAM_Write(tc->ci, tc->conn_id, buf, sizeof(buf)) != sizeof(buf))
	{
		KdPrint((LOG_PREFIX "%s: failed to send T_DELETE_T_C", __FUNCTION__));
		EN50221_SetConnectionState(tc, EN50221_TC_STATE_ERROR);
		return EN50221_IO_ERROR;
	}
	EN50221_SetConnectionState(tc, EN50221_TC_STATE_WAIT_CLOSE);
	PUCHAR reply;
	LONG size;
	UCHAR reply_conn_id;
	size = Netup_CAM_Read(tc->ci, &reply_conn_id, &reply, NULL);
	if(size <= 0)
	{
		KdPrint((LOG_PREFIX "%s: CAM not responding: %d", __FUNCTION__, size));
		EN50221_SetConnectionState(tc, EN50221_TC_STATE_ERROR);
		return EN50221_TIMEOUT;
	}
	EN50221_Parse_R_TPU(tc, reply, size);
	ExFreePool(reply);
	if(tc->state != EN50221_TC_STATE_CLOSE)
	{
		EN50221_SetConnectionState(tc, EN50221_TC_STATE_ERROR);
		return EN50221_INVALID_REPLY;
	}
	EN50221_SetConnectionState(tc, EN50221_TC_STATE_NONE);
	return conn_id;
}

LONG EN50221_TL_Write(PVOID ctx, UCHAR conn_id, PUCHAR data, ULONG size)
{
	EN50221_Context * en50221 = (EN50221_Context *)ctx;
	ASSERT(en50221 != NULL && en50221->ci != NULL);
	if(conn_id == 0 || conn_id > EN50221_MAX_TC)
	{
		KdPrint((LOG_PREFIX "%s: invalid conn_id %d", __FUNCTION__, conn_id));
		return EN50221_IO_ERROR;
	}
	EN50221_TransportConnection * tc = &en50221->connections[conn_id - 1];
	if(tc->state != EN50221_TC_STATE_OPEN)
	{
		KdPrint((LOG_PREFIX "%s: invalid connection state", __FUNCTION__));
		return EN50221_IO_ERROR;
	}
	PUCHAR buf = (PUCHAR)ExAllocatePoolWithTag(NonPagedPool, size + 5, 'BPMT');
	if(!buf)
	{
		KdPrint((LOG_PREFIX "%s: unable to allocate tmp buffer", __FUNCTION__));
		return EN50221_IO_ERROR;
	}
	buf[0] = T_DATA_LAST;
	LONG size_field_length = ASN_1_Encode((size + 1) & 0xffff, buf + 1, 3);
	if(size_field_length < 0)
	{
		ExFreePool(buf);
		KdPrint((LOG_PREFIX "ASN.1 encoding failed"));
		return EN50221_IO_ERROR;
	}
	buf[1 + size_field_length] = tc->conn_id;
	if(size > 0)
		RtlCopyMemory(buf + size_field_length + 2, data, size);
	if(Netup_CAM_Write(tc->ci, tc->conn_id, buf, size_field_length + size + 2) <= 0)
	{
		ExFreePool(buf);
		KdPrint((LOG_PREFIX "%s: CAM write failed", __FUNCTION__));
		return EN50221_IO_ERROR;
	}
	ExFreePool(buf);
	PUCHAR reply;
	LONG reply_size;
	UCHAR reply_conn_id;
	reply_size = Netup_CAM_Read(tc->ci, &reply_conn_id, &reply, NULL);
	if(reply_size <= 0)
	{
		KdPrint((LOG_PREFIX "%s: CAM not responding", __FUNCTION__));
		return EN50221_IO_ERROR;
	}
	if(reply_conn_id != conn_id)
	{
		KdPrint((LOG_PREFIX "%s: CAM reply.conn_id != req.conn_id", __FUNCTION__));
		if(reply_conn_id == 0 || reply_conn_id > EN50221_MAX_TC)
		{
			KdPrint((LOG_PREFIX "%s: invalid reply conn_id %d", __FUNCTION__, conn_id));
			return EN50221_IO_ERROR;
		}
		tc = &en50221->connections[reply_conn_id - 1];
	}
	EN50221_Parse_R_TPU(tc, reply, reply_size);
	return 0;
}

LONG EN50221_TL_GetState(PVOID ctx, UCHAR conn_id)
{
	EN50221_Context * en50221 = (EN50221_Context *)ctx;
	ASSERT(en50221 != NULL && en50221->ci != NULL);
	if(conn_id == 0 || conn_id > EN50221_MAX_TC)
	{
		KdPrint((LOG_PREFIX "%s: invalid conn_id %d", __FUNCTION__, conn_id));
		return EN50221_IO_ERROR;
	}
	EN50221_TransportConnection * tc = &en50221->connections[conn_id - 1];
	return tc->state;
}

LONG EN50221_TL_ResetConnection(PVOID ctx, UCHAR conn_id)
{
	EN50221_Context * en50221 = (EN50221_Context *)ctx;
	ASSERT(en50221 != NULL && en50221->ci != NULL);
	if(conn_id == 0 || conn_id > EN50221_MAX_TC)
	{
		KdPrint((LOG_PREFIX "%s: invalid conn_id %d", __FUNCTION__, conn_id));
		return EN50221_IO_ERROR;
	}
	EN50221_TransportConnection * tc = &en50221->connections[conn_id - 1];
	EN50221_SetConnectionState(tc, EN50221_TC_STATE_NONE);
	return conn_id;
}

static LONG EN50221_TL_GetData(PVOID ctx, UCHAR conn_id, PUCHAR * reply)
{
	EN50221_Context * en50221 = (EN50221_Context *)ctx;
	ASSERT(en50221 != NULL && en50221->ci != NULL);
	if(conn_id == 0 || conn_id > EN50221_MAX_TC)
	{
		KdPrint((LOG_PREFIX "%s: invalid conn_id %d", __FUNCTION__, conn_id));
		return EN50221_IO_ERROR;
	}
	EN50221_TransportConnection * tc = &en50221->connections[conn_id - 1];
	if(IsListEmpty(&tc->rx_buffer))
	{
		KdPrint((LOG_PREFIX "%s: no data", __FUNCTION__));
		return 0;
	}
	EN50221_MessageData * msg = (EN50221_MessageData *)CONTAINING_RECORD(tc->rx_buffer.Flink, EN50221_MessageData, link);
	if(msg->type != T_DATA_LAST)
	{
		KdPrint((LOG_PREFIX "%s: type != T_DATA_LAST", __FUNCTION__));
		return 0;
	}
	RemoveHeadList(&tc->rx_buffer);
	*reply = msg->data;
	LONG size = msg->size;
	ExFreePool(msg);
	KdPrint((LOG_PREFIX "%s: size %d", __FUNCTION__, size));
	return size;
}

static LONG EN50221_Util_PutAppTag(PUCHAR dst, ULONG size, ULONG tag)
{
	ASSERT(size >= 3);
	dst[0] = (tag >> 16) & 0xff;
	dst[1] = (tag >> 8) & 0xff;
	dst[2] = tag & 0xff;
	return 3;
}

static LONG EN50221_Util_PutResourceId(PUCHAR dst, ULONG size, ULONG resource_id)
{
	ASSERT(size >= 4);
	dst[0] = (resource_id >> 24) & 0xff;
	dst[1] = (resource_id >> 16) & 0xff;
	dst[2] = (resource_id >> 8) & 0xff;
	dst[3] = resource_id & 0xff;
	return 4;
}

static LONG EN50221_SL_Write(PVOID ctx, UCHAR conn_id, USHORT sess_id, PUCHAR data, ULONG size)
{
	ASSERT(conn_id != 0 && sess_id != 0);
	KdPrint((LOG_PREFIX "%s", __FUNCTION__));
	PUCHAR buf = (PUCHAR)ExAllocatePoolWithTag(NonPagedPool, size + 4, 'RWLS');
	if(!buf)
	{
		KdPrint((LOG_PREFIX "%s: unable to allocate buffer", __FUNCTION__));
		return EN50221_NOMEM;
	}
	buf[0] = ST_SESSION_NUMBER;
	buf[1] = 2;
	buf[2] = (sess_id >> 8) & 0xff;
	buf[3] = (sess_id & 0xff);
	if(size != 0)
		RtlCopyMemory(buf + 4, data, size);
	LONG result = EN50221_TL_Write(ctx, conn_id, buf, size + 4);
	ExFreePool(buf);
	return result;
}

static LONG EN50221_SL_SessionOpened(PVOID ctx, UCHAR conn_id, EN50221_Session * sess)
{
	switch(sess->resource_id) {
	case CAM_RESOURCE_RM:
	{
		KdPrint((LOG_PREFIX "Resource Manager sess_id %d open, sending TAG_PROFILE_CHANGE", sess->id));
		UCHAR hdr[4] = { 0, 0, 0, 0 };
		EN50221_Util_PutAppTag(hdr, 3, TAG_PROFILE_CHANGE);
		return EN50221_SL_Write(ctx, conn_id, sess->id, hdr, sizeof(hdr));
	}
	case CAM_RESOURCE_AI:
	{
		KdPrint((LOG_PREFIX "Application Info sess_id %d open, sending TAG_APPLICATION_INFO", sess->id));
		UCHAR hdr[4] = { 0, 0, 0, 0 };
		EN50221_Util_PutAppTag(hdr, 3, TAG_APP_INFO_ENQUIRY);
		return EN50221_SL_Write(ctx, conn_id, sess->id, hdr, sizeof(hdr));
	}
	case CAM_RESOURCE_CA:
	{
		KdPrint((LOG_PREFIX "Conditional Access sess_id %d open, sending TAG_CA_INFO_ENQUIRY", sess->id));
		UCHAR hdr[4] = { 0, 0, 0, 0 };
		EN50221_Util_PutAppTag(hdr, 3, TAG_CA_INFO_ENQUIRY);
		return EN50221_SL_Write(ctx, conn_id, sess->id, hdr, sizeof(hdr));
	}
	case CAM_RESOURCE_MMI:
	{
		KdPrint((LOG_PREFIX "MMI sess_id %d open", sess->id));
		return 0;
	}
	default:
		KdPrint(("%s: sess_id %d: not implemented resource_id 0x%08x", __FUNCTION__, sess->id, sess->resource_id));
		break;
	}
	return 0;
}

static LONG EN50221_SL_OpenSessionRequest(PVOID ctx, UCHAR conn_id, PUCHAR data, ULONG size)
{
	EN50221_Context * en50221 = (EN50221_Context *)ctx;
	ASSERT(en50221 != NULL && en50221->ci != NULL);
	if(conn_id == 0 || conn_id > EN50221_MAX_TC)
	{
		KdPrint((LOG_PREFIX "%s: invalid conn_id %d", __FUNCTION__, conn_id));
		return EN50221_IO_ERROR;
	}
	if(size != 4)
	{
		KdPrint((LOG_PREFIX "%s: invalid data size %d", __FUNCTION__, size));
		return EN50221_IO_ERROR;
	}
	EN50221_TransportConnection * tc = &en50221->connections[conn_id - 1];
	EN50221_Session * sess = NULL;
	ULONG resource_id = ((ULONG)data[0] << 24) | ((ULONG)data[1] << 16) | ((ULONG)data[2] << 8) | ((ULONG)data[3]);
	KdPrint((LOG_PREFIX "got ST_SESSION_OPEN for resoucre_id 0x%08x", resource_id));
	for(ULONG i = 0; i < EN50221_MAX_SESSION; i++)
	{
		if(tc->sessions[i].resource_id == 0)
		{
			sess = &tc->sessions[i];
			sess->id = (i + 1) & 0xff;
			sess->resource_id = resource_id;
			sess->context = NULL;
			break;
		}
	}
	if(sess == NULL)
	{
		KdPrint((LOG_PREFIX "%s: unable to allocate session slot", __FUNCTION__));
	}
	else
	{
		KdPrint((LOG_PREFIX "%s: opened sess_id %d", __FUNCTION__, sess->id));
	}
	UCHAR hdr[9];
	hdr[0] = ST_OPEN_SESSION_RES;
	hdr[1] = sizeof(hdr) - 2;
	hdr[2] = (sess != NULL) ? 0 : 0xf1;
	RtlCopyMemory(&hdr[3], data, 4);
	hdr[7] = (sess != NULL) ? ((sess->id >> 8) & 0xff) : 0;
	hdr[8] = (sess != NULL) ? (sess->id & 0xff) : 0;
	if(EN50221_TL_Write(ctx, conn_id, hdr, sizeof(hdr)) < 0)
	{
		KdPrint((LOG_PREFIX "%s: unable to sent SL open session response", __FUNCTION__));
		return EN50221_IO_ERROR;
	}
	if(sess != NULL)
	{
		return EN50221_SL_SessionOpened(ctx, conn_id, sess);
	}
	return 0;
}

static LONG EN50221_SL_CloseSessionRequest(PVOID ctx, UCHAR conn_id, PUCHAR data, ULONG size)
{
	EN50221_Context * en50221 = (EN50221_Context *)ctx;
	ASSERT(en50221 != NULL && en50221->ci != NULL);
	if(conn_id == 0 || conn_id > EN50221_MAX_TC)
	{
		KdPrint((LOG_PREFIX "%s: invalid conn_id %d", __FUNCTION__, conn_id));
		return EN50221_IO_ERROR;
	}
	if(size != 2)
	{
		KdPrint((LOG_PREFIX "%s: invalid SPDU size", __FUNCTION__));
		return EN50221_IO_ERROR;
	}
	UCHAR hdr[5];
	hdr[0] = ST_CLOSE_SESSION_RES;
	hdr[1] = sizeof(hdr) - 2;
	hdr[2] = 0;
	hdr[3] = data[0];
	hdr[4] = data[1];
	USHORT sess_id = ((USHORT)data[0] << 8) | (USHORT)data[1];
	if(sess_id == 0 || sess_id > EN50221_MAX_SESSION)
	{
		KdPrint((LOG_PREFIX "%s: invalid sess_id (%d)", __FUNCTION__, sess_id));
		goto close_error;
	}
	EN50221_TransportConnection * tc = &en50221->connections[conn_id - 1];
	if(tc->state != EN50221_TC_STATE_OPEN)
	{
		KdPrint((LOG_PREFIX "%s: conn_id %d already closed", __FUNCTION__, conn_id));
		return EN50221_IO_ERROR;
	}
	EN50221_Session * sess = &tc->sessions[sess_id - 1];
	if(sess->resource_id == 0)
	{
		KdPrint((LOG_PREFIX "%s: conn_id %d session_id %d already closed", __FUNCTION__, conn_id, sess_id));
		goto close_error;
	}
	KdPrint((LOG_PREFIX "got ST_CLOSE_SESSION_REQ for conn_id %d sess_id %d resource_id 0x%08x", conn_id, sess_id, sess->resource_id));
	if(EN50221_TL_Write(ctx, conn_id, hdr, sizeof(hdr)) < 0)
	{
		KdPrint((LOG_PREFIX "%s: unable to sent SL close session response", __FUNCTION__));
		return EN50221_IO_ERROR;
	}
	if(sess->context != NULL)
	{
		ExFreePool(sess->context);
		sess->context = NULL;
	}
	sess->resource_id = 0;
	return 0;
	
close_error:
	KdPrint((LOG_PREFIX "%s: sending ST_CLOSE_SESSION_RES with error status", __FUNCTION__));
	hdr[2] = 0xf0;
	return EN50221_TL_Write(ctx, conn_id, hdr, sizeof(hdr));
}

static LONG EN50221_APP_ProfileEnquiry(PVOID ctx, UCHAR conn_id, USHORT sess_id, PUCHAR app_data, ULONG app_data_size)
{
	ULONG resources[4] = { CAM_RESOURCE_RM, CAM_RESOURCE_AI, CAM_RESOURCE_CA, CAM_RESOURCE_MMI };
	UCHAR hdr[3 + 3 + sizeof(resources)];
	ULONG offset = EN50221_Util_PutAppTag(hdr, 3, TAG_PROFILE_REPLY);
	LONG size_field_length = ASN_1_Encode(sizeof(resources), hdr + offset, 3);
	if(size_field_length < 0)
	{
		KdPrint((LOG_PREFIX "%s: unable to encode ASN.1 data", __FUNCTION__));
		return EN50221_IO_ERROR;
	}
	offset += (ULONG)size_field_length;
	for(ULONG i = 0; i < sizeof(resources)/sizeof(ULONG); i++)
	{
		offset += EN50221_Util_PutResourceId(hdr + offset, 4, resources[i]);
	}
	return EN50221_SL_Write(ctx, conn_id, sess_id, hdr, offset);
}

static LONG EN50221_APP_ApplicationInfo(PVOID ctx, UCHAR conn_id, USHORT sess_id, PUCHAR app_data, ULONG app_data_size)
{
	if(app_data_size < 6)
	{
		KdPrint((LOG_PREFIX "%s: data block too short", __FUNCTION__));
		return EN50221_IO_ERROR;
	}
	EN50221_Context * en50221 = reinterpret_cast<EN50221_Context *>(ctx);
	UCHAR app_type = app_data[0];
	ULONG app_vendor = ((ULONG)app_data[1] << 8) | (ULONG)app_data[2];
	ULONG app_vendor_code = ((ULONG)app_data[3] << 8) | (ULONG)app_data[4];
	UCHAR str_size = app_data[5];
	UCHAR app_str[256];
	for(UCHAR i = 0; i < str_size; i++)
	{
		app_str[i] = app_data[i + 6];
	}
	app_str[str_size] = 0;
	KdPrint((LOG_PREFIX "Application Info: type=0x%02x vendor=0x%04x code=0x%04x string=\"%s\"",
		(ULONG)app_type, app_vendor, app_vendor_code, app_str));
	return 0;
}

static LONG EN50221_APP_CA_Info(PVOID ctx, UCHAR conn_id, USHORT sess_id, PUCHAR app_data, ULONG app_data_size)
{
	ASSERT(ctx != NULL && conn_id > 0 && conn_id <= EN50221_MAX_TC && sess_id > 0 && sess_id <= EN50221_MAX_SESSION);
	if((app_data_size % 2) != 0)
	{
		KdPrint((LOG_PREFIX "%s: invlaid CA system ID list", __FUNCTION__));
		return EN50221_IO_ERROR;
	}
	USHORT * ca_system_list = (USHORT *)ExAllocatePoolWithTag(NonPagedPool, app_data_size + 2, 'SLAC');
	if(!ca_system_list)
	{
		KdPrint((LOG_PREFIX "%s: unable to allocate CA system list", __FUNCTION__));
	}
	for(ULONG i = 0; i < app_data_size/2; i++)
	{
		ca_system_list[i] = ((ULONG)app_data[i*2] << 8) | (ULONG)app_data[i*2 + 1];
		KdPrint((LOG_PREFIX "CA system ID 0x%04x", ca_system_list[i]));
	}
	ca_system_list[app_data_size/2] = 0xffff;
	EN50221_Context * en50221 = (EN50221_Context *)ctx;
	en50221->connections[conn_id - 1].sessions[sess_id - 1].context = ca_system_list;
	return 0;
}

static LONG EN50221_APP_MMI_DisplayControl(PVOID ctx, UCHAR conn_id, USHORT sess_id, PUCHAR app_data, ULONG app_data_size)
{
	if(app_data_size == 0)
	{
		KdPrint((LOG_PREFIX "%s: empty APDO", __FUNCTION__));
		return EN50221_IO_ERROR;
	}
	UCHAR display_control_cmd = app_data[0];
	// MMI_set_mode
	if(display_control_cmd == 1) 
	{
		if(app_data_size < 2)
		{
			KdPrint((LOG_PREFIX "%s: MMI mode not defined", __FUNCTION__));
			return EN50221_IO_ERROR;
		}
		UCHAR mmi_mode = app_data[1];
		//high-level mode
		if(mmi_mode != 1)
		{
			KdPrint((LOG_PREFIX "%s: unsupported MMI mode %d", __FUNCTION__, (ULONG)mmi_mode));
			return EN50221_IO_ERROR;
		}
		UCHAR reply[6];
		EN50221_Util_PutAppTag(reply, 3, TAG_DISPLAY_REPLY);
		reply[3] = 2; // length
		reply[4] = 1; // mmi_mode_ack
		reply[5] = 1; // high-level mode
		return EN50221_SL_Write(ctx, conn_id, sess_id, reply, sizeof(reply));
	}
	else
	{
		KdPrint((LOG_PREFIX "%s: unsupported display_control_cmd %d", __FUNCTION__, (ULONG)display_control_cmd));
		return EN50221_IO_ERROR;
	}
}

static LONG EN50221_APP_MMI_List(PVOID ctx, UCHAR conn_id, USHORT sess_id, BOOLEAN is_menu, PUCHAR app_data, ULONG app_data_size)
{
	if(app_data_size < 4*3 + 1)
	{
		KdPrint((LOG_PREFIX "%s: app data size too short: %d", __FUNCTION__, app_data_size));
		return EN50221_IO_ERROR;
	}
	UCHAR item_nb = app_data[0];
	ULONG offset = 1;
	MMI_Menu * m = (MMI_Menu *)ExAllocatePoolWithTag(PagedPool, sizeof(*m), 'MIMM');
	if(!m)
	{
		KdPrint((LOG_PREFIX "%s: unable to allocate CAM menu structure", __FUNCTION__));
		return EN50221_NOMEM;
	}
	RtlZeroMemory(m, sizeof(*m));
	m->menu.bIsMenu = is_menu;
	ULONG count = 0;
	while(offset < app_data_size)
	{
		if(app_data_size - offset < 4)
		{
			KdPrint((LOG_PREFIX "%s: hmm.. short TEXT field", __FUNCTION__));
			break;
		}
		ULONG app_tag = ((ULONG)app_data[offset] << 16) | ((ULONG)app_data[offset + 1] << 8) | (ULONG)app_data[offset + 2];
		if(app_tag != TAG_TEXT_LAST)
		{
			KdPrint((LOG_PREFIX "%s: unknown tag 0x%06x", app_tag));
			break;
		}
		USHORT text_size;
		LONG size_field_length = ASN_1_Decode(&text_size, app_data + offset + 3, app_data_size - (offset + 3));
		if(size_field_length < 0)
		{
			KdPrint((LOG_PREFIX "%s: unable to parse ASN.1 encoded data", __FUNCTION__));
			break;
		}
		if(text_size != 0)
		{
			PCHAR dst;
			switch(count) {
			case 0:
				dst = m->menu.cTitle;
				break;
			case 1:
				dst = m->menu.cSubTitle;
				break;
			case 2:
				dst = m->menu.cBottomText;
				break;
			default:
				dst = m->menu.cItems[count - 3];
				m->menu.dwItemCount++;
			}
			ULONG item_length = (text_size > (MENU_MAX_ITEM_LENGTH - 1)) ? (MENU_MAX_ITEM_LENGTH - 1) : text_size;
			RtlCopyMemory(dst, app_data + offset + size_field_length + 3, item_length);
			PUCHAR text = (PUCHAR)ExAllocatePoolWithTag(NonPagedPool, text_size + 1, 'TXTM');
			if(!text)
			{
				KdPrint((LOG_PREFIX "%s: uanble to allocate text buffer, size=%d", __FUNCTION__, (ULONG)text_size + 1));
				break;
			}
			for(ULONG i = 0; i < text_size; i++)
			{
				text[i] = app_data[offset + size_field_length + 3 + i];
			}
			text[text_size] = 0;
			KdPrint((LOG_PREFIX "Module say \"%s\"", text));
			ExFreePool(text);
		}
		offset += text_size + size_field_length + 3;
		count++;
		if(count > MENU_MAX_ITEM_COUNT + 3)
			break;
	}
	EN50221_Context * en50221 = (EN50221_Context *)ctx;
	ExAcquireFastMutex(&en50221->menuMutex);
	InsertTailList(&en50221->menuList, &(m->link));
	ExReleaseFastMutex(&en50221->menuMutex);
	return 0;
}

static LONG EN50221_APP_PendingActions(PVOID ctx, UCHAR conn_id)
{
	LONG result = 0;
	EN50221_Context * en50221 = (EN50221_Context *)ctx;
	for(ULONG i = 0; i < EN50221_MAX_SESSION; i++)
	{
		EN50221_Session * sess = &en50221->connections[conn_id - 1].sessions[i];
		if(sess->resource_id == CAM_RESOURCE_CA)
		{
			if(sess->context == NULL)
			{
				KdPrint((LOG_PREFIX "CAM not initialized"));
				return result;
			}
			LIST_ENTRY caPmtListToSend;
			InitializeListHead(&caPmtListToSend);
			ExAcquireFastMutex(&en50221->caPmtMutex);
			if(!IsListEmpty(&en50221->caPmtList))
			{
				CA_PMT * pmt = CONTAINING_RECORD(en50221->caPmtList.Flink, CA_PMT, link);
				if(pmt->ca_pmt_list_management == PMT_LIST_LAST || pmt->ca_pmt_list_management == PMT_LIST_ONLY)
				{
					KdPrint((LOG_PREFIX "%s: updating CA PMT list on CAM", __FUNCTION__));
					while(!IsListEmpty(&en50221->caPmtList))
					{
						LIST_ENTRY * tmp = RemoveTailList(&en50221->caPmtList);
						InsertTailList(&caPmtListToSend, tmp);
					}
				}
				else
				{
					KdPrint((LOG_PREFIX "%s: CA PMT list is not ready", __FUNCTION__));
				}
			}
			ExReleaseFastMutex(&en50221->caPmtMutex);

			while(!IsListEmpty(&caPmtListToSend))
			{
				LIST_ENTRY * tmp = RemoveHeadList(&caPmtListToSend);
				CA_PMT * ca_pmt = CONTAINING_RECORD(tmp, CA_PMT, link);
				if(result >= 0)
				{
					PUCHAR ca_pmt_data;
					LONG ca_pmt_size = EN50221_APP_CA_PMT_Build(ca_pmt, (PUSHORT)sess->context, &ca_pmt_data);
					if(ca_pmt_size > 0)
					{
						KdPrint((LOG_PREFIX "writing CA PMT to CAM, size %d", ca_pmt_size));
						result = EN50221_SL_Write(ctx, conn_id, sess->id, ca_pmt_data, ca_pmt_size);
					}
					ExFreePool(ca_pmt_data);
				}
				EN50221_APP_CA_PMT_Free(ca_pmt);
			}
			if(result != 0)
				return result;
		}
		else if(sess->resource_id == CAM_RESOURCE_AI)
		{
			ExAcquireFastMutex(&en50221->menuMutex);
			if(en50221->menuEnter == TRUE)
			{
				UCHAR hdr[4];
				RtlZeroMemory(hdr, sizeof(hdr));
				EN50221_Util_PutAppTag(hdr, 3, TAG_ENTER_MENU);
				result = EN50221_SL_Write(ctx, conn_id, sess->id, hdr, sizeof(hdr));
				en50221->menuEnter = FALSE;
				ExReleaseFastMutex(&en50221->menuMutex);
				return result;
			}
			else
			{
				ExReleaseFastMutex(&en50221->menuMutex);
			}
		}
		else if(sess->resource_id == CAM_RESOURCE_MMI)
		{
			ExAcquireFastMutex(&en50221->menuMutex);
			if(en50221->menuClose == TRUE)
			{
				KdPrint((LOG_PREFIX "Menu close"));
				UCHAR hdr[5];
				EN50221_Util_PutAppTag(hdr, 3, TAG_CLOSE_MMI);
				hdr[3] = 1;
				hdr[4] = 0;
				result = EN50221_SL_Write(ctx, conn_id, sess->id, hdr, sizeof(hdr));
				en50221->menuClose = FALSE;
				en50221->menuAnsw = 0xff;
				ExReleaseFastMutex(&en50221->menuMutex);
				return result;
			}
			else if(en50221->menuAnsw != 0xff)
			{
				KdPrint((LOG_PREFIX "Menu answer: selected item %d", (ULONG)en50221->menuAnsw));
				UCHAR hdr[5];
				EN50221_Util_PutAppTag(hdr, 3, TAG_MENU_ANSW);
				hdr[3] = 1;
				hdr[4] = en50221->menuAnsw;
				result = EN50221_SL_Write(ctx, conn_id, sess->id, hdr, sizeof(hdr));
				en50221->menuAnsw = 0xff;
				ExReleaseFastMutex(&en50221->menuMutex);
				return result;
			}
			else
			{
				ExReleaseFastMutex(&en50221->menuMutex);
			}
		}
	}
	return 0;
}

static LONG EN50221_Process_Application_Data(PVOID ctx, UCHAR conn_id, USHORT sess_id, PUCHAR data, ULONG size)
{
	ULONG offset = 0;
	while(offset < size)
	{
		if(size - offset < 4)
		{
			KdPrint((LOG_PREFIX "%s: APDO too short", __FUNCTION__));
			return EN50221_IO_ERROR;
		}
		ULONG app_tag = ((ULONG)data[offset] << 16) | ((ULONG)data[offset + 1] << 8) | (ULONG)data[offset + 2];
		USHORT app_data_size;
		LONG size_field_length = ASN_1_Decode(&app_data_size, data + offset + 3, size - (offset + 3));
		if(size_field_length < 0)
		{
			KdPrint((LOG_PREFIX "%s: unable to parse ASN.1 encoded data", __FUNCTION__));
			return EN50221_IO_ERROR;
		}
		LONG app_result;
		PUCHAR app_data = data + offset + size_field_length + 3;
		switch(app_tag) {
			case TAG_PROFILE_ENQUIRY:
				app_result = EN50221_APP_ProfileEnquiry(ctx, conn_id, sess_id, app_data, app_data_size);
				break;
			case TAG_APP_INFO:
				app_result = EN50221_APP_ApplicationInfo(ctx, conn_id, sess_id, app_data, app_data_size);
				break;
			case TAG_CA_INFO:
				app_result = EN50221_APP_CA_Info(ctx, conn_id, sess_id, app_data, app_data_size);
				break;
			case TAG_DISPLAY_CONTROL:
				app_result = EN50221_APP_MMI_DisplayControl(ctx, conn_id, sess_id, app_data, app_data_size);
				break;
			case TAG_LIST_LAST:
			case TAG_MENU_LAST:
				app_result = EN50221_APP_MMI_List(ctx, conn_id, sess_id, app_tag == TAG_MENU_LAST, app_data, app_data_size);
				break;
			default:
				{
				KdPrint((LOG_PREFIX "%s: unknown APDO tag 0x%08x data:", __FUNCTION__, app_tag));
				for(ULONG i = 0; i < app_data_size; i++)
				{
					KdPrint(("%02x '%c'", (ULONG)app_data[i], app_data[i]));
				}
				break;
				}
		}
		offset += size_field_length + app_data_size + 3;
	}
	return offset;
}

static LONG EN50221_SL_ProcessData(PVOID ctx, UCHAR conn_id, PUCHAR data, ULONG size)
{
	ULONG offset = 0;
	while(offset < size)
	{
		USHORT data_size;
		LONG size_field_length = ASN_1_Decode(&data_size, data + offset + 1, size - (offset + 1));
		if(size_field_length < 0)
		{
			KdPrint((LOG_PREFIX "%s: unable to parse ASN.1 encoded data", __FUNCTION__));
			return EN50221_IO_ERROR;
		}
		switch(data[offset])
		{
			case ST_OPEN_SESSION_REQ:
				if(EN50221_SL_OpenSessionRequest(ctx, conn_id, data + offset + size_field_length + 1, data_size) < 0)
				{
					KdPrint((LOG_PREFIX "%s: unable to process OpenSession request", __FUNCTION__));
					return EN50221_IO_ERROR;
				}
				break;
			case ST_CLOSE_SESSION_REQ:
				if(EN50221_SL_CloseSessionRequest(ctx, conn_id, data + offset + size_field_length + 1, data_size) < 0)
				{
					KdPrint((LOG_PREFIX "%s: unable to process CloseSession request", __FUNCTION__));
					return EN50221_IO_ERROR;
				}
				break;
			case ST_SESSION_NUMBER:
				{
					USHORT sess_id = ((USHORT)data[offset + size_field_length + 1] << 8) | (USHORT)data[offset + size_field_length + 2];
					ULONG app_data_offset = offset + size_field_length + data_size + 1;
					KdPrint((LOG_PREFIX "%s: application data sess_id %d size %d", (ULONG)sess_id, size - app_data_offset));
					return EN50221_Process_Application_Data(ctx, conn_id, sess_id, data + app_data_offset, size - app_data_offset);
				}
			default:
				{
					KdPrint((LOG_PREFIX "%s: unknown SL tag 0x%02x", __FUNCTION__, (ULONG)data[offset]));
					break;
				}
		}
		offset += data_size + size_field_length + 1;
	}
	return offset;
}

static BOOLEAN EN50221_CI_Running(PVOID ctx)
{
	EN50221_Context * en50221 = (EN50221_Context *)ctx;
	ASSERT(en50221 != NULL && en50221->ci != NULL);
	return Netup_CAM_Running(en50221->ci);
}

VOID EN50221_Main(PVOID ctx)
{
	LONG conn_id = EN50221_TL_CreateConnection(ctx);
	if(conn_id <= 0)
	{
		KdPrint((LOG_PREFIX "failed to create connection"));
		return;
	}
	PUCHAR reply;
	ULONG reply_size;
	LARGE_INTEGER delay;
	delay.QuadPart = -1000;
	LONG result = 0;
	while(EN50221_CI_Running(ctx))
	{
		result = EN50221_APP_PendingActions(ctx, conn_id & 0xff);
		if(result < 0)
		{
			KdPrint((LOG_PREFIX "%s: PendingActions failed", __FUNCTION__));
			break;
		}
		if(result == 0 && (EN50221_TL_Write(ctx, conn_id & 0xff, NULL, 0) < 0))
		{
			KdPrint((LOG_PREFIX "%s: write failed", __FUNCTION__));
			break;
		}
		reply_size = EN50221_TL_GetData(ctx, conn_id & 0xff, &reply);
		if(reply_size == 0)
		{
			KdPrint((LOG_PREFIX "no data, sleeping"));
			LARGE_INTEGER timeout, curr_time;
			KeQuerySystemTime(&timeout);
			timeout.QuadPart += EN50221_TC_POLL_DELAY;
			while(EN50221_CI_Running(ctx))
			{
				KeDelayExecutionThread(KernelMode, FALSE, &delay);
				KeQuerySystemTime(&curr_time);
				if(curr_time.QuadPart >= timeout.QuadPart)
					break;
			}
			continue;
		}
		KdPrint((LOG_PREFIX "%s: CAM reply size %d", __FUNCTION__, reply_size));
		if(EN50221_SL_ProcessData(ctx, conn_id & 0xff, reply, reply_size) < 0)
		{
			KdPrint((LOG_PREFIX "%s: EN50221 session layer processing failed", __FUNCTION__));
			ExFreePool(reply);
			break;
		}
		ExFreePool(reply);
	}
	EN50221_TL_DeleteConnection(ctx, conn_id & 0xff);
}

static LONG EN50221_APP_CA_PMT_ParseDescriptors(PUCHAR buf, ULONG size, CA_PMT_Descriptor * desc, ULONG desc_count)
{
	ULONG offset = 0;
	ULONG count = 0;
	while(offset < size)
	{
		if(offset + 2 > size)
		{
			KdPrint((LOG_PREFIX "%s: descriptor too short", __FUNCTION__));
			return EN50221_IO_ERROR;
		}
		UCHAR desc_type = buf[offset];
		UCHAR desc_size = buf[offset + 1];
		if(offset + desc_size + 2 > size)
		{
			KdPrint((LOG_PREFIX "%s: descriptor too long", __FUNCTION__));
			return EN50221_IO_ERROR;
		}
		// CA descriptor
		if(desc_type == 9)
		{
			if(desc != NULL)
			{
				if(count >= desc_count)
				{
					KdPrint((LOG_PREFIX "%s: no more slot for CA descriptor", __FUNCTION__));
					return count;
				}
				if(desc_size < 2)
				{
					KdPrint((LOG_PREFIX "%s: invalid CA descriptor", __FUNCTION__));
					return EN50221_IO_ERROR;
				}
				USHORT ca_system = ((USHORT)buf[offset + 2] << 8) | (USHORT)buf[offset + 3];
				KdPrint((LOG_PREFIX "CA descriptor #%d system ID 0x%04x length %d", count, ca_system, desc_size + 2));
				desc[count].size = desc_size + 2;
				desc[count].data = (PUCHAR)ExAllocatePoolWithTag(NonPagedPool, (ULONG)desc_size + 2, 'CSED');
				if(desc[count].data == NULL)
				{
					KdPrint((LOG_PREFIX "unable to allocate descriptor data, length %d", (ULONG)desc + 2));
					return EN50221_IO_ERROR;
				}
				RtlCopyMemory(desc[count].data, &buf[offset], (ULONG)desc_size + 2);
			}
			count++;
		}
		offset += (ULONG)desc_size + 2;
	}
	return count;
}

static LONG EN50221_APP_CA_PMT_ParseStreams(PUCHAR buf, ULONG size, CA_PMT_Stream * streams, ULONG stream_count)
{
	ULONG offset = 0;
	ULONG count = 0;
	while(offset < size)
	{
		if(offset + 5 > size)
		{
			KdPrint((LOG_PREFIX "%s: descriptor too short", __FUNCTION__));
			return EN50221_IO_ERROR;
		}
		UCHAR es_type = buf[offset];
		USHORT es_pid = (((USHORT)buf[offset + 1] << 8) | (USHORT)buf[offset + 2]) & 0x1fff;
		USHORT es_desc_size = (((USHORT)buf[offset + 3] << 8) | (USHORT)buf[offset + 4]) & 0xfff;
		if(streams)
		{
			if(count >= stream_count)
			{
				KdPrint((LOG_PREFIX "%s: no more slot for ES descriptor", __FUNCTION__));
				return count;
			}
			streams[count].type = es_type;
			streams[count].pid = es_pid;
			if(es_desc_size)
			{
				LONG desc_count = EN50221_APP_CA_PMT_ParseDescriptors(buf + offset + 5, es_desc_size, NULL, 0);
				if(desc_count < 0)
				{
					return desc_count;
				}
				if(desc_count > 0)
				{
					streams[count].descriptor_count = desc_count;
					streams[count].descriptors = (CA_PMT_Descriptor *)ExAllocatePoolWithTag(
						NonPagedPool, sizeof(CA_PMT_Descriptor) * desc_count, 'ACSD');
					if(streams[count].descriptors == NULL)
					{
						KdPrint((LOG_PREFIX "%s: unable to allocate descriptors array, size %d", __FUNCTION__, desc_count));
						return EN50221_IO_ERROR;
					}
					RtlZeroMemory(streams[count].descriptors, sizeof(CA_PMT_Descriptor) * desc_count);
					desc_count = EN50221_APP_CA_PMT_ParseDescriptors(
						buf + offset + 5,
						es_desc_size, 
						streams[count].descriptors,
						streams[count].descriptor_count);
					if(desc_count != streams[count].descriptor_count)
					{
						KdPrint((LOG_PREFIX "%s: unable to parse CA descriptors (%d)", __FUNCTION__, desc_count));
						return EN50221_IO_ERROR;
					}
				}
			}
			KdPrint((LOG_PREFIX "PMT: ES type 0x%02x PID 0x%04x CA desc count %d",
				es_type, es_pid, streams[count].descriptor_count));
		}
		count++;
		offset += es_desc_size + 5;
	}
	return count;
}

static VOID EN50221_APP_CA_PMT_Free(CA_PMT * pmt)
{
	if(pmt == NULL)
		return;
	if(pmt->descriptors != NULL)
	{
		for(ULONG i = 0; i < pmt->descriptor_count; i++)
		{
			if(pmt->descriptors[i].data != NULL)
			{
				ExFreePool(pmt->descriptors[i].data);
			}
		}
		ExFreePool(pmt->descriptors);
	}
	if(pmt->streams != NULL)
	{
		for(ULONG i = 0; i < pmt->stream_count; i++)
		{
			if(pmt->streams[i].descriptors != 0)
			{
				for(ULONG j = 0; j < pmt->streams[i].descriptor_count; j++)
				{
					if(pmt->streams[i].descriptors[j].data != NULL)
					{
						ExFreePool(pmt->streams[i].descriptors[j].data);
					}
				}
				ExFreePool(pmt->streams[i].descriptors);
			}
		}
		ExFreePool(pmt->streams);
	}
	ExFreePool(pmt);
}

CA_PMT * EN50221_APP_CA_PMT_Parse(PUCHAR buf, ULONG size)
{
	if(size < 12)
	{
		KdPrint((LOG_PREFIX "%s: PMT too short", __FUNCTION__));
		return NULL;
	}
	CA_PMT * pmt = (CA_PMT *)ExAllocatePoolWithTag(NonPagedPool, sizeof(*pmt), 'MPAC');
	if(!pmt)
	{
		KdPrint((LOG_PREFIX "unable to allocate CA PMT"));
		return NULL;
	}
	RtlZeroMemory(pmt, sizeof(*pmt));

	if(buf[0] != 2)
	{
		KdPrint((LOG_PREFIX "PMT: invalid table_id %d", (ULONG)buf[0]));
		goto pmt_error;
	}
	USHORT section_length = (((USHORT)buf[1] & 0xf) << 8) | (USHORT)buf[2];
	if((ULONG)section_length + 3 > size)
	{
		KdPrint((LOG_PREFIX "PMT: section length too big"));
		goto pmt_error;
	}

	pmt->program_number = ((USHORT)buf[3] << 8) | (USHORT)buf[4];
	pmt->version_number = (buf[5] >> 1) & 0x1f;
	pmt->current_next_indicator = buf[5] & 1;
	
	KdPrint(("PMT: Program number 0x%04x Version number %d Current next indicator %d",
		(ULONG)pmt->program_number, (ULONG)pmt->version_number, (ULONG)pmt->current_next_indicator));

	USHORT prog_info_length = (((USHORT)buf[10] << 8) | (USHORT)buf[11]) & 0xfff;
	if((ULONG)prog_info_length + 12 > size)
	{
		KdPrint((LOG_PREFIX "PMT: program info too long"));
		goto pmt_error;
	}

	if(prog_info_length != 0)
	{
		LONG desc_count = EN50221_APP_CA_PMT_ParseDescriptors(buf + 12, prog_info_length, NULL, 0);
		if(desc_count < 0)
		{
			goto pmt_error;
		}
		if(desc_count > 0)
		{
			pmt->descriptor_count = desc_count;
			pmt->descriptors = (CA_PMT_Descriptor *)ExAllocatePoolWithTag(
				NonPagedPool,
				sizeof(CA_PMT_Descriptor) * desc_count,
				'ACSD');
			if(pmt->descriptors == NULL)
			{
				KdPrint((LOG_PREFIX "%s: unable to allocate descriptors array, size %d", __FUNCTION__, desc_count));
				goto pmt_error;
			}
			RtlZeroMemory(pmt->descriptors, sizeof(CA_PMT_Descriptor) * desc_count);
			desc_count = EN50221_APP_CA_PMT_ParseDescriptors(buf + 12, prog_info_length, pmt->descriptors, pmt->descriptor_count);
			if(desc_count != pmt->descriptor_count)
			{
				KdPrint((LOG_PREFIX "%s: unable to parse CA descriptors (%d)", __FUNCTION__, desc_count));
				goto pmt_error;
			}
		}
	}
	LONG stream_count = EN50221_APP_CA_PMT_ParseStreams(buf + prog_info_length + 12, section_length - (prog_info_length + 13), NULL, 0);
	if(stream_count < 0)
	{
		goto pmt_error;
	}
	if(stream_count > 0)
	{
		pmt->stream_count = stream_count;
		pmt->streams = (CA_PMT_Stream *)ExAllocatePoolWithTag(
				NonPagedPool,
				sizeof(CA_PMT_Stream) * stream_count,
				'SEPM');
		if(pmt->streams == NULL)
		{
			KdPrint((LOG_PREFIX "%s: unable to allocate ES array size %d", __FUNCTION__, stream_count));
			goto pmt_error;
		}
		RtlZeroMemory(pmt->streams, sizeof(CA_PMT_Stream) * stream_count);
		stream_count = EN50221_APP_CA_PMT_ParseStreams(
			buf + prog_info_length + 12,
			section_length - (prog_info_length + 13),
			pmt->streams,
			pmt->stream_count);
		if(stream_count != pmt->stream_count)
		{
			KdPrint((LOG_PREFIX "%s: unable to parse ES info", __FUNCTION__));
			goto pmt_error;
		}
	}
	return pmt;

pmt_error:
	EN50221_APP_CA_PMT_Free(pmt);
	return NULL;
}

static LONG EN50221_APP_CA_PMT_Build(const CA_PMT * pmt, const PUSHORT ca_id_list, PUCHAR * pmt_data)
{
	ULONG prog_info_size = 0;
	for(ULONG i = 0; i < pmt->descriptor_count; i++)
		prog_info_size += pmt->descriptors[i].size;
	ULONG es_info_size = 0;
	for(ULONG i = 0; i < pmt->stream_count; i++)
	{
		es_info_size += 5;
		ULONG es_ca_size = 0;
		for(ULONG j = 0; j < pmt->streams[i].descriptor_count; j++)
		{
			es_ca_size += pmt->streams[i].descriptors[j].size;
		}
		if(es_ca_size > 0)
		{
			es_info_size += es_ca_size + 1;
		}
	}
	ULONG pmt_size = es_info_size + 6;
	if(prog_info_size > 0)
	{
		pmt_size += prog_info_size + 1;
	}
	ULONG total_size = pmt_size;
	if(pmt_size < 0x80)
	{
		total_size += 4;
	}
	else if(pmt_size < 0x100)
	{
		total_size += 5;
	}
	else if(pmt_size < 0x10000)
	{
		total_size += 6;
	}
	else
	{
		KdPrint((LOG_PREFIX "%s: PMT too long", __FUNCTION__));
	}
	PUCHAR ca_pmt = (PUCHAR)ExAllocatePoolWithTag(NonPagedPool, total_size, 'TMPC');
	if(ca_pmt == NULL)
	{
		KdPrint((LOG_PREFIX "%s: unable to allocate space", __FUNCTION__));
		return EN50221_IO_ERROR;
	}
	ULONG offset = EN50221_Util_PutAppTag(ca_pmt, 3, TAG_CA_PMT);
	LONG length_field_size = ASN_1_Encode(pmt_size & 0xffff, ca_pmt + offset, total_size - offset);
	if(length_field_size < 0)
	{
		KdPrint((LOG_PREFIX "%s: unable to encode ASN.1 data"));
		return EN50221_IO_ERROR;
	}
	offset += length_field_size;
	
	ca_pmt[offset++] = pmt->ca_pmt_list_management;
	ca_pmt[offset++] = (pmt->program_number >> 8) & 0xff;
	ca_pmt[offset++] = pmt->program_number & 0xff;
	ca_pmt[offset++] = ((pmt->version_number & 0x1f) << 1) | (pmt->current_next_indicator & 0x01);
	if(prog_info_size > 0)
	{
		ca_pmt[offset++] = ((prog_info_size + 1) >> 8) & 0x0f;
		ca_pmt[offset++] = (prog_info_size + 1) & 0xff;
		ca_pmt[offset++] = pmt->ca_pmt_cmd_id;
		for(ULONG i = 0; i < pmt->descriptor_count; i++)
		{
			RtlCopyMemory(ca_pmt + offset, pmt->descriptors[i].data, pmt->descriptors[i].size);
			offset += pmt->descriptors[i].size;
		}
	}
	else
	{
		ca_pmt[offset++] = 0;
		ca_pmt[offset++] = 0;
	}
	for(ULONG i = 0; i < pmt->stream_count; i++)
	{
		ca_pmt[offset++] = pmt->streams[i].type;
		ca_pmt[offset++] = (pmt->streams[i].pid >> 8) & 0x1f;
		ca_pmt[offset++] = pmt->streams[i].pid & 0xff;
		ULONG es_ca_size = 0;
		for(ULONG j = 0; j < pmt->streams[i].descriptor_count; j++)
		{
			es_ca_size += pmt->streams[i].descriptors[j].size;
		}
		if(es_ca_size > 0)
		{
			ca_pmt[offset++] = ((es_ca_size + 1) >> 8) & 0xf;
			ca_pmt[offset++] = (es_ca_size + 1) & 0xff;
			
			ca_pmt[offset++] = pmt->ca_pmt_cmd_id;
			for(ULONG j = 0; j < pmt->streams[i].descriptor_count; j++)
			{
				RtlCopyMemory(ca_pmt + offset, pmt->streams[i].descriptors[j].data, pmt->streams[i].descriptors[j].size);
				offset += pmt->streams[i].descriptors[j].size;
			}
		}
		else
		{
			ca_pmt[offset++] = 0;
			ca_pmt[offset++] = 0;
		}
	}
	*pmt_data = ca_pmt;
	KdPrint(("%s", __FUNCTION__));
	RawDump(ca_pmt, offset);
	return offset;
}

LONG EN50221_APP_CA_PMT_ListChange(PVOID ctx, UCHAR type, UCHAR cmd, PVOID inBuffer, ULONG inBufferSize, PVOID outBuffer, ULONG outBufferSize)
{
	ASSERT(ctx != NULL);
	if(inBuffer == NULL)
	{
		KdPrint((LOG_PREFIX "%s: input buffer not defined", __FUNCTION__));
		return EN50221_INVALID_ARGUMENT;
	}
	EN50221_Context * en50221 = (EN50221_Context *)ctx;
	if(type > PMT_LIST_ONLY || cmd < PMT_OK_DESCRAMBLING || cmd > PMT_QUERY)
	{
		KdPrint((LOG_PREFIX "%s: invalid argument passed", __FUNCTION__));
		return EN50221_INVALID_ARGUMENT;
	}
	CA_PMT * ca_pmt = EN50221_APP_CA_PMT_Parse((PUCHAR)inBuffer, inBufferSize);
	if(!ca_pmt)
	{
		KdPrint((LOG_PREFIX "%s: unable to parse PMT section", __FUNCTION__));
		return EN50221_GENERAL_ERROR;
	}
	ca_pmt->ca_pmt_list_management = type;
	ca_pmt->ca_pmt_cmd_id = cmd;
	ExAcquireFastMutex(&en50221->caPmtMutex);
	switch(type)
	{
		case PMT_LIST_ONLY:
		case PMT_LIST_FIRST:
			while(!IsListEmpty(&en50221->caPmtList))
			{
				LIST_ENTRY * curr = RemoveHeadList(&en50221->caPmtList);
				CA_PMT * pmt = CONTAINING_RECORD(curr, CA_PMT, link);
				EN50221_APP_CA_PMT_Free(pmt);
			}
		case PMT_LIST_MORE:
		case PMT_LIST_LAST:
			InsertHeadList(&en50221->caPmtList, &ca_pmt->link);
			break;
	}
	ExReleaseFastMutex(&en50221->caPmtMutex);
	KdPrint((LOG_PREFIX "%s: CA PMT list scheduled for update (management=%d, cmd_id=%d)", __FUNCTION__, type, cmd));

	return 0;
}

LONG EN50221_APP_CAM_Status(PVOID ctx, PVOID outBuffer, ULONG outBufferSize)
{
	ASSERT(ctx != NULL);
	if(outBuffer == NULL || outBufferSize < sizeof(NETUP_CAM_STATUS))
	{
		KdPrint((LOG_PREFIX "%s: output buffer too short", __FUNCTION__));
		return EN50221_INVALID_ARGUMENT;
	}
	NETUP_CAM_STATUS * ci_status = reinterpret_cast<NETUP_CAM_STATUS *>(outBuffer);
	RtlZeroMemory(ci_status, sizeof(*ci_status));
	if(EN50221_CI_Running(ctx) == TRUE)
	{
		ci_status->dwCamStatus = NETUP_CAM_PRESENT;
		EN50221_Context * en50221 = (EN50221_Context *)ctx;
		ci_status->wCamVendor = en50221->camVendor;
		ci_status->wCamDevice = en50221->camDevice;
		RtlCopyMemory(ci_status->cCamString, en50221->camString, sizeof(en50221->camString));
		ExAcquireFastMutex(&en50221->menuMutex);
		if(!IsListEmpty(&en50221->menuList))
		{
			ci_status->dwCamStatus |= NETUP_CAM_MMI_DATA_READY;
		}
		ExReleaseFastMutex(&en50221->menuMutex);
	}
	return sizeof(*ci_status);
}

LONG EN50221_APP_CAM_GetMenu(PVOID ctx, PVOID outBuffer, ULONG outBufferSize)
{
	ASSERT(ctx != NULL);
	if(outBuffer == NULL || outBufferSize < sizeof(NETUP_CAM_MENU))
	{
		KdPrint((LOG_PREFIX "%s: output buffer too short", __FUNCTION__));
		return EN50221_INVALID_ARGUMENT;
	}
	LONG result = 0;
	EN50221_Context * en50221 = (EN50221_Context *)ctx;
	ExAcquireFastMutex(&en50221->menuMutex);
	if(!IsListEmpty(&en50221->menuList))
	{
		LIST_ENTRY * tmp = RemoveHeadList(&en50221->menuList);
		MMI_Menu * m = CONTAINING_RECORD(tmp, MMI_Menu, link);
		NETUP_CAM_MENU * outMenu = reinterpret_cast<NETUP_CAM_MENU *>(outBuffer);
		RtlCopyMemory(outMenu, &m->menu, sizeof(*outMenu));
		ExFreePool(m);
		result = sizeof(*outMenu);
	}
	ExReleaseFastMutex(&en50221->menuMutex);
	return result;
}

LONG EN50221_APP_CAM_EnterMenu(PVOID ctx)
{
	ASSERT(ctx != NULL);
	EN50221_Context * en50221 = (EN50221_Context *)ctx;
	ExAcquireFastMutex(&en50221->menuMutex);
	en50221->menuEnter = TRUE;
	ExReleaseFastMutex(&en50221->menuMutex);
	return 0;
}

LONG EN50221_APP_CAM_AnswerMenu(PVOID ctx, UCHAR answ)
{
	KdPrint((LOG_PREFIX "%s: answ=%d", __FUNCTION__, answ));
	ASSERT(ctx != NULL);
	EN50221_Context * en50221 = (EN50221_Context *)ctx;
	ExAcquireFastMutex(&en50221->menuMutex);
	en50221->menuAnsw = answ;
	ExReleaseFastMutex(&en50221->menuMutex);
	return 0;
}

LONG EN50221_APP_CAM_CloseMenu(PVOID ctx)
{
	ASSERT(ctx != NULL);
	EN50221_Context * en50221 = (EN50221_Context *)ctx;
	ExAcquireFastMutex(&en50221->menuMutex);
	en50221->menuClose = TRUE;
	ExReleaseFastMutex(&en50221->menuMutex);
	return 0;
}