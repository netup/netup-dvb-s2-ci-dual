/*
 * automation.cpp - Code to connect the node automation to the hardware
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

#include "automation.h"
#include "device.h"
#include "filter.h"
#include "stv0900.h"
#include "cx23885.h"
#include "en50221.h"
#include "lnbh24.h"
#include "netup_bda_api.h"

PKSDEVICE GETDEVICEFROMIRP(PIRP pIrp)
{
	if(pIrp == NULL)
	{
		KdPrint((LOG_PREFIX "pIrp == NULL"));
		return NULL;
	}
	PKSPIN pPin = KsGetPinFromIrp(pIrp);
	if(pPin == NULL)
	{
		KdPrint((LOG_PREFIX "pPin == NULL"));
		return NULL;
	}
	PKSDEVICE pDev = KsPinGetDevice(pPin);
	if(pDev == NULL)
	{
		KdPrint((LOG_PREFIX "pDev == NULL"));
		return NULL;
	}
	return pDev;
}

PKSFILTER GETFILTERFROMIRP(PIRP pIrp)
{
	if(pIrp == NULL)
	{
		KdPrint((LOG_PREFIX "pIrp == NULL"));
		return NULL;
	}
	PKSPIN pPin = KsGetPinFromIrp(pIrp);
	if(pPin == NULL)
	{
		KdPrint((LOG_PREFIX "pPin == NULL"));
		return NULL;
	}
	PKSFILTER pFilter = KsPinGetParentFilter(pPin);
	if(pFilter == NULL)
	{
		KdPrint((LOG_PREFIX "pFilter == NULL"));
		return NULL;
	}
	return pFilter;
}

ULONG GETCHANNEL(PIRP pIrp)
{
	PKSFILTER pFilter = GETFILTERFROMIRP(pIrp);
	ASSERT(pFilter != NULL);
	return ((FilterContext *)(pFilter->Context))->nr;
}

NTSTATUS NetupTunerLowBandFrequencyGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	ULONG Nr = GETCHANNEL(pIrp);
	*pulProperty = GETCONTEXT(GETDEVICEFROMIRP(pIrp))->tuner[Nr].lnbLowFreq;
	return result;
}

NTSTATUS NetupTunerLowBandFrequencyPut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	ULONG Nr = GETCHANNEL(pIrp);
	GETCONTEXT(GETDEVICEFROMIRP(pIrp))->tuner[Nr].lnbLowFreq = *pulProperty;
	KdPrint((LOG_PREFIX "%s: tuner #%d LNB Low freq %d", __FUNCTION__, Nr, *pulProperty));
	return result;
}

NTSTATUS NetupTunerHighBandFrequencyGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	ULONG Nr = GETCHANNEL(pIrp);
	*pulProperty = GETCONTEXT(GETDEVICEFROMIRP(pIrp))->tuner[Nr].lnbHighFreq;
	return result;
}

NTSTATUS NetupTunerHighBandFrequencyPut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	ULONG Nr = GETCHANNEL(pIrp);
	GETCONTEXT(GETDEVICEFROMIRP(pIrp))->tuner[Nr].lnbHighFreq = *pulProperty;
	KdPrint((LOG_PREFIX "%s: tuner #%d LNB High freq %d", __FUNCTION__, Nr, *pulProperty));
	return result;
}

NTSTATUS NetupTunerSwitchFrequencyGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	ULONG Nr = GETCHANNEL(pIrp);
	*pulProperty = GETCONTEXT(GETDEVICEFROMIRP(pIrp))->tuner[Nr].switchFreq;
	return result;
}

NTSTATUS NetupTunerSwitchFrequencyPut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	ULONG Nr = GETCHANNEL(pIrp);
	GETCONTEXT(GETDEVICEFROMIRP(pIrp))->tuner[Nr].switchFreq = *pulProperty;
	KdPrint((LOG_PREFIX "%s: tuner #%d LNB Switch freq %d", __FUNCTION__, Nr, *pulProperty));
	return result;
}

NTSTATUS NetupTunerFrequencyGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	ULONG Nr = GETCHANNEL(pIrp);
	*pulProperty = GETCONTEXT(GETDEVICEFROMIRP(pIrp))->tuner[Nr].freq;
	KdPrint((LOG_PREFIX "%s: tuner #%d frequency %d", __FUNCTION__, Nr, *pulProperty));
	return result;
}

NTSTATUS NetupTunerFrequencyPut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	ULONG Nr = GETCHANNEL(pIrp);
	GETCONTEXT(GETDEVICEFROMIRP(pIrp))->tuner[Nr].freq = *pulProperty;
	KdPrint((LOG_PREFIX "%s: tuner #%d frequency %d", __FUNCTION__, Nr, *pulProperty));
	return result;
}

NTSTATUS NetupTunerFrequencyMultiplierGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	ULONG Nr = GETCHANNEL(pIrp);
	*pulProperty = GETCONTEXT(GETDEVICEFROMIRP(pIrp))->tuner[Nr].freqMult;
	KdPrint((LOG_PREFIX "%s: tuner #%d freqMult %d", __FUNCTION__, Nr, *pulProperty));
	return result;
}

NTSTATUS NetupTunerFrequencyMultiplierPut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	ULONG Nr = GETCHANNEL(pIrp);
	GETCONTEXT(GETDEVICEFROMIRP(pIrp))->tuner[Nr].freqMult = *pulProperty;
	KdPrint((LOG_PREFIX "%s: tuner #%d freqMult %d", __FUNCTION__, Nr, *pulProperty));
	return result;
}

NTSTATUS NetupTunerFrequencyRangeGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	*pulProperty = BDA_RANGE_NOT_SET;
	return result;
}

NTSTATUS NetupTunerFrequencyRangePut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	ULONG Nr = GETCHANNEL(pIrp);
	KdPrint((LOG_PREFIX "%s: tuner #d range %d", __FUNCTION__, Nr, *pulProperty));
	return result;
}

NTSTATUS NetupTunerPolarityGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	ULONG Nr = GETCHANNEL(pIrp);
	ULONG polarisation = GETCONTEXT(GETDEVICEFROMIRP(pIrp))->tuner[Nr].polarisation;
	if(polarisation == 0)
	{
		*pulProperty = BDA_POLARISATION_NOT_SET;
	}
	else
	{
		*pulProperty = polarisation;
	}
	return result;
}

NTSTATUS NetupTunerPolarityPut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	ULONG Nr = GETCHANNEL(pIrp);
	GETCONTEXT(GETDEVICEFROMIRP(pIrp))->tuner[Nr].polarisation = *pulProperty;
	KdPrint((LOG_PREFIX "%s: tuner #%d Polarisation %d", __FUNCTION__, Nr, *pulProperty));
	return result;
}

NTSTATUS NetupTunerSignalStrengthGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	ULONG Nr = GETCHANNEL(pIrp);
	*pulProperty = STV0900_SignalStrengthGet(GETDEVICEFROMIRP(pIrp), Nr);
	return result;
}

NTSTATUS NetupTunerSignalPresentGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	*pulProperty = 1;
	return result;
}

/* Demodulator automation methods */

NTSTATUS NetupDemodSignalQualityGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	ULONG Nr = GETCHANNEL(pIrp);
	*pulProperty = STV0900_SignalQualityGet(GETDEVICEFROMIRP(pIrp), Nr);
	return result;
}

NTSTATUS NetupDemodSignalLockedGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	ULONG Nr = GETCHANNEL(pIrp);
	*pulProperty = STV0900_HasLocked(GETDEVICEFROMIRP(pIrp), Nr);
	return result;
}

NTSTATUS NetupDemodModulationTypeGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	ULONG Nr = GETCHANNEL(pIrp);
	*pulProperty = GETCONTEXT(GETDEVICEFROMIRP(pIrp))->signal[Nr].modulation;
	return result;
}

NTSTATUS NetupDemodModulationTypePut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	return result;
}

NTSTATUS NetupDemodInnerFecTypeGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	*pulProperty = BDA_FEC_VITERBI;
	return result;
}

NTSTATUS NetupDemodInnerFecTypePut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	if(*pulProperty != BDA_FEC_VITERBI)
	{
		result = STATUS_INVALID_PARAMETER;
	}
	return result;
}

NTSTATUS NetupDemodInnerFecRateGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	ULONG Nr = GETCHANNEL(pIrp);
	*pulProperty = GETCONTEXT(GETDEVICEFROMIRP(pIrp))->signal[Nr].fec;
	return result;
}

NTSTATUS NetupDemodInnerFecRatePut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	return result;
}

NTSTATUS NetupDemodOuterFecTypeGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	*pulProperty = BDA_FEC_VITERBI;
	return result;
}

NTSTATUS NetupDemodOuterFecTypePut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	if(*pulProperty != BDA_FEC_VITERBI)
	{
		result = STATUS_INVALID_PARAMETER;
	}
	return result;
}

NTSTATUS NetupDemodOuterFecRateGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	ULONG Nr = GETCHANNEL(pIrp);
	*pulProperty = GETCONTEXT(GETDEVICEFROMIRP(pIrp))->signal[Nr].fec;
	return result;
}

NTSTATUS NetupDemodOuterFecRatePut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	return result;
}

NTSTATUS NetupDemodSymbolRateGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	ULONG Nr = GETCHANNEL(pIrp);
	*pulProperty = GETCONTEXT(GETDEVICEFROMIRP(pIrp))->tuner[Nr].symbolRate;
	KdPrint((LOG_PREFIX "tuner #%d symbolRate %d", Nr, *pulProperty));
	return result;
}

NTSTATUS NetupDemodSymbolRatePut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	ULONG Nr = GETCHANNEL(pIrp);
	GETCONTEXT(GETDEVICEFROMIRP(pIrp))->tuner[Nr].symbolRate = *pulProperty;
	KdPrint((LOG_PREFIX "tuner #%d symbolRate %d", Nr, *pulProperty));
	return result;
}

NTSTATUS NetupDemodSpectralInversionGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	ULONG Nr = GETCHANNEL(pIrp);
	*pulProperty = GETCONTEXT(GETDEVICEFROMIRP(pIrp))->signal[Nr].spectrum;
	return result;
}

NTSTATUS NetupDemodSpectralInversionPut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	return result;
}

/* Filter automation methods */

NTSTATUS NetupFilterStartChanges(IN PIRP pIrp,IN PKSMETHOD pKSMethod,OPTIONAL PVOID pvIgnored)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	result=BdaStartChanges(pIrp);
	if(NT_SUCCESS(result)) {
		// Reset mirror settings
	}
	return result;
}

NTSTATUS NetupFilterCheckChanges(IN PIRP pIrp,IN PKSMETHOD pKSMethod,OPTIONAL PVOID pvIgnored)
{
	NTSTATUS result = STATUS_SUCCESS;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	result=BdaCheckChanges(pIrp);
	if(NT_SUCCESS(result)) {
		// Validate alterations
	}
	return result;
}

NTSTATUS NetupFilterCommitChanges(IN PIRP pIrp,IN PKSMETHOD pKSMethod,OPTIONAL PVOID pvIgnored)
{
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
	// Validate alterations
	// Commit alterations
	PKSFILTER pFilter = KsGetFilterFromIrp(pIrp);
	ASSERT(pFilter != NULL);
	ASSERT(pFilter->Context != NULL);
	FilterContext * pContext = (FilterContext *)(pFilter->Context);
	if(GETCONTEXT(pContext->device)->tuner[pContext->nr].freq > 0 && 
	   GETCONTEXT(pContext->device)->tuner[pContext->nr].symbolRate > 0)
	{
		// tune frequence
		ULONG freq = GETCONTEXT(pContext->device)->tuner[pContext->nr].freq;
		// LNB switch frequency
		ULONG switchFreq = GETCONTEXT(pContext->device)->tuner[pContext->nr].switchFreq;
		// oscillator frequency
		ULONG oscFreq = 0;
		BOOLEAN toneEnable = FALSE;
		if(switchFreq > 0)
		{
			oscFreq = (freq < switchFreq) ?
				(GETCONTEXT(pContext->device)->tuner[pContext->nr].lnbLowFreq) : (GETCONTEXT(pContext->device)->tuner[pContext->nr].lnbHighFreq);
			toneEnable = (freq > switchFreq);
		}
		else
		{
			oscFreq	= (GETCONTEXT(pContext->device)->tuner[pContext->nr].lnbLowFreq);
		}
		ULONG polarisation = GETCONTEXT(pContext->device)->tuner[pContext->nr].polarisation;
		BOOLEAN highVoltageEnable = (polarisation == BDA_POLARISATION_LINEAR_H || polarisation == BDA_POLARISATION_CIRCULAR_L);
		if(oscFreq != 0 && freq > oscFreq)
		{
			KdPrint((LOG_PREFIX "%s: Tune Freq %d Oscillator Freq %d Tone %d High voltage %d", __FUNCTION__, freq, oscFreq, toneEnable, highVoltageEnable));
			LNBH24_SetVoltage(pContext->device, pContext->nr, highVoltageEnable);
			STV0900_SetTone(pContext->device, pContext->nr, toneEnable);
			STV0900_LockSignal(
				pContext->device,
				pContext->nr,
				freq - oscFreq,
				GETCONTEXT(pContext->device)->tuner[pContext->nr].symbolRate
			);
			STV0900_GetSignalInfo(
				pContext->device,
				pContext->nr,
				GETCONTEXT(pContext->device)->signal[pContext->nr]
			);
		}
		else
		{
			KdPrint((LOG_PREFIX "%s: Tune Freq %d LNB Switch Freq %d Oscillator Freq %d, unable to lock", 
				__FUNCTION__, freq, switchFreq, oscFreq));
		}

	}

	return BdaCommitChanges(pIrp);
}

NTSTATUS NetupFilterGetChangeState(IN PIRP pIrp,IN PKSMETHOD pKSMethod,OUT PULONG pulChangeState)
{
	NTSTATUS result = STATUS_SUCCESS;
	BDA_CHANGE_STATE topologyChangeState;
	KdPrint((LOG_PREFIX "%s()", __FUNCTION__));
    result=BdaGetChangeState(pIrp,&topologyChangeState);
    if(NT_SUCCESS(result)) {
        if(topologyChangeState==BDA_CHANGES_PENDING) {
            *pulChangeState=BDA_CHANGES_PENDING;
        } else {
            *pulChangeState=BDA_CHANGES_COMPLETE;
        }
    }
	return result;
}

static BOOLEAN CopyFromUser(PVOID DstAddress, PVOID SrcVirtualAddress, ULONG Size)
{
	ASSERT(DstAddress != NULL && SrcVirtualAddress != NULL && Size > 0);
	PMDL mdl = IoAllocateMdl(SrcVirtualAddress, Size, FALSE, FALSE, NULL);
	if(!mdl)
	{
		KdPrint((LOG_PREFIX "%s: unable to allocate MDL", __FUNCTION__));
		return FALSE;
	}
	__try
	{
		MmProbeAndLockPages(mdl, KernelMode, IoReadAccess);
	} __except (EXCEPTION_EXECUTE_HANDLER) {
		IoFreeMdl(mdl);
		KdPrint((LOG_PREFIX "%s: unable to map MDL buffer", __FUNCTION__));
		return FALSE;
	}
	PVOID SrcAddress = MmGetSystemAddressForMdlSafe(mdl, NormalPagePriority);
	if(!SrcAddress)
	{
		MmUnlockPages(mdl);
		IoFreeMdl(mdl);
		KdPrint((LOG_PREFIX "%s: unable to get result buffer address", __FUNCTION__));
		return FALSE;
	}
	RtlCopyMemory(DstAddress, SrcAddress, Size);
	MmUnlockPages(mdl);
	IoFreeMdl(mdl);
	return TRUE;
}

static BOOLEAN CopyToUser(PVOID DstVirtualAddress, PVOID SrcAddress, ULONG Size)
{
	ASSERT(DstVirtualAddress != NULL && SrcAddress != NULL && Size > 0);
	PMDL mdl = IoAllocateMdl(DstVirtualAddress, Size, FALSE, FALSE, NULL);
	if(!mdl)
	{
		KdPrint((LOG_PREFIX "%s: unable to allocate MDL", __FUNCTION__));
		return FALSE;
	}
	__try
	{
		MmProbeAndLockPages(mdl, KernelMode, IoWriteAccess);
	} __except (EXCEPTION_EXECUTE_HANDLER) {
		IoFreeMdl(mdl);
		KdPrint((LOG_PREFIX "%s: unable to map MDL buffer", __FUNCTION__));
		return FALSE;
	}
	PVOID DstAddress = MmGetSystemAddressForMdlSafe(mdl, NormalPagePriority);
	if(!DstAddress)
	{
		MmUnlockPages(mdl);
		IoFreeMdl(mdl);
		KdPrint((LOG_PREFIX "%s: unable to get result buffer address", __FUNCTION__));
		return FALSE;
	}
	RtlCopyMemory(DstAddress, SrcAddress, Size);
	MmUnlockPages(mdl);
	IoFreeMdl(mdl);
	return TRUE;
}

NTSTATUS NetupDeviceIoctl(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PVOID data)
{
	NETUP_BDA_EXT_CMD * ioctl = (NETUP_BDA_EXT_CMD *)data;
	ULONG channel = GETCHANNEL(pIrp);
	UCHAR cmd = (ioctl->dwCmd >> 16) & 0xff;
	UCHAR arg1 = (ioctl->dwCmd >> 8) & 0xff;
	UCHAR arg2 = ioctl->dwCmd & 0xff;
	KdPrint((LOG_PREFIX "%s() Channel #%d Cmd 0x%02x Arg1 0x%02x Arg2 0x%02x",
		__FUNCTION__, channel, cmd, arg1, arg2));
	PVOID lpInputBuffer = NULL;
	PVOID lpOutputBuffer = NULL;
	if(ioctl->dwInputBufferLength)
	{
		if(ioctl->dwInputBufferLength > NETUP_IOCTL_MAX_BUFFER_SIZE)
		{
			KdPrint((LOG_PREFIX "%s: lpInputBuffer too big", __FUNCTION__));
			goto ioctl_error;
		}
		if(!ioctl->lpInputBuffer)
		{
			KdPrint((LOG_PREFIX "%s: lpInputBuffer is NULL", __FUNCTION__));
			goto ioctl_error;
		}
		lpInputBuffer = ExAllocatePoolWithTag(NonPagedPool, ioctl->dwInputBufferLength, 'NIPL');
		if(!lpInputBuffer)
		{
			KdPrint((LOG_PREFIX "%s: unable to allocate lpInputBuffer (%s bytes)", __FUNCTION__, ioctl->dwInputBufferLength));
			goto ioctl_error;
		}
		CopyFromUser(lpInputBuffer, ioctl->lpInputBuffer, ioctl->dwInputBufferLength);
	}
	if(ioctl->dwOutputBufferLength)
	{
		if(ioctl->dwOutputBufferLength > NETUP_IOCTL_MAX_BUFFER_SIZE)
		{
			KdPrint((LOG_PREFIX "%s: lpOutputBuffer too big", __FUNCTION__));
			goto ioctl_error;
		}
		if(!ioctl->lpOutputBuffer)
		{
			KdPrint((LOG_PREFIX "%s: lpOutputBuffer is NULL", __FUNCTION__));
			goto ioctl_error;
		}
		lpOutputBuffer = ExAllocatePoolWithTag(NonPagedPool, ioctl->dwOutputBufferLength, 'UOPL');
		if(!lpOutputBuffer)
		{
			KdPrint((LOG_PREFIX "%s: unable to allocate lpOutputBuffer (%s bytes)", __FUNCTION__, ioctl->dwOutputBufferLength));
			goto ioctl_error;
		}
		RtlZeroMemory(lpOutputBuffer, ioctl->dwOutputBufferLength);
	}
	LONG result = 0;
	switch(cmd)
	{
		case NETUP_IOCTL_CI_STATUS:
		{
			result = EN50221_APP_CAM_Status(
				GETCONTEXT(GETDEVICEFROMIRP(pIrp))->en50221_context[channel],
				lpOutputBuffer,
				ioctl->dwOutputBufferLength);
			break;
		}
		case NETUP_IOCTL_CI_MMI_ENTER_MENU:
		{
			result = EN50221_APP_CAM_EnterMenu(GETCONTEXT(GETDEVICEFROMIRP(pIrp))->en50221_context[channel]);
			break;
		}
		case NETUP_IOCTL_CI_MMI_GET_MENU:
		{
			result = EN50221_APP_CAM_GetMenu(
				GETCONTEXT(GETDEVICEFROMIRP(pIrp))->en50221_context[channel],
				lpOutputBuffer,
				ioctl->dwOutputBufferLength);
			break;
		}
		case NETUP_IOCTL_CI_MMI_ANSWER_MENU:
		{
			result = EN50221_APP_CAM_AnswerMenu(GETCONTEXT(GETDEVICEFROMIRP(pIrp))->en50221_context[channel], arg1);
			break;
		}
		case NETUP_IOCTL_CI_MMI_CLOSE:
		{
			result = EN50221_APP_CAM_CloseMenu(GETCONTEXT(GETDEVICEFROMIRP(pIrp))->en50221_context[channel]);
			break;
		}
		case NETUP_IOCTL_CI_PMT_LIST_CHANGE:
		{
			result = EN50221_APP_CA_PMT_ListChange(
				GETCONTEXT(GETDEVICEFROMIRP(pIrp))->en50221_context[channel],
				arg1,
				arg2,
				lpInputBuffer,
				ioctl->dwInputBufferLength,
				lpOutputBuffer,
				ioctl->dwOutputBufferLength);
			break;
		}
		case NETUP_IOCTL_DISEQC_WRITE:
		{
			result = STV0900_DiSEqC_Write(GETDEVICEFROMIRP(pIrp), channel, (PUCHAR)lpInputBuffer, ioctl->dwInputBufferLength);
			break;
		}
		default:
		{
			KdPrint((LOG_PREFIX "%s: unsupported Cmd 0x%02x", __FUNCTION__, cmd));
			goto ioctl_error;
		}
	}
	if(result < 0)
	{
		KdPrint((LOG_PREFIX "%s: failed", __FUNCTION__));
		goto ioctl_error;
	}
	if(lpOutputBuffer != NULL && result > 0)
	{
		KdPrint((LOG_PREFIX "%s: writing output data (%d bytes)", __FUNCTION__, result));
		CopyToUser(ioctl->lpOutputBuffer, lpOutputBuffer, result);
		ExFreePool(lpOutputBuffer);
	}
	if(ioctl->lpOutputLength != NULL)
	{
		CopyToUser(ioctl->lpOutputLength, &result, sizeof(result));
	}
	if(lpInputBuffer != NULL)
	{
		ExFreePool(lpInputBuffer);
	}

	return STATUS_SUCCESS;

ioctl_error:
	if(lpInputBuffer)
	{
		ExFreePool(lpInputBuffer);
	}
	if(lpOutputBuffer)
	{
		ExFreePool(lpOutputBuffer);
	}
	return STATUS_INTERNAL_ERROR;
}
