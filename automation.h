/*
 * automation.h
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

#ifndef __NETUP_AUTOMATION_H__
#define __NETUP_AUTOMATION_H__

#include "driver.h"

// For tuner node
NTSTATUS NetupTunerLowBandFrequencyGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupTunerLowBandFrequencyPut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupTunerHighBandFrequencyGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupTunerHighBandFrequencyPut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupTunerSwitchFrequencyGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupTunerSwitchFrequencyPut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);

NTSTATUS NetupTunerFrequencyGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupTunerFrequencyPut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupTunerFrequencyMultiplierGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupTunerFrequencyMultiplierPut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupTunerFrequencyRangeGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupTunerFrequencyRangePut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupTunerPolarityGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupTunerPolarityPut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupTunerSignalStrengthGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupTunerSignalPresentGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);

// For demodulator node
NTSTATUS NetupDemodSignalQualityGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupDemodSignalLockedGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupDemodModulationTypeGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupDemodModulationTypePut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupDemodInnerFecTypeGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupDemodInnerFecTypePut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupDemodInnerFecRateGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupDemodInnerFecRatePut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupDemodOuterFecTypeGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupDemodOuterFecTypePut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupDemodOuterFecRateGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupDemodOuterFecRatePut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupDemodSymbolRateGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupDemodSymbolRatePut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupDemodSpectralInversionGet(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);
NTSTATUS NetupDemodSpectralInversionPut(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PULONG pulProperty);

NTSTATUS NetupDeviceIoctl(IN PIRP pIrp,IN PKSPROPERTY pKSProperty,IN PVOID);

// For filter
NTSTATUS NetupFilterStartChanges(IN PIRP pIrp,IN PKSMETHOD pKSMethod,OPTIONAL PVOID pvIgnored);
NTSTATUS NetupFilterCheckChanges(IN PIRP pIrp,IN PKSMETHOD pKSMethod,OPTIONAL PVOID pvIgnored);
NTSTATUS NetupFilterCommitChanges(IN PIRP pIrp,IN PKSMETHOD pKSMethod,OPTIONAL PVOID pvIgnored);
NTSTATUS NetupFilterGetChangeState(IN PIRP pIrp,IN PKSMETHOD pKSMethod,OUT PULONG pulChangeState);

#endif
