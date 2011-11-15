/*
 * filter.cpp - BDA filter code
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

#include "filter.h"
#include "device.h"
#include "automation.h"
#include "dispatch.h"
#include "netup_bda_api.h"

extern const BDA_FILTER_TEMPLATE NetupFilter0Template;
extern const BDA_FILTER_TEMPLATE NetupFilter1Template;

NTSTATUS AntennaIntersectDataFormat(IN PVOID pContext,IN PIRP pIrp,IN PKSP_PIN Pin,IN PKSDATARANGE DataRange,IN PKSDATARANGE MatchingDataRange,IN ULONG DataBufferSize,OUT PVOID Data OPTIONAL,OUT PULONG DataSize)
{
    *DataSize=sizeof(KS_DATARANGE_BDA_ANTENNA);
    if(DataBufferSize<sizeof(KS_DATARANGE_BDA_ANTENNA)) return STATUS_BUFFER_OVERFLOW;
    else {
        ASSERT(DataBufferSize==sizeof(KS_DATARANGE_BDA_ANTENNA));
        RtlCopyMemory(Data,(PVOID)&DataRange,sizeof(KS_DATARANGE_BDA_ANTENNA));
        return STATUS_SUCCESS;
    }
}

NTSTATUS OutputIntersectDataFormat(IN PVOID pContext,IN PIRP pIrp,IN PKSP_PIN Pin,IN PKSDATARANGE DataRange,IN PKSDATARANGE MatchingDataRange,IN ULONG DataBufferSize,OUT PVOID Data OPTIONAL,OUT PULONG DataSize)
{
    *DataSize=sizeof(KS_DATARANGE_BDA_TRANSPORT);
    if(DataBufferSize<sizeof(KS_DATARANGE_BDA_TRANSPORT)) return STATUS_BUFFER_OVERFLOW;
    else {
        ASSERT(DataBufferSize==sizeof(KS_DATARANGE_BDA_TRANSPORT));
        RtlCopyMemory(Data,(PVOID)DataRange,sizeof(KS_DATARANGE_BDA_TRANSPORT));
        return STATUS_SUCCESS;
    }
}

NTSTATUS FilterCreate(IN PKSFILTER Filter,IN PIRP Irp, ULONG Nr, const BDA_FILTER_TEMPLATE * FilterTemplate)
{
	ASSERT(Nr < 2);
	NTSTATUS result;
	KdPrint((LOG_PREFIX "FilterCreate #%d", Nr));
	Filter->Context=ExAllocatePoolWithTag(NonPagedPool, sizeof(FilterContext), 'RTLF');
	if(Filter->Context == NULL)
	{
		KdPrint((LOG_PREFIX "unable to allocate FilterContext"));
		return STATUS_INSUFFICIENT_RESOURCES;
	}
	FilterContext * filterContext = (FilterContext *)(Filter->Context);
	filterContext->device = KsFilterGetDevice(Filter);
	if(filterContext->device == NULL)
	{
		KdPrint((LOG_PREFIX "device not connected"));
		return STATUS_DEVICE_NOT_CONNECTED;
	}
#if 0
	if(GETCONTEXT(filterContext->device)->filter[Nr] != NULL)
	{
		KdPrint((LOG_PREFIX "filter #%d already exist", Nr));
		ExFreePool(filterContext);
		return STATUS_DEVICE_NOT_CONNECTED;
	}
#endif
	filterContext->nr = Nr;
#if 0
	GETCONTEXT(filterContext->device)->filter[Nr]=Filter;
#endif
	// Prepare for action
	result=BdaInitFilter(Filter,FilterTemplate);
	if(result!=STATUS_SUCCESS) {
		KdPrint((LOG_PREFIX "failed to initialise filter: %d",result));
		return result;
	}
	KdPrint((LOG_PREFIX "FilterCreate #%d done", Nr));
	return STATUS_SUCCESS;
}

NTSTATUS Filter0Create(IN PKSFILTER Filter,IN PIRP Irp)
{
	return FilterCreate(Filter, Irp, 0, &NetupFilter0Template);
}

NTSTATUS Filter1Create(IN PKSFILTER Filter,IN PIRP Irp)
{
	return FilterCreate(Filter, Irp, 1, &NetupFilter1Template);
}

NTSTATUS FilterClose(IN PKSFILTER Filter,IN PIRP Irp)
{
	FilterContext * filterContext = (FilterContext *)(Filter->Context);
	KdPrint((LOG_PREFIX "FilterClose #%d", filterContext->nr));
#if 0
	if(GETCONTEXT(filterContext->device)->filter[filterContext->nr] == NULL)
	{
		KdPrint((LOG_PREFIX "filter already closed"));
		return STATUS_DEVICE_NOT_CONNECTED;
	}
#endif
	BdaUninitFilter(Filter);
#if 0
	GETCONTEXT(filterContext->device)->filter[filterContext->nr] = NULL;
#endif
	ExFreePool(filterContext);
	// Shut down anything still going on
	return STATUS_SUCCESS;
}

/* Filter dispatch table */
const KSFILTER_DISPATCH NetupFilter0Dispatch = 
{
	/* Create */	Filter0Create,
	/* Close */		FilterClose,
	/* Process */	NULL,
	/* Reset */		NULL
};

/* Filter dispatch table */
const KSFILTER_DISPATCH NetupFilter1Dispatch = 
{
	/* Create */	Filter1Create,
	/* Close */		FilterClose,
	/* Process */	NULL,
	/* Reset */		NULL
};

/* Antenna PIN dispatch table */
const KSPIN_DISPATCH NetupAntennaDispatch={
	/* Create		  */	NetupAntennaCreate,
	/* Close		  */	NetupAntennaClose,
	/* Process		  */	NULL,
	/* Reset		  */	NULL,
	/* SetDataFormat  */	NULL,
	/* SetDeviceState */	NetupOutputSetDeviceState,	// This is meant to say output
	/* Connect		  */	NULL,
	/* Disconnect	  */	NULL,
	/* Allocator	  */	NULL
};

/* Output PIN dispatch table */
const KSPIN_DISPATCH NetupOutputDispatch={
	/* Create		  */	NetupOutputCreate,
	/* Close		  */	NetupOutputClose,
	/* Process		  */	NetupOutputProcess,
	/* Reset		  */	NULL,
	/* SetDataFormat  */	NULL,
	/* SetDeviceState */	NetupAntennaSetDeviceState,	// This is meant to say antenna
	/* Connect		  */	NULL,
	/* Disconnect	  */	NULL,
	/* Allocator	  */	NULL
};

/* Tuner node automation table */

DEFINE_KSPROPERTY_TABLE(NetupTunerFrequencyProperties)
{
    DEFINE_KSPROPERTY_ITEM_BDA_RF_TUNER_FREQUENCY(
        NetupTunerFrequencyGet,
        NetupTunerFrequencyPut
        ),
    DEFINE_KSPROPERTY_ITEM_BDA_RF_TUNER_FREQUENCY_MULTIPLIER(
        NetupTunerFrequencyMultiplierGet,
		NetupTunerFrequencyMultiplierPut
        ),

    DEFINE_KSPROPERTY_ITEM_BDA_RF_TUNER_POLARITY(
        NetupTunerPolarityGet,
		NetupTunerPolarityPut
        ),
    DEFINE_KSPROPERTY_ITEM_BDA_RF_TUNER_RANGE(
        NetupTunerFrequencyRangeGet,
		NetupTunerFrequencyRangePut
        ),

};

DEFINE_KSPROPERTY_TABLE(NetupTunerLNBProperties)
{
    DEFINE_KSPROPERTY_ITEM_BDA_LNB_LOF_LOW_BAND(
        NetupTunerLowBandFrequencyGet,
        NetupTunerLowBandFrequencyPut
        ),
	DEFINE_KSPROPERTY_ITEM_BDA_LNB_LOF_HIGH_BAND(
        NetupTunerHighBandFrequencyGet,
        NetupTunerHighBandFrequencyPut
        ),
	DEFINE_KSPROPERTY_ITEM_BDA_LNB_SWITCH_FREQUENCY(
        NetupTunerSwitchFrequencyGet,
        NetupTunerSwitchFrequencyPut
        ),
};


DEFINE_KSPROPERTY_TABLE(NetupTunerSignalStatsProperties)
{
    DEFINE_KSPROPERTY_ITEM_BDA_SIGNAL_STRENGTH(
        NetupTunerSignalStrengthGet, NULL
        ),
    DEFINE_KSPROPERTY_ITEM_BDA_SIGNAL_PRESENT(
        NetupTunerSignalPresentGet, NULL
        ),
	DEFINE_KSPROPERTY_ITEM_BDA_SIGNAL_QUALITY(
        NetupDemodSignalQualityGet, NULL
        ),
    DEFINE_KSPROPERTY_ITEM_BDA_SIGNAL_LOCKED(
        NetupDemodSignalLockedGet, NULL
        ),
};


DEFINE_KSPROPERTY_SET_TABLE(NetupTunerAutomationProperties)
{
    DEFINE_KSPROPERTY_SET
    (
        &KSPROPSETID_BdaFrequencyFilter,
        SIZEOF_ARRAY(NetupTunerFrequencyProperties),
        NetupTunerFrequencyProperties,
        0,
        NULL
    ),
    DEFINE_KSPROPERTY_SET
    (
        &KSPROPSETID_BdaSignalStats,
        SIZEOF_ARRAY(NetupTunerSignalStatsProperties),
        NetupTunerSignalStatsProperties,
        0,
        NULL
    ),
	DEFINE_KSPROPERTY_SET
    (
        &KSPROPSETID_BdaLNBInfo,
        SIZEOF_ARRAY(NetupTunerLNBProperties),
        NetupTunerLNBProperties,
        0,
        NULL
    )
};


/* Tuner automation table */
DEFINE_KSAUTOMATION_TABLE(NetupTunerAutomation) {
    DEFINE_KSAUTOMATION_PROPERTIES(NetupTunerAutomationProperties),
    DEFINE_KSAUTOMATION_METHODS_NULL,
    DEFINE_KSAUTOMATION_EVENTS_NULL
};


/* For demodulator node */
DEFINE_KSPROPERTY_TABLE(NetupDemodulatorSignalStats)
{
    DEFINE_KSPROPERTY_ITEM_BDA_SIGNAL_QUALITY(
        NetupDemodSignalQualityGet, NULL
        ),
    DEFINE_KSPROPERTY_ITEM_BDA_SIGNAL_LOCKED(
        NetupDemodSignalLockedGet, NULL
        ),
};
DEFINE_KSPROPERTY_TABLE(NetupDemodulatorProps)
{
    DEFINE_KSPROPERTY_ITEM_BDA_MODULATION_TYPE(
        NetupDemodModulationTypeGet,
        NetupDemodModulationTypePut
        ),
    DEFINE_KSPROPERTY_ITEM_BDA_INNER_FEC_TYPE(
        NetupDemodInnerFecTypeGet,
        NetupDemodInnerFecTypePut
        ),
    DEFINE_KSPROPERTY_ITEM_BDA_INNER_FEC_RATE(
        NetupDemodInnerFecRateGet,
        NetupDemodInnerFecRatePut
        ),
    DEFINE_KSPROPERTY_ITEM_BDA_OUTER_FEC_TYPE(
        NetupDemodOuterFecTypeGet,
        NetupDemodOuterFecTypePut
        ),
    DEFINE_KSPROPERTY_ITEM_BDA_OUTER_FEC_RATE(
        NetupDemodOuterFecRateGet,
        NetupDemodOuterFecRatePut
        ),
    DEFINE_KSPROPERTY_ITEM_BDA_SYMBOL_RATE(
        NetupDemodSymbolRateGet,
        NetupDemodSymbolRatePut
        ),
	DEFINE_KSPROPERTY_ITEM_BDA_SPECTRAL_INVERSION(
		NetupDemodSpectralInversionGet,
		NetupDemodSpectralInversionPut
		),
};
DEFINE_KSPROPERTY_TABLE(NetupBdaExtensionProps)
{
		DEFINE_KSPROPERTY_ITEM_BDA_NETUP_IOCTL(
			NetupDeviceIoctl
		),
};
DEFINE_KSPROPERTY_SET_TABLE(NetupDemodulatorProperties)
{
    DEFINE_KSPROPERTY_SET
    (
        &KSPROPSETID_BdaDigitalDemodulator,                // Set
        SIZEOF_ARRAY(NetupDemodulatorProps),   // PropertiesCount
        NetupDemodulatorProps,                 // PropertyItems
        0,                                          // FastIoCount
        NULL                                        // FastIoTable
    ),
    DEFINE_KSPROPERTY_SET
    (
        &KSPROPSETID_BdaSignalStats,                // Set
        SIZEOF_ARRAY(NetupDemodulatorSignalStats),   // PropertiesCount
        NetupDemodulatorSignalStats,                 // PropertyItems
        0,                                          // FastIoCount
        NULL                                        // FastIoTable
    ),
	DEFINE_KSPROPERTY_SET
    (
        &KSPROPSETID_NetupExtProperties,            // Set
        SIZEOF_ARRAY(NetupBdaExtensionProps),       // PropertiesCount
        NetupBdaExtensionProps,                     // PropertyItems
        0,                                          // FastIoCount
        NULL                                        // FastIoTable
    ),
};
/* Demodulator automation table */
DEFINE_KSAUTOMATION_TABLE(NetupDemodulatorAutomation) {
    DEFINE_KSAUTOMATION_PROPERTIES(NetupDemodulatorProperties),
    DEFINE_KSAUTOMATION_METHODS_NULL,
    DEFINE_KSAUTOMATION_EVENTS_NULL
};

/* Filter automation table */

DEFINE_KSMETHOD_TABLE(NetupFilterConfiguration) {
	DEFINE_KSMETHOD_ITEM_BDA_CREATE_TOPOLOGY(BdaMethodCreateTopology,NULL),
};

DEFINE_KSMETHOD_TABLE(NetupFilterChangeSync) {
    DEFINE_KSMETHOD_ITEM_BDA_START_CHANGES(NetupFilterStartChanges,NULL),
    DEFINE_KSMETHOD_ITEM_BDA_CHECK_CHANGES(NetupFilterCheckChanges,NULL),
    DEFINE_KSMETHOD_ITEM_BDA_COMMIT_CHANGES(NetupFilterCommitChanges,NULL),
    DEFINE_KSMETHOD_ITEM_BDA_GET_CHANGE_STATE(NetupFilterGetChangeState,NULL),
};

DEFINE_KSMETHOD_SET_TABLE(NetupFilterMethods) {
	DEFINE_KSMETHOD_SET
	(
		&KSMETHODSETID_BdaDeviceConfiguration,
		SIZEOF_ARRAY(NetupFilterConfiguration),
		NetupFilterConfiguration,
		0,NULL
	),
	DEFINE_KSMETHOD_SET
	(
		&KSMETHODSETID_BdaChangeSync,
		SIZEOF_ARRAY(NetupFilterChangeSync),
		NetupFilterChangeSync,
		0,NULL
	),
};

DEFINE_KSAUTOMATION_TABLE(NetupFilterAutomation) {
    DEFINE_KSAUTOMATION_PROPERTIES_NULL,
    DEFINE_KSAUTOMATION_METHODS(NetupFilterMethods),
    DEFINE_KSAUTOMATION_EVENTS_NULL
};

/* NULL automation table for PINs */
DEFINE_KSAUTOMATION_TABLE(NullAutomation) {
    DEFINE_KSAUTOMATION_PROPERTIES_NULL,
    DEFINE_KSAUTOMATION_METHODS_NULL,
    DEFINE_KSAUTOMATION_EVENTS_NULL
};

/* range of data formats for a BDA antenna stream */
const KS_DATARANGE_BDA_ANTENNA NetupAntennaIn={
	// KSDATARANGE
	{
		sizeof(KS_DATARANGE_BDA_ANTENNA),
		0,
		0,
		0,
		STATICGUIDOF(KSDATAFORMAT_TYPE_BDA_ANTENNA),
		STATICGUIDOF(KSDATAFORMAT_SUBTYPE_NONE),
		STATICGUIDOF(KSDATAFORMAT_SPECIFIER_NONE),
	}
};

/* a range of data formats for a BDA output stream */
const KS_DATARANGE_BDA_TRANSPORT NetupStreamOut={
	// KSDATARANGE
	{
		sizeof(KS_DATARANGE_BDA_TRANSPORT),
		0,
		0,
		0,
		STATICGUIDOF(KSDATAFORMAT_TYPE_STREAM),
		STATICGUIDOF(KSDATAFORMAT_SUBTYPE_BDA_MPEG2_TRANSPORT),
		STATICGUIDOF(KSDATAFORMAT_SPECIFIER_BDA_TRANSPORT),
	},
    //  BDA_TRANSPORT_INFO
    {
        TRANSPORT_PACKET_SIZE,							 //  ulcbPhyiscalPacket
        TRANSPORT_PACKET_COUNT*TRANSPORT_PACKET_SIZE,    //  ulcbPhysicalFrame
        0,										         //  ulcbPhysicalFrameAlignment (no requirement)
        0											     //  AvgTimePerFrame (not known)
    }
};

const PKSDATARANGE NetupAntennaFormat[]={
	(PKSDATARANGE)&NetupAntennaIn,
};

const PKSDATARANGE NetupOutputFormat[]={
	(PKSDATARANGE)&NetupStreamOut,
};

/* output PIN interface */
const KSPIN_INTERFACE NetupStreamInterface[]={
   {
      STATICGUIDOF(KSINTERFACESETID_Standard),
      KSINTERFACE_STANDARD_STREAMING,
      0
   },
};

/* framing */
DECLARE_SIMPLE_FRAMING_EX(NetupOutputAllocator,
						  STATICGUIDOF(KSMEMORY_TYPE_KERNEL_NONPAGED),
						  KSALLOCATOR_REQUIREMENTF_SYSTEM_MEMORY/*|KSALLOCATOR_REQUIREMENTF_PREFERENCES_ONLY*/,
						  64,
						  0,
						  TRANSPORT_PACKET_COUNT*TRANSPORT_PACKET_SIZE,
						  TRANSPORT_PACKET_COUNT*TRANSPORT_PACKET_SIZE);

// {7000C589-7FE4-4B64-837F-B6EDEBB59099}
const GUID guidNetupAntennaInName={ 0x7000C589, 0x7FE4, 0x4B64, 0x83, 0x7F, 0xB6, 0xED, 0xEB, 0xB5, 0x90, 0x99 };

/* PIN descriptor table */
const KSPIN_DESCRIPTOR_EX NetupPinDescriptors[]={
	{	// Antenna input pin
		&NetupAntennaDispatch,
		&NullAutomation,
		{
			0,NULL,
			0,	// Mediums
			NULL,
			SIZEOF_ARRAY(NetupAntennaFormat),
			NetupAntennaFormat,
			KSPIN_DATAFLOW_IN,
			KSPIN_COMMUNICATION_BOTH,
			&guidNetupAntennaInName,	// Category
			&guidNetupAntennaInName,	// Name
			0
		},
		KSPIN_FLAG_DO_NOT_USE_STANDARD_TRANSPORT|KSPIN_FLAG_FRAMES_NOT_REQUIRED_FOR_PROCESSING|KSPIN_FLAG_FIXED_FORMAT,
		1,
		0,
		NULL,
		AntennaIntersectDataFormat
	},
	{	// Transport Stream output pin
		&NetupOutputDispatch,
		&NullAutomation,
		{
			SIZEOF_ARRAY(NetupStreamInterface),
			NetupStreamInterface,
			0,NULL,
			SIZEOF_ARRAY(NetupOutputFormat),
			NetupOutputFormat,
			KSPIN_DATAFLOW_OUT,
			KSPIN_COMMUNICATION_BOTH,
			&PINNAME_BDA_TRANSPORT,
			&PINNAME_BDA_TRANSPORT,
			0
		},
		KSPIN_FLAG_FIXED_FORMAT|/*KSPIN_FLAG_DO_NOT_USE_STANDARD_TRANSPORT|KSPIN_FLAG_FRAMES_NOT_REQUIRED_FOR_PROCESSING|*/
			KSPIN_FLAG_DO_NOT_INITIATE_PROCESSING,
		1,
		1,
		&NetupOutputAllocator,
		OutputIntersectDataFormat
	},
};

/* Categories table */
const GUID NetupCategories[]={
	STATICGUIDOF(KSCATEGORY_BDA_RECEIVER_COMPONENT),
	STATICGUIDOF(KSCATEGORY_BDA_NETWORK_TUNER),
};

// {D5CED2B5-F7FC-44A6-98E5-19DB0040B41B}
const GUID guidNetupTuner0Name={ 0xD5CED2B5, 0xF7FC, 0x44A6, 0x98, 0xE5, 0x19, 0xDB, 0x00, 0x40, 0xB4, 0x1B };
// {D6AD142E-AF3E-4831-A1A7-26DA8280B2C4}
const GUID guidNetupTuner1Name={ 0xD6AD142E, 0xAF3E, 0x4831, 0xA1, 0xA7, 0x26, 0xDA, 0x82, 0x80, 0xB2, 0xC4 };
// {19D4E335-64EF-40F0-972A-D9DF3B631319}
const GUID guidNetupDemod0Name={ 0x19D4E335, 0x64EF, 0x40F0, 0x97, 0x2A, 0xD9, 0xDF, 0x3B, 0x63, 0x13, 0x19 };
// {686D922F-E095-4061-9EEC-972CFD46F42C}
const GUID guidNetupDemod1Name={ 0x686D922F, 0xE095, 0x4061, 0x9E, 0xEC, 0x97, 0x2C, 0xFD, 0x46, 0xF4, 0x2C };

/* Node descriptors table for channel 0 */
const KSNODE_DESCRIPTOR NetupFilter0NodeDescriptors[]={
	DEFINE_NODE_DESCRIPTOR(&NetupTunerAutomation,&KSNODE_BDA_RF_TUNER,&guidNetupTuner0Name),
	DEFINE_NODE_DESCRIPTOR(&NetupDemodulatorAutomation,&KSNODE_BDA_QPSK_DEMODULATOR,&guidNetupDemod0Name),
};

/* Node descriptors table for channel 1 */
const KSNODE_DESCRIPTOR NetupFilter1NodeDescriptors[]={
	DEFINE_NODE_DESCRIPTOR(&NetupTunerAutomation,&KSNODE_BDA_RF_TUNER,&guidNetupTuner1Name),
	DEFINE_NODE_DESCRIPTOR(&NetupDemodulatorAutomation,&KSNODE_BDA_QPSK_DEMODULATOR,&guidNetupDemod1Name),
};

/* Topology connections table */
const KSTOPOLOGY_CONNECTION NetupConnections[]={
	{KSFILTER_NODE,	0,						0,				KSNODEPIN_STANDARD_IN},	// Antenna pin -> Tuner pin 0
	{0,				KSNODEPIN_STANDARD_OUT,	1,				KSNODEPIN_STANDARD_IN},	// Tuner pin 1 -> Demodulator pin 0
	{1,				KSNODEPIN_STANDARD_OUT,	KSFILTER_NODE,	1},						// Demodulator pin 1 -> Transport pin
};

// Manufacturer GUID
// {750298D5-5388-4D44-A7CC-EEC2AE5EF078}
#define GUID_NETUP_INC { 0x750298d5, 0x5388, 0x4d44, 0xa7, 0xcc, 0xee, 0xc2, 0xae, 0x5e, 0xf0, 0x78 }
// Device GUID
// {5C8CCA60-1045-468E-A512-8944DFB889EB}
#define GUID_NETUP_DEVICE { 0x5c8cca60, 0x1045, 0x468e, 0xa5, 0x12, 0x89, 0x44, 0xdf, 0xb8, 0x89, 0xeb }
// Channel 0 GUID
// {367D8C1B-B865-476B-88EA-E6D1ABFF2D3B}
#define GUID_NETUP_DEVICE_FILTER0 { 0x367d8c1b, 0xb865, 0x476b, 0x88, 0xea, 0xe6, 0xd1, 0xab, 0xff, 0x2d, 0x3b }
// Channel 1 GUID
// {5B378678-0C5D-4A26-8427-C2409ADA2D42}
#define GUID_NETUP_DEVICE_FILTER1 { 0x5b378678, 0xc5d, 0x4a26, 0x84, 0x27, 0xc2, 0x40, 0x9a, 0xda, 0x2d, 0x42 }


/* Component ID */
const KSCOMPONENTID NetupFilter0ComponentID={
	GUID_NETUP_INC,
	GUID_NETUP_DEVICE,
	GUID_NETUP_DEVICE_FILTER0,
	NULL,
	1,	// Version
	0	// Revision
};

/* Component ID */
const KSCOMPONENTID NetupFilter1ComponentID={
	GUID_NETUP_INC,
	GUID_NETUP_DEVICE,
	GUID_NETUP_DEVICE_FILTER1,
	NULL,
	1,	// Version
	0	// Revision
};

// {5D4A21D7-D575-4D81-9D35-5D561EA61DF3}
static const GUID guidNetupFilter0 = 
{ 0x5d4a21d7, 0xd575, 0x4d81, { 0x9d, 0x35, 0x5d, 0x56, 0x1e, 0xa6, 0x1d, 0xf3 } };
// {B9C47A07-92DB-406E-9B27-AA20784A4BD6}
static const GUID guidNetupFilter1 = 
{ 0xb9c47a07, 0x92db, 0x406e, { 0x9b, 0x27, 0xaa, 0x20, 0x78, 0x4a, 0x4b, 0xd6 } };

/* Channel 0 filter descriptor */
extern DEFINE_KSFILTER_DESCRIPTOR(NetupFilter0Descriptor)
{
	/* Dispatch */			&NetupFilter0Dispatch,
	/* Automation */		&NetupFilterAutomation,
	/* Version */			KSFILTER_DESCRIPTOR_VERSION,
	/* Flags */				0,
	/* Reference GUID */	&guidNetupFilter0,
    /* Pin descriptors */	DEFINE_KSFILTER_PIN_DESCRIPTORS(NetupPinDescriptors),
    /* Categories */		DEFINE_KSFILTER_CATEGORIES(NetupCategories),
    /* Node descriptors */	DEFINE_KSFILTER_NODE_DESCRIPTORS(NetupFilter0NodeDescriptors),
	/* Connections */		DEFINE_KSFILTER_CONNECTIONS(NetupConnections),
    /* Component ID */		NULL
};

/* Channel 1 filter descriptor */
extern DEFINE_KSFILTER_DESCRIPTOR(NetupFilter1Descriptor)
{
	/* Dispatch */			&NetupFilter1Dispatch,
	/* Automation */		&NetupFilterAutomation,
	/* Version */			KSFILTER_DESCRIPTOR_VERSION,
	/* Flags */				0,
	/* Reference GUID */	&guidNetupFilter1,
    /* Pin descriptors */	DEFINE_KSFILTER_PIN_DESCRIPTORS(NetupPinDescriptors),
    /* Categories */		DEFINE_KSFILTER_CATEGORIES(NetupCategories),
    /* Node descriptors */	DEFINE_KSFILTER_NODE_DESCRIPTORS(NetupFilter1NodeDescriptors),
	/* Connections */		DEFINE_KSFILTER_CONNECTIONS(NetupConnections),
    /* Component ID */		NULL
};

/* Array of joint values. The value given to a joint corresponds to the index of an element in a array of template connections (KSTOPOLOGY_CONNECTION) */
const ULONG NetupTransportJoints[]={
	1,
};

/* this structure describes the topology between a pair of input and output pins */
const BDA_PIN_PAIRING NetupFilterPinPairings[]={
	0,	// Input pin index in PinDescriptors
	1,	// Output pin index in PinDescriptors
	1,	// ulcMaxInputsPerOutput
	1,	// ulcMinInputsPerOutput
	1,	// ulcMaxOutputsPerInput
	1,	// ulcMinOutputsPerInput
	SIZEOF_ARRAY(NetupTransportJoints),
	NetupTransportJoints
};

/* this structure describes the template topology for a BDA filter for filter channel 0 */
extern const BDA_FILTER_TEMPLATE NetupFilter0Template={
	&NetupFilter0Descriptor,
    SIZEOF_ARRAY(NetupFilterPinPairings),
    NetupFilterPinPairings
};

/* this structure describes the template topology for a BDA filter for filter channel 1 */
extern const BDA_FILTER_TEMPLATE NetupFilter1Template={
	&NetupFilter1Descriptor,
    SIZEOF_ARRAY(NetupFilterPinPairings),
    NetupFilterPinPairings
};
