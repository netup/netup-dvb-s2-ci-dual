/*
 * device.cpp
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

#include "device.h"
#include "filter.h"
#include "cx23885.h"

BOOLEAN NetupDeviceInterrupt(IN PKINTERRUPT Interrupt,IN PVOID ServiceContext)
{
	// Return FALSE if the interrupt is not ours, TRUE otherwise
	return CX23885_Interrupt((PKSDEVICE)ServiceContext);
}

void NetupDeviceDPCRoutine(IN PKDPC Dpc,IN PVOID DeferredContext,IN PVOID SystemArgument1,IN PVOID SystemArgument2)
{
	CX23885_DPCRoutine(Dpc,DeferredContext,SystemArgument1,SystemArgument2);
}

NTSTATUS NetupDeviceAdd(IN PKSDEVICE device)
{
	NTSTATUS result = STATUS_SUCCESS;
	PVOID context = ExAllocatePoolWithTag(NonPagedPool, sizeof(DEVICE_CONTEXT), 'noCD');
	RtlZeroMemory(context, sizeof(DEVICE_CONTEXT));
	if(!context)
	{
		return STATUS_INSUFFICIENT_RESOURCES;
	}
	device->Context = context;
	result = KsAddItemToObjectBag(device->Bag, context, NULL);
	if(result != STATUS_SUCCESS)
	{
		ExFreePool(context);
		return result;
	}
	DbgPrint(LOG_PREFIX " *** device added ***");
	return result;
}

VOID NetupDeviceRemove(IN PKSDEVICE device,IN PIRP irp)
{
	DbgPrint(LOG_PREFIX " *** device removed ***");
}

NTSTATUS NetupDeviceStart(IN PKSDEVICE device,IN PIRP irp,IN PCM_RESOURCE_LIST translatedResources,IN PCM_RESOURCE_LIST untranslatedResources)
{
	NTSTATUS result = STATUS_SUCCESS;
	ULONG i,mem,interrupt;

	DbgPrint(LOG_PREFIX " *** starting device ***");
	// Search for memory to map
	for(i=0,mem=-1,interrupt=-1;(i<translatedResources->List[0].PartialResourceList.Count)&&((mem==-1)||(interrupt==-1));i++)
	{
		// Check type
		if(translatedResources->List[0].PartialResourceList.PartialDescriptors[i].Type==CmResourceTypeMemory) mem=i;
		if(translatedResources->List[0].PartialResourceList.PartialDescriptors[i].Type==CmResourceTypeInterrupt) interrupt=i;
	}
	if(mem == -1)
	{
		KdPrint((LOG_PREFIX "Failed to find memory resource"));
		return STATUS_INSUFFICIENT_RESOURCES;
	}
	if(interrupt == -1)
	{
		KdPrint((LOG_PREFIX "Failed to find interrupt resource"));
		return STATUS_INSUFFICIENT_RESOURCES;	// Not really the right error
	}
	if(device->Started)
	{
		KdPrint((LOG_PREFIX "error: device already started"));
		return STATUS_INSUFFICIENT_RESOURCES;
	}
	// Map memory
	GETCONTEXT(device)->pci_memory=MmMapIoSpace(translatedResources->List[0].PartialResourceList.PartialDescriptors[mem].u.Memory.Start,translatedResources->List[0].PartialResourceList.PartialDescriptors[mem].u.Memory.Length,MmNonCached);
	GETCONTEXT(device)->pci_size=translatedResources->List[0].PartialResourceList.PartialDescriptors[mem].u.Memory.Length;
	if(GETCONTEXT(device)->pci_memory==NULL) {
		KdPrint((LOG_PREFIX "Failed to map memory"));
		return STATUS_INSUFFICIENT_RESOURCES;
	}
	KdPrint((LOG_PREFIX "resource acquired: pci_memory (0x%x, %d)",
		GETCONTEXT(device)->pci_memory,
		GETCONTEXT(device)->pci_size));
	// Connect interrupt
	result=IoConnectInterrupt(&(GETCONTEXT(device)->pci_interrupt),NetupDeviceInterrupt,device,NULL,
		translatedResources->List[0].PartialResourceList.PartialDescriptors[interrupt].u.Interrupt.Vector,
		(KIRQL)translatedResources->List[0].PartialResourceList.PartialDescriptors[interrupt].u.Interrupt.Level,
		(KIRQL)translatedResources->List[0].PartialResourceList.PartialDescriptors[interrupt].u.Interrupt.Level,
		(translatedResources->List[0].PartialResourceList.PartialDescriptors[interrupt].Flags&CM_RESOURCE_INTERRUPT_LATCHED)?Latched:LevelSensitive,
		TRUE,
		translatedResources->List[0].PartialResourceList.PartialDescriptors[interrupt].u.Interrupt.Affinity,
		FALSE);
	if(result!=STATUS_SUCCESS) {
		KdPrint((LOG_PREFIX "Failed to connect interrupt: %d",result));
		MmUnmapIoSpace(GETCONTEXT(device)->pci_memory,GETCONTEXT(device)->pci_size);
		return result;
	}
	
	KdPrint((LOG_PREFIX "resource acquired: interrupt (%d)",
		translatedResources->List[0].PartialResourceList.PartialDescriptors[interrupt].u.Interrupt.Vector));

	
	DEVICE_DESCRIPTION desc;
	ULONG mapRegs = 0;
	RtlZeroMemory(&desc, sizeof(desc));
	desc.Version = DEVICE_DESCRIPTION_VERSION;
	desc.Master = TRUE;
	desc.ScatterGather = FALSE;
	desc.Dma32BitAddresses = TRUE;
	desc.InterfaceType = PCIBus;
	desc.MaximumLength = 4 * 1024 * 1024;
	GETCONTEXT(device)->dma = IoGetDmaAdapter( device->PhysicalDeviceObject, &desc, &mapRegs);
	if(GETCONTEXT(device)->dma == NULL)
	{
		KdPrint((LOG_PREFIX "ERROR: Couldn't create DMA adapter"));
		IoDisconnectInterrupt(GETCONTEXT(device)->pci_interrupt);
		MmUnmapIoSpace( GETCONTEXT(device)->pci_memory, GETCONTEXT(device)->pci_size);
		return STATUS_INSUFFICIENT_RESOURCES;
	}
	KdPrint((LOG_PREFIX "%s: DMA NumberOfMapRegisters %d", mapRegs));

	KeInitializeDpc(&GETCONTEXT(device)->ci_dpc, Netup_CI_DPC_Routine, device);

	result=CX23885_Init(device);
	if(result != STATUS_SUCCESS)
	{
		KdPrint((LOG_PREFIX "unable to initialize CX23885 PCIe bridge: %d", result));
		IoDisconnectInterrupt(GETCONTEXT(device)->pci_interrupt);
		MmUnmapIoSpace(GETCONTEXT(device)->pci_memory,GETCONTEXT(device)->pci_size);
		return result;
	}

	result=BdaCreateFilterFactory(device,&NetupFilter0Descriptor,&NetupFilter0Template);
	if(result!=STATUS_SUCCESS) {
		KdPrint((LOG_PREFIX " unable to create filter #0 factory: %d", result));
		return result;
	}
	result=BdaCreateFilterFactory(device,&NetupFilter1Descriptor,&NetupFilter1Template);
	if(result!=STATUS_SUCCESS) {
		KdPrint((LOG_PREFIX " unable to create filter #1 factory: %d", result));
		return result;
	}

	DbgPrint(LOG_PREFIX " *** started ***");
	return result;
}

VOID NetupDeviceStop(IN PKSDEVICE device,IN PIRP irp)
{
	DbgPrint(LOG_PREFIX " *** stop received ***");
	Netup_CI_Stop(device);
	CX23885_Close(device);
	// Release DMA adapter
	GETCONTEXT(device)->dma->DmaOperations->PutDmaAdapter(GETCONTEXT(device)->dma);
	// Disconnect interupt
	IoDisconnectInterrupt(GETCONTEXT(device)->pci_interrupt);
	// Unmap memory
	MmUnmapIoSpace(GETCONTEXT(device)->pci_memory,GETCONTEXT(device)->pci_size);
	DbgPrint(LOG_PREFIX " *** stopped ***");
}

const KSDEVICE_DISPATCH NetupDeviceDispatch = 
{
	/* Add				 */		NetupDeviceAdd,
	/* Start			 */		NetupDeviceStart,
	/* PostStart		 */		NULL,
	/* QueryStop		 */		NULL,
	/* CancelStop		 */		NULL,
	/* Stop				 */		NetupDeviceStop,
	/* QueryRemove		 */		NULL,
	/* CancelRemove		 */		NULL,
	/* Remove			 */		NetupDeviceRemove,
	/* QueryCapabilities */		NULL,
	/* SurpriseRemoval	 */		NULL,
	/* QueryPower		 */		NULL,
	/* SetPower			 */		NULL
};

const KSDEVICE_DESCRIPTOR NetupDeviceDescriptor = 
{
	&NetupDeviceDispatch,
	0,
	NULL,
	0,
	0
};