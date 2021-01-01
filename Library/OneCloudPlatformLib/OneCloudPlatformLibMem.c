/** @file
*
*  Copyright (c) 2011, ARM Limited. All rights reserved.
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/

#include <Library/ArmPlatformLib.h>
#include <Library/DebugLib.h>
#include <Library/PcdLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/IoLib.h>

/**
  Return the Virtual Memory Map of your platform

  This Virtual Memory Map is used by MemoryInitPei Module to initialize the MMU on your platform.

  @param[out]   VirtualMemoryMap    Array of ARM_MEMORY_REGION_DESCRIPTOR describing a Physical-to-
                                    Virtual Memory mapping. This array must be ended by a zero-filled
                                    entry

**/

#define MAX_VIRTUAL_MEMORY_MAP_DESCRIPTORS 12
/*
Start End Region (Boot) Region (Normal)
0x00000000 0x00FFFFFF Boot ROM DDR
0x01000000 0xBFFFFFFF DDR DDR
0xC0000000 0xC12FFFFF APB3 CBUS APB3 CBUS
0xC1300000 0xC13FFFFF AXI AXI
0xC4200000 0xC4200FFF L2 Cache regs L2 Cache regs
0xC4300000 0xC430FFFF A9 Periph base A9 Periph base
0xC8000000 0xC8003FFF DDR APB DDR APB
0xC8006000 0xC8007FFF DMC APB DMC APB
0xC8100000 0xC81FFFFF RTI RTI
0xC9040000 0xC90BFFFF USB0 USB0
0xC90C0000 0xC90FFFFF USB1 USB1
0xC9410000 0xC941FFFF Ethernet slave Ethernet slave
0xCC000000 0xCFFFFFFF SPI SPI
0xD0000000 0xD003FFFF A9_DAPLITE A9_DAPLITE
0xD0042000 0xD0043FFF HDMI TX HDMI TX
0xD0048000 0xD004FFFF NAND NAND
0xD0050000 0xD005FFFF DOS DOS
0xD00C0000 0xD00FFFFF Mali APB Mali APB
0xD0100000 0xD013FFFF VPU VPU
0xD0150000 0xD015FFFF MIPI MIPI
0xD9000000 0xD903FFFF AHB SRAM AHB SRAM
0xD9040000 0xD904FFFF Boot ROM Boot ROM
0xDA000000 0xDA001FFF Efuse Efuse
0xDA002000 0xDA003FFF SEC MMC regs SEC MMC regs
0xDA004000 0xDA005FFF SEC Cntl Regs SEC Cntl Regs
0xDA006000 0xDA007FFF BLKMV BLKMV
0xE0000000 0xFFFFFFFF DDR DDR
*/
VOID
ArmPlatformGetVirtualMemoryMap (
  IN ARM_MEMORY_REGION_DESCRIPTOR** VirtualMemoryMap
  )
{
  
  ARM_MEMORY_REGION_ATTRIBUTES  CacheAttributes;
  UINTN                         Index = 0;
  ARM_MEMORY_REGION_DESCRIPTOR  *VirtualMemoryTable;

  ASSERT(VirtualMemoryMap != NULL);

  VirtualMemoryTable = (ARM_MEMORY_REGION_DESCRIPTOR*)AllocatePages(EFI_SIZE_TO_PAGES (sizeof(ARM_MEMORY_REGION_DESCRIPTOR) * MAX_VIRTUAL_MEMORY_MAP_DESCRIPTORS));
  if (VirtualMemoryTable == NULL) {
    return;
  }

  if (FeaturePcdGet(PcdCacheEnable) == TRUE) {
    CacheAttributes = ARM_MEMORY_REGION_ATTRIBUTE_WRITE_BACK;
  } else {
    CacheAttributes = ARM_MEMORY_REGION_ATTRIBUTE_UNCACHED_UNBUFFERED;
  }


	// Start End Region (Boot) Region (Normal)
	// 0x00000000 0x00FFFFFF Boot ROM DDR
	// 0x01000000 0xBFFFFFFF DDR DDR
  VirtualMemoryTable[Index].PhysicalBase = PcdGet64 (PcdSystemMemoryBase);
  VirtualMemoryTable[Index].VirtualBase  = PcdGet64 (PcdSystemMemoryBase);
  VirtualMemoryTable[Index].Length       = PcdGet64 (PcdSystemMemorySize);
  VirtualMemoryTable[Index].Attributes   = CacheAttributes;
	// 0xC0000000 0xC12FFFFF APB3 CBUS APB3 CBUS
  VirtualMemoryTable[++Index].PhysicalBase = 0xC0000000;
  VirtualMemoryTable[Index].VirtualBase  = 0xC0000000;
  VirtualMemoryTable[Index].Length       = 0x12FFFFF;
  VirtualMemoryTable[Index].Attributes   = ARM_MEMORY_REGION_ATTRIBUTE_DEVICE;
	// 0xC1300000 0xC13FFFFF AXI AXI
  VirtualMemoryTable[++Index].PhysicalBase = 0xC1300000;
  VirtualMemoryTable[Index].VirtualBase  = 0xC1300000;
  VirtualMemoryTable[Index].Length       = 0xFFFFF;
  VirtualMemoryTable[Index].Attributes   = ARM_MEMORY_REGION_ATTRIBUTE_DEVICE;
	// 0xC4200000 0xC4200FFF L2 Cache regs L2 Cache regs
  VirtualMemoryTable[++Index].PhysicalBase = 0xC4200000;
  VirtualMemoryTable[Index].VirtualBase  = 0xC4200000;
  VirtualMemoryTable[Index].Length       = 0xFFF;
  VirtualMemoryTable[Index].Attributes   = ARM_MEMORY_REGION_ATTRIBUTE_DEVICE;
	// 0xC4300000 0xC430FFFF A9 Periph base A9 Periph base
  VirtualMemoryTable[++Index].PhysicalBase = 0xC4300000;
  VirtualMemoryTable[Index].VirtualBase  = 0xC4300000;
  VirtualMemoryTable[Index].Length       = 0xFFFF;
  VirtualMemoryTable[Index].Attributes   = ARM_MEMORY_REGION_ATTRIBUTE_DEVICE;
	// 0xC8000000 0xC8003FFF DDR APB DDR APB
	// 0xC8006000 0xC8007FFF DMC APB DMC APB
  VirtualMemoryTable[++Index].PhysicalBase = 0xC8000000;
  VirtualMemoryTable[Index].VirtualBase  = 0xC8000000;
  VirtualMemoryTable[Index].Length       = 0x7FFF;
  VirtualMemoryTable[Index].Attributes   = ARM_MEMORY_REGION_ATTRIBUTE_DEVICE;
	// 0xC8100000 0xC81FFFFF RTI RTI
  VirtualMemoryTable[++Index].PhysicalBase = 0xC8100000;
  VirtualMemoryTable[Index].VirtualBase  = 0xC8100000;
  VirtualMemoryTable[Index].Length       = 0xFFFFF;
  VirtualMemoryTable[Index].Attributes   = ARM_MEMORY_REGION_ATTRIBUTE_DEVICE;
	// 0xC9040000 0xC90BFFFF USB0 USB0 0xC90C0000 0xC90FFFFF USB1 USB1 0xC9410000 0xC941FFFF Ethernet slave Ethernet slave
  VirtualMemoryTable[++Index].PhysicalBase = 0xC9000000;
  VirtualMemoryTable[Index].VirtualBase  = 0xC9000000;
  VirtualMemoryTable[Index].Length       = 0x41FFFF;
  VirtualMemoryTable[Index].Attributes   = ARM_MEMORY_REGION_ATTRIBUTE_DEVICE;
	// 0xCC000000 0xCFFFFFFF SPI SPI
  VirtualMemoryTable[++Index].PhysicalBase = 0xCC000000;
  VirtualMemoryTable[Index].VirtualBase  = 0xCC000000;
  VirtualMemoryTable[Index].Length       = 0x3FFFFFF;
  VirtualMemoryTable[Index].Attributes   = ARM_MEMORY_REGION_ATTRIBUTE_DEVICE;

	// 0xD0000000 0xD003FFFF A9_DAPLITE A9_DAPLITE 0xD0042000 0xD0043FFF HDMI TX HDMI TX 0xD0048000 0xD004FFFF NAND NAND 0xD0050000 0xD005FFFF DOS DOS 0xD00C0000 0xD00FFFFF Mali APB Mali APB  0xD0100000 0xD013FFFF VPU VPU  0xD0150000 0xD015FFFF MIPI MIPI
  VirtualMemoryTable[++Index].PhysicalBase = 0xD0000000;
  VirtualMemoryTable[Index].VirtualBase  = 0xD0000000;
  VirtualMemoryTable[Index].Length       = 0x15FFFF;
  VirtualMemoryTable[Index].Attributes   = ARM_MEMORY_REGION_ATTRIBUTE_DEVICE;
	// 0xD9000000 0xD903FFFF AHB SRAM AHB SRAM
  VirtualMemoryTable[++Index].PhysicalBase = 0xD9000000;
  VirtualMemoryTable[Index].VirtualBase  = 0xD9000000;
  VirtualMemoryTable[Index].Length       = 0x3FFFF;
  VirtualMemoryTable[Index].Attributes   = ARM_MEMORY_REGION_ATTRIBUTE_UNCACHED_UNBUFFERED;

  // End of Table
  VirtualMemoryTable[++Index].PhysicalBase = 0;
  VirtualMemoryTable[Index].VirtualBase  = 0;
  VirtualMemoryTable[Index].Length       = 0;
  VirtualMemoryTable[Index].Attributes   = (ARM_MEMORY_REGION_ATTRIBUTES)0;

  ASSERT((Index + 1) == MAX_VIRTUAL_MEMORY_MAP_DESCRIPTORS);

  *VirtualMemoryMap = VirtualMemoryTable;
}
