/** @file
 *
 *  Copyright (c) 2017 - 2019, Andrey Warkentin <andrey.warkentin@gmail.com>
 *  Copyright (c) 2015 - 2016, Linaro Limited. All rights reserved.
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

#ifndef _DWUSBHOSTDXE_H_
#define _DWUSBHOSTDXE_H_

#include <Uefi.h>

#include <Protocol/Usb2HostController.h>

#include <Guid/EventGroup.h>

#include <Library/DebugLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/UefiDriverEntryPoint.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/BaseLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/ReportStatusCodeLib.h>
#include <Library/DevicePathLib.h>
#include <Library/PcdLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include <Library/DmaLib.h>
#include <Library/ArmLib.h>

#define MAX_DEVICE                      16
#define MAX_ENDPOINT                    16

#define DWUSB_OTGHC_DEV_SIGNATURE       SIGNATURE_32 ('d', 'w', 'h', 'c')
#define DWHC_FROM_THIS(a)               CR(a, DWUSB_OTGHC_DEV, DwUsbOtgHc, DWUSB_OTGHC_DEV_SIGNATURE)

//
// Iterate through the double linked list. NOT delete safe
//
#define EFI_LIST_FOR_EACH(Entry, ListHead)    \
  for(Entry = (ListHead)->ForwardLink; Entry != (ListHead); Entry = Entry->ForwardLink)

//
// Iterate through the double linked list. This is delete-safe.
// Do not touch NextEntry
//
#define EFI_LIST_FOR_EACH_SAFE(Entry, NextEntry, ListHead)            \
  for(Entry = (ListHead)->ForwardLink, NextEntry = Entry->ForwardLink;\
      Entry != (ListHead); Entry = NextEntry, NextEntry = Entry->ForwardLink)

#define EFI_LIST_CONTAINER(Entry, Type, Field) BASE_CR(Entry, Type, Field)

//
// The RequestType in EFI_USB_DEVICE_REQUEST is composed of
// three fields: One bit direction, 2 bit type, and 5 bit
// target.
//
#define USB_REQUEST_TYPE(Dir, Type, Target)                             \
  ((UINT8)((((Dir) == EfiUsbDataIn ? 0x01 : 0) << 7) | (Type) | (Target)))

typedef struct {
  VENDOR_DEVICE_PATH            Custom;
  EFI_DEVICE_PATH_PROTOCOL      EndDevicePath;
} EFI_DW_DEVICE_PATH;

typedef struct _DWUSB_DEFERRED_REQ {
  IN OUT LIST_ENTRY                         List;
  IN     struct _DWUSB_OTGHC_DEV            *DwHc;
  IN     UINT32                             Channel;
  IN     UINT32                             FrameInterval;
  IN     UINT32                             TargetFrame;
  IN     EFI_USB2_HC_TRANSACTION_TRANSLATOR *Translator;
  IN     UINT8                              DeviceSpeed;
  IN     UINT8                              DeviceAddress;
  IN     UINTN                              MaximumPacketLength;
  IN     UINT32                             TransferDirection;
  IN OUT VOID                               *Data;
  IN OUT UINTN                              DataLength;
  IN OUT UINT32                             Pid;
  IN     UINT32                             EpAddress;
  IN     UINT32                             EpType;
  OUT    UINT32                             TransferResult;
  IN     BOOLEAN                            IgnoreAck;
  IN     EFI_ASYNC_USB_TRANSFER_CALLBACK    CallbackFunction;
  IN     VOID                               *CallbackContext;
  IN     UINTN                              TimeOut;
} DWUSB_DEFERRED_REQ;

typedef struct _DWUSB_OTGHC_DEV {
  UINTN                           Signature;

  EFI_USB2_HC_PROTOCOL            DwUsbOtgHc;

  EFI_USB_HC_STATE                DwHcState;

  EFI_EVENT                       ExitBootServiceEvent;

  EFI_EVENT                       PeriodicEvent;

  EFI_PHYSICAL_ADDRESS            DwUsbBase;
  UINT8                           *StatusBuffer;

  UINT8                           *AlignedBuffer;
  VOID *                          AlignedBufferMapping;
  UINTN                           AlignedBufferBusAddress;
  LIST_ENTRY                      DeferredList;
  /*
   * 1ms frames.
   */
  UINTN                           CurrentFrame;
  /*
   * 125us frames;
   */
  UINT16                          LastMicroFrame;
} DWUSB_OTGHC_DEV;

extern EFI_COMPONENT_NAME_PROTOCOL gComponentName;
extern EFI_COMPONENT_NAME2_PROTOCOL gComponentName2;

EFI_STATUS
CreateDwcOtgDevice (
  IN  UINT32          DwUsbBase,
  OUT DWUSB_OTGHC_DEV **OutDwHc
  );

VOID
DestroyDwUsbHc (
  IN  DWUSB_OTGHC_DEV *Dev
  );

EFI_STATUS
EFIAPI
DwHcReset (
  IN  EFI_USB2_HC_PROTOCOL *This,
  IN  UINT16               Attributes
  );

EFI_STATUS
EFIAPI
DwHcSetState (
  IN  EFI_USB2_HC_PROTOCOL *This,
  IN  EFI_USB_HC_STATE     State
  );

VOID
DwHcQuiesce (
  IN  DWUSB_OTGHC_DEV *DwHc
  );

#define CONFIG_M8_USBPORT_BASE_A	0xC9040000
#define CONFIG_M8_USBPORT_BASE_B	0xC90C0000
#define USB_PHY_CLK_SEL_XTAL	0


#define BOARD_USB_MODE_HOST	0
#define BOARD_USB_MODE_SLAVE	1
#define BOARD_USB_MODE_CHARGER	2
#define BOARD_USB_MODE_MAX	3

#define USB_PHY_PORT_A	    (0x40000)
#define USB_PHY_PORT_B	    (0xc0000)
#define USB_PHY_PORT_C	    (0x100000)
#define USB_PHY_PORT_D	    (0x140000)
#define USB_PHY_PORT_MSK	(0x1f0000)
#define USB_PHY_PORT_MAX	2
#define PREI_USB_PHY_REG_A     0x2200
#define PREI_USB_PHY_REG_B     0x2208

#define IO_CBUS_BASE                                                      0xc1100000
#define CBUS_REG_OFFSET(reg)                                              ((reg) << 2)
#define CBUS_REG_ADDR(reg)                                                (IO_CBUS_BASE + CBUS_REG_OFFSET(reg))

#define USB_AML_REGS_CONFIG                     0x00
#define USB_AML_REGS_CTRL                       0x04
#define USB_AML_REGS_ENDP_INTR                  0x08
#define USB_AML_REGS_ADP_BC                     0x0c
#define USB_AML_REGS_DBG_UART                   0x10
#define USB_AML_REGS_TEST                       0x14
#define USB_AML_REGS_TUNE                       0x1c

// usb_config_data
#define CLK_EN_MASK                            (1<<0)
#define CLK_EN_OFFEST                           0
#define CLK_SEL_MASK                           (0x7<<1)
#define CLK_SEL_OFFEST                          1
#define CLK_DIV_MASK                           (0x7f<<4)
#define CLK_DIV_OFFEST                          4
#define RESERVED0_MASK                         (0xf<<11)
#define RESERVED0_OFFEST                        11
#define CLK_32K_ALT_SEL_MASK                   (1<<15)
#define CLK_32K_ALT_SEL_OFFSET                  15
#define RESERVED1_MASK                         (0xfffff<<10)
#define RESERVED1_OFFEST                        10
#define TEST_TRID_MASK                         (1<<31)
#define TEST_TRID_OFFEST                        31


// usb_ctrl_data
#define	SOFT_PRST_MASK                         (1<<0)
#define	SOFT_PRST_OFFSET                        0
#define	SOFT_HRESET_MASK                       (1<<1)
#define	SOFT_HRESET_OFFSET                      1
#define	SS_SCALEDOWN_MODE_MASK                 (0x3<<2)
#define	SS_SCALEDOWN_MODE_OFFSET                2
#define	CLK_DET_RST_MASK                       (1<<4)
#define	CLK_DET_RST_OFFSET                      4
#define	INTR_SEL_MASK                          (1<<5)
#define	INTR_SEL_OFFSET                         5
#define	RESERVED_MASK                          (0x3<<6)
#define	RESERVED_OFFSET                         6
#define	CLK_DETECTED_MASK                      (1<<8)
#define	CLK_DETECTED_OFFSET                     8
#define	SOF_SENT_RCVD_TGL_MASK                 (1<<9)
#define	SOF_SENT_RCVD_TGL_OFFSET                9
#define	SOF_TOGGLE_OUT_MASK                    (1<<10)
#define	SOF_TOGGLE_OUT_OFFSET                   10
#define	NOT_USED_MASK                          (0xf<<11)
#define	NOT_USED_OFFSET                         11
#define	POR_MASK                               (1<<15)
#define	POR_OFFSET                              15
#define	SLEEPM_MASK                            (1<<16)
#define	SLEEPM_OFFSET                           16
#define	TXBITSTUFFENNH_MASK                    (1<<17)
#define	TXBITSTUFFENNH_OFFSET                   17
#define	TXBITSTUFFENN_MASK                     (1<<18)
#define	TXBITSTUFFENN_OFFSET                    18
#define	COMMONONN_MASK                         (1<<19)
#define	COMMONONN_OFFSET                        19
#define	REFCLKSEL_MASK                         (0x3<<20)
#define	REFCLKSEL_OFFSET                        20
#define	FSEL_MASK                              (0x7<<22)
#define	FSEL_OFFSET                             22
#define	PORTRESET_MASK                         (1<<25)
#define	PORTRESET_OFFSET                        25
#define	THREAD_ID_MASK                         (0x3f<<26)
#define	THREAD_ID_OFFSET                        26

#define MAX_EPS_CHANNELS                        16
#define DWC_PHY_TYPE_PARAM_FS 0
#define DWC_PHY_TYPE_PARAM_UTMI 1
#define DWC_PHY_TYPE_PARAM_ULPI 2

#define DWC_GAHBCFG_TXFEMPTYLVL_EMPTY		1
#define DWC_GAHBCFG_TXFEMPTYLVL_HALFEMPTY	0

#define DWC_GAHBCFG_INT_DMA_BURST_SINGLE	0
#define DWC_GAHBCFG_INT_DMA_BURST_INCR		1
#define DWC_GAHBCFG_INT_DMA_BURST_INCR4		3
#define DWC_GAHBCFG_INT_DMA_BURST_INCR8		5
#define DWC_GAHBCFG_INT_DMA_BURST_INCR16	7

#define DWC_MODE_HNP_SRP_CAPABLE	0
#define DWC_MODE_SRP_ONLY_CAPABLE	1
#define DWC_MODE_NO_HNP_SRP_CAPABLE		2
#define DWC_MODE_SRP_CAPABLE_DEVICE		3
#define DWC_MODE_NO_SRP_CAPABLE_DEVICE	4
#define DWC_MODE_SRP_CAPABLE_HOST	5
#define DWC_MODE_NO_SRP_CAPABLE_HOST	6

#define DWC_OTG_CAP_PARAM_HNP_SRP_CAPABLE 0
#define DWC_OTG_CAP_PARAM_SRP_ONLY_CAPABLE 1
#define DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE 2

#define DWC_SLAVE_ONLY_ARCH 0
#define DWC_EXT_DMA_ARCH 1
#define DWC_INT_DMA_ARCH 2

	/** A-Device is a_host */
#define A_HOST		(1)
	/** A-Device is a_suspend */
#define A_SUSPEND	(2)
	/** A-Device is a_peripherial */
#define A_PERIPHERAL	(3)
	/** B-Device is operating as a Peripheral. */
#define B_PERIPHERAL	(4)
	/** B-Device is operating as a Host. */
#define B_HOST		(5)	

#define USB_ISOCHRONOUS    0
#define USB_INTERRUPT      1
#define USB_CONTROL        2
#define USB_BULK           3

#define DWC_OTG_EP_SPEED_LOW	0
#define DWC_OTG_EP_SPEED_FULL	1
#define DWC_OTG_EP_SPEED_HIGH	2	

#define FORCE_ID_CLEAR	-1
#define FORCE_ID_HOST	0
#define FORCE_ID_SLAVE	1
#define FORCE_ID_ERROR	2

#endif //_DWUSBHOSTDXE_H_
