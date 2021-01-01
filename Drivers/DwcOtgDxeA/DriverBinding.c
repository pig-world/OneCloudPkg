/** @file
 *
 *  Copyright (c) 2018 - 2019, Andrey Warkentin <andrey.warkentin@gmail.com>
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

#include "DwcOtgDxe.h"

STATIC
EFI_STATUS
EFIAPI
DriverSupported (
  IN  EFI_DRIVER_BINDING_PROTOCOL *This,
  IN  EFI_HANDLE                  Controller,
  IN  EFI_DEVICE_PATH_PROTOCOL    *RemainingDevicePath
  );

STATIC
EFI_STATUS
EFIAPI
DriverStart (
  IN  EFI_DRIVER_BINDING_PROTOCOL *This,
  IN  EFI_HANDLE                  Controller,
  IN  EFI_DEVICE_PATH_PROTOCOL    *RemainingDevicePath
  );

STATIC
EFI_STATUS
EFIAPI
DriverStop (
  IN  EFI_DRIVER_BINDING_PROTOCOL *This,
  IN  EFI_HANDLE                  Controller,
  IN  UINTN                       NumberOfChildren,
  IN  EFI_HANDLE                  *ChildHandleBuffer
  );

STATIC EFI_DRIVER_BINDING_PROTOCOL mDriverBinding = {
  DriverSupported,
  DriverStart,
  DriverStop,
  0xa,
  NULL,
  NULL
};

STATIC EFI_DW_DEVICE_PATH mDevicePath = {
  {
    {
      HARDWARE_DEVICE_PATH,
      HW_VENDOR_DP,
      {
        (UINT8)(sizeof(VENDOR_DEVICE_PATH)),
        (UINT8)((sizeof(VENDOR_DEVICE_PATH)) >> 8),
      }
    },
    EFI_CALLER_ID_GUID
  },
  {
    END_DEVICE_PATH_TYPE,
    END_ENTIRE_DEVICE_PATH_SUBTYPE,
    {
      sizeof (EFI_DEVICE_PATH_PROTOCOL),
      0
    }
  }
};

STATIC EFI_HANDLE mDevice;

STATIC
EFI_STATUS
EFIAPI
DriverSupported (
  IN  EFI_DRIVER_BINDING_PROTOCOL *This,
  IN  EFI_HANDLE                  Controller,
  IN  EFI_DEVICE_PATH_PROTOCOL    *RemainingDevicePath
  )
{
  VOID *Temp;

  if (Controller != mDevice) {
    return EFI_UNSUPPORTED;
  }
  
  if (gBS->HandleProtocol(Controller, &gEfiUsb2HcProtocolGuid,
                          (VOID **) &Temp) == EFI_SUCCESS) {
    return EFI_ALREADY_STARTED;
  }

  return EFI_SUCCESS;
}

EFI_STATUS
SetUsbPhyClock(
    IN UINT32 port
    )
{
	DEBUG((EFI_D_INFO,"SetUsbPhyClock\n"));
	UINT32 peri=0;

	if(port == USB_PHY_PORT_A){
		peri = CBUS_REG_ADDR(PREI_USB_PHY_REG_A);
	}else if(port == USB_PHY_PORT_B){
		peri = CBUS_REG_ADDR(PREI_USB_PHY_REG_B);
	}else{
		DEBUG((EFI_D_INFO,"usb base address error\n"));
        return EFI_INVALID_PARAMETER;		
	}	
	DEBUG((EFI_D_INFO,"USB peri reg base: %x\n",peri));

    //set clk_32k_alt_sel 1 to change clock source
    MmioOr32(peri+USB_AML_REGS_CONFIG,CLK_32K_ALT_SEL_MASK);
    MmioOr32(peri+USB_AML_REGS_CTRL,5<<FSEL_OFFSET|POR_MASK); /* PHY default is 24M (5), change to 12M (2) */
	MicroSecondDelay(1);
	
	return EFI_SUCCESS;
}
//call after set clock
VOID
SetUsbPhyPower(
    IN UINT32  port,
    IN BOOLEAN IsPwrOn
    )
{
	int port_idx = 100;
	UINT32 peri = 0x0;

	if(port == USB_PHY_PORT_A){
		peri = CBUS_REG_ADDR(PREI_USB_PHY_REG_A);
	}else if(port == USB_PHY_PORT_B){
		peri = CBUS_REG_ADDR(PREI_USB_PHY_REG_B);
	}
	
	if(IsPwrOn){
		MmioAnd32(peri+USB_AML_REGS_CTRL,~POR_MASK);
		MicroSecondDelay(1);
		if(MmioRead32(peri+USB_AML_REGS_CTRL)&CLK_DETECTED_MASK>>CLK_DETECTED_OFFSET){
			DEBUG((EFI_D_INFO,"USB (%d) PHY Clock not detected!\n",port_idx));
		}
	}else{
		MmioOr32(peri+USB_AML_REGS_CTRL,~POR_MASK);
	}
	MicroSecondDelay(1);
}

STATIC
EFI_STATUS
EFIAPI
DriverStart (
  IN  EFI_DRIVER_BINDING_PROTOCOL *This,
  IN  EFI_HANDLE                  Controller,
  IN  EFI_DEVICE_PATH_PROTOCOL    *RemainingDevicePath
  )
{
  VOID *Dummy;
  EFI_STATUS Status;
  DWUSB_OTGHC_DEV *DwHc = NULL;

  Status = gBS->OpenProtocol (
    Controller,
    &gEfiCallerIdGuid,
    (VOID **) &Dummy,
    This->DriverBindingHandle,
    Controller,
    EFI_OPEN_PROTOCOL_BY_DRIVER
    );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  
  SetUsbPhyClock(USB_PHY_PORT_A);
  SetUsbPhyPower(USB_PHY_PORT_A,TRUE);//on
  
  if (EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "Couldn't power on USB: %r\n", Status));
    return Status;
  }

  Status = CreateDwcOtgDevice (0xC9040000,&DwHc);
  if (EFI_ERROR (Status)) {
    goto out;
  }

  /*
   * UsbBusDxe as of b4e96b82b4e2e47e95014b51787ba5b43abac784 expects
   * the HCD to do this. There is no agent invoking DwHcReset anymore.
   */
  DwHcReset(&DwHc->DwUsbOtgHc,0);
  DwHcSetState(&DwHc->DwUsbOtgHc, EfiUsbHcStateOperational);

  Status = gBS->InstallMultipleProtocolInterfaces (
    &Controller,
    &gEfiUsb2HcProtocolGuid, &DwHc->DwUsbOtgHc,
    NULL
  );

out:
  if (EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "Could not start DwUsbHostDxe: %r\n", Status));

    DestroyDwUsbHc(DwHc);
    
    SetUsbPhyPower(USB_PHY_PORT_A,FALSE);//off

    gBS->CloseProtocol (
      Controller,
      &gEfiCallerIdGuid,
      This->DriverBindingHandle,
      Controller
      );
  }
  return Status;
}

STATIC
EFI_STATUS
EFIAPI
DriverStop (
  IN  EFI_DRIVER_BINDING_PROTOCOL *This,
  IN  EFI_HANDLE                  Controller,
  IN  UINTN                       NumberOfChildren,
  IN  EFI_HANDLE                  *ChildHandleBuffer
  )
{
  EFI_STATUS Status;
  DWUSB_OTGHC_DEV *DwHc;
  EFI_USB2_HC_PROTOCOL *HcProtocol;

  Status = gBS->HandleProtocol (
    Controller,
    &gEfiUsb2HcProtocolGuid,
    (VOID **) &HcProtocol
  );
  if (EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "DriverStop: HandleProtocol: %r\n", Status));
    return Status;
  }

  DwHc = DWHC_FROM_THIS (HcProtocol);

  Status = gBS->UninstallMultipleProtocolInterfaces (
    Controller,
    &gEfiUsb2HcProtocolGuid, &DwHc->DwUsbOtgHc,
    NULL);
  if (EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "DriverStop: UninstallMultipleProtocolInterfaces: %r\n",
           Status));
    return Status;
  }

  DwHcQuiesce (DwHc);
  DestroyDwUsbHc(DwHc);

  gBS->CloseProtocol (
    Controller,
    &gEfiCallerIdGuid,
    This->DriverBindingHandle,
    Controller
    );

  return EFI_SUCCESS;
}

/**
   UEFI Driver Entry Point API

   @param  ImageHandle       EFI_HANDLE.
   @param  SystemTable       EFI_SYSTEM_TABLE.

   @return EFI_SUCCESS       Success.
   EFI_DEVICE_ERROR  Fail.
**/

EFI_STATUS
EFIAPI
DwcOtgEntryPoint (
  IN  EFI_HANDLE ImageHandle,
  IN  EFI_SYSTEM_TABLE *SystemTable
  )
{
  EFI_STATUS Status;

  Status = gBS->InstallMultipleProtocolInterfaces (
    &mDevice,
    &gEfiDevicePathProtocolGuid, &mDevicePath,
    &gEfiCallerIdGuid, NULL,
    NULL);
  if (EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "InstallMultipleProtocolInterfaces: %r\n",
           Status));
    return Status;
  }

  Status = EfiLibInstallDriverBindingComponentName2 (
             ImageHandle,
             SystemTable,
             &mDriverBinding,
             ImageHandle,
             &gComponentName,
             &gComponentName2
             );

  if (EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "EfiLibInstallDriverBindingComponentName2: %r\n",
           Status));
    gBS->UninstallMultipleProtocolInterfaces (
       mDevice,
       &gEfiDevicePathProtocolGuid, &mDevicePath,
       &gEfiCallerIdGuid, NULL,
       NULL);
  }

  return Status;
}
