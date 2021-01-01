/** @file
  Serial I/O Port library functions with no library constructor/destructor

  Copyright (c) 2008 - 2010, Apple Inc. All rights reserved.<BR>
  Copyright (c) 2012 - 2016, ARM Ltd. All rights reserved.<BR>
  Copyright (c) 2015, Intel Corporation. All rights reserved.<BR>

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Base.h>

#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/DebugLib.h>
#include <Library/SerialPortLib.h>

#include "UartSerialPortLib.h"

INT32
 UART_CONTROL_SET(
   UINT32   baud,
   UINT32   clk81
   )
 {
    return (clk81/(baud*4) -1)        \
    | UART_STP_BIT                    \
    | UART_PRTY_BIT                   \
    | UART_CHAR_LEN                   \
    | UART_CNTL_MASK_TX_EN            \
    | UART_CNTL_MASK_RX_EN            \
    | UART_CNTL_MASK_RST_TX           \
    | UART_CNTL_MASK_RST_RX           \
    | UART_CNTL_MASK_CLR_ERR;
 }  

/** Initialise the serial device hardware with default settings.

  @retval RETURN_SUCCESS            The serial device was initialised.
  @retval RETURN_INVALID_PARAMETER  One or more of the default settings
                                    has an unsupported value.
 **/
RETURN_STATUS
EFIAPI
SerialPortInitialize (
  VOID
  )
{
  UINT32 UartBase = (UINT32)FixedPcdGet32(PcdSerialRegisterBase);
  UINT32 ControlRegister= MmioRead32(P_UART_CONTROL(UartBase));
  
  MmioWrite32(P_UART_CONTROL(UartBase),0);
  UART_CONTROL_SET(CONFIG_BAUDRATE,CONFIG_CRYSTAL_MHZ*1000000); 
  
  MmioWrite32(
    (P_UART_CONTROL(UartBase)),
      ControlRegister
	    |UART_CNTL_MASK_RST_TX
	    |UART_CNTL_MASK_RST_RX
	    |UART_CNTL_MASK_CLR_ERR
	    |UART_CNTL_MASK_TX_EN
	    |UART_CNTL_MASK_RX_EN
	);
	
    setbits_le32(P_AO_RTI_PIN_MUX_REG,3<<11);
    clrbits_le32(P_UART_CONTROL(UartBase),
	    UART_CNTL_MASK_RST_TX | UART_CNTL_MASK_RST_RX | UART_CNTL_MASK_CLR_ERR);
  return RETURN_SUCCESS;
}

/**
  Write data to serial device.

  @param  Buffer           Point of data buffer which need to be written.
  @param  NumberOfBytes    Number of output bytes which are cached in Buffer.

  @retval 0                Write data failed.
  @retval !0               Actual number of bytes written to serial device.

**/
UINTN
EFIAPI
SerialPortWrite (
  IN UINT8     *Buffer,
  IN UINTN     NumberOfBytes
  )
{
  UINT32 UartBase = (UINT32)FixedPcdGet32(PcdSerialRegisterBase);
  UINTN BytesSent;

  for (BytesSent = 0; BytesSent < NumberOfBytes; BytesSent++)
  {
    /* Wait till dataTx register is not full */
    while ((MmioRead32(P_UART_STATUS(UartBase)) & UART_STAT_MASK_TFIFO_FULL)) ;
    /* Send data */
    MmioWrite32(P_UART_WFIFO(UartBase), Buffer[BytesSent]);
    /* Wait till dataTx register is empty */
    while (!(MmioRead32(P_UART_STATUS(UartBase)) & UART_STAT_MASK_TFIFO_EMPTY)) ;
  }
  return BytesSent;
}

/**
  Read data from serial device and save the data in buffer.

  @param  Buffer           Point of data buffer which need to be written.
  @param  NumberOfBytes    Number of output bytes which are cached in Buffer.

  @retval 0                Read data failed.
  @retval !0               Actual number of bytes read from serial device.

**/
UINTN
EFIAPI
SerialPortRead (
  OUT UINT8     *Buffer,
  IN  UINTN     NumberOfBytes
)
{
  UINT32 UartBase = (UINT32)FixedPcdGet32(PcdSerialRegisterBase);
  UINTN BytesRead;

  for (BytesRead = 0; BytesRead < NumberOfBytes; BytesRead++,Buffer++)
  {

    /* Wait till character is placed in fifo */
    while ((MmioRead32(P_UART_STATUS(UartBase)) & UART_STAT_MASK_RFIFO_CNT) == 0) ;

    /* Also check for overflow errors */
    if (MmioRead32(P_UART_STATUS(UartBase)) & (UART_STAT_MASK_PRTY_ERR | UART_STAT_MASK_FRAM_ERR))
    {
      setbits_le32(P_UART_CONTROL(UartBase), UART_CNTL_MASK_CLR_ERR);
      clrbits_le32(P_UART_CONTROL(UartBase), UART_CNTL_MASK_CLR_ERR);
    }

    *Buffer = MmioRead32(P_UART_RFIFO(UartBase)) & 0x00ff;
  }
  return BytesRead;
}

/**
  Check to see if any data is available to be read from the debug device.

  @retval TRUE       At least one byte of data is available to be read
  @retval FALSE      No data is available to be read

**/
BOOLEAN
EFIAPI
SerialPortPoll (
  VOID
  )
{
  UINT32 UartBase = (UINT32)FixedPcdGet32(PcdSerialRegisterBase);
  return (MmioRead32(P_UART_STATUS(UartBase)) & UART_STAT_MASK_RFIFO_CNT)==1?TRUE:FALSE;
}
/**
  Set new attributes to PL011.

  @param  BaudRate                The baud rate of the serial device. If the
                                  baud rate is not supported, the speed will
                                  be reduced down to the nearest supported one
                                  and the variable's value will be updated
                                  accordingly.
  @param  ReceiveFifoDepth        The number of characters the device will
                                  buffer on input. If the specified value is
                                  not supported, the variable's value will
                                  be reduced down to the nearest supported one.
  @param  Timeout                 If applicable, the number of microseconds the
                                  device will wait before timing out a Read or
                                  a Write operation.
  @param  Parity                  If applicable, this is the EFI_PARITY_TYPE
                                  that is computed or checked as each character
                                  is transmitted or received. If the device
                                  does not support parity, the value is the
                                  default parity value.
  @param  DataBits                The number of data bits in each character
  @param  StopBits                If applicable, the EFI_STOP_BITS_TYPE number
                                  of stop bits per character. If the device
                                  does not support stop bits, the value is the
                                  default stop bit value.

  @retval EFI_SUCCESS             All attributes were set correctly.
  @retval EFI_INVALID_PARAMETERS  One or more attributes has an unsupported
                                  value.

**/
RETURN_STATUS
EFIAPI
SerialPortSetAttributes (
  IN OUT UINT64              *BaudRate,
  IN OUT UINT32              *ReceiveFifoDepth,
  IN OUT UINT32              *Timeout,
  IN OUT EFI_PARITY_TYPE     *Parity,
  IN OUT UINT8               *DataBits,
  IN OUT EFI_STOP_BITS_TYPE  *StopBits
  )
{
  UINT32 UartBase = (UINT32)FixedPcdGet32(PcdSerialRegisterBase);
  UINT32 ControlRegister= MmioRead32(P_UART_CONTROL(UartBase));

  //
  // Parity
  //
  switch (*Parity) {
      case DefaultParity:
        *Parity = NoParity;
      case NoParity:
        ControlRegister |= UART_CNTL_MASK_PRTY_EN;
        break;
      case EvenParity:
        ControlRegister |= UART_CNTL_MASK_PRTY_EVEN;
        break;
      case OddParity:
        ControlRegister |= UART_CNTL_MASK_PRTY_ODD;
        break;
      case MarkParity:
      case SpaceParity:
      default:
        return RETURN_INVALID_PARAMETER;
  }

  //
  // Data Bits
  //
  switch (*DataBits) {
        case 5:
            ControlRegister |= UART_CNTL_MASK_CHAR_5BIT;
            break;
        case 6:
            ControlRegister |= UART_CNTL_MASK_CHAR_6BIT;
            break;
        case 7:
            ControlRegister |= UART_CNTL_MASK_CHAR_7BIT;
            break;
        case 8:
        default:
            ControlRegister |= UART_CNTL_MASK_CHAR_8BIT;
            break;
  }

  //
  // Stop Bits
  //
  switch (*StopBits)
  {
    case DefaultStopBits:
      *StopBits = OneStopBit;
    case OneStopBit:
      ControlRegister |= UART_CNTL_MASK_STP_1BIT;
      break;
    case TwoStopBits:
      ControlRegister |= UART_CNTL_MASK_STP_2BIT;
      break;
    case OneFiveStopBits:
      // Only 1 or 2 stop bits are supported
    default:
      return RETURN_INVALID_PARAMETER;
  }

  // Don't send the LineControl value to the PL011 yet,
  // wait until after the Baud Rate setting.
  // This ensures we do not mess up the UART settings halfway through
  // in the rare case when there is an error with the Baud Rate.

  //
  // Baud Rate
  // Not Support For Now
  //

  MmioWrite32(P_UART_CONTROL(UartBase),ControlRegister);
  return RETURN_SUCCESS;
}

/**
  Sets the control bits on a serial device.

  @param Control                Sets the bits of Control that are settable.

  @retval RETURN_SUCCESS        The new control bits were set on the serial device.
  @retval RETURN_UNSUPPORTED    The serial device does not support this operation.
  @retval RETURN_DEVICE_ERROR   The serial device is not functioning correctly.

**/
RETURN_STATUS
EFIAPI
SerialPortSetControl (
  IN UINT32  Control
  )
{
  UINT32 UartBase = (UINT32)FixedPcdGet32(PcdSerialRegisterBase);

  MmioWrite32(P_UART_CONTROL(UartBase),Control);

  return RETURN_SUCCESS;
}

/**
  Retrieve the status of the control bits on a serial device.

  @param Control                A pointer to return the current control signals from the serial device.

  @retval RETURN_SUCCESS        The control bits were read from the serial device.
  @retval RETURN_UNSUPPORTED    The serial device does not support this operation.
  @retval RETURN_DEVICE_ERROR   The serial device is not functioning correctly.

**/
RETURN_STATUS
EFIAPI
SerialPortGetControl (
  OUT UINT32  *Control
  )
{
  UINT32 UartBase = (UINT32)FixedPcdGet32(PcdSerialRegisterBase);

  *Control = MmioRead32 (P_UART_CONTROL(UartBase));

  return RETURN_SUCCESS;
}
