/** @file
*
*  Copyright (c) 2011-2016, ARM Limited. All rights reserved.
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

#include <Library/IoLib.h>

#ifndef __UARTSERIALPORT_LIB_H__
#define __UARTSERIALPORT_UART_H__

#define IO_CBUS_BASE			0xc1100000
#define IO_AXI_BUS_BASE			0xc1300000
#define IO_AHB_BUS_BASE			0xc9000000
#define IO_APB_BUS_BASE			0xc8000000
#define IO_USB_A_BASE			0xc9040000
#define IO_USB_B_BASE			0xc90C0000

#define MESON_PERIPHS1_VIRT_BASE	0xc1108400
#define MESON_PERIPHS1_PHYS_BASE	0xc1108400

#define CONFIG_BAUDRATE 115200
#define CONFIG_CRYSTAL_MHZ 24

#define P_AO_RTI_PIN_MUX_REG         (0xc8100000 | (0x00 << 10) | (0x05 << 2))
// ----------------------------
// UART
// ----------------------------
#define P_AO_UART_WFIFO              (0xc8100000 | (0x01 << 10) | (0x30 << 2))
#define P_AO_UART_RFIFO              (0xc8100000 | (0x01 << 10) | (0x31 << 2))
#define P_AO_UART_CONTROL            (0xc8100000 | (0x01 << 10) | (0x32 << 2))
#define P_AO_UART_STATUS             (0xc8100000 | (0x01 << 10) | (0x33 << 2))
#define P_AO_UART_MISC               (0xc8100000 | (0x01 << 10) | (0x34 << 2))

#define CBUS_REG_OFFSET(reg) ((reg) << 2)
#define CBUS_REG_ADDR(reg)	 (IO_CBUS_BASE + CBUS_REG_OFFSET(reg))

#define AXI_REG_OFFSET(reg)  ((reg) << 2)
#define AXI_REG_ADDR(reg)	 (IO_AXI_BUS_BASE + AXI_REG_OFFSET(reg))

#define AHB_REG_OFFSET(reg)  ((reg) << 2)
#define AHB_REG_ADDR(reg)	 (IO_AHB_BUS_BASE + AHB_REG_OFFSET(reg))

#define APB_REG_OFFSET(reg)  (reg)
#define APB_REG_ADDR(reg)	 (IO_APB_BUS_BASE + APB_REG_OFFSET(reg))
#define APB_REG_ADDR_VALID(reg) (((unsigned long)(reg) & 3) == 0)


#define UART_PORT_0     CBUS_REG_ADDR(UART0_WFIFO)
#define UART_PORT_1     CBUS_REG_ADDR(UART1_WFIFO)
#define UART_PORT_2     CBUS_REG_ADDR(UART2_WFIFO)
#define UART_PORT_AO    P_AO_UART_WFIFO

#define UART_WFIFO      (0<<2)
#define UART_RFIFO      (1<<2)
#define UART_CONTROL    (2<<2)
#define UART_STATUS     (3<<2)
#define UART_MISC       (4<<2)

#define P_UART(uart_base,reg)    	(uart_base+reg)
#define P_UART_WFIFO(uart_base)   	P_UART(uart_base,UART_WFIFO)
#define P_UART_RFIFO(uart_base)   	P_UART(uart_base,UART_RFIFO)

#define P_UART_CONTROL(uart_base)    P_UART(uart_base,UART_CONTROL)
    #define UART_CNTL_MASK_BAUD_RATE                (0xfff)
    #define UART_CNTL_MASK_TX_EN                    (1<<12)
    #define UART_CNTL_MASK_RX_EN                    (1<<13)
    #define UART_CNTL_MASK_2WIRE                    (1<<15)
    #define UART_CNTL_MASK_STP_BITS                 (3<<16)
    #define UART_CNTL_MASK_STP_1BIT                 (0<<16)
    #define UART_CNTL_MASK_STP_2BIT                 (1<<16)
    #define UART_CNTL_MASK_PRTY_EVEN                (0<<18)
    #define UART_CNTL_MASK_PRTY_ODD                 (1<<18)
    #define UART_CNTL_MASK_PRTY_TYPE                (1<<18)
    #define UART_CNTL_MASK_PRTY_EN                  (1<<19)
    #define UART_CNTL_MASK_CHAR_LEN                 (3<<20)
    #define UART_CNTL_MASK_CHAR_8BIT                (0<<20)
    #define UART_CNTL_MASK_CHAR_7BIT                (1<<20)
    #define UART_CNTL_MASK_CHAR_6BIT                (2<<20)
    #define UART_CNTL_MASK_CHAR_5BIT                (3<<20)
    #define UART_CNTL_MASK_RST_TX                   (1<<22)
    #define UART_CNTL_MASK_RST_RX                   (1<<23)
    #define UART_CNTL_MASK_CLR_ERR                  (1<<24)
    #define UART_CNTL_MASK_INV_RX                   (1<<25)
    #define UART_CNTL_MASK_INV_TX                   (1<<26)
    #define UART_CNTL_MASK_RINT_EN                  (1<<27)
    #define UART_CNTL_MASK_TINT_EN                  (1<<28)
    #define UART_CNTL_MASK_INV_CTS                  (1<<29)
    #define UART_CNTL_MASK_MASK_ERR                 (1<<30)
    #define UART_CNTL_MASK_INV_RTS                  (1<<31)

#define P_UART_STATUS(uart_base)  P_UART(uart_base,UART_STATUS )
    #define UART_STAT_MASK_RFIFO_CNT                (0x7f<<0)
    #define UART_STAT_MASK_TFIFO_CNT                (0x7f<<8)
    #define UART_STAT_MASK_PRTY_ERR                 (1<<16)
    #define UART_STAT_MASK_FRAM_ERR                 (1<<17)
    #define UART_STAT_MASK_WFULL_ERR                (1<<18)
    #define UART_STAT_MASK_RFIFO_FULL               (1<<19)
    #define UART_STAT_MASK_RFIFO_EMPTY              (1<<20)
    #define UART_STAT_MASK_TFIFO_FULL               (1<<21)
    #define UART_STAT_MASK_TFIFO_EMPTY              (1<<22)
    #define UART_STAT_MASK_CTS_LEVEL                (1<<23)
    #define UART_STAT_MASK_RECV_FIFO_OVERFLOW       (1<<24)
    #define UART_STAT_MASK_UART_XMIT_BUSY           (1<<25)
    #define UART_STAT_MASK_UART_RECV_BUSY           (1<<26)

#ifndef CONFIG_SERIAL_STP_BITS

#define CONFIG_SERIAL_STP_BITS 1 // 1 , 2

#endif

#if CONFIG_SERIAL_STP_BITS==1

#define UART_STP_BIT UART_CNTL_MASK_STP_1BIT

#elif CONFIG_SERIAL_STP_BITS==2

#define UART_STP_BIT UART_CNTL_MASK_STP_2BIT

#else

#error CONFIG_SERIAL_STP_BITS wrong

#endif





#ifndef CONFIG_SERIAL_PRTY_TYPE

#define CONFIG_SERIAL_PRTY_TYPE 0 //0 ,2 ,3

#endif

#if CONFIG_SERIAL_PRTY_TYPE==0

#define UART_PRTY_BIT 0

#elif CONFIG_SERIAL_PRTY_TYPE==2

#define UART_PRTY_BIT    (UART_CNTL_MASK_PRTY_EN|UART_CNTL_MASK_PRTY_EVEN)

#elif CONFIG_SERIAL_PRTY_TYPE==3

#define UART_PRTY_BIT    (UART_CNTL_MASK_PRTY_EN|UART_CNTL_MASK_PRTY_ODD)

#else

#error CONFIG_SERIAL_PRTY_TYPE wrong

#endif



#ifndef CONFIG_SERIAL_CHAR_LEN

#define CONFIG_SERIAL_CHAR_LEN 8 //5,6,7,8

#endif

#if CONFIG_SERIAL_CHAR_LEN==5

#define UART_CHAR_LEN   UART_CNTL_MASK_CHAR_5BIT

#elif CONFIG_SERIAL_CHAR_LEN==6

#define UART_CHAR_LEN   UART_CNTL_MASK_CHAR_6BIT

#elif CONFIG_SERIAL_CHAR_LEN==7

#define UART_CHAR_LEN   UART_CNTL_MASK_CHAR_7BIT

#elif CONFIG_SERIAL_CHAR_LEN==8

#define UART_CHAR_LEN   UART_CNTL_MASK_CHAR_8BIT

#else

#error CONFIG_SERIAL_CHAR_LEN wrong

#endif

#define P_UART_MISC(uart_base)    P_UART(uart_base,UART_MISC   )

#define SPL_STATIC_FUNC

#define WATCHDOG_TC                                0x2640
#define WATCHDOG_RESET                             0x2641

#define P_WATCHDOG_TC					CBUS_REG_ADDR(WATCHDOG_TC)
#define P_WATCHDOG_RESET      CBUS_REG_ADDR(WATCHDOG_RESET)

#define setbits_le32(reg,mask)  MmioWrite32(MmioRead32(reg)|(mask),reg)
#define clrbits_le32(reg,mask)  MmioWrite32(MmioRead32(reg)&(~(mask)),reg)
#define clrsetbits_le32(reg,clr,mask)  MmioWrite32((MmioRead32(reg)&(~(clr)))|(mask),reg)

#endif
