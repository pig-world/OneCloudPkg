/** @file
*
*  Copyright (c) 2011 - 2014, ARM Limited. All rights reserved.
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


#ifndef __REAL_TIME_CLOCK_H__
#define __REAL_TIME_CLOCK_H__

// ---------------------------
// RTC (4)
// ---------------------------
#define RTC_ADDR0                                  0x21d0
#define RTC_ADDR1                                  0x21d1
#define RTC_ADDR2                                  0x21d2
#define RTC_ADDR3                                  0x21d3
#define RTC_ADDR4                                  0x21d4

#define RTC_DBG_VAL 1 << 0
#define RTC_DBG_WR 1 << 1

// Define register (AOBUS/CBUS) RTC_ADDR0 bit map
#define RTC_REG0_BIT_sclk_static    20
#define RTC_REG0_BIT_ildo_ctrl_1    7
#define RTC_REG0_BIT_ildo_ctrl_0    6
#define RTC_REG0_BIT_test_mode      5
#define RTC_REG0_BIT_test_clk       4
#define RTC_REG0_BIT_test_bypass    3
#define RTC_REG0_BIT_sdi            2
#define RTC_REG0_BIT_sen            1
#define RTC_REG0_BIT_sclk           0

// Define register (AOBUS/CBUS) RTC_ADDR1 bit map
#define RTC_REG1_BIT_gpo_to_dig     3
#define RTC_REG1_BIT_gpi_to_dig     2
#define RTC_REG1_BIT_s_ready        1
#define RTC_REG1_BIT_sdo            0

// Define register (AOBUS/CBUS) RTC_ADDR3 bit map
#define RTC_REG3_BIT_count_always   17

// Define RTC serial protocal
#define RTC_SER_DATA_BITS           32
#define RTC_SER_ADDR_BITS           3


#define s_ready                     1 << RTC_REG1_BIT_s_ready
#define RESET_RETRY_TIMES           15

#define RTC_sbus_LOW(x)             MmioAnd32(RTC_ADDR0,  ~((1<<RTC_REG0_BIT_sen)|(1<<RTC_REG0_BIT_sclk)|(1<<RTC_REG0_BIT_sdi)))
#define RTC_sdi_HIGH(x)             MmioOr32(RTC_ADDR0, (1<<RTC_REG0_BIT_sdi))
#define RTC_sdi_LOW(x)              MmioAnd32(RTC_ADDR0, ~(1<<RTC_REG0_BIT_sdi))
#define RTC_sen_HIGH(x)             MmioOr32(RTC_ADDR0, (1<<RTC_REG0_BIT_sen))
#define RTC_sen_LOW(x)              MmioAnd32(RTC_ADDR0, ~(1<<RTC_REG0_BIT_sen))
#define RTC_sclk_HIGH(x)            MmioOr32(RTC_ADDR0, (1<<RTC_REG0_BIT_sclk))
#define RTC_sclk_LOW(x)             MmioAnd32(RTC_ADDR0,~(1<<RTC_REG0_BIT_sclk))
#define RTC_sdo_READBIT             MmioRead32(RTC_ADDR1)&(1<<RTC_REG1_BIT_sdo)
#define RTC_sclk_static_HIGH(x)     MmioOr32(RTC_ADDR0, (1<<RTC_REG0_BIT_sclk_static))
#define RTC_sclk_static_LOW(x)      MmioAnd32(RTC_ADDR0, & ~(1<<RTC_REG0_BIT_sclk_static))
#define RTC_count_always_HIGH(x)    MmioOr32(RTC_ADDR3, (1<<RTC_REG3_BIT_count_always))
#define RTC_count_always_LOW(x)     MmioAnd32(RTC_ADDR3, ~(1<<RTC_REG3_BIT_count_always))


#define RTC_SER_REG_DATA_NOTIFIER   0xb41b// Define RTC register address mapping

//#define P_ISA_TIMERE                (volatile unsigned long *)0xc1109954

// Define RTC register address mapping
#define RTC_COUNTER_ADDR            0
#define RTC_GPO_COUNTER_ADDR        1
#define RTC_SEC_ADJUST_ADDR         2
#define RTC_UNUSED_ADDR_0           3
#define RTC_REGMEM_ADDR_0           4
#define RTC_REGMEM_ADDR_1           5
#define RTC_REGMEM_ADDR_2           6
#define RTC_REGMEM_ADDR_3           7

#endif
