/** @file
*
*  Copyright (c) 2011-2012, ARM Limited. All rights reserved.
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


#ifndef __WATCHDOG_H__
#define __WATCHDOG_H__

#define P_WATCHDOG_TC		0xc1109900
#define P_WATCHDOG_RESET	0xc1109904

//max watchdog timer: 8.388s
#define AML_WATCHDOG_TIME_SLICE				128	//us
#define AML_WATCHDOG_ENABLE_OFFSET			19
#define AML_WATCHDOG_CPU_RESET_CNTL			0xf	//qual-core
#define AML_WATCHDOG_CPU_RESET_OFFSET		24

#endif  // __WATCHDOG_H__
