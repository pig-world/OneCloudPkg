/** @file
*
*  Implement EFI RealTimeClock runtime services based on ARM Performance Counter.
*
*  Currently this driver does not support time setting, alarms, or runtime calls.
*  This special library is NOT meant to replace a HW RTC implementation to
*  measure date/time. Use this library ONLY to measure relative time between
*  two EFI_GET_TIME readings.
*  The performance counter will wrap-around eventually after a long time, make
*  sure to consider this limitation if you are depending on this library for
*  relative time measurement. e.g. For the ARM 64-bit counter with 19.2MHz
*  frequency, the counter will wrap-around after approximately 30465 year.
*
*  Copyright (c) 2018 Microsoft Corporation. All rights reserved.
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

#include <PiDxe.h>

#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/RealTimeClockLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>

#include "RealTimeClock.h"

EFI_STATUS
CheckOscClk()
{
	UINT32   osc_clk_count1; 
	UINT32   osc_clk_count2; 

	MmioOr32(RTC_ADDR3,(1 << 17));   // Enable count always    

	DEBUG((DEBUG_INFO,  "aml_rtc -- check os clk_1\n"));
	osc_clk_count1 = MmioRead32(RTC_ADDR2);    // Wait for 50uS.  32.768khz is 30.5uS.  This should be long   
										// enough for one full cycle of 32.768 khz   
	DEBUG((DEBUG_INFO, "the aml_rtc os clk 1 is %d\n", osc_clk_count1));									
	MicroSecondDelay( 50 );   
	osc_clk_count2 = MmioRead32(RTC_ADDR2);    
	DEBUG((DEBUG_INFO,  "the aml_rtc os clk 2 is %d\n", osc_clk_count2));

	DEBUG((DEBUG_INFO,  "aml_rtc -- check os clk_2\n"));
	MmioAnd32(RTC_ADDR3, ~(1 << 17));  // disable count always    

	if( osc_clk_count1 == osc_clk_count2 ) { 
	       DEBUG((DEBUG_INFO,  "The osc_clk is not running now! need to invcrease the power!\n"));
		return EFI_DEVICE_ERROR; 
	}

    DEBUG((DEBUG_INFO,   "aml_rtc : check_os_clk\n"));
	   
	return EFI_SUCCESS;

}

VOID
RtcSclkPulse()
{
	MicroSecondDelay(1);
	RTC_sclk_HIGH(1);
	MicroSecondDelay(1);
	RTC_sclk_LOW(0);

}
INT32
RtcWaitSReady()
{
	INT32 i = 40000;
	INT32 try_cnt = 0;
	/*
	while (i--){
		if((*(volatile unsigned *)RTC_ADDR1)&s_ready)
			break;
		}
	return i;
	*/
	while(!(MmioRead32(RTC_ADDR1)&s_ready)){
		i--;
		if(i == 0){
				if(try_cnt > RESET_RETRY_TIMES){
						break;
					}
			  MicroSecondDelay(100);
			  try_cnt++;
			  i = 40000;
			}
	}
	
	return i;
}
EFI_STATUS
RtcCommInit()
{
	RTC_sbus_LOW(0);
	if(RtcWaitSReady()>0){
		RTC_sen_HIGH(1);
		return EFI_SUCCESS;
	}
	return EFI_DEVICE_ERROR;
}


VOID 
RtcSendBit(
	UINT32 val
	)
{
	if (val)
		RTC_sdi_HIGH(1);
	else
		RTC_sdi_LOW(0);
	RtcSclkPulse();
}

VOID
RtcSendAddrData(
	UINT8 type,
	UINT32 val
	)
{
	UINT32 cursor = (type? (1<<(RTC_SER_ADDR_BITS-1)) : (1<<(RTC_SER_DATA_BITS-1)));
		
	while(cursor)
	{
		RtcSendBit(val&cursor);
		cursor >>= 1;
	}
}
VOID
RtcSetMode(UINT8 mode)
{
	RTC_sen_LOW(0);
	if (mode)
		RTC_sdi_HIGH (1);//WRITE
	else
		RTC_sdi_LOW(0);  //READ
	RtcSclkPulse();
	RTC_sdi_LOW(0);
}

VOID
RtcGetData(UINT32 *val)
{
	DEBUG((DEBUG_INFO, "rtc-aml -- rtc get data \n"));
	for (UINT8 i=0; i<RTC_SER_DATA_BITS; i++)
	{
		RtcSclkPulse();
		*val <<= 1;
		*val  |= RTC_sdo_READBIT;
	}
}


UINT32
SerAccessRead(UINT8 addr)
{
	UINT32 val = 0;
	UINT32 s_nrdy_cnt = 0;

	DEBUG((DEBUG_INFO, "aml_rtc --SerAccessRead_1\n"));
	if(EFI_ERROR (CheckOscClk())){
		DEBUG((DEBUG_INFO, "aml_rtc -- the osc clk does not work\n"));
		return val;
	}

	DEBUG((DEBUG_INFO,  "aml_rtc -- SerAccessRead_2\n"));
	while(EFI_ERROR(RtcCommInit())){
		DEBUG((DEBUG_INFO, "aml_rtc -- rtc_common_init fail\n"));
		if(s_nrdy_cnt>RESET_RETRY_TIMES)
			return val;
		MicroSecondDelay(100);
		s_nrdy_cnt++;
	}

	DEBUG((DEBUG_INFO,  "aml_rtc -- SerAccessRead_3\n"));
	RtcSendAddrData(1,addr);
	RtcSetMode(0); //Read
	RtcGetData(&val);

	return val;
}

INT32
SerAccessWrite(UINT8 addr, UINT64 data)
{
	INT32 s_nrdy_cnt = 0;

	while(RtcCommInit()<0){
		
		if(s_nrdy_cnt>RESET_RETRY_TIMES)
			return -1;
		MicroSecondDelay(100);
		s_nrdy_cnt++;
	}
	RtcSendAddrData(0,data);
	RtcSendAddrData(1,addr);
	RtcSetMode(1); //Write
	
	RtcWaitSReady();

	return 0;
}

/**
  Returns the current time and date information, and the time-keeping capabilities
  of the virtual RTC.

  For simplicity, this LibGetTime does not report Years/Months, instead it will
  only report current Day, Hours, Minutes and Seconds starting from the beginning
  of CPU up-time. Otherwise, a more complex logic will be required to account
  for leap years and days/month differences.

  @param  Time                  A pointer to storage to receive a snapshot of
                                the current time.
  @param  Capabilities          An optional pointer to a buffer to receive the
                                real time clock device's capabilities.

  @retval EFI_SUCCESS           The operation completed successfully.
  @retval EFI_INVALID_PARAMETER Time is NULL.
  @retval EFI_DEVICE_ERROR      The time could not be retrieved due to hardware error.

**/
EFI_STATUS
EFIAPI
LibGetTime (
  OUT EFI_TIME *Time,
  OUT EFI_TIME_CAPABILITIES *Capabilities
  )
{
  UINT64 ElapsedSeconds;

  if (Time == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  // Depend on ARM ARCH Timer (i.e. performance counter) to report date/time
  // relative to the start of CPU timer counting where date and time will always
  // be relative to the date/time 1/1/1900 00H:00M:00S

  if (Capabilities) {
    Capabilities->Accuracy = 0;
    Capabilities->Resolution = 800000;
    Capabilities->SetsToZero = FALSE;
  }

	
    DEBUG((DEBUG_INFO, "LibGetTime: read rtc time\n"));

    ElapsedSeconds = SerAccessRead(RTC_COUNTER_ADDR);

    DEBUG((DEBUG_INFO, "LibGetTime: have read the rtc time, time is %d\n", ElapsedSeconds));

  // Don't report Year/Month since Leap Year logic is not implemented. This should
  // be fine since the sole purpose of this special implementation is to be
  // used for relative time measurement. e.g. Windows Boot Manager.
  Time->Year = 0;
  Time->Month = 0;

  CONST UINT64 SECONDS_PER_DAY = 24 * 60 * 60;
  Time->Day = (ElapsedSeconds / SECONDS_PER_DAY);
  ElapsedSeconds %= SECONDS_PER_DAY;

  CONST UINT64 SECONDS_PER_HOUR = 60 * 60;
  Time->Hour = (ElapsedSeconds / SECONDS_PER_HOUR);
  ElapsedSeconds %= SECONDS_PER_HOUR;

  CONST UINT64 SECONDS_PER_MINUTE = 60;
  Time->Minute = (ElapsedSeconds / SECONDS_PER_MINUTE);
  ElapsedSeconds %= SECONDS_PER_MINUTE;

  Time->Second = ElapsedSeconds;
  Time->Nanosecond = 0;
  Time->TimeZone = 0;
  Time->Daylight = 0;
  Time->Daylight = 0;

  return EFI_SUCCESS;
}

/**
  Sets the current local time and date information.

  @param  Time                  A pointer to the current time.

  @retval EFI_UNSUPPORTED      This operation is not supported.

**/
EFI_STATUS
EFIAPI
LibSetTime (
  IN EFI_TIME *Time
  )
{
	UINT64 ElapsedSeconds;
	UINT8 Month = Time->Month;
	UINT8 Year = Time->Year;

	/* 1..12 -> 11,12,1..10 */
	if (0 >= (Month -= 2)) {
		Month += 12;	/* Puts Feb last since it has leap day */
		Year -= 1;
	}

	ElapsedSeconds=((((unsigned long)
		  (Year/4 - Year/100 + Year/400 + 367*Month/12 + Time->Day) +
		  Year*365 - 719499
	    )*24 + Time->Hour /* now have hours */
	  )*60 + Time->Minute /* now have minutes */
	)*60 + Time->Second; /* finally seconds */
     
     DEBUG((DEBUG_INFO, "aml_rtc : write the rtc time, time is %ld\n", ElapsedSeconds));
      SerAccessWrite(RTC_COUNTER_ADDR, ElapsedSeconds);
      DEBUG((DEBUG_INFO, "aml_rtc : the time has been written\n"));
      //spin_unlock(&priv->lock);  

  return EFI_UNSUPPORTED;
}

/**
  Returns the current wakeup alarm clock setting.

  @param  Enabled               Indicates if the alarm is currently enabled or
                                disabled.
  @param  Pending               Indicates if the alarm signal is pending and
                                requires acknowledgement.
  @param  Time                  The current alarm setting.

  @retval EFI_UNSUPPORTED       A wakeup timer is not supported on this platform.

**/
EFI_STATUS
EFIAPI
LibGetWakeupTime (
  OUT BOOLEAN *Enabled,
  OUT BOOLEAN *Pending,
  OUT EFI_TIME *Time
  )
{
  return EFI_UNSUPPORTED;
}

/**
  Sets the system wakeup alarm clock time.

  @param  Enabled               Enable or disable the wakeup alarm.
  @param  Time                  If Enable is TRUE, the time to set the wakeup alarm for.

  @retval EFI_UNSUPPORTED       A wakeup timer is not supported on this platform.

**/
EFI_STATUS
EFIAPI
LibSetWakeupTime (
  IN BOOLEAN Enabled,
  OUT EFI_TIME *Time
  )
{
  return EFI_UNSUPPORTED;
}

/**
  This is the declaration of an EFI image entry point. This can be the entry point
  to an application written to this specification, an EFI boot service driver,
  or an EFI runtime driver.

  @param  ImageHandle           Handle that identifies the loaded image.
  @param  SystemTable           System Table for this image.

  @retval EFI_SUCCESS           The operation completed successfully.

**/
EFI_STATUS
EFIAPI
LibRtcInitialize (
  IN EFI_HANDLE ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  // ARM ARCH Timer is already initialized in the SEC/PEI phase.
  return EFI_SUCCESS;
}

/**
  Fixup internal data so that EFI can be call in virtual mode.
  Call the passed in Child Notify event and convert any pointers in
  lib to virtual mode.

  @param[in]    Event   The Event that is being processed
  @param[in]    Context Event Context
**/
VOID
EFIAPI
LibRtcVirtualNotifyEvent (
  IN EFI_EVENT Event,
  IN VOID *Context
  )
{
  // Not supporting OS calling RTC functions in virtual mode.
  return;
}