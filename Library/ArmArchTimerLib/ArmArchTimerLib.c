/** @file
  Generic ARM implementation of TimerLib.h

  Copyright (c) 2011-2016, ARM Limited. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Base.h>
#include <Library/ArmLib.h>
#include <Library/BaseLib.h>
#include <Library/TimerLib.h>
#include <Library/DebugLib.h>
#include <Library/PcdLib.h>
#include <Library/IoLib.h>
#include <Library/ArmGenericTimerCounterLib.h>

#define P_AO_TIMERE_REG 0xc8100054
#define P_AO_TIMER_REG 0xc810004c

RETURN_STATUS
EFIAPI
TimerConstructor(
    VOID
	)
{
  //Enabled AO_TIMER_E and set TimeSpan as 1us
  MmioOr32(P_AO_TIMER_REG, 0x1d);
  return RETURN_SUCCESS;
}

/**
  A local utility function that returns the PCD value, if specified.
  Otherwise it defaults to ArmGenericTimerGetTimerFreq.

  @return The timer frequency.

**/
STATIC
UINTN
EFIAPI
GetPlatformTimerFreq(
    VOID
	)
{
  UINTN TimerFreq;
  TimerFreq = ArmGenericTimerGetTimerFreq();
  DEBUG((DEBUG_INFO, "TimerFreq=%x", TimerFreq));
  return TimerFreq;
}

/**
  Stalls the CPU for the number of microseconds specified by MicroSeconds.

  @param  MicroSeconds  The minimum number of microseconds to delay.

  @return The value of MicroSeconds input.

**/
UINTN
EFIAPI
MicroSecondDelay(
    IN UINTN MicroSeconds
	)
{
  UINT32 StartTicks;

  ASSERT(MicroSeconds >= 1);

  StartTicks = MmioRead32(P_AO_TIMERE_REG);
  
  NanoSecondDelay (MicroSeconds * 1000);

  return MicroSeconds;
}

/**
  Stalls the CPU for at least the given number of nanoseconds.

  Stalls the CPU for the number of nanoseconds specified by NanoSeconds.

  When the timer frequency is 1MHz, each tick corresponds to 1 microsecond.
  Therefore, the nanosecond delay will be rounded up to the nearest 1 microsecond.

  @param  NanoSeconds The minimum number of nanoseconds to delay.

  @return The value of NanoSeconds inputed.

**/
UINTN
EFIAPI
NanoSecondDelay(
    IN UINTN NanoSeconds
	)
{
  UINT32 StartTicks;

  ASSERT(NanoSeconds >= 1);

  StartTicks = MmioRead32(P_AO_TIMERE_REG);

  while (MmioRead32(P_AO_TIMERE_REG) - StartTicks < NanoSeconds)
    ;

  return NanoSeconds;
}

/**
  Retrieves the current value of a 64-bit free running performance counter.

  The counter can either count up by 1 or count down by 1. If the physical
  performance counter counts by a larger increment, then the counter values
  must be translated. The properties of the counter can be retrieved from
  GetPerformanceCounterProperties().

  @return The current value of the free running performance counter.

**/
UINT64
EFIAPI
GetPerformanceCounter(
    VOID
	)
{
  // Just return the value of system count
  DEBUG((DEBUG_INFO, "ArmGenericTimerGetSystemCount=%x", ArmGenericTimerGetSystemCount()));
  return ArmGenericTimerGetSystemCount();
}

/**
  Retrieves the 64-bit frequency in Hz and the range of performance counter
  values.

  If StartValue is not NULL, then the value that the performance counter starts
  with immediately after is it rolls over is returned in StartValue. If
  EndValue is not NULL, then the value that the performance counter end with
  immediately before it rolls over is returned in EndValue. The 64-bit
  frequency of the performance counter in Hz is always returned. If StartValue
  is less than EndValue, then the performance counter counts up. If StartValue
  is greater than EndValue, then the performance counter counts down. For
  example, a 64-bit free running counter that counts up would have a StartValue
  of 0 and an EndValue of 0xFFFFFFFFFFFFFFFF. A 24-bit free running counter
  that counts down would have a StartValue of 0xFFFFFF and an EndValue of 0.

  @param  StartValue  The value the performance counter starts with when it
                      rolls over.
  @param  EndValue    The value that the performance counter ends with before
                      it rolls over.

  @return The frequency in Hz.

**/
UINT64
EFIAPI
GetPerformanceCounterProperties(
    OUT UINT64 *StartValue, OPTIONAL
    OUT UINT64 *EndValue    OPTIONAL
	)
{
  if (StartValue != NULL)
  {
    // Timer starts at 0
    *StartValue = (UINT64)0ULL;
  }

  if (EndValue != NULL)
  {
    // Timer counts up.
    *EndValue = 0xFFFFFFFFFFFFFFFFUL;
  }
  // TODO
  return GetPlatformTimerFreq();
}

/**
  Converts elapsed ticks of performance counter to time in nanoseconds.

  This function converts the elapsed ticks of running performance counter to
  time value in unit of nanoseconds.

  @param  Ticks     The number of elapsed ticks of running performance counter.

  @return The elapsed time in nanoseconds.

**/
UINT64
EFIAPI
GetTimeInNanoSecond(
    IN UINT64 Ticks
	)
{
  return (MmioRead32(P_AO_TIMERE_REG) - Ticks) * 1000;
}
