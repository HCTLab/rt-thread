/*----------------------------------------------------------------------------
 * Name:    system_RV32M1_VEGA.c
 * Purpose: RV32M1_VEGA clock initialisation
 * Version: V2.51
 * Note(s):
 *----------------------------------------------------------------------------
 * This file is part of the uVision/ARM development tools.
 * This software may only be used under the terms of a valid, current,
 * end user licence from KEIL for a compatible version of KEIL software
 * development tools. Nothing else gives you the right to use this software.
 *
 * This software is supplied "AS IS" without warranties of any kind.
 *
 * Copyright (c) 2005-2012 Keil Software. All rights reserved.
 *----------------------------------------------------------------------------*/

#include "RV32M1.h"

#define DEFAULT_SYSTEM_CLOCK           48000000u           /* Default System clock value */

/* ----------------------------------------------------------------------------
   -- Core clock
   ---------------------------------------------------------------------------- */
uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;

/* ----------------------------------------------------------------------------
   -- SystemInitHook()
   ---------------------------------------------------------------------------- */

__attribute__ ((weak)) void SystemInitHook (void) {
  /* Void implementation of the weak function. */
}


/* ----------------------------------------------------------------------------
   -- SystemInit()
   ---------------------------------------------------------------------------- */

void SystemInit (void) {

  // BSS and SDATA sections are initialized by startup_RV32M1_CMX code

  SystemInitHook();

#ifdef BOOT_PROCESSOR
  #if (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2) |                 /* set CP10 Full Access */
                   (3UL << 11*2)  );               /* set CP11 Full Access */
  #endif

  /* Disable SysTick timer                                                    */
  SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk);
#endif
}
