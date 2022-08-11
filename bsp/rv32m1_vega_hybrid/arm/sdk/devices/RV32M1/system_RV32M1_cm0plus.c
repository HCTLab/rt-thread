/*
** ###################################################################
**     Processors:          RV32M1_cm0plus
**                          RV32M1_cm0plus
**
**     Compilers:           Keil ARM C/C++ Compiler
**                          GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**                          MCUXpresso Compiler
**
**     Reference manual:    RV32M1 Series Reference Manual, Rev. 1 , 8/10/2018
**     Version:             rev. 1.0, 2018-10-02
**     Build:               b170713
**
**     Abstract:
**         Provides a system configuration function and a global variable that
**         contains the system frequency. It configures the device and initializes
**         the oscillator (PLL) that is part of the microcontroller device.
**
**     Copyright 2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2016 NXP
**     All rights reserved.
**
**     SPDX-License-Identifier: BSD-3-Clause
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
**     Revisions:
**     - rev. 1.0 (2018-10-02)
**         Initial version.
**
** ###################################################################
*/

/*!
 * @file RV32M1_cm0plus
 * @version 1.0
 * @date 2018-10-02
 * @brief Device specific configuration file for RV32M1_cm0plus
 *        (implementation file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#include <stdint.h>
#include "fsl_device_registers.h"
#include "fsl_common.h"


/* ----------------------------------------------------------------------------
   -- Core clock
   ---------------------------------------------------------------------------- */

uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;

/* ----------------------------------------------------------------------------
   -- SystemInit()
   ---------------------------------------------------------------------------- */

void SystemInit (void) {
#if ((__FPU_PRESENT == 1) && (__FPU_USED == 1))
  SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));    /* set CP10, CP11 Full Access */
#endif /* ((__FPU_PRESENT == 1) && (__FPU_USED == 1)) */

#if (DISABLE_WDOG)
  WDOG1->CNT = 0xD928C520U;
  WDOG1->TOVAL = 0xFFFF;
  WDOG1->CS = (uint32_t) ((WDOG1->CS) & ~WDOG_CS_EN_MASK) | WDOG_CS_UPDATE_MASK;
#endif /* (DISABLE_WDOG) */
}

/* ----------------------------------------------------------------------------
   -- SystemCoreClockUpdate()
   ---------------------------------------------------------------------------- */

void SystemCoreClockUpdate (void) {

  uint32_t SCGOUTClock;                                 /* Variable to store output clock frequency of the SCG module */
  uint16_t Divider;
  Divider = ((SCG->CSR & SCG_CSR_DIVCORE_MASK) >> SCG_CSR_DIVCORE_SHIFT) + 1;

  switch ((SCG->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT) {
    case 0x1:
      /* System OSC */
      SCGOUTClock = CPU_XTAL_CLK_HZ;
      break;
    case 0x2:
      /* Slow IRC */
      SCGOUTClock = (((SCG->SIRCCFG & SCG_SIRCCFG_RANGE_MASK) >> SCG_SIRCCFG_RANGE_SHIFT) ? 8000000 : 2000000);
      break;
    case 0x3:
      /* Fast IRC */
      SCGOUTClock = 48000000 + ((SCG->FIRCCFG & SCG_FIRCCFG_RANGE_MASK) >> SCG_FIRCCFG_RANGE_SHIFT) * 4000000;
      break;
    case 0x5:
      /* Low Power FLL */
      SCGOUTClock = 48000000 + ((SCG->LPFLLCFG & SCG_LPFLLCFG_FSEL_MASK) >> SCG_LPFLLCFG_FSEL_SHIFT) * 24000000;
      break;
    default:
      return;
  }
  SystemCoreClock = (SCGOUTClock / Divider);

}

/* ----------------------------------------------------------------------------
   -- SystemInitHook()
   ---------------------------------------------------------------------------- */

__attribute__ ((weak)) void SystemInitHook (void) {
  /* Void implementation of the weak function. */
}

/* Use LIPT1 channel 1 for systick. */
#define SYSTICK_LPIT LPIT1
#define SYSTICK_LPIT_CH 1
#define SYSTICK_LPIT_IRQn LPIT1_IRQn

/* Leverage LPIT1 to provide Systick */
void SystemSetupSystick(uint32_t tickRateHz, uint32_t intPriority)
{
    // Init pit module
    CLOCK_EnableClock(kCLOCK_Lpit1);

    // Reset the timer channels and registers except the MCR register
    SYSTICK_LPIT->MCR |= LPIT_MCR_SW_RST_MASK;
    SYSTICK_LPIT->MCR &= ~LPIT_MCR_SW_RST_MASK;

    // Setup timer operation in debug and doze modes and enable the module
    SYSTICK_LPIT->MCR = LPIT_MCR_DBG_EN_MASK | LPIT_MCR_DOZE_EN_MASK | LPIT_MCR_M_CEN_MASK;

    // Set timer period for channel 0
    SYSTICK_LPIT->CHANNEL[SYSTICK_LPIT_CH].TVAL = (CLOCK_GetIpFreq(kCLOCK_Lpit1) / tickRateHz) - 1;

    // Enable timer interrupts for channel 0
    SYSTICK_LPIT->MIER |= (1U << SYSTICK_LPIT_CH);

    // Set interrupt priority.
    NVIC_SetPriority(LPIT1_IRQn, (1 << __NVIC_PRIO_BITS) - 1);
    NVIC_EnableIRQ(LPIT1_IRQn);

    // set pend exception priority
    NVIC_SetPriority(PendSV_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

    // Start channel 0
    SYSTICK_LPIT->SETTEN |= (LPIT_SETTEN_SET_T_EN_0_MASK << SYSTICK_LPIT_CH);
}

void SystemClearSystickFlag(void)
{
    // Channel 0.
    SYSTICK_LPIT->MSR = (1U << SYSTICK_LPIT_CH);
}


