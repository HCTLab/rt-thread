/*
 * Copyright 2017 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include "fsl_common.h"
#include "fsl_debug_console.h"
#include "board.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Initialize debug console. */
void BOARD_InitDebugConsole(void)
{
    CLOCK_SetIpSrc(kCLOCK_Lpuart0, kCLOCK_IpSrcFircAsync);

    uint32_t uartClkSrcFreq = BOARD_DEBUG_UART_CLK_FREQ;

    DbgConsole_Init(BOARD_DEBUG_UART_BASEADDR, BOARD_DEBUG_UART_BAUDRATE, BOARD_DEBUG_UART_TYPE, uartClkSrcFreq);
}

void rt_hw_board_init(void)
{
    // Small delay
    for( int i=0; i<20000000; i++ )  { }
    while(1);

    /*
    MU_Init(APP_MU);
    APP_InitDomain();

    SEMA42_Init(APP_SEMA42);
    SEMA42_ResetAllGates(APP_SEMA42);
    SEMA42_Lock(APP_SEMA42, SEMA42_GATE, LOCK_CORE);

    // Boot Core 1 (CM0+)
    MU_BootOtherCore(APP_MU, APP_CORE1_BOOT_MODE);
    //MU_HardwareResetOtherCore(MUA, true, true, kMU_CoreBootFromDflashBase);

    // Wait till Core 1 is Boot Up
    //while (BOOT_FLAG != MU_GetFlags(APP_MU)) { }

    INTMUX_Init(INTMUX0);
    INTMUX_EnableInterrupt(INTMUX0, 0, PORTC_IRQn);

    // initialize hardware interrupt
    rt_hw_uart_init();
    rt_hw_systick_init();

#ifdef RT_USING_CONSOLE
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif

#ifdef RT_USING_HEAP
    // initialize memory system
    rt_system_heap_init(RT_HW_HEAP_BEGIN, RT_HW_HEAP_END);
#endif

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
    */
}
