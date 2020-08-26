/*
 * Copyright 2017 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <rthw.h>
#include <rtthread.h>

#include "board.h"
#include "drv_uart.h"

#include "pin_mux.h"
#include "clock_config.h"

#include <fsl_clock.h>
#include <fsl_intmux.h>
#include <fsl_mu.h>
#include <fsl_xrdc.h>
#include <fsl_sema42.h>
#include <fsl_port.h>
#include <fsl_gpio.h>
#include <fsl_lpuart.h>

#define APP_MU                  MUB
#define APP_SEMA42              SEMA420
#define BOOT_FLAG               0x01U
#define SEMA42_GATE             0U
#define LOCK_CORE               1U


/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

void LPIT1_IRQHandler(void)
{
    rt_tick_increase();

    SystemClearSystickFlag();
}

int rt_hw_systick_init(void)
{
    CLOCK_SetIpSrc(kCLOCK_Lpit1, kCLOCK_IpSrcFircAsync);

    SystemSetupSystick (RT_TICK_PER_SECOND, 0);
    SystemClearSystickFlag();

    return 0;
}

void rt_hw_board_init(void)
{
    BOARD_InitPins_Core1();

    MU_Init(APP_MU);
    MU_SetFlags(APP_MU, BOOT_FLAG);

    // Small delay
    LED1_OFF();
    for( int i=0; i<10000000; i++ )  { }
    LED1_ON();

    // Send a character through core 0 UART
    //LPUART_WriteByte(LPUART0, '.');

    // Initialize hardware interrupt
    rt_hw_uart_init();
    rt_hw_systick_init();  // Core 0 uses LPIT0 and this core uses LPIT1

#ifdef RT_USING_CONSOLE
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif

#ifdef RT_USING_HEAP
    rt_system_heap_init(RT_HW_HEAP_BEGIN, RT_HW_HEAP_END, 0);  // 0=Do not init lowest block ptr (must be done ONLY by starter core)
#endif

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
}
