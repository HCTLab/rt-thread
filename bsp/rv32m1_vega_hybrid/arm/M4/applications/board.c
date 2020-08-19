/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2009-01-05     Bernard      first implementation
 * 2014-06-20     xiaonong     ported to LPC43xx
 * 2020-07-28     jaandres     ported to RV32M1_VEGA
 */

#include <rthw.h>
#include <rtthread.h>

#include "board.h"
#include "drv_uart.h"

extern unsigned char __heap_start;

#define RT_HW_HEAP_BEGIN    (void*)&__heap_start
#define RT_HW_HEAP_END      (void*)(0x20000000 + 0x00030000 - 0x1800)

/**
 * This is the timer interrupt service routine.
 *
 */
void SysTick_Handler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_tick_increase();

    /* leave interrupt */
    rt_interrupt_leave();
}

extern void SystemCoreClockUpdate(void);

/**
 * This function will initial RV32M1_VEGA board.
 */
void rt_hw_board_init()
{
#ifdef CORE_M4
    /* NVIC Configuration */
#ifdef  VECT_TAB_RAM
    /* Set the Vector Table base location at 0x10000000 */
    SCB->VTOR  = 0x10000000;
#else  /* VECT_TAB_FLASH  */
    /* Set the Vector Table base location at 0x00000000 */
    SCB->VTOR  = 0x00000000;
#endif
#endif
    /* init systick */
    SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);

    /* set pend exception priority */
    NVIC_SetPriority(PendSV_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

    /* init uart device */
    rt_hw_uart_init();

    /* setup the console device */
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);

#ifdef RT_USING_HEAP
    /* initialize memory system -> Done by main core */
    //rt_system_heap_init(RT_HW_HEAP_BEGIN, RT_HW_HEAP_END);
#endif

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
}



