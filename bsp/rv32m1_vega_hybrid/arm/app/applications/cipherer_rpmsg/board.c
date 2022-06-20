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
#include <fsl_lpit.h>
#include <fsl_intmux.h>
#include <fsl_mu.h>
#include <fsl_xrdc.h>
#include <fsl_sema42.h>
#include <fsl_port.h>
#include <fsl_gpio.h>
#include <fsl_lpuart.h>

#define APP_MU                   MUB
#define APP_MU_IRQ               MUB_IRQn
#define BOOT_FLAG                0x01U


#ifdef RT_USING_SMP

int rt_hw_cpu_id( void )
{
    return 1;  // Fixed by now according to HYBRID architecture
}

void rt_hw_spin_lock_init( rt_hw_spinlock_t *lock )
{
    lock->slock = 0;
}

void rt_hw_spin_lock( rt_hw_spinlock_t *lock )
{
    //rt_hw_object_take((struct rt_object *) lock);
    lock->tickets.owner++;
}

void rt_hw_spin_unlock( rt_hw_spinlock_t *lock )
{
    lock->tickets.owner--;
    //rt_hw_object_put((struct rt_object *) lock);
}

void rt_hw_ipi_send(int ipi_vector, unsigned int cpu_mask)
{
}

void rt_hw_secondary_cpu_up( void )
{
}

void rt_hw_secondary_cpu_idle_exec( void )
{
    asm volatile ("wfi");
}

rt_thread_t rt_current_thread;

#endif /*RT_USING_SMP*/

void rt_hw_cpu_shutdown()
{
    rt_uint32_t level;
    rt_kprintf( "Shutdown...\n" );

    level = rt_hw_interrupt_disable();
    while( level )
    {
        RT_ASSERT(0);
    }
}

void LPIT1_IRQHandler( void )
{
    rt_tick_increase();

    SystemClearSystickFlag();
}

static uint32_t  tick_max_count   = 0L;
static uint32_t  tick_max_count_o = 0L;

int rt_hw_systick_init( void )
{
    CLOCK_SetIpSrc( kCLOCK_Lpit1, kCLOCK_IpSrcFircAsync );

    // Note: Use the same time base for all cores (==LPIT0)
    tick_max_count   = ((uint32_t) CLOCK_GetIpFreq( kCLOCK_Lpit0 ) / RT_TICK_PER_SECOND);
    tick_max_count_o = tick_max_count / 100;

    SystemSetupSystick( RT_TICK_PER_SECOND, 0 );
    SystemClearSystickFlag();

    return 0;
}

long rt_hw_usec_get(void)
{
    register long       usec;
    register uint32_t   count1, count2;
    //static   long       last_usec  = 0;
    //register uint32_t   last_count = 0x0FFFFFFF;
    
    // Get current usecs from first core tick counter
    // Note: Use the same time base for all cores (==LPIT0)
    // Note: Read channel 0 for SYSTEM TICK counter, please refer to system_RV32M1_xxx.c
    count1 = LPIT_GetCurrentTimerCount( LPIT0, 0 );
    usec   = (long) rt_cpu_index(0)->tick;
    count2 = LPIT_GetCurrentTimerCount( LPIT0, 0 );
    if( count2 > count1 ) { usec = (long) rt_cpu_index(0)->tick; count1 = count2; }
    usec   = usec * (1000000L/RT_TICK_PER_SECOND);

    /* Timer integrity tests -> They should never trigger
    if( count1 > tick_max_count )  // Usually TVAL=0x752FF for 10ms@40Mhz
    {
        rt_kprintf( "Error: CVAL [%08X]  TVAL [%08X]\n", count1, tick_max_count );
        while(1);
    } //endif
    if( usec < last_usec )
    {
        rt_kprintf( "Error: USEC [%ld]  LAST [%ld]\n", usec, last_usec );
        while(1);
    }
    if( (usec == last_usec) && (count1 >= last_count) )
    {
        rt_kprintf( "Error: COUNT [%08X]  LAST [%08X]\n", count1, last_count );
        while(1);
    }
    last_usec  = usec;
    last_count = count1;
    */
   
    // LPIT is a decrementing counter
    count1 = tick_max_count - count1;
    
    // Formula: tick_max_count - 10.000 usec | count - x
    //return usec + ((count * (1000000L/RT_TICK_PER_SECOND)) / (long) tick_max_count);  // Overflows!
    return usec + ((count1 * 100L) / (long) tick_max_count_o);  // Do NOT overflow!
}

void rt_hw_us_delay( rt_uint32_t us )
{
}

void rt_hw_board_init( void )
{
    BOARD_InitPins_Core1();

    MU_Init( APP_MU );
    MU_EnableInterrupts( APP_MU, kMU_GenInt0InterruptEnable );
    MU_SetFlags( APP_MU, BOOT_FLAG );

    // Set interrupt priority.
    NVIC_SetPriority( APP_MU_IRQ, (1 << __NVIC_PRIO_BITS) - 1);
    NVIC_EnableIRQ( APP_MU_IRQ );

    // Move leds during a small delay to indicate that core is booting...
    //LPUART_WriteByte(LPUART0, '.');
    LED1_OFF();
    for( int i=0; i<1000000; i++ )  { }
    LED1_ON();

    /* Initialize intercore AMP syncronization
    rt_object_trytake_sethook( rt_hw_object_trytake );
    rt_object_take_sethook( rt_hw_object_take );
    rt_object_put_sethook( rt_hw_object_put );
    rt_object_detach_sethook( rt_hw_object_delete );
    */

    // Initialize hardware interrupt
    rt_system_scheduler_init();  // Scheduler will be init later on rtthread_startup(), but rt_hw_uart_init() requires some scheduler structure to be init!
    rt_hw_uart_init();
    rt_hw_systick_init();  // Core 0 uses LPIT0 and this core uses LPIT1
    platform_init();  // RMMSG_LITE init
    
#ifdef RT_USING_HEAP
    //rt_system_heap_init( RT_HW_HEAP_BEGIN, RT_HW_HEAP_END, 0 );  // 0=Do not init lowest block ptr (must be done ONLY by starter core)
    rt_system_heap_init( RT_HW_HEAP_BEGIN, RT_HW_HEAP_END, 1 );  // 0=Do not init lowest block ptr (must be done ONLY by starter core)
#endif

#ifdef RT_USING_CONSOLE
    // Note: rt_console_set_device() calls rt_serial_init() which calls uart_configure() which calls CLOCK_SetIpSrc()
    //       CLOCK_SetIpSrc() can assert when several clock configs mismatch
    rt_console_set_device( RT_CONSOLE_DEVICE_NAME );
#endif

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

    // Since components are not init on ARM arch, it must be done manually
    posix_mq_system_init();
    posix_sem_system_init();
    libc_system_init();
}
