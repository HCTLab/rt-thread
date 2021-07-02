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

#ifdef RT_USING_SMP

int rt_hw_cpu_id( void )
{
    return 0;  // Fixed by now
}

void rt_hw_spin_lock_init( rt_hw_spinlock_t *lock )
{
    lock->slock = 0;
}

void rt_hw_spin_lock( rt_hw_spinlock_t *lock )
{
    lock->tickets.owner--;
}

void rt_hw_spin_unlock( rt_hw_spinlock_t *lock )
{
    lock->tickets.owner++;
}

void rt_hw_ipi_send( int ipi_vector, unsigned int cpu_mask )
{
    int idx;

    for( idx=0; idx<RT_CPUS_NR; idx ++ )
    {
        if( cpu_mask & (1 << idx) )
        {
            //clint_ipi_send( idx );
        }
    }
}

extern rt_base_t secondary_boot_flag;

void rt_hw_secondary_cpu_up( void )
{
    mb();
    secondary_boot_flag = 0xa55a;
}

extern void rt_hw_scondary_interrupt_init( void );
extern int  rt_hw_tick_init( void );
extern int  rt_hw_clint_ipi_enable( void );

void secondary_cpu_c_start( void )
{
    rt_hw_spin_lock(&_cpus_lock);

    /* initialize interrupt controller */
    rt_hw_scondary_interrupt_init();

    rt_hw_tick_init();

    rt_hw_clint_ipi_enable();

    rt_system_scheduler_start();
}

void rt_hw_secondary_cpu_idle_exec( void )
{
    //asm volatile ("wfi");
}

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

int rt_hw_systick_init( void )
{
    CLOCK_SetIpSrc( kCLOCK_Lpit1, kCLOCK_IpSrcFircAsync );

    SystemSetupSystick( RT_TICK_PER_SECOND, 0 );
    SystemClearSystickFlag();

    return 0;
}

void rt_hw_us_delay( rt_uint32_t us )
{
    // TBD
}

#define OBJ_APP_SEMA42              SEMA420     // HW instance
#define OBJ_SEMA42_GATE             1U          // The SEMA42 gate (up to 15)
#define OBJ_LOCK_CORE               1U          // Core 1 (CM0+) locking identifier

void rt_hw_object_trytake( struct rt_object *object )
{
    SEMA42_Lock( OBJ_APP_SEMA42, OBJ_SEMA42_GATE, OBJ_LOCK_CORE );
}

void rt_hw_object_put( struct rt_object *object )
{
    SEMA42_Unlock( OBJ_APP_SEMA42, OBJ_SEMA42_GATE );
}

void rt_hw_board_init( void )
{
    BOARD_InitPins_Core1();

    MU_Init( APP_MU );
    MU_SetFlags( APP_MU, BOOT_FLAG );

    // Small delay
    LED1_OFF();
    for( int i=0; i<10000000; i++ )  { }
    LED1_ON();

    // Send a character through core 0 UART
    //LPUART_WriteByte(LPUART0, '.');

    /* initialize intercore AMP syncronization */
    rt_object_trytake_sethook( rt_hw_object_trytake );
    rt_object_put_sethook( rt_hw_object_put );

    // Initialize hardware interrupt
    rt_hw_uart_init();
    rt_hw_systick_init();  // Core 0 uses LPIT0 and this core uses LPIT1

#ifdef RT_USING_CONSOLE
    // Note: rt_console_set_device() calls rt_serial_init() which calls uart_configure() which calls CLOCK_SetIpSrc()
    //       CLOCK_SetIpSrc() can assert when several clock configs mismatch
    rt_console_set_device( RT_CONSOLE_DEVICE_NAME );
#endif

#ifdef RT_USING_HEAP
    rt_system_heap_init( RT_HW_HEAP_BEGIN, RT_HW_HEAP_END, 0 );  // 0=Do not init lowest block ptr (must be done ONLY by starter core)
#endif

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
}
