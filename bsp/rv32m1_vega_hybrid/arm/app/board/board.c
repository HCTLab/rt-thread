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

#define APP_MU                   MUB
#define APP_MU_IRQ               MUB_IRQn
#define APP_SEMA42               SEMA420
#define BOOT_FLAG                0x01U


/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

#ifdef RT_USING_SMP

void rt_hw_object_take(struct rt_object *object);
void rt_hw_object_put(struct rt_object *object);

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
    rt_hw_object_take((struct rt_object *) lock);
    lock->tickets.owner++;
}

void rt_hw_spin_unlock( rt_hw_spinlock_t *lock )
{
    lock->tickets.owner--;
    rt_hw_object_put((struct rt_object *) lock);
}

void rt_hw_ipi_send( int ipi_vector, unsigned int cpu_mask )
{
    MU_TriggerInterrupts( APP_MU, kMU_GenInt0InterruptTrigger );
}

#define MU_ISR_FLAG_BASE    (20)
#define MU_ISR_COUNT        (12)

void MUB_IRQHandler(void)
{
    uint32_t flags;
    int i;

    flags = MU_GetStatusFlags( APP_MU );
    
#if (defined(FSL_FEATURE_MU_HAS_RESET_INT) && FSL_FEATURE_MU_HAS_RESET_INT)
    /* The other core reset assert interrupt pending */
    if( flags & kMU_ResetAssertInterruptFlag )
    {
        MU_ClearStatusFlags( APP_MU, kMU_ResetAssertInterruptFlag );
        return;
    } //endif
#endif

    for( i=MU_ISR_FLAG_BASE; i<(MU_ISR_FLAG_BASE+MU_ISR_COUNT); i++ )
    {
        if( flags & (1 << i) )
        {
            MU_ClearStatusFlags( APP_MU, (1 << i) );
        
            if( flags & kMU_GenInt0Flag )
            {
                // General MU interrupt 0 is used to allow hybrid to be preemtive
                // Other core has re-scheduled tasks on this core!
                rt_schedule();
            } //endif
        } //endif
    } //endfor
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

//#define HYBRID_DEBUG
#define HYBRID_DEBUG_MIN_GATE       2
#define OBJ_APP_SEMA42              SEMA420     // HW instance
#define OBJ_LOCK_CORE               1U          // Core 0 (ARM) locking identifier

extern struct rt_object *rt_hw_gate[16];        // Assigned gates to objects

void rt_hw_object_trytake( struct rt_object *object )
{
    int  g;
    return;  // Trytake is disabled by now

    for( g=1; g<16; g++ ) if( object == rt_hw_gate[g] ) break;  // Reuse gate when same object
    if( g == 16 ) for( g=1; g<16; g++ ) if( rt_hw_gate[g] == NULL ) break;  // Get a new gate if not found
    if( g < 16 )
    {
        rt_hw_gate[g] = object;
#ifdef HYBRID_DEBUG
        //SEMA42_Lock( OBJ_APP_SEMA42, 0, OBJ_LOCK_CORE );  // 0=Reserved gate for debugging
        if(g>HYBRID_DEBUG_MIN_GATE) rt_kprintf("%s Try locking GATE [%p=%d]\n", RT_DEBUG_ARCH, object, g);
        //SEMA42_Unlock( OBJ_APP_SEMA42, 0 );
#endif
        SEMA42_TryLock( OBJ_APP_SEMA42, g, OBJ_LOCK_CORE );
    }
    else
    {
        rt_kprintf("%s Try locking GATE error [%p=no more gates]\n", RT_DEBUG_ARCH, object);
    }
}

void rt_hw_object_take( struct rt_object *object )
{
    int  g;

    for( g=1; g<16; g++ ) if( object == rt_hw_gate[g] ) break;  // Reuse gate when same object
    if( g == 16 ) for( g=1; g<16; g++ ) if( rt_hw_gate[g] == NULL ) break;  // Get a new gate if not found
    if( g < 16 )
    {
        rt_hw_gate[g] = object;
#ifdef HYBRID_DEBUG
        //SEMA42_Lock( OBJ_APP_SEMA42, 0, OBJ_LOCK_CORE );  // 0=Reserved gate for debugging
        if(g>HYBRID_DEBUG_MIN_GATE) rt_kprintf("%s Locking GATE [%p=%d]\n", RT_DEBUG_ARCH, object, g);
        //SEMA42_Unlock( OBJ_APP_SEMA42, 0 );
#endif
        SEMA42_Lock( OBJ_APP_SEMA42, g, OBJ_LOCK_CORE );
    }
    else
    {
        rt_kprintf("%s Locking GATE error [%p=no more gates]\n", RT_DEBUG_ARCH, object);
    }
}

void rt_hw_object_put( struct rt_object *object )
{
    int  g;

    for( g=1; g<16; g++ ) if( object == rt_hw_gate[g] ) break;  // Search object's gate
    if( g < 16 )
    {
        SEMA42_Unlock( OBJ_APP_SEMA42, g );
#ifdef HYBRID_DEBUG
        //SEMA42_Lock( OBJ_APP_SEMA42, 0, OBJ_LOCK_CORE );  // 0=Reserved gate for debugging
        if(g>HYBRID_DEBUG_MIN_GATE) rt_kprintf("%s Unlocked GATE [%p=%d]\n", RT_DEBUG_ARCH, object, g);
        //SEMA42_Unlock( OBJ_APP_SEMA42, 0 );
#endif
    }
    else
    {
        rt_kprintf("%s Unlocked GATE [%p=no gate found]\n", RT_DEBUG_ARCH, object, g);
    }
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

    // Initialize intercore AMP syncronization
    rt_object_trytake_sethook( rt_hw_object_trytake );
    rt_object_take_sethook( rt_hw_object_take );
    rt_object_put_sethook( rt_hw_object_put );

    // Initialize hardware interrupt
    rt_system_scheduler_init();  // Scheduler will be init later on rtthread_startup(), but rt_hw_uart_init() requires some scheduler structure to be init!
    rt_hw_uart_init();
    rt_hw_systick_init();  // Core 0 uses LPIT0 and this core uses LPIT1
    
#ifdef RT_USING_HEAP
    rt_system_heap_init( RT_HW_HEAP_BEGIN, RT_HW_HEAP_END, 0 );  // 0=Do not init lowest block ptr (must be done ONLY by starter core)
#endif

#ifdef RT_USING_CONSOLE
    // Note: rt_console_set_device() calls rt_serial_init() which calls uart_configure() which calls CLOCK_SetIpSrc()
    //       CLOCK_SetIpSrc() can assert when several clock configs mismatch
    rt_console_set_device( RT_CONSOLE_DEVICE_NAME );
#endif

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
}
