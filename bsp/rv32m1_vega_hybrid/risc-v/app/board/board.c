/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018/11/29     Bernard      the first version
 */

#include <rthw.h>
#include <rtthread.h>

#include "board.h"
#include "drv_uart.h"
#include "drv_sdcard.h"

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

// MUA/MUB defined at RV32M1_ri5cy.h
#define APP_MU                   MUA
#define APP_MU_IRQ               MUA_IRQn
#define APP_SEMA42               SEMA420
#define APP_CORE1_BOOT_MODE      kMU_CoreBootFromDflashBase
#define BOOT_FLAG                0x01U
#define MAX_GATES                16


void APP_InitDomain(void)
{
    /*
     * This function assigns core RI5CY to domain 0, assigns core CM0+ to domain 1
     * Then sets the necessary memory and peripheral permission.
     */

    uint32_t i;
    xrdc_periph_access_config_t periConfig;
    xrdc_processor_domain_assignment_t domainConfig;
    xrdc_mem_access_config_t memConfig;

    const xrdc_periph_t periphAccessible[] = {
        kXRDC_PeriphLpuart0,   kXRDC_PeriphSdhc0,     
        kXRDC_PeriphXrdcMgr,   kXRDC_PeriphXrdcMdac,  
        kXRDC_PeriphXrdcPac,   kXRDC_PeriphXrdcMrc,   
        kXRDC_PeriphSema420,   kXRDC_PeriphSema421, 
        kXRDC_PeriphWdog0,     kXRDC_PeriphWdog1,     
        kXRDC_PeriphPcc0,      kXRDC_PeriphPcc1,    
        kXRDC_PeriphMua,       kXRDC_PeriphMub,
        kXRDC_PeriphDma0,      kXRDC_PeriphDma1,
        kXRDC_PeriphDma1Tcd,   kXRDC_PeriphDma1Tcd,
        kXRDC_PeriphDmamux0,   kXRDC_PeriphDmamux1,
        kXRDC_PeriphIntmux0
    };

    XRDC_Init(XRDC);
    XRDC_SetGlobalValid(XRDC, false);
    XRDC_GetDefaultProcessorDomainAssignment(&domainConfig);

    // Assign RI5CY to domain
    domainConfig.domainId = 0;
    XRDC_SetProcessorDomainAssignment(XRDC, kXRDC_MasterRI5CYCodeBus, 0, &domainConfig);
    XRDC_SetProcessorDomainAssignment(XRDC, kXRDC_MasterRI5CYSystemBus, 0, &domainConfig);

    // Assign CM0+ to domain
    domainConfig.domainId = 1;
    XRDC_SetProcessorDomainAssignment(XRDC, kXRDC_MasterCM0P, 0, &domainConfig);
    //XRDC_SetProcessorDomainAssignment(XRDC, kXRDC_MasterCM4CodeBus, 0, &domainConfig);
    //XRDC_SetProcessorDomainAssignment(XRDC, kXRDC_MasterCM4SystemBus, 0, &domainConfig);

    // Configure the peripheral policy.
    XRDC_GetPeriphAccessDefaultConfig(&periConfig);
    // Access permission for domain 0
    periConfig.policy[0] = kXRDC_AccessPolicyAll;
    // Access permission for domain 1
    periConfig.policy[1] = kXRDC_AccessPolicyAll;

    for (i = 0; i < ARRAY_SIZE(periphAccessible); i++)
    {
        periConfig.periph = periphAccessible[i];
        XRDC_SetPeriphAccessConfig(XRDC, &periConfig);
    }

    // Configure default memory policy
    XRDC_GetMemAccessDefaultConfig(&memConfig);

    // Flash 1 code region
    memConfig.mem = kXRDC_MemMrc0_0;
    memConfig.baseAddress = 0x00000000U;
    memConfig.endAddress = 0x000FFFFFU;
    memConfig.codeRegion = kXRDC_MemCodeRegion1;
    memConfig.policy[0] = kXRDC_AccessFlagsAlt4;
    memConfig.policy[1] = kXRDC_AccessFlagsAlt4;
    XRDC_SetMemAccessConfig(XRDC, &memConfig);

    // Flash 2 code region
    memConfig.mem = kXRDC_MemMrc1_0;
    memConfig.baseAddress = 0x01000000U;
    memConfig.endAddress = 0x0103FFFFU;
    memConfig.codeRegion = kXRDC_MemCodeRegion1;
    memConfig.policy[0] = kXRDC_AccessFlagsAlt4;
    memConfig.policy[1] = kXRDC_AccessFlagsAlt4;
    XRDC_SetMemAccessConfig(XRDC, &memConfig);

    // ITCM SRAM, code might be placed here
    memConfig.mem = kXRDC_MemMrc0_1;
    memConfig.baseAddress = 0x08000000U;
    memConfig.endAddress = 0x0800FFFFU;
    memConfig.codeRegion = kXRDC_MemCodeRegion1;
    memConfig.policy[0] = kXRDC_AccessFlagsAlt4;
    memConfig.policy[1] = kXRDC_AccessFlagsAlt4;
    XRDC_SetMemAccessConfig(XRDC, &memConfig);

    // Boot ROM
    memConfig.mem = kXRDC_MemMrc0_2;
    memConfig.baseAddress = 0x08800000U;
    memConfig.endAddress = 0x0880BFFFU;
    memConfig.codeRegion = kXRDC_MemCodeRegion1;
    memConfig.policy[0] = kXRDC_AccessFlagsAlt4;
    memConfig.policy[1] = kXRDC_AccessFlagsAlt4;
    XRDC_SetMemAccessConfig(XRDC, &memConfig);

    // TCM SRAM
    memConfig.mem = kXRDC_MemMrc1_1;
    memConfig.baseAddress = 0x09000000U;
    memConfig.endAddress = 0x0901FFFFU;
    memConfig.codeRegion = kXRDC_MemCodeRegion1;
    memConfig.policy[0] = kXRDC_AccessFlagsAlt4;
    memConfig.policy[1] = kXRDC_AccessFlagsAlt4;
    XRDC_SetMemAccessConfig(XRDC, &memConfig);

    // CTI, for debugger
    memConfig.mem = kXRDC_MemMrc1_2;
    memConfig.baseAddress = 0xF0006000U;
    memConfig.endAddress = 0xF0006FFFU;
    memConfig.codeRegion = kXRDC_MemCodeRegion0;
    memConfig.policy[0] = kXRDC_AccessFlagsNone;
    memConfig.policy[1] = kXRDC_AccessFlagsAlt4;
    XRDC_SetMemAccessConfig(XRDC, &memConfig);

    // DTCM SRAM, data region 1
    memConfig.mem = kXRDC_MemMrc0_3;
    memConfig.baseAddress = 0x20000000U;
    memConfig.endAddress = 0x2001FFFFU;
    memConfig.codeRegion = kXRDC_MemCodeRegion0;
    memConfig.policy[0] = kXRDC_AccessFlagsAlt4;
    memConfig.policy[1] = kXRDC_AccessFlagsAlt4;
    XRDC_SetMemAccessConfig(XRDC, &memConfig);

    // DTCM SRAM, data region 2
    memConfig.mem = kXRDC_MemMrc0_4;
    memConfig.baseAddress = 0x20020000U;
    memConfig.endAddress = 0x2003FFFFU;
    memConfig.codeRegion = kXRDC_MemCodeRegion0;
    memConfig.policy[0] = kXRDC_AccessFlagsAlt4;
    memConfig.policy[1] = kXRDC_AccessFlagsAlt4;
    XRDC_SetMemAccessConfig(XRDC, &memConfig);

    XRDC_SetGlobalValid(XRDC, true);
}

void APP_DeinitDomain(void)
{
    XRDC_SetGlobalValid(XRDC, false);
}

void LPIT0_IRQHandler(void)
{
    rt_tick_increase();

    SystemClearSystickFlag();
}

static uint32_t  tick_max_count   = 0L;
static uint32_t  tick_max_count_o = 0L;

int rt_hw_systick_init(void)
{
    CLOCK_SetIpSrc( kCLOCK_Lpit0, kCLOCK_IpSrcFircAsync );

    // Note: Use the same time base for all cores (==LPIT0)
    tick_max_count   = ((uint32_t) CLOCK_GetIpFreq( kCLOCK_Lpit0 ) / RT_TICK_PER_SECOND);
    tick_max_count_o = tick_max_count / 100;

    SystemSetupSystick( RT_TICK_PER_SECOND, 0 );  // 0 = Top priority
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

const scg_lpfll_config_t g_appScgLpFllConfig_BOARD_BootClockRUN = {
    .enableMode = kSCG_LpFllEnable, /* LPFLL clock disabled */
    .div1 = kSCG_AsyncClkDivBy1,    /* Low Power FLL Clock Divider 1: Clock output is disabled */
    .div2 = kSCG_AsyncClkDisable,   /* Low Power FLL Clock Divider 2: Clock output is disabled */
    .div3 = kSCG_AsyncClkDisable,   /* Low Power FLL Clock Divider 3: Clock output is disabled */
    .range = kSCG_LpFllRange72M,    /* LPFLL is trimmed to 72MHz */
    .trimConfig = NULL,
};

static void BOARD_InitLedPin(void)
{
    const gpio_pin_config_t config = 
    {
        .pinDirection = kGPIO_DigitalOutput, 
        .outputLogic  = 0,
    };

    GPIO_PinInit(BOARD_LED1_GPIO, BOARD_LED1_GPIO_PIN, &config);
}

#ifdef RT_USING_SMP

void rt_hw_object_delete(struct rt_object *object);
void rt_hw_object_trytake( struct rt_object *object );
void rt_hw_object_take(struct rt_object *object);
void rt_hw_object_put(struct rt_object *object);
void rt_hw_object_dump(void);

int rt_hw_cpu_id(void)
{
    //return read_csr(mhartid);
    return 0;  // Fixed by now according to HYBRID architecture
}

void rt_hw_spin_lock_init(rt_hw_spinlock_t *lock)
{
    lock->slock = 0;
}

void rt_hw_spin_lock(rt_hw_spinlock_t *lock)
{
    rt_hw_object_take((struct rt_object *) lock);
    lock->tickets.owner++;
}

void rt_hw_spin_unlock(rt_hw_spinlock_t *lock)
{
    lock->tickets.owner--;
    rt_hw_object_put((struct rt_object *) lock);
}

// Dynamically control preemption on other cores
int  is_preemtive = 1;

void rt_hw_ipi_send(int ipi_vector, unsigned int cpu_mask)
{
    int  i;
    
    if( ipi_vector == RT_SCHEDULE_IPI )
    {
        if( is_preemtive != 0 )
        {
            // Do preemptive rescheduling in a different domain core
            // Note: Currrently this HW only support asserting an IRQ in a single core
            MU_TriggerInterrupts( APP_MU, kMU_GenInt0InterruptTrigger );
        }
        else
        {
            // Do non-preemptive rescheduling in one or different domain cores
            for( i=0; i<RT_CPUS_NR; i++ )
            {
                if( cpu_mask & (1<<i) )
                {
                    struct rt_cpu *pcpu = rt_cpu_index(i);
                    
                    pcpu->ipi_schedule = 1;  // Non-protected access -> TBD!
                } //endif
            } //endfor
        } //endif
    } //endif
}

#define MU_ISR_FLAG_BASE    (20)
#define MU_ISR_COUNT        (12)

void MUA_IRQHandler(void)
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
                // General MU interrupt 0 is used to allow hybrid to be preemptive
                // Other core has re-scheduled tasks on this core!
                rt_scheduler_ipi_handler( flags, NULL );
            } //endif
        } //endif
    } //endfor
}

void rt_hw_secondary_cpu_up(void)
{
}

void rt_hw_secondary_cpu_idle_exec(void)
{
    asm volatile ("wfi");
}

#endif /*RT_USING_SMP*/

void rt_hw_cpu_shutdown()
{
    rt_uint32_t level;
    rt_kprintf("Shutdown...\n");

    level = rt_hw_interrupt_disable();
    while (level)
    {
        RT_ASSERT(0);
    }
}

void rt_hw_us_delay( rt_uint32_t us )
{
}

//#define HYBRID_DEBUG
#define HYBRID_DEBUG_MIN_GATE       2
#define OBJ_APP_SEMA42              SEMA420             // HW instance
#define OBJ_LOCK_CORE               0U                  // Core 0 (RI5CY) locking identifier

struct rt_object *rt_hw_gate[MAX_GATES+1] = { 0 };        // Assigned gates to objects

void rt_hw_object_delete( struct rt_object *object )
{
    int  g;

    for( g=1; g<MAX_GATES; g++ ) 
    {
        if( object == rt_hw_gate[g] )
        {
            rt_hw_gate[g] = NULL;
            break;  // Reuse gate when same object
        } //endif
    } //endfor
}

void rt_hw_object_trytake( struct rt_object *object )
{
    int  g;
    return;  // Trytake is disabled by now

    for( g=1; g<MAX_GATES; g++ ) if( object == rt_hw_gate[g] ) break;  // Reuse gate when same object
    if( g == MAX_GATES ) for( g=1; g<MAX_GATES; g++ ) if( rt_hw_gate[g] == NULL ) break;  // Get a new gate if not found
    if( g < MAX_GATES )
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

    for( g=1; g<MAX_GATES; g++ ) if( object == rt_hw_gate[g] ) break;  // Reuse gate when same object
    if( g == MAX_GATES ) for( g=1; g<MAX_GATES; g++ ) if( rt_hw_gate[g] == NULL ) break;  // Get a new gate if not found
    if( g < MAX_GATES )
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
        rt_kprintf("%s Locking GATE error [no more gates] Addr [%p] Name [%08X]\n", RT_DEBUG_ARCH, object, object->name);
        //rt_hw_object_dump();  //(JAAS) Simple way to find new semaphores/mutexes
    }
}

void rt_hw_object_put( struct rt_object *object )
{
    int  g;

    for( g=1; g<MAX_GATES; g++ ) if( object == rt_hw_gate[g] ) break;  // Search object's gate
    if( g < MAX_GATES )
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

void rt_hw_object_dump(void)
{
    struct rt_object  *object;
    int                g;
    static int         triggered = 0;
    
    if( triggered == 0 )
    {
        triggered = 1;  // Avoid mixing messages
        rt_kprintf("\n\%s objects:\n", RT_DEBUG_ARCH);
        for( g=1; g<MAX_GATES; g++ )
        {
            object = (struct rt_object *) rt_hw_gate[g];
            if( object != NULL )
            {
                rt_kprintf("[%02d] Addr [%08X] Name [%s]\n", g, object, object->name);
            } //endif
        } //endof
    } //endif
}

void rt_hw_board_init(void)
{
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitLedPin();
    LED1_INIT( LOGIC_LED_ON );

    CLOCK_InitLpFll( &g_appScgLpFllConfig_BOARD_BootClockRUN );

    // Small delay (~3 secs) to wait for an external debug TAP connection
    for( int i=0; i<10000000; i++ )  { }

    APP_InitDomain();

    MU_Init( APP_MU );
    MU_EnableInterrupts( APP_MU, kMU_GenInt0InterruptEnable );
    
    EVENT_SetIRQPriority( APP_MU_IRQ, 0 );  // 0 = Top priority (same as TICK timer)
    EnableIRQ( APP_MU_IRQ );
    
    SEMA42_Init( APP_SEMA42 );
    SEMA42_ResetAllGates( APP_SEMA42 );

    /* RFU
    INTMUX_Init(INTMUX0);
    INTMUX_EnableInterrupt( INTMUX0, 0, PORTC_IRQn );
    */

    // Initialize intercore AMP syncronization
    rt_object_trytake_sethook( rt_hw_object_trytake );
    rt_object_take_sethook( rt_hw_object_take );
    rt_object_put_sethook( rt_hw_object_put );
    rt_object_detach_sethook( rt_hw_object_delete );

    // Initialize hardware interrupt
    rt_system_scheduler_init();  // Scheduler will be init later on rtthread_startup(), but rt_hw_uart_init() requires some scheduler structure to be init!
    rt_hw_uart_init();
    rt_hw_systick_init();

#ifdef RT_USING_HEAP
    rt_system_heap_init( RT_HW_HEAP_BEGIN, RT_HW_HEAP_END, 1 );
#endif

    // Other drivers to be initialized
    rt_hw_sdcard_init();

    // Boot Core 1 (CM0+)
    MU_BootOtherCore( APP_MU, APP_CORE1_BOOT_MODE );
    // Wait till Core 1 will boot
    while( BOOT_FLAG != MU_GetFlags(APP_MU) ) { }
    //while(1);  // Only ARM runs

#ifdef RT_USING_CONSOLE
    rt_console_set_device( RT_CONSOLE_DEVICE_NAME );
#endif /* RT_USING_CONSOLE */

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
}
