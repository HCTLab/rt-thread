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

#include "pin_mux.h"
#include "clock_config.h"

#include <fsl_clock.h>
#include <fsl_intmux.h>
#include <fsl_mu.h>
#include <fsl_xrdc.h>
#include <fsl_sema42.h>
#include <fsl_port.h>
#include <fsl_gpio.h>

// MUA/MUB defined at RV32M1_ri5cy.h
#define APP_MU                   MUA
#define APP_SEMA42               SEMA420
#define APP_CORE1_BOOT_MODE      kMU_CoreBootFromDflashBase
#define BOOT_FLAG                0x01U
#define SEMA42_GATE              0U
#define LOCK_CORE                0U


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
        kXRDC_PeriphLpuart0, kXRDC_PeriphWdog0,   kXRDC_PeriphXrdcMgr, kXRDC_PeriphXrdcMdac, kXRDC_PeriphXrdcPac,
        kXRDC_PeriphXrdcMrc, kXRDC_PeriphSema420, kXRDC_PeriphSema421, kXRDC_PeriphWdog1,    kXRDC_PeriphPcc0,
        kXRDC_PeriphPcc1,    kXRDC_PeriphMua,     kXRDC_PeriphMub };

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

int rt_hw_systick_init(void)
{
    CLOCK_SetIpSrc(kCLOCK_Lpit0, kCLOCK_IpSrcFircAsync);

    SystemSetupSystick (RT_TICK_PER_SECOND, 0);
    SystemClearSystickFlag();

    return 0;
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
    const gpio_pin_config_t config = {
        .pinDirection = kGPIO_DigitalOutput, .outputLogic = 1,
    };

    GPIO_PinInit(BOARD_LED1_GPIO, BOARD_LED1_GPIO_PIN, &config);
}

#ifdef RT_USING_SMP

int rt_hw_cpu_id(void)
{
    return read_csr(mhartid);
    //return 0;  // Fixed by now
}

void rt_hw_spin_lock_init(rt_hw_spinlock_t *lock)
{
    ((spinlock_t *)lock)->lock = 0;
}

void rt_hw_spin_lock(rt_hw_spinlock_t *lock)
{
    lock->tickets.owner--;
}

void rt_hw_spin_unlock(rt_hw_spinlock_t *lock)
{
    lock->tickets.owner++;
}

void rt_hw_ipi_send(int ipi_vector, unsigned int cpu_mask)
{
    int idx;

    for (idx = 0; idx < RT_CPUS_NR; idx ++)
    {
        if (cpu_mask & (1 << idx))
        {
            //clint_ipi_send(idx);
        }
    }
}

extern rt_base_t secondary_boot_flag;

void rt_hw_secondary_cpu_up(void)
{
    mb();
    secondary_boot_flag = 0xa55a;
}

extern void rt_hw_scondary_interrupt_init(void);
extern int  rt_hw_tick_init(void);
extern int  rt_hw_clint_ipi_enable(void);

void secondary_cpu_c_start(void)
{
    rt_hw_spin_lock(&_cpus_lock);

    /* initialize interrupt controller */
    rt_hw_scondary_interrupt_init();

    rt_hw_tick_init();

    rt_hw_clint_ipi_enable();

    rt_system_scheduler_start();
}

void rt_hw_secondary_cpu_idle_exec(void)
{
    asm volatile ("wfi");
}

#endif /*RT_USING_SMP*/

void rt_hw_cpu_shutdown()
{
    rt_uint32_t level;
    rt_kprintf("shutdown...\n");

    level = rt_hw_interrupt_disable();
    while (level)
    {
        RT_ASSERT(0);
    }
}

void rt_hw_us_delay( rt_uint32_t us )
{
    // TBD
}

void rt_hw_board_init(void)
{
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitLedPin();
    LED1_INIT(LOGIC_LED_ON);

    CLOCK_InitLpFll(&g_appScgLpFllConfig_BOARD_BootClockRUN);

    // Small delay (~5 secs) to wait for an external debug TAP connection
    for( int i=0; i<20000000; i++ )  { }

    MU_Init(APP_MU);
    APP_InitDomain();

    SEMA42_Init(APP_SEMA42);
    SEMA42_ResetAllGates(APP_SEMA42);

    // Boot Core 1 (CM0+)
    MU_BootOtherCore(APP_MU, APP_CORE1_BOOT_MODE);
    // Wait till Core 1 is Boot Up
    while (BOOT_FLAG != MU_GetFlags(APP_MU)) { }

    INTMUX_Init(INTMUX0);
    INTMUX_EnableInterrupt(INTMUX0, 0, PORTC_IRQn);

    /* initialize hardware interrupt */
    rt_hw_uart_init();
    rt_hw_systick_init();

#ifdef RT_USING_CONSOLE
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif /* RT_USING_CONSOLE */

#ifdef RT_USING_HEAP
    rt_system_heap_init(RT_HW_HEAP_BEGIN, RT_HW_HEAP_END, 1);
#endif

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
}
