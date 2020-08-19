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
#include <fsl_xrdc.h>

#define APP_MU                  MUA
#define APP_SEMA42              SEMA420
#define APP_CORE1_BOOT_MODE     kMU_CoreBootFromDflashBase


void APP_InitDomain(void)
{
    /*
     * This function assigns core 0 (CM4) to domain 0, assigns core 1 (RI5CY)
     * to domain 1. Then sets the necessary memory and peripheral permission.
     */

    uint32_t i;
    xrdc_periph_access_config_t periConfig;
    xrdc_processor_domain_assignment_t domainConfig;
    xrdc_mem_access_config_t memConfig;

    const xrdc_periph_t periphAccessible[] = {
        kXRDC_PeriphLpuart0, kXRDC_PeriphWdog0,   kXRDC_PeriphXrdcMgr, kXRDC_PeriphXrdcMdac, kXRDC_PeriphXrdcPac,
        kXRDC_PeriphXrdcMrc, kXRDC_PeriphSema420, kXRDC_PeriphSema421, kXRDC_PeriphWdog1,    kXRDC_PeriphPcc0,
        kXRDC_PeriphPcc1,    kXRDC_PeriphMua,     kXRDC_PeriphMub};

    XRDC_Init(XRDC);
    XRDC_SetGlobalValid(XRDC, false);

    /* Assign CM4 to domain 0 */
    XRDC_GetDefaultProcessorDomainAssignment(&domainConfig);
    domainConfig.domainId = 0;
    XRDC_SetProcessorDomainAssignment(XRDC, kXRDC_MasterCM4CodeBus, 0, &domainConfig);
    XRDC_SetProcessorDomainAssignment(XRDC, kXRDC_MasterCM4SystemBus, 0, &domainConfig);

    /* Assign ri5cy to domain 1 */
    domainConfig.domainId = 1;
    XRDC_SetProcessorDomainAssignment(XRDC, kXRDC_MasterRI5CYCodeBus, 0, &domainConfig);
    XRDC_SetProcessorDomainAssignment(XRDC, kXRDC_MasterRI5CYSystemBus, 0, &domainConfig);

    /*
     * Configure the peripheral policy.
     */
    XRDC_GetPeriphAccessDefaultConfig(&periConfig);
    /* Access permission for domain 0. */
    periConfig.policy[0] = kXRDC_AccessPolicyAll;
    /* Access permission for domain 1. */
    periConfig.policy[1] = kXRDC_AccessPolicyAll;

    for (i = 0; i < ARRAY_SIZE(periphAccessible); i++)
    {
        periConfig.periph = periphAccessible[i];
        XRDC_SetPeriphAccessConfig(XRDC, &periConfig);
    }

    /* Configure the memory policy. */
    XRDC_GetMemAccessDefaultConfig(&memConfig);
    /* CM4 flash is CM4 code region. */
    memConfig.mem = kXRDC_MemMrc0_0;
    memConfig.baseAddress = 0x00000000U;
    memConfig.endAddress = 0x000FFFFFU;
    memConfig.codeRegion = kXRDC_MemCodeRegion1;
    memConfig.policy[0] = kXRDC_AccessFlagsAlt4;
    memConfig.policy[1] = kXRDC_AccessFlagsNone;
    XRDC_SetMemAccessConfig(XRDC, &memConfig);

    /* CM0+ flash is CM0+ code region. */
    memConfig.mem = kXRDC_MemMrc1_0;
    memConfig.baseAddress = 0x01000000U;
    memConfig.endAddress = 0x0103FFFFU;
    memConfig.codeRegion = kXRDC_MemCodeRegion1;
    memConfig.policy[0] = kXRDC_AccessFlagsNone;
    memConfig.policy[1] = kXRDC_AccessFlagsAlt4;
    XRDC_SetMemAccessConfig(XRDC, &memConfig);

    /* CM4 ITCM SRAM, code might be placed here. */
    memConfig.mem = kXRDC_MemMrc0_1;
    memConfig.baseAddress = 0x08000000U;
    memConfig.endAddress = 0x0800FFFFU;
    memConfig.codeRegion = kXRDC_MemCodeRegion1;
    memConfig.policy[0] = kXRDC_AccessFlagsAlt4;
    memConfig.policy[1] = kXRDC_AccessFlagsNone;
    XRDC_SetMemAccessConfig(XRDC, &memConfig);

    /* Boot ROM. */
    memConfig.mem = kXRDC_MemMrc0_2;
    memConfig.baseAddress = 0x08800000U;
    memConfig.endAddress = 0x0880BFFFU;
    memConfig.codeRegion = kXRDC_MemCodeRegion1;
    memConfig.policy[0] = kXRDC_AccessFlagsAlt4;
    memConfig.policy[1] = kXRDC_AccessFlagsAlt4;
    XRDC_SetMemAccessConfig(XRDC, &memConfig);

    /* CM0+ TCM SRAM. */
    memConfig.mem = kXRDC_MemMrc1_1;
    memConfig.baseAddress = 0x09000000U;
    memConfig.endAddress = 0x0901FFFFU;
    memConfig.codeRegion = kXRDC_MemCodeRegion1;
    memConfig.policy[0] = kXRDC_AccessFlagsNone;
    memConfig.policy[1] = kXRDC_AccessFlagsAlt4;
    XRDC_SetMemAccessConfig(XRDC, &memConfig);

    /* CM0+ CTI, for debugger. */
    memConfig.mem = kXRDC_MemMrc1_2;
    memConfig.baseAddress = 0xF0006000U;
    memConfig.endAddress = 0xF0006FFFU;
    memConfig.codeRegion = kXRDC_MemCodeRegion0;
    memConfig.policy[0] = kXRDC_AccessFlagsNone;
    memConfig.policy[1] = kXRDC_AccessFlagsAlt4;
    XRDC_SetMemAccessConfig(XRDC, &memConfig);

    /* CM4 DTCM SRAM, data region. */
    memConfig.mem = kXRDC_MemMrc0_3;
    memConfig.baseAddress = 0x20000000U;
    memConfig.endAddress = 0x2001FFFFU;
    memConfig.codeRegion = kXRDC_MemCodeRegion0;
    memConfig.policy[0] = kXRDC_AccessFlagsAlt4;
    memConfig.policy[1] = kXRDC_AccessFlagsNone;
    XRDC_SetMemAccessConfig(XRDC, &memConfig);

    memConfig.mem = kXRDC_MemMrc0_4;
    memConfig.baseAddress = 0x20020000U;
    memConfig.endAddress = 0x2003FFFFU;
    memConfig.codeRegion = kXRDC_MemCodeRegion0;
    memConfig.policy[0] = kXRDC_AccessFlagsAlt4;
    memConfig.policy[1] = kXRDC_AccessFlagsNone;
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

void rt_hw_board_init(void)
{
    BOARD_InitPins();
    BOARD_BootClockRUN();
    /* Init LPFLL */
    CLOCK_InitLpFll(&g_appScgLpFllConfig_BOARD_BootClockRUN);

    MU_Init(APP_MU);

    APP_InitDomain();

    /* SEMA42 init */
    SEMA42_Init(APP_SEMA42);
    /* Reset the sema42 gate */
    SEMA42_ResetAllGates(APP_SEMA42);

    /* Boot Core 1. */
    MU_BootCoreB(APP_MU, APP_CORE1_BOOT_MODE);
    /* Wait Core 1 is Boot Up */
    while (BOOT_FLAG != MU_GetFlags(APP_MU))
    {
    }

    INTMUX_Init(INTMUX0);
    INTMUX_EnableInterrupt(INTMUX0, 0, PORTC_IRQn);

    /* initialize hardware interrupt */
    rt_hw_uart_init();
    rt_hw_systick_init();

#ifdef RT_USING_CONSOLE
    /* set console device */
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif /* RT_USING_CONSOLE */

#ifdef RT_USING_HEAP
    /* initialize memory system */
    rt_system_heap_init(RT_HW_HEAP_BEGIN, RT_HW_HEAP_END);
#endif

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
}
