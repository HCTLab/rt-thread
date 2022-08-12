/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2017-5-30      Bernard      the first version
 */

#ifndef BOARD_H__
#define BOARD_H__

#include <stdint.h>

extern unsigned char __heap_start;

#define RT_HW_HEAP_BEGIN            (void*)(0x20007000)
#define RT_HW_HEAP_END              (void*)(0x2001D000)
#define BOARD_SHARED_MEMORY_BASE    (void*)(0x20020000)
#define BOARD_SHARED_MEMORY_SIZE    0x10000

#define LOGIC_LED_ON 1U
#define LOGIC_LED_OFF 0U
#define BOARD_LED1_GPIO GPIOA
#define BOARD_LED1_GPIO_PIN 24U
#define BOARD_LED2_GPIO GPIOA
#define BOARD_LED2_GPIO_PIN 23U
#define BOARD_LED3_GPIO GPIOA
#define BOARD_LED3_GPIO_PIN 22U

#define LED1_INIT(output)                                              \
    GPIO_WritePinOutput(BOARD_LED1_GPIO, BOARD_LED1_GPIO_PIN, output); \
    BOARD_LED1_GPIO->PDDR |= (1U << BOARD_LED1_GPIO_PIN)                                /*!< Enable target LED1 */
#define LED1_ON() GPIO_SetPinsOutput(BOARD_LED1_GPIO, 1U << BOARD_LED1_GPIO_PIN)        /*!< Turn on target LED1 */
#define LED1_OFF() GPIO_ClearPinsOutput(BOARD_LED1_GPIO, 1U << BOARD_LED1_GPIO_PIN)     /*!< Turn off target LED1 */
#define LED1_TOGGLE() GPIO_TogglePinsOutput(BOARD_LED1_GPIO, 1U << BOARD_LED1_GPIO_PIN) /*!< Toggle on target LED1 */

#define LED2_INIT(output)                                              \
    GPIO_WritePinOutput(BOARD_LED2_GPIO, BOARD_LED2_GPIO_PIN, output); \
    BOARD_LED2_GPIO->PDDR |= (1U << BOARD_LED2_GPIO_PIN)                                /*!< Enable target LED2 */
#define LED2_ON() GPIO_SetPinsOutput(BOARD_LED2_GPIO, 1U << BOARD_LED2_GPIO_PIN)        /*!< Turn on target LED2 */
#define LED2_OFF() GPIO_ClearPinsOutput(BOARD_LED2_GPIO, 1U << BOARD_LED2_GPIO_PIN)     /*!< Turn off target LED2 */
#define LED2_TOGGLE() GPIO_TogglePinsOutput(BOARD_LED2_GPIO, 1U << BOARD_LED2_GPIO_PIN) /*!< Toggle on target LED2 */

#define LED3_INIT(output)                                              \
    GPIO_WritePinOutput(BOARD_LED3_GPIO, BOARD_LED3_GPIO_PIN, output); \
    BOARD_LED3_GPIO->PDDR |= (1U << BOARD_LED3_GPIO_PIN)                                /*!< Enable target LED3 */
#define LED3_ON() GPIO_SetPinsOutput(BOARD_LED3_GPIO, 1U << BOARD_LED3_GPIO_PIN)        /*!< Turn on target LED3 */
#define LED3_OFF() GPIO_ClearPinsOutput(BOARD_LED3_GPIO, 1U << BOARD_LED3_GPIO_PIN)     /*!< Turn off target LED3 */
#define LED3_TOGGLE() GPIO_TogglePinsOutput(BOARD_LED3_GPIO, 1U << BOARD_LED3_GPIO_PIN) /*!< Toggle on target LED3 */

#define BOARD_USDHC0_BASEADDR USDHC0
#define BOARD_USDHC_CD_PORT_BASE PORTC
#define BOARD_USDHC_CD_GPIO_BASE GPIOC
#define BOARD_USDHC_CD_GPIO_PIN 27
#define BOARD_USDHC_CD_PORT_IRQ PORTC_IRQn
#define BOARD_USDHC_CD_PORT_IRQ_HANDLER PORTC_IRQHandler

#define BOARD_USDHC_CD_GPIO_INIT()                                                                                \
    {                                                                                                             \
        gpio_pin_config_t sw_config = {kGPIO_DigitalInput, 0};                                                    \
        GPIO_PinInit(BOARD_USDHC_CD_GPIO_BASE, BOARD_USDHC_CD_GPIO_PIN, &sw_config);                              \
        PORT_SetPinInterruptConfig(BOARD_USDHC_CD_PORT_BASE, BOARD_USDHC_CD_GPIO_PIN, kPORT_InterruptRisingEdge); \
    }

#define BOARD_USDHC_CD_STATUS() (GPIO_ReadPinInput(BOARD_USDHC_CD_GPIO_BASE, BOARD_USDHC_CD_GPIO_PIN))

#define BOARD_USDHC_CD_INTERRUPT_STATUS() (GPIO_GetPinsInterruptFlags(BOARD_USDHC_CD_GPIO_BASE))
#define BOARD_USDHC_CD_CLEAR_INTERRUPT(flag) (GPIO_ClearPinsInterruptFlags(BOARD_USDHC_CD_GPIO_BASE, flag))
#define BOARD_USDHC_CARD_INSERT_CD_LEVEL (1U)
#define BOARD_USDHC0_CLK_FREQ (CLOCK_GetIpFreq(kCLOCK_Sdhc0))

#define BOARD_SD_HOST_BASEADDR BOARD_USDHC0_BASEADDR
#define BOARD_SD_HOST_CLK_FREQ BOARD_USDHC0_CLK_FREQ
#define BOARD_SD_HOST_IRQ USDHC0_IRQn
#define BOARD_SD_SUPPORT_180V (0U)
#define BOARD_MMC_HOST_BASEADDR BOARD_USDHC0_BASEADDR
#define BOARD_MMC_HOST_CLK_FREQ BOARD_USDHC0_CLK_FREQ
#define BOARD_MMC_HOST_IRQ USDHC0_IRQn
#define BOARD_MMC_VCCQ_SUPPLY kMMC_VoltageWindows270to360
#define BOARD_MMC_VCC_SUPPLY kMMC_VoltageWindows270to360
#define BOARD_MMC_PIN_CONFIG(speed, strength)

/* this define not implement, due to EVK board have no power reset circuit */
#define BOARD_SD_POWER_RESET_GPIO ()
#define BOARD_SD_POWER_RESET_GPIO_PIN ()
#define BOARD_USDHC_SDCARD_POWER_CONTROL_INIT()
#define BOARD_USDHC_SDCARD_POWER_CONTROL(state)
#define BOARD_SD_PIN_CONFIG(speed, strength)
#define BOARD_USDHC_MMCCARD_POWER_CONTROL(enable)
#define BOARD_USDHC_MMCCARD_POWER_CONTROL_INIT()

void rt_hw_board_init(void);

#endif
