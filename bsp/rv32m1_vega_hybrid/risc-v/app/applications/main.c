/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018/11/28     Bernard      The first version
 * 2020/08/26     Juancho      Hybrid version
 */

#include <stdio.h>
#include <stdlib.h>
#include <rtthread.h>

#include <fsl_sema42.h>
#include <fsl_lpuart.h>


#define APP_SEMA42              SEMA420     // HW instance
#define SEMA42_GATE             0U          // The SEMA42 gate (up to 15)
#define LOCK_CORE               0U          // Core 0 (RI5CY) locking identifier


int main(int argc, char** argv)
{
    int  i=0;

    rt_kprintf("Hello RT-Thread from RISCV!\n");
    while(1)
    {
        if( (i%10) == 0 )
        {
            SEMA42_Lock(APP_SEMA42, SEMA42_GATE, LOCK_CORE);
            //LPUART_WriteByte(LPUART0, 'L');
        } //endif

        rt_thread_delay( RT_TICK_PER_SECOND / 5 );
        LPUART_WriteByte(LPUART0, '+');

        if( (i%10) == 9 )
        {
            //LPUART_WriteByte(LPUART0, 'U');
            SEMA42_Unlock(APP_SEMA42, SEMA42_GATE);
        } //endif
        i++;
    } //wend

    return 0;
}
