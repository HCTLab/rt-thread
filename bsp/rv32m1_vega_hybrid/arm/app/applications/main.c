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
#include <pthread.h>

#include <fsl_sema42.h>
#include <fsl_lpuart.h>


#define APP_SEMA42              SEMA420     // HW instance
#define SEMA42_GATE             0U          // The SEMA42 gate (up to 15)
#define LOCK_CORE               1U          // Core 0 (CM0+) locking identifier

static pthread_t       rwthread1;
static pthread_t       rwthread2;

static void *rw_thread( void *parameter )
{
    char  data = *((char *)parameter);
    int   i    = 0;

    rt_kprintf("ARM thread working (%p=%c)...\n", parameter, data);

    while(1)
    {
        if( (i%10) == 0 )
        {
            SEMA42_Lock(APP_SEMA42, SEMA42_GATE, LOCK_CORE);
            //LPUART_WriteByte(LPUART0, 'l');
        } //endif

        rt_thread_delay( RT_TICK_PER_SECOND / 5 );
        LPUART_WriteByte(LPUART0, data);

        if( (i%10) == 9 )
        {
            //LPUART_WriteByte(LPUART0, 'u');
            SEMA42_Unlock(APP_SEMA42, SEMA42_GATE);
        } //endif
        i++;
    } //wend

    rt_kprintf("Bye ARM thread!\n");

    return NULL;
}

int main( int argc, char **argv )
{
    static char  t1 = '1';
    static char  t2 = '2';

    pthread_create( &rwthread1, NULL, rw_thread, &t1 );
    pthread_create( &rwthread2, NULL, rw_thread, &t2 );

    return 0;
}
