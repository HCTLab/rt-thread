/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021/08/26     Juancho      Hybrid version
 */

#include <stdio.h>
#include <stdlib.h>
#include <rtthread.h>
#include <pthread.h>

// Architecture specific variables (not redefined at redef.arch file)
static pthread_t                rwthread1;
static pthread_t                rwthread2;

// Hybrid MUTEX (used by both architectures) -> Only declared in ONE architecture
extern pthread_mutex_t          global_mutex;

static void *rw_thread( void *parameter )
{
    char  data = *((char *)parameter);
    int   i    = 0;

    rt_kprintf("%s Worker thread started (%p=%c)...\n", RT_DEBUG_ARCH, parameter, data);

    while(i<40)
    {
        if( (i%10) == 0 )
        {
            pthread_mutex_lock( &global_mutex );
            rt_kprintf( "%c", data+1 );
        } //endif

        rt_thread_delay( RT_TICK_PER_SECOND / 5 );
        rt_kprintf( "%c", data );

        if( (i%10) == 9 )
        {
            rt_kprintf( "%c", data-1 );
            pthread_mutex_unlock( &global_mutex );
        } //endif

        i++;
    } //wend

    rt_kprintf("%s Worker thread finished (%p=%c)...\n", RT_DEBUG_ARCH, parameter, data);

    return NULL;
}

int main( int argc, char **argv )
{
    pthread_attr_t   attr;
    char  t1 = '7';
    char  t2 = '0';

    rt_kprintf( "%s Main thread started!\n", RT_DEBUG_ARCH );

    // Define thread stack size
    memset( &attr, 0, sizeof(attr) );
    attr.stackaddr = NULL;
    attr.stacksize = 4096;
    //attr.detachstate = PTHREAD_CREATE_JOINABLE;  // PTHREAD_CREATE_JOINABLE == 0, so it's enabled by default
    attr.schedparam.sched_priority = RT_MAIN_THREAD_PRIORITY;

    pthread_create( &rwthread1, &attr, rw_thread, &t1 );
    //rt_kprintf( "%s Created thread (%ld)\n", RT_DEBUG_ARCH, rwthread1 );
    pthread_create( &rwthread2, &attr, rw_thread, &t2 );
    //rt_kprintf( "%s created thread (%ld)\n", RT_DEBUG_ARCH, rwthread2 );

    // Do not release this thread stack (thread couldn't read parameters!) till threads exit
    pthread_join( rwthread1, NULL );
    pthread_join( rwthread2, NULL );

    rt_kprintf( "%s Main thread finished!\n", RT_DEBUG_ARCH );

    return 0;
}
