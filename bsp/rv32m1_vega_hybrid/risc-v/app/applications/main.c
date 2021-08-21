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

#include <dfs.h>
#include <dfs_elm.h>

extern int mnt_init(void);

// File in SDCARD to be read
#define  SDCARD_PLAIN_FILE      "plain.dat"
#define  SDCARD_CIPHER_FILE     "cipher.dat"

// Architecture specific variables (not redefined at redef.arch file)
static pthread_t                rdthread;
static pthread_t                wrthread;

// Hybrid MUTEX (used by both architectures) -> Only declared in ONE architecture
pthread_mutex_t                 global_mutex = PTHREAD_MUTEX_INITIALIZER;

static void *sdcard_reader_thread( void *parameter )
{
    char   *filename = (char *) parameter;
    char   *data     = NULL;
    FILE   *file;
    int     num;

    rt_kprintf("%s SDCARD reader thread started (%s)...\n", RT_DEBUG_ARCH, filename);
    data = malloc( 512 );
    if( data == NULL )
    {
        rt_kprintf("%s Not enough memory! Exiting thread...\n", RT_DEBUG_ARCH);
        return NULL;
    } //endif
    
    // Try to open the file to be ciphered
    file = fopen( filename, "rb" );
    if( file == NULL )
    {
        rt_kprintf("%s File %s not found in SDCARD. Exiting thread...\n", RT_DEBUG_ARCH, filename);
        return NULL;
    } //endif
    
    // Read file in blocks and queue them to be cipherer by a consumer thread
    num = fread( data, 1, 512, file );
    rt_kprintf("%s Read %d bytes from file\n", RT_DEBUG_ARCH, num);
    
    // Finish thread
    free( data );
    rt_kprintf("%s SDCARD reader thread finished (%s)...\n", RT_DEBUG_ARCH, filename);

    return NULL;
}

static void *sdcard_writer_thread( void *parameter )
{
    char   *filename = (char *) parameter;

    rt_kprintf("%s SDCARD writer thread started (%s)...\n", RT_DEBUG_ARCH, filename);


    rt_kprintf("%s SDCARD writer thread finished (%s)...\n", RT_DEBUG_ARCH, filename);

    return NULL;
}

int main(int argc, char** argv)
{
    pthread_mutexattr_t     mattr;
    pthread_attr_t          attr;

    rt_kprintf( "%s Main thread started!\n", RT_DEBUG_ARCH );

    // Init shared inter-architecture IPC objects
    mattr = PTHREAD_MUTEX_RECURSIVE;
    pthread_mutex_init( &global_mutex, &mattr );
    
    // Mount SDCARD filesystem
    dfs_init();
    elm_init();
    if( mnt_init() )
    {
        rt_kprintf( "Please insert a SDCARD and restart system!\n", RT_DEBUG_ARCH );
        return 0;
    } //endif

    // Create SDCARD reader/writer threads and wait till they finished
    memset( &attr, 0, sizeof(attr) );
    attr.stackaddr = NULL;
    attr.stacksize = 4096;  // Define thread stack size
    attr.schedparam.sched_priority = RT_MAIN_THREAD_PRIORITY;

    pthread_create( &rdthread, &attr, sdcard_reader_thread, SDCARD_PLAIN_FILE );
    pthread_create( &wrthread, &attr, sdcard_writer_thread, SDCARD_CIPHER_FILE );
    pthread_join( rdthread, NULL );
    pthread_join( wrthread, NULL );
    rt_kprintf( "%s Main thread finished!\n", RT_DEBUG_ARCH );

    return 0;
}
