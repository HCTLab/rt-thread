/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021/08/26     Juancho      Hybrid version
 */

// Standard headers
#include <stdio.h>
#include <stdlib.h>
#include <rtthread.h>
#include <pthread.h>
#include <semaphore.h>

// RT-Thread headers
#include <libc.h>
#include <dfs.h>
#include <dfs_elm.h>

// File in SDCARD to be read
#define  SDCARD_PLAIN_FILE      "plain.dat"
#define  SDCARD_CIPHER_FILE     "cipher.dat"

#define  NUM_BLOCKS             10
#define  BLOCK_SIZE             2048

// External non-defined functions
extern int mnt_init(void);

// Define types
typedef struct
{
    unsigned char   is_read;
    unsigned char   is_ciphered;
    unsigned char   is_last;
    
    void           *block;
    
} block_t;

// Architecture specific variables (not redefined at redef.arch file)
static pthread_t                rdthread;
static pthread_t                wrthread;

// Hybrid MUTEX (used by both architectures) -> Only declared in ONE architecture
pthread_mutex_t                 global_mutex = PTHREAD_MUTEX_INITIALIZER;

sem_t                           global_read_sem;
sem_t                           global_cipher_sem;
sem_t                           global_write_sem;

block_t                         global_queue[ NUM_BLOCKS ];

static void *sdcard_reader_thread( void *parameter )
{
    char   *filename = (char *) parameter;
    char   *data     = NULL;
    FILE   *file;
    int     blk, len, num, idx;

    printf("%s SDCARD reader thread started (%s)...\n", RT_DEBUG_ARCH, filename);
    
    // Try to open the file to be ciphered
    file = fopen( filename, "rb" );
    if( file == NULL )
    {
        printf("%s File %s not found in SDCARD. Exiting thread...\n", RT_DEBUG_ARCH, filename);
        return NULL;
    } //endif
    
    // Read file in blocks and queue them to be ciphered by a consumer thread
    blk = 0;
    num = 0;
    idx = 0;
    
    while( !feof(file) )
    {
        // Alloc memory for a new block (it will be free by writer, once ciphered & copied to disk)
        data = malloc( BLOCK_SIZE );
        if( data == NULL )
        {
            printf("%s Not enough memory! Exiting thread...\n", RT_DEBUG_ARCH);
            return NULL;
        } //endif
        
        // Wait for a free block and use such block for reading
        printf("%s Waiting for a new block to be read [%d]\n", RT_DEBUG_ARCH, blk);
        sem_wait( &global_read_sem );
        
        // Do internal checks
        pthread_mutex_lock( &global_mutex );
        if( (global_queue[idx].is_read != 0) || (global_queue[idx].is_ciphered != 0) )
        {
            printf("%s Internal error while reading file...\n", RT_DEBUG_ARCH);
            return NULL;
        } //endif
        global_queue[idx].block = data;
        pthread_mutex_unlock( &global_mutex );
        
        // Read a block from file
        printf("%s Reading block [%d]\n", RT_DEBUG_ARCH, blk);
        len  = fread( data, 1, BLOCK_SIZE, file );
        num += len;
        
        // Mark block as read, and move semaphore to let the cipherer run
        printf("%s Marking block [%d] as read\n", RT_DEBUG_ARCH, blk);
        pthread_mutex_lock( &global_mutex );
        global_queue[idx].is_read = 1;
        if( (len != BLOCK_SIZE) || feof(file) )
        {
            global_queue[idx].is_last = 1;
        } //endif
        pthread_mutex_unlock( &global_mutex );
        sem_post( &global_cipher_sem );
        
        // Increase index
        idx = (idx + 1) % NUM_BLOCKS;
        blk++;
    } //wend
    
    // Finish thread
    printf("%s SDCARD reader thread finished (%s)...\n", RT_DEBUG_ARCH, filename);

    return NULL;
}

static void *sdcard_writer_thread( void *parameter )
{
    char   *filename = (char *) parameter;
    char   *data     = NULL;
    FILE   *file;
    int     blk, end, num, idx;

    printf("%s SDCARD writer thread started (%s)...\n", RT_DEBUG_ARCH, filename);
    
    // Try to open the file to be writen
    file = fopen( filename, "wb" );
    if( file == NULL )
    {
        printf("%s File %s could not be created in SDCARD. Exiting thread...\n", RT_DEBUG_ARCH, filename);
        return NULL;
    } //endif
    
    // Read file in blocks and queue them to be ciphered by a consumer thread
    blk = 0;
    num = 0;
    idx = 0;
    
    while( 1 )
    {
        // Wait for a ciphered block and get (in exclusive mode) a new block to be used for reading
        printf("%s Waiting for a new block to be written [%d]\n", RT_DEBUG_ARCH, blk);
        sem_wait( &global_write_sem );

        pthread_mutex_lock( &global_mutex );
        data = global_queue[idx].block;
        end  = global_queue[idx].is_last;
        pthread_mutex_unlock( &global_mutex );
        
        // Check for end condition
        if( end != 0 )  break;

        // Write block to file
        printf("%s Writting block [%d]\n", RT_DEBUG_ARCH, blk);
        num += fwrite( data, 1, BLOCK_SIZE, file );
        
        // Free allocated block data
        free( data );
        
        // Mark block as written, and move semaphore to let the reader run and repeat the loop
        printf("%s Marking block [%d] as written\n", RT_DEBUG_ARCH, blk);
        pthread_mutex_lock( &global_mutex );
        if( (global_queue[idx].is_read == 0) || (global_queue[idx].is_ciphered == 0) )
        {
            printf("%s Internal error while wrtiting file...\n", RT_DEBUG_ARCH);
            return NULL;
        } //endif
        memset( &global_queue[idx], 0, sizeof(global_queue[idx]) );
        pthread_mutex_unlock( &global_mutex );
        sem_post( &global_read_sem );
        
        // Increase index
        idx = (idx + 1) % NUM_BLOCKS;
        blk++;
    } //wend
    
    // Finish thread
    printf("%s SDCARD writer thread finished (%s)...\n", RT_DEBUG_ARCH, filename);

    return NULL;
}

int main(int argc, char** argv)
{
    pthread_mutexattr_t     mattr;
    pthread_attr_t          attr;

    // Init some subsystems/components
    dfs_init();
    elm_init();
    
    // Configure STDIO stdin/stdout to point to serial port
    // Note: Must be called always after dfs_init(), to set allow setting the console as stdio/stderr
    //libc_stdio_set_console(RT_CONSOLE_DEVICE_NAME, O_RDWR);
    libc_system_init();
    
    // Program starts!
    printf( "%s Main thread started!\n", RT_DEBUG_ARCH );

    // Init shared inter-architecture IPC objects
    mattr = PTHREAD_MUTEX_RECURSIVE;
    pthread_mutex_init( &global_mutex, &mattr );
    sem_init( &global_read_sem,   1, NUM_BLOCKS );
    sem_init( &global_cipher_sem, 1, 0 );
    sem_init( &global_write_sem,  1, 0 );
    memset( global_queue, 0, sizeof(global_queue) );
    
    // Mount SDCARD filesystem
    if( mnt_init() != 0 )
    {
        printf( "Please insert a SDCARD and restart the system!\n", RT_DEBUG_ARCH );
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
    printf( "%s Main thread finished!\n", RT_DEBUG_ARCH );

    return 0;
}
