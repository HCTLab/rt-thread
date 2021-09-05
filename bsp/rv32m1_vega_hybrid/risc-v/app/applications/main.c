/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021/08/26     Juancho      Hybrid version
 */

// RT-Thread headers
#include <rtthread.h>
#include <libc.h>
#include <dfs.h>
#include <dfs_elm.h>

// Standard headers
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>

// Files in SDCARD to be read/writen
#define  SDCARD_PLAIN_FILE      "plain.dat"
#define  SDCARD_CIPHER_FILE     "cipher.dat"

//#define TRACE_LOOP
#define  TEST_NUM               4
#define  NUM_BLOCKS             8
#define  BLOCK_SIZE            (16*1048)

#define  TIME_MIN               0
#define  TIME_MEDIUM            1
#define  TIME_MAX               2
#define  TIME_TYPES             3

// External non-declared functions/variables
extern long  rt_hw_usec_get(void);
extern int   mnt_init(void);
extern int   is_preemtive;

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

static long                     time_start;
static long                     time_full [ TEST_NUM ];
static long                     time_read [ TEST_NUM ][ TIME_TYPES ];
static long                     time_write[ TEST_NUM ][ TIME_TYPES ];

// Hybrid MUTEX (used by both architectures) -> Only declared in ONE architecture
pthread_mutex_t                 global_mutex = PTHREAD_MUTEX_INITIALIZER;

sem_t                           global_read_sem;
sem_t                           global_cipher_sem;
sem_t                           global_write_sem;

block_t                         global_queue[ NUM_BLOCKS ];


static void *sdcard_reader_thread( void *parameter )
{
    char           *filename = (char *) parameter;
    char           *data     = NULL;
    FILE           *file;
    int             test, blk, tim, num, idx, len, first_block, i;
    long            time_ini, time_end, time_op;

    printf("%s SDCARD reader thread started (%s)...\n", RT_DEBUG_ARCH, filename);
    
    /* Verify rt_hw_usec_get() is returning good values (and there is no tick overflow)
    for(tim=1;;tim++)
    {
        time_ini = rt_hw_usec_get();
        time_end = rt_hw_usec_get();
        time_op  = time_end - time_ini;
        if( (time_op<0) || (time_op>100) )
        {
            printf("TIME [%8ld] ITER [%5d]\n", time_op, tim);
            tim = 0;
        } //endif
    } //wend
    */
    
    // Execute multiple times this test
    for( test=0; test<TEST_NUM; test++ )
    {
        // Enable/disable preemption on other core according to test number
        if( (test % 2) == 0 )  is_preemtive = 1;
        else                   is_preemtive = 0;
        
        // Try to open the file to be ciphered
        file = fopen( filename, "rb" );
        if( file == NULL )
        {
            printf("%s File %s not found in SDCARD. Exiting thread...\n", RT_DEBUG_ARCH, filename);
            return NULL;
        } //endif
        
        // Read file in blocks and queue them to be ciphered by a consumer thread
        blk = 0;
        tim = 0;
        num = 0;
        idx = 0;
        first_block = 1;
        time_read[test][TIME_MIN]    = 1000000000L;
        time_read[test][TIME_MEDIUM] = 0L;
        time_read[test][TIME_MAX]    = 0L;
        
        while( !feof(file) )
        {
            // Wait for a free block and use such block for reading
#ifdef TRACE_LOOP
            printf("%s Waiting for a new block to be read [%d]\n", RT_DEBUG_ARCH, blk);
#endif
            sem_wait( &global_read_sem );
            
            // On first block do some specific tasks
            if( first_block != 0 )
            {
                printf("\n\n%s Starting test #%d...\n", RT_DEBUG_ARCH, test);
                time_start = rt_hw_usec_get();
                first_block = 0;
            } //endif

            // Alloc memory for a new block (it will be free by writer, once ciphered & copied to disk)
            data = malloc( BLOCK_SIZE );
            if( data == NULL )
            {
                printf("%s Not enough memory! Exiting thread...\n", RT_DEBUG_ARCH);
                return NULL;
            } //endif
            
            // Do internal checks (detect concurrent block usage)
            pthread_mutex_lock( &global_mutex );
            if( (global_queue[idx].is_read != 0) || (global_queue[idx].is_ciphered != 0) )
            {
                printf("%s Internal error while reading file...\n", RT_DEBUG_ARCH);
                return NULL;
            } //endif
            global_queue[idx].block = data;
            pthread_mutex_unlock( &global_mutex );
            
            // Read a block from file
#ifdef TRACE_LOOP
            printf("%s Reading block [%d]\n", RT_DEBUG_ARCH, blk);
#else
            printf("R");
#endif
            time_ini = rt_hw_usec_get();
            len  = fread( data, 1, BLOCK_SIZE, file );
            num += len;
            time_end = rt_hw_usec_get();
            time_op  = time_end - time_ini;
            
            // Save timings
            if( time_op > 0 )  // Sometimes rt_hw_usec_get() takes an earlier tick (it's not IRQ protected)
            {
                time_read[test][TIME_MEDIUM] += time_op;
                if( time_op < time_read[test][TIME_MIN] ) time_read[test][TIME_MIN] = time_op;
                if( time_op > time_read[test][TIME_MAX] ) time_read[test][TIME_MAX] = time_op;
                tim++;
            } //endif
            
            // Mark block as read, and move semaphore to let the cipherer thread to run
#ifdef TRACE_LOOP
            printf("%s Marking block [%d] as read\n", RT_DEBUG_ARCH, blk);
#endif
            pthread_mutex_lock( &global_mutex );
            global_queue[idx].is_read = 1;
            if( (len != BLOCK_SIZE) || feof(file) )
            {
                global_queue[idx].is_last = 1;
            } //endif
            pthread_mutex_unlock( &global_mutex );
            
            // Notify cipherer thread (awake it!) that a block is ready to be ciphered
            sem_post( &global_cipher_sem );
            
            // Increase index
            idx = (idx + 1) % NUM_BLOCKS;
            blk++;
        } //wend
        
        // Finish thread
        fclose( file );
        //printf("\n%s Reader thread: %d bytes read from %s...\n", RT_DEBUG_ARCH, num, filename);

        // Calculate medium time for all read operations
        time_read[test][TIME_MEDIUM] /= tim;
        
        // Get all reader slots, till they will post on next test (to sync all thread when starting a new test)
        for( i=0; i<NUM_BLOCKS; i++ )
        {
            sem_wait( &global_read_sem );
        } //endfor
    } //endfor
        
    printf("%s SDCARD reader thread finished...\n", RT_DEBUG_ARCH);

    return NULL;
}

static void *sdcard_writer_thread( void *parameter )
{
    char           *filename = (char *) parameter;
    char           *data     = NULL;
    FILE           *file;
    int             test, blk, tim, num, idx, len, end;
    long            time_ini, time_end, time_op;

    printf("%s SDCARD writer thread started (%s)...\n", RT_DEBUG_ARCH, filename);
    
    // Execute multiple times this test
    for( test=0; test<TEST_NUM; test++ )
    {
        // Try to open the file to be writen
        file = fopen( filename, "wb" );
        if( file == NULL )
        {
            printf("%s File %s could not be created in SDCARD. Exiting thread...\n", RT_DEBUG_ARCH, filename);
            return NULL;
        } //endif
        
        // Read file in blocks and queue them to be ciphered by a consumer thread
        blk = 0;
        tim = 0;
        num = 0;
        idx = 0;
        time_write[test][TIME_MIN]    = 1000000000L;
        time_write[test][TIME_MEDIUM] = 0L;
        time_write[test][TIME_MAX]    = 0L;
        
        while( 1 )
        {
            // Wait for a ciphered block and get (in exclusive mode) a new block to be used for reading
#ifdef TRACE_LOOP
            printf("%s Waiting for a new block to be written [%d]\n", RT_DEBUG_ARCH, blk);
#endif
            sem_wait( &global_write_sem );

            pthread_mutex_lock( &global_mutex );
            data = global_queue[idx].block;
            end  = global_queue[idx].is_last;
            pthread_mutex_unlock( &global_mutex );
            
            // Write block to file
#ifdef TRACE_LOOP
            printf("%s Writting block [%d]\n", RT_DEBUG_ARCH, blk);
#else
            printf("W");
#endif
            time_ini = rt_hw_usec_get();
            len  = fwrite( data, 1, BLOCK_SIZE, file );
            num += len;
            fflush( file );
            time_end = rt_hw_usec_get();
            time_op  = time_end - time_ini;
            
            // Save timings
            if( time_op > 0 )  // Sometimes rt_hw_usec_get() takes an earlier tick (it's not IRQ protected)
            {
                time_write[test][TIME_MEDIUM] += time_op;
                if( time_op < time_write[test][TIME_MIN] ) time_write[test][TIME_MIN] = time_op;
                if( time_op > time_write[test][TIME_MAX] ) time_write[test][TIME_MAX] = time_op;
                tim++;
            } //endif
            
            // Free allocated block data
            free( data );
            
            // Mark block as written, and move semaphore to let the reader thread to run and repeat the big loop
#ifdef TRACE_LOOP
            printf("%s Marking block [%d] as written\n", RT_DEBUG_ARCH, blk);
#endif
            pthread_mutex_lock( &global_mutex );
            if( (global_queue[idx].is_read == 0) || (global_queue[idx].is_ciphered == 0) )
            {
                printf("%s Internal error while writing file...\n", RT_DEBUG_ARCH);
                return NULL;
            } //endif
            memset( &global_queue[idx], 0, sizeof(global_queue[idx]) );
            pthread_mutex_unlock( &global_mutex );
            sem_post( &global_read_sem );

            // Increase index
            idx = (idx + 1) % NUM_BLOCKS;
            blk++;

            // Check for end condition
            if( end != 0 )  break;
        } //wend
        
        // Calculate full process timing
        time_full[test] = rt_hw_usec_get() - time_start;

        // Calculate medium time for all write operations
        time_write[test][TIME_MEDIUM] /= tim;

        // Finish thread
        fclose( file );
        //printf("\n%s Writer thread: %d bytes written to %s...\n", RT_DEBUG_ARCH, num, filename);
    } //endfor

    printf("\n\n%s SDCARD writer thread finished...\n", RT_DEBUG_ARCH);

    // Report all timing
    sleep(2);
    time_ini = rt_hw_usec_get();
    time_end = rt_hw_usec_get();
    printf("\n\nMin measured time (usecs): %ld\n", time_end-time_ini);

    printf("\n%s ------------------- TIMING REPORT (usecs) -------------------\n", RT_DEBUG_ARCH);
    for( idx=0; idx<TEST_NUM; idx++ )
    {
        printf("%s TEST #%d : OPS [%02d] --- TOTAL [%9ld] --- RMIN [%8ld] RMED [%8ld] RMAX [%8ld]\n", 
               RT_DEBUG_ARCH, idx, blk, time_full[idx], 
               time_read[idx][TIME_MIN],  time_read[idx][TIME_MEDIUM],  time_read[idx][TIME_MAX]);
    } //endfor
    printf("\n");
    for( idx=0; idx<TEST_NUM; idx++ )
    {
        printf("%s TEST #%d : OPS [%02d] --- TOTAL [%9ld] --- WMIN [%8ld] WMED [%8ld] WMAX [%8ld]\n", 
               RT_DEBUG_ARCH, idx, blk, time_full[idx], 
               time_write[idx][TIME_MIN], time_write[idx][TIME_MEDIUM], time_write[idx][TIME_MAX]);
    } //endfor

    return NULL;
}

int main( int argc, char **argv )
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
    sem_init( &global_read_sem,   1, 1 );  sem_wait( &global_read_sem );
    sem_init( &global_cipher_sem, 1, 1 );  sem_wait( &global_cipher_sem );
    sem_init( &global_write_sem,  1, 1 );  sem_wait( &global_write_sem );
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
   
    printf( "\n%s Main thread finished!\n", RT_DEBUG_ARCH );

    return 0;
}
