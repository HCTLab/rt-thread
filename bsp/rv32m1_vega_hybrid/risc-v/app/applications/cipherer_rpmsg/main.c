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
#include <delay.h>

#include "board.h"
#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "rpmsg_ns.h"

// Files in SDCARD to be read/writen
#define  SDCARD_PLAIN_FILE      "plain.dat"
#define  SDCARD_CIPHER_FILE     "cipher.dat"

#define  VERBOSE
//#define  TRACE_LOOP
#define  TEST_NUM                   8
#define  MAX_NUM_BLOCKS             16

// Define the number of block and the block size on each test
// Number of blocks being read/ciphered/written before locking each thread (and wait for new data)
#define  KB                         8
#define  NUM_BLOCKS                {8        ,4        ,2        ,1        ,8        ,4        ,2        ,1        }
#define  BLOCK_SIZE                {(KB*1024),(KB*1024),(KB*1024),(KB*1024),(KB*1024),(KB*1024),(KB*1024),(KB*1024)}

#define  TIME_MIN                   0
#define  TIME_MEDIUM                1
#define  TIME_MAX                   2
#define  TIME_TYPES                 3

#define  LOCAL_EPT_CIPHER_ADDR      30
#define  LOCAL_EPT_WRITE_ADDR       31
#define  REMOTE_EPT_CIPHER_ADDR     40
#define  REMOTE_EPT_WRITE_ADDR      41

// External non-declared functions/variables
extern long  rt_hw_usec_get(void);
extern int   mnt_init(void);
extern int   platform_init(void);

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

static long                     time_start, tick_start;
static long                     time_full [ TEST_NUM ];
static long                     time_ticks[ TEST_NUM ];
static long                     time_read [ TEST_NUM ][ TIME_TYPES ];
static long                     time_write[ TEST_NUM ][ TIME_TYPES ];

static int                      nblocks[TEST_NUM] = NUM_BLOCKS;
static int                      bsizes [TEST_NUM] = BLOCK_SIZE;
static int                      nrops  [TEST_NUM];
static int                      nwops  [TEST_NUM];

// RPMSG_LITE objects
void                           *rpmsg_lite_base = BOARD_SHARED_MEMORY_BASE;
struct rpmsg_lite_instance     *my_rpmsg = NULL;
struct rpmsg_lite_endpoint     *cipher_ept;
rpmsg_queue_handle              cipher_q;
struct rpmsg_lite_endpoint     *write_ept;
rpmsg_queue_handle              write_q;
void                           *tx_buf[ TEST_NUM ];

// Local architecture MUTEX and queue
//pthread_mutex_t                 local_mutex = PTHREAD_MUTEX_INITIALIZER;
block_t                         local_queue[ MAX_NUM_BLOCKS ];

sem_t                           local_read_sem;

volatile int                    global_nblocks;
volatile int                    global_bsize;


static void *sdcard_reader_thread( void *parameter )
{
    char           *filename = (char *) parameter;
    char           *data     = NULL;
    FILE           *file;
    int             test, blk, tim, num, idx, len, i;
    long            time_ini, time_end, time_op;
    static int      first_block;
    block_t        *block;
    int             error, recved;
    unsigned long   src;
    
    printf("%s SDCARD reader thread started (%s)...\n", RT_DEBUG_ARCH, filename);
    mdelay( 1000 );
    
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
        // Give some time between tests to ARM threads to be ready (and avoid using another valuable IPC synch)
        mdelay( 2000 );
        
        // Configure TEST parameters
        global_nblocks    = nblocks[ test ];
        global_bsize      = bsizes [ test ];
        
        // Prepare the number of blocks which can be read before locking the thread.
        while( sem_trywait( &local_read_sem ) == 0 );
        for( i=0; i<global_nblocks; i++ )  sem_post( &local_read_sem ); 

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
#ifdef VERBOSE
#ifdef TRACE_LOOP
            printf("%s Waiting for a new block to be read [%d] Total count [%d]\n", RT_DEBUG_ARCH, idx, blk);
#endif
#endif
            sem_wait( &local_read_sem );
            
            // On first block do some specific tasks
            if( first_block != 0 )
            {
                first_block = 0;
                printf("\n\n%s Starting test #%d...\n", RT_DEBUG_ARCH, test);
                time_start = rt_hw_usec_get();
                tick_start = rt_tick_get();
            } //endif

            // Alloc memory for a new block (it will be free by writer, once ciphered & copied to disk)
            data = malloc( global_bsize );
            if( data == NULL )
            {
                printf("%s Not enough memory! Exiting thread...\n", RT_DEBUG_ARCH);
                return NULL;
            } //endif
            
            // Get a new block from local queue
            //pthread_mutex_lock( &local_mutex );
            memset( &local_queue[idx], 0, sizeof(local_queue[idx]) );
            local_queue[idx].block = data;
            //pthread_mutex_unlock( &local_mutex );
            
            // Read a block from file
#ifdef VERBOSE
#ifdef TRACE_LOOP
            printf("%s Reading block [%d]\n", RT_DEBUG_ARCH, idx);
#else
            printf("R");
#endif
#endif
            time_ini = rt_hw_usec_get();
            len  = fread( data, 1, global_bsize, file );
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
#ifdef VERBOSE
#ifdef TRACE_LOOP
            printf("%s Marking block [%d] as read\n", RT_DEBUG_ARCH, idx);
#endif
#endif
            //pthread_mutex_lock( &local_mutex );
            local_queue[idx].is_read = 1;
            if( (len != global_bsize) || feof(file) )
            {
                local_queue[idx].is_last = 1;
            } //endif
            //pthread_mutex_unlock( &local_mutex );
            
            // Notify cipherer thread (awake it!) that a block is ready to be ciphered
            //sem_post( &global_cipher_sem );  // POSIX IPC mech to be compared
            src = 0;
            block = &local_queue[idx];
            memcpy( tx_buf[idx], &block, sizeof(block) );
            error = rpmsg_lite_send_nocopy( my_rpmsg, cipher_ept, src, tx_buf[idx], sizeof(block) );
            
            // Increase index
            idx = (idx + 1) % global_nblocks;
            blk++;
        } //wend
        
        // Finish thread
        fclose( file );
        //printf("\n%s Reader thread: %d bytes read from %s...\n", RT_DEBUG_ARCH, num, filename);

        // Calculate medium time for all read operations
        time_read[test][TIME_MEDIUM] /= tim;
        nrops[test] = blk;
    } //endfor
        
    mdelay( 1000 );
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
    block_t        *block;
    int             recved;
    unsigned long   src;

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
#ifdef VERBOSE
#ifdef TRACE_LOOP
            printf("%s Waiting for a new block to be written [%d] Total count [%d]\n", RT_DEBUG_ARCH, idx, blk);
#endif
#endif
            //sem_wait( &global_write_sem );  // POSIX IPC mech to be compared
            recved = 0;
            rpmsg_queue_recv_nocopy( my_rpmsg, write_q, &src, (void *) &block, &recved, RL_BLOCK );

            //pthread_mutex_lock( &local_mutex );
            data = block->block;
            end  = block->is_last;
            //pthread_mutex_unlock( &local_mutex );

            // Write block to file
#ifdef VERBOSE
#ifdef TRACE_LOOP
            printf("%s Writting block [%d]\n", RT_DEBUG_ARCH, idx);
#else
            printf("W");
#endif
#endif
            time_ini = rt_hw_usec_get();
            len  = fwrite( data, 1, global_bsize, file );
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
#ifdef VERBOSE
#ifdef TRACE_LOOP
            printf("%s Marking block [%d] as written\n", RT_DEBUG_ARCH, idx);
#endif
#endif
            //pthread_mutex_lock( &local_mutex );
            if( (block->is_read == 0) || (block->is_ciphered == 0) )
            {
                printf("%s Internal error while writing file...\n", RT_DEBUG_ARCH);
                return NULL;
            } //endif
            memset( block, 0, sizeof(local_queue[idx]) );
            //pthread_mutex_unlock( &local_mutex );
            
            sem_post( &local_read_sem );

            // Increase index
            idx = (idx + 1) % global_nblocks;
            blk++;

            // Check for end condition
            if( end != 0 )  break;
        } //wend
        
        // Calculate full process timing
        time_full [test] = rt_hw_usec_get() - time_start;
        time_ticks[test] = rt_tick_get()    - tick_start;
        printf("\n%s Ending test #%d... Duration [%ld] usecs ", RT_DEBUG_ARCH, test, time_full [test]);

        // Calculate medium time for all write operations
        time_write[test][TIME_MEDIUM] /= tim;
        nwops[test] = blk;

        // Finish thread
        fclose( file );
        //printf("\n%s Writer thread: %d bytes written to %s...\n", RT_DEBUG_ARCH, num, filename);
    } //endfor

    printf("\n\n%s SDCARD writer thread finished...\n", RT_DEBUG_ARCH);

    // Report all timing
    mdelay( 2000 );
    time_ini = rt_hw_usec_get();
    time_end = rt_hw_usec_get();
    //printf("\n\n%s Error in measures (in usecs): %ld\n", RT_DEBUG_ARCH, time_end-time_ini);

    printf("\n%s -------------------------------------------------- TIMING REPORT (usecs) --------------------------------------------------\n", RT_DEBUG_ARCH);
    for( idx=0; idx<TEST_NUM; idx++ )
    {
        printf("%s TEST #%02d : POPS [%2d] BSIZE [%5d] OPS [%4d] --- TOTAL [%9ld] TICKS [%6ld] --- RMIN [%8ld] RMED [%8ld] RMAX [%8ld]\n", 
               RT_DEBUG_ARCH, idx, nblocks[idx], bsizes[idx], nrops[idx],
               time_full[idx], time_ticks[idx], time_read[idx][TIME_MIN],  time_read[idx][TIME_MEDIUM],  time_read[idx][TIME_MAX]);
    } //endfor
    printf("\n");
    for( idx=0; idx<TEST_NUM; idx++ )
    {
        printf("%s TEST #%02d : POPS [%2d] BSIZE [%5d] OPS [%4d] --- TOTAL [%9ld] TICKS [%6ld] --- WMIN [%8ld] WMED [%8ld] WMAX [%8ld]\n", 
               RT_DEBUG_ARCH, idx, nblocks[idx], bsizes[idx], nwops[idx],
               time_full[idx], time_ticks[idx], time_write[idx][TIME_MIN], time_write[idx][TIME_MEDIUM], time_write[idx][TIME_MAX]);
    } //endfor

    return NULL;
}

int main( int argc, char **argv )
{
    pthread_mutexattr_t     mattr;
    pthread_attr_t          attr;
    unsigned long           size, i;
    
    // Program starts!
    printf( "%s Main thread started!\n", RT_DEBUG_ARCH );
    
    // Init RPMSG_LITE OpenAMP env
    platform_init();
    //env_init();  // Called from 'rpmsg_lite_remote_init()'
    my_rpmsg = rpmsg_lite_master_init( rpmsg_lite_base, BOARD_SHARED_MEMORY_SIZE, RL_PLATFORM_RV32M1_M4_M0_LINK_ID, RL_NO_FLAGS );
    
    // Create RPMSG_LITE endpoints and queues
    cipher_q   = rpmsg_queue_create( my_rpmsg );
    cipher_ept = rpmsg_lite_create_ept( my_rpmsg, LOCAL_EPT_CIPHER_ADDR, rpmsg_queue_rx_cb, cipher_q );
    write_q    = rpmsg_queue_create( my_rpmsg );
    write_ept  = rpmsg_lite_create_ept( my_rpmsg, LOCAL_EPT_WRITE_ADDR, rpmsg_queue_rx_cb, write_q );
    
    // Wait till RPMSG is ready
    while( !rpmsg_lite_is_link_up(my_rpmsg) ) { printf("."); sleep(1); }

    // Init shared inter-architecture IPC objects
    mattr = PTHREAD_MUTEX_RECURSIVE;
    //pthread_mutex_init( &local_mutex, &mattr );
    sem_init( &local_read_sem,   1, 0 );    // It will be re-initialized by reader thread at the beginning of each test
    //sem_init( &global_write_sem,  1, 0 );
    memset( local_queue, 0, sizeof(local_queue) );

    // Alloc RPMSG Lite messages that can transmitted
    for( i=0; i<TEST_NUM; i++ )
    {
        tx_buf[i] = rpmsg_lite_alloc_tx_buffer( my_rpmsg, &size, 0 );  // 0 == RL_NONBLOCK
        if( tx_buf[i] == NULL )
        {
            printf( "%s Not enough memory for RPMSG Lite message!\n", RT_DEBUG_ARCH );
            return 0;
        } //endif
    } //endfor
    
    // Mount SDCARD filesystem
    if( mnt_init() != 0 )
    {
        printf( "%s Please insert a SDCARD and restart the system!\n", RT_DEBUG_ARCH );
        return 0;
    } //endif

    // Create SDCARD reader/writer threads and wait till they finished
    memset( &attr, 0, sizeof(attr) );
    attr.stackaddr = NULL;
    attr.stacksize = 4096;  // Define thread stack size
    attr.schedparam.sched_priority = RT_MAIN_THREAD_PRIORITY;

    pthread_create( &rdthread, &attr, sdcard_reader_thread, SDCARD_PLAIN_FILE );
    pthread_create( &wrthread, &attr, sdcard_writer_thread, SDCARD_CIPHER_FILE );
    //list_thread();
    pthread_join( rdthread, NULL );
    pthread_join( wrthread, NULL );
   
    printf( "\n%s Main thread finished!\n", RT_DEBUG_ARCH );

    return 0;
}
