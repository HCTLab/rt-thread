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

// Standard headers
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <semaphore.h>

// Block sizes
//#define TRACE_LOOP
#define  TEST_NUM               4
#define  NUM_BLOCKS             8
#define  BLOCK_SIZE            (16*1048)

#define  printf                 rt_kprintf

// External non-declared functions/variables
extern long rt_hw_usec_get(void);
extern int  is_preemtive;

// Define types
typedef struct
{
    unsigned char   is_read;
    unsigned char   is_ciphered;
    unsigned char   is_last;
    
    void           *block;
    
} block_t;

// Architecture specific variables (not redefined at redef.arch file)
static pthread_t                cthread;

// Hybrid MUTEX (used by both architectures) -> Only declared in ONE architecture
extern pthread_mutex_t          global_mutex;

extern sem_t                    global_read_sem;
extern sem_t                    global_cipher_sem;
extern sem_t                    global_write_sem;

extern block_t                  global_queue[ NUM_BLOCKS ];

static void *cipher_thread( void *parameter )
{
    char   *data = NULL;
    int     test, blk, end, num, idx, i;

    printf("%s CIPHER thread started...\n", RT_DEBUG_ARCH);
    
    // Execute multiple times this test
    for( test=0; test<TEST_NUM; test++ )
    {
        // Enable/disable preemption on other core according to test number
        if( (test % 2) == 0 )  is_preemtive = 1;
        else                   is_preemtive = 0;

        // Unlock main reader thread on other core to start reading
        for( i=0; i<NUM_BLOCKS; i++ )
        {
            sem_post( &global_read_sem );
        } //endfor
        printf("+");
        
        // Read file in blocks and queue them to be ciphered by a consumer thread
        blk = 0;
        num = 0;
        idx = 0;
        
        while( 1 )
        {
            // Wait for a ciphered block and get (in exclusive mode) a new block to be used for reading
#ifdef TRACE_LOOP
            printf("%s Waiting for a new block to be ciphered [%d]\n", RT_DEBUG_ARCH, blk);
#endif
            sem_wait( &global_cipher_sem );

            // Do internal checks
            pthread_mutex_lock( &global_mutex );
            if( (global_queue[idx].is_read == 0) || (global_queue[idx].is_ciphered != 0) )
            {
                printf("%s Internal error while ciphering block...\n", RT_DEBUG_ARCH);
                return NULL;
            } //endif
            data = global_queue[idx].block;
            end  = global_queue[idx].is_last;
            pthread_mutex_unlock( &global_mutex );
            
            // Cipher block
#ifdef TRACE_LOOP
            printf("%s Ciphering block [%d]\n", RT_DEBUG_ARCH, blk);
#else
            printf("C");
#endif
            for( i=0; i<BLOCK_SIZE; i++ )
            {
                data[i] = data[i] ^ 0x0F;
            } //endfor
            num += BLOCK_SIZE;
            //usleep( 2000000L );   //(JAAS) Uncomment to simulate a long cipher operation
            
            // Mark block as ciphered, and move semaphore to let the writer thread to run
#ifdef TRACE_LOOP
            printf("%s Marking block [%d] as ciphered\n", RT_DEBUG_ARCH, blk);
#endif
            pthread_mutex_lock( &global_mutex );
            global_queue[idx].is_ciphered = 1;
            pthread_mutex_unlock( &global_mutex );
            sem_post( &global_write_sem );

            // Check for end condition (after unlocking SDCARD writer thread)
            if( end != 0 )  break;
            
            // Increase index
            idx = (idx + 1) % NUM_BLOCKS;
            blk++;
        } //wend

        // Wait some seconds before starting a new test
        //printf("\n%s Cipher thread: %d bytes ciphered\n", RT_DEBUG_ARCH, num);
        sleep(2);  
    } //endfor
    
    // Finish thread
    printf("\n%s CIPHER thread finished...\n", RT_DEBUG_ARCH);

    return NULL;
}

int main( int argc, char **argv )
{
    pthread_attr_t          attr;

    // Configure STDIO stdin/stdout to point to serial port
    // Note: Must be called always after dfs_init(), to set allow setting the console as stdio/stderr
    //libc_stdio_set_console(RT_CONSOLE_DEVICE_NAME, O_RDWR);
    //libc_system_init();
    
    // Program starts!
    printf( "%s Main thread started!\n", RT_DEBUG_ARCH );

    // Create SDCARD reader/writer threads and wait till they finished
    memset( &attr, 0, sizeof(attr) );
    attr.stackaddr = NULL;
    attr.stacksize = 4096;  // Define thread stack size
    attr.schedparam.sched_priority = RT_MAIN_THREAD_PRIORITY;

    pthread_create( &cthread, &attr, cipher_thread, NULL );
    pthread_join( cthread, NULL );
    printf( "\n%s Main thread finished!\n", RT_DEBUG_ARCH );

    return 0;
}
