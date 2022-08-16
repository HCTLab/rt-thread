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
#include <delay.h>

//#define  VERBOSE
//#define  TRACE_LOOP
//#define  PROTECT_GLOBAL_QUEUE     // Uncomment to obtain an equivalent algorith to non-hybrid app
#define  TEST_NUM               8
#define  MAX_NUM_BLOCKS         8

#define  TIME_MIN               0
#define  TIME_MEDIUM            1
#define  TIME_MAX               2
#define  TIME_TYPES             3

#define  printf                 rt_kprintf

// External non-declared functions/variables
extern long  rt_hw_usec_get(void);
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
static pthread_t                cthread;

static long                     time_cipher[ TEST_NUM ][ TIME_TYPES ];

// Hybrid MUTEX (used by both architectures) -> Only declared in ONE architecture
#ifdef PROTECT_GLOBAL_QUEUE
extern pthread_mutex_t          global_mutex;
#endif

extern sem_t                    global_read_sem;
extern sem_t                    global_cipher_sem;
extern sem_t                    global_write_sem;

extern block_t                  global_queue[ MAX_NUM_BLOCKS ];

extern int                      global_nblocks;
extern int                      global_bsize;

// Cipher algorithm (a nice Bob Jenkins's cipherer taken from https://burtleburtle.net/bob/c/myblock.c)
typedef  unsigned int  ub4;        // 256 bit-block ciphering
//typedef  unsigned long long  ub8;  // 512 bit-block ciphering

#define mix32(a,b,c,d,e,f,g,h) \
{ \
   a-=e; f^=h>>8;  h+=a; \
   b-=f; g^=a<<8;  a+=b; \
   c-=g; h^=b>>11; b+=c; \
   d-=h; a^=c<<3;  c+=d; \
   e-=a; b^=d>>6;  d+=e; \
   f-=b; c^=e<<4;  e+=f; \
   g-=c; d^=f>>13; f+=g; \
   h-=d; e^=g<<13; g+=h; \
}

#define unmix32(a,b,c,d,e,f,g,h) \
{ \
   g-=h; e^=g<<13; h+=d; \
   f-=g; d^=f>>13; g+=c; \
   e-=f; c^=e<<4;  f+=b; \
   d-=e; b^=d>>6;  e+=a; \
   c-=d; a^=c<<3;  d+=h; \
   b-=c; h^=b>>11; c+=g; \
   a-=b; g^=a<<8;  b+=f; \
   h-=a; f^=h>>8;  a+=e; \
}

void enc32(ub4 *block, ub4 *k1, ub4 *k2)
{
  register ub4 a,b,c,d,e,f,g,h;
  a=block[0]^k1[0]; b=block[1]^k1[1]; c=block[2]^k1[2]; d=block[3]^k1[3];
  e=block[4]^k1[4]; f=block[5]^k1[5]; g=block[6]^k1[6]; h=block[7]^k1[7];
  mix32(a,b,c,d,e,f,g,h);
  mix32(a,b,c,d,e,f,g,h);
  mix32(a,b,c,d,e,f,g,h);
  mix32(a,b,c,d,e,f,g,h);
  mix32(a,b,c,d,e,f,g,h);
  mix32(a,b,c,d,e,f,g,h);
  mix32(a,b,c,d,e,f,g,h);
  mix32(a,b,c,d,e,f,g,h);
  mix32(a,b,c,d,e,f,g,h);
  mix32(a,b,c,d,e,f,g,h);
  mix32(a,b,c,d,e,f,g,h);
  mix32(a,b,c,d,e,f,g,h);
  block[0]=a^k2[0]; block[1]=b^k2[1]; block[2]=c^k2[2]; block[3]=d^k2[3];
  block[4]=e^k2[4]; block[5]=f^k2[5]; block[6]=g^k2[6]; block[7]=h^k2[7];
}

void dec32(ub4 *block, ub4 *k1, ub4 *k2)
{
  register ub4 a,b,c,d,e,f,g,h;
  a=block[0]^k2[0]; b=block[1]^k2[1]; c=block[2]^k2[2]; d=block[3]^k2[3];
  e=block[4]^k2[4]; f=block[5]^k2[5]; g=block[6]^k2[6]; h=block[7]^k2[7];
  unmix32(a,b,c,d,e,f,g,h);
  unmix32(a,b,c,d,e,f,g,h);
  unmix32(a,b,c,d,e,f,g,h);
  unmix32(a,b,c,d,e,f,g,h);
  unmix32(a,b,c,d,e,f,g,h);
  unmix32(a,b,c,d,e,f,g,h);
  unmix32(a,b,c,d,e,f,g,h);
  unmix32(a,b,c,d,e,f,g,h);
  unmix32(a,b,c,d,e,f,g,h);
  unmix32(a,b,c,d,e,f,g,h);
  unmix32(a,b,c,d,e,f,g,h);
  unmix32(a,b,c,d,e,f,g,h);
  block[0]=a^k1[0]; block[1]=b^k1[1]; block[2]=c^k1[2]; block[3]=d^k1[3];
  block[4]=e^k1[4]; block[5]=f^k1[5]; block[6]=g^k1[6]; block[7]=h^k1[7];
}

static void *cipher_thread( void *parameter )
{
    char   *data = NULL;
    int     test, blk, tim, num, idx, end, i;
    long    time_ini, time_end, time_op;
    // Cipher keys
    ub4     k1[8]={0,0,0,0,0,0,0,0}, k2[8]={0,0,0,0,0,0,0,0};

    printf("%s CIPHER thread started...\n", RT_DEBUG_ARCH);
    
    // Execute multiple times this test
    for( test=0; test<TEST_NUM; test++ )
    {
        // Read file in blocks and queue them to be ciphered by a consumer thread
        blk = 0;
        tim = 0;
        num = 0;
        idx = 0;
        time_cipher[test][TIME_MIN]    = 1000000000L;
        time_cipher[test][TIME_MEDIUM] = 0L;
        time_cipher[test][TIME_MAX]    = 0L;
        
        while( 1 )
        {
            // Wait for a ciphered block and get (in exclusive mode) a new block to be used for reading
#ifdef VERBOSE
#ifdef TRACE_LOOP
            printf("%s Waiting for a new block to be ciphered [%d] Total count [%d]\n", RT_DEBUG_ARCH, idx, blk);
#endif
#endif
            sem_wait( &global_cipher_sem );

            // Do internal checks
#ifdef PROTECT_GLOBAL_QUEUE
            pthread_mutex_lock( &global_mutex );
#endif
            if( (global_queue[idx].is_read == 0) || (global_queue[idx].is_ciphered != 0) )
            {
                printf("%s Internal error while ciphering block...\n", RT_DEBUG_ARCH);
                return NULL;
            } //endif
            data = global_queue[idx].block;
            end  = global_queue[idx].is_last;
#ifdef PROTECT_GLOBAL_QUEUE
            pthread_mutex_unlock( &global_mutex );
#endif
            
            // Cipher block
#ifdef VERBOSE
#ifdef TRACE_LOOP
            printf("%s Ciphering block [%d]\n", RT_DEBUG_ARCH, idx);
#else
            printf("C");
#endif
#endif
            time_ini = rt_hw_usec_get();
            for( i=0; i<global_bsize>>5; i++ )
            {
#ifdef DO_DECIPHER
                dec32( (ub4 *)&(((char *)data)[i<<5]), k1, k2 );
                dec32( (ub4 *)&(((char *)data)[i<<5]), k1, k2 );
#else
                enc32( (ub4 *)&(((char *)data)[i<<5]), k1, k2 );
                enc32( (ub4 *)&(((char *)data)[i<<5]), k1, k2 );
#endif
            } //endfor
            num += global_bsize;
            time_end = rt_hw_usec_get();
            time_op  = time_end - time_ini;

            // Mark block as ciphered, and move semaphore to let the writer thread to run
#ifdef VERBOSE
#ifdef TRACE_LOOP
            printf("%s Marking block [%d] as ciphered\n", RT_DEBUG_ARCH, idx);
#endif
#endif
#ifdef PROTECT_GLOBAL_QUEUE
            pthread_mutex_lock( &global_mutex );
#endif
            global_queue[idx].is_ciphered = 1;
#ifdef PROTECT_GLOBAL_QUEUE
            pthread_mutex_unlock( &global_mutex );
#endif
            sem_post( &global_write_sem );

            // Save timings
            if( time_op > 0 )  // Sometimes rt_hw_usec_get() takes an earlier tick (it's not IRQ protected)
            {
                time_cipher[test][TIME_MEDIUM] += time_op;
                if( time_op < time_cipher[test][TIME_MIN] ) time_cipher[test][TIME_MIN] = time_op;
                if( time_op > time_cipher[test][TIME_MAX] ) time_cipher[test][TIME_MAX] = time_op;
                tim++;
            } //endif

            // Increase index
            idx = (idx + 1) % global_nblocks;
            blk++;

            // Check for end condition (after unlocking SDCARD writer thread)
            if( end != 0 )  break;
        } //wend

        // Calculate medium time for all cipher operations
        time_cipher[test][TIME_MEDIUM] /= tim;

        // Wait some seconds before starting a new test
        //printf("\n%s Cipher thread: %d bytes ciphered\n", RT_DEBUG_ARCH, num);
    } //endfor
    
    // Finish thread
    mdelay( 1000 );
    printf("%s CIPHER thread finished...\n", RT_DEBUG_ARCH);

    // Report all timing
    mdelay( 2000 );
    time_ini = rt_hw_usec_get();
    time_end = rt_hw_usec_get();
    //printf("\n\n%s Error in measures (in usecs): %ld\n", RT_DEBUG_ARCH, time_end-time_ini);

    printf("\n%s ----------------------------- TIMING REPORT (usecs) -----------------------------\n", RT_DEBUG_ARCH);
    for( idx=0; idx<TEST_NUM; idx++ )
    {
        printf("%s   TEST #%02d : OPS [%02d] --- CMIN [%8ld] CMED [%8ld] CMAX [%8ld]\n", 
               RT_DEBUG_ARCH, idx, blk,
               time_cipher[idx][TIME_MIN], time_cipher[idx][TIME_MEDIUM], time_cipher[idx][TIME_MAX] );
    } //endfor

    return NULL;
}

int main( int argc, char **argv )
{
    pthread_attr_t          attr;

    // Program starts!
    printf( "\n%s Main thread started!  ----------- HYBRID IMPLEMENTATION -----------\n", RT_DEBUG_ARCH );

    // Init shared inter-architecture IPC objects
    sem_init( &global_cipher_sem, 1, 0 );

    // Create SDCARD reader/writer threads and wait till they finished
    memset( &attr, 0, sizeof(attr) );
    attr.stackaddr = NULL;
    attr.stacksize = 4096;  // Define thread stack size
    attr.schedparam.sched_priority = RT_MAIN_THREAD_PRIORITY;

    pthread_create( &cthread, &attr, cipher_thread, NULL );
    //list_thread();
    pthread_join( cthread, NULL );

    printf( "\n%s Main thread finished!\n", RT_DEBUG_ARCH );

    return 0;
}
