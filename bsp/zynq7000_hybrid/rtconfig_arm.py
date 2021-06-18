import os

# toolchains options
ARCH='arm'
CPU='zynq7000'
CROSS_TOOL='gcc'

if os.getenv('RTT_CC'):
    CROSS_TOOL = os.getenv('RTT_CC')

# only support GNU GCC compiler
PLATFORM 	 = 'gcc'
EXEC_PATH 	 = '/c/Xilinx/SDK/2019.1/gnu/aarch32/nt/gcc-arm-none-eabi/bin'
GCC_INC_PATH = '/c/Xilinx/SDK/2019.1/gnu/aarch32/nt/gcc-arm-none-eabi/lib/gcc/arm-none-eabi/8.2.0/include'

if os.getenv('RTT_EXEC_PATH'):
    EXEC_PATH = os.getenv('RTT_EXEC_PATH')

BUILD = 'debug'

if PLATFORM == 'gcc':
    # toolchains
    PREFIX = 'arm-none-eabi-'
    CC = PREFIX + 'gcc'
    CXX = PREFIX + 'g++'
    AS = PREFIX + 'gcc'
    AR = PREFIX + 'ar'
    LINK = PREFIX + 'gcc'
    TARGET_EXT = 'elf'
    SIZE = PREFIX + 'size'
    OBJDUMP = PREFIX + 'objdump'
    OBJCPY = PREFIX + 'objcopy'
    #ARCH    = -mcpu=cortex-a9 -mfpu=vfpv3
    #DEVICE = ' -Wall -mcpu=cortex-a9 -mfpu=vfpv3 -ftree-vectorize -ffast-math -mfloat-abi=softfp -ffreestanding -nostdinc -I' + GCC_INC_PATH   #(JAAS)
    DEVICE = ' -Wall -mcpu=cortex-a9 -mfpu=vfpv3 -ftree-vectorize -ffast-math -mfloat-abi=softfp'
    CFLAGS = DEVICE
    AFLAGS = ' -c' + DEVICE + ' -x assembler-with-cpp'
    LINK_SCRIPT = 'zynq7000.ld'
    LFLAGS = DEVICE + ' -Wl,--gc-sections,-Map=zynq7000.map,-cref,-u,system_vectors -T %s' % LINK_SCRIPT

    CPATH = ''
    LPATH = ''

    if BUILD == 'debug':
        CFLAGS += ' -O0 -gdwarf-2'
        AFLAGS += ' -gdwarf-2'
    else:
        CFLAGS += ' -O2'

    POST_ACTION = OBJCPY + ' -O binary $TARGET rtthread.bin\n' +\
            SIZE + ' $TARGET \n'
