import os

# core to be use
#USE_CORE = 'CORE_M0'
USE_CORE = 'CORE_M4'

# toolchains options
ARCH='arm'

if USE_CORE == 'CORE_M4':
    CPU = 'cortex-m4'
else:
    CPU = 'cortex-m0'

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
    #LINK = PREFIX + 'gcc'
    LINK = PREFIX + 'ar rcs'
    TARGET_EXT = 'elf'
    SIZE = PREFIX + 'size'
    OBJDUMP = PREFIX + 'objdump'
    OBJCPY = PREFIX + 'objcopy'
    DEVICE = ' -mcpu=' + CPU + ' -mthumb -ffunction-sections -fdata-sections -Wall'
    if USE_CORE == 'CORE_M4':
        DEVICE += ' -mfpu=fpv4-sp-d16 -mfloat-abi=softfp'
    CFLAGS = DEVICE + ' -I$BSP_ROOT -I$ARCH'
    AFLAGS = ' -c' + DEVICE + ' -x assembler-with-cpp -Wa,-mimplicit-it=thumb -I $BSP_ROOT -I $ARCH'
    LFLAGS = ''
    LIBS = ''

    CPATH = ''
    LPATH = ''

    if BUILD == 'debug':
        CFLAGS += ' -O0 -gdwarf-2'
        AFLAGS += ' -gdwarf-2'
    else:
        CFLAGS += ' -O2'

    CXXFLAGS = CFLAGS

POST_ACTION = OBJCPY + ' --prefix-symbols arm_ $TARGET && ' + OBJCPY + ' --redefine-syms=redef.arm $TARGET'
