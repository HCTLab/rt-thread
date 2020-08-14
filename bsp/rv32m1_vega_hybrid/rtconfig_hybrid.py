import os

# Hybrid compilation options. 
# Please refer to 'rtconfig_arm.py' or 'rtconfig_risc-v.py' for specific architecture compiling options

ARCH='hybrid'
CROSS_TOOL='gcc'

if os.getenv('RTT_CC'):
    CROSS_TOOL = os.getenv('RTT_CC')

# only support GNU Hybrid GCC compiler
PLATFORM 	 = 'gcc'
EXEC_PATH 	 = '/e/Juancho/UAM/Tesis/software/toolchain/hybrid/bin/'

if os.getenv('RTT_EXEC_PATH'):
    EXEC_PATH = os.getenv('RTT_EXEC_PATH')

BUILD = 'debug'

if PLATFORM == 'gcc':
    # toolchains
    PREFIX = 'hybrid-none-gnueabihf-'
    CC = PREFIX + 'gcc'
    CXX = PREFIX + 'g++'
    AS = PREFIX + 'gcc'
    AR = PREFIX + 'ar'
    LINK = PREFIX + 'ld'
    TARGET_EXT = 'elf'
    SIZE = PREFIX + 'size'
    OBJDUMP = PREFIX + 'objdump'
    OBJCPY = PREFIX + 'objcopy'
    CFLAGS = ''
    AFLAGS = ''
    TARGET_LIBS = 'rtthread.arm.lib rtthread.risc-v.lib'
    LFLAGS = ' --gc-sections -Map=map_hybrid.txt -cref -T link_hybrid.lds startup.o --start-group %s --end-group ' % TARGET_LIBS
    LIBS = ''

    CPATH = ''
    LPATH = ''

    if BUILD == 'debug':
        CFLAGS += ' -O0 -gdwarf-2'
        AFLAGS += ' -gdwarf-2'
    else:
        CFLAGS += ' -O2'

    CXXFLAGS = CFLAGS

# DUMP_ACTION = OBJDUMP + ' -D -S $TARGET > rtt.asm\n'
# POST_ACTION = SIZE + ' $TARGET \n'
# POST_ACTION = OBJCPY + ' -O binary $TARGET rtthread.bin\n' + SIZE + ' $TARGET \n'

POST_ACTION = ''
