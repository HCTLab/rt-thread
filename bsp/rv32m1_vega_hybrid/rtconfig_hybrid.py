import os

# Hybrid compilation options.
# Please refer to 'rtconfig_arm.py' or 'rtconfig_risc-v.py' for specific architecture compiling options
CROSS_TOOL  ='gcc'
ARCH        ='hybrid'

if os.getenv('RTT_CC'):
    CROSS_TOOL = os.getenv('RTT_CC')

# only support GNU Hybrid GCC compiler
if  CROSS_TOOL == 'gcc':
    PLATFORM    = 'gcc'
    EXEC_PATH   = r'/.../hybrid-none-eabi/bin'
else:
    print('Please make sure your toolchains is GNU GCC!')
    exit(0)

if os.getenv('RTT_EXEC_PATH_HYBRID'):
    EXEC_PATH = os.getenv('RTT_EXEC_PATH_HYBRID')

BUILD = 'debug'

if PLATFORM == 'gcc':
    # toolchains
    PREFIX = 'hybrid-none-'
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
    LINK_SCRIPT = 'link_hybrid.lds'
    TARGET_LIBS = 'rtthread.arm.lib rtthread.risc-v.lib'
    LFLAGS = ' --gc-sections -Map=map_hybrid.txt -cref -T %s startup_RV32M1_CM4.o startup_RV32M1_ri5cy.o --start-group %s --end-group ' % (LINK_SCRIPT, TARGET_LIBS)
    LIBS = ''

    CPATH = ''
    LPATH = ''

    if BUILD == 'debug':
        CFLAGS += ' -O0 -gdwarf-2'
        AFLAGS += ' -gdwarf-2'
    else:
        CFLAGS += ' -O2'

    CXXFLAGS = CFLAGS

POST_ACTION = OBJCPY + ' -O binary $TARGET hybrid.bin\n' + SIZE + ' $TARGET \n'
