import os

# toolchains options
CROSS_TOOL  ='gcc'
ARCH        ='arm'
CPU         ='cortex-m0plus'
#CPU         ='cortex-m4'

# core to be use
USE_CORE    ='CORE_M0'
#USE_CORE    ='CORE_M4'

if os.getenv('RTT_CC'):
    CROSS_TOOL = os.getenv('RTT_CC')

if  CROSS_TOOL == 'gcc':
    PLATFORM    = 'gcc'
    EXEC_PATH   = r'/.../gcc-arm-none-eabi/bin'
else:
    print('Please make sure your toolchains is GNU GCC!')
    exit(0)

if os.getenv('RTT_EXEC_PATH_ARM'):
    EXEC_PATH = os.getenv('RTT_EXEC_PATH_ARM')

STD_LIB_PATH = EXEC_PATH + '/../arm-none-eabi/lib/thumb/v6-m/nofp'
GCC_LIB_PATH = EXEC_PATH + '/../lib/gcc/arm-none-eabi/9.3.1/thumb/v6-m/nofp'
#GCC_LIB_PATH = EXEC_PATH + '/../lib/gcc/arm-none-eabi/x.x.x'

OBJ_PATH = 'build/' + ARCH

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
    DEVICE = ' -mcpu=' + CPU + ' -mthumb -ffunction-sections -Wall'
    if USE_CORE == 'CORE_M4':
        DEVICE += ' -mfpu=fpv4-sp-d16 -mfloat-abi=softfp'
    CFLAGS = DEVICE + ' -I$BSP_ROOT -I$ARCH -DCPU_ARM'
    AFLAGS = ' -c' + DEVICE + ' -x assembler-with-cpp -Wa,-mimplicit-it=thumb -I$BSP_ROOT -I$ARCH'
    #AFLAGS = ' -c' + DEVICE + ' -x assembler-with-cpp -I$BSP_ROOT -I$ARCH'
    LFLAGS = ''
    LIBS = ''

    CPATH = ''
    LPATH = ''

    if BUILD == 'debug':
        CFLAGS += ' -O0 -gdwarf-2 -DNDEBUG'
        AFLAGS += ' -gdwarf-2'
    else:
        CFLAGS += ' -O2'

    CXXFLAGS = CFLAGS

#POST_ACTION = 'cd build/' + ARCH + ' && ' + AR + ' x ' + GCC_LIB_PATH + '/libgcc.a && ' + \
#                                            AR + ' x ' + STD_LIB_PATH + '/libc.a && ' + \
#              'ls *.o > list.txt && xargs ' + AR + ' -rcs ../../$TARGET < list.txt && ' + \
#              'cd ../..  && ' + OBJCPY + ' --prefix-symbols arm_ $TARGET && ' + \
#                                OBJCPY + ' --redefine-syms=redef.arm $TARGET && ' + \
#                                AR + ' x $TARGET startup_RV32M1_cm0.o'

POST_ACTION = 'cd ' + OBJ_PATH + ' && ' + \
              'cp ' + GCC_LIB_PATH + '/libgcc.a . && ' + \
              'cp ' + STD_LIB_PATH + '/libc.a . && ' + \
              'cd ../..  && ' + \
              OBJCPY + ' --prefix-symbols arm_ $TARGET && ' + \
              OBJCPY + ' --redefine-syms=redef.arm $TARGET && ' + \
              OBJCPY + ' --prefix-symbols arm_ ' + OBJ_PATH + '/libgcc.a && ' + \
              OBJCPY + ' --prefix-symbols arm_ ' + OBJ_PATH + '/libc.a && ' + \
              AR + ' x $TARGET startup_RV32M1_cm0.o'
