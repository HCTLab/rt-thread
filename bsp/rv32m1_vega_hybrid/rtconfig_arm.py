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

#GCC_LIB_PATH = EXEC_PATH + '/../lib/gcc/arm-none-eabi/x.x.0'
GCC_LIB_PATH = EXEC_PATH + '/../lib/gcc/arm-none-eabi/9.3.1/thumb/v6-m/nofp'

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
    #DEVICE = ' -mcpu=' + CPU + ' -ffunction-sections -Wall'
    if USE_CORE == 'CORE_M4':
        DEVICE += ' -mfpu=fpv4-sp-d16 -mfloat-abi=softfp'
    CFLAGS = DEVICE + ' -I$BSP_ROOT -I$ARCH'
    AFLAGS = ' -c' + DEVICE + ' -x assembler-with-cpp -Wa,-mimplicit-it=thumb -I $BSP_ROOT -I $ARCH'
    #AFLAGS = ' -c' + DEVICE + ' -x assembler-with-cpp -I $BSP_ROOT -I $ARCH'
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
#                                            AR + ' rcs ../../$TARGET *.o && ' + \
#              'cd ../..  && ' + OBJCPY + ' --prefix-symbols arm_ $TARGET --rename-section .bss=.arm_bss --rename-section .sbss=.arm_sbss --rename-section .data=.arm_data --rename-section .sdata=.arm_sdata && ' + \
#                                OBJCPY + ' --redefine-syms=redef.arm $TARGET && ' + \
#                                AR + ' x $TARGET startup_RV32M1_cm0.o'

POST_ACTION = 'cd build/' + ARCH + ' && ' + AR + ' x ' + GCC_LIB_PATH + '/libgcc.a && ' + \
                                            AR + ' rcs ../../$TARGET *.o && ' + \
              'cd ../..  && ' + OBJCPY + ' --prefix-symbols arm_ $TARGET && ' + \
                                OBJCPY + ' --redefine-syms=redef.arm $TARGET && ' + \
                                AR + ' x $TARGET startup_RV32M1_cm0.o'
