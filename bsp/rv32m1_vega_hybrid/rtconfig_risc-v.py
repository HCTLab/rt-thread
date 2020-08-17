import os

# toolchains options
ARCH        ='risc-v'
CPU         ='rv32m1'
CROSS_TOOL  ='gcc'

if os.getenv('RTT_CC'):
    CROSS_TOOL = os.getenv('RTT_CC')

if  CROSS_TOOL == 'gcc':
    PLATFORM    = 'gcc'
    EXEC_PATH   = r'/e/Juancho/UAM/Tesis/software/toolchain/riscv/bin'
else:
    print('Please make sure your toolchains is GNU GCC!')
    exit(0)

if os.getenv('RTT_EXEC_PATH'):
    EXEC_PATH = os.getenv('RTT_EXEC_PATH')

BUILD = 'debug'

if PLATFORM == 'gcc':
    # toolchains
    PREFIX  = 'riscv32-unknown-elf-'
    CC      = PREFIX + 'gcc'
    CXX     = PREFIX + 'g++'
    AS      = PREFIX + 'gcc'
    AR      = PREFIX + 'ar'
    #LINK    = PREFIX + 'gcc'
    LINK    = PREFIX + 'ar rcs'
    TARGET_EXT = 'elf'
    SIZE    = PREFIX + 'size'
    OBJDUMP = PREFIX + 'objdump'
    OBJCPY  = PREFIX + 'objcopy'

    #DEVICE  = ' -march=rv32imc -mabi=ilp32'
    DEVICE  = ''
    CFLAGS  = DEVICE + ' -fno-builtin -fno-exceptions -ffunction-sections -I$BSP_ROOT -I$ARCH'
    AFLAGS  = ' -c' + DEVICE + ' -x assembler-with-cpp -I$BSP_ROOT -I$ARCH'
    LINK_SCRIPT = 'hybrid.lds'
    LFLAGS  = ''
    LIBS = ''

    CPATH   = ''
    LPATH   = ''

    if BUILD == 'debug':
        CFLAGS += ' -O0 -gdwarf-2'
        AFLAGS += ' -gdwarf-2'
    else:
        CFLAGS += ' -O2 -Os'

    CXXFLAGS = CFLAGS

POST_ACTION = OBJCPY + ' --prefix-symbols riscv_ $TARGET --rename-section .bss=.riscv_bss --rename-section .sbss=.riscv_sbss --rename-section COMMON=riscv_COMMON --rename-section .data=.riscv_data --rename-section .sdata=.riscv_sdata && ' + \
              OBJCPY + ' --redefine-syms=redef.riscv $TARGET && ' + \
              AR + ' x $TARGET startup_RV32M1_ri5cy.o'
