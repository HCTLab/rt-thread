import os

# toolchains options
CROSS_TOOL  ='gcc'
ARCH        ='risc-v'
CPU         ='rv32m1'

if os.getenv('RTT_CC'):
    CROSS_TOOL = os.getenv('RTT_CC')

if  CROSS_TOOL == 'gcc':
    PLATFORM    = 'gcc'
    EXEC_PATH   = r'/.../riscv32-unknown-elf-gcc/bin'
else:
    print('Please make sure your toolchains is GNU GCC!')
    exit(0)

if os.getenv('RTT_EXEC_PATH_RISCV'):
    EXEC_PATH = os.getenv('RTT_EXEC_PATH_RISCV')

STD_LIB_PATH = EXEC_PATH + '/../riscv32-unknown-elf/lib'
GCC_LIB_PATH = EXEC_PATH + '/../lib/gcc/riscv32-unknown-elf/7.1.1/rv32ic/ilp32/mreg16'
#GCC_LIB_PATH = EXEC_PATH + '/../lib/gcc/riscv32-unknown-elf/x.x.x'

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
    CFLAGS  = DEVICE + ' -fno-builtin -fno-exceptions -ffunction-sections -I$BSP_ROOT -I$ARCH -DCPU_RISCV'
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

#POST_ACTION = OBJCPY + ' --prefix-symbols riscv_ $TARGET --rename-section .bss=.riscv_bss --rename-section .sbss=.riscv_sbss --rename-section .data=.riscv_data --rename-section .sdata=.riscv_sdata && ' + \
#              OBJCPY + ' --redefine-syms=redef.riscv $TARGET && ' + \
#              AR + ' x $TARGET startup_RV32M1_ri5cy.o'

POST_ACTION = 'cd build/' + ARCH + ' && ' + AR + ' x ' + GCC_LIB_PATH + '/libgcc.a && ' + \
                                            AR + ' x ' + STD_LIB_PATH + '/libc.a && ' + \
              'ls *.o > list.txt && xargs ' + AR + ' -rcs ../../$TARGET < list.txt && ' + \
              'cd ../..  && ' + OBJCPY + ' --prefix-symbols riscv_ $TARGET && ' + \
                                OBJCPY + ' --redefine-syms=redef.riscv $TARGET && ' + \
                                AR + ' x $TARGET startup_RV32M1_ri5cy.o'
