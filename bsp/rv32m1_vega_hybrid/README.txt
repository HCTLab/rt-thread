Introduction
------------

On this BSP you can find several RT-Thread applications that will run in a VEGAboard (https://open-isa.org)

- cipherer:       Hybrid application that encrypts and decrypts a file stored in the SD card, which run on top of an hybrid RT-Thread implementation.
- cipherer_rpmsg: Equivalent application to 'cipherer', but based on the RPMsg framework (for asymmetric multiprocessing or AMP)
- mutex:          Hybrid example applicaction which shows how to use common IPC resources in an hybrid environment.


Compilation
-----------

In order to compile any application on this BSP you will need to follow these steps:

1) Install common GCC toolchains for ARM y RiscV cores provided with VEGAboard.

   Recommended toolchains can be found at:
   
   URL:        https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads
   Package:    GNU Arm Embedded Toolchain: 9-2020-q2-update June 30, 2020
   File:       gcc-arm-none-eabi-9-2020-q2-update-win32.zip
   
   URL:        https://github.com/open-isa-org/open-isa.org/releases/download/1.0.0/Toolchain_Windows.zip
   Package:    Prebuilt GCC and OpenOCD for Windows.
   File:       Toolchain_Windows.zip (inside you can find 'riscv32-unknown-elf-gcc.zip' toolchain)

2) Compile and install modified GCC toolchain v9.1.0, which allows linking hybrid executables.

   Please refer to 'https://github.com/HCTLab/hybrid-gcc' repository.

3) Install a building platform (for Windows) to compile hybrid applications:

   Please refer to MSYS2 tools at https://www.msys2.org

4) Open a MSYS2 shell and create required compilation variables:

   $ export RTT_ROOT=/..../rt-thread-hybrid
   $ export RTT_EXEC_PATH_RISCV=/..../rv32m1_sdk/riscv32-unknown-elf-gcc/bin
   $ export RTT_EXEC_PATH_ARM=/..../gnu/aarch32/nt/gcc-arm-none-eabi/bin
   $ export RTT_EXEC_PATH_HYBRID=/..../toolchain/hybrid/bin

   $ export PATH=$PATH:$RTT_EXEC_PATH_HYBRID:$RTT_EXEC_PATH_RISCV:$RTT_EXEC_PATH_ARM

   $ export APP=cipherer   # Folder of the application to be compiled
   $ export REDEF='yes'    # Set REDEF to 'yes' for hybrid apps, 'no' for common apps.

5) Move to this BSP directory and compile:

   $ cd /..../rt-thread-hybrid/bsp/rv32m1_vega_hybrid

   $ scons -c
   $ scons --verbose


Debugging/loading hybrid application
------------------------------------

In order to load, run and debug an hybrid application in the VEGAboard, you will need:

- Two debug probes: JLink debug are recommended (https://www.segger.com/products/debug-probes/j-link)
- Two different terminals/consoles in order to debug different cores.

   1) Terminal 1: A common Windows shell console (not powershell) to debug ARM cores. This BSP includes a 'debug' folder with scripts to support JLink probes.
   2) Terminal 2: A MSYS2 console used to debug RiscV cores.

In order to debug the hybrid application (ARM threads):

   # Terminal 1 (J-Link adapter #1 must be connected to SWD port) -----------------------------------------------------

   # Important: JLINKGDBSERVER must be called after terminal 2 at RISCV would have reset the board (monitor reset halt)
   #            Otherwise JLINKGDBSERVER will not connect
   #            Try to launch JLINKGDBSERVER when led is turned off (ARM turns off the led when it boots)

   cd X:\....\rt-thread-hybrid\bsp\rv32m1_vega_hybrid\debug
   E:
   .\jlinkgdbserver.bat

   # Terminal 2
   arm-none-eabi-gdb.exe rtthread_hybrid.elf -ex 'target remote localhost:2331'

In order to load/debug the hybrid application (RiscV threads):

   # Terminal 1 (J-Link adapter #2 must be connected to JTAG port)
   /..../rv32m1_sdk/openocd/bin/openocd -f /..../rv32m1_sdk/rv32m1_sdk_riscv/boards/rv32m1_vega/rv32m1_ri5cy.cfg

   # Terminal 2
   riscv32-unknown-elf-gdb rtthread_hybrid.elf -ex 'target remote localhost:3333'

   set mem inaccessible-by-default off
   set remote hardware-breakpoint-limit 4
   set remote hardware-watchpoint-limit 2
   set architecture riscv
   set processor riscv:rv32

   monitor reset halt

   # This function erases flash and prepare core booting options (see 'rv32m1_ri5cy.cfg' procs)
   # This line executes 'hybrid_boot()' function at monitor (openocd)
   monitor hybrid_boot

   # Write ELF into flash
   load

   br MU_BootCoreB
   x/1xw 0x40023010     - FOPT - read-only (see OPENOCD 'rv32m1_ri5cy.cfg' funcs to change it)
   x/1xw 0x40025060     - MUA(SR)
   x/1xw 0x40025068     - MUA(CCR)
   x/1xw 0x41024060     - MUB(SR)
   x/1xw 0x41024068     - MUB(CCR)
   x/4xb 0x20000000

   # Cores(n): CPU0(n=0,CM4F)  CPU1(n=1,CM0+)  CPU2(n=2,RI5CY)  CPU3(n=3,ZERO-RISCY)
