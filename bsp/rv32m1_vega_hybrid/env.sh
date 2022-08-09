#!/bin/bash
export LOCAL=Juancho/UAM
export RTT_ROOT=/e/$LOCAL/Tesis/software/rtos/rt-thread-hybrid
export RTT_EXEC_PATH_RISCV=/e/$LOCAL/Tesis/software/toolchain/rv32m1_sdk/riscv32-unknown-elf-gcc/bin
export RTT_EXEC_PATH_ARM=/c/Xilinx/SDK/2019.1/gnu/aarch32/nt/gcc-arm-none-eabi/bin
export RTT_EXEC_PATH_HYBRID=/e/$LOCAL/Tesis/software/toolchain/hybrid/bin
export PATH=$PATH:$RTT_EXEC_PATH_HYBRID:$RTT_EXEC_PATH_RISCV:$RTT_EXEC_PATH_ARM
cd /e/$LOCAL/Tesis/software/rtos/rt-thread-hybrid/bsp/rv32m1_vega_hybrid
