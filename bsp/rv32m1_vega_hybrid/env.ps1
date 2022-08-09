e:
cd /Juancho/UAM/Tesis/software/rtos/rt-thread-hybrid/bsp/rv32m1_vega_hybrid

$env:RTT_ROOT="e:/Juancho/UAM/Tesis/software/rtos/rt-thread-hybrid"
$env:RTT_EXEC_PATH_RISCV="e:/Juancho/UAM/Tesis/software/toolchain/rv32m1_sdk/riscv32-unknown-elf-gcc/bin"
$env:RTT_EXEC_PATH_ARM="c:/Xilinx/SDK/2019.1/gnu/aarch32/nt/gcc-arm-none-eabi/bin"
$env:RTT_EXEC_PATH_HYBRID="e:/Juancho/UAM/Tesis/software/toolchain/hybrid/bin"

$env:PATH+=$env:RTT_EXEC_PATH_HYBRID+";"
$env:PATH+=$env:RTT_EXEC_PATH_RISCV+";"
$env:PATH+=$env:RTT_EXEC_PATH_ARM+";"

$env:APP="cipherer"
