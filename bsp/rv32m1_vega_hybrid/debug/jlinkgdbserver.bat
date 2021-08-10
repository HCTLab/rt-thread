@ECHO OFF
rem "C:\Program Files\SEGGER\JLink\JLinkGDBServerCL.exe" -select USB -device Cortex-M0+ -endian little -if SWD -speed 4000 -noir -LocalhostOnly -settingsfile arm.jlink
"C:\Program Files\SEGGER\JLink\JLinkGDBServerCL.exe" -select USB -device Cortex-M0+ -endian little -if SWD -speed 4000 -noir -LocalhostOnly -nohalt -jlinkscriptfile RV32M1_M0P.JlinkScript
