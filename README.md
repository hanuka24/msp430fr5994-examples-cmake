# MSP430FR5994

Examples for MSP430FR5994 LaunchPad based on CMake.

# Build process

- Install cmake (https://cmake.org/) 
- Download msp430-gcc (https://www.ti.com/tool/MSP430-GCC-OPENSOURCE) and extract in any folder
- Adjust path to msp430-gcc in msp430-toolchain.cmake accordingly (i.e., replace path of MSP430 support file variables PATH_MSP430_SUPPORT, PATH_MSP430_LIB, PATH_MSP430_BIN, PATH_MSP430_INCLUDE)
- Create folder *build*
- Move into folder (`cd build`) and run `cmake -DCMAKE_TOOLCHAIN_FILE="..\msp430-toolchain.cmake" ..` (Linux) or `cmake -DCMAKE_TOOLCHAIN_FILE="..\msp430-toolchain.cmake" -G 'MSYS Makefiles ..` (Windows) to create build environment
- To build binaries, run `make PROJECT.elf` (to build specific project) or run `make` (to build all projects)
- The binary files are located directly in the build directory (e.g.*build/PROJECT.elf*) and can be flashed to the MSP430 LaunchPad using Uniflash (https://www.ti.com/tool/UNIFLASH?keyMatch=UNIFLASH%20DOWNLOAD) or mspdebug (see below)

# Projects

1) *Blinky*: Simple Blinky program
2) *LPM35*: Set device into LPM3.5, wake up using external interrupt
3) *Timer*: Toggle LED periodically using internal timer, test UART
4) *Test*: Test application to investigate power consumption and lifetime using a supercapacitor (UART Baudrate: 9600)
5) *Test_LPM35*: Test application to investigate power consumption and lifetime using a supercapacitor (UART Baudrate: 9600)
6) *Test_Si7021*: Periodically reads and prints temperature/humidity readings from Si7021 sensor
7) *RFM95*: Periodically transmit LoRa packets using RFM95 transceiver

- *Tools*: includes devices drivers, e.g., UART, temperature sensor
- *MSP430FR5xx_6xx*: device drivers from TI to access board's peripherals (e.g., including driverlib) 

# MSPDebug
MSPDebug allows to flash and debug boards using command line and gdb.
- To install mspdebug, follow instructions on: https://dlbeer.co.nz/mspdebug (for Windows, checkout: https://dlbeer.co.nz/mspdebug/faq.html#compile_windows and build with `make gcc=CC`)
- Flash program using `mspdebug tilib "prog xxxx.elf"`
- Debug program using GDB with two options:
    1) *Command line:* Start debugging server using  `mspdebug tilib "gdb"`, connect to debug server using `msp430-elf-gdb target remote :2000`, debug using GDB
    2) Start debugging session in VSCode (adjust path of msp430-gdb in *.vscode/launch.json* accordingly)
- Before debugging, flash board with newest firmware!

# License
This project is licensed under the terms of the MIT license.