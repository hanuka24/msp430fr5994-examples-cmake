# CMAKE toolchain for the MSP430FR microcontroller

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR msp430)

set(DEVICE "msp430fr5994" CACHE STRING "")
string(TOUPPER ${DEVICE} DEVICE_DEFINE)
set(DEVICE_DEFINE "__${DEVICE_DEFINE}__")

# MSP430 support files (installation dependent --> ADJUST ACCORDINGLY)
set(PATH_MSP430_SUPPORT "C:/ti/msp430-gcc/include" CACHE STRING "")
set(PATH_MSP430_LIB "C:/ti/msp430-gcc/msp430-elf" CACHE STRING "")
set(PATH_MSP430_BIN "C:/ti/msp430-gcc/bin" CACHE STRING "")
set(PATH_MSP430_INCLUDE "C:/ti/msp430-gcc/lib/gcc/msp430-elf/7.3.2/include" CACHE STRING "")


# MSP430 Debug path (installation dependent--> ADJUST ACCORDINGLY)
set(MSPDEBUG "C:/ti/mspdebug/mspdebug.exe" CACHE STRING "")

# Device specific driverlib
set(PATH_MSP430_DRIVERS "${PROJECT_SOURCE_DIR}/MSP430FR5xx_6xx" CACHE STRING "")

# default linkerscript
set(LINKER_SCRIPT
    "${PATH_MSP430_SUPPORT}/${DEVICE}.ld"
    CACHE
    FILEPATH "linkerscript"
    )

set(CMAKE_C_COMPILER    "${PATH_MSP430_BIN}/msp430-elf-gcc.exe")
set(CMAKE_CXX_COMPILER  "${PATH_MSP430_BIN}/msp430-elf-g++.exe")
set(CMAKE_AR            "${PATH_MSP430_BIN}/msp430-elf-ar.exe")
set(CMAKE_LINKER        "${PATH_MSP430_BIN}/msp430-elf-ld.exe")
set(CMAKE_NM            "${PATH_MSP430_BIN}/msp430-elf-nm.exe")
set(CMAKE_OBJDUMP       "${PATH_MSP430_BIN}/msp430-elf-objdump.exe")
set(CMAKE_STRIP         "${PATH_MSP430_BIN}/msp430-elf-strip.exe")
set(CMAKE_RANLIB        "${PATH_MSP430_BIN}/msp430-elf-ranlib.exe")
set(CMAKE_SIZE          "${PATH_MSP430_BIN}/msp430-elf-size.exe")

set(CMAKE_FIND_ROOT_PATH  "")

# Compiler flags
set(COMMON_FLAGS "-gdwarf-3 -gstrict-dwarf -I${PATH_MSP430_SUPPORT} -I${PATH_MSP430_LIB} -I${PATH_MSP430_DRIVERS} -I${PATH_MSP430_INCLUDE}" CACHE STRING "")

#set(MCU_SPECIFIC_CFLAGS "-mmcu=msp430fr5994 -mhwmult=f5series -mcode-region=none -mdata-region=none -mlarge" CACHE STRING "")
set(MCU_SPECIFIC_CFLAGS "-MD -mhwmult=f5series -mmcu=${DEVICE} -D${DEVICE_DEFINE} -mlarge -fno-builtin -lm" CACHE STRING "")

set(CMAKE_C_FLAGS "${MCU_SPECIFIC_CFLAGS} ${COMMON_FLAGS}" CACHE STRING "")

# Linker flags
set(MCU_SPECIFIC_LINKER_FLAGS "-L${PATH_MSP430_LIB}/lib/large" CACHE STRING "")
set(CMAKE_EXE_LINKER_FLAGS "${MCU_SPECIFIC_LINKER_FLAGS} -L${PATH_MSP430_SUPPORT} -T${LINKER_SCRIPT} -Wl,--gc-sections -Wl,-Map,\"${PROJECT_NAME}.map\" -Wl,-lgcc -Wl,-lc" CACHE STRING "")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

function(msp430_add_executable_upload ${EXECUTABLE})
	add_custom_target(upload_${EXECUTABLE} 
    	COMMAND ${MSPDEBUG} tilib "prog ../${EXECUTABLE}.elf"
		DEPENDS ${EXECUTABLE}.elf)
endfunction(msp430_add_executable_upload)