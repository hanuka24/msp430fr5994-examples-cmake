set(EXECUTABLE Blinky)

set(ELF_FILE ${EXECUTABLE}.elf)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

add_executable(${ELF_FILE}
    "${PROJECT_SOURCE_DIR}/Blinky/main.c"
    "${PROJECT_SOURCE_DIR}/MSP430FR5xx_6xx/gpio.c"
    "${PROJECT_SOURCE_DIR}/MSP430FR5xx_6xx/wdt_a.c"
    "${PROJECT_SOURCE_DIR}/MSP430FR5xx_6xx/pmm.c"
    "${SOURCE_FILES}"
    )

msp430_add_executable_upload(EXECUTABLE)