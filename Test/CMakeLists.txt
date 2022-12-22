set(EXECUTABLE Test)

set(ELF_FILE ${EXECUTABLE}.elf)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

include_directories(${PROJECT_SOURCE_DIR}/Tools/Serial ${PROJECT_SOURCE_DIR}/Tools/Si7021)

add_executable(${ELF_FILE}
    "${PROJECT_SOURCE_DIR}/Test/main.c"
    "${PROJECT_SOURCE_DIR}/MSP430FR5xx_6xx/gpio.c"
    "${PROJECT_SOURCE_DIR}/MSP430FR5xx_6xx/cs.c"
    "${PROJECT_SOURCE_DIR}/MSP430FR5xx_6xx/wdt_a.c"
    "${PROJECT_SOURCE_DIR}/MSP430FR5xx_6xx/pmm.c"
    "${PROJECT_SOURCE_DIR}/MSP430FR5xx_6xx/timer_b.c"
    "${SOURCE_FILES}"
    )

target_include_directories(${ELF_FILE} PUBLIC "${PROJECT_SOURCE_DIR}/Tools")
target_link_libraries(${ELF_FILE} Serial Si7021)

msp430_add_executable_upload(${EXECUTABLE})