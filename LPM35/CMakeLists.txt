set(EXECUTABLE LPM35)

set(ELF_FILE ${EXECUTABLE}.elf)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

add_executable(${ELF_FILE}
    "${PROJECT_SOURCE_DIR}/LPM35/main.c"
    "${SOURCE_FILES}"
    )

msp430_add_executable_upload(${EXECUTABLE})