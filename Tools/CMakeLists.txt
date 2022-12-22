
add_library(
	Serial STATIC
	Serial/serial.c
    ${PROJECT_SOURCE_DIR}/MSP430FR5xx_6xx/eusci_a_uart.c
	${PROJECT_SOURCE_DIR}/MSP430FR5xx_6xx/gpio.c
)

target_include_directories(Serial PUBLIC "Serial" "${PROJECT_SOURCE_DIR}/MSP430FR5xx_6xx")

add_library(
	Si7021 STATIC
	Si7021/si7021.c
	Si7021/i2c_driver.c
	Serial/serial.c
    ${PROJECT_SOURCE_DIR}/MSP430FR5xx_6xx/eusci_b_i2c.c
    ${PROJECT_SOURCE_DIR}/MSP430FR5xx_6xx/gpio.c
)

target_include_directories(Si7021 PUBLIC "Si7021" "${PROJECT_SOURCE_DIR}/MSP430FR5xx_6xx" "Serial")