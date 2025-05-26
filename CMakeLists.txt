# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
set(FREERTOS_KERNEL_PATH "/home/carlos2/FreeRTOS-Kernel")
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(ControleDeAcessos C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(ControleDeAcessos ControleDeAcessos.c lib/ssd1306.c lib/numeros.c)

pico_set_program_name(ControleDeAcessos "ControleDeAcessos")
pico_set_program_version(ControleDeAcessos "0.1")

# Generate PIO header
pico_generate_pio_header(ControleDeAcessos ${CMAKE_CURRENT_LIST_DIR}/blink.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(ControleDeAcessos 1)
pico_enable_stdio_usb(ControleDeAcessos 1)

# Add the standard library to the build
target_link_libraries(ControleDeAcessos
        pico_stdlib)

include_directories(${CMAKE_SOURCE_DIR}/lib)

# Add the standard include files to the build
target_include_directories(ControleDeAcessos PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(ControleDeAcessos 
        hardware_i2c
        hardware_pio
        hardware_pwm
        FreeRTOS-Kernel 
        FreeRTOS-Kernel-Heap4
        )

pico_add_extra_outputs(ControleDeAcessos)

