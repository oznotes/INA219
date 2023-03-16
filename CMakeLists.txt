cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

# Set name of project (as PROJECT_NAME) and C/C++ standards
project(INA219-RP2040 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

# Initialize the SDK
pico_sdk_init()


project(INA219-RP2040)

# Tell CMake where to find the executable source file
add_executable(INA219-RP2040
        main.cpp)

add_subdirectory(INA219_driver)

target_link_libraries(INA219-RP2040 pico_stdlib hardware_i2c pico_multicore ina219)

pico_add_extra_outputs(INA219-RP2040)

# Enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)