cmake_minimum_required(VERSION 3.12)

add_library(ina219
        INA219.cpp
        )

target_include_directories(ina219
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        )

target_link_libraries(ina219
        pico_stdlib
        hardware_i2c
        )
