cmake_minimum_required(VERSION 3.13...3.27)
include(pico_sdk_import.cmake)

# Needs mannually specified C/CXX compilers in the cmake command arguments
project(heart-pi)
pico_sdk_init()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if (NOT TARGET tinyusb_device)
    message(FATAL_ERROR "TinyUSB submodule is not initialized in the SDK")
endif()

add_executable(${PROJECT_NAME}
    main.cpp)
target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_spi)
pico_add_extra_outputs(${PROJECT_NAME})

# enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
