cmake_minimum_required(VERSION 3.13...3.27)
include(pico_sdk_import.cmake)

# Needs mannually specified C/CXX compilers in the cmake command arguments
project(heart-pi)
pico_sdk_init()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(${PROJECT_NAME}.elf
    main.cpp
)
target_link_libraries(${PROJECT_NAME}.elf pico_stdlib)
pico_add_extra_outputs(${PROJECT_NAME}.elf)
