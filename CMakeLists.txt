# Minimum CMake version required
cmake_minimum_required(VERSION 3.12)

# Constant EXAMPLE_BUILD for building examples
OPTION(EXAMPLE_BUILD "Build all examples" ON)

# Check if the environment variable 'PICO_SDK_PATH' exists
if(NOT EXISTS $ENV{PICO_SDK_PATH})
message(FATAL_ERROR "The Pico SDK environment path 'PICO_SDK_PATH' was not found. Please set it accordingly, so it points to the 'pico-sdk' folder and/or restart the CLI.")
endif()

# Includes the Pico CMake file
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Initializes the Pico SDK
pico_sdk_init()

# Project name with version
project(haw_mpu6050 
    VERSION 1.0 
    DESCRIPTION "MPU6050 stand-alone library for RPi Pico"
    LANGUAGES C
)

# Versions of the used languages 
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
enable_language(C CXX)

# Adds the CMakeLists.txt file from the ./src directory
add_subdirectory(./src)

if(EXAMPLE_BUILD)
    # Adds the CMakeLists.txt file from the ./example directory
    add_subdirectory(./example)
endif()
