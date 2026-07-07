cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Default to STM32 ARM GCC toolchain when no toolchain is provided.
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake" CACHE FILEPATH "Toolchain file")
endif()


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME RC_485)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Unitree/gom_protocol.c
    Unitree/crc_ccitt.c
    Unitree/control.c
    My_code/motion.c
    My_code/my_system.c
    My_code/pid.c
    My_code/CRSF.c
    My_code/com_uart.c
    Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_cos_f32.c
    Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_sin_f32.c
    Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c
    DM/bsp_fdcan.c
    DM/dm_motor_ctrl.c
    DM/dm_motor_drv.c
    DM/dm_control.c
    DM/delay.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Unitree
    My_code
    Drivers/CMSIS/DSP/Include
    DM
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
# Ensure printf floating-point support is linked (newlib-nano)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE "-Wl,-u,_printf_float")
