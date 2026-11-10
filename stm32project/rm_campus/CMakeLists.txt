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


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME rm_campus)

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
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
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

# CAMPUS_DRIVER_INTEGRATION: 自定义源码与生成文件分开维护。
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Bsp/Src/can.c
    Bsp/Src/motor_3508.c
    Bsp/Src/fxi6x.c
    Bsp/Src/led.c
    Bsp/Src/usb_serial.c
    Bsp/Src/bmi088.c
    Bsp/Src/imu_clock.c
    App/Src/campus_app.c
    App/Src/usb_app.c
    App/Src/imu_app.c
    Algorithm/Src/imu_filter.c
    Algorithm/Src/QuaternionEKFc.c
    Algorithm/Src/kalman_filter.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f32.c
    Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.c
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Bsp/Inc App/Inc Algorithm/Inc Drivers/CMSIS/DSP/Include
)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE ARM_MATH_MATRIX_CHECK)
