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
    Bsp/Src/servo_uart.c
    Bsp/Src/zp_servo.c
    Bsp/Src/fs_servo.c
    App/Src/servo_app.c
    App/Src/robot_control.c
    App/Src/robot_servo.c
    App/Src/robot_servo_usb.c
    Bsp/Src/buzzer.c
    Algorithm/Src/arm_kinematics.c
    App/Src/arm_velocity.c
    App/Src/arm_usb.c
    App/Src/servo_rc.c
    App/Src/servo_rc_usb.c
    App/Src/servo_usb.c
    Bsp/Src/led.c
    Bsp/Src/usb_serial.c
    Bsp/Src/bmi088.c
    Bsp/Src/imu_clock.c
    Bsp/Src/w25q64.c
    App/Src/campus_app.c
    App/Src/motor_debug.c
    App/Src/mecanum.c
    App/Src/motor_debug_usb.c
    App/Src/motor_status_usb.c
    App/Src/usb_app.c
    App/Src/imu_app.c
    App/Src/chassis_targets.c
    App/Src/flash_service.c
    App/Src/flash_tasks.c
    App/Src/flash_test.c
    App/Src/motor_speed_log.c
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

option(SERVO_ID_SETUP "Build the standalone UART7 servo ID setup firmware" OFF)
option(SERVO_MIXED_PROBE "Build a read-only ZP/Fashion Star UART7 probe on the standalone servo task" OFF)
if(SERVO_MIXED_PROBE AND NOT SERVO_ID_SETUP)
    message(FATAL_ERROR "Mixed servo probe requires the standalone servo task")
endif()
option(ROBOT_CONTROL_PROFILE "Build the dedicated CH9 whole robot firmware" OFF)
option(ROBOT_SERVO_TELEMETRY "Add read-only USB servo waveforms to robot control" OFF)
option(ROBOT_FASHIONSTAR_GRIPPER "Use a Fashion Star U25-M for logical gripper 005" OFF)
if(ROBOT_FASHIONSTAR_GRIPPER AND NOT ROBOT_CONTROL_PROFILE)
    message(FATAL_ERROR "Fashion Star gripper requires robot control")
endif()
if(ROBOT_SERVO_TELEMETRY AND NOT ROBOT_CONTROL_PROFILE)
    message(FATAL_ERROR "Servo telemetry requires the robot control profile")
endif()
if(ROBOT_CONTROL_PROFILE OR SERVO_ID_SETUP)
    set(status_only_default OFF)
else()
    set(status_only_default ON)
endif()
option(SERVO_STATUS_ONLY "USB five-servo monitor with optional fixed-step jog test" ${status_only_default})
if(SERVO_STATUS_ONLY)
    if(ROBOT_CONTROL_PROFILE OR SERVO_ID_SETUP)
        message(FATAL_ERROR "Servo status monitor requires exclusive use of UART7 and USB")
    endif()
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE SERVO_STATUS_ONLY=1)
endif()
if(ROBOT_CONTROL_PROFILE)
    if(SERVO_ID_SETUP)
        message(FATAL_ERROR "Robot control and servo ID setup are exclusive")
    endif()
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE ROBOT_CONTROL_ENABLE=1)
    set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES OUTPUT_NAME "rm_campus_robot")
    if(ROBOT_SERVO_TELEMETRY)
        target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE ROBOT_SERVO_TELEMETRY_ENABLE=1)
        set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES OUTPUT_NAME "rm_campus_robot_telemetry")
    endif()
    if(ROBOT_FASHIONSTAR_GRIPPER)
        target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE SERVO_FASHIONSTAR_GRIPPER=1)
        set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES OUTPUT_NAME "rm_campus_robot_fs_gripper")
    endif()
endif()
if(SERVO_ID_SETUP)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE App/Src/servo_id_setup.c)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE SERVO_ID_SETUP)
    set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES OUTPUT_NAME "rm_campus_servo_id")
    if(SERVO_MIXED_PROBE)
        target_sources(${CMAKE_PROJECT_NAME} PRIVATE App/Src/servo_mixed_probe.c)
        target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE SERVO_MIXED_PROBE)
        set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES OUTPUT_NAME "rm_campus_servo_mixed_probe")
    endif()
endif()
