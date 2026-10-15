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
set(CMAKE_PROJECT_NAME RM-2026-Minibot)

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

# Application layers; hardware-generated files remain owned by CubeMX.
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Bsp/motor/bsp_motor.c
    Bsp/encoder/bsp_encoder.c
    Bsp/radio/bsp_nrf24l01.c
    Bsp/time/bsp_time.c
    Bsp/i2c/bsp_i2c.c
    Bsp/input/bsp_buttons.c
    Bsp/arm/bsp_arm.c
    Bsp/uart/bsp_debug_uart.c
    Modules/actuator/servo.c
    Modules/display/oled.c
    Modules/power/ina219.c
    Modules/power/battery_soc.c
    Modules/algorithm/mecanum.c
    Modules/algorithm/pi_controller.c
    Modules/command/command_protocol.c
    Modules/command/arm_debug_protocol.c
    Modules/radio/nrf24l01.c
    Modules/wheel/speed.c
    Modules/wheel/motor_control.c
    UserApp/components/chassis/chassis.c
    UserApp/components/chassis/chassis_test.c
    UserApp/components/command/remote_control.c
    UserApp/components/display/debug_display.c
    UserApp/components/power/power_monitor.c
    UserApp/components/arm/arm.c
    UserApp/components/host/arm_debug.c
    UserApp/robot/robot.c
    UserApp/os_task.c
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE Bsp Modules UserApp)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
