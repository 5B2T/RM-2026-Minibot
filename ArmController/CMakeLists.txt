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
set(CMAKE_PROJECT_NAME ArmController)

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
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
 Bsp/arm_board.c UserApp/arm_controller.c UserApp/arm_targets.c
 ../RemoteController/Bsp/radio/bsp_nrf24l01.c
 ../RemoteController/Bsp/time/bsp_time.c
 ../RemoteController/Modules/control/remote_control.c
 ../RemoteController/UserApp/components/radio/remote_radio.c
 ../Modules/radio/nrf24l01.c ../Modules/command/command_protocol.c
 ../Modules/command/telemetry_protocol.c
 Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c
 Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c
 Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_i2c.c
 Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
 Bsp UserApp ../RemoteController/Modules ../RemoteController/UserApp ../Bsp ../Modules)
target_link_libraries(${CMAKE_PROJECT_NAME} m)
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
 COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ArmController.hex
 COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ArmController.bin)
