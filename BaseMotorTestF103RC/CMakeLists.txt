cmake_minimum_required(VERSION 3.22)
project(BaseMotorTestF103RC LANGUAGES C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Reuse the existing official ST F1 SDK; do not duplicate another driver tree.
set(SDK "${CMAKE_CURRENT_SOURCE_DIR}/../ArmController/Drivers")
set(HAL "${SDK}/STM32F1xx_HAL_Driver")
set(CMSIS "${SDK}/CMSIS")
add_executable(${PROJECT_NAME}
    main.c motor_test.c
    "${CMAKE_CURRENT_SOURCE_DIR}/../ArmController/Core/Src/syscalls.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../ArmController/Core/Src/sysmem.c"
    "${CMSIS}/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xe.s"
    "${CMSIS}/Device/ST/STM32F1xx/Source/Templates/system_stm32f1xx.c"
    "${HAL}/Src/stm32f1xx_hal.c"
    "${HAL}/Src/stm32f1xx_hal_cortex.c"
    "${HAL}/Src/stm32f1xx_hal_rcc.c"
    "${HAL}/Src/stm32f1xx_hal_rcc_ex.c"
    "${HAL}/Src/stm32f1xx_hal_flash.c"
    "${HAL}/Src/stm32f1xx_hal_flash_ex.c"
    "${HAL}/Src/stm32f1xx_hal_gpio.c"
    "${HAL}/Src/stm32f1xx_hal_gpio_ex.c"
    "${HAL}/Src/stm32f1xx_hal_dma.c"
    "${HAL}/Src/stm32f1xx_hal_pwr.c"
    "${HAL}/Src/stm32f1xx_hal_tim.c"
    "${HAL}/Src/stm32f1xx_hal_tim_ex.c"
    "${HAL}/Src/stm32f1xx_hal_uart.c")
target_include_directories(${PROJECT_NAME} PRIVATE . "${HAL}/Inc" "${HAL}/Inc/Legacy"
    "${CMSIS}/Include" "${CMSIS}/Device/ST/STM32F1xx/Include")
target_compile_definitions(${PROJECT_NAME} PRIVATE USE_HAL_DRIVER STM32F103xE)
target_compile_options(${PROJECT_NAME} PRIVATE -mcpu=cortex-m3 -mthumb -Wall -Wextra
    -ffunction-sections -fdata-sections
    $<$<CONFIG:Debug>:-Og;-g3>
    $<$<CONFIG:Release>:-Os;-g3>)
target_link_options(${PROJECT_NAME} PRIVATE -mcpu=cortex-m3 -mthumb
    "-T${CMAKE_CURRENT_SOURCE_DIR}/STM32F103RC_FLASH.ld"
    --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections
    "-Wl,-Map=${PROJECT_NAME}.map" -Wl,--print-memory-usage)
set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX .elf)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin)
