#include "input/bsp_buttons.h"
#include "board_config.h"
uint8_t BSP_Buttons_Read(void)
{
    uint8_t Keys=0;
    if(HAL_GPIO_ReadPin(BOARD_BUTTON_NEXT_PORT,BOARD_BUTTON_NEXT_PIN)==GPIO_PIN_RESET)
        Keys|=BSP_BUTTON_NEXT;
    if(HAL_GPIO_ReadPin(BOARD_BUTTON_BACK_PORT,BOARD_BUTTON_BACK_PIN)==GPIO_PIN_RESET)
        Keys|=BSP_BUTTON_BACK;
    return Keys;
}
