#include "encoder/bsp_encoder.h"
#include "board_config.h"
static TIM_HandleTypeDef *const Timers[BSP_ENCODER_COUNT]=BOARD_ENCODER_MAP;
static uint16_t LastCount[BSP_ENCODER_COUNT];

uint8_t BSP_Encoder_Init(void)
{
    for(uint8_t i=0;i<BSP_ENCODER_COUNT;i++)
    {
        if(HAL_TIM_Encoder_Start(Timers[i], TIM_CHANNEL_ALL)!=HAL_OK)return 0;
        BSP_Encoder_Sync(i);
    }
    return 1;
}

uint16_t BSP_Encoder_GetCount(uint8_t Channel)
{
    if(Channel>=BSP_ENCODER_COUNT)return 0;
    return (uint16_t)__HAL_TIM_GET_COUNTER(Timers[Channel]);
}

void BSP_Encoder_Sync(uint8_t Channel)
{
    if(Channel<BSP_ENCODER_COUNT)LastCount[Channel]=BSP_Encoder_GetCount(Channel);
}

int16_t BSP_Encoder_GetDelta(uint8_t Channel)
{
    if(Channel>=BSP_ENCODER_COUNT)return 0;
    uint16_t Count=BSP_Encoder_GetCount(Channel);
    uint16_t RawDelta=(uint16_t)(Count-LastCount[Channel]);
    int32_t Delta=RawDelta;
    if(RawDelta>=0x8000U)Delta-=0x10000L;
    LastCount[Channel]=Count;
    return (int16_t)Delta;
}
