// Test the real arm BSP: HAL rejects starting an already-running encoder.
#include "board_config.h"
#include "arm/bsp_arm.h"
#include <assert.h>
#include <stdio.h>
TIM_HandleTypeDef FakeTimers[6];
static unsigned EncoderStarts;
static uint8_t FailPWM,BaseEnabled;
static GPIO_PinState Pins[4];
void BSP_Motor_SetBaseEnabled(uint8_t E){BaseEnabled=E;}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *T,uint32_t C)
{if(FailPWM || T->Running[C])return HAL_ERROR;T->Running[C]=1;return HAL_OK;}
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *T,uint32_t C)
{T->Running[C]=0;return HAL_OK;}
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *T,uint32_t C)
{(void)C;EncoderStarts++;if(T->Encoder)return HAL_ERROR;T->Encoder=1;return HAL_OK;}
GPIO_PinState HAL_GPIO_ReadPin(unsigned P,unsigned N){(void)P;return Pins[N];}
void HAL_GPIO_WritePin(unsigned P,unsigned N,GPIO_PinState S){(void)P;Pins[N]=S;}
int main(void)
{
 uint16_t Off[5]={0},Hold[5]={1500,1500,1500,1500,1500};
 assert(BSP_Arm_Init(Off));assert(EncoderStarts==1);
 assert(BSP_Arm_Apply(Hold,300));assert(BaseEnabled);
 BOARD_ARM_ENCODER_TIMER.Counter=321;BSP_Arm_Disable();assert(!BaseEnabled);
 assert(BOARD_ARM_ENCODER_TIMER.Encoder && BSP_Arm_EncoderCount()==321);
 assert(BSP_Arm_Init(Off));assert(EncoderStarts==1 && BSP_Arm_EncoderCount()==321);
 assert(!BaseEnabled);for(unsigned i=0;i<5;i++)assert(FakeTimers[i].Channel[0]==0);
 BSP_Arm_Disable();FailPWM=1;assert(!BSP_Arm_Init(Off) && !BaseEnabled);
 assert(BSP_Arm_EncoderCount()==321);FailPWM=0;assert(BSP_Arm_Init(Off));
 assert(EncoderStarts==1 && BSP_Arm_EncoderCount()==321);
 puts("PASS: arm BSP reopens zero PWM without restarting/resetting live encoder; failed reinit stays disabled");
 return 0;
}
