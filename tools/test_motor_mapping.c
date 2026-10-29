#include "main.h"
#include "tim.h"
#include "motor/bsp_motor.h"
#include "arm/bsp_arm.h"
#include "encoder/bsp_encoder.h"
#include <assert.h>
#include <stdio.h>
GPIO_TypeDef TestB,TestC,TestD,TestE,TestF;
TIM_HandleTypeDef htim1,htim2,htim3,htim4,htim5,htim8,htim9,htim10,htim11,htim12,htim13;
void HAL_GPIO_WritePin(GPIO_TypeDef *G,uint16_t P,GPIO_PinState V){if(V)G->Pins|=P;else G->Pins&=(uint16_t)~P;}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *G,uint16_t P){return (G->Pins&P)?GPIO_PIN_SET:GPIO_PIN_RESET;}
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *T,uint32_t C){T->Running|=1U<<C;return HAL_OK;}
int HAL_TIM_PWM_Stop(TIM_HandleTypeDef *T,uint32_t C){T->Running&=~(1U<<C);return HAL_OK;}
int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *T,uint32_t C){T->Running=C;return HAL_OK;}
int HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *T,uint32_t C){(void)C;T->Running=0;return HAL_OK;}
int main(void){
 uint16_t Pulse[5]={0};int Pwm[4]={101,102,103,104};
 assert(BSP_Motor_Init() && BSP_Encoder_Init() && BSP_Arm_Init(Pulse));
 assert(!(TestD.Pins&(GPIO_PIN_4|GPIO_PIN_7)) && !(TestB.Pins&GPIO_PIN_11));
 assert(BSP_Arm_Apply(Pulse,200));
 assert(htim13.Ccr[0]==200 && (TestD.Pins&GPIO_PIN_7) && !(TestD.Pins&GPIO_PIN_4));
 BSP_Motor_Disable(); // Repeated chassis stop cannot interrupt independent base M5.
 assert(htim13.Ccr[0]==200 && (TestD.Pins&GPIO_PIN_7));
 BSP_Motor_Load(Pwm);BSP_Motor_Enable();
 assert(htim1.Ccr[2]==101 && htim1.Ccr[3]==102 && htim1.Ccr[0]==103 && htim1.Ccr[1]==104);
 assert(BSP_Arm_Apply(Pulse,0)); // Base stop cannot interrupt chassis.
 assert(htim13.Ccr[0]==0 && htim1.Ccr[3]==102 && (TestD.Pins&GPIO_PIN_4) && !(TestD.Pins&GPIO_PIN_7));
 assert(BSP_Arm_Apply(Pulse,-120) && !(TestD.Pins&GPIO_PIN_5) && (TestD.Pins&GPIO_PIN_6));
 BSP_Arm_Disable();
 assert(htim13.Ccr[0]==0 && htim1.Ccr[3]==102 && (htim1.Running&(1U<<3)) && (TestD.Pins&GPIO_PIN_4));
 BSP_Motor_Disable();
 assert(!(TestD.Pins&(GPIO_PIN_4|GPIO_PIN_7)) && !(TestB.Pins&GPIO_PIN_11));
 htim2.Count=11;htim8.Count=22;htim3.Count=33;htim5.Count=44;htim4.Count=55;
 assert(BSP_Encoder_GetCount(0)==11 && BSP_Encoder_GetCount(1)==22);
 assert(BSP_Encoder_GetCount(2)==44 && BSP_Encoder_GetCount(3)==33 && BSP_Arm_EncoderCount()==55);
 puts("PASS: TB3/4/1/2 physical-wheel mapping, TB5 base, independent enable/stop/reverse, paired encoders");
 return 0;
}
