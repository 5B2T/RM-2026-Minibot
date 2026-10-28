#include "input/bsp_input.h"
#include "remote_config.h"
#include "main.h"
#include "adc.h"
#include <string.h>
static _Alignas(4) uint16_t Samples[5];
static Remote_Input Last;
static volatile uint8_t Done,Failed;
static volatile uint32_t DoneTick;
static uint8_t Ready,Pending;
static uint32_t Started,LastStart,Retry;
void BSP_InputInit(void)
{memset(&Last,0,sizeof(Last));Ready=Pending=Done=Failed=0;Retry=HAL_GetTick()-1000;LastStart=HAL_GetTick()-REMOTE_INPUT_PERIOD_MS;}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *ADC){if(ADC==&hadc1){DoneTick=HAL_GetTick();Done=1;}}
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *ADC){if(ADC==&hadc1)Failed=1;}
void BSP_InputStep(uint32_t NowMs)
{
 if(!Ready){if((uint32_t)(NowMs-Retry)<1000)return;Retry=NowMs;
  if(HAL_ADCEx_Calibration_Start(&hadc1)!=HAL_OK)return;
  Ready=1;
 }
 if(Pending){
  if(Failed||(!Done&&(uint32_t)(NowMs-Started)>=10)){(void)HAL_ADC_Stop_DMA(&hadc1);Pending=0;Ready=0;Last.Valid=0;Retry=NowMs;return;}
  if(!Done)return;
  if(HAL_ADC_Stop_DMA(&hadc1)!=HAL_OK){Ready=Pending=0;Last.Valid=0;Retry=NowMs;return;}
  memcpy(Last.ADC,Samples,sizeof(Samples));Last.TickMs=DoneTick;Last.Valid=1;Pending=0;
 }
 if((uint32_t)(NowMs-LastStart)<REMOTE_INPUT_PERIOD_MS)return;
 Done=Failed=0;LastStart=Started=NowMs;
 if(HAL_ADC_Start_DMA(&hadc1,(uint32_t *)Samples,5)==HAL_OK)Pending=1;
 else {Ready=0;Last.Valid=0;Retry=NowMs;}
}
static uint16_t Key(GPIO_TypeDef *Port,uint16_t Pin,Remote_Key Which,GPIO_PinState Active)
{return HAL_GPIO_ReadPin(Port,Pin)==Active?KEY_MASK(Which):0;}
Remote_Input BSP_InputRead(uint32_t NowMs)
{
 Remote_Input Input=Last;
 Input.Valid=Input.Valid&&(uint32_t)(NowMs-Input.TickMs)<REMOTE_INPUT_STALE_MS;
 Input.Keys=Key(GPIOA,GPIO_PIN_11,KEY_L_UP,GPIO_PIN_RESET)|Key(GPIOA,GPIO_PIN_12,KEY_L_DOWN,GPIO_PIN_RESET)|
 Key(GPIOA,GPIO_PIN_15,KEY_L_LEFT,GPIO_PIN_RESET)|Key(GPIOB,GPIO_PIN_3,KEY_L_RIGHT,GPIO_PIN_RESET)|
 Key(GPIOB,GPIO_PIN_6,KEY_R_UP,GPIO_PIN_RESET)|Key(GPIOB,GPIO_PIN_7,KEY_R_DOWN,GPIO_PIN_RESET)|
 Key(GPIOB,GPIO_PIN_8,KEY_R_LEFT,GPIO_PIN_RESET)|Key(GPIOB,GPIO_PIN_9,KEY_R_RIGHT,GPIO_PIN_RESET)|
 Key(GPIOC,GPIO_PIN_14,KEY_GRIPPER,GPIO_PIN_RESET)|Key(GPIOA,GPIO_PIN_0,KEY_DEPLOY,GPIO_PIN_SET)|
 Key(GPIOC,GPIO_PIN_15,KEY_R_PRESS,GPIO_PIN_RESET);
 return Input;
}
