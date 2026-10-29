#include "i2c/bsp_i2c.h"
#include "board_config.h"
#include "semphr.h"
#include <assert.h>
#include <stdio.h>
I2C_HandleTypeDef hi2c1;
GPIO_TypeDef TestPort={.Pins=BOARD_I2C_SCL_PIN|BOARD_I2C_SDA_PIN};
uint32_t SystemCoreClock=1000000;
static uint32_t Tick,Error;
static unsigned Locked,Creates,Takes,Gives,Transfers,Reinits,Mode,Pulses;
static unsigned CreateOK,TakeOK,ClockStuck,DataStuck,ReleaseAfter;
static HAL_StatusTypeDef Result,InitResult;
uint32_t HAL_GetTick(void){return Tick++;}
SemaphoreHandle_t xSemaphoreCreateMutexStatic(StaticSemaphore_t *S){Creates++;return CreateOK?S:NULL;}
int xSemaphoreTake(SemaphoreHandle_t S,uint32_t Wait)
{assert(S && Wait==5);Takes++;if(!TakeOK)return 0;assert(!Locked);Locked=1;return 1;}
int xSemaphoreGive(SemaphoreHandle_t S){assert(S && Locked);Locked=0;Gives++;return 1;}
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *H){assert(Locked && H==&hi2c1);return HAL_OK;}
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *H)
{assert(Locked && H==&hi2c1 && Mode==GPIO_MODE_AF_OD);Reinits++;return InitResult;}
void HAL_GPIO_Init(GPIO_TypeDef *P,GPIO_InitTypeDef *I)
{assert(Locked && P==&TestPort && I->Pin==(BOARD_I2C_SCL_PIN|BOARD_I2C_SDA_PIN));assert(I->Mode==GPIO_MODE_OUTPUT_OD || I->Mode==GPIO_MODE_AF_OD);Mode=I->Mode;}
void HAL_GPIO_WritePin(GPIO_TypeDef *P,uint16_t Pin,GPIO_PinState V)
{
    assert(Locked && P==&TestPort);
    if(Mode==GPIO_MODE_OUTPUT_OD && Pin==BOARD_I2C_SCL_PIN && V && !(P->Pins&Pin))
    {Pulses++;if(ReleaseAfter && Pulses>=ReleaseAfter)DataStuck=0;}
    if(V)P->Pins|=Pin;else P->Pins&=~Pin;
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *P,uint16_t Pin)
{assert(Locked && P==&TestPort);if((Pin==BOARD_I2C_SCL_PIN && ClockStuck)||(Pin==BOARD_I2C_SDA_PIN && DataStuck))return GPIO_PIN_RESET;return (P->Pins&Pin)?GPIO_PIN_SET:GPIO_PIN_RESET;}
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *H,uint16_t A,uint8_t *D,uint16_t N,uint32_t T)
{assert(Locked && H==&hi2c1 && (A==0x78||A==0x80) && D && N<=33 && T==10);Transfers++;H->ErrorCode=Error;return Result;}
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *H,uint16_t A,uint16_t R,uint16_t Size,uint8_t *D,uint16_t N,uint32_t T)
{assert(A==0x80 && R==2 && Size==1);return HAL_I2C_Master_Transmit(H,A,D,N,T);}
static uint8_t Write(HAL_StatusTypeDef R,uint32_t E)
{uint8_t Data[33]={0};Result=R;Error=E;uint8_t OK=BSP_I2C_Write(0x3c,Data,33);assert(!Locked);return OK;}
int main(void)
{
    uint8_t Data[33]={0};assert(!Write(HAL_OK,0));assert(!BSP_I2C_Init());
    CreateOK=1;assert(BSP_I2C_Init() && BSP_I2C_Init() && Creates==2);
    assert(!BSP_I2C_Write(128,Data,33) && !BSP_I2C_Write(0x3c,NULL,1));
    assert(!BSP_I2C_Write(0x3c,Data,0) && !BSP_I2C_Write(0x3c,Data,34));
    assert(!BSP_I2C_ReadRegister(0x40,2,NULL,2) && !BSP_I2C_ReadRegister(128,2,Data,2));
    assert(!Takes);assert(!Write(HAL_OK,0) && BSP_I2C_GetStatus().LockTimeouts==1);
    TakeOK=1;assert(Write(HAL_OK,0));
    assert(!Write(HAL_ERROR,HAL_I2C_ERROR_AF) && !Reinits);
    assert(!Write(HAL_ERROR,HAL_I2C_ERROR_AF) && !Reinits);
    Result=HAL_OK;Error=0;assert(BSP_I2C_ReadRegister(0x40,2,Data,2));
    DataStuck=1;ReleaseAfter=4;unsigned Before=Transfers;
    assert(!Write(HAL_ERROR,HAL_I2C_ERROR_AF));
    assert(Transfers==Before+1 && Reinits==1 && Pulses>=4 && Pulses<=10); // No failed-write replay.
    BSP_I2C_Status S=BSP_I2C_GetStatus();
    assert(S.Recoveries==1 && !S.RecoveryFailures && S.SclHigh && S.SdaHigh);
    assert(S.Device[0].Address==0x3c && S.Device[0].Nacks==3 && S.Device[0].FailureStreak==3);
    assert(S.Device[1].Address==0x40 && S.Device[1].Successes==1 && !S.Device[1].Failures);
    assert(Write(HAL_OK,0) && !BSP_I2C_GetStatus().Device[0].FailureStreak);
    assert(!Write(HAL_BUSY,0) && Reinits==1); // Global cooldown.
    Tick+=1000;ClockStuck=1;uint32_t Start=Tick;Before=Reinits;
    assert(!Write(HAL_TIMEOUT,0));S=BSP_I2C_GetStatus();
    assert(Reinits==Before+1 && S.RecoveryFailures==1 && Tick-Start<20 && Mode==GPIO_MODE_AF_OD);
    ClockStuck=0;DataStuck=1;ReleaseAfter=0;Pulses=0;Tick+=1000;
    assert(!Write(HAL_ERROR,HAL_I2C_ERROR_BERR));
    assert(Pulses<=10 && BSP_I2C_GetStatus().RecoveryFailures==2 && Mode==GPIO_MODE_AF_OD);
    DataStuck=0;InitResult=HAL_ERROR;Tick+=1000;
    assert(!Write(HAL_ERROR,HAL_I2C_ERROR_ARLO));
    assert(BSP_I2C_GetStatus().RecoveryFailures==3 && !Locked);
    InitResult=HAL_OK;Result=HAL_OK;Error=0;assert(BSP_I2C_ReadRegister(0x40,2,Data,2));assert(Write(HAL_OK,0));
    assert(Gives==Transfers && !Locked);
    puts("PASS: shared OLED/INA lock, isolated NACK counters, recovery cooldown, no replay,9-clock limit, stuck SCL deadline, init failure and later recovery");
    return 0;
}
