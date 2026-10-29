#include "i2c/bsp_i2c.h"
#include "board_config.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include <stddef.h>

static StaticSemaphore_t BusMutexStorage;
static SemaphoreHandle_t BusMutex;
static BSP_I2C_Status Status;
static uint8_t RecoveryAttempted;

// GPIO bus-clear edges are deliberately slower than standard-mode minima.
// Interrupts stay enabled: motor and radio tasks can preempt this recovery.
static void BusEdgeDelay(void)
{
    uint32_t Count=SystemCoreClock/1000000U*5U+1U;
    for(volatile uint32_t i=0;i<Count;i++)__NOP();
}
static uint8_t ClockHigh(void)
{
    uint32_t Start=HAL_GetTick();
    while(HAL_GPIO_ReadPin(BOARD_I2C_GPIO_PORT,BOARD_I2C_SCL_PIN)==GPIO_PIN_RESET)
        if((uint32_t)(HAL_GetTick()-Start)>=2U)return 0;
    return 1;
}
static uint8_t RecoverBus(void)
{
    uint8_t Released=0;
    // Do not call MX_I2C1_Init: a failed accessory must not enter Error_Handler.
    (void)HAL_I2C_DeInit(&BOARD_DEVICE_I2C);
    HAL_GPIO_WritePin(BOARD_I2C_GPIO_PORT,BOARD_I2C_SCL_PIN|BOARD_I2C_SDA_PIN,GPIO_PIN_SET);
    GPIO_InitTypeDef Pins={0};
    Pins.Pin=BOARD_I2C_SCL_PIN|BOARD_I2C_SDA_PIN;
    Pins.Mode=GPIO_MODE_OUTPUT_OD;Pins.Pull=GPIO_NOPULL;Pins.Speed=GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BOARD_I2C_GPIO_PORT,&Pins);
    if(!ClockHigh())goto Finish;
    for(unsigned i=0;i<9U && HAL_GPIO_ReadPin(BOARD_I2C_GPIO_PORT,BOARD_I2C_SDA_PIN)==GPIO_PIN_RESET;i++)
    {
        HAL_GPIO_WritePin(BOARD_I2C_GPIO_PORT,BOARD_I2C_SCL_PIN,GPIO_PIN_RESET);BusEdgeDelay();
        HAL_GPIO_WritePin(BOARD_I2C_GPIO_PORT,BOARD_I2C_SCL_PIN,GPIO_PIN_SET);
        if(!ClockHigh())goto Finish;
        BusEdgeDelay();
    }
    // Form STOP without a preceding unintended START: SDA falls while SCL low.
    HAL_GPIO_WritePin(BOARD_I2C_GPIO_PORT,BOARD_I2C_SCL_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BOARD_I2C_GPIO_PORT,BOARD_I2C_SDA_PIN,GPIO_PIN_RESET);BusEdgeDelay();
    HAL_GPIO_WritePin(BOARD_I2C_GPIO_PORT,BOARD_I2C_SCL_PIN,GPIO_PIN_SET);
    if(!ClockHigh())goto Finish;
    BusEdgeDelay();HAL_GPIO_WritePin(BOARD_I2C_GPIO_PORT,BOARD_I2C_SDA_PIN,GPIO_PIN_SET);BusEdgeDelay();
    Released=HAL_GPIO_ReadPin(BOARD_I2C_GPIO_PORT,BOARD_I2C_SDA_PIN)==GPIO_PIN_SET;
Finish:
    HAL_GPIO_WritePin(BOARD_I2C_GPIO_PORT,BOARD_I2C_SCL_PIN|BOARD_I2C_SDA_PIN,GPIO_PIN_SET);
    Pins.Mode=GPIO_MODE_AF_OD;Pins.Alternate=BOARD_I2C_GPIO_AF;Pins.Speed=GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(BOARD_I2C_GPIO_PORT,&Pins);
    HAL_StatusTypeDef Init=HAL_I2C_Init(&BOARD_DEVICE_I2C);
    Status.SclHigh=HAL_GPIO_ReadPin(BOARD_I2C_GPIO_PORT,BOARD_I2C_SCL_PIN)==GPIO_PIN_SET;
    Status.SdaHigh=HAL_GPIO_ReadPin(BOARD_I2C_GPIO_PORT,BOARD_I2C_SDA_PIN)==GPIO_PIN_SET;
    return Released && Init==HAL_OK && Status.SclHigh && Status.SdaHigh;
}
static BSP_I2C_DeviceStatus *Device(uint8_t Address)
{
    for(unsigned i=0;i<BSP_I2C_TRACKED_DEVICES;i++)
        if(Status.Device[i].Used && Status.Device[i].Address==Address)return &Status.Device[i];
    for(unsigned i=0;i<BSP_I2C_TRACKED_DEVICES;i++)
        if(!Status.Device[i].Used)
        {Status.Device[i].Used=1;Status.Device[i].Address=Address;return &Status.Device[i];}
    return NULL;
}
static uint8_t FinishTransfer(uint8_t Address,HAL_StatusTypeDef Result)
{
    BSP_I2C_DeviceStatus *D=Device(Address);
    uint32_t Error=BOARD_DEVICE_I2C.ErrorCode,Now=HAL_GetTick();
    Status.Transfers++;Status.LastAddress=Address;Status.LastHalStatus=(uint8_t)Result;
    if(Result==HAL_OK)
    {
        if(D){D->Successes++;D->FailureStreak=0;}
    }
    else
    {
        Status.Errors++;Status.LastErrorCode=Error;Status.LastErrorTick=Now;
        if(D)
        {
            D->Failures++;if(D->FailureStreak<255U)D->FailureStreak++;
            if(Error&HAL_I2C_ERROR_AF)D->Nacks++;
        }
        uint8_t Recover=Result==HAL_BUSY || Result==HAL_TIMEOUT ||
            (Error&(HAL_I2C_ERROR_BERR|HAL_I2C_ERROR_ARLO|HAL_I2C_ERROR_OVR)) ||
            (D && D->FailureStreak>=BOARD_I2C_NACK_RECOVERY_COUNT);
        if(Recover && (!RecoveryAttempted || (uint32_t)(Now-Status.LastRecoveryTick)>=BOARD_I2C_RECOVERY_INTERVAL_MS))
        {
            RecoveryAttempted=1;Status.LastRecoveryTick=Now;Status.Recoveries++;
            if(!RecoverBus())Status.RecoveryFailures++;
        }
    }
    // A failed operation remains failed. Device state machines retry later;
    // never replay a partly accepted register write or invent a valid sample.
    (void)xSemaphoreGive(BusMutex);
    return Result==HAL_OK;
}
BSP_I2C_Status BSP_I2C_GetStatus(void)
{
    taskENTER_CRITICAL();BSP_I2C_Status Copy=Status;taskEXIT_CRITICAL();return Copy;
}

uint8_t BSP_I2C_Init(void)
{
    if(BusMutex==NULL)BusMutex=xSemaphoreCreateMutexStatic(&BusMutexStorage);
    return BusMutex!=NULL;
}

uint8_t BSP_I2C_Write(uint8_t Address, const uint8_t *Data, uint16_t Length)
{
    if(Address>0x7fU || Data==NULL || Length==0 ||
       Length>BOARD_I2C_MAX_WRITE_SIZE || BusMutex==NULL)return 0;
    if(xSemaphoreTake(BusMutex,pdMS_TO_TICKS(BOARD_I2C_LOCK_TIMEOUT_MS))!=pdTRUE){Status.LockTimeouts++;return 0;}
    HAL_StatusTypeDef Result=HAL_I2C_Master_Transmit(&BOARD_DEVICE_I2C,
        (uint16_t)Address<<1,(uint8_t *)Data,Length,BOARD_I2C_WRITE_TIMEOUT_MS);
    return FinishTransfer(Address,Result);
}
uint8_t BSP_I2C_ReadRegister(uint8_t Address, uint8_t Register, uint8_t *Data, uint16_t Length)
{
    if(Address>0x7fU || Data==NULL || Length==0 ||
       Length>BOARD_I2C_MAX_WRITE_SIZE || BusMutex==NULL)return 0;
    if(xSemaphoreTake(BusMutex,pdMS_TO_TICKS(BOARD_I2C_LOCK_TIMEOUT_MS))!=pdTRUE){Status.LockTimeouts++;return 0;}
    HAL_StatusTypeDef Result=HAL_I2C_Mem_Read(&BOARD_DEVICE_I2C,(uint16_t)Address<<1,
        Register,I2C_MEMADD_SIZE_8BIT,Data,Length,BOARD_I2C_WRITE_TIMEOUT_MS);
    return FinishTransfer(Address,Result);
}
