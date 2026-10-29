#ifndef BSP_I2C_H
#define BSP_I2C_H
#include <stdint.h>
// Initialize once from Robot_Init before tasks start. No GPIO or clock changes.
uint8_t BSP_I2C_Init(void);
// Task context only. Address is seven-bit; return 1 only for an acknowledged write.
// Serialized short transactions; bounded bus recovery runs under the same mutex.
uint8_t BSP_I2C_Write(uint8_t Address, const uint8_t *Data, uint16_t Length);
// Register-pointer write + repeated-start read under the same bus lock.
uint8_t BSP_I2C_ReadRegister(uint8_t Address, uint8_t Register, uint8_t *Data, uint16_t Length);
#define BSP_I2C_TRACKED_DEVICES 4U
typedef struct
{
    uint32_t Successes,Failures,Nacks;
    uint8_t Address,Used,FailureStreak;
} BSP_I2C_DeviceStatus;
typedef struct
{
    uint32_t Transfers,Errors,LockTimeouts,Recoveries,RecoveryFailures;
    uint32_t LastErrorCode,LastErrorTick,LastRecoveryTick;
    uint8_t LastAddress,LastHalStatus,SclHigh,SdaHigh;
    BSP_I2C_DeviceStatus Device[BSP_I2C_TRACKED_DEVICES];
} BSP_I2C_Status;
// Diagnostic snapshot only; reading never resets hardware or clears counters.
BSP_I2C_Status BSP_I2C_GetStatus(void);
#endif
