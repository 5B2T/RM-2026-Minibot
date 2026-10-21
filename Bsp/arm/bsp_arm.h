#ifndef BSP_ARM_H
#define BSP_ARM_H
#include <stdint.h>
#define BSP_ARM_SERVO_COUNT 5U
#define BSP_ARM_BASE_PWM_MAX 999
// Init before scheduling; subsequent hardware writes belong to the control task.
// A zero pulse disables that channel's signal; otherwise 500..2500 us.
uint8_t BSP_Arm_Init(const uint16_t InitialPulseUs[BSP_ARM_SERVO_COUNT]);
uint8_t BSP_Arm_Apply(const uint16_t PulseUs[BSP_ARM_SERVO_COUNT], int BasePWM);
uint16_t BSP_Arm_EncoderCount(void);
void BSP_Arm_Disable(void); // Fault path: all PWM off, base STBY low.
#endif
