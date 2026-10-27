#ifndef BSP_MOTOR_H
#define BSP_MOTOR_H
#include <stdint.h>
#define BSP_MOTOR_COUNT 4U
#define BSP_MOTOR_PWM_MAX 999
// Initialize before scheduling; all output APIs belong to the control task afterwards.
uint8_t BSP_Motor_Init(void);
void BSP_Motor_Enable(void);
void BSP_Motor_Disable(void); // Clear chassis PWM/STBY; independent M5 base is unchanged.
void BSP_Motor_SetBaseEnabled(uint8_t Enabled); // M5 base STBY PD7, control-task only.
void BSP_Motor_Load(const int PWM[BSP_MOTOR_COUNT]); // Signed PWM, clamped to +/-999.
#endif
