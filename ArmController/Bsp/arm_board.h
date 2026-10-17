#ifndef ARM_CONTROLLER_BOARD_H
#define ARM_CONTROLLER_BOARD_H
#include "main.h"
#include "remote_def.h"
void ArmBoard_Init(void);
Remote_Input ArmBoard_Input(uint32_t Now);
uint8_t ArmBoard_Pots(uint16_t Values[5]);
void ArmBoard_Service(void);
void ArmBoard_Print(const char *Text);
extern volatile uint32_t ArmBoard_RxCount,ArmBoard_Errors;
#endif
