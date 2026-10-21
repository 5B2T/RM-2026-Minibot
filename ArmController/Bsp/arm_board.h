#ifndef ARM_CONTROLLER_BOARD_H
#define ARM_CONTROLLER_BOARD_H
#include "main.h"
#include "remote_def.h"
// Frames arrive every 20 ms; age a sample at 60 ms so one lost frame is not treated as an outage.
#define ARM_BOARD_INPUT_STALE_MS 60U
void ArmBoard_Init(void);
Remote_Input ArmBoard_Input(uint32_t Now);
uint8_t ArmBoard_Pots(uint16_t Values[5]);
void ArmBoard_Service(void);
void ArmBoard_Print(const char *Text);
extern volatile uint32_t ArmBoard_RxCount,ArmBoard_Errors;
#endif
