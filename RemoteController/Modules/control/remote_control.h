#ifndef REMOTE_CONTROLLER_H
#define REMOTE_CONTROLLER_H
#include "remote_def.h"
#include "command/command_protocol.h"
void Controller_Init(void);
void Controller_Update(const Remote_Input *Input,uint32_t NowMs);
void Controller_Lock(void);
void Controller_OnTx(uint8_t Flags,uint8_t Ack,uint32_t NowMs);
void Controller_Command(RemoteControl_FrameTypeDef *Frame,uint8_t Sequence);
Remote_Status Controller_GetStatus(void);
#endif
