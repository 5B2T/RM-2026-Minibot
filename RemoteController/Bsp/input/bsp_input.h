#ifndef BSP_REMOTE_INPUT_H
#define BSP_REMOTE_INPUT_H
#include "remote_def.h"
void BSP_InputInit(void);
void BSP_InputStep(uint32_t NowMs);
Remote_Input BSP_InputRead(uint32_t NowMs);
#endif
