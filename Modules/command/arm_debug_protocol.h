#ifndef ARM_DEBUG_PROTOCOL_H
#define ARM_DEBUG_PROTOCOL_H
#include <stdint.h>
#define ARM_DEBUG_LINE_SIZE 96U
typedef enum {
    ARM_DEBUG_ENABLE,ARM_DEBUG_DISABLE,ARM_DEBUG_HEARTBEAT,ARM_DEBUG_STOP,
    ARM_DEBUG_STATUS,ARM_DEBUG_STREAM,ARM_DEBUG_POSITION,ARM_DEBUG_RATE,ARM_DEBUG_PRESET,ARM_DEBUG_NEUTRAL,ARM_DEBUG_ZERO,ARM_DEBUG_INITIALIZE
} ArmDebug_Operation;
typedef struct {
    ArmDebug_Operation Operation;
    uint8_t Joint;
    float Value;
    uint32_t DurationMs;
} ArmDebug_Command;
// Complete, NUL-terminated ASCII line without CR/LF. No HAL or actuator access.
uint8_t ArmDebugProtocol_Parse(const char *Line,ArmDebug_Command *Command);
#endif
