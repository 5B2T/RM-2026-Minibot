#ifndef REMOTE_DEF_H
#define REMOTE_DEF_H
#include <stdint.h>
typedef enum {KEY_L_UP,KEY_L_DOWN,KEY_L_LEFT,KEY_L_RIGHT,
 KEY_R_UP,KEY_R_DOWN,KEY_R_LEFT,KEY_R_RIGHT,KEY_L_PRESS,KEY_R_PRESS,
 KEY_STOP,KEY_RESUME,KEY_COUNT} Remote_Key;
#define KEY_MASK(k) (1U<<(k))
#define REMOTE_ACTION_KEYS (KEY_MASK(KEY_L_LEFT)|KEY_MASK(KEY_L_RIGHT)|KEY_MASK(KEY_R_UP)|KEY_MASK(KEY_R_DOWN)|KEY_MASK(KEY_R_LEFT)|KEY_MASK(KEY_R_RIGHT))
typedef struct {uint16_t ADC[5];uint16_t Keys;uint32_t TickMs;uint8_t Valid;} Remote_Input;
typedef enum {REMOTE_LOCKED,REMOTE_RESUMING,REMOTE_ACTIVE} Remote_Mode;
typedef struct {
 Remote_Mode Mode;uint8_t Calibrated,InputValid,LinkAck,Gear,Page;
 float Axis[4],Vx,Vy,Wz,SupplyV;
 float ArmForward,ArmRight,ArmUp,Gripper; // Reserved local intent, NOT transmitted yet.
 uint16_t Raw[5],Keys;uint32_t LastAckMs;
} Remote_Status;
#endif
