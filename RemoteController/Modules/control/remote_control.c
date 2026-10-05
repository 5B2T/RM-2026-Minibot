#include "control/remote_control.h"
#include "remote_config.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
static Remote_Status Status;
static uint16_t Centers[4],Candidate[4],RawKeys,StableKeys;
static uint8_t KeyStarted,CenterStarted,AwaitingAck;
static uint32_t KeyTick,CenterTick,ResumeTick,UpdateTick;
static const int8_t Signs[4]=REMOTE_AXIS_SIGNS;
static const float Speeds[]={.2f,.35f,.5f};
static const float Turns[]={.5f,.8f,1.2f}; // rad/s, provisional commissioning values.
static uint8_t Neutral(void){for(unsigned i=0;i<4;i++)if(Status.Axis[i]!=0)return 0;return 1;}
void Controller_Lock(void)
{Status.Mode=REMOTE_LOCKED;Status.Vx=Status.Vy=Status.Wz=0;Status.ArmForward=Status.ArmRight=Status.ArmUp=Status.Gripper=0;AwaitingAck=0;}
void Controller_Init(void)
{memset(&Status,0,sizeof(Status));KeyStarted=CenterStarted=AwaitingAck=0;RawKeys=StableKeys=0;UpdateTick=0;Controller_Lock();}
static float Axis(uint16_t Raw,uint16_t Center,int8_t Sign)
{
 int d=(int)Raw-Center;if(d>=-REMOTE_AXIS_DEADZONE&&d<=REMOTE_AXIS_DEADZONE)return 0;
 float n=d>0?(float)(d-REMOTE_AXIS_DEADZONE)/(4095-Center-REMOTE_AXIS_DEADZONE):
             (float)(d+REMOTE_AXIS_DEADZONE)/(Center-REMOTE_AXIS_DEADZONE);
 if(n>1)n=1;
 if(n< -1)n=-1;
 return n*Sign;
}
void Controller_Update(const Remote_Input *Input,uint32_t NowMs)
{
 uint32_t Elapsed=NowMs-UpdateTick;UpdateTick=NowMs;
 if(Elapsed>50)Elapsed=50;
 Status.InputValid=Input!=NULL&&Input->Valid&&(uint32_t)(NowMs-Input->TickMs)<REMOTE_INPUT_STALE_MS;
 if(Status.InputValid)for(unsigned i=0;i<5;i++)if(Input->ADC[i]>4095)Status.InputValid=0;
 uint16_t Keys=Input!=NULL?Input->Keys:0;
 if(!KeyStarted){KeyStarted=1;RawKeys=StableKeys=Keys;KeyTick=NowMs;}
 if(Keys!=RawKeys){RawKeys=Keys;KeyTick=NowMs;}
 uint16_t Rising=0;
 if((uint32_t)(NowMs-KeyTick)>=REMOTE_DEBOUNCE_MS){Rising=Keys&~StableKeys;StableKeys=Keys;}
 Status.Keys=StableKeys;
 // GPIO keys remain usable for inspection even if ADC calibration/conversion fails.
 if(Rising&KEY_MASK(KEY_L_PRESS))Status.Page=(Status.Page+1)%3;
 if(Keys&KEY_MASK(KEY_STOP))Controller_Lock(); // STOP bypasses key debounce.
 if(Status.LinkAck&&(uint32_t)(NowMs-Status.LastAckMs)>=REMOTE_LINK_STALE_MS){Status.LinkAck=0;Controller_Lock();}
 if(Status.Mode==REMOTE_RESUMING&&(uint32_t)(NowMs-ResumeTick)>=REMOTE_LINK_STALE_MS)Controller_Lock();
 if(!Status.InputValid){Controller_Lock();CenterStarted=0;return;}
 memcpy(Status.Raw,Input->ADC,sizeof(Status.Raw));Status.SupplyV=Input->ADC[4]*(REMOTE_ADC_REFERENCE_V*REMOTE_POWER_DIVIDER/4095.0f);
 if(!Status.Calibrated)
 {
  uint8_t Stable=Keys==0;
  for(unsigned i=0;i<4;i++)if(Input->ADC[i]<REMOTE_CENTER_MIN||Input->ADC[i]>REMOTE_CENTER_MAX)Stable=0;
  if(!Stable){CenterStarted=0;return;}
  if(CenterStarted)for(unsigned i=0;i<4;i++)if(abs((int)Input->ADC[i]-Candidate[i])>REMOTE_CENTER_STABILITY)CenterStarted=0;
  if(!CenterStarted){memcpy(Candidate,Input->ADC,sizeof(Candidate));CenterTick=NowMs;CenterStarted=1;}
  if((uint32_t)(NowMs-CenterTick)<REMOTE_CENTER_WAIT_MS)return;
  memcpy(Centers,Candidate,sizeof(Centers));Status.Calibrated=1;
 }
 for(unsigned i=0;i<4;i++)Status.Axis[i]=Axis(Input->ADC[i],Centers[i],Signs[i]);
 uint8_t Idle=Neutral()&&!(Keys&REMOTE_ACTION_KEYS)&&!(StableKeys&REMOTE_ACTION_KEYS);
 uint16_t OtherKeys=(uint16_t)~KEY_MASK(KEY_RESUME);
 uint8_t RecoveryIdle=Idle&&!(Keys&OtherKeys)&&!(StableKeys&OtherKeys);
 if(Idle){if((Rising&KEY_MASK(KEY_L_UP))&&!(Keys&KEY_MASK(KEY_L_DOWN))&&Status.Gear<2)Status.Gear++;
          if((Rising&KEY_MASK(KEY_L_DOWN))&&!(Keys&KEY_MASK(KEY_L_UP))&&Status.Gear>0)Status.Gear--;}
 // A fresh press is required; a held key across connection recovery cannot unlock.
 if((Rising&KEY_MASK(KEY_RESUME))&&RecoveryIdle&&Status.LinkAck&&Status.Mode==REMOTE_LOCKED)
 {Status.Mode=REMOTE_RESUMING;ResumeTick=NowMs;AwaitingAck=1;}
 if(Status.Mode==REMOTE_RESUMING&&!RecoveryIdle)Controller_Lock();
 if(Status.Mode!=REMOTE_ACTIVE)return;
 Status.Vx=Status.Axis[REMOTE_LEFT_Y_INDEX]*Speeds[Status.Gear];
 Status.Vy=-Status.Axis[REMOTE_LEFT_X_INDEX]*Speeds[Status.Gear];
 // Cap diagonal translation to the selected speed, preserve requested direction.
 float norm=sqrtf(Status.Vx*Status.Vx+Status.Vy*Status.Vy);
 if(norm>Speeds[Status.Gear]){Status.Vx*=Speeds[Status.Gear]/norm;Status.Vy*=Speeds[Status.Gear]/norm;}
 int Turn=((StableKeys&KEY_MASK(KEY_L_LEFT))!=0)-((StableKeys&KEY_MASK(KEY_L_RIGHT))!=0);
 if(Turn==0)Status.Wz=0; // Release or opposing keys stops turning immediately.
 else
 {
  float Target=Turn*Turns[Status.Gear];
  if(Status.Wz*Target<0)Status.Wz=0;
  float Delta=Target-Status.Wz,Limit=REMOTE_TURN_ACCEL_RAD_S2*Elapsed*.001f;
  if(Delta>Limit)Delta=Limit;
  if(Delta< -Limit)Delta=-Limit;
  Status.Wz+=Delta;
 }
 Status.ArmForward=Status.Axis[REMOTE_RIGHT_Y_INDEX];Status.ArmRight=Status.Axis[REMOTE_RIGHT_X_INDEX];
 Status.ArmUp=((StableKeys&KEY_MASK(KEY_R_UP))!=0)-((StableKeys&KEY_MASK(KEY_R_DOWN))!=0);
 Status.Gripper=((StableKeys&KEY_MASK(KEY_R_LEFT))!=0)-((StableKeys&KEY_MASK(KEY_R_RIGHT))!=0);
}
void Controller_OnTx(uint8_t Flags,uint8_t Ack,uint32_t NowMs)
{
 if(!Ack){Status.LinkAck=0;Controller_Lock();return;}
 Status.LinkAck=1;Status.LastAckMs=NowMs;
 if(Flags==REMOTE_CONTROL_FLAG_RESUME&&AwaitingAck&&Status.Mode==REMOTE_RESUMING)
 {Status.Mode=REMOTE_ACTIVE;AwaitingAck=0;}
}
void Controller_Command(RemoteControl_FrameTypeDef *Frame,uint8_t Sequence)
{
 if(Frame==NULL)return;
 *Frame=(RemoteControl_FrameTypeDef){.Header=0xa5,.Version=REMOTE_CONTROL_VERSION_VELOCITY,.Sequence=Sequence};
 if(Status.Mode==REMOTE_LOCKED)Frame->Flags=REMOTE_CONTROL_FLAG_STOP;
 else if(Status.Mode==REMOTE_RESUMING)Frame->Flags=REMOTE_CONTROL_FLAG_RESUME;
 else {Frame->M1=(int16_t)lroundf(Status.Vx*1000);Frame->M2=(int16_t)lroundf(Status.Vy*1000);Frame->M3=(int16_t)lroundf(Status.Wz*1000);}
}
Remote_Status Controller_GetStatus(void){return Status;}
