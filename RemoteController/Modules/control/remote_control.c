#include "control/remote_control.h"
#include "remote_config.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
static Remote_Status Status;
static uint16_t Centers[4],Candidate[4],RawKeys,StableKeys;
static uint8_t KeyStarted,CenterStarted,AwaitingAck,JointCentered;
static uint32_t KeyTick,CenterTick,ResumeTick,UpdateTick;
static const int8_t Signs[4]=REMOTE_AXIS_SIGNS;
static uint8_t Neutral(void){for(unsigned i=0;i<4;i++)if(Status.Axis[i]!=0)return 0;return 1;}
static void Lock(Remote_LockReason Reason,uint32_t NowMs)
{
 // Preserve the first cause of each lock; retries/secondary failures cannot overwrite it.
 if(Status.Mode!=REMOTE_LOCKED || Status.LastLockReason==REMOTE_LOCK_STARTUP)
 {Status.LastLockReason=Reason;Status.LastLockMs=NowMs;Status.LockCount++;}
 Status.Mode=REMOTE_LOCKED;Status.Vx=Status.Vy=Status.Wz=0;
 Status.ArmBase=Status.JointRate=0;AwaitingAck=JointCentered=0;
}
void Controller_Lock(void)
{Lock(REMOTE_LOCK_INTERNAL,UpdateTick);}
void Controller_Init(void)
{memset(&Status,0,sizeof(Status));KeyStarted=CenterStarted=AwaitingAck=JointCentered=0;RawKeys=StableKeys=0;UpdateTick=0;}
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
 uint8_t WasValid=Status.InputValid;
 Status.InputValid=Input!=NULL&&Input->Valid&&(uint32_t)(NowMs-Input->TickMs)<REMOTE_INPUT_STALE_MS;
 if(Status.InputValid)for(unsigned i=0;i<5;i++)if(Input->ADC[i]>4095)Status.InputValid=0;
 uint16_t Keys=Input!=NULL?(Input->Keys&~REMOTE_UNUSED_KEYS):0;
 // First recovered input establishes the key baseline; a key held during the
 // I2C outage must be released and freshly pressed before it can enable motion.
 if(!KeyStarted || (Status.InputValid && !WasValid)){KeyStarted=1;RawKeys=StableKeys=Keys;KeyTick=NowMs;}
 if(Keys!=RawKeys){RawKeys=Keys;KeyTick=NowMs;}
 uint16_t Rising=0;
 if((uint32_t)(NowMs-KeyTick)>=REMOTE_DEBOUNCE_MS){Rising=Keys&~StableKeys;StableKeys=Keys;}
 Status.Keys=StableKeys;
 // GPIO keys remain usable for inspection even if ADC calibration/conversion fails.
 if(Keys&KEY_MASK(KEY_L_DOWN))Lock(REMOTE_LOCK_STOP_KEY,NowMs); // Global STOP bypasses debounce.
 if(Status.LinkAck&&(uint32_t)(NowMs-Status.LastAckMs)>=REMOTE_LINK_STALE_MS){Status.LinkAck=0;Lock(REMOTE_LOCK_ACK_TIMEOUT,NowMs);}
 if(Status.Mode==REMOTE_RESUMING&&(uint32_t)(NowMs-ResumeTick)>=REMOTE_LINK_STALE_MS)Lock(REMOTE_LOCK_RESUME_TIMEOUT,NowMs);
 if(!Status.InputValid){Lock(REMOTE_LOCK_INPUT,NowMs);CenterStarted=0;return;}
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
 uint16_t OtherKeys=(uint16_t)~KEY_MASK(KEY_L_UP);
 uint8_t RecoveryIdle=Idle&&!(Keys&OtherKeys)&&!(StableKeys&OtherKeys);
 // A fresh press is required; a held key across connection recovery cannot unlock.
 if((Rising&KEY_MASK(KEY_L_UP))&&RecoveryIdle&&Status.LinkAck&&Status.Mode==REMOTE_LOCKED)
 {Status.Mode=REMOTE_RESUMING;ResumeTick=NowMs;AwaitingAck=1;}
 if(Status.Mode==REMOTE_RESUMING&&!RecoveryIdle)Lock(REMOTE_LOCK_RESUME_MOVED,NowMs);
 if(Status.Mode!=REMOTE_ACTIVE)return;
 Status.Vx=Status.Axis[REMOTE_LEFT_Y_INDEX]*REMOTE_SPEED_MPS;
 Status.Vy=-Status.Axis[REMOTE_LEFT_X_INDEX]*REMOTE_SPEED_MPS;
 // Cap diagonal translation to the selected speed, preserve requested direction.
 float norm=sqrtf(Status.Vx*Status.Vx+Status.Vy*Status.Vy);
 if(norm>REMOTE_SPEED_MPS){Status.Vx*=REMOTE_SPEED_MPS/norm;Status.Vy*=REMOTE_SPEED_MPS/norm;}
 int Turn=((StableKeys&KEY_MASK(KEY_L_LEFT))!=0)-((StableKeys&KEY_MASK(KEY_L_RIGHT))!=0);
 if(Turn==0)Status.Wz=0; // Release or opposing keys stops turning immediately.
 else
 {
  float Target=Turn*REMOTE_TURN_RAD_S;
  if(Status.Wz*Target<0)Status.Wz=0;
  float Delta=Target-Status.Wz,Limit=REMOTE_TURN_ACCEL_RAD_S2*Elapsed*.001f;
  if(Delta>Limit)Delta=Limit;
  if(Delta< -Limit)Delta=-Limit;
  Status.Wz+=Delta;
 }
 const Remote_Key SelectKeys[]={KEY_R_UP,KEY_R_DOWN,KEY_R_LEFT,KEY_R_RIGHT};
 uint16_t Selection=Rising&(KEY_MASK(KEY_R_UP)|KEY_MASK(KEY_R_DOWN)|KEY_MASK(KEY_R_LEFT)|KEY_MASK(KEY_R_RIGHT));
 if(Selection && !(Selection&(Selection-1U)))for(unsigned i=0;i<4;i++)
  if(Selection&KEY_MASK(SelectKeys[i])){Status.SelectedJoint=(uint8_t)i;JointCentered=0;}
 if(Status.Axis[REMOTE_RIGHT_Y_INDEX]==0)JointCentered=1;
 Status.ArmBase=Status.Axis[REMOTE_RIGHT_X_INDEX];
 Status.JointRate=JointCentered?Status.Axis[REMOTE_RIGHT_Y_INDEX]:0;
 // Single actions are latched as counters so a lost frame cannot lose a press.
 // Deploy requires the arm stick centered; gripper can operate during manual motion.
 // Simultaneous action keys are ignored.
 if(Status.Axis[REMOTE_RIGHT_X_INDEX]==0 && Status.Axis[REMOTE_RIGHT_Y_INDEX]==0)
 {
  if((Rising&KEY_MASK(KEY_DEPLOY)) && !(Keys&KEY_MASK(KEY_GRIPPER)))Status.DeployCounter++;
 }
 if((Rising&KEY_MASK(KEY_GRIPPER)) && !(Keys&KEY_MASK(KEY_DEPLOY)))Status.GripperCounter++;
}
void Controller_OnTx(uint8_t Flags,uint8_t Ack,uint32_t NowMs)
{
 if(!Ack){Status.LinkAck=0;Lock(REMOTE_LOCK_TX_FAILED,NowMs);return;}
 Status.LinkAck=1;Status.LastAckMs=NowMs;
 if(Flags==REMOTE_CONTROL_FLAG_RESUME&&AwaitingAck&&Status.Mode==REMOTE_RESUMING)
 {Status.Mode=REMOTE_ACTIVE;AwaitingAck=0;}
}
void Controller_Command(RemoteControl_FrameTypeDef *Frame,uint8_t Sequence)
{
 if(Frame==NULL)return;
 *Frame=(RemoteControl_FrameTypeDef){.Header=0xa5,.Version=REMOTE_CONTROL_VERSION_ROBOT,.Sequence=Sequence};
 Frame->Reserved[0]=Status.SelectedJoint;
 Frame->Reserved[3]=Status.DeployCounter;Frame->Reserved[4]=Status.GripperCounter;
 if(Status.Mode==REMOTE_LOCKED)Frame->Flags=REMOTE_CONTROL_FLAG_STOP;
 else if(Status.Mode==REMOTE_RESUMING)Frame->Flags=REMOTE_CONTROL_FLAG_RESUME;
 else {
  Frame->M1=(int16_t)lroundf(Status.Vx*1000);Frame->M2=(int16_t)lroundf(Status.Vy*1000);Frame->M3=(int16_t)lroundf(Status.Wz*1000);
  Frame->M4=(int16_t)lroundf(Status.ArmBase*1000);
  uint16_t Rate=(uint16_t)(int16_t)lroundf(Status.JointRate*1000);
  Frame->Reserved[1]=(uint8_t)Rate;Frame->Reserved[2]=(uint8_t)(Rate>>8);
 }
}
Remote_Status Controller_GetStatus(void){return Status;}
