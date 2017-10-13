#include "common.h"

#define SPEED_CONTROL_IOUT_MAX 5000
#define SPEED_CONTROL_IOUT_MIN -5000
#define SPEED_CONTROL_PERIOD 100
#define CAR_SPEED_CONSTANT 0.390625

extern int i_LeftMotorPulse;
extern int i_RightMotorPulse;


extern byte b_Left_Dir;
extern byte b_Right_Dir;
extern byte b_SpeedProtect;

extern float Ia;
extern float f_SpeedControl_P;
extern float f_SpeedControl_I;
extern float f_CarSpeed;
extern float f_SpeedControlOut;
extern float f_SpeedControlOutNext;
extern float f_SpeedControlOutPrior;
extern float f_SpeedError;
extern float f_CarSpeedSet;
extern float f_SpeedSet_Init;
extern float f_SpeedSet_Next;
extern byte b_SpeedPulseCount;
extern byte b_SpeedControlCount;
extern byte b_SpeedControlPeriod;
extern byte b_RunningFlag;
extern byte b_JAM;
extern bool JAMFLAG;

void GetMotorPulse(void);
void SpeedControl(void);
void SpeedControlOutput(void);
