#include "speedcontrol.h"
#include "ftm.h"
#include "gpio.h"

byte b_Left_Dir = 0;			//  direction  //1 forward 0 back	
byte b_Right_Dir = 0; 		//0 forward //1 back
float f_SpeedControl_P = 960;
float f_SpeedControl_I = 20;//16;
int i_LeftMotorPulse = 0;
int i_RightMotorPulse = 0;
float f_CarSpeed = 0;
float f_SpeedSet_Init = 0;
float f_SpeedSet_Next = 0;
byte b_SpeedProtect = 0;
float f_SpeedControlOut = 0;
float f_SpeedControlOutNext = 0;
float f_SpeedControlOutPrior = 0;
float f_SpeedError;
float f_CarSpeedSet = 0; //³µËÙ¿ØÖÆ
float Ia;
byte b_SpeedPulseCount = 0;
byte b_SpeedControlCount = 0;
byte b_SpeedControlPeriod = 0;

byte b_JAM = 0;
bool JAMFLAG = false;

void GetMotorPulse()
{
	int Left_pluse;
	int Right_pluse;

	FTM_QD_GetData(HW_FTM2, &Right_pluse, &b_Right_Dir);
	FTM_QD_GetData(HW_FTM1, &Left_pluse, &b_Left_Dir);
	FTM_QD_ClearCount(HW_FTM1);
	FTM_QD_ClearCount(HW_FTM2);

	if (Left_pluse > 20000)
	{
		Left_pluse = 65535 - Left_pluse;
	}
	if (Right_pluse > 20000)
	{
		Right_pluse = 65535 - Right_pluse;
	}

	i_LeftMotorPulse = Left_pluse;
	i_RightMotorPulse = Right_pluse;
}

void SpeedControl()
{
	float fP;
	float fI;
	static float fSpeedControlIntegral = 0;
	if (b_Left_Dir == 0)
	{
		i_LeftMotorPulse *= (-1);
	}
	if (b_Right_Dir >= 1)
	{
		i_RightMotorPulse *= (-1);
	}
	f_CarSpeed = (i_LeftMotorPulse + i_RightMotorPulse) / 40.0;
	f_CarSpeed *= CAR_SPEED_CONSTANT;
	if ((f_CarSpeed > 45 || f_CarSpeed < -20))
	{
		b_SpeedProtect++;
		if (b_SpeedProtect >= 5)
		{
			b_RunningFlag = 0;
		}
	}
	else
		b_SpeedProtect = 0;
	if (f_CarSpeed <= 3 && f_CarSpeed >= 0 && !JAMFLAG)
	{
		b_JAM++;
	}
	if (b_JAM > 0 && f_CarSpeed > 3 && !JAMFLAG)
	{
		b_JAM--;
	}
	if (b_JAM >= 18 && !JAMFLAG)
	{
		b_JAM = 100;
		JAMFLAG = true;
	}

	f_SpeedError = f_CarSpeedSet;
	f_SpeedError -= f_CarSpeed;
	fP = f_SpeedError*f_SpeedControl_P;
	fI = f_SpeedError*f_SpeedControl_I;
	fSpeedControlIntegral += fI;
	Ia = fSpeedControlIntegral;
	if (fSpeedControlIntegral >= SPEED_CONTROL_IOUT_MAX)
		fSpeedControlIntegral = SPEED_CONTROL_IOUT_MAX;
	if (fSpeedControlIntegral <= SPEED_CONTROL_IOUT_MIN)
		fSpeedControlIntegral = SPEED_CONTROL_IOUT_MIN;

	f_SpeedControlOutPrior = f_SpeedControlOutNext;
	f_SpeedControlOutNext = fP + fSpeedControlIntegral;

}

void SpeedControlOutput(void)
{
	float fValue;
	fValue = f_SpeedControlOutNext - f_SpeedControlOutPrior;
	f_SpeedControlOut = fValue * (b_SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + f_SpeedControlOutPrior;
	f_SpeedControlOut *= 0.7;
}
