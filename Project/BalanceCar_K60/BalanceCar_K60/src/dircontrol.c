#include "speedcontrol.h"
#include "dircontrol.h"
#include "adc.h"
#include "camera.h"
#include "gpio.h"
#include "irq.h"

word us_DirGyroscopeVoltage = 0;
float f_DirGyroscope_Offset = 2620; //
float f_DirAngleSpeed = 0;

float f_DirControlOutPrior = 0;
float f_DirControlOutNext = 0;
float f_DirControlOut = 0;
volatile float Kp_dir = 80;     //2.339
volatile float Kd_dir = -5;
float Ks_dir = 60;
//float In_kp = 0;
//float K_p = 0;


volatile float Error = 0;
float Prior_Error = 0;
float PPrior_Error = 0;
float Last_Error = 0;

float MidValue = 0;
byte YU, YD;
byte LastYDiff = 0;
bool SingalFound = false;
bool Obstacle = false;
bool FinishOne = false;

byte LeftSingal = 0;
byte RightSingal = 0;

byte  b_DirectionControlPeriod = 0;
byte  b_DirectionControlCount = 0;

byte nofounddelay = 0;
volatile float CharValue[5] = { -1,-1,-1,-1,-1 };//1 1 -1 -1 1
volatile int UCharValue[5] = { -1,1,-1,-1,-1 };
volatile byte LightCount = 0;



struct CAMVALUE CAMVALUESET[] =
{
	{ 80,15,24,25,25,25,5},//	Ultrasonic_Threshold;U_Error;ErrorNoFound;Speed1;Speed2;Speed3;Offset;
	{ 900,11,25,25,30,30,5 },
};

struct CAMVALUE CAMVALUE1 = { 750,13,22,10,20,30,6 };

byte CAMSELECT = 0;

void DirControl(void)
{
	float fValue = 0;
	float lnDeltaValue = 0;
	lnDeltaValue = (float)us_DirGyroscopeVoltage;
	lnDeltaValue = f_DirGyroscope_Offset - lnDeltaValue;

	f_DirAngleSpeed = (lnDeltaValue*0.2357);

	//In_kp=Kp_dir-K_p*SpeedError;
	fValue = Kp_dir*Error + Kd_dir*f_DirAngleSpeed + Ks_dir*(Error - Prior_Error);
	PPrior_Error = Error - Prior_Error;

	Prior_Error = Error;

	if (fValue >= DIR_OUT_MAX)
		fValue = DIR_OUT_MAX;
	if (fValue <= DIR_OUT_MIN)
		fValue = DIR_OUT_MIN;

	f_DirControlOutPrior = f_DirControlOutNext;
	f_DirControlOutNext = fValue;
}

void DirControlOutput(void)
{
	float fValue;

	fValue = f_DirControlOutNext - f_DirControlOutPrior;

	f_DirControlOut = fValue*(b_DirectionControlPeriod + 1) / DIR_CONTROL_PERIOD + f_DirControlOutPrior;
}

void GetDirGyroscopeVoltage(void)
{
	us_DirGyroscopeVoltage = ADC_QuickReadValue(ADC0_SE9_PB1);
}

void SingalSerach(int xSize, int ySize, uint8_t** ppData)
{
	byte x, y;
	LeftSingal = 0;
	RightSingal = 0;
	YU = 0, YD = 0;
	SingalFound = false;
	Error = 0;
	byte i = 0;
	for (y = 0; y < ySize - 1; y++)
	{
		for (x = 1; x < (xSize / 8) + 1; x++)
		{
			//more than 4 black && next 2 ROW more than 4 black && next 4 ROW more than 4 black && LeftSingal not FOUND
			if (ppData[y][x] < 0xF0 && ppData[y + 2][x] < 0xF0 && ppData[y + 4][x] < 0xF0 && (LeftSingal == 0))
			{
				YU = y;
				LeftSingal = x;
				i = 0;
				while ((x + i < ((xSize / 8) + 1)) && (ppData[y][x + i] < 0xF0))
				{
					i++;
				}
				RightSingal = x + i;
				MidValue = (RightSingal + LeftSingal) / 2;
				SingalFound = true;
				break;
			}
		}
		if (SingalFound)
		{
			Error = MidValue - (OV7725_W / 8 + 1) / 2 + CAMVALUE1.Offset*UCharValue[LightCount];
			Last_Error = Error;
			nofounddelay = 2;
			GPIO_ResetBit(HW_GPIOA, 16);
			while (ppData[y][(byte)MidValue] < 0xF0 && (y < ySize - 1))
			{
				YD = y;
				y++;
			}
			LastYDiff = YD - YU;
			while (f_CarSpeedSet < CAMVALUE1.Speed3)
			{
				f_CarSpeedSet += 5;
			}
			break;
		}
	}
	if (!SingalFound)
	{
		while (f_CarSpeedSet > CAMVALUE1.Speed1)
		{
			f_CarSpeedSet -= 5;
		}

		if (nofounddelay)
		{
			SingalFound = true;
			Error = Last_Error / 2;
			nofounddelay--;
		}
		else
		{
			Error = CAMVALUE1.ErrorNoFound*CharValue[LightCount];
			GPIO_SetBit(HW_GPIOA, 16);
		}
	}
	//Obstacle
	if (Distance <= CAMVALUE1.Ultrasonic_Threshold && Distance >= 5)
	{
		Obstacle = true;
		GPIO_ResetBit(HW_GPIOA, 15);
	}
	else
	{
		Obstacle = false;
		GPIO_SetBit(HW_GPIOA, 15);
	}


	if (SingalFound&&Obstacle)
	{
		Error += CAMVALUE1.U_Error*UCharValue[LightCount];
		if (LastYDiff >= 80)
			FinishOne = true;
	}

	//	if(LastYDiff>=80&&LastYDiff<=110&&!SingalFound)
	//	{
	//		FinishOne = true;
	//	}

	if (FinishOne && !FinishOneCount)
	{
		FinishOneCount++;
	}

	if (FinishOneCount > 600 && SingalFound)
	{
		FinishOneCount = 0;
		FinishOne = false;
		LastYDiff = 0;
	}

	if (FinishOne)
	{
		f_CarSpeedSet = CAMVALUE1.Speed2;
	}


	if (FinishOne && !SingalFound)
	{
		if (LightCount >= 3)
		{
			LightCount = 0;
		}
		else
		{
			LightCount++;
		}
		FinishOne = false;
		FinishOneCount = 0;
		Obstacle = false;
		LastYDiff = 0;
	}
	//JAM
//	if (JAMFLAG)
//	{
//		GPIO_ResetBit(HW_GPIOA, 17);
//		b_JAM--;
//		f_CarSpeedSet = -CAMVALUE1.Speed2;
//		Error = -Error;
//		if (b_JAM == 0)
//		{
//			JAMFLAG = false;
//		}
//	}
//	else
//	{
//		GPIO_SetBit(HW_GPIOA, 17);
//	}
}
