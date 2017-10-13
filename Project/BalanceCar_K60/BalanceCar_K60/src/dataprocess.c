#include "dataprocess.h"
#include "speedcontrol.h"
#include "dircontrol.h"
#include "camera.h"
#include "adc.h"
#include "uart.h"
#include "ftm.h"
#include "irq.h"

///////////////////////////////////////////////ASin///////////////////////////////////////////////////////////
const float  Asin_to_Angle[201] = {
-90.000000,-81.890386,-78.521659,-75.930132,-73.739795,-71.805128,-70.051556,-68.434815,-66.926082,-65.505352,
-64.158067,-62.873247,-61.642363,-60.458639,-59.316583,-58.211669,-57.140120,-56.098738,-55.084794,-54.095931,
-53.130102,-52.185511,-51.260575,-50.353889,-49.464198,-48.590378,-47.731416,-46.886394,-46.054480,-45.234915,
-44.427004,-43.630109,-42.843643,-42.067065,-41.299873,-40.541602,-39.791819,-39.050123,-38.316134,-37.589503,
-36.869898,-36.157008,-35.450543,-34.750226,-34.055798,-33.367013,-32.683639,-32.005455,-31.332251,-30.663830,
-30.000000,-29.340582,-28.685402,-28.034297,-27.387108,-26.743684,-26.103881,-25.467560,-24.834587,-24.204835,
-23.578178,-22.954499,-22.333683,-21.715617,-21.100196,-20.487315,-19.876874,-19.268775,-18.662925,-18.059230,
-17.457603,-16.857956,-16.260205,-15.664267,-15.070062,-14.477512,-13.886540,-13.297072,-12.709033,-12.122352,
-11.536959,-10.952784,-10.369760, -9.787819, -9.206896, -8.626927, -8.047846, -7.469592, -6.892103, -6.315316,
 -5.739170, -5.163607, -4.588566, -4.013987, -3.439813, -2.865984, -2.292443, -1.719131, -1.145992, -0.572967,
  0.000000,  0.572967,  1.145992,  1.719131,  2.292443,  2.865984,  3.439813,  4.013987,  4.588566,  5.163607,
  5.739170,  6.315316,  6.892103,  7.469592,  8.047846,  8.626927,  9.206896,  9.787819, 10.369760, 10.952784,
 11.536959, 12.122352, 12.709033, 13.297072, 13.886540, 14.477512, 15.070062, 15.664267, 16.260205, 16.857956,
 17.457603, 18.059230, 18.662925, 19.268775, 19.876874, 20.487315, 21.100196, 21.715617, 22.333683, 22.954499,
 23.578178, 24.204835, 24.834587, 25.467560, 26.103881, 26.743684, 27.387108, 28.034297, 28.685402, 29.340582,
 30.000000, 30.663830, 31.332251, 32.005455, 32.683639, 33.367013, 34.055798, 34.750226, 35.450543, 36.157008,
 36.869898, 37.589503, 38.316134, 39.050123, 39.791819, 40.541602, 41.299873, 42.067065, 42.843643, 43.630109,
 44.427004, 45.234915, 46.054480, 46.886394, 47.731416, 48.590378, 49.464198, 50.353889, 51.260575, 52.185511,
 53.130102, 54.095931, 55.084794, 56.098738, 57.140120, 58.211669, 59.316583, 60.458639, 61.642363, 62.873247,
 64.158067, 65.505352, 66.926082, 68.434815, 70.051556, 71.805128, 73.739795, 75.930132, 78.521659, 81.890386,
 90.000000 };
///////////////////////////////////////////////ASin///////////////////////////////////////////////////////////

#ifdef CARDEBUG
bool SendData = 0;
#endif // CARDEBUG 

word us_ADGravity[3] = { 0 };
word us_ADGyroscope[3] = { 0 };
word us_GyroscopeVoltage = 0;
word us_GravityVoltage = 0;
volatile float f_Gravity_Offset = 1850;    //offset
volatile float f_Gryoscope_Offset = 980;   //offset
volatile float f_AngleControl_P = 1400;
volatile float f_AngleControl_D = 6;
volatile float LEFT_MOTOR_OUT_DEAD_VAL = 10;
volatile float RIGHT_MOTOR_OUT_DEAD_VAL = 0;
float f_AngleSpeed = 0;   //实际角速度
float f_CarAngle = 0;     //实际角度
volatile float f_AngleZero = -1.0;
volatile float f_Gyro_m = 0;
volatile float f_Angle_m = 0;
float f_LeftMotorOut = 0;
float f_RightMotorOut = 0;
float f_LeftDuty;
float f_RightDuty;
byte b_RunningFlag = 1;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
volatile float Angle, Angle_dot;
const float Q_angle = 0.001, Q_gyro = 0.003, R_angle = 0.67, dt = 0.005;
volatile float P[2][2] = { { 1, 0 },{ 0, 1 } };
volatile char  C_0 = 1;
volatile float E;
volatile float q_bias = 0;
volatile float Angle_err;
volatile float PCt_0, PCt_1;
volatile float K_0, K_1;
volatile float t_0, t_1;
volatile float Pdot[4] = { 0,0,0,0 };
///////////////////////////////////////////////AngleControl///////////////////////////////////////////////////

void GetInputVoltage()
{
	us_GravityVoltage = ADC_QuickReadValue(ADC0_SE13_PB3);
	us_GyroscopeVoltage = ADC_QuickReadValue(ADC0_SE8_PB0);
}

void Kalman_Filter(float angle_m, float gyro_m)			//gyro_m:gyro_measure
{
	Angle += (gyro_m - q_bias) * dt;

	Pdot[0] = Q_angle - P[0][1] - P[1][0];
	Pdot[1] = -P[1][1];
	Pdot[2] = -P[1][1];
	Pdot[3] = Q_gyro;

	P[0][0] += Pdot[0] * dt;
	P[0][1] += Pdot[1] * dt;
	P[1][0] += Pdot[2] * dt;
	P[1][1] += Pdot[3] * dt;

	Angle_err = angle_m - Angle;

	PCt_0 = C_0 * P[0][0];
	PCt_1 = C_0 * P[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * P[0][1];

	P[0][0] -= K_0 * t_0;
	P[0][1] -= K_0 * t_1;
	P[1][0] -= K_1 * t_0;
	P[1][1] -= K_1 * t_1;

	Angle += K_0 * Angle_err;
	q_bias += K_1 * Angle_err;
	Angle_dot = gyro_m - q_bias;
}

void AngleCalculate()
{
	long lnDeltaValue;
	lnDeltaValue = (int)us_GravityVoltage;
	lnDeltaValue = lnDeltaValue - (int)(f_Gravity_Offset);
	lnDeltaValue = (long)(lnDeltaValue*0.1242);
	if (lnDeltaValue > 100)    lnDeltaValue = 100;
	if (lnDeltaValue < -100)    lnDeltaValue = -100;
	f_Angle_m = Asin_to_Angle[lnDeltaValue + 100];

	lnDeltaValue = (int)us_GyroscopeVoltage;
	lnDeltaValue = lnDeltaValue - (int)f_Gryoscope_Offset;
	lnDeltaValue = (long)(lnDeltaValue*0.2357);
	f_Gyro_m = (float)(lnDeltaValue);

	Kalman_Filter(f_Angle_m, f_Gyro_m);

	f_CarAngle = Angle;
	f_AngleSpeed = Angle_dot;
}

void AngleControl()
{
	float nSpeed, nP, nD;
	GetInputVoltage();
	AngleCalculate();
	nP = f_CarAngle - f_AngleZero;
	nP *= f_AngleControl_P;
	nD = f_AngleSpeed;
	nD *= f_AngleControl_D;
	nSpeed = nP + nD;
	f_LeftMotorOut = nSpeed;
	f_RightMotorOut = nSpeed;

	MotorControl();
}
///////////////////////////////////////////////AngleControl///////////////////////////////////////////////////

///////////////////////////////////////////////MotorControl///////////////////////////////////////////////////
volatile float K_Plus = 1;
volatile float K_Sub = 1;

void MotorControl()
{
	float nLeftVal, nRightVal;

	float nLeftVal_sum = 0, nRightVal_sum = 0;
	nLeftVal_sum = f_LeftMotorOut - f_SpeedControlOut;
	nRightVal_sum = f_RightMotorOut - f_SpeedControlOut;


	nLeftVal = nLeftVal_sum - f_DirControlOut*K_Sub;
	nRightVal = nRightVal_sum + f_DirControlOut*K_Plus;


	if (nLeftVal >= 0)
		nLeftVal -= LEFT_MOTOR_OUT_DEAD_VAL;
	else
		nLeftVal += LEFT_MOTOR_OUT_DEAD_VAL;


	if (nRightVal >= 0)
		nRightVal -= RIGHT_MOTOR_OUT_DEAD_VAL;
	else
		nRightVal += RIGHT_MOTOR_OUT_DEAD_VAL;


	if (nLeftVal > MotorMax)
	{
		nLeftVal = MotorMax;
	}
	if (nLeftVal < MotorMin)
	{
		nLeftVal = MotorMin;
	}
	if (nRightVal > MotorMax)
	{
		nRightVal = MotorMax;
	}
	if (nRightVal < MotorMin)
	{
		nRightVal = MotorMin;
	}
	f_LeftDuty = nLeftVal;
	f_RightDuty = nRightVal;

	MotorControlOut();
}

void MotorControlOut()
{
	int R_Duty, L_Duty;
	if (b_SpeedProtect >= 1)
	{
		//		b_RunningFlag=0;
	}
	if (b_RunningFlag == 1)
	{
		L_Duty = 5000 + f_LeftDuty;
		R_Duty = 5000 + f_RightDuty;
		if (L_Duty > 10000)
		{
			L_Duty = 10000;
		}
		if (L_Duty < 10)
		{
			L_Duty = 10;
		}
		if (R_Duty > 10000)
		{
			R_Duty = 10000;
		}
		if (R_Duty < 10)
		{
			R_Duty = 10;
		}
		//		if (f_CarAngle > 25 || f_CarAngle < -40)
		//		{
		//			L_Duty = 5000;
		//			R_Duty = 5000;
		//		}
		FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH0 | HW_FTM_CH1, L_Duty);
		FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH2 | HW_FTM_CH3, R_Duty);
	}
	else
	{
		FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH0 | HW_FTM_CH1, 5000);
		FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH2 | HW_FTM_CH3, 5000);
	}
}

void ReceiveDataProcess(unsigned char * s)
{
	float temp;
	temp = (s[1] - 48) * 1000 + (s[2] - 48) * 100 + (s[3] - 48) * 10 + (s[4] - 48);
	switch (s[0])
	{
	case 'A':LEFT_MOTOR_OUT_DEAD_VAL = temp; break;
	case 'B':RIGHT_MOTOR_OUT_DEAD_VAL = temp; break;
	case 'C':f_AngleZero = ((s[1] == '-') ? (-1) : 1) * ((s[2] - 48) + (s[4] - 48)*0.1); break;
	case 'D':f_CarSpeedSet = ((s[1] == '-') ? (-1) : 1) * ((s[2] - 48) * 100 + (s[3] - 48) * 10 + (s[4] - 48)); break;
	case 'E':f_Gravity_Offset = temp; break;
	case 'F':f_AngleControl_P = temp; break;
	case 'G':f_AngleControl_D = temp; break;
	case 'H':f_SpeedControl_P = temp; break;
	case 'I':f_SpeedControl_I = temp; break;
	case 'J':Error = ((s[1] == '-') ? (-1) : 1) * ((s[2] - 48) * 100 + (s[3] - 48) * 10 + (s[4] - 48)); break;
	case 'K':f_Gryoscope_Offset = temp; break;
	case 'L':K_Plus = temp; break;
	case 'M':K_Sub = temp; break;
	case 'N':CAMVALUE1.U_Error = temp; break;
	case 'O':CAMVALUE1.Ultrasonic_Threshold = temp; break;
	}
}

void SendDataProcess(void)
{
	float f_Send_Data[20] = { 0 };
	char c_Send_Char[4];
	char * c_Send_Point;
	byte j; byte i; char t1 = 'a';
	c_Send_Point = &c_Send_Char[0];
	f_Send_Data[1] = i_LeftMotorPulse;
	f_Send_Data[2] = i_RightMotorPulse;
	f_Send_Data[3] = YD-YU;
	f_Send_Data[4] = us_DirGyroscopeVoltage;
	f_Send_Data[5] = us_GravityVoltage;
	f_Send_Data[6] = us_GyroscopeVoltage;
	f_Send_Data[7] = f_CarAngle;
	f_Send_Data[8] = f_CarSpeed;
	f_Send_Data[9] = f_AngleControl_P;
	f_Send_Data[10] = f_AngleControl_D;
	f_Send_Data[11] = f_SpeedControl_P;
	f_Send_Data[12] = f_SpeedControl_I;
	f_Send_Data[13] = Error;//Error;
	f_Send_Data[14] = f_LeftDuty;//LeftDuty
	f_Send_Data[15] = f_RightDuty;//RightDuty
	f_Send_Data[16] = LEFT_MOTOR_OUT_DEAD_VAL;
	f_Send_Data[17] = RIGHT_MOTOR_OUT_DEAD_VAL;
	f_Send_Data[18] = Distance;
	UART_WriteByte(HW_UART0, '@');
	for (i = 1; i < 19; i++)
	{
		sprintf(c_Send_Point, "%f", f_Send_Data[i]);
		UART_WriteByte(HW_UART0, t1);
		for (j = 0; j < 4; j++)
		{
			UART_WriteByte(HW_UART0, c_Send_Char[j]);
		}
		t1++;
	}
	UART_WriteByte(HW_UART0, '!');
}

void SerialDispCCDImage(int xSize, int ySize, uint8_t** ppData)
{
	int x, y;

	for (y = 0; y < ySize; y++)
	{
		for (x = 1; x < (xSize / 8) + 1; x++)
		{
			if (ppData[y][x] == '\r')
			{
				ppData[y][x]--;
			}
			UART_WriteByte(HW_UART0, ppData[y][x]);
		}
	}
	UART_WriteByte(HW_UART0, '\r');
}
