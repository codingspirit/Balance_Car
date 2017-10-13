#include "common.h"
///* DSP lib */
//#define ARM_MATH_CM4
//#include "arm_math.h"
///* DSP lib */
#define MotorMax 5000
#define MotorMin -5000

#define CARDEBUG
#define WATCHDOG

#ifdef CARDEBUG
extern bool SendData;
#endif // CARDEBUG


extern  volatile float LEFT_MOTOR_OUT_DEAD_VAL;
extern  volatile float RIGHT_MOTOR_OUT_DEAD_VAL;
extern word us_GravityVoltage;     //average
extern word us_GyroscopeVoltage;
extern volatile float f_AngleControl_P;
extern volatile float f_AngleControl_D;
extern float f_AngleSpeed;   //
extern float f_CarAngle;   //
extern  float f_LeftMotorOut;
extern float f_RightMotorOut;
extern float f_LeftDuty;
extern float f_RightDuty;
extern byte b_RunningFlag;
extern volatile float K_Plus;
extern volatile float K_Sub;
void GetInputVoltage(void);
void Kalman_Filter(float angle_m, float gyro_m);
void AngleCalculate(void);
void AngleControl(void);
void MotorControl(void);
void MotorControlOut(void);
void ReceiveDataProcess(unsigned char * s);
void SendDataProcess(void);
void SerialDispCCDImage(int xSize, int ySize, uint8_t** ppData);
