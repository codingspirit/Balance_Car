#include "common.h"

#define DIR_OUT_MAX      2500
#define DIR_OUT_MIN      -2500

#define DIR_CONTROL_PERIOD 10


typedef struct CAMVALUE
{
	int Ultrasonic_Threshold;
	float U_Error;
	float ErrorNoFound;
	float Speed1;
	float Speed2;
	float Speed3;
	float Offset;
}CAMVALUE;

extern float f_DirControlOut;
extern volatile float Error;
extern volatile float Kp_dir;
extern volatile float Kd_dir;
extern float Ks_dir;
extern float K_p;
extern word us_DirGyroscopeVoltage;
extern float f_DirGyroscope_Offset;
extern float f_DirAngleSpeed;
extern float Prior_Error;
extern float PPrior_Error;
extern struct CAMVALUE CAMVALUESET[];
extern struct CAMVALUE CAMVALUE1;
extern byte CAMSELECT;
extern volatile float CharValue[5];
extern volatile int UCharValue[5];


extern float MidValue;
extern byte YU,YD;
extern bool SingalFound;
extern byte LeftSingal;
extern byte RightSingal;

extern byte  b_DirectionControlPeriod;
extern byte  b_DirectionControlCount;

void DirControl(void);
void DirControlOutput(void);
void GetDirGyroscopeVoltage(void);
void SingalSerach(int xSize, int ySize, uint8_t** ppData);
