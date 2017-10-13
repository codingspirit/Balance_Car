#include "dataprocess.h"
#include "uart.h"
extern unsigned char uc_ReceiveData[5];
extern word V_Cnt;
extern volatile float Distance;
extern word w_Delay_Start;
extern word FinishOneCount;
void UART_RX_Handler(uint16_t byteReceived);
void PIT0_Handler(void);
void Camera_Handler(uint32_t index);
//void Ultrasonic_RX_Handler(uint16_t byteReceived);
void Ultrasonic_RX_Handler(uint32_t index);
void PIT1_Handler(void);
