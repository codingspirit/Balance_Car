#include "irq.h"
#include "speedcontrol.h"
#include "dma.h"
#include "camera.h"
#include "gpio.h"
#include "dircontrol.h"
#include "pit.h"

byte UART_i = 0;
unsigned char uc_ReceiveData[5] = { 0 };
bool b_UARTFound = 0;
word w_Delay_Start = 0;
word V_Cnt = 0;
byte b_EventCount = 0;
volatile float Distance = 2500;
int b_DistanceCount = 0;
word FinishOneCount=0;
bool UltrasonicFlag = false;

void UART_RX_Handler(uint16_t byteReceived)
{
	if ((byteReceived >= 0x41) && (byteReceived <= 0x4f))
	{
		b_UARTFound = 1;
		uc_ReceiveData[0] = byteReceived;
	}
	if (b_UARTFound)
	{
		switch (UART_i)
		{
		case 0:UART_i++; break;
		default:uc_ReceiveData[UART_i] = byteReceived; UART_i++; break;
		}
		if (UART_i >= 5)
		{
			ReceiveDataProcess(uc_ReceiveData);
			UART_i = 0;
			b_UARTFound = 0;
		}
	}
}
void PIT0_Handler(void)
{
	if (w_Delay_Start)
	{
		w_Delay_Start--;
	}
	b_EventCount++;
	if(FinishOneCount>=1)
	{
		FinishOneCount++;
	}
	SpeedControlOutput();
	b_SpeedControlPeriod++;
	b_DirectionControlPeriod++;
	DirControlOutput();
	if (b_EventCount == 1)
	{
		GetInputVoltage();
	}
	if (b_EventCount == 2)
	{
		AngleControl();
	}
	if (b_EventCount == 3)
	{

		b_SpeedPulseCount++;
		if (b_SpeedPulseCount >= 20)      //20*5=100ms
		{
			b_SpeedPulseCount = 0;
			GetMotorPulse();
		}
	}
	if (b_EventCount == 4)
	{
		b_SpeedControlCount++;
		if (b_SpeedControlCount >= 20)
		{
			SpeedControl();
			b_SpeedControlCount = 0;
			b_SpeedControlPeriod = 0;
		}
	}
	if (b_EventCount >= 5)
	{
		b_EventCount = 0;
		b_DirectionControlCount++;
		if (b_DirectionControlCount >= 2)   //3*5=15ms
		{
			//UART_WriteByte(HW_UART2, 0x55);   //Ultrasonic Send
			//GPIO_ToggleBit(HW_GPIOD, 3);
			if(!UltrasonicFlag)
			{
				GPIO_SetBit(HW_GPIOD, 3);
			}
			GetDirGyroscopeVoltage();
			DirControl();
			b_DirectionControlPeriod = 0;
			b_DirectionControlCount = 0;
			GPIO_ResetBit(HW_GPIOD, 3);
		}
	}
}
void Camera_Handler(uint32_t index)
{
	static uint8_t status = TRANSFER_IN_PROCESS;
	static uint32_t h_counter, v_counter;
	// uint32_t i;

	 /* HERF */
	if (index & (1 << BOARD_OV7725_HREF_PIN))
	{
		DMA_SetDestAddress(HW_DMA_CH2, (uint32_t)gpHREF[h_counter++]);
		DMA_SetMajorLoopCounter(HW_DMA_CH2, (OV7725_W / 8) + 1);
		DMA_EnableRequest(HW_DMA_CH2);

		return;
	}
	/* VERF */
	if (index & (1 << BOARD_OV7725_VSYNC_PIN))
	{
		GPIO_ITDMAConfig(BOARD_OV7725_VSYNC_PORT, BOARD_OV7725_VSYNC_PIN, kGPIO_IT_FallingEdge, false);
		GPIO_ITDMAConfig(BOARD_OV7725_HREF_PORT, BOARD_OV7725_HREF_PIN, kGPIO_IT_FallingEdge, false);
		switch (status)
		{
		case TRANSFER_IN_PROCESS:
			UserApp(v_counter++);
			status = NEXT_FRAME;
			h_counter = 0;

			break;
		case NEXT_FRAME: // waiting for next transfer 
			status = TRANSFER_IN_PROCESS;
			break;
		default:
			break;
		}
		GPIO_ITDMAConfig(BOARD_OV7725_VSYNC_PORT, BOARD_OV7725_VSYNC_PIN, kGPIO_IT_FallingEdge, true);
		GPIO_ITDMAConfig(BOARD_OV7725_HREF_PORT, BOARD_OV7725_HREF_PIN, kGPIO_IT_FallingEdge, true);
		PORTC->ISFR = 0xFFFFFFFF;
		h_counter = 0;
		return;
	}
}

//void Ultrasonic_RX_Handler(uint16_t byteReceived)
//{
//	if (b_DistanceCount)
//	{
//		b_DistanceCount = 0;
//		Distance += byteReceived;
//	}
//	else
//	{
//		b_DistanceCount++;
//		Distance = byteReceived * 256;
//	}
//}
void Ultrasonic_RX_Handler(uint32_t index)
{
	if (index & (1 << 2))
	{
		if (!GPIO_ReadBit(HW_GPIOD, 2))
		{
			UltrasonicFlag = false;
			PIT_ITDMAConfig(HW_PIT_CH1, kPIT_IT_TOF, DISABLE);
			Distance = b_DistanceCount*1.7;
		}
		else if(GPIO_ReadBit(HW_GPIOD, 2))
		{
			UltrasonicFlag = true;
			b_DistanceCount = 0;
			PIT_ITDMAConfig(HW_PIT_CH1, kPIT_IT_TOF, ENABLE);
		}
	}
}
void PIT1_Handler()
{
	if (UltrasonicFlag)
	{
			b_DistanceCount++;
	}
}
