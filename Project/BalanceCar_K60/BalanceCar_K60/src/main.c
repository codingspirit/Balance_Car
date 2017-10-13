#include "gpio.h"
#include "common.h"
#include "irq.h"
#include "adc.h"
#include "pit.h"
#include "ftm.h"
#include "camera.h"
#include "dircontrol.h"
#include "speedcontrol.h"
#include "wdog.h"

/* CH Kinetis固件库 V2.50 版本 */
/* 修改主频 请使用 CMSIS标准文件 system_MKxxxx.c 中的 CLOCK_SETUP 宏 */

/* UART 快速初始化结构所支持的引脚* 使用时还是推荐标准初始化 */
/*
 UART1_RX_PE01_TX_PE00
 UART0_RX_PF17_TX_PF18
 UART3_RX_PE05_TX_PE04
 UART5_RX_PF19_TX_PF20
 UART5_RX_PE09_TX_PE08
 UART2_RX_PE17_TX_PE16
 UART4_RX_PE25_TX_PE24
 UART0_RX_PA01_TX_PA02
 UART0_RX_PA15_TX_PA14
 UART3_RX_PB10_TX_PB11
 UART0_RX_PB16_TX_PB17
 UART1_RX_PC03_TX_PC04
 UART4_RX_PC14_TX_PC15
 UART3_RX_PC16_TX_PC17
 UART2_RX_PD02_TX_PD03
 UART0_RX_PD06_TX_PD07
 UART2_RX_PF13_TX_PF14
 UART5_RX_PD08_TX_PD09
*/


int main(void)
{
	DelayInit();

	w_Delay_Start = 3000;

	/* 初始化一个引脚 设置为推挽输出 */
	GPIO_QuickInit(HW_GPIOA, 14, kGPIO_Mode_OPP);
	GPIO_QuickInit(HW_GPIOA, 15, kGPIO_Mode_OPP);
	GPIO_QuickInit(HW_GPIOA, 16, kGPIO_Mode_OPP);
	GPIO_QuickInit(HW_GPIOA, 17, kGPIO_Mode_OPP);
	GPIO_SetBit(HW_GPIOA, 14);
	GPIO_SetBit(HW_GPIOA, 15);
	GPIO_SetBit(HW_GPIOA, 16);
	GPIO_SetBit(HW_GPIOA, 17);

	/* Switch Init */
	GPIO_QuickInit(HW_GPIOB, 7, kGPIO_Mode_IPU);
	GPIO_QuickInit(HW_GPIOB, 6, kGPIO_Mode_IPU);
	GPIO_QuickInit(HW_GPIOB, 5, kGPIO_Mode_IPU);
	GPIO_QuickInit(HW_GPIOB, 4, kGPIO_Mode_IPU);
	GPIO_QuickInit(HW_GPIOE, 25, kGPIO_Mode_IPU);
	GPIO_QuickInit(HW_GPIOE, 26, kGPIO_Mode_IPU);
	GPIO_QuickInit(HW_GPIOE, 27, kGPIO_Mode_IPU);
	GPIO_QuickInit(HW_GPIOE, 28, kGPIO_Mode_IPU);

	/* Watch Dog Init */
#ifdef WATCHDOG
	if (GPIO_ReadBit(HW_GPIOB, 4))
	{
		WDOG_InitTypeDef WDOG_InitStruct1;
		WDOG_InitStruct1.windowInMs = 0;
		WDOG_InitStruct1.mode = kWDOG_Mode_Normal;  //设置看门狗处于正常工作模式
		WDOG_InitStruct1.timeOutInMs = 400; // 时限 200MS
		WDOG_Init(&WDOG_InitStruct1);
	}
#endif // WATCHDOG

	/* Ultrasonic Init */
	//UART_QuickInit(UART2_RX_PD02_TX_PD03, 9600);
	//UART_CallbackRxInstall(HW_UART2, Ultrasonic_RX_Handler);
	//UART_ITDMAConfig(HW_UART2, kUART_IT_Rx, true);
	GPIO_QuickInit(HW_GPIOD, 3, kGPIO_Mode_OPP);
	GPIO_QuickInit(HW_GPIOD, 2, kGPIO_Mode_IPD);
	GPIO_ResetBit(HW_GPIOD, 3);
	GPIO_ITDMAConfig(HW_GPIOD, 2, kGPIO_IT_RisingFallingEdge, true);
	GPIO_CallbackInstall(HW_GPIOD, Ultrasonic_RX_Handler);
	PIT_QuickInit(HW_PIT_CH1, 100);
	PIT_CallbackInstall(HW_PIT_CH1, PIT1_Handler);
	PIT_ITDMAConfig(HW_PIT_CH1, kPIT_IT_TOF, DISABLE);

	/* UART Init */
	UART_QuickInit(UART0_RX_PD06_TX_PD07, 115200);
	UART_CallbackRxInstall(HW_UART0, UART_RX_Handler);
	UART_ITDMAConfig(HW_UART0, kUART_IT_Rx, true);

	/* ADC Init */
	ADC_QuickInit(ADC0_SE13_PB3, kADC_SingleDiff12or13);  //us_GravityVoltage
	ADC_QuickInit(ADC0_SE8_PB0, kADC_SingleDiff12or13);   //us_GyroscopeVoltage
	ADC_QuickInit(ADC0_SE9_PB1, kADC_SingleDiff12or13);   //us_DirGyroscopeVoltage

	/* FTM Init */
	FTM_QD_QuickInit(FTM2_QD_PHA_PA10_PHB_PA11, kFTM_QD_NormalPolarity, kQD_CountDirectionEncoding);//Right
	FTM_QD_QuickInit(FTM1_QD_PHA_PA08_PHB_PA09, kFTM_QD_NormalPolarity, kQD_CountDirectionEncoding);//Left

	FTM_PWM_QuickInit(FTM0_CH0_PC01, kPWM_Complementary, 10000);//Left
	FTM_PWM_QuickInit(FTM0_CH1_PC02, kPWM_Complementary, 10000);
	FTM_PWM_QuickInit(FTM0_CH2_PC03, kPWM_Complementary, 10000);//Right
	FTM_PWM_QuickInit(FTM0_CH3_PC04, kPWM_Complementary, 10000);
	FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH0 | HW_FTM_CH1, 5000);
	FTM_PWM_ChangeDuty(HW_FTM0, HW_FTM_CH2 | HW_FTM_CH3, 5000);

	/* IRQ Init */
	NVIC_SetPriorityGrouping(NVIC_PriorityGroup_2);
	//NVIC_SetPriority(UART2_RX_TX_IRQn, NVIC_EncodePriority(NVIC_PriorityGroup_2, 0, 0));
	NVIC_SetPriority(PORTD_IRQn, NVIC_EncodePriority(NVIC_PriorityGroup_2, 0, 0));
	NVIC_SetPriority(PIT1_IRQn, NVIC_EncodePriority(NVIC_PriorityGroup_2, 0, 1));
	NVIC_SetPriority(PORTC_IRQn, NVIC_EncodePriority(NVIC_PriorityGroup_2, 0, 1));
	NVIC_SetPriority(PIT0_IRQn, NVIC_EncodePriority(NVIC_PriorityGroup_2, 1, 0));
	NVIC_SetPriority(UART0_RX_TX_IRQn, NVIC_EncodePriority(NVIC_PriorityGroup_2, 2, 0));

	/* PIT Init */
	PIT_QuickInit(HW_PIT_CH0, 1000);
	PIT_CallbackInstall(HW_PIT_CH0, PIT0_Handler);
	PIT_ITDMAConfig(HW_PIT_CH0, kPIT_IT_TOF, ENABLE);

	/*	CAMVALUE Set */
	if (GPIO_ReadBit(HW_GPIOB, 5))
	{
		CAMSELECT = 0;
		GPIO_SetBit(HW_GPIOA, 15);
	}
	else
	{
		CAMSELECT = 1;
		GPIO_ResetBit(HW_GPIOA, 15);
	}
	CAMVALUE1 = CAMVALUESET[CAMSELECT];
	//if (GPIO_ReadBit(HW_GPIOE, 28))
	//{
	//	CharValue = 1;
	//}
	//else
	//{
	//	CharValue = -1;
	//}

	/* Delay Time */
	//while (w_Delay_Start > 0)
	{
#ifdef WATCHDOG
		WDOG_Refresh();
		b_JAM=0;
#endif // WATCHDOG
	}

	/* Cmaera Init */
	Camera_Init();

	for (;;)
	{
		/* 闪烁小灯 */
#ifdef CARDEBUG
		if (GPIO_ReadBit(HW_GPIOB, 7))
		{
			SendData = 1;
			SendDataProcess();
			GPIO_ResetBit(HW_GPIOA, 14);
		}
		else
		{
			SendData = 0;
			GPIO_SetBit(HW_GPIOA, 14);
		}
#endif // CARDEBUG
#ifdef WATCHDOG
		WDOG_Refresh();
#endif // WATCHDOG
	}
}


