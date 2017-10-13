#include "common.h"
#include "gpio.h"
#include "dma.h"
#include "camera.h"
#include "irq.h"
#include "i2c.h"
#include "ov7725.h"
#include "dircontrol.h"

uint8_t PixDta[(OV7725_H)*((OV7725_W / 8) + 1)];
uint8_t * gpHREF[OV7725_H + 1];


void UserApp(uint32_t vcount)
{
#ifdef CARDEBUG
	if (!SendData&&GPIO_ReadBit(HW_GPIOB, 6))
	{
		GPIO_ToggleBit(HW_GPIOA, 14);
		SerialDispCCDImage(OV7725_W, OV7725_H, gpHREF);
	}
#endif // CARDEBUG
	SingalSerach(OV7725_W, OV7725_H, gpHREF);
}

int SCCB_Init(uint32_t I2C_MAP)
{
	int r;
	uint32_t instance;
	instance = I2C_QuickInit(I2C_MAP, 50 * 1000);
	r = ov7725_probe(instance);
	if (r)
	{
		return 1;
	}
	r = ov7725_set_image_size(IMAGE_SIZE);
	if (r)
	{
		printf("OV7725 set image error\r\n");
		return 1;
	}
	return 0;
}

void Camera_Init(void)
{
	byte i;
	//INIT OV7725
	while (SCCB_Init(I2C1_SCL_PC10_SDA_PC11)) //C10 C11
	{
		;
	}
	//RAM 
	for (i = 0; i < OV7725_H + 1; i++)
	{
		gpHREF[i] = (uint8_t*)&PixDta[i*OV7725_W / 8];
	}

	DMA_InitTypeDef DMA_InitStruct1 = { 0 };

	/* HREF VSYNC PCLK */
	GPIO_QuickInit(BOARD_OV7725_PCLK_PORT, BOARD_OV7725_PCLK_PIN, kGPIO_Mode_IPD);
	GPIO_QuickInit(BOARD_OV7725_VSYNC_PORT, BOARD_OV7725_VSYNC_PIN, kGPIO_Mode_IPD);
	GPIO_QuickInit(BOARD_OV7725_HREF_PORT, BOARD_OV7725_HREF_PIN, kGPIO_Mode_IPD);

	/* install callback */
	GPIO_CallbackInstall(BOARD_OV7725_VSYNC_PORT, Camera_Handler);

	GPIO_ITDMAConfig(BOARD_OV7725_HREF_PORT, BOARD_OV7725_HREF_PIN, kGPIO_IT_FallingEdge, true);
	GPIO_ITDMAConfig(BOARD_OV7725_VSYNC_PORT, BOARD_OV7725_VSYNC_PIN, kGPIO_IT_FallingEdge, true);
	GPIO_ITDMAConfig(BOARD_OV7725_PCLK_PORT, BOARD_OV7725_PCLK_PIN, kGPIO_DMA_RisingEdge, true);

	/* DATA PORT */
	for (i = 0; i < 8; i++)
	{
		GPIO_QuickInit(HW_GPIOE, BOARD_OV7725_DATA_OFFSET + i, kGPIO_Mode_IFT);
	}

	//DMA
	DMA_InitStruct1.chl = HW_DMA_CH2;
	DMA_InitStruct1.chlTriggerSource = PORTC_DMAREQ;
	DMA_InitStruct1.triggerSourceMode = kDMA_TriggerSource_Normal;
	DMA_InitStruct1.minorLoopByteCnt = 1;
	DMA_InitStruct1.majorLoopCnt = ((OV7725_W / 8) + 1);

	DMA_InitStruct1.sAddr = (uint32_t)&PTE->PDIR + BOARD_OV7725_DATA_OFFSET / 8;
	DMA_InitStruct1.sLastAddrAdj = 0;
	DMA_InitStruct1.sAddrOffset = 0;
	DMA_InitStruct1.sDataWidth = kDMA_DataWidthBit_8;
	DMA_InitStruct1.sMod = kDMA_ModuloDisable;

	DMA_InitStruct1.dAddr = (uint32_t)gpHREF[0];
	DMA_InitStruct1.dLastAddrAdj = 0;
	DMA_InitStruct1.dAddrOffset = 1;
	DMA_InitStruct1.dDataWidth = kDMA_DataWidthBit_8;
	DMA_InitStruct1.dMod = kDMA_ModuloDisable;

	/* initialize DMA moudle */
	DMA_Init(&DMA_InitStruct1);
}

void Camera_Init2(void)
{
	byte i;
	//INIT OV7725
	if (SCCB_Init(I2C0_SCL_PD08_SDA_PD09)) //C10 C11
	{
		while (1);
	}
	//RAM 
	for (i = 0; i < OV7725_H + 1; i++)
	{
		gpHREF[i] = (uint8_t*)&PixDta[i*OV7725_W / 8];
	}

	DMA_InitTypeDef DMA_InitStruct1 = { 0 };

	/* HREF VSYNC PCLK */
	GPIO_QuickInit(BOARD_OV7725_PCLK_PORT2, BOARD_OV7725_PCLK_PIN2, kGPIO_Mode_IPD);
	GPIO_QuickInit(BOARD_OV7725_VSYNC_PORT2, BOARD_OV7725_VSYNC_PIN2, kGPIO_Mode_IPD);
	GPIO_QuickInit(BOARD_OV7725_HREF_PORT2, BOARD_OV7725_HREF_PIN2, kGPIO_Mode_IPD);

	/* install callback */
	GPIO_CallbackInstall(BOARD_OV7725_VSYNC_PORT2, Camera_Handler);

	GPIO_ITDMAConfig(BOARD_OV7725_HREF_PORT2, BOARD_OV7725_HREF_PIN2, kGPIO_IT_FallingEdge, true);
	GPIO_ITDMAConfig(BOARD_OV7725_VSYNC_PORT2, BOARD_OV7725_VSYNC_PIN2, kGPIO_IT_FallingEdge, true);
	GPIO_ITDMAConfig(BOARD_OV7725_PCLK_PORT2, BOARD_OV7725_PCLK_PIN2, kGPIO_DMA_RisingEdge, true);

	/* DATA PORT */
	for (i = 0; i < 8; i++)
	{
		GPIO_QuickInit(HW_GPIOB, BOARD_OV7725_DATA_OFFSET2 + i, kGPIO_Mode_IFT);
	}

	//DMA
	DMA_InitStruct1.chl = HW_DMA_CH2;
	DMA_InitStruct1.chlTriggerSource = PORTC_DMAREQ;
	DMA_InitStruct1.triggerSourceMode = kDMA_TriggerSource_Normal;
	DMA_InitStruct1.minorLoopByteCnt = 1;
	DMA_InitStruct1.majorLoopCnt = ((OV7725_W / 8) + 1);

	DMA_InitStruct1.sAddr = (uint32_t)&PTB->PDIR + BOARD_OV7725_DATA_OFFSET2 / 8;
	DMA_InitStruct1.sLastAddrAdj = 0;
	DMA_InitStruct1.sAddrOffset = 0;
	DMA_InitStruct1.sDataWidth = kDMA_DataWidthBit_8;
	DMA_InitStruct1.sMod = kDMA_ModuloDisable;

	DMA_InitStruct1.dAddr = (uint32_t)gpHREF[0];
	DMA_InitStruct1.dLastAddrAdj = 0;
	DMA_InitStruct1.dAddrOffset = 1;
	DMA_InitStruct1.dDataWidth = kDMA_DataWidthBit_8;
	DMA_InitStruct1.dMod = kDMA_ModuloDisable;

	/* initialize DMA moudle */
	DMA_Init(&DMA_InitStruct1);
}
