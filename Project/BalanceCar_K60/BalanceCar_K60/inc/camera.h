#include "common.h"

#define IMAGE_SIZE  2

#if (IMAGE_SIZE  ==  0)
#define OV7725_W    (80)
#define OV7725_H    (60)

#elif (IMAGE_SIZE == 1)
#define OV7725_W    (160)
#define OV7725_H    (120)

#elif (IMAGE_SIZE == 2)
#define OV7725_W    (240)
#define OV7725_H    (180)

#else
#error "Image Size Not Support!"
#endif

#define BOARD_OV7725_PCLK_PORT      HW_GPIOC
#define BOARD_OV7725_PCLK_PIN       (16)
#define BOARD_OV7725_VSYNC_PORT     HW_GPIOC
#define BOARD_OV7725_VSYNC_PIN      (18)
#define BOARD_OV7725_HREF_PORT      HW_GPIOC
#define BOARD_OV7725_HREF_PIN       (19)

#define BOARD_OV7725_PCLK_PORT2      HW_GPIOC
#define BOARD_OV7725_PCLK_PIN2       (17)
#define BOARD_OV7725_VSYNC_PORT2     HW_GPIOC
#define BOARD_OV7725_VSYNC_PIN2      (15)
#define BOARD_OV7725_HREF_PORT2      HW_GPIOC
#define BOARD_OV7725_HREF_PIN2       (14)

/*
DATA PORT pffset
0 :PTE-PTA7
8 :PTE8-PTA15
16:PTE16-PTA24
*/
#define BOARD_OV7725_DATA_OFFSET    (0) 
#define BOARD_OV7725_DATA_OFFSET2    (16) 

extern uint8_t PixDta[(OV7725_H)*((OV7725_W / 8) + 1)];
extern uint8_t * gpHREF[OV7725_H + 1];

typedef enum
{
	TRANSFER_IN_PROCESS,
	NEXT_FRAME,
}OV7725_Status;

void Camera_Init(void);
void Camera_Init2(void);
int SCCB_Init(uint32_t I2C_MAP);
void UserApp(uint32_t vcount);

void DMA_transmit_init(void *SADDR, void *DADDR, uint32_t count, uint8_t source);
