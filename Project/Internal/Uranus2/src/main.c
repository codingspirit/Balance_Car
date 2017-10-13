#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "common.h"
#include "dma.h"
#include "pit.h"
#include "lptmr.h"
#include "gpio.h"
#include "uart.h"
#include "i2c.h"
#include "flash.h"

#include "protocol.h"
#include "mpu9250.h"
#include "bmp180.h"
#include "virtual_eep.h"
#include "imu.h"
#include "filter.h"
#include "calibration.h"
#include "mq.h"

#include "appdef.h"



KalmanState_t KAState[3], KGState[3], KMState[3];
uint8_t gRevBuf[64];
struct dcal_t dcal;
int RunState;

static uint32_t ano_make_packet(uint8_t *buf, attitude_t *angle, int16_t *acc, int16_t *gyo, int16_t *mag, int32_t pressure)
{

    int i;
    uint8_t sum = 0;
    if(dcal.out_data_type == 0)
    {
        buf[0] = 0x88;
        buf[1] = 0xAF;
        buf[2] = 28;
        buf[3] = (acc[0])>>8;
        buf[4] = (acc[0])>>0;
        buf[5] = (acc[1])>>8;
        buf[6] = (acc[1])>>0;
        buf[7] = (acc[2])>>8;
        buf[8] = (acc[2])>>0;
        buf[9] = (gyo[0])>>8;
        buf[10] = (gyo[0])>>0;
        buf[11] = (gyo[1])>>8;
        buf[12] = (gyo[1])>>0;
        buf[13] = (gyo[2])>>8;
        buf[14] = (gyo[2])>>0;
        buf[15] = (mag[0])>>8;
        buf[16] = (mag[0])>>0;
        buf[17] = (mag[1])>>8;
        buf[18] = (mag[1])>>0;
        buf[19] = (mag[2])>>8;
        buf[20] = (mag[2])>>0;
        buf[21] = ((int16_t)((angle->P)*100))>>8;
        buf[22] = ((int16_t)((angle->P)*100))>>0;
        buf[23] = ((int16_t)((angle->R)*100))>>8;
        buf[24] = ((int16_t)((angle->R)*100))>>0;
        buf[25] = (int16_t)((180-(angle->Y))*10)>>8;
        buf[26] = (int16_t)((180-(angle->Y))*10)>>0;
        
        buf[27] = (pressure)>>0;
        buf[28] = (pressure)>>8;
        buf[29] = (pressure)>>16;
        buf[30] = (pressure)>>24;

        for(i=0; i<30; i++)
        {
            sum += buf[i];
        }
        buf[31] = sum;
        return 32;
    }
    else
    {
        buf[0] = 0x88;
        buf[1] = 0xAF;
        buf[2] = 6;
        buf[3] = ((int16_t)((angle->P)*100))>>8;
        buf[4] = ((int16_t)((angle->P)*100))>>0;
        buf[5] = ((int16_t)((angle->R)*100))>>8;
        buf[6] = ((int16_t)((angle->R)*100))>>0;
        buf[7] = (int16_t)((180+(angle->Y))*10)>>8;
        buf[8] = (int16_t)((180+(angle->Y))*10)>>0;
        for(i=0; i<8; i++)
        {
            sum += buf[i];
        }
        buf[9] = sum;
        return 10; 
    }
}



int sensor_init(void)
{
    uint32_t ret;

    I2C_Init(I2C0_SCL_PB00_SDA_PB01, 100*1000);
    DelayMs(15);
    ret = mpu9250_init(0);
    if(ret)
    {
        printf("mpu9250 init:%d\r\n", ret);
        printf("restarting...\r\n");
        DelayMs(300);
        SystemSoftReset(); 
    }
    
    ret = bmp180_init(0);
    if(ret)
    {
        printf("bmp 180 init failed:%d\r\n", ret);
    }
    
    struct mpu_config config;
    
    config.afs = AFS_16G;
    config.gfs = GFS_2000DPS;
    config.mfs = MFS_14BITS;
    config.aenable_self_test = false;
    config.genable_self_test = false;
    mpu9250_config(&config);
    mpu9250_enable_raw_data_int();
    return ret;
}

static void ShowInfo(void)
{
    printf("Uranus %d\r\n", VERSION);
    printf("CoreClock:%dHz\r\n", GetClock(kCoreClock));
    printf("UID:0x%X\r\n", GetUID());
    printf("Reset Status Code:%d\r\n", GetResetStatus());
    
    switch(RunState)
    {
        case kPTL_REQ_MODE_9AXIS:
            printf("9 AXIS mode\r\n");
            break;
        case kPTL_REQ_MODE_6AXIS:
            printf("6 AXIS mode\r\n");
            break;
        default:
            printf("RunState:0x%X\r\n", RunState);
            break;
    }
    dcal_print(&dcal);
}

//#define FALL_LIMIT      100
//uint32_t FallDetection(int16_t *adata)
//{
//    uint32_t ret;
//    
//    ret = 0;
//    if((abs(adata[0])< FALL_LIMIT) && (abs(adata[1]) < FALL_LIMIT) && (abs(adata[2]) < FALL_LIMIT))
//    {
//        ret = 1;
//    }
//    return ret;
//}

//uint32_t FallDetectionG(int16_t *gdata)
//{
//    uint32_t sum, ret;
//    ret = 0;
//    
//    sum = gdata[0]*gdata[0] + gdata[1]*gdata[1] + gdata[2]*gdata[2];
//    if(sum > 500000)
//    {
//        ret = 1;
//    }

//    return ret;
//}

void HWInit(void)
{

    DelayInit();

    veep_init();
    veep_read((uint8_t*)&dcal, sizeof(struct dcal_t));
    dcal_init(&dcal);
    #if defined(ONLY_EULAR)
    dcal.out_data_type = 1;
    #else
    dcal.out_data_type = 0;
    #endif
    
    mq_init();
    
    GPIO_Init(HW_GPIOC, PIN3, kGPIO_Mode_OPP);
    GPIO_Init(HW_GPIOA, MPU9250_INT_PIN, kGPIO_Mode_IFT);
    GPIO_SetIntMode(HW_GPIOA, MPU9250_INT_PIN, kGPIO_Int_RE, true);
    
    UART_Init(UART0_RX_PA01_TX_PA02, dcal.baudrate);
    UART_SetDMAMode(HW_UART0, kUART_DMARx, true);
    
    LPTMR_TC_InitTypeDef init;
    init.timeInMs = (1000/dcal.outfrq) -1 ;
    LPTMR_TC_Init(&init);
    
    DMA_Init_t Init;
    Init.chl = DMA_RX_CH;
    Init.chlTrigSrc = UART0_REV_DMAREQ;
    Init.trigSrcMod = kDMA_TrigSrc_Normal;
    Init.transCnt = 9999;

    Init.sAddr = (uint32_t)&UART0->D;
    Init.sAddrIsInc = false;
    Init.sDataWidth = kDMA_DataWidthBit_8;
    Init.sMod = kDMA_ModuloDisable;

    Init.dAddr = (uint32_t)gRevBuf;
    Init.dAddrIsInc = true;
    Init.dDataWidth = kDMA_DataWidthBit_8;
    Init.dMod = kDMA_ModuloDisable;
    DMA_Init(&Init);

    DMA_EnableReq(DMA_RX_CH);
    
    sensor_init();
    
    UART_SetIntMode(HW_UART0, kUART_IntIdleLine, true);
    LPTMR_ITDMAConfig(kLPTMR_IT_TOF, true);
}


void ano_callback(rev_data_t *rd);
uint32_t DataRevDecode(msg_t *pMsg, uint8_t *buf);

int main(void)
{
#if defined(BOOTLOADER)
    SCB->VTOR = 0x5000;
#endif
    
    int i;
    int16_t ta[3], tg[3], tm[3];
    int16_t adata[3], gdata[3], mdata[3];
    int16_t radata[3], rgdata[3], rmdata[3];
    float fadata[3], fgdata[3], fmdata[3];
    static attitude_t angle;
    uint32_t ret;
    float pressure, dummy, temperature;
    float ares, gres, mres;
    uint32_t len;
    static uint8_t buf[64];
    
    HWInit();

    ares  = mpu9250_get_ares();
    gres  = mpu9250_get_gres();
    mres  = mpu9250_get_mres();
    
    ano_set_callback(ano_callback);

    PIT_Init();

    PIT_SetTime(1, 1000*1000);

    static uint32_t time, load_val, fac_us;
    fac_us = GetClock(kBusClock);
    fac_us /= 1000000;

    RunState = dcal.mode;
    if(RunState == kPTL_REQ_MODE_9AXIS)
    {
        RunState = kPTL_REQ_MODE_9AXIS;
    }
    else
    {
        RunState = kPTL_REQ_MODE_6AXIS;
    }
    ShowInfo();
    
    KalmanSimple1D(&KAState[0], 1, 10);
    KalmanSimple1D(&KAState[1], 1, 10);
    KalmanSimple1D(&KAState[2], 1, 10);
    KalmanSimple1D(&KGState[0], 1, 10);
    KalmanSimple1D(&KGState[1], 1, 10);
    KalmanSimple1D(&KGState[2], 1, 10);
    KalmanSimple1D(&KMState[0], 1, 10);
    KalmanSimple1D(&KMState[1], 1, 10);
    KalmanSimple1D(&KMState[2], 1, 10);  
     
    while(1)
    {
        if(mq_exist())
        {
            msg_t* pMsg;
            pMsg = mq_pop();
            switch(pMsg->cmd)
            {
                /* timer event */
                case kMSG_CMD_TIMER:
                    mpu9250_read_mag_raw(rmdata);

                    if(RunState == kPTL_REQ_MODE_CAL)
                    {
                        dcal_minput(&dcal, rmdata);
                        dcal_output(&dcal);
                        GPIO_PinToggle(HW_GPIOC, 3);
                    }
                    dcal_ginput(gdata);
                    /* bmp read */
                    ret = bmp180_conversion_process(&dummy, &temperature);
                    if(!ret)
                    {
                        pressure = dummy;
                    }
                    break;
                 /* IMU sensor data ready */
                case kMSG_CMD_SENSOR_DATA_READY:
                    mpu9250_read_accel_raw(radata);
                    mpu9250_read_gyro_raw(rgdata);
                    
                    for(i=0; i<3; i++)
                    {
                        KalmanRun(&KAState[i], radata[i]);
                        radata[i] = KAState[i].State;
                        
                        KalmanRun(&KGState[i], rgdata[i]);
                        rgdata[i] = KGState[i].State;
                        
                        KalmanRun(&KMState[i], rmdata[i]);
                        rmdata[i] = KMState[i].State;
                        
                        /* turn fine raw data from cal data */
                        adata[i] = radata[i] - dcal.ao[i];
                        gdata[i] = rgdata[i] - dcal.go[i];
                        mdata[i] = (rmdata[i] - dcal.mo[i])/dcal.mg[i];
                    }

                    /* set timer */
                    time = PIT_GetCnt(HW_PIT_CH1);
                    PIT_SetCnt(HW_PIT_CH1, 1000*1000);
                    load_val = PIT_GetCnt(HW_PIT_CH1);
                    time = load_val - time;
                    time /= fac_us;

                    /* trun raw data input fine turned data */
                    int16_t gadj[3];
                    dcal_get_gadj(gadj);
                    
                    for(i=0;i<3;i++)
                    {
                        fadata[i] = (float)adata[i]*ares;
                        fgdata[i] = ((float)(gdata[i] - gadj[i]))*gres;
                        fmdata[i] = (float)mdata[i]*mres;
                        if((fgdata[i] < 1) && (fgdata[i] > -1))
                        {
                            fgdata[i] = 0;
                        }
                        if(RunState == kPTL_REQ_MODE_6AXIS)
                        {
                            fmdata[i] = 0;
                        }
                    }
                    
                    ret = imu_get_euler_angle(fadata, fgdata, fmdata, &angle);
                    halfT = ((float)time)/1000/2000; 
                        
                break;
                    
                case kMSG_CMD_DATA_OUT:
                    for(i=0; i<3; i++)
                    {
                        ta[i] = (adata[i]*ares*1000);
                        tm[i] = (mdata[i]*mres);
                        tg[i] = (gdata[i] - gadj[i])*(gres*60);
                    }
                    
                    if(RunState != kPTL_REQ_MODE_CAL)
                    {
                        GPIO_PinToggle(HW_GPIOC, 3);                
                        if(UART_DMAGetRemain(HW_UART0) == 0)
                        {
                            len = ano_make_packet(buf, &angle, ta, tg, tm, (int32_t)pressure);
                            UART_DMASend(HW_UART0, DMA_TX_CH, buf, len);
                        }
                    }
                    break;
                /* data reviecved from PC */
                case kMSG_CMD_DATA_REV:
                {
                    dcal.out_data_type = 0;
                    len = DataRevDecode(pMsg, buf);
                    while(UART_DMAGetRemain(HW_UART0) != 0);
                    for(i=0; i<len; i++)
                    {
                        UART_PutChar(HW_UART0, buf[i]);
                    }
                    break;
                }
            }
        }
        else
        {
           // EnterSTOPMode(false);
        }
    }
}

uint32_t DataRevDecode(msg_t *pMsg, uint8_t *buf)
{
    int len, i;
    len = 0;
    
    switch(pMsg->type)
    {
        case kPTL_REQ_FW:
        {
            fw_info_t fwinfo;
            fwinfo.version = VERSION;
            fwinfo.uid = GetUID();
            fwinfo.id = dcal.id;
            fwinfo.mode = RunState;
            len = ano_encode_fwinfo(&fwinfo, buf);
            break;
        }
        /* send all cal data to PC */
        case kPTL_REQ_OFS_ALL:
        {
            offset_t offset;
            for(i=0; i<3; i++)
            {
                offset.acc_offset[i] = dcal.ao[i];
                offset.gyro_offset[i] = dcal.go[i];
                offset.mag_offset[i] = dcal.mo[i];
            }
            len = ano_encode_offset_packet(&offset, buf);
            break;
        }
        /* get all cal data and program to flash */
        case kPTL_DATA_OFS_ALL:
        {
            dcal.ao[0] = (pMsg->payload[0]<<8) + (pMsg->payload[1]<<0);
            dcal.ao[1] = (pMsg->payload[2]<<8) + (pMsg->payload[3]<<0);
            dcal.ao[2] = (pMsg->payload[4]<<8) + (pMsg->payload[5]<<0);
            dcal.go[0] = (pMsg->payload[6]<<8) + (pMsg->payload[7]<<0);
            dcal.go[1] = (pMsg->payload[8]<<8) + (pMsg->payload[9]<<0);
            dcal.go[2] = (pMsg->payload[10]<<8) + (pMsg->payload[11]<<0);
            dcal.mo[0] = (pMsg->payload[12]<<8) + (pMsg->payload[13]<<0);
            dcal.mo[1] = (pMsg->payload[14]<<8) + (pMsg->payload[15]<<0);
            dcal.mo[2] = (pMsg->payload[16]<<8) + (pMsg->payload[17]<<0);
            break;
        }
        case kPTL_REQ_SAVE_OFS:
            dcal.magic = 0x5ACB;
            veep_write((uint8_t*)&dcal, sizeof(struct dcal_t));
            break;
        case kPTL_REQ_MODE_6AXIS:
        case kPTL_REQ_MODE_9AXIS:
            RunState = pMsg->type;
            dcal.mode = RunState;
            break;
        case kPTL_REQ_MODE_CAL:
            dcal_reset_mag(&dcal);
            RunState = pMsg->type;
            break;
        case kPTL_DATA_OUT_FRQ:
            dcal.outfrq = (pMsg->payload[0]<<8) + (pMsg->payload[1]<<0);
            dcal.baudrate = (pMsg->payload[2]<<24) + (pMsg->payload[3]<<16) + (pMsg->payload[4]<<8) + (pMsg->payload[5]<<0);
            UART_SetBaudRate(HW_UART0, dcal.baudrate);
            LPTMR_SetTime((1000/dcal.outfrq) -1 );
            break;
    }
    return len;
}

void ano_callback(rev_data_t *rd)
{
    msg_t msg;
    msg.cmd = kMSG_CMD_DATA_REV;
    msg.type = rd->cmd;
    msg.msg_len = rd->len;
    memcpy(msg.payload, rd->buf, rd->len);
    mq_push(msg);
}

