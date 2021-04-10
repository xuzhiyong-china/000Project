





#include "ddl.h"
#include "uart.h"
#include "gpio.h"
#include "sysctrl.h"
#include "dmac.h"
#include "sysctrl.h"
#include "gpio.h"
#include "flash.h"
#include "pca.h"
#include "lpm.h"
#include "adt.h"
#include "ddl.h"
#include "spi.h"
#include "gpio.h"
#include "reset.h"
//SPI
#include "systemclock.h"//
#include "WS2812.h"


//static void App_SPIInit(void);

const uint8_t tx_buf[10]={0xff};
uint8_t rx_buf[10] = {0};

   

unsigned char color_Data[WS2812_number][3];//每个ws2812 的G、R、B





uint8_t u8RxData[8]={0X00};



 void App_DmaCfg(void);

 

/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return uint32_t return value, if needed
 **
 ** This sample
 **
 ******************************************************************************/
int32_t main(void)
{  
      //当前的配置：pa01是时钟频率的输出口，PD02是模拟的方式控制彩灯,PB04是PCA的输出口
//时钟
    System_ClockInit();
//SPI+DMA
//    App_GpioInit();//SPI端口配置
//    App_SPIInit(); 
   //DMA参数配置
//   App_DmaCfg();
 //  Spi_SetCS(M0P_SPI0, FALSE);
   uint8_t  n;

//   WS2812_init();



    while(1)
    {
  

      //  if(5 == Dma_GetStat(DmaCh1))                  //完成一次通道传输（RAM缓存 -> UART1_TX  2字节）（DMA通道1）
    //    {
     //       Dma_DisableChannel(DmaCh1);               //禁用通道1
    //        for(n=0;n<80;n++)u8TxData[n] =0xF0;//填充数据
      //      Dma_EnableChannel(DmaCh1);                //使能通道0
      //      Dma_ClrStat(DmaCh1);                      //清除通道0状态值    
      //  }     
    }
}


/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


