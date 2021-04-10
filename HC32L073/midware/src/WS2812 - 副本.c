

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "WS2812.h"


/*

#define DMA_send_byte   (1+2)*24 //(WS2812_number+2)*24 //RES的时间大概是50us，用2个灯的时间完成。一个灯珠需要24字节。

#define ws2812_one 0xf8
#define ws2812_zero 0xc0  //SPI数据对应的不同的占空比，以对应1码与0码


static uint8_t DMA_Data[1+2][3][8];//每个ws2812 的G、R、B分别对应8字节
uint32_t ws2812_id;

#define array_G 0
#define array_R 1
#define array_B 2






//SPI
 static void App_GpioInit(void)//only use MOSI---PB05
{
    stc_gpio_cfg_t GpioInitStruct;
    DDL_ZERO_STRUCT(GpioInitStruct);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
    
    ///< SPI0引脚配置:主机
    GpioInitStruct.enDrv = GpioDrvH;
    GpioInitStruct.enDir = GpioDirOut;   
                                                            
    Gpio_Init(GpioPortB, GpioPin5, &GpioInitStruct);           
    Gpio_SetAfMode(GpioPortB, GpioPin5, GpioAf1);         ///<配置SPI0_MOSI
                                                               
}



static void App_SPIInit(void)
{
    stc_spi_cfg_t  SpiInitStruct;        
    ///< 打开外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralSpi0,TRUE);    
    ///<复位模块
    Reset_RstPeripheral0(ResetMskSpi0);    
    //SPI0模块配置：主机
    SpiInitStruct.enSpiMode = SpiMskMaster;     //配置位主机模式
    SpiInitStruct.enPclkDiv = SpiClkMskDiv8;    //波特率：PCLK/8
    SpiInitStruct.enCPHA    = SpiMskCphafirst;  //第一边沿采样
    SpiInitStruct.enCPOL    = SpiMskcpollow;    //极性为低
    Spi_FuncEnable(M0P_SPI0,SpiMskDmaTxEn); 
    Spi_Init(M0P_SPI0, &SpiInitStruct);
}
//SPI





//DMA参数配置
 void App_DmaCfg(void)
 {
     stc_dma_cfg_t stcDmaCfg;
     Sysctrl_SetPeripheralGate(SysctrlPeripheralDma,TRUE);      //使能DMAC外设时钟门控开关    
     Dma_Enable();                                              //DMA模块使能
     //tx dma配置---  DMA与SPI0的联系
     DDL_ZERO_STRUCT(stcDmaCfg);                                //初始化变量
     stcDmaCfg.u32SrcAddress = (uint32_t)&DMA_Data[0][0][0];    //发送数据缓存
     stcDmaCfg.u32DstAddress = 0x4000080c;                      //发送数据寄存器地址
     stcDmaCfg.enSrcAddrReloadCtl = DmaMskSrcAddrReloadEnable;  //使能DMA源地址重载
     stcDmaCfg.enSrcBcTcReloadCtl = DmaMskBcTcReloadEnable;     //使能BC[3:0]和CONFA:TC[15:0]的重载功能
     stcDmaCfg.enDestAddrReloadCtl = DmaMskDstAddrReloadEnable; //使能DMA目的地址重载
     stcDmaCfg.enTransferMode = DmaMskOneTransfer;              //一次传输，DMAC传输完成时清除CONFA:ENS位
     stcDmaCfg.enDstAddrMode = DmaMskDstAddrFix;                //目的地址固定
     stcDmaCfg.enSrcAddrMode = DmaMskSrcAddrInc;                //源地址自增
     stcDmaCfg.u16BlockSize = 1;                                //块传输个数
     stcDmaCfg.u16TransferCnt = DMA_send_byte;                  //块传输次数，一次传输一个字节，传输N次
     stcDmaCfg.enMode = DmaMskBlock;                            //块(Block)传输
     stcDmaCfg.enTransferWidth = DmaMsk8Bit;                    // 8 bit  字节传输
     stcDmaCfg.enRequestNum = DmaSPI0TXTrig;                   //DMA硬件触发源位SPI0 发送 Buf 空
     stcDmaCfg.enPriority = DmaMskPriorityFix;                  //DMA 各通道优先级固定 (CH0>CH1)
     Dma_InitChannel(DmaCh1, &stcDmaCfg);                       //DMA通道1初始化
     Dma_EnableChannel(DmaCh1);                                 //使能通道1
 }
//DMA参数配置
void WS2812_init(void)
{
    App_GpioInit();
    App_SPIInit();
    App_DmaCfg();
}



void ONE_TO_DMA_DATA(void)//编号id的灯的RGB值传递给DMA数组
{
   
   
     uint32_t id,  uint8_t R,uint8_t G,uint8_t B

     uint8_t n;
    for(n=0;n<7;n++)
        {

        }
    if(1)  DMA_Data[id][array_G][n]=ws2812_one;
    else  DMA_Data[id][array_G][n]=ws2812_zero;

    getbit (G, n)
    for(n=0;n<7;n++)
    if(getbit(R, n))  DMA_Data[id][array_R][n]=ws2812_one;
    else  DMA_Data[id][array_R][n]=ws2812_zero;
    
    for(n=0;n<7;n++)
    if(getbit(B, n))  DMA_Data[id][array_B][n]=ws2812_one;
    else  DMA_Data[id][array_B][n]=ws2812_zero;
 
}



void WS2812_set(uint32_t start,uint32_t end,uint8_t *p)//将要设置的灯的编号
{
 
}


*/


