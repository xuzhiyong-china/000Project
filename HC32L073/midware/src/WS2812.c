


#include "WS2812.h"

//无需知道主板控制了多少个灯珠，只管按照最多的数量发数据出去，没有的就不管了。
//灯珠的控制方式：上位机会下发需要控制的灯珠以及需要的颜色。1是亮对应的颜色；0是不操作，维持原来的状态
//下位机无需知道传感器数量与灯珠数量？（有多少灯珠与传感器，直接按照最大数量发）


#define WS2812number_and_rescode    WS2812_number+2 //2个灯是res码
#define DMA_send_byte   (WS2812number_and_rescode)*24 
//(WS2812_number+2)*24 //RES的时间大概是50us，用2个灯的时间完成。一个灯珠需要24字节。
static uint8_t color_Data[WS2812number_and_rescode][3];//每个ws2812 的G、R、B数据
static uint8_t DMA_Data[WS2812number_and_rescode][3][8];//每个ws2812 的G、R、B分别对应8字节



#define array_G 0
#define array_R 1
#define array_B 2
#define ws2812_one 0xf8
#define ws2812_zero 0xc0  //SPI数据对应的不同的占空比，以对应1码与0码








//SPI
 static void App_GpioInit(void)//only use MOSI---PB05
{



    stc_gpio_cfg_t GpioInitStruct;
    DDL_ZERO_STRUCT(GpioInitStruct);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
    
    ///< SPI0引脚配置:主机
    GpioInitStruct.enDrv = GpioDrvH;
    GpioInitStruct.enDir = GpioDirOut;   



    Gpio_Init(GpioPortA, GpioPin15, &GpioInitStruct);
    Gpio_SetAfMode(GpioPortA, GpioPin15, GpioAf1);             ///<配置SPI0_CS
                                                               
    Gpio_Init(GpioPortB, GpioPin3, &GpioInitStruct);            
    Gpio_SetAfMode(GpioPortB, GpioPin3, GpioAf1);           ///<配置SPI0_SCK
                                                               
    Gpio_Init(GpioPortB, GpioPin5, &GpioInitStruct);           
    Gpio_SetAfMode(GpioPortB, GpioPin5, GpioAf1);         ///<配置SPI0_MOSI
                                                               
    GpioInitStruct.enDir = GpioDirIn;                          
    Gpio_Init(GpioPortB, GpioPin4, &GpioInitStruct);            
    Gpio_SetAfMode(GpioPortB, GpioPin4, GpioAf1);         ///<配置SPI0_MISO

                                                               
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
static void App_DmaCfg(void)
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




void WS2812_set_color(uchar *buffer,uchar buffer_length,uchar R,uchar G,uchar B)
//根据上位机传下来数据，改变需要设置颜色的灯珠。1是需要设置为对应的颜色，0是不操作（无需改变）
{    
    uint32_t n,m,a,b,c,id;
    uchar temp;
    a=0;
    b=1;
    c=3;
    for (n = 0; n < buffer_length; n++)
    {
        for (m = 0; m < 8; m++)
            {
              //  if(Get_Bit(*(buffer+n),m))
                    
            }

        a+=1;
        b+=1;
        c+=1;        
    }    
}




//编号id的灯的RGB值转换为SPI数据，并传递给DMA数组
static void Color_data_TO_DMA_DATA(  uint32_t id,  uint8_t R,uint8_t G,uint8_t B)
{  
    uchar n;
    for(n=0;n<8;n++)
    {
        if (Get_Bit(R, n))
        {
           DMA_Data[id][array_R][n]=ws2812_one;
        }
        else
        {
           DMA_Data[id][array_R][n]=ws2812_zero;
        }
    ///////////////////////////////////////////////////////
        if (Get_Bit(G, n))
        {
           DMA_Data[id][array_G][n]=ws2812_one;
        }
        else
        {
           DMA_Data[id][array_G][n]=ws2812_zero;
        }
    ///////////////////////////////////////////////////////

        if (Get_Bit(B, n))
        {
           DMA_Data[id][array_B][n]=ws2812_one;
        }
        else
        {
           DMA_Data[id][array_B][n]=ws2812_zero;
        }        
    } 
}






void light_WS2812(void)//点亮WS2812----填充DMA数据，然后开启DMA传输
{
    //先把数据刷新到color_data中
    //再把RGB数据转换为SPI-DMA数据
    //然后开始DMA，直到完成
    uint32_t n;
 
    if(5 == Dma_GetStat(DmaCh1))                  //完成一次通道传输（RAM缓存 -> UART1_TX  2字节）（DMA通道1）
    {
        Dma_DisableChannel(DmaCh1);               //禁用通道1
        for (n = 0; n < WS2812number_and_rescode; n++)
         {        
             Color_data_TO_DMA_DATA( n, color_Data[n][array_R],color_Data[n][array_G], color_Data[n][array_B]); 
             //填充数据
         }    
        Dma_EnableChannel(DmaCh1);                //使能通道0
        Dma_ClrStat(DmaCh1);                      //清除通道0状态值    
    }     
}
void WS2812_init(void)
{
    uint32_t n,id;
    App_GpioInit();
    App_SPIInit();
    App_DmaCfg();    
    //这里让灯都亮一次白光，表示上电成功
    for (n = 0; n < WS2812_number; n++)
    {        
        color_Data[n][array_R]=0xff;
        color_Data[n][array_G]=0xff;
        color_Data[n][array_B]=0xff;      
    }   
    //最后两个灯的color_Data为复位码--复位码初始化为0
    id=WS2812_number;
    color_Data[id][array_R]=0x00;
    color_Data[id][array_G]=0x00;
    color_Data[id][array_B]=0x00;
    id=WS2812_number+1;
    color_Data[id][array_R]=0x00;
    color_Data[id][array_G]=0x00;
    color_Data[id][array_B]=0x00;   
    light_WS2812();//开启DMA点彩灯
}



