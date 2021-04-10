/******************************************************************************
* Copyright (C) 2017, Huada Semiconductor Co.,Ltd All rights reserved.
*
* This software is owned and published by:
* Huada Semiconductor Co.,Ltd ("HDSC").
*
* BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND
* BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.
*
* This software contains source code for use with HDSC
* components. This software is licensed by HDSC to be adapted only
* for use in systems utilizing HDSC components. HDSC shall not be
* responsible for misuse or illegal use of this software for devices not
* supported herein. HDSC is providing this software "AS IS" and will
* not be responsible for issues arising from incorrect user implementation
* of the software.
*
* Disclaimer:
* HDSC MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE,
* REGARDING THE SOFTWARE (INCLUDING ANY ACOOMPANYING WRITTEN MATERIALS),
* ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE, INCLUDING,
* WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY, THE IMPLIED
* WARRANTY OF FITNESS FOR A PARTICULAR PURPOSE OR USE, AND THE IMPLIED
* WARRANTY OF NONINFRINGEMENT.
* HDSC SHALL HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT,
* NEGLIGENCE OR OTHERWISE) FOR ANY DAMAGES WHATSOEVER (INCLUDING, WITHOUT
* LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION,
* LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY LOSS) ARISING FROM USE OR
* INABILITY TO USE THE SOFTWARE, INCLUDING, WITHOUT LIMITATION, ANY DIRECT,
* INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA,
* SAVINGS OR PROFITS,
* EVEN IF Disclaimer HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* YOU ASSUME ALL RESPONSIBILITIES FOR SELECTION OF THE SOFTWARE TO ACHIEVE YOUR
* INTENDED RESULTS, AND FOR THE INSTALLATION OF, USE OF, AND RESULTS OBTAINED
* FROM, THE SOFTWARE.
*
* This software may be replicated in part or whole for the licensed use,
* with the restriction that this Disclaimer and Copyright notice must be
* included with each copy of this software, whether used in part or whole,
* at all times.
*/
/******************************************************************************/
/** \file main.c
 **
 ** A detailed description is available at
 ** @link Sample Group Some description @endlink
 **
 **   - 2017-05-17  1.0  cj First version for Device Driver Library of Module.
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
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


static void App_SPIInit(void);

const uint8_t tx_buf[10]={0xff};
uint8_t rx_buf[10] = {0};



///< SPI0
#define EVB_SPI0_CS_PORT      GpioPortA
#define EVB_SPI0_CS_PIN       GpioPin15
#define EVB_SPI0_SCK_PORT     GpioPortB
#define EVB_SPI0_SCK_PIN      GpioPin3
#define EVB_SPI0_MISO_PORT    GpioPortB
#define EVB_SPI0_MISO_PIN     GpioPin4
#define EVB_SPI0_MOSI_PORT    GpioPortB
#define EVB_SPI0_MOSI_PIN     GpioPin5

static void App_GpioInit(void)
{
    stc_gpio_cfg_t GpioInitStruct;
    DDL_ZERO_STRUCT(GpioInitStruct);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
    
    ///< SPI0引脚配置:主机
    GpioInitStruct.enDrv = GpioDrvH;
    GpioInitStruct.enDir = GpioDirOut;   

    Gpio_Init(EVB_SPI0_CS_PORT, EVB_SPI0_CS_PIN, &GpioInitStruct);
    Gpio_SetAfMode(EVB_SPI0_CS_PORT, EVB_SPI0_CS_PIN, GpioAf1);             ///<配置SPI0_CS
                                                               
    Gpio_Init(EVB_SPI0_SCK_PORT, EVB_SPI0_SCK_PIN, &GpioInitStruct);            
    Gpio_SetAfMode(EVB_SPI0_SCK_PORT, EVB_SPI0_SCK_PIN, GpioAf1);           ///<配置SPI0_SCK
                                                               
    Gpio_Init(EVB_SPI0_MOSI_PORT, EVB_SPI0_MOSI_PIN, &GpioInitStruct);           
    Gpio_SetAfMode(EVB_SPI0_MOSI_PORT, EVB_SPI0_MOSI_PIN, GpioAf1);         ///<配置SPI0_MOSI
                                                               
    GpioInitStruct.enDir = GpioDirIn;                          
    Gpio_Init(EVB_SPI0_MISO_PORT, EVB_SPI0_MISO_PIN, &GpioInitStruct);            
    Gpio_SetAfMode(EVB_SPI0_MISO_PORT, EVB_SPI0_MISO_PIN, GpioAf1);         ///<配置SPI0_MISO
    
    
    ///< 端口方向配置->输入
    GpioInitStruct.enDir = GpioDirIn;
    ///< 端口驱动能力配置->高驱动能力
    GpioInitStruct.enDrv = GpioDrvL;
    ///< 端口上下拉配置->无
    GpioInitStruct.enPu = GpioPuDisable;
    GpioInitStruct.enPd = GpioPdDisable;
    ///< 端口开漏输出配置->开漏输出关闭
    GpioInitStruct.enOD = GpioOdDisable;
    ///< 端口输入/输出值寄存器总线控制模式配置->AHB
    GpioInitStruct.enCtrlMode = GpioAHB;
    ///< GPIO IO USER KEY初始化
    Gpio_Init(EVB_KEY1_PORT, EVB_KEY1_PIN, &GpioInitStruct); 
    
    
    //PD14:板上LED
    GpioInitStruct.enDrv  = GpioDrvH;
    GpioInitStruct.enDir  = GpioDirOut;
    Gpio_Init(EVB_LEDR_PORT, EVB_LEDR_PIN, &GpioInitStruct);
    Gpio_WriteOutputIO(EVB_LEDR_PORT, EVB_LEDR_PIN, FALSE);     //输出高，熄灭LED        
}

/**
 ******************************************************************************
 ** \brief  初始化SPI
 **
 ** \return 无
 ******************************************************************************/
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
//时钟初始化配置
void App_ClkDivInit(void)
{
    //时钟分频设置
    Sysctrl_SetHCLKDiv(SysctrlHclkDiv1);
    Sysctrl_SetPCLKDiv(SysctrlPclkDiv1);
}





void App_SystemClkInit_RCH(en_sysctrl_rch_freq_t enRchFreq)
{  
    ///< RCH时钟不同频率的切换，需要先将时钟切换到RCL
    Sysctrl_SetRCLTrim(SysctrlRclFreq32768);
    Sysctrl_SetRCLStableTime(SysctrlRclStableCycle64);
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);
    Sysctrl_SysClkSwitch(SysctrlClkRCL);
    
    ///< 加载目标频率的RCH的TRIM值
    Sysctrl_SetRCHTrim(enRchFreq);
    ///< 使能RCH
    Sysctrl_ClkSourceEnable(SysctrlClkRCH, TRUE);
    ///< 时钟切换到RCH
    Sysctrl_SysClkSwitch(SysctrlClkRCH);
    
    ///< HCLK不超过24M：此处设置FLASH读等待周期为0 cycle
    Flash_WaitCycle(FlashWaitCycle0);
    
}

void App_SystemClkInit_RCL(en_sysctrl_rcl_freq_t enRclFreq)
{
    ///< RCH时钟不同频率的切换，需要先将时钟切换到RCL
    Sysctrl_SetRCLTrim(enRclFreq);
    Sysctrl_SetRCLStableTime(SysctrlRclStableCycle64);
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);
    Sysctrl_SysClkSwitch(SysctrlClkRCL);
    
    ///< HCLK不超过24M：此处设置FLASH读等待周期为0 cycle
    Flash_WaitCycle(FlashWaitCycle0);    
}

#ifdef SYSTEM_XTH
///<请注意根据外部晶振配置宏——[SYSTEM_XTH]
void App_SystemClkInit_XTH(en_sysctrl_xth_freq_t enXthFreq)
{
    ///<======================== 切换至XTH32MHz ==============================    
    ///< 当使用的时钟源HCLK大于24M：设置FLASH 读等待周期为1 cycle(默认值也为1 cycle)
    if(SysctrlXthFreq24_32MHz == enXthFreq)
    {
        Flash_WaitCycle(FlashWaitCycle1);    
    }
    
    ///< 切换时钟前（根据外部高速晶振）设置XTH频率范围,配置晶振参数，使能目标时钟，此处为SYSTEM_XTH = 32MHz
    Sysctrl_SetXTHFreq(enXthFreq);
    Sysctrl_XTHDriverCfg(SysctrlXtalDriver1);
    Sysctrl_SetXTHStableTime(SysctrlXthStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkXTH, TRUE);
    delay1ms(10);
    Sysctrl_SysClkSwitch(SysctrlClkXTH);
    
    if(SysctrlXthFreq24_32MHz != enXthFreq)
    {
        Flash_WaitCycle(FlashWaitCycle0);    
    }
}
#endif

#ifdef SYSTEM_XTL
void App_SystemClkInit_XTL(void)
{
    ///< 切换时钟前（根据外部低速晶振）设置XTL晶振参数，使能目标时钟，SYSTEM_XTL = 32768Hz
    Sysctrl_XTLDriverCfg(SysctrlXtlAmp3, SysctrlXtalDriver3);
    Sysctrl_SetXTLStableTime(SysctrlXtlStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkXTL, TRUE);
    Sysctrl_SysClkSwitch(SysctrlClkXTL);

    Flash_WaitCycle(FlashWaitCycle0);    

}
#endif

void App_SystemClkInit_PLL48M_byRCH(void)
{
    stc_sysctrl_pll_cfg_t stcPLLCfg;    
    
    ///< RCH时钟不同频率的切换，需要先将时钟切换到RCL
    Sysctrl_SetRCLTrim(SysctrlRclFreq32768);
    Sysctrl_SetRCLStableTime(SysctrlRclStableCycle64);
    Sysctrl_ClkSourceEnable(SysctrlClkRCL, TRUE);
    Sysctrl_SysClkSwitch(SysctrlClkRCL);
    
    ///< 加载目标频率的RCH的TRIM值
    Sysctrl_SetRCHTrim(SysctrlRchFreq4MHz);
    ///< 使能RCH
    Sysctrl_ClkSourceEnable(SysctrlClkRCH, TRUE);
    
    stcPLLCfg.enInFreq    = SysctrlPllInFreq4_6MHz;     //RCH 4MHz
    stcPLLCfg.enOutFreq   = SysctrlPllOutFreq36_48MHz;  //PLL 输出
    stcPLLCfg.enPllClkSrc = SysctrlPllRch;              //输入时钟源选择RCH
    stcPLLCfg.enPllMul    = SysctrlPllMul12;            //4MHz x 12 = 48MHz
    Sysctrl_SetPLLFreq(&stcPLLCfg); 
    
    ///< 当使用的时钟源HCLK大于24M：设置FLASH 读等待周期为1 cycle(默认值也为1 cycle)
    Flash_WaitCycle(FlashWaitCycle1);    
    
    ///< 使能PLL
    Sysctrl_ClkSourceEnable(SysctrlClkPLL, TRUE);
    ///< 时钟切换到PLL
    Sysctrl_SysClkSwitch(SysctrlClkPLL);

}

#if (SYSTEM_XTH == 8000000u)
///<请注意根据外部晶振配置宏——[SYSTEM_XTH],如果使用PLL，XTH必须小于24MHz
void App_SystemClkInit_PLL48M_byXTH(void)
{
    stc_sysctrl_pll_cfg_t stcPLLCfg;    
    
    ///< 切换时钟前（根据外部高速晶振）设置XTH频率范围,配置晶振参数，使能目标时钟，此处为SYSTEM_XTH = 32MHz
    Sysctrl_SetXTHFreq(SysctrlXthFreq4_8MHz);
    Sysctrl_XTHDriverCfg(SysctrlXtalDriver1);
    Sysctrl_SetXTHStableTime(SysctrlXthStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkXTH, TRUE);
    delay1ms(10);
    
    stcPLLCfg.enInFreq    = SysctrlPllInFreq6_12MHz;    //XTH 8MHz
    stcPLLCfg.enOutFreq   = SysctrlPllOutFreq36_48MHz;  //PLL 输出
    stcPLLCfg.enPllClkSrc = SysctrlPllXthXtal;              //输入时钟源选择 //xzy20210406
    stcPLLCfg.enPllMul    = SysctrlPllMul6;             //8MHz x 6 = 48MHz
    Sysctrl_SetPLLFreq(&stcPLLCfg); 
    
    ///< 当使用的时钟源HCLK大于24M：设置FLASH 读等待周期为1 cycle(默认值也为1 cycle)
    Flash_WaitCycle(FlashWaitCycle1);    

    ///< 使能PLL
    Sysctrl_ClkSourceEnable(SysctrlClkPLL, TRUE);    
    ///< 时钟切换到PLL
    Sysctrl_SysClkSwitch(SysctrlClkPLL);

}
#endif
static void App_LedInit(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< 打开GPIO外设时钟门控
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE); 
    
    ///< 端口方向配置->输出(其它参数与以上（输入）配置参数一致)
    stcGpioCfg.enDir = GpioDirOut;
    ///< 端口上下拉配置->下拉
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdEnable;
    
    ///< LED关闭
    Gpio_ClrIO(EVB_LEDR_PORT, EVB_LEDR_PIN);
    
    ///< GPIO IO LED端口初始化
    Gpio_Init(EVB_LEDR_PORT, EVB_LEDR_PIN, &stcGpioCfg);
    

}

//spi

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                            
 ******************************************************************************/

/******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/******************************************************************************
 * Local type definitions ('typedef')                                         
 ******************************************************************************/

/******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/******************************************************************************
 * Local variable definitions ('static')                                      *
 ******************************************************************************/
uint8_t u8TxData[80] = {0xFF};
uint8_t u8RxData[8]={0X00};

/******************************************************************************
 * Local pre-processor symbols/macros ('#define')                             
 ******************************************************************************/
 
/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
 void App_UartCfg(void);
 void App_DmaCfg(void);
 void App_UartPortInit(void);//

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
        
        //时钟分频初始化
        App_ClkDivInit();    
        //时钟切换    
      //  _UserKeyWait(); //USER KEY 按下后继续执行
        App_SystemClkInit_XTH(SysctrlXthFreq4_8MHz);    
      //  _UserKeyWait(); //USER KEY 按下后继续执行
        App_SystemClkInit_PLL48M_byXTH();   

//SPI
    //uart端口配置--- 
 //   App_UartPortInit();
    App_GpioInit();//SPI端口配置
    //UART模块配置
  //  App_UartCfg();
    App_SPIInit(); 
  //  Spi_FuncEnable(M0P_SPI0,SpiMskDmaTxEn);   //使能DMA发送, DMA相关通道使能后，如果Tx Buff为空，会立马启动传输 挪到SPI初始化函数中
      
    
    //DMA参数配置
    App_DmaCfg();







   ///< 片选，开始通讯
   Spi_SetCS(M0P_SPI0, FALSE);
   ///< 主机向从机发送数据
// while(1)
 //  Spi_SendBuf(M0P_SPI0, (uint8_t*)tx_buf, 10);   






   uint8_t  n;



    
    while(1)
    {
     
        if(5 == Dma_GetStat(DmaCh1))                  //完成一次通道传输（RAM缓存 -> UART1_TX  2字节）（DMA通道1）
        {
            Dma_DisableChannel(DmaCh1);               //禁用通道1
            for(n=0;n<80;n++)u8TxData[n] =0xF0;//填充数据
            Dma_EnableChannel(DmaCh1);                //使能通道0
            Dma_ClrStat(DmaCh1);                      //清除通道0状态值    
        }     
    }
}

//UART模块配置
void App_UartCfg(void)
{
    stc_uart_cfg_t  stcCfg;
    stc_uart_baud_t stcBaud;

    DDL_ZERO_STRUCT(stcCfg);                               //初始化变量
    DDL_ZERO_STRUCT(stcBaud);                              //初始化变量
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralUart1,TRUE);//使能UART1外设时钟门控开关

    stcCfg.enRunMode = UartMskMode3;                       //模式3
    stcCfg.enStopBit = UartMsk1bit;                        //1位停止位
    stcCfg.enMmdorCk = UartMskEven;                        //偶校验
    stcCfg.stcBaud.u32Baud = 9600;                         //波特率9600
    stcCfg.stcBaud.enClkDiv = UartMsk8Or16Div;             //通道采样分频配置
    stcCfg.stcBaud.u32Pclk = Sysctrl_GetPClkFreq();        //获得外设时钟（PCLK）频率值
    Uart_Init(M0P_UART1, &stcCfg);                         //串口初始化

    Uart_ClrStatus(M0P_UART1,UartRC);                      //清接收请求
    Uart_ClrStatus(M0P_UART1,UartTC);                      //清发送请求
        
    Uart_EnableFunc(M0P_UART1,UartDmaTxFunc);              //使能DMA发送, DMA相关通道使能后，如果Tx Buff为空，会立马启动传输
    Uart_EnableFunc(M0P_UART1,UartDmaRxFunc);              //使能DMA接收
}

//DMA参数配置
void App_DmaCfg(void)
{
    stc_dma_cfg_t stcDmaCfg;
    


    Sysctrl_SetPeripheralGate(SysctrlPeripheralDma,TRUE);      //使能DMAC外设时钟门控开关


    
    //rx dma配置
    DDL_ZERO_STRUCT(stcDmaCfg);                                //初始化变量
    stcDmaCfg.u32SrcAddress = 0x40000100;                      //接收数据寄存器地址--
    stcDmaCfg.u32DstAddress = (uint32_t)&u8RxData[0];          //接收缓冲
    stcDmaCfg.enSrcAddrReloadCtl = DmaMskSrcAddrReloadEnable;  //使能DMA源地址重载
    stcDmaCfg.enSrcBcTcReloadCtl = DmaMskBcTcReloadEnable;     //使能BC[3:0]和CONFA:TC[15:0]的重载功能
    stcDmaCfg.enDestAddrReloadCtl = DmaMskDstAddrReloadEnable; //使能DMA目的地址重载
    stcDmaCfg.enTransferMode = DmaMskContinuousTransfer;       //连续传输，DMAC传输完成时不清除CONFA:ENS位
    stcDmaCfg.enDstAddrMode = DmaMskDstAddrInc;                //目的地址自增
    stcDmaCfg.enSrcAddrMode = DmaMskSrcAddrFix;                //源地址固定
    stcDmaCfg.u16BlockSize = 1;                                //块传输个数
    stcDmaCfg.u16TransferCnt = 2;                              //块传输次数
    stcDmaCfg.enMode = DmaMskBlock;                            //块(Block)传输
    stcDmaCfg.enTransferWidth = DmaMsk8Bit;                    // 8 bit  字节传输
    stcDmaCfg.enRequestNum = DmaUart1RxTrig;                   //DMA硬件触发源位Uart1Rx
    stcDmaCfg.enPriority = DmaMskPriorityFix;                  //DMA 各通道优先级固定 (CH0>CH1)
    
    Dma_Enable();                                              //DMA模块使能
    Dma_InitChannel(DmaCh0, &stcDmaCfg);                       //DMA通道0初始化


    
    
        
    //tx dma配置---  DMA与SPI0的联系
    DDL_ZERO_STRUCT(stcDmaCfg);                                //初始化变量
    stcDmaCfg.u32SrcAddress = (uint32_t)&u8TxData[0];          //发送数据缓存
    stcDmaCfg.u32DstAddress = 0x4000080c;                      //发送数据寄存器地址
    stcDmaCfg.enSrcAddrReloadCtl = DmaMskSrcAddrReloadEnable;  //使能DMA源地址重载
    stcDmaCfg.enSrcBcTcReloadCtl = DmaMskBcTcReloadEnable;     //使能BC[3:0]和CONFA:TC[15:0]的重载功能
    stcDmaCfg.enDestAddrReloadCtl = DmaMskDstAddrReloadEnable; //使能DMA目的地址重载
    stcDmaCfg.enTransferMode = DmaMskOneTransfer;              //一次传输，DMAC传输完成时清除CONFA:ENS位
    stcDmaCfg.enDstAddrMode = DmaMskDstAddrFix;                //目的地址固定
    stcDmaCfg.enSrcAddrMode = DmaMskSrcAddrInc;                //源地址自增
    stcDmaCfg.u16BlockSize = 1;                                //块传输个数
    stcDmaCfg.u16TransferCnt = 80;                              //块传输次数，一次传输一个字节，传输两次
    stcDmaCfg.enMode = DmaMskBlock;                            //块(Block)传输
    stcDmaCfg.enTransferWidth = DmaMsk8Bit;                    // 8 bit  字节传输
    stcDmaCfg.enRequestNum = DmaSPI0TXTrig;                   //DMA硬件触发源位SPI0 发送 Buf 空
    stcDmaCfg.enPriority = DmaMskPriorityFix;                  //DMA 各通道优先级固定 (CH0>CH1)
    Dma_EnableChannel(DmaCh1);                                 //使能通道1----------------------------------------原先不知道为啥是通道0

    Dma_InitChannel(DmaCh1, &stcDmaCfg);                       //DMA通道1初始化
    
}



/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/


