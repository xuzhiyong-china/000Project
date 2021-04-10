/******************************************************************************
* Copyright (C) 2016, Huada Semiconductor Co.,Ltd All rights reserved.
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
 **   - 2016-02-16  1.0  XYZ First version for Device Driver Library of Module.
 **
 ******************************************************************************/

/******************************************************************************
 * Include files
 ******************************************************************************/
#include "sysctrl.h"
#include "gpio.h"
#include "flash.h"
#include "pca.h"
#include "lpm.h"
#include "adt.h"
#include "ddl.h"
#include "spi.h"
#include "reset.h"



/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define WS2812_high() (*((volatile uint32_t *)(0x40020DC8)) |= 0x04)
#define WS2812_low()  (*((volatile uint32_t *)(0x40020DC8)) &= 0xFB)
#define _nop_() (1+1)

const uint8_t tx_buf[10]={1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
uint8_t rx_buf[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};



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
__IO uint8_t pulse=125;   //脉冲宽度，占空比=(255-pulse)/100*100%

/*****************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
void App_PortCfg(void);
void App_ClkDivInit(void);
void App_SystemClkInit_RCH(en_sysctrl_rch_freq_t enRchFreq);
void App_SystemClkInit_RCL(en_sysctrl_rcl_freq_t enRclFreq);
void App_SystemClkInit_XTH(en_sysctrl_xth_freq_t enXthFreq);
void App_SystemClkInit_XTL(void);
void App_SystemClkInit_PLL48M_byRCH(void);
void App_SystemClkInit_PLL48M_byXTH(void);
static void App_LedInit(void);

static void App_GpioInit(void); 
static void App_PcaInit(void);


static void App_SPIInit(void);




unsigned int debug;




/**
 ******************************************************************************
 ** \brief  Main function of project
 **
 ** \return uint32_t return value, if needed
 **
 ** check Pxx to verify the clock frequency.
 **
 ******************************************************************************/
int32_t main(void)//6795
{  
  //当前的配置：pa01是时钟频率的输出口，PD02是模拟的方式控制彩灯,PB04是PCA的输出口
//时钟
        App_PortCfg();    
    //时钟分频初始化
    App_ClkDivInit();    
    //时钟切换    
  //  _UserKeyWait(); //USER KEY 按下后继续执行
    App_SystemClkInit_XTH(SysctrlXthFreq4_8MHz);    
  //  _UserKeyWait(); //USER KEY 按下后继续执行
    App_SystemClkInit_PLL48M_byXTH();   

///< LED端口初始化
    App_LedInit();



       uint16_t tmp;
       volatile uint8_t tmp1;
    
       tmp = 0;
    
       ///< 端口初始化
       App_GpioInit();
       ///< SPI初始化
       App_SPIInit(); 
       
       ///< USER 按下启动通信
    //   while(TRUE == Gpio_GetInputIO(EVB_KEY1_PORT, EVB_KEY1_PIN));    
       
       ///< 片选，开始通讯
       Spi_SetCS(M0P_SPI1, FALSE);
 
       ///< 主机向从机发送数据

while(1)
{
       while(FALSE == M0P_SPI1->STAT_f.TXE){;}//发送缓冲器状态标志 :1空,0非空
              M0P_SPI1->DATA =0xff;
}


       
       Spi_SendBuf(M0P_SPI1, (uint8_t*)tx_buf, 10);   





       
       ///< 结束通信
       Spi_SetCS(M0P_SPI1, TRUE);
    
       delay1ms(1);
    
       ///< 片选，开始通讯
       Spi_SetCS(M0P_SPI1, FALSE);
       ///< 主机接收从机数据
       Spi_ReceiveBuf(M0P_SPI1, rx_buf, 10);
       ///< 结束通信
       Spi_SetCS(M0P_SPI1, TRUE);
       
       ///< 判断发送的数据与接收的数据是否相等
       for(tmp = 0; tmp<10; tmp++)
       {
           if(rx_buf[tmp] == tx_buf[tmp])             
               continue;
           else
               break;
       }
       
       if(tmp == 10)                                    //如果接收到的数据与发送的数据相等则点亮板上LED
           Gpio_WriteOutputIO(EVB_LEDR_PORT, EVB_LEDR_PIN, TRUE); 
       

    while (1)
    {
    ///< LED点亮--  *((volatile uint32_t *)(0x40020DC8)) |= 0x04;   
   //0个是139ns 5个是305ns  --10个是442ns--20个是760ns===>1个是33ns   
    ///< LED关闭----   *((volatile uint32_t *)(0x40020DC8)) &= 0xFB;
   //     rainbow(45);     
 
    
    }
}
static void App_GpioInit(void)
{
    stc_gpio_cfg_t GpioInitStruct;
    DDL_ZERO_STRUCT(GpioInitStruct);
    
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);
    
    ///< SPI0引脚配置:主机
    GpioInitStruct.enDrv = GpioDrvH;
    GpioInitStruct.enDir = GpioDirOut;   

    Gpio_Init(EVB_SPI1_CS_PORT, EVB_SPI1_CS_PIN, &GpioInitStruct);
    Gpio_SetAfMode(EVB_SPI1_CS_PORT, EVB_SPI1_CS_PIN, GpioAf2);             ///<配置SPI0_CS
                                                               
    Gpio_Init(EVB_SPI1_SCK_PORT, EVB_SPI1_SCK_PIN, &GpioInitStruct);            
    Gpio_SetAfMode(EVB_SPI1_SCK_PORT, EVB_SPI1_SCK_PIN, GpioAf2);           ///<配置SPI0_SCK
                                                               
    Gpio_Init(EVB_SPI1_MOSI_PORT, EVB_SPI1_MOSI_PIN, &GpioInitStruct);           
    Gpio_SetAfMode(EVB_SPI1_MOSI_PORT, EVB_SPI1_MOSI_PIN, GpioAf2);         ///<配置SPI0_MOSI
                                                               
    GpioInitStruct.enDir = GpioDirIn;                          
    Gpio_Init(EVB_SPI1_MISO_PORT, EVB_SPI1_MISO_PIN, &GpioInitStruct);            
    Gpio_SetAfMode(EVB_SPI1_MISO_PORT, EVB_SPI1_MISO_PIN, GpioAf2);         ///<配置SPI0_MISO
    
    
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
    SpiInitStruct.enPclkDiv = SpiClkMskDiv128;    //波特率：PCLK/2
    SpiInitStruct.enCPHA    = SpiMskCphafirst;  //第一边沿采样
    SpiInitStruct.enCPOL    = SpiMskcpollow;    //极性为低
    Spi_Init(M0P_SPI1, &SpiInitStruct);
}


static void App_PcaInit(void)
{
    stc_pcacfg_t  PcaInitStruct;
    Sysctrl_SetPeripheralGate(SysctrlPeripheralPca, TRUE);
    
    PcaInitStruct.pca_clksrc = PcaPclkdiv2;//提高这个时钟有助于获得更稳定的PWM，感觉这个已经是最高的频率了
    PcaInitStruct.pca_cidl   = FALSE;
    PcaInitStruct.pca_ecom   = PcaEcomEnable;       //允许比较器功能
    PcaInitStruct.pca_capp   = PcaCappDisable;      //禁止上升沿捕获
    PcaInitStruct.pca_capn   = PcaCapnDisable;      //禁止下降沿捕获
    PcaInitStruct.pca_mat    = PcaMatEnable;        //禁止匹配功能
    PcaInitStruct.pca_tog    = PcaTogEnable;        //禁止翻转控制功能
    PcaInitStruct.pca_pwm    = PcaPwm8bitDisable;   //使能PWM控制输出
    PcaInitStruct.pca_epwm   = PcaEpwmEnable;       //禁止16bitPWM输出
    PcaInitStruct.pca_ccap   = 29;                  //当PCA模式用于PWM模式时，用于控制输出占空比装载寄存器
    PcaInitStruct.pca_carr   = 29;                  //占空比(carr-ccap)/carr*100% 计数周期重载寄存器
    //如果是8分频，周期值设置为7大概是750KHz
    Pca_M0Init(&PcaInitStruct);
//设置中断
    Pca_ClrItStatus(PcaCcf2);//PCA 计数器溢出标志清除;4个PCA计数器模块的比较/捕获标志位清除
    EnableNvic(PCA_IRQn, IrqLevel0, TRUE);//只有这个设置为优先级最高，其他的都比这个低
    Pca_ConfModulexIt(PcaModule0, TRUE);//比较捕获中断使能控制  PCA_CCAPMx的 CCIE
    Pca_ConfPcaIt(FALSE);  //当PCA计数溢出时，CF由硬件置位，如果CMOD寄存器的CFIE位为1，则CF标志可以产生中断

}



//时钟初始化配置
void App_ClkDivInit(void)
{
    //时钟分频设置
    Sysctrl_SetHCLKDiv(SysctrlHclkDiv1);
    Sysctrl_SetPCLKDiv(SysctrlPclkDiv1);
}


//端口配置，按键按下，继续运行
void App_PortCfg(void)
{
    stc_gpio_cfg_t stcGpioCfg;
    
    ///< 开启GPIO外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);    
    
    ///<========================== 按键端口配置 ===========================
    ///< 端口方向配置->输出
    stcGpioCfg.enDir = GpioDirIn;
    ///< 端口驱动能力配置->低驱动能力
    stcGpioCfg.enDrv = GpioDrvL;
    ///< 端口上下拉配置->上拉
    stcGpioCfg.enPu = GpioPuEnable;
    stcGpioCfg.enPd = GpioPdDisable;
    ///< 端口开漏输出配置->开漏输出关闭
    stcGpioCfg.enOD = GpioOdDisable;
    ///< GPIO IO 初始化(在STK上外接KEY(USER))
    Gpio_Init(EVB_KEY1_PORT, EVB_KEY1_PIN, &stcGpioCfg);
    ///< User KEY 按下后程序继续执行
//2021    _UserKeyWait();
    
    ///<========================== 时钟输出端口模式配置 ===========================
    ///< 端口方向配置->输出
    stcGpioCfg.enDir = GpioDirOut;
    ///< 端口驱动能力配置->高驱动能力
    stcGpioCfg.enDrv = GpioDrvH;
    ///< 端口上下拉配置->无上下拉
    stcGpioCfg.enPu = GpioPuDisable;
    stcGpioCfg.enPd = GpioPdDisable;
    ///< 端口开漏输出配置->开漏输出关闭
    stcGpioCfg.enOD = GpioOdDisable;
    ///< GPIO IO PA01初始化
    Gpio_Init(GpioPortA, GpioPin1, &stcGpioCfg);    
    ///< 配置PA01复用功能为HCLK输出
    Gpio_SetAfMode(GpioPortA, GpioPin1, GpioAf6);
    
    ///< 使能HCLK从PA01输出
    Gpio_SfHClkOutputCfg(GpioSfHclkOutEnable, GpioSfHclkOutDiv1);
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

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/



