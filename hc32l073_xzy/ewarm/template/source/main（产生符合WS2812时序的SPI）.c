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
#include "gpio.h"
#include "reset.h"



/******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define WS2812_high() (*((volatile uint32_t *)(0x40020DC8)) |= 0x04)
#define WS2812_low()  (*((volatile uint32_t *)(0x40020DC8)) &= 0xFB)
#define _nop_() (1+1)




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

#define numLEDs 8   //灯的个数
unsigned char buf_R[numLEDs] = {0};//颜色缓存
unsigned char buf_G[numLEDs] = {0};
unsigned char buf_B[numLEDs] = {0};

//1码，高电平850ns 低电平400ns 误差正负150ns
void RGB_Set_Up()
{
    uint32_t i=0;
    WS2812_high();
	//经过逻辑分析仪调试的的延时--850ns(//0个大概是139ns 以后1个是33ns)
    i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;
    i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;
    WS2812_low() ;
}

//1码，高电平400ns 低电平850ns 误差正负150ns
void RGB_Set_Down()
{
    uint32_t i=0;
    WS2812_high();
    //经过逻辑分析仪调试的的延时--400ns
    i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;
    WS2812_low() ;
}
void Delay50us()	
{
	uint32_t i, j;
    for(i=0;i<200;i++) { j+=1;j+=1;j+=1;}//示波器延时50us
}
//复位码
void RGB_Rst()
{
    WS2812_low() ;
    Delay50us();
}

//发送24位数据
void Send_2811_24bits(unsigned char G8,unsigned char R8,unsigned char B8)
{
	  
	  unsigned int n = 0;
	  //发送G8位
		for(n=0;n<8;n++)
		{
			G8<<=n;
			if(G8&0x80 == 0x80)
			{
				RGB_Set_Up();
			}
			else  
			{
			  RGB_Set_Down();
			}
		}
		//发送R8位
		for(n=0;n<8;n++)
		{
			R8<<=n;
			if(R8&0x80 == 0x80)
			{
				RGB_Set_Up();
			}
			else  
			{
				RGB_Set_Down();
			}
			
		}
		//发送B8位
	  for(n=0;n<8;n++)
		{
			B8<<=n;
			if(B8&0x80 == 0x80)
			{
				RGB_Set_Up();
			}
			else  
			{
			  RGB_Set_Down();
			}
		}
}
//某一个点显示的颜色
void SetPointColour(unsigned int num,unsigned char r,unsigned char g,unsigned char b)
{
	  unsigned char i;
		for(i=0;i<numLEDs;i++)
	  {
				buf_R[num] = r;//缓冲
			  buf_G[num] = g;
			  buf_B[num] = b;
		}
		for(i=0;i<numLEDs;i++)
		{
			Send_2811_24bits(buf_G[i],buf_R[i],buf_B[i]);//发送显示
		}
}
//颜色交换24位不拆分发
void SetPixelColor(unsigned char num,unsigned long c)
{
	  unsigned char i;
		for(i=0;i<numLEDs;i++)
	  {
				buf_R[num] = (unsigned char)(c>>16);
			  buf_G[num] = (unsigned char)(c>>8);
			  buf_B[num] = (unsigned char)(c);
		}
		for(i=0;i<numLEDs;i++)
		{
			Send_2811_24bits(buf_G[i],buf_R[i],buf_B[i]);
		}
}
//颜色
unsigned long Color(unsigned char r, unsigned char g, unsigned char b)
{
  return ((unsigned long)r << 16) | ((unsigned long)g <<  8) | b;
}

//颜色算法
unsigned long Wheel(unsigned char WheelPos)
{
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) 
	{
    return Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

//复位
void PixelUpdate()
{
	RGB_Rst();
}
//彩虹
void rainbow(unsigned int wait)
{
  unsigned int i, j;

  for(j=0; j<256; j++) 
	{
    for(i=0; i<numLEDs; i++)
		{
      SetPixelColor(i, Wheel((i+j) & 255));
    }
		PixelUpdate();
 //   HAL_Delay(wait);
  }
}
unsigned int debug;



unsigned char n,G8;
#define getbit(x,y)   ((x) >> (y)&1)
//SPI


static void App_SPIInit(void);

const uint8_t tx_buf[10]={0xE0};
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
    Spi_Init(M0P_SPI0, &SpiInitStruct);
}

//spi
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
    
    //时钟分频初始化
    App_ClkDivInit();    
    //时钟切换    
  //  _UserKeyWait(); //USER KEY 按下后继续执行
    App_SystemClkInit_XTH(SysctrlXthFreq4_8MHz);    
  //  _UserKeyWait(); //USER KEY 按下后继续执行
    App_SystemClkInit_PLL48M_byXTH();   

///< LED端口初始化

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
   Spi_SetCS(M0P_SPI0, FALSE);
   ///< 主机向从机发送数据
 while(1)
   Spi_SendBuf(M0P_SPI0, (uint8_t*)tx_buf, 10);   
   ///< 结束通信
   Spi_SetCS(M0P_SPI0, TRUE);

   delay1ms(1);

   ///< 片选，开始通讯
   Spi_SetCS(M0P_SPI0, FALSE);
   ///< 主机接收从机数据
   Spi_ReceiveBuf(M0P_SPI0, rx_buf, 10);
   ///< 结束通信
   Spi_SetCS(M0P_SPI0, TRUE);
   
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
   
   while(1);


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

/******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/



