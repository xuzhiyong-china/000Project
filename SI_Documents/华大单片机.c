 安装IAR时，要选择安装STlink。
 IAR的debugger选择STlink并且选择为SWD接口，降低SWD的下载速度，确保每次烧录都能通讯上。

 华大的例程有用IIC中断方式操作EEPROM
 使用外设的功能：主要是查看寄存器的功能与看官网的模块文件。	

华大库手册：

华大单片机时钟需要配合晶振进行修改：
	重点查看时钟的代码
	 //更新Core时钟（HCLK）
	 SystemCoreClockUpdate();
 	获得系统时钟（HCLK）频率值--uint32_t Sysctrl_GetHClkFreq(void)
	时钟去初始化函数--en_result_t Sysctrl_ClkDeInit(void)
//< 时钟频率获取：根据系统需要，获取当前HCLK及PCLK的频率值
uint32_t Sysctrl_GetHClkFreq(void);
uint32_t Sysctrl_GetPClkFreq(void);

	en_sysctrl_clk_source--系统时钟输入源类型定义
	en_sysctrl_xtal_driver--XTAL驱动能力类型定义
	en_sysctrl_xth_freq--XTH频率值范围选择类型定义
	en_sysctrl_xth_cycle--XTH时钟稳定周期数类型定义
	en_sysctrl_xtl_amp--XTL晶体振幅枚举类型定义
	en_sysctrl_pll_cycle--PLL时钟稳定周期数类型定义
	en_sysctrl_pll_infreq--PLL输入频率范围类型定义
	en_sysctrl_pll_outfreq--PLL输出频率范围类型定义
	en_sysctrl_pll_clksource--PLL输入时钟源类型定义
	en_sysctrl_pll_mul--PLL输入时钟源类型定义（倍频设置）
	en_sysctrl_hclk_div--HCLK时钟分频系数类型定义
	en_sysctrl_pclk_div--PCLK分频系数
	en_sysctrl_usbclk_sel--USB时钟选择数据类型定义
	en_sysctrl_timer_pllclk_sel--定时器时钟选择数据类型定义
	en_sysctrl_peripheral_gate--所有的外设时钟门控开关类型枚举
	
		
	en_sysctrl_func--  
		SysctrlEXTHEn           = 1u,                    ///< 使能外部高速时钟从输入引脚输入
	    SysctrlEXTLEn           = 2u,                    ///< 使能外部低速速时钟从输入引脚输入
	    SysctrlXTLAlwaysOnEn    = 3u,                    ///< 使能后XTL_EN只可置位
	    SysctrlClkFuncRTCLpmEn  = 5u,                    ///< 使能RTC低功耗模式
	    SysctrlCMLockUpEn       = 6u,                    ///< 使能后CPU执行无效指令会复位MCU
	    SysctrlSWDUseIOEn       = 8u,                    ///< SWD端口设为IO功能
		

系统时钟：
规格书提到“内建PLL支持8~48MHz的时钟输出”所以推测他的最高时钟只能到48MHz
PLL控制寄存器：选择XTH晶振生成的时钟为时钟源
XTH---PLL---SystemClk不分频---HCLK---不分频直接给PCLK：所以时钟都是48MHz
高级定时器4有和DMA联系的说明。

定时器的比较功能：
    在计数值与计数基准值比较匹配时输出指定的电平。
    GCMAR、GCMBR 寄存器分别对应了 CHxA、CHxB的计数比较基准值。
    当计数器的计数值和 GCMAR 相等时，CHxA 端口输出指定的电平；
    当计数器的计数值和 GCMBR 相等时，CHxB 端口输出指定电平。
    CHxA、CHxB 端口的计数起始电平、停止电平、计数比较匹配时的电平都可以设置。





华大单片机：
    参考“ example\sysctrl\sysctrl_clk_switch ” 设置系统的时钟
    例程当中可以切换时钟并使用示波器查看系统时钟。
    工程中自带精准的延时函数。
    修改点：
        App_SystemClkInit_XTH(SysctrlXthFreq4_8MHz);--选用XTH时钟范围,其他的删除   
        App_SystemClkInit_PLL48M_byXTH();   ---输入时钟源选择为晶振口 --mark：//xzy20210406
所以工程的起点是这个例程下的main函数。  
    仿真查看变量：SystemCoreClock 变为48MHz

使用“ pca\pca_16bit_pwm\source ”的例程生成800KHz的例程
    修改点：
        PcaInitStruct.pca_clksrc = PcaPclkdiv2;--PCLK的分频系数
        PcaInitStruct.pca_ccap   = 15;// 5是80%；8是70%；9是67%；18是37%；19是33%；20是30%
        PcaInitStruct.pca_carr   = 29; //正好是800KHz

        PcaInitStruct.pca_clksrc = PcaPclkdiv32;
        



        
        32%高电平是 0--400ns的高
        68%高电平是 1--850ns的高
        reset:50us以上。
    先用PCA生成PWM，每次要量测占空比。后面可以考虑直接使用模拟的方式。
    （这种时间更难操作）使用模拟的方式控制彩灯：https://www.itdaan.com/blog/2017/12/13/a96fcf5ce3937c60a9d208365d9f08d3.html  
    需要使用操作寄存器的方式，不然速度慢：
          端口 Px 输出值配置寄存器(PxOUT) (x = A,B,C,D,E,F)  ：
            地址偏移量：0x108(PA),0x148(PB),0x188(PC),0x1C8(PD),0x1108(PE),0x1148(PF) 
            0x1c8 PDOUT RW 端口 PD 输出值配置寄存器
          华大是用地址操作的：
            基地址加偏移地址：端口控制器寄存器描述中“  基地址 1：0x40020C00    ”
          最终操作IO高低电平用：0x40020C00+0x1C8=0x40020DC8
             *((volatile uint32_t *)(0x40020DC8)) |= 0x04;--高点评
              *((volatile uint32_t *)(0x40020DC8)) &= 0xFB;-低电平
              可以达到2.5MHz（使用官网例程，只能达到500Khz）
PCA_CCON：
需要先使能PCA计数器中断使能控制信号
CCIE
溢出中断：当PCA计数溢出时，CF由硬件置位，如果CMOD寄存器的CFIE位为1，则CF标志可以产生中断--计数器计数到与寄存器 CARR 的值相同时溢出后计数器的值变为 0
匹配中断：当出现匹配或捕获时，该位由硬件置位，这个标志位会产生一个PCA中断
启动PCA计数器计数


void PCA_IRQHandler(void)

切换占空比： Pca_IRQHandler 函数
频率会跳，是因为没有设置周期指吗？还是要重新初始化一下pca


