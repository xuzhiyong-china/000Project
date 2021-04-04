 安装IAR时，要选择安装STlink。
 IAR的debugger选择STlink并且选择为SWD接口，降低SWD的下载速度，确保每次烧录都能通讯上。

 华大的例程有用IIC中断方式操作EEPROM
 	

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






