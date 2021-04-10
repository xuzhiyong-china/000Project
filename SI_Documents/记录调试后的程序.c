使用华大官网的middware作为放置模块化程序的文件夹
Driver层禁止修改（防止改错，也便于以后随官网升级）

模块化程序：
    使用官网中的template为模板，但是can外设不行，要去除，才能编译过。
    我们写的代码，放在midware文件夹中。
    systemclock：主要来自官网例程的“example\sysctrl\sysctrl_clk_switch”
    ws2812：
        使用SPI0：
            模仿“example\spi\spi_master”
            改用使用的MCU有的Port引脚，并配置其功能为SPI（端口功能配置寄存器）--与官网例程一样，使用SPI0，只是引脚用别的。
        使用DMA：
            模仿“example\uart\uart_dma\source（串口+DMA）”调试SPI发送+DMA：
            使用了DMAch1
            DMA：使能DMA发送, DMA相关通道使能后，如果Buff为空，会立马启动传输。           
        使用资源：SPI0的PB05脚与DMAch1

定义位域、结构体、union 使用typedef 重命名一下，这样比较简单使用。

IAR有点变态：对书写的格式要求很严格。
if-else要这么写：
    if ()
    {
       ;
    }
    else
    {
        ;
    }









