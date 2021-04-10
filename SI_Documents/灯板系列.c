华大的例程中有虚拟串口的，用它完成与上位机的通讯


原来的方式是DCDC加排线成本      
    大于直接使用12P的XH红白线


每行的灯珠，最多多少个？
    电源layout的考量：1oz的印制板，1mm线宽通过电流的经验值约为1A。
    （2oz的比1oz的每平米贵150元，算下来靠增加铜厚成本贵）
    （焊盘开窗加锡的方式过大电流）
    XH排线的考量：灯珠的数量。
    

看一下当前板子的长度，计算一下用多长的排线，长度要设计得正好，不要松松垮垮的。



排线的淘宝卖家
https://china-ms.taobao.com/search.htm?search=y&keyword=%C5%C5%CF%DF&lowPrice=&highPrice=



相同横截面积，单股硬线比多股软线寿命长。

明纬电源5V：
150w，99元    https://detail.tmall.com/item.htm?spm=a1z10.3-b-s.w4011-23472802678.150.4b803f86s91FH4&id=547672680626&rn=cb7908a1b7f30fb042adb2d8f6c1cc9a&abbucket=1





华大的例程是PA06，改为PB04：
    Gpio_Init(GpioPortB, GpioPin4, &GpioInitStruct);
    Gpio_SetAfMode(GpioPortB, GpioPin4, GpioAf2);
    

    
