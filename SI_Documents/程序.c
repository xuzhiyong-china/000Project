Tab按键用1个空格替代，否则不同IDE打开可能无法对齐。
注释要用英文，不然有的IDE打开是乱码，很难看.	
熟悉官网的例程以完成特定的功能。
外设的功能：主要是看寄存器的功能即可。

华大单片机新建工程：
    使用华大半导体提供的驱动库中的 template 进行修改，添加用户自己的文件。
    可以对文件夹重新命名：template修改为EWARM，然后更改原有的文件夹名字和工程的名字即可。
    为了便捷操作，为工程文件制作快捷方式。



重新指定C文件与头文件路径即可：
    添加C文件或者文件夹的方法：右键项目---选择add---选择add group、Files ---然后添加文件夹或者文件（先添加文件夹，再添加文件进去）
    指定头文件：右键工程---选择option，选择c/c++ compler---选择preprocessor---可以用绝对路径
更改工程的名字：https://blog.csdn.net/qq_20553613/article/details/86369282


在source文件夹中添加我们的代码。
Driver文件夹中的底层代码不要改动。
需要使用某个外设，在driver中添加对应的c文件即可。

编译要0警告。

位操作的宏
一、指定的某一位数置1

宏 #define setbit(x,y)  x|=(1<<y)
二、指定的某一位数置0
宏  #define clrbit(x,y)  x&=~(1<<y)

三、指定的某一位数取反

宏  #define reversebit(x,y)  x^=(1<<y)

四、获取的某一位的值

宏 #define getbit(x,y)   ((x) >> (y)&1)


使用结构体与联合体的方式，而不是使用多维数组（可读性）


https://blog.csdn.net/psr1999/article/details/103247481

联合体的参数共享同一个内存地址，所占的内存大小完全是由联合体中参数类型决定字长，然后数据共享，内存共享等。
https://blog.csdn.net/zzx1107/article/details/41653799
位域+联合体+结构体
    union cpp_union
    {
    struct
      {
        unsigned char b0:1;
        unsigned char b1:1;
        unsigned char b2:1;
        unsigned char b3:1;
        unsigned char b4:1;
        unsigned char b5:1;
        unsigned char b6:1;
        unsigned char b7:1;
      } bitfield;
      unsigned char bValue;
    } data;


    typedef struct
    {
        uint32_t RESERVED0                      : 2;
        __IO uint32_t ALIGN                     : 1;
        __IO uint32_t THCH                      : 5;
        __IO uint32_t DMASQR                    : 1;
        __IO uint32_t DMAJQR                    : 1;
        __IO uint32_t MODE                      : 1;
        __IO uint32_t RACCEN                    : 1;
        __IO uint32_t LTCMP                     : 1;
        __IO uint32_t HTCMP                     : 1;
        __IO uint32_t REGCMP                    : 1;
        __IO uint32_t RACCCLR                   : 1;
        uint32_t RESERVED16                     :15;
        __IO uint32_t RSV                       : 1;
    } stc_adc_cr1_field_t;

通过位域定义位变量，是实现单个位位操作的重要途径和方法，采用位域定义位变量，产生的代码紧凑、高效。

http://home.eeworld.com.cn/my/space-uid-231422-blogid-32187.html
 MISRA C 2004 规则



