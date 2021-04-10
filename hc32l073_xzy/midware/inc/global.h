

#ifndef __GLOBAL_H__
#define __GLOBAL_H__


/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/



/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
#define uchar unsigned char
 typedef struct {
    uchar bit0:1;
    uchar bit1:1;
    uchar bit2:1;
    uchar bit3:1;
    uchar bit4:1;
    uchar bit5:1;
    uchar bit6:1;
    uchar bit7:1;   
    }bit8;

#define SET_ONE_BIT(REG, BIT)     ((REG) |= (BIT))

//#define CPL_BIT(value,bit) (value^=(1<<bit))   //取反指定位
//#define SET_BIT(value,bit) (value|=(1<<bit))   //置位指定位
//#define CLR_BIT(value,bit) (value&=~(1<<bit))  //清零指定位
//#define GET_BIT(value,bit) (value&(1<<bit))    //读取指定位



typedef union {
    bit8 bit;
    uchar byte;
    } uchar_to_bit;//
/*
uchar_to_bit temp;
uchar a;
temp.byte=a;//IAR无法强转，需要这样赋值转一下，不然会报错。
//这样才能对a 用位域来使用


*/



void Set_Bit(uchar data, uchar offset, uchar one_zero);
uchar Get_Bit(uchar data, uchar offset);








//@}
#ifdef __cplusplus
}
#endif

#endif 
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
