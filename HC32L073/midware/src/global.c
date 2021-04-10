/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "global.h"

uchar Get_Bit(uchar data, uchar offset)
{
    return (((data>>offset)&1u)>0?1:0);
    
}
void Set_Bit(uchar data, uchar offset, uchar one_zero)
{
    if(1 == one_zero)
    {
        (data) |= ((1UL)<<(offset));
    }
    else
    {
        (data) &= (~(1UL<<(offset)));
    }    
    
    
}




