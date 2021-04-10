

#ifndef __WS2812_H__
#define __WS2812_H__


/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "ddl.h"
#include "spi.h"
#include "gpio.h"
#include "reset.h"
#include "sysctrl.h"
#include "dmac.h"
#include "global.h"



/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif






















#define WS2812_number    1 







void WS2812_init(void);
void light_WS2812(void);
void WS2812_set_color(uchar *buffer,uchar buffer_length,uchar R,uchar G,uchar B);





//@}
#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
