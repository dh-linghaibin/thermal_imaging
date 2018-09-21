/*
 * This file is part of the 
 *
 * Copyright (c) 2016-2017 linghaibin
 *
 */

#ifndef _bsp_interface_H_
#define _bsp_interface_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"

/*
gpio
*/

/*
iic
*/

/*
spi
*/

/*
rtc
*/
typedef struct _rtc_time_obj {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t week;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} rtc_time_obj;

/*
adc
*/

/*
dac
*/

/*
usart
*/

/*
can
*/

#ifdef __cplusplus
}
#endif
 
#endif
