/*
 * This file is part of the 
 *
 * Copyright (c) 2016-2017 linghaibin
 *
 */

#ifndef _LCD_H_
#define _LCD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp.h"

void lcd_init(void);
void lcd_screen(uint16_t xStart,uint16_t yStart,uint16_t xLong,uint16_t yLong,long * Color);


#ifdef __cplusplus
}
#endif

#endif
