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
void lcd_screen2(uint16_t xStart,uint16_t yStart,uint16_t xLong,uint16_t yLong,long * Color);
void lcd_screen(uint16_t xStart,uint16_t yStart,uint16_t xLong,uint16_t yLong, uint16_t * color);
void lcd_color_box(u16 xStart,u16 yStart,u16 xLong,u16 yLong,u16 Color);
void ILI9341_DispStringLine_EN_CH (  uint16_t line, char * pStr );

#ifdef __cplusplus
}
#endif

#endif
