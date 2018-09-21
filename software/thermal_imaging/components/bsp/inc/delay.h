/*
 * This file is part of the 
 *
 * Copyright (c) 2016-2017 linghaibin
 *
 */

#ifndef _DELAY_H_
#define _DELAY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp.h"

void delay_init(void);
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);

#ifdef __cplusplus
}
#endif

#endif
