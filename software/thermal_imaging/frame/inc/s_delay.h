/*
 * This file is part of the 
 *
 * Copyright (c) 2016-2017 linghaibin
 *
 */

#ifndef _S_DELAY_H_
#define _S_DELAY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <bsp.h>

void sdelay_us(uint32_t dly);
void sdelay_ms(uint32_t dly);

#ifdef __cplusplus
}
#endif

#endif
