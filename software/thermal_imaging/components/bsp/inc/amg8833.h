/*
 * This file is part of the 
 *
 * Copyright (c) 2016-2017 linghaibin
 *
 */

#ifndef _AMG8838_H_
#define _AMG8838_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp.h"

void sda_set(uint8_t val);

//IO方向设置
#define SDA_IN()  sda_set(0)
#define SDA_OUT() sda_set(1)

//IO操作函数	 
#define IIC_SCL_0    GPIO_ResetBits(GPIOB,GPIO_Pin_6) //SCL
#define IIC_SCL_1    GPIO_SetBits(GPIOB,GPIO_Pin_6) //SCL

#define IIC_SDA_0    GPIO_ResetBits(GPIOB,GPIO_Pin_7) //SDA
#define IIC_SDA_1    GPIO_SetBits(GPIOB,GPIO_Pin_7) //SDA

#define READ_SDA   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7) //输入SDA 

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 Init_AMG8833(void);
u8 get_data(void);

#ifdef __cplusplus
}
#endif

#endif
