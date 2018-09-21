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

//IO��������
#define SDA_IN()  sda_set(0)
#define SDA_OUT() sda_set(1)

//IO��������	 
#define IIC_SCL_0    GPIO_ResetBits(GPIOB,GPIO_Pin_6) //SCL
#define IIC_SCL_1    GPIO_SetBits(GPIOB,GPIO_Pin_6) //SCL

#define IIC_SDA_0    GPIO_ResetBits(GPIOB,GPIO_Pin_7) //SDA
#define IIC_SDA_1    GPIO_SetBits(GPIOB,GPIO_Pin_7) //SDA

#define READ_SDA   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7) //����SDA 

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 Init_AMG8833(void);
u8 get_data(void);

#ifdef __cplusplus
}
#endif

#endif
