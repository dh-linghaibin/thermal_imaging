
#include "amg8833.h"
#include "delay.h"

//初始化IIC
void IIC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7); 	//PB10,PB11 输出高
}

void sda_set(uint8_t val) {
    GPIO_InitTypeDef GPIO_InitStructure;
    if(val) {
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
    } else {
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;   //推挽输出
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
    }
}

//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA_1;	  	  
	IIC_SCL_1;
	delay_us(4);
 	IIC_SDA_0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL_0;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL_0;
	IIC_SDA_0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL_1; 
	IIC_SDA_1;//发送I2C总线结束信号
	delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA_1;delay_us(1);	   
	IIC_SCL_1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void IIC_Ack(void)
{
	IIC_SCL_0;
	SDA_OUT();
	IIC_SDA_0;
	delay_us(2);
	IIC_SCL_1;
	delay_us(2);
	IIC_SCL_0;
}
//不产生ACK应答		    
void IIC_NAck(void)
{
	IIC_SCL_0;
	SDA_OUT();
	IIC_SDA_1;
	delay_us(2);
	IIC_SCL_1;
	delay_us(2);
	IIC_SCL_0;
}
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{
    u8 t;
	SDA_OUT();
    IIC_SCL_0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {
        //IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			IIC_SDA_1;
		else
			IIC_SDA_0;
		txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		IIC_SCL_1;
		delay_us(2); 
		IIC_SCL_0;	
		delay_us(2);
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL_0; 
        delay_us(2);
		IIC_SCL_1;
        receive<<=1;
        if(READ_SDA)receive++;
		delay_us(1); 
    }
    if (!ack)
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}


u8 Init_AMG8833(void){
	IIC_Start();
	IIC_Send_Byte(0xD0);
	if(IIC_Wait_Ack())return 0;
	IIC_Send_Byte(0x00);
	if(IIC_Wait_Ack())return 0;
	IIC_Send_Byte(0x00);
	if(IIC_Wait_Ack())return 0;
	IIC_Stop();
	delay_ms(10);
	IIC_Start();
	IIC_Send_Byte(0xD0);
	if(IIC_Wait_Ack())return 0;
	IIC_Send_Byte(0x01);
	if(IIC_Wait_Ack())return 0;
	IIC_Send_Byte(0x3f);
	if(IIC_Wait_Ack())return 0;
	IIC_Stop();
	delay_ms(10);
	IIC_Start();
	IIC_Send_Byte(0xD0);
	if(IIC_Wait_Ack())return 0;
	IIC_Send_Byte(0x02);
	if(IIC_Wait_Ack())return 0;
	IIC_Send_Byte(0x00);
	if(IIC_Wait_Ack())return 0;
	IIC_Stop();

	delay_ms(50);
	get_data();
	return 1;
}





long data[40][40];
long ext[3];
u8 ext_add[2];



u16 test_table[8][8];


u8 get_data(void) {
	int i;
	long buf;
	ext[0]=0;
	ext[1]=0x7fff;
	IIC_Start();
	IIC_Send_Byte(0xD0);
	if(IIC_Wait_Ack())return 0;
	IIC_Send_Byte(0x80);
	if(IIC_Wait_Ack())return 0;
	IIC_Start();
	IIC_Send_Byte(0xD1);
	if(IIC_Wait_Ack())return 0;
	for (i = 0; i < 63; i++) {
		buf=IIC_Read_Byte(1)&0xff;
		buf|=(0xff&IIC_Read_Byte(1))<<8;
 		if((buf&0x800)==0x800){
			buf=~buf;
			buf&=0xFFF;
			buf++;
			buf=-buf;
		}
		else if(buf>0x200){
			buf=0x200;
		}
		if(buf>ext[0]){
			ext[0]=buf;
			ext_add[0]=i;
		}
		if(buf<ext[1]){
			ext[1]=buf;
			ext_add[1]=i;
		}
		data[39-(i / 8 * 5 + 2)][i * 5 % 40 + 2] = buf;
	}
	buf=IIC_Read_Byte(1)&0xff;
	buf|=(0xff&IIC_Read_Byte(0))<<8;
 		if((buf&0x800)==0x800){
			buf=~buf;
			buf&=0xFFF;
			buf++;
			buf=-buf;
		}
		else if(buf>0x200){
			buf=0x200;
		}
	if(buf>ext[0]){
		ext[0]=buf;
		ext_add[0]=i;
	}
	if(buf<ext[1]){
		ext[1]=buf;
		ext_add[1]=i;
	}
	data[39-(i / 8 * 5 + 2)][i * 5 % 40 + 2] = buf;
	IIC_Stop();
	
	return 1;
	
}
