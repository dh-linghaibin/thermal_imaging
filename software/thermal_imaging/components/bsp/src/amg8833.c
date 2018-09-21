
#include "amg8833.h"
#include "delay.h"

//��ʼ��IIC
void IIC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7); 	//PB10,PB11 �����
}

void sda_set(uint8_t val) {
    GPIO_InitTypeDef GPIO_InitStructure;
    if(val) {
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
    } else {
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;   //�������
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
    }
}

//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA_1;	  	  
	IIC_SCL_1;
	delay_us(4);
 	IIC_SDA_0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL_0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL_0;
	IIC_SDA_0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL_1; 
	IIC_SDA_1;//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
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
	IIC_SCL_0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
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
//������ACKӦ��		    
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
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{
    u8 t;
	SDA_OUT();
    IIC_SCL_0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {
        //IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			IIC_SDA_1;
		else
			IIC_SDA_0;
		txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL_1;
		delay_us(2); 
		IIC_SCL_0;	
		delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
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
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
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
