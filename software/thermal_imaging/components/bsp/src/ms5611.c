/*
 * This file is part of the 
 *
 * Copyright (c) 2016-2017 linghaibin
 *
 */
#include "ms5611.h"
#include "delay.h"

#define    MS561101BA_SlaveAddress   0xec          //����������IIC�����еĴӵ�ַ 

#define    MS561101BA_D1      0x40
#define    MS561101BA_D2      0x50
#define    MS561101BA_RST     0x1E

//#define    MS561101BA_D1_OSR_256    0x40
//#define    MS561101BA_D1_OSR_512    0x42
//#define    MS561101BA_D1_OSR_1024   0x44
//#define    MS561101BA_D1_OSR_2048   0x46
#define    MS561101BA_D1_OSR_4096   0x48

//#define    MS561101BA_D2_OSR_256    0x50
//#define    MS561101BA_D2_OSR_512    0x52
//#define    MS561101BA_D2_OSR_1024   0x54
//#define    MS561101BA_D2_OSR_2048   0x56
#define    MS561101BA_D2_OSR_4096   0x58

#define MS561101BA_ADC_RD     0x00
#define    MS561101BA_PROM_RD       0xA0
#define MS561101BA_PROM_CRC   0xAE

static uint16_t Cal_C[7];            //���ڴ��PROM�е�8������
static uint32_t D1_Pres,D2_Temp;    // ���ѹ�����¶�
static float dT,TEMP;
static double OFF_,SENS,T2;
static float Pressure;                //����ѹ
static float TEMP2,Aux,OFF2,SENS2;    //�¶�У��ֵ

static uint32_t ex_Pressure;            //���ڶ���ת��ֵ
static uint8_t exchange_num[8];

static void sda_set(uint8_t val);

//IO��������
#define SDA_IN()  sda_set(0)
#define SDA_OUT() sda_set(1)

//IO�������� 
#define IIC_SCL_0    GPIO_ResetBits(GPIOB,GPIO_Pin_10) //SCL
#define IIC_SCL_1    GPIO_SetBits(GPIOB,GPIO_Pin_10) //SCL

#define IIC_SDA_0    GPIO_ResetBits(GPIOB,GPIO_Pin_11) //SDA
#define IIC_SDA_1    GPIO_SetBits(GPIOB,GPIO_Pin_11) //SDA

#define READ_SDA   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11) //����SDA 

//IIC���в�������
static void iic_init(void);                //��ʼ��IIC��IO��
static void iic_start(void);                //����IIC��ʼ�ź�
static void iic_stop(void);                  //����IICֹͣ�ź�
static u8 iic_wait_ack(void);                 //IIC�ȴ�ACK�ź�
static void iic_ack(void);                    //IIC����ACK�ź�
static void iic_nack(void);                //IIC������ACK�ź�
static void iic_send_byte(u8 txd);            //IIC����һ���ֽ�
static u8 iic_read_byte(unsigned char ack);//IIC��ȡһ���ֽ�

//��ʼ��IIC
static void iic_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(    RCC_APB2Periph_GPIOB, ENABLE );    
       
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11);     //PB10,PB11 �����
}

static void sda_set(uint8_t val) {
    GPIO_InitTypeDef GPIO_InitStructure;
    if(val) {
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
    } else {
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;   //�������
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
    }
}

//����IIC��ʼ�ź�
static void iic_start(void)
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
static void iic_stop(void)
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
static u8 iic_wait_ack(void)
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
            iic_stop();
            return 1;
        }
    }
    IIC_SCL_0;//ʱ�����0
    return 0;  
} 
//����ACKӦ��
static void iic_ack(void)
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
static void iic_nack(void)
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
static void iic_send_byte(u8 txd)
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
static u8 iic_read_byte(unsigned char ack)
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
        iic_nack();//����nACK
    else
        iic_ack(); //����ACK   
    return receive;
}

//=========================================================
//******MS561101BA����********
//=========================================================
//����MS5611
void MS561101BA_RESET()
{    
    iic_start();
    iic_send_byte(MS561101BA_SlaveAddress);
    if(iic_wait_ack())return;
    iic_send_byte(MS561101BA_RST);
    if(iic_wait_ack())return;
    iic_stop();
}

//
void MS561101BA_PROM_READ()
{
    uint8_t d1,d2,i;
    for(i=0;i<=6;i++)
    {
        iic_start();
        iic_send_byte(MS561101BA_SlaveAddress);
        iic_send_byte((MS561101BA_PROM_RD+i*2));


        iic_start();
        iic_send_byte(MS561101BA_SlaveAddress+1);
        d1=iic_read_byte(0);
        d2=iic_read_byte(1);
        iic_stop();

        delay_us(5);

        Cal_C[i]=((uint16_t)d1<<8)|d2;
    }
}

uint32_t MS561101BA_DO_CONVERSION(uint8_t command)
{
    uint32_t conversion=0;
    uint32_t conv1,conv2,conv3; 
    iic_start();
    iic_send_byte(MS561101BA_SlaveAddress);
    iic_send_byte(command);
    iic_stop();

    delay_ms(0xffff);

    iic_start();
    iic_send_byte(MS561101BA_SlaveAddress);
    iic_send_byte(0);

    iic_start();
    iic_send_byte(MS561101BA_SlaveAddress+1);
    conv1=iic_read_byte(0);
    conv2=iic_read_byte(0);
    conv3=iic_read_byte(1);
    iic_stop();

    conversion=conv1*65535+conv2*256+conv3;

    return conversion;
}



void MS561101BA_getTemperature(uint8_t OSR_Temp)
{
    
    D2_Temp= MS561101BA_DO_CONVERSION(OSR_Temp);
    delay_ms(10); 
    dT=D2_Temp - (((uint32_t)Cal_C[5])<<8);
    TEMP=2000+dT*((uint32_t)Cal_C[6])/8388608;

}

void MS561101BA_getPressure(uint8_t OSR_Pres)
{
    
    D1_Pres= MS561101BA_DO_CONVERSION(OSR_Pres);
    delay_ms(10); 
    OFF_=(uint32_t)Cal_C[2]*65536+((uint32_t)Cal_C[4]*dT)/128;
    SENS=(uint32_t)Cal_C[1]*32768+((uint32_t)Cal_C[3]*dT)/256;
    
    if(TEMP<2000)
    {
        // second order temperature compensation when under 20 degrees C
        T2 = (dT*dT) / 0x80000000;
        Aux = TEMP*TEMP;
        OFF2 = 2.5*Aux;
        SENS2 = 1.25*Aux;
        TEMP = TEMP - TEMP2;
        OFF_ = OFF_ - OFF2;
        SENS = SENS - SENS2;
    }

    Pressure=(D1_Pres*SENS/2097152-OFF_)/32768;
}


void ms5611_init(void) {
    iic_init();
    MS561101BA_RESET();
    delay_ms(100);
    MS561101BA_PROM_READ();
    delay_ms(100);
}


#define         HDC_1080_ADD                            0x40
#define         Configuration_register_add              0x02
#define         Temperature_register_add                0x00
#define         Humidity_register_add                   0x01

typedef enum
{
  Temperature_Resolution_14_bit = 0,
  Temperature_Resolution_11_bit = 1
}Temp_Reso;

typedef enum
{
  Humidity_Resolution_14_bit = 0,
  Humidity_Resolution_11_bit = 1,
  Humidity_Resolution_8_bit =2
}Humi_Reso;

void hdc1080_init(void) {
    iic_init();
}

