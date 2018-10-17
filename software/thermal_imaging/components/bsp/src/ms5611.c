/*
 * This file is part of the 
 *
 * Copyright (c) 2016-2017 linghaibin
 *
 */
#include "ms5611.h"
#include "delay.h"

#define    MS561101BA_SlaveAddress   0xec          //定义器件在IIC总线中的从地址 

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

static uint16_t Cal_C[7];            //用于存放PROM中的8组数据
static uint32_t D1_Pres,D2_Temp;    // 存放压力和温度
static float dT,TEMP;
static double OFF_,SENS,T2;
static float Pressure;                //大气压
static float TEMP2,Aux,OFF2,SENS2;    //温度校验值

static uint32_t ex_Pressure;            //串口读数转换值
static uint8_t exchange_num[8];

static void sda_set(uint8_t val);

//IO方向设置
#define SDA_IN()  sda_set(0)
#define SDA_OUT() sda_set(1)

//IO操作函数 
#define IIC_SCL_0    GPIO_ResetBits(GPIOB,GPIO_Pin_10) //SCL
#define IIC_SCL_1    GPIO_SetBits(GPIOB,GPIO_Pin_10) //SCL

#define IIC_SDA_0    GPIO_ResetBits(GPIOB,GPIO_Pin_11) //SDA
#define IIC_SDA_1    GPIO_SetBits(GPIOB,GPIO_Pin_11) //SDA

#define READ_SDA   GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11) //输入SDA 

//IIC所有操作函数
static void iic_init(void);                //初始化IIC的IO口
static void iic_start(void);                //发送IIC开始信号
static void iic_stop(void);                  //发送IIC停止信号
static u8 iic_wait_ack(void);                 //IIC等待ACK信号
static void iic_ack(void);                    //IIC发送ACK信号
static void iic_nack(void);                //IIC不发送ACK信号
static void iic_send_byte(u8 txd);            //IIC发送一个字节
static u8 iic_read_byte(unsigned char ack);//IIC读取一个字节

//初始化IIC
static void iic_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(    RCC_APB2Periph_GPIOB, ENABLE );    
       
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11);     //PB10,PB11 输出高
}

static void sda_set(uint8_t val) {
    GPIO_InitTypeDef GPIO_InitStructure;
    if(val) {
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
    } else {
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU ;   //推挽输出
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
    }
}

//产生IIC起始信号
static void iic_start(void)
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
static void iic_stop(void)
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
static u8 iic_wait_ack(void)
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
            iic_stop();
            return 1;
        }
    }
    IIC_SCL_0;//时钟输出0
    return 0;  
} 
//产生ACK应答
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
//不产生ACK应答
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
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答              
static void iic_send_byte(u8 txd)
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
static u8 iic_read_byte(unsigned char ack)
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
        iic_nack();//发送nACK
    else
        iic_ack(); //发送ACK   
    return receive;
}

//=========================================================
//******MS561101BA程序********
//=========================================================
//重启MS5611
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

