
#include "amg8833.h"
#include "delay.h"

static void sda_set(uint8_t val);

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
       
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7);     //PB10,PB11 输出高
}

static void sda_set(uint8_t val) {
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
static void iic_start(void)
{
    SDA_OUT();     //sda线输出
    IIC_SDA_1;
    IIC_SCL_1;
    delay_us(2);
    IIC_SDA_0;//START:when CLK is high,DATA change form high to low 
    delay_us(2);
    IIC_SCL_0;//钳住I2C总线，准备发送或接收数据 
}
//产生IIC停止信号
static void iic_stop(void)
{
    SDA_OUT();//sda线输出
    IIC_SCL_0;
    IIC_SDA_0;//STOP:when CLK is high DATA change form low to high
    delay_us(2);
    IIC_SCL_1; 
    IIC_SDA_1;//发送I2C总线结束信号
    delay_us(2);
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
    delay_us(1);
    IIC_SCL_1;
    delay_us(1);
    IIC_SCL_0;
}
//不产生ACK应答
static void iic_nack(void)
{
    IIC_SCL_0;
    SDA_OUT();
    IIC_SDA_1;
    delay_us(1);
    IIC_SCL_1;
    delay_us(1);
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
        delay_us(1);   //对TEA5767这三个延时都是必须的
        IIC_SCL_1;
        delay_us(1); 
        IIC_SCL_0;    
        delay_us(1);
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
        delay_us(1);
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


u8 amg8833_init(void){
    iic_init();
    iic_start();
    iic_send_byte(0xD0);
    if(iic_wait_ack())return 0;
    iic_send_byte(0x00);
    if(iic_wait_ack())return 0;
    iic_send_byte(0x00);
    if(iic_wait_ack())return 0;
    iic_stop();
    delay_ms(10);
    iic_start();
    iic_send_byte(0xD0);
    if(iic_wait_ack())return 0;
    iic_send_byte(0x01);
    if(iic_wait_ack())return 0;
    iic_send_byte(0x3f);
    if(iic_wait_ack())return 0;
    iic_stop();
    delay_ms(10);
    iic_start();
    iic_send_byte(0xD0);
    if(iic_wait_ack())return 0;
    iic_send_byte(0x02);
    if(iic_wait_ack())return 0;
    iic_send_byte(0x00);
    if(iic_wait_ack())return 0;
    iic_stop();

    delay_ms(50);
    get_data();
    return 1;
}

uint16_t data[40][40];
uint16_t ext[3];
u8 ext_add[2];

u8 get_data(void) {
    int i;
    uint16_t buf;
    ext[0]=0;
    ext[1]=0x7fff;
    iic_start();
    iic_send_byte(0xD0);
    if(iic_wait_ack())return 0;
    iic_send_byte(0x80);
    if(iic_wait_ack())return 0;
    iic_start();
    iic_send_byte(0xD1);
    if(iic_wait_ack())return 0;
    for (i = 0; i < 63; i++) {
        buf=iic_read_byte(1)&0xff;
        buf|=(0xff&iic_read_byte(1))<<8;
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
    
    buf=iic_read_byte(1)&0xff;
    buf|=(0xff&iic_read_byte(0))<<8;
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
    iic_stop();
    
//    int n = 40;
//    for(int i = 0;2*i<40;i++) {
//        for(int j = 0;2*j<40-1;j++){
//            uint16_t temp = data[i][j];
//            data[i][j] = data[j][n-i-1];
//            data[j][n-i-1] = temp;
//            
//            temp = data[i][j];
//            data[i][j] = data[n-i-1][n-j-1];
//            data[n-i-1][n-j-1] = temp;
//            
//            temp = data[i][j];
//            data[i][j] = data[n-j-1][i];
//            data[n-j-1][i] = temp;
//        }
//    }
    
    int len = 40;
    for (uint16_t i = 0; i < len; i++)
    {
        for (uint16_t j = i+1; j < len; j++)
        {
            uint16_t temp = data[i][j];
            data[i][j] = data[j][i];
            data[j][i] = temp;
        }
    }
    //对称
    for (uint16_t i = 0; i < len; i++)
    {
        for (uint16_t j = 0; j < len / 2; j++)
        {
            uint16_t temp = data[i][j];
            data[i][j] = data[i][len-j-1];
            data[i][len-j-1] = temp;
        }
    }
    
    return 1;
}
