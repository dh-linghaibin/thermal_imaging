

#include "lcd.h"

#define Bank1_LCD_D    ((u32)0x60020000)    //Disp Data ADDR
#define Bank1_LCD_C    ((u32)0x60000000)	  //Disp Reg  ADDR


#define Lcd_Light_ON   GPIO_SetBits(GPIOD,GPIO_Pin_12);
#define Lcd_Light_OFF  GPIO_ResetBits(GPIOD,GPIO_Pin_12);

#define Set_Rst GPIO_SetBits(GPIOD,GPIO_Pin_13);
#define Clr_Rst GPIO_ResetBits(GPIOD,GPIO_Pin_13);

void LCD_GPIO_Config(void)
{ 
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable the FSMC Clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

    /* config lcd gpio clock base on FSMC */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE , ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* config tft rst gpio */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;		
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* config tft back_light gpio base on the PT4101 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ; 	 
    GPIO_Init(GPIOD, &GPIO_InitStructure);  		   

    /* config tft data lines base on FSMC
    * data lines,FSMC-D0~D15: PD 14 15 0 1,PE 7 8 9 10 11 12 13 14 15,PD 8 9 10
    */	
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9 | 
                          GPIO_Pin_10 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | 
                          GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | 
                          GPIO_Pin_15;
    GPIO_Init(GPIOE, &GPIO_InitStructure); 

    /* config tft control lines base on FSMC
    * PD4-FSMC_NOE  :LCD-RD
    * PD5-FSMC_NWE  :LCD-WR
    * PD7-FSMC_NE1  :LCD-CS
    * PD11-FSMC_A16 :LCD-DC
    */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; 
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; 
    GPIO_Init(GPIOD, &GPIO_InitStructure);  

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 
    GPIO_Init(GPIOD, &GPIO_InitStructure);  

    /* tft control gpio init */	 

    GPIO_SetBits(GPIOD, GPIO_Pin_13);		 // RST = 1 
    GPIO_SetBits(GPIOD, GPIO_Pin_4);		 // RD = 1  
    GPIO_SetBits(GPIOD, GPIO_Pin_5);		 // WR = 1 
    GPIO_SetBits(GPIOD, GPIO_Pin_7);		 //	CS = 1 

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		
    GPIO_Init(GPIOA, &GPIO_InitStructure);
                                                        // right          up           left        down
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_1 | GPIO_Pin_5 | GPIO_Pin_6;		
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}

static void LCD_FSMC_Config(void)
{
    FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef  p; 


    p.FSMC_AddressSetupTime = 0x00;	 //地址建立时间
    p.FSMC_AddressHoldTime = 0x00;	 //地址保持时间
    p.FSMC_DataSetupTime = 0x01;		 //数据建立时间
    p.FSMC_BusTurnAroundDuration = 0x00;
    p.FSMC_CLKDivision = 0x00;
    p.FSMC_DataLatency = 0x00;

    p.FSMC_AccessMode = FSMC_AccessMode_B;	 // 一般使用模式B来控制LCD

    FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;
    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
    FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_NOR;
    FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
    FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
    FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
    FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p; 

    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 

    /* Enable FSMC Bank1_SRAM Bank */
    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);  
}


//void DataToWrite(u16 data) 
//{
//	GPIOB->ODR=((GPIOB->ODR&0x00ff)|(data&0xff00));	
//	Clr_nWr;
//	Set_nWr;
//	GPIOB->ODR=((GPIOB->ODR&0x00ff)|(data<<8));
//}


volatile static void Delay(__IO u32 nCount)
{	
	volatile int i;
	for(i=0;i<7200;i++)
    for(; nCount != 0; nCount--);
}  

static void LCD_Rst(void)
{			
    Clr_Rst;
    Delay(10000);					   
    Set_Rst;		 	 
    Delay(10000);	
}
void WriteComm(u16 CMD)
{			
	*(__IO u16 *) (Bank1_LCD_C) = CMD;
}
void WriteData(u16 tem_data)
{			
	*(__IO u16 *) (Bank1_LCD_D) = tem_data;
}

void BlockWrite(unsigned int Xstart,unsigned int Xend,unsigned int Ystart,unsigned int Yend) 
{
	WriteComm(0x2a);   
	WriteData(Xstart>>8);
	WriteData(Xstart&0xff);
	WriteData(Xend>>8);
	WriteData(Xend&0xff);

	WriteComm(0x2b);   
	WriteData(Ystart>>8);
	WriteData(Ystart&0xff);
	WriteData(Yend>>8);
	WriteData(Yend&0xff);
	
	WriteComm(0x2c);
}

void Lcd_ColorBox(u16 xStart,u16 yStart,u16 xLong,u16 yLong,u16 Color)
{
	u32 temp;

	BlockWrite(xStart,xStart+xLong-1,yStart,yStart+yLong-1);
	for (temp=0; temp<xLong*yLong; temp++)
	{
		*(__IO u16 *) (Bank1_LCD_D) = Color;
	}
}

void lcd_screen(uint16_t xStart,uint16_t yStart,uint16_t xLong,uint16_t yLong,long * Color) {
    uint32_t temp;

    BlockWrite(xStart,xStart+xLong-1,yStart,yStart+yLong-1);
    for (temp=0; temp<xLong*yLong; temp++) {
        *(__IO u16 *) (Bank1_LCD_D) = *Color++;
    }
}

/**********************************************
函数名：Lcd初始化函数
功能：初始化Lcd
入口参数：无
返回值：无
***********************************************/
void lcd_init(void) {
    int a;
    Lcd_Light_ON;

    LCD_GPIO_Config();
    LCD_FSMC_Config();
    LCD_Rst();

    WriteComm(0xF8);
    a = *(__IO u16 *) (Bank1_LCD_D); 
    a = *(__IO u16 *) (Bank1_LCD_D); 

    WriteComm(0x13);
    WriteComm(0x35);
    WriteData(0x00);
    WriteComm(0x36);
    WriteData(0x48);
    WriteComm(0xD0);
    WriteData(0x00);
    WriteData(0x05);

    WriteComm(0xEF);
    WriteData(0x07);


    WriteComm(0xF2);
    WriteData(0x1B);
    WriteData(0x16);
    WriteData(0x0F);
    WriteData(0x08);
    WriteData(0x08);
    WriteData(0x08);
    WriteData(0x08);
    WriteData(0x10);
    WriteData(0x00);
    WriteData(0x1C);
    WriteData(0x16);

    WriteComm(0xF3);
    WriteData(0x01);
    WriteData(0x41);
    WriteData(0x15);
    WriteData(0x0D);
    WriteData(0x33);
    WriteData(0x63);
    WriteData(0x46);
    WriteData(0x10);


    WriteComm(0xF4);
    WriteData(0x5B);
    WriteData(0x5B);
    WriteData(0x55);
    WriteData(0x55);
    WriteData(0x44);

    WriteComm(0xF5);
    WriteData(0x12);
    WriteData(0x11);
    WriteData(0x06);
    WriteData(0xF0);
    WriteData(0x00);
    WriteData(0x1F);

    WriteComm(0xF6);
    WriteData(0x80);
    WriteData(0x10);
    WriteData(0x00);

    WriteComm(0xFD);
    WriteData(0x11);
    WriteData(0x1D);
    WriteData(0x00);


    WriteComm(0xF8); //Positive Gamma Control
    WriteData(0x00);
    WriteData(0x15);
    WriteData(0x01);
    WriteData(0x08);
    WriteData(0x15);
    WriteData(0x22);
    WriteData(0x25);
    WriteData(0x28);
    WriteData(0x14);
    WriteData(0x13);
    WriteData(0x10);
    WriteData(0x11);
    WriteData(0x09);
    WriteData(0x24);
    WriteData(0x28);

    WriteComm(0xF9); //Positive Gamma Control
    WriteData(0x00);
    WriteData(0x15);
    WriteData(0x01);
    WriteData(0x08);
    WriteData(0x15);
    WriteData(0x22);
    WriteData(0x25);
    WriteData(0x28);
    WriteData(0x14);
    WriteData(0x13);
    WriteData(0x10);
    WriteData(0x11);
    WriteData(0x09);
    WriteData(0x24);
    WriteData(0x28);

    WriteComm(0xFC); //Positive Gamma Control
    WriteData(0x00);
    WriteData(0x15);
    WriteData(0x01);
    WriteData(0x08);
    WriteData(0x15);
    WriteData(0x22);
    WriteData(0x25);
    WriteData(0x28);
    WriteData(0x14);
    WriteData(0x13);
    WriteData(0x10);
    WriteData(0x11);
    WriteData(0x09);
    WriteData(0x24);
    WriteData(0x28);


    WriteComm(0x36);//Memory Data Access Control
    WriteData(0x48);

    WriteComm(0x3A);//SET 65K Color
    WriteData(0x55);

    WriteComm(0x11); 
    Delay(120);

    WriteComm(0x29);//Display on
    WriteComm(0x2C);//Write GRAM
    Delay(10);
    Lcd_ColorBox(0,0,320,240,0xff00);
    Lcd_Light_ON;
}
