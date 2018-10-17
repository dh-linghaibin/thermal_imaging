

#include "lcd.h"

#define Bank1_LCD_D    ((u32)0x60020000)    //Disp Data ADDR
#define Bank1_LCD_C    ((u32)0x60000000)      //Disp Reg  ADDR


#define Lcd_Light_ON   GPIO_SetBits(GPIOD,GPIO_Pin_12);
#define Lcd_Light_OFF  GPIO_ResetBits(GPIOD,GPIO_Pin_12);

#define Set_Rst GPIO_SetBits(GPIOD,GPIO_Pin_13);
#define Clr_Rst GPIO_ResetBits(GPIOD,GPIO_Pin_13);

static void lcd_gpio_config(void) {
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

    GPIO_SetBits(GPIOD, GPIO_Pin_13);         // RST = 1 
    GPIO_SetBits(GPIOD, GPIO_Pin_4);         // RD = 1  
    GPIO_SetBits(GPIOD, GPIO_Pin_5);         // WR = 1 
    GPIO_SetBits(GPIOD, GPIO_Pin_7);         //    CS = 1 

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;        
    GPIO_Init(GPIOA, &GPIO_InitStructure);
                                                        // right          up           left        down
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_1 | GPIO_Pin_5 | GPIO_Pin_6;        
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}

static void lcd_fsmc_config(void)
{
    FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef  p; 


    p.FSMC_AddressSetupTime = 0x01;     //地址建立时间
    p.FSMC_AddressHoldTime = 0x01;     //地址保持时间
    p.FSMC_DataSetupTime = 0x01;         //数据建立时间
    p.FSMC_BusTurnAroundDuration = 0x00;
    p.FSMC_CLKDivision = 0x00;
    p.FSMC_DataLatency = 0x00;

    p.FSMC_AccessMode = FSMC_AccessMode_B;     // 一般使用模式B来控制LCD

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

volatile static void lcd_delay(__IO u32 nCount)
{    
    volatile int i;
    for(i=0;i<7200;i++)
    for(; nCount != 0; nCount--);
}  

static void lcd_rst(void)
{            
    Clr_Rst;
    lcd_delay(10000);
    Set_Rst;
    lcd_delay(10000);
}
void write_comm(u16 CMD)
{
    *(__IO u16 *) (Bank1_LCD_C) = CMD;
}
void write_data(u16 tem_data)
{
    *(__IO u16 *) (Bank1_LCD_D) = tem_data;
}

void lcd_block_write(unsigned int Xstart,unsigned int Xend,unsigned int Ystart,unsigned int Yend) 
{
    write_comm(0x2a);   
    write_data(Xstart>>8);
    write_data(Xstart&0xff);
    write_data(Xend>>8);
    write_data(Xend&0xff);

    write_comm(0x2b);
    write_data(Ystart>>8);
    write_data(Ystart&0xff);
    write_data(Yend>>8);
    write_data(Yend&0xff);
    
    write_comm(0x2c);
}

void lcd_color_box(u16 xStart,u16 yStart,u16 xLong,u16 yLong,u16 Color)
{
    u32 temp;

    lcd_block_write(xStart,xStart+xLong-1,yStart,yStart+yLong-1);
    for (temp=0; temp<xLong*yLong; temp++)
    {
        *(__IO u16 *) (Bank1_LCD_D) = Color;
    }
}

void lcd_screen2(uint16_t xStart,uint16_t yStart,uint16_t xLong,uint16_t yLong,long * Color) {
    uint32_t temp;

    lcd_block_write(xStart,xStart+xLong-1,yStart,yStart+yLong-1);
    for (temp=0; temp<xLong*yLong; temp++) {
        *(__IO u16 *) (Bank1_LCD_D) = *Color++;
    }
}

void lcd_screen(uint16_t xStart,uint16_t yStart,uint16_t xLong,uint16_t yLong, uint16_t * color) {
    uint32_t temp;

    lcd_block_write(xStart,xStart+xLong-1,yStart,yStart+yLong-1);
    for (temp=0; temp<xLong*yLong; temp++) {
        *(__IO u16 *) (Bank1_LCD_D) = *color++;
    }
}

void lcd_init(void) {
    int a;
    Lcd_Light_ON;

    lcd_gpio_config();
    lcd_fsmc_config();
    lcd_rst();

    write_comm(0xF8);
    a = *(__IO u16 *) (Bank1_LCD_D); 
    a = *(__IO u16 *) (Bank1_LCD_D); 

    write_comm(0x13);
    write_comm(0x35);
    write_data(0x00);
    write_comm(0x36);
    write_data(0x48);
    write_comm(0xD0);
    write_data(0x00);
    write_data(0x05);

    write_comm(0xEF);
    write_data(0x07);


    write_comm(0xF2);
    write_data(0x1B);
    write_data(0x16);
    write_data(0x0F);
    write_data(0x08);
    write_data(0x08);
    write_data(0x08);
    write_data(0x08);
    write_data(0x10);
    write_data(0x00);
    write_data(0x1C);
    write_data(0x16);

    write_comm(0xF3);
    write_data(0x01);
    write_data(0x41);
    write_data(0x15);
    write_data(0x0D);
    write_data(0x33);
    write_data(0x63);
    write_data(0x46);
    write_data(0x10);


    write_comm(0xF4);
    write_data(0x5B);
    write_data(0x5B);
    write_data(0x55);
    write_data(0x55);
    write_data(0x44);

    write_comm(0xF5);
    write_data(0x12);
    write_data(0x11);
    write_data(0x06);
    write_data(0xF0);
    write_data(0x00);
    write_data(0x1F);

    write_comm(0xF6);
    write_data(0x80);
    write_data(0x10);
    write_data(0x00);

    write_comm(0xFD);
    write_data(0x11);
    write_data(0x1D);
    write_data(0x00);


    write_comm(0xF8); //Positive Gamma Control
    write_data(0x00);
    write_data(0x15);
    write_data(0x01);
    write_data(0x08);
    write_data(0x15);
    write_data(0x22);
    write_data(0x25);
    write_data(0x28);
    write_data(0x14);
    write_data(0x13);
    write_data(0x10);
    write_data(0x11);
    write_data(0x09);
    write_data(0x24);
    write_data(0x28);

    write_comm(0xF9); //Positive Gamma Control
    write_data(0x00);
    write_data(0x15);
    write_data(0x01);
    write_data(0x08);
    write_data(0x15);
    write_data(0x22);
    write_data(0x25);
    write_data(0x28);
    write_data(0x14);
    write_data(0x13);
    write_data(0x10);
    write_data(0x11);
    write_data(0x09);
    write_data(0x24);
    write_data(0x28);

    write_comm(0xFC); //Positive Gamma Control
    write_data(0x00);
    write_data(0x15);
    write_data(0x01);
    write_data(0x08);
    write_data(0x15);
    write_data(0x22);
    write_data(0x25);
    write_data(0x28);
    write_data(0x14);
    write_data(0x13);
    write_data(0x10);
    write_data(0x11);
    write_data(0x09);
    write_data(0x24);
    write_data(0x28);


    write_comm(0x36);//Memory Data Access Control
    write_data(0x48);

    write_comm(0x3A);//SET 65K Color
    write_data(0x55);

    write_comm(0x11); 
    lcd_delay(120);

    write_comm(0x29);//Display on
    write_comm(0x2C);//Write GRAM
    lcd_delay(10);
    lcd_color_box(0,0,320,240,0x0000);
    Lcd_Light_ON;
}

#include "fonts.h"

#define      WHITE                         0xFFFF	   //白色
#define      BLACK                         0x0000	   //黑色 
#define      GREY                          0xF7DE	   //灰色 
#define      BLUE                          0x001F	   //蓝色 
#define      BLUE2                         0x051F	   //浅蓝色 
#define      RED                           0xF800	   //红色 
#define      MAGENTA                       0xF81F	   //红紫色，洋红色 
#define      GREEN                         0x07E0	   //绿色 
#define      CYAN                          0x7FFF	   //蓝绿色，青色 
#define      YELLOW                        0xFFE0	   //黄色 
#define      BRED                          0xF81F
#define      GRED                          0xFFE0
#define      GBLUE                         0x07FF

static uint16_t CurrentTextColor   = WHITE;//前景色
static uint16_t CurrentBackColor   = BLACK;//背景色

/**
 * @brief  在 ILI9341 显示器上显示一个中文字符
 * @param  usX ：在特定扫描方向下字符的起始X坐标
 * @param  usY ：在特定扫描方向下字符的起始Y坐标
 * @param  usChar ：要显示的中文字符（国标码）
 * @note 可使用LCD_SetBackColor、LCD_SetTextColor、LCD_SetColors函数设置颜色
 * @retval 无
 */ 
void ILI9341_DispChar_CH ( uint16_t usX, uint16_t usY, uint16_t usChar )
{
    uint32_t rowCount, bitCount;
    uint8_t ucBuffer [ WIDTH_CH_CHAR*HEIGHT_CH_CHAR/8 ];
    uint16_t usTemp; 
    //取字模数据  
    GetGBKCode ( ucBuffer, usChar );
    lcd_block_write(usX, usX+WIDTH_CH_CHAR-1, usY, usY+HEIGHT_CH_CHAR-1);
    for ( rowCount = 0; rowCount < HEIGHT_CH_CHAR; rowCount++ )
    {
        /* 取出两个字节的数据，在lcd上即是一个汉字的一行 */
        usTemp = ucBuffer [ rowCount * 2 ];
        usTemp = ( usTemp << 8 );
        usTemp |= ucBuffer [ rowCount * 2 + 1 ];

        for ( bitCount = 0; bitCount < WIDTH_CH_CHAR; bitCount ++ )
        {
            if ( usTemp & ( 0x8000 >> bitCount ) )  //高位在前 
                write_data(CurrentTextColor);
            else
                write_data(CurrentBackColor);
        }
    }
}

void ILI9341_DispChar_CH2 ( uint16_t usX, uint16_t usY, uint16_t usChar );
void ILI9341_DispChar ( uint16_t usX, uint16_t usY, uint16_t usChar );

uint16_t LCD_X_LENGTH = 320;
uint16_t LCD_Y_LENGTH = 240;

#define      ILI9341_DispWindow_X_Star		    0     //起始点的X坐标
#define      ILI9341_DispWindow_Y_Star		    0     //起始点的Y坐标

void ILI9341_DispStringLine_EN_CH (  uint16_t line, char * pStr )
{
    uint16_t usCh;
    uint16_t usX = 0;
    
    while( * pStr != '\0' )
    {
        if (0 == (*pStr & 0x80)) {
            ILI9341_DispChar ( usX, line, *pStr - ' ' );
            usX += (WIDTH_CH_CHAR-4);
            pStr++;
        } else {
            if ( ( usX - ILI9341_DispWindow_X_Star + WIDTH_CH_CHAR ) > LCD_X_LENGTH )
            {
                usX = ILI9341_DispWindow_X_Star;
                line += HEIGHT_CH_CHAR;
            }
            if ( ( line - ILI9341_DispWindow_Y_Star + HEIGHT_CH_CHAR ) > LCD_Y_LENGTH )
            {
                usX = ILI9341_DispWindow_X_Star;
                line = ILI9341_DispWindow_Y_Star;
            }
//            usCh = * ( uint16_t * ) pStr;
//            usCh = ( usCh << 8 ) + ( usCh >> 8 );
            unsigned char code[2];
            code[0] = ((pStr[0] & 0x0f) <<4) + ((pStr[1] & 0x3c) >>2);
            code[1] = ((pStr[1] & 0x03) <<6) + (pStr[2] & 0x3f);

            ILI9341_DispChar_CH2 ( usX, line,  ((code[0] << 8) | code[1]) - 19968 );

            usX += WIDTH_CH_CHAR;
            pStr += 3;//一个汉字两个字节 
        }
    }
} 

typedef union {
    struct {
        uint16_t r : 5;
        uint16_t g : 6;
        uint16_t b : 5;
    } rgb;
    uint16_t rgb565;
} lui_color565_u;

uint16_t lui_color_alpha_blend(uint16_t fr_c,uint16_t bk_c,uint8_t alpha) {
    register lui_color565_u color, color_fr_c,color_bk_c ;
    color_fr_c.rgb565 = fr_c;
    color_bk_c.rgb565 = bk_c;
    color.rgb.r = ( color_fr_c.rgb.r * alpha + color_bk_c.rgb.r * (0xff-alpha) ) >> 8;
    color.rgb.g = ( color_fr_c.rgb.g * alpha + color_bk_c.rgb.g * (0xff-alpha) ) >> 8;
    color.rgb.b = ( color_fr_c.rgb.b * alpha + color_bk_c.rgb.b * (0xff-alpha) ) >> 8;
    return color.rgb565;
}



void lui_draw_font(int x, int y, uint8_t wighth, uint8_t length, uint16_t color, uint8_t * mate) {
    uint16_t ptr = 0;
    lcd_block_write(x, x+wighth-1, y, y+length-1);
    for(int y_i = 0; y_i < length; y_i ++) {
        for(int x_j = 0; x_j < wighth; x_j++) {
            if(mate[ptr] != 0) {
                if(mate[ptr] == 0xff) {
                    write_data( color );
                } else {
                    write_data( lui_color_alpha_blend(color,CurrentBackColor,mate[(ptr)]) );
                }
            } else {
                write_data( CurrentBackColor );
            }
            ptr++;
        }
    }
}

void ILI9341_DispChar_CH2 ( uint16_t usX, uint16_t usY, uint16_t usChar )
{
    uint32_t rowCount, bitCount;
    uint8_t ucBuffer [ WIDTH_CH_CHAR*HEIGHT_CH_CHAR];
    uint16_t usTemp; 
    //取字模数据
    usChar += 95;
    GetGBKCode_from_sd2 ( ucBuffer, usChar );
    lui_draw_font(usX,usY,WIDTH_CH_CHAR,HEIGHT_CH_CHAR,CurrentTextColor,ucBuffer);
}


void ILI9341_DispChar ( uint16_t usX, uint16_t usY, uint16_t usChar )
{
    uint32_t rowCount, bitCount;
    uint8_t ucBuffer [ WIDTH_CH_CHAR*HEIGHT_CH_CHAR];
    uint16_t usTemp; 
    GetGBKCode_from_sd2 ( ucBuffer, usChar );
    lui_draw_font(usX,usY,WIDTH_CH_CHAR,HEIGHT_CH_CHAR,CurrentTextColor,ucBuffer);
}
