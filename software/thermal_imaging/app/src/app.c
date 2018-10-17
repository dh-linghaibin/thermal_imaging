
#include "bsp.h"
#include "lcd.h"
#include "voice.h"
#include "amg8833.h"
#include "ms5611.h"
#include "ff.h"
#include "stdio.h"

extern uint16_t data[40][40];
extern uint16_t ext[3];
extern uint8_t ext_add[2];

#define BW  1
#define Iron 2
#define RB 3

#define none 0
#define midd 1
#define exts 2

#define t1 1000/5000
#define t2 2000/5000
#define t3 3000/5000
#define t4 4000/5000

 uint8_t color_mod=Iron;
 
void blowup(void) {
    int i;
    for (i = 0; i < 8 * 7; i++) {
        data[i / 7 * 5 + 2][i % 7 * 5 + 2 + 1] = 1+ data[i / 7 * 5 + 2][i % 7 * 5 + 2] * t4 + data[i / 7 * 5 + 2][i % 7 * 5 + 2 + 5] * t1;
        data[i / 7 * 5 + 2][i % 7 * 5 + 2 + 2] = 1+ data[i / 7 * 5 + 2][i % 7 * 5 + 2] * t3 + data[i / 7 * 5 + 2][i % 7 * 5 + 2 + 5] * t2;
        data[i / 7 * 5 + 2][i % 7 * 5 + 2 + 3] = 1+ data[i / 7 * 5 + 2][i % 7 * 5 + 2] * t2 + data[i / 7 * 5 + 2][i % 7 * 5 + 2 + 5] * t3;
        data[i / 7 * 5 + 2][i % 7 * 5 + 2 + 4] = 1+ data[i / 7 * 5 + 2][i % 7 * 5 + 2] * t1 + data[i / 7 * 5 + 2][i % 7 * 5 + 2 + 5] * t4;
    }
    for (i = 0; i < 7 * 36; i++) {
        data[i % 7 * 5 + 2 + 1][i / 7 + 2] = 1+ data[i % 7 * 5 + 2][i / 7 + 2] * t4 + data[i % 7 * 5 + 2 + 5][i / 7 + 2] * t1;
        data[i % 7 * 5 + 2 + 2][i / 7 + 2] = 1+ data[i % 7 * 5 + 2][i / 7 + 2] * t3 + data[i % 7 * 5 + 2 + 5][i / 7 + 2] * t2;
        data[i % 7 * 5 + 2 + 3][i / 7 + 2] = 1+ data[i % 7 * 5 + 2][i / 7 + 2] * t2 + data[i % 7 * 5 + 2 + 5][i / 7 + 2] * t3;
        data[i % 7 * 5 + 2 + 4][i / 7 + 2] = 1+ data[i % 7 * 5 + 2][i / 7 + 2] * t1 + data[i % 7 * 5 + 2 + 5][i / 7 + 2] * t4;
    }
    for (i = 0; i < 36; i++) {
        data[0][i + 2] = data[1][i + 2] = data[2][i + 2];
        data[39][i + 2] = data[38][i + 2] = data[37][i + 2];
    }
    for (i = 0; i < 40; i++) {
        data[i][0] = data[i][1] = data[i][2];
        data[i][39] = data[i][38] = data[i][37];
    }
    ext[2]=data[19][19];
}


uint16_t To_HSB(uint8_t num){ // 0-64 96 - 128  191
    uint8_t R=0,G=0,B=0;
    float a;
    uint32_t b;
    if(color_mod==Iron){
        a=0.7*num;
        a+=20;
        num=(uint8_t)a;
        if (num < 64) {
            B = (unsigned char)(num * 4);
            G = 0;
            R = 0;
        }
        else if (num < 96) {
            B = 255;
            G = 0;
            R = (unsigned char)(4 * (num - 64));
        }
        else if (num < 128) {
            B = (unsigned char)(256 - 8 * (num - 95));
            G = 0;
            R = (unsigned char)(4 * (num - 64) - 1);
        }
        else if (num < 191) {
            B = 0;
            G = (unsigned char)(4 * (num - 128));
            R = 255;
        }
        else {
            B = (unsigned char)(4 * (num - 191));
            G = 255;
            R = 255;
        }
    }else if(color_mod==BW){
        R=G=B=num;
    }else if(color_mod==RB){
        b=240*num;
        b/=255;
        num=(uint8_t)b;
        if (num < 60) {
            B = 255;
            G = num*4;
            R = 0;
        }
        else if (num < 120) {
            B = (120-num)*4;
            G = 255;
            R = 0;
        }
        else if (num < 180) {
            B = 0;
            G = 255;
            R = (num-119)*4;
        }else{
            B = 0;
            G = (240-num)*4;
            R = 255;
        }
    }
    return 0xFFFF&((B&0xf8)>>3|(G&0xf8)<<3|(R&0xf8)<<8);
}

void draw_bar(uint16_t x,uint16_t y) {
    for(int i = 0;i < 200;i++) {
        lcd_color_box(0,i,10,1,To_HSB(i));
    }
}

void get_img(void){
    uint16_t i;
    long diff=ext[0]-ext[1]+2;
    for(i=0;i<40*40;i++){
        data[i/40][i%40]=To_HSB(0xff&((data[i/40][i%40]-ext[1]+1)*0xff/diff));
    }
}

void Draw_img(void){
    uint16_t i;
    for (i = 0; i < 40*40; i++) {
        lcd_color_box(120+i/40*5,40+i%40*5,5,5,data[i/40][i%40]);
    }
    lcd_color_box(240,170,20,20,0xffff);
}

///重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
    return (ch);
}

///重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		return (int)0x00;
}

extern void jpgDisplay(char *pic_name);

int main( void ) {
    bsp_init();
    voice_init();
    lcd_init();
    amg8833_init();
    //ms5611_init();
    ILI9341_DispStringLine_EN_CH( 0,"电池电压123ABCabc" );
    //draw_bar(0,0);
    while(1) {
        get_data();    //获取数据
        blowup();      //插值
        get_img();     //插值转换为rgb图片
        Draw_img();    //显示图片
    }
    return 0x00;
}

///**
//  * @brief  停机唤醒后配置系统时钟: 使能 HSE, PLL
//  *         并且选择PLL作为系统时钟.
//  * @param  None
//  * @retval None
//  */
//static void SYSCLKConfig_STOP(void)
//{
//  /* After wake-up from STOP reconfigure the system clock */
//  /* 使能 HSE */
//  RCC_HSEConfig(RCC_HSE_ON);
//  
//  /* 等待 HSE 准备就绪 */
//  while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET)
//  {
//  }
//  
//  /* 使能 PLL */ 
//  RCC_PLLCmd(ENABLE);
//  
//  /* 等待 PLL 准备就绪 */
//  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
//  {
//  }
//  
//  /* 选择PLL作为系统时钟源 */
//  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
//  
//  /* 等待PLL被选择为系统时钟源 */
//  while (RCC_GetSYSCLKSource() != 0x08)
//  {
//  }
//}
