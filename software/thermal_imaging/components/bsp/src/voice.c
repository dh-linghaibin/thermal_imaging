

#include "voice.h"

#define BEEP_GPIO_PORT    	GPIOA			              /* GPIO端口 */
#define BEEP_GPIO_CLK 	    RCC_APB2Periph_GPIOA		/* GPIO端口时钟 */
#define BEEP_GPIO_PIN		  GPIO_Pin_3			        /* 连接到蜂鸣器的GPIO */

/* 高电平时，蜂鸣器响 */
#define ON  1
#define OFF 0

/* 带参宏，可以像内联函数一样使用 */
#define BEEP(a)	if (a)	\
					GPIO_SetBits(BEEP_GPIO_PORT,BEEP_GPIO_PIN);\
					else		\
					GPIO_ResetBits(BEEP_GPIO_PORT,BEEP_GPIO_PIN)


void voice_init( void ) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_ResetBits(GPIOA, GPIO_Pin_3);
}
