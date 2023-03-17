/*---------------------------------
*file : buzzer.c
*content : Fall in the alarm
----------------------------------*/
#include "buzzer.h"
#include "oled.h"
#include "ds18b20.h"
extern void deca_sleep(unsigned int time_ms);
u8 buzzer=0;
/*-------------------------------
*函数：	buzzer_Init
*功能：	初始化蜂鸣器管脚
*返回值：无
-------------------------------*/
void buzzer_Init(void)//PA3
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_3);
}
/*------------------------------------
*函数：	buzzer_check
*功能：	检测是否有报警动作
*返回值：	无
-------------------------------------*/
void buzzer_check()
{
	/*跌倒*/
 	if(buzzer == 1)
 	{
 		GPIO_SetBits(GPIOA,GPIO_Pin_3);
 		//dace_sleep(1000);
			
 		//OLED_Clear();
// 		OLED_ShowChinese(0,2,0,16);
// 		OLED_ShowChinese(18,2,1,16);
// 		OLED_ShowChinese(36,2,4,16);
// 		OLED_ShowChinese(54,2,5,16);
//		OLED_Refresh_Gram();
 		//deca_sleep(800);
 		GPIO_ResetBits(GPIOA,GPIO_Pin_3);
		show_temp();
		buzzer=0;
 	}
	/*安全*/
 	else
 	{
 		//OLED_Clear();
// 		OLED_ShowChinese(0,10,0,16);
// 		OLED_ShowChinese(18,10,1,16);
// 		OLED_ShowChinese(36,10,2,16);
// 		OLED_ShowChinese(54,10,3,16);
//		OLED_Refresh_Gram();
 		//deca_sleep(800);
 	}	
}
