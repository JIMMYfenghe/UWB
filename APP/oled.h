#ifndef __OLED_H
#define __OLED_H			  	 
//#include "sys.h"
#include "stdlib.h"	 
#include "stm32f4xx.h"
#include "stdint.h"
 
typedef unsigned char u8;
//typedef unsigned int u32;
/*
*采取模拟spi时序通信
	PB10 DC
	PB12 CS
	PB13 SCL
	PB14 RES
	PB15  SDA
*/
//OLED模式设置
//0: 4线串行模式  （模块的BS1，BS2均接GND）
//1: 并行8080模式 （模块的BS1，BS2均接VCC）
#define OLED_MODE 	0 
/*	    						  
//-----------------OLED端口定义----------------  					   
#define OLED_CS 	PAout(3)
#define OLED_RST  	PAout(4)	
#define OLED_DC		PAout(5)
#define OLED_SCLK 	PAout(6)
#define OLED_SDIN 	PAout(7)
*/

#define	OLED_CS_H()		GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET)
#define	OLED_CS_L()		GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET)

#define OLED_RST_H()	GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_SET)
#define	OLED_RST_L()	GPIO_WriteBit(GPIOB, GPIO_Pin_14, Bit_RESET)

#define	OLED_DC_H()		GPIO_WriteBit(GPIOB, GPIO_Pin_10, Bit_SET)
#define	OLED_DC_L()		GPIO_WriteBit(GPIOB, GPIO_Pin_10, Bit_RESET)

#define OLED_SCL_H()	GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_SET)
#define	OLED_SCL_L()	GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_RESET)

#define	OLED_SDA_H()	GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_SET)
#define	OLED_SDA_L()	GPIO_WriteBit(GPIOB, GPIO_Pin_15, Bit_RESET)

#define OLED_CMD  	0		//写命令
#define OLED_DATA 	1		//写数据
//OLED控制用函数
void OLED_WR_Byte(u8 dat,u8 cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);		   
							   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y,u8 *p,u8 size);	
void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size);
#endif  
	 



