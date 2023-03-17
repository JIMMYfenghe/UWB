/*
*file ds18b20.h
*/
#ifndef __DS18B20_H
#define __DS18B20_H
#include "stm32f4xx.h"

//#define 	GPIO_x				GPIOA

		


#define 	DQ_IO_IN()  				GPIOA->MODER&=~(0x3);GPIOA->PUPDR&=~(0x3);GPIOA->PUPDR|=0x1	//设置Px0为上拉输入
#define		DQ_STATUS()					GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)//读取Px0 IO状态


#define 	DQ_IO_OUT()					GPIOA->MODER&=~(0x3);GPIOA->MODER|=0x1;GPIOA->OTYPER&=~(0x3)  //设置Px0为推挽输出
#define 	DQ_TO_HIGH()				GPIO_SetBits(GPIOA, GPIO_Pin_0)				//置1
#define 	DQ_TO_LOW()					GPIO_ResetBits(GPIOA, GPIO_Pin_0)			//清0



void ds18b20_port_init(void);
u8 read_bit(void);
u8 read_byte(void);
void write_byte(u8 data);
void ds18b20_reset(void);
int ds18b20_check(void);
void ds18b20_conversion(void);
void ds18b20_readtemp_cmd(void);
float ds18b20_get_temp(void);
void  show_temp(void);
#endif
