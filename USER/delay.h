#ifndef __DELAY_H
#define __DELAY_H
#include "stdint.h" 			   
#include "stm32f4xx.h" 	
static u8  fac_us=0;//us��ʱ������
static u16 fac_ms=0;//ms��ʱ������

void delay_us(u32 Nus);
#endif
