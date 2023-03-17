#ifndef __SYS_H
#define __SYS_H	
#include "stm32f4xx.h"

//1,支持ucos//0,不支持ucos
#define SYSTEM_SUPPORT_UCOS		0		//定义系统文件夹是否支持UCOS
																	    
//位带操作,实现51类似的GPIO控制功能
//IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)  *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 
//IO口地址映射
#define GPIOA_ODR_Addr    (GPIOA_BASE+0x14) //0x4002000E 
#define GPIOB_ODR_Addr    (GPIOB_BASE+0x14) //0x4002040E 
#define GPIOC_ODR_Addr    (GPIOC_BASE+0x14) //0x4002080E 
#define GPIOD_ODR_Addr    (GPIOD_BASE+0x14) //0x40020C0E 
#define GPIOE_ODR_Addr    (GPIOE_BASE+0x14) //0x4002100E 
#define GPIOF_ODR_Addr    (GPIOF_BASE+0x14) //0x4002140E    
#define GPIOG_ODR_Addr    (GPIOG_BASE+0x14) //0x4002180E    
#define GPIOH_ODR_Addr    (GPIOH_BASE+0x14) //0x40021C0E   

#define GPIOA_IDR_Addr    (GPIOA_BASE+0x10) //0x4002000A 
#define GPIOB_IDR_Addr    (GPIOB_BASE+0x10) //0x4002040A 
#define GPIOC_IDR_Addr    (GPIOC_BASE+0x10) //0x4002080A
#define GPIOD_IDR_Addr    (GPIOD_BASE+0x10) //0x40020C0A 
#define GPIOE_IDR_Addr    (GPIOE_BASE+0x10) //0x4002100A 
#define GPIOF_IDR_Addr    (GPIOF_BASE+0x10) //0x4002140A 
#define GPIOG_IDR_Addr    (GPIOG_BASE+0x10) //0x4002180A 
#define GPIOH_IDR_Addr    (GPIOH_BASE+0x10) //0x40021C0A
 
//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  //输出 
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  //输入 

#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)  //输出 
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)  //输入 

#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)  //输出 
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)  //输入 

#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)  //输出 
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)  //输入 

#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)  //输出 
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)  //输入

#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)  //输出 
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)  //输入

#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)  //输出 
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)  //输入

#define PHout(n)   BIT_ADDR(GPIOH_ODR_Addr,n)  //输出 
#define PHin(n)    BIT_ADDR(GPIOH_IDR_Addr,n)  //输入



#endif
