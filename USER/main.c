/*
*file main.c
UWB定位程序
编写于2022 3 18 第一版
*/
//#include "stm32f4xx.h"
#include "dw1000.h"
#include "port.h"
#include "stdio.h"
#include "oled.h"
#include "mq.h"
#include "ds18b20.h"
#include "rtc_time.h"
#include "timer.h"
#include "FallDetection.h"
#include "max30102.h"


/**********************************************************/
// 函数名：main
// 描述  ：主函数
// 输入  ：无
// 输出  ：无
/*********************************************************/
int main(void)
{  
/*----------------------初始化芯片外设*/
	peripherals_init();						//外设初始化		
/*----------------------end*/

/*----------------------初始化其余模块*/
#ifdef TX_NODE
	/*-------配置环境监测和闹钟*/
	MQ_port_init();			
	ds18b20_port_init();
	rtc_time_init();
	/*--------end*/
	
	while(ADXL345_Init());					//配置ADCL345
	 max30102_init();						//配置MAX30102

	//OLED_Init();			//显示屏
#endif	
/*------------------------end*/

/*--------------------------初始化DW1000且配置*/
	reset_DW1000();						
	if(dw1000_init()==-1)					//DW1000初始化
	{
		return -1;
	}

	dw1000_msg_init();						//DW1000信息包初始化
	dw1000_config();						//DW1000配置初始化
/*-------------------------end*/


#ifdef TX_NODE
		TIM3_init();	
	 /*-------用于配置ADXL345所需的外设*/
	  Tim2_init(100,8400);//20ms中断间隔,作用于adlx345
	  adxl345_exti();//开启adxl345外部中断
	 /*-------end*/
#endif


	while(1)
	{

#ifdef TX_NODE
		tx_main();
#endif

#ifdef RX_NODE
		rx_main();
#endif
		
	}
}



