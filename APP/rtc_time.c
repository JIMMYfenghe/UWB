/*
file rtc_time.c
该文件功能是设置时间，日期
*/


#include "stm32f4xx.h"
#include "delay.h"
#include "rtc_time.h"
#include "oled.h"
#include "port.h"


extern alarm_struct *alarm_a;//声明外部变量alarm_a
extern u8 cmd;//声明外部变量cmd

/*--------------------------------------
*函数：		rtc_time_init
*功能：		rtc初始化
*返回值：	
			0	:	成功
			1	:	失败
---------------------------------------*/
extern void deca_sleep(unsigned int time_ms);
int rtc_time_init(void) 
{
	RTC_InitTypeDef		RTC_Initstruct;
	u16 retry=0x1fff;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);//使能PWR时钟
	PWR_BackupAccessCmd(ENABLE);//使能RTC后备寄存器
	
	if(RTC_ReadBackupRegister(RTC_BKP_DR0)==0x5050)//判断是否第一次配置RTC,该寄存器即便断电不复位，依然保存数据
	{
		RCC_LSEConfig(ENABLE);//开启LSE时钟
		
		while(RCC_GetFlagStatus(RCC_FLAG_LSERDY)==RESET)//等待LSE就绪
		{
			retry++;
			deca_sleep(10);
		}

		if(retry==0)
			return 1;
		// RCC_LSICmd(ENABLE);
		// while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY)==RESET);
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);//设置RTC时钟
		RCC_RTCCLKCmd(ENABLE);//使能RTC时钟
		
		//初步初始化RTC
		//RTC_Initstruct.RTC_AsynchPrediv=0x7F;
		RTC_Initstruct.RTC_AsynchPrediv=0x7C;
		RTC_Initstruct.RTC_SynchPrediv=0XFF;
		RTC_Initstruct.RTC_HourFormat=RTC_HourFormat_24;
		RTC_Init(&RTC_Initstruct);
		
		rtc_set_time(10,53,00,RTC_H12_AM);
		rtc_set_date(22,5,17,2);
		
	}
	RTC_WriteBackupRegister(RTC_BKP_DR0,0x5050);//不再执行初始化
	
	return 0;
}

/*------------------------------
*函数：		rtc_set_time
*功能：		设置时间
*输入：		
			hour	:	小时
			minute	：	分钟
			second	：	秒
			ampm	：	显示格式
*输出：		无
*返回值：	
			0	：	失败
			1	：	成功
-------------------------------*/
int rtc_set_time(u8 hour,u8 minute,u8 second,u8 ampm)
{
	RTC_TimeTypeDef RTC_Timestruct;
	
	RTC_Timestruct.RTC_Hours=hour;
	RTC_Timestruct.RTC_Minutes=minute;
	RTC_Timestruct.RTC_Seconds=second;
	RTC_Timestruct.RTC_H12=ampm;
	
	return RTC_SetTime(RTC_Format_BIN,&RTC_Timestruct);
}

/*------------------------------------
*函数：		rtc_set_date
*功能：		设置日期
*输入：	
			year	:	年
			month	:	月
			day		：	日
			week	：	星期几
*输出：		无
*返回值：	
			0	：	失败
			1	:	成功
-------------------------------------*/
int rtc_set_date(u8 year,u8 month,u8 day,u8 week)
{
	RTC_DateTypeDef	RTC_Datestruce;
	
	RTC_Datestruce.RTC_Year=year;
	RTC_Datestruce.RTC_Month=month;
	RTC_Datestruce.RTC_Date=day;
	RTC_Datestruce.RTC_WeekDay=week;
	
	return RTC_SetDate(RTC_Format_BIN,&RTC_Datestruce);
}

/*--------------------------------
*函数：		rtc_set_alarm_a
*功能：		设置闹钟A
*输入：		
			week	：	星期几
			hour	：	小时
			minute	:	分钟
			second	：	秒
*输出：		无
*返回值：	
			0	：	成功
			1	：	失败
----------------------------------*/
int rtc_set_alarm_a(u8 week,u8 hour,u8 minute,u8 second)
{
	RTC_TimeTypeDef		RTC_Timestruct;
	RTC_AlarmTypeDef	RTC_Alarmstruct;
	EXTI_InitTypeDef	EXTI_initstruct;
	NVIC_InitTypeDef	NVIC_Initstruct;
	
	RTC_AlarmCmd(RTC_Alarm_A,DISABLE);//关闭闹钟A 
	
#if ALARM_A_CON   //设置系统闹钟
	RTC_Timestruct.RTC_Hours=hour;
	RTC_Timestruct.RTC_Minutes=minute;
	RTC_Timestruct.RTC_Seconds=second;
	RTC_Timestruct.RTC_H12=0x00;//24小时制
	
	RTC_Alarmstruct.RTC_AlarmDateWeekDay=week;
	RTC_Alarmstruct.RTC_AlarmDateWeekDaySel=0x40000000;//按星期闹
	RTC_Alarmstruct.RTC_AlarmMask=0x00000000;//精确到日时分秒
	RTC_Alarmstruct.RTC_AlarmTime=RTC_Timestruct;
	
	RTC_SetAlarm(RTC_Format_BIN,RTC_Alarm_A,&RTC_Alarmstruct);
	
#else	//设置用户时钟
	RTC_Timestruct.RTC_Hours=alarm_a->data[0];
	RTC_Timestruct.RTC_Minutes=alarm_a->data[1];
	RTC_Timestruct.RTC_Seconds=alarm_a->data[2];
	RTC_Timestruct.RTC_H12=0x00;//24小时制
	
	RTC_Alarmstruct.RTC_AlarmDateWeekDay=alarm_a->data[3];
	RTC_Alarmstruct.RTC_AlarmDateWeekDaySel=0x40000000;//按星期闹
	RTC_Alarmstruct.RTC_AlarmMask=0x00000000;//精确到日时分秒
	RTC_Alarmstruct.RTC_AlarmTime=RTC_Timestruct;
	
	RTC_SetAlarm(RTC_Format_BIN,RTC_Alarm_A,&RTC_Alarmstruct);
	
	
	
#endif
	
	RTC_ClearITPendingBit(RTC_IT_ALRA);//清除闹钟A中断标志
	EXTI_ClearITPendingBit(EXTI_Line17);//清除中断线17的中断位
	
	EXTI_initstruct.EXTI_Line=EXTI_Line17;
	EXTI_initstruct.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_initstruct.EXTI_Trigger=EXTI_Trigger_Rising;//上升沿触发
	EXTI_initstruct.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_initstruct);
	
	RTC_ITConfig(RTC_IT_ALRA,ENABLE);//使能闹钟A中断
	RTC_AlarmCmd(RTC_Alarm_A,ENABLE);//使能闹钟
	
	NVIC_Initstruct.NVIC_IRQChannel=RTC_Alarm_IRQn;
	NVIC_Initstruct.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_Initstruct.NVIC_IRQChannelSubPriority=0;
	NVIC_Initstruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_Initstruct);
	
	return 0;
}

/*---------------------------------------
*函数：		showtime
*功能：		显示时间
*返回值：	无
-----------------------------------------*/
void showtime()
{



	RTC_TimeTypeDef RTC_TimeStruct;
	RTC_DateTypeDef RTC_DateStruct;
	
	u8 time_buf[50];
	u8 date_buf[50];
	RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
	RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);	
	sprintf((char*)time_buf,"Time:%02d:%02d:%02d",RTC_TimeStruct.RTC_Hours,RTC_TimeStruct.RTC_Minutes,RTC_TimeStruct.RTC_Seconds); 
	sprintf((char*)date_buf,"Date:20%02d-%02d-%02d",RTC_DateStruct.RTC_Year,RTC_DateStruct.RTC_Month,RTC_DateStruct.RTC_Date/*,RTC_DateStruct.RTC_WeekDay*/); 
	// OLED_ShowString(5,36,time_buf,12);
	// OLED_ShowString(5,48,date_buf,12);
#ifdef DEBUG_TEST		
	printf("%s\n",time_buf);
	printf("%s\n",date_buf);	
#endif	
}

/*---------------------------------------
*函数：		RTC_Alarm_IRQHandler
*功能：		RTC闹钟中断服务函数
*返回值：	无
-----------------------------------------*/
void RTC_Alarm_IRQHandler(void)
{    
	if(RTC_GetFlagStatus(RTC_FLAG_ALRAF)==SET)//ALARM A中断
	{
		RTC_ClearFlag(RTC_FLAG_ALRAF);//清除中断标志
		switch(cmd)
		{
			case 0x01:
			#ifdef DEBUG_TEST
				printf("吃药时间到啦!\r\n");
			#endif
				OLED_Clear();
				OLED_ShowString(10,10,"time to take medicine",12);
				OLED_Refresh_Gram();
				break;
			case 0x02:
			#ifdef DEBUG_TEST
				printf("起床时间到啦\r\n");
			#endif
				OLED_Clear();
				OLED_ShowString(10,10,"time to wake up",12);
				OLED_Refresh_Gram();
				break;
			case 0x03:
			#ifdef DEBUG_TEST
				printf("吃饭时间到啦!\r\n");
			#endif
				OLED_Clear();
				OLED_ShowString(10,10,"time to eat ",12);
				OLED_Refresh_Gram();				
				break;
			case 0x04:
			#ifdef DEBUG_TEST
				printf("睡觉时间到啦!\r\n");
			#endif
				OLED_Clear();
				OLED_ShowString(10,10,"time to sleep",12);
				OLED_Refresh_Gram();				
				break;
			default :
				break;
				
		}
		//printf("ALARM A!\r\n");
		delay_us(500);
		OLED_Clear();
	}   
	EXTI_ClearITPendingBit(EXTI_Line17);	//清除中断线17的中断标志 											 
}
