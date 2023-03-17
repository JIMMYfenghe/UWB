/*
file rtc_time.c
���ļ�����������ʱ�䣬����
*/


#include "stm32f4xx.h"
#include "delay.h"
#include "rtc_time.h"
#include "oled.h"
#include "port.h"


extern alarm_struct *alarm_a;//�����ⲿ����alarm_a
extern u8 cmd;//�����ⲿ����cmd

/*--------------------------------------
*������		rtc_time_init
*���ܣ�		rtc��ʼ��
*����ֵ��	
			0	:	�ɹ�
			1	:	ʧ��
---------------------------------------*/
extern void deca_sleep(unsigned int time_ms);
int rtc_time_init(void) 
{
	RTC_InitTypeDef		RTC_Initstruct;
	u16 retry=0x1fff;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE);//ʹ��PWRʱ��
	PWR_BackupAccessCmd(ENABLE);//ʹ��RTC�󱸼Ĵ���
	
	if(RTC_ReadBackupRegister(RTC_BKP_DR0)==0x5050)//�ж��Ƿ��һ������RTC,�üĴ�������ϵ粻��λ����Ȼ��������
	{
		RCC_LSEConfig(ENABLE);//����LSEʱ��
		
		while(RCC_GetFlagStatus(RCC_FLAG_LSERDY)==RESET)//�ȴ�LSE����
		{
			retry++;
			deca_sleep(10);
		}

		if(retry==0)
			return 1;
		// RCC_LSICmd(ENABLE);
		// while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY)==RESET);
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);//����RTCʱ��
		RCC_RTCCLKCmd(ENABLE);//ʹ��RTCʱ��
		
		//������ʼ��RTC
		//RTC_Initstruct.RTC_AsynchPrediv=0x7F;
		RTC_Initstruct.RTC_AsynchPrediv=0x7C;
		RTC_Initstruct.RTC_SynchPrediv=0XFF;
		RTC_Initstruct.RTC_HourFormat=RTC_HourFormat_24;
		RTC_Init(&RTC_Initstruct);
		
		rtc_set_time(10,53,00,RTC_H12_AM);
		rtc_set_date(22,5,17,2);
		
	}
	RTC_WriteBackupRegister(RTC_BKP_DR0,0x5050);//����ִ�г�ʼ��
	
	return 0;
}

/*------------------------------
*������		rtc_set_time
*���ܣ�		����ʱ��
*���룺		
			hour	:	Сʱ
			minute	��	����
			second	��	��
			ampm	��	��ʾ��ʽ
*�����		��
*����ֵ��	
			0	��	ʧ��
			1	��	�ɹ�
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
*������		rtc_set_date
*���ܣ�		��������
*���룺	
			year	:	��
			month	:	��
			day		��	��
			week	��	���ڼ�
*�����		��
*����ֵ��	
			0	��	ʧ��
			1	:	�ɹ�
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
*������		rtc_set_alarm_a
*���ܣ�		��������A
*���룺		
			week	��	���ڼ�
			hour	��	Сʱ
			minute	:	����
			second	��	��
*�����		��
*����ֵ��	
			0	��	�ɹ�
			1	��	ʧ��
----------------------------------*/
int rtc_set_alarm_a(u8 week,u8 hour,u8 minute,u8 second)
{
	RTC_TimeTypeDef		RTC_Timestruct;
	RTC_AlarmTypeDef	RTC_Alarmstruct;
	EXTI_InitTypeDef	EXTI_initstruct;
	NVIC_InitTypeDef	NVIC_Initstruct;
	
	RTC_AlarmCmd(RTC_Alarm_A,DISABLE);//�ر�����A 
	
#if ALARM_A_CON   //����ϵͳ����
	RTC_Timestruct.RTC_Hours=hour;
	RTC_Timestruct.RTC_Minutes=minute;
	RTC_Timestruct.RTC_Seconds=second;
	RTC_Timestruct.RTC_H12=0x00;//24Сʱ��
	
	RTC_Alarmstruct.RTC_AlarmDateWeekDay=week;
	RTC_Alarmstruct.RTC_AlarmDateWeekDaySel=0x40000000;//��������
	RTC_Alarmstruct.RTC_AlarmMask=0x00000000;//��ȷ����ʱ����
	RTC_Alarmstruct.RTC_AlarmTime=RTC_Timestruct;
	
	RTC_SetAlarm(RTC_Format_BIN,RTC_Alarm_A,&RTC_Alarmstruct);
	
#else	//�����û�ʱ��
	RTC_Timestruct.RTC_Hours=alarm_a->data[0];
	RTC_Timestruct.RTC_Minutes=alarm_a->data[1];
	RTC_Timestruct.RTC_Seconds=alarm_a->data[2];
	RTC_Timestruct.RTC_H12=0x00;//24Сʱ��
	
	RTC_Alarmstruct.RTC_AlarmDateWeekDay=alarm_a->data[3];
	RTC_Alarmstruct.RTC_AlarmDateWeekDaySel=0x40000000;//��������
	RTC_Alarmstruct.RTC_AlarmMask=0x00000000;//��ȷ����ʱ����
	RTC_Alarmstruct.RTC_AlarmTime=RTC_Timestruct;
	
	RTC_SetAlarm(RTC_Format_BIN,RTC_Alarm_A,&RTC_Alarmstruct);
	
	
	
#endif
	
	RTC_ClearITPendingBit(RTC_IT_ALRA);//�������A�жϱ�־
	EXTI_ClearITPendingBit(EXTI_Line17);//����ж���17���ж�λ
	
	EXTI_initstruct.EXTI_Line=EXTI_Line17;
	EXTI_initstruct.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_initstruct.EXTI_Trigger=EXTI_Trigger_Rising;//�����ش���
	EXTI_initstruct.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_initstruct);
	
	RTC_ITConfig(RTC_IT_ALRA,ENABLE);//ʹ������A�ж�
	RTC_AlarmCmd(RTC_Alarm_A,ENABLE);//ʹ������
	
	NVIC_Initstruct.NVIC_IRQChannel=RTC_Alarm_IRQn;
	NVIC_Initstruct.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_Initstruct.NVIC_IRQChannelSubPriority=0;
	NVIC_Initstruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_Initstruct);
	
	return 0;
}

/*---------------------------------------
*������		showtime
*���ܣ�		��ʾʱ��
*����ֵ��	��
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
*������		RTC_Alarm_IRQHandler
*���ܣ�		RTC�����жϷ�����
*����ֵ��	��
-----------------------------------------*/
void RTC_Alarm_IRQHandler(void)
{    
	if(RTC_GetFlagStatus(RTC_FLAG_ALRAF)==SET)//ALARM A�ж�
	{
		RTC_ClearFlag(RTC_FLAG_ALRAF);//����жϱ�־
		switch(cmd)
		{
			case 0x01:
			#ifdef DEBUG_TEST
				printf("��ҩʱ�䵽��!\r\n");
			#endif
				OLED_Clear();
				OLED_ShowString(10,10,"time to take medicine",12);
				OLED_Refresh_Gram();
				break;
			case 0x02:
			#ifdef DEBUG_TEST
				printf("��ʱ�䵽��\r\n");
			#endif
				OLED_Clear();
				OLED_ShowString(10,10,"time to wake up",12);
				OLED_Refresh_Gram();
				break;
			case 0x03:
			#ifdef DEBUG_TEST
				printf("�Է�ʱ�䵽��!\r\n");
			#endif
				OLED_Clear();
				OLED_ShowString(10,10,"time to eat ",12);
				OLED_Refresh_Gram();				
				break;
			case 0x04:
			#ifdef DEBUG_TEST
				printf("˯��ʱ�䵽��!\r\n");
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
	EXTI_ClearITPendingBit(EXTI_Line17);	//����ж���17���жϱ�־ 											 
}
