#ifndef __RTC_TIME_H
#define __RTC_TIME_H

#include "stm32f4xx.h"
//#include "usart1.h"

//#define ALARM_A_CON  1 //系统设置闹钟
#define ALARM_A_CON  0	//用户设置闹钟

int rtc_set_time(u8 hour,u8 minute,u8 second,u8 ampm);
int rtc_set_date(u8 year,u8 month,u8 day,u8 week);
int rtc_set_alarm_a(u8 week,u8 hour,u8 minute,u8 second);
u8 dec_to_hex(u8 dec);
void showtime(void);

int rtc_time_init(void);
#endif

