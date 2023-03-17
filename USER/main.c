/*
*file main.c
UWB��λ����
��д��2022 3 18 ��һ��
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
// ��������main
// ����  ��������
// ����  ����
// ���  ����
/*********************************************************/
int main(void)
{  
/*----------------------��ʼ��оƬ����*/
	peripherals_init();						//�����ʼ��		
/*----------------------end*/

/*----------------------��ʼ������ģ��*/
#ifdef TX_NODE
	/*-------���û�����������*/
	MQ_port_init();			
	ds18b20_port_init();
	rtc_time_init();
	/*--------end*/
	
	while(ADXL345_Init());					//����ADCL345
	 max30102_init();						//����MAX30102

	//OLED_Init();			//��ʾ��
#endif	
/*------------------------end*/

/*--------------------------��ʼ��DW1000������*/
	reset_DW1000();						
	if(dw1000_init()==-1)					//DW1000��ʼ��
	{
		return -1;
	}

	dw1000_msg_init();						//DW1000��Ϣ����ʼ��
	dw1000_config();						//DW1000���ó�ʼ��
/*-------------------------end*/


#ifdef TX_NODE
		TIM3_init();	
	 /*-------��������ADXL345���������*/
	  Tim2_init(100,8400);//20ms�жϼ��,������adlx345
	  adxl345_exti();//����adxl345�ⲿ�ж�
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



