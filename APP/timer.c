/*
*file timer.c
*/

#include "stm32f4xx.h"
#include "timer.h"
#include "FallDetection.h"
/*---------------------------------
*������		Tim2_init
*���ܣ�		��ʱ��2��ʼ��������ADXL345��ʱ�ж�ʱ��
*������		arr	:	��װ��ֵ
*������		psc	��	Ԥ��Ƶֵ
*����ֵ��	��
----------------------------------*/
void Tim2_init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE); 
	
	//��ʼ����ʱ������
	TIM_TimeBaseInitStructure.TIM_Period=arr-1;
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc-1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;//42mhz
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
	
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x06; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; 
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//TIM_Cmd(TIM2,ENABLE);
	TIM_Cmd(TIM2,DISABLE);
}

/*
*TIM_IRQhandle����
*/
// extern unsigned char DetectionStatus; 
// extern unsigned char TimerWaitForStable; // ײ����ȴ��ȶ���ʱ�������
// extern unsigned char TimerWaitForStrike; // ʧ�غ�ȴ�ײ����ʱ�������
// extern unsigned char TimerFreeFall; // ����FF��ʱ������� 
// extern short InitialStatus[3]; //  X-, Y-, Z- ��ĳ�ʼ״̬���ٶ� 
// extern short Acceleration[3]; // X-, Y-, Z- ��ĵ�ǰ���ٶ�
// extern unsigned long int DeltaAcceleration[3]; // ��ǰ���ٶ����ʼ��״̬���ٶȵĲ�ֵ
// extern unsigned long int DeltaVectorSum; // ���ٶȲ�ֵ��������
extern u8 buzzer;
/*------------------------------------------
*������		TIM2_IRQHandler
*���ܣ�		��ʱ��2�жϴ�����
*������		��
*����ֵ��	��
--------------------------------------------*/
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)//��ʱ���жϣ����20ms
	{
		if(DetectionStatus == 0xF2)//ʧ�غ��⵽ײ�����ȴ���ֹ
		{
			TimerWaitForStable ++;
			//�ȴ���ֹʱ�䳬��3.5s����Ϊ����Ч����
			if(TimerWaitForStable >= STABLE_WINDOW)
			{
				TIM_Cmd(TIM2,DISABLE); //��Ч������disable��ʱ��
				DetectionStatus=0xF0;
				buzzer=0;
				#ifdef DEBUG_TEST
				printshow(DetectionStatus);
				#endif
				ADXL345_WR_Reg(XL345_THRESH_ACT,STRIKE_THRESHOLD);//2g
				ADXL345_WR_Reg(XL345_THRESH_INACT,NOMOVEMENT_THRESHOLD);//0.1875g
				ADXL345_WR_Reg(XL345_TIME_INACT,STABLE_TIME);//2s
				ADXL345_WR_Reg(XL345_ACT_INACT_CTL,0x7F);
			}
		}
		else if(DetectionStatus == 0xF1)//��⵽ʧ�أ��ȴ�ײ��
		{
			TimerWaitForStrike++;
			//�ȴ�ײ��ʱ�䳬��200ms����Ϊ����Ч����
			if(TimerWaitForStrike >= STRIKE_WINDOW)
			{
				TIM_Cmd(TIM2,DISABLE); //��Ч������disable��ʱ��
				DetectionStatus=0xF0;
				buzzer=0;
				#ifdef DEBUG_TEST
				printshow(DetectionStatus);
				#endif
				ADXL345_WR_Reg(XL345_THRESH_ACT,STRIKE_THRESHOLD);
				ADXL345_WR_Reg(XL345_THRESH_INACT,NOMOVEMENT_THRESHOLD);
				ADXL345_WR_Reg(XL345_TIME_INACT,STABLE_TIME);
				ADXL345_WR_Reg(XL345_ACT_INACT_CTL,0x7F);
			}
		}
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);//���㶨ʱ���ж�
	}	
}



