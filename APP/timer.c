/*
*file timer.c
*/

#include "stm32f4xx.h"
#include "timer.h"
#include "FallDetection.h"
/*---------------------------------
*函数：		Tim2_init
*功能：		定时器2初始化，用作ADXL345定时中断时基
*参数：		arr	:	重装载值
*参数：		psc	：	预分频值
*返回值：	无
----------------------------------*/
void Tim2_init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE); 
	
	//初始化定时器配置
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
*TIM_IRQhandle函数
*/
// extern unsigned char DetectionStatus; 
// extern unsigned char TimerWaitForStable; // 撞击后等待稳定的时间计数器
// extern unsigned char TimerWaitForStrike; // 失重后等待撞击的时间计数器
// extern unsigned char TimerFreeFall; // 连续FF的时间计数器 
// extern short InitialStatus[3]; //  X-, Y-, Z- 轴的初始状态加速度 
// extern short Acceleration[3]; // X-, Y-, Z- 轴的当前加速度
// extern unsigned long int DeltaAcceleration[3]; // 当前加速度与初始化状态加速度的差值
// extern unsigned long int DeltaVectorSum; // 加速度差值的向量和
extern u8 buzzer;
/*------------------------------------------
*函数：		TIM2_IRQHandler
*功能：		定时器2中断处理函数
*参数：		无
*返回值：	无
--------------------------------------------*/
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET)//定时器中断，间隔20ms
	{
		if(DetectionStatus == 0xF2)//失重后检测到撞击，等待静止
		{
			TimerWaitForStable ++;
			//等待静止时间超过3.5s则认为是无效跌倒
			if(TimerWaitForStable >= STABLE_WINDOW)
			{
				TIM_Cmd(TIM2,DISABLE); //无效跌倒，disable定时器
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
		else if(DetectionStatus == 0xF1)//检测到失重，等待撞击
		{
			TimerWaitForStrike++;
			//等待撞击时间超过200ms则认为是无效跌倒
			if(TimerWaitForStrike >= STRIKE_WINDOW)
			{
				TIM_Cmd(TIM2,DISABLE); //无效跌倒，disable定时器
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
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);//清零定时器中断
	}	
}



