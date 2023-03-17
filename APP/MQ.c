/*
*file MQ7.c 
ADC IO:
	PA1  ADC1_1
	PA2	 ADC1_2
*/
#if 1

#include "stm32f4xx.h"
#include "MQ.h"
#include "math.h"
#include "stdio.h"
#include "timer.h"
#include "oled.h"

float MQ7_R0=6,MQ2_R0=6;

#define adc_bufsize 60
u16 ADC1_buffer[adc_bufsize]={0};
u16 ADC1_avgvalue[2];
/*------------------------
*函数：		MQ_port_init
*功能：		初始化MQ端口和ADC采集模式
*参数：		无
*返回值：	无
--------------------------*/
void MQ_port_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef		ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);
	/*GPIO init*/
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;//mq2
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN;//模拟输入
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉

	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;//mq7
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN;//模拟输入
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
	
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	ADC_DeInit();//ADC复位
	
	/*ADC init*/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;//扫描模式	
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//连续转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
	ADC_InitStructure.ADC_NbrOfConversion = 2;//2;//2个转换在规则序列中 也就是只转换规则序列2
	ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div6;//预分频6分频。ADCCLK=PCLK2/6=84/6=14Mhz,ADC时钟最好不要超过36Mhz 
	ADC_CommonInit(&ADC_CommonInitStructure);//初始化

	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_480Cycles);//排在第一序列，通道1，480周期采样
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_480Cycles);//排在第二序列，通道2，480周期采样

	DMA_ADC_config(DMA2_Stream0,DMA_Channel_0,(u32)&ADC1->DR,(u32)ADC1_buffer,adc_bufsize);//配置DMA adc1
	
	

	ADC_DMARequestAfterLastTransferCmd(ADC1,ENABLE);

	ADC_DMACmd(ADC1,ENABLE);
	
	ADC_Cmd(ADC1,ENABLE);
	
	ADC_SoftwareStartConv(ADC1);//开始转换

}

/*-----------------------------------------
*函数：		DMA_ADC_config
*功能：		ADC_DMA配置
*输入：		
*			DMA_Streamx	：	DMA流
*			chx			：	通道
*			per_addr	：	外设地址
*			target_addr	：	目标内存地址
*			ndtr		：	传输的数量
*输出：		无	
*返回值：	无	
-----------------------------------------*/
void DMA_ADC_config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 per_addr,u32 target_addr,u16 ndtr )
{
	DMA_InitTypeDef  DMA_InitStructure;
		
	if((u32)DMA_Streamx>(u32)DMA2)//得到当前stream是属于DMA2还是DMA1
	{
		 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能 
			
	}else 
	{
		 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 
	}
	DMA_DeInit(DMA_Streamx);
		
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}//等待DMA可配置 
		
	/* 配置 DMA Stream */
	DMA_InitStructure.DMA_Channel = chx;  //通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr = per_addr;//DMA外设地址
   	DMA_InitStructure.DMA_Memory0BaseAddr = target_addr;//DMA 存储器0地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设到内存
	DMA_InitStructure.DMA_BufferSize = ndtr;//数据传输量 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//外设数据长度:16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;//存储器数据长度:16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Normal;// 使用循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//中等优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_Init(DMA_Streamx, &DMA_InitStructure);//初始化DMA Stream
	

	DMA_Cmd(DMA_Streamx,ENABLE);

	
}
/*----------------------------------------
*函数：		MQ_get_ad_value
*功能：		获得MQ AD值
*输入：		无
*输出：		无
*返回值：	无
----------------------------------------*/
void MQ_get_ad_value()
{
		int i,status=0;
		while(!DMA_GetFlagStatus(DMA2_Stream0,DMA_FLAG_TCIF0))
		{
			status=ADC_GetFlagStatus(ADC1,ADC_FLAG_OVR);
			if(status)//ADC溢出,重新初始化ADC和DMA并开启转换
			{
				ADC_ClearFlag(ADC1,ADC_FLAG_OVR);
				MQ_port_init();
				
			}
	
		}
		//status=ADC_GetFlagStatus(ADC1,ADC_FLAG_OVR);
		//DMA_Cmd(DMA2_Stream0,DISABLE);//失能DMA2流0
		ADC_DMACmd(ADC1,DISABLE);
		DMA_ClearFlag(DMA2_Stream0,DMA_FLAG_TCIF0);
		ADC1_avgvalue[0]=0;
		ADC1_avgvalue[1]=0;
		for(i=0;i<60;i++)
		{
			if((i%2)==0)
				ADC1_avgvalue[0]+=ADC1_buffer[i];
			else
				ADC1_avgvalue[1]+=ADC1_buffer[i];
		}
		ADC1_avgvalue[0]=ADC1_avgvalue[0]/30;
		ADC1_avgvalue[1]=ADC1_avgvalue[1]/30;
		DMA_SetCurrDataCounter(DMA2_Stream0,adc_bufsize);
		//DMA_Cmd(DMA2_Stream0,ENABLE);//使能DMA2流0	
		ADC_DMACmd(ADC1,ENABLE);		
}

/*--------------------------------------
*函数：		MQ7_calibration
*功能：		校准函数,pow来自数学库math.h，求取x的y次幂
*输入：		RS	:	当前器件的电阻值
*输出：		无
*返回值：	无
--------------------------------------*/
void MQ7_calibration(float RS)
{
	MQ7_R0 = RS / pow(CO_CAL_PPM / 98.322, 1 / -1.458f);
}

void MQ2_calibration(float RS)
{
	MQ2_R0 = RS / pow(CAL_PPM_SMOKE / 613.9f, 1 / -2.074f);
}

/*-------------------------------------
*函数：		MQ_get_ppm
*功能：		获得PPM
*输入：		无
*输出：		*buffer	:	当前的浓度
*返回值：	
			0	：	成功
			1	：	失败
----------------------------------------*/
int timers=0;
float MQ_get_ppm(float *buffer)
{
	float mq2_ppm,mq2_Vrl,mq2_RS;
	float mq7_ppm,mq7_Vrl,mq7_RS;
	MQ_get_ad_value();
	mq2_Vrl = 3.3f * ADC1_avgvalue[0] / 4095.f;
	mq2_Vrl = ( (float)( (int)( (mq2_Vrl+0.005f)*100 ) ) )/100;
    mq2_RS = (3.3f - mq2_Vrl) / mq2_Vrl * MQ2_RL;//即时的器件电阻

	mq7_Vrl = 3.3f * ADC1_avgvalue[1] / 4095.f;
	mq7_Vrl = ( (float)( (int)( (mq7_Vrl+0.005f)*100 ) ) )/100;
    mq7_RS = (3.3f - mq7_Vrl) / mq7_Vrl * MQ7_RL;//即时的器件电阻
    if(timers<30) // 定时器3在5s内校准
    {
		//MQ7_mutex_signal=0;
		MQ7_calibration(mq7_RS);
		MQ2_calibration(mq2_RS);
    }
    mq7_ppm = 98.322f * pow(mq7_RS/MQ7_R0, -1.458f);
	buffer[1]=mq7_ppm;
	mq2_ppm = 613.9f * pow(mq2_RS/MQ2_R0, -2.074f);
    buffer[0]=mq2_ppm;
	return  0;

}

float ppm_buffer[2];
char mq2_ppm[20];
char mq7_ppm[20];
/*---------------------------------
*函数：		show_ppm
*功能：		显示浓度
*输入：		无
*输出：		无
*返回值：	无
---------------------------------*/
extern void deca_sleep(unsigned int time_ms);
void show_ppm()
{
	MQ_get_ppm(ppm_buffer);
	
	sprintf(mq2_ppm,"MQ2_PPM:%3.2f",ppm_buffer[0]);
	sprintf(mq7_ppm,"MQ7_PPM:%3.2f",ppm_buffer[1]);
	// OLED_ShowString(5,2,mq2_ppm,12);
	// OLED_ShowString(5,14,mq7_ppm,12);
#ifdef DEBUG_TEST
	printf("%s,%s\n",mq2_ppm,mq7_ppm);
#endif
	// deca_sleep(500);
}

#endif



