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
*������		MQ_port_init
*���ܣ�		��ʼ��MQ�˿ں�ADC�ɼ�ģʽ
*������		��
*����ֵ��	��
--------------------------*/
void MQ_port_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef		ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);
	/*GPIO init*/
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;//mq2
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN;//ģ������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������

	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;//mq7
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AN;//ģ������
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
	
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	ADC_DeInit();//ADC��λ
	
	/*ADC init*/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12λģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;//ɨ��ģʽ	
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//����ת��
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//��ֹ������⣬ʹ���������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���	
	ADC_InitStructure.ADC_NbrOfConversion = 2;//2;//2��ת���ڹ��������� Ҳ����ֻת����������2
	ADC_Init(ADC1, &ADC_InitStructure);//ADC��ʼ��
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//���������׶�֮����ӳ�5��ʱ��
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMAʧ��
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div6;//Ԥ��Ƶ6��Ƶ��ADCCLK=PCLK2/6=84/6=14Mhz,ADCʱ����ò�Ҫ����36Mhz 
	ADC_CommonInit(&ADC_CommonInitStructure);//��ʼ��

	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_480Cycles);//���ڵ�һ���У�ͨ��1��480���ڲ���
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_480Cycles);//���ڵڶ����У�ͨ��2��480���ڲ���

	DMA_ADC_config(DMA2_Stream0,DMA_Channel_0,(u32)&ADC1->DR,(u32)ADC1_buffer,adc_bufsize);//����DMA adc1
	
	

	ADC_DMARequestAfterLastTransferCmd(ADC1,ENABLE);

	ADC_DMACmd(ADC1,ENABLE);
	
	ADC_Cmd(ADC1,ENABLE);
	
	ADC_SoftwareStartConv(ADC1);//��ʼת��

}

/*-----------------------------------------
*������		DMA_ADC_config
*���ܣ�		ADC_DMA����
*���룺		
*			DMA_Streamx	��	DMA��
*			chx			��	ͨ��
*			per_addr	��	�����ַ
*			target_addr	��	Ŀ���ڴ��ַ
*			ndtr		��	���������
*�����		��	
*����ֵ��	��	
-----------------------------------------*/
void DMA_ADC_config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 per_addr,u32 target_addr,u16 ndtr )
{
	DMA_InitTypeDef  DMA_InitStructure;
		
	if((u32)DMA_Streamx>(u32)DMA2)//�õ���ǰstream������DMA2����DMA1
	{
		 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2ʱ��ʹ�� 
			
	}else 
	{
		 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1ʱ��ʹ�� 
	}
	DMA_DeInit(DMA_Streamx);
		
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}//�ȴ�DMA������ 
		
	/* ���� DMA Stream */
	DMA_InitStructure.DMA_Channel = chx;  //ͨ��ѡ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = per_addr;//DMA�����ַ
   	DMA_InitStructure.DMA_Memory0BaseAddr = target_addr;//DMA �洢��0��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//���赽�ڴ�
	DMA_InitStructure.DMA_BufferSize = ndtr;//���ݴ����� 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//�������ݳ���:16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord;//�洢�����ݳ���:16λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Normal;// ʹ��ѭ��ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//�е����ȼ�
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
	DMA_Init(DMA_Streamx, &DMA_InitStructure);//��ʼ��DMA Stream
	

	DMA_Cmd(DMA_Streamx,ENABLE);

	
}
/*----------------------------------------
*������		MQ_get_ad_value
*���ܣ�		���MQ ADֵ
*���룺		��
*�����		��
*����ֵ��	��
----------------------------------------*/
void MQ_get_ad_value()
{
		int i,status=0;
		while(!DMA_GetFlagStatus(DMA2_Stream0,DMA_FLAG_TCIF0))
		{
			status=ADC_GetFlagStatus(ADC1,ADC_FLAG_OVR);
			if(status)//ADC���,���³�ʼ��ADC��DMA������ת��
			{
				ADC_ClearFlag(ADC1,ADC_FLAG_OVR);
				MQ_port_init();
				
			}
	
		}
		//status=ADC_GetFlagStatus(ADC1,ADC_FLAG_OVR);
		//DMA_Cmd(DMA2_Stream0,DISABLE);//ʧ��DMA2��0
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
		//DMA_Cmd(DMA2_Stream0,ENABLE);//ʹ��DMA2��0	
		ADC_DMACmd(ADC1,ENABLE);		
}

/*--------------------------------------
*������		MQ7_calibration
*���ܣ�		У׼����,pow������ѧ��math.h����ȡx��y����
*���룺		RS	:	��ǰ�����ĵ���ֵ
*�����		��
*����ֵ��	��
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
*������		MQ_get_ppm
*���ܣ�		���PPM
*���룺		��
*�����		*buffer	:	��ǰ��Ũ��
*����ֵ��	
			0	��	�ɹ�
			1	��	ʧ��
----------------------------------------*/
int timers=0;
float MQ_get_ppm(float *buffer)
{
	float mq2_ppm,mq2_Vrl,mq2_RS;
	float mq7_ppm,mq7_Vrl,mq7_RS;
	MQ_get_ad_value();
	mq2_Vrl = 3.3f * ADC1_avgvalue[0] / 4095.f;
	mq2_Vrl = ( (float)( (int)( (mq2_Vrl+0.005f)*100 ) ) )/100;
    mq2_RS = (3.3f - mq2_Vrl) / mq2_Vrl * MQ2_RL;//��ʱ����������

	mq7_Vrl = 3.3f * ADC1_avgvalue[1] / 4095.f;
	mq7_Vrl = ( (float)( (int)( (mq7_Vrl+0.005f)*100 ) ) )/100;
    mq7_RS = (3.3f - mq7_Vrl) / mq7_Vrl * MQ7_RL;//��ʱ����������
    if(timers<30) // ��ʱ��3��5s��У׼
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
*������		show_ppm
*���ܣ�		��ʾŨ��
*���룺		��
*�����		��
*����ֵ��	��
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



