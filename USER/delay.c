/*********************************************************************************

**********************************************************************************/
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "delay.h"

// void delay_init(u8 SYSCLK)
// {
// 	//SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
// 	//SysTick->CTRL&=0xfffffffb;//ѡ���ڲ�ʱ�� HCLK/8
// 	fac_us=SYSCLK;		    
// }								    

//������ʱ���жϺ��us��ʱ
void delay_us(u32 nus)
{
       u32 ticks;
       u32 told,tnow,reload,tcnt=0;

       reload = SysTick->LOAD;                     //��ȡ��װ�ؼĴ���ֵ
       ticks = nus * (SystemCoreClock / 1000000);  //����ʱ��ֵ
       told=SysTick->VAL;                          //��ȡ��ǰ��ֵ�Ĵ���ֵ����ʼʱ��ֵ��

       while(1)
       {
              tnow=SysTick->VAL;          //��ȡ��ǰ��ֵ�Ĵ���ֵ
              if(tnow!=told)              //��ǰֵ�����ڿ�ʼֵ˵�����ڼ���
              {         

                     if(tnow<told)             //��ǰֵС�ڿ�ʼ��ֵ��˵��δ�Ƶ�0
                          tcnt+=told-tnow;     //����ֵ=��ʼֵ-��ǰֵ

                     else                  //��ǰֵ���ڿ�ʼ��ֵ��˵���ѼƵ�0�����¼���
                            tcnt+=reload-tnow+told;   //����ֵ=��װ��ֵ-��ǰֵ+��ʼֵ  ����
                                                      //�ӿ�ʼֵ�Ƶ�0�� 

                     told=tnow;                //���¿�ʼֵ
                     if(tcnt>=ticks)break;     //ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�.
              } 
       }     
}
//��ʱus								   
// void delay_us(u32 Nus)
// {		
// 	u32 temp;	    	 
// 	SysTick->LOAD=Nus*fac_us; //ʱ�����	  		 
// 	SysTick->VAL=0x00;        //��ռ�����
// 	SysTick->CTRL=0x01 ;      //��ʼ���� 	 
// 	do
// 	{
// 		temp=SysTick->CTRL;
// 	}
// 	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
// 	SysTick->CTRL=0x00;       //�رռ�����
// 	SysTick->VAL =0X00;       //��ռ�����	    
// }

