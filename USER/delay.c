/*********************************************************************************

**********************************************************************************/
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "delay.h"

// void delay_init(u8 SYSCLK)
// {
// 	//SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
// 	//SysTick->CTRL&=0xfffffffb;//选择内部时钟 HCLK/8
// 	fac_us=SYSCLK;		    
// }								    

//启动定时器中断后的us延时
void delay_us(u32 nus)
{
       u32 ticks;
       u32 told,tnow,reload,tcnt=0;

       reload = SysTick->LOAD;                     //获取重装载寄存器值
       ticks = nus * (SystemCoreClock / 1000000);  //计数时间值
       told=SysTick->VAL;                          //获取当前数值寄存器值（开始时数值）

       while(1)
       {
              tnow=SysTick->VAL;          //获取当前数值寄存器值
              if(tnow!=told)              //当前值不等于开始值说明已在计数
              {         

                     if(tnow<told)             //当前值小于开始数值，说明未计到0
                          tcnt+=told-tnow;     //计数值=开始值-当前值

                     else                  //当前值大于开始数值，说明已计到0并重新计数
                            tcnt+=reload-tnow+told;   //计数值=重装载值-当前值+开始值  （已
                                                      //从开始值计到0） 

                     told=tnow;                //更新开始值
                     if(tcnt>=ticks)break;     //时间超过/等于要延迟的时间,则退出.
              } 
       }     
}
//延时us								   
// void delay_us(u32 Nus)
// {		
// 	u32 temp;	    	 
// 	SysTick->LOAD=Nus*fac_us; //时间加载	  		 
// 	SysTick->VAL=0x00;        //清空计数器
// 	SysTick->CTRL=0x01 ;      //开始倒数 	 
// 	do
// 	{
// 		temp=SysTick->CTRL;
// 	}
// 	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
// 	SysTick->CTRL=0x00;       //关闭计数器
// 	SysTick->VAL =0X00;       //清空计数器	    
// }

