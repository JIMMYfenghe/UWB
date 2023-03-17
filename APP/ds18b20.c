/*
*file ds18b20.c
*该功能是DS18B20温度传感器驱动代码
*通信接口 :
			PA0			DQ

*编写于2022 03 03 第一版
*/
#include "stm32f4xx.h"
#include "delay.h"
#include "ds18b20.h"
#include "stdio.h"
#include "oled.h"

/*------------------------------
*函数：		ds18b20_port_init
*功能：		ds18b20端口 初始化
*输入：		无
*输出：		无
*返回值：	无
------------------------------*/
void ds18b20_port_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//DQ总线
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//--推免输出模式
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  

	ds18b20_reset();
	ds18b20_check();	

}

/*-------------------------------------------
*函数：		read_bit
*功能：		从ds18b20读一个位，每个读时序必须保持至少为60us
*输入：		无
*输出：		无
*返回值：	(u8)date	:	读取的位
-------------------------------------------*/
u8 read_bit()
{
	
	u8 data;
	DQ_IO_OUT();//输出模式
	
	DQ_TO_LOW();//拉低总线
	delay_us(2);
	DQ_TO_HIGH();//拉高总线
	
	DQ_IO_IN();//输入模式
	delay_us(12);
	
	
	
	//读取IO状态
	if( DQ_STATUS() )
		data=1;
	else
		data=0;
	delay_us(50);
	return data;
}

/*---------------------------------
*函数：		read_byte
*功能：		读取一个字节
*输入：		无
*输出：		无
*返回值：	(u8)data	:	返回的字节
----------------------------------*/
u8 read_byte()
{
	u8 data=0;
	int i;
	for(i=0;i<8;i++)
	{
		data=data>>1;
		if(read_bit())
			data |=0x80;//最低位先读
	}
	return data;
}

/*----------------------------
*函数：		write_byte	
*功能：		写一个字节
*输入：		data	:	写入的数据
*输出：		无
*返回值：	无
-----------------------------*/
void write_byte(u8 data)
{
	int i;
	DQ_IO_OUT();//输出模式
	for(i=0;i<8;i++)
	{
		if(data & 0x01)//写1
		{	
			DQ_TO_LOW();
			delay_us(2);
			DQ_TO_HIGH();
			delay_us(60);
		}
		else //写0
		{
			DQ_TO_LOW();
			delay_us(60);
			DQ_TO_HIGH();
			delay_us(2);
		}
		
		data=data>>1;
	}
}

/*----------------------------------
*函数：		ds18b20_reset
*功能：		发送复位脉冲
*输入：		无
*输出：		无
*返回值：	无
----------------------------------*/
void ds18b20_reset()
{
	DQ_IO_OUT();//输出模式
	DQ_TO_LOW();
	delay_us(750);//至少480us
	DQ_TO_HIGH();
	delay_us(15);//15-60us
}
/*---------------------------------
*函数：		ds18b20_check
*功能：		检测存在脉冲
*返回值：	
			成功	返回0
			失败	返回1
----------------------------------*/
int ds18b20_check()
{
	//检测存在脉冲理应在60-240us内引脚被拉低
	uint8_t retry=0;
	DQ_IO_IN();			
	while(DQ_STATUS() && retry<200) //检验引脚是否在200us内没有被拉低
	{
		retry++;
		delay_us(1);
	}
	if(retry>=200) 
		return 1;	//超过200us时间引脚一直处于高电平
    else 
		retry=0;
	while(!DQ_STATUS() && retry<240)// 200us内有应答，检查低电平保持时间是否在60-240区间
	{
		retry++;
		delay_us(1);
	}
	if(retry>=240)
		return 1;  //    应答超时

    return 0; 			

}


/*----------------------------------------------------
任何事务必无限重复三步骤：复位检测，ROM命令，功能命令。
--------------------------------------------------*/

/*--------------------------------------------------
*函数：		ds18b20_conversion
*功能：		发出转换命令
*返回值：	无
----------------------------------------------------*/
void ds18b20_conversion()
{
	ds18b20_reset();
	ds18b20_check();
	write_byte(0xcc);//SKIP ROM,如只有一个设备，可选择跳过此命令
	write_byte(0x44);//CONVERT TEMP
}

/*---------------------------------------------------
*函数：		ds18b20_readtemp_cmd
*功能：		发出读取温度指令
*返回值：	无
--------------------------------------------------*/
void ds18b20_readtemp_cmd()
{
	ds18b20_reset();
	ds18b20_check();
	write_byte(0xcc);//SKIP ROM,如只有一个设备，可选择跳过此命令
	write_byte(0xbe);//READ SCRATCHPAD
}

/*-------------------------------------------------
*函数：		ds18b20_get_temp
*功能：		获取温度数值
*返回值：	tempcalue	:	温度值
--------------------------------------------------*/
float ds18b20_get_temp()
{
	float tempvalue;
	int temp=1;
	u8 TL,TH;
	u16 TB;
	
	ds18b20_conversion();
	ds18b20_readtemp_cmd();
	
	TL=read_byte();
	TH=read_byte();
	if(TH>7)
	{
		TL=~TL;//温度为负时，数值变换
		TH=-TH;
		temp=0;
	}
	TB=(TH<<8) | TL;

	tempvalue=TB*0.625;//默认12位分辨率，0.0625精度，扩大十倍方便运算
	if(temp)
		return tempvalue;
	else
		return -tempvalue;
	
}

/*-------------------------------------------
*函数：		show_temp
*功能：		显示温度
*返回值：	无
--------------------------------------------*/
void show_temp()
{
		char str[20];
		sprintf(str,"%.1f",ds18b20_get_temp()/10);
	#ifdef DEBUG_TEST
		printf("temp:%s\n",str);
	#endif
		// OLED_ShowString(5,24,"temp:",12);
		// OLED_ShowString(35,24,str,12);		
		// OLED_ShowChar(70,24,'C',12,1);	

}
