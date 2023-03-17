/*
*file ds18b20.c
*�ù�����DS18B20�¶ȴ�������������
*ͨ�Žӿ� :
			PA0			DQ

*��д��2022 03 03 ��һ��
*/
#include "stm32f4xx.h"
#include "delay.h"
#include "ds18b20.h"
#include "stdio.h"
#include "oled.h"

/*------------------------------
*������		ds18b20_port_init
*���ܣ�		ds18b20�˿� ��ʼ��
*���룺		��
*�����		��
*����ֵ��	��
------------------------------*/
void ds18b20_port_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	//DQ����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//--�������ģʽ
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);  

	ds18b20_reset();
	ds18b20_check();	

}

/*-------------------------------------------
*������		read_bit
*���ܣ�		��ds18b20��һ��λ��ÿ����ʱ����뱣������Ϊ60us
*���룺		��
*�����		��
*����ֵ��	(u8)date	:	��ȡ��λ
-------------------------------------------*/
u8 read_bit()
{
	
	u8 data;
	DQ_IO_OUT();//���ģʽ
	
	DQ_TO_LOW();//��������
	delay_us(2);
	DQ_TO_HIGH();//��������
	
	DQ_IO_IN();//����ģʽ
	delay_us(12);
	
	
	
	//��ȡIO״̬
	if( DQ_STATUS() )
		data=1;
	else
		data=0;
	delay_us(50);
	return data;
}

/*---------------------------------
*������		read_byte
*���ܣ�		��ȡһ���ֽ�
*���룺		��
*�����		��
*����ֵ��	(u8)data	:	���ص��ֽ�
----------------------------------*/
u8 read_byte()
{
	u8 data=0;
	int i;
	for(i=0;i<8;i++)
	{
		data=data>>1;
		if(read_bit())
			data |=0x80;//���λ�ȶ�
	}
	return data;
}

/*----------------------------
*������		write_byte	
*���ܣ�		дһ���ֽ�
*���룺		data	:	д�������
*�����		��
*����ֵ��	��
-----------------------------*/
void write_byte(u8 data)
{
	int i;
	DQ_IO_OUT();//���ģʽ
	for(i=0;i<8;i++)
	{
		if(data & 0x01)//д1
		{	
			DQ_TO_LOW();
			delay_us(2);
			DQ_TO_HIGH();
			delay_us(60);
		}
		else //д0
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
*������		ds18b20_reset
*���ܣ�		���͸�λ����
*���룺		��
*�����		��
*����ֵ��	��
----------------------------------*/
void ds18b20_reset()
{
	DQ_IO_OUT();//���ģʽ
	DQ_TO_LOW();
	delay_us(750);//����480us
	DQ_TO_HIGH();
	delay_us(15);//15-60us
}
/*---------------------------------
*������		ds18b20_check
*���ܣ�		����������
*����ֵ��	
			�ɹ�	����0
			ʧ��	����1
----------------------------------*/
int ds18b20_check()
{
	//������������Ӧ��60-240us�����ű�����
	uint8_t retry=0;
	DQ_IO_IN();			
	while(DQ_STATUS() && retry<200) //���������Ƿ���200us��û�б�����
	{
		retry++;
		delay_us(1);
	}
	if(retry>=200) 
		return 1;	//����200usʱ������һֱ���ڸߵ�ƽ
    else 
		retry=0;
	while(!DQ_STATUS() && retry<240)// 200us����Ӧ�𣬼��͵�ƽ����ʱ���Ƿ���60-240����
	{
		retry++;
		delay_us(1);
	}
	if(retry>=240)
		return 1;  //    Ӧ��ʱ

    return 0; 			

}


/*----------------------------------------------------
�κ�����������ظ������裺��λ��⣬ROM����������
--------------------------------------------------*/

/*--------------------------------------------------
*������		ds18b20_conversion
*���ܣ�		����ת������
*����ֵ��	��
----------------------------------------------------*/
void ds18b20_conversion()
{
	ds18b20_reset();
	ds18b20_check();
	write_byte(0xcc);//SKIP ROM,��ֻ��һ���豸����ѡ������������
	write_byte(0x44);//CONVERT TEMP
}

/*---------------------------------------------------
*������		ds18b20_readtemp_cmd
*���ܣ�		������ȡ�¶�ָ��
*����ֵ��	��
--------------------------------------------------*/
void ds18b20_readtemp_cmd()
{
	ds18b20_reset();
	ds18b20_check();
	write_byte(0xcc);//SKIP ROM,��ֻ��һ���豸����ѡ������������
	write_byte(0xbe);//READ SCRATCHPAD
}

/*-------------------------------------------------
*������		ds18b20_get_temp
*���ܣ�		��ȡ�¶���ֵ
*����ֵ��	tempcalue	:	�¶�ֵ
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
		TL=~TL;//�¶�Ϊ��ʱ����ֵ�任
		TH=-TH;
		temp=0;
	}
	TB=(TH<<8) | TL;

	tempvalue=TB*0.625;//Ĭ��12λ�ֱ��ʣ�0.0625���ȣ�����ʮ����������
	if(temp)
		return tempvalue;
	else
		return -tempvalue;
	
}

/*-------------------------------------------
*������		show_temp
*���ܣ�		��ʾ�¶�
*����ֵ��	��
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
