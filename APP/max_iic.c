#include "max_iic.h"
#include "delay.h"
extern void deca_sleep(unsigned int time_ms);
//��ʼ��IIC
void MAX_IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	//RCC->APB2ENR|=1<<4;//��ʹ������IO PORTCʱ�� 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	   
	GPIO_InitStructure.GPIO_Pin = MAX_SCL_PIN|MAX_SDA_PIN;	//SCL--PB6��SDA--PB7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;   //��©���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MAX_IIC_PORT, &GPIO_InitStructure);
//	GPIO_PinAFConfig(IIC_PORT, GPIO_PinSource6, GPIO_AF_I2C1);
//	GPIO_PinAFConfig(IIC_PORT, GPIO_PinSource7, GPIO_AF_I2C1);
	MAX_IIC_SCL_H;//IIC_SCL=1;
	MAX_IIC_SDA_H;//IIC_SDA=1;

}

/*****************************************************
		IIC����ʼ������ֹͣ����
******************************************************/
//����IIC��ʼ�ź�
void MAX_IIC_Start(void)
{
	MAX_SDA_OUT();     //sda�����
	MAX_IIC_SDA_H;//IIC_SDA=1;	  	  
	MAX_IIC_SCL_H;//IIC_SCL=1;
	delay_us(4);
 	MAX_IIC_SDA_L;//IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	MAX_IIC_SCL_L;//IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
	delay_us(4);
}	  
//����IICֹͣ�ź�
void MAX_IIC_Stop(void)
{
	MAX_SDA_OUT();//sda�����
	MAX_IIC_SCL_L;//IIC_SCL=0;
	MAX_IIC_SDA_L;//IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	MAX_IIC_SCL_H;//IIC_SCL=1; 
	delay_us(4);
	MAX_IIC_SDA_H;//IIC_SDA=1;//����I2C���߽����ź�
	delay_us(4);						   	
}
/*****************************************************
		IICӦ���źŵĲ����Ͳ�����
******************************************************/
//����ACKӦ��-------SDA�͵�ƽ
void MAX_IIC_Ack(void)
{
	MAX_IIC_SCL_L;//IIC_SCL=0;
	MAX_SDA_OUT();
	MAX_IIC_SDA_L;//IIC_SDA=0;
	delay_us(2);
	MAX_IIC_SCL_H;//IIC_SCL=1;
	delay_us(5);
	MAX_IIC_SCL_L;//IIC_SCL=0;
	delay_us(2);
}
//������ACKӦ��-----SDA�ߵ�ƽ	    
void MAX_IIC_NAck(void)
{
	MAX_IIC_SCL_L;//IIC_SCL=0;
	MAX_SDA_OUT();
	MAX_IIC_SDA_H;//IIC_SDA=1;
	delay_us(2);
	MAX_IIC_SCL_H;//IIC_SCL=1;
	delay_us(5);
	MAX_IIC_SCL_L;//IIC_SCL=0;
	delay_us(2);
}		

/*****************************************************
		IIC�ȴ�Ӧ���źŵ���
******************************************************/
//����ֵ��1������Ӧ��ʧ��----READ_SDA=1����Ӧ��
//        0������Ӧ��ɹ�----READ_SDA=0��Ӧ��
u8 MAX_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	MAX_SDA_IN();      //SDA����Ϊ����  
	MAX_IIC_SDA_H;delay_us(2);//IIC_SDA=1;	   
	MAX_IIC_SCL_H;delay_us(2);//IIC_SCL=1; 
	while(MAX_READ_SDA)		//��SDA��ƽ
	{
		ucErrTime++;
		if(ucErrTime>250)//��̫���˲���
		{
			MAX_IIC_Stop();
			return 1;
		}
	}
	MAX_IIC_SCL_L;//;IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
/*****************************************************
	IIC�ֽڷ��Ͷ�ȡ�����������ֲ���û�ҵ���Ӧʱ��ͼ��
******************************************************/
//IIC����һ���ֽڣ����شӻ�����Ӧ��//1--��Ӧ��0--��Ӧ��			  
void MAX_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	MAX_SDA_OUT(); 	    
    MAX_IIC_SCL_L;//IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        MAX_IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   
		MAX_IIC_SCL_H;//IIC_SCL=1; 
		delay_us(2);
		MAX_IIC_SCL_L;//IIC_SCL=0;
		delay_us(2);	
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 MAX_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MAX_SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        MAX_IIC_SCL_L;//IIC_SCL=0; 
		delay_us(2);
		MAX_IIC_SCL_H;//IIC_SCL=1;
        receive<<=1;
        if(MAX_READ_SDA)receive++;   
		delay_us(2); 
    }					 
    if (!ack)
        MAX_IIC_NAck();//����nACK
    else
        MAX_IIC_Ack(); //����ACK   
    return receive;
}
/*****************************************************
		IIC�����ֽڶ�д����
******************************************************/
void MAX_IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data)
{				   	  	    																 
    MAX_IIC_Start();  
	//����д����daddr=max30102_WR_address(0xae)
	MAX_IIC_Send_Byte(daddr);
	MAX_IIC_Wait_Ack();		//ACK
	MAX_IIC_Send_Byte(addr);//���ͼĴ�����ַ
	MAX_IIC_Wait_Ack();	   	 										  		   
	MAX_IIC_Send_Byte(data);//�����ֽ�							   
	MAX_IIC_Wait_Ack();  		    	   
    MAX_IIC_Stop();//����һ��ֹͣ���� 
	deca_sleep(10);	 
}

void MAX_IIC_Read_One_Byte(u8 daddr,u8 addr,u8* data)
{				  	  	  																 
    MAX_IIC_Start();  
	//����д����daddr = max30102_WR_address(0xae)
	MAX_IIC_Send_Byte(daddr);
	MAX_IIC_Wait_Ack();			
	MAX_IIC_Send_Byte(addr);//���ͼĴ�����ַ
	MAX_IIC_Wait_Ack();		 
	MAX_IIC_Start();  	 	   
	//�������ģʽdaddr|0X01 = 0xaf
	MAX_IIC_Send_Byte(daddr|0X01);			   
	MAX_IIC_Wait_Ack();	 
    *data = MAX_IIC_Read_Byte(0);		//����nACK
    MAX_IIC_Stop();//����һ��ֹͣ����	 
	deca_sleep(10);
}
/*************************************************************************
		IIC ���ֽڶ�д����
�޸ģ�  ��ԭ�е�IIC_WriteBytes(u8 WriteAddr,u8* data,u8 dataLength)����
		����дһ���ֽڲ��������u8 deviceAddr
		��IIC_Start();�������  IIC_Send_Byte(deviceAddr);IIC_Wait_Ack();
**************************************************************************/
void MAX_IIC_ReadBytes(u8 deviceAddr, u8 writeAddr,u8* data,u8 dataLength)
{		
	u8 i;	
    MAX_IIC_Start();  
	//����д����  deviceAddr = 0xae
	MAX_IIC_Send_Byte(deviceAddr);	    //����д����
	MAX_IIC_Wait_Ack();
	MAX_IIC_Send_Byte(writeAddr);//�Ĵ�����ַ
	MAX_IIC_Wait_Ack();
	//�������ģʽ	deviceAddr|0X01 = 0xaf
	MAX_IIC_Send_Byte(deviceAddr|0X01);		   
	MAX_IIC_Wait_Ack();
	
	for(i=0;i<dataLength-1;i++)
	{
		data[i] = MAX_IIC_Read_Byte(1);//����ACK//AM
	}		
	data[dataLength-1] = MAX_IIC_Read_Byte(0);	//����nACK
    MAX_IIC_Stop();//����һ��ֹͣ���� 
	deca_sleep(10);	 
}

void MAX_IIC_WriteBytes(u8 deviceAddr, u8 WriteAddr,u8* data,u8 dataLength)
{		
	u8 i;	
    MAX_IIC_Start();  
	
	MAX_IIC_Send_Byte(deviceAddr);	 //����д����
	MAX_IIC_Wait_Ack();
	MAX_IIC_Send_Byte(WriteAddr);	 //�Ĵ�����ַ  
	MAX_IIC_Wait_Ack();
	
	for(i=0;i<dataLength;i++)
	{
		MAX_IIC_Send_Byte(data[i]);
		MAX_IIC_Wait_Ack();
	}				    	   
    MAX_IIC_Stop();//����һ��ֹͣ���� 
	deca_sleep(10);	 
}
/***************************************************
			IIC -- SDA ��IO��������
***************************************************/
void MAX_SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = MAX_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_Init(MAX_IIC_PORT,&GPIO_InitStructure);
}
void MAX_SDA_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = MAX_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_Init(MAX_IIC_PORT,&GPIO_InitStructure);
}

