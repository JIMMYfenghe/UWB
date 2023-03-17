#include "iic.h"
#include "delay.h"
#include "sys.h"

/*********************************************************
*��������void ADXL_IIC_Init(void)
*���ܣ���ʼ��ADXL345   IIC����
*********************************************************/
void ADXL_IIC_Init(void)
{				
//	I2C_InitTypeDef  I2C_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	   
	GPIO_InitStructure.GPIO_Pin = ADXL_SCL_PIN|ADXL_SDA_PIN;	//PB8��PB9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;   //�������
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_Init(ADXL_IIC_PORT, &GPIO_InitStructure);
	// ADXL_IIC_SCL_H;
	// ADXL_IIC_SDA_H;
//	GPIO_PinAFConfig(IIC_PORT, GPIO_PinSource8, GPIO_AF_I2C1);
//	GPIO_PinAFConfig(IIC_PORT, GPIO_PinSource9, GPIO_AF_I2C1);

}

/*********************************************************
���ܣ�����SDA��������
*********************************************************/
void ADXL_SDA_IN(void)
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ADXL_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_Init(ADXL_IIC_PORT,&GPIO_InitStructure);
}

/*********************************************************
���ܣ�����SDA�������
*********************************************************/
void ADXL_SDA_OUT(void)
{					     
   GPIO_InitTypeDef GPIO_InitStructure;
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ADXL_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
	GPIO_Init(ADXL_IIC_PORT,&GPIO_InitStructure);
}


/*********************************************************
���ܣ�����IIC��ʼ�ź�
*********************************************************/
void ADXL_IIC_Start(void)
{
    ADXL_SDA_OUT();         
	ADXL_IIC_SDA_H;         
    ADXL_IIC_SCL_H;     
    delay_us(5);      
    ADXL_IIC_SDA_L;        
    delay_us(5);       
    ADXL_IIC_SCL_L;    
	delay_us(5);
}
	
/*********************************************************
���ܣ�����IICֹͣ�ź�
*********************************************************/
void ADXL_IIC_Stop(void)
{
    ADXL_SDA_OUT();      
    ADXL_IIC_SCL_L;      
    ADXL_IIC_SDA_L;     
    delay_us(5);
	ADXL_IIC_SCL_H;  
	delay_us(5);
    ADXL_IIC_SDA_H;       
    delay_us(5);   
}

/*********************************************************
���ܣ�����ACKӦ��
*********************************************************/
void ADXL_IIC_SendACK(void) 
{
	ADXL_IIC_SCL_L;     
	ADXL_SDA_OUT();     
    ADXL_IIC_SDA_L;      

    delay_us(5);
    ADXL_IIC_SCL_H;;     
	delay_us(5);  
    ADXL_IIC_SCL_L;      
    delay_us(5); 
}
/*********************************************************
���ܣ�������ACKӦ��
*****************************************************/
void ADXL_IIC_SendNACK(void) 
{
	ADXL_IIC_SCL_L;      
	ADXL_SDA_OUT();      
	ADXL_IIC_SDA_H;       

	delay_us(5);  
	ADXL_IIC_SCL_H;;     
	delay_us(5);
	ADXL_IIC_SCL_L;       
    delay_us(5); 
}
/*********************************************************
���ܣ��ȴ�Ӧ���źŵ���
����ֵ��1������Ӧ��ʧ��
               0������Ӧ��ɹ� 
*********************************************************/
uint8_t ADXL_IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	
	ADXL_SDA_IN();       
	
  ADXL_IIC_SDA_H;   
	delay_us(5);    
	
	ADXL_IIC_SCL_H;      
  delay_us(5);     

	while(ADXL_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			ADXL_IIC_Stop();
			return 1;
		}
	}
	
	ADXL_IIC_SCL_L;   
   return 0;
}

/*********************************************************
���ܣ�IIC����һ���ֽ�
����ֵ��1����Ӧ��
               0����Ӧ��
*********************************************************/	 
void ADXL_IIC_Send_Byte(uint8_t txd) 
{
		uint8_t t;   
		uint8_t tmp=0;
	    ADXL_SDA_OUT(); 	 
		ADXL_IIC_SCL_L;  
		  for (t=0; t<8; t++)      
		  {
			  tmp = (txd&0x80)>>7;     
		      if(tmp==1)
			  {ADXL_IIC_SDA_H;  
				delay_us(5);
			  }
				else
				{ADXL_IIC_SDA_L;	 
						delay_us(5);}
				txd<<=1; 
				delay_us(5);
		        ADXL_IIC_SCL_H;     
		        delay_us(5);     
		        ADXL_IIC_SCL_L;     
		        delay_us(5);
		 }
}

/*********************************************************
���ܣ���һ���ֽ�
����ֵ����ȡ������
*********************************************************/	 	

uint8_t ADXL_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
    ADXL_SDA_IN();  
    for (i=0; i<8; i++)      
    {
	    ADXL_IIC_SCL_L;        
	    delay_us(5);      
 	    ADXL_IIC_SCL_H;          
	    receive <<= 1;	
      if(ADXL_READ_SDA)
		  receive++;
      delay_us(5); 		
    }
	 if(!ack)
	    ADXL_IIC_SendNACK();
	 else
		ADXL_IIC_SendACK();
    return receive;	
}

