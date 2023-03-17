#include "max_iic.h"
#include "delay.h"
extern void deca_sleep(unsigned int time_ms);
//初始化IIC
void MAX_IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	//RCC->APB2ENR|=1<<4;//先使能外设IO PORTC时钟 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	   
	GPIO_InitStructure.GPIO_Pin = MAX_SCL_PIN|MAX_SDA_PIN;	//SCL--PB6、SDA--PB7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;   //开漏输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MAX_IIC_PORT, &GPIO_InitStructure);
//	GPIO_PinAFConfig(IIC_PORT, GPIO_PinSource6, GPIO_AF_I2C1);
//	GPIO_PinAFConfig(IIC_PORT, GPIO_PinSource7, GPIO_AF_I2C1);
	MAX_IIC_SCL_H;//IIC_SCL=1;
	MAX_IIC_SDA_H;//IIC_SDA=1;

}

/*****************************************************
		IIC的起始条件和停止条件
******************************************************/
//产生IIC起始信号
void MAX_IIC_Start(void)
{
	MAX_SDA_OUT();     //sda线输出
	MAX_IIC_SDA_H;//IIC_SDA=1;	  	  
	MAX_IIC_SCL_H;//IIC_SCL=1;
	delay_us(4);
 	MAX_IIC_SDA_L;//IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	MAX_IIC_SCL_L;//IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
	delay_us(4);
}	  
//产生IIC停止信号
void MAX_IIC_Stop(void)
{
	MAX_SDA_OUT();//sda线输出
	MAX_IIC_SCL_L;//IIC_SCL=0;
	MAX_IIC_SDA_L;//IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	MAX_IIC_SCL_H;//IIC_SCL=1; 
	delay_us(4);
	MAX_IIC_SDA_H;//IIC_SDA=1;//发送I2C总线结束信号
	delay_us(4);						   	
}
/*****************************************************
		IIC应答信号的产生和不产生
******************************************************/
//产生ACK应答-------SDA低电平
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
//不产生ACK应答-----SDA高电平	    
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
		IIC等待应答信号到来
******************************************************/
//返回值：1，接收应答失败----READ_SDA=1，非应答
//        0，接收应答成功----READ_SDA=0，应答
u8 MAX_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	MAX_SDA_IN();      //SDA设置为输入  
	MAX_IIC_SDA_H;delay_us(2);//IIC_SDA=1;	   
	MAX_IIC_SCL_H;delay_us(2);//IIC_SCL=1; 
	while(MAX_READ_SDA)		//读SDA电平
	{
		ucErrTime++;
		if(ucErrTime>250)//不太理解此操作
		{
			MAX_IIC_Stop();
			return 1;
		}
	}
	MAX_IIC_SCL_L;//;IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
/*****************************************************
	IIC字节发送读取操作（数据手册上没找到对应时序图）
******************************************************/
//IIC发送一个字节，返回从机有无应答//1--有应答，0--无应答			  
void MAX_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	MAX_SDA_OUT(); 	    
    MAX_IIC_SCL_L;//IIC_SCL=0;//拉低时钟开始数据传输
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
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 MAX_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MAX_SDA_IN();//SDA设置为输入
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
        MAX_IIC_NAck();//发送nACK
    else
        MAX_IIC_Ack(); //发送ACK   
    return receive;
}
/*****************************************************
		IIC单个字节读写操作
******************************************************/
void MAX_IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data)
{				   	  	    																 
    MAX_IIC_Start();  
	//发送写命令daddr=max30102_WR_address(0xae)
	MAX_IIC_Send_Byte(daddr);
	MAX_IIC_Wait_Ack();		//ACK
	MAX_IIC_Send_Byte(addr);//发送寄存器地址
	MAX_IIC_Wait_Ack();	   	 										  		   
	MAX_IIC_Send_Byte(data);//发送字节							   
	MAX_IIC_Wait_Ack();  		    	   
    MAX_IIC_Stop();//产生一个停止条件 
	deca_sleep(10);	 
}

void MAX_IIC_Read_One_Byte(u8 daddr,u8 addr,u8* data)
{				  	  	  																 
    MAX_IIC_Start();  
	//发送写命令daddr = max30102_WR_address(0xae)
	MAX_IIC_Send_Byte(daddr);
	MAX_IIC_Wait_Ack();			
	MAX_IIC_Send_Byte(addr);//发送寄存器地址
	MAX_IIC_Wait_Ack();		 
	MAX_IIC_Start();  	 	   
	//进入接收模式daddr|0X01 = 0xaf
	MAX_IIC_Send_Byte(daddr|0X01);			   
	MAX_IIC_Wait_Ack();	 
    *data = MAX_IIC_Read_Byte(0);		//发送nACK
    MAX_IIC_Stop();//产生一个停止条件	 
	deca_sleep(10);
}
/*************************************************************************
		IIC 多字节读写操作
修改：  在原有的IIC_WriteBytes(u8 WriteAddr,u8* data,u8 dataLength)函数
		仿造写一个字节操作添加了u8 deviceAddr
		在IIC_Start();后添加了  IIC_Send_Byte(deviceAddr);IIC_Wait_Ack();
**************************************************************************/
void MAX_IIC_ReadBytes(u8 deviceAddr, u8 writeAddr,u8* data,u8 dataLength)
{		
	u8 i;	
    MAX_IIC_Start();  
	//发送写命令  deviceAddr = 0xae
	MAX_IIC_Send_Byte(deviceAddr);	    //发送写命令
	MAX_IIC_Wait_Ack();
	MAX_IIC_Send_Byte(writeAddr);//寄存器地址
	MAX_IIC_Wait_Ack();
	//进入接收模式	deviceAddr|0X01 = 0xaf
	MAX_IIC_Send_Byte(deviceAddr|0X01);		   
	MAX_IIC_Wait_Ack();
	
	for(i=0;i<dataLength-1;i++)
	{
		data[i] = MAX_IIC_Read_Byte(1);//发送ACK//AM
	}		
	data[dataLength-1] = MAX_IIC_Read_Byte(0);	//发送nACK
    MAX_IIC_Stop();//产生一个停止条件 
	deca_sleep(10);	 
}

void MAX_IIC_WriteBytes(u8 deviceAddr, u8 WriteAddr,u8* data,u8 dataLength)
{		
	u8 i;	
    MAX_IIC_Start();  
	
	MAX_IIC_Send_Byte(deviceAddr);	 //发送写命令
	MAX_IIC_Wait_Ack();
	MAX_IIC_Send_Byte(WriteAddr);	 //寄存器地址  
	MAX_IIC_Wait_Ack();
	
	for(i=0;i<dataLength;i++)
	{
		MAX_IIC_Send_Byte(data[i]);
		MAX_IIC_Wait_Ack();
	}				    	   
    MAX_IIC_Stop();//产生一个停止条件 
	deca_sleep(10);	 
}
/***************************************************
			IIC -- SDA 的IO方向设置
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

