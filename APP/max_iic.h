#ifndef __MAX30102_H
#define __MAX30102_H
#include "sys.h" 

#define I2C_WR	0		/* д����bit */
#define I2C_RD	1		/* ������bit */

#define MAX_SCL_PIN 	GPIO_Pin_6
#define MAX_SDA_PIN 	GPIO_Pin_7
#define MAX_IIC_PORT 	GPIOB
//IO��������	 
#define MAX_IIC_SCL    	PBout(6) //SCL
#define MAX_IIC_SDA   	PBout(7) //SDA	 
#define MAX_READ_SDA   	PBin(7)  //����SDA 

#define MAX_IIC_SCL_L	GPIO_ResetBits(MAX_IIC_PORT,MAX_SCL_PIN);
#define MAX_IIC_SCL_H	GPIO_SetBits(MAX_IIC_PORT,MAX_SCL_PIN);
#define MAX_IIC_SDA_L	GPIO_ResetBits(MAX_IIC_PORT,MAX_SDA_PIN);
#define MAX_IIC_SDA_H	GPIO_SetBits(MAX_IIC_PORT,MAX_SDA_PIN);


//IIC���в�������
void MAX_IIC_Init(void);                //��ʼ��IIC��IO��

void MAX_IIC_Start(void);				//����IIC��ʼ�ź�
void MAX_IIC_Stop(void);	  			//����IICֹͣ�ź�

void MAX_IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 MAX_IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�

u8 MAX_IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void MAX_IIC_Ack(void);					//IIC����ACK�ź�
void MAX_IIC_NAck(void);				//IIC������ACK�ź�

void MAX_IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
void MAX_IIC_Read_One_Byte(u8 daddr,u8 addr,u8* data);

void MAX_IIC_WriteBytes(u8 deviceAddr, u8 WriteAddr,u8* data,u8 dataLength);
void MAX_IIC_ReadBytes(u8 deviceAddr, u8 writeAddr,u8* data,u8 dataLength);


//IO�������� 
void MAX_SDA_IN(void);
void MAX_SDA_OUT(void);

#endif
















