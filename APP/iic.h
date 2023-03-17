#ifndef _ADXL345_H
#define _ADXL345_H
#include "sys.h"

#define ADXL_SCL_PIN GPIO_Pin_8
#define ADXL_SDA_PIN GPIO_Pin_9
#define ADXL_IIC_PORT GPIOB

#define ADXL_IIC_SCL_L	GPIO_ResetBits(ADXL_IIC_PORT,ADXL_SCL_PIN);
#define ADXL_IIC_SCL_H	GPIO_SetBits(ADXL_IIC_PORT,ADXL_SCL_PIN);
#define ADXL_IIC_SDA_L	GPIO_ResetBits(ADXL_IIC_PORT,ADXL_SDA_PIN);
#define ADXL_IIC_SDA_H	GPIO_SetBits(ADXL_IIC_PORT,ADXL_SDA_PIN);

#define ADXL_IIC_SCL    PBout(8) //SCL
#define ADXL_IIC_SDA    PBout(9) //SDA	 
#define ADXL_READ_SDA  PBin(9)  //输入SDA 

//IO方向设置 
void ADXL_SDA_IN(void);
void ADXL_SDA_OUT(void);

void ADXL_IIC_Init(void);
void ADXL_IIC_Start(void);
void ADXL_IIC_Stop(void);
void ADXL_IIC_SendACK(void);
void ADXL_IIC_SendNACK(void);
uint8_t ADXL_IIC_Wait_Ack(void);
void ADXL_IIC_Send_Byte(uint8_t txd);
uint8_t ADXL_IIC_Read_Byte(unsigned char ack);
#endif
