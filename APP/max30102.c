#include "max30102.h"
#include "max_iic.h"
#include "delay.h"
#include "algorithm.h"
#include "stdio.h"

/*********************************************************
*函数：		max30102_gpio_init
*功能：		初始化MAX30102管脚
*返回值：	无
**********************************************************/
void max30102_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //上拉输入
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
}	


/*********************************************************
*函数：		max30102_init
*功能：		初始化MAX30102配置
*返回值：	无
**********************************************************/
void max30102_init(void)
{
	max30102_reset();
	/*************温度校准*************/
	max30102_Bus_Write(REG_INTR_ENABLE_1,0xe0); //环境光消除溢出使能
	max30102_Bus_Write(REG_INTR_ENABLE_2, 0x02); //DIE_TEMP_RDY_EN
	max30102_Bus_Write(REG_TEMP_CONFIG, 0x01); //SET   TEMP_EN
	max30102_Bus_Write(REG_SPO2_CONFIG,0x47);  //SPO2_ADC range = 8192nA  100 per second    LED_PW[1:0]=11  18BITS
//	max30102_Bus_Write(REG_LED1_PA, 0x47); 		//(4*16+7)*0.2=14.2mA
//	max30102_Bus_Write(REG_LED2_PA, 0x47); 		//LED电流如何选定

//	max30102_Bus_Write(REG_INTR_ENABLE_1,0xc0);	// INTR setting
//	max30102_Bus_Write(REG_INTR_ENABLE_2,0x00);
	max30102_Bus_Write(REG_FIFO_WR_PTR,0x00);  	//FIFO_WR_PTR[4:0]
	max30102_Bus_Write(REG_OVF_COUNTER,0x00);  	//OVF_COUNTER[4:0]
	max30102_Bus_Write(REG_FIFO_RD_PTR,0x00);  	//FIFO_RD_PTR[4:0]
	max30102_Bus_Write(REG_FIFO_CONFIG,0x0f);  	//sample avg = 1, fifo rollover=false, fifo almost full = 17
	max30102_Bus_Write(REG_MODE_CONFIG,0x03);  	//0x02 for Red only, 0x03 for SpO2 mode, 0x07 multimode LED
//	max30102_Bus_Write(REG_SPO2_CONFIG,0x2a);  	// SPO2_ADC range = 4096nA, SPO2 sample(100per second), LED pulseWidth (411uS) 
	max30102_Bus_Write(REG_LED1_PA,0x2f);   	//Choose value for 7.2mA for LED1
	max30102_Bus_Write(REG_LED2_PA,0x2f);   	// Choose value for (2*16+4)*0.2=7.2mA for LED2
	max30102_Bus_Write(REG_PILOT_PA,0x7f);   	// Choose value for ~ 25mA for Pilot LED(数据手册没有)
}

/*********************************************************
*函数：		max30102_reset
*功能：		复位MAX30102
*返回值：	无
**********************************************************/
void max30102_reset(void)
{
	max30102_Bus_Write(REG_MODE_CONFIG,0x40);
}

/**********************************************************
*函数：		max30102_Bus_Write
*功能：		iic总线向寄存器写数据
*输入：		
			Register_Address：寄存器地址
			Word_Data		：写入的数据
*输出：		无
*返回值：	
			0	：	成功
			1	:	失败
***********************************************************/
u8 max30102_Bus_Write(u8 Register_Address, u8 Word_Data)
{

	/* 采用串行EEPROM随即读取指令序列，连续读取若干字节 */

	/* 第1步：发起I2C总线启动信号 */
	MAX_IIC_Start();

	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	MAX_IIC_Send_Byte(max30102_WR_address | I2C_WR);	/* 此处是写指令 0xae|0x00 */

	/* 第3步：发送ACK */
	if (MAX_IIC_Wait_Ack() != 0)//0--有应答，1--无应答	
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第4步：发送字节地址 */
	MAX_IIC_Send_Byte(Register_Address);
	if (MAX_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}
	
	/* 第5步：开始写入数据 */
	MAX_IIC_Send_Byte(Word_Data);

	/* 第6步：发送ACK */
	if (MAX_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 发送I2C总线停止信号 */
	MAX_IIC_Stop();
	return 1;	/* 执行成功 */

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	MAX_IIC_Stop();
	return 0;
}

/****************************************************
*函数：		max30102_Bus_Read
*功能：		iic总线向寄存器读取数据
*输入：		Register_Address：寄存器地址
*输出：		无
*返回值：	
			data: 成功，读取的数据
			0	：失败
*****************************************************/
u8 max30102_Bus_Read(u8 Register_Address)
{
	u8  data;


	/* 第1步：发起I2C总线启动信号 */
	MAX_IIC_Start();

	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	MAX_IIC_Send_Byte(max30102_WR_address | I2C_WR);	/* 此处是写指令 */

	/* 第3步：发送ACK */
	if (MAX_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第4步：发送字节地址， */
	MAX_IIC_Send_Byte((uint8_t)Register_Address);
	if (MAX_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}
	

	/* 第6步：重新启动I2C总线。下面开始读取数据 */
	MAX_IIC_Start();

	/* 第7步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	MAX_IIC_Send_Byte(max30102_WR_address | I2C_RD);	/* 此处是读指令 */

	/* 第8步：发送ACK */
	if (MAX_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第9步：读取数据 */
	{
		data = MAX_IIC_Read_Byte(0);	/* 读1个字节 */

		MAX_IIC_NAck();	/* 最后1个字节读完后，CPU产生NACK信号(驱动SDA = 1) */
	}
	/* 发送I2C总线停止信号 */
	MAX_IIC_Stop();
	return data;	/* 执行成功 返回data值 */

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	MAX_IIC_Stop();
	return 0;
}

/**************************************************************************
*函数：		max30102_FIFO_ReadWords
*功能：		max30102的fifo读数据
*输入：		
			Register_Address	：	寄存器地址
			count				:	读取的字节数
*输出：		Word_Data[][2]		：	读取的数据
*返回值：	无
***************************************************************************/
void max30102_FIFO_ReadWords(u8 Register_Address,u16 Word_Data[][2],u8 count)
{
	u8 i=0;
	u8 no = count;
	u8 data1, data2;
	/* 第1步：发起I2C总线启动信号 */
	MAX_IIC_Start();

	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	MAX_IIC_Send_Byte(max30102_WR_address | I2C_WR);	/* 此处是写指令 */

	/* 第3步：发送ACK */
	if (MAX_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第4步：发送字节地址， */
	MAX_IIC_Send_Byte((uint8_t)Register_Address);
	if (MAX_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}
	

	/* 第6步：重新启动I2C总线。下面开始读取数据 */
	MAX_IIC_Start();

	/* 第7步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	MAX_IIC_Send_Byte(max30102_WR_address | I2C_RD);	/* 此处是读指令 */

	/* 第8步：发送ACK */
	if (MAX_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第9步：读取数据 */
	while (no)
	{
		data1 = MAX_IIC_Read_Byte(0);	
		MAX_IIC_Ack();
		data2 = MAX_IIC_Read_Byte(0);
		MAX_IIC_Ack();
		Word_Data[i][0] = (((u16)data1 << 8) | data2);  //

		
		data1 = MAX_IIC_Read_Byte(0);	
		MAX_IIC_Ack();
		data2 = MAX_IIC_Read_Byte(0);
		if(1==no)
			MAX_IIC_NAck();	/* 最后1个字节读完后，CPU产生NACK信号(驱动SDA = 1) */
		else
			MAX_IIC_Ack();
		Word_Data[i][1] = (((u16)data1 << 8) | data2); 

		no--;	
		i++;
	}
	/* 发送I2C总线停止信号 */
	MAX_IIC_Stop();

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	MAX_IIC_Stop();
}

/**************************************************************************
*函数：		max30102_FIFO_ReadBytes
*功能：		max30102的fifo读多个字节
*输入：		Register_Address	：	寄存器地址
*输出：		*Data				：	读取的数据
*返回值：	无

***************************************************************************/
void max30102_FIFO_ReadBytes(u8 Register_Address,u8* Data)
{	
	max30102_Bus_Read(REG_INTR_STATUS_1);
	max30102_Bus_Read(REG_INTR_STATUS_2);
	
	/* 第1步：发起I2C总线启动信号 */
	MAX_IIC_Start();

	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	MAX_IIC_Send_Byte(max30102_WR_address | I2C_WR);	/* 此处是写指令 */

	/* 第3步：发送ACK */
	if (MAX_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第4步：发送字节地址， */
	MAX_IIC_Send_Byte((uint8_t)Register_Address);
	if (MAX_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}
	

	/* 第6步：重新启动I2C总线。下面开始读取数据 */
	MAX_IIC_Start();

	/* 第7步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	MAX_IIC_Send_Byte(max30102_WR_address | I2C_RD);	/* 此处是读指令 */

	/* 第8步：发送ACK */
	if (MAX_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第9步：读取数据 */
	Data[0] = MAX_IIC_Read_Byte(1);	
	Data[1] = MAX_IIC_Read_Byte(1);	
	Data[2] = MAX_IIC_Read_Byte(1);	
	Data[3] = MAX_IIC_Read_Byte(1);
	Data[4] = MAX_IIC_Read_Byte(1);	
	Data[5] = MAX_IIC_Read_Byte(0);
	/* 最后1个字节读完后，CPU产生NACK信号(驱动SDA = 1) */
	/* 发送I2C总线停止信号 */
	MAX_IIC_Stop();

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	MAX_IIC_Stop();
}

/*---------------------------------------
*函数：		maxim_max30102_write_reg
*功能：		向MAX寄存器写字节
*输入：		
			uch_addr：寄存器地址
			uch_data：数据
*输出：		无
*返回值：	无
---------------------------------------*/
void maxim_max30102_write_reg(uint8_t uch_addr, uint8_t uch_data)
{
//  char ach_i2c_data[2];
//  ach_i2c_data[0]=uch_addr;
//  ach_i2c_data[1]=uch_data;
//	
//  IIC_WriteBytes(I2C_WRITE_ADDR, ach_i2c_data, 2);
	MAX_IIC_Write_One_Byte(I2C_WRITE_ADDR,uch_addr,uch_data);
}

/*----------------------------------
*函数：		maxim_max30102_read_reg
*功能：		向MAX读字节
*输入：		uch_addr：寄存器地址
*输出：		*puch_data：数据
*返回值：	无
----------------------------------*/
void maxim_max30102_read_reg(uint8_t uch_addr, uint8_t *puch_data)
{
//  char ch_i2c_data;
//  ch_i2c_data=uch_addr;
//  IIC_WriteBytes(I2C_WRITE_ADDR, &ch_i2c_data, 1);
//	
//  i2c.read(I2C_READ_ADDR, &ch_i2c_data, 1);
//  
//   *puch_data=(uint8_t) ch_i2c_data;
	MAX_IIC_Read_One_Byte(I2C_WRITE_ADDR,uch_addr,puch_data);
}

/*-----------------------------
*函数：		maxim_max30102_read_fifo
*功能：		向MAX读FIFO
*输入：		无
*输出：		
			*pun_red_led：
			*pun_ir_led	: 
*返回值：	无
-----------------------------*/
void maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
{
	uint32_t un_temp;
	unsigned char uch_temp;
	char ach_i2c_data[6];
	*pun_red_led=0;
	*pun_ir_led=0;

  //read and clear status register
  maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_temp);
  maxim_max30102_read_reg(REG_INTR_STATUS_2, &uch_temp);
  
  MAX_IIC_ReadBytes(I2C_WRITE_ADDR,REG_FIFO_DATA,(u8 *)ach_i2c_data,6);
  
  un_temp=(unsigned char) ach_i2c_data[0];
  un_temp<<=16;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[1];
  un_temp<<=8;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[2];
  *pun_red_led+=un_temp;
  
  un_temp=(unsigned char) ach_i2c_data[3];
  un_temp<<=16;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[4];
  un_temp<<=8;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[5];
  *pun_ir_led+=un_temp;
  *pun_red_led&=0x03FFFF;  //Mask MSB [23:18]
  *pun_ir_led&=0x03FFFF;  //Mask MSB [23:18]
}



#define PPG_DATA_MIN 10000 	//检测阈值
#define PPG_DATA_MAX 200000
#define START 			100
#define DATA_LENGTH 	500

uint32_t max30102_data[2];
uint32_t aun_ir_buffer[DATA_LENGTH]; //IR LED sensor data
int32_t n_ir_buffer_length;    //data length
uint32_t aun_red_buffer[DATA_LENGTH];    //Red LED sensor data
int32_t n_sp02; 		//SPO2 value
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;   //heart rate value
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;
  
int i,a=0;
float f_temp;
u8 temp[6];
char data1[20],data2[20];
int32_t j=0;

/*-----------------------------------
*函数：		max30102_test
*功能：		采集样本
*返回值：	无
-----------------------------------*/
void max30102_test(void)
{  
	 n_ir_buffer_length=DATA_LENGTH;//buffer length of 100 stores 5 seconds of samples running at 100sps
	if(a==0){
	//read the first 500 samples, and determine the signal range
		for(i=0;i<n_ir_buffer_length;i++)
		{
			if(MAX30102_INT==0)//wait until the interrupt pin asserts
			{
				max30102_FIFO_ReadBytes(REG_FIFO_DATA,temp);
				max30102_data[0] =  (long)((long)((long)temp[0]&0x03)<<16) | (long)temp[1]<<8 | (long)temp[2];    // Combine values to get the actual number
				max30102_data[1] = (long)((long)((long)temp[3] & 0x03)<<16) |(long)temp[4]<<8 | (long)temp[5];   // Combine values to get the actual number
				if(max30102_data[0]>PPG_DATA_MIN && max30102_data[1]>PPG_DATA_MIN && max30102_data[0]<PPG_DATA_MAX && max30102_data[1]<PPG_DATA_MAX)
				{
					aun_ir_buffer[j] = max30102_data[0];
					aun_red_buffer[j] = max30102_data[1];
//					printf("%i\n",aun_ir_buffer[j]);
					j++;
				}
			}
			if(j==n_ir_buffer_length)
			{
		//calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
				a=1;
				maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
				j=n_ir_buffer_length-START;
			}
		}
	}
}

/*-------------------------------
*函数：		max30102_continue
*功能：		采集更新的样本
*返回值：	无
------------------------------*/
void max30102_continue(void)
{
	//将前100组样本转入内存，并将最后400组样本移到顶部
	if(a==1)
		for(i=START;i<n_ir_buffer_length;i++)
		{
			aun_red_buffer[i-START]=aun_red_buffer[i];//将[100]~[499]的值移入[0]~[399]
			aun_ir_buffer[i-START]=aun_ir_buffer[i];
			a=2;
		}
	if(a==2)
	{
		for(i=n_ir_buffer_length-START;i<n_ir_buffer_length;i++)//[400]~[499]存放新数值
		{
			if(MAX30102_INT==0)
			{
				max30102_FIFO_ReadBytes(REG_FIFO_DATA,temp);//采集新的数值
				max30102_data[0] =  (long)((long)((long)temp[0]&0x03)<<16) | (long)temp[1]<<8 | (long)temp[2];    // Combine values to get the actual number
				max30102_data[1] = (long)((long)((long)temp[3] & 0x03)<<16) |(long)temp[4]<<8 | (long)temp[5];   // Combine values to get the actual number 
				if(max30102_data[0]>PPG_DATA_MIN && max30102_data[1]>PPG_DATA_MIN && max30102_data[0]<PPG_DATA_MAX && max30102_data[1]<PPG_DATA_MAX)
				{
					aun_ir_buffer[j] = max30102_data[0];
					aun_red_buffer[j] = max30102_data[1];
//					printf("%i\n",aun_ir_buffer[j]);
					j++;
				}
				
			}
			if(j==n_ir_buffer_length)
			{
				a=1;
				maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);  
			#ifdef DEBUG_TEST
				printf(" HR=%i,", n_heart_rate); 
				printf(" HRvalid=%i,", ch_hr_valid);
				printf(" SpO2=%i,", n_sp02);
				printf(" SPO2Valid=%i\r\n", ch_spo2_valid);
			#endif
				j=n_ir_buffer_length-START;
			
			}
		}  
	}
}


#include "oled.h"
/*------------------------------
*函数：		show_hr_spo2
*功能：		显示数据
*返回值：	无
------------------------------*/
void show_hr_spo2()
{
		if(ch_hr_valid == 1 && n_heart_rate>30  &&n_heart_rate < 170)
		{
			sprintf(data1,  "%3d", n_heart_rate);
			OLED_ShowString(80,0,data1,12);
		}
		else
			OLED_ShowString(80,0,"???",12);
			
		if(ch_spo2_valid == 1 && n_sp02 >60 && n_sp02 <=100)
		{
			sprintf(data2,  "%3d\"%\"", n_sp02);
			OLED_ShowString(80,2,data2,12);
		}
		else
			OLED_ShowString(80,2,"???",12);	
}
