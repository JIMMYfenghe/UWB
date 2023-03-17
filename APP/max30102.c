#include "max30102.h"
#include "max_iic.h"
#include "delay.h"
#include "algorithm.h"
#include "stdio.h"

/*********************************************************
*������		max30102_gpio_init
*���ܣ�		��ʼ��MAX30102�ܽ�
*����ֵ��	��
**********************************************************/
void max30102_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //��������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
}	


/*********************************************************
*������		max30102_init
*���ܣ�		��ʼ��MAX30102����
*����ֵ��	��
**********************************************************/
void max30102_init(void)
{
	max30102_reset();
	/*************�¶�У׼*************/
	max30102_Bus_Write(REG_INTR_ENABLE_1,0xe0); //�������������ʹ��
	max30102_Bus_Write(REG_INTR_ENABLE_2, 0x02); //DIE_TEMP_RDY_EN
	max30102_Bus_Write(REG_TEMP_CONFIG, 0x01); //SET   TEMP_EN
	max30102_Bus_Write(REG_SPO2_CONFIG,0x47);  //SPO2_ADC range = 8192nA  100 per second    LED_PW[1:0]=11  18BITS
//	max30102_Bus_Write(REG_LED1_PA, 0x47); 		//(4*16+7)*0.2=14.2mA
//	max30102_Bus_Write(REG_LED2_PA, 0x47); 		//LED�������ѡ��

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
	max30102_Bus_Write(REG_PILOT_PA,0x7f);   	// Choose value for ~ 25mA for Pilot LED(�����ֲ�û��)
}

/*********************************************************
*������		max30102_reset
*���ܣ�		��λMAX30102
*����ֵ��	��
**********************************************************/
void max30102_reset(void)
{
	max30102_Bus_Write(REG_MODE_CONFIG,0x40);
}

/**********************************************************
*������		max30102_Bus_Write
*���ܣ�		iic������Ĵ���д����
*���룺		
			Register_Address���Ĵ�����ַ
			Word_Data		��д�������
*�����		��
*����ֵ��	
			0	��	�ɹ�
			1	:	ʧ��
***********************************************************/
u8 max30102_Bus_Write(u8 Register_Address, u8 Word_Data)
{

	/* ���ô���EEPROM�漴��ȡָ�����У�������ȡ�����ֽ� */

	/* ��1��������I2C���������ź� */
	MAX_IIC_Start();

	/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	MAX_IIC_Send_Byte(max30102_WR_address | I2C_WR);	/* �˴���дָ�� 0xae|0x00 */

	/* ��3��������ACK */
	if (MAX_IIC_Wait_Ack() != 0)//0--��Ӧ��1--��Ӧ��	
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}

	/* ��4���������ֽڵ�ַ */
	MAX_IIC_Send_Byte(Register_Address);
	if (MAX_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}
	
	/* ��5������ʼд������ */
	MAX_IIC_Send_Byte(Word_Data);

	/* ��6��������ACK */
	if (MAX_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}

	/* ����I2C����ֹͣ�ź� */
	MAX_IIC_Stop();
	return 1;	/* ִ�гɹ� */

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
	/* ����I2C����ֹͣ�ź� */
	MAX_IIC_Stop();
	return 0;
}

/****************************************************
*������		max30102_Bus_Read
*���ܣ�		iic������Ĵ�����ȡ����
*���룺		Register_Address���Ĵ�����ַ
*�����		��
*����ֵ��	
			data: �ɹ�����ȡ������
			0	��ʧ��
*****************************************************/
u8 max30102_Bus_Read(u8 Register_Address)
{
	u8  data;


	/* ��1��������I2C���������ź� */
	MAX_IIC_Start();

	/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	MAX_IIC_Send_Byte(max30102_WR_address | I2C_WR);	/* �˴���дָ�� */

	/* ��3��������ACK */
	if (MAX_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}

	/* ��4���������ֽڵ�ַ�� */
	MAX_IIC_Send_Byte((uint8_t)Register_Address);
	if (MAX_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}
	

	/* ��6������������I2C���ߡ����濪ʼ��ȡ���� */
	MAX_IIC_Start();

	/* ��7������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	MAX_IIC_Send_Byte(max30102_WR_address | I2C_RD);	/* �˴��Ƕ�ָ�� */

	/* ��8��������ACK */
	if (MAX_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}

	/* ��9������ȡ���� */
	{
		data = MAX_IIC_Read_Byte(0);	/* ��1���ֽ� */

		MAX_IIC_NAck();	/* ���1���ֽڶ����CPU����NACK�ź�(����SDA = 1) */
	}
	/* ����I2C����ֹͣ�ź� */
	MAX_IIC_Stop();
	return data;	/* ִ�гɹ� ����dataֵ */

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
	/* ����I2C����ֹͣ�ź� */
	MAX_IIC_Stop();
	return 0;
}

/**************************************************************************
*������		max30102_FIFO_ReadWords
*���ܣ�		max30102��fifo������
*���룺		
			Register_Address	��	�Ĵ�����ַ
			count				:	��ȡ���ֽ���
*�����		Word_Data[][2]		��	��ȡ������
*����ֵ��	��
***************************************************************************/
void max30102_FIFO_ReadWords(u8 Register_Address,u16 Word_Data[][2],u8 count)
{
	u8 i=0;
	u8 no = count;
	u8 data1, data2;
	/* ��1��������I2C���������ź� */
	MAX_IIC_Start();

	/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	MAX_IIC_Send_Byte(max30102_WR_address | I2C_WR);	/* �˴���дָ�� */

	/* ��3��������ACK */
	if (MAX_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}

	/* ��4���������ֽڵ�ַ�� */
	MAX_IIC_Send_Byte((uint8_t)Register_Address);
	if (MAX_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}
	

	/* ��6������������I2C���ߡ����濪ʼ��ȡ���� */
	MAX_IIC_Start();

	/* ��7������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	MAX_IIC_Send_Byte(max30102_WR_address | I2C_RD);	/* �˴��Ƕ�ָ�� */

	/* ��8��������ACK */
	if (MAX_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}

	/* ��9������ȡ���� */
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
			MAX_IIC_NAck();	/* ���1���ֽڶ����CPU����NACK�ź�(����SDA = 1) */
		else
			MAX_IIC_Ack();
		Word_Data[i][1] = (((u16)data1 << 8) | data2); 

		no--;	
		i++;
	}
	/* ����I2C����ֹͣ�ź� */
	MAX_IIC_Stop();

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
	/* ����I2C����ֹͣ�ź� */
	MAX_IIC_Stop();
}

/**************************************************************************
*������		max30102_FIFO_ReadBytes
*���ܣ�		max30102��fifo������ֽ�
*���룺		Register_Address	��	�Ĵ�����ַ
*�����		*Data				��	��ȡ������
*����ֵ��	��

***************************************************************************/
void max30102_FIFO_ReadBytes(u8 Register_Address,u8* Data)
{	
	max30102_Bus_Read(REG_INTR_STATUS_1);
	max30102_Bus_Read(REG_INTR_STATUS_2);
	
	/* ��1��������I2C���������ź� */
	MAX_IIC_Start();

	/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	MAX_IIC_Send_Byte(max30102_WR_address | I2C_WR);	/* �˴���дָ�� */

	/* ��3��������ACK */
	if (MAX_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}

	/* ��4���������ֽڵ�ַ�� */
	MAX_IIC_Send_Byte((uint8_t)Register_Address);
	if (MAX_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}
	

	/* ��6������������I2C���ߡ����濪ʼ��ȡ���� */
	MAX_IIC_Start();

	/* ��7������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	MAX_IIC_Send_Byte(max30102_WR_address | I2C_RD);	/* �˴��Ƕ�ָ�� */

	/* ��8��������ACK */
	if (MAX_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}

	/* ��9������ȡ���� */
	Data[0] = MAX_IIC_Read_Byte(1);	
	Data[1] = MAX_IIC_Read_Byte(1);	
	Data[2] = MAX_IIC_Read_Byte(1);	
	Data[3] = MAX_IIC_Read_Byte(1);
	Data[4] = MAX_IIC_Read_Byte(1);	
	Data[5] = MAX_IIC_Read_Byte(0);
	/* ���1���ֽڶ����CPU����NACK�ź�(����SDA = 1) */
	/* ����I2C����ֹͣ�ź� */
	MAX_IIC_Stop();

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
	/* ����I2C����ֹͣ�ź� */
	MAX_IIC_Stop();
}

/*---------------------------------------
*������		maxim_max30102_write_reg
*���ܣ�		��MAX�Ĵ���д�ֽ�
*���룺		
			uch_addr���Ĵ�����ַ
			uch_data������
*�����		��
*����ֵ��	��
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
*������		maxim_max30102_read_reg
*���ܣ�		��MAX���ֽ�
*���룺		uch_addr���Ĵ�����ַ
*�����		*puch_data������
*����ֵ��	��
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
*������		maxim_max30102_read_fifo
*���ܣ�		��MAX��FIFO
*���룺		��
*�����		
			*pun_red_led��
			*pun_ir_led	: 
*����ֵ��	��
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



#define PPG_DATA_MIN 10000 	//�����ֵ
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
*������		max30102_test
*���ܣ�		�ɼ�����
*����ֵ��	��
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
*������		max30102_continue
*���ܣ�		�ɼ����µ�����
*����ֵ��	��
------------------------------*/
void max30102_continue(void)
{
	//��ǰ100������ת���ڴ棬�������400�������Ƶ�����
	if(a==1)
		for(i=START;i<n_ir_buffer_length;i++)
		{
			aun_red_buffer[i-START]=aun_red_buffer[i];//��[100]~[499]��ֵ����[0]~[399]
			aun_ir_buffer[i-START]=aun_ir_buffer[i];
			a=2;
		}
	if(a==2)
	{
		for(i=n_ir_buffer_length-START;i<n_ir_buffer_length;i++)//[400]~[499]�������ֵ
		{
			if(MAX30102_INT==0)
			{
				max30102_FIFO_ReadBytes(REG_FIFO_DATA,temp);//�ɼ��µ���ֵ
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
*������		show_hr_spo2
*���ܣ�		��ʾ����
*����ֵ��	��
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
