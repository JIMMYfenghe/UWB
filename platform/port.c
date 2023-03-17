/*! ----------------------------------------------------------------------------
 * @file    port.c
 * @brief   HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2016 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

//#include <sys/types.h>
#include "deca_spi.h"
#include "port.h"
//#include <stdarg.h>
#include <stdio.h>
#include "dw1000.h"
#include "adxl345.h"
#include "iic.h"
#include "buzzer.h"
#include "max30102.h"
#include "max_iic.h"




//#include "sleep.h"
extern void deca_sleep(unsigned int time_ms);

//#include "deca_device_api.h"
//#include "stm32f4xx_hal_conf.h"
//#include "main.h"

/****************************************************************************//**
 *
 *                              APP global variables
 *
 *******************************************************************************/
//extern SPI_HandleTypeDef hspi1;

/****************************************************************************//**
 *
 *                  Port private variables and function prototypes
 *
 *******************************************************************************/
static volatile uint32_t signalResetDone;

/****************************************************************************//**
 *
 *                              Time section
 *
 *******************************************************************************/

/* @fn    portGetTickCnt
 * @brief wrapper for to read a SysTickTimer, which is incremented with
 *        CLOCKS_PER_SEC frequency.
 *        The resolution of time32_incr is usually 1/1000 sec.
 * */
/* System tick 32 bit variable defined by the platform */
extern __IO unsigned long time32_incr;
unsigned long portGetTickCnt(void)
{
    return time32_incr;
    // return 0;
}



/*�ص�����*/
void (*rx_callback)(void);
/*------------------------*/
//������	INT_callback
//���ܣ�	��ʼ���ص�����
//������	��Ҫִ�л�ע��Ļص�����
//����ֵ��	��
/*------------------------*/
void INT_callback(void (*callback)(void))
{
	rx_callback=callback;
}

/*--------------------------*/
//������	SysTick_Configuration
//���ܣ�	���õδ�ʱ�����ĺ��ж����ȼ�
//������	��
//����ֵ��	int��
/*---------------------------*/
#define CLOCKS_PER_SEC (1000)
int SysTick_Configuration(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	//SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	if (SysTick_Config(SystemCoreClock / CLOCKS_PER_SEC))
	{
		/* Capture error */
		while (1);
	}
	NVIC_SetPriority (SysTick_IRQn, 5);



	return 0;
}
/****************************************************************************//**
 *
 *                              END OF Time section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 *                              Configuration section
 *
 *******************************************************************************/
/*----------------------*/
//������	NVIC_Configuration
//���ܣ�	�����жϷ���
//������	��
//����ֵ��	��
/*-----------------------*/
void NVIC_Configuration()
{

	/* Set NVIC Grouping to 16 groups of interrupt without sub-grouping */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);



}

/*--------------------------------*/
//������	DW1000_EXTI_Configure
//���ܣ�	����DW1000��MCU�ϵ��ⲿ�ж���
//������	��
//����ֵ��	��
/*--------------------------------*/
void DW1000_EXTI_Configure()
{
		//NVIC_InitTypeDef NVIC_InitStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
		EXTI_InitTypeDef EXTI_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		/*Configure GPIO pin : PB0 */
		GPIO_InitStructure.GPIO_Pin = DECAIRQ;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd= GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &GPIO_InitStructure);

		/* Connect EXTI Line to GPIO Pin */
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0);

		/* Configure EXTI line */
		EXTI_InitStructure.EXTI_Line = DECAIRQ_EXTI;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;	//MPW3 IRQ polarity is high by default
		EXTI_InitStructure.EXTI_LineCmd = DECAIRQ_EXTI_USEIRQ;
		EXTI_Init(&EXTI_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = DECAIRQ_EXTI_IRQn;//�ⲿ�ж�0
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;//��ռ���ȼ�6
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
		NVIC_Init(&NVIC_InitStructure);//����	
}
/*----------------------------------*/
//������	RCC_Configuration
//���ܣ�	ϵͳʱ�ӳ�ʼ��������ѡ��
//������	��
//����ֵ��	��
/*----------------------------------*/
int RCC_Configuration(void)
{
	ErrorStatus HSEStartUpStatus;
	RCC_ClocksTypeDef RCC_ClockFreq;
	u32 p1,p2,hclk,sysclk;
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if(HSEStartUpStatus != ERROR)
	{
		SystemInit();
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(ENABLE);

		/****************************************************************/
		/* HSE= up to 25MHz (on EVB1000 is 12MHz),
		 * HCLK=84MHz, PCLK2=84MHz, PCLK1=42MHz 						*/
		/****************************************************************/
		/* Flash 2 wait state */
		FLASH_SetLatency(FLASH_Latency_2);
		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		/* PCLK2 = HCLK */
		RCC_PCLK2Config(RCC_HCLK_Div1);
		/* PCLK1 = HCLK/2 */
		RCC_PCLK1Config(RCC_HCLK_Div2);
	}

	RCC_GetClocksFreq(&RCC_ClockFreq);
	/**0501BD00----84000000*/
	p2=RCC_ClockFreq.PCLK2_Frequency;
	/**0280DE80----42000000*/
	p1=RCC_ClockFreq.PCLK1_Frequency;

	sysclk=RCC_ClockFreq.SYSCLK_Frequency;
	hclk=RCC_ClockFreq.HCLK_Frequency;
	/* Enable SPI1 clock */
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* Enable SPI1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	/*ENABLE	USART1 CLOCK*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 , ENABLE);

	/* Enable GPIOs clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB,ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
	


	return 0;
}

/*--------------------------*/
//������	DW1000_gpio_init
//���ܣ�	DW1000��GPIO����
//������	��
//����ֵ��	��
/*--------------------------*/
void DW1000_gpio_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//IO����SPI1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);
	//����ΪUSART1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	/*SPI1*/
	GPIO_InitStructure.GPIO_Pin= SPIx_CS;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(SPIx_CS_GPIO,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin= SPIx_SCK | SPIx_MOSI |SPIx_MISO;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(SPIx_GPIO,&GPIO_InitStructure);
	
	GPIO_WriteBit(SPIx_GPIO,SPIx_CS,Bit_SET);

	
	
	/* USART1 GPIO config */
   /* Configure USART1 Tx (PA.09) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//--�������ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    
  	
  /* Configure USART1 Rx (PA.10) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//test enterint//
			/*Configure GPIO pin : PB1 */
			/*
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_WriteBit(GPIOB,GPIO_Pin_1,Bit_RESET);
	*/
  
	
}


/* @fn    peripherals_init
 * */
int peripherals_init (void)
{
	RCC_Configuration();
	NVIC_Configuration();
	SysTick_Configuration();
	
#ifdef TX_NODE
	/*------����ģ��*/
	  ADXL_IIC_Init();  		//��ʼ��IIC����  Ӱ��UWB
	  ADXL345_INT();			//��ʼ���ж�����
	// buzzer_Init();			//��ʼ��������IO
	/*------end*/		

	/*-----MAX30102ģ��*/
	MAX_IIC_Init();
	max30102_gpio_init();
	/*-----end*/	
#endif	
	/*------UWBģ��*/
	DW1000_gpio_init();
	spi_peripheral_init();
	USART1_Config();
	DW1000_EXTI_Configure();
	/*-------end*/

    return 0;
}


/*-------------------------------------*/
//������	USART1_Config
//���ܣ�	���ô���1
//������	��
//����ֵ��	��
/*-------------------------------------*/
static u8 buffsize[BUFFSIZE];//����������
void USART1_Config(void)
{

	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef	NVIC_Initstructure;

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure); 
	
	NVIC_Initstructure.NVIC_IRQChannel=USART1_IRQn;
	NVIC_Initstructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_Initstructure.NVIC_IRQChannelSubPriority=0;
	NVIC_Initstructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_Initstructure);
	

	USART_ClearFlag(USART1,USART_FLAG_IDLE);
	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);//ʹ�ܿ����ж�
	
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);//ʹ��USART_DMA_RX

	DMA_USART_config(DMA2_Stream5,DMA_Channel_4,(u32)&USART1->DR,(u32)buffsize,10);//����DMA USART1-RX
	

    USART_Cmd(USART1, ENABLE);
}

/*--------------------------------------------------*/
//������	DMA_USART_config
//���ܣ�	����USART_DMA����ģʽ
//������	DMA_Streamx	��DMA��
//����		chx	       	��ͨ��
//����		per_addr   	�������ַ
//����		target_addr	��Ŀ���ڴ��ַ
//������	ndtr		����Ҫ���������		
//����ֵ��	��		   
/*--------------------------------------------------*/
void DMA_USART_config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 per_addr,u32 target_addr,u16 ndtr )
{
	DMA_InitTypeDef  DMA_InitStructure;
		
	if((u32)DMA_Streamx>(u32)DMA2)//�õ���ǰstream������DMA2����DMA1
	{
		 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2ʱ��ʹ�� 
			
	}else 
	{
		 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1ʱ��ʹ�� 
	}
	DMA_DeInit(DMA_Streamx);
		
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}//�ȴ�DMA������ 
		
	/* ���� DMA Stream */
	DMA_InitStructure.DMA_Channel = chx;  //ͨ��ѡ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = per_addr;//DMA�����ַ
    DMA_InitStructure.DMA_Memory0BaseAddr = target_addr;//DMA �洢��0��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//���赽�ڴ�
	DMA_InitStructure.DMA_BufferSize = ndtr;//���ݴ����� 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Normal;// ʹ��ѭ��ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//�е����ȼ�
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
	DMA_Init(DMA_Streamx, &DMA_InitStructure);//��ʼ��DMA Stream
		


	DMA_Cmd(DMA_Streamx,ENABLE);
}

//���⣺�޷�DMA��ȡ����
u8 back_flag=0;//��־λ
u8 cmd=0;
alarm_struct *alarm_a;//����һ�����ⲿʹ�õ����ӽṹ��
/*---------------------------*/
//������	USART1_IRQHandler
//���ܣ�	USART1�жϴ�����
//������	��
//����ֵ��	��
/*---------------------------*/
void USART1_IRQHandler()
{
	u8 num;


	
	if(USART_GetITStatus(USART1,USART_IT_IDLE)==SET)//��������ж�
	{
		USART1->SR;
		USART1->DR;//��������жϱ�־λ
		
		DMA_Cmd(DMA2_Stream5,DISABLE);//ʧ��DMA
		num=10-DMA_GetCurrDataCounter(DMA2_Stream5);//�õ��������Ŀ
		

		
		if(num!=BUFFSIZE)
		{

			memset(buffsize,0,BUFFSIZE);

		}
		else//���ճ���ֻ��6
		{
			if((buffsize[0]==0xAA) && (buffsize[BUFFSIZE-1]==0xDD))//�ж�֡ͷ֡β
			{
				back_flag=1;
				alarm_a=(alarm_struct *)buffsize;//��ַָ��buffsize
				cmd=alarm_a->cmd;//�������ͣ���ֹ�����ӵ���֮ǰ���������ֵ
				printf("�趨���ӣ�%02d-%02d-%02d---week:%01d\n",alarm_a->data[0],alarm_a->data[1],alarm_a->data[2],alarm_a->data[3]);
			}
			
		}
		DMA_SetCurrDataCounter(DMA2_Stream5,10);
		DMA_Cmd(DMA2_Stream5,ENABLE);//����ʹ��DMA
	}

}
/*
 * ��������fputc
 * ����  ���ض���c�⺯��printf��USART1
 * ����  ����
 * ���  ����
 * ����  ����printf����
 */
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{
/* ��Printf���ݷ������� */
  
  
  USART_SendData(USART1, (unsigned char) ch);
  while (!(USART1->SR & USART_FLAG_TXE));
  //while ((USART1->SR & USART_FLAG_TC)==0);
  
 
  return (ch);
}
#endif
/*
int fputc(int ch, FILE *f)
{
//��Printf���ݷ������� 
  USART_SendData(USART1, (unsigned char) ch);
  while (!(USART1->SR & USART_FLAG_TXE));
 
  return (ch);
}
*/


void sendChar(unsigned char ch)
{

  USART_SendData(USART1, (unsigned char) ch);
  while (!(USART1->SR & USART_FLAG_TXE));
  //while (!(USART1->SR & USART_FLAG_TXE));
  //while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);//�ж����ݼĴ����Ƿ��Ѿ�Ϊ�գ��ȴ������
  
}
void USART_puts(uint8_t *s,uint8_t len)
{
    int i;
    for(i=0; i<len; i++)
    {
        putchar(s[i]);
    }
}




/*--------------------------------*/
//������	spi_peripheral_init
//���ܣ�	SPI��ʼ��
//������	��
//����ֵ��	��
/*---------------------------------*/
SPI_InitTypeDef SPI_InitStructure;
void spi_peripheral_init()
{
	
	SPI_I2S_DeInit(SPIx);

	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);

    /* SPI's has been initialized in the CubeMx code, see main.c */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;	 
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	//sets BR[2:0] bits - baudrate in SPI_CR1 reg bits 4-6
	SPI_InitStructure.SPI_BaudRatePrescaler = SPIx_PRESCALER_LOW;//fpck/32=84/32 <3MHZ
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPIx, &SPI_InitStructure);

	SPI_Cmd(SPIx,ENABLE);
}

/*-----------------------*/
//������	set_spi_rate_low
//���ܣ�	����SPIͨ������<3mhz
//������	��
//����ֵ��	��
/*------------------------*/
void set_spi_rate_low()
{
	
	uint16_t tmpreg = 0;

	SPI_Cmd(SPIx,DISABLE);
	/* Get the SPIx CR1 value */
	tmpreg = SPIx->CR1;

	/*clear the scaling bits*/
	tmpreg &= 0xFFC7;

	/*set the scaling bits*/
	tmpreg |= SPIx_PRESCALER_LOW;

	/* Write to SPIx CR1 */
	SPIx->CR1 = tmpreg;
	SPI_Cmd(SPIx,ENABLE);
}


/*-----------------------*/
//������	set_spi_rate_high
//���ܣ�	���SPIͨ������>3MHZ
//������	��
//����ֵ��	��
/*------------------------*/
void set_spi_rate_high()
{
	uint16_t tmpreg = 0;

	SPI_Cmd(SPIx,DISABLE);
	/* Get the SPIx CR1 value */
	tmpreg = SPIx->CR1;

	/*clear the scaling bits*/
	tmpreg &= 0xFFC7;

	/*set the scaling bits*/
	tmpreg |= SPIx_PRESCALER_HIGH;

	/* Write to SPIx CR1 */
	SPIx->CR1 = tmpreg;
	SPI_Cmd(SPIx,ENABLE);
}


/**
  * @brief  Checks whether the specified EXTI line is enabled or not.
  * @param  EXTI_Line: specifies the EXTI line to check.
  *   This parameter can be:
  *     @arg EXTI_Linex: External interrupt line x where x(0..19)
  * @retval The "enable" state of EXTI_Line (SET or RESET).
  */
ITStatus EXTI_GetITEnStatus(uint32_t x)
{
	
    return ((NVIC->ISER[(((uint32_t)x) >> 5UL)] &\
            (uint32_t)(1UL << (((uint32_t)x) & 0x1FUL)) ) == (uint32_t)RESET)?(RESET):(SET);
	
	/*
	ITStatus bitstatus = RESET;
    return bitstatus;
	*/
}

/* DW1000 IRQ handler declaration. */
port_deca_isr_t port_deca_isr = NULL;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn port_set_deca_isr()
 *
 * @brief This function is used to install the handling function for DW1000 IRQ.
 *
 * NOTE:
 *   - As EXTI9_5_IRQHandler does not check that port_deca_isr is not null, the user application must ensure that a
 *     proper handler is set by calling this function before any DW1000 IRQ occurs!
 *   - This function makes sure the DW1000 IRQ line is deactivated while the handler is installed.
 *
 * @param deca_isr function pointer to DW1000 interrupt handler to install
 *
 * @return none
 */
void port_set_deca_isr(port_deca_isr_t deca_isr)
{
    /* Check DW1000 IRQ activation status. */
    ITStatus en = port_GetEXT_IRQStatus();

    /* If needed, deactivate DW1000 IRQ during the installation of the new handler. */
    if (en)
    {
        port_DisableEXT_IRQ();
    }
    port_deca_isr = deca_isr;
    if (en)
    {
        port_EnableEXT_IRQ();
    }
}
/****************************************************************************//**
 *
 *                          End of configuration section
 *
 *******************************************************************************/

/****************************************************************************//**
 *
 *                          DW1000 port section
 *
 *******************************************************************************/

/* @fn      reset_DW1000
 * @brief   DW_RESET pin on DW1000 has 2 functions
 *          In general it is output, but it also can be used to reset the digital
 *          part of DW1000 by driving this pin low.
 *          Note, the DW_RESET pin should not be driven high externally.
 * */
//ѡ��SPI ģʽ0
void reset_DW1000(void)
{
    GPIO_InitTypeDef    GPIO_InitStruct;

    // Enable GPIO used for DW1000 reset as open collector output
    GPIO_InitStruct.GPIO_Pin = DW1000_RSTn;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	//GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStruct);
	
    //drive the RSTn pin low
    GPIO_WriteBit(DW1000_RSTn_GPIO, DW1000_RSTn, Bit_RESET);

    //usleep(1);
	deca_sleep(20);
	/*
	//drive the RSTn pin high
	GPIO_WriteBit(DW1000_RSTn_GPIO, DW1000_RSTn, Bit_SET);
	*/
    //put the pin back to tri-state ... as
    //output open-drain (not active)
	
	//NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn);//�ر��ⲿ�ж�
	
    GPIO_InitStruct.GPIO_Pin = DW1000_RSTn;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStruct);//IO����Ϊ���迹
	GPIO_WriteBit(DW1000_RSTn_GPIO, DW1000_RSTn, Bit_SET);
	//Sleep(2);
	deca_sleep(20);

}




/****************************************************************************//**
 *
 *******************************************************************************/

