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



/*回调函数*/
void (*rx_callback)(void);
/*------------------------*/
//函数：	INT_callback
//功能：	初始化回调函数
//参数：	需要执行或注册的回调函数
//返回值：	无
/*------------------------*/
void INT_callback(void (*callback)(void))
{
	rx_callback=callback;
}

/*--------------------------*/
//函数：	SysTick_Configuration
//功能：	配置滴答定时器节拍和中断优先级
//参数：	无
//返回值：	int型
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
//函数：	NVIC_Configuration
//功能：	配置中断分组
//参数：	无
//返回值：	无
/*-----------------------*/
void NVIC_Configuration()
{

	/* Set NVIC Grouping to 16 groups of interrupt without sub-grouping */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);



}

/*--------------------------------*/
//函数：	DW1000_EXTI_Configure
//功能：	配置DW1000在MCU上的外部中断线
//参数：	无
//返回值：	无
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

		NVIC_InitStructure.NVIC_IRQChannel = DECAIRQ_EXTI_IRQn;//外部中断0
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;//抢占优先级6
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
		NVIC_Init(&NVIC_InitStructure);//配置	
}
/*----------------------------------*/
//函数：	RCC_Configuration
//功能：	系统时钟初始化和外设选择
//参数：	无
//返回值：	无
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

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	


	return 0;
}

/*--------------------------*/
//函数：	DW1000_gpio_init
//功能：	DW1000的GPIO配置
//参数：	无
//返回值：	无
/*--------------------------*/
void DW1000_gpio_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//IO复用SPI1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);
	//复用为USART1
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
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//--推免输出模式
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
	/*------跌倒模块*/
	  ADXL_IIC_Init();  		//初始化IIC总线  影响UWB
	  ADXL345_INT();			//初始化中断引脚
	// buzzer_Init();			//初始化蜂鸣器IO
	/*------end*/		

	/*-----MAX30102模块*/
	MAX_IIC_Init();
	max30102_gpio_init();
	/*-----end*/	
#endif	
	/*------UWB模块*/
	DW1000_gpio_init();
	spi_peripheral_init();
	USART1_Config();
	DW1000_EXTI_Configure();
	/*-------end*/

    return 0;
}


/*-------------------------------------*/
//函数：	USART1_Config
//功能：	配置串口1
//参数：	无
//返回值：	无
/*-------------------------------------*/
static u8 buffsize[BUFFSIZE];//缓存数据区
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
	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);//使能空闲中断
	
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);//使能USART_DMA_RX

	DMA_USART_config(DMA2_Stream5,DMA_Channel_4,(u32)&USART1->DR,(u32)buffsize,10);//配置DMA USART1-RX
	

    USART_Cmd(USART1, ENABLE);
}

/*--------------------------------------------------*/
//函数：	DMA_USART_config
//功能：	配置USART_DMA工作模式
//参数：	DMA_Streamx	：DMA流
//参数		chx	       	：通道
//参数		per_addr   	：外设地址
//参数		target_addr	：目标内存地址
//参数：	ndtr		：需要传输的数量		
//返回值：	无		   
/*--------------------------------------------------*/
void DMA_USART_config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 per_addr,u32 target_addr,u16 ndtr )
{
	DMA_InitTypeDef  DMA_InitStructure;
		
	if((u32)DMA_Streamx>(u32)DMA2)//得到当前stream是属于DMA2还是DMA1
	{
		 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能 
			
	}else 
	{
		 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 
	}
	DMA_DeInit(DMA_Streamx);
		
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}//等待DMA可配置 
		
	/* 配置 DMA Stream */
	DMA_InitStructure.DMA_Channel = chx;  //通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr = per_addr;//DMA外设地址
    DMA_InitStructure.DMA_Memory0BaseAddr = target_addr;//DMA 存储器0地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设到内存
	DMA_InitStructure.DMA_BufferSize = ndtr;//数据传输量 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Normal;// 使用循环模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//中等优先级
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_Init(DMA_Streamx, &DMA_InitStructure);//初始化DMA Stream
		


	DMA_Cmd(DMA_Streamx,ENABLE);
}

//问题：无法DMA读取数据
u8 back_flag=0;//标志位
u8 cmd=0;
alarm_struct *alarm_a;//定义一个供外部使用的闹钟结构体
/*---------------------------*/
//函数：	USART1_IRQHandler
//功能：	USART1中断处理函数
//参数：	无
//返回值：	无
/*---------------------------*/
void USART1_IRQHandler()
{
	u8 num;


	
	if(USART_GetITStatus(USART1,USART_IT_IDLE)==SET)//进入空闲中断
	{
		USART1->SR;
		USART1->DR;//清除空闲中断标志位
		
		DMA_Cmd(DMA2_Stream5,DISABLE);//失能DMA
		num=10-DMA_GetCurrDataCounter(DMA2_Stream5);//得到传输的数目
		

		
		if(num!=BUFFSIZE)
		{

			memset(buffsize,0,BUFFSIZE);

		}
		else//接收长度只有6
		{
			if((buffsize[0]==0xAA) && (buffsize[BUFFSIZE-1]==0xDD))//判断帧头帧尾
			{
				back_flag=1;
				alarm_a=(alarm_struct *)buffsize;//地址指向buffsize
				cmd=alarm_a->cmd;//保存类型，防止在闹钟到来之前被清除类型值
				printf("设定闹钟：%02d-%02d-%02d---week:%01d\n",alarm_a->data[0],alarm_a->data[1],alarm_a->data[2],alarm_a->data[3]);
			}
			
		}
		DMA_SetCurrDataCounter(DMA2_Stream5,10);
		DMA_Cmd(DMA2_Stream5,ENABLE);//重新使能DMA
	}

}
/*
 * 函数名：fputc
 * 描述  ：重定向c库函数printf到USART1
 * 输入  ：无
 * 输出  ：无
 * 调用  ：由printf调用
 */
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{
/* 将Printf内容发往串口 */
  
  
  USART_SendData(USART1, (unsigned char) ch);
  while (!(USART1->SR & USART_FLAG_TXE));
  //while ((USART1->SR & USART_FLAG_TC)==0);
  
 
  return (ch);
}
#endif
/*
int fputc(int ch, FILE *f)
{
//将Printf内容发往串口 
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
  //while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);//判断数据寄存器是否已经为空，等大发送完毕
  
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
//函数：	spi_peripheral_init
//功能：	SPI初始化
//参数：	无
//返回值：	无
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
//函数：	set_spi_rate_low
//功能：	降低SPI通信速率<3mhz
//参数：	无
//返回值：	无
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
//函数：	set_spi_rate_high
//功能：	提高SPI通信速率>3MHZ
//参数：	无
//返回值：	无
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
//选定SPI 模式0
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
	
	//NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn);//关闭外部中断
	
    GPIO_InitStruct.GPIO_Pin = DW1000_RSTn;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(DW1000_RSTn_GPIO, &GPIO_InitStruct);//IO设置为高阻抗
	GPIO_WriteBit(DW1000_RSTn_GPIO, DW1000_RSTn, Bit_SET);
	//Sleep(2);
	deca_sleep(20);

}




/****************************************************************************//**
 *
 *******************************************************************************/

