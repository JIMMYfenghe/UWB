/*! ----------------------------------------------------------------------------
 * @file    port.h
 * @brief   HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */


#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include <stdio.h>


#include "stm32f4xx.h"

//是否使用中断模式
#define INT_MODE_ENABLE
//#define INT_MODE_DISABLE


/*****************************************************************************************************************//*
**/

 /****************************************************************************//**
  *
  *                                 Types definitions
  *
  *******************************************************************************/




extern int writetospi_serial(uint16_t headerLength,
                             const uint8_t *headerBuffer,
                             uint32_t bodylength,
                             const uint8_t *bodyBuffer);

extern int readfromspi_serial(uint16_t	headerLength,
                              const uint8_t *headerBuffer,
                              uint32_t readlength,
                              uint8_t *readBuffer );

#define writetospi  writetospi_serial
#define readfromspi readfromspi_serial

typedef enum
{
    LED_PC6, //LED5
    LED_PC7, //LED6
    LED_PC8, //LED7
    LED_PC9, //LED8
    LED_ALL,
    LEDn
} led_t;

/****************************************************************************//**
 *
 *                              MACRO
 *
 *******************************************************************************/






#define SPIx_PRESCALER				SPI_BaudRatePrescaler_8
#define SPIx_PRESCALER_LOW			SPI_BaudRatePrescaler_32
#define SPIx_PRESCALER_HIGH			SPI_BaudRatePrescaler_8

#define SPIx						SPI1
#define SPIx_GPIO					GPIOA
#define SPIx_CS						GPIO_Pin_4
#define SPIx_CS_GPIO				GPIOA
#define SPIx_SCK					GPIO_Pin_5
#define SPIx_MISO					GPIO_Pin_6
#define SPIx_MOSI					GPIO_Pin_7

#define DW1000_RSTn					GPIO_Pin_4
#define DW1000_RSTn_GPIO			GPIOB

/*
#define DECARSTIRQ                  GPIO_Pin_0
#define DECARSTIRQ_GPIO             GPIOA
#define DECARSTIRQ_EXTI             EXTI_Line0
#define DECARSTIRQ_EXTI_PORT        GPIO_PortSourceGPIOA
#define DECARSTIRQ_EXTI_PIN         GPIO_PinSource0
#define DECARSTIRQ_EXTI_IRQn        EXTI0_IRQn
*/
#define DECAIRQ                     GPIO_Pin_0
#define DECAIRQ_GPIO                GPIOB
#define DECAIRQ_EXTI                EXTI_Line0
#define DECAIRQ_EXTI_PORT           GPIO_PortSourceGPIOB
#define DECAIRQ_EXTI_PIN            GPIO_PinSource0
#define DECAIRQ_EXTI_IRQn           EXTI0_IRQn
#define DECAIRQ_EXTI_USEIRQ         ENABLE
#define DECAIRQ_EXTI_NOIRQ          DISABLE








/****************************************************************************//**
 *
 *                              port function prototypes
 *
 *******************************************************************************/
 typedef struct
{
	u8 fhead;//帧头
	u8 cmd;//命令 01 吃药  02 起床 03 吃饭  04 睡觉
	u8 data[4];//闹钟内容(时分秒星期)
	u8 real;//帧尾
}alarm_struct;


#define BUFFSIZE 7

//void NVIC_Configuration(void);
void DMA_USART_config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 per_addr,u32 target_addr,u16 ndtr );
 
 
 
 
 
void usleep(int usec);
void Sleep(uint32_t Delay);
unsigned long portGetTickCnt(void);

#define portGetTickCount() 			portGetTickCnt()
void port_wakeup_dw1000(void);
void port_wakeup_dw1000_fast(void);
void USART_puts(uint8_t *s,uint8_t len);

void NVIC_Configuration(void);
int RCC_Configuration(void);
void DW1000_gpio_init(void);
void USART1_Config(void);
void sendChar(unsigned char ch);
void set_spi_rate_low(void);
void set_spi_rate_high(void);
void DW1000_EXTI_Configure(void);


int  peripherals_init(void);
void spi_peripheral_init(void);

void setup_DW1000RSTnIRQ(int enable);

void reset_DW1000(void);

void INT_callback(void (*callback)(void));
/*
ITStatus EXTI_GetITEnStatus(uint32_t x);

uint32_t port_GetEXT_IRQStatus(void);
uint32_t port_CheckEXT_IRQ(void);
void port_DisableEXT_IRQ(void);
void port_EnableEXT_IRQ(void);
*/
ITStatus EXTI_GetITEnStatus(uint32_t x);

#define port_GetEXT_IRQStatus()             EXTI_GetITEnStatus(DECAIRQ_EXTI_IRQn)
#define port_DisableEXT_IRQ()               NVIC_DisableIRQ(DECAIRQ_EXTI_IRQn)
#define port_EnableEXT_IRQ()                NVIC_EnableIRQ(DECAIRQ_EXTI_IRQn)
#define port_CheckEXT_IRQ()                 GPIO_ReadInputDataBit(DECAIRQ_GPIO, DECAIRQ)
int NVIC_DisableDECAIRQ(void);
//HAL_StatusTypeDef   flush_report_buff(void);


/* DW1000 IRQ (EXTI0_IRQ) handler type. */
typedef void (*port_deca_isr_t)(void);

void port_set_deca_isr(port_deca_isr_t deca_isr);


#ifdef __cplusplus
}
#endif

#endif /* PORT_H_ */
/*
 * Taken from the Linux Kernel
 *
 */

#ifndef _LINUX_CIRC_BUF_H
#define _LINUX_CIRC_BUF_H 1

struct circ_buf {
    char *buf;
    int head;
    int tail;
};

/* Return count in buffer.  */
#define CIRC_CNT(head,tail,size) (((head) - (tail)) & ((size)-1))

/* Return space available, 0..size-1.  We always leave one free char
   as a completely full buffer has head == tail, which is the same as
   empty.  */
#define CIRC_SPACE(head,tail,size) CIRC_CNT((tail),((head)+1),(size))

/* Return count up to the end of the buffer.  Carefully avoid
   accessing head and tail more than once, so they can change
   underneath us without returning inconsistent results.  */
#define CIRC_CNT_TO_END(head,tail,size) \
    ({int end = (size) - (tail); \
      int n = ((head) + end) & ((size)-1); \
      n < end ? n : end;})

/* Return space available up to the end of the buffer.  */
#define CIRC_SPACE_TO_END(head,tail,size) \
    ({int end = (size) - 1 - (head); \
      int n = (end + (tail)) & ((size)-1); \
      n <= end ? n : end+1;})

#endif /* _LINUX_CIRC_BUF_H  */

