/*! ----------------------------------------------------------------------------
 * @file    deca_spi.c
 * @brief   SPI access functions
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

#include "deca_spi.h"
#include "deca_device_api.h"
#include "port.h"
#include "stm32f4xx.h"


//extern  SPI_HandleTypeDef hspi1;    /*clocked from 72MHz*/

/****************************************************************************//**
 *
 *                              DW1000 SPI section
 *
 *******************************************************************************/
/*! ------------------------------------------------------------------------------------------------------------------
 * Function: openspi()
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(/*SPI_TypeDef* SPIx*/)
{
    return 0;
} // end openspi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: closespi()
 *
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void)
{
    return 0;
} // end closespi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success
 */
 int writetospi_serial( uint16_t headerLength,
			   	    const uint8_t *headerBuffer,
					uint32_t bodylength,
					const uint8_t *bodyBuffer
				  );

int readfromspi_serial( uint16_t	headerLength,
			    	 const uint8_t *headerBuffer,
					 uint32_t readlength,
					 uint8_t *readBuffer );
					 
					 

int writetospi_serial(uint16_t headerLength,
               const    uint8_t *headerBuffer,
               uint32_t bodyLength,
               const    uint8_t *bodyBuffer)
{
	int i;
    decaIrqStatus_t  stat ;
    stat = decamutexon() ;
    //SPIx_CS_GPIO->BSRRH = 1<<4;//对GPIOB12复位
    GPIO_WriteBit(GPIOA,GPIO_Pin_4,Bit_RESET);
    //SPI_BiDirectionalLineConfig(SPIx, SPI_Direction_Tx);
    for(i=0; i<headerLength; i++)
    {
        while ((SPIx->SR & SPI_I2S_FLAG_TXE) == (uint16_t)RESET);

				SPIx->DR = headerBuffer[i];

        while ((SPIx->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);
    	
        SPIx->DR ;
        
    }

    for(i=0; i<bodyLength; i++)
    {
    	while((SPIx->SR & SPI_I2S_FLAG_TXE) == (uint16_t)RESET);

     	SPIx->DR = bodyBuffer[i];

      while ((SPIx->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);
    	
      SPIx->DR ;
        
	}

    //SPIx_CS_GPIO->BSRRL = 1<<4;//对GPIOB12置位
    GPIO_WriteBit(GPIOA,GPIO_Pin_4,Bit_SET);

    decamutexoff(stat);

    return 0;
} // end writetospi()


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns 0
 */

int readfromspi_serial(uint16_t headerLength,
                const uint8_t *headerBuffer,
                uint32_t readlength,
                uint8_t *readBuffer)
{
	int i=0;

    decaIrqStatus_t  stat ;

    stat = decamutexon() ;

    /* Wait for SPIx Tx buffer empty */
    //while (port_SPIx_busy_sending());

    //SPIx_CS_GPIO->BSRRH = 1<<4;
    GPIO_WriteBit(GPIOA,GPIO_Pin_4,Bit_RESET);

    //SPI_BiDirectionalLineConfig(SPIx, SPI_Direction_Tx);
    for(i=0; i<headerLength; i++)
    {
     	while( (SPIx->SR & SPI_I2S_FLAG_TXE) == (uint16_t)RESET);

    	SPIx->DR = headerBuffer[i];

      while ((SPIx->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);
     	readBuffer[0] = SPIx->DR ; // Dummy read as we write the header
    }
		
    /*清除ovr flag*/
    SPIx->DR;
		SPIx->SR;

    //SPI_BiDirectionalLineConfig(SPIx, SPI_Direction_Rx);
    for(i=0; i<readlength;i++)
    {
			
       while( (SPIx->SR & SPI_I2S_FLAG_TXE) == (uint16_t)RESET);
    	SPIx->DR = 0x00;  // Dummy write as we read the message body

    	while((SPIx->SR & SPI_I2S_FLAG_RXNE) == (uint16_t)RESET);
	    readBuffer[i] = SPIx->DR ;//port_SPIx_receive_data(); //this clears RXNE bit  
    }

    /*清除ovr flag*/
    SPIx->DR;
		SPIx->SR;

    //SPIx_CS_GPIO->BSRRL = 1<<4;
    GPIO_WriteBit(GPIOA,GPIO_Pin_4,Bit_SET);

    decamutexoff(stat) ;
    return 0;
} // end readfromspi()

/****************************************************************************//**
 *
 *                              END OF DW1000 SPI section
 *
 *******************************************************************************/

