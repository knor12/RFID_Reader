/**
  ******************************************************************************
  * File Name          : SPI.c
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "spi.h"

#include "gpio.h"

#include "trf79xxa.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi5 = {0};

/* SPI5 init function */
void MX_SPI5_Init(void)
{

  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler =SPI_BAUDRATEPRESCALER_256 ; //SPI_BAUDRATEPRESCALER_128;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
 // hspi5.Init.CRCPolynomial = 7;
 // hspi5.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  //hspi5.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_SPI_MspInit(&hspi5);

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(spiHandle->Instance==SPI5)
  {
  /* USER CODE BEGIN SPI5_MspInit 0 */

  /* USER CODE END SPI5_MspInit 0 */
    /* SPI5 clock enable */
    __HAL_RCC_SPI5_CLK_ENABLE();
  
    /**SPI5 GPIO Configuration    
    PF9     ------> SPI5_MOSI
    PF8     ------> SPI5_MISO
    PH6     ------> SPI5_SCK 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI5_MspInit 1 */

  /* USER CODE END SPI5_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI5)
  {
  /* USER CODE BEGIN SPI5_MspDeInit 0 */

  /* USER CODE END SPI5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI5_CLK_DISABLE();
  
    /**SPI5 GPIO Configuration    
    PF9     ------> SPI5_MOSI
    PF8     ------> SPI5_MISO
    PH6     ------> SPI5_SCK 
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_9|GPIO_PIN_8);

    HAL_GPIO_DeInit(GPIOH, GPIO_PIN_6);

  /* USER CODE BEGIN SPI5_MspDeInit 1 */

  /* USER CODE END SPI5_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
/*Nkessa   code for the NFRFID chip*/



//===============================================================
// NAME: void SPI_directCommand (uint8_t *pui8Buffer)
//
// BRIEF: Is used in SPI mode to transmit a Direct Command to
// reader chip.
//
// INPUTS:
//	Parameters:
//		uint8_t		*pui8Buffer		Direct Command
//
// OUTPUTS:
//
// PROCESS:	[1] transmit Direct Command
//
// CHANGE:
// DATE  		WHO	DETAIL
// 24Nov2010	RP	Original Code
// 07Dec2010	RP	integrated wait while busy loops
//===============================================================

void
SPI_directCommand(uint8_t ui8Command)
{
	SLAVE_SELECT_LOW; 						// Start SPI Mode

	// set Address/Command Word Bit Distribution to command
	ui8Command = (0x80 | ui8Command);					// command
	ui8Command = (0x9f & ui8Command);					// command code

	SPI_sendByte(ui8Command);
#if (TRF79xxA_VERSION == 60)
	SPI_sendByte(0x00);					// Dummy TX Write for Direct Commands per TRF796xA SPI Design Tips (sloa140)
#endif

	SLAVE_SELECT_HIGH; 						//Stop SPI Mode
}


//===============================================================
// NAME: void SPI_sendByte(uint8_t ui8TxByte)
//===============================================================
void SPI_sendByte(uint8_t ui8TxByte)
{

	 HAL_SPI_Transmit(&hspi5, &ui8TxByte, 1, NFRFID_WRITE_READ_TIME_OUT_MS);

}


//===============================================================
// NAME: void SPI_usciSet (void)
//
// BRIEF: Is used to set USCI B0 for SPI communication
//
// INPUTS:
//
// OUTPUTS:
//
// PROCESS:	[1] make settings
//
// CHANGE:
// DATE  		WHO	DETAIL
// 24Nov2010	RP	Original Code
// 07Dec2010	RP	reduced SPI clock frequency
//===============================================================

void SPI_usciSet(void)								//Uses USCI_B0
{

	MX_SPI5_Init();
}


//===============================================================
// NAME: void SPI_writeCont (uint8_t *pui8Buffer, uint8_t length)
//
// BRIEF: Is used in SPI mode to write to a specific number of
// reader chip registers from a specific address upwards.
//
// INPUTS:
//	uint8_t	*pui8Buffer	address of first register followed by the
//					contents to write
//	uint8_t	length	number of registers + 1
//
// OUTPUTS:
//
// PROCESS:	[1] write to the registers
//
// CHANGE:
// DATE  		WHO	DETAIL
// 24Nov2010	RP	Original Code
// 07Dec2010	RP	integrated wait while busy loops
//===============================================================

void
SPI_writeCont(uint8_t * pui8Buffer, uint8_t ui8Length)
{
	SLAVE_SELECT_LOW; 						// Start SPI Mode

	// Address/Command Wort Bit Distribution
	*pui8Buffer = (0x20 | *pui8Buffer); 				// address, write, continuous
	*pui8Buffer = (0x3f & *pui8Buffer);					// register address

	while(ui8Length-- > 0)
	{
		SPI_sendByte(*pui8Buffer++);
	}

	SLAVE_SELECT_HIGH; 						// Stop SPI Mode
}


//===============================================================
// NAME: void SPI_writeSingle (uint8_t *pui8Buffer, uint8_t length)
//
// BRIEF: Is used in SPI mode to write to a specified reader chip
// registers.
//
// INPUTS:
//	uint8_t	*pui8Buffer	addresses of the registers followed by the
//					contends to write
//	uint8_t	length	number of registers * 2
//
// OUTPUTS:
//
// PROCESS:	[1] write to the registers
//
// CHANGE:
// DATE  		WHO	DETAIL
// 24Nov2010	RP	Original Code
// 07Dec2010	RP	integrated wait while busy loops
//===============================================================

void SPI_writeSingle(uint8_t * pui8Buffer)
{
	SLAVE_SELECT_LOW; 						// Start SPI Mode

	// Address/Command Word Bit Distribution
	// address, write, single (fist 3 bits = 0)
	*pui8Buffer = (0x1f & *pui8Buffer);				// register address


	//SPI_sendByte(*pui8Buffer++);
	//SPI_sendByte(*pui8Buffer++);

	SPI_sendByte(pui8Buffer[0]);
	SPI_sendByte(pui8Buffer[1]);

	SLAVE_SELECT_HIGH; 						// Stop SPI Mode
}

//===============================================================
// NAME: void SPI_rawWrite (uint8_t *pui8Buffer, uint8_t length)
//
// BRIEF: Is used in SPI mode to write direct to the reader chip.
//
// INPUTS:
//	Parameters:
//		uint8_t		*pui8Buffer		raw data
//		uint8_t		length		number of data bytes
//
// OUTPUTS:
//
// PROCESS:	[1] send raw data to reader chip
//
//===============================================================

void SPI_rawWrite(uint8_t * pui8Buffer, uint8_t ui8Length, bool bContinuedSend)
{
	//Start SPI Mode
	SLAVE_SELECT_LOW;

	if (bContinuedSend)
	{
		SPI_sendByte(0x3F);
	}

	while(ui8Length-- > 0)
	{
		// Check if USCI_B0 TX buffer is ready
		//while (!(IFG2 & UCB0TXIFG));

		// Transmit data
		 HAL_SPI_Transmit(&hspi5, pui8Buffer, 1, NFRFID_WRITE_READ_TIME_OUT_MS);

		pui8Buffer++;
	}

	// Stop SPI Mode
	SLAVE_SELECT_HIGH;
}

//===============================================================
// NAME: void SPI_readCont (uint8_t *pui8Buffer, uint8_t length)
//
// BRIEF: Is used in SPI mode to read a specified number of
// reader chip registers from a specified address upwards.
//
// INPUTS:
//	Parameters:
//		uint8_t		*pui8Buffer		address of first register
//		uint8_t		length		number of registers
//
// OUTPUTS:
//
// PROCESS:	[1] read registers
//			[2] write contents to *pui8Buffer
//
//===============================================================

void
SPI_readCont(uint8_t * pui8Buffer, uint8_t ui8Length)
{
	SLAVE_SELECT_LOW; 							//Start SPI Mode

	// Address/Command Word Bit Distribution
	*pui8Buffer = (0x60 | *pui8Buffer); 					// address, read, continuous
	*pui8Buffer = (0x7f &*pui8Buffer);						// register address

	SPI_sendByte(*pui8Buffer);



	while(ui8Length-- > 0)
	{
		*pui8Buffer = SPI_receiveByte();
		pui8Buffer++;
	}

	SLAVE_SELECT_HIGH; 						// Stop SPI Mode
}


//*****************************************************************************
//
//! MCU_setCounter - Set up the counter for Timer A0
//!
//! \param ui16mSecTimeout is the amount of time to set the counter for.
//!
//! Set up the counter for Timer A0 and enable the interrupt.
//!
//! \return None.
//
//*****************************************************************************
void delay_us(const uint16_t us){
  uint32_t i = us * 400;
  while (i-- > 0) {
    asm("nop");
  }
}
volatile uint32_t NFRFID_timer_settings_ms = 0;
volatile uint32_t NFRFID_timer_running = 0 ;
void MCU_setCounter(uint16_t ui16mSecTimeout)
{
	NFRFID_timer_running =1 ;
	NFRFID_timer_settings_ms   = ui16mSecTimeout;
}


//===============================================================
// NAME: void SPI_readSingle (uint8_t *pui8Buffer)
//
// BRIEF: Is used in SPI mode to read specified reader chip
// registers.
//
// INPUTS:
//	Parameters:
//		uint8_t		*pui8Buffer		addresses of the registers
//
// OUTPUTS:
//
// PROCESS:	[1] read registers
//			[2] write contents to *pui8Buffer
//
//===============================================================

void SPI_readSingle(uint8_t * pui8Buffer)
{
	SLAVE_SELECT_LOW; 						// Start SPI Mode

	// Address/Command Word Bit Distribution
	*pui8Buffer = (0x40 | *pui8Buffer); 			// address, read, single
	*pui8Buffer = (0x5f & *pui8Buffer);				// register address

	uint8_t buff[]={0,0} ;

	HAL_SPI_TransmitReceive(&hspi5, pui8Buffer, &buff[0], 1,
	                                          NFRFID_WRITE_READ_TIME_OUT_MS);

	HAL_SPI_TransmitReceive(&hspi5, &buff[1], pui8Buffer, 1,
		                                          NFRFID_WRITE_READ_TIME_OUT_MS);

	//SPI_sendByte(*pui8Buffer);					// Previous data to TX, RX

	//*pui8Buffer = buff[1] ;


	SLAVE_SELECT_HIGH; 						// Stop SPI Mode
}

//===============================================================
// NAME: uint8_t SPI_receiveByte(void)
//===============================================================

uint8_t SPI_receiveByte(void)
{

	uint8_t buf_rx=0x00;
	uint8_t buf_tx=0x00;
	HAL_SPI_TransmitReceive(&hspi5, &buf_tx, &buf_rx, 1,
		                                          NFRFID_WRITE_READ_TIME_OUT_MS);

	return buf_rx;
}

//===============================================================
// NAME: void SPI_directMode (void)
//
// BRIEF: Is used in SPI mode to start Direct Mode.
//
// INPUTS:
//
// OUTPUTS:
//
// PROCESS:	[1] start Direct Mode
//
// NOTE: No stop condition
//
//===============================================================

void SPI_directMode(void)
{
	uint8_t pui8Command[2];

	pui8Command[0] = TRF79XXA_CHIP_STATUS_CONTROL;
	pui8Command[1] = TRF79XXA_CHIP_STATUS_CONTROL;
	SPI_readSingle(&pui8Command[1]);
	pui8Command[1] |= 0x60;						// RF on and BIT 6 in Chip Status Control Register set
	SPI_writeSingle(pui8Command);
}


void SPI_setup(void)
{
	TRF_ENABLE_SET;

	IRQ_PIN_SET;
	IRQ_EDGE_SET;								// rising edge interrupt

	SPI_usciSet();								// Set the USART

	LED_ALL_OFF;
	LED_PORT_SET;
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
