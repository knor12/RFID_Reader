/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
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
#include "main.h"
#include "stm32f7xx_hal.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include  "stdio.h"
#include "stdlib.h"
#include "nfc_app.h"
#include "trf79xxa.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

volatile int    i =0 ;
void reset_counter(void ){
	//HAL_UART_Transmit(&huart6, "intr\n\r", strlen( "intr\n\r"),100);
	i=0;
}


void log_byte_decimal(uint8_t data){
	char buf[4]={0};
	sprintf(buf , "%d" ,data );
	log_message(buf);
}

void log_message(char * buff){

	HAL_UART_Transmit(&huart6 , buff , strlen(buff) , 100);
	//HAL_UART_STATE_BUSY
	// while ( huart6.gState == HAL_UART_STATE_BUSY_TX){
	//wait to finish sending
	// }
}

void log_char( char c){
	char * buf[2];
	buf[0]=c;
	buf[1]=0;
	log_message(buf);

}

void log_byte(uint8_t ui8TxByte){
	char buf[5]={0,0};

	//log_message(&buf[0]);

	uint8_t	ui8TempVar1 = 0, ui8TempVar2 = 0;

	ui8TempVar1 = (ui8TxByte >> 4) & 0x0F;			// get high nibble
	sprintf(buf ,"%d" , ui8TempVar1 ); // convert to ASCII
	//ui8TempVar2 = UART_nibble2Ascii(ui8TempVar1);
	//UART_putChar(ui8TempVar2);						// output */

	ui8TempVar1 = ui8TxByte & 0x0F;
	sprintf(buf ,"%d" , ui8TempVar1 ); // convert to ASCII
	log_message(&buf[0]);
	//ui8TempVar2 = UART_nibble2Ascii(ui8TempVar1);		// convert to ASCII
	//UART_putChar(ui8TempVar2);
}


void test_NFRFID_SPI_Communication(void){

	char buf[100]={0};
	uint8_t reg_address[]={0x12,0x13};
	uint8_t reg_wrote[] ={0xF0, 0X0F};
	uint8_t reg_read[]={0,0};

	int i =0;
	for(i=0;i<2;i++){

		//write
		TRF79xxA_writeRegister(reg_address[i],reg_wrote[i] );

		//read
		reg_read[i]=TRF79xxA_readRegister(reg_address[i]);

		//build string
		buf[0]=0;
		sprintf(buf , "register:%x, wrote:%x, read:%x , \n\r" , reg_address[i] , reg_wrote[i] , reg_read[i]);

		//print string
		log_message(buf);


	}

	//void TRF79xxA_writeRegister(uint8_t ui8TrfRegister, uint8_t ui8Value);
	//extern uint8_t TRF79xxA_readRegister(uint8_t ui8TrfRegister);

}

int main(void)
{



	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI5_Init();
	MX_USART1_UART_Init();
	MX_USART6_UART_Init();

	/* USER CODE BEGIN 2 */
	SLAVE_SELECT_LOW;
	HAL_Delay(5);
	SLAVE_SELECT_HIGH;
	// Wait until TRF system clock started
	HAL_Delay(5);

	//set the enable Pin high
	TRF_DISABLE;
	HAL_Delay(5);
	TRF_ENABLE;

	// Set up TRF initial settings
	TRF79xxA_initialSettings();
	TRF79xxA_setTrfPowerSetting(TRF79xxA_5V_HALF_POWER);
	// Initialize all enabled technology layers
	NFC_init();

	//test_NFRFID_SPI_Communication();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	HAL_UART_Transmit(&huart6, "Starting....\n\r", strlen( "Starting....\n\r"),100);
	IRQ_ON;
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		// Poll for NFC tags
		//TRF79xxA_irqHandler();
		NFC_findTag();

		char buff[30]={0};
		i+=1;
		sprintf(buff , "%d " , i);

		extern UART_HandleTypeDef huart6;
		HAL_Delay(2000);
		//HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
		HAL_UART_Transmit(&huart6 , buff , sizeof(buff) , 100);
		HAL_UART_Transmit(&huart6, "Hello world\n\r", strlen( "Hello world\n\r"),100);

	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 10;
	RCC_OscInitStruct.PLL.PLLN = 210;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART6;
	PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInitStruct.Usart6ClockSelection = RCC_USART6CLKSOURCE_PCLK2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
