
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "fonts.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

char TxBuff[4];
char RxBuff[4];

char CRxBuff[2];
char CTxBuff[2];
char CBtRst[1]= "$";	//00100100 	$ 	RESET

char StrInit[5];
char StrAxis[5];
char StrMult[5];

int NroAxis, NroAx;
int NroMult, NroFe;
int Tx;
int BHold = 0;

GPIO_PinState EncState1;
GPIO_PinState EncState2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void RstSlave(void);
void InitOLED(char* StrInit);
void MenuOLED(char* StrAxis, char* StrMult);
void AxisOLED(char* StrAxis);
void MultOLED(char* StrMult);
void InitLED(void);
void TxChar(char* CTxBuff);
void BottomHold(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	/** 
	#define UART_IT_MASK  0x0000FFFFU
	#define __HAL_UART_ENABLE_IT(__HANDLE__, __INTERRUPT__)   ((((__INTERRUPT__) >> 28U) == 1U)? ((__HANDLE__)->Instance->CR1 |= ((__INTERRUPT__) & UART_IT_MASK)): \
                                                           (((__INTERRUPT__) >> 28U) == 2U)? ((__HANDLE__)->Instance->CR2 |=  ((__INTERRUPT__) & UART_IT_MASK)): \
                                                        ((__HANDLE__)->Instance->CR3 |= ((__INTERRUPT__) & UART_IT_MASK)))
	*	@brief  Enable the specified UART interrupt.
  * @param  __HANDLE__ specifies the UART Handle.
  *         This parameter can be UARTx where x: 1, 2, 3, 4, 5, 6, 7 or 8 to select the USART or 
  *         UART peripheral.
  * @param  __INTERRUPT__ specifies the UART interrupt source to enable.
  *          This parameter can be one of the following values:
  *            @arg UART_IT_CTS:  CTS change interrupt
  *            @arg UART_IT_LBD:  LIN Break detection interrupt
  *            @arg UART_IT_TXE:  Transmit Data Register empty interrupt
  *            @arg UART_IT_TC:   Transmission complete interrupt
  *            @arg UART_IT_RXNE: Receive Data register not empty interrupt
  *            @arg UART_IT_IDLE: Idle line detection interrupt
  *            @arg UART_IT_PE:   Parity Error interrupt
  *            @arg UART_IT_ERR:  Error interrupt(Frame error, noise error, overrun error)
  * @retval None
  */
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_TC|UART_IT_RXNE);
	ssd1306_Init();
	InitOLED("bts");
	
	HAL_Delay(2000);
	
	MenuOLED("X","  *1");
	InitLED();
	NroAxis = 1;
	NroMult = 1;
	NroAx = 1;
	NroFe = 1;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		/**
		GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
		* @brief  Reads the specified input port pin.
		* @param  GPIOx where x can be (A..K) to select the GPIO peripheral for STM32F429X device or
		*                      x can be (A..I) to select the GPIO peripheral for STM32F40XX and STM32F427X devices.
		* @param  GPIO_Pin specifies the port bit to read.
		*         This parameter can be GPIO_PIN_x where x can be (0..15).
		* @retval The input port pin value.
		*/
		EncState1 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, EncState1);
		EncState2 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, EncState2);	
			
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET)
		{
			switch(NroAxis)
			{
				case 1:
					AxisOLED("Y");
					/*
					//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
					//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
					//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);*/
					NroAxis++;
					break;
				case 2:
					AxisOLED("Z");
					/*
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
					//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
					//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);*/
					NroAxis++;
					break;
				case 3:
					AxisOLED("X");
					/*
					//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
					//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
					//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);*/
					NroAxis++;
					break;
				}
			if(NroAxis == 4)
				NroAxis = 1;
			HAL_Delay(200);
			}
			//Mult
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET)
		{
			switch(NroMult)
			{
				case 1:
					MultOLED(" *10");
					/*
					//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
					//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
					//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);*/
					NroMult++;
					break;
				case 2:
					MultOLED("*100");
					/*
					//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
					//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
					//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);*/
					NroMult++;
					break;
				case 3:
					MultOLED("  *1");
					/*
					//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
					//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
					//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);*/
					NroMult++;
					break;
				}
			if(NroMult == 4)
				NroMult = 1;
			HAL_Delay(200);
			}
		/**
		int sprintf(char * __restrict *s*, const char * __restrict *format*, ...) __attribute__((__nonnull__(1,2)));
		* is equivalent to fprintf, except that the argument s specifies an array into which the generated output is to be written, 
		* rather than to a stream. A null character is written at the end of the characters written;
		* it is not counted as part of the returned sum.
		* Returns: the number of characters written to the array, not counting the terminating null character.
		*/
		BottomHold();
		if(BHold == 0)
			sprintf(TxBuff, "%i%i%u%u", NroAxis, NroMult, EncState1, EncState2);
		else if (BHold == 1)
			sprintf(TxBuff, "%i%i%u%u", NroAx, NroFe, EncState1, EncState2);
		Tx = atoi(TxBuff);
		TxChar(CTxBuff);
		/**
		HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
		* @brief  Sends an amount of data in non blocking mode.
		* @param  huart pointer to a UART_HandleTypeDef structure that contains
		*                the configuration information for the specified UART module.
		* @param  pData Pointer to data buffer
		* @param  Size Amount of data to be sent
		* @retval HAL status
		*/
		HAL_UART_Transmit_IT(&huart3, (uint8_t *)CTxBuff, strlen(CTxBuff));
			
		/**
		BINARY  	ASCII		|
		------------------
		00101000	(				00111000	8				01010000	P				01101000	h				01110100	t
		00101010	*				00111010	:				01010010	R				01101010	j				01110110	v
		00101100	,				00111100	<				01010100	T				01101100	l				01111000	x
		00101110	.				00111110	>				01010110	V				01101110	n				01111010	z
		00110000	0				01001000	H				01011000	X				01110000	p				01111100	|
		00110010	2				01001010	J				01011010	Z				01110010	r				01111110	~
		00110100	4				01001100	L				01011100  \ ->00100110 	&
		00110110	6				01001110	N				01011110	^				
		*/	
			
	}
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_2;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PE12 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void InitLED(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
}
void InitOLED(char* StrInit)
{
	ssd1306_Fill(Black);				//fill black
	ssd1306_SetCursor(40,2);			//position
	ssd1306_WriteString(StrInit, Font_16x26, White);
	ssd1306_UpdateScreen();			//update OLED
}
/*void ParingOLED(char* strParingOled)
{
	ssd1306_Fill(Black);				//fill black
	ssd1306_SetCursor(40,2);			//position
	ssd1306_WriteString(StrParingOled, Font_11x18, White);
	ssd1306_UpdateScreen();			//update OLED
}*/
void RstSlave(void)
{
	HAL_UART_Transmit_IT(&huart3, (uint8_t *)CBtRst, strlen(CBtRst)); // tx "rstsl"
}
void MenuOLED(char* StrAxis, char* StrMult)
{
	
	ssd1306_Fill(Black);				//fill black
	ssd1306_SetCursor(25,8);			//position
	ssd1306_WriteString(StrAxis, Font_11x18, White);
	ssd1306_SetCursor(55,3);			//position
	ssd1306_WriteString("|", Font_16x26, White);
	ssd1306_SetCursor(80,8);			//position
	ssd1306_WriteString(StrMult, Font_11x18, White);
	
	ssd1306_UpdateScreen();			//update OLED
}
void AxisOLED(char* StrAxis)
{
	ssd1306_SetCursor(25,8);			//position
	ssd1306_WriteString(StrAxis, Font_11x18, White);
	ssd1306_UpdateScreen();			//update OLED
}
void MultOLED(char* StrMult)
{
	ssd1306_SetCursor(80,8);			//position
	ssd1306_WriteString(StrMult, Font_11x18, White);
	ssd1306_UpdateScreen();			//update OLED
}

void BottomHold(void)
{
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET && BHold == 0)
	{
		HAL_Delay(3000);
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_RESET)
		{
			NroAx = NroAxis;
			NroFe = NroMult;
		}
	}
}
void TxChar(char* CTxBuff)
{
	switch(Tx)
	{
		//Axis X					// LSB & MSB not count
		case 1100:				// binary = 00101000 
			CTxBuff[0] = '(';	
			break;
		case 1101:				// binary = 00101010
			CTxBuff[0] = '*';	
			break;
		case 1110:				// binary = 00101100
			CTxBuff[0] = ',';	
			break;
		case 1111:				// binary = 00101110
			CTxBuff[0] = '.';	
			break;
		case 1200:				// binary = 00110000 
			CTxBuff[0] = '0';
			break;
		case 1201:				// binary = 00110010 
			CTxBuff[0] = '2';
			break;
		case 1210:				// binary = 00110100 
			CTxBuff[0] = '4';
			break;
		case 1211:				// binary = 00110110 
			CTxBuff[0] = '6';
			break;
		case 1300:				// binary = 00111000 
			CTxBuff[0] = '8';
			break;
		case 1301:				// binary = 00111010 
			CTxBuff[0] = ':';
			break;
		case 1310:				// binary = 00111100 
			CTxBuff[0] = '<';
			break;
		case 1311:				// binary = 00111110 
			CTxBuff[0] = '>';
			break;
		//Axis Y			
		case 2100:				// binary = 01001000 
			CTxBuff[0] = 'H';
			break;
		case 2101:				// binary = 01001010 
			CTxBuff[0] = 'J';
			break;
		case 2110:				// binary = 01001100 
			CTxBuff[0] = 'L';
			break;
		case 2111:				// binary = 01001110 
			CTxBuff[0] = 'N';
			break;
		case 2200:				// binary = 01010000 
			CTxBuff[0] = 'P';
			break;
		case 2201:				// binary = 01010010 
			CTxBuff[0] = 'R';
			break;
		case 2210:				// binary = 01010100 
			CTxBuff[0] = 'T';
			break;
		case 2211:				// binary = 01010110 
			CTxBuff[0] = 'V';
			break;
		case 2300:				// binary = 01011000 
			CTxBuff[0] = 'X';
			break;
		case 2301:				// binary = 01011010 
			CTxBuff[0] = 'Z';
			break;
		case 2310:				// binary = 01011100 = "\" -> 00100110 = &
			CTxBuff[0] = '&';
			break;
		case 2311:				// binary = 01011110 
			CTxBuff[0] = '^';
			break;
		//Axis Z			
		case 3100:				// binary = 01101000 
			CTxBuff[0] = 'h';
			break;
		case 3101:				// binary = 01101010 
			CTxBuff[0] = 'j';
			break;
		case 3110:				// binary = 01101100 
			CTxBuff[0] = 'l';
			break;
		case 3111:				// binary = 01101110 
			CTxBuff[0] = 'n';
			break;
		case 3200:				// binary = 01110000 
			CTxBuff[0] = 'p';
			break;
		case 3201:				// binary = 01110010 
			CTxBuff[0] = 'r';
			break;
		case 3210:				// binary = 01110100 
			CTxBuff[0] = 't';
			break;
		case 3211:				// binary = 01110110 
			CTxBuff[0] = 'v';
			break;
		case 3300:				// binary = 01111000 
			CTxBuff[0] = 'x';
			break;
		case 3301:				// binary = 01111010 
			CTxBuff[0] = 'z';
			break;
		case 3310:				// binary = 01111100 
			CTxBuff[0] = '|';
			break;
		case 3311:				// binary = 01111110 
			CTxBuff[0] = '~';
			break;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
