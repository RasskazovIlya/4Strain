/**
  ******************************************************************************
  * @file    stm32l1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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
#include "stm32l1xx_hal.h"
#include "stm32l1xx.h"
#include "stm32l1xx_it.h"

/* USER CODE BEGIN 0 */
uint8_t data[14], data1[12], data2[12], data3[12], data4[12], sync1, sync2;
extern uint8_t x;

uint32_t  sync2_not_received = 0;

void Transmit_485(int num);
void Disable_485(void);
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim10;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles RCC global interrupt.
*/
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */

  /* USER CODE END RCC_IRQn 1 */
}

/**
* @brief This function handles TIM10 global interrupt.
*/
void TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM10_IRQn 0 */
	
  /* USER CODE END TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim10);
  /* USER CODE BEGIN TIM10_IRQn 1 */
	
	if (x >= 5)
		x = 1;
		
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
	Disable_485();
	Transmit_485(x);
	x++;

  /* USER CODE END TIM10_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

	USART2->CR1 |= USART_CR1_RXNEIE;
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void Transmit_485(int num)//Send command to 485 number "num"
{
	uint8_t com_send[3] = {0x55, 0xAA, 0xBB};
	switch(num)
	{
		case 1:
		{
			com_send[2] = 0xBB;
			HAL_GPIO_WritePin(RS_RE1_GPIO_Port, RS_RE1_Pin, GPIO_PIN_SET);//Enable driver of 1st RS
			HAL_GPIO_WritePin(RS_TE1_GPIO_Port, RS_TE1_Pin, GPIO_PIN_SET);//Disable receiver of 1st RS
			
			HAL_UART_Transmit(&huart2, com_send, 3, 10);//send command 0x55AABB
			
			HAL_GPIO_WritePin(RS_RE1_GPIO_Port, RS_RE1_Pin, GPIO_PIN_RESET);//Disable driver of 1st RS
			HAL_GPIO_WritePin(RS_TE1_GPIO_Port, RS_TE1_Pin, GPIO_PIN_RESET);//Enable receiver of 1st RS
			
			break;
		}
		case 2:
		{
			com_send[2] = 0xCC;
			HAL_GPIO_WritePin(RS_RE2_GPIO_Port, RS_RE2_Pin, GPIO_PIN_SET);//Enable driver of 2nd RS
			HAL_GPIO_WritePin(RS_TE2_GPIO_Port, RS_TE2_Pin, GPIO_PIN_SET);//Disable receiver of 2nd RS
			
			HAL_UART_Transmit(&huart2, com_send, 3, 10);//send command 0x55AACC
			
			HAL_GPIO_WritePin(RS_RE2_GPIO_Port, RS_RE2_Pin, GPIO_PIN_RESET);//Disable driver of 2nd RS
			HAL_GPIO_WritePin(RS_TE2_GPIO_Port, RS_TE2_Pin, GPIO_PIN_RESET);//Enable receiver of 2nd RS
			
			break;
		}
		case 3:
		{
			com_send[2] = 0xDD;
			HAL_GPIO_WritePin(RS_RE3_GPIO_Port, RS_RE3_Pin, GPIO_PIN_SET);//Enable driver of 3rd RS
			HAL_GPIO_WritePin(RS_TE3_GPIO_Port, RS_TE3_Pin, GPIO_PIN_SET);//Disable receiver of 3rd RS
			
			HAL_UART_Transmit(&huart2, com_send, 3, 10);//send command 0x55AADD
			
			HAL_GPIO_WritePin(RS_RE3_GPIO_Port, RS_RE3_Pin, GPIO_PIN_RESET);//Disable driver of 3rd RS
			HAL_GPIO_WritePin(RS_TE3_GPIO_Port, RS_TE3_Pin, GPIO_PIN_RESET);//Enable receiver of 3rd RS
			
			break;
		}
		case 4:
		{
			com_send[2] = 0xEE;
			HAL_GPIO_WritePin(RS_RE4_GPIO_Port, RS_RE4_Pin, GPIO_PIN_SET);//Enable driver of 4th RS
			HAL_GPIO_WritePin(RS_TE4_GPIO_Port, RS_TE4_Pin, GPIO_PIN_SET);//Disable receiver of 4th RS
			
			HAL_UART_Transmit(&huart2, com_send, 3, 10);//send command 0x55AAEE
			
			HAL_GPIO_WritePin(RS_RE4_GPIO_Port, RS_RE4_Pin, GPIO_PIN_RESET);//Disable driver of 4th RS
			HAL_GPIO_WritePin(RS_TE4_GPIO_Port, RS_TE4_Pin, GPIO_PIN_RESET);//Enable receiver of 4th RS
			
			break;
		}
		default:
		{
			
			break;
		}

	}
}

void Disable_485(void) //Disables all 485 transceivers
{
	HAL_GPIO_WritePin(RS_RE1_GPIO_Port, RS_RE1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RS_TE1_GPIO_Port, RS_TE1_Pin, GPIO_PIN_RESET);
	
	HAL_GPIO_WritePin(RS_RE2_GPIO_Port, RS_RE2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RS_TE2_GPIO_Port, RS_TE2_Pin, GPIO_PIN_RESET);
	
	HAL_GPIO_WritePin(RS_RE3_GPIO_Port, RS_RE3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RS_TE3_GPIO_Port, RS_TE3_Pin, GPIO_PIN_RESET);
	
	HAL_GPIO_WritePin(RS_RE4_GPIO_Port, RS_RE4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RS_TE4_GPIO_Port, RS_TE4_Pin, GPIO_PIN_RESET);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive(&huart2, data, 14, 10);
	
	switch(data[1])//switch for second sync-byte to match number of RS485 and number of sensor
	{
		case 0xBB://data received from 1st sensor
		{
			for (int i = 0; i < 12; i++)
				data1[i] = data[i+2];
			HAL_GPIO_WritePin(RS_RE1_GPIO_Port, RS_RE1_Pin, GPIO_PIN_SET);//Disable receiver of 1st RS
			HAL_UART_Transmit(&huart1, data, 14, 10);//send data from 1st sensor to RPi
			break;
		}
		case 0xCC://data received from 2nd sensor
		{
			for (int i = 0; i < 12; i++)
				data2[i] = data[i+2];
			HAL_GPIO_WritePin(RS_RE2_GPIO_Port, RS_RE2_Pin, GPIO_PIN_SET);//Disable receiver of 2nd RS
			HAL_UART_Transmit(&huart1, data, 14, 10);//send data from 2nd sensor to RPi
			break;
		}
		case 0xDD://data received from 3rd sensor
		{
			for (int i = 0; i < 12; i++)
				data3[i] = data[i+2];
			HAL_GPIO_WritePin(RS_RE3_GPIO_Port, RS_RE3_Pin, GPIO_PIN_SET);//Disable receiver of 3rd RS
			HAL_UART_Transmit(&huart1, data, 14, 10);//send data from 3nd sensor to RPi
			break;
		}
		case 0xEE://data received from 4th sensor
		{
			for (int i = 0; i < 12; i++)
				data4[i] = data[i+2];
			HAL_GPIO_WritePin(RS_RE4_GPIO_Port, RS_RE4_Pin, GPIO_PIN_SET);//Disable receiver of 4th RS
			HAL_UART_Transmit(&huart1, data, 14, 10);//send data from 4nd sensor to RPi
			break;
		}
		default:
		{
			sync2_not_received++;
			break;
		}
	}
}
	
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
