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
uint8_t data1[12], data2[12], data3[12], data4[12], sync1, sync2;
extern uint8_t x;

uint8_t transmit_failure_flag = 0, disable_failure_flag = 0;
uint32_t  sync2_not_received = 0;

void Receive_485(int num);
void Transmit_485(int num);
void Disable_485(int num);
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim10;
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
	
	Disable_485(x);
	Transmit_485(x);
	x++;
	
	if (x >= 5)
		x = 1;

  /* USER CODE END TIM10_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
	HAL_UART_Receive(&huart2, &sync1, 1, 10); //receive first sync byte
	HAL_UART_Receive(&huart2, &sync2, 1, 10); //receive second sync byte
	
	switch(sync2)
	{
		case 0xBB:
		{
			HAL_UART_Receive(&huart2, data1, 12, 10); //receive data
			break;
		}
		case 0xCC:
		{
			HAL_UART_Receive(&huart2, data2, 12, 10); //receive data
			break;
		}
		case 0xDD:
		{
			HAL_UART_Receive(&huart2, data3, 12, 10); //receive data
			break;
		}
		case 0xEE:
		{
			HAL_UART_Receive(&huart2, data4, 12, 10); //receive data
			break;
		}
		default :
		{
			sync2_not_received++;
			break;
		}
	}
	
	if (sync2_not_received > 100000)
		sync2_not_received = 0;
	sync2 = 0x00;
	USART2->CR1 |= USART_CR1_RXNEIE;

  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void Receive_485(int num)
{
	
}

void Transmit_485(int num)//Send command to 485 number "num"
{
	uint8_t com_send[2] = {0xBB, 0xBB};
	switch(num)
	{
		case 1:
		{
			com_send[1] = 0xBB;
			HAL_GPIO_WritePin(RS_RE1_GPIO_Port, RS_RE1_Pin, GPIO_PIN_SET);//Enable driver of 1st sensor
			HAL_GPIO_WritePin(RS_TE1_GPIO_Port, RS_TE1_Pin, GPIO_PIN_SET);
			HAL_UART_Transmit(&huart2, com_send, 2, 10);
			HAL_GPIO_WritePin(RS_RE1_GPIO_Port, RS_RE1_Pin, GPIO_PIN_RESET);//Disable driver of 1st sensor
			HAL_GPIO_WritePin(RS_TE1_GPIO_Port, RS_TE1_Pin, GPIO_PIN_RESET);
			break;
		}
		case 2:
		{
			com_send[1] = 0xCC;
			HAL_GPIO_WritePin(RS_RE2_GPIO_Port, RS_RE2_Pin, GPIO_PIN_SET);//Enable driver of 2st sensor
			HAL_GPIO_WritePin(RS_TE2_GPIO_Port, RS_TE2_Pin, GPIO_PIN_SET);
			HAL_UART_Transmit(&huart2, com_send, 2, 10);
			HAL_GPIO_WritePin(RS_RE2_GPIO_Port, RS_RE2_Pin, GPIO_PIN_RESET);//Disable driver of 2st sensor
			HAL_GPIO_WritePin(RS_TE2_GPIO_Port, RS_TE2_Pin, GPIO_PIN_RESET);
			break;
		}
		case 3:
		{
			com_send[1] = 0xDD;
			HAL_GPIO_WritePin(RS_RE3_GPIO_Port, RS_RE3_Pin, GPIO_PIN_SET);//Enable driver of 2st sensor
			HAL_GPIO_WritePin(RS_TE3_GPIO_Port, RS_TE3_Pin, GPIO_PIN_SET);
			HAL_UART_Transmit(&huart2, com_send, 2, 10);
			HAL_GPIO_WritePin(RS_RE3_GPIO_Port, RS_RE3_Pin, GPIO_PIN_RESET);//Disable driver of 2st sensor
			HAL_GPIO_WritePin(RS_TE3_GPIO_Port, RS_TE3_Pin, GPIO_PIN_RESET);
			break;
		}
		case 4:
		{
			com_send[1] = 0xEE;
			HAL_GPIO_WritePin(RS_RE4_GPIO_Port, RS_RE4_Pin, GPIO_PIN_SET);//Enable driver of 2st sensor
			HAL_GPIO_WritePin(RS_TE4_GPIO_Port, RS_TE4_Pin, GPIO_PIN_SET);
			HAL_UART_Transmit(&huart2, com_send, 2, 10);
			HAL_GPIO_WritePin(RS_RE4_GPIO_Port, RS_RE4_Pin, GPIO_PIN_RESET);//Disable driver of 2st sensor
			HAL_GPIO_WritePin(RS_TE4_GPIO_Port, RS_TE4_Pin, GPIO_PIN_RESET);
			break;
		}
		default:
		{
			transmit_failure_flag = 1;
			break;
		}

	}
}

void Disable_485(int num) //Disables all 485 transceivers except for number "num" 
{
	switch(num)
	{
		case 1:
		{
//			HAL_GPIO_WritePin(RS_RE1_GPIO_Port, RS_RE1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RS_RE2_GPIO_Port, RS_RE2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RS_RE2_GPIO_Port, RS_TE2_Pin, GPIO_PIN_RESET);
			
			HAL_GPIO_WritePin(RS_RE3_GPIO_Port, RS_RE3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RS_RE3_GPIO_Port, RS_TE3_Pin, GPIO_PIN_RESET);
			
			HAL_GPIO_WritePin(RS_RE4_GPIO_Port, RS_RE4_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RS_RE4_GPIO_Port, RS_TE4_Pin, GPIO_PIN_RESET);
			break;
		}
		case 2:
		{
			HAL_GPIO_WritePin(RS_RE1_GPIO_Port, RS_RE1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RS_RE2_GPIO_Port, RS_TE1_Pin, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(RS_RE2_GPIO_Port, RS_RE2_Pin, GPIO_PIN_RESET);
			
			HAL_GPIO_WritePin(RS_RE3_GPIO_Port, RS_RE3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RS_RE3_GPIO_Port, RS_TE3_Pin, GPIO_PIN_RESET);
			
			HAL_GPIO_WritePin(RS_RE4_GPIO_Port, RS_RE4_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RS_RE4_GPIO_Port, RS_TE4_Pin, GPIO_PIN_RESET);
			break;
		}
		case 3:
		{
			HAL_GPIO_WritePin(RS_RE1_GPIO_Port, RS_RE1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RS_RE2_GPIO_Port, RS_TE1_Pin, GPIO_PIN_RESET);
			
			HAL_GPIO_WritePin(RS_RE2_GPIO_Port, RS_RE2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RS_RE3_GPIO_Port, RS_TE2_Pin, GPIO_PIN_RESET);
			
//			HAL_GPIO_WritePin(RS_RE3_GPIO_Port, RS_RE3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(RS_RE4_GPIO_Port, RS_RE4_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RS_RE4_GPIO_Port, RS_RE4_Pin, GPIO_PIN_RESET);
			break;
		}
		case 4:
		{
			HAL_GPIO_WritePin(RS_RE1_GPIO_Port, RS_RE1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RS_RE2_GPIO_Port, RS_TE1_Pin, GPIO_PIN_RESET);
			
			HAL_GPIO_WritePin(RS_RE2_GPIO_Port, RS_RE2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RS_RE3_GPIO_Port, RS_TE2_Pin, GPIO_PIN_RESET);
			
			HAL_GPIO_WritePin(RS_RE3_GPIO_Port, RS_RE3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RS_RE4_GPIO_Port, RS_TE3_Pin, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(RS_RE4_GPIO_Port, RS_RE4_Pin, GPIO_PIN_RESET);
			
			
			
			
			break;
		}
		default: 
		{
			disable_failure_flag = 1;
			break;
		}
	}
}
	
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
