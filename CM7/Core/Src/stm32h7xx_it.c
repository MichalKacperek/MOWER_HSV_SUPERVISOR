/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern FDCAN_HandleTypeDef hfdcan1;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
extern uint8_t Rx_message[32];
extern uint8_t Tx_message[32];
extern int Rx_message_count;
extern int Rx_message_timeout;
extern uint16_t sw_set_position;
extern uint16_t sw_current_position;
extern ENGINE engine;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
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
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
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
	  unsigned int crc;
	  char * temp;
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  if (Rx_message_timeout > 0){
  	  Rx_message_timeout--;
  	  if (Rx_message_timeout == 0){
  		  if (Rx_message[0] == 0x01){
  			  crc = crc16(Rx_message, Rx_message_count - 2);
  			  temp = (char *)&crc;

  			  if ((temp[1] == Rx_message[Rx_message_count-2]) && (temp[0] == Rx_message[Rx_message_count-1])){
  				  /* Stop engine */
  				  if (Rx_message[1] == 0x01)
  				  {
  					  engine.set_state.set_state_bits.reset_error = 0;
  					  engine.set_state.set_state_bits.engine_running_enabled = 0;
  				  }
  				  else if (Rx_message[1] == 0x02)
  				  {
  					  engine.set_state.set_state_bits.engine_running_enabled = 1;
  				  }
  				  else if (Rx_message[1] == 0x03)
  				  {
  					  engine.set_state.set_state_bits.engine_running_enabled = 0;
  					  engine.set_state.set_state_bits.reset_error = 1;
  				  }
  			  }
  		  }
  		  else if (Rx_message[0] == 0x02){
  			  crc = crc16(Rx_message, Rx_message_count - 2);
  			  temp = (char *)&crc;

  			  if ((temp[1] == Rx_message[Rx_message_count-2]) && (temp[0] == Rx_message[Rx_message_count-1])){
  				  if (Rx_message[1] == 0x01)
  				  {	//go to the given position
  					  sw_set_position = Rx_message[2]<<8 | Rx_message[3];
  				  }
  			  }
  		  }
  		  Rx_message_count=0;
  	  }
    }
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */

  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 1.
  */
void FDCAN1_IT1_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT1_IRQn 0 */

  /* USER CODE END FDCAN1_IT1_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT1_IRQn 1 */

  /* USER CODE END FDCAN1_IT1_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	  if (__HAL_UART_GET_IT(&huart1, UART_IT_RXNE))
	  {

		  HAL_UART_Receive(&huart1, (uint8_t*)&Rx_message[Rx_message_count], 1, 2);
		  Rx_message_count++;
		  Rx_message_timeout = 8;
	  }
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles CM4 send event interrupt for CM7.
  */
void CM4_SEV_IRQHandler(void)
{
  /* USER CODE BEGIN CM4_SEV_IRQn 0 */

  /* USER CODE END CM4_SEV_IRQn 0 */
  /* USER CODE BEGIN CM4_SEV_IRQn 1 */

  /* USER CODE END CM4_SEV_IRQn 1 */
}

/**
  * @brief This function handles HSEM1 global interrupt.
  */
void HSEM1_IRQHandler(void)
{
  /* USER CODE BEGIN HSEM1_IRQn 0 */

  /* USER CODE END HSEM1_IRQn 0 */
  HAL_HSEM_IRQHandler();
  /* USER CODE BEGIN HSEM1_IRQn 1 */

  /* USER CODE END HSEM1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
