/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "units.h"
#include "../../mower_lib/Inc/units.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
ENGINE engine={0x01};

#define SRAM3_Dest_Addr	((uint32_t)0x30040000)
//uint32_t *Ptr_Dest = (uint32_t *)SRAM3_Dest_Addr;


//char* temp;
char* src;
int engine_struct_size=0;
uint32_t sem1_mask = __HAL_HSEM_SEMID_TO_MASK(1);
//int engine_rpm = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef EngineHandler(ENGINE* dev);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */
  __HAL_RCC_D2SRAM3_CLK_ENABLE();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */



  //HAL_GPIO_WritePin(Ignition_relay_GPIO_Port, Ignition_relay_Pin, GPIO_PIN_SET);
 // HAL_GPIO_WritePin(Fuel_valve_GPIO_Port, Fuel_valve_Pin, GPIO_PIN_SET);
  //HAL_GPIO_WritePin(Starter_relay_GPIO_Port, Starter_relay_Pin, GPIO_PIN_SET);


  //HAL_Delay(2000);

  //HAL_GPIO_WritePin(Ignition_relay_GPIO_Port, Ignition_relay_Pin, GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(Fuel_valve_GPIO_Port, Fuel_valve_Pin, GPIO_PIN_RESET);
 // HAL_GPIO_WritePin(Starter_relay_GPIO_Port, Starter_relay_Pin, GPIO_PIN_RESET);



  HAL_TIM_Base_Start_IT(&htim13);
  HAL_TIM_IC_Start_IT(&htim13, TIM_CHANNEL_1);
  __HAL_TIM_ENABLE_IT(&htim13,TIM_IT_CC1);
  __HAL_TIM_ENABLE_IT(&htim13,TIM_IT_UPDATE);
  src = (char *) SRAM3_Dest_Addr;
  char* dest = (char *) (SRAM3_Dest_Addr+4);
  engine_struct_size = 6; //sizeof(engine);

  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(1));

  engine.pins.ignition.GPIO_Port = Ignition_relay_GPIO_Port;
  engine.pins.ignition.Pin = Ignition_relay_Pin;
  engine.pins.fuel_valve.GPIO_Port = Fuel_valve_GPIO_Port;
  engine.pins.fuel_valve.Pin = Fuel_valve_Pin;
  engine.pins.starter.GPIO_Port = Starter_relay_GPIO_Port;
  engine.pins.starter.Pin = Starter_relay_Pin;
  engine.state = ENGINE_INIT_STATE;
  engine.starter_timeout = 2000;
  engine.rpm_range.rpm_minimal = 200;
  engine.rpm_range.rpm_maximal = 2700;
  engine.rpm_range.rpm_overrange = 2650;
  engine.rpm_range.rpm_underrange = 2200;




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  volatile uint32_t time = HAL_GetTick();
	  switch(engine.state)
	  {
		  case ENGINE_INIT_STATE:

			  engine.starter_retries = 1;
			  engine.starter_started_at = 0;
			  if (engine.set_state.set_state_bits.engine_running_enabled == 1)
			  {
				  if (engine.rpm > engine.rpm_range.rpm_minimal)
				  {
					  engine.state = ENGINE_RUNNING_STATE;
				  }
				  else if (engine.rpm == 0)
				  {
					  engine.state = ENGINE_START_STATE;

				  }
			  }
			  else if (!engine.set_state.set_state_bits.engine_running_enabled)
			  {
				  engine.state = ENGINE_STOP_STATE;
			  }
			  break;

		  case ENGINE_START_STATE:

			  if (engine.starter_retries > 0)
			  {
				  if (engine.set_state.set_state_bits.engine_running_enabled & (engine.starter_started_at == 0))
				  {
					  __TURN_ON_IGNITION(engine);
					  engine.starter_started_at = HAL_GetTick();
					  __TURN_ON_STARTER(engine);
				  }
				  if ((engine.starter_started_at + engine.starter_timeout) < time)
				  {
					  __TURN_OFF_STARTER(engine);
					  if ((engine.starter_started_at + 2* engine.starter_timeout) < time)
					  {
						  engine.starter_retries--;
						  engine.starter_started_at = 0;
					  }
				  }
				  else if (((engine.starter_started_at + engine.starter_timeout) > time) & (engine.rpm >  engine.rpm_range.rpm_minimal))
				  {
					  __TURN_OFF_STARTER(engine);
					  engine.state = ENGINE_RUNNING_STATE;
				  }
			  }
			  else if (engine.starter_retries == 0)
			  {
				  __TURN_OFF_STARTER(engine);
				  __STOP_ENGINE(engine);
				  /* send: not able to run engine */
				  engine.state = ENGINE_ERROR_STATE;
			  }
			  if (engine.set_state.set_state_bits.engine_running_enabled == 0)
			  {
				  __TURN_OFF_STARTER(engine);
				  __STOP_ENGINE(engine);
				  engine.state = ENGINE_STOP_STATE;
			  }
			  break;

		  case ENGINE_RUNNING_STATE:
			  if (engine.set_state.set_state_bits.engine_running_enabled != 0)
			  {
				  if (engine.rpm < engine.rpm_range.rpm_minimal)
				  {
					  engine.error_id = ERROR_RUNNING_UNDER_MINIMAL_RPM;
					  engine.state = ENGINE_ERROR_STATE;
				  }
				  else if (engine.rpm > engine.rpm_range.rpm_maximal)
				  {
					  engine.error_id = ERROR_RUNNING_OVER_MAXIMAL_RPM;
					  engine.state = ENGINE_ERROR_STATE;
				  }
				  else if (engine.rpm < engine.rpm_range.rpm_underrange)
				  {
					  engine.current_state_bits.rpm_underrange = 1;
				  }
				  else if (engine.rpm > engine.rpm_range.rpm_overrange)
				  {
					  engine.current_state_bits.rpm_overrange = 1;
				  }
				  else
				  {
					  engine.current_state_bits.rpm_overrange = 0;
					  engine.current_state_bits.rpm_underrange = 0;
				  }
			  }
			  else
				  engine.state = ENGINE_STOP_STATE;
			  break;
		  case ENGINE_STOP_STATE:

			  __TURN_OFF_STARTER(engine); // just for safety
			  __STOP_ENGINE(engine);
			  if (engine.set_state.set_state_bits.engine_running_enabled == 1)
			  {
				  engine.state = ENGINE_START_STATE;
			  }
			  break;
		  case ENGINE_ERROR_STATE:

			  if (engine.set_state.set_state_bits.reset_error) engine.state = ENGINE_INIT_STATE;
			  break;
	  }
	  HAL_HSEM_Take(2, 2);

	  dest[0] = engine.rpm>>8;
	  dest[1] = engine.rpm;
	  dest[2] = ((char *) &engine.current_state_bits)[0];
	  dest[3] = engine.error_id;
	  HAL_HSEM_Release(2, 2);

	  /* Check engine speed (rpm) range */

	  /* Get current engine state */
	  //__TURN_OFF_STARTER(engine);
	  /*HAL_Delay(2000);
	  __STOP_ENGINE(engine);
	  HAL_Delay(2000);
	  __TURN_ON_IGNITION(engine);
	  HAL_Delay(2000);
	  __TURN_ON_STARTER(engine);
	  HAL_Delay(2000);*/
	  /* Compare current engine state with set values */


	  /* Write checked engine speed range flags if needed */

	  /**/




	  //HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, engine.set_state_bits.ignition_relay);

	  //HAL_GPIO_WritePin(Ignition_relay_GPIO_Port, Ignition_relay_Pin, engine.set_state_bits.ignition_relay);
	  //HAL_GPIO_WritePin(Fuel_valve_GPIO_Port, Fuel_valve_Pin, engine.set_state_bits.fuel_valve);
	  //HAL_GPIO_WritePin(Starter_relay_GPIO_Port, Starter_relay_Pin, engine.set_state_bits.starter_relay);
	  /*
	  HAL_Delay(1000);

	  HAL_GPIO_WritePin(engine.pins.ignition.GPIO_Port, engine.pins.ignition.Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(engine.pins.fuel_valve.GPIO_Port, engine.pins.fuel_valve.Pin, GPIO_PIN_SET);

	  HAL_Delay(1000);
*/








    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
HAL_StatusTypeDef EngineHandler(ENGINE* dev)
{
	//HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)
	/*
	uint16_t rpm;
	struct{
		uint8_t engine_running	:1,
				fuel_valve		:1,
				ignition_relay	:1,
				starter_relay	:1,
								:4;
	}current_state_bits;
	struct{
		uint8_t engine_running	:1,
		fuel_valve		:1,
		ignition_relay	:1,
		starter_relay	:1,
						:4;
	}set_state_bits;
	*/

	return HAL_OK;
}

void HAL_HSEM_FreeCallback(uint32_t SemMask)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(SemMask);

  if (__HAL_HSEM_SEMID_TO_MASK(1) == SemMask)
  {
	  char* temp = (char*) &engine;
	  //for (int i=0; i< engine_struct_size; i++)
	  //{
		  //temp[4] = src[4];
	  engine.set_state.all_byte_data = (uint8_t) src[0];
	  //}
	  HAL_HSEM_ActivateNotification(SemMask);
  }


  /* NOTE : This function should not be modified, when the callback is needed,
  the HAL_HSEM_FreeCallback can be implemented in the user file
    */
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
