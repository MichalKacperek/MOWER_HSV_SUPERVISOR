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
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../mower_lib/Inc/mower.h"
#include "../../mower_lib/Src/mower.c"
#include <string.h>

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
uint8_t CAN_TxData[8] = {0};
uint8_t CAN_RxData[8] = {0};
FDCAN_TxHeaderTypeDef my_pTxHeader;
FDCAN_TxHeaderTypeDef my_pTxHeader101;
FDCAN_RxHeaderTypeDef my_pRxHeader;
uint8_t Rx_message[32];
uint8_t Tx_message[32];
int Rx_message_count = 0;
int Rx_message_timeout = 8;
uint16_t sw_set_position = 0;
uint16_t sw_current_position = 0;
uint16_t sw_timeout=0; /* if timeout is greater then 1000 there is some problem with device or communication
						if timeout is greater then 500 remote frame for current position should be send */
uint8_t sw_position_data_valid=0;
ANGLE_ACTUATOR steering_wheel={0x02};
ENGINE engine = {0x01};
MOWER mower = {&steering_wheel};



#define SRAM3_Dest_Addr	((uint32_t)0x30040000)
uint32_t *Ptr_Dest = (uint32_t *)SRAM3_Dest_Addr;
char* src = (char*)(SRAM3_Dest_Addr)+4;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */
	__HAL_RCC_D2SRAM3_CLK_ENABLE();


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  my_pTxHeader.Identifier = 0x301;
   my_pTxHeader.IdType = FDCAN_STANDARD_ID;
   my_pTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
   my_pTxHeader.DataLength = FDCAN_DLC_BYTES_2;
   my_pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
   my_pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
   my_pTxHeader.MessageMarker = 0;
   my_pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
   my_pTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
   my_pTxHeader101.Identifier = 0x101;
   my_pTxHeader101.IdType = FDCAN_STANDARD_ID;
   my_pTxHeader101.BitRateSwitch = FDCAN_BRS_OFF;
   my_pTxHeader101.DataLength = FDCAN_DLC_BYTES_2;
   my_pTxHeader101.TxFrameType = FDCAN_DATA_FRAME;
   my_pTxHeader101.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    my_pTxHeader101.MessageMarker = 0;
    my_pTxHeader101.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    my_pTxHeader101.FDFormat = FDCAN_CLASSIC_CAN;

   FDCAN_FilterTypeDef FDCAN_Filter_Engine;
   FDCAN_Filter_Engine.IdType = FDCAN_STANDARD_ID;
   FDCAN_Filter_Engine.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
   FDCAN_Filter_Engine.FilterID1 = 0x100 << 5;
   FDCAN_Filter_Engine.FilterID2 = 0x109 << 5;
   FDCAN_Filter_Engine.FilterIndex = 5;
   FDCAN_Filter_Engine.FilterType = FDCAN_FILTER_RANGE;

   HAL_FDCAN_Start(&hfdcan1);

   HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_Filter_Engine);
   HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(1));

   HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
   __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);  // enable receive intterupts

   //int engine_struct_size = 6;//sizeof(engine);

   char* temp;
   char* dest;
   char* src = dest+4;
   HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(2));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  CAN_TxData[0] = (uint8_t) (sw_set_position >> 8);
	  CAN_TxData[1] = (uint8_t) sw_set_position;

	  HAL_FDCAN_AddMessageToTxBuffer(&hfdcan1, &my_pTxHeader, CAN_TxData, FDCAN_TX_BUFFER0);
	  HAL_FDCAN_EnableTxBufferRequest(&hfdcan1, FDCAN_TX_BUFFER0);
	  HAL_Delay(200);
	  SendAngleActuatorData(mower.steering_wheel, &huart1);

	  temp = (char*) &engine;
	  dest = (char*) SRAM3_Dest_Addr;

	  HAL_HSEM_Take(1, 2);
	  //for (int i=0; i< engine_struct_size; i++)
	  //{
	  dest[0]=engine.set_state.all_byte_data;
	  engine.set_state.set_state_bits.reset_error = 0;
		  //memcpy((char*)Ptr_Dest, (char*)&engine, engine_struct_size);
	  //}
	  HAL_HSEM_Release(1, 2);
	  if (HAL_HSEM_IsSemTaken(1))
	  {
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	  }
	  else HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN|RCC_PERIPHCLK_USART1;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 30;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 6;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	static int Rx_counter = 0;
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hfdcan);
  UNUSED(RxFifo0ITs);
  if (RxFifo0ITs == FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
  {
	  HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &my_pRxHeader, CAN_RxData);
	  Rx_counter++;
	  if (my_pRxHeader.Identifier == 0x302)
	  {
		  sw_timeout = 0;
		  mower.steering_wheel->current_position = (uint16_t)(CAN_RxData[0]<<8 | CAN_RxData[1]);
		  //sw_current_position = (uint16_t)(CAN_RxData[0]<<8 | CAN_RxData[1]);
		  sw_position_data_valid += CAN_RxData[2];
		  if (sw_position_data_valid > 5 )
		  {
			  // if sw_position_data_valid > 5 it should be checked if encoder is wired properly and user has to be informed
		  }
	  }
	  else if (my_pRxHeader.Identifier == 0x303)
	  {
		  HAL_FDCAN_AddMessageToTxBuffer(&hfdcan1, &my_pTxHeader101, CAN_TxData, FDCAN_TX_BUFFER1);
		  HAL_FDCAN_EnableTxBufferRequest(&hfdcan1, FDCAN_TX_BUFFER1);
	  }
	  /*
	  if (RxData[0] == 0x01)
	  {
		  //HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, RxData[1]);
	  }
	  else if (RxData[0] == 0x02)
	  {
	  	  //HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, RxData[1]);
	  }
	  */
  }

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_FDCAN_RxFifo0Callback could be implemented in the user file
   */
}


void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	static int Rx_counter = 0;
	static uint8_t sem_flag = 0;
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hfdcan);
  UNUSED(RxFifo1ITs);
  if (RxFifo1ITs == FDCAN_IT_RX_FIFO1_NEW_MESSAGE)
  {
	  HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &my_pRxHeader, CAN_RxData);
	  Rx_counter++;
	  if (my_pRxHeader.Identifier == 0x101)
	  {
		  sem_flag ^= 1;
		  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, sem_flag);
		  if (sem_flag)
		  {
			  //HAL_HSEM_Take(1, 1);
		  }
		  else
		  {
			 // HAL_HSEM_Release(1, 1);
		  }



		  //sw_timeout = 0;
		  //mower.steering_wheel->current_position = (uint16_t)(CAN_RxData[0]<<8 | CAN_RxData[1]);
		  //sw_current_position = (uint16_t)(CAN_RxData[0]<<8 | CAN_RxData[1]);
		  //sw_position_data_valid += CAN_RxData[2];
		  //if (sw_position_data_valid > 5 )
		  //{
			  // if sw_position_data_valid > 5 it should be checked if encoder is wired properly and user has to be informed
		  //}
	  }

  }

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_FDCAN_RxFifo0Callback could be implemented in the user file
   */
}


void HAL_FDCAN_RxBufferNewMessageCallback(FDCAN_HandleTypeDef *hfdcan)
{
  /* Prevent unused argument(s) compilation warning */
	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_BUFFER0, &my_pRxHeader, CAN_RxData);

  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_FDCAN_RxBufferNewMessageCallback could be implemented in the user file
   */
}





//Calculates CRC16 of nBytes of data in byte array message
unsigned int crc16(uint8_t packet[], int nBytes)
{
	unsigned int crc = 0;
	int byte;
	unsigned char bit;
	for (byte=0; byte < nBytes; byte++) {
		crc = crc ^ ((unsigned int)packet[byte] << 8);

		for (bit=0; bit < 8; bit++) {
			if ((crc & 0x8000))
			{
				crc = (crc << 1) ^ 0x1021;
			}
			else
			{
				crc = crc << 1;
			}
		}
	}
	return  crc;
}

void HAL_HSEM_FreeCallback(uint32_t SemMask)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(SemMask);

  if (__HAL_HSEM_SEMID_TO_MASK(2) == SemMask)
  {
	  char* temp = (char*) &engine.rpm;
	  //for (int i=0; i< engine_struct_size; i++)
	  //{
	  temp[0] = src[0];
	  temp[1] = src[1];
	  temp[2] = src[2];
	  temp[3] = src[3];

	  //engine.set_state.all_byte_data = (uint8_t) src[0];
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
