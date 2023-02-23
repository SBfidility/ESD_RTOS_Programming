/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Definitions for the event bits in the event group. */
#define mainFIRST_TASK_BIT	( 1UL << 0UL ) /* Event bit 0, which is set by the first task. */
#define mainSECOND_TASK_BIT	( 1UL << 1UL ) /* Event bit 1, which is set by the second task. */
#define mainTHIRD_TASK_BIT	( 1UL << 2UL ) /* Event bit 2, which is set by the third task. */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Use by the pseudo random number generator. */
static uint32_t ulNextRand;

/* Declare the event group used to synchronize the three tasks. */
EventGroupHandle_t xEventGroup;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
/* Pseudo random number generation functions - implemented in this file as the
MSVC rand() function has unexpected consequences. */
static uint32_t prvRand( void );
static void prvSRand( uint32_t ulSeed );

/* Three instances of this task are created. */
static void vSyncingTask( void *pvParameters );

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void vPrintString(const char *txt)
{
	taskENTER_CRITICAL();
	HAL_UART_Transmit(&huart2, (uint8_t*)txt, strlen(txt), HAL_MAX_DELAY);
	taskEXIT_CRITICAL();
}

void vPrintStringAndNumber( const char *pcString, uint32_t ulValue )
{
	char as8printval[50];
	/* Print the string, using a critical section as a crude method of mutual
	exclusion. */
	taskENTER_CRITICAL();
	{
		sprintf(as8printval,  "%s %lu\r\n", pcString, ulValue );
		HAL_UART_Transmit(&huart2,(uint8_t *)as8printval,strlen(as8printval),HAL_MAX_DELAY);
	}
	taskEXIT_CRITICAL();
}

void vPrintTwoStrings( const char *pcString1, const char *pcString2 )
{
	char as8printval[50];

	/* Print the string, using a critical section as a crude method of mutual
	exclusion. */
	vTaskSuspendAll();
	{
		sprintf(as8printval, "At time %lu: %s %s\r\n", xTaskGetTickCount(), pcString1, pcString2 );
		HAL_UART_Transmit(&huart2,(uint8_t *)as8printval,strlen(as8printval),HAL_MAX_DELAY);
	}
	xTaskResumeAll();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	/* The tasks created in this example block for a random time.  The block
	time is generated using rand() - seed the random number generator. */
	prvSRand( ( uint32_t ) time( NULL ) );

	/* Before an event group can be used it must first be created. */
	xEventGroup = xEventGroupCreate();

	/* Create three instances of the task.  Each task is given a different name,
	which is later printed out to give a visual indication of which task is
	executing.  The event bit to use when the task reaches its synchronization
	point is passed into the task using the task parameter. */
	xTaskCreate( vSyncingTask, "Task 1", 1000, ( void * ) mainFIRST_TASK_BIT, 1, NULL );
	xTaskCreate( vSyncingTask, "Task 2", 1000, ( void * ) mainSECOND_TASK_BIT, 1, NULL );
	xTaskCreate( vSyncingTask, "Task 3", 1000, ( void * ) mainTHIRD_TASK_BIT, 1, NULL );

	/* Start the scheduler so the created tasks start executing. */
	vTaskStartScheduler();

	/* The following line should never be reached because vTaskStartScheduler()
	will only return if there was not enough FreeRTOS heap memory available to
	create the Idle and (if configured) Timer tasks.  Heap management, and
	techniques for trapping heap exhaustion, are described in the book text. */
	for( ;; );
	return 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void vSyncingTask( void *pvParameters )
{
	const EventBits_t uxAllSyncBits = ( mainFIRST_TASK_BIT | mainSECOND_TASK_BIT | mainTHIRD_TASK_BIT );
	const TickType_t xMaxDelay = pdMS_TO_TICKS( 4000UL );
	const TickType_t xMinDelay = pdMS_TO_TICKS( 200UL );
	TickType_t xDelayTime;
	EventBits_t uxThisTasksSyncBit;

	/* Three instances of this task are created - each task uses a different
	event bit in the synchronization.  The event bit to use by this task
	instance is passed into the task using the task's parameter.  Store it in
	the uxThisTasksSyncBit variable. */
	uxThisTasksSyncBit = ( EventBits_t ) pvParameters;

	for( ;; )
	{
		/* Simulate this task taking some time to perform an action by delaying
		for a pseudo random time.  This prevents all three instances of this
		task from reaching the synchronization point at the same time, and
		allows the example's behavior to be observed more easily. */
		xDelayTime = ( prvRand() % xMaxDelay ) + xMinDelay;
		vTaskDelay( xDelayTime );

		/* Print out a message to show this task has reached its synchronization
		point.  pcTaskGetTaskName() is an API function that returns the name
		assigned to the task when the task was created. */
		vPrintTwoStrings( pcTaskGetTaskName( NULL ), "reached sync point" );

		/* Wait for all the tasks to have reached their respective
		synchronization points. */
		xEventGroupSync( /* The event group used to synchronize. */
						 xEventGroup,

						 /* The bit set by this task to indicate it has reached
						 the synchronization point. */
						 uxThisTasksSyncBit,

						 /* The bits to wait for, one bit for each task taking
						 part in the synchronization. */
						 uxAllSyncBits,

						 /* Wait indefinitely for all three tasks to reach the
						 synchronization point. */
						 portMAX_DELAY );

		/* Print out a message to show this task has passed its synchronization
		point.  As an indefinite delay was used the following line will only be
		reached after all the tasks reached their respective synchronization
		points. */
		vPrintTwoStrings( pcTaskGetTaskName( NULL ), "exited sync point" );
	}
}
/*-----------------------------------------------------------*/

static uint32_t prvRand( void )
{
const uint32_t ulMultiplier = 0x015a4e35UL, ulIncrement = 1UL;
uint32_t ulReturn;

	/* Utility function to generate a pseudo random number as the MSVC rand()
	function has unexpected consequences. */
	taskENTER_CRITICAL();
		ulNextRand = ( ulMultiplier * ulNextRand ) + ulIncrement;
		ulReturn = ( ulNextRand >> 16UL ) & 0x7fffUL;
	taskEXIT_CRITICAL();
	return ulReturn;
}
/*-----------------------------------------------------------*/

static void prvSRand( uint32_t ulSeed )
{
	/* Utility function to seed the pseudo random number generator. */
	ulNextRand = ulSeed;
}
/*-----------------------------------------------------------*/


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
