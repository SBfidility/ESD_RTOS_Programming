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
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "timers.h" /* For the xTimerPendFunctionCallFromISR() function. */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Definitions for the event bits in the event group. */
#define mainFIRST_TASK_BIT	( 1UL << 0UL ) /* Event bit 0, which is set by a task. */
#define mainSECOND_TASK_BIT	( 1UL << 1UL ) /* Event bit 1, which is set by a task. */
#define mainISR_BIT			( 1UL << 2UL ) /* Event bit 2, which is set by an ISR. */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Declare the event group in which bits are set from both a task and an ISR. */
EventGroupHandle_t xEventGroup;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */
/* The tasks to be created. */
static void vIntegerGenerator( void *pvParameters );
static void vEventBitSettingTask( void *pvParameters );
static void vEventBitReadingTask( void *pvParameters );

/* A function that can be deferred to run in the RTOS daemon task.  The function
prints out the string passed to it using the pvParameter1 parameter. */
void vPrintStringFromDaemonTask( void *pvParameter1, uint32_t ulParameter2 );

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
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	/* Before an event group can be used it must first be created. */
	xEventGroup = xEventGroupCreate();

	/* Create the task that sets event bits in the event group. */
	xTaskCreate( vEventBitSettingTask, "BitSetter", 1000, NULL, 1, NULL );

	/* Create the task that waits for event bits to get set in the event
	group. */
	xTaskCreate( vEventBitReadingTask, "BitReader", 1000, NULL, 2, NULL );

	/* Create the task that is used to periodically generate a software
	interrupt. */
	xTaskCreate( vIntegerGenerator, "IntGen", 1000, NULL, 3, NULL );

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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 1000-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 16000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
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
static void vEventBitSettingTask( void *pvParameters )
{
	const TickType_t xDelay200ms = pdMS_TO_TICKS( 200UL );

	for( ;; )
	{
		/* Delay for a short while before starting the next loop. */
		vTaskDelay( xDelay200ms );

		/* Print out a message to say event bit 0 is about to be set by the
		task, then set event bit 0. */
		vPrintString( "Bit setting task -\t about to set bit 0.\r\n" );
		xEventGroupSetBits( xEventGroup, mainFIRST_TASK_BIT );

		/* Delay for a short while before setting the other bit set within this
		task. */
		vTaskDelay( xDelay200ms );

		/* Print out a message to say event bit 1 is about to be set by the
		task, then set event bit 1. */
		vPrintString( "Bit setting task -\t about to set bit 1.\r\n" );
		xEventGroupSetBits( xEventGroup, mainSECOND_TASK_BIT );
	}
}

static void vEventBitReadingTask( void *pvParameters )
{
	const EventBits_t xBitsToWaitFor = ( mainFIRST_TASK_BIT | mainSECOND_TASK_BIT | mainISR_BIT );
	EventBits_t xEventGroupValue;

	for( ;; )
	{
		/* Block to wait for event bits to become set within the event group. */
		xEventGroupValue = xEventGroupWaitBits( /* The event group to read. */
												xEventGroup,

												/* Bits to test. */
												xBitsToWaitFor,

												/* Clear bits on exit if the
												unblock condition is met. */
												pdTRUE,

												/* Don't wait for all bits. */
												pdFALSE,

												/* Don't time out. */
												portMAX_DELAY );

		/* Print a message for each bit that was set. */
		if( ( xEventGroupValue & mainFIRST_TASK_BIT ) != 0 )
		{
			vPrintString( "Bit reading task -\t event bit 0 was set\r\n" );
		}

		if( ( xEventGroupValue & mainSECOND_TASK_BIT ) != 0 )
		{
			vPrintString( "Bit reading task -\t event bit 1 was set\r\n" );
		}

		if( ( xEventGroupValue & mainISR_BIT ) != 0 )
		{
			vPrintString( "Bit reading task -\t event bit 2 was set\r\n" );
		}

		vPrintString( "\r\n" );
	}
}
/*-----------------------------------------------------------*/

void vPrintStringFromDaemonTask( void *pvParameter1, uint32_t ulParameter2 )
{
	/* The string to print is passed into this function using the pvParameter1
	parameter. */
	vPrintString( ( const char * ) pvParameter1 );
}
/*-----------------------------------------------------------*/

static void vIntegerGenerator( void *pvParameters )
{
	TickType_t xLastExecutionTime;
	const TickType_t xDelay500ms = pdMS_TO_TICKS( 500UL );

	/* Initialize the variable used by the call to vTaskDelayUntil(). */
	xLastExecutionTime = xTaskGetTickCount();

	for( ;; )
	{
		/* This is a periodic task.  Block until it is time to run again.
		The task will execute every 500ms. */
		vTaskDelayUntil( &xLastExecutionTime, xDelay500ms );

		HAL_TIM_Base_Start_IT(&htim7);
	}
}

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
  if(htim == &htim7)
  {
		BaseType_t xHigherPriorityTaskWoken;
		/* The string is not printed within the interrupt service, but is instead
		sent to the RTOS daemon task for printing.  It is therefore declared static to
		ensure the compiler does not allocate the string on the stack of the ISR (as the
		ISR's stack frame will not exist when the string is printed from the daemon
		task. */
		static const char *pcString = "Bit setting ISR -\t about to set bit 2.\r\n";

			/* As always, xHigherPriorityTaskWoken is initialized to pdFALSE. */
			xHigherPriorityTaskWoken = pdFALSE;

			/* Print out a message to say bit 2 is about to be set.  Messages cannot be
			printed from an ISR, so defer the actual output to the RTOS daemon task by
			pending a function call to run in the context of the RTOS daemon task. */
			xTimerPendFunctionCallFromISR( vPrintStringFromDaemonTask, ( void * ) pcString, 0, &xHigherPriorityTaskWoken );

			/* Set bit 2 in the event group. */
			xEventGroupSetBitsFromISR( xEventGroup, mainISR_BIT, &xHigherPriorityTaskWoken );

			/* xEventGroupSetBitsFromISR() writes to the timer command queue.  If
			writing to the timer command queue results in the RTOS daemon task leaving
			the Blocked state, and if the priority of the RTOS daemon task is higher
			than the priority of the currently executing task (the task this interrupt
			interrupted) then xHigherPriorityTaskWoken will have been set to pdTRUE
			inside xEventGroupSetBitsFromISR().

			xHigherPriorityTaskWoken is used as the parameter to portYIELD_FROM_ISR().
			If xHigherPriorityTaskWoken equals pdTRUE then calling portYIELD_FROM_ISR()
		    will request a context switch.  If xHigherPriorityTaskWoken is still pdFALSE
			then calling portYIELD_FROM_ISR() will have no effect.

			The implementation of portYIELD_FROM_ISR() used by the Windows port includes
			a return statement, which is why this function does not explicitly return a
			value. */
			portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  }
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
