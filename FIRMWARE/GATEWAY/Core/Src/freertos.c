/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <time.h>
#include "sx1278.h"
#include "timers.h"
#include "event_groups.h"
#include "common.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static const char *TAG = "FREERTOS";
EventGroupHandle_t sx1278_evt_group;

/* USER CODE END Variables */
/* Definitions for Task0 */
osThreadId_t Task0Handle;
const osThreadAttr_t Task0_attributes = {
  .name = "Task0",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(GPIO_Pin == GPIO_PIN_5)//PC5
	{
		xEventGroupSetBitsFromISR(sx1278_evt_group, SX1278_DIO0_BIT, &xHigherPriorityTaskWoken);
	}
}

//void sx1278_task(void *param)
//{
//	uint8_t data_send[128] = {0};
//	uint8_t data_recv[128] = {0};
//	float snr;//signal to noise ratio
//	int rssi; //received signal strength indication
//	EventBits_t evt_bits;
//	sx1278_init();
//	sx1278_evt_group = xEventGroupCreate();
//	float temp_test = 16.8;
//	float battery_test = 3.58;
//	float humid_test = 0.7;
//	float soil_test = 0.8;
//	float PH_test = 6.5;
//	while(1)
//	{
//		sx1278_start_recv_data();
//		evt_bits = xEventGroupWaitBits(sx1278_evt_group,SX1278_DIO0_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
//		if(evt_bits & SX1278_DIO0_BIT)
//		{
//			if(sx1278_recv_data(data_recv, &rssi, &snr, &sx1278_node) == SX1278_OK)
//			{
//				char data_log[100];
//				char snr_arr[10];
//				ftoa((double)snr, snr_arr, 2);
//				sprintf(data_log, "Packet rssi: %d, snr: %s", rssi, snr_arr);
//				LOG(TAG, data_log);
//				ftoa((double)temp_test, sx1278_node.temp, 2);
//				ftoa((double)battery_test, sx1278_node.battery, 2);
//				ftoa((double)humid_test, sx1278_node.humid, 2);
//				ftoa((double)soil_test, sx1278_node.soil, 2);
//				ftoa((double)PH_test, sx1278_node.PH, 2);
//				listen_before_talk();
//				send_respond(UPLINK_TX_RESPOND_OPCODE, sx1278_node, data_send);
//			}
//			else
//			{
//
//			}
//		}
//	}
//}

void peripheral_task(void *param)
{
	while(1)
	{
		vTaskDelay(100 / portTICK_RATE_MS);
	}
}
/* USER CODE END FunctionPrototypes */

void Task0Func(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task0 */
  Task0Handle = osThreadNew(Task0Func, NULL, &Task0_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Task0Func */
/**
  * @brief  Function implementing the Task0 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Task0Func */
void Task0Func(void *argument)
{
  /* USER CODE BEGIN Task0Func */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task0Func */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

