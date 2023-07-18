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

#include "string.h"
#include "event_groups.h"
#include "semphr.h"

#include "usart.h"
#include "tim.h"

#include "lcd.h"
#include "pic.h"
#include "sx1278.h"
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

static uint8_t rxBufferUART1[100] = {0};
static uint8_t rxIndexUART1 = 0;

static uint8_t rxBufferUART2[100] = {0};
static uint8_t rxIndexUART2 = 0;

volatile bool is_UART1_RX_Done = true;
volatile bool is_UART2_RX_Done = true;
volatile bool is_LCD_processing = false;

static EventGroupHandle_t sx1278_evt_group;
static EventGroupHandle_t lcd_evt_group;
static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
static SemaphoreHandle_t lcd_semaphore;

/* USER CODE END Variables */
/* Definitions for Task0 */
osThreadId_t Task0Handle;
const osThreadAttr_t Task0_attributes = {
  .name = "Task0",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Task1 */
osThreadId_t Task1Handle;
const osThreadAttr_t Task1_attributes = {
  .name = "Task1",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task2 */
osThreadId_t Task2Handle;
const osThreadAttr_t Task2_attributes = {
  .name = "Task2",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Task0Func(void *argument);
void Task1Func(void *argument);
void Task2Func(void *argument);

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
  // lcd_semaphore = xSemaphoreCreateBinary();
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

  /* creation of Task1 */
  Task1Handle = osThreadNew(Task1Func, NULL, &Task1_attributes);

  /* creation of Task2 */
  Task2Handle = osThreadNew(Task2Func, NULL, &Task2_attributes);

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
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
  HAL_Delay(200);
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);

  HAL_UART_Receive_IT(&huart1, &rxBufferUART1[rxIndexUART1++], sizeof(uint8_t));

  HAL_TIM_Base_Stop_IT(&htim7);
  __HAL_TIM_SetCounter(&htim7, 0U);
  __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_BREAK);

  HAL_UART_Receive_IT(&huart2, &rxBufferUART2[rxIndexUART2++], sizeof(uint8_t));

  HAL_TIM_Base_Stop_IT(&htim6);
  __HAL_TIM_SetCounter(&htim6, 0U);
  __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_BREAK);

  // uint32_t timeKeeper = HAL_GetTick();
  /* Infinite loop */
  for (;;)
  {
    if ((is_UART1_RX_Done == true) && (rxIndexUART1 != 1))
    {
      logPC("UART1 RX - %s", rxBufferUART1);
      rxIndexUART1 = 0;
      memset(rxBufferUART1, '\0', sizeof(rxBufferUART1));
      HAL_UART_Abort_IT(&huart1);
      HAL_UART_Receive_IT(&huart1, &rxBufferUART1[rxIndexUART1++], sizeof(uint8_t));
      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
    }

    if ((is_UART2_RX_Done == true) && (rxIndexUART2 != 1))
    {
      logPC("UART2 RX - %s", rxBufferUART2);
      rxIndexUART2 = 0;
      memset(rxBufferUART2, '\0', sizeof(rxBufferUART2));
      HAL_UART_Abort_IT(&huart2);
      HAL_UART_Receive_IT(&huart2, &rxBufferUART2[rxIndexUART2++], sizeof(uint8_t));
      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
    }

    // if ((HAL_GetTick() - timeKeeper) >= 2000)
    // {
    //   HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    //   timeKeeper = HAL_GetTick();
    // }
    HAL_Delay(10);
    // osDelay(10);
  }
  /* USER CODE END Task0Func */
}

/* USER CODE BEGIN Header_Task1Func */
/**
 * @brief Function implementing the Task1 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task1Func */
void Task1Func(void *argument)
{
  /* USER CODE BEGIN Task1Func */
  static const char *TAG_LCD = "LCD";
  volatile static uint16_t coordinateX = 0;
  volatile static uint16_t coordinateY = 0;
  lcd_semaphore = xSemaphoreCreateBinary();

  HAL_Delay(1000);

  ILI9341_Unselect();
  ILI9341_Init();
  LOG(TAG_LCD, "Initialize Done...");
  {
    uint32_t timeMark = HAL_GetTick();
    ILI9341_FillScreen(ILI9341_WHITE);
    timeMark = HAL_GetTick() - timeMark;
    logPC("%s - Screen Fill time: %d ms\tFPS: %2.1f", TAG_LCD, timeMark, (float)((1000.0)/(float)(timeMark)));
  }
  ILI9341_DrawImage(60, 45, 200, 150, (const uint16_t *)gLogoMain);
  HAL_Delay(2000);
  ILI9341_FillScreen(ILI9341_BLACK);

  /* Infinite loop */
  for (;;)
  {

    xSemaphoreTake(lcd_semaphore, portMAX_DELAY);

    if (ILI9341_TouchGetCoordinates((uint16_t*)&coordinateX, (uint16_t*)&coordinateY) == true)
    {
      logPC("%s - Touched!\tCoordinate X - %d\tCoordinate Y - %d", TAG_LCD, coordinateX, coordinateY);
      ILI9341_DrawPixel(coordinateX, coordinateY, ILI9341_WHITE);
    }

    is_LCD_processing = false;
  }
  /* USER CODE END Task1Func */
}

/* USER CODE BEGIN Header_Task2Func */
/**
 * @brief Function implementing the Task2 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task2Func */
void Task2Func(void *argument)
{
  /* USER CODE BEGIN Task2Func */

  static uint8_t data[100] = {0};
  static int rssi = -1;
  static float snr = -1;
  static EventBits_t evt_bits;
  sx1278_evt_group = xEventGroupCreate();

  sx1278_init();
  sx1278_start_recv_data();

  /* Infinite loop */
  for (;;)
  {
    evt_bits = xEventGroupWaitBits(sx1278_evt_group, SX1278_DIO0_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

    if(((evt_bits & SX1278_DIO0_BIT) != 0U) && (HAL_GPIO_ReadPin(LoRa_EXT0_GPIO_Port, LoRa_EXT0_Pin) == GPIO_PIN_SET))
    {
      if (sx1278_recv_data((uint8_t *)data, &rssi, &snr, NULL) == SX1278_OK)
      {
        logPC("LoRa Received: \'%s\'\trssi: %d\tsnr: %2.1f", data, rssi, snr);
      }
    }
    xEventGroupClearBits(sx1278_evt_group, 0xFF);
    HAL_Delay(1);
  }
  /* USER CODE END Task2Func */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if ((GPIO_Pin == LoRa_EXT0_Pin) && (HAL_GPIO_ReadPin(LoRa_EXT0_GPIO_Port, LoRa_EXT0_Pin) == GPIO_PIN_SET))
  {
    xEventGroupSetBitsFromISR(sx1278_evt_group, SX1278_DIO0_BIT, &xHigherPriorityTaskWoken);
  }

  if ((GPIO_Pin == Touch_EXT_Pin) && (HAL_GPIO_ReadPin(Touch_EXT_GPIO_Port, Touch_EXT_Pin) == GPIO_PIN_RESET) && (is_LCD_processing == false))
  {
    xSemaphoreGiveFromISR(lcd_semaphore, &xHigherPriorityTaskWoken);
    is_LCD_processing = true;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    HAL_TIM_Base_Stop_IT(&htim7);
    if ((is_UART1_RX_Done == true) && (rxIndexUART1 == 1))
    {
      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
      __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_BREAK);
      is_UART1_RX_Done = false;
    }
    __HAL_TIM_SetCounter(&htim7, 0U);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_UART_Receive_IT(&huart1, &rxBufferUART1[rxIndexUART1++], sizeof(uint8_t));
  }

  if (huart->Instance == USART2)
  {
    HAL_TIM_Base_Stop_IT(&htim6);
    if ((is_UART2_RX_Done == true) && (rxIndexUART2 == 1))
    {
      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
      __HAL_TIM_CLEAR_IT(&htim6, TIM_IT_BREAK);
      is_UART2_RX_Done = false;
    }
    __HAL_TIM_SetCounter(&htim6, 0U);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_UART_Receive_IT(&huart2, &rxBufferUART2[rxIndexUART2++], sizeof(uint8_t));
  }
}

/* USER CODE END Application */

