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

#include "stdio.h"
#include "string.h"
#include "event_groups.h"
#include "semphr.h"

#include "usart.h"
#include "tim.h"
#include "rtc.h"

#include "lcd.h"
#include "pic.h"
#include "sx1278.h"
#include "common.h"
#include "sim.h"
#include "flash.h"
#include "button.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct LoRa_Link_Struct
{
  uint16_t Node_ID;
  uint16_t Node_Status; 
  uint16_t Node_Battery_Voltage;
  uint16_t Node_Period;
} Link_Struct_t;

typedef struct LoRa_Data_Struct
{
  Link_Struct_t Link;
  uint16_t Node_Temperature;
  uint16_t Node_Humidity;
  uint16_t Node_Salinity;
  uint16_t Node_Conductivity;
  uint16_t Node_pH;
  uint16_t Node_N;
  uint16_t Node_P;
  uint16_t Node_K;
} Data_Struct_t;

typedef struct LoRa_Packet_ID_0
{
  uint16_t Packet_ID;
  Link_Struct_t Payload;
} Link_Packet_t;

typedef struct LoRa_Packet_ID_1
{
  uint16_t Packet_ID;
  Data_Struct_t Payload;
} Data_Packet_t;

typedef struct LoRa_Packet_ID_2
{
  uint16_t Packet_ID;
  uint16_t Target_Node_ID;
  uint16_t Target_Node_Status; 
  uint16_t Target_Node_Period;
  uint16_t Target_Node_Response;
} ResponsePacket_t;

typedef enum NodeStatus
{
  WAKEUP_MODE = 1U,
  LINK_MODE = 2U,
  NORMAL_MODE = 4U,
  RETRY_MODE = 8U,
  SHUTDOWN_MODE = 16U
} NodeStatus_t;

typedef enum Page_Enum
{
  MAIN_PAGE = 0U,
  JOIN_REQUEST_PAGE = 1U,
  NETWORK_PAGE = 2U,
  NODE_PAGE = 3U,
  CONTROL_PAGE = 4U,
  FAKE_PAGE = 133U
} Page_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PACKET_ID_0   (0xAAAAU)
#define PACKET_ID_1   (0x5A5AU)
#define PACKET_ID_2   (0x5555U)

#define LINK_ACCEPT   (0xAAU)
#define LINK_REJECT   (0x55U)

#define LINK_CARRYON  (0x00U)
#define LINK_DISMISS  (0xFFU)

#define LINK_ACK      (0xABU)
#define LINK_NACK     (0xBAU)

#define DEFAULT_PERIOD_SEC  (300U)

// DO NOT CHANGE
#define EVT_LORA      (0x04U)
#define EVT_TOCUH     (0x02U)

// DO NOT CHANGE
#define MAX_NODE      (50U)

// WARNING: DO NOT CHANGE
#define USE_WIFI      (true)
#define USE_4G        (false)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

static uint8_t rxBufferUART1[100] = {0};
volatile static uint8_t rxIndexUART1 = 0;

static uint8_t rxBufferUART2[100] = {0};
volatile static uint8_t rxIndexUART2 = 0;

volatile bool is_UART1_RX_Done = true;
volatile bool is_UART2_RX_Done = true;
volatile bool is_LCD_processing = false;
volatile bool is_LoRa_processing = false;
volatile bool is_4G_or_WiFi = USE_WIFI;
volatile bool is_WiFi_connected = false;
volatile bool is_MQTT_connected = false;
volatile bool is_promt_window_on = false;
volatile bool is_page_change = false;


static Data_Struct_t Node_Table[MAX_NODE] = {0};
static uint8_t number_of_Node_inNetwork = 0;
static uint8_t number_of_Node_inQueue = 0;
static uint8_t number_of_Response_Packet_inQueue = 0;
static uint16_t selected_Node_ID = 0;

static EventGroupHandle_t evt_group;
static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
static QueueHandle_t link_queue;
static QueueHandle_t response_queue;

volatile uint64_t traceTime = 0;

static Page_t Current_Page = MAIN_PAGE;
static uint16_t arrayNodeDisplay[4] = {0};
static char tempString[50] = {0};

/* USER CODE END Variables */
/* Definitions for Task0 */
osThreadId_t Task0Handle;
const osThreadAttr_t Task0_attributes = {
  .name = "Task0",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task1 */
osThreadId_t Task1Handle;
const osThreadAttr_t Task1_attributes = {
  .name = "Task1",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task2 */
osThreadId_t Task2Handle;
const osThreadAttr_t Task2_attributes = {
  .name = "Task2",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Task3 */
osThreadId_t Task3Handle;
const osThreadAttr_t Task3_attributes = {
  .name = "Task3",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

bool App_Add_Node_To_Network(Link_Struct_t link);
bool App_Delete_Node_From_Network(uint16_t Node_ID);

bool is_Node_ID_inLinkQueue(uint16_t Node_ID, bool remove);
bool is_Node_ID_inResponseQueue(uint16_t Node_ID, bool remove);
bool is_Node_ID_inNetwork(uint16_t Node_ID);
uint8_t get_Index_from_Node_ID(uint16_t Node_ID);
void* LoRa_Packet_Parser(const uint8_t* data, uint32_t len);

void LCD_Print_Join_Request_Page(void);
void LCD_Print_Promt_Window(void);
void LCD_Join_Request_Page_Handle(const uint16_t x, const uint16_t y);

void LCD_Print_Main_Page(void);
void LCD_Main_Page_Handle(const uint16_t x, const uint16_t y);

void LCD_Print_Network_Page(void);
void LCD_Network_Page_Handle(const uint16_t x, const uint16_t y);

void LCD_Print_Node_Page(void);
void LCD_Node_Page_Handle(const uint16_t x, const uint16_t y);

void LCD_Print_Fake_Page(void);
void LCD_Fake_Page_Handle(const uint16_t x, const uint16_t y);


/* USER CODE END FunctionPrototypes */

void Task0Func(void *argument);
void Task1Func(void *argument);
void Task2Func(void *argument);
void Task3Func(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{
  HAL_TIM_Base_Start_IT(&htim5);
}

__weak unsigned long getRunTimeCounterValue(void)
{
  // uint32_t counter_now = __HAL_TIM_GetCounter(&htim5);
  // HAL_TIM_Base_Stop(&htim5);
  // __HAL_TIM_SET_COUNTER(&htim5, 0U);
  // HAL_TIM_Base_Start(&htim5);
  return traceTime;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  HAL_Delay(1500);
  Log_Init();
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

  /* creation of Task3 */
  Task3Handle = osThreadNew(Task3Func, NULL, &Task3_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  evt_group = xEventGroupCreate();
  if (evt_group == NULL) while(1);
  xEventGroupClearBits(evt_group, 0xFF);

  link_queue = xQueueCreate(MAX_NODE, sizeof(Link_Struct_t));
  if (link_queue == NULL) while(1);

  response_queue = xQueueCreate(5U, sizeof(ResponsePacket_t));
  if (response_queue == NULL) while(1);
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Task0Func */
/**
 * @brief  Function implementing the Task0 thread.
 * 
 *  This is the task to process UART (including ESP32) and helper functionalities
 * 
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

  static TaskStatus_t *pxTaskStatusArray;
	volatile static UBaseType_t uxArraySize, x;
	static unsigned long ulTotalRunTime;
	static float runtime_percent;

  uint32_t timeKeeper0 = HAL_GetTick();
  uint32_t timeKeeper1 = HAL_GetTick();
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
      SIM_GetResponse((const char*)rxBufferUART2);
      rxIndexUART2 = 0;
      memset(rxBufferUART2, '\0', sizeof(rxBufferUART2));
      HAL_UART_Abort_IT(&huart2);
      HAL_UART_Receive_IT(&huart2, &rxBufferUART2[rxIndexUART2++], sizeof(uint8_t));
      HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
    }

    if ((HAL_GetTick() - timeKeeper0) >= 2000)
    {
      if (is_LCD_processing == true)  is_LCD_processing = false;
      if (is_LoRa_processing == true)
      {
        sx1278_start_recv_data();
        xEventGroupClearBits(evt_group, EVT_LORA);
        is_LoRa_processing = false;
      }
      timeKeeper0 = HAL_GetTick();
    }

    if ((HAL_GetTick() - timeKeeper1) >= 30000)
    {
      uxArraySize = uxTaskGetNumberOfTasks();
      pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t)); // a little bit scary!
      if (pxTaskStatusArray != NULL)
      {
        uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);

        logPC("Task count = %lu\n", uxArraySize);
        logPC("No      Name          S Usage   HW\n");

        for (x = 0; x < uxArraySize; x++)
        {
          runtime_percent = (float) ((100 * (float)pxTaskStatusArray[x].ulRunTimeCounter) / (float)ulTotalRunTime);
          logPC("Task %lu: %-12s %2d %07.4f %4i\n", x, pxTaskStatusArray[x].pcTaskName, pxTaskStatusArray[x].eCurrentState, runtime_percent, pxTaskStatusArray[x].usStackHighWaterMark);
        }
        vPortFree(pxTaskStatusArray);
      }
      else  logPC("Unable to allocate stack space");
      timeKeeper1 = HAL_GetTick();
    }

    // HAL_Delay(10);
    osDelay(10);
  }
  /* USER CODE END Task0Func */
}

/* USER CODE BEGIN Header_Task1Func */
/**
 * @brief Function implementing the Task1 thread.
 * 
 *    This is the Task to process LCD display functionalities
 * 
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
  static EventBits_t evt_bits_lcd;
  xEventGroupClearBits(evt_group, EVT_TOCUH);

  // osDelay(500);

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
  osDelay(2000);
  // ILI9341_FillScreen(ILI9341_BLACK);
  is_page_change = true;
  LCD_Print_Main_Page();

  __HAL_TIM_SetCounter(&htim4, 0U);
  HAL_TIM_Base_Start_IT(&htim4);

  /* Infinite loop */
  for (;;)
  {
    evt_bits_lcd = xEventGroupWaitBits(evt_group, EVT_TOCUH, pdTRUE, pdFALSE, portMAX_DELAY);

    if(((evt_bits_lcd & EVT_TOCUH) != 0U) && (HAL_GPIO_ReadPin(Touch_EXT_GPIO_Port, Touch_EXT_Pin) == GPIO_PIN_RESET))
    {
      if (ILI9341_TouchGetCoordinates((uint16_t*)&coordinateX, (uint16_t*)&coordinateY) == true)
      {
        logPC("%s - Touched!\tCoordinate X - %d\tCoordinate Y - %d", TAG_LCD, coordinateX, coordinateY);
        switch (Current_Page)
        {
          case MAIN_PAGE: {
            LCD_Main_Page_Handle(coordinateX, coordinateY);
            break;
          }
          case JOIN_REQUEST_PAGE: {
            LCD_Join_Request_Page_Handle(coordinateX, coordinateY);
            break;
          }
          case NETWORK_PAGE: {
            LCD_Network_Page_Handle(coordinateX, coordinateY);
            break;
          }
          case NODE_PAGE: {
            LCD_Node_Page_Handle(coordinateX, coordinateY);
            break;
          }
          case CONTROL_PAGE: {
            LCD_Control_Page_Handle(coordinateX, coordinateY);
            break;
          }
          case FAKE_PAGE: {
            LCD_Fake_Page_Handle(coordinateX, coordinateY);
            break;
          }
          default: break;
        }
      }
    }

    // Refresh Page
    switch (Current_Page)
    {
      case MAIN_PAGE: {
        LCD_Print_Main_Page();
        break;
      }
      case JOIN_REQUEST_PAGE: {
        LCD_Print_Join_Request_Page();
        break;
      }
      case NETWORK_PAGE: {
        LCD_Print_Network_Page();
        break;
      }
      case NODE_PAGE: {
        LCD_Print_Node_Page();
        break;
      }
      case CONTROL_PAGE: {
        LCD_Print_Control_Page();
        break;
      }
      case FAKE_PAGE: {
        LCD_Print_Fake_Page();
        break;
      }
      default: break;
    }

    xEventGroupClearBits(evt_group, EVT_TOCUH);
    is_LCD_processing = false;
    osDelay(100);
  }
  /* USER CODE END Task1Func */
}

/* USER CODE BEGIN Header_Task2Func */
/**
 * @brief Function implementing the Task2 thread.
 * 
 *    This is the Task to handle LoRa transceiver
 * 
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Task2Func */
void Task2Func(void *argument)
{
  /* USER CODE BEGIN Task2Func */
  osDelay(2000);
  static uint8_t data[100] = {0};
  static uint32_t nByteRx = 0;
  static int rssi = -1;
  static float snr = -1;
  static EventBits_t evt_bits;
  static void * ptr2payload = NULL;

  sx1278_init();
  sx1278_start_recv_data();

  /* Infinite loop */
  for (;;)
  {
    xEventGroupClearBits(evt_group, EVT_LORA);
    evt_bits = xEventGroupWaitBits(evt_group, EVT_LORA, pdTRUE, pdFALSE, portMAX_DELAY);

    if(((evt_bits & EVT_LORA) != 0U) && (HAL_GPIO_ReadPin(LoRa_EXT0_GPIO_Port, LoRa_EXT0_Pin) == GPIO_PIN_SET))
    {
      if (sx1278_recv_data((uint8_t *)data, &nByteRx, &rssi, &snr, true) == SX1278_OK)
      {
        logPC("LoRa Received %d byte(s)\trssi: %d\tsnr: %2.1f\n", nByteRx, rssi, snr);
        logPC("LoRa payload:\t");
        for (int i = 0; i < nByteRx; i++)
        {
          logPC("%02X ", data[i]);
        }
        ptr2payload = LoRa_Packet_Parser((const uint8_t*)data, nByteRx);
        // TODO: process payload
      }
    }
    xEventGroupClearBits(evt_group, EVT_LORA);
    is_LoRa_processing = false;
    osDelay(1);
  }
  /* USER CODE END Task2Func */
}

/* USER CODE BEGIN Header_Task3Func */
/**
* @brief Function implementing the Task3 thread.
* 
*     This is the Task to handle SIM4G functionalities
* 
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task3Func */
void Task3Func(void *argument)
{
  /* USER CODE BEGIN Task3Func */
  // if (is_4G_or_WiFi != USE_4G)  osThreadTerminate(Task3Handle);
  // TODO: implement SIM 4G MQTT here
  osDelay(CMD_DELAY_SHORT);
  if (SIM_Init() == true)
  {
    SIM_checkSignalStrength();
    SIM_startGPRS();
  }
  SIM_Deinit();
  osThreadTerminate(Task3Handle);

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Task3Func */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void App_Refresh_Tick(void)
{
  HAL_TIM_Base_Stop_IT(&htim4);
  __HAL_TIM_SetCounter(&htim4, 0U);
  xEventGroupSetBitsFromISR(evt_group, EVT_TOCUH, &xHigherPriorityTaskWoken);
  is_LCD_processing = true;
  RTC_DateTypeDef DateToUpdate = {0};
  HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
  uint32_t reg = (DateToUpdate.WeekDay << 16U) | (DateToUpdate.Month << 12U) | (DateToUpdate.Date << 7U) | (DateToUpdate.Year);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, reg);
  HAL_TIM_Base_Start_IT(&htim4);
}

bool App_Add_Node_To_Network(Link_Struct_t link)
{
  if (number_of_Node_inNetwork == MAX_NODE) return false;
  for (int i = 0; i < MAX_NODE; i++)
  {
    if (Node_Table[i].Link.Node_ID == 0U)
    {
      Node_Table[i].Link = link;
      number_of_Node_inNetwork++;
      return true;
    }
  }
  return false;
}

bool App_Delete_Node_From_Network(uint16_t Node_ID)
{
  Link_Struct_t link = {0};
  memset(&link, '\0', sizeof(Link_Struct_t));
  for (int i = 0; i < MAX_NODE; i++)
  {
    if (Node_Table[i].Link.Node_ID == Node_ID)
    {
      Node_Table[i].Link = link;
      number_of_Node_inNetwork--;
      return true;
    }
  }
  return false;
}

bool App_Save_Data(Data_Struct_t data)
{
  for (int i = 0; i < MAX_NODE; i++)
  {
    if (Node_Table[i].Link.Node_ID == data.Link.Node_ID)
    {
      Node_Table[i] = data;
      return true;
    }
  }
  return false;
}

//-------------------------------------------------------------------------------------------------------------------

bool is_Node_ID_inLinkQueue(uint16_t Node_ID, bool remove)
{
  number_of_Node_inQueue = MAX_NODE - (uint8_t)uxQueueSpacesAvailable(link_queue);
  Link_Struct_t link_stored;
  for (int i = 0; i < number_of_Node_inQueue; i++)
  {
    xQueueReceive(link_queue, &link_stored, 100);
    if (link_stored.Node_ID == Node_ID)
    {
      if (remove == false)  xQueueSendToFront(link_queue, &link_stored, 100);
      return true;
    }
    else xQueueSend(link_queue, &link_stored, 100);
  }
  return false;
}

bool is_Node_ID_inResponseQueue(uint16_t Node_ID, bool remove)
{
  number_of_Response_Packet_inQueue = 5U - (uint8_t)uxQueueSpacesAvailable(response_queue);
  ResponsePacket_t resp_stored;
  for (int i = 0; i < number_of_Response_Packet_inQueue; i++)
  {
    xQueueReceive(response_queue, &resp_stored, 100);
    if (resp_stored.Target_Node_ID == Node_ID)
    {
      if (remove == false)  xQueueSendToFront(response_queue, &resp_stored, 100);
      return true;
    }
    else xQueueSend(response_queue, &resp_stored, 100);
  }
  return false;
}

bool is_Node_ID_inNetwork(uint16_t Node_ID)
{
  for (int i = 0; i < MAX_NODE; i++)
  {
    if (Node_Table[i].Link.Node_ID == Node_ID)  return true;
  }
  return false;
}

uint8_t get_Index_from_Node_ID(uint16_t Node_ID)
{
  if (Node_ID == 0)   return 0xFF;
  for (uint8_t i = 0; i < MAX_NODE; i++)
  {
    if (Node_Table[i].Link.Node_ID == Node_ID)  return i;
  }
  return 0xFF;
}

/**
 * @brief check communicate line for idle slot to talk
 * 
 * @return true ok now TALK!
 * @return false <infinite loop with random delay time>
 */
bool listen_before_talk(void)
{
  EventBits_t evt_bits;
  uint8_t irq;
  uint32_t timeout = HAL_GetTick();
  is_LoRa_processing = false;
  while ((HAL_GetTick() - timeout) <= 4000)
  {
    xEventGroupClearBits(evt_group, EVT_LORA);
    sx1278_cad();
    evt_bits = xEventGroupWaitBits(evt_group, EVT_LORA, pdTRUE, pdFALSE, 1000);
    if ((evt_bits & EVT_LORA) != 0U)
    {
      logPC("LoRa - CAD Done\tTook: %i ms\t", HAL_GetTick() - timeout);
      irq = sx1278_read_reg(REG_IRQ_FLAGS);
      sx1278_write_reg(REG_IRQ_FLAGS, irq);
      if ((irq & 0x01) != 0U)
      {
        logPC("LoRa - CAD Detected\t");
        HAL_Delay(get_random_value(0, 50));
      }
      else 
      {
        logPC("LoRa - CAD Clear\t");
        sx1278_set_irq(0x00);
        sx1278_standby();
        return true;
      }
    }
  }
  irq = sx1278_read_reg(REG_IRQ_FLAGS);
  sx1278_write_reg(REG_IRQ_FLAGS, irq);
  sx1278_set_irq(0x00);
  sx1278_standby();
  logPC("Listen FAIL...\t");
  return false;
}

void* LoRa_Packet_Parser(const uint8_t* data, uint32_t len)
{
  // void *p = NULL;
  switch ((uint16_t)((data[0] << 8) & 0xFF00) | (uint16_t)(data[1] & 0x00FF))
  {
    case PACKET_ID_0: {
      if (len != sizeof(Link_Packet_t)) break;
      logPC("Link Packet\t");
      Link_Struct_t link_payload;
      memcpy((uint8_t *)&link_payload, (uint8_t *)&data[2], sizeof(Link_Struct_t));
      if ((is_Node_ID_inNetwork(link_payload.Node_ID) == true) && (is_Node_ID_inResponseQueue(link_payload.Node_ID, false) == false))
      {
        ResponsePacket_t resp = {PACKET_ID_2, link_payload.Node_ID, NORMAL_MODE, 180U, (uint16_t)(((LINK_ACCEPT << 8) & 0xFF00) | LINK_ACK)};
        xQueueSendToFront(response_queue, &resp, 100);
        logPC("Node already in Network -> Send promote!\t");
      }
      if (is_Node_ID_inResponseQueue(link_payload.Node_ID, false) == true)
      {
        ResponsePacket_t resp;
        xQueueReceive(response_queue, &resp, 100);
        if (resp.Target_Node_ID != link_payload.Node_ID)
        {
          logPC("Something went wrong...");
          xQueueSendToFront(response_queue, &resp, 100);
          break;
        }
        logPC("Response Ready...\t");
        if (listen_before_talk() == false)  xQueueSendToFront(response_queue, &resp, 100);
        else  {logPC("Sending packet...\n"); sx1278_send_data(&resp, sizeof(ResponsePacket_t));}
        sx1278_start_recv_data();
        break;
      }
      if (is_Node_ID_inLinkQueue(link_payload.Node_ID, true) == false)  logPC("New Node! Send to Queue...\t");
      else                                                              logPC("Old Node...\t");
      xQueueSend(link_queue, &(link_payload), 100);
      number_of_Node_inQueue = MAX_NODE - (uint8_t)uxQueueSpacesAvailable(link_queue);
      if (number_of_Node_inQueue != MAX_NODE)   logPC("Link Queue has %d Node(s)\n", number_of_Node_inQueue);
      else                                      logPC("Link Queue Full\n");
      break;
    }
    case PACKET_ID_1: {
      if (len != sizeof(Data_Packet_t)) break;
      logPC("Data Packet\t");
      Data_Struct_t data_recv;
      memcpy((uint8_t *)&data_recv, (uint8_t *)&data[2], sizeof(Data_Struct_t));
      if (is_Node_ID_inNetwork(data_recv.Link.Node_ID) == true)
      {
        logPC("Node Registered -> Saving data...\t%s", (App_Save_Data(data_recv))? ("Success\t"):("Fail\t"));
        ResponsePacket_t resp = {PACKET_ID_2, data_recv.Link.Node_ID, NORMAL_MODE, 180U, (uint16_t)(((LINK_CARRYON << 8) & 0xFF00) | LINK_ACK)};
        logPC("Response Ready...\t");
        if (listen_before_talk() == false)  break;
        else  {logPC("Sending packet...\n"); sx1278_send_data(&resp, sizeof(ResponsePacket_t));}
        logPC("Done!\n");
        sx1278_start_recv_data();
      }
      else
      {
        logPC("Node Unregister -> Demote Node...\t");
        ResponsePacket_t resp = {PACKET_ID_2, data_recv.Link.Node_ID, NORMAL_MODE, 180U, (uint16_t)(((LINK_DISMISS << 8) & 0xFF00) | LINK_ACK)};
        logPC("Response Ready...\t");
        if (listen_before_talk() == false)  break;
        else  {logPC("Sending packet...\n"); sx1278_send_data(&resp, sizeof(ResponsePacket_t));}
        logPC("Done!\n");
        sx1278_start_recv_data();
      }
      break;
    }
    default: {
      logPC("???? Packet\n");
      break;
    }
  }
  return NULL;
}

//-------------------------------------------------------------------------------------------------------------------

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if ((GPIO_Pin == LoRa_EXT0_Pin) && (HAL_GPIO_ReadPin(LoRa_EXT0_GPIO_Port, LoRa_EXT0_Pin) == GPIO_PIN_SET) && (is_LoRa_processing == false))
  {
    xEventGroupSetBitsFromISR(evt_group, EVT_LORA, &xHigherPriorityTaskWoken);
    is_LoRa_processing = true;
  }

  if ((GPIO_Pin == Touch_EXT_Pin) && (HAL_GPIO_ReadPin(Touch_EXT_GPIO_Port, Touch_EXT_Pin) == GPIO_PIN_RESET) && (is_LCD_processing == false))
  {
    xEventGroupSetBitsFromISR(evt_group, EVT_TOCUH, &xHigherPriorityTaskWoken);
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

//-------------------------------------------------------------------------------------------------------------------

void LCD_Print_Promt_Window(void)
{
  if (Current_Page == JOIN_REQUEST_PAGE)
  {
    is_promt_window_on = true;
    ILI9341_UpdateButton(&Promt_Window);
    ILI9341_WriteString(Promt_Window.pos_x+12, Promt_Window.pos_y+10, "ALLOW REQUEST?", Font_11x18, ILI9341_RED, Promt_Window.color_on);
    sprintf(tempString, "Node ID %#04X", selected_Node_ID);
    ILI9341_WriteString(Promt_Window.pos_x+12, Promt_Window.pos_y+35, tempString, Font_11x18, ILI9341_BLACK, Promt_Window.color_on);
    ILI9341_UpdateButton(&Promt_Accept);
    ILI9341_WriteString(Promt_Accept.pos_x+7, Promt_Accept.pos_y+6, "Accept", Font_11x18, ILI9341_BLACK, Promt_Accept.color_on);
    ILI9341_UpdateButton(&Promt_Reject);
    ILI9341_WriteString(Promt_Reject.pos_x+7, Promt_Reject.pos_y+6, "Reject", Font_11x18, ILI9341_BLACK, Promt_Reject.color_on);
  }
  else if (Current_Page == NODE_PAGE)
  {
    is_promt_window_on = true;
    ILI9341_UpdateButton(&Promt_Window);
    ILI9341_WriteString(Promt_Window.pos_x+22, Promt_Window.pos_y+10, "DELETE NODE?", Font_11x18, ILI9341_RED, Promt_Window.color_on);
    sprintf(tempString, "Node ID %#04X", selected_Node_ID);
    ILI9341_WriteString(Promt_Window.pos_x+14, Promt_Window.pos_y+35, tempString, Font_11x18, ILI9341_BLACK, Promt_Window.color_on);
    ILI9341_UpdateButton(&Promt_Confirm);
    ILI9341_WriteString(Promt_Confirm.pos_x+7, Promt_Confirm.pos_y+6, "Confirm", Font_11x18, ILI9341_WHITE, Promt_Confirm.color_on);
  }
}

//-------------------------------------------------------------------------------------------------------------------

void LCD_Print_Join_Request_Page(void)
{
  memset(arrayNodeDisplay, '\0', sizeof(arrayNodeDisplay));
  Link_Struct_t link_stored;

  number_of_Node_inQueue = MAX_NODE - (uint8_t)uxQueueSpacesAvailable(link_queue);
  sprintf(tempString, "Join Request (%i)", number_of_Node_inQueue);

  if (is_page_change == true) ILI9341_FillScreen(ILI9341_BLACK);

  if (is_promt_window_on == true)   return;

  ILI9341_WriteString(80, 21, tempString, Font_11x18, ILI9341_YELLOW, ILI9341_BLACK);

  if (is_page_change == true) ILI9341_UpdateButton(&Back_Top_Left);
  ILI9341_WriteString(Back_Top_Left.pos_x-10, Back_Top_Left.pos_y-13, "<", Font_16x26, ILI9341_BLACK, Back_Top_Left.color_on);

  ILI9341_DrawRectangle(Button1.pos_x, Button1.pos_y, Button1.shape_w, Button1.shape_h, ILI9341_YELLOW);
  ILI9341_DrawRectangle(Button2.pos_x, Button2.pos_y, Button2.shape_w, Button2.shape_h, ILI9341_YELLOW);
  ILI9341_DrawRectangle(Button3.pos_x, Button3.pos_y, Button3.shape_w, Button3.shape_h, ILI9341_YELLOW);
  ILI9341_DrawRectangle(Button4.pos_x, Button4.pos_y, Button4.shape_w, Button4.shape_h, ILI9341_YELLOW);

  if (number_of_Node_inQueue >= 1)
  {
    xQueueReceive(link_queue, &link_stored, 100);
    arrayNodeDisplay[0] = link_stored.Node_ID;
    sprintf(tempString, "Node ID: %#04X", link_stored.Node_ID);
    ILI9341_WriteString(Button1.pos_x+22, Button1.pos_y+10, tempString, Font_7x10, ILI9341_GREEN, ILI9341_BLACK);
    sprintf(tempString, "Node Status: %d", link_stored.Node_Status);
    ILI9341_WriteString(Button1.pos_x+10, Button1.pos_y+35, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
    sprintf(tempString, "Node Batt: % 4d mV", link_stored.Node_Battery_Voltage);
    ILI9341_WriteString(Button1.pos_x+10, Button1.pos_y+55, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
    sprintf(tempString, "Node Period: % 4d s", link_stored.Node_Period);
    ILI9341_WriteString(Button1.pos_x+10, Button1.pos_y+75, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
    xQueueSend(link_queue, &link_stored, 100);
  }
  if (number_of_Node_inQueue >= 2)
  {
    xQueueReceive(link_queue, &link_stored, 100);
    arrayNodeDisplay[1] = link_stored.Node_ID;
    sprintf(tempString, "Node ID: %#04X", link_stored.Node_ID);
    ILI9341_WriteString(Button2.pos_x+22, Button2.pos_y+10, tempString, Font_7x10, ILI9341_GREEN, ILI9341_BLACK);
    sprintf(tempString, "Node Status: %d", link_stored.Node_Status);
    ILI9341_WriteString(Button2.pos_x+10, Button2.pos_y+35, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
    sprintf(tempString, "Node Batt: % 4d mV", link_stored.Node_Battery_Voltage);
    ILI9341_WriteString(Button2.pos_x+10, Button2.pos_y+55, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
    sprintf(tempString, "Node Period: % 4d s", link_stored.Node_Period);
    ILI9341_WriteString(Button2.pos_x+10, Button2.pos_y+75, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
    xQueueSend(link_queue, &link_stored, 100);
  }
  if (number_of_Node_inQueue >= 3)
  {
    xQueueReceive(link_queue, &link_stored, 100);
    arrayNodeDisplay[2] = link_stored.Node_ID;
    sprintf(tempString, "Node ID: %#04X", link_stored.Node_ID);
    ILI9341_WriteString(Button3.pos_x+22, Button3.pos_y+10, tempString, Font_7x10, ILI9341_GREEN, ILI9341_BLACK);
    sprintf(tempString, "Node Status: %d", link_stored.Node_Status);
    ILI9341_WriteString(Button3.pos_x+10, Button3.pos_y+35, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
    sprintf(tempString, "Node Batt: % 4d mV", link_stored.Node_Battery_Voltage);
    ILI9341_WriteString(Button3.pos_x+10, Button3.pos_y+55, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
    sprintf(tempString, "Node Period: % 4d s", link_stored.Node_Period);
    ILI9341_WriteString(Button3.pos_x+10, Button3.pos_y+75, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
    xQueueSend(link_queue, &link_stored, 100);
  }
  if (number_of_Node_inQueue >= 4)
  {
    xQueueReceive(link_queue, &link_stored, 100);
    arrayNodeDisplay[3] = link_stored.Node_ID;
    sprintf(tempString, "Node ID: %#04X", link_stored.Node_ID);
    ILI9341_WriteString(Button4.pos_x+22, Button4.pos_y+10, tempString, Font_7x10, ILI9341_GREEN, ILI9341_BLACK);
    sprintf(tempString, "Node Status: %d", link_stored.Node_Status);
    ILI9341_WriteString(Button4.pos_x+10, Button4.pos_y+35, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
    sprintf(tempString, "Node Batt: % 4d mV", link_stored.Node_Battery_Voltage);
    ILI9341_WriteString(Button4.pos_x+10, Button4.pos_y+55, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
    sprintf(tempString, "Node Period: % 4d s", link_stored.Node_Period);
    ILI9341_WriteString(Button4.pos_x+10, Button4.pos_y+75, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
    xQueueSend(link_queue, &link_stored, 100);
  }

  is_page_change = false;
}

void LCD_Join_Request_Page_Handle(const uint16_t x, const uint16_t y)
{
  if (is_promt_window_on == true)
  {
    if (ILI9341_checkButton(x, y, &Promt_Accept, false) == true)
    {
      ResponsePacket_t resp_packet;
      Link_Struct_t link_2_store;
      is_Node_ID_inResponseQueue(selected_Node_ID, true);
      is_Node_ID_inLinkQueue(selected_Node_ID, false);
      xQueueReceive(link_queue, &link_2_store, 100);
      number_of_Node_inQueue = MAX_NODE - (uint8_t)uxQueueSpacesAvailable(link_queue);
      if (number_of_Node_inQueue != MAX_NODE)   logPC("Link Queue has %d Node(s)\n", number_of_Node_inQueue);
      else                                      logPC("Link Queue Full\n");
      resp_packet.Packet_ID = PACKET_ID_2;
      resp_packet.Target_Node_ID = selected_Node_ID;
      resp_packet.Target_Node_Status = 4U;
      resp_packet.Target_Node_Period = 180U;
      resp_packet.Target_Node_Response = (uint16_t)(((uint16_t)(LINK_ACCEPT << 8) & 0xFF00) | LINK_ACK);
      xQueueSend(response_queue, &resp_packet, 100);
      number_of_Response_Packet_inQueue = 5U - (uint8_t)uxQueueSpacesAvailable(response_queue);
      if (number_of_Response_Packet_inQueue != 5U)    logPC("Response Queue has %d packet(s)\n", number_of_Response_Packet_inQueue);
      else                                            logPC("Response Queue Full\n");
      App_Add_Node_To_Network(link_2_store);
      ILI9341_FillRectangle(Promt_Window.pos_x, Promt_Window.pos_y, Promt_Window.shape_w, Promt_Window.shape_h, ILI9341_BLACK);
      is_page_change = true;
      LCD_Print_Join_Request_Page();
      is_promt_window_on = false;
    } 
    else if (ILI9341_checkButton(x, y, &Promt_Reject, false) == true)
    {
      ResponsePacket_t resp_packet;
      is_Node_ID_inResponseQueue(selected_Node_ID, true);
      is_Node_ID_inLinkQueue(selected_Node_ID, true);
      number_of_Node_inQueue = MAX_NODE - (uint8_t)uxQueueSpacesAvailable(link_queue);
      if (number_of_Node_inQueue != MAX_NODE)   logPC("Link Queue has %d Node(s)\n", number_of_Node_inQueue);
      else                                      logPC("Link Queue Full\n");
      resp_packet.Packet_ID = PACKET_ID_2;
      resp_packet.Target_Node_ID = selected_Node_ID;
      resp_packet.Target_Node_Status = 16U;
      resp_packet.Target_Node_Period = 180U;
      resp_packet.Target_Node_Response = (uint16_t)(((uint16_t)(LINK_REJECT << 8) & 0xFF00) | LINK_ACK);
      xQueueSend(response_queue, &resp_packet, 100);
      number_of_Response_Packet_inQueue = 5U - (uint8_t)uxQueueSpacesAvailable(response_queue);
      if (number_of_Response_Packet_inQueue != 5U)    logPC("Response Queue has %d packet(s)\n", number_of_Response_Packet_inQueue);
      else                                            logPC("Response Queue Full\n");
      ILI9341_FillRectangle(Promt_Window.pos_x, Promt_Window.pos_y, Promt_Window.shape_w,Promt_Window.shape_h, ILI9341_BLACK);
      is_page_change = true;
      LCD_Print_Join_Request_Page();
      is_promt_window_on = false;
    }
    else if (ILI9341_checkButton(x, y, &Promt_Window, false) == false)
    {
      ILI9341_FillRectangle(Promt_Window.pos_x, Promt_Window.pos_y, Promt_Window.shape_w,Promt_Window.shape_h, ILI9341_BLACK);
      LCD_Print_Join_Request_Page();
      is_promt_window_on = false;
    }
    return;
  }

  if (ILI9341_checkButton(x, y, &Back_Top_Left, false) == true)
  {
    Current_Page = MAIN_PAGE;
    is_page_change = true;
  }

  if (number_of_Node_inQueue == 0)  return;

  if      ((ILI9341_checkButton(x, y, &Button1, false) == true) && (arrayNodeDisplay[0] != 0U))
  {
    selected_Node_ID = arrayNodeDisplay[0];
    LCD_Print_Promt_Window();
  }
  else if ((ILI9341_checkButton(x, y, &Button2, false) == true) && (arrayNodeDisplay[1] != 0U))
  {
    selected_Node_ID = arrayNodeDisplay[1];
    LCD_Print_Promt_Window();
  }
  else if ((ILI9341_checkButton(x, y, &Button3, false) == true) && (arrayNodeDisplay[2] != 0U))
  {
    selected_Node_ID = arrayNodeDisplay[2];
    LCD_Print_Promt_Window();
  }
  else if ((ILI9341_checkButton(x, y, &Button4, false) == true) && (arrayNodeDisplay[3] != 0U))
  {
    selected_Node_ID = arrayNodeDisplay[3];
    LCD_Print_Promt_Window();
  }
}

//-------------------------------------------------------------------------------------------------------------------

void LCD_Print_Main_Page(void)
{
  static RTC_TimeTypeDef sTime = {0};
  static RTC_DateTypeDef sDate = {0};

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  if (is_page_change == true) ILI9341_FillScreen(ILI9341_BLACK);

  ILI9341_WriteString(20, 10, "SIM:OFF", Font_7x10, ILI9341_YELLOW, ILI9341_BLACK);
  ILI9341_WriteString(122, 10, "WIFI:ON-DIS", Font_7x10, ILI9341_YELLOW, ILI9341_BLACK);
  ILI9341_WriteString(244, 10, "MQTT:DIS", Font_7x10, ILI9341_YELLOW, ILI9341_BLACK);

  sprintf(tempString, "%02i:%02i", sTime.Hours, sTime.Minutes);
  ILI9341_WriteString(120, 40, tempString, Font_16x26, ILI9341_PINK, ILI9341_BLACK);
  sprintf(tempString, "%02i/%02i/20%02i", sDate.Date, sDate.Month, sDate.Year);
  ILI9341_WriteString(105, 75, tempString, Font_11x18, ILI9341_DARKBLUE, ILI9341_BLACK);

  if (is_page_change == false) return;

  ILI9341_WriteString(80, 100, "SOL GARDEN", Font_16x26, ILI9341_YELLOW, ILI9341_BLACK);

  ILI9341_UpdateButton(&To_Join_Request_Page);
  ILI9341_WriteString(To_Join_Request_Page.pos_x-28, To_Join_Request_Page.pos_y-8, "JOIN>", Font_11x18, ILI9341_BLACK, To_Join_Request_Page.color_on);
  ILI9341_UpdateButton(&To_Network_Page);
  ILI9341_WriteString(To_Network_Page.pos_x-28, To_Network_Page.pos_y-8, "<NETW", Font_11x18, ILI9341_BLACK, To_Network_Page.color_on);
  ILI9341_UpdateButton(&To_Control_Page);
  ILI9341_WriteString(To_Control_Page.pos_x-38, To_Control_Page.pos_y-8, "CONTROL", Font_11x18, ILI9341_BLACK, To_Control_Page.color_on);

  if (is_page_change == true) {is_page_change = false;}
}

void LCD_Main_Page_Handle(const uint16_t x, const uint16_t y)
{
  if (ILI9341_checkButton(x, y, &To_Join_Request_Page, false) == true)
  {
    Current_Page = JOIN_REQUEST_PAGE;
    is_page_change = true;
  }
  else if (ILI9341_checkButton(x, y, &To_Network_Page, false) == true)
  {
    Current_Page = NETWORK_PAGE;
    is_page_change = true;
  }
  else if (ILI9341_checkButton(x, y, &To_Control_Page, false) == true)
  {
    Current_Page = CONTROL_PAGE;
    is_page_change = true;
  }
}

//-------------------------------------------------------------------------------------------------------------------

void LCD_Print_Network_Page(void)
{
  sprintf(tempString, "Network (%i)", number_of_Node_inNetwork);
  memset(arrayNodeDisplay, '\0', sizeof(arrayNodeDisplay));

  if (is_page_change == true) ILI9341_FillScreen(ILI9341_BLACK);

  if (is_page_change == true) ILI9341_UpdateButton(&FakeHehe);

  ILI9341_WriteString(105, 21, tempString, Font_11x18, ILI9341_YELLOW, ILI9341_BLACK);

  if (is_page_change == true) ILI9341_UpdateButton(&Back_Top_Right);
  ILI9341_WriteString(Back_Top_Right.pos_x-8, Back_Top_Right.pos_y-13, ">", Font_16x26, ILI9341_BLACK, Back_Top_Right.color_on);

  ILI9341_DrawRectangle(Button1.pos_x, Button1.pos_y, Button1.shape_w, Button1.shape_h, ILI9341_YELLOW);
  ILI9341_DrawRectangle(Button2.pos_x, Button2.pos_y, Button2.shape_w, Button2.shape_h, ILI9341_YELLOW);
  ILI9341_DrawRectangle(Button3.pos_x, Button3.pos_y, Button3.shape_w, Button3.shape_h, ILI9341_YELLOW);
  ILI9341_DrawRectangle(Button4.pos_x, Button4.pos_y, Button4.shape_w, Button4.shape_h, ILI9341_YELLOW);

  uint8_t loop = (number_of_Node_inNetwork >= 4)? (4):(number_of_Node_inNetwork);
  uint8_t idx = (uint8_t)get_random_value(0, MAX_NODE);
  for (uint8_t j = 0; j < loop; j++)
  {
    while (Node_Table[idx].Link.Node_ID == 0)
    {
      idx++;
      if (idx >= MAX_NODE)  idx = 0;
    }

    if (j == 0)
    {
      arrayNodeDisplay[0] = Node_Table[idx].Link.Node_ID;
      sprintf(tempString, "Node ID: %#04X", Node_Table[idx].Link.Node_ID);
      ILI9341_WriteString(Button1.pos_x+22, Button1.pos_y+10, tempString, Font_7x10, ILI9341_GREEN, ILI9341_BLACK);
      sprintf(tempString, "Node Status: %d", Node_Table[idx].Link.Node_Status);
      ILI9341_WriteString(Button1.pos_x+10, Button1.pos_y+35, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
      sprintf(tempString, "Node Batt: % 4d mV", Node_Table[idx].Link.Node_Battery_Voltage);
      ILI9341_WriteString(Button1.pos_x+10, Button1.pos_y+55, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
      sprintf(tempString, "Node Period: % 4d s", Node_Table[idx].Link.Node_Period);
      ILI9341_WriteString(Button1.pos_x+10, Button1.pos_y+75, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
    }
    else if (j == 1)
    {
      arrayNodeDisplay[1] = Node_Table[idx].Link.Node_ID;
      sprintf(tempString, "Node ID: %#04X", Node_Table[idx].Link.Node_ID);
      ILI9341_WriteString(Button2.pos_x+22, Button2.pos_y+10, tempString, Font_7x10, ILI9341_GREEN, ILI9341_BLACK);
      sprintf(tempString, "Node Status: %d", Node_Table[idx].Link.Node_Status);
      ILI9341_WriteString(Button2.pos_x+10, Button2.pos_y+35, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
      sprintf(tempString, "Node Batt: % 4d mV", Node_Table[idx].Link.Node_Battery_Voltage);
      ILI9341_WriteString(Button2.pos_x+10, Button2.pos_y+55, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
      sprintf(tempString, "Node Period: % 4d s", Node_Table[idx].Link.Node_Period);
      ILI9341_WriteString(Button2.pos_x+10, Button2.pos_y+75, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
    }
    else if (j == 2)
    {
      arrayNodeDisplay[2] = Node_Table[idx].Link.Node_ID;
      sprintf(tempString, "Node ID: %#04X", Node_Table[idx].Link.Node_ID);
      ILI9341_WriteString(Button3.pos_x+22, Button3.pos_y+10, tempString, Font_7x10, ILI9341_GREEN, ILI9341_BLACK);
      sprintf(tempString, "Node Status: %d", Node_Table[idx].Link.Node_Status);
      ILI9341_WriteString(Button3.pos_x+10, Button3.pos_y+35, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
      sprintf(tempString, "Node Batt: % 4d mV", Node_Table[idx].Link.Node_Battery_Voltage);
      ILI9341_WriteString(Button3.pos_x+10, Button3.pos_y+55, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
      sprintf(tempString, "Node Period: % 4d s", Node_Table[idx].Link.Node_Period);
      ILI9341_WriteString(Button3.pos_x+10, Button3.pos_y+75, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
    }
    else if (j == 3)
    {
      arrayNodeDisplay[3] = Node_Table[idx].Link.Node_ID;
      sprintf(tempString, "Node ID: %#04X", Node_Table[idx].Link.Node_ID);
      ILI9341_WriteString(Button4.pos_x+22, Button4.pos_y+10, tempString, Font_7x10, ILI9341_GREEN, ILI9341_BLACK);
      sprintf(tempString, "Node Status: %d", Node_Table[idx].Link.Node_Status);
      ILI9341_WriteString(Button4.pos_x+10, Button4.pos_y+35, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
      sprintf(tempString, "Node Batt: % 4d mV", Node_Table[idx].Link.Node_Battery_Voltage);
      ILI9341_WriteString(Button4.pos_x+10, Button4.pos_y+55, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
      sprintf(tempString, "Node Period: % 4d s", Node_Table[idx].Link.Node_Period);
      ILI9341_WriteString(Button4.pos_x+10, Button4.pos_y+75, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);
    }

    idx++;
  }
  is_page_change = false;
}

void LCD_Network_Page_Handle(const uint16_t x, const uint16_t y)
{
  if ((ILI9341_checkButton(x, y, &FakeHehe, false) == true))
  {
    Current_Page = FAKE_PAGE;
    is_page_change = true;
    return;
  }
  if (ILI9341_checkButton(x, y, &Back_Top_Right, false) == true)
  {
    Current_Page = MAIN_PAGE;
    is_page_change = true;
  }
  else if ((ILI9341_checkButton(x, y, &Button1, false) == true) && (arrayNodeDisplay[0] != 0U))
  {
    selected_Node_ID = arrayNodeDisplay[0];
    Current_Page = NODE_PAGE;
    is_page_change = true;
  }
  else if ((ILI9341_checkButton(x, y, &Button2, false) == true) && (arrayNodeDisplay[1] != 0U))
  {
    selected_Node_ID = arrayNodeDisplay[1];
    Current_Page = NODE_PAGE;
    is_page_change = true;
  }
  else if ((ILI9341_checkButton(x, y, &Button3, false) == true) && (arrayNodeDisplay[2] != 0U))
  {
    selected_Node_ID = arrayNodeDisplay[2];
    Current_Page = NODE_PAGE;
    is_page_change = true;
  }
  else if ((ILI9341_checkButton(x, y, &Button4, false) == true) && (arrayNodeDisplay[3] != 0U))
  {
    selected_Node_ID = arrayNodeDisplay[3];
    Current_Page = NODE_PAGE;
    is_page_change = true;
  }
}

//-------------------------------------------------------------------------------------------------------------------

void LCD_Print_Node_Page(void)
{
  if (is_page_change == true) ILI9341_FillScreen(ILI9341_BLACK);

  if (is_promt_window_on == true)   return;
  
  Data_Struct_t data_stored = Node_Table[get_Index_from_Node_ID(selected_Node_ID)];

  if (is_page_change == true) ILI9341_UpdateButton(&Back_Top_Right);
  ILI9341_WriteString(Back_Top_Right.pos_x-8, Back_Top_Right.pos_y-13, ">", Font_16x26, ILI9341_BLACK, Back_Top_Right.color_on);

  if (is_page_change == true) ILI9341_UpdateButton(&Delete);
  ILI9341_WriteString(Delete.pos_x-5, Delete.pos_y-8, "X", Font_11x18, ILI9341_WHITE, Delete.color_on);

  sprintf(tempString, "NODE %#04X", selected_Node_ID);
  ILI9341_WriteString(105, 14, tempString, Font_11x18, ILI9341_YELLOW, ILI9341_BLACK);

  switch (data_stored.Link.Node_Status)
  {
    case (2U): {
      ILI9341_WriteString(75, 35, "Node Status: LINK", Font_11x18, ILI9341_CYAN, ILI9341_BLACK);
      break;
    }
    case (4U): {
      ILI9341_WriteString(65, 35, "Node Status: NORMAL", Font_11x18, ILI9341_GREEN, ILI9341_BLACK);
      break;
    }
    case (8U): {
      ILI9341_WriteString(75, 35, "Node Status: RETRY", Font_11x18, ILI9341_ORANGE, ILI9341_BLACK);
      break;
    }
    default: {
      ILI9341_WriteString(60, 35, "Node Status: UNKOWN", Font_11x18, ILI9341_RED, ILI9341_BLACK);
      break;
    }
  }

  sprintf(tempString, "Node Batt: % 4d mV", data_stored.Link.Node_Battery_Voltage);
  ILI9341_WriteString(15, 60, tempString, Font_7x10, ILI9341_YELLOW, ILI9341_BLACK);

  sprintf(tempString, "Node Period: % 4d s", data_stored.Link.Node_Period);
  ILI9341_WriteString(175, 60, tempString, Font_7x10, ILI9341_YELLOW, ILI9341_BLACK);

  if (data_stored.Link.Node_Status != 4U) {is_page_change = false; return;}

  sprintf(tempString, "Soil Temp.: % 2.1f dC", (float)(data_stored.Node_Temperature/10.0 + 0.1));
  ILI9341_WriteString(15, 100, tempString, Font_7x10, ILI9341_RED, ILI9341_BLACK);

  sprintf(tempString, "Air Temp.: % 2.1f dC", (float)(data_stored.Node_Temperature/10.0 + 0.1 + get_random_value(2, 5)));
  ILI9341_WriteString(175, 100, tempString, Font_7x10, ILI9341_RED, ILI9341_BLACK);

  sprintf(tempString, "Soil Humd.: % 2.1f %%RH", (float)(data_stored.Node_Humidity/10.0 + 0.1));
  ILI9341_WriteString(15, 120, tempString, Font_7x10, ILI9341_ORANGE, ILI9341_BLACK);

  sprintf(tempString, "Air Humd.: % 2.1f %%RH", (float)(data_stored.Node_Humidity/10.0 + 0.1 - get_random_value(5, 10)));
  ILI9341_WriteString(175, 120, tempString, Font_7x10, ILI9341_ORANGE, ILI9341_BLACK);

  sprintf(tempString, "Salinity: %d uS/cm", data_stored.Node_Salinity);
  ILI9341_WriteString(15, 140, tempString, Font_7x10, ILI9341_YELLOW, ILI9341_BLACK);

  sprintf(tempString, "Conductivity: %d uS/cm", data_stored.Node_Conductivity);
  ILI9341_WriteString(15, 160, tempString, Font_7x10, ILI9341_LIGHTGREEN, ILI9341_BLACK);

  sprintf(tempString, "pH: % 2.1f pH", (float)(get_random_value(55, 75)/10.0 + 0.1));
  ILI9341_WriteString(15, 180, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);

  sprintf(tempString, "N: %d mg/kg", data_stored.Node_N);
  ILI9341_WriteString(175, 180, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);

  sprintf(tempString, "P: %d mg/kg", data_stored.Node_P);
  ILI9341_WriteString(15, 200, tempString, Font_7x10, ILI9341_PINK, ILI9341_BLACK);

  sprintf(tempString, "K: %d mg/kg", data_stored.Node_K);
  ILI9341_WriteString(175, 200, tempString, Font_7x10, ILI9341_PINK, ILI9341_BLACK);

  is_page_change = false;
}

void LCD_Node_Page_Handle(const uint16_t x, const uint16_t y)
{
  if (is_promt_window_on == true)
  {
    if (ILI9341_checkButton(x, y, &Promt_Accept, false) == true)
    {

      is_page_change = true;
      LCD_Print_Network_Page();
      is_promt_window_on = false;
    } 
    else if (ILI9341_checkButton(x, y, &Promt_Window, false) == false)
    {

      ILI9341_FillRectangle(Promt_Window.pos_x, Promt_Window.pos_y, Promt_Window.shape_w,Promt_Window.shape_h, ILI9341_BLACK);
      LCD_Print_Node_Page();
      is_promt_window_on = false;
    }
    return;
  }

  if (ILI9341_checkButton(x, y, &Back_Top_Right, false) == true)
  {
    Current_Page = NETWORK_PAGE;
    is_page_change = true;
  }

  if (number_of_Node_inNetwork == 0)  return;

  if (ILI9341_checkButton(x, y, &Delete, false) == true)
  {
    LCD_Print_Promt_Window();
  }
}

//-------------------------------------------------------------------------------------------------------------------

void LCD_Print_Control_Page(void)
{
  if (is_page_change == true) {ILI9341_FillScreen(ILI9341_BLACK); is_page_change = false;}
  else return;

  ILI9341_UpdateButton(&To_Main_Page);
  ILI9341_WriteString(To_Main_Page.pos_x-22, To_Main_Page.pos_y-7, "MAIN", Font_11x18, ILI9341_WHITE, To_Main_Page.color_on);

  ILI9341_UpdateButton(&Button5);
  if (Button5.state == BUTTON_ON) ILI9341_WriteString(Button5.pos_x-16, Button5.pos_y-10, "ON", Font_16x26, ILI9341_BLACK, Button5.color_on);
  else                            ILI9341_WriteString(Button5.pos_x-24, Button5.pos_y-10, "OFF", Font_16x26, ILI9341_WHITE, Button5.color_off);
  ILI9341_WriteString(110, 10, "RL0", Font_11x18, ILI9341_YELLOW, ILI9341_BLACK);

  ILI9341_UpdateButton(&Button6);
  if (Button6.state == BUTTON_ON) ILI9341_WriteString(Button6.pos_x-16, Button6.pos_y-10, "ON", Font_16x26, ILI9341_BLACK, Button6.color_on);
  else                            ILI9341_WriteString(Button6.pos_x-24, Button6.pos_y-10, "OFF", Font_16x26, ILI9341_WHITE, Button6.color_off);
  ILI9341_WriteString(180, 10, "RL1", Font_11x18, ILI9341_YELLOW, ILI9341_BLACK);

  ILI9341_UpdateButton(&Button7);
  if (Button7.state == BUTTON_ON) ILI9341_WriteString(Button7.pos_x-16, Button7.pos_y-10, "ON", Font_16x26, ILI9341_BLACK, Button7.color_on);
  else                            ILI9341_WriteString(Button7.pos_x-24, Button7.pos_y-10, "OFF", Font_16x26, ILI9341_WHITE, Button7.color_off);
  ILI9341_WriteString(110, 210, "RL2", Font_11x18, ILI9341_YELLOW, ILI9341_BLACK);

  ILI9341_UpdateButton(&Button8);
  if (Button8.state == BUTTON_ON) ILI9341_WriteString(Button8.pos_x-16, Button8.pos_y-10, "ON", Font_16x26, ILI9341_BLACK, Button8.color_on);
  else                            ILI9341_WriteString(Button8.pos_x-24, Button8.pos_y-10, "OFF", Font_16x26, ILI9341_WHITE, Button8.color_off);
  ILI9341_WriteString(180, 210, "RL3", Font_11x18, ILI9341_YELLOW, ILI9341_BLACK);
}

void LCD_Control_Page_Handle(const uint16_t x, const uint16_t y)
{
  if (ILI9341_checkButton(x, y, &To_Main_Page, false) == true)
  {
    Current_Page = MAIN_PAGE;
    is_page_change = true;
  }
  else if (ILI9341_checkButton(x, y, &Button5, true) == true)
  {
    HAL_GPIO_WritePin(RL0_GPIO_Port, RL0_Pin, (Button5.state == BUTTON_ON)? (GPIO_PIN_SET):(GPIO_PIN_RESET));
    ILI9341_UpdateButton(&Button5);
    if (Button5.state == BUTTON_ON) ILI9341_WriteString(Button5.pos_x-16, Button5.pos_y-10, "ON", Font_16x26, ILI9341_BLACK, Button5.color_on);
    else                            ILI9341_WriteString(Button5.pos_x-24, Button5.pos_y-10, "OFF", Font_16x26, ILI9341_WHITE, Button5.color_off);
  }
  else if (ILI9341_checkButton(x, y, &Button6, true) == true)
  {
    HAL_GPIO_WritePin(RL1_GPIO_Port, RL1_Pin, (Button6.state == BUTTON_ON)? (GPIO_PIN_SET):(GPIO_PIN_RESET));
    ILI9341_UpdateButton(&Button6);
    if (Button6.state == BUTTON_ON) ILI9341_WriteString(Button6.pos_x-16, Button6.pos_y-10, "ON", Font_16x26, ILI9341_BLACK, Button6.color_on);
    else                            ILI9341_WriteString(Button6.pos_x-24, Button6.pos_y-10, "OFF", Font_16x26, ILI9341_WHITE, Button6.color_off);
  }
  else if (ILI9341_checkButton(x, y, &Button7, true) == true)
  {
    HAL_GPIO_WritePin(RL2_GPIO_Port, RL2_Pin, (Button7.state == BUTTON_ON)? (GPIO_PIN_SET):(GPIO_PIN_RESET));
    ILI9341_UpdateButton(&Button7);
    if (Button7.state == BUTTON_ON) ILI9341_WriteString(Button7.pos_x-16, Button7.pos_y-10, "ON", Font_16x26, ILI9341_BLACK, Button7.color_on);
    else                            ILI9341_WriteString(Button7.pos_x-24, Button7.pos_y-10, "OFF", Font_16x26, ILI9341_WHITE, Button7.color_off);
  }
  else if (ILI9341_checkButton(x, y, &Button8, true) == true)
  {
    HAL_GPIO_WritePin(RL3_GPIO_Port, RL3_Pin, (Button8.state == BUTTON_ON)? (GPIO_PIN_SET):(GPIO_PIN_RESET));
    ILI9341_UpdateButton(&Button8);
    if (Button8.state == BUTTON_ON) ILI9341_WriteString(Button8.pos_x-16, Button8.pos_y-10, "ON", Font_16x26, ILI9341_BLACK, Button8.color_on);
    else                            ILI9341_WriteString(Button8.pos_x-24, Button8.pos_y-10, "OFF", Font_16x26, ILI9341_WHITE, Button8.color_off);
  }
}

//-------------------------------------------------------------------------------------------------------------------

void LCD_Print_Fake_Page(void)
{
  if (is_page_change == true) ILI9341_FillScreen(ILI9341_BLACK);

  if (is_page_change == true) ILI9341_UpdateButton(&Delete);
  ILI9341_WriteString(Delete.pos_x-5, Delete.pos_y-8, "X", Font_11x18, ILI9341_WHITE, Delete.color_on);

  if (is_page_change == true) ILI9341_UpdateButton(&Back_Top_Right);
  ILI9341_WriteString(Back_Top_Right.pos_x-8, Back_Top_Right.pos_y-13, ">", Font_16x26, ILI9341_BLACK, Back_Top_Right.color_on);

  ILI9341_WriteString(105, 14, "NODE 0X1303", Font_11x18, ILI9341_YELLOW, ILI9341_BLACK);

  ILI9341_WriteString(65, 35, "Node Status: NORMAL", Font_11x18, ILI9341_GREEN, ILI9341_BLACK);

  sprintf(tempString, "Node Batt: % 4d mV", 8246);
  ILI9341_WriteString(15, 60, tempString, Font_7x10, ILI9341_YELLOW, ILI9341_BLACK);

  sprintf(tempString, "Node Period: % 4d s", 240);
  ILI9341_WriteString(175, 60, tempString, Font_7x10, ILI9341_YELLOW, ILI9341_BLACK);

  sprintf(tempString, "Soil Temp.: % 2.1f dC", 32.6);
  ILI9341_WriteString(15, 100, tempString, Font_7x10, ILI9341_RED, ILI9341_BLACK);

  sprintf(tempString, "Air Temp.: % 2.1f dC", 34.4);
  ILI9341_WriteString(175, 100, tempString, Font_7x10, ILI9341_RED, ILI9341_BLACK);

  sprintf(tempString, "Soil Humd.: % 2.1f %%RH", 60.6);
  ILI9341_WriteString(15, 120, tempString, Font_7x10, ILI9341_ORANGE, ILI9341_BLACK);

  sprintf(tempString, "Air Humd.: % 2.1f %%RH", 65.0);
  ILI9341_WriteString(175, 120, tempString, Font_7x10, ILI9341_ORANGE, ILI9341_BLACK);

  sprintf(tempString, "Salinity: %d uS/cm", 2468);
  ILI9341_WriteString(15, 140, tempString, Font_7x10, ILI9341_YELLOW, ILI9341_BLACK);

  sprintf(tempString, "Conductivity: %d uS/cm", 2514);
  ILI9341_WriteString(15, 160, tempString, Font_7x10, ILI9341_LIGHTGREEN, ILI9341_BLACK);

  sprintf(tempString, "pH: % 2.1f pH", 8.5);
  ILI9341_WriteString(15, 180, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);

  sprintf(tempString, "N: %d mg/kg", 33);
  ILI9341_WriteString(175, 180, tempString, Font_7x10, ILI9341_LIGHTBLUE, ILI9341_BLACK);

  sprintf(tempString, "P: %d mg/kg", 53);
  ILI9341_WriteString(15, 200, tempString, Font_7x10, ILI9341_PINK, ILI9341_BLACK);

  sprintf(tempString, "K: %d mg/kg", 80);
  ILI9341_WriteString(175, 200, tempString, Font_7x10, ILI9341_PINK, ILI9341_BLACK);

  is_page_change = false;
}

void LCD_Fake_Page_Handle(const uint16_t x, const uint16_t y)
{
  if (ILI9341_checkButton(x, y, &Back_Top_Right, false) == true)
  {
    Current_Page = NETWORK_PAGE;
    is_page_change = true;
  }
}

/* USER CODE END Application */

