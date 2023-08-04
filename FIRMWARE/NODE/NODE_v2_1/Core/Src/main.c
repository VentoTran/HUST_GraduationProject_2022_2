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
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "stdbool.h"
#include "stdlib.h"
#include "common.h"
#include "sx1278.h"
#include "string.h"

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

typedef struct SoilData
{
  uint16_t rawTemperature;
  uint16_t rawHumidity;
  float Temperature;
  float Humidity;

  uint16_t Salinity;
  uint16_t Conductivity;

  uint16_t rawPH;
  float pH;

  uint16_t N;
  uint16_t P;
  uint16_t K;
} SoilData_t;

typedef enum NodeStatus
{
  WAKEUP_MODE = 1U,
  LINK_MODE = 2U,
  NORMAL_MODE = 4U,
  RETRY_MODE = 8U,
  SHUTDOWN_MODE = 16U
} NodeStatus_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NODE_ID                     (0x3112U)

#define TIMEOUT_MS                  (50U)
#define LINK_PACKET_ID              (0xAAAAU)
#define DATA_PACKET_ID              (0x5A5AU)
#define RESPONSE_PACKET_ID          (0x5555U)

#define SLEEPTIME_NORMAL_MODE_MIN   (3U)
#define SLEEPTIME_LINK_MODE_MIN     (1U)
#define SLEEPTIME_RETRY_MODE_SEC    (15U)

#define READ_SENSORS_DURATION_MS    (5000U)

#define WAIT_RESPONSE_DURATION_MS   (2000U)

#define NODE_WAKEUP_BIT_SHIFT       (0U)
#define NODE_WAKEUP_BIT             (1U << NODE_WAKEUP_BIT_SHIFT)
#define NODE_LINK_MODE_BIT_SHIFT    (1U)
#define NODE_LINK_MODE_BIT          (1U << NODE_LINK_MODE_BIT_SHIFT)
#define NODE_NORMAL_MODE_BIT_SHIFT  (2U)
#define NODE_NORMAL_MODE_BIT        (1U << NODE_NORMAL_MODE_BIT_SHIFT)
#define NODE_RETRY_MODE_BIT_SHIFT   (3U)
#define NODE_RETRY_MODE_BIT         (1U << NODE_RETRY_MODE_BIT_SHIFT)

#define NODE_SHUTDOWN_MODE_BIT_SHIFT   (4U)
#define NODE_SHUTDOWN_MODE_BIT         (1U << NODE_RETRY_MODE_BIT_SHIFT)

#define LINK_ACCEPT   (0xAAU)
#define LINK_REJECT   (0x55U)

#define LINK_CARRYON  (0x00U)
#define LINK_DISMISS  (0xFFU)

#define LINK_ACK      (0xABU)
#define LINK_NACK     (0xBAU)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

bool readSensor(void);

uint16_t CRC16(unsigned char *buf, int len);

void goingDark(void);
void sleep4ever(void);

void Link_Mode_Handle(void);
void Normal_Mode_Handle(void);
void Retry_Mode_Handle(void);

void Response_Handle(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static SoilData_t mySoil;
static NodeStatus_t myStatus;
static uint32_t backupStorage = 0U;
static uint8_t payload[100] = {0};
static volatile bool is_LoRa_EXTI = false;
static uint8_t array2store[28] = {0};

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  myStatus = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);

  if (myStatus == 0U)
  {
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, SLEEPTIME_NORMAL_MODE_MIN * 60);
    myStatus = LINK_MODE;
    // HAL_Delay(20000);
  }

  if ((myStatus & LINK_MODE) != 0U)
  {
    Link_Mode_Handle();
  }
  else if ((myStatus & NORMAL_MODE) != 0U)
  {
    Normal_Mode_Handle();
  }
  else if ((myStatus & RETRY_MODE) != 0U)
  {
    Retry_Mode_Handle();
  }

  while (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2) != myStatus)  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, myStatus);

  if ((myStatus & SHUTDOWN_MODE) != 0U)
  {
    sleep4ever();
    while (1);
  }

  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  HAL_Delay(100);
  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);

  goingDark();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

uint16_t CRC16(unsigned char *buf, int len)
{
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++)
  {
    // XOR byte into least sig. byte of crc
    crc ^= (unsigned int)buf[pos];

    for (int i = 8; i != 0; i--)
    {
      // Loop over each bit
      if ((crc & 0x0001) != 0)
      {
        // If the LSB is set => Shift right and XOR 0xA001
        crc >>= 1;
        crc ^= 0xA001;
      }
      else
      {
        // Else LSB is not set => Just shift right
        crc >>= 1;
      }
    }
  }

  return crc;
}

bool readSensor(void)
{
  static uint8_t txbuffer[10] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x08, 0x44, 0x0C};
  static uint8_t rxbuffer[25];

  HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RE_GPIO_Port, RE_Pin, GPIO_PIN_SET);

  HAL_UART_Transmit(&huart1, txbuffer, 8, 10);

  HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RE_GPIO_Port, RE_Pin, GPIO_PIN_RESET);

  HAL_UART_Receive(&huart1, rxbuffer, 21, 200);

  mySoil.rawTemperature = (uint16_t)((rxbuffer[3] << 8U) | (uint16_t)rxbuffer[4]);
  mySoil.Temperature = (float)((float)mySoil.rawTemperature / 10.0);
  mySoil.rawHumidity = (uint16_t)((rxbuffer[5] << 8U) | (uint16_t)rxbuffer[6]);
  mySoil.Humidity = (float)((float)mySoil.rawHumidity / 10.0);
  mySoil.Salinity = (uint16_t)((rxbuffer[7] << 8U) | (uint16_t)rxbuffer[8]);
  mySoil.Conductivity = (uint16_t)((rxbuffer[9] << 8U) | (uint16_t)rxbuffer[10]);
  mySoil.rawPH = (uint16_t)((rxbuffer[11] << 8U) | (uint16_t)rxbuffer[12]);
  mySoil.pH = (float)((float)mySoil.rawPH / 100.0);
  mySoil.N = (uint16_t)((rxbuffer[13] << 8U) | (uint16_t)rxbuffer[14]);
  mySoil.P = (uint16_t)((rxbuffer[15] << 8U) | (uint16_t)rxbuffer[16]);
  mySoil.K = (uint16_t)((rxbuffer[17] << 8U) | (uint16_t)rxbuffer[18]);

  uint16_t calculatedCRC = CRC16(rxbuffer, 19);

  if (((uint8_t)(calculatedCRC & 0x00FF) != (uint8_t)rxbuffer[19]) || ((uint8_t)((uint16_t)(calculatedCRC & 0xFF00) >> 8) != (uint8_t)rxbuffer[20]))
  {
    memset(&mySoil, '\0', sizeof(mySoil));
    return false;
  }

  memset(rxbuffer, '\0', sizeof(rxbuffer));
  return true;
}

void goingDark(void)
{
  /* Enable WUPIN PA0 */
  SET_BIT(PWR->CSR, PWR_CSR_EWUP);

  /* Set RTC Alarm for 300s */
  RTC_TimeTypeDef timeNow;
  RTC_AlarmTypeDef timeAlarm;
  HAL_RTC_GetTime(&hrtc, &timeNow, RTC_FORMAT_BCD);
  timeAlarm.Alarm = RTC_ALARM_A;
  timeAlarm.AlarmTime = timeNow;
  if (myStatus == NORMAL_MODE)
  {
    timeAlarm.AlarmTime.Seconds += HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3);
  }
  else if (myStatus == LINK_MODE)
  {
    timeAlarm.AlarmTime.Minutes += SLEEPTIME_LINK_MODE_MIN;
  }
  else if (myStatus == RETRY_MODE)
  {
    timeAlarm.AlarmTime.Seconds += SLEEPTIME_RETRY_MODE_SEC;
  }
  __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);
  HAL_RTC_SetAlarm_IT(&hrtc, &timeAlarm, RTC_FORMAT_BCD);

  /* Set SLEEPDEEP bit of Cortex System Control Register */
  SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));

  /* Select Standby mode */
  SET_BIT(PWR->CR, PWR_CR_PDDS);

  /* Clear WUF bit in Power Control/Status register */
  SET_BIT(PWR->CR, PWR_CR_CWUF);

  /* Clear SBF bit in Power Control/Status register */
  SET_BIT(PWR->CR, PWR_CR_CSBF);

  (void)PWR->CR;
  (void)PWR->CSR;

  // Disable debug, trace and IWDG in low-power modes
  DBGMCU->CR = (uint32_t)0x00;

  for (;;)
  {
    __DSB();
    __WFI();
  }
}

void sleep4ever(void)
{
  /* Enable WUPIN PA0 */
  SET_BIT(PWR->CSR, PWR_CSR_EWUP);

  __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);

  /* Set SLEEPDEEP bit of Cortex System Control Register */
  SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));

  /* Select Standby mode */
  SET_BIT(PWR->CR, PWR_CR_PDDS);

  /* Clear WUF bit in Power Control/Status register */
  SET_BIT(PWR->CR, PWR_CR_CWUF);

  /* Clear SBF bit in Power Control/Status register */
  SET_BIT(PWR->CR, PWR_CR_CSBF);

  (void)PWR->CR;
  (void)PWR->CSR;

  // Disable debug, trace and IWDG in low-power modes
  DBGMCU->CR = (uint32_t)0x00;

  for (;;)
  {
    __DSB();
    __WFI();
  }
}

bool is_OK_2_Talk(void)
{
  sx1278_standby();
  uint32_t checkTimeout = HAL_GetTick();
  uint8_t irq = sx1278_read_reg(REG_IRQ_FLAGS);
  sx1278_write_reg(REG_IRQ_FLAGS, irq);     //reset irq
  is_LoRa_EXTI = false;
  sx1278_cad();
  while ((is_LoRa_EXTI == false) && ((HAL_GetTick() - checkTimeout) < TIMEOUT_MS));
  if (is_LoRa_EXTI == false)  return false;
  irq = sx1278_read_reg(REG_IRQ_FLAGS);
  sx1278_write_reg(REG_IRQ_FLAGS, irq);     //reset irq
  sx1278_standby();
  if (((irq & 0x01U) != 0U) && ((irq & 0x04U) != 0U))
  {
    return false;
  }
  else if (((irq & 0x01U) == 0U) && ((irq & 0x04U) != 0U))
  {
    return true;
  }
  return false;
}

void Link_Mode_Handle(void)
{
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 200);
  backupStorage = HAL_ADC_GetValue(&hadc1);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, backupStorage);
  HAL_ADC_Stop(&hadc1);
  HAL_ADC_DeInit(&hadc1);

  HAL_I2C_DeInit(&hi2c1);
  HAL_UART_DeInit(&huart1);

  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);

  sx1278_init();

  Link_Packet_t LinkReq;
  LinkReq.Packet_ID = LINK_PACKET_ID;
  LinkReq.Payload.Node_ID = NODE_ID;
  LinkReq.Payload.Node_Status = myStatus;
  LinkReq.Payload.Node_Battery_Voltage = (uint16_t)((double)backupStorage  * 2847.868f  * 3.3f / 4095.0f) - 70U;
  LinkReq.Payload.Node_Period = (uint16_t)(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3));
  memcpy((uint8_t*)&payload, (uint8_t*)&LinkReq, sizeof(Link_Packet_t));
  
  uint8_t nTry = 0;
  while ((is_OK_2_Talk() == false) && (nTry < 10))
  {
    nTry++;
    HAL_Delay(TIMEOUT_MS);
  }
  if (nTry == 10)
  {
    myStatus = LINK_MODE;
    return;
  }

  sx1278_send_data((uint8_t *)&payload, sizeof(Link_Packet_t));

  Response_Handle();

  sx1278_sleep();
  HAL_SPI_DeInit(&hspi1);

  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
}

void Normal_Mode_Handle(void)
{
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, 200);
  backupStorage = HAL_ADC_GetValue(&hadc1);
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, backupStorage);
  HAL_ADC_Stop(&hadc1);
  HAL_ADC_DeInit(&hadc1);

  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PWR_CTRL_GPIO_Port, PWR_CTRL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  HAL_Delay(READ_SENSORS_DURATION_MS);
  readSensor();

  HAL_GPIO_WritePin(PWR_CTRL_GPIO_Port, PWR_CTRL_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  HAL_I2C_DeInit(&hi2c1);
  HAL_UART_DeInit(&huart1);

  sx1278_init();

  Data_Packet_t data_packet;
  data_packet.Packet_ID = DATA_PACKET_ID;
  data_packet.Payload.Link.Node_ID = NODE_ID;
  data_packet.Payload.Link.Node_Status = myStatus;
  data_packet.Payload.Link.Node_Battery_Voltage = (uint16_t)((double)backupStorage  * 2847.868f  * 3.38f / 4095.0f) - 70U;
  data_packet.Payload.Link.Node_Period = (uint16_t)(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3));
  data_packet.Payload.Node_Temperature = mySoil.rawTemperature;
  data_packet.Payload.Node_Humidity = mySoil.rawHumidity;
  data_packet.Payload.Node_Salinity = mySoil.Salinity;
  data_packet.Payload.Node_Conductivity = mySoil.Conductivity;
  data_packet.Payload.Node_pH = mySoil.rawPH;
  data_packet.Payload.Node_N = mySoil.N;
  data_packet.Payload.Node_P = mySoil.P;
  data_packet.Payload.Node_K = mySoil.K;
  memset(payload, '\0', sizeof(payload));
  memcpy((uint8_t *)payload, (uint8_t *)&data_packet, sizeof(Data_Packet_t));
  
  uint8_t nTry = 0;
  while ((is_OK_2_Talk() == false) && (nTry < 10))
  {
    nTry++;
    HAL_Delay(TIMEOUT_MS);
  }
  if (nTry == 10)
  {
    myStatus = RETRY_MODE;
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
    return;
  }

  sx1278_send_data(payload, sizeof(Data_Packet_t));

  Response_Handle();

  if (myStatus == RETRY_MODE)
  {
    memset(array2store, '\0', sizeof(array2store));
    memcpy((uint8_t *)&array2store, (uint8_t *)&data_packet, sizeof(Data_Packet_t));
    Flash_Write_Data(0x08006000, (uint32_t*)&array2store, (sizeof(array2store)/4));
  }

  sx1278_sleep();
  HAL_SPI_DeInit(&hspi1);

  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
}

void Retry_Mode_Handle(void)
{
  HAL_ADC_DeInit(&hadc1);
  HAL_I2C_DeInit(&hi2c1);
  HAL_UART_DeInit(&huart1);
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_RESET);

  sx1278_init();
  Flash_Read_Data(0x08006000, (uint32_t *)&array2store, sizeof(array2store));
  memcpy((uint8_t *)payload, (uint8_t *)&array2store, sizeof(Data_Packet_t));
  
  uint8_t nTry = 0;
  while ((is_OK_2_Talk() == false) && (nTry < 10))
  {
    nTry++;
    HAL_Delay(TIMEOUT_MS);
  }
  if (nTry == 10)
  {
    myStatus = RETRY_MODE;
    HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
    return;
  }

  sx1278_send_data(payload, sizeof(Data_Packet_t));

  Response_Handle();

  sx1278_sleep();
  HAL_SPI_DeInit(&hspi1);

  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
}

void Response_Handle(void)
{
  static ResponsePacket_t resp;
  static uint8_t data[100] = {0};
  static uint32_t nByteRx = 0;
  static int rssi = -1;
  static float snr = -1;

  is_LoRa_EXTI = false;
  sx1278_start_recv_data();

  uint32_t timeOut = HAL_GetTick();
  while ((HAL_GetTick() - timeOut) < 5000U)
  {
    if (is_LoRa_EXTI == true)
    {
      if (sx1278_recv_data((uint8_t *)data, &nByteRx, &rssi, &snr, false) == SX1278_OK)
      {
        if (nByteRx == sizeof(ResponsePacket_t))  memcpy(&resp, &data, sizeof(ResponsePacket_t));
        else
        {
          is_LoRa_EXTI = false;
          sx1278_set_irq(0x00);
          sx1278_write_reg(REG_IRQ_FLAGS, sx1278_read_reg(REG_IRQ_FLAGS));
          continue;
        }
        
        if ((resp.Packet_ID != RESPONSE_PACKET_ID) || (resp.Target_Node_ID != NODE_ID))
        {
          // this is not what I want OR this is not for me
          is_LoRa_EXTI = false;
          sx1278_start_recv_data();
          continue;
        }
        else
        {
          // Yep ok this is my packet and ID
          if (myStatus == LINK_MODE)
          {
            if ((resp.Target_Node_Response & 0x00FF) != LINK_ACK) {myStatus = LINK_MODE; return;}
            if      (((resp.Target_Node_Response >> 8) & 0x00FF) == LINK_ACCEPT)    myStatus = NORMAL_MODE;
            else if (((resp.Target_Node_Response >> 8) & 0x00FF) == LINK_REJECT)    myStatus = (uint8_t)resp.Target_Node_Status;
          }
          else if (myStatus == NORMAL_MODE)
          {
            if ((resp.Target_Node_Response & 0x00FF) != LINK_ACK) {myStatus = RETRY_MODE; return;}
            if      (((resp.Target_Node_Response >> 8) & 0x00FF) == LINK_CARRYON)   myStatus = NORMAL_MODE;
            else if (((resp.Target_Node_Response >> 8) & 0x00FF) == LINK_DISMISS)   myStatus = LINK_MODE;
          }
          else if (myStatus == RETRY_MODE)
          {
            if      ((resp.Target_Node_Response & 0x00FF) != LINK_ACK) {myStatus = RETRY_MODE;  return;}
            else if ((resp.Target_Node_Response & 0x00FF) == LINK_ACK) {myStatus = NORMAL_MODE;}
            if      (((resp.Target_Node_Response >> 8) & 0x00FF) == LINK_CARRYON)   myStatus = NORMAL_MODE;
            else if (((resp.Target_Node_Response >> 8) & 0x00FF) == LINK_DISMISS)   myStatus = LINK_MODE;
          }
          HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, resp.Target_Node_Period);
          return;
        }
      }
      else
      {
        is_LoRa_EXTI = false;
        sx1278_set_irq(0x00);
        sx1278_write_reg(REG_IRQ_FLAGS, sx1278_read_reg(REG_IRQ_FLAGS));
        continue;
      }
    }
  }
  
  if (myStatus == LINK_MODE)
  {
    myStatus = LINK_MODE;
  }
  else if (myStatus == NORMAL_MODE)
  {
    myStatus = RETRY_MODE;
  }
  else if (myStatus == RETRY_MODE)
  {
    myStatus = RETRY_MODE;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if ((GPIO_Pin == LoRa_EXT0_Pin) && (HAL_GPIO_ReadPin(LoRa_EXT0_GPIO_Port, LoRa_EXT0_Pin) == GPIO_PIN_SET) && (is_LoRa_EXTI == false))
  {
    is_LoRa_EXTI = true;
  }
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
