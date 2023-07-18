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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SLEEPTIME_MIN (1U)

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static SoilData_t mySoil;

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
#ifdef USE_USB
#if DEBUG == 1
    GPIO_USB_INIT();
    MX_USB_DEVICE_Init();
#endif /* DEBUG == 1 */
#endif /* USE_USB */
       /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();
    MX_RTC_Init();
    /* USER CODE BEGIN 2 */

    /* check FLAGs */
    // if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) == RESET)
    // {
    // }
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 200);

    uint32_t backupStorage = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);

    sx1278_init();

    static uint8_t msg[50] = {0};

    for (int i = 0; i < 1; i++)
    {
        sprintf((char *)msg, "Hello from Node! This is a random number %d!", (int)(HAL_ADC_GetValue(&hadc1) % backupStorage));
        sx1278_send_data(msg, strlen((const char *)msg));
        HAL_Delay(2000);
    }

    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, backupStorage + 1);
    HAL_ADC_Stop(&hadc1);
    HAL_ADC_DeInit(&hadc1);

    sx1278_sleep();

    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

    HAL_GPIO_WritePin(PWR_CTRL_GPIO_Port, PWR_CTRL_Pin, GPIO_PIN_SET);
    HAL_Delay(3000);
    readSensor();
    HAL_GPIO_WritePin(PWR_CTRL_GPIO_Port, PWR_CTRL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    HAL_Delay(1000);

    // uint32_t timeKeeper = HAL_GetTick();

#if DEBUG == 1
    logPC("This is the end of setup, now entering while loop!");
#endif /* DEBUG == 1 */

    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    HAL_Delay(500);
    HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    HAL_Delay(500);
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
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_ADC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
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

    // memcpy(&mySoil[i].rawTemperature, &rxbuffer[3], 2U);
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

    if (((uint8_t)(calculatedCRC & 0x00FF) != (uint8_t)rxbuffer[19]) ||
        ((uint8_t)((uint16_t)(calculatedCRC & 0xFF00) >> 8) != (uint8_t)rxbuffer[20]))
    {
        memset(&mySoil, '\0', sizeof(mySoil));
#if DEBUG == 1
        logPC("Checksum Failed! Calculated: 0x%04X\tReceived: 0x%02X%02X\r\n", calculatedCRC, rxbuffer[20], rxbuffer[19]);
#endif /* DEBUG == 1 */
        return false;
    }

    // HAL_Delay(500);
#if DEBUG == 1
    logPC("Checksum Passed! Calculated: 0x%04X\tReceived: 0x%02X%02X\r\n", calculatedCRC, rxbuffer[20], rxbuffer[19]);
    rxbuffer[21] = '\r';
    rxbuffer[22] = '\n';
    logPC("Raw Data Read:\n");
    for (int j = 0; j < 21; j++)
        logPC("0x%02X ", rxbuffer[j]);
    logPC("%c%c", rxbuffer[21], rxbuffer[22]);
    logPC("Temp: %02.1f °C\nHumd: %02.1f %%RH\n", mySoil.Temperature, mySoil.Humidity);
    logPC("Salinity: %4d uS/cm\nConducticity: %4d uS/cm\n", mySoil.Salinity, mySoil.Conductivity);
    logPC("pH: %1.2f\nN: %4d mg/kg\nP: %4d mg/kg\nK: %4d mg/kg\n\n", mySoil.pH, mySoil.N, mySoil.P, mySoil.K);
#endif /* DEBUG == 1 */
    memset(rxbuffer, '\0', sizeof(rxbuffer));
    // HAL_Delay(100);
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
    timeAlarm.AlarmTime.Minutes += SLEEPTIME_MIN;
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

#if DEBUG == 1
    logPC("Bravo 6\nGoing Dark\n");
#endif /* DEBUG == 1 */

    for (;;)
    {
        __DSB();
        __WFI();
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

#ifdef USE_FULL_ASSERT
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
