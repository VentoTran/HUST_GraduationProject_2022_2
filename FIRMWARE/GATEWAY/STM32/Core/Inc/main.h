/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal_cortex.h"

#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void App_Refresh_Tick(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED0_Pin GPIO_PIN_13
#define LED0_GPIO_Port GPIOC
#define RL0_Pin GPIO_PIN_0
#define RL0_GPIO_Port GPIOC
#define RL1_Pin GPIO_PIN_1
#define RL1_GPIO_Port GPIOC
#define RL2_Pin GPIO_PIN_2
#define RL2_GPIO_Port GPIOC
#define RL3_Pin GPIO_PIN_3
#define RL3_GPIO_Port GPIOC
#define SIM_PWRKEY_Pin GPIO_PIN_0
#define SIM_PWRKEY_GPIO_Port GPIOA
#define SIM_RST_Pin GPIO_PIN_1
#define SIM_RST_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define LoRa_RST_Pin GPIO_PIN_4
#define LoRa_RST_GPIO_Port GPIOC
#define LoRa_EXT0_Pin GPIO_PIN_5
#define LoRa_EXT0_GPIO_Port GPIOC
#define LoRa_EXT0_EXTI_IRQn EXTI9_5_IRQn
#define D8_Pin GPIO_PIN_0
#define D8_GPIO_Port GPIOB
#define D10_Pin GPIO_PIN_1
#define D10_GPIO_Port GPIOB
#define D9_Pin GPIO_PIN_2
#define D9_GPIO_Port GPIOB
#define WR_Pin GPIO_PIN_10
#define WR_GPIO_Port GPIOB
#define RD_Pin GPIO_PIN_11
#define RD_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define Touch_EXT_Pin GPIO_PIN_6
#define Touch_EXT_GPIO_Port GPIOC
#define Touch_EXT_EXTI_IRQn EXTI9_5_IRQn
#define LoRa_EXT1_Pin GPIO_PIN_7
#define LoRa_EXT1_GPIO_Port GPIOC
#define LoRa_EXT2_Pin GPIO_PIN_8
#define LoRa_EXT2_GPIO_Port GPIOC
#define LoRa_EXT3_Pin GPIO_PIN_9
#define LoRa_EXT3_GPIO_Port GPIOC
#define LoRa_EXT4_Pin GPIO_PIN_8
#define LoRa_EXT4_GPIO_Port GPIOA
#define LoRa_EXT5_Pin GPIO_PIN_11
#define LoRa_EXT5_GPIO_Port GPIOA
#define SIM_PWR_EN_Pin GPIO_PIN_12
#define SIM_PWR_EN_GPIO_Port GPIOA
#define ESP_EN_Pin GPIO_PIN_15
#define ESP_EN_GPIO_Port GPIOA
#define RST_Pin GPIO_PIN_12
#define RST_GPIO_Port GPIOC
#define BL_Pin GPIO_PIN_2
#define BL_GPIO_Port GPIOD
#define D11_Pin GPIO_PIN_3
#define D11_GPIO_Port GPIOB
#define D12_Pin GPIO_PIN_4
#define D12_GPIO_Port GPIOB
#define D13_Pin GPIO_PIN_5
#define D13_GPIO_Port GPIOB
#define D14_Pin GPIO_PIN_6
#define D14_GPIO_Port GPIOB
#define D15_Pin GPIO_PIN_7
#define D15_GPIO_Port GPIOB
#define RS_Pin GPIO_PIN_8
#define RS_GPIO_Port GPIOB
#define CS_Pin GPIO_PIN_9
#define CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
