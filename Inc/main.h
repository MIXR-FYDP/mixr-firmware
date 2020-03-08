/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DEBUG_LED_0_Pin GPIO_PIN_0
#define DEBUG_LED_0_GPIO_Port GPIOF
#define LED_1_Pin GPIO_PIN_1
#define LED_1_GPIO_Port GPIOF
#define LED_2_Pin GPIO_PIN_2
#define LED_2_GPIO_Port GPIOF
#define LED_3_Pin GPIO_PIN_3
#define LED_3_GPIO_Port GPIOF
#define FLASH_NRESET_Pin GPIO_PIN_5
#define FLASH_NRESET_GPIO_Port GPIOF
#define STM32_PA2_UART2_TX_ESP32_RX_Pin GPIO_PIN_2
#define STM32_PA2_UART2_TX_ESP32_RX_GPIO_Port GPIOA
#define INST0_CLIPPING_Pin GPIO_PIN_2
#define INST0_CLIPPING_GPIO_Port GPIOH
#define INST1_CLIPPING_Pin GPIO_PIN_3
#define INST1_CLIPPING_GPIO_Port GPIOH
#define STM32_PA3_UART_RX_ESP32_TX_Pin GPIO_PIN_3
#define STM32_PA3_UART_RX_ESP32_TX_GPIO_Port GPIOA
#define ADC_nRESET_Pin GPIO_PIN_0
#define ADC_nRESET_GPIO_Port GPIOB
#define STM32_PB14_DEBUG_UART_TX_Pin GPIO_PIN_14
#define STM32_PB14_DEBUG_UART_TX_GPIO_Port GPIOB
#define STM32_PB15_DEBUG_UART_RX_Pin GPIO_PIN_15
#define STM32_PB15_DEBUG_UART_RX_GPIO_Port GPIOB
#define SD_DETECT_Pin GPIO_PIN_13
#define SD_DETECT_GPIO_Port GPIOG
#define ESP32_IO25_STM32_SD_ACCESS_EN_Pin GPIO_PIN_14
#define ESP32_IO25_STM32_SD_ACCESS_EN_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
