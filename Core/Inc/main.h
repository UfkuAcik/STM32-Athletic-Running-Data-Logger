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
#include "stm32f4xx_hal.h"

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
#define EXTI_Nabiz_Pin GPIO_PIN_13
#define EXTI_Nabiz_GPIO_Port GPIOC
#define EXTI_Nabiz_EXTI_IRQn EXTI15_10_IRQn
#define DS18B20_Pin GPIO_PIN_1
#define DS18B20_GPIO_Port GPIOA
#define USART2_TX_GPS_Pin GPIO_PIN_2
#define USART2_TX_GPS_GPIO_Port GPIOA
#define USART2_RX_GPS_Pin GPIO_PIN_3
#define USART2_RX_GPS_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_0
#define SPI_CS_GPIO_Port GPIOB
#define I2C2_SCL_BME280_Pin GPIO_PIN_10
#define I2C2_SCL_BME280_GPIO_Port GPIOB
#define button1_Pin GPIO_PIN_12
#define button1_GPIO_Port GPIOB
#define button1_EXTI_IRQn EXTI15_10_IRQn
#define button2_Pin GPIO_PIN_14
#define button2_GPIO_Port GPIOB
#define button2_EXTI_IRQn EXTI15_10_IRQn
#define I2C3_SCL_LCD_Pin GPIO_PIN_8
#define I2C3_SCL_LCD_GPIO_Port GPIOA
#define USART1_TX_DEBUG_Pin GPIO_PIN_9
#define USART1_TX_DEBUG_GPIO_Port GPIOA
#define USART1_RX_DEBUG_Pin GPIO_PIN_10
#define USART1_RX_DEBUG_GPIO_Port GPIOA
#define I2C2_SDA_BME280_Pin GPIO_PIN_3
#define I2C2_SDA_BME280_GPIO_Port GPIOB
#define I2C3_SDA_LCD_Pin GPIO_PIN_4
#define I2C3_SDA_LCD_GPIO_Port GPIOB
#define I2C1_SCL_MAX30102_Pin GPIO_PIN_6
#define I2C1_SCL_MAX30102_GPIO_Port GPIOB
#define I2C1_SDA_MAX30102_Pin GPIO_PIN_7
#define I2C1_SDA_MAX30102_GPIO_Port GPIOB
#define MAX30102_INT_Pin GPIO_PIN_8
#define MAX30102_INT_GPIO_Port GPIOB
#define MAX30102_INT_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
