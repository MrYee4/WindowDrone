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
#include "stm32l4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define Pitch_PWM_IN_Pin GPIO_PIN_0
#define Pitch_PWM_IN_GPIO_Port GPIOC
#define Pitch_PWM_IN_EXTI_IRQn EXTI0_IRQn
#define Roll_PWM_IN_Pin GPIO_PIN_1
#define Roll_PWM_IN_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Yaw_PWM_IN_Pin GPIO_PIN_4
#define Yaw_PWM_IN_GPIO_Port GPIOA
#define Throttle_PWM_IN_Pin GPIO_PIN_0
#define Throttle_PWM_IN_GPIO_Port GPIOB
#define BR_Motor_PWM_Pin GPIO_PIN_10
#define BR_Motor_PWM_GPIO_Port GPIOB
#define BL_Motor_PWM_Pin GPIO_PIN_15
#define BL_Motor_PWM_GPIO_Port GPIOB
#define FL_Motor_PWM_Pin GPIO_PIN_9
#define FL_Motor_PWM_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define FR_Motor_PWM_Pin GPIO_PIN_6
#define FR_Motor_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
// Throttle defines
#define Max_Throttle 160		 // Max Throttle for the PWM generation
#define Min_Throttle 80			 // Min Throttle for the PWM generation
#define Adujst_Value 2

// Communication Delay defines
#define I2C_DELAY 100			 // I2C Delay 50ms
#define UART_DELAY 50			 // UART Communication Delay
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
