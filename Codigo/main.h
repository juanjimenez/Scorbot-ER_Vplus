/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include <scorbot_Lib.h>
#include <arm_common_tables.h>
#include <arm_const_structs.h>
#include <arm_math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <atCommands.h>
#include <gamepad.h>
#include <math.h>
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
#define ENAB_GRIP_Pin GPIO_PIN_13
#define ENAB_GRIP_GPIO_Port GPIOC
#define SW_5_Pin GPIO_PIN_0
#define SW_5_GPIO_Port GPIOC
#define X1_Pin GPIO_PIN_1
#define X1_GPIO_Port GPIOC
#define Y1_Pin GPIO_PIN_2
#define Y1_GPIO_Port GPIOC
#define X2_Pin GPIO_PIN_3
#define X2_GPIO_Port GPIOC
#define Encoder_5A_Pin GPIO_PIN_0
#define Encoder_5A_GPIO_Port GPIOA
#define Encoder_5B_Pin GPIO_PIN_1
#define Encoder_5B_GPIO_Port GPIOA
#define TX_UART_Pin GPIO_PIN_2
#define TX_UART_GPIO_Port GPIOA
#define RX_UART_Pin GPIO_PIN_3
#define RX_UART_GPIO_Port GPIOA
#define SEGMENT_DATA_Pin GPIO_PIN_4
#define SEGMENT_DATA_GPIO_Port GPIOA
#define Encoder_2A_Pin GPIO_PIN_5
#define Encoder_2A_GPIO_Port GPIOA
#define Motor_5_Pin GPIO_PIN_6
#define Motor_5_GPIO_Port GPIOA
#define Motor_6_Pin GPIO_PIN_7
#define Motor_6_GPIO_Port GPIOA
#define Y2_Pin GPIO_PIN_4
#define Y2_GPIO_Port GPIOC
#define SEGMENT_MOSI_Pin GPIO_PIN_0
#define SEGMENT_MOSI_GPIO_Port GPIOB
#define SW_2_Pin GPIO_PIN_1
#define SW_2_GPIO_Port GPIOB
#define SW_3_Pin GPIO_PIN_2
#define SW_3_GPIO_Port GPIOB
#define SW_4_Pin GPIO_PIN_10
#define SW_4_GPIO_Port GPIOB
#define LOAD_Pin GPIO_PIN_12
#define LOAD_GPIO_Port GPIOB
#define SAVE_Pin GPIO_PIN_13
#define SAVE_GPIO_Port GPIOB
#define Motor_3_Pin GPIO_PIN_14
#define Motor_3_GPIO_Port GPIOB
#define Motor_4_Pin GPIO_PIN_15
#define Motor_4_GPIO_Port GPIOB
#define Encoder_6A_Pin GPIO_PIN_6
#define Encoder_6A_GPIO_Port GPIOC
#define Encoder_6B_Pin GPIO_PIN_7
#define Encoder_6B_GPIO_Port GPIOC
#define HOME_Pin GPIO_PIN_8
#define HOME_GPIO_Port GPIOC
#define START_STOP_CONTROL_Pin GPIO_PIN_9
#define START_STOP_CONTROL_GPIO_Port GPIOC
#define Encoder_1A_Pin GPIO_PIN_8
#define Encoder_1A_GPIO_Port GPIOA
#define Encoder_1B_Pin GPIO_PIN_9
#define Encoder_1B_GPIO_Port GPIOA
#define ENABLE_MOTOR_Pin GPIO_PIN_10
#define ENABLE_MOTOR_GPIO_Port GPIOA
#define PITCH_DOWN_Pin GPIO_PIN_11
#define PITCH_DOWN_GPIO_Port GPIOA
#define PITCH_UP_Pin GPIO_PIN_12
#define PITCH_UP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SW_1_Pin GPIO_PIN_15
#define SW_1_GPIO_Port GPIOA
#define SEGMENT_SCK_Pin GPIO_PIN_10
#define SEGMENT_SCK_GPIO_Port GPIOC
#define GRIP_OPEN_CLOSE_Pin GPIO_PIN_2
#define GRIP_OPEN_CLOSE_GPIO_Port GPIOD
#define Encoder_2B_Pin GPIO_PIN_3
#define Encoder_2B_GPIO_Port GPIOB
#define Encoder_3A_Pin GPIO_PIN_4
#define Encoder_3A_GPIO_Port GPIOB
#define Encoder_3B_Pin GPIO_PIN_5
#define Encoder_3B_GPIO_Port GPIOB
#define Encoder_4A_Pin GPIO_PIN_6
#define Encoder_4A_GPIO_Port GPIOB
#define Encoder_4B_Pin GPIO_PIN_7
#define Encoder_4B_GPIO_Port GPIOB
#define Motor_1_Pin GPIO_PIN_8
#define Motor_1_GPIO_Port GPIOB
#define Motor_2_Pin GPIO_PIN_9
#define Motor_2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
