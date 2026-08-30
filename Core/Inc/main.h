/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stdint.h"
#include "motor_config.h"
#include "DJmotor.h"
#include "ZDrive.h"
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
#define LED_1_Pin GPIO_PIN_4
#define LED_1_GPIO_Port GPIOA
#define LED_2_Pin GPIO_PIN_5
#define LED_2_GPIO_Port GPIOA
#define LED_3_Pin GPIO_PIN_6
#define LED_3_GPIO_Port GPIOA
#define LED_4_Pin GPIO_PIN_7
#define LED_4_GPIO_Port GPIOA
#define BEEP_Pin GPIO_PIN_8
#define BEEP_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/*
 * 气泵控制引脚。
 *
 * PA9 = 左臂气泵, PA2 = 右臂气泵。
 * 输出高电平 = 开气泵。
 *
 * 注意：
 * 这两个引脚同时是 USART1_TX / USART2_TX 的默认复用脚，
 * 引脚初始化放在 Arm_Task 启动时(Arm_PumpGpioInit)，
 * 避免被外设 MspInit 覆盖。
 */
#define LEFT_PUMP_Pin       GPIO_PIN_9
#define LEFT_PUMP_GPIO_Port GPIOA

#define RIGHT_PUMP_Pin      GPIO_PIN_2
#define RIGHT_PUMP_GPIO_Port GPIOA

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
