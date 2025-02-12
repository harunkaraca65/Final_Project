/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
// Buton durum bayrakları

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
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
#define Joystick_1_Pin GPIO_PIN_0
#define Joystick_1_GPIO_Port GPIOC
#define Water_Level_1_Pin GPIO_PIN_1
#define Water_Level_1_GPIO_Port GPIOC
#define Joystick_2_Pin GPIO_PIN_2
#define Joystick_2_GPIO_Port GPIOC
#define Water_Level_2_Pin GPIO_PIN_3
#define Water_Level_2_GPIO_Port GPIOC
#define Button_12_V__leri_Pin GPIO_PIN_0
#define Button_12_V__leri_GPIO_Port GPIOA
#define Button_12_V_Geri_Pin GPIO_PIN_1
#define Button_12_V_Geri_GPIO_Port GPIOA
#define IN1_Forward_12V_Motor_Pin GPIO_PIN_2
#define IN1_Forward_12V_Motor_GPIO_Port GPIOA
#define IN2_Backward_12V_Motor_Pin GPIO_PIN_3
#define IN2_Backward_12V_Motor_GPIO_Port GPIOA
#define IN1_Forward_Pumps_Pin GPIO_PIN_4
#define IN1_Forward_Pumps_GPIO_Port GPIOA
#define IN_2_Forward_Output_Pin GPIO_PIN_5
#define IN_2_Forward_Output_GPIO_Port GPIOA
#define IN3_Backward_Pumps_Pin GPIO_PIN_6
#define IN3_Backward_Pumps_GPIO_Port GPIOA
#define IN4_Backward_Pumps_Pin GPIO_PIN_7
#define IN4_Backward_Pumps_GPIO_Port GPIOA
#define Automation_Pin GPIO_PIN_1
#define Automation_GPIO_Port GPIOB
#define PUMP_RIGHT_1_Pin GPIO_PIN_9
#define PUMP_RIGHT_1_GPIO_Port GPIOE
#define PUMP_RIGHT_2_Pin GPIO_PIN_11
#define PUMP_RIGHT_2_GPIO_Port GPIOE
#define PUMP_LEFT_1_Pin GPIO_PIN_13
#define PUMP_LEFT_1_GPIO_Port GPIOE
#define PUMP_LEFT_2_Pin GPIO_PIN_14
#define PUMP_LEFT_2_GPIO_Port GPIOE
#define OLED_SCL_Pin GPIO_PIN_10
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_11
#define OLED_SDA_GPIO_Port GPIOB
#define DC_MOTOR_Pin GPIO_PIN_6
#define DC_MOTOR_GPIO_Port GPIOC
#define BRUSHLESS_RIGHT_Pin GPIO_PIN_15
#define BRUSHLESS_RIGHT_GPIO_Port GPIOA
#define BRUHSLESS_RIGHT_Pin GPIO_PIN_3
#define BRUHSLESS_RIGHT_GPIO_Port GPIOB
#define MPU_SCL_Pin GPIO_PIN_6
#define MPU_SCL_GPIO_Port GPIOB
#define MPU_SDA_Pin GPIO_PIN_7
#define MPU_SDA_GPIO_Port GPIOB
#define Switch_ON_Pin GPIO_PIN_0
#define Switch_ON_GPIO_Port GPIOE
#define Switch_ARKA_Pin GPIO_PIN_1
#define Switch_ARKA_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
