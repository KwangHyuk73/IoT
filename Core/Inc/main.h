/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"
#include "cmsis_rv.h"

#include "crt/kprintf.h"
#include "dev/dp83848.h"
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
#define JOY_A_Pin GPIO_PIN_4
#define JOY_A_GPIO_Port GPIOA
#define JOY_B_Pin GPIO_PIN_6
#define JOY_B_GPIO_Port GPIOA
#define LCD_BL_Pin GPIO_PIN_0
#define LCD_BL_GPIO_Port GPIOB
#define JOY_C_Pin GPIO_PIN_15
#define JOY_C_GPIO_Port GPIOB
#define GLED_Pin GPIO_PIN_12
#define GLED_GPIO_Port GPIOD
#define OLED_Pin GPIO_PIN_13
#define OLED_GPIO_Port GPIOD
#define JOY_D_Pin GPIO_PIN_6
#define JOY_D_GPIO_Port GPIOC
#define JOY_CNTR_Pin GPIO_PIN_7
#define JOY_CNTR_GPIO_Port GPIOC
#define GEN_INT_Pin GPIO_PIN_8
#define GEN_INT_GPIO_Port GPIOC
#define EXT_INT9_Pin GPIO_PIN_9
#define EXT_INT9_GPIO_Port GPIOC
#define EXT_INT9_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
