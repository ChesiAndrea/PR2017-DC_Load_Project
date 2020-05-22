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
#include "stm32f7xx_hal.h"
#include "stm32f7xx_ll_iwdg.h"
#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_cortex.h"
#include "stm32f7xx_ll_rcc.h"
#include "stm32f7xx_ll_system.h"
#include "stm32f7xx_ll_utils.h"
#include "stm32f7xx_ll_pwr.h"
#include "stm32f7xx_ll_gpio.h"
#include "stm32f7xx_ll_dma.h"

#include "stm32f7xx_ll_exti.h"

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
#define TTK_PullUP_Pin GPIO_PIN_2
#define TTK_PullUP_GPIO_Port GPIOE
#define HW_New_Pin GPIO_PIN_3
#define HW_New_GPIO_Port GPIOE
#define SD_Power_Pin GPIO_PIN_13
#define SD_Power_GPIO_Port GPIOC
#define Temp_EXT_Pin GPIO_PIN_10
#define Temp_EXT_GPIO_Port GPIOF
#define Temp_INT_Pin GPIO_PIN_0
#define Temp_INT_GPIO_Port GPIOC
#define AN_INP_Pin GPIO_PIN_1
#define AN_INP_GPIO_Port GPIOC
#define LCD_Pow_ON_Pin GPIO_PIN_2
#define LCD_Pow_ON_GPIO_Port GPIOC
#define Buzzer_Pin GPIO_PIN_2
#define Buzzer_GPIO_Port GPIOA
#define BT_Enable_Pin GPIO_PIN_4
#define BT_Enable_GPIO_Port GPIOH
#define BKLight_Pin GPIO_PIN_7
#define BKLight_GPIO_Port GPIOA
#define VirtualPort_Enable_Pin GPIO_PIN_4
#define VirtualPort_Enable_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOG
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOG
#define BT_TX_Pin GPIO_PIN_5
#define BT_TX_GPIO_Port GPIOD
#define BT_RX_Pin GPIO_PIN_6
#define BT_RX_GPIO_Port GPIOD
#define BT_Reset_Pin GPIO_PIN_7
#define BT_Reset_GPIO_Port GPIOD
#define TTK_RX_Pin GPIO_PIN_9
#define TTK_RX_GPIO_Port GPIOG
#define LED3_Pin GPIO_PIN_12
#define LED3_GPIO_Port GPIOG
#define TTK_TX_Pin GPIO_PIN_14
#define TTK_TX_GPIO_Port GPIOG
#define SD_Detect_Pin GPIO_PIN_8
#define SD_Detect_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
