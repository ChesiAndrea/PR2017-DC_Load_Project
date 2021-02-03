/**
  ******************************************************************************
  * @file    iwdg.h
  * @brief   This file contains all the function prototypes for
  *          the iwdg.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IWDG_H__
#define __IWDG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
/* Private macro -------------------------------------------------------------*/
#define IWDG_BSPTASK						(0x01)
#define IWDG_LOGICTASK					(0x02)
#define IWDG_USERTASK						(0x04)
#define IWDG_GUITASK						(0x08)
#define IWDG_ALLFLAGS						(IWDG_BSPTASK | IWDG_LOGICTASK | IWDG_USERTASK | IWDG_GUITASK)
#define SET_IWDGFLAG(x)					IWDG_TaskFlag |= x
#define RES_IWDGFLAGS()					IWDG_TaskFlag = 0
#define IS_ALLIWDGFLAGS()				(IWDG_TaskFlag == IWDG_ALLFLAGS)
	
/* Private variables ---------------------------------------------------------*/
extern uint32_t IWDG_TaskFlag;

/* USER CODE END Private defines */

void MX_IWDG_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __IWDG_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
