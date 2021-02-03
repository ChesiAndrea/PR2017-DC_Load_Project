/**
  ******************************************************************************
  * File Name          : STM32TouchController.cpp
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

/* USER CODE BEGIN STM32TouchController */

#include <STM32TouchController.hpp>
#include "stm32746g_bsp_ts.hpp"
#include "stm32746g_bsp_beeper.h"

volatile uint8_t TS_EnableFlag = TS_DEVICE_NOT_FOUND; 
void STM32TouchController::init()
{
	if(TS_EnableFlag != TS_OK)
	{
		TS_EnableFlag = BSP_TS_Init();	
	}
}

int32_t  mX, qX, mY, qY;
bool STM32TouchController::sampleTouch(int32_t& x, int32_t& y)
{
/* USER CODE BEGIN  F4TouchController_sampleTouch  */
	static char pressed = 0;
  TS_StateTypeDef state = { 0 };
	if( TS_EnableFlag == TS_OK){
    BSP_TS_GetState(&state);
    if (state.touchDetected)
    {		
				x =(((mX * state.touchX[0]) + qX)/1000);
				y =(((mY * state.touchY[0]) + qY)/1000);
			
				if (!pressed) Beep_on_Keys();
				pressed = 1;
        return true;
    }/**/
	}
		pressed = 0;
    return false; 
}

/* USER CODE END STM32TouchController */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
