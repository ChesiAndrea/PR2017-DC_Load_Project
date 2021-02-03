  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32746G_BSP_H
#define __STM32746G_BSP_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "stm32746g_bsp_adc.h"
#include "stm32746g_bsp_beeper.h"
	 
#include "stm32746g_bsp_ts.hpp"

void TouchControllerOff(I2C_HandleTypeDef* hi2c);	 
void TouchControllerOn(I2C_HandleTypeDef* hi2c);
	 
void Set_Backlight(uint16_t Brightness);
	
void BSP_Init(void);	 
void BSP_Clk(void);	 

		 
void IOE_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);

	 
#ifdef __cplusplus
}
#endif

#endif /* __STM32746G_BSP_H */

/*****END OF FILE****/
