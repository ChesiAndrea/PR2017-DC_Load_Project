
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32746G_BSP_BEEPER_H
#define __STM32746G_BSP_BEEPER_H

#ifdef __cplusplus
 extern "C" {
#endif   
   
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32746g_bsp.hpp"
   
/* ---------------------------------------------------------------------------*/
/* Beeper --------------------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/

#define My_BEEPER						htim5
#define My_CH_BEEP 					TIM_CHANNEL_3

extern 	TIM_HandleTypeDef 	My_BEEPER;

extern void Beeper_Init(TIM_HandleTypeDef * htim , uint32_t TIM_CHANNEL);

extern void Set_Beep_Note(uint32_t Val);
extern void Set_Beeper(void);
extern void Res_Beeper(void);
extern void Beeper_Clk(void);
extern void Set_BeepStream(void);
extern void Res_BeepStream(void);
extern void Tog_BeepStream(void);
extern void Beep_on_Keys(void);


#ifdef __cplusplus
}
#endif

#endif /* __STM32746G_BSP_BEEPER_H */

/*****END OF FILE****/
