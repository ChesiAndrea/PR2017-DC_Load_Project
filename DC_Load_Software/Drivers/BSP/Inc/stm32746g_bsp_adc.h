
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32746G_BSP_ADC_H
#define __STM32746G_BSP_ADC_H

#ifdef __cplusplus
 extern "C" {
#endif   
   
/* Includes ------------------------------------------------------------------*/
#include "stm32746g_bsp.hpp"
   


/* ---------------------------------------------------------------------------*/
/* ADC  ----------------------------------------------------------------------*/
/* ---------------------------------------------------------------------------*/

#define My_ADC_TINT					hadc3
#define My_CH_TINT 					ADC_CHANNEL_10

#define My_ADC_TEXT					hadc3
#define My_CH_TEXT 					ADC_CHANNEL_8

#define My_ADC_AINP					hadc3
#define My_CH_AINP 					ADC_CHANNEL_11


extern 	ADC_HandleTypeDef 	My_ADC_TINT;
extern 	ADC_HandleTypeDef 	My_ADC_TEXT;
extern 	ADC_HandleTypeDef 	My_ADC_AINP;

extern void BSP_ADC_StartConversion(ADC_HandleTypeDef *hadc, uint32_t channel);
extern uint8_t BSP_ADC_GetConversion(ADC_HandleTypeDef *hadc, uint32_t *value);

#ifdef __cplusplus
}
#endif

#endif /* __STM32746G_BSP_ADC_H */

/*****END OF FILE****/
