/* Includes ------------------------------------------------------------------*/
#include "stm32746g_bsp_adc.h"

ADC_ChannelConfTypeDef sConfig;	

void BSP_ADC_StartConversion(ADC_HandleTypeDef *hadc, uint32_t channel)
{
			/**Configure for the selected ADC regular channel 3 */
			sConfig.Channel = channel;
			sConfig.Rank = 1;
			sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
			HAL_ADC_ConfigChannel(hadc, &sConfig);
			HAL_ADC_Start(hadc);	
}

uint8_t BSP_ADC_GetConversion(ADC_HandleTypeDef *hadc, uint32_t *value)
{
	HAL_ADC_PollForConversion(hadc,1);
	if(HAL_ADC_GetState(&My_ADC_TINT) == ((HAL_ADC_STATE_EOC_REG)|(HAL_ADC_STATE_READY)))
	{
		if (value) (*value) = HAL_ADC_GetValue(&My_ADC_TINT);
		return 1;
	}	
	return 0;
}

/*****END OF FILE****/
