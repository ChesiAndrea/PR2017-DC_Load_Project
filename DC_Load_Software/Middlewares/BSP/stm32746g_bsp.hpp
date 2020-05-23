  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32746G_BSP_H
#define __STM32746G_BSP_H

#include "main.h"
#include "tim.h"

#ifdef __cplusplus
 extern "C" {
#endif

void BacklightManagement(int Bright_LCD);
void BSP_Init(void);
	 
#ifdef __cplusplus
}
#endif

#endif /* __STM32746G_BSP_H */

/*****END OF FILE****/
