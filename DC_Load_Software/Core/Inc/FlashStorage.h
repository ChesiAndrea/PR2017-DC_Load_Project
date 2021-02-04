/**
  ******************************************************************************
  * @Author         : Luca Istorico
  ******************************************************************************
  */
  

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASHSTORAGE_H
#define __FLASHSTORAGE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include <stdbool.h>


/* Exported types ------------------------------------------------------------*/


typedef struct
{
	char StorageVersion; //NON RIMUOVERE
	struct
	{
		bool isValid;
		struct
		{
			int32_t x;
			int32_t y;
		} Point[3];
	} touchCalib;
} tFLASHSTORAGE_Storage;

/* Exported constants --------------------------------------------------------*/
extern const tFLASHSTORAGE_Storage Storage;


/* Exported macro ------------------------------------------------------------*/

/**
  * @brief  Definisce la versione dello Storage.
  * @note   Questo valore deve essere modificato ogni volta che lo Storage
  *         viene modificato
  */
#define FLASHSTORAGE_StorageVersion_Value	(0x02)


/* Exported functions prototypes ---------------------------------------------*/
extern tFLASHSTORAGE_Storage* FLASHSTORAGE_Unlock(void);
extern void FLASHSTORAGE_Discard(void);
extern bool FLASHSTORAGE_Save(void);
extern bool FLASHSTORAGE_isValid(void);


#ifdef __cplusplus
}
#endif

#endif /* __FLASHSTORAGE_H */

/*****************************END OF FILE****************************/
