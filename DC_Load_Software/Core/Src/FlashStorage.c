#include "FlashStorage.h"
#include "string.h"

/* This config region is reserved on the second sector of the flash memory, its
size is of 32kB you could use the dual bank mode so you could have a 16kB sector
but this isn't the case. The first sector contain the interrupt vector while the
sectors after the 2nd contain the rest of the program, all these things are
defined in the linker scatter file so don't set the auto-generation of scatter
file in Keil but use your own scatter file */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#pragma O0
/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base address of Sector 0, 32 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08008000) /* Base address of Sector 1, 32 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08010000) /* Base address of Sector 2, 32 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x08018000) /* Base address of Sector 3, 32 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08020000) /* Base address of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08040000) /* Base address of Sector 5, 256 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08080000) /* Base address of Sector 6, 256 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x080C0000) /* Base address of Sector 7, 256 Kbytes */


#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_1  			/* Start @ of user Flash area */
#define USER_SECTOR				(FLASH_SECTOR_1)
#define SECTOR_DIMENSION		(0x00008000)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

const tFLASHSTORAGE_Storage Storage __attribute__((section("StorageSection")));/* =
{
	FLASHSTORAGE_StorageVersion_Value
};*/
static void *editableStorage = (void *)0xC1FF8000;
/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct = 
{	
	/* Fill EraseInit structure*/
	.TypeErase     = FLASH_TYPEERASE_SECTORS,
	.VoltageRange  = FLASH_VOLTAGE_RANGE_3,
	.Sector        = USER_SECTOR,
	.NbSectors     = 1,
};
static uint32_t SECTORError = 0;

/* Private Function Prototypes -----------------------------------------------*/


/* Private Functions ---------------------------------------------------------*/

/* Exported Functions --------------------------------------------------------*/


tFLASHSTORAGE_Storage* FLASHSTORAGE_Unlock(void)
{
	memcpy(editableStorage, (void *)&Storage, sizeof(tFLASHSTORAGE_Storage));
	return editableStorage;
}



void FLASHSTORAGE_Discard(void);

bool FLASHSTORAGE_isValid(void)
{
	return (Storage.StorageVersion == FLASHSTORAGE_StorageVersion_Value);
}

bool FLASHSTORAGE_Save(void)
{
	((tFLASHSTORAGE_Storage *)editableStorage)->StorageVersion = FLASHSTORAGE_StorageVersion_Value;
	uint8_t *prt = editableStorage;
	int i, res = true;
	__disable_irq();
	//unlock flash writing
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP|FLASH_FLAG_PGAERR|FLASH_FLAG_WRPERR|FLASH_FLAG_PGPERR);
	
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK)
	{
		HAL_FLASH_Lock();
		return false;
	}
	SCB_CleanInvalidateDCache_by_Addr((uint32_t*)FLASH_USER_START_ADDR, SECTOR_DIMENSION);
	
	for(i = 0; i < (sizeof(tFLASHSTORAGE_Storage)/4)*4; i += 4)
	{
		res &= (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (FLASH_USER_START_ADDR + i), *((uint32_t *)(prt+i))) == HAL_OK);
	}
	for(; i < sizeof(tFLASHSTORAGE_Storage); i += 1)
	{
		res &= (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (FLASH_USER_START_ADDR + i), *(prt+i)) == HAL_OK);
	}
	
	//lock the flash after writing
	HAL_FLASH_Lock();
	__enable_irq();
	return res;
}

