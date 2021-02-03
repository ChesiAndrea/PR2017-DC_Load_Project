//==========================================================================================================================================
//==========================================================================================================================================
//  LCD7_CPCM4 Bsp  
//==========================================================================================================================================
//==========================================================================================================================================

/* Includes ------------------------------------------------------------------*/
extern "C" {
#include "fatfs.h"
#include "string.h" 
#include "stdio.h" 
#include "stm32f7xx_hal.h"
#include "i2c.h"	
}

#include "stm32746g_bsp.hpp"
#include "stm32746g_bsp_ts.hpp"


/* TOUCHSCREEN IO functions */
void		TS_IO_Init(void);
void		TS_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t	TS_IO_Read(uint8_t Addr, uint8_t Reg);
void		TS_IO_Delay(uint32_t Delay);

void Set_Backlight(uint16_t Brightness){
	__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, Brightness);
}

void TouchControllerOff (I2C_HandleTypeDef* hi2c){
	HAL_I2C_MspDeInit(hi2c);
	HAL_GPIO_WritePin(LCD_Pow_ON_GPIO_Port, LCD_Pow_ON_Pin, (GPIO_PinState)0);		
}

void TouchControllerOn (I2C_HandleTypeDef* hi2c){
	HAL_GPIO_WritePin(LCD_Pow_ON_GPIO_Port, LCD_Pow_ON_Pin, (GPIO_PinState)1);	
	HAL_I2C_MspInit(hi2c);
}
				
tADC_Measure ADCValue;		
void Keypad_Decode(uint16_t KeyAnalog){
//----------------------------
//-- Decodifica Tasti Premuti Analog Keypad 
//	
	if(KeyAnalog < 80) ADCValue.AN_Push |= 0x01; else ADCValue.AN_Push &= ~0x01;
	if( (KeyAnalog >= 80) && (KeyAnalog < 282) ) ADCValue.AN_Push |= 0x02; else ADCValue.AN_Push &= ~0x02;
	if( (KeyAnalog >= 282) && (KeyAnalog < 574) ) ADCValue.AN_Push |= 0x04; else ADCValue.AN_Push &= ~0x04;
	if( (KeyAnalog >= 574) && (KeyAnalog < 1057) ) ADCValue.AN_Push |= 0x08; else ADCValue.AN_Push &= ~0x08;
	if( (KeyAnalog >= 1057) && (KeyAnalog < 2000) ) ADCValue.AN_Push |= 0x10; else ADCValue.AN_Push &= ~0x10;
}

void ADC_Clock(void){
	static uint8_t bsp_ADC_SW=0;
//----------------------------
//-- Analog Input
//	
	switch(bsp_ADC_SW) 
	{
		case 0:	
			BSP_ADC_StartConversion(&My_ADC_TINT, My_CH_TINT);
			bsp_ADC_SW++;	
		break;
	
		case 1:
			BSP_ADC_GetConversion(&My_ADC_TINT, &(ADCValue.raw.AN_TINT));
			ADCValue.Tint.Val = ( ((57.25f - 0.0014211f * (float) (ADCValue.raw.AN_TINT << 4)) ) + (63*ADCValue.Tint.Val) )/64;
			ADCValue.Tint.INT=(int32_t)(ADCValue.Tint.Val*10);
			if(ADCValue.Tint.INT > -28) ADCValue.Tint.PLUG=1; else ADCValue.Tint.PLUG=0;		
			BSP_ADC_StartConversion(&My_ADC_TEXT, My_CH_TEXT);
			bsp_ADC_SW++;	
		break;		
	
		case 2:		
			BSP_ADC_GetConversion(&My_ADC_TEXT, &(ADCValue.raw.AN_TEXT));
			ADCValue.Text.Val = ( ((57.25f - 0.0014211f * (float) (ADCValue.raw.AN_TEXT << 4)) ) + (63*ADCValue.Text.Val) )/64;
			ADCValue.Text.INT=(int32_t)(ADCValue.Text.Val*10);
			if(ADCValue.Text.INT > -28) ADCValue.Text.PLUG=1; else ADCValue.Text.PLUG=0;		
			BSP_ADC_StartConversion(&My_ADC_AINP, My_CH_AINP);	
			bsp_ADC_SW++;	
		break;	
			
		case 3:		
			BSP_ADC_GetConversion(&My_ADC_AINP, &(ADCValue.raw.AN_AINP));
			Keypad_Decode(ADCValue.raw.AN_AINP);		
			BSP_ADC_StartConversion(&My_ADC_TINT, My_CH_TINT);
			bsp_ADC_SW=1;	
		break;	
			
		default:
			HAL_ADC_Stop(&My_ADC_TINT);	
			HAL_ADC_Stop(&My_ADC_TEXT);	
			HAL_ADC_Stop(&My_ADC_AINP);	
			bsp_ADC_SW=0;	
		break;
	}
}

extern volatile uint8_t TS_EnableFlag;
void BSP_Init(void){
	TS_EnableFlag = BSP_TS_Init();
	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);	
	Beeper_Init(&My_BEEPER, My_CH_BEEP);
}

void BSP_Clk(void){
	ADC_Clock();
	Beeper_Clk();
}

/*******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/

/******************************* I2C Routines *********************************/
void 											I2Cx_Init(I2C_HandleTypeDef* hi2c);
static void   					  I2Cx_Write(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint8_t Reg, uint8_t Value);
static uint8_t  					I2Cx_Read(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint8_t Reg);
static HAL_StatusTypeDef	I2Cx_ReadMultiple(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint16_t Reg, uint16_t MemAddSize, uint8_t *Buffer, uint16_t Length);
static HAL_StatusTypeDef	I2Cx_WriteMultiple(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint16_t Reg, uint16_t MemAddSize, uint8_t *Buffer, uint16_t Length);
static HAL_StatusTypeDef	I2Cx_IsDeviceReady(I2C_HandleTypeDef* hi2c, uint16_t DevAddress, uint32_t Trials);
static void     					I2Cx_Error(I2C_HandleTypeDef* hi2c, uint8_t Addr);

void I2Cx_Init(I2C_HandleTypeDef* hi2c){
	if(hi2c == &hi2c1)
	{
		if(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_RESET)
		{
			MX_I2C1_Init();
		}
	}
	if(hi2c == &hi2c2)
	{
		if(HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_RESET)
		{
			MX_I2C2_Init();
		}
	}
}

/**
  * @brief  Writes a single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
  * @param  Value: Data to be written
  * @retval None
  */
static void I2Cx_Write(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint8_t Reg, uint8_t Value){
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_I2C_Mem_Write(hi2c, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 100); 

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2Cx_Error(hi2c, Addr);
  }
}

/**
  * @brief  Writes a single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
  * @param  Value: Data to be written
  * @retval None
  */
static void I2Cx_Transmit(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint8_t *Buffer, uint16_t Length){
  HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Master_Transmit(hi2c, Addr, Buffer, Length, 100);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2Cx_Error(hi2c, Addr);
  }
}

/**
  * @brief  Reads a single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
  * @retval Read data
  */
static uint8_t I2Cx_Read(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint8_t Reg){
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t Value = 0;
  
  status = HAL_I2C_Mem_Read(hi2c, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 1000);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2Cx_Error(hi2c, Addr);
  }
  return Value;   
}

/**
  * @brief  Reads data in Master Mode.
  * @param  Addr: I2C address
  * @retval Read data
  */
static uint8_t I2Cx_Master_Receive(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint8_t *Buffer, uint16_t Length){
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t Value = 0;
  
  status = HAL_I2C_Master_Receive(hi2c, Addr, Buffer, Length, 1000);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2Cx_Error(hi2c, Addr);
  }
  return Value;   
}

/**
  * @brief  Reads data in non-blocking mode.
  * @param  Addr: I2C address
  * @retval Read data
  */
static uint8_t I2Cx_Mem_Read_It(I2C_HandleTypeDef* hi2c, uint8_t Addr,uint16_t Reg, uint8_t *Buffer, uint16_t Length){
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t Value = 0;
  
  status = HAL_I2C_Mem_Read_IT(hi2c, Addr, Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2Cx_Error(hi2c, Addr);
  }
  return Value;   
}

/**
  * @brief  Reads data in non-blocking mode.
  * @param  Addr: I2C address
  * @retval Read data
  */
static uint8_t I2Cx_Mem_Read(I2C_HandleTypeDef* hi2c, uint8_t Addr,uint16_t Reg, uint8_t *Buffer, uint16_t Length){
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t Value = 0;
  
  status = HAL_I2C_Mem_Read(hi2c, Addr, Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length ,100);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    I2Cx_Error(hi2c, Addr);
  }
  return Value;   
}

/**
  * @brief  Reads multiple data.
  * @param  Addr: I2C address
  * @param  Reg: Reg address 
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
static HAL_StatusTypeDef I2Cx_ReadMultiple(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length){
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Read(hi2c, Addr, (uint16_t)Reg, MemAddress, Buffer, Length, 1000);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* I2C error occurred */
    I2Cx_Error(hi2c, Addr);
  }
  return status;    
}

/**
  * @brief  Writes a value in a register of the device through BUS in using DMA mode.
  * @param  Addr: Device address on BUS Bus.  
  * @param  Reg: The target register address to write
  * @param  pBuffer: The target register value to be written 
  * @param  Length: buffer size to be written
  * @retval HAL status
  */
static HAL_StatusTypeDef I2Cx_WriteMultiple(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint16_t Reg, uint16_t MemAddress, uint8_t *Buffer, uint16_t Length){
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Write(hi2c, Addr, (uint16_t)Reg, MemAddress, Buffer, Length, 1000);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initialize the I2C Bus */
    I2Cx_Error(hi2c, Addr);
  }
  return status;
}


/**
  * @brief  Checks if target device is ready for communication. 
  * @note   This function is used with Memory devices
  * @param  DevAddress: Target device address
  * @param  Trials: Number of trials
  * @retval HAL status
  */
static HAL_StatusTypeDef I2Cx_IsDeviceReady(I2C_HandleTypeDef* hi2c, uint16_t DevAddress, uint32_t Trials){ 
  return HAL_I2C_IsDeviceReady(hi2c, DevAddress, Trials, 1000);
}

/**
  * @brief  Manages error callback by re-initializing I2C.
  * @param  Addr: I2C Address
  * @retval None
  */
volatile uint8_t TS_I2C_Error=0;
static void I2Cx_Error(I2C_HandleTypeDef* hi2c, uint8_t Addr){	
	if((Addr == TS_I2C_ADDRESS) || (Addr == TSC2013_I2C_ADDRESS))
	{	
		/* De-initialize the I2C communication bus */
		HAL_I2C_DeInit(hi2c);
		/* Re-Initialize the I2C communication bus */
		I2Cx_Init(hi2c);
		TS_I2C_Error=1;
	}
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	IOE_MemRxCpltCallback(hi2c);
}	


/*******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/
uint8_t IOE_GetError(void){
	return(TS_I2C_Error);
}
extern "C" uint8_t IOE_GetErrorAck(void){ 
	uint8_t ans= TS_I2C_Error;
	TS_I2C_Error=0;	
	return(ans);
}
/********************************* LINK TOUCHSCREEN *********************************/
/**
  * @brief  Initializes IOE low level.
  * @param  None
  * @retval None
  */
void IOE_Init(I2C_HandleTypeDef* hi2c) {
  I2Cx_Init(hi2c);
}


/**
  * @brief  IOE writes single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
  * @param  Value: Data to be written
  * @retval None
  */
void IOE_Write(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint8_t Reg, uint8_t Value){
  I2Cx_Write(hi2c, Addr, Reg, Value);
}

/**
  * @brief  IOE transmit.
  * @param  Addr: I2C address
  * @param  Value: Data to be written
  * @retval None
  */
void IOE_Transmit(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint8_t *Buffer, uint16_t Length){
  I2Cx_Transmit(hi2c, Addr, Buffer, Length);
}


/**
  * @brief  IOE reads single data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
  * @retval Read data
  */
uint8_t IOE_Read(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint8_t Reg){
  return I2Cx_Read(hi2c, Addr, Reg);
}

/**
  * @brief  IOE reads multiple data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
uint16_t IOE_ReadMultiple(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length){
 return I2Cx_ReadMultiple(hi2c, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}


/**
  * @brief  IOE receive multiple data.
  * @param  Addr: I2C address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
uint16_t IOE_Master_Receive(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint8_t *Buffer, uint16_t Length){
 return I2Cx_Master_Receive(hi2c, Addr, Buffer, Length);
}

/**
  * @brief  IOE receive multiple data non blobking mode.
  * @param  Addr: I2C address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
uint16_t IOE_Mem_Read_It(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length){
 return I2Cx_Mem_Read_It(hi2c, Addr, Reg, Buffer, Length);
}

/**
  * @brief  IOE receive multiple data 
  * @param  Addr: I2C address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval Number of read data
  */
uint16_t IOE_Mem_Read(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length){
 return I2Cx_Mem_Read(hi2c, Addr, Reg, Buffer, Length);
}
/**
  * @brief  IOE writes multiple data.
  * @param  Addr: I2C address
  * @param  Reg: Register address 
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Length of the data
  * @retval None
  */
void IOE_WriteMultiple(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length){
  I2Cx_WriteMultiple(hi2c, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length);
}

/**
  * @brief  IOE delay 
  * @param  Delay: Delay in ms
  * @retval None
  */
void IOE_Delay(uint32_t Delay){
  HAL_Delay(Delay);
}

__weak void IOE_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
  UNUSED(hi2c);
}	


/*****END OF FILE****/
