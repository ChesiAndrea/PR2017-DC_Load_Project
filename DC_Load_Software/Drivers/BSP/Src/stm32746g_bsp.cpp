/**
  ******************************************************************************
  * @file    stm32746g_discovery.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    25-June-2015
  * @brief   This file provides a set of firmware functions to manage LEDs, 
  *          push-buttons and COM ports available on STM32746G-Discovery
  *          board(MB1191) from STMicroelectronics.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
extern "C" {
#include "fatfs.h"
#include "string.h" 
#include "stdio.h" 
#include "stm32f7xx_hal.h"
}

#include "stm32746g_bsp.hpp"
#include "stm32746g_bsp_ts.hpp"

void 											I2Cx_Init(I2C_HandleTypeDef* hi2c);
static void   					  I2Cx_Write(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint8_t Reg, uint8_t Value);
static uint8_t  					I2Cx_Read(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint8_t Reg);
static HAL_StatusTypeDef	I2Cx_ReadMultiple(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint16_t Reg, uint16_t MemAddSize, uint8_t *Buffer, uint16_t Length);
static HAL_StatusTypeDef	I2Cx_WriteMultiple(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint16_t Reg, uint16_t MemAddSize, uint8_t *Buffer, uint16_t Length);
static HAL_StatusTypeDef	I2Cx_IsDeviceReady(I2C_HandleTypeDef* hi2c, uint16_t DevAddress, uint32_t Trials);
static void     					I2Cx_Error(I2C_HandleTypeDef* hi2c, uint8_t Addr);

/* TOUCHSCREEN IO functions */
void		TS_IO_Init(void);
void		TS_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t	TS_IO_Read(uint8_t Addr, uint8_t Reg);
void		TS_IO_Delay(uint32_t Delay);

void Set_Backlight(uint16_t Brightness){
	__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, Brightness);
}
void ClockManagement(){
	HAL_RTCEx_BKUPWrite(&hrtc, 3, SyncClockWithVehicle);
}

void TouchControllerOff (I2C_HandleTypeDef* hi2c){
	HAL_I2C_MspDeInit(hi2c);
	HAL_GPIO_WritePin(LCD_Pow_ON_GPIO_Port, LCD_Pow_ON_Pin, (GPIO_PinState)0);		
}

void TouchControllerOn (I2C_HandleTypeDef* hi2c){
	HAL_GPIO_WritePin(LCD_Pow_ON_GPIO_Port, LCD_Pow_ON_Pin, (GPIO_PinState)1);	
	HAL_I2C_MspInit(hi2c);
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
			BSP_ADC_GetConversion(&My_ADC_TINT, &BSPS.AN_TINT);
			BSP_ADC_StartConversion(&My_ADC_TEXT, My_CH_TEXT);
			bsp_ADC_SW++;	
		break;		
	
		case 2:		
			BSP_ADC_GetConversion(&My_ADC_TEXT, &BSPS.AN_TEXT);
			BSP_ADC_StartConversion(&My_ADC_AINP, My_CH_AINP);	
			bsp_ADC_SW++;	
		break;	
			
		case 3:		
			BSP_ADC_GetConversion(&My_ADC_AINP, &BSPS.AN_AINP);
			BSP_ADC_StartConversion(&My_ADC_TINT, My_CH_TINT);
			bsp_ADC_SW=1;	
			
			
			//-- Calcolo Valori Sonde Temperatura
			BSPS.T0_Val = ( ((57.25f - 0.0014211f * (float) (BSPS.AN_TINT << 4)) ) + (63*BSPS.T0_Val) )/64;
			BSPS.T0_INT=(int32_t)(BSPS.T0_Val*10);
			if(BSPS.AN_TINT > 0x0EEE) BSPS.T_INT_PLUG=0; else BSPS.T_INT_PLUG=1;			

			BSPS.T1_Val = ( ((57.25f - 0.0014211f * (float) (BSPS.AN_TEXT << 4)) ) + (63*BSPS.T1_Val) )/64;
			BSPS.T1_EXT=(int32_t)(BSPS.T1_Val*10);
			if(BSPS.AN_TEXT > 0x0EEE) BSPS.T_EXT_PLUG=0; else BSPS.T_EXT_PLUG=1;
		break;	
			
		default:
			HAL_ADC_Stop(&My_ADC_TINT);	
			HAL_ADC_Stop(&My_ADC_TEXT);	
			HAL_ADC_Stop(&My_ADC_AINP);	
			bsp_ADC_SW=0;	
		break;
	}
}

void Keypad_Decode(uint16_t KeyAnalog){
//----------------------------
//-- Decodifica Tasti Premuti Analog Keypad 
//	
	if(KeyAnalog < 80) BSPS.AN_Push |= 0x01; else BSPS.AN_Push &= ~0x01;
	if( (KeyAnalog >= 80) && (KeyAnalog < 282) ) BSPS.AN_Push |= 0x02; else BSPS.AN_Push &= ~0x02;
	if( (KeyAnalog >= 282) && (KeyAnalog < 574) ) BSPS.AN_Push |= 0x04; else BSPS.AN_Push &= ~0x04;
	if( (KeyAnalog >= 574) && (KeyAnalog < 1057) ) BSPS.AN_Push |= 0x08; else BSPS.AN_Push &= ~0x08;
	if( (KeyAnalog >= 1057) && (KeyAnalog < 2000) ) BSPS.AN_Push |= 0x10; else BSPS.AN_Push &= ~0x10;
}


void BSP_Init(void){
	BSP_TS_Init();
	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);	
	Beeper_Init(&My_BEEPER, My_CH_BEEP);
}

void BSP_Clk(void){
	ADC_Clock();
	Keypad_Decode(BSPS.AN_AINP);
	Beeper_Clk();
}

/*******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/

/******************************* I2C Routines *********************************/
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

void I2Cx_Interrupt_Enable(I2C_HandleTypeDef* hi2c){
	if(hi2c == &hi2c2)
	{
		HAL_NVIC_SetPriority(I2C2_EV_IRQn, 7, 0);
		HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
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
  return (HAL_I2C_IsDeviceReady(hi2c, DevAddress, Trials, 1000));
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
uint8_t IOE_GetErrorAck(void){ 
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

void IOE_Interrupt_Enable(I2C_HandleTypeDef* hi2c) {
  I2Cx_Interrupt_Enable(hi2c);
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
