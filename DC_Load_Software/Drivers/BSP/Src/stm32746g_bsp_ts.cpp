
/* Includes ------------------------------------------------------------------*/
#include "stm32746g_bsp_ts.hpp"
#include "stm32746g_bsp.hpp"
extern "C" {

}

extern uint8_t IOE_GetErrorAck(void);
extern uint8_t Touch_Press; 
extern uint32_t AmbuPanelState;
extern uint32_t Off_Panel;
extern uint32_t On_Panel;

static TS_DrvTypeDef *tsDriver;
static uint16_t tsXBoundary, tsYBoundary; 
static uint8_t  tsOrientation;
static uint8_t  I2cAddress;

//-------------------------------------------------------------------------
//		BSP_TS_Init
//-------------------------------------------------------------------------
// variable
TOUCH_ControllerTypeDef Touch_Controller;	
I2C_HandleTypeDef* Touch_hi2c;
extern uint16_t TsInitErrorCnt;
S_TsRegConfig TsRegConfig;

uint8_t BSP_TS_Config (void)
{
	I2cAddress = TS_I2C_ADDRESS;
	tsOrientation = TS_SWAP_Y;
	if(Touch_Controller == STMPE811)	
	{
		if(stmpe811_ts_drv.ReadID(TS_I2C_ADDRESS) == STMPE811_ID)
		{ 
			/* Initialize the TS driver structure */
			tsDriver = &stmpe811_ts_drv;
			/* Initialize the TS driver */
			tsDriver->Start(I2cAddress, &TsRegConfig);
			/* Test of the right configuration */
			if(IOE_GetErrorAck()==0) 
			{
				if (
					 ((IOE_Read(Stmpe811_I2C, TS_I2C_ADDRESS, STMPE811_REG_ADC_CTRL1) & (0x7A)) == ((TsRegConfig.Adc_Ctrl1) & (0x7A))) &&
						(IOE_Read(Stmpe811_I2C, TS_I2C_ADDRESS, STMPE811_REG_TSC_CFG)		          ==   TsRegConfig.Tsc_Gfg)   					 &&
					 ((IOE_Read(Stmpe811_I2C, TS_I2C_ADDRESS, STMPE811_REG_TSC_CTRL) 	& (0x01))	== ((TsRegConfig.Tsc_Ctrl)	& (0x01)))
					 )
				{
					return TS_OK;
				}	
				else
				{
					return TS_ERROR;
				}
			}
			else
			{
				return TS_ERROR;
			}
		}
		return TS_DEVICE_NOT_FOUND;	
	}
	else	if(Touch_Controller == TSC2013)	
	{
		uint8_t Status;
		Status = tsc2013_write_config_values();
		if(IOE_GetErrorAck()==0) 
		{
			return Status;
		}
		else
		{
			return TS_ERROR;
		}
	}
	return TS_DEVICE_NOT_VALID;	
}

uint8_t BSP_TS_Settings(I2C_HandleTypeDef* hi2c, TOUCH_ControllerTypeDef hTSC, uint16_t ts_SizeX, uint16_t ts_SizeY)
{
	tsXBoundary = ts_SizeX;
  tsYBoundary = ts_SizeY;
	Touch_Controller = hTSC;
	Touch_hi2c = hi2c;
	return TS_OK;
}
extern uint16_t TsInitIntCnt;
extern uint16_t TsInitExtCnt;   
uint8_t BSP_TS_Init(void)
{
  uint8_t status = TS_OK;
	volatile uint16_t ConfigErrorCnt = 0;
	volatile long Error = 0;

	while (Error < 100)
	{
		//power TS controller off  
		TouchControllerOff(Touch_hi2c);	
		IOE_Delay(10);
		
		//power TS controller on
		TouchControllerOn(Touch_hi2c);
		IOE_Delay(5);		
		
		/* Read ID and verify if the touch screen driver is ready */
		IOE_GetErrorAck();
		if(Touch_Controller == STMPE811)	
		{
			stmpe811_ts_drv.Init(Touch_hi2c, TS_I2C_ADDRESS);
		}
		else if(Touch_Controller == TSC2013)
		{
			tsc2013_init(Touch_hi2c);
		}
		else 
		{
			status = TS_ERROR; 
			return status;
		}
		
		status = IOE_GetErrorAck();	
		if(status == TS_OK)
		{
			/* Configuration cycle */
			ConfigErrorCnt = 0;
			#define MaxConfigCycle 3
			while (ConfigErrorCnt < MaxConfigCycle)		
			{
				LL_IWDG_ReloadCounter(IWDG);
				status = BSP_TS_Config();
				if(status == TS_OK)
				{
					TsInitIntCnt += ConfigErrorCnt;
					TsInitExtCnt += Error;
					return TS_OK;
				}
				else if(status == TS_ERROR)
				{
						ConfigErrorCnt++;
				}
				else
				{
					Error++;
					break;
				}			
				HAL_Delay(2);
			}
		}
		Error++;
	}
	if(Error > 99)
	{
		status = TS_DEVICE_NOT_FOUND;
	}
	TsInitIntCnt += ConfigErrorCnt;
	TsInitExtCnt += Error;
	return status;
}

extern uint16_t R_i2c;
extern uint16_t R_Ts;
extern uint16_t RConfig;
uint8_t block = 0;



uint8_t BSP_TS_GetState(TS_StateTypeDef *TS_State)
{
//----------------------------------------------------------------------------------------
	#define ENABLE_TSC 0x01
	static  uint8_t CntTest_Ts = 0;
	uint8_t ReconfigurationCnt = 0;
	CntTest_Ts++;
	if(CntTest_Ts>30)
	{
		CntTest_Ts = 0;
		if(((Touch_Controller == STMPE811) && ((IOE_Read(Stmpe811_I2C, TS_I2C_ADDRESS, STMPE811_REG_TSC_CTRL) & ENABLE_TSC) == 0))	|| 
			 ((Touch_Controller == TSC2013)  && ((tsc2013_read_touch_status()&0x80) != 0x80))) // check if the device has been reset
		{
			RConfig++;	
			while (ReconfigurationCnt < 5)
			{
				if(BSP_TS_Config() == TS_OK)	break;
				else	ReconfigurationCnt++;
			}
		}
	}
// in case of 5 Ts controller inizializzated failed -> system reset
	if(ReconfigurationCnt > 4)
	{
		R_Ts++;
		HAL_NVIC_SystemReset();
	}
	
// in case of 5 i2c error -> system reset
	static uint8_t        I2c_Error = 0;
	if(IOE_GetErrorAck())	I2c_Error++;		
	else if(I2c_Error)    I2c_Error--;
	if(I2c_Error == 4) // 5 error
	{
		R_i2c++;	
		HAL_NVIC_SystemReset();			
	}	
	
  static uint32_t _x[TS_MAX_NB_TOUCH] = {0};
  static uint32_t _y[TS_MAX_NB_TOUCH] = {0};
  uint8_t ts_status = TS_OK;
  uint16_t x[TS_MAX_NB_TOUCH];
  uint16_t y[TS_MAX_NB_TOUCH];
  uint16_t brute_x[TS_MAX_NB_TOUCH];
  uint16_t brute_y[TS_MAX_NB_TOUCH];
  uint16_t x_diff;
  uint16_t y_diff;
  uint32_t index;
#if (TS_MULTI_TOUCH_SUPPORTED == 1)
  uint32_t weight = 0;
  uint32_t area = 0;
  uint32_t event = 0;
#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

  /* Check and update the number of touches active detected */
	if(Touch_Controller == STMPE811)	
	{
		TS_State->touchDetected = tsDriver->DetectTouch(I2cAddress);
	}
	else if(Touch_Controller == TSC2013)	  
	{
		TS_State->touchDetected = tsc2013_Detect_touch();
  }
	
  if(TS_State->touchDetected)
  {
		Touch_Press = 1;	//commented only for the debug software
    for(index=0; index < TS_State->touchDetected; index++)
    {
      /* Get each touch coordinates */
			if(Touch_Controller == STMPE811)	
			{
				tsDriver->GetXY(I2cAddress, &(brute_x[index]), &(brute_y[index]));
			}
			else if(Touch_Controller == TSC2013)	  
			{
				tsc2013_read_touch_data(&(brute_x[index]), &(brute_y[index]));
			}

      if(tsOrientation == TS_SWAP_NONE)
      {
        x[index] = brute_x[index];
        y[index] = brute_y[index];
      }

      if(tsOrientation & TS_SWAP_X)
      {
        x[index] = 4096 - brute_x[index];
				y[index] = brute_y[index];
      }

      if(tsOrientation & TS_SWAP_Y)
      {
				x[index] = brute_x[index];
        y[index] = 4096 - brute_y[index];
      }

      if(tsOrientation & TS_SWAP_XY)
      {
        y[index] = brute_x[index];
        x[index] = brute_y[index];
      }

      x_diff = x[index] > _x[index]? (x[index] - _x[index]): (_x[index] - x[index]);
      y_diff = y[index] > _y[index]? (y[index] - _y[index]): (_y[index] - y[index]);

      if ((x_diff + y_diff) > 5)
      {
        _x[index] = x[index];
        _y[index] = y[index];
      }

      if(I2cAddress == TS_I2C_ADDRESS)
      {
        TS_State->touchX[index] = x[index];
        TS_State->touchY[index] = y[index];
      }
      else
      {
        /* 2^12 = 4096 : indexes are expressed on a dynamic of 4096 */
        TS_State->touchX[index] = (tsXBoundary * _x[index]) >> 12;
        TS_State->touchY[index] = (tsYBoundary * _y[index]) >> 12;
      }
        TS_State->touchX[index] = (tsXBoundary * x[index]) >> 12;
        TS_State->touchY[index] = (tsYBoundary * y[index]) >> 12;

#if (TS_MULTI_TOUCH_SUPPORTED == 1)

      /* Get touch info related to the current touch */
      ft5336_TS_GetTouchInfo(I2cAddress, index, &weight, &area, &event);

      /* Update TS_State structure */
      TS_State->touchWeight[index] = weight;
      TS_State->touchArea[index]   = area;

      /* Remap touch event */
      switch(event)
      {
        case FT5336_TOUCH_EVT_FLAG_PRESS_DOWN	:
          TS_State->touchEventId[index] = TOUCH_EVENT_PRESS_DOWN;
          break;
        case FT5336_TOUCH_EVT_FLAG_LIFT_UP :
          TS_State->touchEventId[index] = TOUCH_EVENT_LIFT_UP;
          break;
        case FT5336_TOUCH_EVT_FLAG_CONTACT :
          TS_State->touchEventId[index] = TOUCH_EVENT_CONTACT;
          break;
        case FT5336_TOUCH_EVT_FLAG_NO_EVENT :
          TS_State->touchEventId[index] = TOUCH_EVENT_NO_EVT;
          break;
        default :
          ts_status = TS_ERROR;
          break;
      } /* of switch(event) */

#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

    } /* of for(index=0; index < TS_State->touchDetected; index++) */

#if (TS_MULTI_TOUCH_SUPPORTED == 1)
    /* Get gesture Id */
    ts_status = BSP_TS_Get_GestureId(TS_State);
#endif /* TS_MULTI_TOUCH_SUPPORTED == 1 */

  } /* end of if(TS_State->touchDetected != 0) */
	else	
	{
	  Touch_Press = 0;	//commented only for the debug software
	}
  return (ts_status);
}


/*****END OF FILE****/
