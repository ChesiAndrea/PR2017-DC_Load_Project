#include "tsc2013.h"
#include "stm32746g_bsp_ts.hpp"
#include "stm32746g_bsp.hpp"

volatile uint16_t g_touch_detected;
extern I2C_HandleTypeDef hi2c2;
extern uint8_t IOE_GetErrorAck(void);
tsc2013_data_set data_set;
uint8_t valid_Status = 0;
uint8_t Reset_Happined = 0;
static void tsc2013_i2c_init(void);
static void tsc2013_reset(void);
static void tsc2013_interrupt_init(void);

static void tsc2013_i2c_write_register(uint8_t reg_address, uint16_t value);
static void tsc2013_i2c_write_byte(uint16_t value);
static void tsc2013_i2c_Mem_Read_It(uint16_t value, uint8_t *data, uint16_t length);
static void tsc2013_i2c_Mem_Read(uint16_t value, uint8_t *data, uint16_t length);
static void tsc2013_i2c_receive(uint8_t *data, uint16_t length);
static void swap(uint8_t *high_byte, uint8_t *low_byte);
I2C_HandleTypeDef* Tsc2013_I2C;

uint8_t I2C_Mutex = 0;

uint8_t tsc2013_init(I2C_HandleTypeDef* hi2c)
{
	Tsc2013_I2C = hi2c;
  tsc2013_i2c_init();    
	
  tsc2013_reset();

  return TS_OK;
}

void tsc2013_i2c_init(void)
{
	IOE_Init(Tsc2013_I2C); 
}

void tsc2013_interrupt_init(void)
{
	/*Configure GPIO pin : PINTDAV */
	GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = TSC2013_PINTDAV_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TSC2013_PINTDAV_PORT, &GPIO_InitStruct);
	
  HAL_NVIC_SetPriority(EXTI4_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

uint8_t Temp_Read[18];
uint8_t Pintdav_IRQ = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_4)
  {
		Pintdav_IRQ++;
	}
}

void tsc2013_read_touch_data(uint16_t *X, uint16_t *Y){  
	if (data_set.values.X1 >= data_set.values.X2)
	{
		*X = data_set.values.X2 + ((data_set.values.X1 - data_set.values.X2) >> 1);
	}
	else
	{
		*X = data_set.values.X1 + ((data_set.values.X2 - data_set.values.X1) >> 1);         
	}
	
	if (data_set.values.Y1 >= data_set.values.Y2)
	{
		*Y = data_set.values.Y2 + ((data_set.values.Y1 - data_set.values.Y2) >> 1);
	}
	else
	{
		*Y = data_set.values.Y1 + ((data_set.values.Y2 - data_set.values.Y1) >> 1);         
	}
}

uint8_t tsc2013_Detect_touch(void){
	tsc2013_i2c_Mem_Read(REG_X1_RD, Temp_Read, 18);
	for (uint8_t i = 0; i < (18/2); i++)	{swap(&Temp_Read[i * 2],&Temp_Read[i*2+1]);}
	for (uint8_t i = 1; i < 16; i=i+2)		{Temp_Read[i] &= 0x1F;}
	for (uint8_t i = 0; i < 18; i++)  		{data_set.buffer[i] = Temp_Read[i];}		

	if(Pintdav_IRQ > 1)
	{
		Pintdav_IRQ = 1;
		if((data_set.values.Status & TSC2013_STATUS_RESET) == TSC2013_STATUS_RESET) // NO reset
		{	
			return 1;
		}
		else
		{
			Reset_Happined = 1;
		}	
	}
	else
	{
		Pintdav_IRQ = 0;		
	}
	return 0;
}

void tsc2013_reset(void){
	//power TS controller off  
	TouchControllerOff(Tsc2013_I2C);	
	IOE_Delay(10);

	//power TS controller on
	TouchControllerOn(Tsc2013_I2C);
	IOE_Delay(5);		
}

uint16_t tsc2013_read_touch_status(void){   
	if(Reset_Happined)
	{
		Reset_Happined = 0;
		return 0;
	}
  tsc2013_i2c_Mem_Read(REG_X1_RD, Temp_Read, 18);
  return data_set.values.Status;
}

void tsc2013_i2c_write_byte(uint16_t value)
{
	IOE_Transmit(Tsc2013_I2C, TSC2013_I2C_ADDRESS, (uint8_t*)&value, 0x01);
}

void tsc2013_i2c_Mem_Read_It(uint16_t value, uint8_t *data, uint16_t length)
{
	IOE_Mem_Read_It(Tsc2013_I2C, TSC2013_I2C_ADDRESS, (uint8_t)value, data, length);
}

void tsc2013_i2c_Mem_Read(uint16_t value, uint8_t *data, uint16_t length)
{
	IOE_Mem_Read(Tsc2013_I2C, TSC2013_I2C_ADDRESS, (uint8_t)value, data, length);
}

void tsc2013_i2c_write_register(uint8_t reg_address, uint16_t value)
{
	uint16_t RegisterLength = 2;
	swap((uint8_t*)&value, (uint8_t*)&value+1);
	IOE_WriteMultiple(Tsc2013_I2C, TSC2013_I2C_ADDRESS, reg_address, (uint8_t*)&value, RegisterLength);
}

void tsc2013_i2c_receive(uint8_t *data, uint16_t length)
{  
	IOE_Master_Receive(Tsc2013_I2C, (TSC2013_I2C_ADDRESS|1), data, length);
  for (uint8_t i = 0; i < (length/2); i++)
  {
     swap(&data[i * 2],&data[i*2+1]);
  }
}

void swap(uint8_t *high_byte, uint8_t *low_byte)
{
  uint8_t temp;
  temp = *high_byte;
  *high_byte = *low_byte;
  *low_byte = temp;
}

uint8_t tsc2013_write_config_values(void)
{
  tsc2013_register reg;
  uint16_t value;
  
  value = 		CFR0_PSM_1 |   //Conversion Inizialized by the tsc 	
              CFR0_STS_0 |   //Normal Operation
              CFR0_RESOLUTION_12_BIT | 
              CFR0_CONVERSION_CLOCK_1MHz | 
              CFR0_PANEL_VOLTAGE_STABILIZATION_TIME_3|   //1ms
              CFR0_PRECHARGE_TIME_4 |                     //1.044ms
              CFR0_SENSE_TIME_3 |                         //608us
              CFR0_DTW_DISABLED |
              CFR0_LSM_DISABLED; 
  tsc2013_i2c_write_register(REG_CFR0_WR, value);
	tsc2013_i2c_write_byte(	REG_CFR0_RD	);
  tsc2013_i2c_receive((uint8_t *)&reg.value, 2);
 	if((reg.value & 0x3FFF) != (value & 0x3FFF))
	{
		return TS_ERROR;
	}  
	
	
  value = CFR1_BTD_4; //10mS
  tsc2013_i2c_write_register(REG_CFR1_WR, value);  
	tsc2013_i2c_write_byte( REG_CFR1_RD );
  tsc2013_i2c_receive((uint8_t *)&reg.value, 2);
 	if ((reg.value&0x07) != (value&0x07))
	{
		return TS_ERROR;
	}  
  
  value = 		CFR2_PINTS_0 |  //DAV
              CFR2_M_1 |     	//M
              CFR2_W_1 |     	//W
              CFR2_MAVE_X_ENABLED |
              CFR2_MAVE_Y_ENABLED |
              CFR2_MAVE_Z_ENABLED |
              CFR2_MAVE_AUX_DISABLED;
  tsc2013_i2c_write_register(REG_CFR2_WR, value);
	tsc2013_i2c_write_byte( REG_CFR2_RD );
  tsc2013_i2c_receive((uint8_t *)&reg.value, 2);
	if ((reg.value&0xFC1E) != (value&0xFC1E))
	{
		return TS_ERROR;
	}
 
  //Control Byte ID : 1
  //          C3:C0 : 0 
  //            RM  : 0  (10 Bit)
  //          SWRST : 0
  //            STS : 0
  tsc2013_i2c_write_byte(0x84);   //Scan X,Y,Z
	
	tsc2013_read_touch_status();
	
	tsc2013_interrupt_init();	
	
	return TS_OK;
}

