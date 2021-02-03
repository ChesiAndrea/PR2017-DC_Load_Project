#ifndef TSC2013_H
#define TSC2013_H

#ifdef __cplusplus
 extern "C" {
#endif   
	 
/* Includes ------------------------------------------------------------------*/
#include "stm32746g_bsp_ts.hpp"	 
	 
#define TSC2013_I2C_ADDRESS	 													 0x90
#define TSC2013_PINTDAV_PIN                            GPIO_PIN_4
#define TSC2013_PINTDAV_PORT													 GPIOD
	 
#define TSC_IRQ                                        BIT7
#define TSC_RST                                        BIT0

#define REG_CFR0_WR                                    ((0x0c << 3) | 0x02)  
#define REG_CFR1_WR                                    ((0x0d << 3) | 0x02)
#define REG_CFR2_WR                                    ((0x0e << 3) | 0x02)
#define REG_CFN_WR                                     ((0x0f << 3) | 0x02)

#define REG_X1_RD                                      ((0x00 << 3) | 0x03)
#define REG_STATUS_RD                                  ((0x08 << 3) | 0x03)
#define REG_CFR0_RD                                    ((0x0c << 3) | 0x03)  
#define REG_CFR1_RD                                    ((0x0d << 3) | 0x03)
#define REG_CFR2_RD                                    ((0x0e << 3) | 0x03)
#define REG_CFN_RD                                     ((0x0f << 3) | 0x03)

#define CFR0_PSM_0                                     (0 << 15)
#define CFR0_PSM_1                                     (1 << 15)

#define CFR0_STS_0                                     (0 << 14)
#define CFR0_STS_1                                     (1 << 14)

#define CFR0_RESOLUTION_10_BIT                         (0 << 13)
#define CFR0_RESOLUTION_12_BIT                         (1 << 13)

#define CFR0_CONVERSION_CLOCK_4MHz                     (0 << 11)
#define CFR0_CONVERSION_CLOCK_2MHz                     (1 << 11)
#define CFR0_CONVERSION_CLOCK_1MHz                     (2 << 11)

#define CFR0_PANEL_VOLTAGE_STABILIZATION_TIME_0        (0 << 8) //   0uS
#define CFR0_PANEL_VOLTAGE_STABILIZATION_TIME_1        (1 << 8) // 100uS
#define CFR0_PANEL_VOLTAGE_STABILIZATION_TIME_2        (2 << 8) // 500uS
#define CFR0_PANEL_VOLTAGE_STABILIZATION_TIME_3        (3 << 8) //   1mS
#define CFR0_PANEL_VOLTAGE_STABILIZATION_TIME_4        (4 << 8) //   5mS
#define CFR0_PANEL_VOLTAGE_STABILIZATION_TIME_5        (5 << 8) //  10mS
#define CFR0_PANEL_VOLTAGE_STABILIZATION_TIME_6        (6 << 8) //  50mS
#define CFR0_PANEL_VOLTAGE_STABILIZATION_TIME_7        (7 << 8) // 100mS

#define CFR0_PRECHARGE_TIME_0                          (0 << 5)
#define CFR0_PRECHARGE_TIME_1                          (1 << 5)
#define CFR0_PRECHARGE_TIME_2                          (2 << 5)
#define CFR0_PRECHARGE_TIME_3                          (3 << 5)
#define CFR0_PRECHARGE_TIME_4                          (4 << 5)
#define CFR0_PRECHARGE_TIME_5                          (5 << 5)
#define CFR0_PRECHARGE_TIME_6                          (6 << 5)
#define CFR0_PRECHARGE_TIME_7                          (7 << 5)

#define CFR0_SENSE_TIME_0                              (0 << 2)
#define CFR0_SENSE_TIME_1                              (1 << 2)
#define CFR0_SENSE_TIME_2                              (2 << 2)
#define CFR0_SENSE_TIME_3                              (3 << 2)
#define CFR0_SENSE_TIME_4                              (4 << 2)
#define CFR0_SENSE_TIME_5                              (5 << 2)
#define CFR0_SENSE_TIME_6                              (6 << 2)
#define CFR0_SENSE_TIME_7                              (7 << 2)

#define CFR0_DTW_ENABLED                               (1 << 1)
#define CFR0_DTW_DISABLED                              (0 << 1)

#define CFR0_LSM_ENABLED                               1
#define CFR0_LSM_DISABLED                              0

#define CFR1_BTD_0                                     0
#define CFR1_BTD_1                                     1
#define CFR1_BTD_2                                     2
#define CFR1_BTD_3                                     3
#define CFR1_BTD_4                                     4
#define CFR1_BTD_5                                     5
#define CFR1_BTD_6                                     6
#define CFR1_BTD_7                                     7

#define CFR2_PINTS_0                                   (0 << 14)
#define CFR2_PINTS_1                                   (1 << 14)
#define CFR2_PINTS_2                                   (2 << 14)
#define CFR2_PINTS_3                                   (3 << 14)

#define CFR2_M_0                                       (0 << 12)
#define CFR2_M_1                                       (1 << 12)
#define CFR2_M_2                                       (2 << 12)
#define CFR2_M_3                                       (3 << 12)

#define CFR2_W_0                                       (0 << 10)
#define CFR2_W_1                                       (1 << 10)
#define CFR2_W_2                                       (2 << 10)
#define CFR2_W_3                                       (3 << 10)

#define CFR2_MAVE_X_ENABLED                            (1 << 4)
#define CFR2_MAVE_X_DISABLED                           (0 << 4)

#define CFR2_MAVE_Y_ENABLED                            (1 << 3)
#define CFR2_MAVE_Y_DISABLED                           (0 << 3)

#define CFR2_MAVE_Z_ENABLED                            (1 << 2)
#define CFR2_MAVE_Z_DISABLED                           (0 << 2)

#define CFR2_MAVE_AUX_ENABLED                          (1 << 1)
#define CFR2_MAVE_AUX_DISABLED                         (0 << 1)

#define TSC2013_STATUS_DAVX                            (1 << 15)
#define TSC2013_STATUS_DAVY                            (1 << 14)
#define TSC2013_STATUS_DAVZ1                           (1 << 13)
#define TSC2013_STATUS_DAVZ2                           (1 << 12)
#define TSC2013_STATUS_RESET                     			 (1 <<  7)
uint8_t  tsc2013_init(I2C_HandleTypeDef* hi2c);
uint8_t  tsc2013_write_config_values(void);
uint8_t  tsc2013_Detect_touch(void);
uint16_t tsc2013_read_touch_status(void);
void 		 tsc2013_read_touch_data(uint16_t *X, uint16_t *Y);
void     IOE_Init(I2C_HandleTypeDef* hi2c);
void     IOE_Interrupt_Enable(I2C_HandleTypeDef* hi2c);
void     IOE_ITConfig (void);
void     IOE_Delay(uint32_t delay);
void     IOE_Write(I2C_HandleTypeDef* hi2c,uint8_t addr, uint8_t reg, uint8_t value);
void 		 IOE_Transmit(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint8_t *Buffer, uint16_t Length);
void 		 IOE_WriteMultiple(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
uint8_t  IOE_Read(I2C_HandleTypeDef* hi2c,uint8_t addr, uint8_t reg);
uint16_t IOE_ReadMultiple(I2C_HandleTypeDef* hi2c,uint8_t addr, uint8_t reg, uint8_t *buffer, uint16_t length);
uint16_t IOE_Master_Receive(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint8_t *Buffer, uint16_t Length);
uint16_t IOE_Mem_Read_It(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
uint16_t IOE_Mem_Read(I2C_HandleTypeDef* hi2c, uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length);
typedef union
{
   uint16_t value;
   struct
   {
      uint8_t low_byte;
      uint8_t high_byte;
   }bytes;
}tsc2013_register;

typedef union
{
  uint8_t buffer[18];
  struct
  {  
    uint16_t X1;
    uint16_t X2;
    uint16_t Y1;
    uint16_t Y2;
    uint16_t IX;
    uint16_t IY;
    uint16_t Z1;
    uint16_t Z2;
    uint16_t Status;
  }values;
}tsc2013_data_set;

extern volatile uint16_t g_touch_detected;

#ifdef __cplusplus
}
#endif
#endif
