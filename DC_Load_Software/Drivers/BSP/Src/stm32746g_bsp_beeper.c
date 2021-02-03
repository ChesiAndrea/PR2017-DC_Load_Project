#include "stm32746g_bsp_beeper.h"

//==========================================================================================================================================
//==========================================================================================================================================
// Beeper 
//==========================================================================================================================================
//==========================================================================================================================================
extern void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void Beeper_Init(TIM_HandleTypeDef * htim , uint32_t TIM_CHANNEL) {
TIM_MasterConfigTypeDef sMasterConfig;
TIM_OC_InitTypeDef sConfigOC;

		htim->Instance = TIM5;
		//APB1 Prescaler è 4 quindi Clock Timer è APB1_Clk x 2 => (SystemCoreClock / 4) * 2 
		htim->Init.Prescaler = (uint32_t)((float)(SystemCoreClock/2))/(1000000); 
		htim->Init.CounterMode = TIM_COUNTERMODE_UP;
		htim->Init.Period = 225;
		htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		HAL_TIM_PWM_Init(htim);

		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig);

		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = 112;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL);

		HAL_TIM_MspPostInit(htim);	
	
}



/**
  * @brief  Change PWM period with duty 50%
  * @param  value 1-7 (Music Note G major scale)
  * @retval None
  */

void Set_Beep_Note( uint32_t Val) {
	
uint32_t uwBeepPeriod;
TIM_OC_InitTypeDef sConfigOC;

//-- Il clock che pilota il Timer è impostato a 1MHz
	
//-- G major
	
		switch(Val) {
			
			case 1:
				/* Compute the Timer period to generate a signal frequency around 4434 Hz 
					Period = Fclk timer / Fout  */
				uwBeepPeriod = (1000000 / 3136 ) - 1;					
			break;

			case 2:
				/* Compute the Timer period to generate a signal frequency around 4434 Hz 
					Period = Fclk timer / Fout  */
				uwBeepPeriod = (1000000 / 3520 ) - 1;					
			break;
	
			case 3:
				/* Compute the Timer period to generate a signal frequency around 4434 Hz 
					Period = Fclk timer / Fout  */
				uwBeepPeriod = (1000000 / 3951 ) - 1;					
			break;
			
			case 4:
				/* Compute the Timer period to generate a signal frequency around 4434 Hz 
					Period = Fclk timer / Fout  */
				uwBeepPeriod = (1000000 / 4186 ) - 1;					
			break;

			case 5:
				/* Compute the Timer period to generate a signal frequency around 4434 Hz 
					Period = Fclk timer / Fout  */
				uwBeepPeriod = (1000000 / 4698 ) - 1;					
			break;
			
			case 6:
				/* Compute the Timer period to generate a signal frequency around 4434 Hz 
					Period = Fclk timer / Fout  */
				uwBeepPeriod = (1000000 / 5274 ) - 1;					
			break;

			case 7:
				/* Compute the Timer period to generate a signal frequency around 4434 Hz 
					Period = Fclk timer / Fout  */
				uwBeepPeriod = (1000000 / 5920 ) - 1;					
			break;
			
			default:
				/* Compute the Timer period to generate a signal frequency around 4434 Hz 
					Period = Fclk timer / Fout  */
				uwBeepPeriod = (1000000 / 4698 ) - 1;				
			break;			

		}
	
		My_BEEPER.Init.Period = uwBeepPeriod;
		HAL_TIM_PWM_Init(&My_BEEPER);
		
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = uwBeepPeriod/2 + 1; 
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		HAL_TIM_PWM_ConfigChannel(&My_BEEPER, &sConfigOC, My_CH_BEEP);		
	
}


#define Key_Click_Time		30

uint8_t Key_Click;
uint16_t Beep_Count;
uint8_t Beep_Action;
uint8_t Beep_Start;

/**
  * @brief  Beeper Start
  * @param  None
  * @retval None
  */

void Set_Beeper(void) {
	Beep_Start=1;
}

/**
  * @brief  Beeper  Stop 
  * @param  None
  * @retval None
  */

void Res_Beeper(void) {
	Beep_Start=0;
}


/**
  * @brief  Beeper Toggle
  * @param  None
  * @retval None
  */

void Tog_Beeper(void) {
	Beep_Start^=1;	
}

HAL_StatusTypeDef beepres;

uint8_t BeepStatus,LAST_BeepStatus;


void Beeper_Clk(void) {
	if (Key_Click) {
			
		if(Key_Click == (Key_Click_Time & 0x03)) {
			
				Set_Beep_Note(7);
				BeepStatus=1;
		
		}
		if(Key_Click > 0) Key_Click--;
		
	}
	else {
		
			if(Beep_Start != 0) {
			
					BeepStatus=1;

			}
			else {
					if(Beep_Action != 0) {
						if((Beep_Count & 0x3F)==0x00) {Set_Beep_Note(5);BeepStatus=1;}
						if((Beep_Count & 0x3F)==0x0F) {BeepStatus=0;}
						if((Beep_Count & 0x3F)==0x10) {Set_Beep_Note(1);BeepStatus=1;}
						if((Beep_Count & 0x3F)==0x1F) {BeepStatus=0;}
						Beep_Count++;
					}
					else {
						BeepStatus=0;
						Beep_Count=0;
					}
			
			}
			
	}
	
	
	if( LAST_BeepStatus != BeepStatus ) {
			if(BeepStatus) beepres=HAL_TIM_PWM_Start(&My_BEEPER, My_CH_BEEP); else beepres=HAL_TIM_PWM_Stop(&My_BEEPER, My_CH_BEEP);
	}
	LAST_BeepStatus = BeepStatus;
}

void Beep_on_Keys(void) {
	Key_Click = (Key_Click_Time & 0x03);
}

/**
  * @brief  Beeper Start
  * @param  None
  * @retval None
  */

void Set_BeepStream(void) {
	Beep_Action |= 0x01;
}

/**
  * @brief  Beeper  Stop 
  * @param  None
  * @retval None
  */

void Res_BeepStream(void) {
	Beep_Action &= ~0x01;
}


/**
  * @brief  Beeper  Toggle 
  * @param  None
  * @retval None
  */

void Tog_BeepStream(void) {
		Beep_Action ^= 0x01;
}
