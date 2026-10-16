#include <stdint.h>
#include <stdio.h>
#include "main.h"
#include "status_led.h"
#include "GopherCAN.h"
#include "GopherCAN_config.h"


extern TIM_HandleTypeDef htim2;

void initStatus_LEDs(void){
    HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, 0); // green 
    HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin, 0); // yellow
    HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin, 0); // red
    HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin, 0); // buzzer
}

uint8_t red_LED(void){
	if (//bmsChargerOverVoltageAlert_state.data||
	//bmsChargerOverCurrentAlert_state.data||
	//bmsChargerVoltageMismatchAlert_state.data||
	//bmsChargerCurrentMismatchAlert_state.data||
	//bmsChargerHardwareFailureAlert_state.data||
	//bmsChargerOverTempAlert_state.data||
	//bmsChargerInputVoltageErrorAlert_state.data||
	//bmsChargerBatteryNotDetectedErrorAlert_state.data||
	//bmsChargerCommunicationErrorAlert_state.data
		0){
		return 1;
	}
	return 0;
}

uint8_t yellow_LED(void){
	if(//sdcStatus1.data&&
	   //sdcStatus2.data&&
	   //sdcStatus3.data&&
	   //sdcStatus4.data&&
	   //chargerStatusByte.data
	   1){
		return 1;
	   }
	return 0;
}

uint8_t green_LED(void){
	if(//!sdcStatus1.data&&
        //!sdcStatus2.data&&
        //!sdcStatus3.data&&
        //!sdcStatus4.data&&
        //chargerStatusByte.data
        0){
		return 1;
	} 
	return 0;
}

void buzzer(void){
		HAL_GPIO_WritePin(LED4_GPIO_Port,LED4_Pin,1);
		HAL_TIM_Base_Init(&htim2);
		HAL_TIM_Base_Start_IT(&htim2); //start interrupt
		__HAL_TIM_SET_COUNTER(&htim2,0); // reset
	}
	

void check_LEDs(void){
    static uint8_t last_error = 0;
	uint8_t ready_state = green_LED();
	uint8_t standby_state = yellow_LED();
    uint8_t current_error = red_LED();

    if(current_error){
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 1);
        if (!last_error) {
            buzzer();
        }
    }
	else{
		HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, 0);
	}

    if(ready_state && !current_error && !standby_state){
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);  // green on
    }
	else{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
	}

	if (standby_state && !current_error && !ready_state){
		HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin, 1);
	}
	else{
		HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin, 0);
	}
		
	last_error = current_error;
	
}