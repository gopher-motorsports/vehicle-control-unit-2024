/*
 * vcu.c
 *
 *  Created on: Dec 4, 2022
 *      Author: Ben Abbott
 */

#include "vcu.h"
#include "gopher_sense.h"
#include <stdlib.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "main.h"
// The HAL_CAN struct
CAN_HandleTypeDef* hcan;

#define HBEAT_LED_DELAY_TIME_ms 500
#define RTD_DEBOUNCE_TIME_ms 25
//#define SET_INV_DISABLED() do{ desiredCurrent_A = 0; maxcurrentLimit_A = MAX_TEST_CMD_CURRENT_A; inverter_enable_state = INVERTER_DISABLE; } while(0)

BUTTON swButton0 = {
    .param = &swButon0_state,
    .port = FB0_GPIO_Port,
    .pin = FB0_Pin
};

BUTTON swButton1 = {
    .param = &swButon1_state,
    .port = FB1_GPIO_Port,
    .pin = FB1_Pin
};


BUTTON swButton4 = {
    .param = &swButon4_state,
    .port = FB4_GPIO_Port,
    .pin = FB4_Pin
};


BUTTON* buttons[NUM_OF_BUTTONS] = {
	&swButton0,
    &swButton1,
  	&swButton4,
};

// Initialization code goes here
void init(CAN_HandleTypeDef* hcan_ptr) {
	hcan = hcan_ptr;

	init_can(hcan, GCAN1);
}

void main_loop() {

	//process_sensors();
	update_gcan_states(); // Should be after proceass_sensors
	LED_task();
}

/**
 * Services the CAN RX and TX hardware task
 */
void can_buffer_handling_loop()
{
	// Handle each RX message in the buffer
	if (service_can_rx_buffer())
	{
		// An error has occurred
	}

	// Handle the transmission hardware for each CAN bus
	service_can_tx(hcan);
}



void update_gcan_states() {
	update_and_queue_param_u8(&swButon0_state,
			(HAL_GPIO_ReadPin(FB0_GPIO_Port, FB0_Pin) == 0));
	update_and_queue_param_u8(&swButon1_state,
			HAL_GPIO_ReadPin(FB1_GPIO_Port, FB1_Pin) == 0);
	update_and_queue_param_u8(&swButon4_state,
			HAL_GPIO_ReadPin(FB4_GPIO_Port, FB4_Pin) == 0);

}

/*
void process_sensors() {
	current_mode_button_state = swButon2_state.data;
	if(past_mode_button_state == 0 && current_mode_button_state == 1){
		current_driving_mode = !current_driving_mode;
	}
	past_mode_button_state = current_mode_button_state;

	// read in the RTD button. This is a software low pass to make sure noise does not press the button
	static U32 new_event_time;
	static U8 new_event = FALSE;
	if (!new_event)
	{
		// check if there is a change in polarity of the button
		if (readyToDriveButtonPressed_state != (HAL_GPIO_ReadPin(RTD_BUTTON_GPIO_Port, RTD_BUTTON_Pin) == RTD_BUTTON_PUSHED))
		{
			new_event = TRUE;
			new_event_time = HAL_GetTick();
		}
	}
	else
	{
		// the button change was not held long enough
		if (readyToDriveButtonPressed_state == (HAL_GPIO_ReadPin(RTD_BUTTON_GPIO_Port, RTD_BUTTON_Pin) == RTD_BUTTON_PUSHED))
		{
			new_event = FALSE;
		}
		else
		{
			// see if enough time has passed to actually call this a press
			if (HAL_GetTick() - new_event_time >= RTD_DEBOUNCE_TIME_ms)
			{
				new_event = FALSE;
				readyToDriveButtonPressed_state = !readyToDriveButtonPressed_state;
			}
		}
	}

	maxcurrentLimit_A = get_current_limit(current_driving_mode);


	update_struct_fault_data(); //refresh sensor data
	// Input Validation (Out of Range) and Correlation Checks
	SOFTWARE_FAULT* fault;
	for(int i = 0; i < NUM_OF_TIMED_FAULTS; i++){
		fault = TIMED_SOFTWARE_FAULTS[i];
		if(fault->data > fault->max_threshold || fault->data < fault->min_threshold){ //correlation has no min, but edge case accounted for in defines
			fault->fault_timer++;
			if(fault->fault_timer > fault->input_delay_threshold){
				fault->fault_timer = fault->input_delay_threshold + 1; //cap at delay_threshold + 1 so that it trips but doesn't count up more
			}
		}
		else{
			fault->fault_timer = 0;
			fault->state = false;
		}

		if(fault->fault_timer > fault->input_delay_threshold){
			fault->state = true;
			maxcurrentLimit_A = 0;
		}
	}


	// APPS/Braking Pedal Plausibility Check, edge case handled without struct
	if(brakePressureFront_psi.data > APPS_BRAKE_PRESS_THRESH_psi && pedalPosition1_mm.data > APPS_BRAKE_APPS1_THRESH_mm) {
		appsBrakeLatched_state = TRUE;
	} else if (pedalPosition1_mm.data <= APPS_BRAKE_RESET_APPS1_THRESH_mm) {
		appsBrakeLatched_state = FALSE;
	}

	if(!BYPASS_ACTIVE){
		if(brakePressureFront_psi.data > APPS_BRAKE_PRESS_THRESH_psi && pedalPosition1_percent.data > 25) {
				appsBrakeLatched_state = TRUE;
		} else if (pedalPosition1_percent.data <= 5) {
			appsBrakeLatched_state = FALSE;
		}

		if(appsBrakeLatched_state) {
			maxcurrentLimit_A = 0;
		}
	}
	//Sensor overcurrent Logic, turn off power to inverter if any of the sensor power lines are overcurrenting
#ifdef USING_SOFTWARE_OVERCURRENT_PROT
	Current_Fault_3V3_state = HAL_GPIO_ReadPin(CURR_FAULT_3V3_GPIO_Port, CURR_FAULT_3V3_Pin) == SENSOR_OVERCURRENT_TRIPPED; //active low
	Current_Fault_5V_state  = HAL_GPIO_ReadPin(CURR_FAULT_5V_GPIO_Port, CURR_FAULT_5V_Pin) == SENSOR_OVERCURRENT_TRIPPED; //active low

	static U32 overcurrent_event_timer_3V3 = 0;
	static U32 overcurrent_event_timer_5V = 0;

	if(Current_Fault_3V3_state){
		overcurrent_event_timer_3V3++;
		if(overcurrent_event_timer_3V3 >= SENSOR_OVERCURRENT_TIME_THRESH)
			maxcurrentLimit_A = 0;
	}
	else{
		overcurrent_event_timer_3V3 = 0;
	}

	if(Current_Fault_5V_state){
		overcurrent_event_timer_5V++;
		if(overcurrent_event_timer_5V >= SENSOR_OVERCURRENT_TIME_THRESH)
			maxcurrentLimit_A = 0;
	}
	else{
		overcurrent_event_timer_5V = 0;
	}
#endif
	// TODO make some hysteresis on this in order to make it less jumpy
	if(!BYPASS_ACTIVE){
		if(bspdTractiveSystemBrakingFault_state.data) {
			float tractiveSystemBrakingLimit_A = 0;
			if(inputInverterVoltage_V.data != 0) {
				tractiveSystemBrakingLimit_A = bspd_power_limit / inputInverterVoltage_V.data; //stay below 5 kW I = P/V
			}
			// If the tractive system braking limit is less (more restrictive),
			// then set the torque limit to that amount
			if(tractiveSystemBrakingLimit_A < maxcurrentLimit_A) {
				update_and_queue_param_u8(&vcuBrakingClampingCurrent_state, TRUE);
				maxcurrentLimit_A = tractiveSystemBrakingLimit_A;
			} else {
				update_and_queue_param_u8(&vcuBrakingClampingCurrent_state, FALSE);
			}
		}
	}
	desiredCurrent_A = ((pedalPosition1_mm.data-APPS_1_MIN_CURRENT_POS_mm)/APPS_1_TOTAL_TRAVEL_mm) * get_current_limit(current_driving_mode);

	if(pedalPosition1_mm.data < APPS_1_MIN_CURRENT_POS_mm) {
		desiredCurrent_A = 0;
	}

	if(desiredCurrent_A > get_current_limit(current_driving_mode)) {
		desiredCurrent_A =  get_current_limit(current_driving_mode);
	}
}
*/


void LED_task(){
	static U32 last_led = 0;
	if(HAL_GetTick() - last_led >= HBEAT_LED_DELAY_TIME_ms) {
		HAL_GPIO_TogglePin(MCU_STATUS_LED_GPIO_Port, MCU_STATUS_LED_Pin);
		last_led = HAL_GetTick();
	}

	// Turn off RGB
//	HAL_GPIO_WritePin(STATUS_R_GPIO_Port, STATUS_R_Pin, SET);
//	HAL_GPIO_WritePin(STATUS_G_GPIO_Port, STATUS_G_Pin, SET);
//	HAL_GPIO_WritePin(STATUS_B_GPIO_Port, STATUS_B_Pin, SET);
}

// End of vcu.c
