/*
 * vcu.c
 *
 *  Created on: Dec 4, 2022
 *      Author: Ben Abbott
 */

#include "vcu.h"
#include "vcu_software_faults.h"
#include "gopher_sense.h"
#include "drs.h"
#include <stdlib.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "main.h"
// The HAL_CAN struct
CAN_HandleTypeDef* hcan;

uint32_t maxCurrent_mA = 0;
uint32_t preDriveTimer_ms = 0;

float desiredCurrent_A = 0;
float maxcurrentLimit_A = 15;

float torqueLimit_Nm = 0;
float desiredTorque_Nm = 0;

boolean appsBrakeLatched_state = 0;

boolean Current_Fault_3V3_state = 0;
boolean Current_Fault_5V5_state = 0;

boolean readyToDriveButtonPressed_state = 0;

VEHICLE_STATE_t vehicle_state = VEHICLE_NO_COMMS;

TIM_HandleTypeDef* PWM_Timer;
U32 PUMP_Channel;

#define HBEAT_LED_DELAY_TIME_ms 500
#define RTD_DEBOUNCE_TIME_ms 25
#define SET_INV_DISABLED() do{ desiredCurrent_A = 0; maxcurrentLimit_A = MAX_TEST_CMD_CURRENT_A; inverter_enable_state = INVERTER_DISABLE; } while(0)

// Initialization code goes here
void init(CAN_HandleTypeDef* hcan_ptr) {
	hcan = hcan_ptr;

	init_can(hcan, GCAN1);
}

void main_loop() {

	process_sensors();
	process_inverter();
	update_outputs();
	//update_cooling();
	update_display_fault_status();
	update_gcan_states(); // Should be after process_sensors
	LED_task();
	set_DRS_Servo_Position(FALSE);
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
	// Log pedal position percentages
	float pedalPos1 = 100.0*(pedalPosition1_mm.data-APPS_MIN_TORQUE_POS_mm)/APPS_TOTAL_TRAVEL_mm;
	if(pedalPos1 < 0) {
		pedalPos1 = 0;
	} else if (pedalPos1 > 100) {
		pedalPos1 = 100;
	}
	float pedalPos2 = 100.0*(pedalPosition2_mm.data-APPS_MIN_TORQUE_POS_mm)/APPS_TOTAL_TRAVEL_mm;
	if(pedalPos2 < 0) {
		pedalPos2 = 0;
	} else if (pedalPos2 > 100) {
		pedalPos2 = 100;
	}
	update_and_queue_param_float(&pedalPosition1_percent, pedalPos1);
	update_and_queue_param_float(&pedalPosition2_percent, pedalPos2);
	// Log BSPD sensor faults
	update_and_queue_param_u8(&bspdBrakePressureSensorFault_state,
			HAL_GPIO_ReadPin(BSPD_BRK_FAULT_GPIO_Port, BSPD_BRK_FAULT_Pin) == BSPD_BRAKE_FAULT);
	update_and_queue_param_u8(&bspdTractiveSystemCurrentSensorFault_state,
			HAL_GPIO_ReadPin(BSPD_TS_SNS_FAULT_GPIO_Port, BSPD_TS_SNS_FAULT_Pin) == BSPD_TS_SNS_FAULT);
	// Log BSPD current/braking fault
	update_and_queue_param_u8(&bspdTractiveSystemBrakingFault_state,
			HAL_GPIO_ReadPin(BSPD_TS_BRK_FAULT_GPIO_Port, BSPD_TS_BRK_FAULT_Pin) == BSPD_TS_BRK_FAULT);
	// VSC sensors faults, out of range checks
	update_and_queue_param_u8(&vcuPedalPosition1Fault_state, TIMED_SOFTWARE_FAULTS[0]->state);
	update_and_queue_param_u8(&vcuPedalPosition2Fault_state, TIMED_SOFTWARE_FAULTS[1]->state);
	update_and_queue_param_u8(&vcuBrakePressureSensorFault_state, TIMED_SOFTWARE_FAULTS[2]->state);
	update_and_queue_param_u8(&vcuTractiveSystemCurrentSensorFault_state, TIMED_SOFTWARE_FAULTS[3]->state);
	// VSC safety checks, correlation and APPS/Brake Plausibility check
	update_and_queue_param_u8(&vcuPedalPositionCorrelationFault_state, TIMED_SOFTWARE_FAULTS[4]->state);
	update_and_queue_param_u8(&vcuPedalPositionBrakingFault_state, appsBrakeLatched_state);
	// Requested torque
	update_and_queue_param_float(&vcuTorqueRequested_Nm, desiredTorque_Nm);
	// Cooling
	update_and_queue_param_u8(&coolantFanPower_percent, 100*HAL_GPIO_ReadPin(RAD_FAN_GPIO_Port, RAD_FAN_Pin));
	//update_and_queue_param_u8(&coolantPumpPower_percent, 100*HAL_GPIO_ReadPin(PUMP_GPIO_Port, PUMP_Pin)); //TODO change to duty cycle percent
	// Vehicle state
	update_and_queue_param_u8(&vehicleState_state, vehicle_state);
	update_and_queue_param_u8(&readyToDriveButton_state, readyToDriveButtonPressed_state);

	update_and_queue_param_u8(&vcuGSenseStatus_state, HAL_GPIO_ReadPin(GSENSE_LED_GPIO_Port, GSENSE_LED_Pin));
	// Calculate wheel speed from rpm
	wheelSpeedRearRight_mph.data = ((motorSpeed_rpm.data * MINUTES_PER_HOUR) * WHEEL_DIAMETER_IN * MATH_PI) / (FINAL_DRIVE_RATIO * IN_PER_FT);
	wheelSpeedFrontLeft_mph.data = wheelSpeedFrontRight_mph.data;
}

void update_cooling() {
	// TODO: Ramp up cooling based on temperatures of the inverter and motor using PWM
	// TODO: Set pump and fan pins to PWM
	//change controlBoardTEMP_C to
    double temp_readings[] = {controlBoardTemp_C. data, motorTemp_C.data};
	static double cooling_thresholds[] = {INVERTER_TEMP_THRESH_C, MOTOR_TEMP_THRESH_C};
	double total_cooling_thresholds = sizeof(cooling_thresholds) / sizeof(cooling_thresholds[0]); //amount of cooling thresholds
	static U8 rad_fan_state = PLM_CONTROL_OFF;
	static U16 pwm_pump_intensity = PUMP_INTENSITY_OFF;
	int readings_below_HYS_threshold = 0;

	//rad fan cooling
	for(int i = 0; i < total_cooling_thresholds; i++){
		if(rad_fan_state == PLM_CONTROL_OFF && (temp_readings[i] >= (cooling_thresholds[i] + HYSTERESIS_DIGITAL))){
			rad_fan_state = PLM_CONTROL_ON;
			HAL_GPIO_WritePin(RAD_FAN_GPIO_Port, RAD_FAN_Pin, rad_fan_state);
			readings_below_HYS_threshold = 0;
			break;
		}

		else if(rad_fan_state == PLM_CONTROL_ON){
			if(temp_readings[i] <= (cooling_thresholds[i] - HYSTERESIS_DIGITAL)){
				readings_below_HYS_threshold++;
			}

			if(readings_below_HYS_threshold == total_cooling_thresholds){
				rad_fan_state = PLM_CONTROL_OFF;
				HAL_GPIO_WritePin(RAD_FAN_GPIO_Port, RAD_FAN_Pin, rad_fan_state);
			}
		}
	}

	//pump cooling
	for(int i = 0; i < total_cooling_thresholds; i++){
		if(pwm_pump_intensity == PUMP_INTENSITY_OFF && (temp_readings[i] >= (cooling_thresholds[i] + HYSTERESIS_ANALOG))){
			pwm_pump_intensity = PUMP_INTENSITY_1;
			__HAL_TIM_SET_COMPARE(PWM_Timer, PUMP_Channel, pwm_pump_intensity);
			HAL_TIM_PWM_Start(PWM_Timer, PUMP_Channel);
			readings_below_HYS_threshold = 0;
			break;
		}

		else if(pwm_pump_intensity > PUMP_INTENSITY_OFF){
			if(temp_readings[i] >= INVERTER_TEMP_THRESH_C_4 || temp_readings[i] >= MOTOR_TEMP_THRESH_C_4) pwm_pump_intensity = PUMP_INTENSITY_4;
			else if(temp_readings[i] >= INVERTER_TEMP_THRESH_C_3 || temp_readings[i] >= MOTOR_TEMP_THRESH_C_3) pwm_pump_intensity = PUMP_INTENSITY_3;
			else if(temp_readings[i] >= INVERTER_TEMP_THRESH_C_2 || temp_readings[i] >= MOTOR_TEMP_THRESH_C_2) pwm_pump_intensity = PUMP_INTENSITY_2;
			else if(temp_readings[i] >= INVERTER_TEMP_THRESH_C_1 || temp_readings[i] >= MOTOR_TEMP_THRESH_C_1) pwm_pump_intensity = PUMP_INTENSITY_1;
			else if(temp_readings[i] <= (cooling_thresholds[i] - HYSTERESIS_ANALOG)){
				readings_below_HYS_threshold++;
			}

			if(readings_below_HYS_threshold == total_cooling_thresholds){
				pwm_pump_intensity = PUMP_INTENSITY_OFF;
				HAL_TIM_PWM_Stop(PWM_Timer, PUMP_Channel);
			}
			__HAL_TIM_SET_COMPARE(PWM_Timer, PUMP_Channel, pwm_pump_intensity);
		}
	}
}

void process_sensors() {

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

	desiredCurrent_A = MAX_TEST_CMD_CURRENT_A;


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
			torqueLimit_Nm = 0;
		}
	}


	// APPS/Braking Pedal Plausibility Check, edge case handled without struct
	if(brakePressureFront_psi.data > APPS_BRAKE_PRESS_THRESH_psi && pedalPosition1_mm.data > APPS_BRAKE_APPS1_THRESH_mm) {
		appsBrakeLatched_state = TRUE;
	} else if (pedalPosition1_mm.data <= APPS_BRAKE_RESET_THRESH_mm) {
		appsBrakeLatched_state = FALSE;
	}

	if(appsBrakeLatched_state) {
		torqueLimit_Nm = 0;
	}

	//Sensor overcurrent Logic, turn off power to inverter if any of the sensor power lines are overcurrenting
	Current_Fault_3V3_state = HAL_GPIO_ReadPin(RTD_BUTTON_GPIO_Port, RTD_BUTTON_Pin) == SENSOR_OVERCURRENT_TRIPPED; //active low
	Current_Fault_5V5_state = HAL_GPIO_ReadPin(RTD_BUTTON_GPIO_Port, RTD_BUTTON_Pin) == SENSOR_OVERCURRENT_TRIPPED; //active low
	if(Current_Fault_3V3_state || Current_Fault_5V5_state){
		torqueLimit_Nm = 0;
	}

	// TODO make some hysteresis on this in order to make it less jumpy
	if(bspdTractiveSystemBrakingFault_state.data) {
		float tractiveSystemBrakingLimit_Nm = 0;
		float accumulatorVoltage_V = dcBusVoltage_V.data;
		if(motorSpeed_rpm.data != 0) {
			// Calculate max torque from speed and voltage, using angular velocity (rad/s)
			tractiveSystemBrakingLimit_Nm = (BRAKE_TS_CURRENT_THRESH_A * accumulatorVoltage_V) //stay below 5kw, when driver breaking hard
					/ ((motorSpeed_rpm.data * MATH_TAU) / SECONDS_PER_MIN);
		}
		// If the tractive system braking limit is less (more restrictive),
		// then set the torque limit to that amount
		if(tractiveSystemBrakingLimit_Nm < torqueLimit_Nm) {
			update_and_queue_param_u8(&vcuBrakingClampingCurrent_state, TRUE);
			torqueLimit_Nm = tractiveSystemBrakingLimit_Nm;
		} else {
			update_and_queue_param_u8(&vcuBrakingClampingCurrent_state, FALSE);
		}
	}

	desiredCurrent_A = ((pedalPosition1_mm.data-APPS_MIN_CURRENT_POS_mm)/APPS_TOTAL_TRAVEL_mm) * MAX_TEST_CMD_CURRENT_A;

	if(pedalPosition1_mm.data < APPS_MIN_CURRENT_POS_mm) {
		desiredCurrent_A = 0;
	}

	if(desiredCurrent_A >  MAX_TEST_CMD_CURRENT_A) {
		desiredCurrent_A =  MAX_TEST_CMD_CURRENT_A;
	}
}



void update_display_fault_status() {
	int status = NONE;
	if(amsFault_state.data) status = AMS_FAULT;
	else if(bmsNumActiveAlerts_state.data) status = BMS_FAULT;
	else if(vcuPedalPositionBrakingFault_state.data) status = RELEASE_PEDAL;
	else if(bspdTractiveSystemBrakingFault_state.data || vcuBrakingClampingCurrent_state.data) status = BRAKING_FAULT;
	else if(vcuPedalPositionCorrelationFault_state.data) status = APPS_FAULT;
	else if(bspdFault_state.data
			|| bspdBrakePressureSensorFault_state.data
			|| bspdPedalPosition1Fault_state.data
			|| bspdPedalPosition2Fault_state.data
			|| bspdTractiveSystemCurrentSensorFault_state.data
			) status = BSPD_FAULT;
	else if(vcuBrakePressureSensorFault_state.data
			|| vcuPedalPosition2Fault_state.data
			|| vcuTractiveSystemCurrentSensorFault_state.data
			) status = VCU_FAULT;

	update_and_queue_param_u8(&displayFaultStatus_state, status);
}

void simplified_process_inverter(){

}
/*
uint8_t test_state = 0;
uint16_t desired_test_current = 0;
uint16_t max_test_limit = 0;*/

void process_inverter() {
	U8 inverter_enable_state = INVERTER_DISABLE;
	/*
	if(faultCode.data != 0x00) {
		//send
		vehicle_state = VEHICLE_FAULT;
	}*/

	switch (vehicle_state)
	{
	case VEHICLE_NO_COMMS:
		// check too see we are receiving messages from inverter to validate comms
		if ((HAL_GetTick() -  driveEnableInvStatus_state.info.last_rx) < INVERTER_TIMEOUT_ms)
		{
			vehicle_state = VEHICLE_STANDBY;
		}
		SET_INV_DISABLED();
		break;

	case VEHICLE_FAULT:
		// try and exit the lockout mode of the inverter. This will be present whenever there is a fault
		if(faultCode.data == 0x00) {
			//do nothing, see if the fault goes away
			vehicle_state = VEHICLE_NO_COMMS;
		}
		SET_INV_DISABLED();
		break;

	case VEHICLE_STANDBY:
		// everything is good to go in this state, we are just waiting to enable the RTD button
		if (brakePressureFront_psi.data > PREDRIVE_BRAKE_THRESH_psi &&
				readyToDriveButtonPressed_state &&
				inputInverterVoltage_V.data > TS_ON_THRESHOLD_VOLTAGE_V)
		{
			// Button is pressed, set state to VEHICLE_PREDRIVE
			vehicle_state = VEHICLE_PREDRIVE;
			preDriveTimer_ms = 0;
		}
		SET_INV_DISABLED();
		break;

	case VEHICLE_PREDRIVE:
		// buzz the RTD buzzer for the correct amount of time
		if(++preDriveTimer_ms > PREDRIVE_TIME_ms) {
			vehicle_state = VEHICLE_DRIVING;
		}
		SET_INV_DISABLED();
		break;

	case VEHICLE_DRIVING:
		// let the car run as normal. Do not change desiredTorque
		//limit_motor_torque();
		inverter_enable_state = INVERTER_ENABLE;
		break;

	default:
		vehicle_state = VEHICLE_NO_COMMS;
		SET_INV_DISABLED();
		break;
	}

	// send the current request
	desiredInvCurrentPeakToPeak_A.data = desiredCurrent_A;
	maxCurrentLimitPeakToPeak_A.data = maxcurrentLimit_A;
	driveEnable_state.data = inverter_enable_state;

	/*desiredInvCurrentPeakToPeak_A.data = desired_test_current;
	maxCurrentLimitPeakToPeak_A.data = max_test_limit;
	driveEnable_state.data = test_state;*/

	send_group(INVERTER_SET_CURRENT_AC_CMD_ID);
	send_group(INVERTER_MAX_CURRENT_AC_LIMIT_CMD_ID);
	send_group(INVERTER_DRIVE_ENABLE_CMD_ID);
	service_can_tx(hcan);
}


void limit_motor_torque()
{
	// TODO this is where all the fun launch and traction control will go

	float new_torque_limit;
	// right now we just want to limit torque based on back EMF to keep our
	// current under 100A at the accumulator
	if (motorSpeed_rpm.data > MIN_LIMIT_SPEED_rpm)
	{
		new_torque_limit = (TS_CURRENT_MAX_A * dcBusVoltage_V.data) / ((motorSpeed_rpm.data * MATH_TAU) / SECONDS_PER_MIN); //limit to 80Kw of power
	}
	if (new_torque_limit < torqueCmdLim_Nm.data) torqueCmdLim_Nm.data = new_torque_limit;
}


void update_outputs() {
	if(vehicle_state == VEHICLE_PREDRIVE) {
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, MOSFET_PULL_DOWN_ON);
		update_and_queue_param_u8(&vehicleBuzzerOn_state, TRUE);
	} else {
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, MOSFET_PULL_DOWN_OFF);
		update_and_queue_param_u8(&vehicleBuzzerOn_state, FALSE);
	}

	if(brakePressureFront_psi.data > BRAKE_LIGHT_THRESH_psi) {
		HAL_GPIO_WritePin(BRK_LT_GPIO_Port, BRK_LT_Pin, MOSFET_PULL_DOWN_ON);
		update_and_queue_param_u8(&brakeLightOn_state, TRUE);
	} else {
		HAL_GPIO_WritePin(BRK_LT_GPIO_Port, BRK_LT_Pin, MOSFET_PULL_DOWN_OFF);
		update_and_queue_param_u8(&brakeLightOn_state, FALSE);
	}
	return;
}

void LED_task(){
	static U32 last_led = 0;
	if(HAL_GetTick() - last_led >= HBEAT_LED_DELAY_TIME_ms) {
		HAL_GPIO_TogglePin(MCU_STATUS_LED_GPIO_Port, MCU_STATUS_LED_Pin);
		last_led = HAL_GetTick();
	}

	// Turn off RGB
	HAL_GPIO_WritePin(STATUS_R_GPIO_Port, STATUS_R_Pin, SET);
	HAL_GPIO_WritePin(STATUS_G_GPIO_Port, STATUS_G_Pin, SET);
	HAL_GPIO_WritePin(STATUS_B_GPIO_Port, STATUS_B_Pin, SET);
}


// End of vcu.c
