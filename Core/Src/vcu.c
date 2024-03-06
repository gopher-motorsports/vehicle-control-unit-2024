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

uint32_t maxCurrent_mA = 0;
uint32_t preDriveTimer_ms = 0;
float desiredTorque_Nm = 0;
float torqueLimit_Nm = 0;

uint16_t apps1FaultTimer_ms = 0;
uint16_t apps2FaultTimer_ms = 0;
uint16_t brakeSensorFaultTimer_ms = 0;
uint16_t currentSensorFaultTimer_ms = 0;

uint16_t correlationTimer_ms = 0;

boolean appsBrakeLatched_state = 0;

boolean readyToDriveButtonPressed_state = 0;

VEHICLE_STATE_t vehicle_state = VEHICLE_NO_COMMS;

TIM_HandleTypeDef* PWM_Timer;
U32 DRS_Channel;
U32 PUMP_Channel;

#define HBEAT_LED_DELAY_TIME_ms 500
#define RTD_DEBOUNCE_TIME_ms 25
#define SET_INV_DISABLED() do{ desiredTorque_Nm = 0; torqueLimit_Nm = MAX_CMD_TORQUE_Nm; inverter_enable_state = INVERTER_DISABLE; } while(0)

// Initialization code goes here
void init(CAN_HandleTypeDef* hcan_ptr) {
	hcan = hcan_ptr;

	init_can(GCAN1, hcan, VCU_ID, BXTYPE_SLAVE);
}

void main_loop() {

	process_sensors();
	process_inverter();
	update_outputs();
	update_cooling();
	update_display_fault_status();
	update_gcan_states(); // Should be after process_sensors
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
	// VSC sensors faults
	update_and_queue_param_u8(&vcuPedalPosition1Fault_state, apps1FaultTimer_ms > INPUT_TRIP_DELAY_ms);
	update_and_queue_param_u8(&vcuPedalPosition2Fault_state, apps2FaultTimer_ms > INPUT_TRIP_DELAY_ms);
	update_and_queue_param_u8(&vcuBrakePressureSensorFault_state, brakeSensorFaultTimer_ms > INPUT_TRIP_DELAY_ms);
	update_and_queue_param_u8(&vcuTractiveSystemCurrentSensorFault_state, currentSensorFaultTimer_ms > INPUT_TRIP_DELAY_ms);
	// VSC safety checks
	update_and_queue_param_u8(&vcuPedalPositionCorrelationFault_state, correlationTimer_ms > CORRELATION_TRIP_DELAY_ms);
	update_and_queue_param_u8(&vcuPedalPositionBrakingFault_state, appsBrakeLatched_state);
	// Requested torque
	update_and_queue_param_float(&vcuTorqueRequested_Nm, desiredTorque_Nm);
	// Cooling
	update_and_queue_param_u8(&coolantFanPower_percent, 100*HAL_GPIO_ReadPin(RAD_FAN_GPIO_Port, RAD_FAN_Pin));
	//update_and_queue_param_u8(&coolantPumpPower_percent, 100*HAL_GPIO_ReadPin(PUMP_GPIO_Port, PUMP_Pin));
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

    double temp_readings[] = {igbtATemp_C.data, igbtBTemp_C.data, igbtCTemp_C.data, gateDriverBoardTemp_C.data, controlBoardTemp_C. data, motorTemp_C.data};
	static double cooling_thresholds[] = {IGBT_TEMP_THRESH_C, IGBT_TEMP_THRESH_C, IGBT_TEMP_THRESH_C, GDB_TEMP_THRESH_C, CTRL_BOARD_TEMP_THRESH_C, MOTOR_TEMP_THRESH_C};
	double total_cooling_thresholds = sizeof(cooling_thresholds) / sizeof(cooling_thresholds[0]); //amount of cooling thresholds
	static U8 rad_fan_state = PLM_CONTROL_OFF;
	int readings_below_HYS_threshold = 0;

	for(int i = 0; i < total_cooling_thresholds; i++){
		if(rad_fan_state == PLM_CONTROL_OFF && (temp_readings[i] >= (cooling_thresholds[i] + HYSTERESIS))){
			rad_fan_state = PLM_CONTROL_ON;
			HAL_GPIO_WritePin(RAD_FAN_GPIO_Port, RAD_FAN_Pin, rad_fan_state);
			readings_below_HYS_threshold = 0;
			break;
		}

		else if(rad_fan_state == PLM_CONTROL_ON){
			if(temp_readings[i] <= (cooling_thresholds[i] - HYSTERESIS)){
				readings_below_HYS_threshold++;
			}

			if(readings_below_HYS_threshold == total_cooling_thresholds){
				rad_fan_state = PLM_CONTROL_OFF;
				HAL_GPIO_WritePin(RAD_FAN_GPIO_Port, RAD_FAN_Pin, rad_fan_state);
			}
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
		if (readyToDriveButtonPressed_state != (HAL_GPIO_ReadPin(MCU_AUX_1_GPIO_Port, MCU_AUX_1_Pin) == RTD_BUTTON_PUSHED))
		{
			new_event = TRUE;
			new_event_time = HAL_GetTick();
		}
	}
	else
	{
		// the button change was not held long enough
		if (readyToDriveButtonPressed_state == (HAL_GPIO_ReadPin(MCU_AUX_2_GPIO_Port, MCU_AUX_2_Pin) == RTD_BUTTON_PUSHED))
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

	torqueLimit_Nm = MAX_CMD_TORQUE_Nm;

	// Input Validation Checks
	if(pedalPosition1_mm.data > APPS_MAX_ERROR_POS_mm
			|| pedalPosition1_mm.data < APPS_MIN_ERROR_POS_mm) {
		apps1FaultTimer_ms = (apps1FaultTimer_ms > INPUT_TRIP_DELAY_ms) ? INPUT_TRIP_DELAY_ms + 1 : apps1FaultTimer_ms + 1;
	} else {
		apps1FaultTimer_ms = 0;
	}
	if(pedalPosition2_mm.data > APPS_MAX_ERROR_POS_mm
				|| pedalPosition2_mm.data < APPS_MIN_ERROR_POS_mm) {
		apps2FaultTimer_ms = (apps2FaultTimer_ms > INPUT_TRIP_DELAY_ms) ? INPUT_TRIP_DELAY_ms + 1 : apps2FaultTimer_ms + 1;
	} else {
		apps2FaultTimer_ms = 0;
	}
	if(brakePressureFront_psi.data > BRAKE_PRESS_MAX_psi
			|| brakePressureFront_psi.data < BRAKE_PRESS_MIN_psi) {
		brakeSensorFaultTimer_ms = (brakeSensorFaultTimer_ms > INPUT_TRIP_DELAY_ms) ? INPUT_TRIP_DELAY_ms + 1 : brakeSensorFaultTimer_ms + 1;
	} else {
		brakeSensorFaultTimer_ms = 0;
	}
	if(vcuTractiveSystemCurrent_A.data > TS_CURRENT_MAX_A
			|| vcuTractiveSystemCurrent_A.data < TS_CURRENT_MIN_A) {
		currentSensorFaultTimer_ms = (currentSensorFaultTimer_ms > INPUT_TRIP_DELAY_ms) ? INPUT_TRIP_DELAY_ms + 1 : currentSensorFaultTimer_ms + 1;
	} else {
		currentSensorFaultTimer_ms = 0;
	}
	if(apps1FaultTimer_ms > INPUT_TRIP_DELAY_ms
			|| apps2FaultTimer_ms > INPUT_TRIP_DELAY_ms
			|| brakeSensorFaultTimer_ms > INPUT_TRIP_DELAY_ms
			|| currentSensorFaultTimer_ms > INPUT_TRIP_DELAY_ms) {
		torqueLimit_Nm = 0;
	}

	// Correlation Check
	if(pedalPosition1_mm.data - pedalPosition2_mm.data > APPS_CORRELATION_THRESH_mm
			|| pedalPosition2_mm.data - pedalPosition1_mm.data > APPS_CORRELATION_THRESH_mm) {
		correlationTimer_ms = (correlationTimer_ms > CORRELATION_TRIP_DELAY_ms) ? CORRELATION_TRIP_DELAY_ms + 1 : currentSensorFaultTimer_ms + 1;
	} else {
		correlationTimer_ms = 0;
	}
	if(correlationTimer_ms > CORRELATION_TRIP_DELAY_ms) {
		torqueLimit_Nm = 0;
	}

	// APPS/Braking Check
	if((brakePressureFront_psi.data > APPS_BRAKE_PRESS_THRESH_psi
			&& pedalPosition1_mm.data > APPS_BRAKE_APPS1_THRESH_mm)) {
		appsBrakeLatched_state = TRUE;
	} else if (pedalPosition1_mm.data <= APPS_BRAKE_RESET_THRESH_mm) {
		appsBrakeLatched_state = FALSE;
	}

	if(appsBrakeLatched_state) {
		torqueLimit_Nm = 0;
	}
	// TODO make some hysteresis on this in order to make it less jumpy
	if(bspdTractiveSystemBrakingFault_state.data) {
		float tractiveSystemBrakingLimit_Nm = 0;
		float accumulatorVoltage_V = dcBusVoltage_V.data;
		if(motorSpeed_rpm.data != 0) {
			// Calculate max torque from speed and voltage, using angular velocity (rad/s)
			tractiveSystemBrakingLimit_Nm = (BRAKE_TS_CURRENT_THRESH_A * accumulatorVoltage_V)
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

	desiredTorque_Nm = ((pedalPosition1_mm.data-APPS_MIN_TORQUE_POS_mm)/APPS_TOTAL_TRAVEL_mm)*MAX_CMD_TORQUE_Nm;

	if(pedalPosition1_mm.data < APPS_MIN_TORQUE_POS_mm) {
		desiredTorque_Nm = 0;
	}

	if(desiredTorque_Nm > torqueLimit_Nm) {
		desiredTorque_Nm = torqueLimit_Nm;
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


void process_inverter() {
	U8 inverter_enable_state = INVERTER_DISABLE;

//	// if we loose comms with the inverter we should enter no NO_COMMS state. Also include a 100ms
//	// startup time to make sure all the different systems have a chance to boot up
//	if (HAL_GetTick() <= INVERTER_TIMEOUT_ms ||
//			(HAL_GetTick() - inverterState_state.info.last_rx) > INVERTER_TIMEOUT_ms)
//	{
//		// if we have no comms, do not do the rest of this function until we have them
//		vehicle_state = VEHICLE_NO_COMMS;
//		return;
//	}

	// at this point we know we are receiving data from the inverter

	// error out if we are in speed mode for some reason
	if(invStatesByte4_state.data & 0x01) {
		// TODO: ADD AN ERROR CODE IF WE ARE IN SPEED MODE
	}

	// if we ever enter lockout, make sure to go to that state to correctly handle it
	if((invStatesByte6_state.data & INVERTER_LOCKOUT)) {
		vehicle_state = VEHICLE_LOCKOUT;

		invParameterAddress_state.data = PARAM_CMD_FAULT_CLEAR;
		invParameterRW_state.data = PARAM_CMD_WRITE;
		invParameterReserved1_state.data = PARAM_CMD_RESERVED1;
		invParameterData_state.data = PARAM_FAULT_CLEAR_DATA;
		invParameterReserved2_state.data = PARAM_CMD_RESERVED2;
		send_group(INVERTER_PARAM_ID);
		service_can_tx(hcan);
	}

	switch (vehicle_state)
	{
	case VEHICLE_NO_COMMS:
		// wait for comms to try and exit lockout
		if ((HAL_GetTick() - inverterState_state.info.last_rx) < INVERTER_TIMEOUT_ms)
		{
			vehicle_state = VEHICLE_LOCKOUT;
		}
		SET_INV_DISABLED();
		break;

	case VEHICLE_LOCKOUT:
		// try and exit the lockout mode of the inverter. This will be present whenever there is a fault
		if((invStatesByte6_state.data & INVERTER_LOCKOUT)) {
			// We're in lockout; reset faults
			invParameterAddress_state.data = PARAM_CMD_FAULT_CLEAR;
			invParameterRW_state.data = PARAM_CMD_WRITE;
			invParameterReserved1_state.data = PARAM_CMD_RESERVED1;
			invParameterData_state.data = PARAM_FAULT_CLEAR_DATA;
			invParameterReserved2_state.data = PARAM_CMD_RESERVED2;
			send_group(INVERTER_PARAM_ID);
			service_can_tx(hcan);
			vehicle_state = VEHICLE_LOCKOUT;
		} else {
			// We've exited lockout
			vehicle_state = VEHICLE_STANDBY;
		}
		SET_INV_DISABLED();
		break;

	case VEHICLE_STANDBY:
		// everything is good to go in this state, we are just waiting to enable the RTD button
		if (brakePressureFront_psi.data > PREDRIVE_BRAKE_THRESH_psi &&
				readyToDriveButtonPressed_state &&
				dcBusVoltage_V.data > TS_ON_THRESHOLD_VOLTAGE_V)
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
		limit_motor_torque();
		inverter_enable_state = INVERTER_ENABLE;
		break;

	default:
		vehicle_state = VEHICLE_NO_COMMS;
		SET_INV_DISABLED();
		break;
	}

	// send the torque request
	torqueCmd_Nm.data = desiredTorque_Nm;
	torqueCmdLim_Nm.data = torqueLimit_Nm;
	invCmdFlags_state.data = inverter_enable_state;
	speedCmd_rpm.data = 0;
	cmdDir_state.data = (U8)(MOTOR_DIRECTION > 0);
	send_group(INVERTER_CMD_ID);
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
		new_torque_limit = (dcBusVoltage_V.data * (9.593*TS_CURRENT_MAX_A)) / (motorSpeed_rpm.data);
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

void pass_on_timer_info(TIM_HandleTypeDef* timer_address, U32 channel1, U32 channel2){
	PWM_Timer = timer_address;
	DRS_Channel = channel1;
	PUMP_Channel = channel2;
}

void set_DRS_Servo_Position(){
	static int DRS_POS_LUT[] = {DRS_POS_0, DRS_POS_1, DRS_POS_2, DRS_POS_3, DRS_POS_4,
	                            DRS_POS_5, DRS_POS_6, DRS_POS_7, DRS_POS_8, DRS_POS_9,
	                            DRS_POS_10, DRS_POS_11, DRS_POS_12, DRS_POS_13,
	                            DRS_POS_14, DRS_POS_15};
    int rot_dial = ROT_DIAL_POS; //change macro to actual g-can variable
	rot_dial = DRS_POS_LUT[rot_dial];

    if (DRS_BUTTON_STATE == 1){ //change macro to actual g-can variable
    	__HAL_TIM_SET_COMPARE(PWM_Timer, DRS_Channel, rot_dial);
    	HAL_TIM_PWM_Start(PWM_Timer, DRS_Channel);
    }
    else{
    	__HAL_TIM_SET_COMPARE(PWM_Timer, DRS_Channel, 0);
    	HAL_TIM_PWM_Stop(PWM_Timer,DRS_Channel); //is it better to just leave it on with duty 0?

    }
}
// End of vcu.c
