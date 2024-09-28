/*
 * drs.c
 *
 *  Created on: Mar 21, 2024
 *      Author: chris
 */
#include "drs.h"
#include "main.h"
#include "gopher_sense.h"
#include "pulse_sensor.h"

#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdbool.h>
#include <cmsis_os.h>

TIM_HandleTypeDef* DRS_Timer;
U32 DRS_Channel;
int rot_dial_timer_val = 0; //keeping this in here if we want to use rotary dial
U8 drs_button_state;
U8 drs_steering_angle_limit_state = 0; //SA = steering angle
U8 next_steering_angle_limit_state = 0;

//local function prototypes
bool drs_shutoff_conditions_reached();
void update_power_channel(POWER_CHANNEL* channel);

void init_DRS_servo(TIM_HandleTypeDef* timer_address, U32 channel){
	DRS_Timer = timer_address;
	DRS_Channel = channel;
	HAL_TIM_PWM_Start(DRS_Timer, DRS_Channel); //turn on PWM generation
}

void set_DRS_Servo_Position(U8 start_up_condition){
	//duty cycle lookup table for each DRS position, optional if we are using the rotary dial

	drs_button_state = swButon0_state.data; //place holder button
	if(start_up_condition){
			__HAL_TIM_SET_COMPARE(DRS_Timer, DRS_Channel, OPEN_POS);
	}
	else{
		if(drs_button_state == 1){
#ifdef DRS_SHUTDOWN_CHECKS
			if(drs_shutoff_conditions_reached()){
				__HAL_TIM_SET_COMPARE(DRS_Timer, DRS_Channel, CLOSED_POS);
			}
			else{
			__HAL_TIM_SET_COMPARE(DRS_Timer, DRS_Channel, OPEN_POS);
			}
#else
			__HAL_TIM_SET_COMPARE(DRS_Timer, DRS_Channel, OPEN_POS);
#endif

		}
		else{
			__HAL_TIM_SET_COMPARE(DRS_Timer, DRS_Channel, CLOSED_POS);
		}

	}
	drs_steering_angle_limit_state = next_steering_angle_limit_state;
}

bool drs_shutoff_conditions_reached(){
	static long current_tick = 0;
	current_tick = HAL_GetTick();

	//if you haven't received brake or steering angle data in .2 seconds don't close drs
//	if(current_tick - brakePressureFront_psi.info.last_rx < CAN_VALUE_TRUST_THRESHOLD){// ||
////	   current_tick - brakePressureRear_psi.info.last_rx  < CAN_VALUE_TRUST_THRESHOLD){
//	   if(brakePressureFront_psi.data > BRAKE_SHUTOFF_THRESHOLD){// ||
////	   	   brakePressureRear_psi.data > BRAKE_SHUTOFF_THRESHOLD){
//	   		return true;
//	   	}
//	}
/*
	if(current_tick - steeringAngle_deg.info.last_rx < CAN_VALUE_TRUST_THRESHOLD){
		switch (drs_steering_angle_limit_state)
			{
			case NORMAL_STATE:
				if(steeringAngle_deg.data < STEERING_ANGLE_LEFT_SHUTOFF){
					next_steering_angle_limit_state = LEFT_LIMIT_BREACHED;
					return true;
				}

				if(steeringAngle_deg.data > STEERING_ANGLE_RIGHT_SHUTOFF){
					next_steering_angle_limit_state = RIGHT_LIMIT_BREACHED;
					return true;
				}
				else{return false;}
				break;


			case LEFT_LIMIT_BREACHED:
				if(steeringAngle_deg.data > STEERING_ANGLE_LEFT_RETURN){
					next_steering_angle_limit_state = NORMAL_STATE;
					return false;
				}
				else{return true;}
				break;

			case RIGHT_LIMIT_BREACHED:
				if(steeringAngle_deg.data < STEERING_ANGLE_RIGHT_RETURN){
					next_steering_angle_limit_state = NORMAL_STATE;
					return false;
				}
				else{return true;}
				break;
		}
	}
	*/
	//reaches here if all of the data is not being updated on the can bus
	return false;

}

