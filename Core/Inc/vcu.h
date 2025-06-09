/*
 * vcu.h
 *
 *  Created on: Dec 4, 2022
 *      Author: Ben Abbott
 */

#ifndef INC_SWM_H_
#define INC_SWM_H_

#include "main.h"
#include "GopherCAN.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdbool.h>
#include <cmsis_os.h>

typedef struct {
    U8_CAN_STRUCT* param;
    GPIO_TypeDef* port;
    U16 pin;
} BUTTON;

#define NUM_OF_BUTTONS 3

void init(CAN_HandleTypeDef* hcan_ptr);
void main_loop();
void can_buffer_handling_loop();

void update_RTD();         // Ready to drive logic
void process_sensors();    // Runs safety checks on driver inputs
void update_gcan_states(); // Updates GopherCAN states
void process_inverter();   // Updates vehicle state and applicable commands
void update_outputs();     // Updates brake light and buzzer
void update_cooling();     // Controls/updates the cooling system
void update_display_fault_status(); 	// Check all vehicle fault messages and sends best one to display
void limit_motor_torque();
void LED_task();
void pass_on_timer_info(); //this is def not the best way to do this
void set_DRS_Servo_Position(U8 start_up_condition);
void init_Pump(TIM_HandleTypeDef* timer_address, U32 channel);
void launch_control_sm();
boolean isVehicleMoving();
void set_inv_disabled();
int get_current_limit(boolean driving_mode);
#endif /* INC_VCU_H_ */
