/*
 * vcu.h
 *
 *  Created on: Dec 4, 2022
 *      Author: Ben Abbott
 */

#ifndef INC_VCU_H_
#define INC_VCU_H_

#include "main.h"


// ========================================== CONSTANTS =========================================
#define MATH_PI           3.14159265
#define MATH_TAU          MATH_PI*2
#define SECONDS_PER_MIN   60
#define MINUTES_PER_HOUR  60     // For conversion from rpm to mph
#define IN_PER_FT         12     // For conversion from rpm to mph
// ==============================================================================================

// ========================================== PWM CONSTANTS =========================================
#define TIM2_PWM_MAX	1600 //value where max overflow happens, defined in IOC in timer config

// ======================================= APPS PARAMETERS ======================================
#define APPS_MAX_TORQUE_POS_mm  20.0f // The position of the pedal at 100% torque
#define APPS_MIN_TORQUE_POS_mm  10.0f  // The position of the pedal at 0% torque
#define APPS_MAX_ERROR_POS_mm 24.0f // position where the error begins
#define APPS_MIN_ERROR_POS_mm 1.0f // position where the error begins
#define APPS_TOTAL_TRAVEL_mm ( APPS_MAX_TORQUE_POS_mm - APPS_MIN_TORQUE_POS_mm )
// ==============================================================================================

// ====================================== BRAKE PARAMETERS ======================================
#define BRAKE_PRESS_MIN_psi    -50   // The minimum value of the brake pressure sensor
#define BRAKE_PRESS_MAX_psi    2050  // The maximum value of the brake pressure sensor
#define BRAKE_LIGHT_THRESH_psi 25 // The pressure at which the brake light will be activated
// ==============================================================================================
// ============================= TRACTIVE SYSTEM CURRENT PARAMETERS =============================
#define TS_CURRENT_MIN_A   -85   // The minimum value of the current sensor
#define TS_CURRENT_MAX_A   100  // The maximum value of the current sensor
#define MIN_LIMIT_SPEED_rpm 1000 // minimum RPM that the VCU begins current limiting
// =============================================================================================

// ================================== READY TO DRIVE PARAMETERS =================================
#define PREDRIVE_BRAKE_THRESH_psi  100  // The minimum brake pressure to enter the driving state
#define PREDRIVE_BUTTON_PRESSED    1    // The value of the button parameter when pressed
#define PREDRIVE_TIME_ms           2000 // The length of predrive in ms
#define RTD_BUTTON_PUSHED          (GPIO_PIN_RESET)
#define TS_ON_THRESHOLD_VOLTAGE_V  190.0
// ==============================================================================================


// ====================================== SAFETY PARAMETERS =====================================
// -------------------------------------- Input Validation --------------------------------------
#define INPUT_TRIP_DELAY_ms 100  // The amount of time it takes an input fault to take effect

// -------------------------------------- APPS/Brake Check --------------------------------------
// This check is done using APPS1 (since APPS1 determines the applied torque) and the BSE
#define APPS_BRAKE_PRESS_THRESH_psi  50.0f  // The minimum amount of brake pressure that will trip
// The minimum APPS position that will trip the APPS/Brake check
#define APPS_BRAKE_APPS1_THRESH_mm   ( APPS_TOTAL_TRAVEL_mm * 0.25 ) + APPS_MIN_TORQUE_POS_mm
// The maximum APPS position that will reset the APPS/Brake check
#define APPS_BRAKE_RESET_THRESH_mm   ( APPS_TOTAL_TRAVEL_mm * 0.05 ) + APPS_MIN_TORQUE_POS_mm

// ------------------------------------ APPS Correlation Check ----------------------------------
#define APPS_CORRELATION_THRESH_mm   ( APPS_TOTAL_TRAVEL_mm * 0.1 )
#define CORRELATION_TRIP_DELAY_ms    100  // The amount of time it takes a correlation fault to take effect
#define APPS_CORRELATION_OFFSET_mm   0.0  // The average offset of the APPSs, positive is APPS1 > APPS2

// ------------------------------------ TS Current/Brake Check ----------------------------------
#define BRAKE_TS_CURRENT_THRESH_A  14.0f  // The current limit when the brake and
#define BRAKE_TS_PRESS_THRESH_psi  450.0f // The amount of brake pressure needed
#define BRAKE_TS_MAX_TORQUE            // A clamp on the maximum amount of torque when triggered
#define BRAKE_TS_ON_DELAY_ms       50  // The amount of timer it takes the limit to turn on
#define BRAKE_TS_OFF_DELAY_ms      50  // The amount of timer it takes the limit to turn off
// ==============================================================================================

// ====================================== COOLING PARAMETERS ====================================
#define INVERTER_TEMP_THRESH_C    40.0f // Minimum Inverter temp threshold for cooling fan to turn on
#define INVERTER_TEMP_THRESH_C_1  45.0f //Inverter temp threshold + HYS + 3.0C
#define INVERTER_TEMP_THRESH_C_2  48.0f //Inverter temp threshold + HYS + 6.0C
#define INVERTER_TEMP_THRESH_C_3  51.0f //Inverter temp threshold + HYS + 8.0C
#define INVERTER_TEMP_THRESH_C_4  54.0f //Inverter temp threshold + HYS + 11.0C

#define MOTOR_TEMP_THRESH_C       50.0f // Minimum Motor temperature for cooling fan to turn on
#define MOTOR_TEMP_THRESH_C_1     55.0f //Motor temp threshold + HYS + 3.0C
#define MOTOR_TEMP_THRESH_C_2     58.0f //Motor temp threshold + HYS + 6.0C
#define MOTOR_TEMP_THRESH_C_3     61.0f //Motor temp threshold + HYS + 8.0C
#define MOTOR_TEMP_THRESH_C_4     64.0f //Motor temp threshold + HYS + 11.0C

#define HYSTERESIS_DIGITAL	      5.0f // Hysteresis when confined to digital signal (on/off)
#define HYSTERESIS_ANALOG	      2.0f // Hysteresis when have PWM output signal (variable duty cycle)

#define PUMP_INTENSITY_OFF		  0  //0% duty cycle --> 0/1600
#define PUMP_INTENSITY_1		  400 //25% duty cycle --> 400/1600
#define PUMP_INTENSITY_2		  800 //50% duty cycle --> 800/1600
#define PUMP_INTENSITY_3		  1200 //75% duty cycle --> 1200/1600
#define PUMP_INTENSITY_4		  1600 //100% duty cycle --> 1600/1600
// ==============================================================================================

// =============================== SENSOR OVERCURRENT PARAMETERS ================================
#define SENSOR_OVERCURRENT_TRIPPED      (GPIO_PIN_RESET)

// ================================== TRACTIVE SYSTEM PARAMETERS ================================
#define MOTOR_DIRECTION         1      // Motor direction; 0 is reverse, 1 is forward
#define MAX_CMD_TORQUE_Nm       150.0f    // The maximum torque that will be commanded
#define INVERTER_TIMEOUT_ms     100    // The time after which the vehicle will enter STARTUP
#define INVERTER_ENABLE         0x01   // Flags to enable the inverter
#define INVERTER_DISABLE        0x00   // Flags to disable the inverter
#define INVERTER_LOCKOUT        0x80   // Lockout is bit 7 of byte 6
#define INVERTER_CMD_ID         0x0C0  // The CAN ID of the inverter command
#define INVERTER_PARAM_ID       0x0C1  // The CAN ID of the parameter message
#define PARAM_CMD_FAULT_CLEAR   20     // Address of the fault clear parameter
#define PARAM_CMD_READ          0      // Value to send in parameter command to read value
#define PARAM_CMD_WRITE         1      // Value to send in parameter command to read value
#define PARAM_CMD_RESERVED1     0x00   // Reserved value in inverter parameter
#define PARAM_FAULT_CLEAR_DATA  0      // Value to send in the data field when clearing faults
#define PARAM_CMD_RESERVED2     0x0000 // Reserved value in inverter parameter
#define FINAL_DRIVE_RATIO       4.363  // The final drive ratio
#define WHEEL_DIAMETER_IN       10     // Wheel diameter
// ==============================================================================================

// ======================================== I/O PARAMETERS ======================================
#define MOSFET_PULL_DOWN_ON (GPIO_PIN_SET)
#define MOSFET_PULL_DOWN_OFF (GPIO_PIN_RESET)
#define PLM_CONTROL_ON (GPIO_PIN_RESET)
#define PLM_CONTROL_OFF (GPIO_PIN_SET)
// ==============================================================================================

// ======================================= BSPD PARAMETERS ======================================
// Whether each fault is active high or active low
#define BSPD_APPS1_FAULT     GPIO_PIN_RESET
#define BSPD_APPS2_FAULT     GPIO_PIN_RESET
#define BSPD_BRAKE_FAULT     GPIO_PIN_RESET
#define BSPD_TS_SNS_FAULT    GPIO_PIN_RESET
#define BSPD_TS_BRK_FAULT    GPIO_PIN_SET
// ==============================================================================================

// =================== THROTTLE CALCULATION ===================
// Throttle is calculated using APPS1 with APPS2 being used
// for redundancy in the correlation  check mandated by rules
// ============================================================

// ======================================= DRS PARAMETERS ======================================
#define DRS_POS_0 100
#define DRS_POS_1 200
#define DRS_POS_2 300
#define DRS_POS_3 400
#define DRS_POS_4 500
#define DRS_POS_5 600
#define DRS_POS_6 700
#define DRS_POS_7 800
#define DRS_POS_8 900
#define DRS_POS_9 1000
#define DRS_POS_10 1100
#define DRS_POS_11 1200
#define DRS_POS_12 1300
#define DRS_POS_13 1400
#define DRS_POS_14 1500
#define DRS_POS_15 1600

#define DRS_BUTTON_STATE 0
#define ROT_DIAL_POS 0
// ======================================= BSPD PARAMETERS ======================================
// This is the vehicle state which is affected both by the actions
// of the driver and the current state of the inverter
typedef enum
{
	VEHICLE_NO_COMMS  = 0, // When the inverter first turns on and if there is ever a loss of communication
	VEHICLE_LOCKOUT   = 1, // The vehicle can detect that the inverter is in lockout
	VEHICLE_STANDBY   = 2, // The inverter has exited lockout but no torque commands will be sent
	VEHICLE_PREDRIVE  = 3, // The vehicle buzzer is active and the driving state will be entered
	VEHICLE_DRIVING   = 4, // Torque commands are actively being sent from APPS positions
	VEHICLE_FAULT     = 5
} VEHICLE_STATE_t;

typedef enum {
	NONE = 0,
	RELEASE_PEDAL,
	BRAKING_FAULT,
	APPS_FAULT,
	BSPD_FAULT,
	AMS_FAULT,
	IMD_FAULT,
	VCU_FAULT,
	BMS_FAULT
} DISPLAY_FAULT_STATUS_t;

extern VEHICLE_STATE_t vehicle_state;

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
void set_DRS_Servo_Position();
#endif /* INC_VCU_H_ */
