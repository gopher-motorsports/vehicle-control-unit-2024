/*
 * drs.h
 *
 *  Created on: Mar 21, 2024
 *      Author: chris
 */

#ifndef INC_DRS_H_
#define INC_DRS_H_
#include "GopherCAN.h"
#include "gopher_sense.h"
#include <stdbool.h>
/*DRS duty cycle periods: 500us-2500us pulses with a max period of 3000ms
	corresponds to 2500-24999 compare match value and 29999 overflow value
	for timer 3 */

//proportional steps from 2500-24999, each step is 1250
#define DRS_POS_0  4999
#define DRS_POS_1  6249
#define DRS_POS_2  7499
#define DRS_POS_3  8749
#define DRS_POS_4  9999
#define DRS_POS_5  11249
#define DRS_POS_6  12499
#define DRS_POS_7  13749
#define DRS_POS_8  14999
#define DRS_POS_9  16249
#define DRS_POS_10 17499
#define DRS_POS_11 18749
#define DRS_POS_12 19999
#define DRS_POS_13 21249
#define DRS_POS_14 22499
#define DRS_POS_15 23749
#define DRS_POS_16 24999

#define OPEN_POS 18959 //63.2% duty cycle
#define CLOSED_POS 25660 //86.4% duty cycle

//constants for smart shutoff when cornering
#define BRAKE_SHUTOFF_THRESHOLD 25 //10 psi
//#define DRS_SHUTDOWN_CHECKS

#define STEERING_ANGLE_LEFT_SHUTOFF 75 //0-210 deg is full, 105 is center
#define STEERING_ANGLE_LEFT_RETURN 90
#define STEERING_ANGLE_RIGHT_SHUTOFF 135
#define STEERING_ANGLE_RIGHT_RETURN 120

#define CAN_VALUE_TRUST_THRESHOLD 500 //data has to updated every 500 ms otherwise don't listen to it
typedef enum
{
	NORMAL_STATE = 0,
	LEFT_LIMIT_BREACHED,
	RIGHT_LIMIT_BREACHED
} STEERING_ANGLE_STATE;

typedef struct {
    FLOAT_CAN_STRUCT* parameter;
    GPIO_TypeDef* enable_switch_port;
    uint16_t enable_switch_pin;
    uint8_t enabled;
    float amp_max;
    float ampsec_max;
    float ampsec_sum;
    uint32_t trip_time;
    uint32_t reset_delay_ms;
    uint32_t last_update;
    uint32_t overcurrent_count;
    uint8_t max_overcurrent_count;
} POWER_CHANNEL;

void init_DRS_servo(TIM_HandleTypeDef* timer_address, U32 channel);
void set_DRS_Servo_Position(U8 start_up_condition);
#endif /* INC_DRS_H_ */
