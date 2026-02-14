/*
 * motor_driver.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Akshat
 */

#include "motor_driver.h"
#include "Stepperv2.h"
#include "globals.h"
#include "pid_controller.h"
#include <math.h>
#include <stdlib.h>
#include <stddef.h>
#include <ctype.h>
#include <string.h>
extern PID_Handle_t pid_b1;
extern PID_Handle_t pid_b2;
extern PID_Handle_t pid_b3;
extern PID_Handle_t pid_b4;

void Motor_Create(Motor_Handle_t* handle, TIM_HandleTypeDef* pwm_timer, uint32_t pwm_channel, GPIO_TypeDef* dir_port, uint16_t dir_pin){

	if (handle == NULL) return;

	handle->pwm_timer = pwm_timer;
	handle->pwm_channel = pwm_channel;
	handle->dir_port = dir_port;
	handle->dir_pin = dir_pin;
}

//// Akshat parse for scaling:
void handle_command(char *cmd)
{
	const char *p = cmd;

	while (*p){

		while (isspace((unsigned char)*p)) p++;

		if (*p == 'b' || *p == 'B'){

			p++;
			int id = parse_cmd(&p);
			int rpm = parse_cmd(&p);

			if      (id == 1) b1_target_rpm = rpm;
			else if (id == 2) b2_target_rpm = rpm;
			else if (id == 3) b3_target_rpm = rpm;
			else if (id == 4) b4_target_rpm = rpm;
		}

		else if (*p == 's' || *p == 'S')
		{
			p++;
			int id = parse_cmd(&p);
			int angle = parse_cmd(&p);
			//s1_target_angle = angle;
			Stepper_Handle_t *S = NULL;

			if (id == 1) S = &S1;
			else if (id == 2) S = &S2;
			else if (id == 3) S = &S3;
			else if (id == 4) S = &S4;

			if (S)
			{
				if (angle == (int) S->lastInstruct.degree){
					continue;
				}

					Wrapper temp;
					temp.degree = angle;
					temp.rpm = 30;
					S->lastInstruct.degree = angle;
					S->lastInstruct.rpm = temp.rpm;
					enqueueW(&S->q, temp);
					if (!S->queueMode){
						Stepper_Stop(S);
						S->step_counter = 0;
						S->target_steps = 0;
					}
			  }		// 5 kiya coz gearbox 5:! he behenchod mujhe nahi khelna
//                S->target_steps = (uint32_t)(fabsf(angle) * S->steps_per_rev / 360.0f);
//                S->step_counter = 0;
//                S->dir = (angle >= 0) ? 0 : 1;
//                S->recievedStepper = 1;
			}
		else {
			while (*p && !isspace((unsigned char)*p)) p++;
		}
	}
}

// pid tuner requirements:

//void handle_command(char *cmd)
//{
//    char *p = cmd;
//
//    /* 1. Find 'B' or 'b' */
//    while (*p && *p != 'B' && *p != 'b') {
//        p++;
//    }
//    if (*p == '\0') return;
//
//    /* 2. Parse motor ID */
//    p++; // skip 'B'
//    int id = (int)strtof(p, &p);
//
//    /* 3. Parse target RPM */
//    int rpm = (int)strtof(p, &p);
//
//    /* 4. Apply RPM target */
//    if      (id == 1) b1_target_rpm = rpm;
//    else if (id == 2) b2_target_rpm = rpm;
//    else if (id == 3) b3_target_rpm = rpm;
//    else if (id == 4) b4_target_rpm = rpm;
//    else return; // invalid motor ID
//
//    /* 5. Parse PID values (order independent) */
//    float new_kp = -1.0f;
//    float new_ki = -1.0f;
//    float new_kd = -1.0f;
//
//    while (*p && *p != ';') {
//        if (*p == 'P') {
//            p++;
//            new_kp = strtof(p, &p);
//        }
//        else if (*p == 'I') {
//            p++;
//            new_ki = strtof(p, &p);
//        }
//        else if (*p == 'D') {
//            p++;
//            new_kd = strtof(p, &p);
//        }
//        else {
//            p++; // skip spaces or unknown chars
//        }
//    }
//
//    /* 6. Select PID handle */
//    PID_Handle_t *pid = NULL;
//
//    if      (id == 1) pid = &pid_b1;
//    else if (id == 2) pid = &pid_b2;
//    else if (id == 3) pid = &pid_b3;
//    else if (id == 4) pid = &pid_b4;
//
//    if (!pid) return;
//
//    /* 7. Apply PID updates (partial updates allowed) */
//    if (new_kp >= 0.0f) pid->kp = new_kp;
//    if (new_ki >= 0.0f) pid->ki = new_ki;
//    if (new_kd >= 0.0f) pid->kd = new_kd;
//}


// Soham's Parse Function (from Nova):
int parse_cmd(const char **p) {
	const char *str = *p;
	int sign = 1, result = 0;

	while (isspace((unsigned char)*str)) str++;

	if (*str == '-') { sign = -1; str++; }
	else if (*str == '+') { str++; }

	if (!isdigit((unsigned char)*str)) return 0;

	while (isdigit((unsigned char)*str)) {
		result = result * 10 + (*str - '0');
		str++;
	}

	while (isspace((unsigned char)*str)) str++;

	*p = str; // update pointer for next parse
	return sign * result;
}

void Motor_SetOutput(Motor_Handle_t* handle, float output) {

	if (handle == NULL) return;

	// clamped input - change later
	if (output > 1.0f) output = 1.0f;	// bruv +ve is forward -ve is reverse similar to how we handled NOVA
	if (output < -1.0f) output = -1.0f;

	// set direction
	if (output >= 0.0f) {
		HAL_GPIO_WritePin(handle->dir_port, handle->dir_pin, GPIO_PIN_SET); // Forward
	} else {
		HAL_GPIO_WritePin(handle->dir_port, handle->dir_pin, GPIO_PIN_RESET); // Reverse
	}

	// calculate and set PWM
	uint32_t timer_period = __HAL_TIM_GET_AUTORELOAD(handle->pwm_timer);
	uint32_t pulse_width = (uint32_t)(fabsf(output) * timer_period);
	__HAL_TIM_SET_COMPARE(handle->pwm_timer, handle->pwm_channel, pulse_width);
}
