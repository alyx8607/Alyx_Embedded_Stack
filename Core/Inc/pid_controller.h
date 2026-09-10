/*
 * pid_controller.h
 *
 *  Created on: Oct 10, 2025
 *      Author: Akshat
 */

#ifndef INC_PID_CONTROLLER_H_
#define INC_PID_CONTROLLER_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct{
	// tunings
	float kp;
	float ki;
	float kd;
	// timing
	float sample_time_s;	// >0 -> fixed dt; <=0 -> HAL_GetTick measured dt
	uint32_t last_tick_ms;
	// integrator
	float integral_sum;
	float out_min;
	float out_max;
	// derivative
	float last_measurement;
	// slew limiting
	float last_output;
	// smoothening and stuff
	float deadband_measurement;		// ignore measurement magnitude below this
	float deadband_setpoint;		// setpoint under this is 0
	float integral_decay;			// multiplicative decay factor
	float max_slew_rate;			// max change in output per second

} PID_Handle_t;

void PID_Create(PID_Handle_t* handle, float kp, float ki, float kd, float sample_time_s);

float PID_Compute(PID_Handle_t* handle, float setpoint, float measurement);

void PID_SetTunings(PID_Handle_t* handle, float kp, float ki, float kd);	// for tuning
void PID_SetOutputLimits(PID_Handle_t* handle, float min, float max);	// clamping for safety
void PID_SetDeadbands(PID_Handle_t* handle, float setpoint_deadband, float measurement_deadband);
void PID_SetIntegralDecay(PID_Handle_t* handle, float decay_factor);
void PID_SetMaxSlewRate(PID_Handle_t* handle, float units_per_sec);

void PID_Reset(PID_Handle_t* handle);	// resets for integral sum

#endif /* INC_PID_CONTROLLER_H_ */
