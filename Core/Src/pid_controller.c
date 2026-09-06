/*
 * pid_controller.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Akshat
 */

#include "pid_controller.h"
#include "main.h"
#include <math.h>

volatile uint8_t pid_legacy_zerospeed = 0;

void PID_Create(PID_Handle_t* handle, float kp, float ki, float kd, float sample_time_s){

	if (handle == NULL) return;

		handle->kp = kp;
		handle->ki = ki;
		handle->kd = kd;
		handle->sample_time_s = sample_time_s;
		handle->last_tick_ms = HAL_GetTick();

		PID_Reset(handle);
		// change this to 1.0 after bench test
		handle->out_min = -1.0f;	// default full duty cycle in rev
		handle->out_max = 1.0f;		// default full duty cycle in fwd

		// tune this:
		handle->deadband_measurement = 1.0f;	// 1 RPM measurement deadband
		handle->deadband_setpoint = 0.5f;		// trade <= 0.5 setpoint as 0
		handle->integral_decay = 0.90f;			// decay integral by 10%/iteration when stopped
		handle->max_slew_rate = 50.0f;			// units/sec
}

float PID_Compute(PID_Handle_t* handle, float setpoint, float measurement){

	if (handle == NULL) return 0.0f;

	bool target_zero;
	if (pid_legacy_zerospeed) {
		/* historical implementation: no deadbands at all */
		target_zero = (setpoint == 0.0f);
	} else {
		if (fabsf(measurement) < handle->deadband_measurement) measurement = 0.0f;		// measurement deadband for jitters
		target_zero = fabsf(setpoint) <= handle->deadband_setpoint;
		if (target_zero) setpoint = 0.0f;												// make change based on cmd_vel working
	}
											// make change based on cmd_vel working

	// for dt:
	float dt = handle->sample_time_s;		// ideal dt
	if (handle->sample_time_s <= 0.0f){		// adaptive dt
		uint32_t now = HAL_GetTick();
		uint32_t elapsed = now - handle->last_tick_ms;
		handle->last_tick_ms = now;
		if (elapsed == 0) dt = 0.001f; else dt = (float)elapsed / 1000.0f;
	}

	if (dt <= 0.0f) dt = 0.001f;		// safety (?)

	float error = setpoint - measurement;

	// Proportional
	float p_term = (handle->kp) * error;

	// Integral: integrate when target != 0. otherwise decay
	if (!target_zero){
		handle->integral_sum += handle->ki * error * dt;
	}
	else if (pid_legacy_zerospeed) {
		handle->integral_sum = 0.0f;			/* hard reset: the old way */
	}
	else {
		handle->integral_sum *= handle->integral_decay;
		if (fabsf(handle->integral_sum) < 0.0001f) handle->integral_sum = 0.0f;
	}

	// anti-windup logic (trying windup as well pata nahi chalega ya nahi):
	if ((handle->integral_sum) > (handle->out_max))handle->integral_sum = handle->out_max;
	if ((handle->integral_sum) < (handle->out_min))handle->integral_sum = handle->out_min;

	float i_term = handle->integral_sum;

	// Derivative
	float d_term = 0.0f;
	d_term = -(handle->kd)*(measurement - (handle->last_measurement))/dt;
	handle->last_measurement = measurement;

	// total summed up output
	float output = p_term + i_term + d_term;

	// Slew limiter (units/sec):
	float maxDelta = handle->max_slew_rate * dt;
	float delta = output - handle->last_output;
	if (delta > maxDelta) output = handle->last_output + maxDelta;
	else if (delta < -maxDelta) output = handle->last_output - maxDelta;

	// clamped output
	if(output > (handle->out_max)) output = handle->out_max;
	else if (output < (handle->out_min)) output = handle->out_min;

	handle->last_output = output;

	return output;
}

void PID_SetTunings(PID_Handle_t* handle, float kp, float ki, float kd){

	if (handle == NULL) return;
	handle->kp = kp;
	handle->ki = ki;
	handle->kd = kd;
}

void PID_SetOutputLimits(PID_Handle_t* handle, float min, float max){

	if (handle == NULL) return;
	if (min >= max) return;
	handle->out_min = min;
	handle->out_max = max;
}

void PID_SetDeadbands(PID_Handle_t* handle, float setpoint_deadband, float measurement_deadband){
	if (handle == NULL) return;
	if (setpoint_deadband >= 0.0f) handle->deadband_setpoint = setpoint_deadband;
	if(measurement_deadband >= 0.0f) handle->deadband_measurement = measurement_deadband;
}

void PID_SetIntegralDecay(PID_Handle_t* handle, float decay_factor){
	if (handle == NULL) return;
	if (decay_factor >= 0.0f && decay_factor <= 1.0f) handle->integral_decay = decay_factor;
}

void PID_SetMaxSlewRate(PID_Handle_t* handle, float units_per_sec){
	if (handle == NULL) return;
	if (units_per_sec >= 0.0f) handle->max_slew_rate = units_per_sec;
}

void PID_Reset(PID_Handle_t* handle){

	if (handle == NULL) return;
	handle->integral_sum = 0.0f;
	handle->last_measurement = 0.0f;
	handle->last_output = 0.0f;
	handle->last_tick_ms = HAL_GetTick();
}
