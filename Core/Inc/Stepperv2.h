/*
 * Stepperv2.h
 *
 *  Created on: Jan 11, 2026
 *      Author: Soham Saxena
 */

#ifndef INC_STEPPERV2_H_
#define INC_STEPPERV2_H_
#include <stdint.h>
#include <math.h>
#include "main.h"
#include "motor_driver.h"
#include <stdio.h>


#define BURST_SIZE 25

typedef struct Wrapper{
	float degree, rpm;
}Wrapper;

typedef struct WQueue{
	Wrapper queue[100];
	int head, tail;
}WQueue;

void initWQueue(WQueue* q);

void enqueueW(WQueue* q, Wrapper elem);

Wrapper dequeueW(WQueue* q);
uint8_t WisEmpty(WQueue* q);


typedef struct{
    TIM_HandleTypeDef* step_timer;
    uint32_t step_channel;
    GPIO_TypeDef* step_dir_port;
    uint16_t step_dir_pin;
    GPIO_TypeDef* ena_port;
    uint16_t ena_pin;
    float steps_per_rev;	// 200*microsteps
    volatile uint32_t step_counter;
    volatile uint32_t target_steps;
    uint16_t maxRPM;
    uint8_t queueMode; //0 -> no QUEUE (instant reaction), 1 -> default QUEUE ready
    WQueue q;
    volatile uint8_t isMoving;
	volatile uint8_t recievedStepper;
	volatile int8_t dir; //-1->forward direction, 1->backward direction
	float step_accum; //for preventing rounding off errors
	Wrapper lastInstruct;
	uint64_t totalPulses;

	//Absolute angle stuff
	long abs_step_count;
	float absolute_angle_f;
	float absolute_angle_b;
	uint8_t rpm_smoothening; //experimental mode, not sure if working or not yet
	volatile uint8_t pending_preemption;
	volatile uint32_t last_burst_size;

	//homing stuff
	float limSwitchOffset; //offset of limit switch from true 0
	uint8_t correctOffset; //0 -> not initiated/initiated and completed, 1 -> send command to correct offset, 2 -> correcting offset currently
	uint8_t homing_status; //homing_status & !correctOffset -> offset has been corrected, homing is ready
	float db_lower_bound;
	float db_upper_bound;

	//constraints stuff
	uint8_t constraintMode;//0 -> follows commands as is, 1-> makes decisions based on the following constraints:
	// 1 -> picks the shortest path between forward (target_angle) and backwards (target_angle + 180)
	// 2 -> ensures deadzone of limit switch is not hit and as a result also doesnt do more than +-360 absolute rotations
} Stepper_Handle_t;

extern Stepper_Handle_t S1;
extern Stepper_Handle_t S2;
extern Stepper_Handle_t S3;
extern Stepper_Handle_t S4;

void Stepper_Create(
    Stepper_Handle_t* handle,
    TIM_HandleTypeDef* step_timer,
    uint32_t step_channel,
    GPIO_TypeDef* step_dir_port,
    uint16_t step_dir_pin,
    GPIO_TypeDef* ena_port,
    uint16_t ena_pin,
    uint16_t steps_per_rev,
    uint8_t queueMode,
	uint8_t rpm_smoothening,
	uint8_t constraintMode,
	float limSwitchOffset,
	float db_lower_bound,
	float db_upper_bound
);

void initTimer(Stepper_Handle_t* handle);
void setRPM(Stepper_Handle_t* stepper, float rpm);
void sendPulses(Stepper_Handle_t* stepper, uint32_t pulses, float rpm);
void moveAngle(Stepper_Handle_t* stepper, float degree, float rpm);
void moveAngleAbsolute(Stepper_Handle_t* stepper, float absolute_angle, float rpm, Motor_Handle_t* bdc);
void toggleQueue(Stepper_Handle_t* stepper);
void Stepper_Stop(Stepper_Handle_t* stepper);

void Stepper_Enable(Stepper_Handle_t* handle);		// active low

void Stepper_Disable(Stepper_Handle_t* handle);		// active high

#endif /* INC_STEPPERV2_H_ */
