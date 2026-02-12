/*
 * Stepperv2.h
 *
 *  Created on: Jan 11, 2026
 *      Author: Soham Saxena
 */

#ifndef INC_STEPPERV2_H_
#define INC_STEPPERV2_H_
#include "stm32g4xx_hal.h"
#include <stdint.h>

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
    uint8_t accessed; //lock to avoid concurrency issues
    WQueue q;
    volatile uint8_t isMoving;
	volatile uint8_t recievedStepper;
	volatile int8_t dir; //-1->forward direction, 1->backward direction
	float step_accum; //for preventing rounding off errors
	Wrapper lastInstruct;
	uint64_t totalPulses;

	//Absolute angle stuff
	long abs_step_count;
	float absolute_angle;
	uint8_t rpm_smoothening; //experimental mode, not sure if working or not yet
	int offset;
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
	uint8_t rpm_smoothening
);

void initTimer(Stepper_Handle_t* handle);
void setRPM(Stepper_Handle_t* stepper, float rpm);
void sendPulses(Stepper_Handle_t* stepper, uint32_t pulses, float rpm);
void moveAngle(Stepper_Handle_t* stepper, float degree, float rpm);
void moveAngleAbsolute(Stepper_Handle_t* stepper, float absolute_angle, float rpm);
void toggleQueue(Stepper_Handle_t* stepper);
void Stepper_Stop(Stepper_Handle_t* stepper);

void Stepper_Enable(Stepper_Handle_t* handle);		// active low

void Stepper_Disable(Stepper_Handle_t* handle);		// active high

#endif /* INC_STEPPERV2_H_ */
