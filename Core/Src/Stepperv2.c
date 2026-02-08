/*
 * Stepperv2.c
 *
 *  Created on: Jan 11, 2026
 *      Author: Soham Saxena
 */

#include "Stepperv2.h"
#include <math.h>
#include <main.h>
static inline float clamp_deg_180_pos(float a)
{
    a = fmodf(a, 360.0f);

    if (a <= -180.0f) a += 360.0f;  // -180 → +180
    if (a >  180.0f)  a -= 360.0f;

    return a;   // (-180, 180]
}

void Stepper_Create(Stepper_Handle_t* handle, TIM_HandleTypeDef* step_timer ,uint32_t step_channel, GPIO_TypeDef* step_dir_port, uint16_t step_dir_pin,  GPIO_TypeDef* ena_port, uint16_t ena_pin, uint16_t steps_per_rev, uint8_t queueMode){

	if (handle == NULL) return;

	handle->step_timer = step_timer;
	handle->step_channel = step_channel;
	handle->step_dir_port = step_dir_port;
	handle->step_dir_pin = step_dir_pin;
	handle->ena_port = ena_port;
	handle->ena_pin = ena_pin;
	handle->steps_per_rev = steps_per_rev;
	handle->maxRPM = 1500;
	handle->queueMode = queueMode;
	initWQueue(&handle->q);

	handle->step_counter = 0;
	handle->target_steps = 0;
	handle->isMoving = 0;
	handle->recievedStepper = 0;
	handle->dir = 0;
	handle->lastInstruct.degree = 0.0f;
	handle->lastInstruct.rpm = 0;
	handle->step_accum = 0.0f;
	handle->totalPulses = 0;
	handle->abs_step_count = 0;
	handle->absolute_angle = 0.0f;
}

void initTimer(Stepper_Handle_t* handle){
    HAL_TIM_Base_Start(handle->step_timer);
    //__HAL_TIM_URS_ENABLE(handle->step_timer);
    HAL_TIM_PWM_Start(handle->step_timer, handle->step_channel);
}

void Stepper_Stop(Stepper_Handle_t* stepper)
{
    if (!stepper || !stepper->isMoving) return;


    //__HAL_TIM_DISABLE_IT(stepper->step_timer, TIM_IT_UPDATE);
    //HAL_TIM_PWM_Stop(stepper->step_timer, stepper->step_channel);
    __HAL_TIM_SET_COMPARE(stepper->step_timer, stepper->step_channel, 0);
    //__HAL_TIM_CLEAR_IT(stepper->step_timer, TIM_IT_UPDATE);


    //stepper->totalPulses++;
    stepper->isMoving = 0;
    stepper->absolute_angle = clamp_deg_180_pos(stepper->abs_step_count * 360 / stepper->steps_per_rev);
}


void toggleQueue(Stepper_Handle_t* stepper){
	if (stepper->queueMode) stepper->queueMode = 0;
	else stepper->queueMode = 1;
}

void setRPM(Stepper_Handle_t* stepper, float rpm){
	if (rpm > stepper->maxRPM) rpm = stepper->maxRPM;
    float pulseFreq = (rpm * stepper->steps_per_rev) / 60.0f;
    if (rpm <= 0.0f || stepper->steps_per_rev == 0) return;
    uint32_t timer_clk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
        timer_clk *= 2;
    float arr_f = (timer_clk / ( (125 + 1) * pulseFreq )) - 1.0f;
    uint32_t ARR = (uint32_t)(arr_f + 0.5f);

    if (ARR < 2) ARR = 2; //sanity check to ensure ARR isnt too small
    if ((ARR & 1) == 0) ARR++; // rounding  // force odd if you want perfect 50/50 duty
    uint32_t CCR = ARR / 2;


    __HAL_TIM_SET_AUTORELOAD(stepper->step_timer, ARR);
    __HAL_TIM_SET_COMPARE(stepper->step_timer, stepper->step_channel, CCR);

    //stepper->step_timer->Instance->EGR = TIM_EGR_UG;
}

void sendPulses(Stepper_Handle_t* stepper, uint32_t pulses, float rpm){
	if (pulses == 0) {
	        stepper->isMoving = 0;
	        return;
	    }
	setRPM(stepper, rpm);
	//__HAL_TIM_DISABLE_IT(stepper->step_timer, TIM_IT_UPDATE);
	//__HAL_TIM_CLEAR_IT(stepper->step_timer, TIM_IT_UPDATE);
	__HAL_TIM_SET_COUNTER(stepper->step_timer, 0);
	stepper->step_counter = 0;
	stepper->target_steps = pulses;
    __HAL_TIM_ENABLE_IT(stepper->step_timer, TIM_IT_UPDATE); // enable UPDATE IRQ
	//HAL_TIM_PWM_Start(stepper->step_timer, stepper->step_channel);     // start PWM (no need _IT here)
	stepper->isMoving = 1;
}

void moveAngle(Stepper_Handle_t* stepper, float degree, float rpm){
	if (degree >=0) {
		HAL_GPIO_WritePin(stepper->step_dir_port, stepper->step_dir_pin, GPIO_PIN_SET);
		stepper->dir = 1;
	}
	else{
		HAL_GPIO_WritePin(stepper->step_dir_port, stepper->step_dir_pin, GPIO_PIN_RESET);
		stepper->dir = -1;
	}
	delay_us(5);
	float signed_fpulses = (degree / 360.0f) * stepper->steps_per_rev;
	stepper->step_accum += signed_fpulses;

	int32_t pulses = (int32_t) stepper->step_accum;
	stepper->step_accum -= pulses;

	if (pulses < 0) {
	    pulses = -pulses;
	}
	//if (pulses < 1) return;

	sendPulses(stepper, pulses, rpm);
}

void moveAngleAbsolute(Stepper_Handle_t* stepper, float absolute_angle, float rpm){
	float targetAngle_360 = fmodf(absolute_angle + 360.0f, 360.0f);
	float currAngle_360   = fmodf(stepper->absolute_angle + 360.0f, 360.0f); //conversion to 360


	float delta = targetAngle_360 - currAngle_360;
	if (fabs(delta) < (float) 360.0f/stepper->steps_per_rev) return; //if movement is lesser than motor resolution can handle, just skip

	if (delta > 180.0f)  delta -= 360.0f;
	if (delta < -180.0f) delta += 360.0f; //shortest path chosen

	moveAngle(stepper, delta, rpm);
}

void Stepper_Enable(Stepper_Handle_t* handle){	// active low
	if (handle == NULL) return;

	HAL_GPIO_WritePin(handle->ena_port, handle->ena_pin, GPIO_PIN_RESET);	// active low
}

void Stepper_Disable(Stepper_Handle_t* handle){	// active high
	HAL_GPIO_WritePin(handle->ena_port, handle->ena_pin, GPIO_PIN_SET);
}

void initWQueue(WQueue* q){
	q->head = -1;
	q->tail = -1;
	for(int i = 0; i < 100; i++){
		q->queue[i].degree = 0;
		q->queue[i].rpm = 0;
	}
}

void enqueueW(WQueue* q, Wrapper elem){
	if (q->head == -1){
		q->head = 0;
		q->tail = 0;
		q->queue[q->head] = elem;
		return;
	}
	if((q->head+1)%100 == q->tail) return;

	q->head = (q->head+1)%100;
	q->queue[q->head] = elem;
}

Wrapper dequeueW(WQueue* q){
	if (q->head == -1){
		Wrapper temp = {0.0f, 0.0f};
		//CDC_Transmit_FS("Queue is empty.\n", sizeof("Queue is empty.\n"));
		return temp;
	}
	if (q->head == q->tail){//only one element in queue
		Wrapper elem = q->queue[q->tail];
		q->head = -1;
		q->tail = -1;
		return elem;
	}
	Wrapper elem = q->queue[q->tail];
	q->tail = (q->tail+1)%100;

	return elem;
}

uint8_t WisEmpty(WQueue* q){
	return (q->tail == -1);
}
