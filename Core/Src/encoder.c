/*
 * encoder.c
 *
 *  Created on: Oct 10, 2025
 *      Author: Akshat
 */

#include "encoder.h"
#include "main.h"
#include <stdlib.h>
#include <stddef.h>
#include <math.h>

// begin Soham's code
void Queue_init(Queue* q){
	q->head = -1;
	q->tail = -1;
}

void enqueue(Queue* q, float value){
	if (q->head == -1){ //no elements in queue
		q->head = 0;
		q->tail = 0;
		q->queue[q->head] = value;
		return;
	}
	if ((q->head + 1)%100 == q->tail) return; //queue is full
	q->head = (q->head + 1) % 100;
	q->queue[q->head] = value;
}

float dequeue(Queue* q){
	if (q->head == -1) return 0; //no elements in queue
	if (q->head == q->tail){ //only one element in queue
		float element = q->queue[q->tail];
		q->head = -1;
		q->tail = -1;
		return element;
	}
	float element = q->queue[q->tail];
	q->tail = (q->tail + 1) % 100;

	return element;
}


void MA_init(MA_calc* calculator, int N, uint8_t type){
	calculator->type = type;
	Queue_init(&calculator->q);
	if (N>100) N = 100; //clamp N value to max
	calculator->N = N;
	for(int i = 0; i < N; i++){
		enqueue(&calculator->q, 0.0f);
	}
	switch(type){
	case 1:
		calculator->weights = (float*) malloc(N * sizeof(float));
		if (!calculator->weights)
			return; //failed to allocate memory.
		for(int i = 1; i <= N; i++)
			*(calculator->weights + i - 1) = 2*i;
		break;
	case 2:
		calculator->weights = (float*) malloc(N * sizeof(float));
		if (!calculator->weights)
			return; //failed to allocate memory
		for(int i = 1; i <=N; i++)
				*(calculator->weights + i - 1) =  expf(-0.07f * (N - i));
		break;
	default:
		calculator->weights = NULL;
		calculator->type = 0;
	};
}


float calc_MA(MA_calc* calculator, float value){
	dequeue(&calculator->q);
	enqueue(&calculator->q, value);
	float sum = 0.0f;
	float weightSum = 0.0f;
	if (calculator->type == 0) weightSum = calculator->N;
	for(int i = 0; i < calculator->N; i++){
		if (calculator->type != 0) {weightSum += *(calculator->weights + i);
		sum += *(calculator->weights + i) * (calculator->q.queue[(calculator->q.tail+i)%100]);}
		else sum += calculator->q.queue[(calculator->q.tail+i)%100];
	}
	return sum/weightSum;
}

void Encoder_free(Encoder_Handle_t* handle){
	free(handle->calculator.weights);
}
// end Soham's code

void Encoder_Create(Encoder_Handle_t* handle, TIM_HandleTypeDef* timer, float counts_per_rev){

	if (handle == NULL) return;

	handle->timer = timer;
	handle->counts_per_rev = counts_per_rev;
	handle->prev_time = 0;
	handle->last_count = 0.0f;
	handle->speed_rpm = 0.0f;
	MA_init(&handle->calculator, 100, 2);
}

void Encoder_Start(Encoder_Handle_t* handle){

	if (handle == NULL) return;

	__HAL_TIM_SET_COUNTER(handle->timer, 0);	// resetting counter
	handle->last_count = 0;

	// start timer in encoder mode on both channels:
	HAL_TIM_Encoder_Start(handle->timer, TIM_CHANNEL_ALL);
}

//float Encoder_GetSpeedRPM(Encoder_Handle_t* handle){
//
//	int32_t current_count = (int32_t)__HAL_TIM_GET_COUNTER(handle->timer);
//	int32_t delta_counts = current_count - (handle->last_count);
//	handle->last_count = current_count;
//
//	// convert delta to RPM w/ formula (delta*60/CPR*sample_time)
//	handle->speed_rpm = calc_MA(&handle->calculator, (delta_counts * 60.0f) / (handle->counts_per_rev * handle->sample_time_s));
//	//handle->speed_rpm = (delta_counts * 60.0f) / (handle->counts_per_rev * handle->sample_time_s);
//
//	return handle->speed_rpm;
//}

float Encoder_GetSpeedRPM(Encoder_Handle_t* handle){
    uint32_t current = __HAL_TIM_GET_COUNTER(handle->timer);
    uint32_t now = HAL_GetTick();

    // handle 16-bit timer wrap-around
    int16_t delta_16 = (int16_t)(current - (uint32_t)handle->last_count);
    int32_t delta = (int32_t)delta_16;

    uint32_t dt_ms = now - handle->prev_time;
    if (dt_ms == 0) dt_ms = 1; // prevent divide by zero on the first loop

    // Dynamic RPM calculation: (Delta / CPR) * (60,000ms / dt_ms)
    float rpm_f = (((float)delta * 60000.0f) / (float)dt_ms) / handle->counts_per_rev;

    handle->speed_rpm = calc_MA(&handle->calculator, rpm_f);

    handle->last_count = current;
    handle->prev_time = now;

    return handle->speed_rpm;
}

//float Encoder_GetSpeedRPM(Encoder_Handle_t* handle){
//
//    uint32_t current = __HAL_TIM_GET_COUNTER(handle->timer);
//    int16_t delta_16 = (int16_t)(current - (uint32_t)handle->last_count);
//    int32_t delta = (int32_t)delta_16;
//
//    // loop runs every 10ms (0.01s)
//    // RPM = (Delta / CPR) * (60s / 0.01s)
//    // RPM = (Delta / CPR) * 6000
//
//    float rpm_f = (((float)delta * 60.0f * CTRL_Loop_Freq) / handle->counts_per_rev);
//
//    handle->speed_rpm = calc_MA(&handle->calculator, rpm_f);
//
//    handle->last_count = current;
//    return handle->speed_rpm;
//}
