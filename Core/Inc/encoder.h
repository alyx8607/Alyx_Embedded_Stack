/*
 * encoder.h
 *
 *  Created on: Oct 10, 2025
 *      Author: Akshat
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "stm32g4xx_hal.h"

typedef struct{
	int head, tail;
	float queue[100]; //100 is the max number of elements possible in the queue
}Queue;//technically circular queue

typedef struct{
	int N; //window size
	uint8_t type;//0 -> non-weighted, 1 -> linear, 2 -> exponential
	float* weights;
	Queue q;
} MA_calc; //calculator for calculating moving average

typedef struct {
    TIM_HandleTypeDef* timer;
    float counts_per_rev;
    float prev_time;
    int32_t last_count;
    float speed_rpm;
    MA_calc calculator;
} Encoder_Handle_t;

void Encoder_Create(Encoder_Handle_t* handle, TIM_HandleTypeDef* timer, float counts_per_rev);

void Encoder_Start(Encoder_Handle_t* handle);

float Encoder_GetSpeedRPM(Encoder_Handle_t* handle);
void Encoder_free(Encoder_Handle_t* handle);
// call this at a fixed interval (whtvr ur sample_time_s is)

#endif /* INC_ENCODER_H_ */
