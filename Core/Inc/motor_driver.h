/*
 * motor_driver.h
 *
 *  Created on: Oct 10, 2025
 *      Author: Akshat
 */

#ifndef INC_MOTOR_DRIVER_H_
#define INC_MOTOR_DRIVER_H_

#include "stm32g4xx_hal.h"

extern volatile int b1_target_rpm;
extern volatile int b2_target_rpm;
extern volatile int b3_target_rpm;
extern volatile int b4_target_rpm;

typedef struct{
    TIM_HandleTypeDef* pwm_timer;
    uint32_t pwm_channel;
    GPIO_TypeDef* dir_port;
    uint16_t dir_pin;
} Motor_Handle_t;

void Motor_Create(Motor_Handle_t* handle, TIM_HandleTypeDef* pwm_timer, uint32_t pwm_channel, GPIO_TypeDef* dir_port, uint16_t dir_pin);

void handle_command(char *cmd);

int parse_cmd(const char **p);

void Motor_SetOutput(Motor_Handle_t* handle, float output);

#endif /* INC_MOTOR_DRIVER_H_ */
