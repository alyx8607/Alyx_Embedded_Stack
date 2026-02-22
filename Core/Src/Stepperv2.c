/*
 * Stepperv2.c
 *
 *  Created on: Jan 11, 2026
 *      Author: Soham Saxena
 */

#include "Stepperv2.h"
static inline float clamp_deg_180_pos(float a)
{
    a = fmodf(a, 360.0f);

    if (a <= -180.0f) a += 360.0f;  // -180 → +180
    if (a >  180.0f)  a -= 360.0f;

    return a;   // (-180, 180]
}
static inline float wrap360(float a)
{
    a = fmodf(a, 360.0f);
    if (a < 0) a += 360.0f;
    return a;
}

//static void enableCCRPreload(TIM_HandleTypeDef *htim, uint32_t channel)
//{
//    if (channel == TIM_CHANNEL_1) {
//        htim->Instance->CCMR1 |= TIM_CCMR1_OC1PE;
//    }
//    else if (channel == TIM_CHANNEL_2) {
//        htim->Instance->CCMR1 |= TIM_CCMR1_OC2PE;
//    }
//    else if (channel == TIM_CHANNEL_3) {
//        htim->Instance->CCMR2 |= TIM_CCMR2_OC3PE;
//    }
//    else if (channel == TIM_CHANNEL_4) {
//        htim->Instance->CCMR2 |= TIM_CCMR2_OC4PE;
//    }
//}

float calcAngularDiff(float a, float b){
	float result = a - b;
	if (result < -180) result += 360;
	else if (result > 180) result -= 360;

	return result;
}

uint8_t arcContains(float start, float end, uint8_t dir, float angle){
    start = wrap360(start);
    end   = wrap360(end);
    angle = wrap360(angle);

    if(dir == 1){  // CCW (increasing angle)
        if(start <= end)
            return angle >= start && angle <= end;
        else  // wraps past 0
            return angle >= start || angle <= end;
    } else {        // CW
        if(start >= end)
            return angle <= start && angle >= end;
        else
            return angle <= start || angle >= end;
    }
}

static uint8_t angleInDeadband(float angle, float center, float halfWidth)
{
    float diff = clamp_deg_180_pos(angle - center);
    return fabsf(diff) <= halfWidth;
}

uint8_t checkDeadband(float startAngle, float endAngle, uint8_t dir, float avoid, int deadbandSize)
{
    float half = deadbandSize / 2.0f;

    float lDB = wrap360(avoid - half);
    float uDB = wrap360(avoid + half);

    uint8_t l_hit = arcContains(startAngle, endAngle, dir, lDB);
    uint8_t u_hit = arcContains(startAngle, endAngle, dir, uDB);

    uint8_t start_inside = angleInDeadband(startAngle, avoid, half);
    uint8_t end_inside   = angleInDeadband(endAngle, avoid, half);

    return l_hit || u_hit || (start_inside && end_inside);
}

void Stepper_Create(Stepper_Handle_t* handle, TIM_HandleTypeDef* step_timer ,uint32_t step_channel, GPIO_TypeDef* step_dir_port, uint16_t step_dir_pin,  GPIO_TypeDef* ena_port, uint16_t ena_pin, uint16_t steps_per_rev, uint8_t queueMode, uint8_t rpm_smoothening, uint8_t constraintMode, float limSwitchOffset){

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
	handle->absolute_angle_f = 0.0f;
	handle->absolute_angle_b = 180.0f;
	handle->rpm_smoothening = rpm_smoothening;
	handle->pending_preemption = 0;
	handle->last_burst_size = 0;
	handle->homing_status = 0;
	handle->correctOffset = 0;
	handle->constraintMode = constraintMode;
	handle->limSwitchOffset = limSwitchOffset;
}

void initTimer(Stepper_Handle_t* handle){
	__HAL_TIM_DISABLE(handle->step_timer);
	__HAL_TIM_URS_ENABLE(handle->step_timer);

	    // Enable CC channel and MOE once, never touch again
	TIM_CCxChannelCmd(handle->step_timer->Instance, handle->step_channel, TIM_CCx_ENABLE);
	__HAL_TIM_MOE_ENABLE(handle->step_timer);

	    // Enforce OPM once here while timer is stopped
	handle->step_timer->Instance->CR1 |= TIM_CR1_OPM;
}

void Stepper_Stop(Stepper_Handle_t* stepper)
{
    if (!stepper)
        return;

    TIM_TypeDef *tim = stepper->step_timer->Instance;

    // If timer is running, stop immediately
    if (tim->CR1 & TIM_CR1_CEN)
    {
        __HAL_TIM_DISABLE(stepper->step_timer);   // Clear CEN
    }

    // Disable update interrupt
    __HAL_TIM_DISABLE_IT(stepper->step_timer, TIM_IT_UPDATE);

    // Clear pending update flag (important)
    __HAL_TIM_CLEAR_FLAG(stepper->step_timer, TIM_FLAG_UPDATE);

    stepper->isMoving = 0;
    stepper->step_timer->Instance->RCR = 0;
    stepper->absolute_angle_f =
        clamp_deg_180_pos(
            stepper->abs_step_count * 360.0f / stepper->steps_per_rev
        );
    stepper->absolute_angle_b = clamp_deg_180_pos(stepper->absolute_angle_f + 180);
}



void toggleQueue(Stepper_Handle_t* stepper){
	if (stepper->queueMode) stepper->queueMode = 0;
	else stepper->queueMode = 1;
}

void setRPM(Stepper_Handle_t* stepper, float rpm){
    //__HAL_TIM_SET_COMPARE(stepper->step_timer, stepper->step_channel, 0);
	//Stepper_Stop(stepper);
	if (rpm > stepper->maxRPM) rpm = stepper->maxRPM;
    float pulseFreq = (rpm * stepper->steps_per_rev) / 60.0f;
    if (rpm <= 0.0f || stepper->steps_per_rev == 0) return;
    uint32_t timer_clk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
        timer_clk *= 2;
    uint32_t psc = stepper->step_timer->Instance->PSC;
    float arr_f = (timer_clk / ((psc + 1) * pulseFreq)) - 1.0f;

    uint32_t ARR = (uint32_t)(arr_f + 0.5f);

    if (ARR < 2) ARR = 2; //sanity check to ensure ARR isnt too small
    if ((ARR & 1) == 0) ARR++; // rounding  // force odd if you want perfect 50/50 duty
    uint32_t CCR = ARR / 2;


    __HAL_TIM_SET_AUTORELOAD(stepper->step_timer, ARR);
    __HAL_TIM_SET_COMPARE(stepper->step_timer, stepper->step_channel, CCR);

    //stepper->step_timer->Instance->EGR = TIM_EGR_UG;
}

void sendPulses(Stepper_Handle_t* stepper, uint32_t pulses, float rpm)
{
    if (!stepper || pulses == 0) {
        if (stepper) Stepper_Stop(stepper);
        return;
    }

    TIM_TypeDef *tim = stepper->step_timer->Instance;

    // 1. Hard stop - timer must be off before touching RCR/ARR
    __HAL_TIM_DISABLE(stepper->step_timer);
    __HAL_TIM_DISABLE_IT(stepper->step_timer, TIM_IT_UPDATE);
    __HAL_TIM_CLEAR_FLAG(stepper->step_timer, TIM_FLAG_UPDATE);

    // 2. Set RPM (writes ARR/CCR preloads while timer is stopped - safe)
    if (rpm > 0.0f) setRPM(stepper, rpm);

    // 3. Set up burst
    stepper->target_steps   = pulses;
    stepper->isMoving       = 1;

    uint32_t chunk = (pulses < BURST_SIZE) ? pulses : BURST_SIZE;
    stepper->last_burst_size = chunk;

    tim->RCR = chunk - 1;
    tim->CNT = 0;

    // 4. Generate update event to push preloads (ARR, CCR, RCR) into shadows
    tim->EGR = TIM_EGR_UG;

    // 5. MUST clear UIF immediately - EGR_UG sets it even with URS=1
    //    on advanced timers when the timer is stopped
    __HAL_TIM_CLEAR_FLAG(stepper->step_timer, TIM_FLAG_UPDATE);

    // 6. Now safe to enable IT and start
    __HAL_TIM_ENABLE_IT(stepper->step_timer, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE(stepper->step_timer);
}



void moveAngle(Stepper_Handle_t* stepper, float degree, float rpm){
	if (degree >=0) {
		HAL_GPIO_WritePin(stepper->step_dir_port, stepper->step_dir_pin, GPIO_PIN_RESET);
		stepper->dir = 1;
	}
	else{
		HAL_GPIO_WritePin(stepper->step_dir_port, stepper->step_dir_pin, GPIO_PIN_SET);
		stepper->dir = -1;
	}
	delay_us(5);
	float signed_fpulses = (degree / 360.0f) * stepper->steps_per_rev;
	stepper->step_accum += signed_fpulses;

	int32_t pulses = (int32_t)(stepper->step_accum +
	                 (stepper->step_accum >= 0 ? 0.5f : -0.5f)); // round, don't truncate
	stepper->step_accum -= pulses;

	if (pulses < 0) {
	    pulses = -pulses;
	}

	if (pulses < 1) return;

	sendPulses(stepper, pulses, rpm);
}

void moveAngleAbsolute(Stepper_Handle_t* stepper, float absolute_angle, float rpm, Motor_Handle_t* bdc){
	uint8_t mode = bdc->mode;
	float targetAngle_360 = fmodf(absolute_angle + 360.0f, 360.0f);
	float currAngle_360   = fmodf(mode ? stepper->absolute_angle_f + 360.0f : stepper->absolute_angle_b + 360.0f, 360.0f); //conversion to 360

	float delta = calcAngularDiff(targetAngle_360, currAngle_360);
	if (fabs(delta) < (float) 360.0f/stepper->steps_per_rev) return; //if movement is lesser than motor resolution can handle, just skip



	if (stepper->constraintMode){
		uint8_t modeA = bdc->mode;
		uint8_t modeB = !modeA;
		float startAngleA = currAngle_360;
		float startAngleB = fmodf(currAngle_360 + 180.0f, 360.0f);
		float deltaA = calcAngularDiff(targetAngle_360, startAngleA);
		float deltaB = calcAngularDiff(targetAngle_360, startAngleB);
		uint8_t deadA = checkDeadband(stepper->absolute_angle_f, stepper->absolute_angle_f + deltaA, deltaA > 0, stepper->limSwitchOffset, 10);
		uint8_t deadB = checkDeadband(stepper->absolute_angle_f, stepper->absolute_angle_f + deltaB, deltaB > 0, stepper->limSwitchOffset, 10);
		if(deadA){
			if(deadB) return; //hope it never hits this
			delta = deltaB;
			mode = modeB;
			currAngle_360 = startAngleB;
		}
		else if (deadB){
			delta = deltaA;
			mode = modeA;
			currAngle_360 = startAngleA;
		}
		else{
			uint8_t selectionCond = (fabs(deltaA) < fabs(deltaB));
			delta = selectionCond ? deltaA : deltaB;
			mode = selectionCond ? modeA : modeB;
			currAngle_360 = selectionCond ? startAngleA : startAngleB;
		}

	}

	if (stepper->rpm_smoothening){
		rpm = 100*tanhf(fabs(0.008f*delta));
		if (rpm < 3) rpm = 3;
	}

//	float future_steps = stepper->abs_step_count + (delta / 360.0f) * stepper->steps_per_rev;
//
//	if (future_steps > stepper->steps_per_rev ||
//	    future_steps < -stepper->steps_per_rev)
//	{
//	    // reject move
//	    return;
//	}

	bdc->mode = mode;
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
