/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_driver.h"
#include "encoder.h"
#include "pid_controller.h"
#include "Stepperv2.h"
#include "globals.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>   // isspace
#include <stdlib.h>  // strtol & strtof
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CTRL_Loop_Period (1.0f / CTRL_Loop_Freq)
#define ENCODERS_CPR 1993			// bilda motors
//#define ENCODERS_CPR 92733		// old rhino motors
#define DEBOUNCE_TIME_PERIOD 50		// debounce time for lim switches
#define Stepper_Microsteps 1
#define Stepper_Motor_Steps_Per_Rev 800
//#define Stepper_Motor_Steps_Per_Rev 1600
#define Stepper_Steps_Per_Rev (Stepper_Motor_Steps_Per_Rev * Stepper_Microsteps)
#define rx_buf_size 64
#define feedback_buf_size 512
#define feedback_transmission_freq 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
PID_Handle_t pid_b1;
PID_Handle_t pid_b2;
PID_Handle_t pid_b3;
PID_Handle_t pid_b4;
// numbering system
// 1   2
// 3   4
// for back-left motors
Motor_Handle_t B1; 		// B -> BDC
Motor_Handle_t B2;
Motor_Handle_t B3;
Motor_Handle_t B4;
Encoder_Handle_t E1; 	// E -> Encoder
Encoder_Handle_t E2;
Encoder_Handle_t E3;
Encoder_Handle_t E4;
Stepper_Handle_t S1;	// S -> Stepper
Stepper_Handle_t S2;
Stepper_Handle_t S3;
Stepper_Handle_t S4;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;
TIM_HandleTypeDef htim20;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart5_rx;

/* USER CODE BEGIN PV */
// Encoder + RPM vars
//BDC motors
volatile int b1_target_rpm = 0;
volatile int b2_target_rpm = 0;
volatile int b3_target_rpm = 0;
volatile int b4_target_rpm = 0;
volatile float b1_current_rpm = 0.0f;
volatile float b2_current_rpm = 0.0f;
volatile float b3_current_rpm = 0.0f;
volatile float b4_current_rpm = 0.0f;
float b1_control_signal = 0.0f;		// PID output - more responsive than taman with headphones on
float b2_control_signal = 0.0f;
float b3_control_signal = 0.0f;
float b4_control_signal = 0.0f;

// stepper motor
volatile int s1_target_angle = 0;
volatile int s2_target_angle = 0;
volatile int s3_target_angle = 0;
volatile int s4_target_angle = 0;

// homing steppers
volatile uint32_t last_debounce_time[4] = {0, 0, 0, 0};
volatile uint32_t current_debounce_time = 0;

volatile uint8_t control_loop = 0;		// control loop flag

// USB - USki Baat sunle lolololololol
//uint8_t instruct_buffer[64];
//volatile uint8_t usb_data_ready = 0;	// im ready for a date-a...get it? coz like...

uint8_t rx_buf[rx_buf_size];
uint8_t rx_byte;
volatile int rx_idx = 0;
volatile uint8_t callback_flag = 0;
uint8_t feedback_buf[feedback_buf_size];
volatile uint8_t uart_data_ready = 0;
uint8_t shadow_rx_buf[rx_buf_size * 2]; // This is the CPU's private copy: large to hold accumulations
char main_cmd_buf[128];                 // The CPU parses this in the while(1) loop
uint32_t lastTransmissionTime = 0;

// UART stop waala error (YOU ARE T) lololol
volatile uint8_t error_entered = 0;
volatile uint32_t last_error;
volatile uint8_t uarterror = 0;
volatile uint32_t uarterror3_error;

//uint32_t last_cmd_time = 0;
//volatile uint8_t wdt_active = 0; // indicate teleop watchdog has taken control
volatile uint8_t estop_active = 0;
volatile uint8_t estop_action_done = 0;

volatile uint8_t bpill_rx_buf[3];
volatile uint32_t last_bpill_hearbeat = 0;		// for software watchdog
uint8_t bpill_rx_byte;          // Buffer for 1 byte
volatile uint8_t bpill_rx_state = 0;
uint8_t bpill_sync_state = 0;   // Keeps track of where we are in the packet
uint8_t bpill_tx_buf[3];

static float current_kp = 0.004893002197721693f;		// par kp toh senior he lmaoooo
static float current_ki = 0.02823752341330259f;
static float current_kd = 0.00013409059780944936f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM20_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TIM6_SetPeriod_us(uint32_t period_us)
{
    uint32_t timer_clk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
        timer_clk *= 2;

    // Choose prescaler so timer ticks at 1 MHz (1 tick = 1 µs)
    uint32_t presc = (timer_clk / 1000000UL) - 1;

    TIM6->PSC = presc;
    TIM6->ARR = period_us - 1;

    TIM6->EGR = TIM_EGR_UG;   // load PSC & ARR immediately
}
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < cycles);
}

static inline float clamp_deg_180_pos(float a)
{
    a = fmodf(a, 360.0f);

    if (a <= -180.0f) a += 360.0f;  // -180 → +180
    if (a >  180.0f)  a -= 360.0f;

    return a;   // (-180, 180]
}

typedef enum {
	MODE_ESTOP = 0x00,
	MODE_HOMING = 0x01,
	MODE_TELEOP = 0x02,
	MODE_AUTONAV = 0x03,
	MODE_SELFDRIVE = 0x04,
	MODE_IDLE = 0x05
} MODES;

float angles[4] = {180.00f*5, 90.00f*5, 30.00f*5, 90.00f*5};
uint8_t i = 0;
MODES mode = MODE_HOMING;
MODES prev_rec_mode = MODE_IDLE, rec_mode = MODE_IDLE;
uint8_t last_sent_mode = 255;
uint8_t sending_mode = 0;

//float map_rpm_to_signal(float rpm) {
//
//    if (rpm > 100.0f) return 0.8f;
//    if (rpm < 0.0f)   return 0.0f;
//
//    // multiplication factor (0.8 / 100 = 0.008)
//    return rpm * 0.008f;
//}

float c_angle = 0.0f;
int c_rpm = 120;
int cmd_count = 0;

int prev_rpm = 100;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM15_Init();
  MX_TIM17_Init();
  MX_TIM20_Init();
  MX_UART5_Init();
  MX_TIM16_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  PID_Create(&pid_b1, current_kp, current_ki, current_kd, CTRL_Loop_Period);
  PID_Create(&pid_b2, current_kp, current_ki, current_kd, CTRL_Loop_Period);
  PID_Create(&pid_b3, current_kp, current_ki, current_kd, CTRL_Loop_Period);
  PID_Create(&pid_b4, current_kp, current_ki, current_kd, CTRL_Loop_Period);

  //Motors
  Motor_Create(&B1, &htim3, TIM_CHANNEL_1, GPIOB, GPIO_PIN_15);
  Motor_Create(&B2, &htim3, TIM_CHANNEL_2, GPIOB, GPIO_PIN_13);
  Motor_Create(&B3, &htim3, TIM_CHANNEL_3, GPIOC, GPIO_PIN_9);
  Motor_Create(&B4, &htim3, TIM_CHANNEL_4, GPIOC, GPIO_PIN_5);

  //Motor encoders
  Encoder_Create(&E1, &htim1, ENCODERS_CPR);
  Encoder_Create(&E2, &htim2, ENCODERS_CPR);
  Encoder_Create(&E3, &htim4, ENCODERS_CPR);
  Encoder_Create(&E4, &htim8, ENCODERS_CPR);

  //Steppers
  Stepper_Create(&S1, &htim17, TIM_CHANNEL_1, GPIOB, GPIO_PIN_8, 0, 0, Stepper_Motor_Steps_Per_Rev * 5, 0, 1, 1, -96, -127, -94);	// -99
  Stepper_Create(&S2, &htim15, TIM_CHANNEL_1, GPIOA, GPIO_PIN_10, 0, 0, Stepper_Motor_Steps_Per_Rev * 5, 0, 1, 1, -93, -120, -86);	// -92
  Stepper_Create(&S3, &htim16, TIM_CHANNEL_1, GPIOB, GPIO_PIN_12, 0, 0, Stepper_Motor_Steps_Per_Rev * 5, 0, 1, 1, -92.5, -125, -90);	// -95
  Stepper_Create(&S4, &htim20, TIM_CHANNEL_1, GPIOC, GPIO_PIN_8, 0, 0, Stepper_Motor_Steps_Per_Rev * 5, 0, 1, 1, -91.5, -123, -90);	// -94

  initTimer(&S1);
  initTimer(&S2);
  initTimer(&S3);
  initTimer(&S4);

  // timer period callback

  uint32_t period_us = 1000000 / CTRL_Loop_Freq;
  //TIM6_SetPeriod_us((1.0f/CTRL_Loop_Freq) * 1000000);
  TIM6_SetPeriod_us(period_us);
  DWT_Init();

  //Stepper_Create(&stepper_handle_BL, &htim17, TIM_CHANNEL_1, STEP_BL_DIR_GPIO_Port,  STEP_BL_DIR_Pin, STEP_BL_ENA_GPIO_Port, STEP_BL_ENA_Pin, Stepper_Steps_Per_Rev);

  PID_SetOutputLimits(&pid_b1, -0.8f, 0.8f);
  PID_SetDeadbands(&pid_b1, 0.5f, 1.0f);
  PID_SetIntegralDecay(&pid_b1, 0.9f);
  PID_SetMaxSlewRate(&pid_b1, 10.0f);		// tune later

  PID_SetOutputLimits(&pid_b2, -0.8f, 0.8f);
  PID_SetDeadbands(&pid_b2, 0.5f, 1.0f);
  PID_SetIntegralDecay(&pid_b2, 0.9f);
  PID_SetMaxSlewRate(&pid_b2, 10.0f);		// tune later

  PID_SetOutputLimits(&pid_b3, -0.8f, 0.8f);
  PID_SetDeadbands(&pid_b3, 0.5f, 1.0f);
  PID_SetIntegralDecay(&pid_b3, 0.9f);
  PID_SetMaxSlewRate(&pid_b3, 10.0f);		// tune later

  PID_SetOutputLimits(&pid_b4, -0.8f, 0.8f);
  PID_SetDeadbands(&pid_b4, 0.5f, 1.0f);
  PID_SetIntegralDecay(&pid_b4, 0.9f);
  PID_SetMaxSlewRate(&pid_b4, 10.0f);		// tune later

  // back left arm setup
  Encoder_Start(&E1);
  Encoder_Start(&E2);
  Encoder_Start(&E3);
  Encoder_Start(&E4);  // Tim4 - Encoder
  //HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // B1
  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // B2
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // B3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  //HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); // B4
  HAL_TIM_Base_Start_IT(&htim6);	//scheduling interrupts

  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buf, rx_buf_size);	// for incoming ros commands
  HAL_UART_Receive_IT(&huart2, &bpill_rx_byte, 1);					// listen to black pill for mode instructions
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  S1.homing_status = 0;
  S2.homing_status = 0;
  S3.homing_status = 0;
  S4.homing_status = 0;
  while (1)
  {

	  // sending mode to shubh

//	  // software watchdog for bpill instructions (can't use rn because of keybaord teleop node
//	  if ((HAL_GetTick() - last_bpill_hearbeat) > 500){
//		  mode = MODE_ESTOP;
//		  estop_active = 1;
//	  }

	  if (mode == MODE_IDLE) mode = MODE_TELEOP; //forcing teleop instead of idle for now, will change when switches.

	  switch(mode){

	  case MODE_IDLE:
		  break;

	  case MODE_HOMING:

		  if(S1.correctOffset == 1){
			  moveAngle(&S1, -S1.limSwitchOffset, 30);
			  S1.correctOffset = 2;
		  }
		  if(S2.correctOffset == 1){
			  moveAngle(&S2, -S2.limSwitchOffset, 30);
			  S2.correctOffset = 2;
		  }
		  if(S3.correctOffset == 1){
			  moveAngle(&S3, -S3.limSwitchOffset, 30);
			  S3.correctOffset = 2;
		  }
		  if(S4.correctOffset == 1){
			  moveAngle(&S4, -S4.limSwitchOffset, 30);
			  S4.correctOffset = 2;
		  }

		  if(!S1.homing_status && !S1.isMoving && !S1.totalPulses){
			  moveAngle(&S1, -360, 10);
		  }
		  if(!S2.homing_status && !S2.isMoving && !S2.totalPulses){
		  			  moveAngle(&S2, -360, 10);
		  		  }
		  if(!S3.homing_status && !S3.isMoving && !S3.totalPulses){
		  			  moveAngle(&S3, -360, 10);
		  		  }
		  if(!S4.homing_status && !S4.isMoving && !S4.totalPulses){
		  			  moveAngle(&S4, -360, 10);
		  		  }
		  if(S1.homing_status && S2.homing_status && S3.homing_status && S4.homing_status) mode = MODE_IDLE;
		  break;

	  case MODE_AUTONAV:

	  case MODE_SELFDRIVE:

	  case MODE_TELEOP:

		  if (estop_active){
			  if (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_SET){
				  estop_active = 0;
				  estop_action_done = 0;		// for releasing locked steppers at zero on e-stop
			  }
		  }

		  if (uart_data_ready) {
			uart_data_ready = 0;
			if (!estop_active){
				//last_cmd_time = HAL_GetTick();
				handle_command(main_cmd_buf);
			}
		  }
		  // so that new parsing doesnt parse P.A.I.N (bas failsafe in case estop press ke baad bhi instructions aa rahe he)
		  // oh also - FUCK EMI

		  // E-stop provisions
		  if (estop_active) {
			  b1_target_rpm = b2_target_rpm = b3_target_rpm = b4_target_rpm = 0;
			  	// preempt normal movements so they cleanly stop and calculate exact absolute angles
				// we only do this if we haven't already started the return-to-zero sequence
				if (!estop_action_done) {
					if (S1.isMoving) S1.pending_preemption = 1;
					if (S2.isMoving) S2.pending_preemption = 1;
					if (S3.isMoving) S3.pending_preemption = 1;
					if (S4.isMoving) S4.pending_preemption = 1;

					// yaw-estop-yaw fails the motordriver.c check for lastInstruct because last is still yaw angle so gets skipped
					S1.lastInstruct.degree = 0;
					S2.lastInstruct.degree = 0;
					S3.lastInstruct.degree = 0;
					S4.lastInstruct.degree = 0;
				}

				// once steppers naturally halt and update their angles, trigger zeroing
				if (!S1.isMoving && !S2.isMoving && !S3.isMoving && !S4.isMoving) {
				  if (!estop_action_done) {
					  estop_action_done = 1;

					  if (S1.homing_status) moveAngleAbsolute(&S1, 0, 30, &B1);
					  if (S2.homing_status) moveAngleAbsolute(&S2, 0, 30, &B2);
					  if (S3.homing_status) moveAngleAbsolute(&S3, 0, 30, &B3);
					  if (S4.homing_status) moveAngleAbsolute(&S4, 0, 30, &B4);

					  initWQueue(&S1.q); initWQueue(&S2.q); initWQueue(&S3.q); initWQueue(&S4.q);
				  }
				}

		  }


		  // ! software watchdog to make sure continuous commands are received !

//		  if (HAL_GetTick() - last_cmd_time > Command_WDT_Max_Time){
//			  // software watchdog for teleop commands
//			  if (!wdt_active){
//
//				  wdt_active = 1;
//
//				  b1_target_rpm = b2_target_rpm = b3_target_rpm = b4_target_rpm = 0;
//				  if (S1.homing_status) moveAngleAbsolute(&S1, 0, 30, &B1);
//				  if (S2.homing_status) moveAngleAbsolute(&S2, 0, 30, &B2);
//				  if (S3.homing_status) moveAngleAbsolute(&S3, 0, 30, &B3);
//				  if (S4.homing_status) moveAngleAbsolute(&S4, 0, 30, &B4);
//				  initWQueue(&S1.q); initWQueue(&S2.q); initWQueue(&S3.q); initWQueue(&S4.q);
//			  }
//		  }

		  //if (!estop_active && !wdt_active){
		  if (!estop_active){
			  if(!S1.isMoving && !WisEmpty(&S1.q) && !S1.pending_preemption){
				  Wrapper temp = dequeueW(&S1.q);
				  moveAngleAbsolute(&S1, temp.degree, temp.rpm, &B1);
			  }
			  if(!S2.isMoving && !WisEmpty(&S2.q) && !S2.pending_preemption){
				  Wrapper temp = dequeueW(&S2.q);
				  moveAngleAbsolute(&S2, temp.degree, temp.rpm, &B2);
			  }
			  if(!S3.isMoving && !WisEmpty(&S3.q) && !S3.pending_preemption){
				  Wrapper temp = dequeueW(&S3.q);
				  moveAngleAbsolute(&S3, temp.degree, temp.rpm, &B3);
			  }
			  if(!S4.isMoving && !WisEmpty(&S4.q) && !S4.pending_preemption){
				  Wrapper temp = dequeueW(&S4.q);
				  moveAngleAbsolute(&S4, temp.degree, temp.rpm, &B4);
			  }
		  }

			if (control_loop){
				b1_current_rpm = Encoder_GetSpeedRPM(&E1);
				b1_control_signal = PID_Compute(&pid_b1, (float)B1.mode ? -b1_target_rpm : b1_target_rpm, b1_current_rpm);
				//b1_control_signal = map_rpm_to_signal((float)b1_target_rpm);
				Motor_SetOutput(&B1, b1_control_signal);
				b2_current_rpm = Encoder_GetSpeedRPM(&E2);
				b2_control_signal = PID_Compute(&pid_b2, (float)B2.mode ? b2_target_rpm : -b2_target_rpm, b2_current_rpm);
				Motor_SetOutput(&B2, b2_control_signal);
				b3_current_rpm = Encoder_GetSpeedRPM(&E3);
				b3_control_signal = PID_Compute(&pid_b3, (float)B3.mode ? -b3_target_rpm : b3_target_rpm, b3_current_rpm);
				Motor_SetOutput(&B3, b3_control_signal);
				b4_current_rpm = Encoder_GetSpeedRPM(&E4);
				b4_control_signal = PID_Compute(&pid_b4, (float)B4.mode ? -b4_target_rpm : b4_target_rpm, b4_current_rpm);
				Motor_SetOutput(&B4, b4_control_signal);

				if (HAL_GetTick() - lastTransmissionTime >= 1000/feedback_transmission_freq){
					 if (last_sent_mode != mode){
						  bpill_tx_buf[0] = 0xAA;
						  bpill_tx_buf[1] = (uint8_t)mode;
						  bpill_tx_buf[2] = bpill_tx_buf[0] ^ bpill_tx_buf[1];
						  HAL_UART_Transmit(&huart5, bpill_tx_buf, 3, 10);
						  sending_mode++;		// remove after testing
						  last_sent_mode = mode;
					 }
					int tel_len = snprintf((char*)feedback_buf, feedback_buf_size,
										   "@S1%ld;S2%ld;S3%ld;S4%ld;B1%.4f;B2%.4f;B3%.4f;B4%.4f;\r\n",
										   S1.abs_step_count, S2.abs_step_count, S3.abs_step_count, S4.abs_step_count,
										   Encoder_GetSpeedRPM(&E1), Encoder_GetSpeedRPM(&E2), Encoder_GetSpeedRPM(&E3), Encoder_GetSpeedRPM(&E4));
					lastTransmissionTime += 1000/feedback_transmission_freq;
					HAL_UART_Transmit(&huart5, feedback_buf, tel_len, 10);
				}
				control_loop = 0;
			}
		  break;

	  default:
		  break;
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }


  Encoder_free(&E1);
  Encoder_free(&E2);
  Encoder_free(&E3);
  Encoder_free(&E4);
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 169;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 125;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 3;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim15, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 125;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 3;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim16, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 125;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 3;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim17, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

}

/**
  * @brief TIM20 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM20_Init(void)
{

  /* USER CODE BEGIN TIM20_Init 0 */

  /* USER CODE END TIM20_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM20_Init 1 */

  /* USER CODE END TIM20_Init 1 */
  htim20.Instance = TIM20;
  htim20.Init.Prescaler = 125;
  htim20.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim20.Init.Period = 3;
  htim20.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim20.Init.RepetitionCounter = 0;
  htim20.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim20) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim20, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim20, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim20, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim20, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM20_Init 2 */

  /* USER CODE END TIM20_Init 2 */
  HAL_TIM_MspPostInit(&htim20);

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, S1_ENA_Pin|S2_ENA_Pin|B4_DIR_Pin|S4_DIR_Pin
                          |B3_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, S4_ENA_Pin|S2_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, S3_DIR_Pin|B2_DIR_Pin|B1_DIR_Pin|S3_ENA_Pin
                          |S1_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : S1_LIM_Pin S4_LIM_Pin S3_LIM_Pin */
  GPIO_InitStruct.Pin = S1_LIM_Pin|S4_LIM_Pin|S3_LIM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : S1_ENA_Pin S2_ENA_Pin B4_DIR_Pin S4_DIR_Pin
                           B3_DIR_Pin */
  GPIO_InitStruct.Pin = S1_ENA_Pin|S2_ENA_Pin|B4_DIR_Pin|S4_DIR_Pin
                          |B3_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : S4_ENA_Pin S2_DIR_Pin */
  GPIO_InitStruct.Pin = S4_ENA_Pin|S2_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : S3_DIR_Pin B2_DIR_Pin B1_DIR_Pin S3_ENA_Pin
                           S1_DIR_Pin */
  GPIO_InitStruct.Pin = S3_DIR_Pin|B2_DIR_Pin|B1_DIR_Pin|S3_ENA_Pin
                          |S1_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ESTOP_Pin S2_LIM_Pin */
  GPIO_InitStruct.Pin = ESTOP_Pin|S2_LIM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	// check timer for interrupt
	if(htim->Instance == TIM6){
		control_loop = 1;
		return;
	}
	Stepper_Handle_t *s = NULL;

	/* Identify stepper */
	if (htim->Instance == S1.step_timer->Instance) s = &S1;
	else if (htim->Instance == S2.step_timer->Instance) s = &S2;
	else if (htim->Instance == S3.step_timer->Instance) s = &S3;
	else if (htim->Instance == S4.step_timer->Instance) s = &S4;
	else return;

	/* If not moving, nothing to do */
	if (!s->isMoving)
	    return;

	/* A burst has completed */
	uint32_t completed = s->last_burst_size;

	s->totalPulses += completed;
	s->abs_step_count += s->dir * completed;

	/* Preemption check */
	if (s->pending_preemption)
	{
	    s->target_steps = 0;
	    Stepper_Stop(s);
	    s->pending_preemption = 0;
	    return;
	}

	/* More steps remaining? */
	/* More steps remaining? */
	if (s->target_steps > completed)
	{
	    s->target_steps -= completed;

	    uint32_t next_chunk = (s->target_steps > BURST_SIZE)
	                        ? BURST_SIZE : s->target_steps;
	    s->last_burst_size = next_chunk;

	    // Timer is already stopped at this point (fired its last pulse)
	    // but be explicit
	    __HAL_TIM_DISABLE(s->step_timer);
	    __HAL_TIM_DISABLE_IT(s->step_timer, TIM_IT_UPDATE);

	    s->step_timer->Instance->RCR = next_chunk - 1;
	    __HAL_TIM_SET_COUNTER(s->step_timer, 0);

	    s->step_timer->Instance->EGR = TIM_EGR_UG;
	    __HAL_TIM_CLEAR_FLAG(s->step_timer, TIM_FLAG_UPDATE);  // clear AFTER EGR

	    // untested for estop:
	    // If the EXTI E-stop fired while we were doing the math above, abort!
		if (!s->isMoving) return;

	    __HAL_TIM_ENABLE_IT(s->step_timer, TIM_IT_UPDATE);
	    __HAL_TIM_ENABLE(s->step_timer);
	}
	else
	{
	    s->target_steps = 0;
	    if (s->correctOffset == 2) {
	    	s->homing_status = 1;
	    	s->abs_step_count = 0;
		    s->correctOffset = 0;
	    }
	    //s->totalPulses = 0; //optional
	    Stepper_Stop(s);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART5)
    {
    	static uint16_t old_pos = 0;
		static uint16_t shadow_idx = 0;
		uint16_t len = 0;

		// copy chunk from DMA buffer into accumulator
		if (Size > old_pos) {			// linear buffer
			len = Size - old_pos;
			memcpy(&shadow_rx_buf[shadow_idx], &rx_buf[old_pos], len);
		}
		else if (Size < old_pos) {		// wrap-around in buffer
			len = rx_buf_size - old_pos;
			memcpy(&shadow_rx_buf[shadow_idx], &rx_buf[old_pos], len);
			memcpy(&shadow_rx_buf[shadow_idx + len], rx_buf, Size);
			len += Size;
		}

		shadow_idx += len;
		shadow_rx_buf[shadow_idx] = '\0'; // seal string
		old_pos = Size;

		// last semi-colon in string
		char *last_semi = strrchr((char*)shadow_rx_buf, ';');
		if (last_semi != NULL) {
			uint16_t split_index = (last_semi - (char*)shadow_rx_buf) + 1;
			uint16_t leftover_len = shadow_idx - split_index;
			memcpy(main_cmd_buf, shadow_rx_buf, split_index);			// copy complete command into main loop's buffer
			main_cmd_buf[split_index] = '\0';
			uart_data_ready = 1; // flag main loop

			// save the incomplete fragment ("B3 5" instead of "B3 50") for the next interrupt
			char temp_leftover[64];
			if (leftover_len > 0) {
				memcpy(temp_leftover, &shadow_rx_buf[split_index], leftover_len);
				memcpy(shadow_rx_buf, temp_leftover, leftover_len);
			}
			shadow_idx = leftover_len;
			shadow_rx_buf[shadow_idx] = '\0';
		}
		else if (shadow_idx >= sizeof(shadow_rx_buf) - 10)
			shadow_idx = 0;										// full buffer but no semicolons
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
        {

    	switch (bpill_rx_state) {
			case 0:		// look for header
				if (bpill_rx_byte == 0xAA) {
					bpill_rx_buf[0] = bpill_rx_byte;
					bpill_rx_state = 1;
				}
				break;

			case 1:		// get which mode bot is in
				bpill_rx_buf[1] = bpill_rx_byte;
				bpill_rx_state = 2;
				break;

			case 2:		// verify checksum
				bpill_rx_buf[2] = bpill_rx_byte;
				bpill_rx_state = 0;
				// get actual checksum value
				uint8_t expected_checksum = bpill_rx_buf[0] ^ bpill_rx_buf[1];
				if (bpill_rx_buf[2] == expected_checksum) {
					if (mode == MODE_HOMING && !(S1.homing_status && S2.homing_status && S3.homing_status && S4.homing_status)){
						mode = MODE_HOMING;
						break;
					}
					prev_rec_mode = rec_mode;
					rec_mode = bpill_rx_buf[1];					// apply mode if checksum is correct
					if (rec_mode == MODE_HOMING && prev_rec_mode != MODE_HOMING){
						Stepper_Stop(&S1); Stepper_Stop(&S2);
						Stepper_Stop(&S3); Stepper_Stop(&S4);
						S1.totalPulses = 0;
						S2.totalPulses = 0;
						S3.totalPulses = 0;
						S4.totalPulses = 0;
						S1.homing_status = 0;
						S2.homing_status = 0;
						S3.homing_status = 0;
						S4.homing_status = 0;
						S1.correctOffset = 0;
						S2.correctOffset = 0;
						S3.correctOffset = 0;
						S4.correctOffset = 0;
					}
				}
				mode = rec_mode;
				break;
		}
			// re-arm interrupt for next byte
			HAL_UART_Receive_IT(huart, &bpill_rx_byte, 1);
        }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == UART5){

		error_entered++;					// for debugging
		last_error = huart->ErrorCode;		// check huart5.Instance->ISR in live expressions, might give more info

		// clear flags at error
		__HAL_UART_CLEAR_OREFLAG(huart);
		__HAL_UART_CLEAR_NEFLAG(huart);
		__HAL_UART_CLEAR_FEFLAG(huart);

		// restart UART DMA
		HAL_UART_AbortReceive(huart);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buf, rx_buf_size);
	}

	else if (huart->Instance == USART2){
		// clear flags at error for the bpill link
		uarterror++;
		uarterror3_error = huart->ErrorCode;
		__HAL_UART_CLEAR_OREFLAG(huart);
		__HAL_UART_CLEAR_NEFLAG(huart);
		__HAL_UART_CLEAR_FEFLAG(huart);

		// restart the 1-byte interrupt listener
		HAL_UART_AbortReceive(huart);
		HAL_UART_Receive_IT(&huart2, &bpill_rx_byte, 1);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if (GPIO_Pin == ESTOP_Pin){
		// noise filter: kyuki EMI = BT
		if (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_RESET) {
			estop_active = 1;
			b1_target_rpm = b2_target_rpm = b3_target_rpm = b4_target_rpm = 0;
			PID_Reset(&pid_b1); PID_Reset(&pid_b2); PID_Reset(&pid_b3); PID_Reset(&pid_b4);

			// following is removed because pressing estop mid-stepper rotation caused timer issues causing offset:

//			Stepper_Stop(&S1); Stepper_Stop(&S2); Stepper_Stop(&S3); Stepper_Stop(&S4);
//			initWQueue(&S1.q); initWQueue(&S2.q); initWQueue(&S3.q); initWQueue(&S4.q);
		}
	}

	current_debounce_time = HAL_GetTick();

	if (GPIO_Pin == S1_LIM_Pin){ 						// stepper 1
		if (S1.homing_status || S1.correctOffset == 2) return; //if already homed, do nothing
		if (current_debounce_time - last_debounce_time[0] > DEBOUNCE_TIME_PERIOD){
			last_debounce_time[0] = current_debounce_time;
			Stepper_Stop(&S1);
			S1.correctOffset = 1;
		}
	}
	if (GPIO_Pin == S2_LIM_Pin){						// stepper 2
		if (S2.homing_status || S2.correctOffset == 2) return; //if already homed, do nothing
		if (current_debounce_time - last_debounce_time[1] > DEBOUNCE_TIME_PERIOD){
			last_debounce_time[1] = current_debounce_time;
			Stepper_Stop(&S2);
			S2.correctOffset = 1;

		}
	}
	if (GPIO_Pin == S3_LIM_Pin){						// stepper 3
		if (S3.homing_status || S3.correctOffset == 2) return;//if already homed, do nothing
		if (current_debounce_time - last_debounce_time[2] > DEBOUNCE_TIME_PERIOD){
			last_debounce_time[2] = current_debounce_time;
			Stepper_Stop(&S3);
			S3.correctOffset = 1;
		}
	}
	if (GPIO_Pin == S4_LIM_Pin){						// stepper 4
		if (S4.homing_status || S4.correctOffset == 2) return; //if already homed, do nothing
		if (current_debounce_time - last_debounce_time[3] > DEBOUNCE_TIME_PERIOD){
			last_debounce_time[3] = current_debounce_time;
			Stepper_Stop(&S4);
			S4.correctOffset = 1;
		}
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
