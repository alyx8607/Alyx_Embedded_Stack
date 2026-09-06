/*
 * telemetry.h
 *
 * Bench-test telemetry for ALYX drive controller (RCAAI 2026 experiments).
 *
 * Purpose: capture per-control-cycle data over UART5 for offline analysis,
 * without disturbing control-loop timing.
 *
 * Design notes:
 *  - The control loop calls tel_push(), which is cheap: it writes one packed
 *    struct into a ring buffer. No snprintf, no UART access, no floats
 *    beyond a couple of divides.
 *  - The main loop calls tel_service(), which formats records into CSV and
 *    ships them out by DMA. Formatting cost stays out of the control path.
 *  - Every record carries a sequence number. If the link ever falls behind
 *    and the ring overflows, the sequence numbers show a gap and the
 *    analysis script reports exactly how many samples were lost. Silent
 *    data loss is the thing that quietly ruins bench results, so we make
 *    it loud instead.
 *
 * Author: Akshat P. Kakade
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include <stdint.h>
#include "stm32g4xx_hal.h"

/* ---- build-time switches -------------------------------------------------
 * Set TEL_ENABLE to 0 to compile the whole module out for competition builds.
 */
#define TEL_ENABLE        1
#define TEL_RING_SIZE     256      /* records buffered; 256 * ~44 B = ~11 kB */
#define TEL_MAX_PER_PASS  8        /* records formatted per main-loop pass   */
#define TEL_TXBUF_SIZE    1024

/* Nominal control period in microseconds, used for the fixed-interval
 * (legacy) velocity estimate. 100 Hz -> 10000 us. */
#define TEL_NOMINAL_DT_US 10000.0f

typedef struct {
    uint32_t seq;            /* monotonic sample counter                     */
    uint32_t t_us;           /* timestamp, microseconds since boot           */
    uint32_t loop_dt_us;     /* time since previous control-loop service     */
    uint32_t svc_lat_us;     /* TIM6 ISR -> control-loop service latency     */
    uint32_t dt_enc_us;      /* elapsed time used by the DWT estimator       */
    uint32_t dt_tick_ms;     /* elapsed time the shipping estimator saw      */
    uint16_t raw_cnt;        /* raw quadrature counter value                 */
    int16_t  delta;          /* wraparound-corrected count delta             */
    float    rpm_fixed;      /* estimate assuming a fixed 10 ms interval     */
    float    rpm_tick;       /* estimate from HAL_GetTick dt (as shipped)    */
    float    rpm_dwt;        /* estimate from DWT dt (microsecond resolution)*/
    float    rpm_filt;       /* filtered value the controller actually used  */
    float    setpoint;
    float    output;
    float    iterm;
    int32_t  abs_step;       /* steering module absolute step count          */
    uint32_t total_pulses;   /* pulses since last homing reset              */
    uint8_t  hstat;          /* homing_status | correctOffset<<1            */
    uint8_t  scn;            /* scenario tag set by the host                 */
} tel_record_t;

/* Call once after DWT_Init(), before the main loop. */
void tel_init(float counts_per_rev);

/* Call from HAL_TIM_PeriodElapsedCallback when TIM6 fires. Cheap. */
void tel_mark_isr(void);

/* Call from inside the control loop, once per cycle, per logged wheel.
 * Pass the raw counter and the wraparound-corrected delta straight from
 * the encoder module so the log reflects exactly what the estimator saw. */
void tel_push(uint16_t raw_cnt, int16_t delta, uint32_t dt_tick_ms,
              float rpm_tick, float rpm_filt,
              float setpoint, float output, float iterm,
              int32_t abs_step, uint32_t total_pulses, uint8_t hstat);

/* Call from the main loop as often as convenient. Non-blocking. */
void tel_service(UART_HandleTypeDef *huart);

/* Called from HAL_UART_TxCpltCallback. */
void tel_tx_done(void);

/* Host sets a scenario tag (0-255) so trials are separable offline. */
void tel_set_scenario(uint8_t s);

/* Emits a CSV header line and resets counters. Called on host request. */
void tel_reset(void);

/* Number of records dropped because the ring was full. */
uint32_t tel_dropped(void);

#endif /* INC_TELEMETRY_H_ */
