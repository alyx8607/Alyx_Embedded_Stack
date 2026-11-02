/*
 * telemetry.h
 *
 * Telemetry v2: all four wheels, for the fake-ALYX validation session
 * ("Is Looking Worth Turning?", docs/telemetry_v2.md in the research repo).
 *
 * One CSV row per control cycle (100 Hz) over UART5 by DMA, integers only,
 * CRC-8 per row. The control loop calls tel_push4() (cheap: one struct into a
 * ring buffer); the main loop calls tel_service() (formats rows and starts the
 * DMA). Every row carries seq, so lost rows show up as gaps on the host.
 *
 * Row: #v2,seq,t_us,tick_ms,loop_dt_us,svc_lat_us,mode,estop,scn,dropped,frm,frm_age_us,
 *      enc1,rpm1,tgt1,out1,step1,cmd1,fl1,hsw1, ... ,hsw4*CC
 * frm: number of the last command frame processed (host tag "#F <n>;"),
 * frm_age_us: how long before this row it was processed -> exact apply time.
 *
 * Author: Akshat P. Kakade
 */

#ifndef INC_TELEMETRY_H_
#define INC_TELEMETRY_H_

#include <stdint.h>
#include "stm32g4xx_hal.h"

#define TEL_ENABLE          1
#define TEL_RING_SIZE       128     /* rows buffered (~120 B each)              */
#define TEL_MAX_PER_PASS    4       /* rows formatted per main-loop pass         */
#define TEL_TXBUF_SIZE      1536
#define TEL_ROW_MAX         320     /* longest possible row incl. CRC and CRLF   */
#define TEL_HEADER_EVERY    500     /* resend the header every N rows (5 s)      */

/* fl<i> bits */
#define TEL_FL_MODE         0x01    /* B<i>.mode (1 = forward)                   */
#define TEL_FL_MOVING       0x02    /* S<i>.isMoving                             */
#define TEL_FL_PREEMPT      0x04    /* S<i>.pending_preemption                   */
#define TEL_FL_HOMED        0x08    /* S<i>.homing_status                        */
#define TEL_FL_OFFSET       0x10    /* S<i>.correctOffset != 0                   */

typedef struct {
    uint16_t enc_raw;       /* hardware encoder counter (E<i>.last_count)        */
    float    rpm_filt;      /* b<i>_current_rpm, raw wiring sign                 */
    int16_t  target_rpm;    /* b<i>_target_rpm, ROS convention                   */
    float    out;           /* b<i>_control_signal                               */
    int32_t  abs_step;      /* S<i>.abs_step_count                               */
    int16_t  cmd_deg;       /* (int)S<i>.lastInstruct.degree                     */
    uint8_t  flags;         /* TEL_FL_* bits                                     */
} tel_wheel_t;

void tel_init(void);
void tel_reset(void);                       /* restart seq and resend the header */
void tel_mark_isr(void);                    /* TIM6 ISR                          */
void tel_push4(const tel_wheel_t w[4], uint8_t mode, uint8_t estop);
void tel_service(UART_HandleTypeDef *huart);
void tel_tx_done(void);                     /* HAL_UART_TxCpltCallback (UART5)   */
void tel_set_scenario(uint8_t s);           /* host "#SCN <n>;"                  */
void tel_note_frame(uint16_t n);            /* host "#F <n>;", when processed    */

/* Homing timing, for the missed-step check: microseconds from the start of a
 * module's -360 deg homing sweep to its limit switch. idx 0..3. */
void tel_homing_sweep_start(uint8_t idx);   /* main loop, when the sweep starts  */
void tel_homing_switch(uint8_t idx);        /* EXTI, when the switch is accepted */

#endif /* INC_TELEMETRY_H_ */
