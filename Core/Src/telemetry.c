/*
 * telemetry.c
 *
 * See telemetry.h for the design rationale.
 *
 * Author: Akshat P. Kakade
 */

#include "telemetry.h"
#include <stdio.h>
#include <string.h>

#if TEL_ENABLE

/* ---- 64-bit microsecond clock built on DWT->CYCCNT ------------------------
 * CYCCNT is 32-bit and at 170 MHz it wraps roughly every 25 seconds. We
 * accumulate the difference into a wider counter. Safe as long as this is
 * called more often than once per 25 s, which at 100 Hz it comfortably is.
 */
static volatile uint32_t s_last_cyc = 0;
static volatile uint64_t s_us_accum = 0;
static uint32_t s_cyc_per_us = 170;

static uint32_t tel_now_us(void)
{
    uint32_t now = DWT->CYCCNT;
    uint32_t d   = now - s_last_cyc;      /* unsigned wrap is well defined */
    s_last_cyc   = now;
    s_us_accum  += (uint64_t)(d / s_cyc_per_us);
    return (uint32_t)s_us_accum;
}

/* ---- state ---------------------------------------------------------------*/
static tel_record_t s_ring[TEL_RING_SIZE];
static volatile uint16_t s_head = 0;      /* written by control loop  */
static volatile uint16_t s_tail = 0;      /* read by main loop        */
static volatile uint32_t s_seq = 0;
static volatile uint32_t s_dropped = 0;

static volatile uint32_t s_isr_us = 0;    /* stamped in the TIM6 ISR  */
static uint32_t s_last_service_us = 0;

static float s_cpr = 1993.0f;
static volatile uint8_t s_scn = 0;

static uint8_t  s_txbuf[TEL_TXBUF_SIZE];
static volatile uint8_t s_tx_busy = 0;
static uint8_t  s_want_header = 1;

void tel_init(float counts_per_rev)
{
    s_cpr = (counts_per_rev > 1.0f) ? counts_per_rev : 1.0f;
    s_cyc_per_us = SystemCoreClock / 1000000u;
    if (s_cyc_per_us == 0) s_cyc_per_us = 1;

    s_last_cyc = DWT->CYCCNT;
    s_us_accum = 0;
    s_head = s_tail = 0;
    s_seq = 0;
    s_dropped = 0;
    s_last_service_us = 0;
    s_tx_busy = 0;
    s_want_header = 1;
}

void tel_reset(void)
{
    s_head = s_tail = 0;
    s_seq = 0;
    s_dropped = 0;
    s_want_header = 1;
}

void tel_set_scenario(uint8_t s) { s_scn = s; }

uint32_t tel_dropped(void) { return s_dropped; }

void tel_mark_isr(void)
{
    s_isr_us = tel_now_us();
}

void tel_tx_done(void) { s_tx_busy = 0; }

void tel_push(uint16_t raw_cnt, int16_t delta, uint32_t dt_tick_ms,
              float rpm_tick, float rpm_filt,
              float setpoint, float output, float iterm,
              int32_t abs_step, uint32_t total_pulses, uint8_t hstat)
{
    uint32_t now = tel_now_us();

    uint16_t next = (uint16_t)((s_head + 1u) % TEL_RING_SIZE);
    if (next == s_tail) {          /* ring full: drop, but count it */
        s_dropped++;
        s_last_service_us = now;
        return;
    }

    tel_record_t *r = &s_ring[s_head];

    r->seq        = s_seq++;
    r->t_us       = now;
    r->loop_dt_us = (s_last_service_us == 0) ? 0u : (now - s_last_service_us);
    r->svc_lat_us = (now >= s_isr_us) ? (now - s_isr_us) : 0u;
    r->dt_enc_us  = r->loop_dt_us ? r->loop_dt_us : 1u;
    r->dt_tick_ms = dt_tick_ms;

    r->raw_cnt = raw_cnt;
    r->delta   = delta;

    /* Three estimates from the SAME count delta, same instant.
     * This is what makes the comparison free of confounds. */
    r->rpm_fixed = ((float)delta * 60.0f * 1000000.0f / TEL_NOMINAL_DT_US) / s_cpr;
    r->rpm_dwt   = ((float)delta * 60.0f * 1000000.0f / (float)r->dt_enc_us) / s_cpr;
    r->rpm_tick  = rpm_tick;

    r->rpm_filt  = rpm_filt;
    r->setpoint  = setpoint;
    r->output    = output;
    r->iterm     = iterm;
    r->abs_step     = abs_step;
    r->total_pulses = total_pulses;
    r->hstat        = hstat;
    r->scn          = s_scn;

    s_head = next;
    s_last_service_us = now;
}

void tel_service(UART_HandleTypeDef *huart)
{
    if (s_tx_busy) return;
    if (s_head == s_tail && !s_want_header) return;

    int n = 0;

    if (s_want_header) {
        n += snprintf((char*)s_txbuf + n, TEL_TXBUF_SIZE - n,
            "#seq,t_us,loop_dt_us,svc_lat_us,dt_enc_us,dt_tick_ms,"
            "raw,delta,rpm_fixed,rpm_tick,rpm_dwt,rpm_filt,"
            "sp,out,iterm,absstep,totpulse,hstat,scn,dropped\r\n");
        s_want_header = 0;
    }

    int emitted = 0;
    while (s_tail != s_head && emitted < TEL_MAX_PER_PASS
           && n < (TEL_TXBUF_SIZE - 140)) {

        tel_record_t *r = &s_ring[s_tail];

        n += snprintf((char*)s_txbuf + n, TEL_TXBUF_SIZE - n,
            "%lu,%lu,%lu,%lu,%lu,%lu,%u,%d,"
            "%.3f,%.3f,%.3f,%.3f,%.3f,%.5f,%.5f,%ld,%lu,%u,%u,%lu\r\n",
            (unsigned long)r->seq, (unsigned long)r->t_us,
            (unsigned long)r->loop_dt_us, (unsigned long)r->svc_lat_us,
            (unsigned long)r->dt_enc_us, (unsigned long)r->dt_tick_ms,
            (unsigned)r->raw_cnt, (int)r->delta,
            r->rpm_fixed, r->rpm_tick, r->rpm_dwt, r->rpm_filt,
            r->setpoint, r->output, r->iterm,
            (long)r->abs_step, (unsigned long)r->total_pulses,
            (unsigned)r->hstat, (unsigned)r->scn,
            (unsigned long)s_dropped);

        s_tail = (uint16_t)((s_tail + 1u) % TEL_RING_SIZE);
        emitted++;
    }

    if (n > 0) {
        if (HAL_UART_Transmit_DMA(huart, s_txbuf, (uint16_t)n) == HAL_OK) {
            s_tx_busy = 1;
        }
    }
}

#else  /* TEL_ENABLE == 0 : compile to nothing */

void tel_init(float c) { (void)c; }
void tel_reset(void) {}
void tel_set_scenario(uint8_t s) { (void)s; }
uint32_t tel_dropped(void) { return 0; }
void tel_mark_isr(void) {}
void tel_tx_done(void) {}
void tel_push(uint16_t a, int16_t b, uint32_t c, float d, float e,
              float f, float g, float h, int32_t i, uint32_t j, uint8_t k)
{ (void)a;(void)b;(void)c;(void)d;(void)e;(void)f;(void)g;(void)h;(void)i;
  (void)j;(void)k; }
void tel_service(UART_HandleTypeDef *h) { (void)h; }

#endif
