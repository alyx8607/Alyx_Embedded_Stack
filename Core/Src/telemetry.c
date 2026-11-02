/*
 * telemetry.c
 *
 * See telemetry.h. Integers only (no %f), CRC-8 per row, header resent every
 * TEL_HEADER_EVERY rows so a host that connects late still gets column names.
 *
 * Author: Akshat P. Kakade
 */

#include "telemetry.h"
#include <stdio.h>
#include <string.h>

#if TEL_ENABLE

/* ---- clock ---------------------------------------------------------------
 * t_us is a 64-bit microsecond count built on DWT->CYCCNT (wraps every ~25 s
 * at 170 MHz), printed as u32 (wraps after ~71 min; the host unwraps it).
 * tel_now_us() is only called from the main loop. ISRs store raw CYCCNT.
 */
static uint32_t s_last_cyc = 0;
static uint64_t s_us_accum = 0;
static uint32_t s_cyc_rem = 0;
static uint32_t s_cyc_per_us = 170;

static uint32_t tel_now_us(void)
{
    uint32_t now = DWT->CYCCNT;
    uint32_t d = now - s_last_cyc + s_cyc_rem;
    s_last_cyc = now;
    s_us_accum += d / s_cyc_per_us;
    s_cyc_rem = d % s_cyc_per_us;
    return (uint32_t)s_us_accum;
}

/* ---- ring ----------------------------------------------------------------*/
typedef struct {
    int32_t  enc;
    int32_t  rpm10;
    int16_t  tgt;
    int16_t  out1e4;
    int32_t  step;
    int16_t  cmd;
    uint8_t  fl;
    uint32_t hsw;
} tel_wrec_t;

typedef struct {
    uint32_t seq;
    uint32_t t_us;
    uint32_t tick_ms;
    uint32_t loop_dt_us;
    uint32_t svc_lat_us;
    uint8_t  mode;
    uint8_t  estop;
    uint8_t  scn;
    uint16_t frm;
    uint32_t frm_age_us;
    tel_wrec_t w[4];
} tel_row_t;

static tel_row_t s_ring[TEL_RING_SIZE];
static volatile uint16_t s_head = 0;        /* written by the control loop */
static volatile uint16_t s_tail = 0;        /* read by tel_service         */
static uint32_t s_seq = 0;
static volatile uint32_t s_dropped = 0;
static uint32_t s_last_push_us = 0;
static volatile uint32_t s_isr_cyc = 0;
static volatile uint8_t s_scn = 0;
static uint16_t s_frm = 0;
static uint32_t s_frm_cyc = 0;
static uint8_t  s_frm_seen = 0;

static int32_t  s_enc_acc[4];
static uint16_t s_enc_prev[4];
static uint8_t  s_enc_primed = 0;

static volatile uint32_t s_sweep_cyc[4];
static volatile uint32_t s_hsw_us[4];

static uint8_t  s_txbuf[TEL_TXBUF_SIZE];
static volatile uint8_t s_tx_busy = 0;
static uint32_t s_tx_start_ms = 0;
static uint8_t  s_want_header = 1;
static uint32_t s_rows_since_header = 0;

static const char s_header[] =
    "#v2,seq,t_us,tick_ms,loop_dt_us,svc_lat_us,mode,estop,scn,dropped,frm,frm_age_us,"
    "enc1,rpm1,tgt1,out1,step1,cmd1,fl1,hsw1,"
    "enc2,rpm2,tgt2,out2,step2,cmd2,fl2,hsw2,"
    "enc3,rpm3,tgt3,out3,step3,cmd3,fl3,hsw3,"
    "enc4,rpm4,tgt4,out4,step4,cmd4,fl4,hsw4\r\n";

static uint8_t crc8(const uint8_t *p, int n)
{
    uint8_t c = 0;
    while (n--) {
        c ^= *p++;
        for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
    }
    return c;
}

static int32_t round_i32(float x)
{
    return (int32_t)(x + (x >= 0.0f ? 0.5f : -0.5f));
}

void tel_init(void)
{
    s_cyc_per_us = SystemCoreClock / 1000000u;
    if (s_cyc_per_us == 0) s_cyc_per_us = 1;
    s_last_cyc = DWT->CYCCNT;
    s_us_accum = 0;
    s_cyc_rem = 0;
    s_enc_primed = 0;
    for (int i = 0; i < 4; i++) { s_enc_acc[i] = 0; s_sweep_cyc[i] = 0; s_hsw_us[i] = 0; }
    s_tx_busy = 0;
    tel_reset();
}

void tel_reset(void)
{
    s_head = s_tail = 0;
    s_seq = 0;
    s_dropped = 0;
    s_last_push_us = 0;
    s_want_header = 1;
}

void tel_set_scenario(uint8_t s) { s_scn = s; }

void tel_note_frame(uint16_t n) { s_frm = n; s_frm_cyc = DWT->CYCCNT; s_frm_seen = 1; }

void tel_mark_isr(void) { s_isr_cyc = DWT->CYCCNT; }

void tel_tx_done(void) { s_tx_busy = 0; }

void tel_homing_sweep_start(uint8_t idx)
{
    if (idx < 4) { s_sweep_cyc[idx] = DWT->CYCCNT; s_hsw_us[idx] = 0; }
}

void tel_homing_switch(uint8_t idx)
{
    if (idx < 4 && s_sweep_cyc[idx] != 0)
        s_hsw_us[idx] = (DWT->CYCCNT - s_sweep_cyc[idx]) / s_cyc_per_us;
}

void tel_push4(const tel_wheel_t w[4], uint8_t mode, uint8_t estop)
{
    uint32_t now = tel_now_us();
    uint32_t isr = s_isr_cyc;

    /* encoder: accumulate signed 16-bit deltas, even if the row is dropped */
    for (int i = 0; i < 4; i++) {
        if (s_enc_primed) s_enc_acc[i] += (int16_t)(w[i].enc_raw - s_enc_prev[i]);
        s_enc_prev[i] = w[i].enc_raw;
    }
    s_enc_primed = 1;

    uint16_t next = (uint16_t)((s_head + 1u) % TEL_RING_SIZE);
    if (next == s_tail) {
        s_dropped++;
        s_seq++;                    /* a dropped row is a seq gap on the host */
        s_last_push_us = now;
        return;
    }

    tel_row_t *r = &s_ring[s_head];
    r->seq = s_seq++;
    r->t_us = now;
    r->tick_ms = HAL_GetTick();
    r->loop_dt_us = (s_last_push_us == 0) ? 0u : (now - s_last_push_us);
    r->svc_lat_us = (DWT->CYCCNT - isr) / s_cyc_per_us;
    r->mode = mode;
    r->estop = estop;
    r->scn = s_scn;
    r->frm = s_frm;
    r->frm_age_us = s_frm_seen ? (DWT->CYCCNT - s_frm_cyc) / s_cyc_per_us : 0xFFFFFFFFu;
    for (int i = 0; i < 4; i++) {
        tel_wrec_t *o = &r->w[i];
        o->enc = s_enc_acc[i];
        o->rpm10 = round_i32(w[i].rpm_filt * 10.0f);
        o->tgt = w[i].target_rpm;
        o->out1e4 = (int16_t)round_i32(w[i].out * 10000.0f);
        o->step = w[i].abs_step;
        o->cmd = w[i].cmd_deg;
        o->fl = w[i].flags;
        o->hsw = s_hsw_us[i];
    }

    s_head = next;
    s_last_push_us = now;
}

void tel_service(UART_HandleTypeDef *huart)
{
    if (s_tx_busy) {
        /* recover if a TX complete was lost (UART error mid-DMA) */
        if (HAL_GetTick() - s_tx_start_ms > 100 && huart->gState == HAL_UART_STATE_READY)
            s_tx_busy = 0;
        else
            return;
    }
    if (s_head == s_tail && !s_want_header) return;

    /* rows leave the ring only once the DMA has started (tail committed below) */
    int n = 0;
    uint8_t header = (s_want_header || s_rows_since_header >= TEL_HEADER_EVERY);
    if (header) {
        memcpy(s_txbuf, s_header, sizeof(s_header) - 1);
        n = sizeof(s_header) - 1;
    }

    uint16_t tail = s_tail;
    int emitted = 0;
    while (tail != s_head && emitted < TEL_MAX_PER_PASS && n < TEL_TXBUF_SIZE - TEL_ROW_MAX) {
        const tel_row_t *r = &s_ring[tail];
        int start = n;
        n += snprintf((char *)s_txbuf + n, TEL_TXBUF_SIZE - n, "%lu,%lu,%lu,%lu,%lu,%u,%u,%u,%lu,%u,%lu",
                      (unsigned long)r->seq, (unsigned long)r->t_us, (unsigned long)r->tick_ms,
                      (unsigned long)r->loop_dt_us, (unsigned long)r->svc_lat_us,
                      (unsigned)r->mode, (unsigned)r->estop, (unsigned)r->scn,
                      (unsigned long)s_dropped, (unsigned)r->frm, (unsigned long)r->frm_age_us);
        for (int i = 0; i < 4; i++) {
            const tel_wrec_t *o = &r->w[i];
            n += snprintf((char *)s_txbuf + n, TEL_TXBUF_SIZE - n, ",%ld,%ld,%d,%d,%ld,%d,%u,%lu",
                          (long)o->enc, (long)o->rpm10, (int)o->tgt, (int)o->out1e4,
                          (long)o->step, (int)o->cmd, (unsigned)o->fl, (unsigned long)o->hsw);
        }
        uint8_t cc = crc8(s_txbuf + start, n - start);
        n += snprintf((char *)s_txbuf + n, TEL_TXBUF_SIZE - n, "*%02X\r\n", (unsigned)cc);

        tail = (uint16_t)((tail + 1u) % TEL_RING_SIZE);
        emitted++;
    }

    if (n > 0 && HAL_UART_Transmit_DMA(huart, s_txbuf, (uint16_t)n) == HAL_OK) {
        s_tx_busy = 1;
        s_tx_start_ms = HAL_GetTick();
        s_tail = tail;
        if (header) { s_want_header = 0; s_rows_since_header = 0; }
        s_rows_since_header += emitted;
    }
}

#else  /* TEL_ENABLE == 0 */

void tel_init(void) {}
void tel_reset(void) {}
void tel_mark_isr(void) {}
void tel_push4(const tel_wheel_t w[4], uint8_t m, uint8_t e) { (void)w; (void)m; (void)e; }
void tel_service(UART_HandleTypeDef *h) { (void)h; }
void tel_tx_done(void) {}
void tel_set_scenario(uint8_t s) { (void)s; }
void tel_note_frame(uint16_t n) { (void)n; }
void tel_homing_sweep_start(uint8_t i) { (void)i; }
void tel_homing_switch(uint8_t i) { (void)i; }

#endif
