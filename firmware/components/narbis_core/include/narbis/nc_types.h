/*
 * nc_types.h — shared platform-free types for the Narbis earclip firmware.
 *
 * narbis_core rule: this header (and everything under narbis/) compiles
 * with only <stdint.h>/<stdbool.h> — no ESP-IDF, no FreeRTOS — so the
 * same objects build on the target and under mingw for host tests.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/* PPG sample rates                                                    */
/* ------------------------------------------------------------------ */
typedef enum {
    NC_RATE_50 = 0,
    NC_RATE_100 = 1,   /* product default */
    NC_RATE_200 = 2,
    NC_RATE_250 = 3,
    NC_RATE_500 = 4,
    NC_RATE_COUNT = 5
} nc_rate_t;

static inline uint16_t nc_rate_sps(nc_rate_t r)
{
    static const uint16_t sps[NC_RATE_COUNT] = { 50, 100, 200, 250, 500 };
    return (r < NC_RATE_COUNT) ? sps[r] : 0;
}

/* Accel ODR codes (LIS2DH12) */
typedef enum {
    NC_ODR_10 = 0, NC_ODR_25 = 1, NC_ODR_50 = 2,
    NC_ODR_100 = 3, NC_ODR_200 = 4, NC_ODR_400 = 5,
    NC_ODR_COUNT = 6
} nc_acc_odr_t;

static inline uint16_t nc_acc_odr_hz(nc_acc_odr_t o)
{
    static const uint16_t hz[NC_ODR_COUNT] = { 10, 25, 50, 100, 200, 400 };
    return (o < NC_ODR_COUNT) ? hz[o] : 0;
}

/* ------------------------------------------------------------------ */
/* Samples (plain structs; the FreeRTOS queue wrappers live in main/)  */
/* ------------------------------------------------------------------ */
typedef struct {
    uint64_t t_us;   /* esp_timer_get_time() captured in the ADC_RDY ISR */
    int32_t  ir;     /* LED1VAL, 24-bit two's-complement sign-extended   */
    int32_t  red;    /* LED2VAL                                          */
    int32_t  amb;    /* ALED1VAL (0 if ambient not read)                 */
} nc_ppg_sample_t;

typedef struct {
    uint64_t t_us;
    int16_t  x, y, z;   /* raw counts, scale per configured FS */
} nc_accel_sample_t;

/* ------------------------------------------------------------------ */
/* System state (STATUS wire values — keep stable)                     */
/* ------------------------------------------------------------------ */
typedef enum {
    NC_STATE_IDLE = 0,
    NC_STATE_CONNECTED = 1,
    NC_STATE_STREAMING = 2,
    NC_STATE_OTA = 3,
    NC_STATE_SELFTEST = 4,
    NC_STATE_LOWBATT = 5
} nc_sys_state_t;

typedef enum {
    NC_CHG_ON_BATTERY = 0,
    NC_CHG_CHARGING = 1,
    NC_CHG_COMPLETE = 2
} nc_charger_state_t;

/* ------------------------------------------------------------------ */
/* Internal event record (fixed size, hot-path safe).                  */
/* The BLE wire encoding of these lives in proto.h/proto_encode.c.     */
/* ------------------------------------------------------------------ */
typedef enum {
    NC_EV_AGC_STEP = 0x01,   /* data: led, old_ma, new_ma, old_rf, new_rf */
    NC_EV_GATE = 0x02,       /* data: state, reason_mask                  */
    NC_EV_WEAR = 0x03,       /* data: worn                                */
    NC_EV_MARKER = 0x04,     /* data: source, marker_id lo, hi            */
    NC_EV_ERROR = 0x05,      /* data: code lo/hi, arg (4B LE)             */
    NC_EV_RATE_CHANGE = 0x06,/* data: old_code, new_code                  */
    NC_EV_AGC_OFFDAC = 0x07, /* data: phase, old_code, new_code           */
    NC_EV_SELFTEST_DONE = 0x08 /* data: pass_count, fail_count            */
} nc_event_type_t;

/* Gate reason mask bits (NC_EV_GATE payload + gate module output) */
#define NC_GATE_REASON_ACCEL     (1u << 0)
#define NC_GATE_REASON_SAT       (1u << 1)
#define NC_GATE_REASON_DC_STEP   (1u << 2)
#define NC_GATE_REASON_COLLAPSE  (1u << 3)
#define NC_GATE_REASON_AGC       (1u << 4)
#define NC_GATE_REASON_WEAR      (1u << 5)

typedef struct {
    uint64_t t_us;
    uint8_t  type;      /* nc_event_type_t */
    uint8_t  len;       /* used bytes of data[] */
    uint8_t  data[14];
} nc_event_t;

/* ------------------------------------------------------------------ */
/* PPG stream flag bits (per-batch; proto.h re-exports on the wire)    */
/* ------------------------------------------------------------------ */
#define NC_PPGF_GATE          (1u << 0)
#define NC_PPGF_AGC_SETTLING  (1u << 1)
#define NC_PPGF_USB_PRESENT   (1u << 2)
#define NC_PPGF_RATE_CHANGED  (1u << 3)
#define NC_PPGF_AMB           (1u << 4)  /* amb field present in batch  */
#define NC_PPGF_WEAR_OFF      (1u << 5)
#define NC_PPGF_CLIPPED       (1u << 6)  /* >=1 sample near rail        */

/* IBI record flag bits */
#define NC_IBIF_GATED_CTX     (1u << 0)
#define NC_IBIF_AGC_SETTLING  (1u << 1)
#define NC_IBIF_INTERPOLATED  (1u << 2)
#define NC_IBIF_FIRST_AFTER_GAP (1u << 3)

/* ------------------------------------------------------------------ */
/* Fixed-point helpers (no FPU on the C6 — hot paths are Q15/Q31/i64)  */
/* ------------------------------------------------------------------ */
static inline int32_t nc_sat32(int64_t v)
{
    if (v > INT32_MAX) return INT32_MAX;
    if (v < INT32_MIN) return INT32_MIN;
    return (int32_t)v;
}

/* (a * b) >> 31 with int64 intermediate — Q31 multiply */
static inline int32_t nc_mul_q31(int32_t a, int32_t b)
{
    return (int32_t)(((int64_t)a * b) >> 31);
}

/* (a * b) >> 30 — Q30 coefficient multiply, saturated */
static inline int64_t nc_mac_q30(int64_t acc, int32_t x, int32_t c_q30)
{
    return acc + (int64_t)x * c_q30;
}

static inline int32_t nc_clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}
