/*
 * knob_list.h — THE single source of truth for every tunable parameter.
 *
 * X-macro columns: (SYM, id, "name", TYPE, min, max, default, "unit", flags)
 *
 * Expanded by nc_knobs.c into: enum knob index, flash descriptor table,
 * RAM value array. Exposed over BLE CONTROL (get/set/save/reset/discover)
 * and the USB console. IDs are PERMANENT and append-only: a semantic
 * change means a NEW id + a tombstone in the NVS migration table.
 * t_knobs.c enforces id uniqueness and default-in-range.
 *
 * Types are UI/range metadata; all values travel and store as i32.
 * Flags: NC_KF_PERSIST | NC_KF_LIVE (immediate) | NC_KF_RESTREAM
 * (applies at next stream start / rate reprogram) | NC_KF_REBOOT.
 */
#pragma once
#include "narbis/proto.h"

#define KP  (NC_KF_PERSIST)
#define KPL (NC_KF_PERSIST | NC_KF_LIVE)
#define KPS (NC_KF_PERSIST | NC_KF_RESTREAM)
#define KPR (NC_KF_PERSIST | NC_KF_REBOOT)

#define KNOB_LIST(X) \
/*   SYM                 id      name                     type          min    max      def   unit    flags */ \
/* -- power / battery (0x00xx) ------------------------------------------------------ */ \
X(IDLE_TIMEOUT_S,      0x0001, "idle_timeout_s",        NC_KNOB_U16,     10,   3600,    300, "s",     KPL) \
X(OFFEAR_SLEEP_S,      0x0002, "offear_sleep_s",        NC_KNOB_U16,     10,   3600,    120, "s",     KPL) \
X(MOTION_WAKE_EN,      0x0003, "motion_wake_en",        NC_KNOB_BOOL,     0,      1,      1, "",      KPL) \
X(VBATT_WARN_MV,       0x0004, "vbatt_warn_mv",         NC_KNOB_U16,   3200,   4000,   3500, "mV",    KPL) \
X(VBATT_STOP_MV,       0x0005, "vbatt_stop_mv",         NC_KNOB_U16,   3100,   3900,   3400, "mV",    KPL) \
X(VBATT_OFF_MV,        0x0006, "vbatt_off_mv",          NC_KNOB_U16,   3000,   3800,   3300, "mV",    KPL) \
X(STREAM_ON_USB,       0x0007, "stream_on_usb",         NC_KNOB_BOOL,     0,      1,      1, "",      KPL) \
/* -- button (0x01xx) --------------------------------------------------------------- */ \
X(PRESS_LONG_MS,       0x0101, "press_long_ms",         NC_KNOB_U16,    500,   5000,   1500, "ms",    KPL) \
X(PRESS_DOUBLE_MS,     0x0102, "press_double_ms",       NC_KNOB_U16,    200,   1000,    400, "ms",    KPL) \
X(PRESS_REBOOT_MS,     0x0103, "press_reboot_ms",       NC_KNOB_U16,   4000,  15000,   8000, "ms",    KPL) \
/* -- BLE (0x02xx) ------------------------------------------------------------------ */ \
X(PAIRING_WINDOW_S,    0x0201, "pairing_window_s",      NC_KNOB_U16,     10,    600,     60, "s",     KPL) \
X(OPEN_PAIRING,        0x0202, "open_pairing",          NC_KNOB_BOOL,     0,      1,      0, "",      KPR) \
X(HRS_EN,              0x0203, "hrs_en",                NC_KNOB_BOOL,     0,      1,      1, "",      KPR) \
X(PPG_BATCH_MS,        0x0204, "ppg_batch_ms",          NC_KNOB_U8,      20,    200,     50, "ms",    KPL) \
/* -- PPG engine (0x03xx) ----------------------------------------------------------- */ \
X(PPG_RATE,            0x0301, "ppg_rate",              NC_KNOB_U8,       0,      4,      1, "code",  KPS) \
X(AMB_STREAM,          0x0302, "amb_stream",            NC_KNOB_BOOL,     0,      1,      0, "",      KPL) \
X(AMB_SUBTRACT,        0x0303, "amb_subtract",          NC_KNOB_BOOL,     0,      1,      1, "",      KPL) \
/* -- AGC (0x04xx) ------------------------------------------------------------------ */ \
X(AGC_EN,              0x0401, "agc_en",                NC_KNOB_BOOL,     0,      1,      1, "",      KPL) \
X(AGC_PERIOD_MS,       0x0402, "agc_period_ms",         NC_KNOB_U16,    100,   5000,    500, "ms",    KPL) \
X(AGC_TARGET_PCT,      0x0403, "agc_target_pct",        NC_KNOB_U8,      20,     80,     50, "%FS",   KPL) \
X(AGC_DEADBAND_PCT,    0x0404, "agc_deadband_pct",      NC_KNOB_U8,       5,     40,     15, "%FS",   KPL) \
X(AGC_STEP_MA,         0x0405, "agc_step_ma",           NC_KNOB_U8,       1,      5,      1, "mA",    KPL) \
X(AGC_MIN_MA_IR,       0x0406, "agc_min_ma_ir",         NC_KNOB_U8,       1,     50,      2, "mA",    KPL) \
X(AGC_MAX_MA_IR,       0x0407, "agc_max_ma_ir",         NC_KNOB_U8,       1,     50,     50, "mA",    KPL) \
X(AGC_MIN_MA_RED,      0x0408, "agc_min_ma_red",        NC_KNOB_U8,       1,     40,      2, "mA",    KPL) \
X(AGC_MAX_MA_RED,      0x0409, "agc_max_ma_red",        NC_KNOB_U8,       1,     40,     40, "mA",    KPL) \
X(AGC_SETTLE_MS,       0x040A, "agc_settle_ms",         NC_KNOB_U16,     50,   1000,    200, "ms",    KPL) \
X(AGC_HOLD_MS,         0x040B, "agc_hold_ms",           NC_KNOB_U16,    500,  10000,   2000, "ms",    KPL) \
X(AGC_OFFDAC_EN,       0x040C, "agc_offdac_en",         NC_KNOB_BOOL,     0,      1,      1, "",      KPL) \
/* -- DSP (0x05xx) ------------------------------------------------------------------ */ \
X(HP_FC_X100,          0x0501, "hp_fc_x100",            NC_KNOB_U16,     10,    200,     50, "cHz",   KPS) \
X(BP_LO_X100,          0x0502, "bp_lo_x100",            NC_KNOB_U16,     20,    200,     50, "cHz",   KPS) \
X(BP_HI_X100,          0x0503, "bp_hi_x100",            NC_KNOB_U16,    300,   1500,    800, "cHz",   KPS) \
X(NOTCH_EN,            0x0504, "notch_en",              NC_KNOB_BOOL,     0,      1,      0, "",      KPS) \
X(NOTCH_HZ,            0x0505, "notch_hz",              NC_KNOB_U8,      50,     60,     60, "Hz",    KPS) \
/* -- IBI (0x06xx) ------------------------------------------------------------------ */ \
X(IBI_CHANNEL,         0x0601, "ibi_channel",           NC_KNOB_U8,       0,      1,      0, "",      KPL) \
X(SSF_WIN_MS,          0x0602, "ssf_win_ms",            NC_KNOB_U8,      20,    200,     80, "ms",    KPL) \
X(THR_FRAC_X100,       0x0603, "thr_frac_x100",         NC_KNOB_U8,      10,     90,     50, "%",     KPL) \
X(THR_TAU_MS,          0x0604, "thr_tau_ms",            NC_KNOB_U16,    500,  10000,   3000, "ms",    KPL) \
X(REFRACT_MS,          0x0605, "refract_ms",            NC_KNOB_U16,    150,    500,    280, "ms",    KPL) \
X(INTERP_EN,           0x0606, "interp_en",             NC_KNOB_BOOL,     0,      1,      1, "",      KPL) \
X(IBI_MIN_MS,          0x0607, "ibi_min_ms",            NC_KNOB_U16,    200,   1000,    300, "ms",    KPL) \
X(IBI_MAX_MS,          0x0608, "ibi_max_ms",            NC_KNOB_U16,   1000,   3000,   2000, "ms",    KPL) \
X(IBI_THR_MIN,         0x0609, "ibi_thr_min",           NC_KNOB_I32,      1, 4194304,   100, "cnt",   KPL) \
/* -- artifact gate (0x07xx) -------------------------------------------------------- */ \
X(GATE_EN,             0x0701, "gate_en",               NC_KNOB_BOOL,     0,      1,      1, "",      KPL) \
X(GATE_ACC_WIN_MS,     0x0702, "gate_acc_win_ms",       NC_KNOB_U16,     50,   1000,    200, "ms",    KPL) \
X(GATE_ACC_THR,        0x0703, "gate_acc_thr",          NC_KNOB_I32,      1, 1000000,  8000, "cnt2",  KPL) \
X(SAT_PCT,             0x0704, "sat_pct",               NC_KNOB_U8,      80,    100,     95, "%FS",   KPL) \
X(STEP_SIGMA_X10,      0x0705, "step_sigma_x10",        NC_KNOB_U8,      10,    100,     40, "x0.1",  KPL) \
X(GATE_COLLAPSE_PCT,   0x0706, "gate_amp_collapse_pct", NC_KNOB_U8,       5,     90,     30, "%",     KPL) \
X(GATE_RELEASE_MS,     0x0707, "gate_release_ms",       NC_KNOB_U16,      0,   5000,    500, "ms",    KPL) \
/* -- accelerometer (0x08xx) -------------------------------------------------------- */ \
X(ACC_ODR,             0x0801, "acc_odr",               NC_KNOB_U8,       0,      5,      2, "code",  KPS) \
X(ACC_FS,              0x0802, "acc_fs",                NC_KNOB_U8,       0,      3,      1, "code",  KPS) \
X(ACC_STREAM_EN,       0x0803, "acc_stream_en",         NC_KNOB_BOOL,     0,      1,      1, "",      KPL) \
/* -- wear detection (0x09xx) ------------------------------------------------------- */ \
X(WEAR_ON_THR,         0x0901, "wear_on_thr",           NC_KNOB_I32,      0, 4194304, 20000, "cnt",   KPL) \
X(WEAR_OFF_THR,        0x0902, "wear_off_thr",          NC_KNOB_I32,      0, 4194304, 10000, "cnt",   KPL) \
X(WEAR_OFF_S,          0x0903, "wear_off_s",            NC_KNOB_U16,      5,    600,     30, "s",     KPL) \
X(WEAR_DARK_THR,       0x0904, "wear_dark_thr",         NC_KNOB_I32,      0, 4194304,  2000, "cnt",   KPL) \
/* -- self-test thresholds (0x0Axx, bench-tuned; VERIFY-ON-BENCH) ------------------- */ \
X(ST_DARK_NOISE_MAX,   0x0A01, "st_dark_noise_max",     NC_KNOB_I32,      1, 4194304,   200, "cntRMS",KPL) \
X(ST_DARK_AMB_MAX,     0x0A02, "st_dark_amb_max",       NC_KNOB_I32,      1, 4194304,  5000, "cnt",   KPL) \
X(ST_XTALK_MAX,        0x0A03, "st_xtalk_max",          NC_KNOB_I32,      1, 4194304, 50000, "cnt",   KPL) \
X(ST_BATT_MIN_MV,      0x0A04, "st_batt_min_mv",        NC_KNOB_U16,   2500,   4000,   3000, "mV",    KPL) \
X(ST_BATT_MAX_MV,      0x0A05, "st_batt_max_mv",        NC_KNOB_U16,   4000,   4500,   4300, "mV",    KPL)
