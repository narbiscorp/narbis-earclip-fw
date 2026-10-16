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
/*   standby (no central, no stream): on battery and not worn this long -> OFF     */ \
/*   (0 = never). Worn = ear posture (accel) confirmed by the optical detector.    */ \
X(STANDBY_OFF_S,       0x0008, "standby_off_s",         NC_KNOB_U16,      0,   3600,    120, "s",     KPL) \
/*   standby indicator: one smooth breath (dark -> bright -> dark) per period      */ \
X(IND_BREATH_MS,       0x0009, "ind_breath_ms",         NC_KNOB_U16,    600,   5000,   1500, "ms",    KPL) \
/*   a central that subscribes to nothing of ours this long is dropped (phone /   */ \
/*   PC background reconnects to a bonded clip); OTA exempt; 0 = never.           */ \
X(IDLE_KICK_S,         0x000A, "idle_kick_s",           NC_KNOB_U16,      0,    600,     15, "s",     KPL) \
/*   BLE transmit power, advertising + connections (C6: -15..+20 dBm in 3 dB    */ \
/*   steps, +20 the top; values between round down). +20 for range (user,       */ \
/*   2026-10-09; it was the IDF default +9); lower to save battery.              */ \
X(BLE_TX_DBM,          0x000B, "ble_tx_dbm",            NC_KNOB_I32,    -15,     20,     20, "dBm",   KPL) \
/*   a NEW central still pairing (no bond yet, link not encrypted) gets this    */ \
/*   long before the idle kick: the user may be reading the phone's pairing     */ \
/*   prompt. Bonded / encrypted links keep idle_kick_s.                         */ \
X(IDLE_KICK_PAIR_S,    0x000C, "idle_kick_pair_s",      NC_KNOB_U16,     15,    600,     60, "s",     KPL) \
/* -- button (0x01xx) --------------------------------------------------------------- */ \
X(PRESS_LONG_MS,       0x0101, "press_long_ms",         NC_KNOB_U16,    200,   5000,   1500, "ms",    KPL) \
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
/*   acquire -> lock (nc_agc.h): 0 = legacy continuous ladder; 1 = adjust only   */ \
/*   until clean beats lock it, then re-acquire only on sustained clip / drift;  */ \
/*   2 = as 1, but acquire with the OPTIMIZER (model-based jump to the target at */ \
/*   the preferred LED currents) and re-acquire on drift beyond agc_drift_band.  */ \
X(AGC_MODE,            0x040D, "agc_mode",              NC_KNOB_U8,       0,      2,      2, "",      KPL) \
X(AGC_LOCK_BEATS,      0x040E, "agc_lock_beats",        NC_KNOB_U8,       1,     30,      6, "beats", KPL) \
X(AGC_LOCK_STABLE_MS,  0x040F, "agc_lock_stable_ms",    NC_KNOB_U16,   1000,  60000,   8000, "ms",    KPL) \
X(AGC_LOCK_BAND_PCT,   0x0410, "agc_lock_band_pct",     NC_KNOB_U8,      10,     50,     38, "%FS",   KPL) \
X(AGC_REACQ_SAT_MS,    0x0411, "agc_reacq_sat_ms",      NC_KNOB_U16,    500,  30000,   3000, "ms",    KPL) \
X(AGC_REACQ_DRIFT_MS,  0x0412, "agc_reacq_drift_ms",    NC_KNOB_U16,   1000,  60000,  15000, "ms",    KPL) \
X(AGC_REACQ_MIN_S,     0x0413, "agc_reacq_min_s",       NC_KNOB_U16,      5,   3600,    180, "s",     KPL) \
/*   off-ear hold: once a stream has locked, leave the settings alone while the    */ \
/*   wear detector says off-ear, for at most this long (0 = never hold).           */ \
X(AGC_OFFEAR_HOLD_S,   0x0414, "agc_offear_hold_s",     NC_KNOB_U16,      0,    600,     20, "s",     KPL) \
/*   probe: before the first worn lock of a stream, while off-ear, only the LEDs  */ \
/*   move and never above this — an empty clip cannot crush offdac / gain.        */ \
X(AGC_PROBE_MAX_MA,    0x0415, "agc_probe_max_ma",      NC_KNOB_U8,       1,     50,     10, "mA",    KPL) \
/*   optimizer (agc_mode 2): preferred LED currents — more light at lower TIA gain */ \
/*   (user bench 2026-10-09: 25/20 mA @ 50k vs the ladder's 8/5 mA @ 100k at the  */ \
/*   same level: pulse 2x, rest beat error 38 -> 16 ms vs H10).                   */ \
X(AGC_PREF_MA_IR,      0x0416, "agc_pref_ma_ir",        NC_KNOB_U8,       1,     50,     25, "mA",    KPL) \
X(AGC_PREF_MA_RED,     0x0417, "agc_pref_ma_red",       NC_KNOB_U8,       1,     40,     20, "mA",    KPL) \
/*   agc_mode 2, LOCKED: raw DC beyond target +- this for agc_reacq_drift_ms      */ \
/*   re-runs the optimizer (beats or not), never sooner than agc_reacq_min_s.     */ \
X(AGC_DRIFT_BAND,      0x0418, "agc_drift_band",        NC_KNOB_I32,  50000, 1000000, 500000, "cnt", KPL) \
/*   AGC step compensation (nc_agcstep.h): rescale the DSP + beat chain across  */ \
/*   every LED / gain step instead of gating the settle window, so beats keep   */ \
/*   coming through a re-acquire. 0 = the old settle gating (A/B).              */ \
X(AGC_STEP_COMP,       0x0419, "agc_step_comp",         NC_KNOB_BOOL,     0,      1,      1, "",      KPL) \
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
/*   plausibility (nc_ibi.h): flag ARTIFACT when an interval is more than tol %   */ \
/*   off BOTH the median of the last 5 accepted and the last accepted interval,    */ \
/*   and outside the breathing envelope (the range of the last 16 accepted).       */ \
X(IBI_ART_EN,          0x060A, "ibi_art_en",            NC_KNOB_BOOL,     0,      1,      1, "",      KPL) \
X(IBI_ART_TOL_PCT,     0x060B, "ibi_art_tol_pct",       NC_KNOB_U8,       5,     50,     20, "%",     KPL) \
/*   adaptive refractory (nc_ibi.c commit): after a beat, no new one for max(refract_ms, */ \
/*   this % of the median clean interval); 0 = refract_ms only. Stops double counts.     */ \
X(REFRACT_FRAC_PCT,    0x060C, "refract_frac_pct",      NC_KNOB_U8,       0,     80,     60, "%",     KPL) \
/*   ...and while the beat was found during light motion (proto 1.4 MOTION), where junk beats   */ \
/*   cluster: 20 % still / 15 % moving flagged 5 good beats (of 283) and 43 junk on 2026-10-14. */ \
X(IBI_ART_TOL_MOTION_PCT, 0x060D, "ibi_art_tol_motion_pct", NC_KNOB_U8,  5,     50,     15, "%",     KPL) \
/*   beat time. A two-stage upstroke (the reflected wave) gives the slope sum two near-equal     */ \
/*   humps ~150 ms apart; the plain peak mark flipped between them (2026-10-15: IBIs             */ \
/*   800/1150/800 on an even ~990 ms pulse, a third of beats flagged ARTIFACT).                  */ \
/*   0 = the slope-sum peak (to v0.3.0-52);                                                      */ \
/*   1 = the onset, where the slope sum first rises through half its peak: steadiest on          */ \
/*       that recording (median beat-to-beat change 148 -> 22 ms) but 12 -> 58 ms p90 off        */ \
/*       the H10 at rest on 2026-10-14;                                                          */ \
/*   2 = the first hump reaching 70 % of the peak (default): 148 -> 32 ms there, and as          */ \
/*       good as 0 against the H10 at rest (p90 12 / 8 ms), talking 61 -> 74 % found.            */ \
X(IBI_FIDUCIAL,        0x060E, "ibi_fiducial",          NC_KNOB_U8,       0,      2,      2, "",      KPL) \
/* -- artifact gate (0x07xx) -------------------------------------------------------- */ \
X(GATE_EN,             0x0701, "gate_en",               NC_KNOB_BOOL,     0,      1,      1, "",      KPL) \
X(GATE_ACC_WIN_MS,     0x0702, "gate_acc_win_ms",       NC_KNOB_U16,     50,   1000,    200, "ms",    KPL) \
X(GATE_ACC_THR,        0x0703, "gate_acc_thr",          NC_KNOB_I32,      1, 1000000,  8000, "cnt2",  KPL) \
X(SAT_PCT,             0x0704, "sat_pct",               NC_KNOB_U8,      80,    100,     95, "%FS",   KPL) \
X(STEP_SIGMA_X10,      0x0705, "step_sigma_x10",        NC_KNOB_U8,      10,    100,     40, "x0.1",  KPL) \
X(GATE_COLLAPSE_PCT,   0x0706, "gate_amp_collapse_pct", NC_KNOB_U8,       5,     90,     30, "%",     KPL) \
X(GATE_RELEASE_MS,     0x0707, "gate_release_ms",       NC_KNOB_U16,      0,   5000,    500, "ms",    KPL) \
/*   two-level motion: accel energy over gate_acc_thr but under this is LIGHT motion    */ \
/*   (chewing, talking, a foot tap): detected through, beats flagged MOTION; over it the */ \
/*   gate closes. <= gate_acc_thr restores the single threshold (every motion gates).    */ \
/*   200 k: 3 guided H10 runs peaked at 359 k chewing / talking (p99 20-193 k), clip     */ \
/*   removal reads 0.87-4.5 M; replayed, chew/talk beats found 32-51 % -> 81-92 %.       */ \
X(GATE_ACC_HARD_THR,   0x0708, "gate_acc_hard_thr",     NC_KNOB_I32,      1, 50000000, 200000, "cnt2",  KPL) \
/* -- accelerometer (0x08xx) -------------------------------------------------------- */ \
X(ACC_ODR,             0x0801, "acc_odr",               NC_KNOB_U8,       0,      5,      2, "code",  KPS) \
X(ACC_FS,              0x0802, "acc_fs",                NC_KNOB_U8,       0,      3,      1, "code",  KPS) \
X(ACC_STREAM_EN,       0x0803, "acc_stream_en",         NC_KNOB_BOOL,     0,      1,      1, "",      KPL) \
/*   ear posture (nc_posture.h): the clip hangs from the lobe with gravity along   */ \
/*   ear_axis (0 +x, 1 -x, 2 +y, 3 -y, 4 +z, 5 -z; V2.1 worn: -x), within the cone  */ \
/*   cos >= ear_cos_pct %, held ear_hold_ms (both ways).                             */ \
X(EAR_AXIS,            0x0804, "ear_axis",              NC_KNOB_U8,       0,      5,      1, "code",  KPL) \
X(EAR_COS_PCT,         0x0805, "ear_cos_pct",           NC_KNOB_U8,      50,     99,     75, "%",     KPL) \
X(EAR_HOLD_MS,         0x0806, "ear_hold_ms",           NC_KNOB_U16,    200,  10000,   2000, "ms",    KPL) \
/* -- wear detection (0x09xx) ------------------------------------------------------- */ \
/*   DC thresholds are on the ambient-subtracted IR DC. V2.1 earlobe, AGC locked:   */ \
/*   ~565k; empty clip in air ~35k; empty clip on a table: raw IR at the rail.      */ \
X(WEAR_ON_THR,         0x0901, "wear_on_thr",           NC_KNOB_I32,      0, 4194304, 2050000, "cnt", KPL) \
X(WEAR_OFF_THR,        0x0902, "wear_off_thr",          NC_KNOB_I32,      0, 4194304, 1900000, "cnt", KPL) \
X(WEAR_OFF_S,          0x0903, "wear_off_s",            NC_KNOB_U16,      5,    600,     30, "s",     KPL) \
X(WEAR_DARK_THR,       0x0904, "wear_dark_thr",         NC_KNOB_I32,      0, 4194304, 100000, "cnt",  KPL) \
/*   fast off: raw IR held at the ADC rail this many whole seconds = empty clip.   */ \
X(WEAR_RAIL_S,         0x0905, "wear_rail_s",           NC_KNOB_U8,       1,     30,      2, "s",     KPL) \
/*   handling jolt (gate accel energy, cnt^2): within wear_jolt_win_s of one, the  */ \
/*   off countdown is wear_jolt_off_s instead of wear_off_s.                        */ \
X(WEAR_JOLT_THR,       0x0906, "wear_jolt_thr",         NC_KNOB_I32,   1000, 100000000, 400000, "cnt2", KPL) \
X(WEAR_JOLT_OFF_S,     0x0907, "wear_jolt_off_s",       NC_KNOB_U16,      1,    600,      3, "s",     KPL) \
X(WEAR_JOLT_WIN_S,     0x0908, "wear_jolt_win_s",       NC_KNOB_U16,      1,    600,     10, "s",     KPL) \
/* -- self-test thresholds (0x0Axx, bench-tuned; VERIFY-ON-BENCH) ------------------- */ \
X(ST_DARK_NOISE_MAX,   0x0A01, "st_dark_noise_max",     NC_KNOB_I32,      1, 4194304,   200, "cntRMS",KPL) \
X(ST_DARK_AMB_MAX,     0x0A02, "st_dark_amb_max",       NC_KNOB_I32,      1, 4194304,  5000, "cnt",   KPL) \
X(ST_XTALK_MAX,        0x0A03, "st_xtalk_max",          NC_KNOB_I32,      1, 4194304, 50000, "cnt",   KPL) \
X(ST_BATT_MIN_MV,      0x0A04, "st_batt_min_mv",        NC_KNOB_U16,   2500,   4000,   3000, "mV",    KPL) \
X(ST_BATT_MAX_MV,      0x0A05, "st_batt_max_mv",        NC_KNOB_U16,   4000,   4500,   4300, "mV",    KPL)
