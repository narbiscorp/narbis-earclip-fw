/*
 * replay_ibi.c — run the firmware's own DSP chain, artifact gate and beat detector
 * (narbis_core nc_dsp + nc_gate + nc_ibi) over a recorded dashboard capture's v2/
 * streams, so two settings can be compared on the SAME data instead of on two people.
 *
 *   replay_ibi <ppg_raw.csv> <out.csv> [--accel accel_raw.csv]
 *              [--gate recorded|none|replay] [--knob 0xID=value]... [--dump-acc acc.csv]
 *
 * Gating:
 *   recorded (default)  the capture's own PPG batch flags (GATE, WEAR_OFF): the gate as it ran;
 *   none                never gated: the upper bound a gentler motion gate could recover;
 *   replay              recomputed here from --accel (nc_gate_accel_feed -> nc_gate_update, with
 *                       the DC-step and saturation inputs from the PPG), WEAR_OFF still recorded —
 *                       what a gate knob change WOULD have done.
 * Not replayed: ambient subtraction (captures without the AMB channel), AGC step compensation
 * (a capture with LED / gain steps replays them as raw steps), the AGC itself.
 *
 * out.csv: dev_t_beat_us,ibi_ms,confidence,flags — IBI records, as the firmware emits them.
 * --dump-acc: dev_us,acc_energy,gated,motion per PPG sample (replay mode), for picking thresholds.
 * --dump-ssf: dev_us,bp,ssf,thr,in_peak per PPG sample: the beat detector from the inside.
 * Build and the scorer: tools/replay/README.md.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "narbis/nc_agc.h"
#include "narbis/nc_dsp.h"
#include "narbis/nc_gate.h"
#include "narbis/nc_ibi.h"
#include "narbis/nc_knobs.h"

#define PPGF_GATE     0x01
#define PPGF_WEAR_OFF 0x20

typedef struct { unsigned long long dev_us; int flags; int ir; int red; int rate_sps; } smp_t;
typedef struct { unsigned long long dev_us; int x, y, z; int odr_hz; } acc_t;

/* Split [line] in place on commas, keeping empty fields. Returns the count. */
static int split(char *line, char **f, int max)
{
    int n = 0;
    f[n++] = line;
    for (char *p = line; *p && n < max; p++) {
        if (*p == ',') { *p = 0; f[n++] = p + 1; }
        else if (*p == '\n' || *p == '\r') *p = 0;
    }
    return n;
}

static int col(char **f, int nf, const char *name)
{
    for (int i = 0; i < nf; i++) if (!strcmp(f[i], name)) return i;
    return -1;
}

static int cmp_u32(const void *a, const void *b)
{
    unsigned x = *(const unsigned *)a, y = *(const unsigned *)b;
    return (x > y) - (x < y);
}

static smp_t *load_ppg(const char *path, size_t *n_out)
{
    FILE *in = fopen(path, "r");
    if (!in) { perror(path); return NULL; }
    static char line[512];
    char *f[32];
    if (!fgets(line, sizeof line, in)) { fclose(in); return NULL; }
    int nf = split(line, f, 32);
    int c_dev = col(f, nf, "dev_us"), c_flags = col(f, nf, "flags"), c_ir = col(f, nf, "ir");
    int c_red = col(f, nf, "red"), c_rate = col(f, nf, "rate_sps");
    if (c_dev < 0 || c_flags < 0 || c_ir < 0 || c_red < 0 || c_rate < 0) {
        fprintf(stderr, "%s: missing columns\n", path);
        fclose(in);
        return NULL;
    }
    size_t cap = 1 << 16, n = 0;
    smp_t *s = malloc(cap * sizeof *s);
    while (fgets(line, sizeof line, in)) {
        if (split(line, f, 32) <= c_ir) continue;
        if (n == cap) s = realloc(s, (cap *= 2) * sizeof *s);
        s[n].dev_us = strtoull(f[c_dev], NULL, 10);
        s[n].flags = (int)strtol(f[c_flags], NULL, 0);
        s[n].ir = (int)strtol(f[c_ir], NULL, 10);
        s[n].red = (int)strtol(f[c_red], NULL, 10);
        s[n].rate_sps = (int)strtol(f[c_rate], NULL, 10);
        n++;
    }
    fclose(in);
    *n_out = n;
    return s;
}

static acc_t *load_acc(const char *path, size_t *n_out)
{
    FILE *in = fopen(path, "r");
    if (!in) { perror(path); return NULL; }
    static char line[512];
    char *f[32];
    if (!fgets(line, sizeof line, in)) { fclose(in); return NULL; }
    int nf = split(line, f, 32);
    int c_dev = col(f, nf, "dev_us"), c_x = col(f, nf, "x"), c_y = col(f, nf, "y");
    int c_z = col(f, nf, "z"), c_odr = col(f, nf, "odr_hz");
    if (c_dev < 0 || c_x < 0 || c_y < 0 || c_z < 0 || c_odr < 0) {
        fprintf(stderr, "%s: missing columns\n", path);
        fclose(in);
        return NULL;
    }
    size_t cap = 1 << 15, n = 0;
    acc_t *a = malloc(cap * sizeof *a);
    while (fgets(line, sizeof line, in)) {
        if (split(line, f, 32) <= c_z) continue;
        if (n == cap) a = realloc(a, (cap *= 2) * sizeof *a);
        a[n].dev_us = strtoull(f[c_dev], NULL, 10);
        a[n].x = (int)strtol(f[c_x], NULL, 10);
        a[n].y = (int)strtol(f[c_y], NULL, 10);
        a[n].z = (int)strtol(f[c_z], NULL, 10);
        a[n].odr_hz = (int)strtol(f[c_odr], NULL, 10);
        n++;
    }
    fclose(in);
    *n_out = n;
    return a;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "usage: replay_ibi <ppg_raw.csv> <out.csv> [--accel accel_raw.csv] "
                        "[--gate recorded|none|replay] [--knob 0xID=value]... [--dump-acc acc.csv] [--dump-ssf ssf.csv]\n");
        return 2;
    }
    enum { G_RECORDED, G_NONE, G_REPLAY } gate = G_RECORDED;
    const char *acc_path = NULL, *dump_path = NULL, *ssf_path = NULL;
    nc_knobs_init();
    for (int i = 3; i < argc; i++) {
        if (!strcmp(argv[i], "--gate") && i + 1 < argc) {
            const char *g = argv[++i];
            gate = !strcmp(g, "none") ? G_NONE : !strcmp(g, "replay") ? G_REPLAY : G_RECORDED;
        } else if (!strcmp(argv[i], "--accel") && i + 1 < argc) {
            acc_path = argv[++i];
        } else if (!strcmp(argv[i], "--dump-ssf") && i + 1 < argc) {
            ssf_path = argv[++i];
        } else if (!strcmp(argv[i], "--dump-acc") && i + 1 < argc) {
            dump_path = argv[++i];
        } else if (!strcmp(argv[i], "--knob") && i + 1 < argc) {
            unsigned id; long v;
            if (sscanf(argv[++i], "%x=%ld", &id, &v) != 2 || nc_knob_set_id((uint16_t)id, (int32_t)v) != NC_ST_OK) {
                fprintf(stderr, "bad knob %s\n", argv[i]);
                return 2;
            }
        } else {
            fprintf(stderr, "unknown argument %s\n", argv[i]);
            return 2;
        }
    }
    if (gate == G_REPLAY && !acc_path) { fprintf(stderr, "--gate replay needs --accel\n"); return 2; }

    size_t n = 0, na = 0;
    smp_t *s = load_ppg(argv[1], &n);
    if (!s || n < 100) { fprintf(stderr, "too few PPG samples\n"); return 1; }
    acc_t *a = acc_path ? load_acc(acc_path, &na) : NULL;
    if (acc_path && (!a || na == 0)) { fprintf(stderr, "no accel samples\n"); return 1; }

    /* The MEASURED sample period, as the firmware's timing EMA would settle. */
    unsigned *dt = malloc((n - 1) * sizeof *dt);
    size_t nd = 0;
    for (size_t i = 1; i < n; i++) {
        unsigned long long d = s[i].dev_us - s[i - 1].dev_us;
        if (d > 0 && d < 100000) dt[nd++] = (unsigned)d;
    }
    qsort(dt, nd, sizeof *dt, cmp_u32);
    const uint32_t ts_us = dt[nd / 2];
    free(dt);

    nc_rate_t rate = s[0].rate_sps >= 500 ? NC_RATE_500 : s[0].rate_sps >= 250 ? NC_RATE_250 :
                     s[0].rate_sps >= 200 ? NC_RATE_200 : s[0].rate_sps >= 100 ? NC_RATE_100 : NC_RATE_50;
    nc_chan_dsp_t ch;
    nc_dsp_init(&ch, rate);
    nc_ibi_t ibi;
    nc_ibi_init(&ibi, ts_us);
    nc_gate_t g;
    nc_gate_init(&g);
    nc_gate_dcstep_t ds;
    nc_gate_dcstep_init(&ds);
    nc_gate_acc_t ga;
    nc_gate_acc_init(&ga, (uint16_t)(a ? a[0].odr_hz : 50));
    const int32_t sat_thr = (int32_t)(((int64_t)nc_knob_get(KNOB_SAT_PCT) * NC_AGC_FS_COUNTS) / 100);

    FILE *out = fopen(argv[2], "w");
    if (!out) { perror(argv[2]); return 1; }
    FILE *dump = dump_path ? fopen(dump_path, "w") : NULL;
    FILE *ssfd = ssf_path ? fopen(ssf_path, "w") : NULL;
    if (ssfd) fprintf(ssfd, "dev_us,bp,ssf,thr,in_peak\n");
    if (dump) fprintf(dump, "dev_us,acc_energy,gated,motion\n");
    fprintf(out, "dev_t_beat_us,ibi_ms,confidence,flags\n");
    size_t beats = 0, gated_n = 0, motion_n = 0, ai = 0;
    uint32_t acc_e = 0;
    for (size_t i = 0; i < n; i++) {
        /* The latest accel window energy at this PPG sample's time, as dsp_task reads it. */
        while (a && ai < na && a[ai].dev_us <= s[i].dev_us) {
            acc_e = nc_gate_accel_feed(&ga, (int16_t)a[ai].x, (int16_t)a[ai].y, (int16_t)a[ai].z);
            ai++;
        }
        nc_dsp_out_t o;
        nc_dsp_run(&ch, s[i].ir, &o);
        const int wear_off = (s[i].flags & PPGF_WEAR_OFF) != 0;
        int gated = 0, motion = 0;
        if (gate == G_RECORDED) {
            gated = (s[i].flags & (PPGF_GATE | PPGF_WEAR_OFF)) != 0;
        } else if (gate == G_REPLAY) {
            const int sat = s[i].ir >= sat_thr || s[i].ir <= -sat_thr || s[i].red >= sat_thr || s[i].red <= -sat_thr;
            nc_gate_in_t gin;
            nc_gate_build_input(&gin, &ds, acc_e, o.dc, sat, false, false, wear_off);
            uint8_t reason = 0;
            gated = nc_gate_update(&g, &gin, (uint32_t)(s[i].dev_us / 1000u), &reason) || wear_off;
            motion = nc_gate_motion(&g);
        }
        gated_n += gated != 0;
        motion_n += motion != 0;
        if (dump) fprintf(dump, "%llu,%u,%d,%d\n", s[i].dev_us, acc_e, gated, motion);
        if (motion) nc_ibi_note_motion(&ibi);
        nc_ibi_rec_t r;
        const bool hit = nc_ibi_update(&ibi, o.bp, s[i].dev_us, gated, &r);
        if (ssfd) {
            int64_t thr = (ibi.peak_track * ibi.k_thr_frac_x100 / 100) >> 8;
            if (thr < ibi.k_thr_min) thr = ibi.k_thr_min;
            fprintf(ssfd, "%llu,%d,%lld,%lld,%d\n", s[i].dev_us, (int)o.bp,
                    (long long)ibi.ssf_sum, (long long)thr, ibi.in_peak ? 1 : 0);
        }
        if (hit) {
            fprintf(out, "%llu,%u,%u,%u\n", (unsigned long long)r.t_beat_us, r.ibi_ms, r.confidence, r.flags);
            beats++;
        }
    }
    fclose(out);
    if (dump) fclose(dump);
    if (ssfd) fclose(ssfd);
    free(s);
    free(a);
    fprintf(stderr, "replay_ibi: %zu samples (period %u us), %zu gated, %zu in soft motion, %zu beat records\n",
            n, ts_us, gated_n, motion_n, beats);
    return 0;
}
