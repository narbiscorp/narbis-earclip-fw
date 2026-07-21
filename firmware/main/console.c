/*
 * console.c — esp_console REPL on USB-Serial-JTAG. The dev tool of
 * first resort (handoff §5.12.7).
 *
 * DESIGN: the console is a SECOND CONTROL TRANSPORT. Read-only commands
 * touch lock-free surfaces directly (diag counters, knob RAM words,
 * aggregates, the console tap). Every state-changing command builds the
 * exact proto.h CONTROL request bytes and posts them to sys_q as
 * SYS_CTRL_REQ — sys_task dispatches it through the same
 * nc_ctrl_dispatch() path as a BLE write, so ALL mutations serialize in
 * one place and the console can never race the radio. Consequences,
 * printed optimistically and documented here:
 *   - the eventual response indication goes to the BLE CONTROL char
 *     (harmlessly discarded when nobody is connected/subscribed);
 *   - console tids live in 0xC0..0xFF so a sniffing host can tell
 *     console-originated responses from its own (hosts should use tids
 *     below 0xC0; tid 0x00 is reserved for device-internal requests).
 *
 * Exception: `bonds clear` calls NimBLE ble_store_clear() directly —
 * ble_iface.h has no bond-management surface (flagged to the BLE owner);
 * a dev-console-only, non-streaming operation.
 */
#include "console.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_console.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_app_desc.h"
#include "esp_core_dump.h"
#include "argtable3/argtable3.h"
#include "host/ble_store.h"           /* ble_store_clear() — see header */

#include "narbis/proto.h"
#include "narbis/nc_types.h"
#include "narbis/nc_knobs.h"
#include "narbis/nc_proto_encode.h"

#include "board.h"
#include "app_msgs.h"
#include "acq.h"
#include "ble_iface.h"
#include "battery.h"
#include "charger.h"
#include "diag.h"
#include "selftest.h"
#include "ota.h"

static const char *TAG = "console";

/* ------------------------------------------------------------------ */
/* CONTROL-over-sys_q plumbing                                         */
/* ------------------------------------------------------------------ */
static uint8_t next_tid(void)
{
    static uint8_t t;
    return (uint8_t)(0xC0 | (t++ & 0x3F));   /* console tid space */
}

static bool post_ctrl(const uint8_t *req, uint8_t len)
{
    if (len > SYS_CTRL_REQ_MAX) {
        return false;
    }
    sys_msg_t m = { .type = SYS_CTRL_REQ };
    m.u.ctrl.len = len;
    memcpy(m.u.ctrl.buf, req, len);
    if (!sys_post(&m)) {
        printf("ERR: sys_q full — command dropped\n");
        return false;
    }
    printf("queued tid 0x%02X (applies in sys_task; response rides BLE "
           "CONTROL)\n", req[1]);
    return true;
}

/* ------------------------------------------------------------------ */
/* ver / stats / batt / state                                          */
/* ------------------------------------------------------------------ */
static int cmd_ver(int argc, char **argv)
{
    (void)argc; (void)argv;
    const esp_app_desc_t *a = esp_app_get_description();
    printf("fw:        %s (%s %s)\n", a->version, a->date, a->time);
    printf("project:   %s\n", a->project_name);
    printf("idf:       %s\n", esp_get_idf_version());
    printf("proto:     %u.%u\n", NC_PROTO_VER_MAJOR, NC_PROTO_VER_MINOR);
    printf("hw:        V2.1\n");
    printf("test_mode: %d\n", NARBIS_TEST_MODE);
    return 0;
}

static int cmd_stats(int argc, char **argv)
{
    (void)argc; (void)argv;
    diag_console_dump();
    return 0;
}

static int cmd_batt(int argc, char **argv)
{
    (void)argc; (void)argv;
    static const char *chg[] = { "on-battery", "charging", "complete" };
    uint16_t mv = 0;
    uint8_t pct = 0;
    if (battery_status(&mv, &pct) != ESP_OK) {
        printf("ERR: battery read failed\n");
        return 1;
    }
    bool vusb = false;
    nc_charger_state_t cs = charger_poll(&vusb);
    printf("battery: %u mV (%u%%)  charger: %s  vusb: %d\n",
           mv, pct, (cs <= NC_CHG_COMPLETE) ? chg[cs] : "?", vusb);
    return 0;
}

static int cmd_state(int argc, char **argv)
{
    (void)argc; (void)argv;
    printf("uptime:   %lld s\n",
           (long long)(esp_timer_get_time() / 1000000));
    bool conn = ble_is_connected();
    if (conn) {
        uint16_t mtu = 0, itvl = 0;
        uint8_t phy = 0;
        ble_get_conn_stats(&mtu, &phy, &itvl);
        printf("ble:      connected (MTU %u, PHY %uM, interval %u.%02u ms)\n",
               mtu, phy, (itvl * 125) / 100, (itvl * 125) % 100);
    } else {
        printf("ble:      not connected\n");
    }
    printf("ppg:      %s", acq_ppg_running() ? "RUNNING" : "stopped");
    if (acq_ppg_running()) {
        printf(" (measured Ts %" PRIu32 " us)", acq_measured_ts_us());
    }
    printf("\naccel:    %s\n", acq_accel_running() ? "RUNNING" : "stopped");
    acq_aggregates_t ag;
    acq_get_aggregates(&ag);
    printf("worn:     %d   gated: %d (duty %u.%02u%%)\n", ag.worn, ag.gated,
           ag.gate_duty_x100 / 100, ag.gate_duty_x100 % 100);
    printf("hr:       %u bpm (last IBI %u ms)\n", ag.hr_bpm, ag.ibi_last_ms);
    printf("dc ir/red: %" PRId32 " / %" PRId32 "\n", ag.dc_ir, ag.dc_red);
    printf("ota:      %s\n", ota_active() ? "ACTIVE" : "idle");
    return 0;
}

/* ------------------------------------------------------------------ */
/* knob                                                                */
/* ------------------------------------------------------------------ */
static struct {
    struct arg_str *action;   /* list|get|set|save|reset */
    struct arg_str *name;
    struct arg_str *value;
    struct arg_end *end;
} knob_args;

static int knob_resolve(const char *s)
{
    char *end;
    long v = strtol(s, &end, 0);
    if (end != s && *end == '\0') {
        return nc_knob_index_of((uint16_t)v);
    }
    for (size_t i = 0; i < NC_KNOB_COUNT; i++) {
        const nc_knob_desc_t *d = nc_knob_desc(i);
        if (d != NULL && strcmp(d->name, s) == 0) {
            return (int)i;
        }
    }
    return -1;
}

static void knob_print(int idx)
{
    const nc_knob_desc_t *d = nc_knob_desc((size_t)idx);
    printf("0x%04X %-22s = %" PRId32 " %s  [%" PRId32 "..%" PRId32
           " def %" PRId32 "]%s%s\n",
           d->id, d->name, nc_knob_get(idx), d->unit, d->min, d->max,
           d->def,
           (d->flags & NC_KF_RESTREAM) ? " (restream)" : "",
           (d->flags & NC_KF_REBOOT) ? " (reboot)" : "");
}

static int cmd_knob(int argc, char **argv)
{
    if (arg_parse(argc, argv, (void **)&knob_args) != 0) {
        arg_print_errors(stderr, knob_args.end, argv[0]);
        return 1;
    }
    const char *act = knob_args.action->sval[0];

    if (strcmp(act, "list") == 0) {
        for (size_t i = 0; i < NC_KNOB_COUNT; i++) {
            knob_print((int)i);
        }
        return 0;
    }
    if (strcmp(act, "save") == 0) {
        uint8_t req[2] = { NC_OP_KNOB_SAVE, next_tid() };
        return post_ctrl(req, 2) ? 0 : 1;
    }
    if (strcmp(act, "reset") == 0) {
        uint8_t scope = 0;
        if (knob_args.name->count &&
            strcmp(knob_args.name->sval[0], "nvs") == 0) {
            scope = 1;
        }
        uint8_t req[3] = { NC_OP_KNOB_RESET, next_tid(), scope };
        return post_ctrl(req, 3) ? 0 : 1;
    }
    if (knob_args.name->count == 0) {
        printf("usage: knob %s NAME|0xID%s\n", act,
               strcmp(act, "set") == 0 ? " VALUE" : "");
        return 1;
    }
    int idx = knob_resolve(knob_args.name->sval[0]);
    if (idx < 0) {
        printf("ERR: unknown knob '%s'\n", knob_args.name->sval[0]);
        return 1;
    }
    if (strcmp(act, "get") == 0) {
        knob_print(idx);
        return 0;
    }
    if (strcmp(act, "set") == 0) {
        if (knob_args.value->count == 0) {
            printf("usage: knob set NAME VALUE\n");
            return 1;
        }
        const nc_knob_desc_t *d = nc_knob_desc((size_t)idx);
        int32_t v = (int32_t)strtol(knob_args.value->sval[0], NULL, 0);
        if (v < d->min || v > d->max) {
            printf("ERR: %" PRId32 " outside [%" PRId32 "..%" PRId32 "]\n",
                   v, d->min, d->max);
            return 1;
        }
        uint8_t req[8];
        req[0] = NC_OP_KNOB_SET;
        req[1] = next_tid();
        nc_wr_u16(req + 2, d->id);
        nc_wr_i32(req + 4, v);
        return post_ctrl(req, 8) ? 0 : 1;
    }
    printf("ERR: unknown action '%s' (list|get|set|save|reset)\n", act);
    return 1;
}

/* ------------------------------------------------------------------ */
/* rate / agc / marker                                                 */
/* ------------------------------------------------------------------ */
static struct {
    struct arg_int *sps;
    struct arg_end *end;
} rate_args;

static int cmd_rate(int argc, char **argv)
{
    if (arg_parse(argc, argv, (void **)&rate_args) != 0) {
        arg_print_errors(stderr, rate_args.end, argv[0]);
        return 1;
    }
    int sps = rate_args.sps->ival[0];
    int code = -1;
    for (int r = 0; r < NC_RATE_COUNT; r++) {
        if (nc_rate_sps((nc_rate_t)r) == sps) {
            code = r;
        }
    }
    if (code < 0) {
        printf("ERR: rate must be one of 50|100|200|250|500\n");
        return 1;
    }
    uint8_t req[3] = { NC_OP_SET_RATE, next_tid(), (uint8_t)code };
    return post_ctrl(req, 3) ? 0 : 1;
}

static struct {
    struct arg_str *action;   /* freeze|unfreeze|set */
    struct arg_int *ir, *red, *rf;
    struct arg_end *end;
} agc_args;

static int cmd_agc(int argc, char **argv)
{
    if (arg_parse(argc, argv, (void **)&agc_args) != 0) {
        arg_print_errors(stderr, agc_args.end, argv[0]);
        return 1;
    }
    const char *act = agc_args.action->sval[0];
    if (strcmp(act, "freeze") == 0 || strcmp(act, "unfreeze") == 0) {
        uint8_t req[3] = { NC_OP_AGC_FREEZE, next_tid(),
                           (uint8_t)(act[0] == 'f') };
        return post_ctrl(req, 3) ? 0 : 1;
    }
    if (strcmp(act, "set") == 0) {
        if (!agc_args.ir->count || !agc_args.red->count ||
            !agc_args.rf->count) {
            printf("usage: agc set IR_MA RED_MA RF_CODE\n");
            return 1;
        }
        int ir = agc_args.ir->ival[0], red = agc_args.red->ival[0],
            rf = agc_args.rf->ival[0];
        if (ir < 0 || ir > LED_IR_MAX_MA || red < 0 ||
            red > LED_RED_MAX_MA || rf < 0 || rf > 7) {
            printf("ERR: limits ir<=%d red<=%d rf<=7\n", LED_IR_MAX_MA,
                   LED_RED_MAX_MA);
            return 1;
        }
        uint8_t req[6] = { NC_OP_AGC_MANUAL, next_tid(), (uint8_t)ir,
                           (uint8_t)red, (uint8_t)rf, 0x07 };
        return post_ctrl(req, 6) ? 0 : 1;
    }
    printf("ERR: agc freeze|unfreeze|set\n");
    return 1;
}

static struct {
    struct arg_int *id;
    struct arg_end *end;
} marker_args;

static int cmd_marker(int argc, char **argv)
{
    if (arg_parse(argc, argv, (void **)&marker_args) != 0) {
        arg_print_errors(stderr, marker_args.end, argv[0]);
        return 1;
    }
    uint16_t id = marker_args.id->count ?
                  (uint16_t)marker_args.id->ival[0] : 0;
    uint8_t req[4];
    req[0] = NC_OP_MARKER;
    req[1] = next_tid();
    nc_wr_u16(req + 2, id);
    return post_ctrl(req, 4) ? 0 : 1;
}

/* ------------------------------------------------------------------ */
/* selftest — post the run op, poll the blob, print the table          */
/* ------------------------------------------------------------------ */
static struct {
    struct arg_str *mask;
    struct arg_end *end;
} st_args;

static const char *st_name(uint8_t id)
{
    static const char *names[] = {
        "i2c_scan", "accel_whoami", "afe_reg_rw", "afe_dark",
        "xtalk", "accel_selftest", "battery", "charger",
    };
    return (id >= 1 && id <= 8) ? names[id - 1] : "?";
}

static void st_print_blob(const uint8_t *b, uint16_t len)
{
    if (len < 10 || b[0] != NC_ST_BLOB_VER) {
        printf("ERR: unexpected blob (ver %u, len %u)\n", b ? b[0] : 0, len);
        return;
    }
    uint8_t n = b[9];
    printf("self-test @ t=%llu us\n", (unsigned long long)nc_rd_u64(b + 1));
    printf("ID   %-15s %-5s %11s %11s\n", "test", "res", "value", "threshold");
    int fails = 0;
    for (uint8_t i = 0; i < n && (size_t)10 + (i + 1) * 10 <= len; i++) {
        const uint8_t *r = b + 10 + i * 10;
        uint8_t st = r[1];
        if (st == NC_TR_FAIL) {
            fails++;
        }
        printf("T%02u  %-15s %-5s %11" PRId32 " %11" PRId32 "\n", r[0],
               st_name(r[0]),
               st == NC_TR_PASS ? "PASS" : st == NC_TR_FAIL ? "FAIL" : "skip",
               nc_rd_i32(r + 2), nc_rd_i32(r + 6));
    }
    if (fails) {
        printf("==> %d FAILURE(S)\n", fails);
    } else {
        printf("==> ALL PASS\n");
    }
}

static int cmd_selftest(int argc, char **argv)
{
    if (arg_parse(argc, argv, (void **)&st_args) != 0) {
        arg_print_errors(stderr, st_args.end, argv[0]);
        return 1;
    }
    uint32_t mask = st_args.mask->count ?
                    (uint32_t)strtoul(st_args.mask->sval[0], NULL, 0) : 0;

    uint16_t blen = 0;
    const uint8_t *b = selftest_blob(&blen);
    uint64_t old_t = (b != NULL && blen >= 10 && b[0] == NC_ST_BLOB_VER) ?
                     nc_rd_u64(b + 1) : 0;

    uint8_t req[6];
    req[0] = NC_OP_SELFTEST_RUN;
    req[1] = next_tid();
    nc_wr_u32(req + 2, mask);
    if (!post_ctrl(req, 6)) {
        return 1;
    }
    printf("running (sys_task stops acquisition; dark test takes 2 s)...\n");

    /* Blob identity = t_run_us; poll until sys_task's run replaces it. */
    for (int i = 0; i < 150; i++) {              /* <= 30 s */
        vTaskDelay(pdMS_TO_TICKS(200));
        b = selftest_blob(&blen);
        if (b != NULL && blen >= 10 && b[0] == NC_ST_BLOB_VER &&
            nc_rd_u64(b + 1) != old_t) {
            st_print_blob(b, blen);
            return 0;
        }
    }
    printf("ERR: timed out — sys_task busy or run refused (check BLE "
           "CONTROL response / logs)\n");
    return 1;
}

/* ------------------------------------------------------------------ */
/* tap — raw-sample CSV from the dsp_task console ring                 */
/* ------------------------------------------------------------------ */
static struct {
    struct arg_int *n;
    struct arg_end *end;
} tap_args;

static int cmd_tap(int argc, char **argv)
{
    if (arg_parse(argc, argv, (void **)&tap_args) != 0) {
        arg_print_errors(stderr, tap_args.end, argv[0]);
        return 1;
    }
    int n = tap_args.n->ival[0];
    if (n < 1 || n > 2000) {
        printf("ERR: N must be 1..2000\n");
        return 1;
    }
    if (!acq_ppg_running()) {
        printf("ERR: PPG not streaming — tap captures live frames only\n");
        return 1;
    }
    acq_tap_arm((uint16_t)n);
    printf("t_us,ir,red,amb\n");
    nc_ppg_sample_t s[16];
    int got = 0, idle = 0;
    while (got < n && idle < 50) {               /* 5 s starvation cap */
        int k = acq_tap_read(s, 16);
        if (k <= 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            idle++;
            continue;
        }
        idle = 0;
        for (int i = 0; i < k; i++) {
            printf("%llu,%" PRId32 ",%" PRId32 ",%" PRId32 "\n",
                   (unsigned long long)s[i].t_us, s[i].ir, s[i].red,
                   s[i].amb);
        }
        got += k;
    }
    if (got < n) {
        printf("# short read: %d/%d (stream stopped?)\n", got, n);
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* bonds / power / factory-reset / coredump                            */
/* ------------------------------------------------------------------ */
static struct {
    struct arg_str *action;
    struct arg_end *end;
} bonds_args;

static int cmd_bonds(int argc, char **argv)
{
    if (arg_parse(argc, argv, (void **)&bonds_args) != 0) {
        arg_print_errors(stderr, bonds_args.end, argv[0]);
        return 1;
    }
    if (strcmp(bonds_args.action->sval[0], "clear") != 0) {
        printf("usage: bonds clear\n");
        return 1;
    }
    int rc = ble_store_clear();
    if (rc == 0) {
        printf("bond store cleared\n");
    } else {
        printf("ERR: ble_store_clear=%d\n", rc);
    }
    return rc == 0 ? 0 : 1;
}

static int cmd_reboot(int argc, char **argv)
{
    (void)argc; (void)argv;
    uint8_t req[2] = { NC_OP_REBOOT, next_tid() };
    return post_ctrl(req, 2) ? 0 : 1;
}

static int cmd_off(int argc, char **argv)
{
    (void)argc; (void)argv;
    uint8_t req[2] = { NC_OP_POWER_OFF, next_tid() };
    return post_ctrl(req, 2) ? 0 : 1;
}

static struct {
    struct arg_str *confirm;
    struct arg_end *end;
} freset_args;

static int cmd_factory_reset(int argc, char **argv)
{
    if (arg_parse(argc, argv, (void **)&freset_args) != 0) {
        arg_print_errors(stderr, freset_args.end, argv[0]);
        return 1;
    }
    if (strcmp(freset_args.confirm->sval[0], "CONFIRM") != 0) {
        printf("ERR: type 'factory-reset CONFIRM' (erases knobs + bonds)\n");
        return 1;
    }
    uint8_t req[6];
    req[0] = NC_OP_FACTORY_RESET;
    req[1] = next_tid();
    nc_wr_u32(req + 2, NC_FACTORY_MAGIC);
    return post_ctrl(req, 6) ? 0 : 1;
}

static struct {
    struct arg_str *action;   /* info|erase */
    struct arg_end *end;
} core_args;

static int cmd_coredump(int argc, char **argv)
{
    if (arg_parse(argc, argv, (void **)&core_args) != 0) {
        arg_print_errors(stderr, core_args.end, argv[0]);
        return 1;
    }
    const char *act = core_args.action->sval[0];
    if (strcmp(act, "info") == 0) {
        size_t addr = 0, size = 0;
        if (esp_core_dump_image_get(&addr, &size) == ESP_OK) {
            printf("coredump: PRESENT @0x%08x, %u bytes "
                   "(pull with 'espcoredump.py --core-format elf')\n",
                   (unsigned)addr, (unsigned)size);
        } else {
            printf("coredump: none\n");
        }
        return 0;
    }
    if (strcmp(act, "erase") == 0) {
        esp_err_t err = esp_core_dump_image_erase();
        if (err == ESP_OK) {
            printf("coredump erased\n");
        } else {
            printf("ERR: %s\n", esp_err_to_name(err));
        }
        return err == ESP_OK ? 0 : 1;
    }
    printf("usage: coredump info|erase\n");
    return 1;
}

/* ------------------------------------------------------------------ */
/* Registration                                                        */
/* ------------------------------------------------------------------ */
static void register_all(void)
{
    knob_args.action = arg_str1(NULL, NULL, "<action>",
                                "list|get|set|save|reset");
    knob_args.name  = arg_str0(NULL, NULL, "NAME|0xID", "knob (or 'nvs' for reset)");
    knob_args.value = arg_str0(NULL, NULL, "VALUE", "value for set");
    knob_args.end   = arg_end(3);

    rate_args.sps = arg_int1(NULL, NULL, "<sps>", "50|100|200|250|500");
    rate_args.end = arg_end(2);

    agc_args.action = arg_str1(NULL, NULL, "<action>", "freeze|unfreeze|set");
    agc_args.ir  = arg_int0(NULL, NULL, "IR_MA", "IR current");
    agc_args.red = arg_int0(NULL, NULL, "RED_MA", "red current");
    agc_args.rf  = arg_int0(NULL, NULL, "RF", "TIA gain code 0..7");
    agc_args.end = arg_end(5);

    marker_args.id  = arg_int0(NULL, NULL, "[id]", "marker id (default 0)");
    marker_args.end = arg_end(2);

    st_args.mask = arg_str0(NULL, NULL, "[mask]", "test bitmask, 0/omit = all");
    st_args.end  = arg_end(2);

    tap_args.n   = arg_int1(NULL, NULL, "<n>", "frames to capture (1..2000)");
    tap_args.end = arg_end(2);

    bonds_args.action = arg_str1(NULL, NULL, "<action>", "clear");
    bonds_args.end    = arg_end(2);

    freset_args.confirm = arg_str1(NULL, NULL, "CONFIRM", "literal CONFIRM");
    freset_args.end     = arg_end(2);

    core_args.action = arg_str1(NULL, NULL, "<action>", "info|erase");
    core_args.end    = arg_end(2);

    const esp_console_cmd_t cmds[] = {
        { .command = "ver", .help = "firmware/protocol versions",
          .func = cmd_ver },
        { .command = "stats", .help = "diagnostic counters + heap",
          .func = cmd_stats },
        { .command = "batt", .help = "battery mV/%, charger state",
          .func = cmd_batt },
        { .command = "state", .help = "system state snapshot",
          .func = cmd_state },
        { .command = "knob", .help = "knob list|get|set|save|reset [nvs]",
          .func = cmd_knob, .argtable = &knob_args },
        { .command = "rate", .help = "set PPG sample rate (sps)",
          .func = cmd_rate, .argtable = &rate_args },
        { .command = "agc", .help = "agc freeze|unfreeze|set IR RED RF",
          .func = cmd_agc, .argtable = &agc_args },
        { .command = "marker", .help = "emit an event marker",
          .func = cmd_marker, .argtable = &marker_args },
        { .command = "selftest", .help = "run self-test, print PASS/FAIL table",
          .func = cmd_selftest, .argtable = &st_args },
        { .command = "tap", .help = "print N raw frames as CSV",
          .func = cmd_tap, .argtable = &tap_args },
        { .command = "bonds", .help = "bonds clear — wipe BLE bond store",
          .func = cmd_bonds, .argtable = &bonds_args },
        { .command = "reboot", .help = "reboot via sys_task",
          .func = cmd_reboot },
        { .command = "off", .help = "power off (deep sleep) via sys_task",
          .func = cmd_off },
        { .command = "factory-reset", .help = "factory-reset CONFIRM",
          .func = cmd_factory_reset, .argtable = &freset_args },
        { .command = "coredump", .help = "coredump info|erase",
          .func = cmd_coredump, .argtable = &core_args },
    };
    for (size_t i = 0; i < sizeof cmds / sizeof cmds[0]; i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }
    esp_console_register_help_command();
}

esp_err_t console_init(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t rc = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    rc.prompt = "narbis>";
    rc.max_cmdline_length = 160;
    rc.task_priority = 4;             /* plan task table: console prio 4 */
    rc.task_stack_size = 6144;        /* argtable + selftest table print */

    esp_console_dev_usb_serial_jtag_config_t hw =
        ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    esp_err_t err = esp_console_new_repl_usb_serial_jtag(&hw, &rc, &repl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "REPL create failed: %s", esp_err_to_name(err));
        return err;
    }
    register_all();
    err = esp_console_start_repl(repl);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "REPL up on USB-Serial-JTAG");
    }
    return err;
}
