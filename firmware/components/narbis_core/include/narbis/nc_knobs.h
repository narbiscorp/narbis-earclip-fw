/*
 * nc_knobs.h — tunable-parameter registry (pure; persistence lives in
 * main/knobs_nvs.c, wire encoding in proto_encode.c/control_dispatch.c).
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "narbis/proto.h"
#include "narbis/knob_list.h"

/* Dense index enum: KNOB_<SYM>. Array order == KNOB_LIST order. */
enum {
#define X(SYM, id, name, type, min, max, def, unit, flags) KNOB_##SYM,
    KNOB_LIST(X)
#undef X
    NC_KNOB_COUNT
};

typedef struct {
    uint16_t    id;
    const char *name;
    uint8_t     type;    /* nc_knob_type_t */
    uint8_t     flags;   /* NC_KF_* */
    int32_t     min, max, def;
    const char *unit;
} nc_knob_desc_t;

/* RAM current values, index = KNOB_<SYM>. Direct reads are the hot-path
 * access pattern (single aligned word load — atomic on RV32 and x86). */
extern int32_t nc_knob_val[NC_KNOB_COUNT];

void nc_knobs_init(void);                       /* all -> defaults        */
const nc_knob_desc_t *nc_knob_desc(size_t idx); /* NULL if idx OOB        */
int nc_knob_index_of(uint16_t id);              /* -1 if unknown id       */

static inline int32_t nc_knob_get(int idx) { return nc_knob_val[idx]; }
int32_t nc_knob_get_id(uint16_t id);            /* 0 if unknown (log-free) */

/* Validates range; stores; returns NC_ST_OK, NC_ST_NEEDS_RESTART (stored,
 * effect deferred), NC_ST_OUT_OF_RANGE or NC_ST_BAD_PARAM (unknown id). */
nc_ctrl_status_t nc_knob_set_id(uint16_t id, int32_t v);

/* Single change hook (main routes it to sys_task; NULL ok). Called after
 * the value is stored, with the dense index. */
void nc_knobs_set_change_cb(void (*cb)(int idx, int32_t newval));
