#include "narbis/nc_knobs.h"

static const nc_knob_desc_t knob_tab[NC_KNOB_COUNT] = {
#define X(SYM, id_, name_, type_, min_, max_, def_, unit_, flags_) \
    { .id = id_, .name = name_, .type = type_, .flags = flags_,    \
      .min = min_, .max = max_, .def = def_, .unit = unit_ },
    KNOB_LIST(X)
#undef X
};

int32_t nc_knob_val[NC_KNOB_COUNT];

static void (*change_cb)(int idx, int32_t v);

void nc_knobs_init(void)
{
    for (int i = 0; i < NC_KNOB_COUNT; i++) {
        nc_knob_val[i] = knob_tab[i].def;
    }
}

const nc_knob_desc_t *nc_knob_desc(size_t idx)
{
    return (idx < NC_KNOB_COUNT) ? &knob_tab[idx] : (const nc_knob_desc_t *)0;
}

int nc_knob_index_of(uint16_t id)
{
    /* 62 entries, slow-path only: linear scan is fine. */
    for (int i = 0; i < NC_KNOB_COUNT; i++) {
        if (knob_tab[i].id == id) return i;
    }
    return -1;
}

int32_t nc_knob_get_id(uint16_t id)
{
    int i = nc_knob_index_of(id);
    return (i >= 0) ? nc_knob_val[i] : 0;
}

nc_ctrl_status_t nc_knob_set_id(uint16_t id, int32_t v)
{
    int i = nc_knob_index_of(id);
    if (i < 0) return NC_ST_BAD_PARAM;
    const nc_knob_desc_t *d = &knob_tab[i];
    if (v < d->min || v > d->max) return NC_ST_OUT_OF_RANGE;
    nc_knob_val[i] = v;
    if (change_cb) change_cb(i, v);
    return (d->flags & NC_KF_REBOOT) ? NC_ST_NEEDS_RESTART : NC_ST_OK;
}

void nc_knobs_set_change_cb(void (*cb)(int idx, int32_t v))
{
    change_cb = cb;
}
