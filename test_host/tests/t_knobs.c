/*
 * t_knobs.c — knob registry invariants (table hygiene is a wire contract:
 * ids are permanent, names feed auto-built tuning UIs) plus the set/get
 * API semantics of nc_knobs.c.
 */
#include "tst.h"
#include "narbis/nc_knobs.h"

TST_SUITE("knobs");

/* ------------------------------------------------------------------ */
static void test_table_shape(void)
{
    CHECK(NC_KNOB_COUNT >= 60);
    CHECK(nc_knob_desc(0) != NULL);
    CHECK(nc_knob_desc(NC_KNOB_COUNT) == NULL);
    CHECK(nc_knob_desc((size_t)-1) == NULL);
}

static void test_ids_unique_ascending(void)
{
    /* strictly ascending in table order implies uniqueness AND lets
     * discovery chunks be walked with a monotonic-id check client-side */
    for (int i = 1; i < NC_KNOB_COUNT; i++) {
        CHECK(nc_knob_desc(i)->id > nc_knob_desc(i - 1)->id);
    }
    for (int i = 0; i < NC_KNOB_COUNT; i++) {
        CHECK_EQ(nc_knob_index_of(nc_knob_desc(i)->id), i);
    }
    CHECK_EQ(nc_knob_index_of(0x0000), -1);
    CHECK_EQ(nc_knob_index_of(0xFFFF), -1);
}

static void test_ranges_and_defaults(void)
{
    for (int i = 0; i < NC_KNOB_COUNT; i++) {
        const nc_knob_desc_t *d = nc_knob_desc(i);
        CHECK(d->min <= d->max);
        CHECK(d->def >= d->min);
        CHECK(d->def <= d->max);
    }
}

static void test_names(void)
{
    for (int i = 0; i < NC_KNOB_COUNT; i++) {
        const nc_knob_desc_t *d = nc_knob_desc(i);
        size_t len = strlen(d->name);
        CHECK(len > 0);
        CHECK(len <= 22);
        for (size_t k = 0; k < len; k++) {
            char c = d->name[k];
            CHECK((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_');
        }
        CHECK(d->unit != NULL);   /* may be "", never NULL (wire needs strlen) */
    }
}

static void test_type_ranges(void)
{
    for (int i = 0; i < NC_KNOB_COUNT; i++) {
        const nc_knob_desc_t *d = nc_knob_desc(i);
        switch (d->type) {
        case NC_KNOB_BOOL:
            CHECK_EQ(d->min, 0);
            CHECK_EQ(d->max, 1);
            break;
        case NC_KNOB_U8:
            CHECK(d->min >= 0);
            CHECK(d->max <= 255);
            break;
        case NC_KNOB_U16:
            CHECK(d->min >= 0);
            CHECK(d->max <= 65535);
            break;
        case NC_KNOB_I32:
            break;
        default:
            CHECK(0 && "unknown knob type");
            break;
        }
    }
}

/* ------------------------------------------------------------------ */
static int cb_count;
static int cb_last_idx;
static int32_t cb_last_val;

static void change_cb(int idx, int32_t v)
{
    cb_count++;
    cb_last_idx = idx;
    cb_last_val = v;
}

static void test_set_get_semantics(void)
{
    nc_knobs_init();

    /* init loads defaults */
    for (int i = 0; i < NC_KNOB_COUNT; i++) {
        CHECK_EQ(nc_knob_get(i), nc_knob_desc(i)->def);
        CHECK_EQ(nc_knob_get_id(nc_knob_desc(i)->id), nc_knob_desc(i)->def);
    }
    CHECK_EQ(nc_knob_get_id(0xABCD), 0);   /* unknown id reads as 0 */

    /* boundary acceptance + off-by-one rejection for every knob (all
     * mins >= 0 and maxes < INT32_MAX in the table, so +-1 can't wrap) */
    for (int i = 0; i < NC_KNOB_COUNT; i++) {
        const nc_knob_desc_t *d = nc_knob_desc(i);
        nc_ctrl_status_t want = (d->flags & NC_KF_REBOOT) ? NC_ST_NEEDS_RESTART
                                                          : NC_ST_OK;
        CHECK_EQ(nc_knob_set_id(d->id, d->min), want);
        CHECK_EQ(nc_knob_get(i), d->min);
        CHECK_EQ(nc_knob_set_id(d->id, d->max), want);
        CHECK_EQ(nc_knob_get(i), d->max);
        CHECK_EQ(nc_knob_set_id(d->id, d->min - 1), NC_ST_OUT_OF_RANGE);
        CHECK_EQ(nc_knob_set_id(d->id, d->max + 1), NC_ST_OUT_OF_RANGE);
        CHECK_EQ(nc_knob_get(i), d->max);   /* rejects leave value alone */
        CHECK_EQ(nc_knob_set_id(d->id, d->def), want);
    }

    CHECK_EQ(nc_knob_set_id(0xABCD, 0), NC_ST_BAD_PARAM);
}

static void test_change_cb(void)
{
    nc_knobs_init();
    nc_knobs_set_change_cb(change_cb);
    cb_count = 0;

    int idx = nc_knob_index_of(0x0403);   /* agc_target_pct, 20..80 */
    CHECK(idx >= 0);

    CHECK_EQ(nc_knob_set_id(0x0403, 42), NC_ST_OK);
    CHECK_EQ(cb_count, 1);
    CHECK_EQ(cb_last_idx, idx);
    CHECK_EQ(cb_last_val, 42);

    /* rejected writes must not fire the hook */
    CHECK_EQ(nc_knob_set_id(0x0403, 81), NC_ST_OUT_OF_RANGE);
    CHECK_EQ(nc_knob_set_id(0xABCD, 1), NC_ST_BAD_PARAM);
    CHECK_EQ(cb_count, 1);

    /* NEEDS_RESTART is still a store: hook fires */
    CHECK_EQ(nc_knob_set_id(0x0203, 0), NC_ST_NEEDS_RESTART);   /* hrs_en, KPR */
    CHECK_EQ(cb_count, 2);
    CHECK_EQ(cb_last_idx, nc_knob_index_of(0x0203));
    CHECK_EQ(cb_last_val, 0);

    nc_knobs_set_change_cb(NULL);
    CHECK_EQ(nc_knob_set_id(0x0403, 50), NC_ST_OK);   /* no cb, no crash */
    CHECK_EQ(cb_count, 2);
    nc_knobs_init();
}

static void test_reboot_flagged_knobs(void)
{
    nc_knobs_init();
    int reboot_knobs = 0;
    for (int i = 0; i < NC_KNOB_COUNT; i++) {
        const nc_knob_desc_t *d = nc_knob_desc(i);
        if (d->flags & NC_KF_REBOOT) {
            reboot_knobs++;
            CHECK_EQ(nc_knob_set_id(d->id, d->def), NC_ST_NEEDS_RESTART);
        } else {
            CHECK_EQ(nc_knob_set_id(d->id, d->def), NC_ST_OK);
        }
    }
    CHECK(reboot_knobs >= 2);   /* open_pairing + hrs_en at minimum */
}

/* ------------------------------------------------------------------ */
int main(void)
{
    TST_RUN(test_table_shape);
    TST_RUN(test_ids_unique_ascending);
    TST_RUN(test_ranges_and_defaults);
    TST_RUN(test_names);
    TST_RUN(test_type_ranges);
    TST_RUN(test_set_get_semantics);
    TST_RUN(test_change_cb);
    TST_RUN(test_reboot_flagged_knobs);
    TST_REPORT();
}
