#include "knobs_nvs.h"
#include <stdio.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "narbis/nc_knobs.h"

static const char *TAG = "knobs_nvs";

#define KNOBS_NS       "knobs"
#define KNOBS_VER_KEY  "__ver"
#define KNOBS_SCHEMA   1

static void key_for(uint16_t id, char out[8])
{
    snprintf(out, 8, "k%04X", id);
}

/* Schema migrations: entries map old id -> new id (0 = tombstone, value
 * discarded). Append-only; runs when stored __ver < KNOBS_SCHEMA. */
typedef struct { uint16_t old_id, new_id; } knob_migration_t;
static const knob_migration_t migrations[] = {
    /* none yet — v1 */
    { 0, 0 }
};

esp_err_t knobs_nvs_load(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(KNOBS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "open failed (%s) — defaults only", esp_err_to_name(err));
        return err;
    }

    uint16_t ver = 0;
    if (nvs_get_u16(h, KNOBS_VER_KEY, &ver) != ESP_OK) ver = 0;
    if (ver != 0 && ver < KNOBS_SCHEMA) {
        for (const knob_migration_t *m = migrations; m->old_id; m++) {
            char okey[8];
            key_for(m->old_id, okey);
            int32_t v;
            if (nvs_get_i32(h, okey, &v) == ESP_OK) {
                nvs_erase_key(h, okey);
                if (m->new_id) {
                    char nkey[8];
                    key_for(m->new_id, nkey);
                    nvs_set_i32(h, nkey, v);
                }
            }
        }
        ESP_LOGI(TAG, "migrated knob schema %u -> %u", ver, KNOBS_SCHEMA);
    }

    int applied = 0, dropped = 0;
    nvs_iterator_t it = NULL;
    err = nvs_entry_find(NVS_DEFAULT_PART_NAME, KNOBS_NS, NVS_TYPE_I32, &it);
    while (err == ESP_OK) {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);
        unsigned id;
        if (sscanf(info.key, "k%04X", &id) == 1) {
            int32_t v;
            if (nvs_get_i32(h, info.key, &v) == ESP_OK) {
                nc_ctrl_status_t st = nc_knob_set_id((uint16_t)id, v);
                if (st == NC_ST_OK || st == NC_ST_NEEDS_RESTART) {
                    applied++;
                } else {
                    /* unknown/out-of-range (stale firmware value): drop it */
                    nvs_erase_key(h, info.key);
                    dropped++;
                }
            }
        }
        err = nvs_entry_next(&it);
    }
    nvs_release_iterator(it);

    nvs_set_u16(h, KNOBS_VER_KEY, KNOBS_SCHEMA);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "loaded %d persisted knobs (%d stale dropped)", applied, dropped);
    return ESP_OK;
}

esp_err_t knobs_nvs_save(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(KNOBS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    int written = 0, erased = 0;
    for (size_t i = 0; i < NC_KNOB_COUNT; i++) {
        const nc_knob_desc_t *d = nc_knob_desc(i);
        if (!(d->flags & NC_KF_PERSIST)) continue;
        char key[8];
        key_for(d->id, key);
        int32_t cur = nc_knob_val[i];
        if (cur != d->def) {
            int32_t stored;
            if (nvs_get_i32(h, key, &stored) != ESP_OK || stored != cur) {
                nvs_set_i32(h, key, cur);
                written++;
            }
        } else {
            if (nvs_erase_key(h, key) == ESP_OK) erased++;
        }
    }
    nvs_set_u16(h, KNOBS_VER_KEY, KNOBS_SCHEMA);
    err = nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "saved: %d deltas written, %d reverted-to-default erased", written, erased);
    return err;
}

esp_err_t knobs_nvs_reset(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(KNOBS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    err = nvs_erase_all(h);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}
