/*
 * main.c — boot orchestration.
 *
 * Init order (old spec §8): i2c_bus -> battery -> lis2dh12 -> afe4404 ->
 * power -> ble -> tasks. Each init logs; a sensor failure halts with a
 * clear error (no silent continue). Skeleton for now — modules land in
 * milestone order.
 */
#include <stdio.h>
#include "esp_log.h"
#include "esp_app_desc.h"
#include "board.h"
#include "narbis/nc_types.h"

static const char *TAG = "main";

void app_main(void)
{
    const esp_app_desc_t *app = esp_app_get_description();
    ESP_LOGI(TAG, "Narbis Edge Earclip fw %s (test_mode=%d)",
             app->version, NARBIS_TEST_MODE);
    ESP_LOGI(TAG, "default rate %u sps", nc_rate_sps(NC_RATE_100));
}
