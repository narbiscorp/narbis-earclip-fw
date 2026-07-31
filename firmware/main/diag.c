#include "diag.h"
#include <stdio.h>
#include <stdbool.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_core_dump.h"
#include "esp_heap_caps.h"

static const char *TAG = "diag";

diag_counters_t g_diag;

static esp_reset_reason_t boot_reason;

const char *diag_reset_reason_str(void)
{
    switch (boot_reason) {
    case ESP_RST_POWERON:   return "poweron";
    case ESP_RST_SW:        return "software";
    case ESP_RST_PANIC:     return "panic";
    case ESP_RST_INT_WDT:   return "int_wdt";
    case ESP_RST_TASK_WDT:  return "task_wdt";
    case ESP_RST_WDT:       return "other_wdt";
    case ESP_RST_DEEPSLEEP: return "deepsleep_wake";
    case ESP_RST_BROWNOUT:  return "brownout";
    case ESP_RST_USB:       return "usb";       /* esptool / USB re-plug */
    case ESP_RST_JTAG:      return "jtag";
    default:                return "unknown";
    }
}

bool diag_coredump_present(void)
{
    size_t addr, size;
    return esp_core_dump_image_get(&addr, &size) == ESP_OK;
}

esp_err_t diag_init(void)
{
    boot_reason = esp_reset_reason();
    ESP_LOGI(TAG, "reset reason: %s", diag_reset_reason_str());
    if (boot_reason == ESP_RST_PANIC && diag_coredump_present()) {
        ESP_LOGW(TAG, "coredump present in flash — fetch via console 'coredump'");
    }
    return ESP_OK;
}

void diag_console_dump(void)
{
    printf("reset:        %s\n", diag_reset_reason_str());
    printf("coredump:     %s\n", diag_coredump_present() ? "PRESENT" : "none");
    printf("i2c_err:      %lu\n", (unsigned long)g_diag.i2c_err);
    printf("notify_drop:  %lu\n", (unsigned long)g_diag.notify_drop);
    printf("ppg_overrun:  %lu\n", (unsigned long)g_diag.ppg_overrun);
    printf("fifo_overrun: %lu\n", (unsigned long)g_diag.fifo_overrun);
    printf("agc_steps:    %lu\n", (unsigned long)g_diag.agc_steps);
    printf("gate_trans:   %lu\n", (unsigned long)g_diag.gate_transitions);
    printf("q_hw ppg/ble: %lu/%lu\n",
           (unsigned long)g_diag.queue_hw_ppg, (unsigned long)g_diag.queue_hw_ble);
    printf("heap free:    %u (min %u)\n",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT));
}
