idf_component_register(
    SRCS
        acq_afe.c
        acq_imu.c
        afe4404.c
        app_tasks.c
        battery.c
        ble_gatt.c
        ble_tx.c
        button.c
        charger.c
        console.c
        dfu_legacy.c
        diag.c
        dsp_task.c
        i2c_bus.c
        knobs_nvs.c
        lis2dh12.c
        main.c
        ota.c
        power.c
        selftest_hw.c
        sys_task.c
        test_led_ind.c
        test_ops.c
    INCLUDE_DIRS "."
    REQUIRES narbis_core
    PRIV_REQUIRES bt nvs_flash esp_adc esp_timer driver esp_pm
                  app_update esp_app_format console vfs
                  esp_hw_support esp_system spi_flash espcoredump)
