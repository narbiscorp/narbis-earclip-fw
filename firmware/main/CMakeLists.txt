file(GLOB main_srcs ${CMAKE_CURRENT_SOURCE_DIR}/*.c)
idf_component_register(
    SRCS ${main_srcs}
    INCLUDE_DIRS "."
    REQUIRES narbis_core
    PRIV_REQUIRES bt nvs_flash esp_adc esp_timer driver esp_pm
                  app_update esp_app_format console vfs
                  esp_hw_support esp_system spi_flash)
