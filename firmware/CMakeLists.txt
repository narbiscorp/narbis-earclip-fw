cmake_minimum_required(VERSION 3.16)

# Firmware version = git describe (exposed via esp_app_desc -> DIS Firmware Revision).
# Only v<digit> tags count: the OTA release tags (functest-v0.3.0-NN-g...,
# tools/publish_ota_release.ps1) would otherwise become the base of every
# later version string.
execute_process(
    COMMAND git describe --tags --dirty --always --match "v[0-9]*"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE NARBIS_GIT_VER
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
if(NOT NARBIS_GIT_VER)
    set(NARBIS_GIT_VER "v0.0.0-nogit")
endif()
set(PROJECT_VER ${NARBIS_GIT_VER})

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "_v2": the app descriptor's project_name is the OTA image identity
# (dfu_legacy.c dfu_image_hdr_ok, the web updater). The V1 earclip
# firmware (edge-earclip) is "narbis_earclip" on the same chip.
project(narbis_earclip_v2)
