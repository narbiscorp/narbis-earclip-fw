cmake_minimum_required(VERSION 3.16)

# Firmware version = git describe (exposed via esp_app_desc -> DIS Firmware Revision).
execute_process(
    COMMAND git describe --tags --dirty --always
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE NARBIS_GIT_VER
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
if(NOT NARBIS_GIT_VER)
    set(NARBIS_GIT_VER "v0.0.0-nogit")
endif()
set(PROJECT_VER ${NARBIS_GIT_VER})

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(narbis_earclip)
