# Pure C component: no ESP-IDF dependencies by design (dual-compiled for
# host tests by test_host/run_tests.sh with mingw gcc).
# Sources are listed explicitly: ESP-IDF expands component CMakeLists in
# script mode where CONFIGURE_DEPENDS globs are unavailable, and explicit
# lists make a missing file a loud configure error.
idf_component_register(
    SRCS
        src/control_dispatch.c
        src/crc32.c
        src/nc_acq_policy.c
        src/nc_agc.c
        src/nc_batt_curve.c
        src/nc_button.c
        src/nc_dsp.c
        src/nc_gate.c
        src/nc_ibi.c
        src/nc_knobs.c
        src/nc_timesync.c
        src/nc_wear.c
        src/proto_encode.c
    INCLUDE_DIRS include)
