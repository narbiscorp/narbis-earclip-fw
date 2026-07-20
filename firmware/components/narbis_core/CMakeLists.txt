file(GLOB narbis_core_srcs ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)
idf_component_register(
    SRCS ${narbis_core_srcs}
    INCLUDE_DIRS include)
# Pure C component: no ESP-IDF dependencies by design (dual-compiled for
# host tests by test_host/run_tests.sh with mingw gcc).
