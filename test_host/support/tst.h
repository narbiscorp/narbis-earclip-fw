/*
 * tst.h — minimal host test harness (mingw gcc).
 * Each suite is one t_*.c defining TST_SUITE("name") and test functions
 * registered with TST_RUN(fn). Nonzero exit on any failure.
 */
#pragma once
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

static int tst_pass_, tst_fail_;
static const char *tst_cur_;

#define TST_SUITE(name) static const char *tst_suite_ = name

#define CHECK(cond) do { \
    if (cond) { tst_pass_++; } else { tst_fail_++; \
        fprintf(stderr, "FAIL %s:%d [%s] %s\n", __FILE__, __LINE__, tst_cur_, #cond); } \
} while (0)

#define CHECK_EQ(a, b) do { \
    long long va_ = (long long)(a), vb_ = (long long)(b); \
    if (va_ == vb_) { tst_pass_++; } else { tst_fail_++; \
        fprintf(stderr, "FAIL %s:%d [%s] %s == %s (%lld != %lld)\n", \
                __FILE__, __LINE__, tst_cur_, #a, #b, va_, vb_); } \
} while (0)

#define CHECK_MEMEQ(a, b, n) do { \
    if (memcmp((a), (b), (n)) == 0) { tst_pass_++; } else { tst_fail_++; \
        fprintf(stderr, "FAIL %s:%d [%s] memcmp(%s, %s, %s)\n", \
                __FILE__, __LINE__, tst_cur_, #a, #b, #n); } \
} while (0)

#define TST_RUN(fn) do { tst_cur_ = #fn; fn(); } while (0)

#define TST_REPORT() do { \
    printf("%-24s %d passed, %d failed\n", tst_suite_, tst_pass_, tst_fail_); \
    return tst_fail_ ? 1 : 0; \
} while (0)
