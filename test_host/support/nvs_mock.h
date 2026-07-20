/*
 * nvs_mock.h — RAM key/value store implementing the knobs persistence
 * backend for host tests (real NVS backend lives in main/knobs_nvs.c).
 * Keys are short strings ("kXXXX", "__ver"), values i32.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

void nvs_mock_reset(void);
bool nvs_mock_read(const char *key, int32_t *out);
void nvs_mock_write(const char *key, int32_t v);
void nvs_mock_erase_key(const char *key);
void nvs_mock_erase_all(void);
size_t nvs_mock_count(void);
int nvs_mock_commits(void);   /* commit call counter */
void nvs_mock_commit(void);
