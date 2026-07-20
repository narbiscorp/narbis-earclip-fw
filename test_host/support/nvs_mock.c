#include "nvs_mock.h"
#include <string.h>

#define MOCK_MAX 128

typedef struct { char key[16]; int32_t val; bool used; } ent_t;
static ent_t tab[MOCK_MAX];
static int commits;

void nvs_mock_reset(void) { memset(tab, 0, sizeof(tab)); commits = 0; }

static ent_t *find(const char *key)
{
    for (int i = 0; i < MOCK_MAX; i++) {
        if (tab[i].used && strcmp(tab[i].key, key) == 0) return &tab[i];
    }
    return NULL;
}

bool nvs_mock_read(const char *key, int32_t *out)
{
    ent_t *e = find(key);
    if (!e) return false;
    *out = e->val;
    return true;
}

void nvs_mock_write(const char *key, int32_t v)
{
    ent_t *e = find(key);
    if (!e) {
        for (int i = 0; i < MOCK_MAX; i++) {
            if (!tab[i].used) { e = &tab[i]; break; }
        }
        if (!e) return;
        e->used = true;
        strncpy(e->key, key, sizeof(e->key) - 1);
        e->key[sizeof(e->key) - 1] = 0;
    }
    e->val = v;
}

void nvs_mock_erase_key(const char *key)
{
    ent_t *e = find(key);
    if (e) e->used = false;
}

void nvs_mock_erase_all(void)
{
    for (int i = 0; i < MOCK_MAX; i++) tab[i].used = false;
}

size_t nvs_mock_count(void)
{
    size_t n = 0;
    for (int i = 0; i < MOCK_MAX; i++) if (tab[i].used) n++;
    return n;
}

int nvs_mock_commits(void) { return commits; }
void nvs_mock_commit(void) { commits++; }
