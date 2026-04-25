#include "tcache.h"
#include <stdlib.h>
#include <string.h>

#define LINE_SIZE 64
#define OFFSET_BITS 6

#define L1I_WAYS HW11_L1_INSTR_ASSOC
#define L1I_SETS (HW11_L1_SIZE / (L1I_WAYS * LINE_SIZE))
#define L1I_IDX_BITS 9

#define L1D_WAYS HW11_L1_DATA_ASSOC
#define L1D_SETS (HW11_L1_SIZE / (L1D_WAYS * LINE_SIZE))
#define L1D_IDX_BITS 8

#define L2_WAYS HW11_L2_ASSOC
#define L2_SETS (HW11_L2_SIZE / (L2_WAYS * LINE_SIZE))
#define L2_IDX_BITS 13

typedef struct {
    cache_line_t lines[4];
    uint64_t lru[4];
} cache_set_t;

static cache_set_t l1i_cache[L1I_SETS];
static cache_set_t l1d_cache[L1D_SETS];
static cache_set_t l2_cache[L2_SETS];

static cache_stats_t l1i_stats, l1d_stats, l2_stats;
static replacement_policy_e policy;
static uint64_t tick;

static uint64_t get_offset(uint64_t addr) { return addr & 63; }
static uint64_t get_l1i_idx(uint64_t addr) { return (addr >> OFFSET_BITS) & (L1I_SETS - 1); }
static uint64_t get_l1i_tag(uint64_t addr) { return addr >> (OFFSET_BITS + L1I_IDX_BITS); }
static uint64_t get_l1d_idx(uint64_t addr) { return (addr >> OFFSET_BITS) & (L1D_SETS - 1); }
static uint64_t get_l1d_tag(uint64_t addr) { return addr >> (OFFSET_BITS + L1D_IDX_BITS); }
static uint64_t get_l2_idx(uint64_t addr) { return (addr >> OFFSET_BITS) & (L2_SETS - 1); }
static uint64_t get_l2_tag(uint64_t addr) { return addr >> (OFFSET_BITS + L2_IDX_BITS); }
static uint64_t line_base(uint64_t addr) { return addr & ~(uint64_t)63; }

static uint64_t make_l1i_addr(uint64_t idx, uint64_t tag) {
    return (tag << (OFFSET_BITS + L1I_IDX_BITS)) | (idx << OFFSET_BITS);
}
static uint64_t make_l1d_addr(uint64_t idx, uint64_t tag) {
    return (tag << (OFFSET_BITS + L1D_IDX_BITS)) | (idx << OFFSET_BITS);
}
static uint64_t make_l2_addr(uint64_t idx, uint64_t tag) {
    return (tag << (OFFSET_BITS + L2_IDX_BITS)) | (idx << OFFSET_BITS);
}

static int pick_victim(cache_set_t *set, int ways) {
    int i;
    for (i = 0; i < ways; i++)
        if (!set->lines[i].valid) return i;

    if (policy == LRU) {
        int v = 0;
        for (i = 1; i < ways; i++)
            if (set->lru[i] < set->lru[v]) v = i;
        return v;
    }
    return rand() % ways;
}

static int l2_find(uint64_t addr) {
    uint64_t idx = get_l2_idx(addr);
    uint64_t tag = get_l2_tag(addr);
    int i;
    for (i = 0; i < L2_WAYS; i++)
        if (l2_cache[idx].lines[i].valid && l2_cache[idx].lines[i].tag == tag)
            return i;
    return -1;
}

static void wb_l1i_to_l2(uint64_t base, uint8_t *data) {
    l2_stats.accesses++;
    int w = l2_find(base);
    if (w >= 0) {
        uint64_t idx = get_l2_idx(base);
        memcpy(l2_cache[idx].lines[w].data, data, LINE_SIZE);
        l2_cache[idx].lines[w].modified = 1;
        l2_cache[idx].lru[w] = tick++;
    } else {
        int b;
        for (b = 0; b < LINE_SIZE; b++)
            write_memory(base + b, data[b]);
    }
}

static void wb_l1d_to_l2(uint64_t base, uint8_t *data) {
    l2_stats.accesses++;
    int w = l2_find(base);
    if (w >= 0) {
        uint64_t idx = get_l2_idx(base);
        memcpy(l2_cache[idx].lines[w].data, data, LINE_SIZE);
        l2_cache[idx].lines[w].modified = 1;
        l2_cache[idx].lru[w] = tick++;
    } else {
        int b;
        for (b = 0; b < LINE_SIZE; b++)
            write_memory(base + b, data[b]);
    }
}

// called when L2 is evicting a line - pull dirty L1i data into L2 first
static void l2_evict_collect_l1i(uint64_t base, cache_line_t *l2line) {
    uint64_t idx = get_l1i_idx(base);
    uint64_t tag = get_l1i_tag(base);
    int i;
    for (i = 0; i < L1I_WAYS; i++) {
        if (l1i_cache[idx].lines[i].valid && l1i_cache[idx].lines[i].tag == tag) {
            if (l1i_cache[idx].lines[i].modified) {
                l2_stats.accesses++;
                memcpy(l2line->data, l1i_cache[idx].lines[i].data, LINE_SIZE);
                l2line->modified = 1;
            }
            l1i_cache[idx].lines[i].valid = 0;
            l1i_cache[idx].lines[i].modified = 0;
        }
    }
}

static void l2_evict_collect_l1d(uint64_t base, cache_line_t *l2line) {
    uint64_t idx = get_l1d_idx(base);
    uint64_t tag = get_l1d_tag(base);
    int i;
    for (i = 0; i < L1D_WAYS; i++) {
        if (l1d_cache[idx].lines[i].valid && l1d_cache[idx].lines[i].tag == tag) {
            if (l1d_cache[idx].lines[i].modified) {
                l2_stats.accesses++;
                memcpy(l2line->data, l1d_cache[idx].lines[i].data, LINE_SIZE);
                l2line->modified = 1;
            }
            l1d_cache[idx].lines[i].valid = 0;
            l1d_cache[idx].lines[i].modified = 0;
        }
    }
}

// make sure addr is in L2, fetch from memory if not. caller already counted the access
static int l2_ensure(uint64_t addr) {
    int w = l2_find(addr);
    if (w >= 0) {
        l2_cache[get_l2_idx(addr)].lru[w] = tick++;
        return w;
    }

    l2_stats.misses++;
    uint64_t idx = get_l2_idx(addr);
    uint64_t tag = get_l2_tag(addr);
    uint64_t base = line_base(addr);

    int v = pick_victim(&l2_cache[idx], L2_WAYS);

    if (l2_cache[idx].lines[v].valid) {
        uint64_t evict_base = make_l2_addr(idx, l2_cache[idx].lines[v].tag);
        l2_evict_collect_l1i(evict_base, &l2_cache[idx].lines[v]);
        l2_evict_collect_l1d(evict_base, &l2_cache[idx].lines[v]);
        if (l2_cache[idx].lines[v].modified) {
            int b;
            for (b = 0; b < LINE_SIZE; b++)
                write_memory(evict_base + b, l2_cache[idx].lines[v].data[b]);
        }
    }

    int b;
    for (b = 0; b < LINE_SIZE; b++)
        l2_cache[idx].lines[v].data[b] = read_memory(base + b);
    l2_cache[idx].lines[v].valid = 1;
    l2_cache[idx].lines[v].modified = 0;
    l2_cache[idx].lines[v].tag = tag;
    l2_cache[idx].lru[v] = tick++;
    return v;
}

static int l1i_load(uint64_t addr) {
    uint64_t idx = get_l1i_idx(addr);
    uint64_t tag = get_l1i_tag(addr);

    int v = pick_victim(&l1i_cache[idx], L1I_WAYS);
    if (l1i_cache[idx].lines[v].valid && l1i_cache[idx].lines[v].modified)
        wb_l1i_to_l2(make_l1i_addr(idx, l1i_cache[idx].lines[v].tag), l1i_cache[idx].lines[v].data);

    int l2w = l2_find(addr);
    uint64_t l2idx = get_l2_idx(addr);
    l1i_cache[idx].lines[v].valid = 1;
    l1i_cache[idx].lines[v].modified = 0;
    l1i_cache[idx].lines[v].tag = tag;
    memcpy(l1i_cache[idx].lines[v].data, l2_cache[l2idx].lines[l2w].data, LINE_SIZE);
    l1i_cache[idx].lru[v] = tick++;
    return v;
}

static int l1d_load(uint64_t addr) {
    uint64_t idx = get_l1d_idx(addr);
    uint64_t tag = get_l1d_tag(addr);

    int v = pick_victim(&l1d_cache[idx], L1D_WAYS);
    if (l1d_cache[idx].lines[v].valid && l1d_cache[idx].lines[v].modified)
        wb_l1d_to_l2(make_l1d_addr(idx, l1d_cache[idx].lines[v].tag), l1d_cache[idx].lines[v].data);

    int l2w = l2_find(addr);
    uint64_t l2idx = get_l2_idx(addr);
    l1d_cache[idx].lines[v].valid = 1;
    l1d_cache[idx].lines[v].modified = 0;
    l1d_cache[idx].lines[v].tag = tag;
    memcpy(l1d_cache[idx].lines[v].data, l2_cache[l2idx].lines[l2w].data, LINE_SIZE);
    l1d_cache[idx].lru[v] = tick++;
    return v;
}

// flush dirty l1i to l2 without invalidating (used on reads from l1d)
static void flush_l1i(uint64_t addr) {
    uint64_t base = line_base(addr);
    uint64_t idx = get_l1i_idx(base);
    uint64_t tag = get_l1i_tag(base);
    int i;
    for (i = 0; i < L1I_WAYS; i++) {
        if (l1i_cache[idx].lines[i].valid && l1i_cache[idx].lines[i].tag == tag
                && l1i_cache[idx].lines[i].modified) {
            wb_l1i_to_l2(base, l1i_cache[idx].lines[i].data);
            l1i_cache[idx].lines[i].modified = 0;
        }
    }
}

static void flush_l1d(uint64_t addr) {
    uint64_t base = line_base(addr);
    uint64_t idx = get_l1d_idx(base);
    uint64_t tag = get_l1d_tag(base);
    int i;
    for (i = 0; i < L1D_WAYS; i++) {
        if (l1d_cache[idx].lines[i].valid && l1d_cache[idx].lines[i].tag == tag
                && l1d_cache[idx].lines[i].modified) {
            wb_l1d_to_l2(base, l1d_cache[idx].lines[i].data);
            l1d_cache[idx].lines[i].modified = 0;
        }
    }
}

// flush dirty + invalidate (used on writes to the other cache)
static void inval_l1i(uint64_t addr) {
    uint64_t base = line_base(addr);
    uint64_t idx = get_l1i_idx(base);
    uint64_t tag = get_l1i_tag(base);
    int i;
    for (i = 0; i < L1I_WAYS; i++) {
        if (l1i_cache[idx].lines[i].valid && l1i_cache[idx].lines[i].tag == tag) {
            if (l1i_cache[idx].lines[i].modified)
                wb_l1i_to_l2(base, l1i_cache[idx].lines[i].data);
            l1i_cache[idx].lines[i].valid = 0;
            l1i_cache[idx].lines[i].modified = 0;
        }
    }
}

static void inval_l1d(uint64_t addr) {
    uint64_t base = line_base(addr);
    uint64_t idx = get_l1d_idx(base);
    uint64_t tag = get_l1d_tag(base);
    int i;
    for (i = 0; i < L1D_WAYS; i++) {
        if (l1d_cache[idx].lines[i].valid && l1d_cache[idx].lines[i].tag == tag) {
            if (l1d_cache[idx].lines[i].modified)
                wb_l1d_to_l2(base, l1d_cache[idx].lines[i].data);
            l1d_cache[idx].lines[i].valid = 0;
            l1d_cache[idx].lines[i].modified = 0;
        }
    }
}

void init_cache(replacement_policy_e p) {
    policy = p;
    tick = 0;
    memset(l1i_cache, 0, sizeof(l1i_cache));
    memset(l1d_cache, 0, sizeof(l1d_cache));
    memset(l2_cache, 0, sizeof(l2_cache));
    memset(&l1i_stats, 0, sizeof(l1i_stats));
    memset(&l1d_stats, 0, sizeof(l1d_stats));
    memset(&l2_stats, 0, sizeof(l2_stats));
}

uint8_t read_cache(uint64_t mem_addr, mem_type_t type) {
    int i, w;
    uint64_t idx, tag;

    if (type == INSTR) {
        flush_l1d(mem_addr);

        l1i_stats.accesses++;
        idx = get_l1i_idx(mem_addr);
        tag = get_l1i_tag(mem_addr);
        for (i = 0; i < L1I_WAYS; i++) {
            if (l1i_cache[idx].lines[i].valid && l1i_cache[idx].lines[i].tag == tag) {
                l1i_cache[idx].lru[i] = tick++;
                return l1i_cache[idx].lines[i].data[get_offset(mem_addr)];
            }
        }
        l1i_stats.misses++;
        l2_stats.accesses++;
        l2_ensure(mem_addr);
        w = l1i_load(mem_addr);
        return l1i_cache[idx].lines[w].data[get_offset(mem_addr)];
    } else {
        flush_l1i(mem_addr);

        l1d_stats.accesses++;
        idx = get_l1d_idx(mem_addr);
        tag = get_l1d_tag(mem_addr);
        for (i = 0; i < L1D_WAYS; i++) {
            if (l1d_cache[idx].lines[i].valid && l1d_cache[idx].lines[i].tag == tag) {
                l1d_cache[idx].lru[i] = tick++;
                return l1d_cache[idx].lines[i].data[get_offset(mem_addr)];
            }
        }
        l1d_stats.misses++;
        l2_stats.accesses++;
        l2_ensure(mem_addr);
        w = l1d_load(mem_addr);
        return l1d_cache[idx].lines[w].data[get_offset(mem_addr)];
    }
}

void write_cache(uint64_t mem_addr, uint8_t value, mem_type_t type) {
    int i, w;
    uint64_t idx, tag;

    if (type == INSTR) {
        inval_l1d(mem_addr);

        l1i_stats.accesses++;
        idx = get_l1i_idx(mem_addr);
        tag = get_l1i_tag(mem_addr);
        for (i = 0; i < L1I_WAYS; i++) {
            if (l1i_cache[idx].lines[i].valid && l1i_cache[idx].lines[i].tag == tag) {
                l1i_cache[idx].lines[i].data[get_offset(mem_addr)] = value;
                l1i_cache[idx].lines[i].modified = 1;
                l1i_cache[idx].lru[i] = tick++;
                return;
            }
        }
        l1i_stats.misses++;
        l2_stats.accesses++;
        l2_ensure(mem_addr);
        w = l1i_load(mem_addr);
        l1i_cache[idx].lines[w].data[get_offset(mem_addr)] = value;
        l1i_cache[idx].lines[w].modified = 1;
    } else {
        inval_l1i(mem_addr);

        l1d_stats.accesses++;
        idx = get_l1d_idx(mem_addr);
        tag = get_l1d_tag(mem_addr);
        for (i = 0; i < L1D_WAYS; i++) {
            if (l1d_cache[idx].lines[i].valid && l1d_cache[idx].lines[i].tag == tag) {
                l1d_cache[idx].lines[i].data[get_offset(mem_addr)] = value;
                l1d_cache[idx].lines[i].modified = 1;
                l1d_cache[idx].lru[i] = tick++;
                return;
            }
        }
        l1d_stats.misses++;
        l2_stats.accesses++;
        l2_ensure(mem_addr);
        w = l1d_load(mem_addr);
        l1d_cache[idx].lines[w].data[get_offset(mem_addr)] = value;
        l1d_cache[idx].lines[w].modified = 1;
    }
}

cache_stats_t get_l1_instr_stats() { return l1i_stats; }
cache_stats_t get_l1_data_stats() { return l1d_stats; }
cache_stats_t get_l2_stats() { return l2_stats; }

cache_line_t *get_l1_instr_cache_line(uint64_t mem_addr) {
    uint64_t idx = get_l1i_idx(mem_addr);
    uint64_t tag = get_l1i_tag(mem_addr);
    int i;
    for (i = 0; i < L1I_WAYS; i++)
        if (l1i_cache[idx].lines[i].valid && l1i_cache[idx].lines[i].tag == tag)
            return &l1i_cache[idx].lines[i];
    return NULL;
}

cache_line_t *get_l1_data_cache_line(uint64_t mem_addr) {
    uint64_t idx = get_l1d_idx(mem_addr);
    uint64_t tag = get_l1d_tag(mem_addr);
    int i;
    for (i = 0; i < L1D_WAYS; i++)
        if (l1d_cache[idx].lines[i].valid && l1d_cache[idx].lines[i].tag == tag)
            return &l1d_cache[idx].lines[i];
    return NULL;
}

cache_line_t *get_l2_cache_line(uint64_t mem_addr) {
    uint64_t idx = get_l2_idx(mem_addr);
    uint64_t tag = get_l2_tag(mem_addr);
    int i;
    for (i = 0; i < L2_WAYS; i++)
        if (l2_cache[idx].lines[i].valid && l2_cache[idx].lines[i].tag == tag)
            return &l2_cache[idx].lines[i];
    return NULL;
}
