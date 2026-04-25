#include "tcache.h"
#include <stdlib.h>
#include <string.h>

#define LINE_SIZE 64
#define OFFSET_BITS 6

#define L1_INSTR_WAYS     HW11_L1_INSTR_ASSOC
#define L1_INSTR_SETS     (HW11_L1_SIZE / (L1_INSTR_WAYS * LINE_SIZE))  /* 512 */
#define L1_INSTR_IDX_BITS 9

#define L1_DATA_WAYS      HW11_L1_DATA_ASSOC
#define L1_DATA_SETS      (HW11_L1_SIZE / (L1_DATA_WAYS * LINE_SIZE))   /* 256 */
#define L1_DATA_IDX_BITS  8

#define L2_WAYS           HW11_L2_ASSOC
#define L2_SETS           (HW11_L2_SIZE / (L2_WAYS * LINE_SIZE))        /* 8192 */
#define L2_IDX_BITS       13

typedef struct {
    cache_line_t lines[4];
    uint64_t     lru[4];
} cache_set_t;

static cache_set_t    l1i[L1_INSTR_SETS];
static cache_set_t    l1d[L1_DATA_SETS];
static cache_set_t    l2 [L2_SETS];

static cache_stats_t  st_l1i, st_l1d, st_l2;
static replacement_policy_e g_policy;
static uint64_t       g_tick;

/* ── address decomposition ── */
static uint64_t off    (uint64_t a) { return a & 63u; }
static uint64_t l1i_idx(uint64_t a) { return (a >> OFFSET_BITS) & (L1_INSTR_SETS - 1); }
static uint64_t l1i_tag(uint64_t a) { return a >> (OFFSET_BITS + L1_INSTR_IDX_BITS); }
static uint64_t l1d_idx(uint64_t a) { return (a >> OFFSET_BITS) & (L1_DATA_SETS - 1); }
static uint64_t l1d_tag(uint64_t a) { return a >> (OFFSET_BITS + L1_DATA_IDX_BITS); }
static uint64_t l2_idx (uint64_t a) { return (a >> OFFSET_BITS) & (L2_SETS - 1); }
static uint64_t l2_tag (uint64_t a) { return a >> (OFFSET_BITS + L2_IDX_BITS); }
static uint64_t baddr  (uint64_t a) { return a & ~(uint64_t)63u; }

static uint64_t rebuild_l1i(uint64_t idx, uint64_t tag) {
    return (tag << (OFFSET_BITS + L1_INSTR_IDX_BITS)) | (idx << OFFSET_BITS);
}
static uint64_t rebuild_l1d(uint64_t idx, uint64_t tag) {
    return (tag << (OFFSET_BITS + L1_DATA_IDX_BITS)) | (idx << OFFSET_BITS);
}
static uint64_t rebuild_l2(uint64_t idx, uint64_t tag) {
    return (tag << (OFFSET_BITS + L2_IDX_BITS)) | (idx << OFFSET_BITS);
}

/* ── victim selection ── */
static int pick_victim(cache_set_t *s, int ways) {
    for (int i = 0; i < ways; i++)
        if (!s->lines[i].valid) return i;
    if (g_policy == LRU) {
        int v = 0;
        for (int i = 1; i < ways; i++)
            if (s->lru[i] < s->lru[v]) v = i;
        return v;
    }
    return rand() % ways;
}

/* ── L2 lookup (no stats, no LRU update) ── */
static int l2_find_way(uint64_t a) {
    uint64_t si = l2_idx(a), t = l2_tag(a);
    for (int i = 0; i < L2_WAYS; i++)
        if (l2[si].lines[i].valid && l2[si].lines[i].tag == t) return i;
    return -1;
}

/* ── Write dirty L1i line back to L2. Counts as L2 access, updates L2 LRU. ── */
static void wb_l1i_to_l2(uint64_t ba, uint8_t *data) {
    st_l2.accesses++;
    int w = l2_find_way(ba);
    if (w >= 0) {
        uint64_t si = l2_idx(ba);
        memcpy(l2[si].lines[w].data, data, LINE_SIZE);
        l2[si].lines[w].modified = 1;
        l2[si].lru[w]            = g_tick++;  /* retouch: now MRU */
    } else {
        /* inclusive invariant broken; fall back to memory */
        for (int b = 0; b < LINE_SIZE; b++) write_memory(ba + b, data[b]);
    }
}

/* ── Write dirty L1d line back to L2. Counts as L2 access, updates L2 LRU. ── */
static void wb_l1d_to_l2(uint64_t ba, uint8_t *data) {
    st_l2.accesses++;
    int w = l2_find_way(ba);
    if (w >= 0) {
        uint64_t si = l2_idx(ba);
        memcpy(l2[si].lines[w].data, data, LINE_SIZE);
        l2[si].lines[w].modified = 1;
        l2[si].lru[w]            = g_tick++;  /* retouch: now MRU */
    } else {
        for (int b = 0; b < LINE_SIZE; b++) write_memory(ba + b, data[b]);
    }
}

/* ── During L2 eviction of the line at base `ba`:
      Write dirty L1i data into the soon-to-be-evicted L2 line (L2 access),
      then invalidate the L1i line. Non-dirty L1i lines are just invalidated. ── */
static void l2_evict_handle_l1i(uint64_t ba, cache_line_t *l2_line) {
    uint64_t si = l1i_idx(ba), t = l1i_tag(ba);
    for (int i = 0; i < L1_INSTR_WAYS; i++) {
        if (l1i[si].lines[i].valid && l1i[si].lines[i].tag == t) {
            if (l1i[si].lines[i].modified) {
                st_l2.accesses++;
                memcpy(l2_line->data, l1i[si].lines[i].data, LINE_SIZE);
                l2_line->modified = 1;
            }
            l1i[si].lines[i].valid    = 0;
            l1i[si].lines[i].modified = 0;
        }
    }
}

/* ── Same for L1d during L2 eviction. ── */
static void l2_evict_handle_l1d(uint64_t ba, cache_line_t *l2_line) {
    uint64_t si = l1d_idx(ba), t = l1d_tag(ba);
    for (int i = 0; i < L1_DATA_WAYS; i++) {
        if (l1d[si].lines[i].valid && l1d[si].lines[i].tag == t) {
            if (l1d[si].lines[i].modified) {
                st_l2.accesses++;
                memcpy(l2_line->data, l1d[si].lines[i].data, LINE_SIZE);
                l2_line->modified = 1;
            }
            l1d[si].lines[i].valid    = 0;
            l1d[si].lines[i].modified = 0;
        }
    }
}

/* ── Ensure addr is in L2, loading from memory if needed. Returns way.
      Caller must have already incremented st_l2.accesses. ── */
static int l2_ensure(uint64_t a) {
    int w = l2_find_way(a);
    if (w >= 0) {
        l2[l2_idx(a)].lru[w] = g_tick++;
        return w;
    }

    st_l2.misses++;
    uint64_t si = l2_idx(a), t = l2_tag(a), ba = baddr(a);
    int v = pick_victim(&l2[si], L2_WAYS);

    if (l2[si].lines[v].valid) {
        uint64_t eb = rebuild_l2(si, l2[si].lines[v].tag);
        /* Collect dirty L1 data into the L2 line (as L2 accesses) before evicting */
        l2_evict_handle_l1i(eb, &l2[si].lines[v]);
        l2_evict_handle_l1d(eb, &l2[si].lines[v]);
        /* Write back dirty L2 line to memory */
        if (l2[si].lines[v].modified)
            for (int b = 0; b < LINE_SIZE; b++)
                write_memory(eb + b, l2[si].lines[v].data[b]);
    }

    for (int b = 0; b < LINE_SIZE; b++)
        l2[si].lines[v].data[b] = read_memory(ba + b);
    l2[si].lines[v].valid    = 1;
    l2[si].lines[v].modified = 0;
    l2[si].lines[v].tag      = t;
    l2[si].lru[v]            = g_tick++;
    return v;
}

/* ── Load addr into L1i from L2 (L2 must already have it). Returns way. ── */
static int l1i_load(uint64_t a) {
    uint64_t si = l1i_idx(a), t = l1i_tag(a);
    int v = pick_victim(&l1i[si], L1_INSTR_WAYS);
    if (l1i[si].lines[v].valid && l1i[si].lines[v].modified)
        wb_l1i_to_l2(rebuild_l1i(si, l1i[si].lines[v].tag), l1i[si].lines[v].data);

    int l2w = l2_find_way(a);
    uint64_t l2si = l2_idx(a);
    l1i[si].lines[v].valid    = 1;
    l1i[si].lines[v].modified = 0;
    l1i[si].lines[v].tag      = t;
    memcpy(l1i[si].lines[v].data, l2[l2si].lines[l2w].data, LINE_SIZE);
    l1i[si].lru[v] = g_tick++;
    return v;
}

/* ── Load addr into L1d from L2 (L2 must already have it). Returns way. ── */
static int l1d_load(uint64_t a) {
    uint64_t si = l1d_idx(a), t = l1d_tag(a);
    int v = pick_victim(&l1d[si], L1_DATA_WAYS);
    if (l1d[si].lines[v].valid && l1d[si].lines[v].modified)
        wb_l1d_to_l2(rebuild_l1d(si, l1d[si].lines[v].tag), l1d[si].lines[v].data);

    int l2w = l2_find_way(a);
    uint64_t l2si = l2_idx(a);
    l1d[si].lines[v].valid    = 1;
    l1d[si].lines[v].modified = 0;
    l1d[si].lines[v].tag      = t;
    memcpy(l1d[si].lines[v].data, l2[l2si].lines[l2w].data, LINE_SIZE);
    l1d[si].lru[v] = g_tick++;
    return v;
}

/* ── Flush dirty L1i to L2 without invalidating (for READ coherency). ── */
static void coherency_flush_l1i(uint64_t a) {
    uint64_t ba = baddr(a), si = l1i_idx(ba), t = l1i_tag(ba);
    for (int i = 0; i < L1_INSTR_WAYS; i++)
        if (l1i[si].lines[i].valid && l1i[si].lines[i].tag == t && l1i[si].lines[i].modified) {
            wb_l1i_to_l2(ba, l1i[si].lines[i].data);
            l1i[si].lines[i].modified = 0;
        }
}

/* ── Flush dirty L1d to L2 without invalidating (for READ coherency). ── */
static void coherency_flush_l1d(uint64_t a) {
    uint64_t ba = baddr(a), si = l1d_idx(ba), t = l1d_tag(ba);
    for (int i = 0; i < L1_DATA_WAYS; i++)
        if (l1d[si].lines[i].valid && l1d[si].lines[i].tag == t && l1d[si].lines[i].modified) {
            wb_l1d_to_l2(ba, l1d[si].lines[i].data);
            l1d[si].lines[i].modified = 0;
        }
}

/* ── Flush dirty L1i to L2 AND INVALIDATE it (for WRITE coherency). ── */
static void coherency_inval_l1i(uint64_t a) {
    uint64_t ba = baddr(a), si = l1i_idx(ba), t = l1i_tag(ba);
    for (int i = 0; i < L1_INSTR_WAYS; i++)
        if (l1i[si].lines[i].valid && l1i[si].lines[i].tag == t) {
            if (l1i[si].lines[i].modified)
                wb_l1i_to_l2(ba, l1i[si].lines[i].data);
            l1i[si].lines[i].valid    = 0;
            l1i[si].lines[i].modified = 0;
        }
}

/* ── Flush dirty L1d to L2 AND INVALIDATE it (for WRITE coherency). ── */
static void coherency_inval_l1d(uint64_t a) {
    uint64_t ba = baddr(a), si = l1d_idx(ba), t = l1d_tag(ba);
    for (int i = 0; i < L1_DATA_WAYS; i++)
        if (l1d[si].lines[i].valid && l1d[si].lines[i].tag == t) {
            if (l1d[si].lines[i].modified)
                wb_l1d_to_l2(ba, l1d[si].lines[i].data);
            l1d[si].lines[i].valid    = 0;
            l1d[si].lines[i].modified = 0;
        }
}

/* ═══════════════════════ PUBLIC API ═══════════════════════ */

void init_cache(replacement_policy_e policy) {
    g_policy = policy;
    g_tick   = 0;
    memset(l1i,     0, sizeof(l1i));
    memset(l1d,     0, sizeof(l1d));
    memset(l2,      0, sizeof(l2));
    memset(&st_l1i, 0, sizeof(st_l1i));
    memset(&st_l1d, 0, sizeof(st_l1d));
    memset(&st_l2,  0, sizeof(st_l2));
}

uint8_t read_cache(uint64_t mem_addr, mem_type_t type) {
    if (type == INSTR) {
        /* Flush dirty L1d copy to L2 so L1i reads the latest value from L2 */
        coherency_flush_l1d(mem_addr);

        st_l1i.accesses++;
        uint64_t si = l1i_idx(mem_addr), t = l1i_tag(mem_addr);
        for (int i = 0; i < L1_INSTR_WAYS; i++) {
            if (l1i[si].lines[i].valid && l1i[si].lines[i].tag == t) {
                l1i[si].lru[i] = g_tick++;
                return l1i[si].lines[i].data[off(mem_addr)];
            }
        }
        st_l1i.misses++;
        st_l2.accesses++;
        l2_ensure(mem_addr);
        int w = l1i_load(mem_addr);
        return l1i[si].lines[w].data[off(mem_addr)];
    } else {
        /* Flush dirty L1i copy to L2 so L1d reads the latest value from L2 */
        coherency_flush_l1i(mem_addr);

        st_l1d.accesses++;
        uint64_t si = l1d_idx(mem_addr), t = l1d_tag(mem_addr);
        for (int i = 0; i < L1_DATA_WAYS; i++) {
            if (l1d[si].lines[i].valid && l1d[si].lines[i].tag == t) {
                l1d[si].lru[i] = g_tick++;
                return l1d[si].lines[i].data[off(mem_addr)];
            }
        }
        st_l1d.misses++;
        st_l2.accesses++;
        l2_ensure(mem_addr);
        int w = l1d_load(mem_addr);
        return l1d[si].lines[w].data[off(mem_addr)];
    }
}

void write_cache(uint64_t mem_addr, uint8_t value, mem_type_t type) {
    if (type == INSTR) {
        /* Write invalidate: flush dirty L1d copy to L2 then invalidate L1d line */
        coherency_inval_l1d(mem_addr);

        st_l1i.accesses++;
        uint64_t si = l1i_idx(mem_addr), t = l1i_tag(mem_addr);
        for (int i = 0; i < L1_INSTR_WAYS; i++) {
            if (l1i[si].lines[i].valid && l1i[si].lines[i].tag == t) {
                l1i[si].lines[i].data[off(mem_addr)] = value;
                l1i[si].lines[i].modified             = 1;
                l1i[si].lru[i]                        = g_tick++;
                return;
            }
        }
        st_l1i.misses++;
        st_l2.accesses++;
        l2_ensure(mem_addr);
        int w = l1i_load(mem_addr);
        l1i[si].lines[w].data[off(mem_addr)] = value;
        l1i[si].lines[w].modified             = 1;
    } else {
        /* Write invalidate: flush dirty L1i copy to L2 then invalidate L1i line */
        coherency_inval_l1i(mem_addr);

        st_l1d.accesses++;
        uint64_t si = l1d_idx(mem_addr), t = l1d_tag(mem_addr);
        for (int i = 0; i < L1_DATA_WAYS; i++) {
            if (l1d[si].lines[i].valid && l1d[si].lines[i].tag == t) {
                l1d[si].lines[i].data[off(mem_addr)] = value;
                l1d[si].lines[i].modified             = 1;
                l1d[si].lru[i]                        = g_tick++;
                return;
            }
        }
        st_l1d.misses++;
        st_l2.accesses++;
        l2_ensure(mem_addr);
        int w = l1d_load(mem_addr);
        l1d[si].lines[w].data[off(mem_addr)] = value;
        l1d[si].lines[w].modified             = 1;
    }
}

cache_stats_t get_l1_instr_stats() { return st_l1i; }
cache_stats_t get_l1_data_stats()  { return st_l1d; }
cache_stats_t get_l2_stats()       { return st_l2;  }

cache_line_t *get_l1_instr_cache_line(uint64_t mem_addr) {
    uint64_t si = l1i_idx(mem_addr), t = l1i_tag(mem_addr);
    for (int i = 0; i < L1_INSTR_WAYS; i++)
        if (l1i[si].lines[i].valid && l1i[si].lines[i].tag == t) return &l1i[si].lines[i];
    return NULL;
}

cache_line_t *get_l1_data_cache_line(uint64_t mem_addr) {
    uint64_t si = l1d_idx(mem_addr), t = l1d_tag(mem_addr);
    for (int i = 0; i < L1_DATA_WAYS; i++)
        if (l1d[si].lines[i].valid && l1d[si].lines[i].tag == t) return &l1d[si].lines[i];
    return NULL;
}

cache_line_t *get_l2_cache_line(uint64_t mem_addr) {
    uint64_t si = l2_idx(mem_addr), t = l2_tag(mem_addr);
    for (int i = 0; i < L2_WAYS; i++)
        if (l2[si].lines[i].valid && l2[si].lines[i].tag == t) return &l2[si].lines[i];
    return NULL;
}
