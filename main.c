#include "tcache.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// stride between addresses that hit the same L1d set (256 sets * 64 bytes)
#define L1D_SET_STRIDE ((HW11_L1_SIZE / (HW11_L1_DATA_ASSOC * 64)) * 64)
// stride between addresses that hit the same L2 set (8192 sets * 64 bytes)
#define L2_SET_STRIDE ((HW11_L2_SIZE / (HW11_L2_ASSOC * 64)) * 64)

static int pass = 0, fail = 0;

static void check(int cond, const char *msg) {
    if (cond) {
        printf("  PASS: %s\n", msg);
        pass++;
    } else {
        printf("  FAIL: %s\n", msg);
        fail++;
    }
}

static void test_basic_rw() {
    printf("\n-- test_basic_rw --\n");
    init_cache(LRU);

    write_cache(0x1000, 0xAB, DATA);
    check(read_cache(0x1000, DATA) == 0xAB, "write then read");
    write_cache(0x1001, 0xCD, DATA);
    check(read_cache(0x1001, DATA) == 0xCD, "adjacent byte");
    check(read_cache(0x1000, DATA) == 0xAB, "first byte still correct");
}

static void test_line_granularity() {
    printf("\n-- test_line_granularity --\n");
    init_cache(LRU);

    write_cache(0x2000, 0x55, DATA);
    check(read_cache(0x2001, DATA) == 0, "unwritten offset in same line is 0");
    check(read_cache(0x203F, DATA) == 0, "last byte in 64B line is 0");
}

static void test_l1d_stats() {
    printf("\n-- test_l1d_stats --\n");
    init_cache(LRU);

    read_cache(0x3000, DATA);
    cache_stats_t s = get_l1_data_stats();
    check(s.accesses == 1, "1 access");
    check(s.misses == 1, "cold miss");

    read_cache(0x3008, DATA);
    s = get_l1_data_stats();
    check(s.accesses == 2, "2 accesses");
    check(s.misses == 1, "same line = hit, still 1 miss");

    read_cache(0x3040, DATA);
    s = get_l1_data_stats();
    check(s.misses == 2, "new line = miss");
}

static void test_l1i_stats() {
    printf("\n-- test_l1i_stats --\n");
    init_cache(LRU);

    read_cache(0x4000, INSTR);
    cache_stats_t s = get_l1_instr_stats();
    check(s.accesses == 1 && s.misses == 1, "cold instr miss");
    read_cache(0x4008, INSTR);
    s = get_l1_instr_stats();
    check(s.misses == 1, "same line hit");
}

static void test_l2_on_miss() {
    printf("\n-- test_l2_on_miss --\n");
    init_cache(LRU);

    read_cache(0x5000, DATA);
    cache_line_t *line = get_l2_cache_line(0x5000);
    check(line != NULL && line->valid, "L2 has valid line after L1 miss");
}

static void test_writeback() {
    printf("\n-- test_writeback --\n");
    init_cache(LRU);

    write_cache(0x100, 0x77, DATA);

    // force eviction by hitting the same L1d set 3 more times (2-way, need 3 conflicts)
    read_cache(0x100 + L1D_SET_STRIDE, DATA);
    read_cache(0x100 + 2 * L1D_SET_STRIDE, DATA);
    read_cache(0x100 + 3 * L1D_SET_STRIDE, DATA);

    check(read_cache(0x100, DATA) == 0x77, "value survives eviction via write-back");
}

static void test_l2_inclusive() {
    printf("\n-- test_l2_inclusive --\n");
    init_cache(LRU);

    read_cache(0x200, DATA);
    check(get_l1_data_cache_line(0x200) != NULL, "L1d has line");

    // evict from L2 by filling its set (4-way, need 5 conflicts)
    int i;
    for (i = 1; i <= 5; i++)
        read_cache(0x200 + i * L2_SET_STRIDE, DATA);

    check(get_l1_data_cache_line(0x200) == NULL, "L1d invalidated after L2 eviction");
}

static void test_coherency() {
    printf("\n-- test_coherency --\n");
    init_cache(LRU);

    write_cache(0x6000, 0x42, INSTR);
    check(read_cache(0x6000, DATA) == 0x42, "DATA sees INSTR write");

    write_cache(0x7000, 0x99, DATA);
    check(read_cache(0x7000, INSTR) == 0x99, "INSTR sees DATA write");

    // write to DATA invalidates L1i line
    read_cache(0xA000, INSTR);
    check(get_l1_instr_cache_line(0xA000) != NULL, "L1i has line");
    write_cache(0xA000, 0x11, DATA);
    check(get_l1_instr_cache_line(0xA000) == NULL, "L1i gone after DATA write");

    // write to INSTR invalidates L1d line
    read_cache(0xB000, DATA);
    check(get_l1_data_cache_line(0xB000) != NULL, "L1d has line");
    write_cache(0xB000, 0x22, INSTR);
    check(get_l1_data_cache_line(0xB000) == NULL, "L1d gone after INSTR write");
}

static void test_get_line() {
    printf("\n-- test_get_line --\n");
    init_cache(LRU);

    check(get_l1_data_cache_line(0x8000) == NULL, "L1d null before access");
    read_cache(0x8000, DATA);
    check(get_l1_data_cache_line(0x8000) != NULL, "L1d non-null after access");
    check(get_l2_cache_line(0x8000) != NULL, "L2 non-null after access");

    check(get_l1_instr_cache_line(0x9000) == NULL, "L1i null before access");
    read_cache(0x9000, INSTR);
    check(get_l1_instr_cache_line(0x9000) != NULL, "L1i non-null after access");
}

typedef struct {
    uint64_t l1d, l1i, l2;
} misses_t;

static misses_t run_seq_scan(replacement_policy_e p) {
    init_cache(p);
    uint64_t i;
    for (i = 0; i < 1024 * 1024; i++)
        read_cache(i, DATA);
    misses_t m = {
        get_l1_data_stats().misses,
        get_l1_instr_stats().misses,
        get_l2_stats().misses
    };
    return m;
}

static misses_t run_strided(replacement_policy_e p) {
    init_cache(p);
    int i;
    for (i = 0; i < 10000; i++)
        read_cache((uint64_t)i * 4096 % HW11_MEM_SIZE, DATA);
    misses_t m = {
        get_l1_data_stats().misses,
        get_l1_instr_stats().misses,
        get_l2_stats().misses
    };
    return m;
}

static misses_t run_working_set(replacement_policy_e p) {
    init_cache(p);
    // 128KB working set, 8 passes
    int pass, j;
    for (pass = 0; pass < 8; pass++)
        for (j = 0; j < 128 * 1024; j += 64)
            read_cache(j, DATA);
    misses_t m = {
        get_l1_data_stats().misses,
        get_l1_instr_stats().misses,
        get_l2_stats().misses
    };
    return m;
}

static misses_t run_thrash(replacement_policy_e p) {
    srand(42);
    init_cache(p);
    // 5 addresses all mapping to the same L1d set, 2000 rounds
    uint64_t addrs[5];
    int k;
    for (k = 0; k < 5; k++)
        addrs[k] = (uint64_t)k * L1D_SET_STRIDE;
    int r;
    for (r = 0; r < 2000; r++)
        for (k = 0; k < 5; k++)
            read_cache(addrs[k], DATA);
    misses_t m = {
        get_l1_data_stats().misses,
        get_l1_instr_stats().misses,
        get_l2_stats().misses
    };
    return m;
}

static void print_cmp(const char *name, misses_t lru, misses_t rnd) {
    printf("\n%s\n", name);
    printf("           LRU       RANDOM\n");
    printf("  L1d:  %-9llu %-9llu\n", (unsigned long long)lru.l1d, (unsigned long long)rnd.l1d);
    printf("  L2:   %-9llu %-9llu\n", (unsigned long long)lru.l2,  (unsigned long long)rnd.l2);
}

int main() {
    printf("=== unit tests ===\n");

    test_basic_rw();
    test_line_granularity();
    test_l1d_stats();
    test_l1i_stats();
    test_l2_on_miss();
    test_writeback();
    test_l2_inclusive();
    test_coherency();
    test_get_line();

    printf("\n%d passed, %d failed\n", pass, fail);

    printf("\n=== LRU vs Random ===\n");
    srand(12345);

    misses_t lru, rnd;

    lru = run_seq_scan(LRU);
    rnd = run_seq_scan(RANDOM);
    print_cmp("Sequential scan (1MB)", lru, rnd);

    lru = run_strided(LRU);
    rnd = run_strided(RANDOM);
    print_cmp("Strided access (4KB stride, 10000 iters)", lru, rnd);

    lru = run_working_set(LRU);
    rnd = run_working_set(RANDOM);
    print_cmp("Working set loop (128KB, 8 passes)", lru, rnd);

    lru = run_thrash(LRU);
    srand(42);
    rnd = run_thrash(RANDOM);
    print_cmp("L1d thrash (5-way conflict into 2-way, 2000 rounds)", lru, rnd);

    return (fail == 0) ? 0 : 1;
}
