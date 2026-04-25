#include "tcache.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

/* Derived cache geometry constants */
#define L1_DATA_SETS_STRIDE  ((HW11_L1_SIZE / (HW11_L1_DATA_ASSOC * 64)) * 64)   /* 16384 */
#define L2_SETS_STRIDE       ((HW11_L2_SIZE / (HW11_L2_ASSOC * 64)) * 64)         /* 524288 */

/* ── test helpers ── */
static int tests_run = 0, tests_passed = 0;

#define CHECK(cond, msg) do { \
    tests_run++; \
    if (cond) { tests_passed++; printf("  PASS: %s\n", msg); } \
    else { printf("  FAIL: %s\n", msg); } \
} while(0)

/* ── unit tests ── */

static void test_basic_read_write(void) {
    printf("\n=== test_basic_read_write ===\n");
    init_cache(LRU);

    write_cache(0x1000, 0xAB, DATA);
    uint8_t v = read_cache(0x1000, DATA);
    CHECK(v == 0xAB, "write then read same byte");

    write_cache(0x1001, 0xCD, DATA);
    CHECK(read_cache(0x1001, DATA) == 0xCD, "adjacent byte");
    CHECK(read_cache(0x1000, DATA) == 0xAB, "original byte still correct");
}

static void test_cache_line_granularity(void) {
    printf("\n=== test_cache_line_granularity ===\n");
    init_cache(LRU);

    /* Write to offset 0 of a line; rest of line should come from memory (zeroed) */
    write_cache(0x2000, 0x55, DATA);
    CHECK(read_cache(0x2001, DATA) == 0x00, "unwritten byte in same line is 0");
    CHECK(read_cache(0x203F, DATA) == 0x00, "last byte in 64B line is 0");
}

static void test_l1_stats(void) {
    printf("\n=== test_l1_stats ===\n");
    init_cache(LRU);

    /* First access: miss */
    read_cache(0x3000, DATA);
    cache_stats_t s = get_l1_data_stats();
    CHECK(s.accesses == 1, "1 access after 1 read");
    CHECK(s.misses   == 1, "1 miss after cold read");

    /* Same line: hit */
    read_cache(0x3004, DATA);
    s = get_l1_data_stats();
    CHECK(s.accesses == 2, "2 accesses after 2 reads");
    CHECK(s.misses   == 1, "still 1 miss (same cache line)");

    /* New line: miss */
    read_cache(0x3040, DATA);
    s = get_l1_data_stats();
    CHECK(s.misses == 2, "2 misses after accessing new line");
}

static void test_instr_stats(void) {
    printf("\n=== test_instr_stats ===\n");
    init_cache(LRU);

    read_cache(0x4000, INSTR);
    cache_stats_t s = get_l1_instr_stats();
    CHECK(s.accesses == 1 && s.misses == 1, "instr: cold miss counted");

    read_cache(0x4008, INSTR);
    s = get_l1_instr_stats();
    CHECK(s.misses == 1, "instr: same line = hit");
}

static void test_l2_populated_on_l1_miss(void) {
    printf("\n=== test_l2_populated_on_l1_miss ===\n");
    init_cache(LRU);

    read_cache(0x5000, DATA);
    cache_line_t *l2line = get_l2_cache_line(0x5000);
    CHECK(l2line != NULL, "L2 has line after L1 miss");
    CHECK(l2line->valid, "L2 line is valid");
}

static void test_write_back_dirty(void) {
    printf("\n=== test_write_back_dirty ===\n");
    init_cache(LRU);

    write_cache(0x100, 0x77, DATA);

    /* Evict from L1d by filling its set with conflicting addresses.
       L1d: 256 sets, 2-way.  Addresses that map to same set differ by 256*64 = 16384 bytes. */
    uint64_t conflict1 = 0x100 + 16384;
    uint64_t conflict2 = 0x100 + 2*16384;
    uint64_t conflict3 = 0x100 + 3*16384;
    read_cache(conflict1, DATA);
    read_cache(conflict2, DATA);
    read_cache(conflict3, DATA);  /* now 0x100 must have been evicted */

    /* Value should have been written back to L2 (and ultimately visible) */
    /* Re-read after eviction: should return our written value */
    uint8_t v = read_cache(0x100, DATA);
    CHECK(v == 0x77, "written value survives L1 eviction (write-back)");
}

static void test_l2_inclusive_invalidates_l1(void) {
    printf("\n=== test_l2_inclusive_invalidates_l1 ===\n");
    init_cache(LRU);

    read_cache(0x200, DATA);
    CHECK(get_l1_data_cache_line(0x200) != NULL, "L1d has line before L2 eviction");

    /* Fill enough of L2 to evict 0x200's line.
       L2: 8192 sets, 4-way. stride = 8192*64 bytes hits same L2 set. */
    uint64_t stride = (uint64_t)L2_SETS_STRIDE;
    for (int i = 1; i <= 5; i++)
        read_cache(0x200 + i * stride, DATA);

    /* After L2 eviction of 0x200's line, L1d copy should be gone */
    CHECK(get_l1_data_cache_line(0x200) == NULL, "L1d line invalidated after L2 eviction");
}

static void test_coherency(void) {
    printf("\n=== test_coherency ===\n");
    init_cache(LRU);

    /* Write via INSTR cache, read back via DATA cache */
    write_cache(0x6000, 0x42, INSTR);
    uint8_t v = read_cache(0x6000, DATA);
    CHECK(v == 0x42, "DATA read sees INSTR write (coherency)");

    /* Write via DATA, read via INSTR */
    write_cache(0x7000, 0x99, DATA);
    v = read_cache(0x7000, INSTR);
    CHECK(v == 0x99, "INSTR read sees DATA write (coherency)");

    /* Write to DATA invalidates L1i (write-invalidate protocol) */
    read_cache(0xA000, INSTR);  /* load into L1i */
    CHECK(get_l1_instr_cache_line(0xA000) != NULL, "L1i has line before DATA write");
    write_cache(0xA000, 0x11, DATA);
    CHECK(get_l1_instr_cache_line(0xA000) == NULL, "L1i invalidated after DATA write");

    /* Write to INSTR invalidates L1d (write-invalidate protocol) */
    read_cache(0xB000, DATA);   /* load into L1d */
    CHECK(get_l1_data_cache_line(0xB000) != NULL, "L1d has line before INSTR write");
    write_cache(0xB000, 0x22, INSTR);
    CHECK(get_l1_data_cache_line(0xB000) == NULL, "L1d invalidated after INSTR write");
}

static void test_get_cache_line(void) {
    printf("\n=== test_get_cache_line ===\n");
    init_cache(LRU);

    CHECK(get_l1_data_cache_line(0x8000) == NULL, "L1d returns NULL before access");
    read_cache(0x8000, DATA);
    CHECK(get_l1_data_cache_line(0x8000) != NULL, "L1d returns line after access");
    CHECK(get_l2_cache_line(0x8000) != NULL, "L2 returns line after L1d access");

    CHECK(get_l1_instr_cache_line(0x9000) == NULL, "L1i returns NULL before access");
    read_cache(0x9000, INSTR);
    CHECK(get_l1_instr_cache_line(0x9000) != NULL, "L1i returns line after access");
}

/* ── LRU vs Random comparison ── */

typedef struct {
    uint64_t l1d_misses;
    uint64_t l1i_misses;
    uint64_t l2_misses;
} miss_counts_t;

/* Simulate sequential scan (streaming): accesses 1 MB of data sequentially,
   which causes repeated capacity misses in L1d. */
static miss_counts_t run_sequential_scan(replacement_policy_e policy) {
    init_cache(policy);
    uint64_t base = 0x0;
    uint64_t bytes = 1024 * 1024;  /* 1 MB */
    for (uint64_t i = 0; i < bytes; i++)
        read_cache(base + i, DATA);
    miss_counts_t m;
    m.l1d_misses = get_l1_data_stats().misses;
    m.l1i_misses = get_l1_instr_stats().misses;
    m.l2_misses  = get_l2_stats().misses;
    return m;
}

/* Simulate strided access pattern: accesses every 4KB (page-size stride),
   which causes conflict misses in direct-mapped / low-associativity caches. */
static miss_counts_t run_strided_access(replacement_policy_e policy) {
    init_cache(policy);
    uint64_t stride = 4096;
    int iterations = 10000;
    for (int i = 0; i < iterations; i++)
        read_cache((uint64_t)i * stride % (HW11_MEM_SIZE), DATA);
    miss_counts_t m;
    m.l1d_misses = get_l1_data_stats().misses;
    m.l1i_misses = get_l1_instr_stats().misses;
    m.l2_misses  = get_l2_stats().misses;
    return m;
}

/* Simulate working-set loop: repeatedly accesses a fixed set of addresses
   that fits in L2 but exceeds L1d capacity, cycling through them. */
static miss_counts_t run_working_set_loop(replacement_policy_e policy) {
    init_cache(policy);
    /* Working set: 128 KB, slightly larger than L1d (32 KB) */
    uint64_t ws_size = 128 * 1024;
    int passes = 8;
    for (int p = 0; p < passes; p++)
        for (uint64_t i = 0; i < ws_size; i += 64)
            read_cache(i, DATA);
    miss_counts_t m;
    m.l1d_misses = get_l1_data_stats().misses;
    m.l1i_misses = get_l1_instr_stats().misses;
    m.l2_misses  = get_l2_stats().misses;
    return m;
}

/* Simulate thrashing: access a set of addresses all mapping to the same
   L1d set (conflict-heavy), worse than the associativity can handle. */
static miss_counts_t run_thrash_l1d(replacement_policy_e policy) {
    srand(42);
    init_cache(policy);
    /* L1d set stride = 256 * 64 = 16384 bytes.
       Access 5 addresses mapping to the same set (> 2-way capacity). */
    uint64_t set_stride = L1_DATA_SETS_STRIDE;
    uint64_t addrs[5];
    for (int k = 0; k < 5; k++) addrs[k] = (uint64_t)k * set_stride;
    int rounds = 2000;
    for (int r = 0; r < rounds; r++)
        for (int k = 0; k < 5; k++)
            read_cache(addrs[k], DATA);
    miss_counts_t m;
    m.l1d_misses = get_l1_data_stats().misses;
    m.l1i_misses = get_l1_instr_stats().misses;
    m.l2_misses  = get_l2_stats().misses;
    return m;
}

static void print_comparison(const char *name,
                              miss_counts_t lru, miss_counts_t rnd) {
    printf("\n--- %s ---\n", name);
    printf("              LRU        RANDOM\n");
    printf("  L1d misses: %-10llu %-10llu\n",
           (unsigned long long)lru.l1d_misses, (unsigned long long)rnd.l1d_misses);
    printf("  L2  misses: %-10llu %-10llu\n",
           (unsigned long long)lru.l2_misses,  (unsigned long long)rnd.l2_misses);
}

int main(int argc, char *argv[]) {
    printf("========== Unit Tests ==========\n");

    test_basic_read_write();
    test_cache_line_granularity();
    test_l1_stats();
    test_instr_stats();
    test_l2_populated_on_l1_miss();
    test_write_back_dirty();
    test_coherency();
    test_get_cache_line();

    printf("\n%d/%d tests passed\n", tests_passed, tests_run);

    printf("\n========== LRU vs Random Replacement Policy Comparison ==========\n");
    srand(12345);

    miss_counts_t lru, rnd;

    lru = run_sequential_scan(LRU);
    rnd = run_sequential_scan(RANDOM);
    print_comparison("Sequential Scan (1MB)", lru, rnd);

    lru = run_strided_access(LRU);
    rnd = run_strided_access(RANDOM);
    print_comparison("Strided Access (4KB stride, 10000 accesses)", lru, rnd);

    lru = run_working_set_loop(LRU);
    rnd = run_working_set_loop(RANDOM);
    print_comparison("Working-Set Loop (128KB ws, 8 passes)", lru, rnd);

    lru = run_thrash_l1d(LRU);
    srand(42);
    rnd = run_thrash_l1d(RANDOM);
    print_comparison("L1d Thrash (5-way conflict, 2-way cache, 2000 rounds)", lru, rnd);

    return (tests_passed == tests_run) ? 0 : 1;
}
