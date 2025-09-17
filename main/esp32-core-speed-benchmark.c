/**
 * @file esp32-core-speed-benchmark.c
 * @brief Simple memcpy/manual copy micro-benchmark across CPU frequencies and memories (SRAM/PSRAM).
 *
 * This benchmark measures throughput (MB/s) for:
 *  - libc memcpy() on internal SRAM
 *  - a naive byte-wise manual copy on internal SRAM
 *  - optionally, memcpy between SRAM and PSRAM (if available)
 *
 * The CPU frequency is locked to several fixed values via Power Management.
 *
 * @note Tested with ESP-IDF v5.5. Requires: esp_timer, esp_pm, heap_caps, clk tree APIs.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#include "esp_pm.h"
#include "esp_clk_tree.h"      
#include "soc/clk_tree_defs.h" 

static const char *TAG = "benchmark";
static const char *WHITE_COLOR_CONSOLE_OUTPUT = "\033[1;37m";
static const char *YELLOW_COLOR_CONSOLE_OUTPUT = "\033[1;33m";
static const char *BLUE_COLOR_CONSOLE_OUTPUT = "\033[1;34m";

typedef struct {
    size_t size_bytes;  /**< Buffer size in bytes for a single copy operation. */
    int    iterations;  /**< Number of repetitions for averaging. */
} bench_case_t;

// Use cases (choose sizes that could fit SRAM)
static const bench_case_t CASES[] = {
    { 1024,      2000 },  // 1 KB, 2000 iterations
    { 16*1024,    400 },
    { 64*1024,    120 }
};

/**
 * @brief CPU frequencies to lock for each sweep.
 */
#if CONFIG_IDF_TARGET_ESP32
    static const int FREQUENCY_VALUES_MHZ[] = {80, 160, 240};

#elif CONFIG_IDF_TARGET_ESP32S2
    static const int FREQUENCY_VALUES_MHZ[] = {80, 160, 240};

#elif CONFIG_IDF_TARGET_ESP32S3
    static const int FREQUENCY_VALUES_MHZ[] = {80, 160, 240};

#elif CONFIG_IDF_TARGET_ESP32C3
    static const int FREQUENCY_VALUES_MHZ[] = {80, 160};

#elif CONFIG_IDF_TARGET_ESP32C6
    static const int FREQUENCY_VALUES_MHZ[] = {80, 120, 160};

#elif CONFIG_IDF_TARGET_ESP32C2
    static const int FREQUENCY_VALUES_MHZ[] = {80, 120};

#elif CONFIG_IDF_TARGET_ESP32H2
    static const int FREQUENCY_VALUES_MHZ[] = {32, 64, 96};

#else
    static const int FREQUENCY_VALUES_MHZ[] = {80, 160}; // fallback 
#endif

/**
 * @brief Measure memcpy() throughput.
 *
 * @param dst   Destination buffer.
 * @param src   Source buffer.
 * @param bytes Number of bytes per copy.
 * @param iterations Number of iterations.
 * @return Throughput in MB/s.
 */
static double bench_memcpy(uint8_t *dst, const uint8_t *src, size_t bytes, int iterations) {
    int64_t t0 = esp_timer_get_time();
    for (int i = 0; i < iterations; i++) {
        memcpy(dst, src, bytes);
    }
    int64_t t1 = esp_timer_get_time();
    double us = (double)(t1 - t0);
    double total_bytes = (double)bytes * (double)iterations;
    return (total_bytes / (1024.0 * 1024.0)) / (us / 1e6);
}

/**
 * @brief Measure naive byte-wise copy throughput (volatile to reduce over-optimization).
 *
 * @param dst   Destination buffer.
 * @param src   Source buffer.
 * @param bytes Number of bytes per copy.
 * @param iterations Number of iterations.
 * @return Throughput in MB/s.
 */
static double bench_manual(uint8_t *dst, const uint8_t *src, size_t bytes, int iterations) {
    volatile uint8_t *vd = dst; // volatile to prevent overly aggressive optimization
    const volatile uint8_t *vs = src;
    int64_t t0 = esp_timer_get_time();
    for (int i = 0; i < iterations; i++) {
        for (size_t j = 0; j < bytes; j++) {
            ((volatile uint8_t*)vd)[j] = ((const volatile uint8_t*)vs)[j];
        }
    }
    int64_t t1 = esp_timer_get_time();
    double us = (double)(t1 - t0);
    double total_bytes = (double)bytes * (double)iterations;
    return (total_bytes / (1024.0 * 1024.0)) / (us / 1e6);
}

/**
 * @brief Run one benchmark case on internal SRAM, and optionally across PSRAM if present.
 *
 * Allocates buffers with the appropriate capability flags, initializes patterns,
 * runs memcpy/manual tests, and logs MB/s.
 *
 * @param bc         Benchmark case pointer.
 * @param test_psram If true and PSRAM is enabled, also test SRAM->PSRAM and PSRAM->SRAM memcpy.
 */
static void run_case(const bench_case_t *bc, bool test_psram) {
    // Buffers in internal SRAM (8-bit capable, internal)
    uint8_t *src = (uint8_t *)heap_caps_malloc(bc->size_bytes, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    uint8_t *dst = (uint8_t *)heap_caps_malloc(bc->size_bytes, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);

    if (!src || !dst) {
        ESP_LOGE(TAG, "Failed to allocate %u B in internal SRAM (src=%p dst=%p)", (unsigned)bc->size_bytes, src, dst);
        if (src) free(src);
        if (dst) free(dst);
        return;
    }

    // Initialize source pattern
    for (size_t i = 0; i < bc->size_bytes; i++) src[i] = (uint8_t)(i * 131u + 7u);

    double mbs_memcpy  = bench_memcpy(dst, src, bc->size_bytes, bc->iterations);
    double mbs_manual  = bench_manual(dst, src, bc->size_bytes, bc->iterations);

    ESP_LOGI(TAG, "[SRAM] size=%uB iterations=%d %s| memcpy=%.1f MB/s %s| manual=%.1f MB/s",
             (unsigned)bc->size_bytes, bc->iterations, YELLOW_COLOR_CONSOLE_OUTPUT, mbs_memcpy, BLUE_COLOR_CONSOLE_OUTPUT, mbs_manual);

    free(src); free(dst);

#if CONFIG_SPIRAM
    vTaskDelay(1); // yield for WDT
    if (test_psram) {
        // If PSRAM exists, also test two directions: SRAM->PSRAM and PSRAM->SRAM
        uint8_t *ps  = (uint8_t *)heap_caps_malloc(bc->size_bytes, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
        uint8_t *pi  = (uint8_t *)heap_caps_malloc(bc->size_bytes, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
        uint8_t *is  = (uint8_t *)heap_caps_malloc(bc->size_bytes, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
        uint8_t *ip  = (uint8_t *)heap_caps_malloc(bc->size_bytes, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
        if (ps && pi && is && ip) {
            for (size_t i = 0; i < bc->size_bytes; i++) { pi[i] = (uint8_t)(i ^ 0xA5); ip[i] = (uint8_t)(i ^ 0x5A); }
            double m1 = bench_memcpy(ps, pi, bc->size_bytes, bc->iterations); // SRAM->PSRAM
            double m2 = bench_memcpy(is, ip, bc->size_bytes, bc->iterations); // PSRAM->SRAM
            ESP_LOGI(TAG, "[PSRAM] size=%uB iterations=%d | SRAM->PSRAM memcpy=%.1f MB/s | PSRAM->SRAM memcpy=%.1f MB/s",
                     (unsigned)bc->size_bytes, bc->iterations, m1, m2);
        } else {
            ESP_LOGW(TAG, "PSRAM test: allocation failed (ps=%p pi=%p is=%p ip=%p)", ps, pi, is, ip);
        }
        if (ps) free(ps); if (pi) free(pi); if (is) free(is); if (ip) free(ip);
    }
#else
    (void)test_psram;
#endif
}

/**
 * @brief Entry point: sweeps CPU frequencies and runs all cases.
 *
 * - Locks CPU to each frequency in @ref FREQUENCY_VALUES_MHZ using esp_pm_configure().
 * - Reads back the actual CPU frequency from clk tree.
 * - Runs SRAM/PSRAM benchmarks for each case.
 *
 * @note The watchdog is yielded via a short vTaskDelay() between run cases.
 */
void app_main(void) {
    printf("\n");

    for (int index = 0; index < sizeof(FREQUENCY_VALUES_MHZ) / sizeof(FREQUENCY_VALUES_MHZ[0]); index++){
        esp_pm_config_t cfg = {
            .min_freq_mhz = FREQUENCY_VALUES_MHZ[index],
            .max_freq_mhz = FREQUENCY_VALUES_MHZ[index],
            .light_sleep_enable = false
        };  
        esp_err_t r = esp_pm_configure(&cfg);
        if (r != ESP_OK) {
            ESP_LOGW(TAG, "Power management is not enabled or freq %dMHz not supported (%s). Skipping.",
                    cfg.max_freq_mhz, esp_err_to_name(r));
            continue;
        }

        uint32_t cpu_hz = 0;
        esp_err_t err = esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_CPU,
                                                    ESP_CLK_TREE_SRC_FREQ_PRECISION_EXACT,
                                                    &cpu_hz);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "CPU freq: %s%" PRIu32 "MHz", WHITE_COLOR_CONSOLE_OUTPUT, cpu_hz / 1000000);
        }
    
        for (size_t i = 0; i < sizeof(CASES)/sizeof(CASES[0]); i++) {
            run_case(&CASES[i], true /* test PSRAM if present */);
            vTaskDelay(1); // yield for WDT
        }

        printf("\n");
    }
}
