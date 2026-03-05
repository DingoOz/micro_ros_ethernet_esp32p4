#ifndef SYSTEM_STATS_H
#define SYSTEM_STATS_H

#include <stdint.h>
#include <stdbool.h>

#define MAX_TASKS 32

typedef struct {
    char name[16];
    uint32_t priority;
    uint32_t stack_hwm;     // stack high-water mark in bytes
    int core_id;            // 0, 1, or -1 (no affinity)
} task_info_t;

typedef struct {
    // Chip
    const char *chip_model;
    uint8_t chip_revision;
    const char *reset_reason;

    // CPU
    uint32_t cpu_freq_mhz;

    // Temperature
    float temperature_c;

    // Internal SRAM heap
    uint32_t internal_free;
    uint32_t internal_total;

    // PSRAM heap
    uint32_t psram_free;
    uint32_t psram_total;

    // Heap watermark (minimum-ever-free)
    uint32_t heap_min_free;

    // Flash
    uint32_t flash_size;

    // FreeRTOS tasks
    task_info_t tasks[MAX_TASKS];
    uint32_t task_count;
    uint32_t core0_tasks;
    uint32_t core1_tasks;

    // Uptime
    uint32_t uptime_s;
} system_stats_t;

// Install temperature sensor and prepare collection
void system_stats_init(void);

// Gather all system stats into the provided struct
void system_stats_collect(system_stats_t *stats);

#endif // SYSTEM_STATS_H
