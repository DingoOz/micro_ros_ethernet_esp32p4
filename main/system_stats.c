#include "system_stats.h"

#include <string.h>
#include <stdio.h>

#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "driver/temperature_sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/reset_reasons.h"

static temperature_sensor_handle_t temp_sensor = NULL;

static const char *get_reset_reason_str(esp_reset_reason_t reason)
{
    switch (reason) {
        case ESP_RST_POWERON:  return "Power-on";
        case ESP_RST_SW:       return "Software";
        case ESP_RST_PANIC:    return "Panic";
        case ESP_RST_INT_WDT:  return "Int WDT";
        case ESP_RST_TASK_WDT: return "Task WDT";
        case ESP_RST_WDT:      return "WDT";
        case ESP_RST_DEEPSLEEP:return "Deep sleep";
        case ESP_RST_BROWNOUT: return "Brownout";
        case ESP_RST_SDIO:     return "SDIO";
        default:               return "Unknown";
    }
}

void system_stats_init(void)
{
    temperature_sensor_config_t conf = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    esp_err_t err = temperature_sensor_install(&conf, &temp_sensor);
    if (err != ESP_OK) {
        printf("Temp sensor install failed: %s\n", esp_err_to_name(err));
        return;
    }
    temperature_sensor_enable(temp_sensor);
    printf("Temperature sensor initialised\n");
}

void system_stats_collect(system_stats_t *stats)
{
    memset(stats, 0, sizeof(*stats));

    // Chip info
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    stats->chip_model = "ESP32-P4";
    stats->chip_revision = chip.revision;
    stats->reset_reason = get_reset_reason_str(esp_reset_reason());

    // CPU frequency
    stats->cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;

    // Temperature
    if (temp_sensor) {
        temperature_sensor_get_celsius(temp_sensor, &stats->temperature_c);
    }

    // Internal SRAM heap
    stats->internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    stats->internal_total = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);

    // PSRAM heap
    stats->psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    stats->psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);

    // Heap watermark
    stats->heap_min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);

    // Flash size
    uint32_t flash_size = 0;
    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        stats->flash_size = flash_size;
    }

    // Uptime
    stats->uptime_s = (uint32_t)(esp_timer_get_time() / 1000000);

    // FreeRTOS task list
    uint32_t num_tasks = uxTaskGetNumberOfTasks();
    if (num_tasks > MAX_TASKS) num_tasks = MAX_TASKS;

    TaskStatus_t task_status[MAX_TASKS];
    uint32_t total_runtime;
    uint32_t actual = uxTaskGetSystemState(task_status, num_tasks, &total_runtime);

    stats->task_count = actual;
    stats->core0_tasks = 0;
    stats->core1_tasks = 0;

    for (uint32_t i = 0; i < actual; i++) {
        strncpy(stats->tasks[i].name, task_status[i].pcTaskName, sizeof(stats->tasks[i].name) - 1);
        stats->tasks[i].priority = task_status[i].uxCurrentPriority;
        stats->tasks[i].stack_hwm = task_status[i].usStackHighWaterMark * sizeof(StackType_t);
#if configTASKLIST_INCLUDE_COREID == 1
        stats->tasks[i].core_id = task_status[i].xCoreID;
        if (task_status[i].xCoreID == 0) stats->core0_tasks++;
        else if (task_status[i].xCoreID == 1) stats->core1_tasks++;
#else
        stats->tasks[i].core_id = -1;
#endif
    }
}
