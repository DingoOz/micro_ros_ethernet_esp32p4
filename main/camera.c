#include "camera.h"

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_cache.h"
#include "esp_ldo_regulator.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "driver/isp.h"
#include "driver/jpeg_encode.h"
#include "esp_private/esp_cache_private.h"
#include "esp_sccb_intf.h"
#include "esp_sccb_i2c.h"
#include "esp_cam_sensor_detect.h"
#include "esp_cam_sensor.h"

static const char *TAG = "camera";

// MIPI CSI config for OV5647
#define CSI_LANE_BITRATE_MBPS   200
#define CSI_DATA_LANES          2
#define SCCB_FREQ_HZ            (100 * 1000)
#define MIPI_LDO_CHAN_ID        3
#define MIPI_LDO_VOLTAGE_MV     2500

// Frame buffer: RGB565
#define FB_BPP                  2  // bytes per pixel for RGB565
#define FB_SIZE                 (CAM_WIDTH * CAM_HEIGHT * FB_BPP)

// JPEG output buffer (max size)
#define JPEG_OUT_BUF_SIZE       (200 * 1024)

static esp_cam_ctlr_handle_t s_cam_handle = NULL;
static isp_proc_handle_t s_isp_proc = NULL;
static jpeg_encoder_handle_t s_jpeg_enc = NULL;
static void *s_frame_buffer = NULL;      // DMA target (continuous capture)
static void *s_process_buffer = NULL;    // Copy for JPEG encoding
static uint8_t *s_jpeg_buf = NULL;
static SemaphoreHandle_t s_frame_sem = NULL;   // Signaled when a new frame arrives
static SemaphoreHandle_t s_encode_mutex = NULL; // Serializes JPEG encoding
static volatile bool s_init_ok = false;
static i2c_master_bus_handle_t s_i2c_bus = NULL;

// Background task continuously calls receive to keep the CSI pipeline fed
static void camera_rx_task(void *arg);

static bool IRAM_ATTR on_get_new_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    // Always provide the same DMA buffer for the next frame
    esp_cam_ctlr_trans_t *t = (esp_cam_ctlr_trans_t *)user_data;
    trans->buffer = t->buffer;
    trans->buflen = t->buflen;
    return false;
}

static bool IRAM_ATTR on_trans_finished(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_frame_sem, &xHigherPriorityTaskWoken);
    return xHigherPriorityTaskWoken == pdTRUE;
}

bool camera_init(i2c_master_bus_handle_t i2c_bus)
{
    s_i2c_bus = i2c_bus;
    ESP_LOGI(TAG, "Camera module registered (deferred init)");
    return true;
}

bool camera_start(void)
{
    if (s_init_ok) return true;
    if (!s_i2c_bus) {
        ESP_LOGE(TAG, "No I2C bus available");
        return false;
    }

    ESP_LOGI(TAG, "Starting camera hardware init...");

    // MIPI PHY LDO
    esp_ldo_channel_handle_t ldo_mipi = NULL;
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = MIPI_LDO_CHAN_ID,
        .voltage_mv = MIPI_LDO_VOLTAGE_MV,
    };
    esp_err_t err = esp_ldo_acquire_channel(&ldo_cfg, &ldo_mipi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MIPI LDO init failed: %s", esp_err_to_name(err));
        return false;
    }

    // Allocate frame buffers in PSRAM with cache alignment
    size_t fb_alignment = 0;
    esp_cache_get_alignment(0, &fb_alignment);
    if (fb_alignment == 0) fb_alignment = 64;

    s_frame_buffer = heap_caps_aligned_calloc(fb_alignment, 1, FB_SIZE,
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_process_buffer = heap_caps_aligned_calloc(fb_alignment, 1, FB_SIZE,
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_frame_buffer || !s_process_buffer) {
        ESP_LOGE(TAG, "Failed to allocate frame buffers in PSRAM");
        return false;
    }
    ESP_LOGI(TAG, "Frame buffers: %d x %d RGB565 = %d KB each", CAM_WIDTH, CAM_HEIGHT, FB_SIZE / 1024);

    // Semaphores
    s_frame_sem = xSemaphoreCreateBinary();
    s_encode_mutex = xSemaphoreCreateMutex();

    // Auto-detect camera sensor via SCCB on the shared I2C bus
    esp_cam_sensor_config_t cam_cfg = {
        .reset_pin = -1,
        .pwdn_pin = -1,
        .xclk_pin = -1,
    };

    esp_cam_sensor_device_t *cam_sensor = NULL;
    for (esp_cam_sensor_detect_fn_t *p = &__esp_cam_sensor_detect_fn_array_start;
         p < &__esp_cam_sensor_detect_fn_array_end; ++p) {
        sccb_i2c_config_t sccb_i2c_cfg = {
            .scl_speed_hz = SCCB_FREQ_HZ,
            .device_address = p->sccb_addr,
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        };
        err = sccb_new_i2c_io(s_i2c_bus, &sccb_i2c_cfg, &cam_cfg.sccb_handle);
        if (err != ESP_OK) continue;

        cam_cfg.sensor_port = p->port;
        cam_sensor = (*(p->detect))(&cam_cfg);
        if (cam_sensor) {
            if (p->port != ESP_CAM_SENSOR_MIPI_CSI) {
                cam_sensor = NULL;
                esp_sccb_del_i2c_io(cam_cfg.sccb_handle);
                continue;
            }
            ESP_LOGI(TAG, "Camera sensor detected at SCCB addr 0x%02X", p->sccb_addr);
            break;
        }
        esp_sccb_del_i2c_io(cam_cfg.sccb_handle);
    }

    if (!cam_sensor) {
        ESP_LOGE(TAG, "No camera sensor detected");
        return false;
    }

    // Select format
    esp_cam_sensor_format_array_t fmt_array = {0};
    esp_cam_sensor_query_format(cam_sensor, &fmt_array);
    const esp_cam_sensor_format_t *selected_fmt = NULL;
    const char *target_fmt = "MIPI_2lane_24Minput_RAW8_800x640_50fps";

    for (int i = 0; i < fmt_array.count; i++) {
        ESP_LOGI(TAG, "  fmt[%d]: %s", i, fmt_array.format_array[i].name);
        if (strcmp(fmt_array.format_array[i].name, target_fmt) == 0) {
            selected_fmt = &fmt_array.format_array[i];
        }
    }
    if (!selected_fmt) {
        for (int i = 0; i < fmt_array.count; i++) {
            if (strstr(fmt_array.format_array[i].name, "MIPI")) {
                selected_fmt = &fmt_array.format_array[i];
                break;
            }
        }
    }
    if (!selected_fmt) {
        ESP_LOGE(TAG, "No suitable camera format found");
        return false;
    }

    ESP_LOGI(TAG, "Using format: %s", selected_fmt->name);
    esp_cam_sensor_set_format(cam_sensor, selected_fmt);

    int stream_on = 1;
    esp_cam_sensor_ioctl(cam_sensor, ESP_CAM_SENSOR_IOC_S_STREAM, &stream_on);

    // CSI controller
    esp_cam_ctlr_csi_config_t csi_cfg = {
        .ctlr_id = 0,
        .h_res = CAM_WIDTH,
        .v_res = CAM_HEIGHT,
        .lane_bit_rate_mbps = CSI_LANE_BITRATE_MBPS,
        .input_data_color_type = CAM_CTLR_COLOR_RAW8,
        .output_data_color_type = CAM_CTLR_COLOR_RGB565,
        .data_lane_num = CSI_DATA_LANES,
        .byte_swap_en = false,
        .queue_items = 1,
    };
    err = esp_cam_new_csi_ctlr(&csi_cfg, &s_cam_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CSI controller init failed: %s", esp_err_to_name(err));
        return false;
    }

    // Transaction struct for the callback (lives in static memory)
    static esp_cam_ctlr_trans_t s_cb_trans;
    s_cb_trans.buffer = s_frame_buffer;
    s_cb_trans.buflen = FB_SIZE;

    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans = on_get_new_trans,
        .on_trans_finished = on_trans_finished,
    };
    esp_cam_ctlr_register_event_callbacks(s_cam_handle, &cbs, &s_cb_trans);
    esp_cam_ctlr_enable(s_cam_handle);

    // ISP: RAW8 -> RGB565
    esp_isp_processor_cfg_t isp_cfg = {
        .clk_hz = 80 * 1000 * 1000,
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RAW8,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
        .h_res = CAM_WIDTH,
        .v_res = CAM_HEIGHT,
    };
    err = esp_isp_new_processor(&isp_cfg, &s_isp_proc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ISP init failed: %s", esp_err_to_name(err));
        return false;
    }
    esp_isp_enable(s_isp_proc);

    // Hardware JPEG encoder
    jpeg_encode_engine_cfg_t enc_cfg = {
        .intr_priority = 0,
        .timeout_ms = 1000,
    };
    err = jpeg_new_encoder_engine(&enc_cfg, &s_jpeg_enc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "JPEG encoder init failed: %s", esp_err_to_name(err));
        return false;
    }

    // JPEG output buffer
    jpeg_encode_memory_alloc_cfg_t jpeg_mem_cfg = {
        .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
    };
    size_t jpeg_allocated = 0;
    s_jpeg_buf = jpeg_alloc_encoder_mem(JPEG_OUT_BUF_SIZE, &jpeg_mem_cfg, &jpeg_allocated);
    if (!s_jpeg_buf) {
        ESP_LOGE(TAG, "Failed to allocate JPEG output buffer");
        return false;
    }
    ESP_LOGI(TAG, "JPEG output buffer: %d KB allocated", (int)(jpeg_allocated / 1024));

    // Start camera
    err = esp_cam_ctlr_start(s_cam_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera start failed: %s", esp_err_to_name(err));
        return false;
    }

    // Kick off the first receive — the callback keeps it going continuously
    static esp_cam_ctlr_trans_t first_trans;
    first_trans.buffer = s_frame_buffer;
    first_trans.buflen = FB_SIZE;
    esp_cam_ctlr_receive(s_cam_handle, &first_trans, ESP_CAM_CTLR_MAX_DELAY);

    s_init_ok = true;
    ESP_LOGI(TAG, "Camera ready: OV5647 %dx%d via MIPI CSI + ISP + HW JPEG", CAM_WIDTH, CAM_HEIGHT);

    // Start background receive task
    xTaskCreatePinnedToCore(camera_rx_task, "cam_rx", 4096, NULL, 4, NULL, 0);

    return true;
}

// Background task: continuously calls receive to keep the CSI pipeline running.
// Each completed frame signals the semaphore so capture can grab the latest.
static void camera_rx_task(void *arg)
{
    esp_cam_ctlr_trans_t trans = {
        .buffer = s_frame_buffer,
        .buflen = FB_SIZE,
    };
    while (1) {
        esp_cam_ctlr_receive(s_cam_handle, &trans, ESP_CAM_CTLR_MAX_DELAY);
        // Frame is now in s_frame_buffer — semaphore is given by on_trans_finished callback
    }
}

bool camera_capture_jpeg(uint8_t **out_buf, size_t *out_len)
{
    if (!s_init_ok || !out_buf || !out_len) return false;

    xSemaphoreTake(s_encode_mutex, portMAX_DELAY);

    // Wait for a fresh frame (up to 2 seconds)
    xSemaphoreTake(s_frame_sem, pdMS_TO_TICKS(100));  // Drain any stale signal
    if (xSemaphoreTake(s_frame_sem, pdMS_TO_TICKS(2000)) != pdTRUE) {
        ESP_LOGE(TAG, "Timeout waiting for frame");
        xSemaphoreGive(s_encode_mutex);
        return false;
    }

    // Invalidate cache so CPU sees DMA-written data, then copy to process buffer
    esp_cache_msync(s_frame_buffer, FB_SIZE, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    memcpy(s_process_buffer, s_frame_buffer, FB_SIZE);

    // HW JPEG encode from the copy (safe from DMA overwrites)
    jpeg_encode_cfg_t enc_cfg = {
        .width = CAM_WIDTH,
        .height = CAM_HEIGHT,
        .src_type = JPEG_ENCODE_IN_FORMAT_RGB565,
        .sub_sample = JPEG_DOWN_SAMPLING_YUV422,
        .image_quality = 40,
    };
    uint32_t jpeg_size = 0;
    esp_err_t err = jpeg_encoder_process(s_jpeg_enc, &enc_cfg, s_process_buffer, FB_SIZE,
                                          s_jpeg_buf, JPEG_OUT_BUF_SIZE, &jpeg_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "JPEG encode failed: %s", esp_err_to_name(err));
        xSemaphoreGive(s_encode_mutex);
        return false;
    }

    // Copy JPEG to caller-owned buffer
    *out_buf = malloc(jpeg_size);
    if (!*out_buf) {
        ESP_LOGE(TAG, "Failed to allocate %lu bytes for JPEG output", (unsigned long)jpeg_size);
        xSemaphoreGive(s_encode_mutex);
        return false;
    }
    memcpy(*out_buf, s_jpeg_buf, jpeg_size);
    *out_len = jpeg_size;

    xSemaphoreGive(s_encode_mutex);
    return true;
}

bool camera_is_ready(void)
{
    return s_init_ok;
}
