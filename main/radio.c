#include "radio.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netdb.h>
#include <errno.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "driver/i2s_std.h"
#include "driver/sdmmc_host.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_log.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "rgb_led.h"
#include "esp_codec_dev_defaults.h"
#include "esp_codec_dev.h"

#define MINIMP3_IMPLEMENTATION
#define MINIMP3_ONLY_MP3
#define MINIMP3_NO_STDIO
#include "minimp3.h"

static const char *TAG = "radio";

// Audio hardware pins (Waveshare ESP32-P4)
#define I2S_MCLK_PIN        GPIO_NUM_13
#define I2S_BCLK_PIN        GPIO_NUM_12
#define I2S_WS_PIN          GPIO_NUM_10
#define I2S_DOUT_PIN        GPIO_NUM_9
#define I2S_DIN_PIN         GPIO_NUM_11
#define PA_ENABLE_PIN       GPIO_NUM_53
#define ES8311_I2C_ADDR     0x30

// SD card pins (Waveshare ESP32-P4-WiFi6-PoE-ETH)
#define SD_CLK_PIN          43
#define SD_CMD_PIN          44
#define SD_D0_PIN           39
#define SD_D1_PIN           40
#define SD_D2_PIN           41
#define SD_D3_PIN           42

#define SD_MOUNT_POINT      "/sdcard"
#define MAX_MP3_FILES       16
#define MP3_PATH_MAX        128

// Stream settings
#define STREAM_HOST_PRIMARY   "relay4.slayradio.org"
#define STREAM_HOST_FALLBACK  "relay1.slayradio.org"
#define STREAM_PORT           8000
#define STREAM_PATH           "/"

// Buffer sizes
#define RING_BUF_SIZE       (64 * 1024)
#define PREBUFFER_SD        (16 * 1024)
#define PREBUFFER_STREAM    (32 * 1024)
#define READ_BUF_SIZE       4096
#define RECV_BUF_SIZE       8192
#define MP3_BUF_SIZE        (16 * 1024)
#define PCM_BUF_SIZE        (MINIMP3_MAX_SAMPLES_PER_FRAME * 2)

// State
static esp_codec_dev_handle_t s_codec = NULL;
static i2s_chan_handle_t s_tx_handle = NULL;
static volatile bool s_playing = false;
static volatile bool s_stop_req = false;
static volatile bool s_init_ok = false;
static volatile bool s_feeder_done = false;
static volatile int s_skip_req = 0;  // +1 = next, -1 = prev
static volatile bool s_dec_flush = false;
static volatile bool s_paused = false;
static volatile radio_source_t s_source = RADIO_SOURCE_SDCARD;
static int s_volume = 70;
static char s_title[128] = {0};
static char s_status[128] = "Not initialised";
static SemaphoreHandle_t s_title_mutex = NULL;
static TaskHandle_t s_feeder_task_handle = NULL;
static TaskHandle_t s_dec_task_handle = NULL;
static StreamBufferHandle_t s_stream_buf = NULL;

// SD card state
static sdmmc_card_t *s_card = NULL;
static bool s_sd_mounted = false;
static char s_mp3_files[MAX_MP3_FILES][MP3_PATH_MAX];
static int s_mp3_count = 0;
static int s_current_file = 0;

// Decode buffers (static to keep off task stack)
static mp3dec_t s_mp3d;
static int16_t s_pcm[PCM_BUF_SIZE];
static uint8_t s_mp3_buf[MP3_BUF_SIZE];
// I/O buffers
static uint8_t s_read_buf[RECV_BUF_SIZE]; // shared for file read and network recv

// ---- I2S & codec init (unchanged) ----

static bool init_i2s(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    chan_cfg.dma_desc_num = 12;
    chan_cfg.dma_frame_num = 1024;

    esp_err_t err = i2s_new_channel(&chan_cfg, &s_tx_handle, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2S channel create failed: %s", esp_err_to_name(err));
        snprintf(s_status, sizeof(s_status), "I2S channel failed: %s", esp_err_to_name(err));
        return false;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCLK_PIN,
            .bclk = I2S_BCLK_PIN,
            .ws   = I2S_WS_PIN,
            .dout = I2S_DOUT_PIN,
            .din  = I2S_DIN_PIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

    err = i2s_channel_init_std_mode(s_tx_handle, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2S std mode init failed: %s", esp_err_to_name(err));
        snprintf(s_status, sizeof(s_status), "I2S init failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

static bool init_codec(i2c_master_bus_handle_t i2c_bus)
{
    esp_err_t probe_err = i2c_master_probe(i2c_bus, 0x18, 500);
    ESP_LOGI(TAG, "I2C probe ES8311 (0x18): %s", esp_err_to_name(probe_err));
    if (probe_err != ESP_OK) {
        snprintf(s_status, sizeof(s_status), "ES8311 not found at 0x18 (%s)",
                 esp_err_to_name(probe_err));
        return false;
    }

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = I2C_NUM_0,
        .addr = ES8311_I2C_ADDR,
        .bus_handle = i2c_bus,
    };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    if (!ctrl_if) {
        snprintf(s_status, sizeof(s_status), "Codec: I2C ctrl failed");
        return false;
    }

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = I2S_NUM_0,
        .rx_handle = NULL,
        .tx_handle = s_tx_handle,
    };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    if (!data_if) {
        snprintf(s_status, sizeof(s_status), "Codec: I2S data if failed");
        return false;
    }

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    if (!gpio_if) {
        snprintf(s_status, sizeof(s_status), "Codec: GPIO if failed");
        return false;
    }

    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC,
        .master_mode = false,
        .use_mclk = true,
        .pa_pin = PA_ENABLE_PIN,
        .pa_reverted = false,
        .hw_gain = {
            .pa_voltage = 5.0,
            .codec_dac_voltage = 3.3,
        },
        .mclk_div = I2S_MCLK_MULTIPLE_256,
    };
    const audio_codec_if_t *es8311_if = es8311_codec_new(&es8311_cfg);
    if (!es8311_if) {
        snprintf(s_status, sizeof(s_status), "Codec: es8311_codec_new failed");
        return false;
    }

    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = es8311_if,
        .data_if = data_if,
    };
    s_codec = esp_codec_dev_new(&dev_cfg);
    if (!s_codec) {
        snprintf(s_status, sizeof(s_status), "Codec: esp_codec_dev_new failed");
        return false;
    }

    return true;
}

// ---- SD card ----

static bool mount_sd_card(void)
{
    if (s_sd_mounted) return true;

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    // ESP32-P4 requires on-chip LDO for SDMMC I/O power
    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = 4,
    };
    sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;
    esp_err_t pwr_err = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
    if (pwr_err != ESP_OK) {
        ESP_LOGE(TAG, "SD LDO power init failed: %s", esp_err_to_name(pwr_err));
        snprintf(s_status, sizeof(s_status), "SD power init failed");
        return false;
    }
    host.pwr_ctrl_handle = pwr_ctrl_handle;

    sdmmc_slot_config_t slot_cfg = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_cfg.clk = SD_CLK_PIN;
    slot_cfg.cmd = SD_CMD_PIN;
    slot_cfg.d0  = SD_D0_PIN;
    slot_cfg.d1  = SD_D1_PIN;
    slot_cfg.d2  = SD_D2_PIN;
    slot_cfg.d3  = SD_D3_PIN;
    slot_cfg.width = 4;
    slot_cfg.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    ESP_LOGI(TAG, "Mounting SD card...");
    esp_err_t err = esp_vfs_fat_sdmmc_mount(SD_MOUNT_POINT, &host, &slot_cfg,
                                             &mount_cfg, &s_card);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SD card mount failed: %s", esp_err_to_name(err));
        snprintf(s_status, sizeof(s_status), "SD mount failed: %s", esp_err_to_name(err));
        return false;
    }

    sdmmc_card_print_info(stdout, s_card);
    s_sd_mounted = true;
    ESP_LOGI(TAG, "SD card mounted at %s", SD_MOUNT_POINT);
    return true;
}

static int scan_mp3_files(void)
{
    s_mp3_count = 0;

    DIR *dir = opendir(SD_MOUNT_POINT);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open SD card root directory");
        return 0;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL && s_mp3_count < MAX_MP3_FILES) {
        if (entry->d_type == DT_DIR) continue;

        const char *name = entry->d_name;
        size_t len = strlen(name);
        if (len < 5) continue;
        const char *ext = name + len - 4;
        if (strcasecmp(ext, ".mp3") != 0) continue;

        snprintf(s_mp3_files[s_mp3_count], MP3_PATH_MAX, "%s/%s", SD_MOUNT_POINT, name);
        ESP_LOGI(TAG, "Found MP3: %s", s_mp3_files[s_mp3_count]);
        s_mp3_count++;
    }
    closedir(dir);

    ESP_LOGI(TAG, "Found %d MP3 file(s) on SD card", s_mp3_count);
    return s_mp3_count;
}

// ---- SD card feeder task ----
static void sd_feeder_task(void *arg)
{
    (void)arg;

    while (!s_stop_req) {
        if (s_mp3_count == 0) {
            ESP_LOGW(TAG, "No MP3 files found");
            snprintf(s_status, sizeof(s_status), "No MP3 files on SD card");
            break;
        }

        if (s_current_file >= s_mp3_count) {
            s_current_file = 0;
        }

        const char *filepath = s_mp3_files[s_current_file];
        const char *basename = strrchr(filepath, '/');
        basename = basename ? basename + 1 : filepath;

        if (s_title_mutex) xSemaphoreTake(s_title_mutex, portMAX_DELAY);
        snprintf(s_title, sizeof(s_title), "%s", basename);
        if (s_title_mutex) xSemaphoreGive(s_title_mutex);

        ESP_LOGI(TAG, "Playing file %d/%d: %s", s_current_file + 1, s_mp3_count, basename);
        snprintf(s_status, sizeof(s_status), "SD: Playing %d/%d", s_current_file + 1, s_mp3_count);

        FILE *f = fopen(filepath, "rb");
        if (!f) {
            ESP_LOGE(TAG, "Failed to open %s", filepath);
            s_current_file++;
            continue;
        }

        while (!s_stop_req && s_skip_req == 0) {
            size_t bytes_read = fread(s_read_buf, 1, READ_BUF_SIZE, f);
            if (bytes_read == 0) break;

            size_t total_sent = 0;
            while (total_sent < bytes_read && !s_stop_req && s_skip_req == 0) {
                size_t sent = xStreamBufferSend(s_stream_buf,
                                                s_read_buf + total_sent,
                                                bytes_read - total_sent,
                                                pdMS_TO_TICKS(100));
                total_sent += sent;
            }
        }

        fclose(f);

        if (s_skip_req != 0) {
            int skip = s_skip_req;
            s_skip_req = 0;
            // Flush stream buffer so decoder doesn't play leftover data
            xStreamBufferReset(s_stream_buf);
            if (skip > 0) {
                s_current_file++;
            } else {
                s_current_file--;
                if (s_current_file < 0) s_current_file = s_mp3_count - 1;
            }
            ESP_LOGI(TAG, "Skipped to file %d/%d", s_current_file + 1, s_mp3_count);
            s_dec_flush = true;
        } else {
            ESP_LOGI(TAG, "Finished reading %s", basename);
            s_current_file++;
        }
    }

    s_feeder_done = true;
    ESP_LOGI(TAG, "SD feeder task exiting");
    s_feeder_task_handle = NULL;
    vTaskDelete(NULL);
}

// ---- Network stream helpers ----

static int connect_stream(const char *host)
{
    struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res = NULL;
    char port_str[8];
    snprintf(port_str, sizeof(port_str), "%d", STREAM_PORT);

    ESP_LOGI(TAG, "Resolving %s ...", host);
    snprintf(s_status, sizeof(s_status), "DNS: %s", host);
    int err = getaddrinfo(host, port_str, &hints, &res);
    if (err != 0 || res == NULL) {
        ESP_LOGE(TAG, "DNS lookup failed for %s: err=%d", host, err);
        snprintf(s_status, sizeof(s_status), "DNS failed: %s (err %d)", host, err);
        if (res) freeaddrinfo(res);
        return -1;
    }

    int sock = socket(res->ai_family, res->ai_socktype, 0);
    if (sock < 0) {
        ESP_LOGE(TAG, "Socket creation failed: %d", errno);
        freeaddrinfo(res);
        return -1;
    }

    struct timeval tv = { .tv_sec = 5, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    int rcvbuf = 16384;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    ESP_LOGI(TAG, "Connecting to %s:%d ...", host, STREAM_PORT);
    if (connect(sock, res->ai_addr, res->ai_addrlen) != 0) {
        ESP_LOGE(TAG, "Connect to %s:%d failed: %d", host, STREAM_PORT, errno);
        close(sock);
        freeaddrinfo(res);
        return -1;
    }
    freeaddrinfo(res);

    char req[256];
    int len = snprintf(req, sizeof(req),
        "GET %s HTTP/1.0\r\n"
        "Host: %s\r\n"
        "User-Agent: ESP32-P4/1.0\r\n"
        "Icy-MetaData: 1\r\n"
        "Connection: close\r\n"
        "\r\n",
        STREAM_PATH, host);

    if (send(sock, req, len, 0) < 0) {
        ESP_LOGE(TAG, "Failed to send HTTP request");
        close(sock);
        return -1;
    }

    return sock;
}

static int parse_http_headers(int sock)
{
    int metaint = 0;
    char line[256];
    int pos = 0;
    bool header_done = false;

    while (!header_done && !s_stop_req) {
        char c;
        int r = recv(sock, &c, 1, 0);
        if (r <= 0) break;
        if (c == '\r') continue;
        if (c == '\n') {
            line[pos] = '\0';
            if (pos == 0) {
                header_done = true;
            } else {
                if (strncasecmp(line, "icy-metaint:", 12) == 0) {
                    metaint = atoi(line + 12);
                    ESP_LOGI(TAG, "Icy-MetaInt: %d", metaint);
                }
                if (strncasecmp(line, "icy-name:", 9) == 0) {
                    ESP_LOGI(TAG, "Station: %s", line + 9);
                }
            }
            pos = 0;
        } else if (pos < (int)sizeof(line) - 1) {
            line[pos++] = c;
        }
    }
    return metaint;
}

static void parse_icy_metadata(const uint8_t *data, int len)
{
    (void)len;
    const char *start = strstr((const char *)data, "StreamTitle='");
    if (!start) return;
    start += 13;
    const char *end = strchr(start, '\'');
    if (!end) return;

    int title_len = end - start;
    if (title_len >= (int)sizeof(s_title)) title_len = sizeof(s_title) - 1;

    if (s_title_mutex) xSemaphoreTake(s_title_mutex, portMAX_DELAY);
    memcpy(s_title, start, title_len);
    s_title[title_len] = '\0';
    if (s_title_mutex) xSemaphoreGive(s_title_mutex);

    ESP_LOGI(TAG, "Now playing: %s", s_title);
}

static int recv_exact(int sock, uint8_t *buf, int len)
{
    int total = 0;
    while (total < len && !s_stop_req) {
        int r = recv(sock, buf + total, len - total, 0);
        if (r <= 0) return r;
        total += r;
    }
    return total;
}

// ---- Network stream feeder task ----
static void net_feeder_task(void *arg)
{
    (void)arg;

    while (!s_stop_req) {
        snprintf(s_status, sizeof(s_status), "Connecting...");
        int sock = connect_stream(STREAM_HOST_PRIMARY);
        if (sock < 0) {
            ESP_LOGW(TAG, "Primary relay failed, trying fallback...");
            sock = connect_stream(STREAM_HOST_FALLBACK);
        }
        if (sock < 0) {
            ESP_LOGE(TAG, "All relays failed, retrying in 5s...");
            snprintf(s_status, sizeof(s_status), "Connection failed, retrying...");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        ESP_LOGI(TAG, "Connected to stream");

        int metaint = parse_http_headers(sock);
        int meta_countdown = metaint;

        snprintf(s_status, sizeof(s_status), "Buffering...");

        while (!s_stop_req) {
            int to_read = RECV_BUF_SIZE;
            if (metaint > 0 && meta_countdown < to_read) {
                to_read = meta_countdown;
            }

            if (to_read <= 0 && metaint > 0) {
                uint8_t meta_len_byte;
                int r = recv(sock, &meta_len_byte, 1, 0);
                if (r <= 0) break;
                int meta_len = meta_len_byte * 16;
                if (meta_len > 0 && meta_len <= 4080) {
                    uint8_t *meta_buf = malloc(meta_len + 1);
                    if (meta_buf) {
                        int meta_read = recv_exact(sock, meta_buf, meta_len);
                        if (meta_read > 0) {
                            meta_buf[meta_read] = '\0';
                            parse_icy_metadata(meta_buf, meta_read);
                        }
                        free(meta_buf);
                    }
                }
                meta_countdown = metaint;
                continue;
            }

            int received = recv(sock, s_read_buf, to_read, 0);
            if (received <= 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }
                ESP_LOGW(TAG, "Stream recv error: %d", errno);
                break;
            }

            if (metaint > 0) meta_countdown -= received;

            size_t total_sent = 0;
            while (total_sent < (size_t)received && !s_stop_req) {
                size_t sent = xStreamBufferSend(s_stream_buf,
                                                s_read_buf + total_sent,
                                                received - total_sent,
                                                pdMS_TO_TICKS(100));
                total_sent += sent;
            }
        }

        close(sock);
        ESP_LOGI(TAG, "Network connection closed");

        if (!s_stop_req) {
            snprintf(s_status, sizeof(s_status), "Reconnecting...");
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }

    s_feeder_done = true;
    ESP_LOGI(TAG, "Network feeder task exiting");
    s_feeder_task_handle = NULL;
    vTaskDelete(NULL);
}

static void reconfigure_i2s_clock(uint32_t rate)
{
    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(rate);
    clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    i2s_channel_disable(s_tx_handle);
    i2s_channel_reconfig_std_clock(s_tx_handle, &clk_cfg);
    i2s_channel_enable(s_tx_handle);
    ESP_LOGI(TAG, "I2S clock reconfigured to %lu Hz", (unsigned long)rate);
}

// Apply software attenuation to prevent clipping (6 dB headroom)
static void attenuate_pcm(int16_t *buf, int count)
{
    for (int i = 0; i < count; i++) {
        buf[i] = (int16_t)(buf[i] >> 1);
    }
}

// ---- Decoder / playback task ----
static void decode_task(void *arg)
{
    (void)arg;
    mp3dec_frame_info_t info;
    int mp3_fill = 0;
    int prebuffer = (s_source == RADIO_SOURCE_SDCARD) ? PREBUFFER_SD : PREBUFFER_STREAM;
    uint32_t current_rate = 0;
    bool codec_opened = false;

    esp_codec_dev_sample_info_t sample_cfg = {
        .bits_per_sample = 16,
        .channel = 2,
        .channel_mask = 0x03,
        .sample_rate = 44100,
    };

    mp3dec_init(&s_mp3d);

    // Wait for pre-buffer to fill
    ESP_LOGI(TAG, "Waiting for pre-buffer (%d KB)...", prebuffer / 1024);
    while (!s_stop_req) {
        size_t avail = xStreamBufferBytesAvailable(s_stream_buf);
        if (avail >= (size_t)prebuffer) break;
        if (s_feeder_done && avail > 0) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (s_stop_req) goto done;

    ESP_LOGI(TAG, "Pre-buffer complete, starting playback");
    if (s_source == RADIO_SOURCE_STREAM) {
        snprintf(s_status, sizeof(s_status), "Playing");
    }
    s_playing = true;

    // Main decode loop
    while (!s_stop_req) {
        // Handle track skip — flush decoder state
        if (s_dec_flush) {
            s_dec_flush = false;
            mp3_fill = 0;
            mp3dec_init(&s_mp3d);
        }

        // Fill mp3 reassembly buffer
        while (mp3_fill < MP3_BUF_SIZE) {
            size_t got = xStreamBufferReceive(s_stream_buf,
                                              s_mp3_buf + mp3_fill,
                                              MP3_BUF_SIZE - mp3_fill,
                                              0);
            if (got == 0) break;
            mp3_fill += (int)got;
        }

        if (mp3_fill < 512) {
            if (s_feeder_done) {
                ESP_LOGI(TAG, "Feeder done and buffer empty");
                break;
            }
            size_t got = xStreamBufferReceive(s_stream_buf,
                                              s_mp3_buf + mp3_fill,
                                              MP3_BUF_SIZE - mp3_fill,
                                              pdMS_TO_TICKS(100));
            mp3_fill += (int)got;
            if (mp3_fill < 512) continue;
        }

        // Decode frames
        while (mp3_fill >= 512 && !s_stop_req) {
            int samples = mp3dec_decode_frame(&s_mp3d, s_mp3_buf, mp3_fill, s_pcm, &info);
            if (info.frame_bytes == 0) break;

            mp3_fill -= info.frame_bytes;
            if (mp3_fill > 0) {
                memmove(s_mp3_buf, s_mp3_buf + info.frame_bytes, mp3_fill);
            }

            if (samples <= 0) continue;

            // samples = per-channel count from minimp3
            // pcm_count = total int16_t values in s_pcm
            int pcm_count;

            if (info.channels == 1) {
                // Mono to stereo: duplicate each sample
                for (int i = samples - 1; i >= 0; i--) {
                    s_pcm[i * 2 + 1] = s_pcm[i];
                    s_pcm[i * 2] = s_pcm[i];
                }
                pcm_count = samples * 2;
            } else {
                // Stereo: buffer has samples * channels values
                pcm_count = samples * info.channels;
            }

            // Software attenuation to prevent clipping
            attenuate_pcm(s_pcm, pcm_count);

            // Configure codec and I2S on first frame or sample rate change
            if ((uint32_t)info.hz != current_rate) {
                current_rate = (uint32_t)info.hz;
                sample_cfg.sample_rate = current_rate;
                sample_cfg.channel = 2; // always stereo output

                // Reconfigure I2S bus clock to match actual sample rate
                reconfigure_i2s_clock(current_rate);

                // (Re)open codec with correct sample rate
                if (codec_opened) {
                    esp_codec_dev_close(s_codec);
                }
                if (esp_codec_dev_open(s_codec, &sample_cfg) != ESP_CODEC_DEV_OK) {
                    ESP_LOGE(TAG, "Codec open failed at %lu Hz", (unsigned long)current_rate);
                    goto done;
                }
                esp_codec_dev_set_out_vol(s_codec, s_volume);
                codec_opened = true;
                ESP_LOGI(TAG, "Audio: %lu Hz, %d ch -> stereo", (unsigned long)current_rate, info.channels);
            }

            // Feed VU meter if active
            if (rgb_led_vu_active()) {
                int16_t peak = 0;
                for (int i = 0; i < pcm_count; i++) {
                    int16_t abs_val = s_pcm[i] < 0 ? (int16_t)-s_pcm[i] : s_pcm[i];
                    if (abs_val > peak) peak = abs_val;
                }
                rgb_led_vu_feed((float)peak / 32767.0f);
            }

            // Wait while paused
            while (s_paused && !s_stop_req && s_skip_req == 0) {
                vTaskDelay(pdMS_TO_TICKS(50));
            }

            esp_codec_dev_write(s_codec, s_pcm, pcm_count * sizeof(int16_t));
        }
    }

done:
    s_playing = false;
    if (codec_opened) {
        esp_codec_dev_close(s_codec);
    }
    // Disable I2S to stop MCLK/BCLK when not playing
    i2s_channel_disable(s_tx_handle);
    snprintf(s_status, sizeof(s_status), "Stopped");
    s_dec_task_handle = NULL;
    ESP_LOGI(TAG, "Decode task exiting");
    vTaskDelete(NULL);
}

// ---- Public API ----

bool radio_init(i2c_master_bus_handle_t i2c_bus)
{
    s_title_mutex = xSemaphoreCreateMutex();
    if (!s_title_mutex) {
        ESP_LOGE(TAG, "Failed to create title mutex");
        return false;
    }

    if (!init_i2s()) {
        ESP_LOGE(TAG, "I2S init failed");
        return false;
    }
    i2s_channel_enable(s_tx_handle);
    snprintf(s_status, sizeof(s_status), "I2S OK, init codec...");

    if (!init_codec(i2c_bus)) {
        i2s_channel_disable(s_tx_handle);
        ESP_LOGE(TAG, "ES8311 codec init failed (status: %s)", s_status);
        return false;
    }

    i2s_channel_disable(s_tx_handle);

    // Mount SD card and scan for MP3 files
    if (mount_sd_card()) {
        scan_mp3_files();
    } else {
        ESP_LOGW(TAG, "SD card not available, stream-only mode");
    }

    s_init_ok = true;
    if (s_mp3_count > 0) {
        snprintf(s_status, sizeof(s_status), "Ready (SD: %d files)", s_mp3_count);
    } else if (s_sd_mounted) {
        snprintf(s_status, sizeof(s_status), "Ready (no MP3 files on SD)");
    } else {
        snprintf(s_status, sizeof(s_status), "Ready (no SD card)");
    }
    ESP_LOGI(TAG, "Radio initialised (ES8311 + I2S, %d MP3 on SD)", s_mp3_count);
    return true;
}

void radio_set_source(radio_source_t source)
{
    if (source == s_source) return;

    bool was_playing = s_playing;
    if (was_playing) radio_stop();

    s_source = source;
    ESP_LOGI(TAG, "Source changed to %s", source == RADIO_SOURCE_SDCARD ? "SD card" : "Stream");

    if (was_playing) radio_play();
}

radio_source_t radio_get_source(void)
{
    return s_source;
}

void radio_play(void)
{
    if (!s_init_ok) {
        ESP_LOGW(TAG, "Radio not initialised, cannot play");
        return;
    }
    if (s_feeder_task_handle != NULL || s_dec_task_handle != NULL) return;

    if (s_source == RADIO_SOURCE_SDCARD && (!s_sd_mounted || s_mp3_count == 0)) {
        ESP_LOGW(TAG, "SD card not available (mounted=%d, files=%d)", s_sd_mounted, s_mp3_count);
        snprintf(s_status, sizeof(s_status), "No MP3 files on SD card");
        return;
    }

    s_stop_req = false;
    s_feeder_done = false;
    s_paused = false;

    if (s_stream_buf) vStreamBufferDelete(s_stream_buf);
    s_stream_buf = xStreamBufferCreate(RING_BUF_SIZE, 1);
    if (!s_stream_buf) {
        ESP_LOGW(TAG, "Failed to create %d KB stream buffer, trying 32 KB", RING_BUF_SIZE / 1024);
        s_stream_buf = xStreamBufferCreate(32 * 1024, 1);
    }
    if (!s_stream_buf) {
        ESP_LOGE(TAG, "Failed to create stream buffer");
        snprintf(s_status, sizeof(s_status), "No memory for stream buffer");
        return;
    }

    if (s_source == RADIO_SOURCE_SDCARD) {
        s_current_file = 0;
        xTaskCreatePinnedToCore(sd_feeder_task, "radio_sd", 4096, NULL, 5, &s_feeder_task_handle, 0);
    } else {
        xTaskCreatePinnedToCore(net_feeder_task, "radio_net", 6144, NULL, 5, &s_feeder_task_handle, 0);
    }

    xTaskCreatePinnedToCore(decode_task, "radio_dec", 32768, NULL, 7, &s_dec_task_handle, 1);
    ESP_LOGI(TAG, "%s playback started",
             s_source == RADIO_SOURCE_SDCARD ? "SD card" : "Stream");
}

void radio_stop(void)
{
    if (s_feeder_task_handle == NULL && s_dec_task_handle == NULL) return;

    s_stop_req = true;
    for (int i = 0; i < 50 && (s_feeder_task_handle != NULL || s_dec_task_handle != NULL); i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    if (s_feeder_task_handle) { vTaskDelete(s_feeder_task_handle); s_feeder_task_handle = NULL; }
    if (s_dec_task_handle) { vTaskDelete(s_dec_task_handle); s_dec_task_handle = NULL; }

    if (s_stream_buf) { vStreamBufferDelete(s_stream_buf); s_stream_buf = NULL; }

    s_playing = false;
    s_feeder_done = false;
    snprintf(s_status, sizeof(s_status), "Stopped");
    ESP_LOGI(TAG, "Playback stopped");

    if (s_title_mutex) xSemaphoreTake(s_title_mutex, portMAX_DELAY);
    s_title[0] = '\0';
    if (s_title_mutex) xSemaphoreGive(s_title_mutex);
}

void radio_pause(void)
{
    if (!s_playing) return;
    s_paused = !s_paused;
    ESP_LOGI(TAG, "Playback %s", s_paused ? "paused" : "resumed");
}

bool radio_is_paused(void)
{
    return s_paused;
}

void radio_next(void)
{
    if (s_source != RADIO_SOURCE_SDCARD || !s_playing) return;
    s_skip_req = 1;
}

void radio_prev(void)
{
    if (s_source != RADIO_SOURCE_SDCARD || !s_playing) return;
    s_skip_req = -1;
}

void radio_set_volume(int vol)
{
    if (vol < 0) vol = 0;
    if (vol > 100) vol = 100;
    s_volume = vol;
    if (s_codec && s_playing) {
        esp_codec_dev_set_out_vol(s_codec, vol);
    }
}

int radio_get_volume(void)
{
    return s_volume;
}

bool radio_is_playing(void)
{
    return s_playing;
}

void radio_get_title(char *buf, int len)
{
    if (!buf || len <= 0) return;
    if (s_title_mutex) xSemaphoreTake(s_title_mutex, portMAX_DELAY);
    strncpy(buf, s_title, len - 1);
    buf[len - 1] = '\0';
    if (s_title_mutex) xSemaphoreGive(s_title_mutex);
}

void radio_get_status(char *buf, int len)
{
    if (!buf || len <= 0) return;
    strncpy(buf, s_status, len - 1);
    buf[len - 1] = '\0';
}
