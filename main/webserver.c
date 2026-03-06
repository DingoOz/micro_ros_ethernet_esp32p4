#include "webserver.h"
#include "dashboard.h"
#include "system_stats.h"
#include "eth_transport.h"
// #include "wifi_transport.h"  // TODO: re-enable after C6 slave firmware
#include "pca9685.h"
#include "radio.h"
#include "rgb_led.h"
#include "ultrasonic.h"
#include "ssd1306.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_random.h"
#include "cJSON.h"

static const char *TAG = "webserver";
static app_stats_t s_stats = {0};

static esp_err_t dashboard_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    return httpd_resp_send(req, dashboard_html, sizeof(dashboard_html) - 1);
}

static esp_err_t api_stats_handler(httpd_req_t *req)
{
    // Collect system stats
    system_stats_t sys;
    system_stats_collect(&sys);

    // Get Ethernet status
    eth_status_t eth;
    eth_transport_get_status(&eth);

    // Build JSON with cJSON
    cJSON *root = cJSON_CreateObject();

    // System
    cJSON_AddStringToObject(root, "chip_model", sys.chip_model);
    cJSON_AddNumberToObject(root, "chip_rev", sys.chip_revision);
    cJSON_AddStringToObject(root, "reset_reason", sys.reset_reason);
    cJSON_AddNumberToObject(root, "cpu_freq_mhz", sys.cpu_freq_mhz);
    cJSON_AddNumberToObject(root, "temperature", sys.temperature_c);
    cJSON_AddNumberToObject(root, "uptime_s", sys.uptime_s);

    // Memory
    cJSON_AddNumberToObject(root, "internal_free", sys.internal_free);
    cJSON_AddNumberToObject(root, "internal_total", sys.internal_total);
    cJSON_AddNumberToObject(root, "psram_free", sys.psram_free);
    cJSON_AddNumberToObject(root, "psram_total", sys.psram_total);
    cJSON_AddNumberToObject(root, "heap_min_free", sys.heap_min_free);
    cJSON_AddNumberToObject(root, "flash_size", sys.flash_size);

    // Ethernet
    cJSON_AddBoolToObject(root, "eth_link", eth.link_up);
    cJSON_AddBoolToObject(root, "eth_duplex", eth.full_duplex);
    cJSON_AddNumberToObject(root, "eth_speed", eth.speed);
    cJSON_AddStringToObject(root, "eth_phy", eth.phy_chip ? eth.phy_chip : "Unknown");
    cJSON_AddStringToObject(root, "eth_mac", eth.mac_str);
    cJSON_AddStringToObject(root, "eth_ip", eth.ip_str);
    cJSON_AddStringToObject(root, "eth_gw", eth.gw_str);
    cJSON_AddStringToObject(root, "eth_netmask", eth.netmask_str);

    // WiFi — disabled until C6 slave firmware is flashed
    // wifi_status_t wifi;
    // wifi_transport_get_status(&wifi);
    cJSON_AddBoolToObject(root, "wifi_connected", false);
    cJSON_AddBoolToObject(root, "wifi_got_ip", false);
    cJSON_AddNumberToObject(root, "wifi_rssi", 0);
    cJSON_AddNumberToObject(root, "wifi_channel", 0);
    cJSON_AddStringToObject(root, "wifi_ssid", "");
    cJSON_AddStringToObject(root, "wifi_ip", "");
    cJSON_AddStringToObject(root, "wifi_gw", "");
    cJSON_AddStringToObject(root, "wifi_netmask", "");
    cJSON_AddStringToObject(root, "wifi_mac", "");
    cJSON_AddStringToObject(root, "wifi_auth", "");

    // micro-ROS app stats
    cJSON_AddNumberToObject(root, "pub_count", s_stats.publish_count);

    cJSON *servos = cJSON_AddArrayToObject(root, "servos");
    for (int i = 0; i < 4; i++) {
        cJSON_AddItemToArray(servos, cJSON_CreateNumber(s_stats.servo_angles[i]));
    }

    // Radio status
    cJSON_AddBoolToObject(root, "radio_playing", radio_is_playing());
    cJSON_AddBoolToObject(root, "radio_paused", radio_is_paused());
    cJSON_AddNumberToObject(root, "radio_volume", radio_get_volume());
    cJSON_AddStringToObject(root, "radio_source",
                            radio_get_source() == RADIO_SOURCE_SDCARD ? "sdcard" : "stream");
    char title[128];
    radio_get_title(title, sizeof(title));
    cJSON_AddStringToObject(root, "radio_title", title);
    char radio_status[128];
    radio_get_status(radio_status, sizeof(radio_status));
    cJSON_AddStringToObject(root, "radio_status", radio_status);

    // Ultrasonic rangefinder
    int dist_mm = ultrasonic_read_mm();
    cJSON_AddNumberToObject(root, "ultrasonic_mm", dist_mm);

    // LED status
    cJSON_AddBoolToObject(root, "led_lightshow", rgb_led_lightshow_active());
    cJSON_AddBoolToObject(root, "led_vu", rgb_led_vu_active());

    // FreeRTOS tasks
    cJSON_AddNumberToObject(root, "task_count", sys.task_count);
    cJSON_AddNumberToObject(root, "core0_tasks", sys.core0_tasks);
    cJSON_AddNumberToObject(root, "core1_tasks", sys.core1_tasks);

    cJSON *tasks = cJSON_AddArrayToObject(root, "tasks");
    for (uint32_t i = 0; i < sys.task_count; i++) {
        cJSON *task = cJSON_CreateObject();
        cJSON_AddStringToObject(task, "name", sys.tasks[i].name);
        cJSON_AddNumberToObject(task, "core", sys.tasks[i].core_id);
        cJSON_AddNumberToObject(task, "pri", sys.tasks[i].priority);
        cJSON_AddNumberToObject(task, "hwm", sys.tasks[i].stack_hwm);
        cJSON_AddItemToArray(tasks, task);
    }

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t ret = httpd_resp_send(req, json_str, strlen(json_str));
    free(json_str);

    return ret;
}

static esp_err_t bandwidth_test_handler(httpd_req_t *req)
{
    char query[64] = {0};
    size_t total_size = 2 * 1024 * 1024;
    int test_type = 0; // 0=zero, 1=random, 2=pattern

    if (httpd_req_get_url_query_str(req, query, sizeof(query) - 1) == ESP_OK) {
        char param[16];
        if (httpd_query_key_value(query, "size", param, sizeof(param)) == ESP_OK) {
            int val = atoi(param);
            if (val > 0) total_size = (size_t)val;
        }
        if (httpd_query_key_value(query, "type", param, sizeof(param)) == ESP_OK) {
            if (strcmp(param, "random") == 0) test_type = 1;
            else if (strcmp(param, "pattern") == 0) test_type = 2;
        }
    }

    if (total_size > 10 * 1024 * 1024) total_size = 10 * 1024 * 1024;
    if (total_size < 1024) total_size = 1024;

    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char buf[4096];
    uint32_t rng_state = 0x12345678; // xorshift32 seed
    if (test_type == 0) {
        memset(buf, 0, sizeof(buf));
    } else if (test_type == 2) {
        for (size_t i = 0; i < sizeof(buf); i++) buf[i] = (char)(i & 0xFF);
    }

    size_t sent = 0;
    while (sent < total_size) {
        size_t chunk = sizeof(buf);
        if (total_size - sent < chunk) chunk = total_size - sent;
        if (test_type == 1) {
            // Fast xorshift32 PRNG — much faster than esp_fill_random() for bandwidth testing
            uint32_t *p = (uint32_t *)buf;
            for (size_t i = 0; i < chunk / 4; i++) {
                rng_state ^= rng_state << 13;
                rng_state ^= rng_state >> 17;
                rng_state ^= rng_state << 5;
                p[i] = rng_state;
            }
        }
        if (httpd_resp_send_chunk(req, buf, chunk) != ESP_OK) {
            httpd_resp_send_chunk(req, NULL, 0);
            return ESP_FAIL;
        }
        sent += chunk;
    }
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t api_radio_handler(httpd_req_t *req)
{
    char buf[128];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body");
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *action = cJSON_GetObjectItem(json, "action");
    cJSON *volume = cJSON_GetObjectItem(json, "volume");

    cJSON *source = cJSON_GetObjectItem(json, "source");

    if (source && cJSON_IsString(source)) {
        if (strcmp(source->valuestring, "sdcard") == 0) {
            radio_set_source(RADIO_SOURCE_SDCARD);
        } else if (strcmp(source->valuestring, "stream") == 0) {
            radio_set_source(RADIO_SOURCE_STREAM);
        }
    }
    if (action && cJSON_IsString(action)) {
        if (strcmp(action->valuestring, "play") == 0) {
            radio_play();
        } else if (strcmp(action->valuestring, "stop") == 0) {
            radio_stop();
        } else if (strcmp(action->valuestring, "pause") == 0) {
            radio_pause();
        } else if (strcmp(action->valuestring, "next") == 0) {
            radio_next();
        } else if (strcmp(action->valuestring, "prev") == 0) {
            radio_prev();
        }
    }
    if (volume && cJSON_IsNumber(volume)) {
        radio_set_volume((int)volume->valuedouble);
    }

    cJSON_Delete(json);

    // Return current status
    cJSON *resp = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp, "playing", radio_is_playing());
    cJSON_AddBoolToObject(resp, "paused", radio_is_paused());
    cJSON_AddNumberToObject(resp, "volume", radio_get_volume());
    cJSON_AddStringToObject(resp, "source",
                            radio_get_source() == RADIO_SOURCE_SDCARD ? "sdcard" : "stream");
    char title[128];
    radio_get_title(title, sizeof(title));
    cJSON_AddStringToObject(resp, "title", title);
    char rstatus[128];
    radio_get_status(rstatus, sizeof(rstatus));
    cJSON_AddStringToObject(resp, "status", rstatus);

    char *json_str = cJSON_PrintUnformatted(resp);
    cJSON_Delete(resp);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t err = httpd_resp_send(req, json_str, strlen(json_str));
    free(json_str);
    return err;
}

static esp_err_t api_led_handler(httpd_req_t *req)
{
    char buf[128];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body");
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *action = cJSON_GetObjectItem(json, "action");
    if (action && cJSON_IsString(action)) {
        if (strcmp(action->valuestring, "lightshow") == 0) {
            if (rgb_led_lightshow_active()) {
                rgb_led_lightshow_stop();
            } else {
                rgb_led_lightshow_start();
            }
        } else if (strcmp(action->valuestring, "vu") == 0) {
            if (rgb_led_vu_active()) {
                rgb_led_vu_stop();
            } else {
                rgb_led_vu_start();
            }
        } else if (strcmp(action->valuestring, "off") == 0) {
            rgb_led_lightshow_stop();
            rgb_led_vu_stop();
            rgb_led_clear();
            rgb_led_show();
        }
    }
    cJSON_Delete(json);

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp, "lightshow", rgb_led_lightshow_active());
    cJSON_AddBoolToObject(resp, "vu", rgb_led_vu_active());
    char *json_str = cJSON_PrintUnformatted(resp);
    cJSON_Delete(resp);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t err = httpd_resp_send(req, json_str, strlen(json_str));
    free(json_str);
    return err;
}

static esp_err_t api_servo_handler(httpd_req_t *req)
{
    char buf[64];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body");
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *ch = cJSON_GetObjectItem(json, "channel");
    cJSON *angle = cJSON_GetObjectItem(json, "angle");
    bool success = false;
    int channel = -1;
    float a = 0;
    if (ch && angle && cJSON_IsNumber(ch) && cJSON_IsNumber(angle)) {
        channel = (int)ch->valuedouble;
        a = (float)angle->valuedouble;
        if (channel >= 0 && channel < 16 && a >= 0 && a <= 180) {
            success = pca9685_set_servo_angle((uint8_t)channel, a);
            s_stats.servo_angles[channel] = a;
        }
    }
    cJSON_Delete(json);

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddBoolToObject(resp, "ok", success);
    cJSON_AddNumberToObject(resp, "channel", channel);
    cJSON_AddNumberToObject(resp, "angle", a);
    char *json_str = cJSON_PrintUnformatted(resp);
    cJSON_Delete(resp);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t err = httpd_resp_send(req, json_str, strlen(json_str));
    free(json_str);
    return err;
}

static esp_err_t api_oled_handler(httpd_req_t *req)
{
    char buf[512];
    int total_len = req->content_len;
    if (total_len <= 0 || total_len >= (int)sizeof(buf)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad length");
        return ESP_FAIL;
    }
    int received = 0;
    while (received < total_len) {
        int ret = httpd_req_recv(req, buf + received, total_len - received);
        if (ret <= 0) {
            httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Recv failed");
            return ESP_FAIL;
        }
        received += ret;
    }
    buf[received] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *action = cJSON_GetObjectItem(json, "action");
    if (action && cJSON_IsString(action)) {
        if (strcmp(action->valuestring, "clear") == 0) {
            ssd1306_clear();
            ssd1306_show();
        } else if (strcmp(action->valuestring, "text") == 0) {
            cJSON *lines = cJSON_GetObjectItem(json, "lines");
            cJSON *large = cJSON_GetObjectItem(json, "large");
            bool use_large = large && cJSON_IsTrue(large);
            ssd1306_clear();
            if (lines && cJSON_IsArray(lines)) {
                int y = 0;
                int line_h = use_large ? 18 : 10;
                cJSON *line;
                cJSON_ArrayForEach(line, lines) {
                    if (cJSON_IsString(line)) {
                        if (use_large) {
                            ssd1306_text_large(0, y, line->valuestring);
                        } else {
                            ssd1306_text(0, y, line->valuestring);
                        }
                    }
                    y += line_h;
                    if (y >= SSD1306_HEIGHT) break;
                }
            }
            ssd1306_show();
        } else if (strcmp(action->valuestring, "invert") == 0) {
            cJSON *val = cJSON_GetObjectItem(json, "value");
            ssd1306_invert(val && cJSON_IsTrue(val));
        } else if (strcmp(action->valuestring, "contrast") == 0) {
            cJSON *val = cJSON_GetObjectItem(json, "value");
            if (val && cJSON_IsNumber(val)) {
                ssd1306_contrast((uint8_t)val->valuedouble);
            }
        } else if (strcmp(action->valuestring, "on") == 0) {
            ssd1306_display_on(true);
        } else if (strcmp(action->valuestring, "off") == 0) {
            ssd1306_display_on(false);
        }
    }
    cJSON_Delete(json);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "{\"ok\":true}", 11);
}

// TODO: re-enable after C6 slave firmware is flashed
// static esp_err_t api_wifi_handler(httpd_req_t *req) { ... }

void webserver_start(void)
{
    // Default servo angles to 90
    for (int i = 0; i < 4; i++) {
        s_stats.servo_angles[i] = 90.0f;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = CONFIG_WEBSERVER_PORT;
    config.stack_size = 16384;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return;
    }

    httpd_uri_t uri_dashboard = {
        .uri      = "/",
        .method   = HTTP_GET,
        .handler  = dashboard_handler,
    };
    httpd_register_uri_handler(server, &uri_dashboard);

    httpd_uri_t uri_api = {
        .uri      = "/api/stats",
        .method   = HTTP_GET,
        .handler  = api_stats_handler,
    };
    httpd_register_uri_handler(server, &uri_api);

    httpd_uri_t uri_bw = {
        .uri      = "/api/bandwidth-test",
        .method   = HTTP_GET,
        .handler  = bandwidth_test_handler,
    };
    httpd_register_uri_handler(server, &uri_bw);

    httpd_uri_t uri_radio = {
        .uri      = "/api/radio",
        .method   = HTTP_POST,
        .handler  = api_radio_handler,
    };
    httpd_register_uri_handler(server, &uri_radio);

    httpd_uri_t uri_led = {
        .uri      = "/api/led",
        .method   = HTTP_POST,
        .handler  = api_led_handler,
    };
    httpd_register_uri_handler(server, &uri_led);

    httpd_uri_t uri_servo = {
        .uri      = "/api/servo",
        .method   = HTTP_POST,
        .handler  = api_servo_handler,
    };
    httpd_register_uri_handler(server, &uri_servo);

    httpd_uri_t uri_oled = {
        .uri      = "/api/oled",
        .method   = HTTP_POST,
        .handler  = api_oled_handler,
    };
    httpd_register_uri_handler(server, &uri_oled);

    // TODO: re-enable after C6 slave firmware
    // httpd_uri_t uri_wifi = { ... };
    // httpd_register_uri_handler(server, &uri_wifi);

    ESP_LOGI(TAG, "HTTP server started on port %d", CONFIG_WEBSERVER_PORT);
}

app_stats_t *webserver_get_stats(void)
{
    return &s_stats;
}
