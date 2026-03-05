#include "webserver.h"
#include "dashboard.h"
#include "system_stats.h"
#include "eth_transport.h"

#include <stdio.h>
#include <string.h>

#include "esp_http_server.h"
#include "esp_log.h"
#include "cJSON.h"

static const char *TAG = "webserver";
static app_stats_t s_stats = {0};

static esp_err_t dashboard_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
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

    // micro-ROS app stats
    cJSON_AddNumberToObject(root, "pub_count", s_stats.publish_count);

    cJSON *servos = cJSON_AddArrayToObject(root, "servos");
    for (int i = 0; i < 4; i++) {
        cJSON_AddItemToArray(servos, cJSON_CreateNumber(s_stats.servo_angles[i]));
    }

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

void webserver_start(void)
{
    // Default servo angles to 90
    for (int i = 0; i < 4; i++) {
        s_stats.servo_angles[i] = 90.0f;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = CONFIG_WEBSERVER_PORT;
    config.stack_size = 8192;

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

    ESP_LOGI(TAG, "HTTP server started on port %d", CONFIG_WEBSERVER_PORT);
}

app_stats_t *webserver_get_stats(void)
{
    return &s_stats;
}
