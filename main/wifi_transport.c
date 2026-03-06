#include "wifi_transport.h"

#include <string.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

static const char *TAG = "wifi_transport";

#define WIFI_GOT_IP_BIT   BIT0
#define WIFI_CONNECTED_BIT BIT1
#define WIFI_FAIL_BIT      BIT2

#define WIFI_MAX_RETRY 10

static EventGroupHandle_t s_wifi_event_group;
static wifi_status_t s_status = {0};
static SemaphoreHandle_t s_status_mutex;
static int s_retry_count = 0;
static bool s_user_disconnected = false;
static esp_netif_t *s_wifi_netif = NULL;

static const char *auth_mode_str(wifi_auth_mode_t mode)
{
    switch (mode) {
    case WIFI_AUTH_OPEN:            return "Open";
    case WIFI_AUTH_WEP:             return "WEP";
    case WIFI_AUTH_WPA_PSK:         return "WPA-PSK";
    case WIFI_AUTH_WPA2_PSK:        return "WPA2-PSK";
    case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA/WPA2";
    case WIFI_AUTH_WPA3_PSK:        return "WPA3-PSK";
    case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2/WPA3";
    default:                        return "Unknown";
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "WiFi STA started, connecting...");
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_CONNECTED: {
            wifi_event_sta_connected_t *event = (wifi_event_sta_connected_t *)event_data;
            xSemaphoreTake(s_status_mutex, portMAX_DELAY);
            s_status.connected = true;
            s_status.channel = event->channel;
            memset(s_status.ssid, 0, sizeof(s_status.ssid));
            memcpy(s_status.ssid, event->ssid, event->ssid_len);
            xSemaphoreGive(s_status_mutex);
            ESP_LOGI(TAG, "WiFi connected to %s (ch %d)", s_status.ssid, event->channel);
            break;
        }
        case WIFI_EVENT_STA_DISCONNECTED:
            xSemaphoreTake(s_status_mutex, portMAX_DELAY);
            s_status.connected = false;
            s_status.got_ip = false;
            s_status.rssi = 0;
            xSemaphoreGive(s_status_mutex);
            if (s_user_disconnected) {
                ESP_LOGI(TAG, "WiFi disconnected by user");
            } else if (s_retry_count < WIFI_MAX_RETRY) {
                s_retry_count++;
                ESP_LOGI(TAG, "WiFi disconnected, retry %d/%d", s_retry_count, WIFI_MAX_RETRY);
                esp_wifi_connect();
            } else {
                ESP_LOGW(TAG, "WiFi connection failed after %d retries", WIFI_MAX_RETRY);
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            }
            break;
        default:
            break;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        esp_netif_ip_info_t *ip_info = &event->ip_info;

        xSemaphoreTake(s_status_mutex, portMAX_DELAY);
        s_status.got_ip = true;
        snprintf(s_status.ip_str, sizeof(s_status.ip_str), IPSTR, IP2STR(&ip_info->ip));
        snprintf(s_status.gw_str, sizeof(s_status.gw_str), IPSTR, IP2STR(&ip_info->gw));
        snprintf(s_status.netmask_str, sizeof(s_status.netmask_str), IPSTR, IP2STR(&ip_info->netmask));
        xSemaphoreGive(s_status_mutex);

        s_retry_count = 0;
        ESP_LOGI(TAG, "WiFi got IP: %s", s_status.ip_str);
        xEventGroupSetBits(s_wifi_event_group, WIFI_GOT_IP_BIT);
    }
}

bool wifi_transport_init(void)
{
    s_status_mutex = xSemaphoreCreateMutex();
    s_wifi_event_group = xEventGroupCreate();

    // esp_netif_init() and esp_event_loop_create_default() are already
    // called by eth_transport_init(), so we only create the WiFi STA netif.
    s_wifi_netif = esp_netif_create_default_wifi_sta();

    // Init WiFi with default config
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init failed: %s", esp_err_to_name(err));
        return false;
    }

    // Register event handlers
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                               &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                               &wifi_event_handler, NULL);

    // Configure STA
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char *)wifi_config.sta.ssid, CONFIG_WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, CONFIG_WIFI_PASSWORD, sizeof(wifi_config.sta.password) - 1);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);

    // Get MAC address
    uint8_t mac_addr[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac_addr);
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    snprintf(s_status.mac_str, sizeof(s_status.mac_str),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5]);
    xSemaphoreGive(s_status_mutex);

    // Start WiFi
    err = esp_wifi_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start failed: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "Waiting for WiFi connection...");
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                            WIFI_GOT_IP_BIT | WIFI_FAIL_BIT,
                                            pdFALSE, pdFALSE,
                                            pdMS_TO_TICKS(30000));

    if (bits & WIFI_GOT_IP_BIT) {
        // Query RSSI and auth mode
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            xSemaphoreTake(s_status_mutex, portMAX_DELAY);
            s_status.rssi = ap_info.rssi;
            strncpy(s_status.auth_mode, auth_mode_str(ap_info.authmode),
                    sizeof(s_status.auth_mode) - 1);
            xSemaphoreGive(s_status_mutex);
        }
        ESP_LOGI(TAG, "WiFi connected, RSSI: %d dBm", s_status.rssi);
        return true;
    }

    ESP_LOGW(TAG, "WiFi connection timeout");
    return false;
}

void wifi_transport_connect(void)
{
    ESP_LOGI(TAG, "WiFi connect requested");
    s_user_disconnected = false;
    s_retry_count = 0;
    xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
    esp_wifi_connect();
}

void wifi_transport_disconnect(void)
{
    ESP_LOGI(TAG, "WiFi disconnect requested");
    s_user_disconnected = true;
    esp_wifi_disconnect();
}

void wifi_transport_get_status(wifi_status_t *status)
{
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);

    // Update RSSI each time status is queried
    if (s_status.connected) {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            s_status.rssi = ap_info.rssi;
        }
    }

    memcpy(status, &s_status, sizeof(wifi_status_t));
    xSemaphoreGive(s_status_mutex);
}
