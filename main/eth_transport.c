#include "eth_transport.h"

#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_eth.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

static const char *TAG = "eth_transport";

#define ETH_GOT_IP_BIT BIT0

static EventGroupHandle_t s_eth_event_group;
static eth_status_t s_status = {0};
static SemaphoreHandle_t s_status_mutex;

static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Ethernet link up");
        xSemaphoreTake(s_status_mutex, portMAX_DELAY);
        s_status.link_up = true;
        xSemaphoreGive(s_status_mutex);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet link down");
        xSemaphoreTake(s_status_mutex, portMAX_DELAY);
        s_status.link_up = false;
        s_status.got_ip = false;
        xSemaphoreGive(s_status_mutex);
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet stopped");
        break;
    default:
        break;
    }
}

static void ip_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data)
{
    if (event_id == IP_EVENT_ETH_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        esp_netif_ip_info_t *ip_info = &event->ip_info;

        xSemaphoreTake(s_status_mutex, portMAX_DELAY);
        s_status.got_ip = true;
        snprintf(s_status.ip_str, sizeof(s_status.ip_str), IPSTR, IP2STR(&ip_info->ip));
        snprintf(s_status.gw_str, sizeof(s_status.gw_str), IPSTR, IP2STR(&ip_info->gw));
        snprintf(s_status.netmask_str, sizeof(s_status.netmask_str), IPSTR, IP2STR(&ip_info->netmask));
        xSemaphoreGive(s_status_mutex);

        ESP_LOGI(TAG, "Got IP: %s", s_status.ip_str);
        xEventGroupSetBits(s_eth_event_group, ETH_GOT_IP_BIT);
    }
}

bool eth_transport_init(void)
{
    s_status_mutex = xSemaphoreCreateMutex();
    s_eth_event_group = xEventGroupCreate();

    // Initialize TCP/IP and event loop
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create default Ethernet netif
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&netif_cfg);

    // Configure EMAC
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_config.smi_gpio.mdc_num = CONFIG_ETH_MDC_GPIO;
    emac_config.smi_gpio.mdio_num = CONFIG_ETH_MDIO_GPIO;
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_config, &mac_config);

    // Configure IP101GRI PHY
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = CONFIG_ETH_PHY_ADDR;
    phy_config.reset_gpio_num = CONFIG_ETH_PHY_RST_GPIO;
    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_config);

    // Install Ethernet driver
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));

    // Attach driver to netif
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                                &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                                &ip_event_handler, NULL));

    // Get MAC address
    uint8_t mac_addr[6];
    esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    snprintf(s_status.mac_str, sizeof(s_status.mac_str),
             "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5]);
    s_status.phy_chip = "IP101GRI";
    xSemaphoreGive(s_status_mutex);

    // Start Ethernet
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

    // Query link speed/duplex after a short delay for negotiation
    ESP_LOGI(TAG, "Waiting for IP address...");
    EventBits_t bits = xEventGroupWaitBits(s_eth_event_group, ETH_GOT_IP_BIT,
                                            pdFALSE, pdTRUE,
                                            pdMS_TO_TICKS(30000));

    if (bits & ETH_GOT_IP_BIT) {
        // Query speed and duplex
        eth_speed_t speed;
        eth_duplex_t duplex;
        esp_eth_ioctl(eth_handle, ETH_CMD_G_SPEED, &speed);
        esp_eth_ioctl(eth_handle, ETH_CMD_G_DUPLEX_MODE, &duplex);

        xSemaphoreTake(s_status_mutex, portMAX_DELAY);
        s_status.speed = (speed == ETH_SPEED_100M) ? 100 : 10;
        s_status.full_duplex = (duplex == ETH_DUPLEX_FULL);
        xSemaphoreGive(s_status_mutex);

        ESP_LOGI(TAG, "Link: %d Mbps %s-duplex",
                 s_status.speed, s_status.full_duplex ? "full" : "half");
        return true;
    }

    ESP_LOGW(TAG, "Timeout waiting for IP address");
    return false;
}

void eth_transport_get_status(eth_status_t *status)
{
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    memcpy(status, &s_status, sizeof(eth_status_t));
    xSemaphoreGive(s_status_mutex);
}
