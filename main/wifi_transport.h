#ifndef WIFI_TRANSPORT_H
#define WIFI_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool connected;
    bool got_ip;
    int8_t rssi;
    uint8_t channel;
    char ssid[33];
    char ip_str[16];
    char gw_str[16];
    char netmask_str[16];
    char mac_str[18];
    char auth_mode[16];
} wifi_status_t;

// Initialize WiFi STA and connect.
// Blocks until IP is assigned or timeout.
// Returns true if IP was obtained.
bool wifi_transport_init(void);

// Get current WiFi status snapshot
void wifi_transport_get_status(wifi_status_t *status);

// Connect WiFi (resets retry counter and initiates connection)
void wifi_transport_connect(void);

// Disconnect WiFi (stops auto-reconnect)
void wifi_transport_disconnect(void);

#endif // WIFI_TRANSPORT_H
