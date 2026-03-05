#ifndef ETH_TRANSPORT_H
#define ETH_TRANSPORT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool link_up;
    bool got_ip;
    uint8_t speed;          // 10 or 100 Mbps
    bool full_duplex;
    char ip_str[16];
    char gw_str[16];
    char netmask_str[16];
    char mac_str[18];
    const char *phy_chip;
} eth_status_t;

// Initialize EMAC + IP101GRI PHY + esp_netif with DHCP.
// Blocks until IP is assigned or timeout.
// Returns true if IP was obtained.
bool eth_transport_init(void);

// Get current Ethernet status snapshot
void eth_transport_get_status(eth_status_t *status);

#endif // ETH_TRANSPORT_H
