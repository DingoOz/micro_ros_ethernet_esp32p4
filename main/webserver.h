#ifndef WEBSERVER_H
#define WEBSERVER_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"

typedef struct {
    int32_t publish_count;
    float servo_angles[4];
} app_stats_t;

// Start the HTTP server (call after Ethernet is up)
void webserver_start(i2c_master_bus_handle_t i2c_bus);

// Get pointer to app stats (for updating from micro-ROS callbacks)
app_stats_t *webserver_get_stats(void);

#endif // WEBSERVER_H
