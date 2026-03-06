#ifndef WEBSERVER_H
#define WEBSERVER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int32_t publish_count;
    float servo_angles[4];
} app_stats_t;

// Start the HTTP server (call after Ethernet is up)
void webserver_start(void);

// Get pointer to app stats (for updating from micro-ROS callbacks)
app_stats_t *webserver_get_stats(void);

#endif // WEBSERVER_H
