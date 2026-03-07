#ifndef CAMERA_H
#define CAMERA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "driver/i2c_master.h"

// Camera resolution (OV5647 via MIPI CSI, downscaled for web streaming)
#define CAM_WIDTH   800
#define CAM_HEIGHT  640

// Register camera module (saves I2C bus handle, no hardware init yet).
bool camera_init(i2c_master_bus_handle_t i2c_bus);

// Start camera hardware (sensor detect, CSI, ISP, JPEG). Call on demand.
bool camera_start(void);

// Capture a single JPEG frame.
// *out_buf receives a pointer to JPEG data in PSRAM (caller must free).
// *out_len receives the JPEG size in bytes.
// Returns true on success.
bool camera_capture_jpeg(uint8_t **out_buf, size_t *out_len);

// Returns true if camera was initialised successfully.
bool camera_is_ready(void);

#endif // CAMERA_H
