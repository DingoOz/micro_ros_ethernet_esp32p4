#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"

#define ULTRASONIC_DEFAULT_ADDR 0x35
#define ULTRASONIC_DEVICE_ID    578

// Initialize the PiicoDev Ultrasonic Rangefinder on an existing I2C bus.
// Returns true on success.
bool ultrasonic_init(i2c_master_bus_handle_t bus_handle, uint8_t addr);

// Read distance in millimeters. Returns -1 on error.
int ultrasonic_read_mm(void);

// Check if a new sample is available.
bool ultrasonic_new_sample(void);

// Read the device ID (WHOAMI register). Returns 0 on error.
uint16_t ultrasonic_whoami(void);

// Read firmware version. Returns true on success.
bool ultrasonic_firmware(uint8_t *major, uint8_t *minor);

// Control the onboard power LED.
void ultrasonic_set_led(bool on);

#endif // ULTRASONIC_H
