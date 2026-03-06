#ifndef RGB_LED_H
#define RGB_LED_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"

// Initialise PiicoDev 3x RGB LED module on I2C bus.
bool rgb_led_init(i2c_master_bus_handle_t i2c_bus);

// Set individual LED color (led: 0-2, r/g/b: 0-255).
void rgb_led_set(int led, uint8_t r, uint8_t g, uint8_t b);

// Set all 3 LEDs at once.
void rgb_led_set_all(uint8_t r, uint8_t g, uint8_t b);

// Set global brightness (0-255).
void rgb_led_set_brightness(uint8_t brightness);

// Write current LED values to device.
void rgb_led_show(void);

// Clear all LEDs (off).
void rgb_led_clear(void);

// Start light show task. No-op if already running.
void rgb_led_lightshow_start(void);

// Stop light show task.
void rgb_led_lightshow_stop(void);

// Returns true if light show is running.
bool rgb_led_lightshow_active(void);

// Start VU meter mode (call from audio decode task with PCM levels).
void rgb_led_vu_start(void);

// Stop VU meter mode.
void rgb_led_vu_stop(void);

// Returns true if VU meter is active.
bool rgb_led_vu_active(void);

// Feed audio level to VU meter (0.0 - 1.0). Called from decode task.
void rgb_led_vu_feed(float level);

#endif // RGB_LED_H
