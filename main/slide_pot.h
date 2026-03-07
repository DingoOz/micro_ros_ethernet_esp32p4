#ifndef SLIDE_POT_H
#define SLIDE_POT_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"

// Default address 0x35 conflicts with ultrasonic — use 0x0F (ASW switch 1-3 on)
#define SLIDE_POT_DEFAULT_ADDR 0x0F
#define SLIDE_POT_DEVICE_ID    411

// Initialize PiicoDev slide potentiometer on an existing I2C bus.
bool slide_pot_init(i2c_master_bus_handle_t bus_handle, uint8_t addr);

// Read raw ADC value (0-1023).
int slide_pot_read_raw(void);

// Read scaled value (0-100).
int slide_pot_read_percent(void);

// Get WHO_AM_I device ID.
uint16_t slide_pot_whoami(void);

// Control onboard LED.
void slide_pot_set_led(bool on);

#endif // SLIDE_POT_H
