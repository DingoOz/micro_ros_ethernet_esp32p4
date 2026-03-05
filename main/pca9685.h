#ifndef PCA9685_H
#define PCA9685_H

#include <stdbool.h>
#include <stdint.h>

// PCA9685 registers
#define PCA9685_MODE1       0x00
#define PCA9685_MODE2       0x01
#define PCA9685_PRE_SCALE   0xFE
#define PCA9685_LED0_ON_L   0x06

// Servo constants (50 Hz, 4096 counts per cycle)
#define PCA9685_SERVO_FREQ_HZ   50
#define PCA9685_PRESCALE_50HZ   121   // round(25MHz / (4096 * 50)) - 1
#define PCA9685_SERVO_MIN_COUNT 204   // ~1.0 ms pulse -> 0 deg
#define PCA9685_SERVO_MAX_COUNT 409   // ~2.0 ms pulse -> 180 deg

// Initialise PCA9685 on the given I2C bus.
// Returns true on success.
bool pca9685_init(int sda_pin, int scl_pin, uint8_t addr);

// Set a servo channel (0-15) to the specified angle (0-180 deg).
// Returns true on success.
bool pca9685_set_servo_angle(uint8_t channel, float angle_deg);

#endif // PCA9685_H
