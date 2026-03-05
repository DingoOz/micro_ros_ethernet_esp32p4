#include "pca9685.h"

#include <stdio.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// MODE1 bits
#define MODE1_RESTART 0x80
#define MODE1_SLEEP   0x10
#define MODE1_AI      0x20  // auto-increment

// MODE2 bits
#define MODE2_OUTDRV  0x04  // totem-pole outputs

static i2c_master_dev_handle_t g_dev = NULL;

static bool pca9685_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    esp_err_t err = i2c_master_transmit(g_dev, buf, 2, 100);
    return err == ESP_OK;
}

static bool pca9685_read_reg(uint8_t reg, uint8_t *val)
{
    esp_err_t err = i2c_master_transmit_receive(g_dev, &reg, 1, val, 1, 100);
    return err == ESP_OK;
}

bool pca9685_init(int sda_pin, int scl_pin, uint8_t addr)
{
    // Configure I2C master bus
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = scl_pin,
        .sda_io_num = sda_pin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
    if (err != ESP_OK) {
        printf("PCA9685: I2C bus init failed: %s\n", esp_err_to_name(err));
        return false;
    }

    // Add PCA9685 device
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,  // 100 kHz
    };

    err = i2c_master_bus_add_device(bus_handle, &dev_config, &g_dev);
    if (err != ESP_OK) {
        printf("PCA9685: I2C device add failed: %s\n", esp_err_to_name(err));
        return false;
    }

    // Put to sleep so we can set the prescaler
    if (!pca9685_write_reg(PCA9685_MODE1, MODE1_SLEEP)) {
        printf("PCA9685: failed to write MODE1\n");
        return false;
    }

    // Set prescaler for 50 Hz
    if (!pca9685_write_reg(PCA9685_PRE_SCALE, PCA9685_PRESCALE_50HZ)) {
        printf("PCA9685: failed to set prescaler\n");
        return false;
    }

    // Totem-pole outputs
    pca9685_write_reg(PCA9685_MODE2, MODE2_OUTDRV);

    // Wake up with auto-increment enabled
    pca9685_write_reg(PCA9685_MODE1, MODE1_AI);
    vTaskDelay(pdMS_TO_TICKS(5));

    // Set restart bit
    uint8_t mode1;
    pca9685_read_reg(PCA9685_MODE1, &mode1);
    pca9685_write_reg(PCA9685_MODE1, mode1 | MODE1_RESTART);

    printf("PCA9685: initialised at 0x%02X\n", addr);
    return true;
}

bool pca9685_set_servo_angle(uint8_t channel, float angle_deg)
{
    if (g_dev == NULL) return false;
    if (channel > 15) return false;

    // Clamp angle
    if (angle_deg < 0.0f)   angle_deg = 0.0f;
    if (angle_deg > 180.0f) angle_deg = 180.0f;

    // Linear interpolation: 0 deg -> MIN_COUNT, 180 deg -> MAX_COUNT
    uint16_t off_count = (uint16_t)(
        PCA9685_SERVO_MIN_COUNT +
        (angle_deg / 180.0f) * (PCA9685_SERVO_MAX_COUNT - PCA9685_SERVO_MIN_COUNT)
    );

    // Each channel has 4 registers: ON_L, ON_H, OFF_L, OFF_H
    uint8_t reg = PCA9685_LED0_ON_L + 4 * channel;
    uint8_t buf[5] = {
        reg,
        0x00,                           // ON_L
        0x00,                           // ON_H
        (uint8_t)(off_count & 0xFF),    // OFF_L
        (uint8_t)(off_count >> 8),      // OFF_H
    };

    esp_err_t err = i2c_master_transmit(g_dev, buf, 5, 100);
    return err == ESP_OK;
}
