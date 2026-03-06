#include "ultrasonic.h"

#include <stdio.h>
#include "esp_log.h"

static const char *TAG = "ultrasonic";

// PiicoDev Ultrasonic Rangefinder registers
#define REG_WHOAMI      0x01
#define REG_FIRM_MAJ    0x02
#define REG_FIRM_MIN    0x03
#define REG_I2C_ADDR    0x04
#define REG_RAW         0x05
#define REG_PERIOD      0x06
#define REG_LED         0x07
#define REG_STATUS      0x08
#define REG_SELF_TEST   0x09

// Write bit (MSB set) for writable registers
#define REG_WRITE_BIT   0x80

static i2c_master_dev_handle_t s_dev = NULL;

// Speed of sound: 0.343 mm/us
// Distance = round_trip_us * 0.343 / 2

static bool read_reg(uint8_t reg, uint8_t *buf, size_t len)
{
    esp_err_t err = i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, 100);
    return err == ESP_OK;
}

static bool write_reg(uint8_t reg, const uint8_t *buf, size_t len)
{
    uint8_t txbuf[4];
    if (len > sizeof(txbuf) - 1) return false;
    txbuf[0] = reg;
    for (size_t i = 0; i < len; i++) txbuf[i + 1] = buf[i];
    esp_err_t err = i2c_master_transmit(s_dev, txbuf, 1 + len, 100);
    return err == ESP_OK;
}

static uint16_t read_u16(uint8_t reg)
{
    uint8_t buf[2] = {0};
    if (!read_reg(reg, buf, 2)) return 0;
    return ((uint16_t)buf[0] << 8) | buf[1]; // big-endian
}

bool ultrasonic_init(i2c_master_bus_handle_t bus_handle, uint8_t addr)
{
    // Probe device
    esp_err_t probe_err = i2c_master_probe(bus_handle, addr, 200);
    if (probe_err != ESP_OK) {
        ESP_LOGW(TAG, "Ultrasonic not found at 0x%02X (%s)", addr, esp_err_to_name(probe_err));
        return false;
    }
    ESP_LOGI(TAG, "Ultrasonic found at 0x%02X", addr);

    // Add device to bus
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_config, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C device add failed: %s", esp_err_to_name(err));
        return false;
    }

    // Verify WHOAMI
    uint16_t id = ultrasonic_whoami();
    if (id != ULTRASONIC_DEVICE_ID) {
        ESP_LOGW(TAG, "Unexpected WHOAMI: %d (expected %d)", id, ULTRASONIC_DEVICE_ID);
    }

    uint8_t maj, min;
    if (ultrasonic_firmware(&maj, &min)) {
        ESP_LOGI(TAG, "Ultrasonic firmware v%d.%d", maj, min);
    }

    // Set sample period to 20ms
    uint8_t period_buf[2] = {0, 20}; // big-endian 20
    write_reg(REG_PERIOD | REG_WRITE_BIT, period_buf, 2);

    ESP_LOGI(TAG, "Ultrasonic rangefinder initialised");
    return true;
}

int ultrasonic_read_mm(void)
{
    if (s_dev == NULL) return -1;
    uint16_t raw = read_u16(REG_RAW);
    if (raw == 0) return -1;
    // raw is round-trip time in microseconds
    // distance_mm = raw * 0.343 / 2
    int mm = (int)(raw * 343 / 2000);
    return mm;
}

bool ultrasonic_new_sample(void)
{
    if (s_dev == NULL) return false;
    uint8_t status = 0;
    if (!read_reg(REG_STATUS, &status, 1)) return false;
    return (status & 0x01) != 0;
}

uint16_t ultrasonic_whoami(void)
{
    if (s_dev == NULL) return 0;
    return read_u16(REG_WHOAMI);
}

bool ultrasonic_firmware(uint8_t *major, uint8_t *minor)
{
    if (s_dev == NULL) return false;
    if (!read_reg(REG_FIRM_MAJ, major, 1)) return false;
    if (!read_reg(REG_FIRM_MIN, minor, 1)) return false;
    return true;
}

void ultrasonic_set_led(bool on)
{
    if (s_dev == NULL) return;
    uint8_t val = on ? 1 : 0;
    write_reg(REG_LED | REG_WRITE_BIT, &val, 1);
}
