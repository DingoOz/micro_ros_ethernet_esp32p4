#include "slide_pot.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "slide_pot";

// Register addresses
#define REG_WHOAMI      0x01
#define REG_FIRM_MAJ    0x02
#define REG_FIRM_MIN    0x03
#define REG_I2C_ADDR    0x04
#define REG_POT         0x05
#define REG_LED         0x07
#define REG_SELF_TEST   0x09

static i2c_master_dev_handle_t s_dev = NULL;
static bool s_init = false;

static bool write_reg(uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t buf[8];
    buf[0] = reg;
    if (len > sizeof(buf) - 1) return false;
    memcpy(buf + 1, data, len);
    return i2c_master_transmit(s_dev, buf, 1 + len, 200) == ESP_OK;
}

static bool read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t cmd = reg;
    return i2c_master_transmit_receive(s_dev, &cmd, 1, data, len, 200) == ESP_OK;
}

bool slide_pot_init(i2c_master_bus_handle_t bus_handle, uint8_t addr)
{
    if (!bus_handle) return false;

    esp_err_t probe_err = i2c_master_probe(bus_handle, addr, 200);
    if (probe_err != ESP_OK) {
        ESP_LOGW(TAG, "Slide pot not found at 0x%02X (%s)", addr, esp_err_to_name(probe_err));
        return false;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(err));
        return false;
    }

    uint16_t id = slide_pot_whoami();
    ESP_LOGI(TAG, "WHO_AM_I: %u (expected %u)", id, SLIDE_POT_DEVICE_ID);

    // Read firmware version
    uint8_t maj = 0, min = 0;
    read_reg(REG_FIRM_MAJ, &maj, 1);
    read_reg(REG_FIRM_MIN, &min, 1);
    ESP_LOGI(TAG, "Slide pot firmware v%u.%u at 0x%02X", maj, min, addr);

    s_init = true;
    return true;
}

int slide_pot_read_raw(void)
{
    if (!s_init) return -1;
    uint8_t data[2];
    if (!read_reg(REG_POT, data, 2)) return -1;
    // 16-bit big-endian
    return (data[0] << 8) | data[1];
}

int slide_pot_read_percent(void)
{
    int raw = slide_pot_read_raw();
    if (raw < 0) return -1;
    return (raw * 100) / 1023;
}

uint16_t slide_pot_whoami(void)
{
    uint8_t data[2];
    if (!read_reg(REG_WHOAMI, data, 2)) return 0;
    return (data[0] << 8) | data[1];
}

void slide_pot_set_led(bool on)
{
    if (!s_init) return;
    uint8_t val = on ? 0x80 : 0x00;
    write_reg(REG_LED, &val, 1);
}
