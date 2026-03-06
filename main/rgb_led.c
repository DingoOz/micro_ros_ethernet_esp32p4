#include "rgb_led.h"

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "rgb_led";

#define PIICODEV_RGB_ADDR   0x08
#define REG_CTRL            0x03
#define REG_CLEAR           0x04
#define REG_BRIGHTNESS      0x06
#define REG_LED_VALUES      0x07

static i2c_master_dev_handle_t s_dev = NULL;
static bool s_init_ok = false;
static uint8_t s_leds[9] = {0}; // 3 LEDs x 3 bytes (R,G,B)

static TaskHandle_t s_show_task = NULL;
static volatile bool s_show_stop = false;

static TaskHandle_t s_vu_task = NULL;
static volatile bool s_vu_stop = false;
static volatile float s_vu_level = 0.0f;

static bool i2c_write_reg(uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t buf[16];
    if (len + 1 > sizeof(buf)) return false;
    buf[0] = reg;
    memcpy(buf + 1, data, len);
    return i2c_master_transmit(s_dev, buf, len + 1, 100) == ESP_OK;
}

bool rgb_led_init(i2c_master_bus_handle_t i2c_bus)
{
    esp_err_t err = i2c_master_probe(i2c_bus, PIICODEV_RGB_ADDR, 100);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "PiicoDev RGB not found at 0x%02X", PIICODEV_RGB_ADDR);
        return false;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PIICODEV_RGB_ADDR,
        .scl_speed_hz = 100000,
    };
    err = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(err));
        return false;
    }

    // Power on the LED controller
    uint8_t on = 1;
    i2c_write_reg(REG_CTRL, &on, 1);

    // Set brightness to medium
    rgb_led_set_brightness(50);
    rgb_led_clear();

    s_init_ok = true;
    ESP_LOGI(TAG, "PiicoDev 3x RGB LED initialised");
    return true;
}

void rgb_led_set(int led, uint8_t r, uint8_t g, uint8_t b)
{
    if (led < 0 || led > 2) return;
    s_leds[led * 3 + 0] = r;
    s_leds[led * 3 + 1] = g;
    s_leds[led * 3 + 2] = b;
}

void rgb_led_set_all(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < 3; i++) {
        rgb_led_set(i, r, g, b);
    }
}

void rgb_led_set_brightness(uint8_t brightness)
{
    if (!s_init_ok) return;
    i2c_write_reg(REG_BRIGHTNESS, &brightness, 1);
}

void rgb_led_show(void)
{
    if (!s_init_ok) return;
    i2c_write_reg(REG_LED_VALUES, s_leds, 9);
}

void rgb_led_clear(void)
{
    memset(s_leds, 0, sizeof(s_leds));
    if (!s_init_ok) return;
    uint8_t cmd = 1;
    i2c_write_reg(REG_CLEAR, &cmd, 1);
}

// ---- Light show task ----
static void lightshow_task(void *arg)
{
    (void)arg;
    int phase = 0;

    rgb_led_set_brightness(80);

    while (!s_show_stop) {
        // Cycle through color patterns
        int pattern = (phase / 30) % 5;
        int step = phase % 30;
        float t = (float)step / 30.0f;

        switch (pattern) {
        case 0: // Rainbow chase
            for (int i = 0; i < 3; i++) {
                float h = fmodf((t + (float)i / 3.0f) * 360.0f, 360.0f);
                float c = 1.0f, x = 1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f);
                uint8_t r = 0, g = 0, b = 0;
                if (h < 60)       { r = (uint8_t)(c*255); g = (uint8_t)(x*255); }
                else if (h < 120) { r = (uint8_t)(x*255); g = (uint8_t)(c*255); }
                else if (h < 180) { g = (uint8_t)(c*255); b = (uint8_t)(x*255); }
                else if (h < 240) { g = (uint8_t)(x*255); b = (uint8_t)(c*255); }
                else if (h < 300) { r = (uint8_t)(x*255); b = (uint8_t)(c*255); }
                else              { r = (uint8_t)(c*255); b = (uint8_t)(x*255); }
                rgb_led_set(i, r, g, b);
            }
            break;
        case 1: // Pulse red
            { uint8_t v = (uint8_t)(sinf(t * 3.14159f) * 255.0f);
              rgb_led_set_all(v, 0, 0); }
            break;
        case 2: // Pulse green
            { uint8_t v = (uint8_t)(sinf(t * 3.14159f) * 255.0f);
              rgb_led_set_all(0, v, 0); }
            break;
        case 3: // Pulse blue
            { uint8_t v = (uint8_t)(sinf(t * 3.14159f) * 255.0f);
              rgb_led_set_all(0, 0, v); }
            break;
        case 4: // Sparkle white
            for (int i = 0; i < 3; i++) {
                uint8_t v = ((phase + i * 7) % 5 == 0) ? 255 : 0;
                rgb_led_set(i, v, v, v);
            }
            break;
        }

        rgb_led_show();
        phase++;
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    rgb_led_clear();
    rgb_led_show();
    s_show_task = NULL;
    ESP_LOGI(TAG, "Light show stopped");
    vTaskDelete(NULL);
}

void rgb_led_lightshow_start(void)
{
    if (!s_init_ok || s_show_task) return;
    // Stop VU if running
    if (s_vu_task) rgb_led_vu_stop();

    s_show_stop = false;
    xTaskCreatePinnedToCore(lightshow_task, "led_show", 2048, NULL, 3, &s_show_task, 0);
    ESP_LOGI(TAG, "Light show started");
}

void rgb_led_lightshow_stop(void)
{
    if (!s_show_task) return;
    s_show_stop = true;
    for (int i = 0; i < 20 && s_show_task; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (s_show_task) { vTaskDelete(s_show_task); s_show_task = NULL; }
    rgb_led_clear();
    rgb_led_show();
}

bool rgb_led_lightshow_active(void)
{
    return s_show_task != NULL;
}

// ---- VU meter task ----
static void vu_task(void *arg)
{
    (void)arg;

    rgb_led_set_brightness(100);

    while (!s_vu_stop) {
        float level = s_vu_level;

        // Map level to 3 LEDs: green -> yellow -> red
        if (level < 0.33f) {
            // Low: LED 0 green, proportional brightness
            uint8_t v = (uint8_t)(level / 0.33f * 255.0f);
            rgb_led_set(0, 0, v, 0);
            rgb_led_set(1, 0, 0, 0);
            rgb_led_set(2, 0, 0, 0);
        } else if (level < 0.66f) {
            // Mid: LED 0 green, LED 1 yellow proportional
            float mid = (level - 0.33f) / 0.33f;
            uint8_t v = (uint8_t)(mid * 255.0f);
            rgb_led_set(0, 0, 255, 0);
            rgb_led_set(1, v, v, 0);
            rgb_led_set(2, 0, 0, 0);
        } else {
            // High: LED 0 green, LED 1 yellow, LED 2 red proportional
            float hi = (level - 0.66f) / 0.34f;
            uint8_t v = (uint8_t)(hi * 255.0f);
            rgb_led_set(0, 0, 255, 0);
            rgb_led_set(1, 255, 255, 0);
            rgb_led_set(2, v, 0, 0);
        }

        rgb_led_show();
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    rgb_led_clear();
    rgb_led_show();
    s_vu_task = NULL;
    ESP_LOGI(TAG, "VU meter stopped");
    vTaskDelete(NULL);
}

void rgb_led_vu_start(void)
{
    if (!s_init_ok || s_vu_task) return;
    // Stop light show if running
    if (s_show_task) rgb_led_lightshow_stop();

    s_vu_stop = false;
    s_vu_level = 0.0f;
    xTaskCreatePinnedToCore(vu_task, "led_vu", 2048, NULL, 3, &s_vu_task, 0);
    ESP_LOGI(TAG, "VU meter started");
}

void rgb_led_vu_stop(void)
{
    if (!s_vu_task) return;
    s_vu_stop = true;
    for (int i = 0; i < 20 && s_vu_task; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (s_vu_task) { vTaskDelete(s_vu_task); s_vu_task = NULL; }
    rgb_led_clear();
    rgb_led_show();
}

bool rgb_led_vu_active(void)
{
    return s_vu_task != NULL;
}

void rgb_led_vu_feed(float level)
{
    if (level < 0.0f) level = 0.0f;
    if (level > 1.0f) level = 1.0f;
    // Smooth with simple IIR filter
    s_vu_level = s_vu_level * 0.7f + level * 0.3f;
}
