#ifndef SSD1306_H
#define SSD1306_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"

#define SSD1306_DEFAULT_ADDR 0x3C
#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64
#define SSD1306_BUF_SIZE (SSD1306_WIDTH * SSD1306_HEIGHT / 8) // 1024 bytes

// Initialize SSD1306 OLED on an existing I2C bus. Returns true on success.
bool ssd1306_init(i2c_master_bus_handle_t bus_handle, uint8_t addr);

// Clear the framebuffer (all pixels off).
void ssd1306_clear(void);

// Set a single pixel. x: 0-127, y: 0-63.
void ssd1306_pixel(int x, int y, bool on);

// Draw a horizontal line.
void ssd1306_hline(int x, int y, int w, bool on);

// Draw a vertical line.
void ssd1306_vline(int x, int y, int h, bool on);

// Draw a rectangle outline.
void ssd1306_rect(int x, int y, int w, int h, bool on);

// Fill a rectangle.
void ssd1306_fill_rect(int x, int y, int w, int h, bool on);

// Draw a character (8x6 font). Returns character width (6).
int ssd1306_char(int x, int y, char c);

// Draw a string. Returns ending x position.
int ssd1306_text(int x, int y, const char *str);

// Draw a string with large font (2x scale). Returns ending x position.
int ssd1306_text_large(int x, int y, const char *str);

// Send the framebuffer to the display.
bool ssd1306_show(void);

// Turn display on/off.
void ssd1306_display_on(bool on);

// Set display contrast (0-255).
void ssd1306_contrast(uint8_t val);

// Invert display colours.
void ssd1306_invert(bool inv);

#endif // SSD1306_H
