#ifndef RADIO_H
#define RADIO_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"

typedef enum {
    RADIO_SOURCE_SDCARD = 0,
    RADIO_SOURCE_STREAM,
} radio_source_t;

// Initialise audio hardware (I2S + ES8311 codec + SD card).
// Must be called once from app_main before play/stop.
bool radio_init(i2c_master_bus_handle_t i2c_bus);

// Set the audio source (SD card or web stream). Stops playback if switching.
void radio_set_source(radio_source_t source);

// Get the current audio source.
radio_source_t radio_get_source(void);

// Start playback from current source. No-op if already playing.
void radio_play(void);

// Stop playback. No-op if already stopped.
void radio_stop(void);

// Toggle pause/resume.
void radio_pause(void);

// Returns true while paused.
bool radio_is_paused(void);

// Skip to next track (SD card mode only).
void radio_next(void);

// Skip to previous track (SD card mode only).
void radio_prev(void);

// Set output volume (0-100).
void radio_set_volume(int vol);

// Get current volume (0-100).
int radio_get_volume(void);

// Returns true while audio is active.
bool radio_is_playing(void);

// Copy the current song title into buf (max len bytes).
void radio_get_title(char *buf, int len);

// Copy status message into buf (for diagnostics).
void radio_get_status(char *buf, int len);

// Get current stereo VU levels (0.0-1.0). Returns false if not playing.
bool radio_get_vu(float *left, float *right);

#endif // RADIO_H
