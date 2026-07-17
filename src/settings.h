#ifndef NXIC_SETTINGS_H_
#define NXIC_SETTINGS_H_

#include <stdbool.h>
#include <stdint.h>

// Runtime settings, persisted in the last flash sector. Defaults come from
// config.h; the web configurator (config_mode.c) edits and saves them.

// All members are naturally aligned as ordered, so the layout is stable
// without packing (and unpacked members avoid unaligned-pointer issues)
typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;

    // Gyro
    float yaw_counts_per_360;
    float pitch_counts_per_360;
    float pitch_limit_deg;
    uint8_t invert_yaw;
    uint8_t invert_pitch;

    // Misc
    uint8_t rapid_half_period;
    uint8_t led_brightness;

    // Key map (HID usage codes)
    uint8_t key_btn_a, key_btn_b, key_btn_x, key_btn_y;
    uint8_t key_btn_l, key_btn_zl, key_lstick_click;
    uint8_t key_dpad_up, key_dpad_down, key_dpad_left, key_dpad_right;
    uint8_t key_plus, key_minus, key_home, key_capture;
    uint8_t key_ls_up, key_ls_down, key_ls_left, key_ls_right;
    uint8_t key_rs_up, key_rs_down, key_rs_left, key_rs_right;
    uint8_t key_y_hold_toggle, key_rapid_toggle, key_pitch_reset;

    uint8_t reserved[10];
    uint32_t crc;
} settings_t;

extern settings_t g_settings;

// Load settings from flash (falls back to defaults on magic/CRC mismatch)
void settings_init(void);

// Reset the in-memory settings to compile-time defaults (does not save)
void settings_defaults(settings_t *s);

// Persist g_settings to flash. Must only be called while core1 is not
// running (config mode) -- flash writes stall XIP.
bool settings_save(void);

#endif // NXIC_SETTINGS_H_
