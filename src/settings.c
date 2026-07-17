#include <string.h>

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#include "config.h"
#include "settings.h"

#define SETTINGS_MAGIC   0x4349584Eu // 'NXIC'
#define SETTINGS_VERSION 1

// Last 4 KB sector of flash
#define SETTINGS_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

settings_t g_settings;

static uint32_t crc32_calc(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

static uint32_t settings_crc(const settings_t *s) {
    return crc32_calc((const uint8_t *)s, offsetof(settings_t, crc));
}

void settings_defaults(settings_t *s) {
    memset(s, 0, sizeof(*s));
    s->magic = SETTINGS_MAGIC;
    s->version = SETTINGS_VERSION;
    s->size = sizeof(settings_t);

    s->yaw_counts_per_360 = GYRO_COUNTS_PER_360;
    s->pitch_counts_per_360 = PITCH_COUNTS_PER_360;
    s->pitch_limit_deg = PITCH_LIMIT_DEG;
    s->invert_yaw = GYRO_INVERT_YAW;
    s->invert_pitch = GYRO_INVERT_PITCH;
    s->rapid_half_period = RAPID_HALF_PERIOD;
    s->led_brightness = LED_BRIGHTNESS;

    s->key_btn_a = KEY_BTN_A;
    s->key_btn_b = KEY_BTN_B;
    s->key_btn_x = KEY_BTN_X;
    s->key_btn_y = KEY_BTN_Y;
    s->key_btn_l = KEY_BTN_L;
    s->key_btn_zl = KEY_BTN_ZL;
    s->key_lstick_click = KEY_LSTICK_CLICK;
    s->key_dpad_up = KEY_DPAD_UP;
    s->key_dpad_down = KEY_DPAD_DOWN;
    s->key_dpad_left = KEY_DPAD_LEFT;
    s->key_dpad_right = KEY_DPAD_RIGHT;
    s->key_plus = KEY_BTN_PLUS;
    s->key_minus = KEY_BTN_MINUS;
    s->key_home = KEY_BTN_HOME;
    s->key_capture = KEY_BTN_CAPTURE;
    s->key_ls_up = KEY_LS_UP;
    s->key_ls_down = KEY_LS_DOWN;
    s->key_ls_left = KEY_LS_LEFT;
    s->key_ls_right = KEY_LS_RIGHT;
    s->key_rs_up = KEY_RS_UP;
    s->key_rs_down = KEY_RS_DOWN;
    s->key_rs_left = KEY_RS_LEFT;
    s->key_rs_right = KEY_RS_RIGHT;
    s->key_y_hold_toggle = KEY_Y_HOLD_TOGGLE;
    s->key_rapid_toggle = KEY_RAPID_TOGGLE;
    s->key_pitch_reset = KEY_PITCH_RESET;
}

void settings_init(void) {
    const settings_t *stored = (const settings_t *)(XIP_BASE + SETTINGS_FLASH_OFFSET);
    if (stored->magic == SETTINGS_MAGIC &&
        stored->version == SETTINGS_VERSION &&
        stored->size == sizeof(settings_t) &&
        settings_crc(stored) == stored->crc) {
        memcpy(&g_settings, stored, sizeof(g_settings));
    } else {
        settings_defaults(&g_settings);
    }
}

bool settings_save(void) {
    g_settings.magic = SETTINGS_MAGIC;
    g_settings.version = SETTINGS_VERSION;
    g_settings.size = sizeof(settings_t);
    g_settings.crc = settings_crc(&g_settings);

    static_assert(sizeof(settings_t) <= FLASH_PAGE_SIZE, "settings must fit one flash page");
    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));
    memcpy(page, &g_settings, sizeof(g_settings));

    uint32_t irq = save_and_disable_interrupts();
    flash_range_erase(SETTINGS_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(SETTINGS_FLASH_OFFSET, page, sizeof(page));
    restore_interrupts(irq);

    const settings_t *stored = (const settings_t *)(XIP_BASE + SETTINGS_FLASH_OFFSET);
    return memcmp(stored, &g_settings, sizeof(g_settings)) == 0;
}
