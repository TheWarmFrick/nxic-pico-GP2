// NXIC-pico: USB keyboard/mouse -> Nintendo Switch Pro Controller converter
// for the Waveshare RP2350-USB-A.
//
//   core0: TinyUSB device stack on the native USB (USB-C) -> Switch dock
//   core1: TinyUSB host stack on Pico-PIO-USB GP12/GP13 (USB-A) -> hub + kbd/mouse

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/watchdog.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "pio_usb.h"
#include "tusb.h"

#include "config.h"
#include "config_mode.h"
#include "hid_input.h"
#include "mapping.h"
#include "procon.h"
#include "settings.h"
#include "ws2812.pio.h"

// Watchdog scratch magic that requests web config mode after reboot
#define CONFIG_MODE_MAGIC 0x4E434647u // 'NCFG'

static void core1_main(void) {
    sleep_ms(10);

    pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
    pio_cfg.pin_dp = PIN_USB_HOST_DP; // D+ = GP12, D- = GP13
    tuh_configure(BOARD_TUH_RHPORT, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg);
    tuh_init(BOARD_TUH_RHPORT);

    for (;;) {
        tuh_task();
    }
}

// WS2812 RGB LED on GP16, driven by PIO2 (PIO-USB occupies PIO0/PIO1)
#define WS2812_PIO pio2
static uint ws2812_sm;

static void ws2812_init(void) {
    uint offset = pio_add_program(WS2812_PIO, &ws2812_program);
    ws2812_sm = pio_claim_unused_sm(WS2812_PIO, true);
    ws2812_program_init(WS2812_PIO, ws2812_sm, offset, PIN_WS2812, 800000.0f, false);
}

void led_set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    static uint32_t last_color = 0xFFFFFFFF;
    // The RP2350-USB-A's LED takes RGB byte order (not the usual WS2812 GRB)
    uint32_t color = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    if (color != last_color) {
        last_color = color;
        pio_sm_put_blocking(WS2812_PIO, ws2812_sm, color << 8u);
    }
}

static void led_task(void) {
    uint32_t ms = to_ms_since_boot(get_absolute_time());
    uint8_t bright = g_settings.led_brightness;
    uint8_t r = 0, g = 0, b = 0;

    if (!procon_active()) {
        // Waiting for the Switch handshake: blinking blue
        if ((ms / 500) & 1) b = bright;
    } else {
        switch (mapping_debug_accel_mode()) {
            default: g = bright; break;             // attitude tilt: green
            case 1:  r = bright; g = bright; break; // constant flat: yellow
            case 2:  r = bright; break;             // zero (NXIC): red
        }
    }
    led_set_rgb(r, g, b);
}

// Ctrl+Alt+W reboots into the web configurator
static void config_chord_task(void) {
    uint8_t mods = hid_kbd_modifiers();
    if ((mods & KEYBOARD_MODIFIER_LEFTCTRL) && (mods & KEYBOARD_MODIFIER_LEFTALT) &&
        hid_key_down(HID_KEY_W)) {
        watchdog_hw->scratch[2] = CONFIG_MODE_MAGIC;
        watchdog_reboot(0, 0, 0);
        for (;;) tight_loop_contents();
    }
}

int main(void) {
    // Pico-PIO-USB requires the system clock to be a multiple of 12 MHz
    set_sys_clock_khz(120000, true);

    settings_init();
    ws2812_init();

    if (watchdog_hw->scratch[2] == CONFIG_MODE_MAGIC) {
        watchdog_hw->scratch[2] = 0;
        config_mode_main(); // core0 only, never returns
    }

    hid_input_init();
    procon_init();

    multicore_reset_core1();
    multicore_launch_core1(core1_main);

    tud_init(BOARD_TUD_RHPORT);

    for (;;) {
        tud_task();
        procon_task();
        config_chord_task();
        led_task();
    }
}
