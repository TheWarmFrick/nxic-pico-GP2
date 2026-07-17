#ifndef NXIC_CONFIG_MODE_H_
#define NXIC_CONFIG_MODE_H_

#include <stdint.h>

// Web configurator: USB NCM network device + DHCP + HTTP at 192.168.7.1.
// Runs on core0 only (core1 must not be started). Never returns.
void config_mode_main(void);

// Provided by main.c (WS2812 status LED)
void led_set_rgb(uint8_t r, uint8_t g, uint8_t b);

#endif // NXIC_CONFIG_MODE_H_
