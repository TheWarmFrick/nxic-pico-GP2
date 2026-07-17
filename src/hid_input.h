#ifndef NXIC_HID_INPUT_H_
#define NXIC_HID_INPUT_H_

#include <stdbool.h>
#include <stdint.h>

// Aggregated state of all connected USB keyboards / mice (host side, core1).
// Getters are safe to call from core0.

void hid_input_init(void);

// True if the HID usage code is held on any connected keyboard
bool hid_key_down(uint8_t keycode);

// OR of KEYBOARD_MODIFIER_* bits over all connected keyboards
uint8_t hid_kbd_modifiers(void);

// OR of MOUSE_BUTTON_* bits over all connected mice
uint8_t hid_mouse_buttons(void);

// Fetch and clear accumulated mouse movement
void hid_mouse_take_deltas(int32_t *dx, int32_t *dy);

#endif // NXIC_HID_INPUT_H_
