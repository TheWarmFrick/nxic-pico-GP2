#ifndef NXIC_CONFIG_H_
#define NXIC_CONFIG_H_

#include "tusb.h"   // HID_KEY_* / MOUSE_BUTTON_*

//--------------------------------------------------------------------
// Hardware (Waveshare RP2350-USB-A)
//   USB-A port: D+ = GP12, D- = GP13  (host mode requires removing R13)
//   WS2812 RGB LED on GP16
//--------------------------------------------------------------------
#define PIN_USB_HOST_DP        12
#define PIN_WS2812             16
#define LED_BRIGHTNESS         32   // 0-255

//--------------------------------------------------------------------
// Input report timing. Each 0x30 report carries 3 IMU frames of 5 ms,
// so reports are sent every 15 ms like a real Pro Controller -- sending
// faster would make the gyro integrate faster than the accel tilt
// evolves and break sensor fusion in games.
//--------------------------------------------------------------------
#define REPORT_INTERVAL_US     15000

//--------------------------------------------------------------------
// Key map (NXIC compatible)
//--------------------------------------------------------------------
#define KEY_BTN_A              HID_KEY_L
#define KEY_BTN_B              HID_KEY_K
#define KEY_BTN_X              HID_KEY_I
#define KEY_BTN_Y              HID_KEY_J
#define KEY_Y_HOLD_TOGGLE      HID_KEY_Y   // toggles holding the Y button

#define KEY_BTN_L              HID_KEY_R
#define KEY_BTN_ZL             HID_KEY_E
#define KEY_LSTICK_CLICK       HID_KEY_Q

#define KEY_DPAD_UP            HID_KEY_F
#define KEY_DPAD_DOWN          HID_KEY_V
#define KEY_DPAD_LEFT          HID_KEY_C
#define KEY_DPAD_RIGHT         HID_KEY_B

#define KEY_BTN_PLUS           HID_KEY_U
#define KEY_BTN_MINUS          HID_KEY_T
#define KEY_BTN_HOME           HID_KEY_H
#define KEY_BTN_CAPTURE        HID_KEY_G

// Left stick
#define KEY_LS_UP              HID_KEY_W
#define KEY_LS_DOWN            HID_KEY_S
#define KEY_LS_LEFT            HID_KEY_A
#define KEY_LS_RIGHT           HID_KEY_D

// Right stick
#define KEY_RS_UP              HID_KEY_ARROW_UP
#define KEY_RS_DOWN            HID_KEY_ARROW_DOWN
#define KEY_RS_LEFT            HID_KEY_ARROW_LEFT
#define KEY_RS_RIGHT           HID_KEY_ARROW_RIGHT

// Rapid fire: toggled with this key, applies to mouse-left (ZR)
#define KEY_RAPID_TOGGLE       HID_KEY_P

//--------------------------------------------------------------------
// Mouse buttons
//--------------------------------------------------------------------
#define MOUSEBTN_ZR            MOUSE_BUTTON_LEFT
#define MOUSEBTN_R             MOUSE_BUTTON_RIGHT
#define MOUSEBTN_RSTICK_CLICK  MOUSE_BUTTON_MIDDLE
#define MOUSEBTN_A             MOUSE_BUTTON_FORWARD
#define MOUSEBTN_B             MOUSE_BUTTON_BACKWARD

// Rapid fire half period in reports (4 -> 32 ms on / 32 ms off = 15.6 shots/s)
#define RAPID_HALF_PERIOD      4

//--------------------------------------------------------------------
// Mouse -> gyro conversion
//
// GYRO_COUNTS_PER_360 : mouse counts for a full 360 deg camera turn.
//   Lower = more sensitive. Tune to taste (depends on mouse DPI).
// The conversion is angle-preserving: every count is eventually emitted,
// residual rotation is carried over when the int16 gyro range saturates.
//--------------------------------------------------------------------
// Yaw (horizontal): mouse counts for a full 360 deg turn. Yaw is rate
// based, games integrate it, so this behaves like normal mouse aim.
#define GYRO_COUNTS_PER_360    8000.0f

// Pitch (vertical): mouse counts that would correspond to 360 deg of
// controller tilt. Games like Splatoon tie the vertical view to the
// ABSOLUTE controller pitch, so vertical works like positioning a tilt
// within +/-PITCH_LIMIT_DEG rather than an endless rate -- tune this
// independently of yaw. Lower = more sensitive.
#define PITCH_COUNTS_PER_360   8000.0f

#define GYRO_INVERT_YAW        0
#define GYRO_INVERT_PITCH      0

// The firmware tracks a virtual controller attitude: mouse Y pitches the
// controller about its (lateral) Y axis, mouse X yaws it about the world-up
// axis. The accelerometer always reports gravity for the current attitude,
// so games fusing gyro + accel see a physically consistent controller.

// Pitch is clamped to this attitude range (deg). Splatoon-style games map
// the absolute tilt to the vertical view, so this must cover the tilt
// range the game expects (raise it if the view cannot reach fully
// up/down; keep below 90). Input past the stop is discarded so reversing
// the mouse responds immediately.
#define PITCH_LIMIT_DEG        80.0f

// Key that re-levels the virtual pitch to horizontal (no gyro is emitted;
// use together with the in-game camera reset if they ever drift apart)
#define KEY_PITCH_RESET        HID_KEY_M

// Per-axis sign fixes (+1 / -1) in case the real controller's IMU axis
// conventions differ; flips the gyro and accel component of that axis together
#define IMU_AXIS_SIGN_X        1
#define IMU_AXIS_SIGN_Y        1
#define IMU_AXIS_SIGN_Z        1

// 1G in raw accelerometer units (0.244 mg/LSB)
#define ACCEL_1G               4096

// Accelerometer at-rest sign: the real controller reads +1G on the up axis
// when lying flat, i.e. (0, 0, +4096). Confirmed against SDL's Pro
// Controller driver (SDL_hidapi_switch.c) and by in-game testing: sending
// -1G makes the console believe the controller is upside down, which
// inverts the yaw direction.
#define ACCEL_REST_SIGN        (+1)

//--------------------------------------------------------------------
// IMU debug keys (for in-game A/B testing without reflashing).
// Only active while Left-Ctrl + Left-Alt are held, so stray input can
// never trigger them. Remove once the right combination is confirmed.
//--------------------------------------------------------------------
#define KEY_DEBUG_ACCEL_SIGN   HID_KEY_1  // Ctrl+Alt+1: toggle accel sign
#define KEY_DEBUG_ACCEL_MODE   HID_KEY_2  // Ctrl+Alt+2: cycle attitude/flat/zero

#endif // NXIC_CONFIG_H_
