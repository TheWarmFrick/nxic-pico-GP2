// Keyboard/mouse -> Pro Controller state conversion (key map follows NXIC).
//
// Mouse -> gyro is reworked from NXIC:
//  * deltas are accumulated (no counts lost between reports)
//  * angle-preserving: gyro rate is derived from the rotation angle the
//    accumulated counts represent, using the 15 ms (3 x 5 ms IMU frames)
//    the console integrates per report
//  * when the int16 gyro range saturates the residual angle is carried
//    over to following reports, so fast flicks land on the exact angle
//  * a virtual controller attitude (pitch about body X, yaw about world up)
//    is tracked by integrating exactly the emitted gyro; the accelerometer
//    reports the gravity vector for that attitude and yaw rate is split
//    between body Y/Z, so gyro and accel always describe the same rigid
//    body and orientation-fusing games behave correctly

#include <math.h>
#include <stdbool.h>

#include "config.h"
#include "hid_input.h"
#include "mapping.h"
#include "settings.h"

#define STICK_MIN    0x000
#define STICK_CENTER 0x800
#define STICK_MAX    0xFFF

// Pro Controller standard input report button bits
#define BTN0_Y        0x01
#define BTN0_X        0x02
#define BTN0_B        0x04
#define BTN0_A        0x08
#define BTN0_R        0x40
#define BTN0_ZR       0x80

#define BTN1_MINUS    0x01
#define BTN1_PLUS     0x02
#define BTN1_RCLICK   0x04
#define BTN1_LCLICK   0x08
#define BTN1_HOME     0x10
#define BTN1_CAPTURE  0x20

#define BTN2_DOWN     0x01
#define BTN2_UP       0x02
#define BTN2_RIGHT    0x04
#define BTN2_LEFT     0x08
#define BTN2_L        0x40
#define BTN2_ZL       0x80

static uint16_t axis12(bool neg, bool pos) {
    if (pos && !neg) return STICK_MAX;
    if (neg && !pos) return STICK_MIN;
    return STICK_CENTER;
}

static void pack_stick(uint8_t *d, uint16_t x, uint16_t y) {
    d[0] = (uint8_t)(x & 0xFF);
    d[1] = (uint8_t)(((x >> 8) & 0x0F) | ((y & 0x0F) << 4));
    d[2] = (uint8_t)(y >> 4);
}

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Runtime IMU debug switches (see KEY_DEBUG_* in config.h)
static int accel_sign = ACCEL_REST_SIGN;
static int accel_mode; // 0: attitude tilt, 1: constant flat, 2: zero (NXIC)

int mapping_debug_accel_mode(void) {
    return accel_mode;
}

static void fill_imu(int16_t imu[3][6]) {
    // Residual rotation not yet emitted (deg)
    static float pend_yaw, pend_pitch;
    // Virtual controller attitude, positive = aiming up.
    // Pro Controller body frame: X = forward, Y = lateral (left), Z = up,
    // so pitch is a rotation about the Y axis (tilting happens in the X-Z
    // plane -- the factory "horizontal offset" calibration also has its
    // components on X/Z with Y = 0).
    // Integrated from the gyro values actually sent, so the reported gravity
    // vector can never disagree with what the console integrated.
    static float pitch_deg;

    static bool prev_sign_key, prev_mode_key;

    // Debug keys only fire together with Ctrl+Alt so that they can never be
    // triggered by stray input
    uint8_t mods = hid_kbd_modifiers();
    bool chord = (mods & KEYBOARD_MODIFIER_LEFTCTRL) &&
                 (mods & KEYBOARD_MODIFIER_LEFTALT);

    bool sign_key = chord && hid_key_down(KEY_DEBUG_ACCEL_SIGN);
    if (sign_key && !prev_sign_key) accel_sign = -accel_sign;
    prev_sign_key = sign_key;

    bool mode_key = chord && hid_key_down(KEY_DEBUG_ACCEL_MODE);
    if (mode_key && !prev_mode_key) accel_mode = (accel_mode + 1) % 3;
    prev_mode_key = mode_key;

    int32_t dx, dy;
    hid_mouse_take_deltas(&dx, &dy);

    const float yaw_deg_per_count   = 360.0f / g_settings.yaw_counts_per_360;
    const float pitch_deg_per_count = 360.0f / g_settings.pitch_counts_per_360;
    // Mouse right (+dx) = turn right = negative rotation about world up.
    // Mouse down (+dy) = aim down = pitch decreases.
    pend_yaw   += (g_settings.invert_yaw   ? (float)dx : -(float)dx) * yaw_deg_per_count;
    pend_pitch += (g_settings.invert_pitch ? (float)dy : -(float)dy) * pitch_deg_per_count;

    if (hid_key_down(g_settings.key_pitch_reset)) {
        pitch_deg = 0.0f;
        pend_pitch = 0.0f;
    }

    const float lsb_dps  = 0.070f;     // gyro deg/s per LSB (default calibration)
    const float frame_dt = 0.005f;     // one IMU frame as integrated by the console
    const float game_dt  = 3 * frame_dt; // 3 frames per report
    const float max_deg  = 32000.0f * lsb_dps * game_dt; // ~33.6 deg per report

    const float pitch_limit = g_settings.pitch_limit_deg;

    float take_yaw   = clampf(pend_yaw,   -max_deg, max_deg);
    // Pitch is additionally limited so the attitude stays inside the pitch
    // limit (rolling over the pole would break the model)
    float take_pitch = clampf(pend_pitch, -max_deg, max_deg);
    take_pitch = clampf(take_pitch, -pitch_limit - pitch_deg,
                                     pitch_limit - pitch_deg);

    int16_t raw_yaw   = (int16_t)lrintf(take_yaw   / game_dt / lsb_dps);
    int16_t raw_pitch = (int16_t)lrintf(take_pitch / game_dt / lsb_dps);

    // Subtract exactly what was encoded so quantization never drifts
    pend_yaw   -= (float)raw_yaw   * lsb_dps * game_dt;
    pend_pitch -= (float)raw_pitch * lsb_dps * game_dt;

    for (int f = 0; f < 3; f++) {
        // Advance the attitude frame by frame so accel moves smoothly
        pitch_deg += (float)raw_pitch * lsb_dps * frame_dt;
        float rad = pitch_deg * 0.017453293f;
        float s = sinf(rad);
        float c = cosf(rad);

        // Gravity in the body frame: world up in body coordinates is
        // (sin(theta), 0, cos(theta)); the accelerometer reads it with
        // ACCEL_REST_SIGN (real controller: +1G on Z when flat)
        float s_a = (accel_mode == 0) ? s : 0.0f;
        float c_a = (accel_mode == 2) ? 0.0f : ((accel_mode == 0) ? c : 1.0f);
        imu[f][0] = (int16_t)(accel_sign * IMU_AXIS_SIGN_X * lrintf((float)ACCEL_1G * s_a));
        imu[f][1] = 0;
        imu[f][2] = (int16_t)(accel_sign * IMU_AXIS_SIGN_Z * lrintf((float)ACCEL_1G * c_a));

        // Body angular rate = pitch_rate about Y + yaw_rate about world up.
        // Positive rotation about +Y (left) is pitch DOWN, hence the minus.
        imu[f][3] = (int16_t)(IMU_AXIS_SIGN_X * lrintf((float)raw_yaw * s));
        imu[f][4] = (int16_t)(IMU_AXIS_SIGN_Y * -raw_pitch);
        imu[f][5] = (int16_t)(IMU_AXIS_SIGN_Z * lrintf((float)raw_yaw * c));
    }

    // Guard against float rounding creeping past the limit
    pitch_deg = clampf(pitch_deg, -pitch_limit, pitch_limit);
    // At the stop, drop further same-direction input so reversing the mouse
    // responds immediately instead of unwinding an invisible backlog
    if (pitch_deg >= pitch_limit - 0.01f && pend_pitch > 0) pend_pitch = 0;
    if (pitch_deg <= -(pitch_limit - 0.01f) && pend_pitch < 0) pend_pitch = 0;
}

void mapping_get_state(controller_state_t *st) {
    static bool y_hold, prev_y_toggle;
    static bool rapid, prev_rapid_toggle;
    static uint32_t report_no;

    report_no++;

    const settings_t *cfg = &g_settings;

    // Edge-triggered toggles
    bool y_toggle = hid_key_down(cfg->key_y_hold_toggle);
    if (y_toggle && !prev_y_toggle) y_hold = !y_hold;
    prev_y_toggle = y_toggle;

    bool rapid_toggle = hid_key_down(cfg->key_rapid_toggle);
    if (rapid_toggle && !prev_rapid_toggle) rapid = !rapid;
    prev_rapid_toggle = rapid_toggle;

    uint8_t mb = hid_mouse_buttons();
    uint8_t b0 = 0, b1 = 0, b2 = 0;

    if (hid_key_down(cfg->key_btn_a) || (mb & MOUSEBTN_A)) b0 |= BTN0_A;
    if (hid_key_down(cfg->key_btn_b) || (mb & MOUSEBTN_B)) b0 |= BTN0_B;
    if (hid_key_down(cfg->key_btn_x))                      b0 |= BTN0_X;
    if (hid_key_down(cfg->key_btn_y) || y_hold)            b0 |= BTN0_Y;
    if (mb & MOUSEBTN_R)                                   b0 |= BTN0_R;

    bool zr = (mb & MOUSEBTN_ZR) != 0;
    uint32_t rapid_half = cfg->rapid_half_period ? cfg->rapid_half_period : 1;
    if (zr && rapid) zr = ((report_no / rapid_half) & 1) == 0;
    if (zr) b0 |= BTN0_ZR;

    if (hid_key_down(cfg->key_minus))        b1 |= BTN1_MINUS;
    if (hid_key_down(cfg->key_plus))         b1 |= BTN1_PLUS;
    if (mb & MOUSEBTN_RSTICK_CLICK)          b1 |= BTN1_RCLICK;
    if (hid_key_down(cfg->key_lstick_click)) b1 |= BTN1_LCLICK;
    if (hid_key_down(cfg->key_home))         b1 |= BTN1_HOME;
    if (hid_key_down(cfg->key_capture))      b1 |= BTN1_CAPTURE;

    if (hid_key_down(cfg->key_dpad_down))  b2 |= BTN2_DOWN;
    if (hid_key_down(cfg->key_dpad_up))    b2 |= BTN2_UP;
    if (hid_key_down(cfg->key_dpad_right)) b2 |= BTN2_RIGHT;
    if (hid_key_down(cfg->key_dpad_left))  b2 |= BTN2_LEFT;
    if (hid_key_down(cfg->key_btn_l))      b2 |= BTN2_L;
    if (hid_key_down(cfg->key_btn_zl))     b2 |= BTN2_ZL;

    st->btn[0] = b0;
    st->btn[1] = b1;
    st->btn[2] = b2;

    uint16_t lx = axis12(hid_key_down(cfg->key_ls_left), hid_key_down(cfg->key_ls_right));
    uint16_t ly = axis12(hid_key_down(cfg->key_ls_down), hid_key_down(cfg->key_ls_up));
    uint16_t rx = axis12(hid_key_down(cfg->key_rs_left), hid_key_down(cfg->key_rs_right));
    uint16_t ry = axis12(hid_key_down(cfg->key_rs_down), hid_key_down(cfg->key_rs_up));
    pack_stick(&st->stick[0], lx, ly);
    pack_stick(&st->stick[3], rx, ry);

    fill_imu(st->imu);
}
