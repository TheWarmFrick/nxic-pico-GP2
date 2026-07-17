#ifndef NXIC_MAPPING_H_
#define NXIC_MAPPING_H_

#include <stdint.h>

// One Pro Controller input state, laid out for the 0x30 input report.
typedef struct {
    uint8_t btn[3];    // report bytes 3..5 (right / shared / left)
    uint8_t stick[6];  // report bytes 6..11 (L, R stick, 12-bit packed)
    int16_t imu[3][6]; // 3 frames x {ax, ay, az, gx, gy, gz}
} controller_state_t;

// Build the current state from keyboard / mouse input.
// Must be called exactly once per input report (consumes mouse deltas).
void mapping_get_state(controller_state_t *st);

// Current accel debug mode (0: attitude tilt, 1: constant flat, 2: zero)
int mapping_debug_accel_mode(void);

#endif // NXIC_MAPPING_H_
