#ifndef NXIC_PROCON_H_
#define NXIC_PROCON_H_

#include <stdbool.h>

// Pro Controller protocol state machine (device side, core0)

void procon_init(void);

// Call from the main loop after tud_task()
void procon_task(void);

// True once the Switch has enabled input streaming (0x80 0x04 received)
bool procon_active(void);

#endif // NXIC_PROCON_H_
