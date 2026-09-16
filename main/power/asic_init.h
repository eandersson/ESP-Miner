#ifndef ASIC_INIT_H_
#define ASIC_INIT_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct GlobalState GlobalState;

// ASIC lifecycle is deliberately more expressive than ASIC_initalized. The
// compatibility boolean is kept for older consumers, but is true only while
// this state is RUNNING so no work is submitted during a reset or ramp-down.
typedef enum {
    ASIC_LIFECYCLE_STOPPED = 0,
    ASIC_LIFECYCLE_STARTING,
    ASIC_LIFECYCLE_RUNNING,
    ASIC_LIFECYCLE_STOPPING,
} asic_lifecycle_state_t;

typedef enum {
    ASIC_INIT_COLD_BOOT,    // Fresh system startup - calls SERIAL_init()
    ASIC_INIT_RECOVERY      // Live recovery - only resets baud rate
} asic_init_mode_t;

/**
 * Initialize or reinitialize ASIC chip(s)
 * 
 * Handles both cold boot initialization and live recovery scenarios.
 * Key difference: Cold boot does full UART init, recovery only resets baud.
 * 
 * @param mode ASIC_INIT_COLD_BOOT for startup, ASIC_INIT_RECOVERY for live recovery
 */
uint8_t asic_initialize(GlobalState *GLOBAL_STATE, asic_init_mode_t mode, uint32_t stabilization_delay_ms);

void asic_lifecycle_set(GlobalState *GLOBAL_STATE, asic_lifecycle_state_t state);
bool asic_lifecycle_try_set_running(GlobalState *GLOBAL_STATE);
asic_lifecycle_state_t asic_lifecycle_get(const GlobalState *GLOBAL_STATE);
bool asic_lifecycle_is_running(const GlobalState *GLOBAL_STATE);

#endif /* ASIC_INIT_H_ */
