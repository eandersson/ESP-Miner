#ifndef FREQUENCY_TRANSITION_H
#define FREQUENCY_TRANSITION_H

#include <stdbool.h>
#include "esp_err.h"

typedef struct GlobalState GlobalState;

extern const char *FREQUENCY_TRANSITION_TAG;

/**
 * @brief Function pointer type for ASIC hash frequency setting functions
 * 
 * This type defines the signature for functions that set the hash frequency
 * for different ASIC types.
 * 
 * @param frequency The frequency to set in MHz
 */
typedef esp_err_t (*set_hash_frequency_fn)(float frequency,
                                           float *applied_frequency);

/**
 * @brief Transition the ASIC frequency to a target value
 * 
 * This function gradually adjusts the ASIC frequency to reach the target value,
 * stepping up or down in increments to ensure stability.
 * 
 * @param GLOBAL_STATE Pointer to the GlobalState structure
 * @param set_frequency_fn Function pointer to the appropriate ASIC's set_hash_frequency function
 */
esp_err_t do_frequency_transition(GlobalState *GLOBAL_STATE,
                                  set_hash_frequency_fn set_frequency_fn);

/**
 * @brief Transition frequency while requiring the ASIC to remain RUNNING
 *
 * Each PLL write is serialized with stop/reset, while delays between steps
 * remain interruptible by an emergency shutdown.
 */
esp_err_t do_runtime_frequency_transition(
    GlobalState *GLOBAL_STATE, set_hash_frequency_fn set_frequency_fn);

#endif // FREQUENCY_TRANSITION_H
