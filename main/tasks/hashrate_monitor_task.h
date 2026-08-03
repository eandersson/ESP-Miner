#ifndef HASHRATE_MONITOR_TASK_H_
#define HASHRATE_MONITOR_TASK_H_

#include "asic_common.h"
#include <pthread.h>

typedef struct {
    uint32_t value;
    uint64_t time_us;
    float hashrate;
} measurement_t;

typedef struct {
    measurement_t* total_measurement;
    measurement_t** domain_measurements;
    measurement_t* error_measurement;

    pthread_mutex_t lock;
    bool is_initialized;
    uint64_t monitor_started_us;
    uint64_t last_response_us;
    uint64_t last_progress_us;
} HashrateMonitorModule;

typedef struct {
    bool initialized;
    bool response_seen;
    bool progress_seen;
    uint32_t monitor_age_ms;
    uint32_t response_age_ms;
    uint32_t progress_age_ms;
} hashrate_liveness_t;

void hashrate_monitor_task(void *pvParameters);
void hashrate_monitor_register_read(void *pvParameters, register_type_t register_type, uint8_t asic_nr, uint32_t value, uint64_t timestamp_us);
void hashrate_monitor_reset_measurements(void *pvParameters);
void hashrate_monitor_get_liveness(void *pvParameters, hashrate_liveness_t *snapshot);

void update_hashrate(measurement_t * measurement, uint32_t value);
void update_hash_counter(measurement_t * measurement, uint32_t value, uint64_t time_us);
#endif /* HASHRATE_MONITOR_TASK_H_ */
