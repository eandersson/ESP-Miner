#include "work_queue.h"
#include "esp_log.h"
#include <stdlib.h>
#include <time.h>
#include <errno.h>
#include <string.h>

static const char *TAG = "work_queue";

static void *queue_take_locked(work_queue *queue,
                               work_queue_item_metadata *metadata)
{
    int slot = queue->head;
    void *next_work = queue->buffer[slot];
    if (metadata != NULL) {
        *metadata = queue->metadata[slot];
    }
    queue->buffer[slot] = NULL;
    queue->metadata[slot] = (work_queue_item_metadata){0};
    queue->head = (queue->head + 1) % QUEUE_SIZE;
    queue->count--;

    pthread_cond_signal(&queue->not_full);
    return next_work;
}

void queue_init(work_queue *queue)
{
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
    memset(queue->buffer, 0, sizeof(queue->buffer));
    memset(queue->metadata, 0, sizeof(queue->metadata));
    pthread_mutex_init(&queue->lock, NULL);
    pthread_cond_init(&queue->not_empty, NULL);
    pthread_cond_init(&queue->not_full, NULL);
}

void queue_enqueue(work_queue *queue, void *new_work,
                   work_queue_item_metadata metadata)
{
    pthread_mutex_lock(&queue->lock);

    while (queue->count == QUEUE_SIZE)
    {
        pthread_cond_wait(&queue->not_full, &queue->lock);
    }

    queue->buffer[queue->tail] = new_work;
    queue->metadata[queue->tail] = metadata;
    queue->tail = (queue->tail + 1) % QUEUE_SIZE;
    queue->count++;

    pthread_cond_signal(&queue->not_empty);
    pthread_mutex_unlock(&queue->lock);
}

void *queue_dequeue(work_queue *queue, work_queue_item_metadata *metadata)
{
    pthread_mutex_lock(&queue->lock);

    while (queue->count == 0)
    {
        pthread_cond_wait(&queue->not_empty, &queue->lock);
    }

    void *next_work = queue_take_locked(queue, metadata);
    pthread_mutex_unlock(&queue->lock);

    return next_work;
}

void *queue_dequeue_timeout(work_queue *queue, int timeout_ms,
                            work_queue_item_metadata *metadata)
{
    pthread_mutex_lock(&queue->lock);
    if (metadata != NULL) {
        *metadata = (work_queue_item_metadata){0};
    }

    // An expired scheduler deadline is a poll, not an invalid absolute
    // timespec. This also keeps callers from accidentally blocking after
    // their remaining timeout rounds down to zero.
    if (timeout_ms <= 0) {
        void *next_work = NULL;
        if (queue->count > 0) {
            next_work = queue_take_locked(queue, metadata);
        }
        pthread_mutex_unlock(&queue->lock);
        return next_work;
    }

    struct timespec timeout_time;
    if (clock_gettime(CLOCK_REALTIME, &timeout_time) != 0) {
        ESP_LOGE(TAG, "Unable to read queue timeout clock: %s", strerror(errno));
        pthread_mutex_unlock(&queue->lock);
        return NULL;
    }

    // pthread_cond_timedwait expects one absolute deadline. Reusing it after
    // a spurious wakeup prevents the total wait from being extended.
    timeout_time.tv_sec += timeout_ms / 1000;
    timeout_time.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
    if (timeout_time.tv_nsec >= 1000000000L) {
        timeout_time.tv_sec += timeout_time.tv_nsec / 1000000000L;
        timeout_time.tv_nsec %= 1000000000L;
    }

    while (queue->count == 0)
    {
        int result = pthread_cond_timedwait(&queue->not_empty, &queue->lock, &timeout_time);
        if (result == ETIMEDOUT) {
            pthread_mutex_unlock(&queue->lock);
            return NULL;
        }
        if (result != 0) {
            ESP_LOGE(TAG, "Queue timed wait failed: %s", strerror(result));
            pthread_mutex_unlock(&queue->lock);
            return NULL;
        }
    }

    void *next_work = queue_take_locked(queue, metadata);
    pthread_mutex_unlock(&queue->lock);

    return next_work;
}

void queue_clear(work_queue *queue)
{
    pthread_mutex_lock(&queue->lock);

    while (queue->count > 0)
    {
        int slot = queue->head;
        void *next_work = queue->buffer[slot];
        work_queue_free_fn free_fn =
            queue->metadata[slot].free_fn;
        if (free_fn != NULL) {
            free_fn(next_work);
        } else {
            free(next_work);
        }
        queue->buffer[slot] = NULL;
        queue->metadata[slot] = (work_queue_item_metadata){0};
        queue->head = (queue->head + 1) % QUEUE_SIZE;
        queue->count--;
    }

    pthread_cond_broadcast(&queue->not_full);
    pthread_mutex_unlock(&queue->lock);
}
