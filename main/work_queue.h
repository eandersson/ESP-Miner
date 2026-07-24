#ifndef WORK_QUEUE_H
#define WORK_QUEUE_H

#include <pthread.h>
#include <stdint.h>

#define QUEUE_SIZE 12

typedef void (*work_queue_free_fn)(void *);

typedef enum
{
    WORK_QUEUE_ITEM_UNKNOWN = 0,
    WORK_QUEUE_ITEM_STRATUM_V1,
    WORK_QUEUE_ITEM_STRATUM_V2_STANDARD,
    WORK_QUEUE_ITEM_STRATUM_V2_EXTENDED,
} work_queue_item_kind;

typedef struct
{
    uint32_t generation;
    work_queue_item_kind kind;
    work_queue_free_fn free_fn;
} work_queue_item_metadata;

typedef struct
{
    void *buffer[QUEUE_SIZE];
    work_queue_item_metadata metadata[QUEUE_SIZE];
    int head;
    int tail;
    int count;
    pthread_mutex_t lock;
    pthread_cond_t not_empty;
    pthread_cond_t not_full;
} work_queue;

void queue_init(work_queue *queue);
void queue_enqueue(work_queue *queue, void *new_work,
                   work_queue_item_metadata metadata);
void *queue_dequeue(work_queue *queue, work_queue_item_metadata *metadata);
void *queue_dequeue_timeout(work_queue *queue, int timeout_ms,
                            work_queue_item_metadata *metadata);
void queue_clear(work_queue *queue);

#endif // WORK_QUEUE_H
