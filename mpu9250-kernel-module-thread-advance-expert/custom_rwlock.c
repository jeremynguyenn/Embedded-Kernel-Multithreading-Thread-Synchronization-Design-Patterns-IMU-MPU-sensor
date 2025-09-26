#include <linux/spinlock.h>
#include <linux/wait.h>
#include "mpu9250.h"

struct custom_rwlock {
    spinlock_t lock;
    int readers;
    int writer_waiting;
    wait_queue_head_t wq;
};

void custom_rwlock_init(struct custom_rwlock *rwlock) {
    spin_lock_init(&rwlock->lock);
    rwlock->readers = 0;
    rwlock->writer_waiting = 0;
    init_waitqueue_head(&rwlock->wq);
}

void custom_read_lock(struct custom_rwlock *rwlock) {
    spin_lock(&rwlock->lock);
    while (rwlock->writer_waiting)
        wait_event_interruptible(rwlock->wq, !rwlock->writer_waiting);
    rwlock->readers++;
    spin_unlock(&rwlock->lock);
}

void custom_read_unlock(struct custom_rwlock *rwlock) {
    spin_lock(&rwlock->lock);
    rwlock->readers--;
    if (rwlock->readers == 0)
        wake_up_all(&rwlock->wq);
    spin_unlock(&rwlock->lock);
}

void custom_write_lock(struct custom_rwlock *rwlock) {
    spin_lock(&rwlock->lock);
    rwlock->writer_waiting = 1;
    while (rwlock->readers > 0)
        wait_event_interruptible(rwlock->wq, rwlock->readers == 0);
    spin_unlock(&rwlock->lock);
}

void custom_write_unlock(struct custom_rwlock *rwlock) {
    spin_lock(&rwlock->lock);
    rwlock->writer_waiting = 0;
    wake_up_all(&rwlock->wq);
    spin_unlock(&rwlock->lock);
}