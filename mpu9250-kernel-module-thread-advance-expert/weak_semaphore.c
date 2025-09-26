#include <linux/semaphore.h>
#include <linux/wait.h>
#include "mpu9250.h"

struct weak_semaphore {
    atomic_t count;
    wait_queue_head_t wq;
};

void weak_sem_init(struct weak_semaphore *sem, int val) {
    atomic_set(&sem->count, val);
    init_waitqueue_head(&sem->wq);
}

int weak_sem_wait(struct weak_semaphore *sem) {
    while (atomic_dec_return(&sem->count) < 0) {
        atomic_inc(&sem->count);
        wait_event_interruptible(sem->wq, atomic_read(&sem->count) > 0);
    }
    return 0;
}

void weak_sem_post(struct weak_semaphore *sem) {
    atomic_inc(&sem->count);
    wake_up_all(&sem->wq);
}