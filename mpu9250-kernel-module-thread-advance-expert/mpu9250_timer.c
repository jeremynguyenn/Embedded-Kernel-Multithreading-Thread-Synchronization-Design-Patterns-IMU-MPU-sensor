#include <linux/timer.h>
#include <linux/jiffies.h>
#include "mpu9250.h"

struct mpu9250_timer {
    struct timer_list timer;
    struct mpu9250_data *data;
    unsigned int interval_ms;
};

static void timer_callback(struct timer_list *t) {
    struct mpu9250_timer *timer = from_timer(timer, t, timer);
    struct mpu9250_data *data = timer->data;

    write_lock(&data->monitor_rwlock);
    mpu9250_check_deadlock(data);
    data->lock_ordered = true;
    recursive_mutex_lock(&data->recursive_lock);
    mpu9250_read_work(&data->read_work);
    recursive_mutex_unlock(&data->recursive_lock);
    data->lock_ordered = false;
    write_unlock(&data->monitor_rwlock);
    raw_notifier_call_chain(&data->notifier, 0, data);

    mod_timer(&timer->timer, jiffies + msecs_to_jiffies(timer->interval_ms));
}

int mpu9250_timer_init(struct mpu9250_data *data, unsigned int interval_ms) {
    struct mpu9250_timer *timer = kzalloc(sizeof(*timer), GFP_KERNEL);
    if (!timer) return -ENOMEM;

    timer->data = data;
    timer->interval_ms = interval_ms;
    timer_setup(&timer->timer, timer_callback, 0);
    mod_timer(&timer->timer, jiffies + msecs_to_jiffies(interval_ms));
    data->timer = timer;
    return 0;
}

void mpu9250_timer_cleanup(struct mpu9250_data *data) {
    if (data->timer) {
        del_timer_sync(&data->timer->timer);
        kfree(data->timer);
        data->timer = NULL;
    }
}