#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/of_device.h>
#include <linux/kthread.h>
#include <linux/vmalloc.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/notifier.h>
#include "mpu9250.h"

// Module parameters
static unsigned int i2c_addr = MPU9250_DEFAULT_ADDR;
static unsigned int irq_type = MPU9250_INTERRUPT_DATA_READY;
static int debug = 0;
module_param(i2c_addr, uint, 0644);
module_param(irq_type, uint, 0644);
module_param(debug, int, 0644);

// Thread pool worker function with pause/resume, barrier
static int sensor_thread_func(void *arg) {
    struct mpu9250_thread_task *task = arg;
    struct mpu9250_data *data = task->data;
    while (!kthread_should_stop()) {
        if (task->paused) {
            wait_event_interruptible(data->thread_pool.pause_wq, !task->paused || kthread_should_stop());
        }
        barrier_wait(&data->thread_pool.thread_barrier); // Sync with other threads
        write_lock(&data->monitor_rwlock);
        mpu9250_check_deadlock(data);
        data->lock_ordered = true;
        mutex_lock(&data->recursive_lock);
        mpu9250_read_work(&data->read_work);
        mutex_unlock(&data->recursive_lock);
        data->lock_ordered = false;
        write_unlock(&data->monitor_rwlock);
        raw_notifier_call_chain(&data->notifier, 0, data); // ITC
        msleep(100);
    }
    return 0;
}

// Sysfs attributes with rwlock
static ssize_t mpu9250_data_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
    struct mpu9250_data *data = container_of(kobj, struct mpu9250_data, device->kobj);
    read_lock(&data->monitor_rwlock);
    ssize_t len = sprintf(buf, "Accel: %.2f %.2f %.2f\nGyro: %.2f %.2f %.2f\nMag: %.2f %.2f %.2f\nQuat: %.2f %.2f %.2f %.2f\n",
                          data->accel_g[0], data->accel_g[1], data->accel_g[2],
                          data->gyro_dps[0], data->gyro_dps[1], data->gyro_dps[2],
                          data->mag_uT[0], data->mag_uT[1], data->mag_uT[2],
                          data->quat[0], data->quat[1], data->quat[2], data->quat[3]);
    read_unlock(&data->monitor_rwlock);
    return len;
}

static struct kobj_attribute dev_attr_mpu9250_data = __ATTR(mpu9250_data, 0444, mpu9250_data_show, NULL);

static ssize_t accel_scale_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
    struct mpu9250_data *data = container_of(kobj, struct mpu9250_data, device->kobj);
    read_lock(&data->monitor_rwlock);
    ssize_t len = sprintf(buf, "%d\n", data->accel_scale);
    read_unlock(&data->monitor_rwlock);
    return len;
}

static ssize_t accel_scale_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
    struct mpu9250_data *data = container_of(kobj, struct mpu9250_data, device->kobj);
    int scale;
    if (kstrtoint(buf, 10, &scale)) return -EINVAL;
    write_lock(&data->monitor_rwlock);
    if (mpu9250_set_accel_scale(data, scale)) {
        write_unlock(&data->monitor_rwlock);
        return -EINVAL;
    }
    write_unlock(&data->monitor_rwlock);
    return count;
}

static struct kobj_attribute dev_attr_accel_scale = __ATTR(accel_scale, 0664, accel_scale_show, accel_scale_store);

static ssize_t gyro_scale_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
    struct mpu9250_data *data = container_of(kobj, struct mpu9250_data, device->kobj);
    read_lock(&data->monitor_rwlock);
    ssize_t len = sprintf(buf, "%d\n", data->gyro_scale);
    read_unlock(&data->monitor_rwlock);
    return len;
}

static ssize_t gyro_scale_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count) {
    struct mpu9250_data *data = container_of(kobj, struct mpu9250_data, device->kobj);
    int scale;
    if (kstrtoint(buf, 10, &scale)) return -EINVAL;
    write_lock(&data->monitor_rwlock);
    if (mpu9250_set_gyro_scale(data, scale)) {
        write_unlock(&data->monitor_rwlock);
        return -EINVAL;
    }
    write_unlock(&data->monitor_rwlock);
    return count;
}

static struct kobj_attribute dev_attr_gyro_scale = __ATTR(gyro_scale, 0664, gyro_scale_show, gyro_scale_store);

static struct attribute *mpu9250_attrs[] = {
    &dev_attr_mpu9250_data.attr,
    &dev_attr_accel_scale.attr,
    &dev_attr_gyro_scale.attr,
    NULL
};

static const struct attribute_group mpu9250_attr_group = {
    .attrs = mpu9250_attrs,
};

// Thread pool initialization with notifier, barrier, semaphore
int mpu9250_thread_pool_init(struct mpu9250_data *data, int num_threads) {
    data->thread_pool.tasks = kcalloc(num_threads, sizeof(struct mpu9250_thread_task), GFP_KERNEL);
    if (!data->thread_pool.tasks) return -ENOMEM;
    data->thread_pool.num_threads = num_threads > MAX_THREADS ? MAX_THREADS : num_threads;
    data->thread_pool.priority = 0;
    sema_init(&data->thread_pool.thread_sem, data->thread_pool.num_threads);
    RAW_INIT_NOTIFIER_HEAD(&data->thread_pool.notifier);
    init_completion(&data->thread_pool.barrier_comp);
    init_waitqueue_head(&data->thread_pool.pause_wq);
    barrier_init(&data->thread_pool.thread_barrier, data->thread_pool.num_threads);
    for (int i = 0; i < data->thread_pool.num_threads; i++) {
        data->thread_pool.tasks[i].data = data;
        data->thread_pool.tasks[i].task_id = i;
        data->thread_pool.tasks[i].paused = false;
        char name[16];
        snprintf(name, sizeof(name), "mpu9250_task_%d", i);
        data->thread_pool.tasks[i].thread = kthread_create(sensor_thread_func, &data->thread_pool.tasks[i], name);
        if (IS_ERR(data->thread_pool.tasks[i].thread)) {
            mpu9250_thread_pool_cleanup(data);
            return PTR_ERR(data->thread_pool.tasks[i].thread);
        }
        kthread_bind(data->thread_pool.tasks[i].thread, i % num_online_cpus()); // For parallelism
        wake_up_process(data->thread_pool.tasks[i].thread);
    }
    return 0;
}

// Thread pool cleanup
void mpu9250_thread_pool_cleanup(struct mpu9250_data *data) {
    for (int i = 0; i < data->thread_pool.num_threads; i++) {
        if (data->thread_pool.tasks[i].thread) kthread_stop(data->thread_pool.tasks[i].thread);
    }
    kfree(data->thread_pool.tasks);
    barrier_destroy(&data->thread_pool.thread_barrier);
}

// Pause threads
int mpu9250_pause_threads(struct mpu9250_data *data) {
    for (int i = 0; i < data->thread_pool.num_threads; i++) data->thread_pool.tasks[i].paused = true;
    return 0;
}

// Resume threads
int mpu9250_resume_threads(struct mpu9250_data *data) {
    for (int i = 0; i < data->thread_pool.num_threads; i++) data->thread_pool.tasks[i].paused = false;
    wake_up_all(&data->thread_pool.pause_wq);
    return 0;
}

// Set number of threads (dynamic resize)
int mpu9250_set_num_threads(struct mpu9250_data *data, int num_threads) {
    mpu9250_thread_pool_cleanup(data);
    return mpu9250_thread_pool_init(data, num_threads);
}

// Notify subscribers
void mpu9250_notify_subscribers(struct mpu9250_data *data) {
    raw_notifier_call_chain(&data->notifier, 0, data);
}

// Suspend (pause threads)
static int mpu9250_suspend(struct device *dev) {
    struct i2c_client *client = to_i2c_client(dev);
    struct mpu9250_data *data = i2c_get_clientdata(client);
    uint8_t buf;
    if (mpu9250_read(data, MPU9250_PWR_MGMT_1, &buf, 1)) return -EIO;
    buf |= 0x40;
    int ret = mpu9250_write(data, MPU9250_PWR_MGMT_1, &buf, 1);
    mpu9250_pause_threads(data);
    return ret;
}

// Resume
static int mpu9250_resume(struct device *dev) {
    struct i2c_client *client = to_i2c_client(dev);
    struct mpu9250_data *data = i2c_get_clientdata(client);
    uint8_t buf;
    if (mpu9250_read(data, MPU9250_PWR_MGMT_1, &buf, 1)) return -EIO;
    buf &= ~0x40;
    int ret = mpu9250_write(data, MPU9250_PWR_MGMT_1, &buf, 1);
    mpu9250_resume_threads(data);
    return ret;
}

static const struct dev_pm_ops mpu9250_pm_ops = {
    .suspend = mpu9250_suspend,
    .resume = mpu9250_resume,
};

// Alloc/free buffers
int mpu9250_alloc_buffers(struct mpu9250_data *data) {
    data->fifo_buf = vmalloc(MPU9250_MAX_FIFO);
    if (!data->fifo_buf) return -ENOMEM;
    return 0;
}

void mpu9250_free_buffers(struct mpu9250_data *data) {
    vfree(data->fifo_buf);
}

// Probe
static int mpu9250_probe(struct i2c_client *client, const struct i2c_device_id *id) {
    struct mpu9250_data *data;
    int ret;

    data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
    if (!data) return -ENOMEM;

    i2c_set_clientdata(client, data);
    data->client = client;
    data->regmap = devm_regmap_init_i2c(client, &mpu9250_regmap_config);
    if (IS_ERR(data->regmap)) return PTR_ERR(data->regmap);

    mutex_init(&data->lock);
    rwlock_init(&data->rwlock);
    rwlock_init(&data->monitor_rwlock);
    spin_lock_init(&data->data_lock);
    INIT_WORK(&data->read_work, mpu9250_read_work);
    init_waitqueue_head(&data->wq);
    init_waitqueue_head(&data->monitor_wq);
    sema_init(&data->sem, 1);
    mutex_init(&data->recursive_lock);
    init_completion(&data->event_pair);
    RAW_INIT_NOTIFIER_HEAD(&data->notifier);
    data->cleanup_handler = mpu9250_cleanup;
    atomic_set(&data->hold_count, 0);
    atomic_set(&data->wait_count, 0);

    ret = mpu9250_alloc_buffers(data);
    if (ret) return ret;

    ret = mpu9250_i2c_init(data);
    if (ret) goto err_free;

    data->mag_client = i2c_new_ancillary_device(client, "ak8963", 0x0C);
    if (!data->mag_client) {
        ret = -ENODEV;
        goto err_free;
    }

    ret = mpu9250_fileops_init(data);
    if (ret) goto err_mag;

    ret = sysfs_create_group(&data->device->kobj, &mpu9250_attr_group);
    if (ret) goto err_device;

    pm_runtime_enable(&client->dev);
    ret = devm_request_threaded_irq(&client->dev, client->irq, mpu9250_irq_handler, NULL,
                                   IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "mpu9250", data);
    if (ret) goto err_sysfs;

    ret = mpu9250_thread_pool_init(data, MAX_THREADS);
    if (ret) goto err_sysfs;

    mpu9250_calibrate(data);

    data->input_dev = devm_input_allocate_device(&client->dev);
    if (!data->input_dev) {
        ret = -ENOMEM;
        goto err_threads;
    }
    data->input_dev->name = "MPU9250 IMU";
    input_set_abs_params(data->input_dev, ABS_X, -32768, 32767, 0, 0);
    input_set_abs_params(data->input_dev, ABS_Y, -32768, 32767, 0, 0);
    input_set_abs_params(data->input_dev, ABS_Z, -32768, 32767, 0, 0);
    input_set_abs_params(data->input_dev, ABS_RX, -32768, 32767, 0, 0);
    input_set_abs_params(data->input_dev, ABS_RY, -32768, 32767, 0, 0);
    input_set_abs_params(data->input_dev, ABS_RZ, -32768, 32767, 0, 0);
    ret = input_register_device(data->input_dev);
    if (ret) goto err_threads;

    dev_info(&client->dev, "MPU9250 probed successfully, version %s\n", DRIVER_VERSION);
    return 0;

err_threads:
    mpu9250_thread_pool_cleanup(data);
err_sysfs:
    sysfs_remove_group(&data->device->kobj, &mpu9250_attr_group);
err_device:
    mpu9250_fileops_cleanup(data);
err_mag:
    i2c_unregister_device(data->mag_client);
err_free:
    mpu9250_free_buffers(data);
    return ret;
}

// Remove
static int mpu9250_remove(struct i2c_client *client) {
    struct mpu9250_data *data = i2c_get_clientdata(client);
    mpu9250_thread_pool_cleanup(data);
    input_unregister_device(data->input_dev);
    pm_runtime_disable(&client->dev);
    cancel_work_sync(&data->read_work);
    sysfs_remove_group(&data->device->kobj, &mpu9250_attr_group);
    mpu9250_fileops_cleanup(data);
    i2c_unregister_device(data->mag_client);
    mpu9250_free_buffers(data);
    dev_info(&client->dev, "MPU9250 removed\n");
    return 0;
}

static const struct i2c_device_id mpu9250_id[] = {
    { "mpu9250", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, mpu9250_id);

static const struct of_device_id mpu9250_of_match[] = {
    { .compatible = "invensense,mpu9250" },
    { }
};
MODULE_DEVICE_TABLE(of, mpu9250_of_match);

static struct i2c_driver mpu9250_driver = {
    .driver = {
        .name = "mpu9250",
        .owner = THIS_MODULE,
        .of_match_table = mpu9250_of_match,
        .pm = &mpu9250_pm_ops,
    },
    .probe = mpu9250_probe,
    .remove = mpu9250_remove,
    .id_table = mpu9250_id,
};

static int __init mpu9250_init(void) {
    return i2c_add_driver(&mpu9250_driver);
}

static void __exit mpu9250_exit(void) {
    i2c_del_driver(&mpu9250_driver);
}

module_init(mpu9250_init);
module_exit(mpu9250_exit);

MODULE_AUTHOR("Nguyen Nhan");
MODULE_DESCRIPTION("MPU9250 Kernel Driver with advanced threading");
MODULE_LICENSE("GPL");
MODULE_VERSION("2.3");