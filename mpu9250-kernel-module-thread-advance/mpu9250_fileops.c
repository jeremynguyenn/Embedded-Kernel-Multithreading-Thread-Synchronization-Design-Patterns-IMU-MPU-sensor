#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/mmu_context.h>
#include <linux/delay.h>
#include <linux/mqueue.h>
#include "mpu9250.h"

static int mpu9250_open(struct inode *inode, struct file *filp) {
    struct mpu9250_data *data = container_of(inode->i_cdev, struct mpu9250_data, cdev);
    filp->private_data = data;
    return 0;
}

static int mpu9250_release(struct inode *inode, struct file *filp) {
    struct mpu9250_data *data = filp->private_data;
    if (data->cleanup_handler) data->cleanup_handler(data);
    return 0;
}

static ssize_t mpu9250_read(struct file *filp, char __user *buf, size_t len, loff_t *off) {
    struct mpu9250_data *data = filp->private_data;
    struct mpu9250_mq_data mq_data = {0};
    int ret;

    read_lock(&data->monitor_rwlock);
    wait_event_interruptible_timeout(data->wq, data->data_ready || kthread_should_stop(), HZ); // Bounded waiting
    if (!data->data_ready) {
        read_unlock(&data->monitor_rwlock);
        return -EINTR;
    }

    spin_lock(&data->data_lock);
    memcpy(mq_data.accel, data->accel_g, sizeof(data->accel_g));
    memcpy(mq_data.gyro, data->gyro_dps, sizeof(data->gyro_dps));
    memcpy(mq_data.mag, data->mag_uT, sizeof(data->mag_uT));
    memcpy(mq_data.quat, data->quat, sizeof(data->quat));
    spin_unlock(&data->data_lock);

    ret = copy_to_user(buf, &mq_data, min(len, sizeof(mq_data)));
    if (ret) {
        read_unlock(&data->monitor_rwlock);
        return -EFAULT;
    }

    data->data_ready = false;
    wake_up_all(&data->monitor_wq);
    read_unlock(&data->monitor_rwlock);
    if (data->data_ready_cb) data->data_ready_cb(data);
    raw_notifier_call_chain(&data->notifier, 0, data);
    return sizeof(mq_data);
}

static ssize_t mpu9250_write(struct file *filp, const char __user *buf, size_t len, loff_t *off) {
    struct mpu9250_data *data = filp->private_data;
    uint8_t write_buf[2];
    if (len != sizeof(write_buf)) return -EINVAL;

    write_lock(&data->monitor_rwlock);
    mpu9250_check_deadlock(data);
    data->lock_ordered = true;
    mutex_lock(&data->recursive_lock);
    if (copy_from_user(write_buf, buf, len)) {
        mutex_unlock(&data->recursive_lock);
        data->lock_ordered = false;
        write_unlock(&data->monitor_rwlock);
        return -EFAULT;
    }
    int ret = mpu9250_write(data, write_buf[0], &write_buf[1], 1);
    mutex_unlock(&data->recursive_lock);
    data->lock_ordered = false;
    write_unlock(&data->monitor_rwlock);
    return ret ? -EIO : len;
}

static long mpu9250_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
    struct mpu9250_data *data = filp->private_data;
    int ret = 0;

    write_lock(&data->monitor_rwlock);
    mpu9250_check_deadlock(data);
    data->lock_ordered = true;
    mutex_lock(&data->recursive_lock);

    switch (cmd) {
    case MPU9250_IOCTL_READ_REG: {
        struct mpu9250_reg reg;
        if (copy_from_user(&reg, (void __user *)arg, sizeof(reg))) {
            ret = -EFAULT;
            break;
        }
        ret = mpu9250_read(data, reg.reg, &reg.val, 1);
        if (!ret && copy_to_user((void __user *)arg, &reg, sizeof(reg)))
            ret = -EFAULT;
        break;
    }
    case MPU9250_IOCTL_WRITE_REG: {
        struct mpu9250_reg reg;
        if (copy_from_user(&reg, (void __user *)arg, sizeof(reg))) {
            ret = -EFAULT;
            break;
        }
        ret = mpu9250_write(data, reg.reg, &reg.val, 1);
        break;
    }
    case MPU9250_IOCTL_READ_ACCEL: {
        struct mpu9250_sensor_data accel_data;
        uint8_t raw[6];
        ret = mpu9250_read(data, MPU9250_ACCEL_XOUT_H, raw, 6);
        if (!ret) {
            convert_accel(raw, accel_data.values, data->accel_scale);
            if (copy_to_user((void __user *)arg, &accel_data, sizeof(accel_data)))
                ret = -EFAULT;
            else
                data->data_ready = true;
        }
        break;
    }
    case MPU9250_IOCTL_READ_GYRO: {
        struct mpu9250_sensor_data gyro_data;
        uint8_t raw[6];
        ret = mpu9250_read(data, MPU9250_GYRO_XOUT_H, raw, 6);
        if (!ret) {
            convert_gyro(raw, gyro_data.values, data->gyro_scale);
            if (copy_to_user((void __user *)arg, &gyro_data, sizeof(gyro_data)))
                ret = -EFAULT;
            else
                data->data_ready = true;
        }
        break;
    }
    case MPU9250_IOCTL_READ_MAG: {
        struct mpu9250_sensor_data mag_data;
        uint8_t raw[6];
        ret = mpu9250_read(data, MPU9250_MAG_XOUT_L, raw, 6);
        if (!ret) {
            convert_mag(raw, mag_data.values);
            if (copy_to_user((void __user *)arg, &mag_data, sizeof(mag_data)))
                ret = -EFAULT;
            else
                data->data_ready = true;
        }
        break;
    }
    case MPU9250_IOCTL_READ_FIFO: {
        struct mpu9250_sensor_data fifo_data;
        int len;
        ret = mpu9250_read_fifo(data, data->fifo_buf, &len);
        if (!ret) {
            // Process FIFO data (example: copy first 4 floats)
            memcpy(fifo_data.values, data->fifo_buf, sizeof(float)*4);
            if (copy_to_user((void __user *)arg, &fifo_data, sizeof(fifo_data)))
                ret = -EFAULT;
            else
                data->data_ready = true;
        }
        break;
    }
    case MPU9250_IOCTL_RESET:
        ret = mpu9250_i2c_init(data);
        break;
    case MPU9250_IOCTL_INIT_DMP:
        ret = mpu9250_dmp_init(data);
        break;
    case MPU9250_IOCTL_READ_DMP: {
        struct mpu9250_sensor_data dmp_data;
        mpu9250_read_dmp_quat(data, dmp_data.values);
        if (copy_to_user((void __user *)arg, &dmp_data, sizeof(dmp_data)))
            ret = -EFAULT;
        else
            data->data_ready = true;
        break;
    }
    case MPU9250_IOCTL_SET_NONBLOCK: {
        int nonblock;
        if (copy_from_user(&nonblock, (void __user *)arg, sizeof(nonblock))) {
            ret = -EFAULT;
            break;
        }
        if (nonblock)
            filp->f_flags |= O_NONBLOCK;
        else
            filp->f_flags &= ~O_NONBLOCK;
        break;
    }
    case MPU9250_IOCTL_SET_ACCEL_SCALE: {
        int scale;
        if (copy_from_user(&scale, (void __user *)arg, sizeof(scale))) {
            ret = -EFAULT;
            break;
        }
        ret = mpu9250_set_accel_scale(data, scale);
        break;
    }
    case MPU9250_IOCTL_SET_GYRO_SCALE: {
        int scale;
        if (copy_from_user(&scale, (void __user *)arg, sizeof(scale))) {
            ret = -EFAULT;
            break;
        }
        ret = mpu9250_set_gyro_scale(data, scale);
        break;
    }
    case MPU9250_IOCTL_SET_SAMPLE_RATE: {
        unsigned int rate;
        if (copy_from_user(&rate, (void __user *)arg, sizeof(rate))) {
            ret = -EFAULT;
            break;
        }
        ret = mpu9250_set_sample_rate(data, rate);
        break;
    }
    case MPU9250_IOCTL_CALIBRATE:
        ret = mpu9250_calibrate(data);
        break;
    case MPU9250_IOCTL_PAUSE_THREADS:
        ret = mpu9250_pause_threads(data);
        break;
    case MPU9250_IOCTL_RESUME_THREADS:
        ret = mpu9250_resume_threads(data);
        break;
    case MPU9250_IOCTL_SET_NUM_THREADS: {
        int num_threads;
        if (copy_from_user(&num_threads, (void __user *)arg, sizeof(num_threads))) {
            ret = -EFAULT;
            break;
        }
        ret = mpu9250_set_num_threads(data, num_threads);
        break;
    }
    default:
        ret = -EINVAL;
    }

    mutex_unlock(&data->recursive_lock);
    data->lock_ordered = false;
    write_unlock(&data->monitor_rwlock);
    if (!ret && data->data_ready_cb) data->data_ready_cb(data);
    raw_notifier_call_chain(&data->notifier, 0, data);
    return ret;
}

static unsigned int mpu9250_poll(struct file *filp, struct poll_table_struct *wait) {
    struct mpu9250_data *data = filp->private_data;
    unsigned int mask = 0;
    poll_wait(filp, &data->wq, wait);
    read_lock(&data->monitor_rwlock);
    if (data->data_ready)
        mask |= POLLIN | POLLRDNORM;
    read_unlock(&data->monitor_rwlock);
    return mask;
}

static int mpu9250_mmap(struct file *filp, struct vm_area_struct *vma) {
    struct mpu9250_data *data = filp->private_data;
    unsigned long size = vma->vm_end - vma->vm_start;
    if (size > MPU9250_MAX_FIFO) return -EINVAL;

    read_lock(&data->monitor_rwlock);
    int ret = remap_vmalloc_range(vma, data->fifo_buf, vma->vm_pgoff);
    read_unlock(&data->monitor_rwlock);
    return ret;
}

irqreturn_t mpu9250_irq_handler(int irq, void *dev_id) {
    struct mpu9250_data *data = dev_id;
    schedule_work(&data->read_work);
    complete(&data->event_pair); // Event pair for sync
    return IRQ_HANDLED;
}

void mpu9250_read_work(struct work_struct *work) {
    struct mpu9250_data *data = container_of(work, struct mpu9250_data, read_work);
    uint8_t raw[14];
    write_lock(&data->monitor_rwlock);
    mpu9250_check_deadlock(data);
    data->lock_ordered = true;
    mutex_lock(&data->recursive_lock);

    if (mpu9250_read(data, MPU9250_ACCEL_XOUT_H, raw, 6)) goto out;
    convert_accel(raw, data->accel_g, data->accel_scale);
    if (mpu9250_read(data, MPU9250_GYRO_XOUT_H, raw, 6)) goto out;
    convert_gyro(raw, data->gyro_dps, data->gyro_scale);
    if (mpu9250_read(data, MPU9250_MAG_XOUT_L, raw, 6)) goto out;
    convert_mag(raw, data->mag_uT);
    mpu9250_read_dmp_quat(data, data->quat);
    data->data_ready = true;

    spin_lock(&data->data_lock);
    wake_up_all(&data->wq);
    spin_unlock(&data->data_lock);
    raw_notifier_call_chain(&data->notifier, 0, data);

out:
    mutex_unlock(&data->recursive_lock);
    data->lock_ordered = false;
    write_unlock(&data->monitor_rwlock);
}

int mpu9250_fileops_init(struct mpu9250_data *data) {
    int ret;
    ret = alloc_chrdev_region(&data->dev_t, 0, 1, MPU9250_DEVICE_NAME);
    if (ret) return ret;

    cdev_init(&data->cdev, &(struct file_operations){
        .owner = THIS_MODULE,
        .open = mpu9250_open,
        .release = mpu9250_release,
        .read = mpu9250_read,
        .write = mpu9250_write,
        .unlocked_ioctl = mpu9250_ioctl,
        .poll = mpu9250_poll,
        .mmap = mpu9250_mmap,
    });
    ret = cdev_add(&data->cdev, data->dev_t, 1);
    if (ret) goto err_chrdev;

    data->class = class_create(THIS_MODULE, MPU9250_CLASS_NAME);
    if (IS_ERR(data->class)) {
        ret = PTR_ERR(data->class);
        goto err_cdev;
    }

    data->device = device_create(data->class, NULL, data->dev_t, NULL, MPU9250_DEVICE_NAME);
    if (IS_ERR(data->device)) {
        ret = PTR_ERR(data->device);
        goto err_class;
    }
    return 0;

err_class:
    class_destroy(data->class);
err_cdev:
    cdev_del(&data->cdev);
err_chrdev:
    unregister_chrdev_region(data->dev_t, 1);
    return ret;
}

void mpu9250_fileops_cleanup(struct mpu9250_data *data) {
    device_destroy(data->class, data->dev_t);
    class_destroy(data->class);
    cdev_del(&data->cdev);
    unregister_chrdev_region(data->dev_t, 1);
}