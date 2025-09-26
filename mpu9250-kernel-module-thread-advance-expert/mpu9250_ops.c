#include <linux/delay.h>
#include <linux/kthread.h>
#include "mpu9250.h"

static const struct regmap_config mpu9250_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
    .max_register = 0xFF,
};

int mpu9250_read(struct mpu9250_data *data, uint8_t reg, uint8_t *buf, uint16_t len) {
    int ret;
    write_lock(&data->monitor_rwlock);
    mpu9250_check_deadlock(data);
    data->lock_ordered = true;
    recursive_mutex_lock(&data->recursive_lock);
    ret = regmap_bulk_read(data->regmap, reg, buf, len);
    recursive_mutex_unlock(&data->recursive_lock);
    data->lock_ordered = false;
    write_unlock(&data->monitor_rwlock);
    return ret;
}

int mpu9250_write(struct mpu9250_data *data, uint8_t reg, uint8_t *buf, uint16_t len) {
    int ret;
    write_lock(&data->monitor_rwlock);
    mpu9250_check_deadlock(data);
    data->lock_ordered = true;
    recursive_mutex_lock(&data->recursive_lock);
    ret = regmap_bulk_write(data->regmap, reg, buf, len);
    recursive_mutex_unlock(&data->recursive_lock);
    data->lock_ordered = false;
    write_unlock(&data->monitor_rwlock);
    return ret;
}

int mpu9250_i2c_init(struct mpu9250_data *data) {
    uint8_t buf;
    read_lock(&data->monitor_rwlock);
    if (mpu9250_read(data, MPU9250_WHO_AM_I, &buf, 1)) {
        read_unlock(&data->monitor_rwlock);
        return -EIO;
    }
    read_unlock(&data->monitor_rwlock);
    if (buf != 0x71 && buf != 0x73) {
        dev_err(&data->client->dev, "Invalid WHO_AM_I: 0x%02X\n", buf);
        return -ENODEV;
    }
    buf = 0x80;
    if (mpu9250_write(data, MPU9250_PWR_MGMT_1, &buf, 1)) return -EIO;
    msleep(100);
    buf = 0x00;
    if (mpu9250_write(data, MPU9250_PWR_MGMT_1, &buf, 1)) return -EIO;
    buf = 0x01;
    if (mpu9250_write(data, MPU9250_PWR_MGMT_1, &buf, 1)) return -EIO;
    buf = 0x00;
    if (mpu9250_write(data, MPU9250_INT_ENABLE, &buf, 1)) return -EIO;
    if (mpu9250_set_sample_rate(data, 100)) return -EIO;
    if (mpu9250_set_accel_scale(data, ACCEL_SCALE_2G)) return -EIO;
    if (mpu9250_set_gyro_scale(data, GYRO_SCALE_250DPS)) return -EIO;
    return 0;
}

int mpu9250_fifo_init(struct mpu9250_data *data) {
    uint8_t buf = 0x00;
    if (mpu9250_write(data, MPU9250_USER_CTRL, &buf, 1)) return -EIO;
    buf = 0x40;
    if (mpu9250_write(data, MPU9250_USER_CTRL, &buf, 1)) return -EIO;
    buf = 0x78;
    return mpu9250_write(data, MPU9250_FIFO_EN, &buf, 1);
}

int mpu9250_read_fifo(struct mpu9250_data *data, uint8_t *buf, int *len) {
    uint8_t count[2];
    uint16_t fifo_count;
    write_lock(&data->monitor_rwlock);
    if (mpu9250_read(data, MPU9250_FIFO_COUNTH, count, 2)) {
        write_unlock(&data->monitor_rwlock);
        return -EIO;
    }
    fifo_count = (count[0] << 8) | count[1];
    if (fifo_count > MPU9250_MAX_FIFO) fifo_count = MPU9250_MAX_FIFO;
    *len = fifo_count;
    int ret = mpu9250_read(data, MPU9250_FIFO_R_W, buf, fifo_count);
    write_unlock(&data->monitor_rwlock);
    return ret;
}

int mpu9250_dmp_init(struct mpu9250_data *data) {
    uint8_t buf = 0x00;
    if (mpu9250_write(data, MPU9250_USER_CTRL, &buf, 1)) return -EIO;
    buf = 0x08;
    return mpu9250_write(data, MPU9250_USER_CTRL, &buf, 1);
}

void mpu9250_read_dmp_quat(struct mpu9250_data *data, float *quat) {
    uint8_t buf[24];
    read_lock(&data->monitor_rwlock);
    if (mpu9250_read(data, MPU9250_FIFO_R_W, buf, 24)) {
        read_unlock(&data->monitor_rwlock);
        return;
    }
    int16_t raw[4] = {
        (buf[0] << 8) | buf[1],
        (buf[4] << 8) | buf[5],
        (buf[8] << 8) | buf[9],
        (buf[12] << 8) | buf[13]
    };
    for (int i = 0; i < 4; i++)
        quat[i] = raw[i] / 16384.0f;
    read_unlock(&data->monitor_rwlock);
    if (data->data_ready_cb) data->data_ready_cb(data);
    raw_notifier_call_chain(&data->notifier, 0, data); // ITC
}

void convert_accel(uint8_t *raw, float *out, enum mpu9250_accel_scale scale) {
    int16_t accel[3];
    float scale_factor;
    accel[0] = (raw[0] << 8) | raw[1];
    accel[1] = (raw[2] << 8) | raw[3];
    accel[2] = (raw[4] << 8) | raw[5];
    switch (scale) {
        case ACCEL_SCALE_2G: scale_factor = ACCEL_SCALE_2G; break;
        case ACCEL_SCALE_4G: scale_factor = ACCEL_SCALE_4G; break;
        case ACCEL_SCALE_8G: scale_factor = ACCEL_SCALE_8G; break;
        case ACCEL_SCALE_16G: scale_factor = ACCEL_SCALE_16G; break;
        default: scale_factor = ACCEL_SCALE_2G;
    }
    for (int i = 0; i < 3; i++)
        out[i] = accel[i] / scale_factor;
}

void convert_gyro(uint8_t *raw, float *out, enum mpu9250_gyro_scale scale) {
    int16_t gyro[3];
    float scale_factor;
    gyro[0] = (raw[0] << 8) | raw[1];
    gyro[1] = (raw[2] << 8) | raw[3];
    gyro[2] = (raw[4] << 8) | raw[5];
    switch (scale) {
        case GYRO_SCALE_250DPS: scale_factor = GYRO_SCALE_250; break;
        case GYRO_SCALE_500DPS: scale_factor = GYRO_SCALE_500; break;
        case GYRO_SCALE_1000DPS: scale_factor = GYRO_SCALE_1000; break;
        case GYRO_SCALE_2000DPS: scale_factor = GYRO_SCALE_2000; break;
        default: scale_factor = GYRO_SCALE_250;
    }
    for (int i = 0; i < 3; i++)
        out[i] = gyro[i] / scale_factor;
}

void convert_mag(uint8_t *raw, float *out) {
    int16_t mag[3];
    mag[0] = (raw[1] << 8) | raw[0];
    mag[1] = (raw[3] << 8) | raw[2];
    mag[2] = (raw[5] << 8) | raw[4];
    for (int i = 0; i < 3; i++)
        out[i] = mag[i] * MAG_SCALE;
}

int mpu9250_set_accel_scale(struct mpu9250_data *data, enum mpu9250_accel_scale scale) {
    uint8_t buf;
    switch (scale) {
        case ACCEL_SCALE_2G: buf = 0x00; break;
        case ACCEL_SCALE_4G: buf = 0x08; break;
        case ACCEL_SCALE_8G: buf = 0x10; break;
        case ACCEL_SCALE_16G: buf = 0x18; break;
        default: return -EINVAL;
    }
    read_lock(&data->monitor_rwlock);
    if (mpu9250_write(data, MPU9250_ACCEL_CONFIG, &buf, 1)) {
        read_unlock(&data->monitor_rwlock);
        return -EIO;
    }
    data->accel_scale = scale;
    read_unlock(&data->monitor_rwlock);
    return 0;
}

int mpu9250_set_gyro_scale(struct mpu9250_data *data, enum mpu9250_gyro_scale scale) {
    uint8_t buf;
    switch (scale) {
        case GYRO_SCALE_250DPS: buf = 0x00; break;
        case GYRO_SCALE_500DPS: buf = 0x08; break;
        case GYRO_SCALE_1000DPS: buf = 0x10; break;
        case GYRO_SCALE_2000DPS: buf = 0x18; break;
        default: return -EINVAL;
    }
    read_lock(&data->monitor_rwlock);
    if (mpu9250_write(data, MPU9250_GYRO_CONFIG, &buf, 1)) {
        read_unlock(&data->monitor_rwlock);
        return -EIO;
    }
    data->gyro_scale = scale;
    read_unlock(&data->monitor_rwlock);
    return 0;
}

int mpu9250_set_sample_rate(struct mpu9250_data *data, unsigned int rate) {
    if (rate < 4 || rate > 1000) return -EINVAL;
    uint8_t buf = (uint8_t)(1000 / rate - 1);
    read_lock(&data->monitor_rwlock);
    if (mpu9250_write(data, MPU9250_SMPLRT_DIV, &buf, 1)) {
        read_unlock(&data->monitor_rwlock);
        return -EIO;
    }
    data->sample_rate = rate;
    read_unlock(&data->monitor_rwlock);
    return 0;
}

static void calibration_cleanup(void *arg) {
    struct mpu9250_data *data = (struct mpu9250_data *)arg;
    write_unlock(&data->monitor_rwlock);
    if (data->cleanup_handler) data->cleanup_handler(data);
}

static int mpu9250_calibrate(struct mpu9250_data *data) {
    uint8_t accel_raw[6], gyro_raw[6];
    float accel_sum[3] = {0}, gyro_sum[3] = {0};
    int samples = 100;

    // Set cleanup handler for deferred cancellation
    kthread_set_cleanup_handler(calibration_cleanup, data);
    write_lock(&data->monitor_rwlock);
    for (int i = 0; i < samples; i++) {
        if (kthread_should_stop()) {
            write_unlock(&data->monitor_rwlock);
            return -EINTR;
        }
        // Dining philosophers: Acquire 'chopsticks' (sems) to avoid deadlock in resource access
        down(&data->philos_sem[i % 5]);
        down(&data->philos_sem[(i + 1) % 5]);
        if (mpu9250_read(data, MPU9250_ACCEL_XOUT_H, accel_raw, 6)) {
            up(&data->philos_sem[i % 5]);
            up(&data->philos_sem[(i + 1) % 5]);
            write_unlock(&data->monitor_rwlock);
            return -EIO;
        }
        if (mpu9250_read(data, MPU9250_GYRO_XOUT_H, gyro_raw, 6)) {
            up(&data->philos_sem[i % 5]);
            up(&data->philos_sem[(i + 1) % 5]);
            write_unlock(&data->monitor_rwlock);
            return -EIO;
        }
        float accel[3], gyro[3];
        convert_accel(accel_raw, accel, data->accel_scale);
        convert_gyro(gyro_raw, gyro, data->gyro_scale);
        for (int j = 0; j < 3; j++) {
            accel_sum[j] += accel[j];
            gyro_sum[j] += gyro[j];
        }
        // Monitor: Wait for condition if resource not ready
        wait_event_interruptible(data->monitor_wq, true); // Example condition
        msleep(10);
        up(&data->philos_sem[i % 5]);
        up(&data->philos_sem[(i + 1) % 5]);
    }
    for (int i = 0; i < 3; i++) {
        data->accel_offset[i] = accel_sum[i] / samples;
        data->gyro_offset[i] = gyro_sum[i] / samples;
    }
    write_unlock(&data->monitor_rwlock);
    if (data->data_ready_cb) data->data_ready_cb(data);
    raw_notifier_call_chain(&data->notifier, 0, data);
    return 0;
}