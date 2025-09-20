#ifndef _MPU9250_H_
#define _MPU9250_H_

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/semaphore.h>
#include <linux/rwlock.h>
#include <linux/notifier.h>
#include <linux/completion.h>
#include <linux/atomic.h>
#include <linux/barrierg.h> // Assuming barrier header

// Constants
#define MPU9250_DEFAULT_ADDR    0x68
#define MPU9250_DEVICE_NAME     "mpu9250"
#define MPU9250_CLASS_NAME      "mpu9250_class"
#define MPU9250_MAX_FIFO        4096
#define DRIVER_VERSION          "2.3"
#define MPU9250_MQ_NAME         "/mpu9250_mq"
#define MAX_THREADS             8

// Scaling factors
#define ACCEL_SCALE_2G  16384.0f
#define ACCEL_SCALE_4G  8192.0f
#define ACCEL_SCALE_8G  4096.0f
#define ACCEL_SCALE_16G 2048.0f
#define GYRO_SCALE_250  131.0f
#define GYRO_SCALE_500  65.5f
#define GYRO_SCALE_1000 32.8f
#define GYRO_SCALE_2000 16.4f
#define MAG_SCALE       0.15f

// Registers
#define MPU9250_WHO_AM_I        0x75
#define MPU9250_PWR_MGMT_1      0x6B
#define MPU9250_ACCEL_XOUT_H    0x3B
#define MPU9250_GYRO_XOUT_H     0x43
#define MPU9250_MAG_XOUT_L      0x03
#define MPU9250_FIFO_EN         0x23
#define MPU9250_FIFO_COUNTH     0x72
#define MPU9250_FIFO_COUNTL     0x73
#define MPU9250_FIFO_R_W        0x74
#define MPU9250_DMP_CFG_1       0x01
#define MPU9250_DMP_CFG_2       0x02
#define MPU9250_ACCEL_CONFIG    0x1C
#define MPU9250_GYRO_CONFIG     0x1B
#define MPU9250_SMPLRT_DIV      0x19
#define MPU9250_INT_STATUS      0x3A
#define MPU9250_INT_ENABLE      0x38
#define MPU9250_INT_PIN_CFG     0x37
#define MPU9250_USER_CTRL       0x6A

// Enums
enum mpu9250_accel_scale {
    ACCEL_SCALE_2G = 0,
    ACCEL_SCALE_4G,
    ACCEL_SCALE_8G,
    ACCEL_SCALE_16G
};

enum mpu9250_gyro_scale {
    GYRO_SCALE_250DPS = 0,
    GYRO_SCALE_500DPS,
    GYRO_SCALE_1000DPS,
    GYRO_SCALE_2000DPS
};

enum mpu9250_interrupt {
    MPU9250_INTERRUPT_DATA_READY = 0,
    MPU9250_INTERRUPT_FIFO_OVERFLOW,
    MPU9250_INTERRUPT_DMP
};

// Structs
struct mpu9250_reg {
    uint8_t reg;
    uint8_t val;
};

struct mpu9250_sensor_data {
    float values[4];
};

struct mpu9250_mq_data {
    float accel[3];
    float gyro[3];
    float mag[3];
    float quat[4];
};

// Thread task with pause
struct mpu9250_thread_task {
    struct task_struct *thread;
    struct mpu9250_data *data;
    int task_id;
    bool paused;
};

// Thread pool with ITC, sync primitives
struct mpu9250_thread_pool {
    struct mpu9250_thread_task *tasks;
    int num_threads;
    int priority;
    struct semaphore thread_sem; // Strong semaphore for bounded waiting
    void (*data_ready_cb)(struct mpu9250_data *); // Callback for ITC
    struct raw_notifier_head notifier; // Notification chain
    struct completion barrier_comp; // For event pairs
    wait_queue_head_t pause_wq; // For pausing/resuming
    struct barrier_t thread_barrier; // Barrier for sync start
};

// Data struct with full sync
struct mpu9250_data {
    struct i2c_client *client;
    struct regmap *regmap;
    struct i2c_client *mag_client;
    struct mutex lock;
    struct rwlock_t rwlock;
    spinlock_t data_lock;
    struct work_struct read_work;
    wait_queue_head_t wq;
    dev_t dev_t;
    struct cdev cdev;
    struct class *class;
    struct device *device;
    float accel_g[3];
    float gyro_dps[3];
    float mag_uT[3];
    float quat[4];
    uint8_t *fifo_buf;
    int fifo_len;
    bool data_ready;
    uint32_t gs_flag;
    enum mpu9250_accel_scale accel_scale;
    enum mpu9250_gyro_scale gyro_scale;
    unsigned int sample_rate;
    float accel_offset[3];
    float gyro_offset[3];
    struct input_dev *input_dev;
    struct mpu9250_thread_pool thread_pool;
    struct semaphore sem;
    struct raw_notifier_head notifier;
    struct mutex recursive_lock;
    struct completion event_pair;
    struct rwlock_t monitor_rwlock;
    wait_queue_head_t monitor_wq;
    bool lock_ordered; // Deadlock prevention flag
    void (*cleanup_handler)(struct mpu9250_data *); // Cleanup handler
    atomic_t hold_count; // For deadlock detection
    atomic_t wait_count;
};

// IOCTL commands
#define MPU9250_IOC_MAGIC 'm'
#define MPU9250_IOCTL_READ_REG   _IOR('m', 1, struct mpu9250_reg)
#define MPU9250_IOCTL_WRITE_REG  _IOW('m', 2, struct mpu9250_reg)
#define MPU9250_IOCTL_READ_ACCEL _IOR('m', 3, struct mpu9250_sensor_data)
#define MPU9250_IOCTL_READ_GYRO  _IOR('m', 4, struct mpu9250_sensor_data)
#define MPU9250_IOCTL_READ_MAG   _IOR('m', 5, struct mpu9250_sensor_data)
#define MPU9250_IOCTL_READ_FIFO  _IOR('m', 6, struct mpu9250_sensor_data)
#define MPU9250_IOCTL_RESET      _IO('m', 7)
#define MPU9250_IOCTL_INIT_DMP   _IO('m', 8)
#define MPU9250_IOCTL_READ_DMP   _IOR('m', 9, struct mpu9250_sensor_data)
#define MPU9250_IOCTL_SET_NONBLOCK _IOW('m', 10, int)
#define MPU9250_IOCTL_SET_ACCEL_SCALE _IOW('m', 11, enum mpu9250_accel_scale)
#define MPU9250_IOCTL_SET_GYRO_SCALE _IOW('m', 12, enum mpu9250_gyro_scale)
#define MPU9250_IOCTL_SET_SAMPLE_RATE _IOW('m', 13, unsigned int)
#define MPU9250_IOCTL_CALIBRATE  _IO('m', 14)
#define MPU9250_IOCTL_PAUSE_THREADS _IO('m', 15)
#define MPU9250_IOCTL_RESUME_THREADS _IO('m', 16)
#define MPU9250_IOCTL_SET_NUM_THREADS _IOW('m', 17, int)

// Function prototypes
int mpu9250_alloc_buffers(struct mpu9250_data *data);
void mpu9250_free_buffers(struct mpu9250_data *data);
int mpu9250_read(struct mpu9250_data *data, uint8_t reg, uint8_t *buf, uint16_t len);
int mpu9250_write(struct mpu9250_data *data, uint8_t reg, uint8_t *buf, uint16_t len);
int mpu9250_i2c_init(struct mpu9250_data *data);
int mpu9250_fifo_init(struct mpu9250_data *data);
int mpu9250_read_fifo(struct mpu9250_data *data, uint8_t *buf, int *len);
int mpu9250_dmp_init(struct mpu9250_data *data);
void mpu9250_read_dmp_quat(struct mpu9250_data *data, float *quat);
void mpu9250_read_work(struct work_struct *work);
irqreturn_t mpu9250_irq_handler(int irq, void *dev_id);
int mpu9250_fileops_init(struct mpu9250_data *data);
void mpu9250_fileops_cleanup(struct mpu9250_data *data);
int mpu9250_set_accel_scale(struct mpu9250_data *data, enum mpu9250_accel_scale scale);
int mpu9250_set_gyro_scale(struct mpu9250_data *data, enum mpu9250_gyro_scale scale);
int mpu9250_set_sample_rate(struct mpu9250_data *data, unsigned int rate);
int mpu9250_calibrate(struct mpu9250_data *data);
int mpu9250_thread_pool_init(struct mpu9250_data *data, int num_threads);
void mpu9250_thread_pool_cleanup(struct mpu9250_data *data);
int mpu9250_pause_threads(struct mpu9250_data *data);
int mpu9250_resume_threads(struct mpu9250_data *data);
void mpu9250_notify_subscribers(struct mpu9250_data *data);
int mpu9250_set_num_threads(struct mpu9250_data *data, int num_threads);
void mpu9250_check_deadlock(struct mpu9250_data *data);

#endif /* _MPU9250_H_ */