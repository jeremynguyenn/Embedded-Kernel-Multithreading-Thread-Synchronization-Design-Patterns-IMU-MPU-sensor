#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <stdint.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <mqueue.h>
#include <getopt.h>
#include "mpu9250.h"

#define MAX_THREADS 8

static int fd = -1;
static mqd_t mq = -1;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
static pthread_rwlock_t rwlock = PTHREAD_RWLOCK_INITIALIZER;
static volatile int running = 1;
static int test_accel = 0, test_gyro = 0, test_mag = 0, test_dmp = 0, test_detach = 0;
static unsigned int sample_rate = 100;
static int accel_scale = ACCEL_SCALE_2G;
static int gyro_scale = GYRO_SCALE_250DPS;
static pthread_barrier_t thread_barrier;

// Thread pool structure with ITC, monitor, recursive lock
struct sensor_thread_pool {
    pthread_t threads[MAX_THREADS];
    int num_threads;
    bool detached;
    void (*callback)(struct mpu9250_mq_data *); // Publisher callback
    struct mpu9250_mq_data *shared_data;
    pthread_mutex_t monitor_lock;
    pthread_cond_t monitor_cond;
    struct notifier_head notifier; // ITC
    pthread_mutexattr_t recursive_attr;
    pthread_mutex_t recursive_lock; // Recursive
    bool lock_ordered;
    sem_t philos_sem[5]; // Dining philosophers for resource sync (5 resources)
};

// Callback
static void data_ready_callback(struct mpu9250_mq_data *data) {
    printf("Callback: Accel: %.2f %.2f %.2f, Gyro: %.2f %.2f %.2f, Mag: %.2f %.2f %.2f, Quat: %.2f %.2f %.2f %.2f\n",
           data->accel[0], data->accel[1], data->accel[2],
           data->gyro[0], data->gyro[1], data->gyro[2],
           data->mag[0], data->mag[1], data->mag[2],
           data->quat[0], data->quat[1], data->quat[2], data->quat[3]);
}

static void cleanup_handler(void *arg) {
    struct sensor_thread_pool *pool = (struct sensor_thread_pool *)arg;
    if (fd >= 0) close(fd);
    if (mq != -1) {
        mq_close(mq);
        mq_unlink(MPU9250_MQ_NAME);
    }
    pthread_mutex_unlock(&pool->monitor_lock);
    pthread_cond_broadcast(&pool->monitor_cond);
    // Deadlock check
    if (pool->lock_ordered) printf("Warning: Potential deadlock condition\n");
}

static void signal_handler(int sig) {
    if (sig == SIGUSR1) {
        printf("MPU9250: Received SIGUSR1, continuing\n");
        return;
    }
    running = 0;
    pthread_cond_broadcast(&cond);
}

// Accel thread with deferred cancel, monitor, rwlock, recursive, deadlock prevention
static void *accel_thread(void *arg) {
    struct sensor_thread_pool *pool = (struct sensor_thread_pool *)arg;
    struct mpu9250_sensor_data *data = malloc(sizeof(struct mpu9250_sensor_data));
    if (!data) pthread_exit(NULL);
    pthread_cleanup_push(cleanup_handler, pool);
    pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
    pthread_barrier_wait(&thread_barrier);
    printf("Accel Thread ID: %lu\n", (unsigned long)pthread_self());
    while (running) {
        pthread_rwlock_rdlock(&rwlock);
        pthread_cond_wait(&cond, &mutex);
        pthread_mutex_lock(&pool->recursive_lock);
        if (!pool->lock_ordered) {
            pool->lock_ordered = true;
            if (pthread_mutex_trylock(&mutex) == EBUSY) {
                printf("Deadlock detected: Hold and wait\n");
                pthread_mutex_unlock(&pool->recursive_lock);
                pthread_rwlock_unlock(&rwlock);
                continue;
            }
            pthread_mutex_unlock(&mutex);
        }
        // Dining philosophers
        sem_wait(&pool->philos_sem[0]);
        sem_wait(&pool->philos_sem[1]);
        if (ioctl(fd, MPU9250_IOCTL_READ_ACCEL, data) < 0) {
            fprintf(stderr, "Accel thread read failed: %s\n", strerror(errno));
        } else {
            struct mpu9250_mq_data mq_data = {0};
            memcpy(mq_data.accel, data->values, sizeof(data->values));
            pthread_mutex_lock(&pool->monitor_lock);
            memcpy(pool->shared_data->accel, mq_data.accel, sizeof(mq_data.accel));
            pool->callback(pool->shared_data);
            raw_notifier_call_chain(&pool->notifier, 0, pool->shared_data);
            pthread_cond_signal(&pool->monitor_cond);
            pthread_mutex_unlock(&pool->monitor_lock);
        }
        pthread_rwlock_unlock(&rwlock);
        pthread_mutex_unlock(&pool->recursive_lock);
        sem_post(&pool->philos_sem[0]);
        sem_post(&pool->philos_sem[1]);
        pthread_testcancel();
    }
    pthread_cleanup_pop(1);
    free(data);
    pthread_exit(NULL);
}

// Gyro thread
static void *gyro_thread(void *arg) {
    struct sensor_thread_pool *pool = (struct sensor_thread_pool *)arg;
    struct mpu9250_sensor_data *data = malloc(sizeof(struct mpu9250_sensor_data));
    if (!data) pthread_exit(NULL);
    pthread_cleanup_push(cleanup_handler, pool);
    pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
    pthread_barrier_wait(&thread_barrier);
    printf("Gyro Thread ID: %lu\n", (unsigned long)pthread_self());
    while (running) {
        pthread_rwlock_rdlock(&rwlock);
        pthread_cond_wait(&cond, &mutex);
        pthread_mutex_lock(&pool->recursive_lock);
        if (!pool->lock_ordered) {
            pool->lock_ordered = true;
            if (pthread_mutex_trylock(&mutex) == EBUSY) {
                printf("Deadlock detected: Hold and wait\n");
                pthread_mutex_unlock(&pool->recursive_lock);
                pthread_rwlock_unlock(&rwlock);
                continue;
            }
            pthread_mutex_unlock(&mutex);
        }
        // Dining philosophers
        sem_wait(&pool->philos_sem[1]);
        sem_wait(&pool->philos_sem[2]);
        if (ioctl(fd, MPU9250_IOCTL_READ_GYRO, data) < 0) {
            fprintf(stderr, "Gyro thread read failed: %s\n", strerror(errno));
        } else {
            struct mpu9250_mq_data mq_data = {0};
            memcpy(mq_data.gyro, data->values, sizeof(data->values));
            pthread_mutex_lock(&pool->monitor_lock);
            memcpy(pool->shared_data->gyro, mq_data.gyro, sizeof(mq_data.gyro));
            pool->callback(pool->shared_data);
            raw_notifier_call_chain(&pool->notifier, 0, pool->shared_data);
            pthread_cond_signal(&pool->monitor_cond);
            pthread_mutex_unlock(&pool->monitor_lock);
        }
        pthread_rwlock_unlock(&rwlock);
        pthread_mutex_unlock(&pool->recursive_lock);
        sem_post(&pool->philos_sem[1]);
        sem_post(&pool->philos_sem[2]);
        pthread_testcancel();
    }
    pthread_cleanup_pop(1);
    free(data);
    pthread_exit(NULL);
}

// Mag thread
static void *mag_thread(void *arg) {
    struct sensor_thread_pool *pool = (struct sensor_thread_pool *)arg;
    struct mpu9250_sensor_data *data = malloc(sizeof(struct mpu9250_sensor_data));
    if (!data) pthread_exit(NULL);
    pthread_cleanup_push(cleanup_handler, pool);
    pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
    pthread_barrier_wait(&thread_barrier);
    printf("Mag Thread ID: %lu\n", (unsigned long)pthread_self());
    while (running) {
        pthread_rwlock_rdlock(&rwlock);
        pthread_cond_wait(&cond, &mutex);
        pthread_mutex_lock(&pool->recursive_lock);
        if (!pool->lock_ordered) {
            pool->lock_ordered = true;
            if (pthread_mutex_trylock(&mutex) == EBUSY) {
                printf("Deadlock detected: Hold and wait\n");
                pthread_mutex_unlock(&pool->recursive_lock);
                pthread_rwlock_unlock(&rwlock);
                continue;
            }
            pthread_mutex_unlock(&mutex);
        }
        // Dining philosophers
        sem_wait(&pool->philos_sem[2]);
        sem_wait(&pool->philos_sem[3]);
        if (ioctl(fd, MPU9250_IOCTL_READ_MAG, data) < 0) {
            fprintf(stderr, "Mag thread read failed: %s\n", strerror(errno));
        } else {
            struct mpu9250_mq_data mq_data = {0};
            memcpy(mq_data.mag, data->values, sizeof(data->values));
            pthread_mutex_lock(&pool->monitor_lock);
            memcpy(pool->shared_data->mag, mq_data.mag, sizeof(mq_data.mag));
            pool->callback(pool->shared_data);
            raw_notifier_call_chain(&pool->notifier, 0, pool->shared_data);
            pthread_cond_signal(&pool->monitor_cond);
            pthread_mutex_unlock(&pool->monitor_lock);
        }
        pthread_rwlock_unlock(&rwlock);
        pthread_mutex_unlock(&pool->recursive_lock);
        sem_post(&pool->philos_sem[2]);
        sem_post(&pool->philos_sem[3]);
        pthread_testcancel();
    }
    pthread_cleanup_pop(1);
    free(data);
    pthread_exit(NULL);
}

// DMP thread
static void *dmp_thread(void *arg) {
    struct sensor_thread_pool *pool = (struct sensor_thread_pool *)arg;
    struct mpu9250_sensor_data *data = malloc(sizeof(struct mpu9250_sensor_data));
    if (!data) pthread_exit(NULL);
    pthread_cleanup_push(cleanup_handler, pool);
    pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
    pthread_barrier_wait(&thread_barrier);
    printf("DMP Thread ID: %lu\n", (unsigned long)pthread_self());
    while (running) {
        pthread_rwlock_rdlock(&rwlock);
        pthread_cond_wait(&cond, &mutex);
        pthread_mutex_lock(&pool->recursive_lock);
        if (!pool->lock_ordered) {
            pool->lock_ordered = true;
            if (pthread_mutex_trylock(&mutex) == EBUSY) {
                printf("Deadlock detected: Hold and wait\n");
                pthread_mutex_unlock(&pool->recursive_lock);
                pthread_rwlock_unlock(&rwlock);
                continue;
            }
            pthread_mutex_unlock(&mutex);
        }
        // Dining philosophers
        sem_wait(&pool->philos_sem[3]);
        sem_wait(&pool->philos_sem[4]);
        if (ioctl(fd, MPU9250_IOCTL_READ_DMP, data) < 0) {
            fprintf(stderr, "DMP thread read failed: %s\n", strerror(errno));
        } else {
            struct mpu9250_mq_data mq_data = {0};
            memcpy(mq_data.quat, data->values, sizeof(float)*4);
            pthread_mutex_lock(&pool->monitor_lock);
            memcpy(pool->shared_data->quat, mq_data.quat, sizeof(mq_data.quat));
            pool->callback(pool->shared_data);
            raw_notifier_call_chain(&pool->notifier, 0, pool->shared_data);
            pthread_cond_signal(&pool->monitor_cond);
            pthread_mutex_unlock(&pool->monitor_lock);
        }
        pthread_rwlock_unlock(&rwlock);
        pthread_mutex_unlock(&pool->recursive_lock);
        sem_post(&pool->philos_sem[3]);
        sem_post(&pool->philos_sem[4]);
        pthread_testcancel();
    }
    pthread_cleanup_pop(1);
    free(data);
    pthread_exit(NULL);
}

static int init_thread_pool(struct sensor_thread_pool *pool, int num_threads, bool detached) {
    pool->num_threads = num_threads > MAX_THREADS ? MAX_THREADS : num_threads;
    pool->detached = detached;
    pool->shared_data = malloc(sizeof(struct mpu9250_mq_data));
    if (!pool->shared_data) return -ENOMEM;
    pool->callback = data_ready_callback;
    pthread_mutex_init(&pool->monitor_lock, NULL);
    pthread_cond_init(&pool->monitor_cond, NULL);
    pthread_mutexattr_init(&pool->recursive_attr);
    pthread_mutexattr_settype(&pool->recursive_attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&pool->recursive_lock, &pool->recursive_attr);
    RAW_INIT_NOTIFIER_HEAD(&pool->notifier);
    pool->lock_ordered = false;
    for (int i = 0; i < 5; i++) sem_init(&pool->philos_sem[i], 0, 1); // Dining philosophers
    return 0;
}

static int run_sensor_test(int test_accel, int test_gyro, int test_mag, int test_dmp, bool detached) {
    struct sensor_thread_pool pool = {0};
    int num_threads = (test_accel + test_gyro + test_mag + test_dmp);
    if (init_thread_pool(&pool, num_threads, detached)) return 1;
    pthread_barrier_init(&thread_barrier, NULL, num_threads + 1);

    int thread_idx = 0;
    if (test_accel && thread_idx < pool.num_threads)
        pthread_create(&pool.threads[thread_idx++], NULL, accel_thread, &pool);
    if (test_gyro && thread_idx < pool.num_threads)
        pthread_create(&pool.threads[thread_idx++], NULL, gyro_thread, &pool);
    if (test_mag && thread_idx < pool.num_threads)
        pthread_create(&pool.threads[thread_idx++], NULL, mag_thread, &pool);
    if (test_dmp && thread_idx < pool.num_threads)
        pthread_create(&pool.threads[thread_idx++], NULL, dmp_thread, &pool);

    pthread_barrier_wait(&thread_barrier);
    if (pool.detached) {
        for (int i = 0; i < pool.num_threads; i++)
            pthread_detach(pool.threads[i]);
    } else {
        for (int i = 0; i < pool.num_threads; i++)
            pthread_join(pool.threads[i], NULL);
    }

    pthread_barrier_destroy(&thread_barrier);
    pthread_mutex_destroy(&pool.monitor_lock);
    pthread_cond_destroy(&pool.monitor_cond);
    pthread_mutex_destroy(&pool.recursive_lock);
    for (int i = 0; i < 5; i++) sem_destroy(&pool->philos_sem[i]);
    free(pool.shared_data);
    return 0;
}

int main(int argc, char **argv) {
    struct option long_options[] = {
        {"accel", no_argument, &test_accel, 1},
        {"gyro", no_argument, &test_gyro, 1},
        {"mag", no_argument, &test_mag, 1},
        {"dmp", no_argument, &test_dmp, 1},
        {"detach", no_argument, &test_detach, 1},
        {"sample-rate", required_argument, NULL, 'r'},
        {"accel-scale", required_argument, NULL, 'a'},
        {"gyro-scale", required_argument, NULL, 'g'},
        {0, 0, 0, 0}
    };
    int opt;
    while ((opt = getopt_long(argc, argv, "", long_options, NULL)) != -1) {
        switch (opt) {
            case 'r': sample_rate = atoi(optarg); break;
            case 'a': accel_scale = atoi(optarg); break;
            case 'g': gyro_scale = atoi(optarg); break;
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGUSR1, signal_handler);

    fd = open("/dev/mpu9250", O_RDWR);
    if (fd < 0) {
        perror("Open /dev/mpu9250 failed");
        return 1;
    }

    int ret = ioctl(fd, MPU9250_IOCTL_SET_ACCEL_SCALE, accel_scale);
    if (ret < 0) {
        perror("Set accel scale failed");
        goto err;
    }
    ret = ioctl(fd, MPU9250_IOCTL_SET_GYRO_SCALE, gyro_scale);
    if (ret < 0) {
        perror("Set gyro scale failed");
        goto err;
    }
    ret = ioctl(fd, MPU9250_IOCTL_SET_SAMPLE_RATE, sample_rate);
    if (ret < 0) {
        perror("Set sample rate failed");
        goto err;
    }

    uint8_t write_buf[2] = {MPU9250_PWR_MGMT_1, 0x00};
    ret = write(fd, write_buf, 2);
    if (ret != 2) {
        perror("Write wakeup failed");
        goto err;
    }
    printf("MPU9250: Wakeup written\n");

    struct mpu9250_reg reg;
    reg.reg = MPU9250_WHO_AM_I;
    ret = ioctl(fd, MPU9250_IOCTL_READ_REG, &reg);
    if (ret < 0) {
        perror("IOCTL read WHO_AM_I failed");
        goto err;
    }
    printf("MPU9250: WHO_AM_I = 0x%02X\n", reg.val);

    pid_t exec_pid = fork();
    if (exec_pid == 0) {
        // In child, close threads if any (but no threads in child)
        execl("/bin/ls", "ls", NULL);
        perror("execl failed");
        exit(1);
    } else if (exec_pid > 0) {
        // Parent waits
        wait(NULL);
    } else {
        perror("fork failed");
        goto err;
    }

    ret = run_sensor_test(test_accel, test_gyro, test_mag, test_dmp, test_detach);
    close(fd);
    return ret;

err:
    close(fd);
    return 1;
}