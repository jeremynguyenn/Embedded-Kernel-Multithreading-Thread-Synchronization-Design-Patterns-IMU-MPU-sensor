#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <semaphore.h>
#include <mqueue.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <getopt.h>
#include <signal.h>
#include <time.h>
#include <pthread.h>
#include "mpu9250.h"

#define SHM_NAME "/mpu9250_shm"
#define SEM_NAME "/mpu9250_sem"
#define FIFO_NAME "/tmp/mpu9250_fifo"
#define PIPE_NAME "/tmp/mpu9250_pipe"
#define MAX_THREADS 8

static volatile int running = 1;
static long queue_size = 10;
static sem_t strict_alt_sem;
static pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t data_cond = PTHREAD_COND_INITIALIZER;
static pthread_rwlock_t data_rwlock = PTHREAD_RWLOCK_INITIALIZER;
static pthread_barrier_t thread_barrier;

// Thread pool structure with ITC, monitor, recursive lock, deadlock flag
struct ipc_thread_pool {
    pthread_t threads[MAX_THREADS];
    int num_threads;
    bool detached;
    void (*callback)(struct mpu9250_mq_data *); // Publisher callback
    struct mpu9250_mq_data *shared_data;
    pthread_mutex_t monitor_lock;
    pthread_cond_t monitor_cond;
    struct notifier_head notifier; // Notification chain for ITC
    pthread_mutexattr_t recursive_attr; // For recursive mutex
    pthread_mutex_t recursive_lock; // Recursive mutex for nested locks
    bool lock_ordered; // Deadlock prevention flag
    sem_t philos_sem[5]; // Dining philosophers
};

// Callback for publisher-subscriber
static void data_ready_callback(struct mpu9250_mq_data *data) {
    printf("Callback: Accel: %.2f %.2f %.2f, Gyro: %.2f %.2f %.2f, Mag: %.2f %.2f %.2f, Quat: %.2f %.2f %.2f %.2f\n",
           data->accel[0], data->accel[1], data->accel[2],
           data->gyro[0], data->gyro[1], data->gyro[2],
           data->mag[0], data->mag[1], data->mag[2],
           data->quat[0], data->quat[1], data->quat[2], data->quat[3]);
}

static void cleanup_func(void *arg) {
    struct ipc_thread_pool *pool = (struct ipc_thread_pool *)arg;
    if (pool->shared_data) free(pool->shared_data);
    pthread_mutex_unlock(&pool->monitor_lock);
    pthread_cond_broadcast(&pool->monitor_cond);
    sem_post(&strict_alt_sem);
    pthread_rwlock_unlock(&data_rwlock);
    // Deadlock check
    if (pool->lock_ordered) printf("Warning: Potential deadlock condition\n");
}

// Signal handler for termination
static void signal_handler(int sig) {
    running = 0;
    pthread_cond_broadcast(&data_cond);
}

// Listener thread for MQ with deferred cancellation, monitor, rwlock, recursive lock
static void *mq_thread(void *arg) {
    struct ipc_thread_pool *pool = (struct ipc_thread_pool *)arg;
    mqd_t mq = -1;
    struct mpu9250_mq_data *data = malloc(sizeof(struct mpu9250_mq_data));
    if (!data) pthread_exit(NULL);
    pthread_cleanup_push(cleanup_func, pool);
    pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL); // Deferred cancellation

    pthread_barrier_wait(&thread_barrier); // Sync start with other threads
    struct mq_attr attr = { .mq_maxmsg = queue_size, .mq_msgsize = sizeof(struct mpu9250_mq_data) };
    sem_wait(&strict_alt_sem);
    mq = mq_open(MPU9250_MQ_NAME, O_RDONLY | O_CREAT, 0666, &attr);
    if (mq == -1) {
        fprintf(stderr, "Open message queue failed: %s\n", strerror(errno));
        free(data);
        sem_post(&strict_alt_sem);
        pthread_exit(NULL);
    }

    while (running) {
        pthread_rwlock_rdlock(&data_rwlock); // Read lock for shared data
        pthread_mutex_lock(&pool->recursive_lock); // Recursive lock
        if (!pool->lock_ordered) { // Deadlock prevention: lock ordering
            pool->lock_ordered = true;
            if (pthread_mutex_trylock(&data_mutex) == EBUSY) {
                printf("Deadlock detected: Hold and wait\n");
                pthread_mutex_unlock(&pool->recursive_lock);
                pthread_rwlock_unlock(&data_rwlock);
                continue;
            }
            pthread_mutex_unlock(&data_mutex);
        }
        // Dining philosophers example
        sem_wait(&pool->philos_sem[0]);
        sem_wait(&pool->philos_sem[1]);
        if (mq_receive(mq, (char *)data, sizeof(*data), NULL) == -1) {
            if (errno == EINTR) continue;
            fprintf(stderr, "MQ receive failed: %s\n", strerror(errno));
            sem_post(&pool->philos_sem[0]);
            sem_post(&pool->philos_sem[1]);
            break;
        }
        pthread_mutex_lock(&pool->monitor_lock); // Monitor enter for producer-consumer
        memcpy(pool->shared_data, data, sizeof(*data));
        pool->callback(pool->shared_data); // Publish
        raw_notifier_call_chain(&pool->notifier, 0, data); // ITC notify subscribers
        pthread_cond_signal(&pool->monitor_cond); // Signal consumer threads
        pthread_mutex_unlock(&pool->monitor_lock); // Monitor exit
        pthread_rwlock_unlock(&data_rwlock);
        pthread_mutex_unlock(&pool->recursive_lock);
        sem_post(&pool->philos_sem[0]);
        sem_post(&pool->philos_sem[1]);
        sem_post(&strict_alt_sem);
        usleep(100000);
        pthread_testcancel(); // Cancellation point
    }
    pthread_cleanup_pop(1);
    mq_close(mq);
    mq_unlink(MPU9250_MQ_NAME);
    free(data);
    pthread_exit(NULL);
}

// Listener thread for SHM
static void *shm_thread(void *arg) {
    struct ipc_thread_pool *pool = (struct ipc_thread_pool *)arg;
    int shm_fd = shm_open(SHM_NAME, O_RDONLY | O_CREAT, 0666);
    if (shm_fd == -1) {
        fprintf(stderr, "Open shared memory failed: %s\n", strerror(errno));
        pthread_exit(NULL);
    }
    ftruncate(shm_fd, sizeof(struct mpu9250_mq_data));
    struct mpu9250_mq_data *shm_data = mmap(NULL, sizeof(struct mpu9250_mq_data), PROT_READ, MAP_SHARED, shm_fd, 0);
    if (shm_data == MAP_FAILED) {
        fprintf(stderr, "mmap failed: %s\n", strerror(errno));
        close(shm_fd);
        shm_unlink(SHM_NAME);
        pthread_exit(NULL);
    }
    sem_t *sem = sem_open(SEM_NAME, O_CREAT, 0666, 0);
    if (sem == SEM_FAILED) {
        fprintf(stderr, "Open semaphore failed: %s\n", strerror(errno));
        munmap(shm_data, sizeof(struct mpu9250_mq_data));
        close(shm_fd);
        shm_unlink(SHM_NAME);
        pthread_exit(NULL);
    }

    pthread_cleanup_push(cleanup_func, pool);
    pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
    pthread_barrier_wait(&thread_barrier);
    struct timespec ts;
    while (running) {
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 1; // Timeout for bounded waiting
        if (sem_timedwait(sem, &ts) == -1) {
            if (errno == ETIMEDOUT) continue;
            fprintf(stderr, "Sem wait failed: %s\n", strerror(errno));
            break;
        }
        pthread_rwlock_rdlock(&data_rwlock);
        pthread_mutex_lock(&pool->recursive_lock);
        if (!pool->lock_ordered) {
            pool->lock_ordered = true;
            if (pthread_mutex_trylock(&data_mutex) == EBUSY) {
                printf("Deadlock detected: Hold and wait\n");
                pthread_mutex_unlock(&pool->recursive_lock);
                pthread_rwlock_unlock(&data_rwlock);
                continue;
            }
            pthread_mutex_unlock(&data_mutex);
        }
        // Dining philosophers
        sem_wait(&pool->philos_sem[2]);
        sem_wait(&pool->philos_sem[3]);
        pthread_mutex_lock(&pool->monitor_lock);
        memcpy(pool->shared_data, shm_data, sizeof(*shm_data));
        pool->callback(pool->shared_data);
        raw_notifier_call_chain(&pool->notifier, 0, pool->shared_data);
        pthread_cond_signal(&pool->monitor_cond);
        pthread_mutex_unlock(&pool->monitor_lock);
        pthread_rwlock_unlock(&data_rwlock);
        pthread_mutex_unlock(&pool->recursive_lock);
        sem_post(&pool->philos_sem[2]);
        sem_post(&pool->philos_sem[3]);
        usleep(100000);
        pthread_testcancel();
    }
    pthread_cleanup_pop(1);
    sem_close(sem);
    sem_unlink(SEM_NAME);
    munmap(shm_data, sizeof(struct mpu9250_mq_data));
    close(shm_fd);
    shm_unlink(SHM_NAME);
    pthread_exit(NULL);
}

// Listener thread for FIFO
static void *fifo_thread(void *arg) {
    struct ipc_thread_pool *pool = (struct ipc_thread_pool *)arg;
    mkfifo(FIFO_NAME, 0666);
    int fifo_fd = open(FIFO_NAME, O_RDONLY);
    if (fifo_fd == -1) {
        fprintf(stderr, "Open FIFO failed: %s\n", strerror(errno));
        pthread_exit(NULL);
    }
    struct mpu9250_mq_data *data = malloc(sizeof(struct mpu9250_mq_data));
    if (!data) {
        close(fifo_fd);
        unlink(FIFO_NAME);
        pthread_exit(NULL);
    }

    pthread_cleanup_push(cleanup_func, pool);
    pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
    pthread_barrier_wait(&thread_barrier);
    while (running) {
        pthread_rwlock_rdlock(&data_rwlock);
        pthread_mutex_lock(&pool->recursive_lock);
        if (!pool->lock_ordered) {
            pool->lock_ordered = true;
            if (pthread_mutex_trylock(&data_mutex) == EBUSY) {
                printf("Deadlock detected: Hold and wait\n");
                pthread_mutex_unlock(&pool->recursive_lock);
                pthread_rwlock_unlock(&data_rwlock);
                continue;
            }
            pthread_mutex_unlock(&data_mutex);
        }
        // Dining philosophers
        sem_wait(&pool->philos_sem[3]);
        sem_wait(&pool->philos_sem[4]);
        if (read(fifo_fd, data, sizeof(*data)) != sizeof(*data)) {
            fprintf(stderr, "FIFO read failed: %s\n", strerror(errno));
            sem_post(&pool->philos_sem[3]);
            sem_post(&pool->philos_sem[4]);
            break;
        }
        pthread_mutex_lock(&pool->monitor_lock);
        memcpy(pool->shared_data, data, sizeof(*data));
        pool->callback(pool->shared_data);
        raw_notifier_call_chain(&pool->notifier, 0, data);
        pthread_cond_signal(&pool->monitor_cond);
        pthread_mutex_unlock(&pool->monitor_lock);
        pthread_rwlock_unlock(&data_rwlock);
        pthread_mutex_unlock(&pool->recursive_lock);
        sem_post(&pool->philos_sem[3]);
        sem_post(&pool->philos_sem[4]);
        usleep(100000);
        pthread_testcancel();
    }
    pthread_cleanup_pop(1);
    free(data);
    close(fifo_fd);
    unlink(FIFO_NAME);
    pthread_exit(NULL);
}

// Listener thread for Pipe
static void *pipe_thread(void *arg) {
    struct ipc_thread_pool *pool = (struct ipc_thread_pool *)arg;
    int pipe_fd[2];
    if (pipe(pipe_fd) == -1) {
        fprintf(stderr, "Pipe creation failed: %s\n", strerror(errno));
        pthread_exit(NULL);
    }
    int dev_fd = open("/dev/mpu9250", O_RDONLY);
    if (dev_fd < 0) {
        close(pipe_fd[0]);
        close(pipe_fd[1]);
        pthread_exit(NULL);
    }
    pid_t child_pid = fork();
    if (child_pid == 0) {
        close(pipe_fd[0]);
        struct mpu9250_mq_data data;
        while (running) {
            sem_wait(&strict_alt_sem);
            pthread_rwlock_wrlock(&data_rwlock);
            pthread_mutex_lock(&pool->recursive_lock);
            if (!pool->lock_ordered) {
                pool->lock_ordered = true;
                if (pthread_mutex_trylock(&data_mutex) == EBUSY) {
                    printf("Deadlock detected: Hold and wait\n");
                    pthread_mutex_unlock(&pool->recursive_lock);
                    pthread_rwlock_unlock(&data_rwlock);
                    sem_post(&strict_alt_sem);
                    continue;
                }
                pthread_mutex_unlock(&data_mutex);
            }
            // Dining philosophers
            sem_wait(&pool->philos_sem[0]);
            sem_wait(&pool->philos_sem[4]);
            struct mpu9250_sensor_data sensor;
            if (ioctl(dev_fd, MPU9250_IOCTL_READ_ACCEL, &sensor) < 0) break;
            memcpy(data.accel, sensor.values, sizeof(float)*3);
            if (ioctl(dev_fd, MPU9250_IOCTL_READ_GYRO, &sensor) < 0) break;
            memcpy(data.gyro, sensor.values, sizeof(float)*3);
            if (ioctl(dev_fd, MPU9250_IOCTL_READ_MAG, &sensor) < 0) break;
            memcpy(data.mag, sensor.values, sizeof(float)*3);
            if (ioctl(dev_fd, MPU9250_IOCTL_READ_DMP, &sensor) < 0) break;
            memcpy(data.quat, sensor.values, sizeof(float)*4);
            if (write(pipe_fd[1], &data, sizeof(data)) != sizeof(data)) {
                fprintf(stderr, "Pipe write failed: %s\n", strerror(errno));
                break;
            }
            pthread_mutex_lock(&pool->monitor_lock);
            memcpy(pool->shared_data, &data, sizeof(data));
            pool->callback(pool->shared_data);
            raw_notifier_call_chain(&pool->notifier, 0, &data);
            pthread_cond_signal(&pool->monitor_cond);
            pthread_mutex_unlock(&pool->monitor_lock);
            pthread_rwlock_unlock(&data_rwlock);
            pthread_mutex_unlock(&pool->recursive_lock);
            sem_post(&pool->philos_sem[0]);
            sem_post(&pool->philos_sem[4]);
            sem_post(&strict_alt_sem);
            usleep(100000);
            pthread_testcancel();
        }
        close(pipe_fd[1]);
        close(dev_fd);
        exit(0);
    } else {
        close(pipe_fd[1]);
        struct mpu9250_mq_data data;
        pthread_cleanup_push(cleanup_func, pool);
        pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
        pthread_barrier_wait(&thread_barrier);
        while (running) {
            if (read(pipe_fd[0], &data, sizeof(data)) != sizeof(data)) {
                fprintf(stderr, "Pipe read failed: %s\n", strerror(errno));
                break;
            }
            pthread_mutex_lock(&pool->monitor_lock);
            memcpy(pool->shared_data, &data, sizeof(data));
            pool->callback(pool->shared_data);
            raw_notifier_call_chain(&pool->notifier, 0, &data);
            pthread_cond_signal(&pool->monitor_cond);
            pthread_mutex_unlock(&pool->monitor_lock);
            usleep(100000);
            pthread_testcancel();
        }
        pthread_cleanup_pop(1);
        close(pipe_fd[0]);
        kill(child_pid, SIGTERM);
        wait(NULL);
        pthread_exit(NULL);
    }
}

static int init_thread_pool(struct ipc_thread_pool *pool, int num_threads, bool detached) {
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
    RAW_INIT_NOTIFIER_HEAD(&pool->notifier); // ITC
    pool->lock_ordered = false;
    for (int i = 0; i < 5; i++) sem_init(&pool->philos_sem[i], 0, 1); // Dining philosophers
    return 0;
}

static int run_ipc_test(int test_mq, int test_shm, int test_fifo, int test_pipe, bool detached) {
    struct ipc_thread_pool pool = {0};
    int num_threads = (test_mq + test_shm + test_fifo + test_pipe);
    if (init_thread_pool(&pool, num_threads, detached)) return 1;
    pthread_barrier_init(&thread_barrier, NULL, num_threads + 1); // Barrier for sync start/resume

    int thread_idx = 0;
    if (test_mq && thread_idx < pool.num_threads)
        pthread_create(&pool.threads[thread_idx++], NULL, mq_thread, &pool);
    if (test_shm && thread_idx < pool.num_threads)
        pthread_create(&pool.threads[thread_idx++], NULL, shm_thread, &pool);
    if (test_fifo && thread_idx < pool.num_threads)
        pthread_create(&pool.threads[thread_idx++], NULL, fifo_thread, &pool);
    if (test_pipe && thread_idx < pool.num_threads)
        pthread_create(&pool.threads[thread_idx++], NULL, pipe_thread, &pool);

    pthread_barrier_wait(&thread_barrier); // Resume all threads
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
    int test_mq = 0, test_shm = 0, test_fifo = 0, test_pipe = 0, test_detach = 0;
    int opt;

    static struct option long_options[] = {
        {"mq", no_argument, &test_mq, 1},
        {"shm", no_argument, &test_shm, 1},
        {"fifo", no_argument, &test_fifo, 1},
        {"pipe", no_argument, &test_pipe, 1},
        {"detach", no_argument, &test_detach, 1},
        {"queue-size", required_argument, 0, 'q'},
        {0, 0, 0, 0}
    };
    int opt_index;
    while (getopt_long(argc, argv, "", long_options, &opt_index) != -1) {
        switch (opt_index) {
            case 4: test_detach = 1; break;
            case 5:
                queue_size = atol(optarg);
                if (queue_size < 1 || queue_size > 100) {
                    fprintf(stderr, "Invalid queue size (1-100)\n");
                    return 1;
                }
                break;
            default:
                printf("Usage: %s [--mq | --shm | --fifo | --pipe | --detach] [--queue-size SIZE]\n", argv[0]);
                return 1;
        }
    }
    if (!test_mq && !test_shm && !test_fifo && !test_pipe) {
        printf("Please specify --mq, --shm, --fifo, or --pipe\n");
        return 1;
    }

    sem_init(&strict_alt_sem, 0, 1);
    pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    signal(SIGSEGV, signal_handler);

    return run_ipc_test(test_mq, test_shm, test_fifo, test_pipe, test_detach);
}