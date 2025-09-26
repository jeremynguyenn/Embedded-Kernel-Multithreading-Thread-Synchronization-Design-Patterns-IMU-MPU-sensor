#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <semaphore.h>
#include <string.h>
#include <unistd.h>
#include "mpu9250.h"

#define MAX_THREADS 4
#define BUFFER_SIZE 100

struct map_reduce_pool {
    pthread_t map_threads[MAX_THREADS];
    pthread_t reduce_thread;
    struct mpu9250_mq_data buffer[BUFFER_SIZE];
    int buffer_count;
    pthread_mutex_t buffer_mutex;
    pthread_cond_t buffer_cond;
    sem_t philos_sem[5]; // Dining philosophers for resource sync
    volatile int running;
};

static void data_ready_callback(struct mpu9250_mq_data *data) {
    printf("Reduce: Accel: %.2f %.2f %.2f, Gyro: %.2f %.2f %.2f, Mag: %.2f %.2f %.2f, Quat: %.2f %.2f %.2f %.2f\n",
           data->accel[0], data->accel[1], data->accel[2],
           data->gyro[0], data->gyro[1], data->gyro[2],
           data->mag[0], data->mag[1], data->mag[2],
           data->quat[0], data->quat[1], data->quat[2], data->quat[3]);
}

static void *map_thread(void *arg) {
    struct map_reduce_pool *pool = (struct map_reduce_pool *)arg;
    int fd = open("/dev/mpu9250", O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Open /dev/mpu9250 failed\n");
        return NULL;
    }
    struct mpu9250_sensor_data sensor_data;
    unsigned int ioctls[] = { MPU9250_IOCTL_READ_ACCEL, MPU9250_IOCTL_READ_GYRO, 
                              MPU9250_IOCTL_READ_MAG, MPU9250_IOCTL_READ_DMP };
    int thread_id = *(int *)arg % MAX_THREADS;

    while (pool->running) {
        sem_wait(&pool->philos_sem[thread_id]);
        sem_wait(&pool->philos_sem[(thread_id + 1) % 5]);
        if (ioctl(fd, ioctls[thread_id], &sensor_data) < 0) {
            fprintf(stderr, "Map thread %d read failed\n", thread_id);
            sem_post(&pool->philos_sem[thread_id]);
            sem_post(&pool->philos_sem[(thread_id + 1) % 5]);
            continue;
        }
        pthread_mutex_lock(&pool->buffer_mutex);
        if (pool->buffer_count < BUFFER_SIZE) {
            struct mpu9250_mq_data *data = &pool->buffer[pool->buffer_count++];
            if (thread_id == 0) memcpy(data->accel, sensor_data.values, sizeof(float)*3);
            else if (thread_id == 1) memcpy(data->gyro, sensor_data.values, sizeof(float)*3);
            else if (thread_id == 2) memcpy(data->mag, sensor_data.values, sizeof(float)*3);
            else if (thread_id == 3) memcpy(data->quat, sensor_data.values, sizeof(float)*4);
            pthread_cond_signal(&pool->buffer_cond);
        }
        pthread_mutex_unlock(&pool->buffer_mutex);
        sem_post(&pool->philos_sem[thread_id]);
        sem_post(&pool->philos_sem[(thread_id + 1) % 5]);
        usleep(10000);
    }
    close(fd);
    return NULL;
}

static void *reduce_thread(void *arg) {
    struct map_reduce_pool *pool = (struct map_reduce_pool *)arg;
    struct mpu9250_mq_data aggregate = {0};
    int count = 0;

    while (pool->running) {
        pthread_mutex_lock(&pool->buffer_mutex);
        while (pool->buffer_count == 0 && pool->running)
            pthread_cond_wait(&pool->buffer_cond, &pool->buffer_mutex);
        if (!pool->running) {
            pthread_mutex_unlock(&pool->buffer_mutex);
            break;
        }
        for (int i = 0; i < pool->buffer_count; i++) {
            for (int j = 0; j < 3; j++) {
                aggregate.accel[j] += pool->buffer[i].accel[j];
                aggregate.gyro[j] += pool->buffer[i].gyro[j];
                aggregate.mag[j] += pool->buffer[i].mag[j];
            }
            for (int j = 0; j < 4; j++) aggregate.quat[j] += pool->buffer[i].quat[j];
            count++;
        }
        pool->buffer_count = 0;
        pthread_mutex_unlock(&pool->buffer_mutex);
        if (count > 0) {
            for (int j = 0; j < 3; j++) {
                aggregate.accel[j] /= count;
                aggregate.gyro[j] /= count;
                aggregate.mag[j] /= count;
            }
            for (int j = 0; j < 4; j++) aggregate.quat[j] /= count;
            data_ready_callback(&aggregate);
            count = 0;
            memset(&aggregate, 0, sizeof(aggregate));
        }
        usleep(50000);
    }
    return NULL;
}

int main(int argc, char **argv) {
    struct map_reduce_pool pool = {0};
    pool.running = 1;
    pthread_mutex_init(&pool.buffer_mutex, NULL);
    pthread_cond_init(&pool.buffer_cond, NULL);
    for (int i = 0; i < 5; i++) sem_init(&pool.philos_sem[i], 0, 1);

    for (int i = 0; i < MAX_THREADS; i++)
        pthread_create(&pool.map_threads[i], NULL, map_thread, &pool);
    pthread_create(&pool.reduce_thread, NULL, reduce_thread, &pool);

    sleep(10);
    pool.running = 0;
    pthread_cond_broadcast(&pool.buffer_cond);
    for (int i = 0; i < MAX_THREADS; i++) pthread_join(pool.map_threads[i], NULL);
    pthread_join(pool.reduce_thread, NULL);

    pthread_mutex_destroy(&pool.buffer_mutex);
    pthread_cond_destroy(&pool.buffer_cond);
    for (int i = 0; i < 5; i++) sem_destroy(&pool.philos_sem[i]);
    return 0;
}