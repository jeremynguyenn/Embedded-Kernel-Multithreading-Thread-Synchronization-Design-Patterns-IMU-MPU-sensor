#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include "mpu9250.h"

#define NUM_STAGES 4
#define BUFFER_SIZE 10

struct assembly_line {
    pthread_t threads[NUM_STAGES];
    struct mpu9250_mq_data buffers[NUM_STAGES][BUFFER_SIZE];
    int buffer_counts[NUM_STAGES];
    pthread_mutex_t mutexes[NUM_STAGES];
    pthread_cond_t conds[NUM_STAGES];
    sem_t philos_sem[5];
    volatile int running;
};

static void *stage_thread(void *arg) {
    struct assembly_line *line = (struct assembly_line *)arg;
    int stage = *(int *)arg % NUM_STAGES;
    int fd = open("/dev/mpu9250", O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Open /dev/mpu9250 failed\n");
        return NULL;
    }
    unsigned int ioctls[] = { MPU9250_IOCTL_READ_ACCEL, MPU9250_IOCTL_READ_GYRO, 
                              MPU9250_IOCTL_READ_MAG, MPU9250_IOCTL_READ_DMP };

    while (line->running) {
        sem_wait(&line->philos_sem[stage]);
        sem_wait(&line->philos_sem[(stage + 1) % 5]);
        struct mpu9250_mq_data data = {0};
        if (stage == 0) {
            struct mpu9250_sensor_data sensor_data;
            if (ioctl(fd, ioctls[stage], &sensor_data) < 0) {
                fprintf(stderr, "Stage %d read failed\n", stage);
                sem_post(&line->philos_sem[stage]);
                sem_post(&line->philos_sem[(stage + 1) % 5]);
                continue;
            }
            memcpy(data.accel, sensor_data.values, sizeof(float)*3);
        } else {
            pthread_mutex_lock(&line->mutexes[stage - 1]);
            while (line->buffer_counts[stage - 1] == 0 && line->running)
                pthread_cond_wait(&line->conds[stage - 1], &line->mutexes[stage - 1]);
            if (!line->running) {
                pthread_mutex_unlock(&line->mutexes[stage - 1]);
                sem_post(&line->philos_sem[stage]);
                sem_post(&line->philos_sem[(stage + 1) % 5]);
                break;
            }
            data = line->buffers[stage - 1][--line->buffer_counts[stage - 1]];
            pthread_cond_signal(&line->conds[stage - 1]);
            pthread_mutex_unlock(&line->mutexes[stage - 1]);
        }
        pthread_mutex_lock(&line->mutexes[stage]);
        if (line->buffer_counts[stage] < BUFFER_SIZE) {
            line->buffers[stage][line->buffer_counts[stage]++] = data;
            pthread_cond_signal(&line->conds[stage]);
            if (stage == NUM_STAGES - 1) {
                printf("Final: Accel: %.2f %.2f %.2f\n", data.accel[0], data.accel[1], data.accel[2]);
            }
        }
        pthread_mutex_unlock(&line->mutexes[stage]);
        sem_post(&line->philos_sem[stage]);
        sem_post(&line->philos_sem[(stage + 1) % 5]);
        usleep(10000);
    }
    close(fd);
    return NULL;
}

int main(int argc, char **argv) {
    struct assembly_line line = {0};
    line.running = 1;
    for (int i = 0; i < NUM_STAGES; i++) {
        pthread_mutex_init(&line.mutexes[i], NULL);
        pthread_cond_init(&line.conds[i], NULL);
    }
    for (int i = 0; i < 5; i++) sem_init(&line.philos_sem[i], 0, 1);

    for (int i = 0; i < NUM_STAGES; i++)
        pthread_create(&line.threads[i], NULL, stage_thread, &line);

    sleep(10);
    line.running = 0;
    for (int i = 0; i < NUM_STAGES; i++) {
        pthread_cond_broadcast(&line.conds[i]);
        pthread_join(line.threads[i], NULL);
        pthread_mutex_destroy(&line.mutexes[i]);
        pthread_cond_destroy(&line.conds[i]);
    }
    for (int i = 0; i < 5; i++) sem_destroy(&line.philos_sem[i]);
    return 0;
}