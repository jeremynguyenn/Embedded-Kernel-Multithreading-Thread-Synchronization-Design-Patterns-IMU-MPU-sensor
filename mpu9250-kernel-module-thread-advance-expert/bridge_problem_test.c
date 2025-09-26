#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>

#define MAX_CARS 10
#define MAX_THREADS 4

struct bridge_monitor {
    pthread_mutex_t lock;
    pthread_cond_t cond;
    int cars_on_bridge;
    int direction; // 0: left-to-right, 1: right-to-left, -1: empty
    sem_t philos_sem[5]; // Dining philosophers for resource sync
    volatile int running;
};

static void *car_thread(void *arg) {
    struct bridge_monitor *monitor = (struct bridge_monitor *)arg;
    int car_id = *(int *)arg % MAX_CARS;
    int my_direction = car_id % 2; // Even: left-to-right, Odd: right-to-left

    while (monitor->running) {
        sem_wait(&monitor->philos_sem[car_id % 5]);
        sem_wait(&monitor->philos_sem[(car_id + 1) % 5]);
        pthread_mutex_lock(&monitor->lock);
        while (monitor->cars_on_bridge > 0 && monitor->direction != my_direction && monitor->direction != -1)
            pthread_cond_wait(&monitor->cond, &monitor->lock);
        monitor->cars_on_bridge++;
        if (monitor->direction == -1) monitor->direction = my_direction;
        printf("Car %d (dir %d) on bridge, total: %d\n", car_id, my_direction, monitor->cars_on_bridge);
        pthread_mutex_unlock(&monitor->lock);
        sem_post(&monitor->philos_sem[car_id % 5]);
        sem_post(&monitor->philos_sem[(car_id + 1) % 5]);

        usleep(100000); // Simulate crossing bridge

        pthread_mutex_lock(&monitor->lock);
        monitor->cars_on_bridge--;
        if (monitor->cars_on_bridge == 0) monitor->direction = -1;
        printf("Car %d left bridge, total: %d\n", car_id, monitor->cars_on_bridge);
        pthread_cond_broadcast(&monitor->cond);
        pthread_mutex_unlock(&monitor->lock);
        usleep(50000);
    }
    return NULL;
}

int main(int argc, char **argv) {
    struct bridge_monitor monitor = {0};
    pthread_t threads[MAX_THREADS];
    monitor.running = 1;
    pthread_mutex_init(&monitor.lock, NULL);
    pthread_cond_init(&monitor.cond, NULL);
    for (int i = 0; i < 5; i++) sem_init(&monitor.philos_sem[i], 0, 1);

    for (int i = 0; i < MAX_THREADS; i++)
        pthread_create(&threads[i], NULL, car_thread, &monitor);

    sleep(10);
    monitor.running = 0;
    pthread_cond_broadcast(&monitor.cond);
    for (int i = 0; i < MAX_THREADS; i++) pthread_join(threads[i], NULL);

    pthread_mutex_destroy(&monitor.lock);
    pthread_cond_destroy(&monitor.cond);
    for (int i = 0; i < 5; i++) sem_destroy(&monitor.philos_sem[i]);
    return 0;
}