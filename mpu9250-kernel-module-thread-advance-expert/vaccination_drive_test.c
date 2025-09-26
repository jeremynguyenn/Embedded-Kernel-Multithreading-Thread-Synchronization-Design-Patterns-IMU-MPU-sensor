#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>

#define MAX_PEOPLE 10
#define MAX_STATIONS 2

struct vaccination_monitor {
    pthread_mutex_t lock;
    pthread_cond_t cond;
    int stations_available;
    sem_t philos_sem[5];
    volatile int running;
};

static void *person_thread(void *arg) {
    struct vaccination_monitor *monitor = (struct vaccination_monitor *)arg;
    int person_id = *(int *)arg % MAX_PEOPLE;

    while (monitor->running) {
        sem_wait(&monitor->philos_sem[person_id % 5]);
        sem_wait(&monitor->philos_sem[(person_id + 1) % 5]);
        pthread_mutex_lock(&monitor->lock);
        while (monitor->stations_available == 0 && monitor->running)
            pthread_cond_wait(&monitor->cond, &monitor->lock);
        if (!monitor->running) {
            pthread_mutex_unlock(&monitor->lock);
            sem_post(&monitor->philos_sem[person_id % 5]);
            sem_post(&monitor->philos_sem[(person_id + 1) % 5]);
            break;
        }
        monitor->stations_available--;
        printf("Person %d is being vaccinated, stations left: %d\n", person_id, monitor->stations_available);
        pthread_mutex_unlock(&monitor->lock);
        sem_post(&monitor->philos_sem[person_id % 5]);
        sem_post(&monitor->philos_sem[(person_id + 1) % 5]);

        usleep(200000); // Simulate vaccination time

        pthread_mutex_lock(&monitor->lock);
        monitor->stations_available++;
        printf("Person %d finished vaccination, stations left: %d\n", person_id, monitor->stations_available);
        pthread_cond_signal(&monitor->cond);
        pthread_mutex_unlock(&monitor->lock);
        usleep(50000);
    }
    return NULL;
}

int main(int argc, char **argv) {
    struct vaccination_monitor monitor = {0};
    pthread_t threads[MAX_PEOPLE];
    monitor.stations_available = MAX_STATIONS;
    monitor.running = 1;
    pthread_mutex_init(&monitor.lock, NULL);
    pthread_cond_init(&monitor.cond, NULL);
    for (int i = 0; i < 5; i++) sem_init(&monitor.philos_sem[i], 0, 1);

    for (int i = 0; i < MAX_PEOPLE; i++)
        pthread_create(&threads[i], NULL, person_thread, &monitor);

    sleep(10);
    monitor.running = 0;
    pthread_cond_broadcast(&monitor.cond);
    for (int i = 0; i < MAX_PEOPLE; i++) pthread_join(threads[i], NULL);

    pthread_mutex_destroy(&monitor.lock);
    pthread_cond_destroy(&monitor.cond);
    for (int i = 0; i < 5; i++) sem_destroy(&monitor.philos_sem[i]);
    return 0;
}