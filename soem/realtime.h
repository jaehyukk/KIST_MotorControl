#include <limits.h>
#include <sys/mman.h>
#include <pthread.h>
#include <stdio.h>

/**
 * Start realtime thread with priority (1 ~ 99(max))
 *
 */
void pthread_create_rt(pthread_t *thread, pthread_attr_t *__attr, sched_param *__param, int priority, void *(*__start_routine)(void *), void *__restrict __arg); // __THROWNL __nonnull ((1, 3));

void add_ns(timespec *__ts, int ns);

void sync_period(timespec *__ts, int ns);
