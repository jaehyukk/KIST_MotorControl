#include "realtime.h"

#define SEC_IN_NSEC 1000000000

void pthread_create_rt(pthread_t *thread, pthread_attr_t *__attr, sched_param *__param, int priority, void *(*__start_routine)(void *), void *__restrict __arg)
{
    int ret = pthread_attr_init(__attr);
    if (ret)
    {
        printf("init pthread_attributes failed");
    }

    ret = pthread_attr_setstacksize(__attr, PTHREAD_STACK_MIN);

    if (ret)
    {
        printf("pthread setstacksize failed");
    }
    ret = pthread_attr_setschedpolicy(__attr, SCHED_FIFO);

    if (ret)
    {
        printf("pthread setschedpolicy failed");
    }
    __param->sched_priority = priority;

    ret = pthread_attr_setschedparam(__attr, __param);

    if (ret)
    {
        printf("pthread setschedparam failed");
    }

    ret = pthread_attr_setinheritsched(__attr, PTHREAD_EXPLICIT_SCHED);
    if (ret)
    {
        printf("pthread setinheritsched failed\n");
    }

    ret = pthread_create(thread, __attr, __start_routine, __arg);
    if (ret)
    {
        printf("create pthread 1 failed\n");
    }
}





void add_ns(timespec *__ts, int __ns)
{
    __ts->tv_nsec += __ns;

    if (__ts->tv_nsec >= SEC_IN_NSEC)
    {
        __ts->tv_sec++;

        __ts->tv_nsec -= SEC_IN_NSEC;
    }
}

void sync_period(timespec *__ts, int __ns)
{
    add_ns(__ts, __ns);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, __ts, NULL);
}
