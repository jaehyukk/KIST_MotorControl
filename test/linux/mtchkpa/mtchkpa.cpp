#define T_FPU 0

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/mman.h>
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <thread>
#include <pthread.h>
#include <math.h>

#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <mutex.h>
#include <copperplate/clockobj.h>
#include <copperplate/registry.h>

#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"
#include "slaveinfo.h"
#include "ethc.h"
#include "realtime.h"
#include "shme.h"

#include <time.h>
#include <signal.h>

uint8 currentgroup = 0;

void operatecyclictask_helper(void *context)
{
    printf("suc link the class therad\n");
    ((CEtherCat *)context)->operatecyclictask();
}

void operatecyclictask_helper2(void *context){
    printf("suc link the class therad2\n");
    ((CEtherCat *)context)->operatecyclictask2();
}

int main(int argc, char *argv[])
{
    mlockall(MCL_CURRENT | MCL_FUTURE);
    int iret1, iret2, iret3, ret, ret2;
    uint16 map_1c12[3] = {0x0002, 0x1605};
    uint16 map_1c13[5] = {0x0004, 0x1a00, 0x1a11, 0x1a13, 0x1a1e};
    CEtherCat ethc;
    RT_TASK rtthread1, rtthread2;

    // setting for rt///////////////////////////////////////////
    //  pthread_attr_t thread1_t;
    //  sched_param param1_;
    //  pthread_attr_init(&thread1_t);
    //  param1_.sched_priority = 99;
    //  pthread_attr_setschedparam(&thread1_t, &param1_);
    //  pthread_attr_setschedpolicy(&thread1_t, SCHED_FIFO);
    //  pthread_attr_setinheritsched(&thread1_t, PTHREAD_EXPLICIT_SCHED);
    ////////////////////////////////////////////////////////////

    ethc.connect(argv[1]);

    printf("is it working& ec_slavecout:%d?\n", ec_slavecount);

    for (int j = 1; j < ec_slavecount + 1; j++)
    {
        if (ec_slave[j].Obits != 0)
        {
            printf("start for llop\n");
            ethc.mappingpdo(j);
            printf("end for llop%d\n", j);
        }
    }
    printf("slidnf\n");

    // Create and start the real-time thread1
    ret = rt_task_create(&rtthread1, "rt_thread1", 0, 99, T_JOINABLE);
    if (ret == 0)
    {
        ret = rt_task_start(&rtthread1, &operatecyclictask_helper, &ethc);
        if (ret != 0)
        {
            printf("Failed to start real-time thread1\n");
            rt_task_delete(&rtthread1);
            return ret;
        }
    }
    else
    {
        printf("Failed to create real-time thread1\n");
        return ret;
    }
    

    // Create and start the real-time thread2
    ret2 = rt_task_create(&rtthread2, "rt_thread2", 0, 90, T_JOINABLE);
    if (ret2 == 0)
    {
        ret2 = rt_task_start(&rtthread2, &operatecyclictask_helper2, &ethc);
        if (ret2 != 0)
        {
            printf("Failed to start real-time thread2\n");
            rt_task_delete(&rtthread2);
            return ret2;
        }
    }
    else
    {
        printf("Failed to create real-time thread2\n");
        return ret2;
    }

    // Wait for the user to terminate the program
    pause();

    rt_task_join(&rtthread1);
    rt_task_join(&rtthread2);

    rt_task_delete(&rtthread1);
    rt_task_delete(&rtthread2);

    return 0;
}