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
#include <sys/mman.h>

#include<alchemy/task.h>
#include<alchemy/timer.h>
#include<mutex.h>
#include<copperplate/clockobj.h>
#include<copperplate/registry.h>

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

uint8 currentgroup = 0;
//int expectedWKC;



int main(int argc, char *argv[]){
    mlockall(MCL_CURRENT|MCL_FUTURE);
    int iret1 ,iret2,iret3;     
    uint16 map_1c12[2] = {0x0001, 0x1605};
    uint16 map_1c13[5] = {0x0004, 0x1a00, 0x1a11, 0x1a13, 0x1a1e};
    CEtherCat ethc;
    pthread_t thread1 ,thread2,rtthread;
    

    //setting for rt///////////////////////////////////////////
    pthread_attr_t thread1_t;
    sched_param param1_;
    RT_TASK my_task;
    pthread_attr_init(&thread1_t);
    param1_.sched_priority = 95;
    pthread_attr_setschedparam(&thread1_t, &param1_);
    pthread_attr_setschedpolicy(&thread1_t, SCHED_FIFO);
    pthread_attr_setinheritsched(&thread1_t, PTHREAD_EXPLICIT_SCHED);
    ////////////////////////////////////////////////////////////
  
    ethc.connect(argv[1]);
    printf("is it working& ec_slavecout:%d?\n",ec_slavecount);
    iret1= pthread_create(&thread1,NULL, &ecatcheck1, (void*) &ctime); //thread for state chk
    
    for(int j=1;j<ec_slavecount+1;j++)
    {
        printf("start for llop\n");
        ethc.RXPdoassi(j,0,TRUE,&map_1c12,EC_TIMEOUTRXM);
        ethc.TXPdoassi(j,0,TRUE,&map_1c13,EC_TIMEOUTRXM);
        ethc.mappingpdo(j);
        printf("end for llop\n");
    }

  
    //iret2= pthread_create(&rtthread,NULL, &CEtherCat::operatecyclictask_helper, &ethc); //thread for state chk
    pthread_create_rt(&rtthread,&thread1_t,&param1_, 99,&CEtherCat::operatecyclictask_helper, &ethc);
    pthread_join(rtthread,NULL);

}







