#ifndef _shme_
#define _shme_

#ifdef __cplusplus
extern "C"
{

#endif

class shmdata{

shmdata();
~shmdata();


public:
    //int shm_cnt;
    int shm_cnt = 0;
    int32_t shm_mtpos;
    int32_t shm_mtvel;
    int32_t shm_torq;
    double shm_roll;
    double shm_pitch;
    double shm_yaw;
    
};

void *shmwork(shmdata *context);
    //void * shmwork_helper(void *ptr);













#ifdef __cplusplus
}
#endif

#endif
