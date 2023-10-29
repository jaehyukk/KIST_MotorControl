#ifndef _ethc_
#define _ethc_

#ifdef __cplusplus
extern "C"
{



#include<alchemy/task.h>
#include<alchemy/timer.h>
//#include<mutex>
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

    
using namespace std;



//char IOmap[4096];
//pthread_t thread1 ,rtthread;

// rx, tx setting




#endif
class CEtherCat{
public:
CEtherCat();

//shmdata *shm_motor;

int connect(char *ifname);

int RXPdoassi(uint16 Slave, uint8 SubIndex, boolean CA, void *p, int Timeout);
int TXPdoassi(uint16 Slave, uint8 SubIndex, boolean CA, void *p, int Timeout);
void mappingpdo(int slavenum);
void mappingpdo1(uint16 map_1c12[],uint16 map_1c13[]);


void* operatecyclictask();
void* operatecyclictask2();


static void* operatecyclictask_helper(void *context);
static void* operatecyclictask_helper2(void *context);


void masterState();
void slaveState();
};

bool controlWordGenerate(const uint16_t statusWord, uint16_t &controlWord);
void *ecatcheck1( void *ptr );

//void *shmwork(void *ptr);

#ifdef __cplusplus
}
#endif

#endif
