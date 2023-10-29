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
#include "shme.h"
#include "ethercat.h"

#include <fstream>
#include <time.h>
#include <signal.h>
#include <vector>
#include <tuple>

using namespace std;

#define EC_TIMEOUTMON 500
#define JDOF 1 // before = 4
#define MAX_TORQUE 1000
#define ELMO_DOF 33
#define PERIOD_NS 1000000
#define INITIAL_POS 0
#define SEC_IN_NSEC 1000000000

#define NAME_POSIX_SHM "/mmapfile123"
#define SZ_SHM_SEGMENT 4096

#define COUNT2Rad (500 * M_PI) / 10000 // different from Motor. 10000 is 2 PI in this motor
#define Rad2COUNT 10000 / (500 * M_PI)
#define Continuous_Current 7.03 // Amp

int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup1 = 0;
int cnt_err1 = 0;
RTIME previous1, previous2, previous3;
struct timespec tr, ts, ts1;

std::ofstream timetest, timetest2;
double execution_time_us, execution_time_us2;
std::chrono::steady_clock::time_point st_start_time, start_time1, start_time2, end_time1, end_time2;
vector<tuple<double, double, double>> exe_value;

// boolean printMAP = TRUE;

// class shmdata{

// shmdata();
// ~shmdata();

// public:
//     int shm_cnt;
//     int32_t shm_mtpos;
//     int32_t shm_mtvel;
//     int32_t shm_torq;
//     double shm_roll;
//     double shm_pitch;
//     double shm_yaw;

// };

namespace EtherCAT_Elmo
{
    enum MODE_OF_OPERATION
    {
        ProfilePositionmode = 1,
        ProfileVelocitymode = 3,
        ProfileTorquemode = 4,
        Homingmode = 6,
        InterpolatedPositionmode = 7,
        CyclicSynchronousPositionmode = 8,
        CyclicSynchronousVelocitymode = 9,
        CyclicSynchronousTorquemode = 10,
        CyclicSy = 11
    };

    struct ElmoGoldDevice
    {
        struct elmo_gold_tx
        {
            int32_t targetPosition;
            int32_t targetVelocity;
            int16_t targetTorque;
            uint16_t maxTorque;
            uint16_t controlWord;
            int8_t modeOfOperation;
        };
        struct elmo_gold_rx
        {
            int32_t positionActualValue; // this one !!!!!
            uint32_t hommingSensor;
            uint16_t statusWord;
            int32_t velocityActualValue;
            int16_t torqueActualValue;
            int32_t positionExternal;
        };
    };
} // namespace EtherCAT_Elmo

EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_rx *rxPDO[JDOF];
EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_tx *txPDO[JDOF];

int ElmoSafteyMode[JDOF];
double control_time_real_;

bool hommingElmo[JDOF];
int stateElmo[JDOF];
int ElmoMode[ELMO_DOF];

double positionElmo, velocityElmo, torqueElmo, positionExternalElmo, torqueElmocheck, positionActualValue;
double ELMO_torque[JDOF];

const int FAULT_BIT = 3;
const int OPERATION_ENABLE_BIT = 2;
const int SWITCHED_ON_BIT = 1;
const int READY_TO_SWITCH_ON_BIT = 0;
enum
{
    CW_SHUTDOWN = 6,
    CW_SWITCHON = 7,
    CW_ENABLEOP = 15,
    CW_DISABLEOP = 7,
};

enum
{
    EM_POSITION = 11,
    EM_TORQUE = 22,
    EM_DEFAULT = 33,
    EM_COMMUTATION = 44,
};

bool ecat_number_ok = false;
bool ecat_WKC_ok = false;
bool de_shutdown = false;

// bool reachedInitial[JDOF] = {false};//JODF 확인

int tistp;

CEtherCat::CEtherCat()
{
}

int CEtherCat::connect(char *ifname)
{
    std::cout << "ethercatThread Start" << std::endl;
    char IOmap[4096]; //// 중요

    bool exit_middle = false;
    rt_printf("Usage: simple_test ifname1\nifname = enp89s0 for example\n you can see by 'ifconfig' \n");

    // 슬레이브를 찾았는가?
    if (ec_init(ifname)) // master를 초기화 하고 소켓을 Ifname에 바인딩
    {
        rt_printf("ec_init on %s succeeded.\n", ifname);

        // 슬레이브가 사용 가능한 상태인가?
        if (ec_config_init(FALSE) > 0) // slave를 찾고 찾은 slave에 PRE-OP상태 요청
        {
            rt_printf("%d slaves found and configured.\n", ec_slavecount); // print slave_count

            // 자유도(Joint Degree of Free)가 1개인가?
            if (ec_slavecount == JDOF)
            {
                ecat_number_ok = true; // 여기로 빠짐
            }
            else
            {
                std::cout << "WARNING : SLAVE NUMBER INSUFFICIENT" << std::endl;
            }

            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                // 슬레이브 통신이 EtherCAT 방식으로 변환할 수 있는가?(CAN -> EtherCAT)
                if (!(ec_slave[slave].CoEdetails & ECT_COEDET_SDOCA))
                {
                    rt_printf("ELMO : slave[%d] CA? : false , shutdown request \n ", slave);
                    exit_middle = true;
                }
                ec_slave[slave].CoEdetails ^= ECT_COEDET_SDOCA;
            }

            // 슬레이브 상태 체크
            ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);

            // 데이터 주고받을 map 구성
            // Position, Velocity, Torque, word, Modes of Operation을 받겠다.
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                // 0x1605 :  Target Position             32bit
                //           Target Velocity             32bit
                //           Target Torque               16bit
                //           Max Torque                  16bit
                //           Control word                16bit
                //           Modes of Operation          16bit
                // 0x607A : Target Position
                // uint16 map_1c12[1] = {0x1605};
                // uint16 map_1c12[1] = {0x1605};
                // uint16 map_1c12[2] = {0x607A,0x6040};
                // uint16 map_1c12[2] = {0x6040, 0x6072};
                uint16 map_1c12[2] = {0x0001, 0x1605};

                // 0x1a00 :  position actual value       32bit
                //           Digital Inputs              32bit
                //           Status word                 16bit
                // 0x6064 :  position actual value       32bit
                // 0x6041 :  Status word                 16bit
                // 0x60FD :  Digital Inputs              32bit
                // 0x1a11 :  velocity actual value       32bit
                // 0x606c :  velocity actual value       32bit
                // 0x1a12 :  Torque demand Value         16bit
                // 0x1a13 :  Torque actual value         16bit
                // 0x6077 :  Torque actual value         16bit
                // 0x1a1e :  Auxiliary position value    32bit

                uint16 map_1c13[5] = {0x0004, 0x1a00, 0x1a11, 0x1a13, 0x1a1e}; //, 0x1a12};
                // uint16 map_1c13[4] = {0x1a00, 0x1a11, 0x1a13, 0x1a1e}; //, 0x1a12};
                // uint16 map_1c13[4] = {0x6041, 0x6064, 0x6077, 0x606C}; //, 0x1a12}; homming X
                // uint16 map_1c13[3] = {0x1a00 , 0x606C, 0x6077};
                // uint16 map_1c13[2] = {0x0001,0x6041}; // status word only
                // uint16 map_1c13[3] = {0x0002, 0x1a11, 0x1a00};
                // uint16 map_1c13[6] = {0x0005, 0x1a04, 0x1a11, 0x1a12, 0x1a1e, 0X1a1c};

                int os;
                os = sizeof(map_1c12);
                int retVal = 0;
                // wkc 값을 retval에 대입(PDO mapping 설정이 성공적으로 대입되었는지 확인)
                // mailbox 송수신 진행, 슬레이브 설정값 송수신 = 그냥 슬레이브 설정임
                // 성공했다면 retVal#은 1 반환
                retVal = RXPdoassi(1, 0, TRUE, &map_1c12, EC_TIMEOUTRXM);
                std::cout << "retVal1 : " << retVal << std::endl;
                os = sizeof(map_1c13);
                retVal = TXPdoassi(1, 0, TRUE, &map_1c13, EC_TIMEOUTRXM);
                std::cout << "retVal2 : " << retVal << std::endl;
            }

            ec_config_map(&IOmap); // 슬레이브를 순서대로 슬레이브에서 iomap으로 모든 pdo 매핑   ->매핑이 완료되면 슬레이브에게 SAFE_OP로 들어가도록 요청

            ec_configdc(); // create distribution clock

            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4); // checking the slave state and if reached not yet wait until state be request state

            // expectedWKC 계산
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

            rt_printf("outputwkc :%d\n", ec_group[0].outputsWKC * 2);
            rt_printf("inputwkc :%d\n", ec_group[0].inputsWKC);
            rt_printf("expectedwkd :%d\n", expectedWKC);
            rt_printf("current wkc:%d\n", wkc);

            rt_printf("ELMO : Request operational state for all slaves. Calculated workcounter : %d\n", expectedWKC);

            exit_middle = 0;

            ec_slave[0].state = EC_STATE_OPERATIONAL;

            // data send&receive ontime
            // EtherCAT process data 송수신
            previous2 = rt_timer_read();
            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);
            rt_printf("processdata time for cycle:%dmicrosecond\n", (rt_timer_read() - previous2) / 1000);

            ec_writestate(0); // request OP state for all slaves

            rt_printf("%x it should be 8\n", ec_slave[0].state);

            // 슬레이브가 동작 준비 완료 상태인가?
            if (ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                if (ecat_number_ok && ecat_WKC_ok)
                {
                    std::cout << "All slaves Status OK" << std::endl;
                }
                else
                {
                    rt_printf("%d it should be 8\n", ec_slave[0].state);
                    std::cout << "Please Check Slave status" << std::endl;
                }
            }
            std::cout << wkc << "@" << expectedWKC << "@" << std::endl;
        }

        else
        {
            rt_printf("fail to connect\n");
            exit(0);
        }
    }

    else
    {
        rt_printf("fail to connect\n");
        exit(0);
    }

    rt_printf("connecting end \n");
    return true;
}

// boolean CA= FALSE = single subindex. TRUE = Complete Access, all subindexes written.
// uint8 = Subindex to write, must be 0 or 1 if CA is used.
int CEtherCat::RXPdoassi(uint16 Slave, uint8 SubIndex, boolean CA, void *p, int Timeout)
{
    int psize = sizeof(p);
    rt_printf("start rxpdoassi\n");
    return ec_SDOwrite(Slave, 0x1c12, SubIndex, CA, psize, p, Timeout);
}

// boolean CA= FALSE = single subindex. TRUE = Complete Access, all subindexes written.
// uint8 = Subindex to write, must be 0 or 1 if CA is used.
int CEtherCat::TXPdoassi(uint16 Slave, uint8 SubIndex, boolean CA, void *p, int Timeout)
{
    int psize = sizeof(p);
    rt_printf("start txpdoassi\n");
    return ec_SDOwrite(Slave, 0x1c13, SubIndex, CA, psize, p, Timeout);
}

void CEtherCat::mappingpdo(int slavenum)
{

    txPDO[slavenum - 1] = (EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_tx *)(ec_slave[slavenum].outputs);
    rxPDO[slavenum - 1] = (EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_rx *)(ec_slave[slavenum].inputs);
}

void CEtherCat::mappingpdo1(uint16 map_1c12[], uint16 map_1c13[])
{
    rt_printf("wow\n");
    for (int j = 1; j < ec_slavecount + 1; j++)
    {
        rt_printf("wow%d\n", j);
        if (ec_slave[j].Obits != 0)
        {

            rt_printf("start for llop%d\n", j);
            RXPdoassi(j, 0, TRUE, &map_1c12, EC_TIMEOUTRXM);
            TXPdoassi(j, 0, TRUE, &map_1c13, EC_TIMEOUTRXM);
            mappingpdo(j);
            rt_printf("end for llop%d\n", j);
        }
        else
        {
        }
    }
}

// thread2
void *CEtherCat::operatecyclictask2()
{
    shmdata *shm_motor;
    int shm_fd;

    // shmwork(shm_motor);

    timetest2.open("./timetest2.txt");

    rt_task_set_periodic(NULL, TM_NOW, 3000000);

    // shared memory open--------------------------------------------
    rt_printf("* SHM Name : %s\n", NAME_POSIX_SHM);

    // if((shm_fd = shm_open(NAME_POSIX_SHM, O_RDWR|O_CREAT|O_EXCL, 0660)) > 0) { //shm_open :shm을 생성하거나 생성된 shm을 open
    if ((shm_fd = shm_open(NAME_POSIX_SHM, O_RDWR | O_CREAT | O_EXCL, 0660)) > 0)
    {
        rt_printf("* create SHM : /dev/shm/%s\n", NAME_POSIX_SHM);
        // if (ftruncate(shm_fd, sizeof(shmdata)) == -1) // ftruncate :사이즈 할당
        //     exit(EXIT_FAILURE);
    }
    else
    {
        if (errno != EEXIST)
            exit(EXIT_FAILURE);

        if ((shm_fd = shm_open(NAME_POSIX_SHM, O_RDWR, 0)) == -1)
            exit(EXIT_FAILURE);
    }

    ftruncate(shm_fd, sizeof(shmdata));

    // mmap :shm매핑(memory를 process의 주소 공간에 매핑)
    shm_motor = (shmdata *)mmap(NULL, sizeof(shmdata), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);

    if (shm_motor == MAP_FAILED)
    {
        exit(EXIT_FAILURE);
    }

    while (true)
    {
        previous3 = rt_timer_read();

        int value;
        value = shm_motor->shm_cnt++;
        // rt_printf("@@@@@@@ %d\n", value);

        timetest2 << control_time_real_ << "\t";

        rt_task_wait_period(NULL);
        timetest2 << (rt_timer_read() - previous3) / 1000 << "\t";
        timetest2 << std::endl;
    }
    // rt_printf("@@@@@\n");
}

// thread1
void *CEtherCat::operatecyclictask()
{
    shmdata *shm_motor;
    int shm_fd;
    shmwork(shm_motor);

    // shared memory open--------------------------------------------
    rt_printf("* SHM Name : %s\n", NAME_POSIX_SHM);

    // if((shm_fd = shm_open(NAME_POSIX_SHM, O_RDWR|O_CREAT|O_EXCL, 0660)) > 0) { //shm_open :shm을 생성하거나 생성된 shm을 open
    if ((shm_fd = shm_open(NAME_POSIX_SHM, O_RDWR | O_CREAT | O_EXCL, 0660)) > 0)
    {
        rt_printf("* create SHM : /dev/shm/%s\n", NAME_POSIX_SHM);
        if (ftruncate(shm_fd, sizeof(shmdata)) == -1) // ftruncate :사이즈 할당
            exit(EXIT_FAILURE);
    }
    else
    {
        if (errno != EEXIST)
            exit(EXIT_FAILURE);

        if ((shm_fd = shm_open(NAME_POSIX_SHM, O_RDWR, 0)) == -1)
            exit(EXIT_FAILURE);
    }

    ftruncate(shm_fd, sizeof(shmdata));

    // mmap :shm매핑(memory를 process의 주소 공간에 매핑)
    shm_motor = (shmdata *)mmap(NULL, sizeof(shmdata), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);

    if (shm_motor == MAP_FAILED)
    {
        exit(EXIT_FAILURE);
    }

    // shared memory--------------------------------------------

    timetest.open("./timetest.txt");

    bool reachedInitial[JDOF] = {false};

    rt_printf("cyclictask start\n");

    rt_task_set_periodic(NULL, TM_NOW, 1000000);

    while (true)
    {
        st_start_time = std::chrono::steady_clock::now();

        while (!de_shutdown)
        {
            previous1 = rt_timer_read();

            // 480us ~ 500us
            ec_send_processdata();
            wkc = ec_receive_processdata(1);

            rt_printf("process data : %ldus\n", (rt_timer_read() - previous1) / 1000);
            timetest << (rt_timer_read() - previous1) / 1000 << "\t";

            control_time_real_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - st_start_time).count() / 1000000.0;

            // wkc 측정값이 계산값보다 작으면 -> 데이터 손실 위험이 있어 통신 안 함
            if (wkc >= expectedWKC)
            {
                start_time1 = std::chrono::steady_clock::now();

                for (int slave = 1; slave <= ec_slavecount; slave++)
                {
                    // 제어 신호 생성 및 전송
                    if (controlWordGenerate(rxPDO[slave - 1]->statusWord, txPDO[slave - 1]->controlWord))
                    {
                        reachedInitial[slave - 1] = true;
                    }
                }

                for (int slave = 1; slave <= ec_slavecount; slave++)
                {
                    if (reachedInitial[slave - 1]) // set target value
                    {
                        txPDO[slave - 1]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousVelocitymode;
                        txPDO[slave - 1]->targetVelocity = (int)10000;

                        // txPDO[slave-1]->modeOfOperation = EtherCAT_Elmo::CyclicSynchronousPositionmode;
                        // txPDO[slave-1]->targetPosition = (int) 300000;

                        velocityElmo =
                            (((int32_t)ec_slave[slave].inputs[10]) +
                             ((int32_t)ec_slave[slave].inputs[11] << 8) +
                             ((int32_t)ec_slave[slave].inputs[12] << 16) +
                             ((int32_t)ec_slave[slave].inputs[13] << 24));

                        torqueElmo =
                            (((int16_t)ec_slave[slave].inputs[14]) +
                             ((int16_t)ec_slave[slave].inputs[15] << 8));

                        torqueElmocheck = torqueElmo;

                        if (torqueElmocheck >= 30000)
                        {
                            torqueElmocheck = 65536 - torqueElmocheck;
                        }

                        txPDO[slave - 1]->maxTorque = (uint16)MAX_TORQUE; // originaly 1000

                        // if (cnt_err1 >= 15)
                        // {
                        //     printf("fatal erro occur\n")    ;
                        //     //break;
                        // }
                        // rt_printf("mt_velocity %0.4f\n", rxPDO[slave - 1]->velocityActualValue);
                        // rt_printf("mt_position %0.4f\n", shm_motor->shm_mtpos);
                        // rt_printf("mt_torque %0.4f\n", shm_motor->shm_torq);

                        // shm_motor->shm_mtvel = rxPDO[slave - 1]->velocityActualValue;
                        shm_motor->shm_mtvel = velocityElmo;
                        shm_motor->shm_mtpos = rxPDO[slave - 1]->positionActualValue;
                        // shm_motor->shm_torq = rxPDO[slave - 1]->torqueActualValue;
                        shm_motor->shm_torq = torqueElmocheck / MAX_TORQUE * Continuous_Current;

                        // rt_printf("mt_velocity %.4f\n", shm_motor->shm_mtvel);
                        // rt_printf("mt_position %.4f\n", shm_motor->shm_mtpos);
                        // rt_printf("mt_torque %.4f\n", shm_motor->shm_torq);
                    }
                }
                timetest << (rt_timer_read() - previous1) / 1000 << "\t";
                timetest << control_time_real_ << "\t";
            }

            rt_task_wait_period(NULL); // 한 주기가 끝날 때까지 대기

            double rt_execution_time = (rt_timer_read() - previous1) / 1000;

            timetest << rt_execution_time << "\t";
            timetest << std::endl;
        }
    }
    timetest.close();
}

void *CEtherCat::operatecyclictask_helper2(void *context)
{
    rt_printf("suc link the class therad\n");
    return ((CEtherCat *)context)->operatecyclictask2();
}

void *CEtherCat::operatecyclictask_helper(void *context)
{
    rt_printf("suc link the class therad\n");
    return ((CEtherCat *)context)->operatecyclictask();
}

void CEtherCat::masterState()
{
    rt_printf("master state:%x\n", ec_slave[0].state);

    // EC_STATE_NONE           = 0x00,
    // /** Init state*/
    // EC_STATE_INIT           = 0x01,
    // /** Pre-operational. */
    // EC_STATE_PRE_OP         = 0x02,
    // /** Boot state*/
    // EC_STATE_BOOT           = 0x03,
    // /** Safe-operational. */
    // EC_STATE_SAFE_OP        = 0x04,
    // /** Operational */
    // EC_STATE_OPERATIONAL    = 0x08,

    // /** Error or ACK error */
    // EC_STATE_ACK            = 0x10,
    // EC_STATE_ERROR          = 0x10
}

void CEtherCat::slaveState()
{
    for (int i = 1; i < ec_slavecount + 1; i++)
    {
        rt_printf("slave%d state:%x\n", i, ec_slave[i].state);
    }
}

// controlWordGenerate
// statusword : 536 -> 592 -> 561 -> 4663
// controlword : 32656 -> 128 -> 6 -> 15
bool controlWordGenerate(const uint16_t statusWord, uint16_t &controlWord)
{
    //
    // rt_printf("%d\n", statusWord);
    // rt_printf("%d\n", controlWord);
    if (!(statusWord & (1 << READY_TO_SWITCH_ON_BIT))) // 1, 동작 준비 상태인가?(초기화 및 설정 과정을 거쳤는가?)
    {
        if (!(statusWord & (1 << SWITCHED_ON_BIT))) // 2, 드라이버의 전원이 커져 동작 가능한 상태로 전환이 되었는가?
        {
            if (!(statusWord & (1 << OPERATION_ENABLE_BIT))) // 4, 드라이버가 정상적으로 동작하며 작동을 시작할 수 있는가?
            {
                if (statusWord & (1 << FAULT_BIT)) // 8, 고장 상태(오류 상태)인가?
                {
                    rt_printf("false1\n");
                    // std::cout << "false1" << std::endl;
                    controlWord = 0x80; // 128
                    cnt_err1++;
                    return false;
                    // exit(0);
                }
                else
                {
                    rt_printf("false2\n");
                    // std::cout << "false2" << std::endl;
                    controlWord = CW_SHUTDOWN; // 6
                    cnt_err1++;
                    return false;
                    // exit(0);
                }
            }
            else
            {
                rt_printf("true1\n");
                // std::cout << "true1" <<std::endl;
                controlWord = CW_SWITCHON;
                return true;
            }
        }

        else
        {
            rt_printf("true2\n");
            // std::cout << "true2" <<std::endl;
            controlWord = CW_ENABLEOP;
            return true;
        }
    }
    else
    {
        rt_printf("true3\n");
        rt_printf("Motor Control...\n");
        // std::cout << "true3" <<std::endl;
        // std::cout << "Motor Control..."<<std::endl;
        controlWord = CW_ENABLEOP; // 15
        return true;
    }
    rt_printf("false3\n");
    // std::cout << "false3" << std::endl;
    controlWord = 0;
    cnt_err1++;
    return false;
}

/* create thread to handle slave error handling in OP */
void *ecatcheck1(void *ptr)
{
    int slave;

    while (1)
    {
        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup1].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                rt_printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup1].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup1) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup1].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        rt_printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        rt_printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state > 0)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            rt_printf("MESSAGE : slave %d reconfigured\n", slave);
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (!ec_slave[slave].state)
                        {
                            ec_slave[slave].islost = TRUE;
                            rt_printf("ERROR : slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (!ec_slave[slave].state)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            rt_printf("MESSAGE : slave %d recovered\n", slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        rt_printf("MESSAGE : slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup1].docheckstate)
                rt_printf(".");
        }
        usleep(250);
    }
}
