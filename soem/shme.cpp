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
#include "shme.h"


// 생성되거나 접근할 POSIX 공유 메모리 세그먼트의 이름을 정의
#define		NAME_POSIX_SHM		"/mmapfile123"
// 공유 메모리 세그먼트의 크기를 지정
#define		SZ_SHM_SEGMENT		4096




// 메소드 정의 안 되어 있음
shmdata::shmdata(){
}







// 공유 메모리 세그먼트(mmapfile123)의 생성 및 접근 처리
void * shmwork(shmdata *context) {
    int shm_fd;
	

	printf("* SHM Name : %s\n", NAME_POSIX_SHM);

	//if((shm_fd = shm_open(NAME_POSIX_SHM, O_RDWR|O_CREAT|O_EXCL, 0660)) > 0) { //shm_open :shm을 생성하거나 생성된 shm을 open
	if((shm_fd = shm_open(NAME_POSIX_SHM, O_RDWR|O_CREAT|O_EXCL, 0660)) > 0) {	
        
        //printf("* create SHM : /dev/shm/%s\n", NAME_POSIX_SHM);
		
        if(ftruncate(shm_fd, sizeof(shmdata)) == -1)//ftruncate :사이즈 할당
			exit(EXIT_FAILURE);

	}
	else {
		if(errno != EEXIST)
			exit(EXIT_FAILURE);

		if((shm_fd = shm_open(NAME_POSIX_SHM, O_RDWR, 0)) == -1)
			exit(EXIT_FAILURE);
	}

	// 공유 메모리 세그먼트(mmapfile123)의 크기 설정
    ftruncate(shm_fd, sizeof(shmdata));
	
	// mmap : 공유 메모리 세그먼트(mmapfile123)를 프로세스 주소 공간에 매핑
	// 성공하면 공유 메모리 데이터에 액세스 가능
	context = (shmdata*)mmap(NULL, sizeof(shmdata), PROT_READ|PROT_WRITE, MAP_SHARED, shm_fd, 0); 


    if(context == MAP_FAILED)
	 	exit(EXIT_FAILURE);

    for(int i=0;i<10000;i++){
   
   

      printf("print position : %d\n",context->shm_mtpos);
      printf("print velocity : %d\n",context->shm_mtvel);
      printf("print torque : %d\n",context->shm_torq);
      }



	// 공유 메모리 언맵
	munmap(&context, sizeof(shmdata));
	close(shm_fd);
	rt_printf("* would you remove shm (name : %s)(y/n)\n", NAME_POSIX_SHM);
	// 공유 메모리 삭제
	shm_unlink(NAME_POSIX_SHM);
   

	return 0;
}


// }
