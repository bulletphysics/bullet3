/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///USE_LIBSPE2: this define should be in the build system, or in <LinearMath/btScalar.h>
//#define USE_LIBSPE2 1
#ifdef USE_LIBSPE2

#ifndef SPU_LIBSPE2_SUPPORT_H
#define SPU_LIBSPE2_SUPPORT_H

#include "LinearMath/btAlignedObjectArray.h"
#include <libspe2.h>
#include <pthread.h>


/**
 * Note:
 * The order of elements in this enum are important, that's why each one is explicitly
 * given a value.  They will correspond to the .elf names/addresses that will be
 * loaded into Libspe2.
 * Mixing up these values will cause the wrong code to execute, for instance, the
 * solver may be asked to do a collision detection job.
 */
#ifdef WIN32 // original enum, but for libspe2 will pass the SPE program handle ptr directly
typedef enum {
	SPU_ELF_COLLISION_DETECTION=0,
	SPU_ELF_SAMPLE,
//SPU_ELF_INTEGRATION,
//SPU_ELF_SOLVER,
	SPU_ELF_LAST,
} SpuLibspe2ElfId_t;
#endif
#ifdef WIN32
#include <malloc.h>
#define memalign(alignment, size) malloc(size);
#else
#include <stdlib.h>
#endif // WIN32

#define MAXSPUS 16
typedef struct ppu_pthread_data {
spe_context_ptr_t context;
pthread_t pthread;
unsigned int entry;
unsigned int flags;
void *argp;
void *envp;
spe_stop_info_t stopinfo;
} ppu_pthread_data_t;

void *ppu_pthread_function(void *arg)
{
    ppu_pthread_data_t datap = *(ppu_pthread_data_t *)arg;
    int rc;
    do {
        rc = spe_context_run(datap->context, &datap->entry, datap->flags, datap->argp, datap->envp, &datap->stopinfo);
    } while (rc > 0); // loop until exit or error, and while any stop & signal
    pthread_exit(NULL);
}



#include <LinearMath/btScalar.h> //for uint32_t etc.

///placeholder, until libspe2 support is there
struct	btSpuStatus
{
	uint32_t m_taskId;
	uint32_t	m_commandId;
	uint32_t m_status;

	struct SpuGatherAndProcessPairsTaskDesc* m_taskDesc;

	void*	m_threadHandle;
	void*	m_lsMemory;

	void*	m_eventStartHandle;
	char	m_eventStartHandleName[32];

	void*	m_eventCompletetHandle;
	char	m_eventCompletetHandleName[32];
	

};

///SpuLibspe2Support helps to initialize/shutdown libspe2, start/stop SPU tasks and communication
class SpuLibspe2Support {

	btAlignedObjectArray<btSpuStatus>	m_activeSpuStatus;

public:
	///Setup and initialize SPU/CELL/Libspe2
	SpuLibspe2Support(spe_program_handle_t *speprog,int numThreads);
	// SPE program handle ptr.
	spe_program_handle_t *program;
	// SPE program data
	ppu_pthread_data_t data[MAX_SPUS];
	// num SPE Threads
	unsigned int N;
	///cleanup/shutdown Libspe2
	~SpuLibspe2Support();

	///send messages to SPUs
	void sendRequest(uint32_t uiCommand, uint32_t uiArgument0, uint32_t uiArgument1=0);

	///check for messages from SPUs
	void waitForResponse(unsigned int *puiArgument0, unsigned int *puiArgument1);

	///start the spus (can be called at the beginning of each frame, to make sure that the right SPU program is loaded)
	void startSPUs(int numThreads);

	///tell the task scheduler we are done with the SPU tasks
	void stopSPUs();

};

#endif //SPU_LIBSPE2_SUPPORT_H

#endif //USE_LIBSPE2
