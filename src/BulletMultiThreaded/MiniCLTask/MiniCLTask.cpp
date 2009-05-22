/*
Bullet Continuous Collision Detection and Physics Library, Copyright (c) 2007 Erwin Coumans

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

*/


#include "MiniCLTask.h"
#include "../PlatformDefinitions.h"
#include "../SpuFakeDma.h"
#include "LinearMath/btMinMax.h"
#include "BulletMultiThreaded/MiniCLTask/MiniCLTask.h"

#ifdef __SPU__
#include <spu_printf.h>
#else
#include <stdio.h>
#define spu_printf printf
#endif

#define __kernel
#define __global
#define get_global_id(a) guid

struct MiniCLTask_LocalStoreMemory
{
	
};


///////////////////////////////////////////////////
// OpenCL Kernel Function for element by element vector addition
__kernel void VectorAdd(__global const float8* a, __global const float8* b, __global float8* c, int guid)
{
    // get oct-float index into global data array
    int iGID = get_global_id(0);

    // read inputs into registers
    float8 f8InA = a[iGID];
    float8 f8InB = b[iGID];
    float8 f8Out = (float8)0.0f;
    
    // add the vector elements
    f8Out.s0 = f8InA.s0 + f8InB.s0;
    f8Out.s1 = f8InA.s1 + f8InB.s1;
    f8Out.s2 = f8InA.s2 + f8InB.s2;
    f8Out.s3 = f8InA.s3 + f8InB.s3;
    f8Out.s4 = f8InA.s4 + f8InB.s4;
    f8Out.s5 = f8InA.s5 + f8InB.s5;
    f8Out.s6 = f8InA.s6 + f8InB.s6;
    f8Out.s7 = f8InA.s7 + f8InB.s7;

    // write back out to GMEM
    c[get_global_id(0)] = f8Out;
}
///////////////////////////////////////////////////


//-- MAIN METHOD
void processMiniCLTask(void* userPtr, void* lsMemory)
{
	//	BT_PROFILE("processSampleTask");

	MiniCLTask_LocalStoreMemory* localMemory = (MiniCLTask_LocalStoreMemory*)lsMemory;

	MiniCLTaskDesc* taskDescPtr = (MiniCLTaskDesc*)userPtr;
	MiniCLTaskDesc& taskDesc = *taskDescPtr;

	printf("Compute Unit[%d] executed kernel %d work items [%d..%d)\n",taskDesc.m_taskId,taskDesc.m_kernelProgramId,taskDesc.m_firstWorkUnit,taskDesc.m_lastWorkUnit);
	
	
	switch (taskDesc.m_kernelProgramId)
	{
	case CMD_MINICL_ADDVECTOR:
		{
			for (unsigned int i=taskDesc.m_firstWorkUnit;i<taskDesc.m_lastWorkUnit;i++)
			{
				VectorAdd(*(const float8**)&taskDesc.m_argData[0][0],*(const float8**)&taskDesc.m_argData[1][0],*(float8**)&taskDesc.m_argData[2][0],i);
			}
			break;
		}

	default:
		{
			printf("error in processMiniCLTask: unknown command id: %d\n",taskDesc.m_kernelProgramId);

		}
	};

}


#if defined(__CELLOS_LV2__) || defined (LIBSPE2)

ATTRIBUTE_ALIGNED16(MiniCLTask_LocalStoreMemory	gLocalStoreMemory);

void* createMiniCLLocalStoreMemory()
{
	return &gLocalStoreMemory;
}
#else
void* createMiniCLLocalStoreMemory()
{
	return new MiniCLTask_LocalStoreMemory;
};

#endif
