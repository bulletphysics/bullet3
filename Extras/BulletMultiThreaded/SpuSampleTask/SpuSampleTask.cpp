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


#include "SpuSampleTask.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "../PlatformDefinitions.h"
#include "../SpuFakeDma.h"
#include "LinearMath/btMinMax.h"
#include <stdio.h>


struct SampleTask_LocalStoreMemory
{

};



//-- MAIN METHOD
void processSampleTask(void* userPtr, void* lsMemory)
{
//	BT_PROFILE("processSampleTask");

	SampleTask_LocalStoreMemory* localMemory = (SampleTask_LocalStoreMemory*)lsMemory;

	SpuSampleTaskDesc* taskDescPtr = (SpuSampleTaskDesc*)userPtr;
	SpuSampleTaskDesc& taskDesc = *taskDescPtr;

	switch (taskDesc.m_sampleCommand)
	{
	case CMD_SAMPLE_INTEGRATE_BODIES:
		{
#ifdef __SPU__
			spu_printf("hello SPU world\n");
#else
			printf("hello world\n");
#endif
			break;
		}
	default:
		{

		}
	};
}


#if defined(__CELLOS_LV2__) || defined (LIBSPE2)

ATTRIBUTE_ALIGNED16(SampleTask_LocalStoreMemory	gLocalStoreMemory);

void* createSampleLocalStoreMemory()
{
	return &gLocalStoreMemory;
}
#else
void* createSampleLocalStoreMemory()
{
        return new SampleTask_LocalStoreMemory;
};

#endif
