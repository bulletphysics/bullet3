//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "PxvGlobals.h"
#include "PxsContext.h"
#include "PxcContactMethodImpl.h"
#include "GuContactMethodImpl.h"


#if PX_SUPPORT_GPU_PHYSX
#include "PxPhysXGpu.h"

physx::PxPhysXGpu* gPxPhysXGpu;
#endif

namespace physx
{

PxvOffsetTable gPxvOffsetTable;

bool PxcPCMContactSphereHeightField		(GU_CONTACT_METHOD_ARGS);
bool PxcPCMContactCapsuleHeightField	(GU_CONTACT_METHOD_ARGS);
bool PxcPCMContactBoxHeightField		(GU_CONTACT_METHOD_ARGS);
bool PxcPCMContactConvexHeightField		(GU_CONTACT_METHOD_ARGS);

bool PxcContactSphereHeightField		(GU_CONTACT_METHOD_ARGS);
bool PxcContactCapsuleHeightField		(GU_CONTACT_METHOD_ARGS);
bool PxcContactBoxHeightField			(GU_CONTACT_METHOD_ARGS);
bool PxcContactConvexHeightField		(GU_CONTACT_METHOD_ARGS);

void PxvRegisterHeightFields()
{
	g_ContactMethodTable[PxGeometryType::eSPHERE][PxGeometryType::eHEIGHTFIELD]			= PxcContactSphereHeightField;
	g_ContactMethodTable[PxGeometryType::eCAPSULE][PxGeometryType::eHEIGHTFIELD]		= PxcContactCapsuleHeightField;
	g_ContactMethodTable[PxGeometryType::eBOX][PxGeometryType::eHEIGHTFIELD]			= PxcContactBoxHeightField;
	g_ContactMethodTable[PxGeometryType::eCONVEXMESH][PxGeometryType::eHEIGHTFIELD]		= PxcContactConvexHeightField;

	g_PCMContactMethodTable[PxGeometryType::eSPHERE][PxGeometryType::eHEIGHTFIELD]		= PxcPCMContactSphereHeightField;
	g_PCMContactMethodTable[PxGeometryType::eCAPSULE][PxGeometryType::eHEIGHTFIELD]		= PxcPCMContactCapsuleHeightField;
	g_PCMContactMethodTable[PxGeometryType::eBOX][PxGeometryType::eHEIGHTFIELD]			= PxcPCMContactBoxHeightField;
	g_PCMContactMethodTable[PxGeometryType::eCONVEXMESH][PxGeometryType::eHEIGHTFIELD]	= PxcPCMContactConvexHeightField;
}

void PxvInit(const PxvOffsetTable& offsetTable)
{
#if PX_SUPPORT_GPU_PHYSX
	gPxPhysXGpu = NULL;
#endif
	gPxvOffsetTable = offsetTable;
}

void PxvTerm()
{
#if PX_SUPPORT_GPU_PHYSX
	if (gPxPhysXGpu)
	{
		gPxPhysXGpu->release();
		gPxPhysXGpu = NULL;
	}
#endif
}

}

#if PX_SUPPORT_GPU_PHYSX
namespace physx
{
	//forward declare stuff from PxPhysXGpuModuleLoader.cpp
	void PxLoadPhysxGPUModule(const char* appGUID);
	typedef physx::PxPhysXGpu* (PxCreatePhysXGpu_FUNC)();
	extern PxCreatePhysXGpu_FUNC* g_PxCreatePhysXGpu_Func;

	PxPhysXGpu* PxvGetPhysXGpu(bool createIfNeeded)
	{
		if (!gPxPhysXGpu && createIfNeeded)
		{
#ifdef PX_PHYSX_GPU_STATIC
			gPxPhysXGpu = PxCreatePhysXGpu();
#else
			PxLoadPhysxGPUModule(NULL);
			if (g_PxCreatePhysXGpu_Func)
			{
				gPxPhysXGpu = g_PxCreatePhysXGpu_Func();
			}
#endif
		}
		
		return gPxPhysXGpu;
	}
}
#endif
