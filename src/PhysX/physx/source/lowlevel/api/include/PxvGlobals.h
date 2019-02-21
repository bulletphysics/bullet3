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
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#ifndef PXD_INIT_H
#define PXD_INIT_H

#include "PxvConfig.h"
#include "PsBasicTemplates.h"

namespace physx
{

/*!
\file
PhysX Low-level, Memory management
*/

/************************************************************************/
/* Error Handling                                                       */
/************************************************************************/


enum PxvErrorCode
{
	PXD_ERROR_NO_ERROR = 0,
	PXD_ERROR_INVALID_PARAMETER,
	PXD_ERROR_INVALID_PARAMETER_SIZE,
	PXD_ERROR_INTERNAL_ERROR,
	PXD_ERROR_NOT_IMPLEMENTED,
	PXD_ERROR_NO_CONTEXT,
	PXD_ERROR_NO_TASK_MANAGER,
	PXD_ERROR_WARNING
};

class PxShape;
class PxRigidActor;
struct PxsShapeCore;
struct PxsRigidCore;

struct PxvOffsetTable
{
	PX_FORCE_INLINE PxvOffsetTable() {}

	PX_FORCE_INLINE const PxShape* convertPxsShape2Px(const PxsShapeCore* pxs) const
	{
		return shdfnd::pointerOffset<const PxShape*>(pxs, pxsShapeCore2PxShape); 
	}

	PX_FORCE_INLINE const PxRigidActor* convertPxsRigidCore2PxRigidBody(const PxsRigidCore* pxs) const
	{
		return shdfnd::pointerOffset<const PxRigidActor*>(pxs, pxsRigidCore2PxRigidBody); 
	}

	PX_FORCE_INLINE const PxRigidActor* convertPxsRigidCore2PxRigidStatic(const PxsRigidCore* pxs) const
	{
		return shdfnd::pointerOffset<const PxRigidActor*>(pxs, pxsRigidCore2PxRigidStatic); 
	}

	ptrdiff_t	pxsShapeCore2PxShape;
	ptrdiff_t	pxsRigidCore2PxRigidBody;
	ptrdiff_t	pxsRigidCore2PxRigidStatic;
};
extern PxvOffsetTable gPxvOffsetTable;

/*!
Initialize low-level implementation.
*/

void PxvInit(const PxvOffsetTable& offsetTable);


/*!
Shut down low-level implementation.
*/
void PxvTerm();

/*!
Initialize low-level implementation.
*/

void PxvRegisterHeightFields();

#if PX_SUPPORT_GPU_PHYSX
class PxPhysXGpu* PxvGetPhysXGpu(bool createIfNeeded);
#endif

}

#endif
