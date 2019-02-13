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


#ifndef PX_CHARACTER_INTERNAL_STRUCTS_H
#define PX_CHARACTER_INTERNAL_STRUCTS_H

#include "CctController.h"

namespace physx
{
	class PxObstacle;	// (*)

namespace Cct
{
	class ObstacleContext;

	enum UserObjectType
	{
		USER_OBJECT_CCT					= 0,
		USER_OBJECT_BOX_OBSTACLE		= 1,
		USER_OBJECT_CAPSULE_OBSTACLE	= 2
	};

	PX_FORCE_INLINE	PxU32	encodeUserObject(PxU32 index, UserObjectType type)
	{
		PX_ASSERT(index<=0xffff);
		PX_ASSERT(PxU32(type)<=0xffff);
		return (PxU16(index)<<16)|PxU32(type);
	}

	PX_FORCE_INLINE	UserObjectType	decodeType(PxU32 code)
	{
		return UserObjectType(code & 0xffff);
	}

	PX_FORCE_INLINE	PxU32	decodeIndex(PxU32 code)
	{
		return code>>16;
	}

	struct PxInternalCBData_OnHit : InternalCBData_OnHit
	{
		Controller*				controller;
		const ObstacleContext*	obstacles;
		const PxObstacle*		touchedObstacle;
		ObstacleHandle			touchedObstacleHandle;
	};

	struct PxInternalCBData_FindTouchedGeom : InternalCBData_FindTouchedGeom
	{
		PxScene*				scene;
		Cm::RenderBuffer*		renderBuffer;	// Render buffer from controller manager, not the one from the scene

		Ps::HashSet<PxShape*>*	cctShapeHashSet;
	};
}
}

#endif
