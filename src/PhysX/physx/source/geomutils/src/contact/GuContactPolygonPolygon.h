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

#ifndef GU_CONTACTPOLYGONPOLYGON_H
#define GU_CONTACTPOLYGONPOLYGON_H

#include "foundation/PxVec3.h"
#include "CmPhysXCommon.h"
#include "PxPhysXCommonConfig.h"

namespace physx
{

namespace Cm
{
	class Matrix34;
	class FastVertex2ShapeScaling;
}

namespace Gu
{
class ContactBuffer;

PX_PHYSX_COMMON_API PxMat33 findRotationMatrixFromZ(const PxVec3& to);

PX_PHYSX_COMMON_API bool contactPolygonPolygonExt(	PxU32 numVerts0, const PxVec3* vertices0, const PxU8* indices0,//polygon 0
													const Cm::Matrix34& world0, const PxPlane& localPlane0,			//xform of polygon 0, plane of polygon
													const PxMat33& RotT0,

													PxU32 numVerts1, const PxVec3* vertices1, const PxU8* indices1,//polygon 1
													const Cm::Matrix34& world1, const PxPlane& localPlane1,			//xform of polygon 1, plane of polygon
													const PxMat33& RotT1,

													const PxVec3& worldSepAxis,									//world normal of separating plane - this is the world space normal of polygon0!!
													const Cm::Matrix34& transform0to1, const Cm::Matrix34& transform1to0,//transforms between polygons
													PxU32 polyIndex0, PxU32 polyIndex1,	//face indices for contact callback,
													ContactBuffer& contactBuffer,
													bool flipNormal, const PxVec3& posShift, float sepShift
													);	// shape order, post gen shift.
}
}

#endif
