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

#include "GuContactMethodImpl.h"
#include "GuContactBuffer.h"
#include "GuInternal.h"
#include "GuSegment.h"
#include "GuGeometryUnion.h"

namespace physx
{
namespace Gu
{
bool contactPlaneCapsule(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);
	PX_UNUSED(cache);
	PX_UNUSED(shape0);

	// Get actual shape data
	//const PxPlaneGeometry& shapePlane = shape.get<const PxPlaneGeometry>();
	const PxCapsuleGeometry& shapeCapsule = shape1.get<const PxCapsuleGeometry>();

	const PxTransform capsuleToPlane = transform0.transformInv(transform1);

	//Capsule in plane space
	Segment segment;
	getCapsuleSegment(capsuleToPlane, shapeCapsule, segment);
	
	const PxVec3 negPlaneNormal = transform0.q.getBasisVector0();
	
	bool contact = false;

	const PxReal separation0 = segment.p0.x - shapeCapsule.radius;
	const PxReal separation1 = segment.p1.x - shapeCapsule.radius;
	if(separation0 <= params.mContactDistance)
	{
		const PxVec3 temp(segment.p0.x - shapeCapsule.radius, segment.p0.y, segment.p0.z);
		const PxVec3 point = transform0.transform(temp);
		contactBuffer.contact(point, -negPlaneNormal, separation0);
		contact = true;
	}

	if(separation1 <= params.mContactDistance)
	{
		const PxVec3 temp(segment.p1.x - shapeCapsule.radius, segment.p1.y, segment.p1.z);
		const PxVec3 point = transform0.transform(temp);
		contactBuffer.contact(point, -negPlaneNormal, separation1);
		contact = true;
	}
	return contact;
}
}//Gu
}//physx
