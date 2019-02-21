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

#include "GuContactBuffer.h"
#include "GuContactMethodImpl.h"
#include "GuGeometryUnion.h"

namespace physx
{
namespace Gu
{
bool contactSpherePlane(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);
	PX_UNUSED(cache);
	PX_UNUSED(shape1);

	// Get actual shape data
	const PxSphereGeometry& shapeSphere = shape0.get<const PxSphereGeometry>();
	//const PxPlaneGeometry& shapePlane = shape1.get<const PxPlaneGeometry>();

	//Sphere in plane space
	const PxVec3 sphere = transform1.transformInv(transform0.p);
	
	//Make sure we have a normalized plane
	//The plane is implicitly n=<1,0,0> d=0 (in plane-space)
	//PX_ASSERT(PxAbs(shape1.mNormal.magnitudeSquared() - 1.0f) < 0.000001f);

	//Separation
	const PxReal separation = sphere.x - shapeSphere.radius;

	if(separation<=params.mContactDistance)
	{
		const PxVec3 normal = transform1.q.getBasisVector0();
		const PxVec3 point  = transform0.p - normal * shapeSphere.radius;
		contactBuffer.contact(point, normal, separation);
		return true;
	}
	return false;
}
}//Gu
}//physx
