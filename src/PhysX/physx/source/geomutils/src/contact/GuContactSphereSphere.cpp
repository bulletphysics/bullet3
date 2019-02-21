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
bool contactSphereSphere(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);
	PX_UNUSED(cache);

	const PxSphereGeometry& sphereGeom0 = shape0.get<const PxSphereGeometry>();
	const PxSphereGeometry& sphereGeom1 = shape1.get<const PxSphereGeometry>();

	PxVec3 delta = transform0.p - transform1.p;

	const PxReal distanceSq = delta.magnitudeSquared();
	const PxReal radiusSum = sphereGeom0.radius + sphereGeom1.radius;
	const PxReal inflatedSum = radiusSum + params.mContactDistance;
	if(distanceSq >= inflatedSum*inflatedSum)
		return false;

	// We do a *manual* normalization to check for singularity condition
	const PxReal magn = PxSqrt(distanceSq);
	if(magn<=0.00001f)
		delta = PxVec3(1.0f, 0.0f, 0.0f);	// PT: spheres are exactly overlapping => can't create normal => pick up random one
	else
		delta *= 1.0f/magn;

	// PT: TODO: why is this formula different from the original code?
	const PxVec3 contact = delta * ((sphereGeom0.radius + magn - sphereGeom1.radius)*-0.5f) + transform0.p;
		
	contactBuffer.contact(contact, delta, magn - radiusSum);
	return true;
}
}//Gu
}//physx
