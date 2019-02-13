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

#include "GuGeometryUnion.h"

#include "GuContactBuffer.h"
#include "GuContactMethodImpl.h"
#include "PsVecTransform.h"

namespace physx
{
namespace Gu
{
bool pcmContactSphereSphere(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(cache);
	PX_UNUSED(renderOutput);

	using namespace Ps::aos;
	const PxSphereGeometry& shapeSphere0 = shape0.get<const PxSphereGeometry>();
	const PxSphereGeometry& shapeSphere1 = shape1.get<const PxSphereGeometry>();
	
	const FloatV cDist	= FLoad(params.mContactDistance);
	const Vec3V p0 =  V3LoadA(&transform0.p.x);
	const Vec3V p1 =  V3LoadA(&transform1.p.x);

	const FloatV r0		= FLoad(shapeSphere0.radius);
	const FloatV r1		= FLoad(shapeSphere1.radius);
	

	const Vec3V _delta = V3Sub(p0, p1);
	const FloatV distanceSq = V3Dot(_delta, _delta);
	const FloatV radiusSum = FAdd(r0, r1);
	const FloatV inflatedSum = FAdd(radiusSum, cDist);
	
	if(FAllGrtr(FMul(inflatedSum, inflatedSum), distanceSq))
	{
		const FloatV eps	=  FLoad(0.00001f);
		const FloatV dist = FSqrt(distanceSq);
		const BoolV bCon = FIsGrtrOrEq(eps, dist);
		const Vec3V normal = V3Sel(bCon, V3UnitX(), V3ScaleInv(_delta, dist));
		const Vec3V point = V3ScaleAdd(normal, r1, p1);
		const FloatV pen = FSub(dist, radiusSum);
		
		PX_ASSERT(contactBuffer.count < ContactBuffer::MAX_CONTACTS);
		Gu::ContactPoint& contact = contactBuffer.contacts[contactBuffer.count++];
		V4StoreA(Vec4V_From_Vec3V(normal), &contact.normal.x);
		V4StoreA(Vec4V_From_Vec3V(point), &contact.point.x);
		FStore(pen, &contact.separation);

		contact.internalFaceIndex1 = PXC_CONTACT_NO_FACE_INDEX;

		return true;
	}
	return false;
}
}//Gu
}//physx
