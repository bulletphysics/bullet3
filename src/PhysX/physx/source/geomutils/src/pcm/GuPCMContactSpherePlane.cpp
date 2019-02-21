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
#include "PsVecTransform.h"

#include "GuGeometryUnion.h"
#include "GuContactMethodImpl.h"

namespace physx
{

namespace Gu
{
bool pcmContactSpherePlane(GU_CONTACT_METHOD_ARGS)
{
	using namespace Ps::aos;
	PX_UNUSED(renderOutput);
	PX_UNUSED(cache);
	PX_UNUSED(shape1);

	// Get actual shape data
	const PxSphereGeometry& shapeSphere = shape0.get<const PxSphereGeometry>();

	//sphere transform
	const Vec3V p0 = V3LoadU_SafeReadW(transform0.p);	// PT: safe because 'mRefCount' follows 'mTransform' in PxsTransform

	//plane transform
	const Vec3V p1 = V3LoadU_SafeReadW(transform1.p);	// PT: safe because 'mRefCount' follows 'mTransform' in PxsTransform
	const QuatV q1 = QuatVLoadU(&transform1.q.x);

	const FloatV radius = FLoad(shapeSphere.radius);
	const FloatV contactDist = FLoad(params.mContactDistance);

	const PsTransformV transf1(p1, q1);
	//Sphere in plane space
	const Vec3V sphereCenterInPlaneSpace = transf1.transformInv(p0);
	

	//Separation
	const FloatV separation = FSub(V3GetX(sphereCenterInPlaneSpace), radius);

	if(FAllGrtrOrEq(contactDist, separation))
	{
		//get the plane normal
		const Vec3V worldNormal = QuatGetBasisVector0(q1);
		const Vec3V worldPoint = V3NegScaleSub(worldNormal, radius, p0);
		Gu::ContactPoint& contact = contactBuffer.contacts[contactBuffer.count++];
		//Fast allign store
		V4StoreA(Vec4V_From_Vec3V(worldNormal), &contact.normal.x);
		V4StoreA(Vec4V_From_Vec3V(worldPoint), &contact.point.x);
		FStore(separation, &contact.separation);
		contact.internalFaceIndex1 = PXC_CONTACT_NO_FACE_INDEX;
	
		return true;
	}
	return false;
}
}//Gu
}//physx
