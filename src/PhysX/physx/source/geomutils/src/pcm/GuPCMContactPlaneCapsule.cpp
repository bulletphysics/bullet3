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


#include "GuVecCapsule.h"
#include "GuGeometryUnion.h"

#include "GuContactMethodImpl.h"
#include "GuContactBuffer.h"
#include "GuPersistentContactManifold.h"

namespace physx
{
namespace Gu
{
bool pcmContactPlaneCapsule(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(shape0);
	PX_UNUSED(renderOutput);

	using namespace Ps::aos;

	Gu::PersistentContactManifold& manifold = cache.getManifold();
	Ps::prefetchLine(&manifold, 256);

	// Get actual shape data
	const PxCapsuleGeometry& shapeCapsule = shape1.get<const PxCapsuleGeometry>();

	const PsTransformV transf0 = loadTransformA(transform1);//capsule transform
	const PsTransformV transf1 = loadTransformA(transform0);//plane transform
	//capsule to plane
	const PsTransformV aToB(transf1.transformInv(transf0));

	//in world space
	const Vec3V planeNormal = V3Normalize(QuatGetBasisVector0(transf1.q));
	const Vec3V contactNormal = V3Neg(planeNormal);
	
	//ML:localNormal is the local space of plane normal, however, because shape1 is capulse and shape0 is plane, we need to use the reverse of contact normal(which will be the plane normal) to make the refreshContactPoints
	//work out the correct pentration for points
	const Vec3V localNormal = V3UnitX();

	const FloatV contactDist = FLoad(params.mContactDistance);

	const FloatV radius = FLoad(shapeCapsule.radius);
	const FloatV halfHeight = FLoad(shapeCapsule.halfHeight);

	//capsule is in the local space of plane(n = (1.f, 0.f, 0.f), d=0.f)
	const Vec3V basisVector = QuatGetBasisVector0(aToB.q);
	const Vec3V tmp = V3Scale(basisVector, halfHeight);
	const Vec3V s = V3Add(aToB.p, tmp);
	const Vec3V e = V3Sub(aToB.p, tmp);

	const FloatV inflatedRadius = FAdd(radius, contactDist);
	const FloatV replaceBreakingThreshold = FMul(radius, FLoad(0.001f));
	const FloatV projectBreakingThreshold = FMul(radius, FLoad(0.05f));
	const PxU32 initialContacts = manifold.mNumContacts;

	//manifold.refreshContactPoints(curRTrans, projectBreakingThreshold, contactDist);
	const FloatV refreshDist = FAdd(contactDist, radius);
	manifold.refreshContactPoints(aToB, projectBreakingThreshold, refreshDist);

	const PxU32 newContacts = manifold.mNumContacts;
	const bool bLostContacts = (newContacts != initialContacts);//((initialContacts == 0) || (newContacts != initialContacts));

	if(bLostContacts || manifold.invalidate_PrimitivesPlane(aToB, radius, FLoad(0.02f)))  
	{
		manifold.mNumContacts = 0;
		manifold.setRelativeTransform(aToB);
		//calculate the distance from s to the plane
		const FloatV signDist0  = V3GetX(s);//V3Dot(localNormal, s);
		if(FAllGrtr(inflatedRadius, signDist0))
		{
			const Vec3V localPointA = aToB.transformInv(s);
			const Vec3V localPointB = V3NegScaleSub(localNormal, signDist0, s);
			const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(localNormal), signDist0);
			//add to manifold
			manifold.addManifoldPoint2(localPointA, localPointB, localNormalPen, replaceBreakingThreshold); 
		}

		const FloatV signDist1 = V3GetX(e);//V3Dot(localNormal, e);
		if(FAllGrtr(inflatedRadius, signDist1))
		{
			const Vec3V localPointA = aToB.transformInv(e);
			
			const Vec3V localPointB = V3NegScaleSub(localNormal, signDist1, e);
			const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(localNormal), signDist1);
			//add to manifold
			manifold.addManifoldPoint2(localPointA, localPointB, localNormalPen, replaceBreakingThreshold); 
		}

		manifold.addManifoldContactsToContactBuffer(contactBuffer, contactNormal, planeNormal, transf0, radius, contactDist);

#if	PCM_LOW_LEVEL_DEBUG
		manifold.drawManifold(*renderOutput, transf0, transf1);
#endif

		return manifold.getNumContacts() > 0;
	}
	else
	{
		manifold.addManifoldContactsToContactBuffer(contactBuffer, contactNormal, planeNormal, transf0, radius, contactDist);
		return manifold.getNumContacts() > 0;
	}
}

}//Gu
}//physx
