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

#include "GuVecBox.h"
#include "GuVecConvexHull.h"
#include "GuVecConvexHullNoScale.h"
#include "GuVecTriangle.h"
#include "GuGeometryUnion.h"

#include "GuContactMethodImpl.h"
#include "PxTriangleMesh.h"
#include "GuContactBuffer.h"
#include "GuHeightField.h"
#include "GuPCMContactConvexCommon.h"
#include "GuPCMContactMeshCallback.h"

using namespace physx;
using namespace Gu;
using namespace physx::shdfnd::aos;

namespace physx
{

struct PCMSphereVsHeightfieldContactGenerationCallback :  PCMHeightfieldContactGenerationCallback<PCMSphereVsHeightfieldContactGenerationCallback>
{

public:
	PCMSphereVsMeshContactGeneration		mGeneration;

	PCMSphereVsHeightfieldContactGenerationCallback(
		const Ps::aos::Vec3VArg						sphereCenter,
		const Ps::aos::FloatVArg					sphereRadius,
		const Ps::aos::FloatVArg					contactDistance,
		const Ps::aos::FloatVArg						replaceBreakingThreshold,
	
		const PsTransformV&								sphereTransform, 
		const PsTransformV&								heightfieldTransform,
		const PxTransform&								heightfieldTransform1,
		Gu::MultiplePersistentContactManifold&			multiManifold,
		Gu::ContactBuffer&								contactBuffer,
		Ps::InlineArray<PxU32, LOCAL_CONTACTS_SIZE>*	deferredContacts,
		Gu::HeightFieldUtil& hfUtil 
		
		
	) :
		PCMHeightfieldContactGenerationCallback<PCMSphereVsHeightfieldContactGenerationCallback>(hfUtil, heightfieldTransform1),
		mGeneration(sphereCenter, sphereRadius, contactDistance, replaceBreakingThreshold, sphereTransform,
			heightfieldTransform, multiManifold, contactBuffer, deferredContacts)
	{
	}

	template<PxU32 CacheSize>
	void processTriangleCache(Gu::TriangleCache<CacheSize>& cache)
	{
		mGeneration.processTriangleCache<CacheSize, PCMSphereVsMeshContactGeneration>(cache);
	}

};


bool Gu::pcmContactSphereHeightField(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);

	const PxSphereGeometry& shapeSphere = shape0.get<const PxSphereGeometry>();
	const PxHeightFieldGeometryLL& shapeHeight = shape1.get<const PxHeightFieldGeometryLL>();

	Gu::MultiplePersistentContactManifold& multiManifold = cache.getMultipleManifold();

	const QuatV q0 = QuatVLoadA(&transform0.q.x);
	const Vec3V p0 = V3LoadA(&transform0.p.x);

	const QuatV q1 = QuatVLoadA(&transform1.q.x);
	const Vec3V p1 = V3LoadA(&transform1.p.x);

	const FloatV sphereRadius = FLoad(shapeSphere.radius);
	const FloatV contactDist = FLoad(params.mContactDistance);
	
	const PsTransformV sphereTransform(p0, q0);//sphere transform
	const PsTransformV heightfieldTransform(p1, q1);//height feild
	const PsTransformV curTransform = heightfieldTransform.transformInv(sphereTransform);
	

	// We must be in local space to use the cache

	if(multiManifold.invalidate(curTransform, sphereRadius, FLoad(0.02f)))
	{
		multiManifold.mNumManifolds = 0;
		multiManifold.setRelativeTransform(curTransform);

		const FloatV replaceBreakingThreshold = FMul(sphereRadius, FLoad(0.001f));
		Gu::HeightFieldUtil hfUtil(shapeHeight);
		const PxVec3 sphereCenterShape1Space = transform1.transformInv(transform0.p);
		const Vec3V sphereCenter = V3LoadU(sphereCenterShape1Space);
		PxReal inflatedRadius = shapeSphere.radius + params.mContactDistance;
		PxVec3 inflatedRadiusV(inflatedRadius);

		PxBounds3 bounds(sphereCenterShape1Space - inflatedRadiusV, sphereCenterShape1Space + inflatedRadiusV);

		Ps::InlineArray<PxU32, LOCAL_CONTACTS_SIZE> delayedContacts;

		PCMSphereVsHeightfieldContactGenerationCallback blockCallback(
			sphereCenter,
			sphereRadius,
			contactDist,
			replaceBreakingThreshold,
			sphereTransform,
			heightfieldTransform,
			transform1,
			multiManifold,
			contactBuffer,
			&delayedContacts,
			hfUtil);

		hfUtil.overlapAABBTriangles(transform1, bounds, 0, &blockCallback);

		blockCallback.mGeneration.generateLastContacts();
		blockCallback.mGeneration.processContacts(GU_SPHERE_MANIFOLD_CACHE_SIZE, false);
	}
	else
	{
		const PsMatTransformV aToB(curTransform);
		const FloatV projectBreakingThreshold = FMul(sphereRadius, FLoad(0.05f));
		const FloatV refereshDistance = FAdd(sphereRadius, contactDist);
		multiManifold.refreshManifold(aToB, projectBreakingThreshold, refereshDistance);

	}

	return multiManifold.addManifoldContactsToContactBuffer(contactBuffer, sphereTransform, heightfieldTransform, sphereRadius);
}


}
