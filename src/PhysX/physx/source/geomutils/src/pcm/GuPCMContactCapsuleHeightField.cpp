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

#include "PsVecMath.h"
#include "PsVecTransform.h"
#include "GuVecTriangle.h"
#include "GuGeometryUnion.h"

#include "GuContactMethodImpl.h"
#include "PxTriangleMesh.h"
#include "GuContactBuffer.h"
#include "GuHeightField.h"
#include "GuPCMContactConvexCommon.h"
#include "GuSegment.h"
#include "GuInternal.h"
#include "GuPCMContactMeshCallback.h"

using namespace physx;
using namespace Gu;
using namespace Ps::aos;

namespace physx
{

struct PCMCapsuleVsHeightfieldContactGenerationCallback :  PCMHeightfieldContactGenerationCallback<PCMCapsuleVsHeightfieldContactGenerationCallback>
{
	PCMCapsuleVsHeightfieldContactGenerationCallback& operator=(const PCMCapsuleVsHeightfieldContactGenerationCallback&);

public:
	PCMCapsuleVsMeshContactGeneration		mGeneration;

	PCMCapsuleVsHeightfieldContactGenerationCallback(
		const Gu::CapsuleV&								capsule,
		const Ps::aos::FloatVArg						contactDistance,
		const Ps::aos::FloatVArg						replaceBreakingThreshold,
	
		const PsTransformV&								capsuleTransform, 
		const PsTransformV&								heightfieldTransform,
		const PxTransform&								heightfieldTransform1,
		Gu::MultiplePersistentContactManifold&			multiManifold,
		Gu::ContactBuffer&								contactBuffer,
		Ps::InlineArray<PxU32, LOCAL_CONTACTS_SIZE>*	deferredContacts,
		Gu::HeightFieldUtil& hfUtil 
		
		
	) :
		PCMHeightfieldContactGenerationCallback<PCMCapsuleVsHeightfieldContactGenerationCallback>(hfUtil, heightfieldTransform1),
		mGeneration(capsule, contactDistance, replaceBreakingThreshold, capsuleTransform, heightfieldTransform, multiManifold, 
			contactBuffer, deferredContacts)
	{
	}

	template<PxU32 CacheSize>
	void processTriangleCache(Gu::TriangleCache<CacheSize>& cache)
	{
		mGeneration.processTriangleCache<CacheSize, PCMCapsuleVsMeshContactGeneration>(cache);
	}
	
};

bool Gu::pcmContactCapsuleHeightField(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);

	const PxCapsuleGeometry& shapeCapsule = shape0.get<const PxCapsuleGeometry>();
	const PxHeightFieldGeometryLL& shapeHeight = shape1.get<const PxHeightFieldGeometryLL>();

	Gu::MultiplePersistentContactManifold& multiManifold = cache.getMultipleManifold();

	const FloatV capsuleRadius = FLoad(shapeCapsule.radius);
	const FloatV contactDist = FLoad(params.mContactDistance);

	const PsTransformV capsuleTransform = loadTransformA(transform0);//capsule transform
	const PsTransformV heightfieldTransform = loadTransformA(transform1);//height feild

	const PsTransformV curTransform = heightfieldTransform.transformInv(capsuleTransform);
	
	const FloatV replaceBreakingThreshold = FMul(capsuleRadius, FLoad(0.001f));

	if(multiManifold.invalidate(curTransform, capsuleRadius, FLoad(0.02f)))
	{

		multiManifold.mNumManifolds = 0;
		multiManifold.setRelativeTransform(curTransform); 

		Gu::HeightFieldUtil hfUtil(shapeHeight);

		const PxVec3 tmp = getCapsuleHalfHeightVector(transform0, shapeCapsule);

		const PxReal inflatedRadius = shapeCapsule.radius + params.mContactDistance;

		const PxVec3 capsuleCenterInMesh = transform1.transformInv(transform0.p);
		const PxVec3 capsuleDirInMesh = transform1.rotateInv(tmp);
		const Gu::CapsuleV capsule(V3LoadU(capsuleCenterInMesh), V3LoadU(capsuleDirInMesh), capsuleRadius);

		PCMCapsuleVsHeightfieldContactGenerationCallback callback(
			capsule,
			contactDist,
			replaceBreakingThreshold,
			capsuleTransform,
			heightfieldTransform,
			transform1,
			multiManifold,
			contactBuffer,
			NULL,
			hfUtil
		);

		PxBounds3 bounds;
		bounds.maximum = PxVec3(shapeCapsule.halfHeight + inflatedRadius, inflatedRadius, inflatedRadius);
		bounds.minimum = -bounds.maximum;

		bounds = PxBounds3::transformFast(transform1.transformInv(transform0), bounds);

		hfUtil.overlapAABBTriangles(transform1, bounds, 0, &callback);

		callback.mGeneration.processContacts(GU_CAPSULE_MANIFOLD_CACHE_SIZE, false);
	}
	else
	{
		const PsMatTransformV aToB(curTransform);
		// We must be in local space to use the cache
		const FloatV projectBreakingThreshold = FMul(capsuleRadius, FLoad(0.05f));
		const FloatV refereshDistance = FAdd(capsuleRadius, contactDist);
		multiManifold.refreshManifold(aToB, projectBreakingThreshold, refereshDistance);

	}
	return multiManifold.addManifoldContactsToContactBuffer(contactBuffer, capsuleTransform, heightfieldTransform, capsuleRadius);
}


}
