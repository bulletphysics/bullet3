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


#include "GuVecTriangle.h"
#include "GuVecCapsule.h"
#include "GuGeometryUnion.h"
#include "PsVecMath.h"
#include "PsVecTransform.h"

#include "GuContactMethodImpl.h"
#include "PxTriangleMesh.h"
#include "GuContactBuffer.h"
#include "GuPCMContactConvexCommon.h"
#include "GuSegment.h"
#include "GuVecCapsule.h"
#include "GuInternal.h"
#include "GuPCMContactMeshCallback.h"
#include "GuConvexEdgeFlags.h"
#include "GuBox.h"

using namespace physx;
using namespace Gu;
using namespace physx::shdfnd::aos;

namespace physx
{

struct PCMCapsuleVsMeshContactGenerationCallback : PCMMeshContactGenerationCallback< PCMCapsuleVsMeshContactGenerationCallback >
{
	PCMCapsuleVsMeshContactGenerationCallback& operator=(const PCMCapsuleVsMeshContactGenerationCallback&);
public:
	PCMCapsuleVsMeshContactGeneration		mGeneration;

	PCMCapsuleVsMeshContactGenerationCallback(
		const CapsuleV&									capsule,
		const Ps::aos::FloatVArg						contactDist,
		const Ps::aos::FloatVArg						replaceBreakingThreshold,
		const PsTransformV&								sphereTransform,
		const PsTransformV&								meshTransform,
		MultiplePersistentContactManifold&				multiManifold,
		ContactBuffer&									contactBuffer,
		const PxU8*										extraTriData,
		const Cm::FastVertex2ShapeScaling&				meshScaling,
		bool											idtMeshScale,
		Ps::InlineArray<PxU32, LOCAL_CONTACTS_SIZE>*	deferredContacts,
		Cm::RenderOutput*								renderOutput = NULL
	) :
		PCMMeshContactGenerationCallback<PCMCapsuleVsMeshContactGenerationCallback>(meshScaling, extraTriData, idtMeshScale),
		mGeneration(capsule, contactDist, replaceBreakingThreshold, sphereTransform, meshTransform, multiManifold, contactBuffer, 
			deferredContacts, renderOutput)
	{
	}

	PX_FORCE_INLINE bool doTest(const PxVec3&, const PxVec3&, const PxVec3&) { return true; }

	template<PxU32 CacheSize>
	void processTriangleCache(TriangleCache<CacheSize>& cache)
	{
		mGeneration.processTriangleCache<CacheSize, PCMCapsuleVsMeshContactGeneration>(cache);
	}
	
};

bool Gu::pcmContactCapsuleMesh(GU_CONTACT_METHOD_ARGS)
{
	using namespace Ps::aos;
	MultiplePersistentContactManifold& multiManifold = cache.getMultipleManifold();
	const PxCapsuleGeometry& shapeCapsule= shape0.get<const PxCapsuleGeometry>();
	const PxTriangleMeshGeometryLL& shapeMesh = shape1.get<const PxTriangleMeshGeometryLL>();

	//gRenderOutPut = cache.mRenderOutput;
	const FloatV capsuleRadius = FLoad(shapeCapsule.radius);
	const FloatV contactDist = FLoad(params.mContactDistance);

	const PsTransformV capsuleTransform = loadTransformA(transform0);//capsule transform
	const PsTransformV meshTransform = loadTransformA(transform1);//triangleMesh  

	const PsTransformV curTransform = meshTransform.transformInv(capsuleTransform);
	
	// We must be in local space to use the cache
	if(multiManifold.invalidate(curTransform, capsuleRadius, FLoad(0.02f)))
	{
		const FloatV replaceBreakingThreshold = FMul(capsuleRadius, FLoad(0.001f));
		//const FloatV capsuleHalfHeight = FloatV_From_F32(shapeCapsule.halfHeight);
		Cm::FastVertex2ShapeScaling meshScaling;
		const bool idtMeshScale = shapeMesh.scale.isIdentity();
		if(!idtMeshScale)
			meshScaling.init(shapeMesh.scale);

		// Capsule data
		const PxVec3 tmp = getCapsuleHalfHeightVector(transform0, shapeCapsule);
		Segment worldCapsule;
		worldCapsule.p0 = transform0.p + tmp;
		worldCapsule.p1 = transform0.p - tmp;

		
		const Segment meshCapsule(	// Capsule in mesh space
			transform1.transformInv(worldCapsule.p0),
			transform1.transformInv(worldCapsule.p1));

		const PxReal inflatedRadius = shapeCapsule.radius + params.mContactDistance;

		const PxVec3 capsuleCenterInMesh = transform1.transformInv(transform0.p);
		const PxVec3 capsuleDirInMesh = transform1.rotateInv(tmp);
		const CapsuleV capsule(V3LoadU(capsuleCenterInMesh), V3LoadU(capsuleDirInMesh), capsuleRadius);

		// We must be in local space to use the cache
		const Capsule queryCapsule(meshCapsule, inflatedRadius);

		const TriangleMesh* meshData = shapeMesh.meshData;

		multiManifold.mNumManifolds = 0;
		multiManifold.setRelativeTransform(curTransform); 

		const PxU8* PX_RESTRICT extraData = meshData->getExtraTrigData();
		// mesh scale is not baked into cached verts
		PCMCapsuleVsMeshContactGenerationCallback callback(
			capsule,
			contactDist,
			replaceBreakingThreshold,
			capsuleTransform,
			meshTransform,
			multiManifold,
			contactBuffer,
			extraData,
			meshScaling,
			idtMeshScale,
			NULL,
			renderOutput);

		//bound the capsule in shape space by an OBB:
		Box queryBox;
		queryBox.create(queryCapsule);

		//apply the skew transform to the box:
		if(!idtMeshScale)
			meshScaling.transformQueryBounds(queryBox.center, queryBox.extents, queryBox.rot);

		Midphase::intersectOBB(meshData, queryBox, callback, true);

		callback.flushCache();
	
		callback.mGeneration.processContacts(GU_CAPSULE_MANIFOLD_CACHE_SIZE, false);
	}
	else
	{
		const PsMatTransformV aToB(curTransform);
		const FloatV projectBreakingThreshold = FMul(capsuleRadius, FLoad(0.05f));
		const FloatV refereshDistance = FAdd(capsuleRadius, contactDist);
		//multiManifold.refreshManifold(aToB, projectBreakingThreshold, contactDist);
		multiManifold.refreshManifold(aToB, projectBreakingThreshold, refereshDistance);
	}

	//multiManifold.drawManifold(*gRenderOutPut, capsuleTransform, meshTransform);
	return multiManifold.addManifoldContactsToContactBuffer(contactBuffer, capsuleTransform, meshTransform, capsuleRadius);
}

}
