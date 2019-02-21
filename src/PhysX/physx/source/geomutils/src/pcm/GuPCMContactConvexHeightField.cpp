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
#include "GuPCMShapeConvex.h"
#include "PxTriangleMesh.h"
#include "GuContactBuffer.h"
#include "GuHeightField.h"
#include "GuHeightFieldUtil.h"
#include "GuPCMContactConvexCommon.h"
#include "GuPCMContactMeshCallback.h"

#include "PsVecMath.h"


using namespace physx;
using namespace Gu;
using namespace physx::shdfnd::aos;

namespace physx
{

struct PCMConvexVsHeightfieldContactGenerationCallback
	: PCMHeightfieldContactGenerationCallback< PCMConvexVsHeightfieldContactGenerationCallback >
{
	PCMConvexVsHeightfieldContactGenerationCallback& operator=(const PCMConvexVsHeightfieldContactGenerationCallback&);
public:
	PCMConvexVsMeshContactGeneration		mGeneration;

	PCMConvexVsHeightfieldContactGenerationCallback(
		const Ps::aos::FloatVArg					contactDistance,
		const Ps::aos::FloatVArg					replaceBreakingThreshold,
		const Gu::PolygonalData&					polyData,
		SupportLocal*								polyMap,
		const Cm::FastVertex2ShapeScaling&			convexScaling,
		bool										idtConvexScale,
		const PsTransformV&							convexTransform, 
		const PsTransformV&							heightfieldTransform,
		const PxTransform&							heightfieldTransform1,
		Gu::MultiplePersistentContactManifold&		multiManifold,
		Gu::ContactBuffer&							contactBuffer,
		Gu::HeightFieldUtil&						hfUtil,
		Ps::InlineArray<PxU32,LOCAL_CONTACTS_SIZE>*	delayedContacts,
		bool										silhouetteEdgesAreActive,
		Cm::RenderOutput*							renderOutput = NULL
		
	) :
		PCMHeightfieldContactGenerationCallback< PCMConvexVsHeightfieldContactGenerationCallback >(hfUtil, heightfieldTransform1),
		mGeneration(contactDistance, replaceBreakingThreshold, convexTransform, heightfieldTransform,  multiManifold,
			contactBuffer, polyData, polyMap, delayedContacts, convexScaling, idtConvexScale, silhouetteEdgesAreActive, renderOutput)
	{
	}

	template<PxU32 CacheSize>
	void processTriangleCache(Gu::TriangleCache<CacheSize>& cache)
	{
		mGeneration.processTriangleCache<CacheSize, PCMConvexVsMeshContactGeneration>(cache);
	}
	
};

bool Gu::PCMContactConvexHeightfield(
	const Gu::PolygonalData& polyData, Gu::SupportLocal* polyMap, const Ps::aos::FloatVArg minMargin,
	const PxBounds3& hullAABB, const PxHeightFieldGeometry& shapeHeightfield,
	const PxTransform& transform0, const PxTransform& transform1,
	PxReal contactDistance, Gu::ContactBuffer& contactBuffer,
	const Cm::FastVertex2ShapeScaling& convexScaling, bool idtConvexScale,
	Gu::MultiplePersistentContactManifold& multiManifold, Cm::RenderOutput* renderOutput)

{

	using namespace Ps::aos;
	using namespace Gu;

	const QuatV q0 = QuatVLoadA(&transform0.q.x);
	const Vec3V p0 = V3LoadA(&transform0.p.x);

	const QuatV q1 = QuatVLoadA(&transform1.q.x);
	const Vec3V p1 = V3LoadA(&transform1.p.x);

	const FloatV contactDist = FLoad(contactDistance);
	//Transfer A into the local space of B
	const PsTransformV convexTransform(p0, q0);//box
	const PsTransformV heightfieldTransform(p1, q1);//heightfield  
	const PsTransformV curTransform = heightfieldTransform.transformInv(convexTransform);
	

	if(multiManifold.invalidate(curTransform, minMargin))
	{
		const FloatV replaceBreakingThreshold = FMul(minMargin, FLoad(0.05f));
		multiManifold.mNumManifolds = 0;
		multiManifold.setRelativeTransform(curTransform); 

		////////////////////

		const PxTransform t0to1 = transform1.transformInv(transform0);
		
		Gu::HeightFieldUtil hfUtil(shapeHeightfield);
		const Gu::HeightField& hf = hfUtil.getHeightField();

		////////////////////

		/*const Cm::Matrix34 world0(transform0);
		const Cm::Matrix34 world1(transform1);

		const PxU8* PX_RESTRICT extraData = meshData->mExtraTrigData;*/

	    Ps::InlineArray<PxU32,LOCAL_CONTACTS_SIZE> delayedContacts;
			
		PCMConvexVsHeightfieldContactGenerationCallback blockCallback(
			contactDist,
			replaceBreakingThreshold,
			polyData,
			polyMap, 
			convexScaling, 
			idtConvexScale,
			convexTransform, 
			heightfieldTransform,
			transform1,
			multiManifold,
			contactBuffer,
			hfUtil,
			&delayedContacts,
			!(hf.getFlags() & PxHeightFieldFlag::eNO_BOUNDARY_EDGES),
			renderOutput
		);

		hfUtil.overlapAABBTriangles(transform1, PxBounds3::transformFast(t0to1, hullAABB), 0, &blockCallback);

		PX_ASSERT(multiManifold.mNumManifolds <= GU_MAX_MANIFOLD_SIZE);
		blockCallback.mGeneration.generateLastContacts();
		blockCallback.mGeneration.processContacts(GU_SINGLE_MANIFOLD_CACHE_SIZE, false);
	}
	else
	{
		const PsMatTransformV aToB(curTransform);

		const FloatV projectBreakingThreshold = FMul(minMargin, FLoad(0.6f));
		multiManifold.refreshManifold(aToB, projectBreakingThreshold, contactDist);
	}

#if PCM_LOW_LEVEL_DEBUG
	 multiManifold.drawManifold(*renderOutput, convexTransform, heightfieldTransform);
#endif
	return multiManifold.addManifoldContactsToContactBuffer(contactBuffer, heightfieldTransform);

}


bool Gu::pcmContactConvexHeightField(GU_CONTACT_METHOD_ARGS)
{
	using namespace Ps::aos;
	
	const PxConvexMeshGeometryLL& shapeConvex = shape0.get<const PxConvexMeshGeometryLL>();
	const physx::PxHeightFieldGeometryLL& shapHeightField = shape1.get<const PxHeightFieldGeometryLL>();

	const Gu::ConvexHullData* hullData = shapeConvex.hullData;
	Gu::MultiplePersistentContactManifold& multiManifold = cache.getMultipleManifold();

	const QuatV q0 = QuatVLoadA(&transform0.q.x);
	const Vec3V p0 = V3LoadA(&transform0.p.x);

	const PsTransformV convexTransform(p0, q0);

	//const bool idtScaleMesh = shapeMesh.scale.isIdentity();

	//Cm::FastVertex2ShapeScaling meshScaling;
	//if(!idtScaleMesh)
	//	meshScaling.init(shapeMesh.scale);

	Cm::FastVertex2ShapeScaling convexScaling;
	PxBounds3 hullAABB;
	PolygonalData polyData;
	const bool idtScaleConvex = getPCMConvexData(shape0, convexScaling, hullAABB, polyData);

	const Vec3V vScale = V3LoadU_SafeReadW(shapeConvex.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
	
	const PxReal toleranceLength = params.mToleranceLength;
	const FloatV minMargin = Gu::CalculatePCMConvexMargin(hullData, vScale, toleranceLength, GU_PCM_MESH_MANIFOLD_EPSILON);

	const QuatV vQuat = QuatVLoadU(&shapeConvex.scale.rotation.x);
	Gu::ConvexHullV convexHull(hullData, V3Zero(), vScale, vQuat, shapeConvex.scale.isIdentity());

	if(idtScaleConvex)
	{
		SupportLocalImpl<Gu::ConvexHullNoScaleV> convexMap(static_cast<ConvexHullNoScaleV&>(convexHull), convexTransform, convexHull.vertex2Shape, convexHull.shape2Vertex, idtScaleConvex);
		return Gu::PCMContactConvexHeightfield(polyData, &convexMap, minMargin, hullAABB, shapHeightField, transform0, transform1, params.mContactDistance, contactBuffer, convexScaling, 
			idtScaleConvex, multiManifold, renderOutput);
	}
	else
	{
		SupportLocalImpl<Gu::ConvexHullV> convexMap(convexHull, convexTransform, convexHull.vertex2Shape, convexHull.shape2Vertex, idtScaleConvex);
		return Gu::PCMContactConvexHeightfield(polyData, &convexMap, minMargin, hullAABB, shapHeightField, transform0, transform1, params.mContactDistance, contactBuffer, convexScaling, 
			idtScaleConvex, multiManifold, renderOutput);
	}
}  

bool Gu::pcmContactBoxHeightField(GU_CONTACT_METHOD_ARGS)
{
	using namespace Ps::aos;

	MultiplePersistentContactManifold& multiManifold = cache.getMultipleManifold();

	const PxBoxGeometry& shapeBox = shape0.get<const PxBoxGeometry>();
	const physx::PxHeightFieldGeometryLL& shapHeightField = shape1.get<const PxHeightFieldGeometryLL>();

	const PxVec3 ext = shapeBox.halfExtents + PxVec3(params.mContactDistance);
	const PxBounds3 hullAABB(-ext, ext);

	Cm::FastVertex2ShapeScaling idtScaling;

	const QuatV q0 = QuatVLoadA(&transform0.q.x);
	const Vec3V p0 = V3LoadA(&transform0.p.x);

	const Vec3V boxExtents = V3LoadU(shapeBox.halfExtents);

	const PxReal toranceLength = params.mToleranceLength;
	const FloatV minMargin = Gu::CalculatePCMBoxMargin(boxExtents, toranceLength, GU_PCM_MESH_MANIFOLD_EPSILON);

	Gu::BoxV boxV(V3Zero(), boxExtents);

	const PsTransformV boxTransform(p0, q0);//box

	Gu::PolygonalData polyData;
	Gu::PCMPolygonalBox polyBox(shapeBox.halfExtents);
	polyBox.getPolygonalData(&polyData);

	Mat33V identity =  M33Identity();
	//SupportLocalImpl<Gu::BoxV> boxMap(boxV, boxTransform, identity, identity);
	SupportLocalImpl<Gu::BoxV> boxMap(boxV, boxTransform, identity, identity, true);

	return Gu::PCMContactConvexHeightfield(polyData, &boxMap, minMargin, hullAABB, shapHeightField, transform0, transform1, params.mContactDistance, contactBuffer, 
		idtScaling, true, multiManifold, renderOutput);
}
}
