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

#include "GuGJKPenetration.h"
#include "GuEPA.h"
#include "GuVecBox.h"
#include "GuVecConvexHull.h"
#include "GuVecConvexHullNoScale.h"
#include "GuGeometryUnion.h"

#include "GuContactMethodImpl.h"
#include "GuPCMContactGen.h"
#include "GuPCMShapeConvex.h"
#include "GuContactBuffer.h"



#define PCM_BOX_HULL_DEBUG 0

namespace physx
{

using namespace Ps::aos;

namespace Gu
{

static bool fullContactsGenerationBoxConvex(const GjkConvex* relativeConvex, const GjkConvex* localConvex, const PsTransformV& transf0, const PsTransformV& transf1,
									PersistentContact* manifoldContacts, ContactBuffer& contactBuffer, Gu::PersistentContactManifold& manifold, Vec3VArg normal, 
									const Vec3VArg closestA, const Vec3VArg closestB, const FloatVArg contactDist, const bool idtScale, const bool doOverlapTest, Cm::RenderOutput* renderOutput,
									const PxReal toleranceLength)
{
	Gu::PolygonalData polyData0;

	const BoxV& box = relativeConvex->getConvex<BoxV>();
	const ConvexHullV& convexHull = localConvex->getConvex<ConvexHullV>();

	PxVec3 halfExtents;
	V3StoreU(box.extents, halfExtents);
	PCMPolygonalBox polyBox0(halfExtents);
	polyBox0.getPolygonalData(&polyData0);
	polyData0.mPolygonVertexRefs = gPCMBoxPolygonData;
	
	Gu::PolygonalData polyData1;
	getPCMConvexData(convexHull, idtScale, polyData1);
	
	Mat33V identity =  M33Identity();
	SupportLocalImpl<BoxV> map0(box, transf0, identity, identity, true);

	PxU8 buff1[sizeof(SupportLocalImpl<ConvexHullV>)];

	SupportLocal* map1 = (idtScale ? static_cast<SupportLocal*>(PX_PLACEMENT_NEW(buff1, SupportLocalImpl<ConvexHullNoScaleV>)(static_cast<const ConvexHullNoScaleV&>(convexHull), transf1, convexHull.vertex2Shape, convexHull.shape2Vertex, idtScale)) : 
		static_cast<SupportLocal*>(PX_PLACEMENT_NEW(buff1, SupportLocalImpl<ConvexHullV>)(convexHull, transf1, convexHull.vertex2Shape, convexHull.shape2Vertex, idtScale)));

	PxU32 numContacts = 0;
	if(generateFullContactManifold(polyData0, polyData1, &map0, map1, manifoldContacts, numContacts, contactDist, normal, closestA, closestB, box.getMarginF(), convexHull.getMarginF(), 
		doOverlapTest, renderOutput, toleranceLength))
	{
		if (numContacts > 0)
		{
			//reduce contacts
			manifold.addBatchManifoldContacts(manifoldContacts, numContacts, toleranceLength);

#if	PCM_LOW_LEVEL_DEBUG
			manifold.drawManifold(*renderOutput, transf0, transf1);
#endif

			const Vec3V worldNormal = manifold.getWorldNormal(transf1);

			manifold.addManifoldContactsToContactBuffer(contactBuffer, worldNormal, transf1, contactDist);
		}
		else
		{
			//if doOverlapTest is true, which means GJK/EPA degenerate so we won't have any contact in the manifoldContacts array
			if (!doOverlapTest)
			{
				const Vec3V worldNormal = manifold.getWorldNormal(transf1);

				manifold.addManifoldContactsToContactBuffer(contactBuffer, worldNormal, transf1, contactDist);
			}
		}
		
		return true;
	}

	return false;

}

static bool generateOrProcessContactsBoxConvex(const GjkConvex* relativeConvex, const GjkConvex* localConvex, const PsTransformV& transf0, const PsTransformV& transf1, 
	const PsMatTransformV& aToB, GjkStatus status, GjkOutput& output, PersistentContactManifold& manifold, ContactBuffer& contactBuffer, 
	const PxU32 initialContacts, const FloatV minMargin,  const FloatV contactDist,
	const bool idtScale, const PxReal toleranceLength, Cm::RenderOutput* renderOutput)
{

	if (status == GJK_NON_INTERSECT)
	{
		return false;
	}
	else
	{
		Gu::PersistentContact* manifoldContacts = PX_CP_TO_PCP(contactBuffer.contacts);

		const Vec3V localNor = manifold.mNumContacts ? manifold.getLocalNormal() : V3Zero();

		const FloatV replaceBreakingThreshold = FMul(minMargin, FLoad(0.05f));

		//addGJKEPAContacts will increase the number of contacts in manifold. If status == EPA_CONTACT, we need to run epa algorithm and generate closest points, normal and
		//pentration. If epa doesn't degenerate, we will store the contacts information in the manifold. Otherwise, we will return true to do the fallback test
		const bool doOverlapTest = addGJKEPAContacts(relativeConvex, localConvex, aToB, status, manifoldContacts, replaceBreakingThreshold, FLoad(toleranceLength), output, manifold);

#if PCM_LOW_LEVEL_DEBUG
		manifold.drawManifold(*renderOutput, transf0, transf1);
#endif
		//ML: after we refresh the contacts(newContacts) and generate a GJK/EPA contacts(we will store that in the manifold), if the number of contacts is still less than the original contacts,
		//which means we lose too mang contacts and we should regenerate all the contacts in the current configuration
		//Also, we need to look at the existing contacts, if the existing contacts has very different normal than the GJK/EPA contacts,
		//which means we should throw away the existing contacts and do full contact gen
		const bool fullContactGen = FAllGrtr(FLoad(0.707106781f), V3Dot(localNor, output.normal)) || (manifold.mNumContacts < initialContacts);

		if (fullContactGen || doOverlapTest)
		{
			return fullContactsGenerationBoxConvex(relativeConvex, localConvex, transf0, transf1, manifoldContacts, contactBuffer,
				manifold, output.normal, output.closestA, output.closestB, contactDist, idtScale, doOverlapTest, renderOutput, toleranceLength);
		}
		else
		{
			const Vec3V newLocalNor = V3Add(localNor, output.normal);
			const Vec3V worldNormal = V3Normalize(transf1.rotate(newLocalNor));
			//const Vec3V worldNormal = transf1.rotate(normal);
			manifold.addManifoldContactsToContactBuffer(contactBuffer, worldNormal, transf1, contactDist);
			return true;
		}
	}
}

bool pcmContactBoxConvex(GU_CONTACT_METHOD_ARGS)
{
	using namespace Ps::aos;
	
	const PxConvexMeshGeometryLL& shapeConvex = shape1.get<const PxConvexMeshGeometryLL>();
	const PxBoxGeometry& shapeBox = shape0.get<const PxBoxGeometry>();
	
	Gu::PersistentContactManifold& manifold = cache.getManifold();
	Ps::prefetchLine(shapeConvex.hullData);
	
	PX_ASSERT(transform1.q.isSane());
	PX_ASSERT(transform0.q.isSane());

	const FloatV contactDist = FLoad(params.mContactDistance);
	const Vec3V boxExtents = V3LoadU(shapeBox.halfExtents);

	const Vec3V vScale = V3LoadU_SafeReadW(shapeConvex.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
	
	const PsTransformV transf0 = loadTransformA(transform0);
	const PsTransformV transf1 = loadTransformA(transform1);
	const PsTransformV curRTrans(transf1.transformInv(transf0));
	const PsMatTransformV aToB(curRTrans);

	const PxReal toleranceLength = params.mToleranceLength;
	const Gu::ConvexHullData* hullData = shapeConvex.hullData;
	const FloatV convexMargin = Gu::CalculatePCMConvexMargin(hullData, vScale, toleranceLength);
	const FloatV boxMargin = Gu::CalculatePCMBoxMargin(boxExtents, toleranceLength);


#if PCM_BOX_HULL_DEBUG
	const PxVec3* verts = hullData->getHullVertices();
	
	for (PxU32 i = 0; i < hullData->mNbPolygons; ++i)
	{
		const HullPolygonData& polygon = hullData->mPolygons[i];
		const PxU8* inds = hullData->getVertexData8() + polygon.mVRef8;
		Vec3V* points = reinterpret_cast<Vec3V*>(PxAllocaAligned(sizeof(Vec3V)*polygon.mNbVerts, 16));
		
		for (PxU32 j = 0; j < polygon.mNbVerts; ++j)
		{
			points[j] = V3LoadU_SafeReadW(verts[inds[j]]);
		}

		Gu::PersistentContactManifold::drawPolygon(*renderOutput, transf1, points, (PxU32)polygon.mNbVerts, 0x00ff0000);
	}
#endif

	const FloatV minMargin = FMin(convexMargin, boxMargin);//FMin(boxMargin, convexMargin);
	const FloatV projectBreakingThreshold = FMul(minMargin, FLoad(0.8f));
	const PxU32 initialContacts = manifold.mNumContacts;

	manifold.refreshContactPoints(aToB, projectBreakingThreshold, contactDist);  
	
	//After the refresh contact points, the numcontacts in the manifold will be changed
	const bool bLostContacts = (manifold.mNumContacts != initialContacts);

	if(bLostContacts || manifold.invalidate_BoxConvex(curRTrans, minMargin))	
	{
		
		manifold.setRelativeTransform(curRTrans);
	
		GjkStatus status = manifold.mNumContacts > 0 ? GJK_UNDEFINED : GJK_NON_INTERSECT;

		const QuatV vQuat = QuatVLoadU(&shapeConvex.scale.rotation.x);
		const bool idtScale = shapeConvex.scale.isIdentity();
		Gu::ConvexHullV convexHull(hullData, V3LoadU(hullData->mCenterOfMass), vScale, vQuat, idtScale);
		Gu::BoxV box(V3Zero(), boxExtents);
		GjkOutput output;
		
		RelativeConvex<BoxV> relativeConvex(box, aToB);

		if(idtScale)
		{

			LocalConvex<ConvexHullNoScaleV> localConvex(static_cast<ConvexHullNoScaleV&>(convexHull));
			
			status = gjkPenetration<RelativeConvex<BoxV>, LocalConvex<ConvexHullNoScaleV> >(relativeConvex, localConvex, aToB.p, contactDist, true,
				manifold.mAIndice, manifold.mBIndice, manifold.mNumWarmStartPoints, output);

			return generateOrProcessContactsBoxConvex(&relativeConvex, &localConvex, transf0, transf1, aToB,
				status, output, manifold, contactBuffer, initialContacts,
				minMargin, contactDist, idtScale, toleranceLength, renderOutput);
	
		}
		else
		{
			LocalConvex<ConvexHullV> localConvex(convexHull);

			status = gjkPenetration<RelativeConvex<BoxV>, LocalConvex<ConvexHullV> >(relativeConvex, localConvex, aToB.p, contactDist, true,
				manifold.mAIndice, manifold.mBIndice, manifold.mNumWarmStartPoints, output);

			return generateOrProcessContactsBoxConvex(&relativeConvex, &localConvex, transf0, transf1, aToB,
				status, output, manifold, contactBuffer, initialContacts,
				minMargin, contactDist, idtScale, toleranceLength, renderOutput);
		} 
	}
	else if(manifold.getNumContacts()>0)
	{
		const Vec3V worldNormal =  manifold.getWorldNormal(transf1);
		manifold.addManifoldContactsToContactBuffer(contactBuffer, worldNormal, transf1, contactDist);
#if	PCM_LOW_LEVEL_DEBUG
		manifold.drawManifold(*renderOutput, transf0, transf1);
#endif
		return true;
	}

	return false;

}

}//Gu
}//physx
