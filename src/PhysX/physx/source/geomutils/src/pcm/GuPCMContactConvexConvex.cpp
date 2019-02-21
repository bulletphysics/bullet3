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
#include "GuVecConvexHull.h"
#include "GuVecConvexHullNoScale.h"
#include "GuGeometryUnion.h"   

#include "GuContactMethodImpl.h"
#include "GuPCMShapeConvex.h"
#include "GuPCMContactGen.h"
#include "GuContactBuffer.h"


namespace physx
{
using namespace Ps::aos;

namespace Gu
{

static bool fullContactsGenerationConvexConvex(const GjkConvex* relativeConvex, const GjkConvex* localConvex, const PsTransformV& transf0, const PsTransformV& transf1, 
											   const bool idtScale0, const bool idtScale1, PersistentContact* manifoldContacts,  ContactBuffer& contactBuffer, 
											   PersistentContactManifold& manifold, Vec3VArg normal, const Vec3VArg closestA, const Vec3VArg closestB,
											   const FloatVArg contactDist, const bool doOverlapTest, Cm::RenderOutput* renderOutput, const PxReal toleranceLength)
{
	Gu::PolygonalData polyData0, polyData1;
	const ConvexHullV& convexHull0 = relativeConvex->getConvex<ConvexHullV>();
	const ConvexHullV& convexHull1 = localConvex->getConvex<ConvexHullV>();
	getPCMConvexData(convexHull0, idtScale0, polyData0);
	getPCMConvexData(convexHull1, idtScale1, polyData1);

	PxU8 buff0[sizeof(SupportLocalImpl<ConvexHullV>)];
	PxU8 buff1[sizeof(SupportLocalImpl<ConvexHullV>)];

	SupportLocal* map0 = (idtScale0 ? static_cast<SupportLocal*>(PX_PLACEMENT_NEW(buff0, SupportLocalImpl<ConvexHullNoScaleV>)(static_cast<const ConvexHullNoScaleV&>(convexHull0), transf0, convexHull0.vertex2Shape, convexHull0.shape2Vertex, idtScale0)) : 
		static_cast<SupportLocal*>(PX_PLACEMENT_NEW(buff0, SupportLocalImpl<ConvexHullV>)(convexHull0, transf0, convexHull0.vertex2Shape, convexHull0.shape2Vertex, idtScale0)));

	SupportLocal* map1 = (idtScale1 ? static_cast<SupportLocal*>(PX_PLACEMENT_NEW(buff1, SupportLocalImpl<ConvexHullNoScaleV>)(static_cast<const ConvexHullNoScaleV&>(convexHull1), transf1, convexHull1.vertex2Shape, convexHull1.shape2Vertex, idtScale1)) : 
		static_cast<SupportLocal*>(PX_PLACEMENT_NEW(buff1, SupportLocalImpl<ConvexHullV>)(convexHull1, transf1, convexHull1.vertex2Shape, convexHull1.shape2Vertex, idtScale1)));


	PxU32 numContacts = 0;

	if(generateFullContactManifold(polyData0, polyData1, map0, map1, manifoldContacts, numContacts, contactDist, normal, closestA, closestB, convexHull0.getMarginF(), 
		convexHull1.getMarginF(), doOverlapTest, renderOutput, toleranceLength))
	{
		
		if (numContacts > 0)
		{
			//reduce contacts
			manifold.addBatchManifoldContacts(manifoldContacts, numContacts, toleranceLength);

			const Vec3V worldNormal = manifold.getWorldNormal(transf1);

			//add the manifold contacts;
			manifold.addManifoldContactsToContactBuffer(contactBuffer, worldNormal, transf1, contactDist);


#if	PCM_LOW_LEVEL_DEBUG
			manifold.drawManifold(*renderOutput, transf0, transf1);
#endif
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

static bool generateOrProcessContactsConvexConvex(const GjkConvex* relativeConvex, const GjkConvex* localConvex, const PsTransformV& transf0, const PsTransformV& transf1,
	const PsMatTransformV& aToB, GjkStatus status, GjkOutput& output, PersistentContactManifold& manifold, ContactBuffer& contactBuffer,
	const PxU32 initialContacts, const FloatV minMargin, const FloatV contactDist,
	const bool idtScale0, const bool idtScale1, const PxReal toleranceLength, Cm::RenderOutput* renderOutput)
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
			return fullContactsGenerationConvexConvex(relativeConvex, localConvex, transf0, transf1, idtScale0, idtScale1, manifoldContacts, contactBuffer,
				manifold, output.normal, output.closestA, output.closestB, contactDist, doOverlapTest, renderOutput, toleranceLength);
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

static bool convexHullNoScale0(const ConvexHullV& convexHull0, const ConvexHullV& convexHull1, const PsTransformV& transf0, const PsTransformV& transf1,
	const PsMatTransformV& aToB, GjkOutput& output, PersistentContactManifold& manifold, ContactBuffer& contactBuffer,
	const PxU32 initialContacts, const FloatV minMargin, const FloatV contactDist,
	const bool idtScale1, const PxReal toleranceLength, Cm::RenderOutput* renderOutput)
{

	const RelativeConvex<ConvexHullNoScaleV> convexA(static_cast<const ConvexHullNoScaleV&>(convexHull0), aToB);
	if(idtScale1)
	{
		const LocalConvex<ConvexHullNoScaleV> convexB(static_cast<const ConvexHullNoScaleV&>(convexHull1));
		GjkStatus status = gjkPenetration<RelativeConvex<ConvexHullNoScaleV>, LocalConvex<ConvexHullNoScaleV> >(convexA, convexB, aToB.p, contactDist, true,
						manifold.mAIndice, manifold.mBIndice, manifold.mNumWarmStartPoints, output);

		return generateOrProcessContactsConvexConvex(&convexA, &convexB, transf0, transf1, aToB, status, output, manifold,
			contactBuffer, initialContacts, minMargin, contactDist, true, true, toleranceLength, renderOutput);
	
	}
	else
	{
		const LocalConvex<ConvexHullV> convexB(convexHull1);
		GjkStatus status = gjkPenetration<RelativeConvex<ConvexHullNoScaleV>, LocalConvex<ConvexHullV> >(convexA, convexB, aToB.p, contactDist, true,
					manifold.mAIndice, manifold.mBIndice, manifold.mNumWarmStartPoints, output);


		return generateOrProcessContactsConvexConvex(&convexA, &convexB, transf0, transf1, aToB, status, output, manifold,
			contactBuffer, initialContacts, minMargin, contactDist, true, false, toleranceLength, renderOutput);
	}
}

static bool convexHullHasScale0(const ConvexHullV& convexHull0, const ConvexHullV& convexHull1, const PsTransformV& transf0, const PsTransformV& transf1,
	const PsMatTransformV& aToB, GjkOutput& output, PersistentContactManifold& manifold, ContactBuffer& contactBuffer,
	const PxU32 initialContacts, const FloatV minMargin, const FloatV contactDist,
	const bool idtScale1, const PxReal toleranceLength, Cm::RenderOutput* renderOutput)
{
	
	RelativeConvex<ConvexHullV> convexA(convexHull0, aToB);
	if(idtScale1)
	{
		LocalConvex<ConvexHullNoScaleV> convexB(static_cast<const ConvexHullNoScaleV&>(convexHull1));
		GjkStatus status = gjkPenetration< RelativeConvex<ConvexHullV>, LocalConvex<ConvexHullNoScaleV> >(convexA, convexB, aToB.p, contactDist, true,
						manifold.mAIndice, manifold.mBIndice, manifold.mNumWarmStartPoints,output);
		
		return generateOrProcessContactsConvexConvex(&convexA, &convexB, transf0, transf1, aToB, status, output, manifold,
			contactBuffer, initialContacts, minMargin, contactDist, false, true, toleranceLength, renderOutput);
	
	}
	else
	{
		LocalConvex<ConvexHullV> convexB(convexHull1);
		GjkStatus status = gjkPenetration<RelativeConvex<ConvexHullV>, LocalConvex<ConvexHullV> >(convexA, convexB, aToB.p, contactDist, true,
					manifold.mAIndice, manifold.mBIndice, manifold.mNumWarmStartPoints, output);

		return generateOrProcessContactsConvexConvex(&convexA, &convexB, transf0, transf1, aToB, status, output, manifold,
			contactBuffer, initialContacts, minMargin, contactDist, false, false, toleranceLength, renderOutput);
	}
}


bool pcmContactConvexConvex(GU_CONTACT_METHOD_ARGS)
{
	const PxConvexMeshGeometryLL& shapeConvex0 = shape0.get<const PxConvexMeshGeometryLL>();
	const PxConvexMeshGeometryLL& shapeConvex1 = shape1.get<const PxConvexMeshGeometryLL>();
	PersistentContactManifold& manifold = cache.getManifold();

	Ps::prefetchLine(shapeConvex0.hullData);
	Ps::prefetchLine(shapeConvex1.hullData);

	PX_ASSERT(transform1.q.isSane());
	PX_ASSERT(transform0.q.isSane());

	const Vec3V vScale0 = V3LoadU_SafeReadW(shapeConvex0.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
	const Vec3V vScale1 = V3LoadU_SafeReadW(shapeConvex1.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
	const FloatV contactDist = FLoad(params.mContactDistance);

	//Transfer A into the local space of B
	const PsTransformV transf0 = loadTransformA(transform0);
	const PsTransformV transf1 = loadTransformA(transform1);
	const PsTransformV curRTrans(transf1.transformInv(transf0));
	const PsMatTransformV aToB(curRTrans);
	
	const Gu::ConvexHullData* hullData0 = shapeConvex0.hullData;
	const Gu::ConvexHullData* hullData1 = shapeConvex1.hullData;

	const PxReal toleranceLength = params.mToleranceLength;
	const FloatV convexMargin0 = Gu::CalculatePCMConvexMargin(hullData0, vScale0, toleranceLength);
	const FloatV convexMargin1 = Gu::CalculatePCMConvexMargin(hullData1, vScale1, toleranceLength);
	
	const PxU32 initialContacts = manifold.mNumContacts;

	const FloatV minMargin = FMin(convexMargin0, convexMargin1);
	const FloatV projectBreakingThreshold = FMul(minMargin, FLoad(0.8f));
	
	manifold.refreshContactPoints(aToB, projectBreakingThreshold, contactDist);

	//ML: after refreshContactPoints, we might lose some contacts
	const bool bLostContacts = (manifold.mNumContacts != initialContacts);

	if(bLostContacts || manifold.invalidate_BoxConvex(curRTrans, minMargin))
	{
		manifold.setRelativeTransform(curRTrans);

		const bool idtScale0 = shapeConvex0.scale.isIdentity();
		const bool idtScale1 = shapeConvex1.scale.isIdentity();
		const QuatV vQuat0 = QuatVLoadU(&shapeConvex0.scale.rotation.x);
		const QuatV vQuat1 = QuatVLoadU(&shapeConvex1.scale.rotation.x);
		
		Gu::ConvexHullV convexHull0(hullData0, V3LoadU(hullData0->mCenterOfMass), vScale0, vQuat0, idtScale0);
		Gu::ConvexHullV convexHull1(hullData1, V3LoadU(hullData1->mCenterOfMass), vScale1, vQuat1, idtScale1);

		GjkOutput output;
		
		if(idtScale0)
		{
			return convexHullNoScale0(convexHull0, convexHull1, transf0, transf1, aToB, output, manifold,
				contactBuffer, initialContacts, minMargin, contactDist, idtScale1, toleranceLength, renderOutput);
		}
		else
		{
			return convexHullHasScale0(convexHull0, convexHull1, transf0, transf1, aToB, output, manifold,
				contactBuffer, initialContacts, minMargin, contactDist, idtScale1, toleranceLength, renderOutput);
		}
	
	}
	else if(manifold.getNumContacts()> 0)
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
}
}
