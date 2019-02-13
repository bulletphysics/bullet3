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


#include "GuGJKPenetration.h"
#include "GuEPA.h"
#include "GuVecCapsule.h"
#include "GuVecConvexHull.h"
#include "GuVecConvexHullNoScale.h"
#include "GuGeometryUnion.h"

#include "GuContactMethodImpl.h"
#include "GuContactBuffer.h"
#include "GuPCMContactGen.h"
#include "GuPCMShapeConvex.h"

namespace physx
{

using namespace Ps::aos;

namespace Gu
{

static bool fullContactsGenerationCapsuleConvex(const CapsuleV& capsule, const ConvexHullV& convexHull,  const PsMatTransformV& aToB, const PsTransformV& transf0,const PsTransformV& transf1,
								PersistentContact* manifoldContacts, ContactBuffer& contactBuffer, const bool idtScale, PersistentContactManifold& manifold, Vec3VArg normal, 
								const Vec3VArg closest, const PxReal tolerance, const FloatVArg contactDist, const bool doOverlapTest, Cm::RenderOutput* renderOutput, const PxReal toleranceLength)
{

	PX_UNUSED(renderOutput);
	Gu::PolygonalData polyData;
	getPCMConvexData(convexHull,idtScale, polyData);

	PxU8 buff[sizeof(SupportLocalImpl<ConvexHullV>)];
	SupportLocal* map = (idtScale ? static_cast<SupportLocal*>(PX_PLACEMENT_NEW(buff, SupportLocalImpl<ConvexHullNoScaleV>)(static_cast<const ConvexHullNoScaleV&>(convexHull), transf1, convexHull.vertex2Shape, convexHull.shape2Vertex, idtScale)) : 
	static_cast<SupportLocal*>(PX_PLACEMENT_NEW(buff, SupportLocalImpl<ConvexHullV>)(convexHull, transf1, convexHull.vertex2Shape, convexHull.shape2Vertex, idtScale)));

	PxU32 numContacts = 0;
	if (generateFullContactManifold(capsule, polyData, map, aToB, manifoldContacts, numContacts, contactDist, normal, closest, tolerance, doOverlapTest, toleranceLength))
	{

		if (numContacts > 0)
		{
			manifold.addBatchManifoldContacts2(manifoldContacts, numContacts);
			//transform normal into the world space
			normal = transf1.rotate(normal);
			manifold.addManifoldContactsToContactBuffer(contactBuffer, normal, normal, transf0, capsule.radius, contactDist);
		}
		else
		{
			if (!doOverlapTest)
			{
				normal = transf1.rotate(normal);
				manifold.addManifoldContactsToContactBuffer(contactBuffer, normal, normal, transf0, capsule.radius, contactDist);
			}
		}

#if	PCM_LOW_LEVEL_DEBUG
		manifold.drawManifold(*renderOutput, transf0, transf1);
#endif
		return true;
		
	}
	return false;

}

bool pcmContactCapsuleConvex(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);


	const PxConvexMeshGeometryLL& shapeConvex = shape1.get<const PxConvexMeshGeometryLL>();
	const PxCapsuleGeometry& shapeCapsule = shape0.get<const PxCapsuleGeometry>();

	PersistentContactManifold& manifold = cache.getManifold();

	Ps::prefetchLine(shapeConvex.hullData);

		
	PX_ASSERT(transform1.q.isSane());
	PX_ASSERT(transform0.q.isSane());

	const Vec3V vScale = V3LoadU_SafeReadW(shapeConvex.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale

	const FloatV contactDist = FLoad(params.mContactDistance);
	const FloatV capsuleHalfHeight = FLoad(shapeCapsule.halfHeight);
	const FloatV capsuleRadius = FLoad(shapeCapsule.radius);
	const ConvexHullData* hullData =shapeConvex.hullData;
	
	//Transfer A into the local space of B
	const PsTransformV transf0 = loadTransformA(transform0);
	const PsTransformV transf1 = loadTransformA(transform1);
	const PsTransformV curRTrans(transf1.transformInv(transf0));
	const PsMatTransformV aToB(curRTrans);
	
	const PxReal toleranceLength = params.mToleranceLength ;
	const FloatV convexMargin = Gu::CalculatePCMConvexMargin(hullData, vScale, toleranceLength);
	const FloatV capsuleMinMargin = Gu::CalculateCapsuleMinMargin(capsuleRadius);
	const FloatV minMargin = FMin(convexMargin, capsuleMinMargin);
	
	const PxU32 initialContacts = manifold.mNumContacts;
	const FloatV projectBreakingThreshold = FMul(minMargin, FLoad(1.25f));
	const FloatV refreshDist = FAdd(contactDist, capsuleRadius);

	manifold.refreshContactPoints(aToB,  projectBreakingThreshold, refreshDist);

	//ML: after refreshContactPoints, we might lose some contacts
	const bool bLostContacts = (manifold.mNumContacts != initialContacts);

	GjkStatus status = manifold.mNumContacts > 0 ? GJK_UNDEFINED : GJK_NON_INTERSECT;

	PX_UNUSED(bLostContacts);
	if(bLostContacts || manifold.invalidate_SphereCapsule(curRTrans, minMargin))
	{
		const bool idtScale = shapeConvex.scale.isIdentity();

		manifold.setRelativeTransform(curRTrans);
		const QuatV vQuat = QuatVLoadU(&shapeConvex.scale.rotation.x);  
		ConvexHullV convexHull(hullData, V3LoadU(hullData->mCenterOfMass), vScale, vQuat, idtScale);
	
		//transform capsule(a) into the local space of convexHull(b)
		CapsuleV capsule(aToB.p, aToB.rotate(V3Scale(V3UnitX(), capsuleHalfHeight)), capsuleRadius);
	
		GjkOutput output;
		LocalConvex<CapsuleV> convexA(capsule);
		const Vec3V initialSearchDir = V3Sub(capsule.getCenter(), convexHull.getCenter());
		if(idtScale)
		{
			LocalConvex<ConvexHullNoScaleV> convexB(*PX_CONVEX_TO_NOSCALECONVEX(&convexHull));

			status = gjkPenetration<LocalConvex<CapsuleV>, LocalConvex<ConvexHullNoScaleV> >(convexA, convexB, initialSearchDir, contactDist, true,
				manifold.mAIndice, manifold.mBIndice, manifold.mNumWarmStartPoints, output);
		}
		else
		{
			LocalConvex<ConvexHullV> convexB(convexHull);
			status = gjkPenetration<LocalConvex<CapsuleV>, LocalConvex<ConvexHullV> >(convexA, convexB, initialSearchDir, contactDist, true,
				manifold.mAIndice, manifold.mBIndice, manifold.mNumWarmStartPoints, output);

		}     

		Gu::PersistentContact* manifoldContacts = PX_CP_TO_PCP(contactBuffer.contacts);
		bool doOverlapTest = false;
		if(status == GJK_NON_INTERSECT)
		{
			return false;
		}
		else if(status == GJK_DEGENERATE)
		{
			return fullContactsGenerationCapsuleConvex(capsule, convexHull, aToB, transf0, transf1, manifoldContacts, contactBuffer, idtScale, manifold, output.normal, 
				output.closestB, convexHull.getMarginF(), contactDist, true, renderOutput, toleranceLength);
		}
		else 
		{
			const FloatV replaceBreakingThreshold = FMul(minMargin, FLoad(0.05f));

			if(status == GJK_CONTACT)
			{
				const Vec3V localPointA = aToB.transformInv(output.closestA);//curRTrans.transformInv(closestA);
				const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(output.normal), output.penDep);
				//Add contact to contact stream
				manifoldContacts[0].mLocalPointA = localPointA;
				manifoldContacts[0].mLocalPointB = output.closestB;
				manifoldContacts[0].mLocalNormalPen = localNormalPen;

				//Add contact to manifold
				manifold.addManifoldPoint2(localPointA, output.closestB, localNormalPen, replaceBreakingThreshold);
			}
			else
			{
				PX_ASSERT(status == EPA_CONTACT);
				
				if(idtScale)
				{
					LocalConvex<ConvexHullNoScaleV> convexB(*PX_CONVEX_TO_NOSCALECONVEX(&convexHull));

					status= Gu::epaPenetration(convexA, convexB, manifold.mAIndice, manifold.mBIndice, manifold.mNumWarmStartPoints,
					 true, FLoad(toleranceLength), output);
				}
				else
				{
					LocalConvex<ConvexHullV> convexB(convexHull);
					status= Gu::epaPenetration(convexA, convexB,  manifold.mAIndice, manifold.mBIndice, manifold.mNumWarmStartPoints,
					true, FLoad(toleranceLength), output);
				}
				
				
				if(status == EPA_CONTACT)
				{
					const Vec3V localPointA = aToB.transformInv(output.closestA);//curRTrans.transformInv(closestA);
					const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(output.normal), output.penDep);
					//Add contact to contact stream
					manifoldContacts[0].mLocalPointA = localPointA;
					manifoldContacts[0].mLocalPointB = output.closestB;
					manifoldContacts[0].mLocalNormalPen = localNormalPen;

					//Add contact to manifold
					manifold.addManifoldPoint2(localPointA, output.closestB, localNormalPen, replaceBreakingThreshold);
					

				}
				else
				{
					doOverlapTest = true;   
				}
			}

		
			if(initialContacts == 0 || bLostContacts || doOverlapTest)
			{
				return fullContactsGenerationCapsuleConvex(capsule, convexHull, aToB, transf0, transf1, manifoldContacts, contactBuffer, idtScale, manifold, output.normal, 
					output.closestB, convexHull.getMarginF(), contactDist, doOverlapTest, renderOutput, toleranceLength);
			}
			else
			{
				//This contact is either come from GJK or EPA
				const Vec3V normal = transf1.rotate(output.normal);
				manifold.addManifoldContactsToContactBuffer(contactBuffer, normal, normal, transf0, capsuleRadius, contactDist);
#if	PCM_LOW_LEVEL_DEBUG
				manifold.drawManifold(*renderOutput, transf0, transf1);
#endif
				return true;
			}
		}	
	}
	else if (manifold.getNumContacts() > 0)
	{
		const Vec3V normal = manifold.getWorldNormal(transf1);
		manifold.addManifoldContactsToContactBuffer(contactBuffer, normal, normal, transf0, capsuleRadius, contactDist);
#if	PCM_LOW_LEVEL_DEBUG
		manifold.drawManifold(*renderOutput, transf0, transf1);
#endif
		return true;
	}
	return false;
}

}//Gu
}//physx
