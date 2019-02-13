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


#include "GuVecConvexHull.h"
#include "GuGeometryUnion.h"

#include "GuContactMethodImpl.h"
#include "GuContactBuffer.h"
#include "GuPersistentContactManifold.h"


namespace physx
{
namespace Gu
{
bool pcmContactPlaneConvex(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(shape0);
	PX_UNUSED(renderOutput);

	using namespace Ps::aos;

	Gu::PersistentContactManifold& manifold = cache.getManifold();
	Ps::prefetchLine(&manifold, 256);

	// Get actual shape data
	const PxConvexMeshGeometryLL& shapeConvex = shape1.get<const PxConvexMeshGeometryLL>();

	const PsTransformV transf0 = loadTransformA(transform1);//convex transform
	const PsTransformV transf1 = loadTransformA(transform0);//plane transform
	//convex to plane
	const PsTransformV curTransf(transf1.transformInv(transf0));
	
	const Vec3V vScale = V3LoadU_SafeReadW(shapeConvex.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
	const Gu::ConvexHullData* hullData = shapeConvex.hullData;

	const PxReal toleranceLength = params.mToleranceLength;
	const FloatV convexMargin = Gu::CalculatePCMConvexMargin(hullData, vScale, toleranceLength);
	
	//in world space
	const Vec3V planeNormal = V3Normalize(QuatGetBasisVector0(transf1.q));
	const Vec3V negPlaneNormal = V3Neg(planeNormal);
	
	const FloatV contactDist = FLoad(params.mContactDistance);

	//const FloatV replaceBreakingThreshold = FMul(convexMargin, FLoad(0.001f));
	const FloatV projectBreakingThreshold = FMul(convexMargin, FLoad(0.2f));
	const PxU32 initialContacts = manifold.mNumContacts;
	
	manifold.refreshContactPoints(curTransf, projectBreakingThreshold, contactDist);

	const PxU32 newContacts = manifold.mNumContacts;
	const bool bLostContacts = (newContacts != initialContacts);//((initialContacts == 0) || (newContacts != initialContacts));

	
	if(bLostContacts || manifold.invalidate_PrimitivesPlane(curTransf, convexMargin, FLoad(0.2f)))
	{
		const PsMatTransformV aToB(curTransf);
		const QuatV vQuat = QuatVLoadU(&shapeConvex.scale.rotation.x);

		const Mat33V vertex2Shape = ConstructVertex2ShapeMatrix(vScale, vQuat);
		
		//ML:localNormal is the local space of plane normal, however, because shape1 is box and shape0 is plane, we need to use the reverse of contact normal(which will be the plane normal) to make the refreshContactPoints
		//work out the correct pentration for points
		const Vec3V localNormal = V3UnitX();

		manifold.mNumContacts = 0;
		manifold.setRelativeTransform(curTransf);
		const PxVec3* PX_RESTRICT verts = hullData->getHullVertices();
		const PxU8 numVerts = hullData->mNbHullVertices;

		Gu::PersistentContact* manifoldContacts = PX_CP_TO_PCP(contactBuffer.contacts);
		PxU32 numContacts = 0;
		
		const PsMatTransformV aToBVertexSpace(aToB.p, M33MulM33(aToB.rot, vertex2Shape));
		//brute force each points
		for(PxU8 i=0; i<numVerts; ++i)
		{
			//in the vertex space of convex
			const Vec3V pInVertexSpace = V3LoadU(verts[i]);
			
			//transform p into plane space
			const Vec3V pInPlaneSpace = aToBVertexSpace.transform(pInVertexSpace);//V3Add(aToB.p, M33MulV3(temp1, pInVertexSpace));
		
			const FloatV signDist = V3GetX(pInPlaneSpace);
			
			if(FAllGrtr(contactDist, signDist))
			{
				//transform p into shape space
				const Vec3V pInShapeSpace = M33MulV3(vertex2Shape, pInVertexSpace);
				//add to manifold
				
				manifoldContacts[numContacts].mLocalPointA = pInShapeSpace;
				manifoldContacts[numContacts].mLocalPointB = V3NegScaleSub(localNormal, signDist, pInPlaneSpace); 
				manifoldContacts[numContacts++].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(localNormal), signDist);

				if(numContacts >= Gu::ContactBuffer::MAX_CONTACTS)
				{
					//ML: number of contacts are more than MAX_CONTACTS, we need to force contact reduction
					manifold.reduceBatchContacts(manifoldContacts, numContacts, toleranceLength);
					numContacts = GU_MANIFOLD_CACHE_SIZE;
					for(PxU32 j=0; j<GU_MANIFOLD_CACHE_SIZE; ++j)
					{
						manifoldContacts[j] = manifold.mContactPoints[j];
					}
				}
			}
		}

		//reduce contacts
		//manifold.addBatchManifoldContactsCluster(manifoldContacts, numContacts);

		manifold.addBatchManifoldContacts(manifoldContacts, numContacts, toleranceLength);
		manifold.addManifoldContactsToContactBuffer(contactBuffer, negPlaneNormal, transf1, contactDist);
#if	PCM_LOW_LEVEL_DEBUG
		manifold.drawManifold(*renderOutput, transf0, transf1);
#endif

		return manifold.getNumContacts() > 0;
	}
	else
	{
		manifold.addManifoldContactsToContactBuffer(contactBuffer, negPlaneNormal, transf1, contactDist);
#if	PCM_LOW_LEVEL_DEBUG
		manifold.drawManifold(*renderOutput, transf0, transf1);
#endif
		return manifold.getNumContacts() > 0;
	}	
}

}//Gu
}//physx
