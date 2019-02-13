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


#include "GuVecBox.h"
#include "GuGeometryUnion.h"
#include "GuContactMethodImpl.h"
#include "GuContactBuffer.h"

#include "GuPersistentContactManifold.h"

namespace physx
{
namespace Gu
{
bool pcmContactPlaneBox(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(shape0);
	PX_UNUSED(renderOutput);

	using namespace Ps::aos;

	Gu::PersistentContactManifold& manifold = cache.getManifold();
	Ps::prefetchLine(&manifold, 256);

	// Get actual shape data
	const PxBoxGeometry& shapeBox = shape1.get<const PxBoxGeometry>();

	const PsTransformV transf0 = loadTransformA(transform1);//box transform
	const PsTransformV transf1 = loadTransformA(transform0);//plane transform

	//box to plane
	const PsTransformV curTransf(transf1.transformInv(transf0));

	//in world space
	const Vec3V negPlaneNormal = V3Normalize(V3Neg(QuatGetBasisVector0(transf1.q)));
	
	const FloatV contactDist = FLoad(params.mContactDistance);

	const Vec3V boxExtents = V3LoadU(shapeBox.halfExtents);

	const PxReal toleranceLength = params.mToleranceLength;
	const FloatV boxMargin = CalculatePCMBoxMargin(boxExtents, toleranceLength);
	const FloatV projectBreakingThreshold = FMul(boxMargin, FLoad(0.2f));
	const PxU32 initialContacts = manifold.mNumContacts;
	
	manifold.refreshContactPoints(curTransf, projectBreakingThreshold, contactDist);

	const PxU32 newContacts = manifold.mNumContacts;
	const bool bLostContacts = (newContacts != initialContacts);//((initialContacts == 0) || (newContacts != initialContacts));

	if(bLostContacts || manifold.invalidate_PrimitivesPlane(curTransf, boxMargin, FLoad(0.2f)))
	{
		//ML:localNormal is the local space of plane normal, however, because shape1 is box and shape0 is plane, we need to use the reverse of contact normal(which will be the plane normal) to make the refreshContactPoints
		//work out the correct pentration for points
		const Vec3V localNormal = V3UnitX();

		manifold.mNumContacts = 0;
		manifold.setRelativeTransform(curTransf);

		const PsMatTransformV aToB(curTransf);
		const FloatV bx = V3GetX(boxExtents);
		const FloatV by = V3GetY(boxExtents);
		const FloatV bz = V3GetZ(boxExtents);

		const FloatV nbx = FNeg(bx);
		const FloatV nby = FNeg(by);
		const FloatV nbz = FNeg(bz);

		const Vec3V temp0 = V3Scale(aToB.getCol0(), bx);
		const Vec3V temp1 = V3Scale(aToB.getCol1(), by);
		const Vec3V temp2 = V3Scale(aToB.getCol2(), bz);

		const Vec3V ntemp2 = V3Neg(temp2);

		const FloatV px = V3GetX(aToB.p);

		//box's points in the local space of plane
		const Vec3V temp01 = V3Add(temp0, temp1);//(x, y)
		const Vec3V temp02 = V3Sub(temp0, temp1);//(x, -y)

		const FloatV s0 = V3GetX(V3Add(temp2,	temp01));//(x, y, z)
		const FloatV s1 = V3GetX(V3Add(ntemp2,	temp01));//(x, y, -z)
		const FloatV s2 = V3GetX(V3Add(temp2,	temp02));//(x, -y, z)
		const FloatV s3 = V3GetX(V3Add(ntemp2,	temp02));//(x, -y, -z)
		const FloatV s4 = V3GetX(V3Sub(temp2,	temp02));//(-x, y, z)
		const FloatV s5 = V3GetX(V3Sub(ntemp2,	temp02));//(-x, y, -z)
		const FloatV s6 = V3GetX(V3Sub(temp2,	temp01));//(-x, -y, z)
		const FloatV s7 = V3GetX(V3Sub(ntemp2,	temp01));//(-x, -y, -z)

		const FloatV acceptanceDist = FSub(contactDist, px);

		Gu::PersistentContact* manifoldContacts = PX_CP_TO_PCP(contactBuffer.contacts);
		PxU32 numContacts = 0;

		if(FAllGrtr(acceptanceDist, s0))
		{
			const FloatV pen = FAdd(s0, px);
			//(x, y, z)
			manifoldContacts[numContacts].mLocalPointA = boxExtents;//aToB.transformInv(p); 
			manifoldContacts[numContacts].mLocalPointB = V3NegScaleSub(localNormal, pen, aToB.transform(boxExtents)); 
			manifoldContacts[numContacts++].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(localNormal), pen);
		}
	
		if(FAllGrtr(acceptanceDist, s1))
		{
			const FloatV pen = FAdd(s1, px);
			//(x, y, -z)
			const Vec3V p = V3Merge(bx, by, nbz);
			//add to contact stream
			manifoldContacts[numContacts].mLocalPointA = p;//aToB.transformInv(p); 
			manifoldContacts[numContacts].mLocalPointB = V3NegScaleSub(localNormal, pen, aToB.transform(p)); 
			manifoldContacts[numContacts++].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(localNormal), pen);
		}

		if(FAllGrtr(acceptanceDist, s2))
		{
			const FloatV pen = FAdd(s2, px);
			//(x, -y, z)
			const Vec3V p = V3Merge(bx, nby, bz);
			manifoldContacts[numContacts].mLocalPointA = p;//aToB.transformInv(p); 
			manifoldContacts[numContacts].mLocalPointB = V3NegScaleSub(localNormal, pen, aToB.transform(p)); 
			manifoldContacts[numContacts++].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(localNormal), pen);
		}

		if(FAllGrtr(acceptanceDist, s3))
		{
			const FloatV pen = FAdd(s3, px);
			//(x, -y, -z)
			const Vec3V p = V3Merge(bx, nby, nbz);
			manifoldContacts[numContacts].mLocalPointA = p; 
			manifoldContacts[numContacts].mLocalPointB = V3NegScaleSub(localNormal, pen, aToB.transform(p)); 
			manifoldContacts[numContacts++].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(localNormal), pen);
		}
	
		if(FAllGrtr(acceptanceDist, s4))
		{
			const FloatV pen = FAdd(s4, px);
			//(-x, y, z)
			const Vec3V p =V3Merge(nbx, by, bz);
			manifoldContacts[numContacts].mLocalPointA = p;
			manifoldContacts[numContacts].mLocalPointB = V3NegScaleSub(localNormal, pen, aToB.transform(p)); 
			manifoldContacts[numContacts++].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(localNormal), pen);
		}

		if(FAllGrtr(acceptanceDist, s5))
		{
			const FloatV pen = FAdd(s5, px);
			//(-x, y, -z)
			const Vec3V p = V3Merge(nbx, by, nbz);
			manifoldContacts[numContacts].mLocalPointA = p;//aToB.transformInv(p); 
			manifoldContacts[numContacts].mLocalPointB = V3NegScaleSub(localNormal, pen, aToB.transform(p)); 
			manifoldContacts[numContacts++].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(localNormal), pen);
		}

	
		if(FAllGrtr(acceptanceDist, s6))
		{
			const FloatV pen = FAdd(s6, px);
			//(-x, -y, z)
			const Vec3V p = V3Merge(nbx, nby, bz);
			manifoldContacts[numContacts].mLocalPointA = p;//aToB.transformInv(p); 
			manifoldContacts[numContacts].mLocalPointB = V3NegScaleSub(localNormal, pen, aToB.transform(p)); 
			manifoldContacts[numContacts++].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(localNormal), pen);
		}
	
		if(FAllGrtr(acceptanceDist, s7))
		{
			const FloatV pen = FAdd(s7, px);
			//(-x, -y, -z)
			const Vec3V p = V3Merge(nbx, nby, nbz);
			manifoldContacts[numContacts].mLocalPointA = p;
			manifoldContacts[numContacts].mLocalPointB = V3NegScaleSub(localNormal, pen, aToB.transform(p)); 
			manifoldContacts[numContacts++].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(localNormal), pen);
		}
		
		//reduce contacts
		manifold.addBatchManifoldContactsCluster(manifoldContacts, numContacts);
		manifold.addManifoldContactsToContactBuffer(contactBuffer, negPlaneNormal, transf1, contactDist);

		return manifold.getNumContacts() > 0;
	}
	else
	{
		manifold.addManifoldContactsToContactBuffer(contactBuffer, negPlaneNormal, transf1, contactDist);
		
		//manifold.drawManifold(*gRenderOutPut, transf0, transf1);
		return manifold.getNumContacts() > 0;
	}	
}

}//Gu
}//physx
