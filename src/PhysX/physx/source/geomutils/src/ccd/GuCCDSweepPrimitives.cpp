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

#include "Ps.h"
#include "GuVecCapsule.h"
#include "GuVecBox.h"
#include "GuVecConvexHull.h"
#include "GuVecTriangle.h"
#include "GuGJKRaycast.h"
#include "GuCCDSweepConvexMesh.h"
#include "GuGJKType.h"

namespace physx
{
namespace Gu
{

using namespace Ps::aos;

template<typename Geom> PX_FORCE_INLINE  PxReal getRadius(const PxGeometry&) 
{	
	return 0;	
}

template<> PX_FORCE_INLINE PxReal getRadius<CapsuleV>(const PxGeometry& g) 
{ 
	PX_ASSERT(g.getType() == PxGeometryType::eCAPSULE || g.getType() == PxGeometryType::eSPHERE);
	PX_COMPILE_TIME_ASSERT(PX_OFFSET_OF(PxSphereGeometry, radius) == PX_OFFSET_OF(PxCapsuleGeometry, radius));
	return static_cast<const PxSphereGeometry&>(g).radius;
}



template<class ConvexA, class ConvexB>
static PxReal CCDSweep(ConvexA& a, ConvexB& b,  const PxTransform& transform0, const PxTransform& transform1, const PxTransform& lastTm0, const PxTransform& lastTm1,
					 const Ps::aos::FloatV& toiEstimate, PxVec3& worldPoint, PxVec3& worldNormal, PxReal inflation = 0.f)
{
	PX_UNUSED(toiEstimate); //KS - TODO - can we use this again?
	using namespace Ps::aos;

	const Vec3V zero = V3Zero();

	const QuatV q0 = QuatVLoadA(&transform0.q.x);
	const Vec3V p0 = V3LoadA(&lastTm0.p.x);

	const QuatV q1 = QuatVLoadA(&transform1.q.x);
	const Vec3V p1 = V3LoadA(&lastTm1.p.x);

	const PsTransformV tr0(p0, q0);
	const PsTransformV tr1(p1, q1);

	const PsMatTransformV aToB(tr1.transformInv(tr0));

	const Vec3V trans0p = V3LoadU(transform0.p);
	const Vec3V trans1p = V3LoadU(transform1.p);
	const Vec3V trA = V3Sub(trans0p, p0);
	const Vec3V trB = V3Sub(trans1p, p1);
	const Vec3V relTr = tr1.rotateInv(V3Sub(trB, trA));

	FloatV lambda;
	Vec3V closestA, normal;
	const FloatV initialLambda = FZero();
	const RelativeConvex<ConvexA> convexA(a, aToB);
	const LocalConvex<ConvexB> convexB(b);
	if(gjkRaycastPenetration<RelativeConvex<ConvexA>, LocalConvex<ConvexB> >(convexA, convexB, aToB.p, initialLambda, zero, relTr, lambda, normal, closestA, inflation, true))
	{
		//Adjust closestA because it will be on the surface of convex a in its initial position (s). If the TOI > 0, we need to move 
		//the point along the sweep direction to get the world-space hit position.
		PxF32 res;
		FStore(lambda, &res);
		closestA = V3ScaleAdd(trA, FMax(lambda, FZero()), tr1.transform(closestA));
		normal = tr1.rotate(normal);

		V3StoreU(normal, worldNormal);
		V3StoreU(closestA, worldPoint);
		return res;
	}
	return PX_MAX_REAL;
}



//
// lookup table for geometry-vs-geometry sweeps
//


PxReal UnimplementedSweep (GU_SWEEP_METHOD_ARGS_UNUSED)
{
	return PX_MAX_REAL;	//no impact
}

template<typename Geom0, typename Geom1>	
PxReal SweepGeomGeom(GU_SWEEP_METHOD_ARGS)
{
	PX_UNUSED(outCCDFaceIndex);
	PX_UNUSED(fastMovingThreshold);

	const PxGeometry& g0 = shape0.mGeometry->getGeometry();
	const PxGeometry& g1 = shape1.mGeometry->getGeometry();

	typename ConvexGeom<Geom0>::Type geom0(g0);
	typename ConvexGeom<Geom1>::Type geom1(g1);

	return CCDSweep(geom0, geom1, transform0, transform1, lastTm0, lastTm1, FLoad(toiEstimate), worldPoint, worldNormal, restDistance+getRadius<Geom0>(g0)+getRadius<Geom1>(g1) );
}

typedef PxReal (*SweepMethod) (GU_SWEEP_METHOD_ARGS);

PxReal SweepAnyShapeHeightfield(GU_SWEEP_METHOD_ARGS);
PxReal SweepAnyShapeMesh(GU_SWEEP_METHOD_ARGS);

SweepMethod g_SweepMethodTable[PxGeometryType::eGEOMETRY_COUNT][PxGeometryType::eGEOMETRY_COUNT] = 
{
	//PxGeometryType::eSPHERE
	{
		SweepGeomGeom<CapsuleV, CapsuleV>,				//PxGeometryType::eSPHERE
		UnimplementedSweep,								//PxGeometryType::ePLANE
		SweepGeomGeom<CapsuleV, CapsuleV>,				//PxGeometryType::eCAPSULE
		SweepGeomGeom<CapsuleV, BoxV>,					//PxGeometryType::eBOX
		SweepGeomGeom<CapsuleV, ConvexHullV>,			//PxGeometryType::eCONVEXMESH
		SweepAnyShapeMesh,								//PxGeometryType::eTRIANGLEMESH
		SweepAnyShapeHeightfield,						//PxGeometryType::eHEIGHTFIELD		//TODO		
	},

	//PxGeometryType::ePLANE
	{
		0,												//PxGeometryType::eSPHERE
		UnimplementedSweep,								//PxGeometryType::ePLANE
		UnimplementedSweep,								//PxGeometryType::eCAPSULE
		UnimplementedSweep,								//PxGeometryType::eBOX
		UnimplementedSweep,								//PxGeometryType::eCONVEXMESH
		UnimplementedSweep,								//PxGeometryType::eTRIANGLEMESH
		UnimplementedSweep,								//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::eCAPSULE
	{
		0,												//PxGeometryType::eSPHERE
		0,												//PxGeometryType::ePLANE
		SweepGeomGeom<CapsuleV, CapsuleV>,				//PxGeometryType::eCAPSULE
		SweepGeomGeom<CapsuleV, BoxV>,					//PxGeometryType::eBOX
		SweepGeomGeom<CapsuleV, ConvexHullV>,			//PxGeometryType::eCONVEXMESH
		SweepAnyShapeMesh,								//PxGeometryType::eTRIANGLEMESH
		SweepAnyShapeHeightfield,						//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::eBOX
	{
		0,											//PxGeometryType::eSPHERE
		0,											//PxGeometryType::ePLANE
		0,											//PxGeometryType::eCAPSULE
		SweepGeomGeom<BoxV, BoxV>,					//PxGeometryType::eBOX
		SweepGeomGeom<BoxV, ConvexHullV>,			//PxGeometryType::eCONVEXMESH
		SweepAnyShapeMesh,							//PxGeometryType::eTRIANGLEMESH
		SweepAnyShapeHeightfield,					//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::eCONVEXMESH
	{
		0,											//PxGeometryType::eSPHERE
		0,											//PxGeometryType::ePLANE
		0,											//PxGeometryType::eCAPSULE
		0,											//PxGeometryType::eBOX
		SweepGeomGeom<ConvexHullV, ConvexHullV>,	//PxGeometryType::eCONVEXMESH
		SweepAnyShapeMesh,							//PxGeometryType::eTRIANGLEMESH
		SweepAnyShapeHeightfield,					//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::eTRIANGLEMESH
	{
		0,											//PxGeometryType::eSPHERE
		0,											//PxGeometryType::ePLANE
		0,											//PxGeometryType::eCAPSULE
		0,											//PxGeometryType::eBOX
		0,											//PxGeometryType::eCONVEXMESH
		UnimplementedSweep,							//PxGeometryType::eTRIANGLEMESH
		UnimplementedSweep,							//PxGeometryType::eHEIGHTFIELD
	},

	//PxGeometryType::eHEIGHTFIELD
	{
		0,											//PxGeometryType::eSPHERE
		0,											//PxGeometryType::ePLANE
		0,											//PxGeometryType::eCAPSULE
		0,											//PxGeometryType::eBOX
		0,											//PxGeometryType::eCONVEXMESH
		0,											//PxGeometryType::eTRIANGLEMESH
		UnimplementedSweep,							//PxGeometryType::eHEIGHTFIELD
	},
};


PxReal SweepShapeShape(GU_SWEEP_METHOD_ARGS)
{
	PxGeometryType::Enum type0 = shape0.mGeometry->getType();
	PxGeometryType::Enum type1 = shape1.mGeometry->getType();

	return g_SweepMethodTable[type0][type1](shape0, shape1, transform0, transform1, lastTm0, lastTm1,
		restDistance, worldNormal, worldPoint, toiEstimate, outCCDFaceIndex, fastMovingThreshold);
		
}

//
// lookup table for sweeps agains triangles
//

PxReal UnimplementedTriangleSweep(GU_TRIANGLE_SWEEP_METHOD_ARGS)
{
	PX_UNUSED(shape0);
	PX_UNUSED(shape1);
	PX_UNUSED(transform0);
	PX_UNUSED(transform1);
	PX_UNUSED(lastTm0);
	PX_UNUSED(lastTm1);
	PX_UNUSED(restDistance);
	PX_UNUSED(worldNormal);
	PX_UNUSED(worldPoint);
	PX_UNUSED(meshScaling);
	PX_UNUSED(triangle);
	PX_UNUSED(toiEstimate);

	return 1e10f;	//no impact
}

template<typename Geom>	
PxReal SweepGeomTriangles(GU_TRIANGLE_SWEEP_METHOD_ARGS)
{
	PX_UNUSED(meshScaling);
	PX_UNUSED(shape1);

	const PxGeometry& g = shape0.getGeometry();
	//Geom geom(g);
	typename ConvexGeom<Geom>::Type geom(g);

	return CCDSweep<TriangleV, Geom>(triangle, geom, transform1, transform0, lastTm1, lastTm0, FLoad(toiEstimate), worldPoint, worldNormal, restDistance+getRadius<Geom>(g) );
}

typedef PxReal (*TriangleSweepMethod) (GU_TRIANGLE_SWEEP_METHOD_ARGS);
TriangleSweepMethod g_TriangleSweepMethodTable[PxGeometryType::eGEOMETRY_COUNT] = 
{
	SweepGeomTriangles<CapsuleV>,		//PxGeometryType::eSPHERE
	UnimplementedTriangleSweep,			//PxGeometryType::ePLANE
	SweepGeomTriangles<CapsuleV>,		//PxGeometryType::eCAPSULE
	SweepGeomTriangles<BoxV>,			//PxGeometryType::eBOX
	SweepGeomTriangles<ConvexHullV>,	//PxGeometryType::eCONVEXMESH
	UnimplementedTriangleSweep,			//PxGeometryType::eTRIANGLEMESH
	UnimplementedTriangleSweep,			//PxGeometryType::eHEIGHTFIELD
};

PxReal SweepShapeTriangle(GU_TRIANGLE_SWEEP_METHOD_ARGS)
{
	const PxGeometryType::Enum type0 = shape0.getType();
	TriangleSweepMethod method = g_TriangleSweepMethodTable[type0];
	return method(shape0, shape1, transform0, transform1, lastTm0, lastTm1, restDistance, worldNormal, worldPoint, meshScaling, triangle, toiEstimate);
}

}
}

