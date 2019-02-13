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

#include "GuHeightFieldUtil.h"
#include "GuEntityReport.h"
#include "PxConvexMeshGeometry.h"
#include "GuConvexMesh.h"
#include "GuSweepSharedTests.h"  
#include "GuConvexUtilsInternal.h"
#include "GuTriangleMesh.h"
#include "GuVecBox.h"
#include "GuVecTriangle.h"
#include "GuVecConvexHullNoScale.h"
#include "GuMidphaseInterface.h"
#include "GuPCMContactConvexCommon.h"
#include "GuSweepMTD.h"
#include "GuPCMShapeConvex.h"
#include "GuDistanceSegmentSegment.h"
#include "GuDistancePointSegment.h"
#include "GuInternal.h"
#include "GuConvexEdgeFlags.h"

using namespace physx;
using namespace Gu;

// PT: TODO: refactor with GuMTD.cpp versions
static PX_FORCE_INLINE PxF32 manualNormalize(PxVec3& mtd, const PxVec3& normal, PxReal lenSq)
{
	const PxF32 len = PxSqrt(lenSq);

	// We do a *manual* normalization to check for singularity condition
	if(lenSq < 1e-6f)
		mtd = PxVec3(1.0f, 0.0f, 0.0f);			// PT: zero normal => pick up random one
	else
		mtd = normal * 1.0f / len;

	return len;
}

static PX_FORCE_INLINE void getScaledTriangle(const PxTriangleMeshGeometry& triGeom, const Cm::Matrix34& vertex2worldSkew, bool flipsNormal, PxTriangle& triangle, PxTriangleID triangleIndex)
{
	TriangleMesh* tm = static_cast<TriangleMesh*>(triGeom.triangleMesh);
	tm->computeWorldTriangle(triangle, triangleIndex, vertex2worldSkew, flipsNormal);
}

#define	BATCH_TRIANGLE_NUMBER	32u

struct MTDTriangle : public PxTriangle
{
public:
	PxU8 extraTriData;//active edge flag data
};

struct MeshMTDGenerationCallback : MeshHitCallback<PxRaycastHit>
{
public:
	
	Ps::Array<PxU32>&	container;

	MeshMTDGenerationCallback(Ps::Array<PxU32>& tempContainer)
	:	MeshHitCallback<PxRaycastHit>(CallbackMode::eMULTIPLE), container(tempContainer)
	{
	}

	virtual PxAgain processHit(
		const PxRaycastHit& hit, const PxVec3&, const PxVec3&, const PxVec3&, PxReal&, const PxU32*)
	{
		container.pushBack(hit.faceIndex);

		return true;
	}

	void operator=(const MeshMTDGenerationCallback&) {}
};

static bool getMTDPerTriangle(const MeshPersistentContact* manifoldContacts, const PxU32 numContacts, const PxU32 triangleIndex, Ps::aos::Vec3V& normal, Ps::aos::Vec3V& closestA, Ps::aos::Vec3V& closestB, PxU32& faceIndex, Ps::aos::FloatV& deepestPen)
{
	using namespace Ps::aos;

	FloatV deepest =  V4GetW(manifoldContacts[0].mLocalNormalPen);
	PxU32 index = 0;
	for(PxU32 k=1; k<numContacts; ++k)
	{
		const FloatV pen = V4GetW(manifoldContacts[k].mLocalNormalPen);
		if(FAllGrtr(deepest, pen))
		{
			deepest = pen;
			index = k;
		}
	}

	if(FAllGrtr(deepestPen, deepest))
	{
		PX_ASSERT(triangleIndex == manifoldContacts[index].mFaceIndex);
		faceIndex = triangleIndex;
		deepestPen = deepest;
		normal = Vec3V_From_Vec4V(manifoldContacts[index].mLocalNormalPen);
		closestA = manifoldContacts[index].mLocalPointB;
		closestB = manifoldContacts[index].mLocalPointA;
		return true;
	}

	return false;
}
static void midPhaseQuery(const PxTriangleMeshGeometry& meshGeom, const PxTransform& pose, const Box& bound, Ps::Array<PxU32>& tempContainer)
{
	TriangleMesh* meshData = static_cast<TriangleMesh*>(meshGeom.triangleMesh);

	Box vertexSpaceBox;
	computeVertexSpaceOBB(vertexSpaceBox, bound, pose, meshGeom.scale);

	MeshMTDGenerationCallback callback(tempContainer);
	Midphase::intersectOBB(meshData, vertexSpaceBox, callback, true);
}

// PT: TODO: refactor with EntityReportContainerCallback
struct MidPhaseQueryLocalReport : EntityReport<PxU32>
{
	MidPhaseQueryLocalReport(Ps::Array<PxU32>& _container) : container(_container)
	{

	}
	virtual bool onEvent(PxU32 nb, PxU32* indices)
	{
		for(PxU32 i=0; i<nb; i++)
			container.pushBack(indices[i]);
		return true;
	}

	Ps::Array<PxU32>& container;

private:
	MidPhaseQueryLocalReport operator=(MidPhaseQueryLocalReport& report);
} ;

static void midPhaseQuery(const HeightFieldUtil& hfUtil, const PxTransform& pose, const PxBounds3& bounds, Ps::Array<PxU32>& tempContainer, PxU32 flags)
{
	MidPhaseQueryLocalReport localReport(tempContainer);
	hfUtil.overlapAABBTriangles(pose, bounds, flags, &localReport);
}

static bool calculateMTD(	const CapsuleV& capsuleV, const Ps::aos::FloatVArg inflatedRadiusV, const bool isDoubleSide, const MTDTriangle* triangles, const PxU32 nbTriangles, const PxU32 startIndex, MeshPersistentContact* manifoldContacts, 
							PxU32& numContacts, Ps::aos::Vec3V& normal, Ps::aos::Vec3V& closestA, Ps::aos::Vec3V& closestB, PxU32& faceIndex, Ps::aos::FloatV& mtd)
{
	using namespace Ps::aos;
	const FloatV zero = FZero();
	bool hadContacts = false;
	FloatV deepestPen = mtd;

	for(PxU32 j=0; j<nbTriangles; ++j)
	{
		numContacts = 0;

		const MTDTriangle& curTri = triangles[j];
		TriangleV triangleV;
		triangleV.verts[0] = V3LoadU(curTri.verts[0]);
		triangleV.verts[1] = V3LoadU(curTri.verts[1]);
		triangleV.verts[2] = V3LoadU(curTri.verts[2]);
		const PxU8 triFlag = curTri.extraTriData;

		const Vec3V triangleNormal = triangleV.normal();
		const Vec3V v = V3Sub(capsuleV.getCenter(), triangleV.verts[0]);
		const FloatV dotV = V3Dot(triangleNormal, v);

		// Backface culling
		const bool culled = !isDoubleSide && (FAllGrtr(zero, dotV));
		if(culled)
			continue;
		
		PCMCapsuleVsMeshContactGeneration::processTriangle(triangleV, j+startIndex, capsuleV, inflatedRadiusV, triFlag, manifoldContacts, numContacts);

		if(numContacts ==0)
			continue;

		hadContacts = true;

		getMTDPerTriangle(manifoldContacts, numContacts, j + startIndex, normal, closestA, closestB, faceIndex, deepestPen);
	}

	mtd = deepestPen;
	return hadContacts;
}

bool physx::Gu::computeCapsule_TriangleMeshMTD(	const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose, CapsuleV& capsuleV, PxReal inflatedRadius, 
												bool isDoubleSided, PxSweepHit& hit)
{
	using namespace Ps::aos;

	const Vec3V zeroV = V3Zero();

	TriangleMesh* triMesh = static_cast<TriangleMesh*>(triMeshGeom.triangleMesh);
	const PxU8* extraTrigData = triMesh->getExtraTrigData();
	const bool flipsNormal = triMeshGeom.scale.hasNegativeDeterminant();
	
	MeshPersistentContact manifoldContacts[64];   
	PxU32 numContacts = 0;
	//inflated the capsule by 15% in case of some disagreement between sweep and mtd calculation. If sweep said initial overlap, but mtd has a positive separation,
	//we are still be able to return a valid normal but we should zero the distance.
	const FloatV inflatedRadiusV = FLoad(inflatedRadius*1.15f);
	
	bool foundInitial = false;
	const PxU32 iterations = 4;

	const Cm::Matrix34 vertexToWorldSkew = pose * triMeshGeom.scale;

	Ps::Array<PxU32> tempContainer;
	tempContainer.reserve(128);

	Vec3V closestA = zeroV, closestB = zeroV, normal = zeroV;
	PxU32 triangleIndex = 0xfffffff;

	Vec3V translation = zeroV;
	FloatV mtd;
	FloatV distV;
	for(PxU32 i=0; i<iterations; ++i)
	{
		tempContainer.forceSize_Unsafe(0);
		Capsule inflatedCapsule;
		V3StoreU(capsuleV.p0, inflatedCapsule.p0);
		V3StoreU(capsuleV.p1, inflatedCapsule.p1);
		inflatedCapsule.radius = inflatedRadius;

		Box capsuleBox;
		computeBoxAroundCapsule(inflatedCapsule, capsuleBox);
		midPhaseQuery(triMeshGeom, pose, capsuleBox, tempContainer);

		// Get results
		const PxU32 nbTriangles = tempContainer.size();
		if(!nbTriangles)
			break;

		const PxU32* indices = tempContainer.begin();

		bool hadContacts = false;

		const PxU32 nbBatches = (nbTriangles + BATCH_TRIANGLE_NUMBER - 1)/BATCH_TRIANGLE_NUMBER;
		mtd = FMax();
		MTDTriangle triangles[BATCH_TRIANGLE_NUMBER];
		for(PxU32 a = 0; a < nbBatches; ++a)
		{
			const PxU32 startIndex = a * BATCH_TRIANGLE_NUMBER;
			const PxU32 nbTrigs = PxMin(nbTriangles - startIndex, BATCH_TRIANGLE_NUMBER);
			for(PxU32 k=0; k<nbTrigs; k++)
			{
				//triangle world space
				const PxU32 triangleIndex1 = indices[startIndex+k];
				::getScaledTriangle(triMeshGeom, vertexToWorldSkew, flipsNormal, triangles[k], triangleIndex1);
				triangles[k].extraTriData = getConvexEdgeFlags(extraTrigData, triangleIndex1);
			}

			//ML: mtd has back face culling, so if the capsule's center is below the triangle, we won't generate any contacts
			hadContacts = calculateMTD(capsuleV, inflatedRadiusV, isDoubleSided, triangles, nbTrigs, startIndex, manifoldContacts, numContacts, normal, closestA, closestB, triangleIndex, mtd) || hadContacts;
		}

		if(!hadContacts)
			break;

		triangleIndex = indices[triangleIndex];

		foundInitial = true;

		//move the capsule to depenetrate it
		
		distV = FSub(mtd, capsuleV.radius);
		if(FAllGrtrOrEq(FZero(), distV))
		{
			Vec3V center = capsuleV.getCenter();
			const Vec3V t = V3Scale(normal, distV);
			translation = V3Sub(translation, t);
			center = V3Sub(center, t);
			capsuleV.setCenter(center);
		}
		else
		{
			if(i == 0)
			{
				//First iteration so keep this normal
				hit.distance = 0.f;
				V3StoreU(closestA, hit.position);
				V3StoreU(normal, hit.normal);
				hit.faceIndex = triangleIndex;
				return true;
			}
			break;
		}
	}

	const FloatV translationF = V3Length(translation);
	distV = FNeg(translationF);

	const BoolV con = FIsGrtr(translationF, FZero());
	normal = V3Sel(con, V3ScaleInv(translation, translationF), zeroV);
	

	if(foundInitial)
	{
		FStore(distV, &hit.distance);
		V3StoreU(closestA, hit.position);
		V3StoreU(normal, hit.normal);
		hit.faceIndex = triangleIndex;
	}
	return foundInitial;
}


bool physx::Gu::computeCapsule_HeightFieldMTD(const PxHeightFieldGeometry& heightFieldGeom, const PxTransform& pose, CapsuleV& capsuleV, PxReal inflatedRadius, bool isDoubleSided, const PxU32 flags, PxSweepHit& hit)
{
	using namespace Ps::aos;

	const Vec3V zeroV = V3Zero();

	MeshPersistentContact manifoldContacts[64]; 
	PxU32 numContacts = 0;
	//inflated the capsule by 1% in case of some disagreement between sweep and mtd calculation.If sweep said initial overlap, but mtd has a positive separation,
	//we are still be able to return a valid normal but we should zero the distance.
	const FloatV inflatedRadiusV = FLoad(inflatedRadius*1.01f);  
	
	bool foundInitial = false;
	const PxU32 iterations = 4;

	Ps::Array<PxU32> tempContainer;
	tempContainer.reserve(128);

	const HeightFieldUtil hfUtil(heightFieldGeom);

	Vec3V closestA = zeroV, closestB = zeroV, normal = zeroV;
	PxU32 triangleIndex = 0xfffffff;

	Vec3V translation = zeroV;
	FloatV mtd;
	FloatV distV;
	for(PxU32 i=0; i<iterations; ++i)
	{
		tempContainer.empty();
		Capsule inflatedCapsule;
		V3StoreU(capsuleV.p0, inflatedCapsule.p0);
		V3StoreU(capsuleV.p1, inflatedCapsule.p1);
		inflatedCapsule.radius = inflatedRadius;

		Box capsuleBox;
		computeBoxAroundCapsule(inflatedCapsule, capsuleBox);
		const PxTransform capsuleBoxTransform = capsuleBox.getTransform();
		const PxBounds3 bounds = PxBounds3::poseExtent(capsuleBoxTransform, capsuleBox.extents);
		midPhaseQuery(hfUtil, pose, bounds, tempContainer, flags);

		// Get results
		const PxU32 nbTriangles = tempContainer.size();
		if(!nbTriangles)
			break;

		bool hadContacts = false;

		const PxU32 nbBatches = (nbTriangles + BATCH_TRIANGLE_NUMBER - 1)/BATCH_TRIANGLE_NUMBER;
		mtd = FMax();
		MTDTriangle triangles[BATCH_TRIANGLE_NUMBER];
		for(PxU32 a = 0; a < nbBatches; ++a)
		{
			const PxU32 startIndex = a * BATCH_TRIANGLE_NUMBER;
			const PxU32 nbTrigs = PxMin(nbTriangles - startIndex, BATCH_TRIANGLE_NUMBER);
			for(PxU32 k=0; k<nbTrigs; k++)
			{
				//triangle vertex space
				const PxU32 triangleIndex1 = tempContainer[startIndex+k];
				hfUtil.getTriangle(pose, triangles[k], NULL, NULL, triangleIndex1, true);
				triangles[k].extraTriData = ETD_CONVEX_EDGE_ALL;
			}

			//ML: mtd has back face culling, so if the capsule's center is below the triangle, we won't generate any contacts
			hadContacts = calculateMTD(capsuleV, inflatedRadiusV, isDoubleSided, triangles, nbTrigs, startIndex, manifoldContacts, numContacts, normal, closestA, closestB, triangleIndex, mtd) || hadContacts;
		}

		if(!hadContacts)
			break;

		triangleIndex = tempContainer[triangleIndex];

		foundInitial = true;

		distV = FSub(mtd, capsuleV.radius);
		if(FAllGrtrOrEq(FZero(), distV))
		{
			//move the capsule to depenetrate it
			Vec3V center = capsuleV.getCenter();
			const Vec3V t = V3Scale(normal, distV);
			translation = V3Sub(translation, t);
			center = V3Sub(center, t);
			capsuleV.setCenter(center);
		}
		else
		{
			if(i == 0)
			{
				//First iteration so keep this normal
				hit.distance = 0.f;
				V3StoreU(closestA, hit.position);
				V3StoreU(normal, hit.normal);
				hit.faceIndex = triangleIndex;
				return true;
			}
			break;
		}
	}

	const FloatV translationF = V3Length(translation);
	distV = FNeg(translationF);

	const BoolV con = FIsGrtr(translationF, FZero());
	normal = V3Sel(con, V3ScaleInv(translation, translationF), zeroV);

	if(foundInitial)
	{
		FStore(distV, &hit.distance);
		V3StoreU(closestA, hit.position);
		V3StoreU(normal, hit.normal);
		hit.faceIndex = triangleIndex;
	}
	return foundInitial;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static bool calculateMTD(	const PolygonalData& polyData, SupportLocal* polyMap, Ps::aos::PsTransformV& convexTransform, const Ps::aos::PsMatTransformV& meshToConvex, bool isDoubleSided, const Ps::aos::FloatVArg inflation, const MTDTriangle* triangles, const PxU32 nbTriangles,  const PxU32 startIndex, 
							MeshPersistentContact* manifoldContacts, PxU32& numContacts, Ps::aos::Vec3V& normal, Ps::aos::Vec3V& closestA, Ps::aos::Vec3V& closestB, PxU32& faceIndex, Ps::aos::FloatV& mtd)
{
	using namespace Ps::aos;

	bool hadContacts = false;
	FloatV deepestPen = mtd;
	
	for(PxU32 j=0; j<nbTriangles; ++j)
	{
		numContacts = 0;
		const MTDTriangle& curTri = triangles[j];
		const PxU8 triFlag = curTri.extraTriData;
		
		PCMConvexVsMeshContactGeneration::processTriangle(polyData, polyMap, curTri.verts,  j+startIndex, triFlag, inflation, isDoubleSided, convexTransform, meshToConvex, manifoldContacts, numContacts);

		if(numContacts ==0)
			continue;

		hadContacts = true;
		getMTDPerTriangle(manifoldContacts, numContacts, j+startIndex, normal, closestA, closestB, faceIndex, deepestPen);
	}

	mtd = deepestPen;

	return hadContacts;
}

bool physx::Gu::computeBox_TriangleMeshMTD(const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose, const Box& _box, const PxTransform& boxTransform, PxReal inflation, bool isDoubleSided, PxSweepHit& hit)
{
	using namespace Ps::aos;

	TriangleMesh* triMesh = static_cast<TriangleMesh*>(triMeshGeom.triangleMesh);
	const PxU8* extraTrigData = triMesh->getExtraTrigData();

	const Vec3V zeroV = V3Zero();
	
	MeshPersistentContact manifoldContacts[64]; 
	PxU32 numContacts = 0;
	
	bool foundInitial = false;
	const PxU32 iterations = 4;

	Ps::Array<PxU32> tempContainer;
	tempContainer.reserve(128);

	Vec3V closestA = zeroV, closestB = zeroV, normal = zeroV;
	Vec3V worldNormal = zeroV, worldContactA = zeroV;//, worldContactB = zeroV;
	PxU32 triangleIndex = 0xfffffff;

	Vec3V translation = zeroV;

	Box box = _box;
	
	const QuatV q0 = QuatVLoadU(&boxTransform.q.x);
	const Vec3V p0 = V3LoadU(&boxTransform.p.x);

	const Vec3V boxExtents = V3LoadU(box.extents);
	const FloatV minMargin = CalculateMTDBoxMargin(boxExtents);
	const FloatV inflationV = FAdd(FLoad(inflation), minMargin);
	PxReal boundInflation;
	FStore(inflationV, &boundInflation);

	box.extents += PxVec3(boundInflation);
	const BoxV boxV(zeroV, boxExtents);

	Vec3V boxCenter = V3LoadU(box.center);

	//create the polyData based on the original data
	PolygonalData polyData;
	const PCMPolygonalBox polyBox(_box.extents);
	polyBox.getPolygonalData(&polyData);

	const Mat33V identity =  M33Identity();

	const Cm::Matrix34 meshToWorldSkew = pose * triMeshGeom.scale;

	PsTransformV boxTransformV(p0, q0);//box
	
	FloatV mtd;
	FloatV distV;
	for(PxU32 i=0; i<iterations; ++i)
	{		
		tempContainer.forceSize_Unsafe(0);

		midPhaseQuery(triMeshGeom, pose, box, tempContainer);

		// Get results
		const PxU32 nbTriangles = tempContainer.size();
		if(!nbTriangles)
			break;

		boxTransformV.p = boxCenter;
		SupportLocalImpl<BoxV> boxMap(boxV, boxTransformV, identity, identity, true);

		boxMap.setShapeSpaceCenterofMass(zeroV);
		// Move to AABB space
		Cm::Matrix34 WorldToBox;
		computeWorldToBoxMatrix(WorldToBox, box);
		const Cm::Matrix34 meshToBox = WorldToBox*meshToWorldSkew;

		const Ps::aos::Mat33V rot(V3LoadU(meshToBox.m.column0), V3LoadU(meshToBox.m.column1), V3LoadU(meshToBox.m.column2));
		const Ps::aos::PsMatTransformV meshToConvex(V3LoadU(meshToBox.p), rot);

		bool hadContacts = false;

		const PxU32 nbBatches = (nbTriangles + BATCH_TRIANGLE_NUMBER - 1)/BATCH_TRIANGLE_NUMBER;
		mtd = FMax();
		MTDTriangle triangles[BATCH_TRIANGLE_NUMBER];
		for(PxU32 a = 0; a < nbBatches; ++a)
		{
			const PxU32 startIndex = a * BATCH_TRIANGLE_NUMBER;
			const PxU32 nbTrigs = PxMin(nbTriangles - startIndex, BATCH_TRIANGLE_NUMBER);
			for(PxU32 k=0; k<nbTrigs; k++)
			{
				//triangle vertex space
				const PxU32 triangleIndex1 = tempContainer[startIndex+k];
				triMesh->getLocalTriangle(triangles[k], triangleIndex1, triMeshGeom.scale.hasNegativeDeterminant());
				triangles[k].extraTriData = getConvexEdgeFlags(extraTrigData, triangleIndex1);
			}

			//ML: mtd has back face culling, so if the capsule's center is below the triangle, we won't generate any contacts
			hadContacts = calculateMTD(polyData, &boxMap, boxTransformV, meshToConvex,  isDoubleSided, inflationV,  triangles, nbTrigs, startIndex, manifoldContacts, numContacts, normal, closestA, closestB, triangleIndex, mtd) || hadContacts;
		}
	
		if(!hadContacts)
			break;

		triangleIndex = tempContainer[triangleIndex];

		foundInitial = true;

		distV = mtd;
		worldNormal = boxTransformV.rotate(normal);
		worldContactA = boxTransformV.transform(closestA);
		if(FAllGrtrOrEq(FZero(), distV))
		{
			const Vec3V t = V3Scale(worldNormal, mtd);
			translation = V3Sub(translation, t);
			boxCenter = V3Sub(boxCenter, t);
			V3StoreU(boxCenter, box.center);
		}
		else
		{
			if(i == 0)
			{
				//First iteration so keep this normal
				hit.distance = 0.f;
				V3StoreU(worldContactA, hit.position);
				V3StoreU(worldNormal, hit.normal);
				hit.faceIndex = triangleIndex;
				return true;
			}
			break;
		}
	}

	const FloatV translationF = V3Length(translation);
	distV = FNeg(translationF);

	const BoolV con = FIsGrtr(translationF, FZero());
	worldNormal = V3Sel(con, V3ScaleInv(translation, translationF), zeroV);

	if(foundInitial)
	{
		//transform closestA to world space
		FStore(distV, &hit.distance);
		V3StoreU(worldContactA, hit.position);
		V3StoreU(worldNormal, hit.normal);
		hit.faceIndex = triangleIndex;
	}
	return foundInitial;
}

bool physx::Gu::computeBox_HeightFieldMTD(	const PxHeightFieldGeometry& heightFieldGeom, const PxTransform& pose, const Box& _box, const PxTransform& boxTransform, PxReal inflation,
											bool isDoubleSided, const PxU32 flags, PxSweepHit& hit)
{
	using namespace Ps::aos;

	const Vec3V zeroV = V3Zero();
	MeshPersistentContact manifoldContacts[64]; 
	PxU32 numContacts = 0;
	bool foundInitial = false;
	const PxU32 iterations = 4;

	Ps::Array<PxU32> tempContainer;
	tempContainer.reserve(128);
	const HeightFieldUtil hfUtil(heightFieldGeom);

	Vec3V closestA = zeroV, closestB = zeroV, normal = zeroV;
	Vec3V worldNormal = zeroV, worldContactA = zeroV;//, worldContactB = zeroV;
	PxU32 triangleIndex = 0xfffffff;

	Vec3V translation = zeroV;

	Box box = _box;
	
	const QuatV q0 = QuatVLoadU(&boxTransform.q.x);
	const Vec3V p0 = V3LoadU(&boxTransform.p.x);

	const Vec3V boxExtents = V3LoadU(box.extents);
	const FloatV minMargin = CalculateMTDBoxMargin(boxExtents);
	const FloatV inflationV = FAdd(FLoad(inflation), minMargin);
	//const FloatV inflationV = FLoad(inflation);

	PxReal boundInflation;
	FStore(inflationV, &boundInflation);
	box.extents += PxVec3(boundInflation);
	
	const BoxV boxV(zeroV, boxExtents);

	Vec3V boxCenter = V3LoadU(box.center);

	//create the polyData based on the original box
	PolygonalData polyData;
	const PCMPolygonalBox polyBox(_box.extents);
	polyBox.getPolygonalData(&polyData);

	const Mat33V identity = M33Identity();

	const Cm::Matrix34 meshToWorldSkew(pose);

	PsTransformV boxTransformV(p0, q0);//box
	
	FloatV mtd;
	FloatV distV;
	for(PxU32 i=0; i<iterations; ++i)
	{
		tempContainer.forceSize_Unsafe(0);

		const PxBounds3 bounds = PxBounds3::poseExtent(box.getTransform(), box.extents);
		midPhaseQuery(hfUtil, pose, bounds, tempContainer, flags);

		// Get results
		const PxU32 nbTriangles = tempContainer.size();
		if(!nbTriangles)
			break;

		boxTransformV.p = boxCenter;
		SupportLocalImpl<BoxV> boxMap(boxV, boxTransformV, identity, identity, true);
		boxMap.setShapeSpaceCenterofMass(zeroV);
		// Move to AABB space
		Cm::Matrix34 WorldToBox;
		computeWorldToBoxMatrix(WorldToBox, box);
		const Cm::Matrix34 meshToBox = WorldToBox*meshToWorldSkew;

		const Ps::aos::Mat33V rot(V3LoadU(meshToBox.m.column0), V3LoadU(meshToBox.m.column1), V3LoadU(meshToBox.m.column2));
		const Ps::aos::PsMatTransformV meshToConvex(V3LoadU(meshToBox.p), rot);

		bool hadContacts = false;

		const PxU32 nbBatches = (nbTriangles + BATCH_TRIANGLE_NUMBER-1)/BATCH_TRIANGLE_NUMBER;
		mtd = FMax();
		MTDTriangle triangles[BATCH_TRIANGLE_NUMBER];
		for(PxU32 a = 0; a < nbBatches; ++a)
		{
			const PxU32 startIndex = a * BATCH_TRIANGLE_NUMBER;
			const PxU32 nbTrigs = PxMin(nbTriangles - startIndex, BATCH_TRIANGLE_NUMBER);
			for(PxU32 k=0; k<nbTrigs; k++)
			{
				//triangle vertex space
				const PxU32 triangleIndex1 = tempContainer[startIndex+k];
				hfUtil.getTriangle(pose, triangles[k], NULL, NULL, triangleIndex1, false, false);
				triangles[k].extraTriData = ETD_CONVEX_EDGE_ALL;
			}

			//ML: mtd has back face culling, so if the box's center is below the triangle, we won't generate any contacts
			hadContacts = calculateMTD(polyData, &boxMap, boxTransformV, meshToConvex,  isDoubleSided, inflationV,  triangles, nbTrigs, startIndex, manifoldContacts, numContacts, normal, closestA, closestB, triangleIndex, mtd) || hadContacts;
		}

		if(!hadContacts)
			break;

		triangleIndex = tempContainer[triangleIndex];

		foundInitial = true;

		distV = mtd;
		worldNormal = boxTransformV.rotate(normal);
		worldContactA = boxTransformV.transform(closestA);
		if(FAllGrtrOrEq(FZero(), distV))
		{
			//worldContactB = boxTransformV.transform(closestB);
			const Vec3V t = V3Scale(worldNormal, mtd);
			translation = V3Sub(translation, t);
			boxCenter = V3Sub(boxCenter, t);
			V3StoreU(boxCenter, box.center);
		}
		else
		{
			if(i == 0)
			{
				//First iteration so keep this normal
				hit.distance = 0.f;
				V3StoreU(worldContactA, hit.position);
				V3StoreU(worldNormal, hit.normal);
				hit.faceIndex = triangleIndex;
				return true;
			}
			break;
		}
	}


	const FloatV translationF = V3Length(translation);
	distV = FNeg(translationF);

	const BoolV con = FIsGrtr(translationF, FZero());
	worldNormal = V3Sel(con, V3ScaleInv(translation, translationF), zeroV);
	
	if(foundInitial)
	{
		FStore(distV, &hit.distance);
		V3StoreU(worldContactA, hit.position);
		V3StoreU(worldNormal, hit.normal);
		hit.faceIndex = triangleIndex;
	}
	return foundInitial;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool physx::Gu::computeConvex_TriangleMeshMTD(	const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose, PxReal inflation,
												bool isDoubleSided, PxSweepHit& hit)
{
	using namespace Ps::aos;

	const Vec3V zeroV = V3Zero();

	TriangleMesh* triMesh = static_cast<TriangleMesh*>(triMeshGeom.triangleMesh);
	ConvexMesh* cm = static_cast<ConvexMesh*>(convexGeom.convexMesh);
	const PxU8* extraTrigData = triMesh->getExtraTrigData();

	MeshPersistentContact manifoldContacts[64]; 
	PxU32 numContacts = 0;
	
	bool foundInitial = false;
	const PxU32 iterations = 2;

	ConvexHullData* hullData = &cm->getHull();

	const bool idtScaleConvex = convexGeom.scale.isIdentity();
	
	 Cm::FastVertex2ShapeScaling convexScaling;
	if(!idtScaleConvex)
			convexScaling.init(convexGeom.scale);

	const PxVec3 _shapeSpaceCenterOfMass = convexScaling * hullData->mCenterOfMass;
	const Vec3V shapeSpaceCenterOfMass = V3LoadU(_shapeSpaceCenterOfMass);

	const QuatV q0 = QuatVLoadU(&convexPose.q.x);
	const Vec3V p0 = V3LoadU(&convexPose.p.x);
	PsTransformV convexTransformV(p0, q0);

	const Vec3V vScale = V3LoadU_SafeReadW(convexGeom.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
	const QuatV vQuat = QuatVLoadU(&convexGeom.scale.rotation.x);
	ConvexHullV convexHull(hullData, V3Zero(), vScale, vQuat, idtScaleConvex);
	PX_ALIGN(16, PxU8 convexBuff[sizeof(SupportLocalImpl<ConvexHullV>)]);
	
	const FloatV convexMargin = CalculateMTDConvexMargin(hullData, vScale);
	const FloatV inflationV = FAdd(FLoad(inflation), convexMargin);
	PxReal boundInflation;
	FStore(inflationV, &boundInflation);

	Ps::Array<PxU32> tempContainer;
	tempContainer.reserve(128);

	Vec3V closestA = zeroV, closestB = zeroV, normal = zeroV;
	PxU32 triangleIndex = 0xfffffff;

	Vec3V translation = zeroV;

	const Cm::Matrix34 meshToWorldSkew = pose * triMeshGeom.scale;

	PolygonalData polyData;
	getPCMConvexData(convexHull, idtScaleConvex, polyData);
	
	FloatV mtd;
	FloatV distV;
	Vec3V center = p0;
	PxTransform tempConvexPose = convexPose;
	Vec3V worldNormal = zeroV, worldContactA = zeroV;//, worldContactB = zeroV;
	
	for(PxU32 i=0; i<iterations; ++i)
	{
		tempContainer.forceSize_Unsafe(0);

		//ML:: construct convex hull data
		V3StoreU(center, tempConvexPose.p);
		convexTransformV.p = center;
		SupportLocal* convexMap = (idtScaleConvex ? static_cast<SupportLocal*>(PX_PLACEMENT_NEW(convexBuff, SupportLocalImpl<ConvexHullNoScaleV>)(static_cast<ConvexHullNoScaleV&>(convexHull), convexTransformV, convexHull.vertex2Shape, convexHull.shape2Vertex, idtScaleConvex)) : 
		static_cast<SupportLocal*>(PX_PLACEMENT_NEW(convexBuff, SupportLocalImpl<ConvexHullV>)(convexHull, convexTransformV, convexHull.vertex2Shape, convexHull.shape2Vertex, idtScaleConvex)));

		convexMap->setShapeSpaceCenterofMass(shapeSpaceCenterOfMass);
		
		Box hullOBB;
		computeOBBAroundConvex(hullOBB, convexGeom, cm, tempConvexPose);

		hullOBB.extents += PxVec3(boundInflation);

		midPhaseQuery(triMeshGeom, pose, hullOBB, tempContainer);

		// Get results
		const PxU32 nbTriangles = tempContainer.size();
		if(!nbTriangles)
			break;
	
		// Move to AABB space
		const Cm::Matrix34 worldToConvex(tempConvexPose.getInverse());
		const Cm::Matrix34 meshToConvex = worldToConvex*meshToWorldSkew;

		const Ps::aos::Mat33V rot(V3LoadU(meshToConvex.m.column0), V3LoadU(meshToConvex.m.column1), V3LoadU(meshToConvex.m.column2));
		const Ps::aos::PsMatTransformV meshToConvexV(V3LoadU(meshToConvex.p), rot);

		bool hadContacts = false;

		const PxU32 nbBatches = (nbTriangles + BATCH_TRIANGLE_NUMBER-1)/BATCH_TRIANGLE_NUMBER;
		mtd = FMax();
		MTDTriangle triangles[BATCH_TRIANGLE_NUMBER];
		for(PxU32 a = 0; a < nbBatches; ++a)
		{
			const PxU32 startIndex = a * BATCH_TRIANGLE_NUMBER;
			const PxU32 nbTrigs = PxMin(nbTriangles - startIndex, BATCH_TRIANGLE_NUMBER);
			for(PxU32 k=0; k<nbTrigs; k++)
			{
				//triangle vertex space
				const PxU32 triangleIndex1 = tempContainer[startIndex+k];
				triMesh->getLocalTriangle(triangles[k], triangleIndex1, triMeshGeom.scale.hasNegativeDeterminant());
				triangles[k].extraTriData = getConvexEdgeFlags(extraTrigData, triangleIndex1);
			}

			//ML: mtd has back face culling, so if the capsule's center is below the triangle, we won't generate any contacts
			hadContacts = calculateMTD(polyData, convexMap, convexTransformV, meshToConvexV,  isDoubleSided, inflationV,  triangles, nbTrigs, startIndex, manifoldContacts, numContacts, normal, closestA, closestB, triangleIndex, mtd) || hadContacts;
		}
	
		if(!hadContacts)
			break;

		triangleIndex = tempContainer[triangleIndex];

		foundInitial = true;

		distV = mtd;
		worldNormal = convexTransformV.rotate(normal);
		worldContactA = convexTransformV.transform(closestA);
		if(FAllGrtrOrEq(FZero(), distV))
		{
			const Vec3V t = V3Scale(worldNormal, mtd);
			translation = V3Sub(translation, t);
			center = V3Sub(center, t);
		}
		else
		{
			if(i == 0)
			{
				//First iteration so keep this normal
				hit.distance = 0.f;
				V3StoreU(worldContactA, hit.position);
				V3StoreU(worldNormal, hit.normal);
				hit.faceIndex = triangleIndex;
				return true;
			}
			break;
		}
	}

	
	const FloatV translationF = V3Length(translation);
	distV = FNeg(translationF);

	const BoolV con = FIsGrtr(translationF, FZero());
	worldNormal = V3Sel(con, V3ScaleInv(translation, translationF), zeroV);

	if(foundInitial)
	{
		//transform closestA to world space
		FStore(distV, &hit.distance);
		V3StoreU(worldContactA, hit.position);
		V3StoreU(worldNormal, hit.normal);
		hit.faceIndex = triangleIndex;
	}
	return foundInitial;
}

bool physx::Gu::computeConvex_HeightFieldMTD(	const PxHeightFieldGeometry& heightFieldGeom, const PxTransform& pose, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose, PxReal inflation,
												bool isDoubleSided, const PxU32 flags, PxSweepHit& hit)
{
	using namespace Ps::aos;

	const HeightFieldUtil hfUtil(heightFieldGeom);
	
	const Vec3V zeroV = V3Zero();
	MeshPersistentContact manifoldContacts[64]; 
	PxU32 numContacts = 0;
	
	bool foundInitial = false;
	const PxU32 iterations = 2;

	ConvexMesh* cm = static_cast<ConvexMesh*>(convexGeom.convexMesh);

	ConvexHullData* hullData = &cm->getHull();

	const bool idtScaleConvex = convexGeom.scale.isIdentity();
	
	 Cm::FastVertex2ShapeScaling convexScaling;
	if(!idtScaleConvex)
			convexScaling.init(convexGeom.scale);

	const PxVec3 _shapeSpaceCenterOfMass = convexScaling * hullData->mCenterOfMass;
	const Vec3V shapeSpaceCenterOfMass = V3LoadU(_shapeSpaceCenterOfMass);

	const QuatV q0 = QuatVLoadU(&convexPose.q.x);
	const Vec3V p0 = V3LoadU(&convexPose.p.x);
	PsTransformV convexTransformV(p0, q0);

	const Vec3V vScale = V3LoadU_SafeReadW(convexGeom.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
	const QuatV vQuat = QuatVLoadU(&convexGeom.scale.rotation.x);
	ConvexHullV convexHull(hullData, zeroV, vScale, vQuat, idtScaleConvex);
	PX_ALIGN(16, PxU8 convexBuff[sizeof(SupportLocalImpl<ConvexHullV>)]);

	const FloatV convexMargin = CalculateMTDConvexMargin(hullData, vScale);
	const FloatV inflationV = FAdd(FLoad(inflation), convexMargin);
	PxReal boundInflation;
	FStore(inflationV, &boundInflation);

	Ps::Array<PxU32> tempContainer;
	tempContainer.reserve(128);

	Vec3V closestA = zeroV, closestB = zeroV, normal = zeroV;
	Vec3V worldNormal = zeroV, worldContactA = zeroV;//, worldContactB = zeroV;
	PxU32 triangleIndex = 0xfffffff;

	Vec3V translation = zeroV;

	PolygonalData polyData;
	getPCMConvexData(convexHull, idtScaleConvex, polyData);

	FloatV mtd;
	FloatV distV;
	Vec3V center = p0;
	PxTransform tempConvexPose = convexPose;
	const Cm::Matrix34 meshToWorldSkew(pose);
	
	for(PxU32 i=0; i<iterations; ++i)
	{
		tempContainer.forceSize_Unsafe(0);

		//ML:: construct convex hull data
	
		V3StoreU(center, tempConvexPose.p);
		convexTransformV.p = center;

		SupportLocal* convexMap = (idtScaleConvex ? static_cast<SupportLocal*>(PX_PLACEMENT_NEW(convexBuff, SupportLocalImpl<ConvexHullNoScaleV>)(static_cast<ConvexHullNoScaleV&>(convexHull), convexTransformV, convexHull.vertex2Shape, convexHull.shape2Vertex, idtScaleConvex)) : 
		static_cast<SupportLocal*>(PX_PLACEMENT_NEW(convexBuff, SupportLocalImpl<ConvexHullV>)(convexHull, convexTransformV, convexHull.vertex2Shape, convexHull.shape2Vertex, idtScaleConvex)));

		convexMap->setShapeSpaceCenterofMass(shapeSpaceCenterOfMass);
		
		Box hullOBB;
		computeOBBAroundConvex(hullOBB, convexGeom, cm, tempConvexPose);
		hullOBB.extents += PxVec3(boundInflation);

		const PxBounds3 bounds = PxBounds3::basisExtent(hullOBB.center, hullOBB.rot, hullOBB.extents);

		midPhaseQuery(hfUtil, pose, bounds, tempContainer, flags);

		// Get results
		const PxU32 nbTriangles = tempContainer.size();

		if(!nbTriangles)
			break;
	
		// Move to AABB space
		const Cm::Matrix34 worldToConvex(tempConvexPose.getInverse());
		const Cm::Matrix34 meshToConvex = worldToConvex*meshToWorldSkew;

		const Ps::aos::Mat33V rot(V3LoadU(meshToConvex.m.column0), V3LoadU(meshToConvex.m.column1), V3LoadU(meshToConvex.m.column2));
		const Ps::aos::PsMatTransformV meshToConvexV(V3LoadU(meshToConvex.p), rot);

		bool hadContacts = false;

		const PxU32 nbBatches = (nbTriangles + BATCH_TRIANGLE_NUMBER-1)/BATCH_TRIANGLE_NUMBER;
		mtd = FMax();
		MTDTriangle triangles[BATCH_TRIANGLE_NUMBER];
		for(PxU32 a = 0; a < nbBatches; ++a)
		{
			const PxU32 startIndex = a * BATCH_TRIANGLE_NUMBER;   
			const PxU32 nbTrigs = PxMin(nbTriangles - startIndex, BATCH_TRIANGLE_NUMBER);
			for(PxU32 k=0; k<nbTrigs; k++)
			{
				//triangle vertex space
				const PxU32 triangleIndex1 = tempContainer[startIndex+k];
				hfUtil.getTriangle(pose, triangles[k], NULL, NULL, triangleIndex1, false, false);
				triangles[k].extraTriData = ETD_CONVEX_EDGE_ALL;
			}

			//ML: mtd has back face culling, so if the capsule's center is below the triangle, we won't generate any contacts
			hadContacts = calculateMTD(polyData, convexMap, convexTransformV, meshToConvexV, isDoubleSided, inflationV,  triangles, nbTrigs, startIndex, manifoldContacts, numContacts, normal, closestA, closestB, triangleIndex, mtd) || hadContacts;
		}
	
		if(!hadContacts)
			break;

		triangleIndex = tempContainer[triangleIndex];

		foundInitial = true;

		distV = mtd;
		worldNormal = convexTransformV.rotate(normal);
		worldContactA = convexTransformV.transform(closestA);

		if(FAllGrtrOrEq(FZero(), distV))
		{
			const Vec3V t = V3Scale(worldNormal, mtd);
			translation = V3Sub(translation, t);
			center = V3Sub(center, t);
		}
		else
		{
			if(i == 0)
			{
				//First iteration so keep this normal
				hit.distance = 0.f;
				V3StoreU(worldContactA, hit.position);
				V3StoreU(worldNormal, hit.normal);
				hit.faceIndex = triangleIndex;
				return true;
			}
			break;
		}
	}


	const FloatV translationF = V3Length(translation);
	distV = FNeg(translationF);

	const BoolV con = FIsGrtr(translationF, FZero());
	worldNormal = V3Sel(con, V3ScaleInv(translation, translationF), zeroV);

	if(foundInitial)
	{
		//transform closestA to world space
		FStore(distV, &hit.distance);
		V3StoreU(worldContactA, hit.position);
		V3StoreU(worldNormal, hit.normal);
		hit.faceIndex = triangleIndex;
	}
	return foundInitial;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool physx::Gu::computeSphere_SphereMTD(const Sphere& sphere0, const Sphere& sphere1, PxSweepHit& hit)
{
	const PxVec3 delta = sphere1.center - sphere0.center;
	const PxReal d2 = delta.magnitudeSquared();
	const PxReal radiusSum = sphere0.radius + sphere1.radius;

	const PxReal d = manualNormalize(hit.normal, delta, d2);
	hit.distance = d - radiusSum ;
	hit.position = sphere0.center + hit.normal * sphere0.radius;
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool physx::Gu::computeSphere_CapsuleMTD( const Sphere& sphere, const Capsule& capsule, PxSweepHit& hit)
{
	const PxReal radiusSum = sphere.radius + capsule.radius;

	PxReal u;
	distancePointSegmentSquared(capsule, sphere.center, &u);

	const PxVec3 normal = capsule.getPointAt(u) -  sphere.center;
	
	const PxReal lenSq = normal.magnitudeSquared();
	const PxF32 d = manualNormalize(hit.normal, normal, lenSq);
	hit.distance = d - radiusSum;
	hit.position = sphere.center + hit.normal * sphere.radius;
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool physx::Gu::computeCapsule_CapsuleMTD(const Capsule& capsule0, const Capsule& capsule1, PxSweepHit& hit)
{
	PxReal s,t;
	distanceSegmentSegmentSquared(capsule0, capsule1, &s, &t);

	const PxReal radiusSum = capsule0.radius + capsule1.radius;

	const PxVec3 pointAtCapsule0 = capsule0.getPointAt(s);
	const PxVec3 pointAtCapsule1 = capsule1.getPointAt(t);

	const PxVec3 normal = pointAtCapsule0 - pointAtCapsule1;
	const PxReal lenSq = normal.magnitudeSquared();
	const PxF32 len = manualNormalize(hit.normal, normal, lenSq);
	hit.distance = len - radiusSum;
	hit.position = pointAtCapsule1 + hit.normal * capsule1.radius;
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool physx::Gu::computePlane_CapsuleMTD(const PxPlane& plane, const Capsule& capsule, PxSweepHit& hit)
{
	const PxReal d0 = plane.distance(capsule.p0);
	const PxReal d1 = plane.distance(capsule.p1);
	PxReal dmin;
	PxVec3 point;
	if(d0 < d1)
	{
		dmin = d0;
		point = capsule.p0;
	}
	else
	{
		dmin = d1;
		point = capsule.p1;
	}

	hit.normal		= plane.n;
	hit.distance	= dmin - capsule.radius;
	hit.position	= point - hit.normal * dmin;
	return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool physx::Gu::computePlane_BoxMTD(const PxPlane& plane, const Box& box, PxSweepHit& hit)
{
	PxVec3 pts[8];
	box.computeBoxPoints(pts);

	PxReal dmin = plane.distance(pts[0]);
	PxU32 index = 0;
	for(PxU32 i=1;i<8;i++)
	{
		const PxReal d = plane.distance(pts[i]);
		if(dmin > d)
		{
			index = i;
			dmin = d;
		}
	}
	hit.normal		= plane.n;
	hit.distance	= dmin;
	hit.position	= pts[index] - plane.n*dmin;
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool physx::Gu::computePlane_ConvexMTD(const PxPlane& plane, const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose, PxSweepHit& hit)
{
	const ConvexMesh* convexMesh = static_cast<const ConvexMesh*>(convexGeom.convexMesh);
	const Cm::FastVertex2ShapeScaling convexScaling(convexGeom.scale);
	PxU32 nbVerts = convexMesh->getNbVerts();
	const PxVec3* PX_RESTRICT verts = convexMesh->getVerts();

	PxVec3 worldPointMin = convexPose.transform(convexScaling * verts[0]);
	PxReal dmin = plane.distance(worldPointMin);
	for(PxU32 i=1;i<nbVerts;i++)
	{
		const PxVec3 worldPoint = convexPose.transform(convexScaling * verts[i]);
		const PxReal d = plane.distance(worldPoint);
		if(dmin > d)
		{
			dmin = d;
			worldPointMin = worldPoint;
		}
	}

	hit.normal		= plane.n;
	hit.distance	= dmin;
	hit.position	= worldPointMin - plane.n * dmin;
	return true;
}
