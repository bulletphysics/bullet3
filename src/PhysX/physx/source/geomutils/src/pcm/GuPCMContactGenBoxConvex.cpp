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

#include "GuGeometryUnion.h"
#include "GuPCMContactGen.h"
#include "GuPCMShapeConvex.h"
#include "CmRenderOutput.h"
#include "GuPCMContactGenUtil.h"
#include "PsVecMath.h"
#include "GuVecCapsule.h"
#include "GuVecBox.h"
#include "GuEPA.h"


#define PCM_USE_INTERNAL_OBJECT 1

using namespace physx;
using namespace Gu;
using namespace Ps::aos;

	//Precompute the convex data
	//     7+------+6			0 = ---
	//     /|     /|			1 = +--
	//    / |    / |			2 = ++-
	//   / 4+---/--+5			3 = -+-
	// 3+------+2 /    y   z	4 = --+
	//  | /    | /     |  /		5 = +-+
	//  |/     |/      |/		6 = +++
	// 0+------+1      *---x	7 = -++

namespace physx
{

namespace Gu
{
	
	static bool testFaceNormal(const PolygonalData& polyData0, const PolygonalData& polyData1, SupportLocal* map0, SupportLocal* map1, const PsMatTransformV& transform0To1, const PsMatTransformV& transform1To0, const FloatVArg contactDist, 
		FloatV& minOverlap, PxU32& feature, Vec3V& faceNormal, const FeatureStatus faceStatus, FeatureStatus& status)
	{
		PX_UNUSED(polyData1);
		
		FloatV _minOverlap = FMax();//minOverlap;
		PxU32  _feature = 0;
		Vec3V  _faceNormal = faceNormal;
		FloatV min0, max0;
		FloatV min1, max1;
	
		const Vec3V center1To0 = transform1To0.p;
		
#if PCM_USE_INTERNAL_OBJECT
		const Vec3V zeroV = V3Zero();
		const Vec3V shapeSpaceCenter1 = V3LoadU(polyData1.mCenter);
		const Vec3V internalCenter1In0 = transform1To0.transform(shapeSpaceCenter1);
		const FloatV internalRadius1 = FLoad(polyData1.mInternal.mRadius);
		const Vec3V internalExtents1 = V3LoadU(polyData1.mInternal.mExtents);
		const Vec3V negInternalExtents1 = V3Neg(internalExtents1);

#endif

		//in the local space of polyData0
		for(PxU32 i=0; i<polyData0.mNbPolygons; ++i)
		{
			const Gu::HullPolygonData& polygon = polyData0.mPolygons[i];

			const Vec3V minVert = V3LoadU(polyData0.mVerts[polygon.mMinIndex]);
			const FloatV planeDist = FLoad(polygon.mPlane.d);
			const Vec3V vertexSpacePlaneNormal = V3LoadU(polygon.mPlane.n);

			//transform plane n to shape space
			const Vec3V shapeSpacePlaneNormal = M33TrnspsMulV3(map0->shape2Vertex, vertexSpacePlaneNormal);

			const FloatV magnitude = FRecip(V3Length(shapeSpacePlaneNormal));
			//ML::use this to avoid LHS
			min0 = FMul(V3Dot(vertexSpacePlaneNormal, minVert), magnitude);
			max0 = FMul(FNeg(planeDist), magnitude);

			//normalize shape space normal
			const Vec3V n0 = V3Scale(shapeSpacePlaneNormal, magnitude);

			//calculate polyData1 projection
			//rotate polygon's normal into the local space of polyData1
			const Vec3V n1 = transform0To1.rotate(n0);


#if PCM_USE_INTERNAL_OBJECT
			
			//test internal object	
			//ML: we don't need to transform the normal into the vertex space. If polyData1 don't have scale,
			//the vertex2Shape matrix will be identity, shape space normal will be the same as vertex space's normal.
			//If polyData0 have scale, internalExtens1 will be 0.
			const Vec3V proj = V3Sel(V3IsGrtr(n1, zeroV), internalExtents1, negInternalExtents1);
			const FloatV radius = FMax(V3Dot(n1, proj), internalRadius1);
			const FloatV internalTrans = V3Dot(internalCenter1In0, n0);

			const FloatV _min1 = FSub(internalTrans, radius);
			const FloatV _max1 = FAdd(internalTrans, radius);

			const FloatV _min = FMax(min0, _min1);
			const FloatV _max = FMin(max0, _max1);

			const FloatV _tempOverlap = FSub(_max, _min);
			//const FloatV _tempOverlap = FSub(max0, _min1);	

			//Internal object overlaps more than current min, so can skip it
			//because (a) it isn't a separating axis and (b) it isn't the smallest axis
			if(FAllGrtr(_tempOverlap, _minOverlap))
			{
				continue;
			}

#endif

			const FloatV translate = V3Dot(center1To0, n0);
			map1->doSupport(n1, min1, max1);

			min1 = FAdd(translate, min1);
			max1 = FAdd(translate, max1);

			const BoolV con = BOr(FIsGrtr(min1, FAdd(max0, contactDist)), FIsGrtr(min0, FAdd(max1, contactDist)));

			if(BAllEqTTTT(con))
				return false;

			const FloatV tempOverlap = FSub(max0, min1);

			if(FAllGrtr(_minOverlap, tempOverlap))
			{
				_minOverlap = tempOverlap;
				_feature = i;
				_faceNormal = n0;
			}
		}   

		if(FAllGrtr(minOverlap, _minOverlap))
		{
			faceNormal = _faceNormal; 
			minOverlap = _minOverlap;
			status = faceStatus;
		}

		

		feature = _feature;

		return true;

	}


	//plane is in the shape space of polyData
	void buildPartialHull(const PolygonalData& polyData, SupportLocal* map, SeparatingAxes& validAxes, const Vec3VArg planeP, const Vec3VArg planeDir)
	{		
		const FloatV zero = FZero();
		const Vec3V dir = V3Normalize(planeDir);
		for(PxU32 i=0; i<polyData.mNbPolygons; ++i)
		{
			const Gu::HullPolygonData& polygon = polyData.mPolygons[i];
			const PxU8* inds = polyData.mPolygonVertexRefs + polygon.mVRef8;
			
			Vec3V v0 = M33MulV3(map->vertex2Shape, V3LoadU_SafeReadW(polyData.mVerts[inds[0]]));	// PT: safe because of the way vertex memory is allocated in ConvexHullData
		
			FloatV dist0 = V3Dot(dir, V3Sub(v0, planeP));

			for (PxU32 iStart = 0, iEnd = PxU32(polygon.mNbVerts - 1); iStart < polygon.mNbVerts; iEnd = iStart++)
			{
				const Vec3V v1 = M33MulV3(map->vertex2Shape, V3LoadU_SafeReadW(polyData.mVerts[inds[iEnd]]));	// PT: safe because of the way vertex memory is allocated in ConvexHullData
				
				const FloatV dist1 = V3Dot(dir, V3Sub(v1, planeP));

				const BoolV con = BOr(FIsGrtr(dist0, zero), FIsGrtr(dist1, zero));

				//cull edge if either of the vertex will on the positive size of the plane
				if(BAllEqTTTT(con))
				{
					const Vec3V tempV = V3Sub(v0, v1); 
					PxVec3 temp;
					V3StoreU(tempV, temp);
					validAxes.addAxis(temp.getNormalized());
				}

				v0 = v1;
				dist0 = dist1;
			}
		}
	}


	static bool testEdgeNormal(const PolygonalData& polyData0, const PolygonalData& polyData1, SupportLocal* map0, SupportLocal* map1, const PsMatTransformV& transform0To1, const PsMatTransformV& transform1To0, const FloatVArg contactDist,
		FloatV& minOverlap, Vec3V& edgeNormalIn0, const FeatureStatus edgeStatus, FeatureStatus& status)
	{
		
		FloatV overlap = minOverlap;
		FloatV min0, max0;
		FloatV min1, max1;
		const FloatV eps = FEps();

		const Vec3V shapeSpaceCenter0 = V3LoadU(polyData0.mCenter);
		const Vec3V shapeSpaceCenter1 = V3LoadU(polyData1.mCenter);

#if PCM_USE_INTERNAL_OBJECT
		const Vec3V zeroV = V3Zero();
		const Vec3V internalCenter1In0 = V3Sub(transform1To0.transform(shapeSpaceCenter1), shapeSpaceCenter0);
		const FloatV internalRadius1 = FLoad(polyData1.mInternal.mRadius);
		const Vec3V internalExtents1 = V3LoadU(polyData1.mInternal.mExtents);
		const Vec3V negInternalExtents1 = V3Neg(internalExtents1);

		const FloatV internalRadius0 = FLoad(polyData0.mInternal.mRadius);
		const Vec3V internalExtents0 = V3LoadU(polyData0.mInternal.mExtents);
		const Vec3V negInternalExtents0 = V3Neg(internalExtents0);
#endif

		const Vec3V center1To0 = transform1To0.p;

		//in polyData0 shape space
		const Vec3V dir0 = V3Sub(transform1To0.transform(shapeSpaceCenter1), shapeSpaceCenter0);
		const Vec3V support0 = map0->doSupport(dir0);

		//in polyData1 shape space
		const Vec3V dir1 = transform0To1.rotate(V3Neg(dir0));
		const Vec3V support1 = map1->doSupport(dir1);
		
		const Vec3V support0In1 = transform0To1.transform(support0);
		const Vec3V support1In0 = transform1To0.transform(support1);

		SeparatingAxes mSA0;
		SeparatingAxes mSA1;
		mSA0.reset();
		mSA1.reset();
		
		buildPartialHull(polyData0, map0, mSA0, support1In0, dir0);
		buildPartialHull(polyData1, map1, mSA1, support0In1, dir1);

		const PxVec3* axe0 = mSA0.getAxes();
		const PxVec3* axe1 = mSA1.getAxes();
		const PxU32 numAxe0 = mSA0.getNumAxes();
		const PxU32 numAxe1 = mSA1.getNumAxes();
		for(PxU32 i=0; i < numAxe0; ++i)
		{
			//axe0[i] is in the shape space of polyData0
			const Vec3V v0 = V3LoadU(axe0[i]);

			for(PxU32 j=0; j< numAxe1; ++j)
			{
				//axe1[j] is in the shape space of polyData1
				const Vec3V v1 = V3LoadU(axe1[j]);

				const Vec3V dir = V3Cross(v0, transform1To0.rotate(v1));
				const FloatV lenSq = V3Dot(dir, dir);
				if(FAllGrtr(eps, lenSq))
					continue;

				//n0 is in polyData0's local space
				const Vec3V n0 = V3Scale(dir, FRsqrt(lenSq));
				
				//n1 is in polyData1's local space
				const Vec3V n1 = transform0To1.rotate(n0);

#if PCM_USE_INTERNAL_OBJECT

				//ML: we don't need to transform the normal into the vertex space. If polyData1 don't have scale,
				//the vertex2Shape matrix will be identity, shape space normal will be the same as vertex space's normal.
				//If polyData0 have scale, internalExtens1 will be 0.
				//vertex space n1
				const Vec3V proj = V3Sel(V3IsGrtr(n1, zeroV), internalExtents1, negInternalExtents1);
				const FloatV radius = FMax(V3Dot(n1, proj), internalRadius1);

				const FloatV internalTrans = V3Dot(internalCenter1In0, n0);

				const FloatV _min1 = FSub(internalTrans, radius);
				const FloatV _max1 = FAdd(internalTrans, radius);

				const Vec3V proj0 = V3Sel(V3IsGrtr(n0, zeroV), internalExtents0, negInternalExtents0);
				const FloatV radius0 = FMax(V3Dot(n0, proj0), internalRadius0);

				const FloatV _max0 = radius0;
				const FloatV _min0 = FNeg(radius0);

				PX_ASSERT(FAllGrtrOrEq(_max0, _min0));
				PX_ASSERT(FAllGrtrOrEq(_max1, _min1));


				const FloatV _min = FMax(_min0, _min1);
				const FloatV _max = FMin(_max0, _max1);

				const FloatV _tempOverlap = FSub(_max, _min);

				//Internal object overlaps more than current min, so can skip it
				//because (a) it isn't a separating axis and (b) it isn't the smallest axis
				if(FAllGrtr(_tempOverlap, overlap))
				{
					continue;
				}

#endif
				//get polyData0's projection
				map0->doSupport(n0, min0, max0);

				const FloatV translate = V3Dot(center1To0, n0);

				//get polyData1's projection
				map1->doSupport(n1, min1, max1);

				min1 = FAdd(translate, min1);
				max1 = FAdd(translate, max1);

				const BoolV con = BOr(FIsGrtr(min1, FAdd(max0, contactDist)), FIsGrtr(min0, FAdd(max1, contactDist)));
				if(BAllEqTTTT(con))
					return false;

				const FloatV tempOverlap = FSub(max0, min1);

#if PCM_USE_INTERNAL_OBJECT
				PX_ASSERT(FAllGrtrOrEq(tempOverlap, _tempOverlap));
#endif

				if(FAllGrtr(overlap, tempOverlap))
				{
					overlap = tempOverlap;
					edgeNormalIn0 = n0;
					status = edgeStatus;
				}
			}
		}

		minOverlap = overlap;
		
		return true;

	}


	//contactNormal is in the space of polyData0
	void generatedContacts(PolygonalData& polyData0, PolygonalData& polyData1, const HullPolygonData& referencePolygon, const HullPolygonData& incidentPolygon,  
		SupportLocal* map0, SupportLocal* map1, const PsMatTransformV& transform0To1, PersistentContact* manifoldContacts, 
		PxU32& numContacts, const FloatVArg contactDist, Cm::RenderOutput* renderOutput)
	{

		PX_UNUSED(renderOutput);
		const FloatV zero = FZero();

		const PxU8* inds0 = polyData0.mPolygonVertexRefs + referencePolygon.mVRef8;

		//transform the plane normal to shape space
		const Vec3V contactNormal = V3Normalize(M33TrnspsMulV3(map0->shape2Vertex, V3LoadU(referencePolygon.mPlane.n)));
		
		//this is the matrix transform all points to the 2d plane
		const Mat33V rot = findRotationMatrixFromZAxis(contactNormal);

		const PxU8* inds1 = polyData1.mPolygonVertexRefs + incidentPolygon.mVRef8;


		Vec3V* points0In0 = reinterpret_cast<Vec3V*>(PxAllocaAligned(sizeof(Vec3V)*referencePolygon.mNbVerts, 16));
		Vec3V* points1In0 = reinterpret_cast<Vec3V*>(PxAllocaAligned(sizeof(Vec3V)*incidentPolygon.mNbVerts, 16));
		bool* points1In0Penetration = reinterpret_cast<bool*>(PxAlloca(sizeof(bool)*incidentPolygon.mNbVerts));
		FloatV* points1In0TValue = reinterpret_cast<FloatV*>(PxAllocaAligned(sizeof(FloatV)*incidentPolygon.mNbVerts, 16));

		//Transform all the verts from vertex space to shape space
		map0->populateVerts(inds0, referencePolygon.mNbVerts, polyData0.mVerts, points0In0);
		map1->populateVerts(inds1, incidentPolygon.mNbVerts, polyData1.mVerts, points1In0);

#if PCM_LOW_LEVEL_DEBUG
		Gu::PersistentContactManifold::drawPolygon(*renderOutput, map0->transform, points0In0, (PxU32)referencePolygon.mNbVerts, 0x00ff0000); 
		Gu::PersistentContactManifold::drawPolygon(*renderOutput, map1->transform, points1In0, (PxU32)incidentPolygon.mNbVerts,  0x0000ff00); 
#endif
	
		//This is used to calculate the project point when the 2D reference face points is inside the 2D incident face point
		const Vec3V sPoint = points1In0[0]; 
	
		PX_ASSERT(incidentPolygon.mNbVerts <= 64);
 
		Vec3V eps = Vec3V_From_FloatV(FEps());
		Vec3V max = Vec3V_From_FloatV(FMax());
		Vec3V nmax = V3Neg(max); 

		//transform reference polygon to 2d, calculate min and max
		Vec3V rPolygonMin= max;
		Vec3V rPolygonMax = nmax;
		for(PxU32 i=0; i<referencePolygon.mNbVerts; ++i)
		{
			points0In0[i] = M33MulV3(rot, points0In0[i]);
			rPolygonMin = V3Min(rPolygonMin, points0In0[i]);
			rPolygonMax = V3Max(rPolygonMax, points0In0[i]);
		}
		
		rPolygonMin = V3Sub(rPolygonMin, eps);
		rPolygonMax = V3Add(rPolygonMax, eps);

		const FloatV d = V3GetZ(points0In0[0]);
		const FloatV rd = FAdd(d, contactDist);

		Vec3V iPolygonMin= max; 
		Vec3V iPolygonMax = nmax;


		PxU32 inside = 0;
		for(PxU32 i=0; i<incidentPolygon.mNbVerts; ++i)
		{
			const Vec3V vert1 =points1In0[i]; //this still in polyData1's local space
			const Vec3V a = transform0To1.transformInv(vert1);
			points1In0[i] = M33MulV3(rot, a);
			const FloatV z = V3GetZ(points1In0[i]);
			points1In0TValue[i] = FSub(z, d);
			points1In0[i] = V3SetZ(points1In0[i], d);
			iPolygonMin = V3Min(iPolygonMin, points1In0[i]);
			iPolygonMax = V3Max(iPolygonMax, points1In0[i]);
			if(FAllGrtr(rd, z))
			{
				points1In0Penetration[i] = true;

				if(contains(points0In0, referencePolygon.mNbVerts, points1In0[i], rPolygonMin, rPolygonMax))
				{
					inside++;

					const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(contactNormal), points1In0TValue[i]);
					manifoldContacts[numContacts].mLocalPointA = vert1;
					manifoldContacts[numContacts].mLocalPointB = M33TrnspsMulV3(rot, points1In0[i]);
					manifoldContacts[numContacts++].mLocalNormalPen = localNormalPen;
					
				}
			}
			else
			{
				points1In0Penetration[i] = false;
			}
			
		}


		if(inside == incidentPolygon.mNbVerts)
		{
			return;
		}

		inside = 0;
		iPolygonMin = V3Sub(iPolygonMin, eps);
		iPolygonMax = V3Add(iPolygonMax, eps);

		const Vec3V incidentNormal = V3Normalize(M33TrnspsMulV3(map1->shape2Vertex, V3LoadU(incidentPolygon.mPlane.n)));
	
		const Vec3V contactNormalIn1 = transform0To1.rotate(contactNormal);

		for(PxU32 i=0; i<referencePolygon.mNbVerts; ++i)
		{
			if(contains(points1In0, incidentPolygon.mNbVerts, points0In0[i], iPolygonMin, iPolygonMax))
			{
				//const Vec3V vert0=Vec3V_From_PxVec3(polyData0.mVerts[inds0[i]]);
				const Vec3V vert0 = M33TrnspsMulV3(rot, points0In0[i]);
				const Vec3V a = transform0To1.transform(vert0);

				const FloatV nom = V3Dot(incidentNormal, V3Sub(sPoint, a)); 
				const FloatV denom = V3Dot(incidentNormal, contactNormalIn1);
				PX_ASSERT(FAllEq(denom, zero)==0);
				const FloatV t = FDiv(nom, denom);

				if(FAllGrtr(t, contactDist))
					continue;


				inside++;

				const Vec3V projPoint = V3ScaleAdd(contactNormalIn1, t, a);
				const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(contactNormal), t);
		
				manifoldContacts[numContacts].mLocalPointA = projPoint;
				manifoldContacts[numContacts].mLocalPointB = vert0;
				manifoldContacts[numContacts++].mLocalNormalPen = localNormalPen;

			}
		}

		if(inside == referencePolygon.mNbVerts)
			return;


		//(2) segment intesection
		for (PxU32 iStart = 0, iEnd = PxU32(incidentPolygon.mNbVerts - 1); iStart < incidentPolygon.mNbVerts; iEnd = iStart++)
		{
			if((!points1In0Penetration[iStart] && !points1In0Penetration[iEnd] ) )//|| (points1In0[i].status == POINT_OUTSIDE && points1In0[incidentIndex].status == POINT_OUTSIDE))
				continue;

	
			const Vec3V ipA = points1In0[iStart];
			const Vec3V ipB = points1In0[iEnd];

			Vec3V ipAOri = V3SetZ(points1In0[iStart], FAdd(points1In0TValue[iStart], d));
			Vec3V ipBOri = V3SetZ(points1In0[iEnd], FAdd(points1In0TValue[iEnd], d));

			const Vec3V iMin = V3Min(ipA, ipB);
			const Vec3V iMax = V3Max(ipA, ipB);
			

			for (PxU32 rStart = 0, rEnd = PxU32(referencePolygon.mNbVerts - 1); rStart < referencePolygon.mNbVerts; rEnd = rStart++) 
			{
	
				const Vec3V rpA = points0In0[rStart];
				const Vec3V rpB = points0In0[rEnd];

				const Vec3V rMin = V3Min(rpA, rpB);
				const Vec3V rMax = V3Max(rpA, rpB);
				
				const BoolV tempCon =BOr(V3IsGrtr(iMin, rMax), V3IsGrtr(rMin, iMax));
				const BoolV con = BOr(BGetX(tempCon), BGetY(tempCon));
		
				if(BAllEqTTTT(con))
					continue;
			
					
				FloatV a1 = signed2DTriArea(rpA, rpB, ipA);
				FloatV a2 = signed2DTriArea(rpA, rpB, ipB);


				if(FAllGrtr(zero, FMul(a1, a2)))
				{
					FloatV a3 = signed2DTriArea(ipA, ipB, rpA);
					FloatV a4 = signed2DTriArea(ipA, ipB, rpB);

					if(FAllGrtr(zero, FMul(a3, a4)))
					{
					
						//these two segment intersect

						const FloatV t = FDiv(a1, FSub(a2, a1));

						const Vec3V pBB = V3NegScaleSub(V3Sub(ipBOri, ipAOri), t, ipAOri); 
						const Vec3V pAA = V3SetZ(pBB, d);
						const Vec3V pA = M33TrnspsMulV3(rot, pAA);
						const Vec3V pB = transform0To1.transform(M33TrnspsMulV3(rot, pBB));
						const FloatV pen = FSub(V3GetZ(pBB), V3GetZ(pAA));
						if(FAllGrtr(pen, contactDist))
							continue;


						const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(contactNormal), pen);
						manifoldContacts[numContacts].mLocalPointA = pB;
						manifoldContacts[numContacts].mLocalPointB = pA;
						manifoldContacts[numContacts++].mLocalNormalPen = localNormalPen;
					}
				}
			}

		}
	}


	bool generateFullContactManifold(PolygonalData& polyData0, PolygonalData& polyData1, SupportLocal* map0, SupportLocal* map1,  PersistentContact* manifoldContacts, PxU32& numContacts,
		const FloatVArg contactDist, const Vec3VArg normal, const Vec3VArg closestA, const Vec3VArg closestB, const PxReal marginA, const PxReal marginB, const bool doOverlapTest, 
		Cm::RenderOutput* renderOutput, const PxReal toleranceLength)
	{
	
		const PsMatTransformV transform1To0V = map0->transform.transformInv(map1->transform);
		const PsMatTransformV transform0To1V = map1->transform.transformInv(map0->transform);

	
		if(doOverlapTest)
		{
			//if gjk fail, SAT based yes/no test
			FeatureStatus status = POLYDATA0;
			FloatV minOverlap = FMax();
			Vec3V minNormal = V3Zero();
	
			PxU32 feature0;
			//in the local space of polyData0, minNormal is in polyData0 space
			if(!testFaceNormal(polyData0, polyData1, map0, map1, transform0To1V, transform1To0V, contactDist, minOverlap, feature0, minNormal, POLYDATA0, status))
				return false;
			
			PxU32 feature1;
			//in the local space of polyData1, if minOverlap is overwrite inside this function, minNormal will be in polyData1 space
			if(!testFaceNormal(polyData1, polyData0, map1, map0, transform1To0V, transform0To1V, contactDist, minOverlap, feature1, minNormal, POLYDATA1, status))
				return false;

		
			bool doEdgeTest = false;
			
EdgeTest:
			if(doEdgeTest)
			{

				if(!testEdgeNormal(polyData0, polyData1, map0, map1, transform0To1V, transform1To0V, contactDist, minOverlap, minNormal, EDGE, status))
					return false;

				if(status != EDGE)
					return true;
			}

			
			if(status == POLYDATA0)
			{
				//minNormal is in the local space of polydata0
		
				const HullPolygonData& referencePolygon = polyData0.mPolygons[feature0];
				const Vec3V n = transform0To1V.rotate(minNormal);
				const HullPolygonData& incidentPolygon = polyData1.mPolygons[getPolygonIndex(polyData1, map1, n)];
				
				generatedContacts(polyData0, polyData1, referencePolygon,  incidentPolygon, map0, map1, transform0To1V, manifoldContacts, numContacts, contactDist, renderOutput);
				
				if (numContacts > 0)
				{
					const Vec3V nn = V3Neg(n);
					//flip the contacts
					for(PxU32 i=0; i<numContacts; ++i)
					{
						const Vec3V localPointB = manifoldContacts[i].mLocalPointB;
						manifoldContacts[i].mLocalPointB = manifoldContacts[i].mLocalPointA;
						manifoldContacts[i].mLocalPointA = localPointB;
						manifoldContacts[i].mLocalNormalPen = V4SetW(nn, V4GetW(manifoldContacts[i].mLocalNormalPen));
					}
				}
			}
			else if(status == POLYDATA1)
			{
				//minNormal is in the local space of polydata1
				const HullPolygonData& referencePolygon = polyData1.mPolygons[feature1];
				const HullPolygonData& incidentPolygon =  polyData0.mPolygons[getPolygonIndex(polyData0, map0, transform1To0V.rotate(minNormal))];
				
				//reference face is polyData1
				generatedContacts(polyData1, polyData0, referencePolygon,  incidentPolygon, map1, map0, transform1To0V, manifoldContacts, numContacts, contactDist, renderOutput);

			}
			else //if(status == EDGE0)
			{
				//minNormal is in the local space of polydata0
			
				const HullPolygonData& incidentPolygon = polyData0.mPolygons[getPolygonIndex(polyData0, map0, V3Neg(minNormal))];
				const HullPolygonData& referencePolygon = polyData1.mPolygons[getPolygonIndex(polyData1, map1, transform0To1V.rotate(minNormal))];
				generatedContacts(polyData1, polyData0, referencePolygon,  incidentPolygon, map1, map0, transform1To0V, manifoldContacts, numContacts, contactDist, renderOutput);
			}

			if(numContacts == 0 && !doEdgeTest)
			{
				doEdgeTest = true;
				goto EdgeTest;
			}

		}
		else
		{
			const PxReal lowerEps = toleranceLength * PCM_WITNESS_POINT_LOWER_EPS;
			const PxReal upperEps = toleranceLength * PCM_WITNESS_POINT_UPPER_EPS;
			const PxReal toleranceA = PxClamp(marginA, lowerEps, upperEps);
			const PxReal toleranceB = PxClamp(marginB, lowerEps, upperEps);

			const Vec3V negNormal = V3Neg(normal);
			const Vec3V normalIn0 = transform0To1V.rotateInv(normal);

			PxU32 faceIndex1 = getWitnessPolygonIndex(polyData1, map1, negNormal, closestB, toleranceB);
			PxU32 faceIndex0 = getWitnessPolygonIndex(polyData0, map0, normalIn0, transform0To1V.transformInv(closestA),
				toleranceA);

			const HullPolygonData& referencePolygon = polyData1.mPolygons[faceIndex1];
			const HullPolygonData& incidentPolygon = polyData0.mPolygons[faceIndex0];

			const Vec3V referenceNormal = V3Normalize(M33TrnspsMulV3(map1->shape2Vertex, V3LoadU(referencePolygon.mPlane.n)));
			const Vec3V incidentNormal = V3Normalize(M33TrnspsMulV3(map0->shape2Vertex, V3LoadU(incidentPolygon.mPlane.n)));
		
			const FloatV referenceProject = FAbs(V3Dot(referenceNormal, negNormal));
			const FloatV incidentProject = FAbs(V3Dot(incidentNormal, normalIn0));

			if (FAllGrtrOrEq(referenceProject, incidentProject))
			{
				generatedContacts(polyData1, polyData0, referencePolygon, incidentPolygon, map1, map0, transform1To0V, manifoldContacts, numContacts, contactDist, renderOutput);
			}
			else
			{
				generatedContacts(polyData0, polyData1, incidentPolygon, referencePolygon, map0, map1, transform0To1V, manifoldContacts, numContacts, contactDist, renderOutput);

				if (numContacts > 0)
				{
					const Vec3V n = transform0To1V.rotate(incidentNormal);

					const Vec3V nn = V3Neg(n);
					//flip the contacts
					for (PxU32 i = 0; i<numContacts; ++i)
					{
						const Vec3V localPointB = manifoldContacts[i].mLocalPointB;
						manifoldContacts[i].mLocalPointB = manifoldContacts[i].mLocalPointA;
						manifoldContacts[i].mLocalPointA = localPointB;
						manifoldContacts[i].mLocalNormalPen = V4SetW(nn, V4GetW(manifoldContacts[i].mLocalNormalPen));
					}
				}
			}
		}
	
		return true;
	}

	//This function called by box/convex and convex/convex pcm function
	bool addGJKEPAContacts(const GjkConvex* relativeConvex, const GjkConvex* localConvex, const PsMatTransformV& aToB, GjkStatus status,
		Gu::PersistentContact* manifoldContacts, const FloatV replaceBreakingThreshold, const FloatV tolerenceLength, GjkOutput& output,
		Gu::PersistentContactManifold& manifold)
	{
		bool doOverlapTest = false;
		if (status == GJK_DEGENERATE)
		{
			const Vec3V normal = output.normal;
			const FloatV costheta = V3Dot(output.searchDir, normal);
			if (FAllGrtr(costheta, FLoad(0.9999f)))
			{
				const Vec3V centreA = relativeConvex->getCenter();
				const Vec3V centreB = localConvex->getCenter();

				const Vec3V dir = V3Normalize(V3Sub(centreA, centreB));
				const FloatV theta = V3Dot(dir, normal);

				if (FAllGrtr(theta, FLoad(0.707f)))
				{
					addManifoldPoint(manifoldContacts, manifold, output, aToB, replaceBreakingThreshold);
				}
				else
				{
					doOverlapTest = true;
				}
			}
			else
			{
				doOverlapTest = true;
			}
		}
		else if (status == GJK_CONTACT)
		{
			addManifoldPoint(manifoldContacts, manifold, output, aToB, replaceBreakingThreshold);
		}
		else
		{
			PX_ASSERT(status == EPA_CONTACT);

			status = epaPenetration(*relativeConvex, *localConvex, manifold.mAIndice, manifold.mBIndice, manifold.mNumWarmStartPoints,
				true, tolerenceLength, output);
			if (status == EPA_CONTACT)
			{
				addManifoldPoint(manifoldContacts, manifold, output, aToB, replaceBreakingThreshold);
			}
			else
			{
				doOverlapTest = true;
			}
		}

		return doOverlapTest;
	}


	bool computeMTD(Gu::PolygonalData& polyData0, Gu::PolygonalData& polyData1,  SupportLocal* map0, SupportLocal* map1, Ps::aos::FloatV& penDepth, Ps::aos::Vec3V& normal)
	{
	
		using namespace Ps::aos;

		const PsMatTransformV transform1To0V = map0->transform.transformInv(map1->transform);
		const PsMatTransformV transform0To1V = map1->transform.transformInv(map0->transform);
	
		FeatureStatus status = POLYDATA0;
		FloatV minOverlap = FMax();
		Vec3V minNormal = V3Zero();
		const FloatV contactDist = FZero();

		PxU32 feature0;
		//in the local space of polyData0, minNormal is in polyData0 space
		if(!testFaceNormal(polyData0, polyData1, map0, map1, transform0To1V, transform1To0V, contactDist, minOverlap, feature0, minNormal, POLYDATA0, status))
			return false;
		
		PxU32 feature1;
		//in the local space of polyData1, if minOverlap is overwrite inside this function, minNormal will be in polyData1 space
		if(!testFaceNormal(polyData1, polyData0, map1, map0, transform1To0V, transform0To1V, contactDist, minOverlap, feature1, minNormal, POLYDATA1, status))
			return false;
	
		if(!testEdgeNormal(polyData0, polyData1, map0, map1, transform0To1V, transform1To0V, contactDist, minOverlap, minNormal, EDGE, status))
			return false;

		penDepth = minOverlap;
		if(status == POLYDATA1)
		{
			//minNormal is in the local space of polydata1
			normal = map1->transform.rotate(minNormal);
		}
		else
		{
			PX_ASSERT(status == POLYDATA0 || status == EDGE);
			//ML: status == POLYDATA0 or status == EDGE, minNormal is in the local space of polydata0
			normal = V3Neg(map0->transform.rotate(minNormal));
		}

		return true;
	}

}//Gu
}//physx
