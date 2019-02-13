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

#include "GuGeometryUnion.h"
#include "GuPCMTriangleContactGen.h"
#include "GuPCMContactConvexCommon.h"
#include "GuVecTriangle.h"
#include "GuBarycentricCoordinates.h"
#include "GuConvexEdgeFlags.h"

#if	PCM_LOW_LEVEL_DEBUG
#include "PxRenderBuffer.h"
#endif

#define EDGE_EDGE_GAUSS_MAP  0
#define BRUTE_FORCE_EDGE_EDGE	0

using namespace physx;
using namespace Gu;
using namespace Ps::aos;

namespace physx
{
	static bool testPolyFaceNormal(const Gu::TriangleV& triangle, const PolygonalData& polyData, SupportLocalImpl<TriangleV>* triMap, SupportLocal* polyMap,  const FloatVArg contactDist, 
		FloatV& minOverlap, PxU32& feature, Vec3V& faceNormal, const FeatureStatus faceStatus, FeatureStatus& status)
	{
		PX_UNUSED(triangle);

		FloatV _minOverlap = FMax();
		PxU32  _feature = 0;
		Vec3V  _faceNormal = faceNormal;
		FloatV min0, max0;
		FloatV min1, max1;
		const FloatV eps = FEps();

		if(polyMap->isIdentityScale)
		{
			//in the local space of polyData0
			for(PxU32 i=0; i<polyData.mNbPolygons; ++i)
			{
				const Gu::HullPolygonData& polygon = polyData.mPolygons[i];

				const Vec3V minVert = V3LoadU_SafeReadW(polyData.mVerts[polygon.mMinIndex]);	// PT: safe because of the way vertex memory is allocated in ConvexHullData
				const FloatV planeDist = FLoad(polygon.mPlane.d);
				//shapeSpace and vertexSpace are the same
				const Vec3V planeNormal = V3LoadU_SafeReadW(polygon.mPlane.n);	// PT: safe because 'd' follows 'n' in the plane class
	
				//ML::avoid lHS, don't use the exiting function
				min0 = V3Dot(planeNormal, minVert);
				max0 = FNeg(planeDist);

				triMap->doSupport(planeNormal, min1, max1);

				const BoolV con = BOr(FIsGrtr(min1, FAdd(max0, contactDist)), FIsGrtr(min0, FAdd(max1, contactDist)));

				if(BAllEqTTTT(con))
					return false;

				const FloatV tempOverlap = FSub(max0, min1);

				if(FAllGrtr(_minOverlap, tempOverlap))
				{
					_minOverlap = tempOverlap;
					_feature = i;
					_faceNormal = planeNormal;
				}
			}   
		}
		else
		{
		
			//in the local space of polyData0
			for(PxU32 i=0; i<polyData.mNbPolygons; ++i)
			{
				const Gu::HullPolygonData& polygon = polyData.mPolygons[i];

				const Vec3V minVert = V3LoadU_SafeReadW(polyData.mVerts[polygon.mMinIndex]);	// PT: safe because of the way vertex memory is allocated in ConvexHullData
				const FloatV planeDist = FLoad(polygon.mPlane.d);
				const Vec3V vertexSpacePlaneNormal = V3LoadU_SafeReadW(polygon.mPlane.n);	// PT: safe because 'd' follows 'n' in the plane class
				//transform plane n to shape space
				const Vec3V shapeSpacePlaneNormal = M33TrnspsMulV3(polyMap->shape2Vertex, vertexSpacePlaneNormal);

				const FloatV magnitude = FRsqrtFast(V3LengthSq(shapeSpacePlaneNormal)); //FRecip(V3Length(shapeSpacePlaneNormal));

				//ML::avoid lHS, don't use the exiting function
				min0 = FMul(V3Dot(vertexSpacePlaneNormal, minVert), magnitude);
				max0 = FMul(FNeg(planeDist), magnitude);

				//normalize the shapeSpacePlaneNormal
				const Vec3V planeN = V3Scale(shapeSpacePlaneNormal, magnitude);

				triMap->doSupport(planeN, min1, max1);

				const BoolV con = BOr(FIsGrtr(min1, FAdd(max0, contactDist)), FIsGrtr(min0, FAdd(max1, contactDist)));

				if(BAllEqTTTT(con))
					return false;

				const FloatV tempOverlap = FSub(max0, min1);

				if(FAllGrtr(_minOverlap, tempOverlap))
				{
					_minOverlap = tempOverlap;
					_feature = i;
					_faceNormal = planeN;
				}
			} 
		}

		if(FAllGrtr(minOverlap, FAdd(_minOverlap, eps)))
		{
			faceNormal = _faceNormal;
			minOverlap = _minOverlap;
			status = faceStatus;
		}

		feature = _feature;

		return true;

	}



	//triangle is in the local space of polyData
	static bool testTriangleFaceNormal(const TriangleV& triangle, const PolygonalData& polyData, SupportLocalImpl<TriangleV>* triMap, SupportLocal* polyMap, const FloatVArg contactDist, 
		FloatV& minOverlap, PxU32& feature, Vec3V& faceNormal, const FeatureStatus faceStatus, FeatureStatus& status)
	{
		PX_UNUSED(triMap);
		PX_UNUSED(polyData);

		FloatV min1, max1;
		const FloatV eps = FEps();

		const Vec3V triangleLocNormal = triangle.normal();

		const FloatV min0 = V3Dot(triangleLocNormal, triangle.verts[0]);
		const FloatV max0 = min0;

		//triangle normal is in the vertex space
		polyMap->doSupport(triangleLocNormal, min1, max1);

		const BoolV con = BOr(FIsGrtr(min1, FAdd(max0, contactDist)), FIsGrtr(min0, FAdd(max1, contactDist)));

		if(BAllEqTTTT(con))
			return false;

		minOverlap = FSub(FSub(max0, min1), eps);
		status = faceStatus;
		feature = 0;
		faceNormal=triangleLocNormal;

		return true;

	}

	static bool testPolyEdgeNormal(const TriangleV& triangle, const PxU8 triFlags, const PolygonalData& polyData, SupportLocalImpl<TriangleV>* triMap, SupportLocal* polyMap, const FloatVArg contactDist,
		FloatV& minOverlap, Vec3V& minNormal, const FeatureStatus edgeStatus, FeatureStatus& status)
	{
		PX_UNUSED(triFlags);
		FloatV overlap = minOverlap;
		FloatV min0, max0;
		FloatV min1, max1;
		const FloatV zero = FZero();
		const Vec3V eps2 = V3Splat(FLoad(1e-6f));
		
		const Vec3V v0 = M33MulV3(polyMap->shape2Vertex, triangle.verts[0]);
		const Vec3V v1 = M33MulV3(polyMap->shape2Vertex, triangle.verts[1]);
		const Vec3V v2 = M33MulV3(polyMap->shape2Vertex, triangle.verts[2]);

		TriangleV vertexSpaceTriangle(v0, v1, v2);

		PxU32 nbTriangleAxes = 0;
		Vec3V triangleAxes[3];
		for(PxI8 kStart = 0, kEnd =2; kStart<3; kEnd = kStart++)
		{
			bool active = (triFlags & (1 << (kEnd+3))) != 0;
	
			if(active)
			{
				const Vec3V p00 = vertexSpaceTriangle.verts[kStart];
				const Vec3V p01 = vertexSpaceTriangle.verts[kEnd];
				triangleAxes[nbTriangleAxes++] =  V3Sub(p01, p00);
			}
		}

		if(nbTriangleAxes == 0)
			return true;

		//create localTriPlane in the vertex space
		const Vec3V vertexSpaceTriangleNormal = vertexSpaceTriangle.normal();
	
		for(PxU32 i =0; i<polyData.mNbPolygons; ++i)
		{
			const Gu::HullPolygonData& polygon = polyData.mPolygons[i];
			const PxU8* inds = polyData.mPolygonVertexRefs + polygon.mVRef8;
			const Vec3V vertexSpacePlaneNormal = V3LoadU(polygon.mPlane.n);

			//fast culling. 
			if(FAllGrtr(V3Dot(vertexSpacePlaneNormal, vertexSpaceTriangleNormal), zero))
				continue;

			// Loop through polygon vertices == polygon edges;
			for(PxU32 lStart = 0, lEnd =PxU32(polygon.mNbVerts-1); lStart<polygon.mNbVerts; lEnd = PxU32(lStart++))
			{
				//in the vertex space
				const Vec3V p10 = V3LoadU_SafeReadW(polyData.mVerts[inds[lStart]]);	// PT: safe because of the way vertex memory is allocated in ConvexHullData
				const Vec3V p11 = V3LoadU_SafeReadW(polyData.mVerts[inds[lEnd]]);	// PT: safe because of the way vertex memory is allocated in ConvexHullData

				const Vec3V convexEdge = V3Sub(p11, p10);

				for (PxU32 j = 0; j < nbTriangleAxes; ++j)
				{

					const Vec3V currentPolyEdge = triangleAxes[j];
					const Vec3V v = V3Cross(convexEdge, currentPolyEdge);

					//two edges aren't parallel
					if ((!V3AllGrtr(eps2, V3Abs(v))) && (FAllGrtr(V3Dot(v, vertexSpaceTriangleNormal), zero)))
					{
						//transform the v back to the shape space
						const Vec3V shapeSpaceV = M33TrnspsMulV3(polyMap->shape2Vertex, v);
						const Vec3V n0 = V3Normalize(shapeSpaceV);
						triMap->doSupport(n0, min0, max0);
						polyMap->doSupport(n0, min1, max1);
						const BoolV con = BOr(FIsGrtr(min1, FAdd(max0, contactDist)), FIsGrtr(min0, FAdd(max1, contactDist)));
						if (BAllEqTTTT(con))
							return false;

						const FloatV tempOverlap = FSub(max0, min1);

						if (FAllGrtr(overlap, tempOverlap))
						{
							overlap = tempOverlap;
							minNormal = n0;
							status = edgeStatus;
						}

					}
				}
				
			}
		}
		minOverlap = overlap;
		
		return true;

	}

#if BRUTE_FORCE_EDGE_EDGE


	bool testPolyEdgeNormalBruteForce(const TriangleV& triangle, const PxU8 triFlags, const PolygonalData& polyData, SupportLocalImpl<TriangleV>* triMap, SupportLocal* polyMap, const FloatVArg contactDist,
		FloatV& minOverlap, Vec3V& minNormal, const FeatureStatus edgeStatus, FeatureStatus& status)
	{
		PX_UNUSED(triFlags);
		FloatV min0, max0;
		FloatV min1, max1;

		FloatV bestDist = FLoad(PX_MAX_F32);
		Vec3V bestAxis = V3Zero();

		const Vec3V eps2 = V3Splat(FLoad(1e-6));

		PxU32 bestPolyIndex = 0;
		PxU32 bestStart = 0;
		PxU32 bestEnd = 0;
		PxI8 bestTriStart = 0;
		PxI8 bestTriEnd = 0;

		for(PxU32 i =0; i<polyData.mNbPolygons; ++i)
		{
			const Gu::HullPolygonData& polygon = polyData.mPolygons[i];
			const PxU8* inds = polyData.mPolygonVertexRefs + polygon.mVRef8;
			
			// Loop through polygon vertices == polygon edges;
			for(PxU32 lStart = 0, lEnd =PxU32(polygon.mNbVerts-1); lStart<polygon.mNbVerts; lEnd = PxU32(lStart++))
			{
				//in the vertex space
				const Vec3V p10 = V3LoadU_SafeReadW(polyData.mVerts[inds[lStart]]);	// PT: safe because of the way vertex memory is allocated in ConvexHullData
				const Vec3V p11 = V3LoadU_SafeReadW(polyData.mVerts[inds[lEnd]]);	// PT: safe because of the way vertex memory is allocated in ConvexHullData

				//shape sapce
				const Vec3V vertex10 = M33MulV3(polyMap->vertex2Shape, p10);
				const Vec3V vertex11 = M33MulV3(polyMap->vertex2Shape, p11);

				const Vec3V convexEdge = V3Sub(vertex11, vertex10);

				for (PxI8 kEnd = 0, kStart = 2; kEnd<3; kStart = kEnd++)
				{

					const Vec3V triVert0 = triangle.verts[kStart];
					const Vec3V triVert1 = triangle.verts[kEnd];

					const Vec3V triangleEdge = V3Sub(triVert1, triVert0);
					const Vec3V v = V3Cross(convexEdge, triangleEdge);

					if (!V3AllGrtr(eps2, V3Abs(v)))
					{
						//transform the v back to the shape space
						const Vec3V n0 = V3Normalize(v);
						triMap->doSupport(n0, min0, max0);
						polyMap->doSupport(n0, min1, max1);
						const BoolV con = BOr(FIsGrtr(min1, FAdd(max0, contactDist)), FIsGrtr(min0, FAdd(max1, contactDist)));
						if(BAllEqTTTT(con))
							return false;

						const FloatV tempOverlap = FSub(max0, min1);

						if (FAllGrtr(bestDist, tempOverlap))
						{
							bestDist = tempOverlap;
							bestAxis = n0;
							bestPolyIndex = i;
							bestStart = lStart;
							bestEnd = lEnd;
							bestTriStart = kStart;
							bestTriEnd = kEnd;
						}

					}
				}
			}
		}

		if (FAllGrtr(minOverlap, bestDist))
		{
			minOverlap = bestDist;
			minNormal = bestAxis;
			status = edgeStatus;
		}

		return true;

	}

	bool testPolyEdgeNormalBruteForceVertsByEdges(const TriangleV& triangle, const PxU8 triFlags, const PolygonalData& polyData, SupportLocalImpl<TriangleV>* triMap, SupportLocal* polyMap, const FloatVArg contactDist,
		FloatV& minOverlap, Vec3V& minNormal, const FeatureStatus edgeStatus, FeatureStatus& status, PxU32& bestEdgeIndex, PxI8& bestTriStart, PxI8& bestTriEnd)
	{

		PX_UNUSED(triFlags);

		const PxU32 numConvexEdges = polyData.mNbEdges;
		const PxU16* verticesByEdges16 = polyData.mVerticesByEdges;
		const PxVec3* vertices = polyData.mVerts;

		FloatV bestDist = FLoad(PX_MAX_F32);
		Vec3V bestAxis = V3Zero();
		const Vec3V eps2 = V3Splat(FLoad(1e-6));

		for (PxU32 convexEdgeIdx = 0; convexEdgeIdx < numConvexEdges; ++convexEdgeIdx)
		{
			const PxU16 v0idx1 = verticesByEdges16[convexEdgeIdx * 2];
			const PxU32 v1idx1 = verticesByEdges16[convexEdgeIdx * 2 + 1];

			//shape sapce
			const Vec3V vertex10 = M33MulV3(polyMap->vertex2Shape, V3LoadU(vertices[v0idx1]));
			const Vec3V vertex11 = M33MulV3(polyMap->vertex2Shape, V3LoadU(vertices[v1idx1]));

			Vec3V convexEdge = V3Sub(vertex11, vertex10);


			for (PxI8 kEnd = 0, kStart = 2; kEnd<3; kStart = kEnd++)
			{
				const Vec3V triVert0 = triangle.verts[kStart];
				const Vec3V triVert1 = triangle.verts[kEnd];
				const Vec3V triEdge = V3Sub(triVert1, triVert0);

				// compute the separation along this axis in vertex space
				Vec3V axis = V3Cross(convexEdge, triEdge);

				if (!V3AllGrtr(eps2, V3Abs(axis)))
				{
					axis = V3Normalize(axis);

					FloatV min0, max0;
					FloatV min1, max1;
					triMap->doSupport(axis, min0, max0);
					polyMap->doSupport(axis, min1, max1);
					const BoolV con = BOr(FIsGrtr(min1, FAdd(max0, contactDist)), FIsGrtr(min0, FAdd(max1, contactDist)));
					if (BAllEqTTTT(con))
						return false;

					const FloatV dist0 = FSub(max0, min1);
					const FloatV dist1 = FSub(max1, min0);

					if (FAllGrtr(dist0, dist1))
						axis = V3Neg(axis);

					const FloatV dist = FMin(dist0, dist1);

					if (FAllGrtr(bestDist, dist))
					{
						bestDist = dist;
						bestAxis = axis;

						bestEdgeIndex = convexEdgeIdx;
						bestTriStart = kStart;
						bestTriEnd = kEnd;
					}
				}
			}

		}

		if (FAllGrtr(minOverlap, bestDist))
		{
			minOverlap = bestDist;
			minNormal = bestAxis;
			status = edgeStatus;

		}
		return true;

	}


#endif

#if EDGE_EDGE_GAUSS_MAP



	bool isMinkowskiFace(const Vec3V& A, const Vec3V& B, const Vec3V& B_x_A, const Vec3V& C, const Vec3V& D, const Vec3V& D_x_C)
	{

		const FloatV zero = FZero();
		// Two edges build a face on the Minkowski sum if the associated arcs AB and CD intersect on the Gauss map. 
		// The associated arcs are defined by the adjacent face normals of each edge.  
		const FloatV CBA = V3Dot(C, B_x_A);
		const FloatV DBA = V3Dot(D, B_x_A);
		const FloatV ADC = V3Dot(A, D_x_C);
		const FloatV BDC = V3Dot(B, D_x_C);

		const BoolV con0 = FIsGrtrOrEq(zero, FMul(CBA, DBA));
		const BoolV con1 = FIsGrtrOrEq(zero, FMul(ADC, BDC));
		const BoolV con2 = FIsGrtr(FMul(CBA, BDC), zero);

		const BoolV con = BAnd(con2, BAnd(con0, con1));

		return (BAllEqTTTT(con) != 0);

		//return CBA * DBA < 0.0f && ADC * BDC < 0.0f && CBA * BDC > 0.0f;
	}

#define SAT_VARIFY	0

	bool testPolyEdgeNormalGaussMap(const TriangleV& triangle, const PxU8 triFlags, const PolygonalData& polyData, SupportLocalImpl<TriangleV>* triMap,  SupportLocal* polyMap, const FloatVArg contactDist,
		FloatV& minOverlap, Vec3V& minNormal, const FeatureStatus edgeStatus, FeatureStatus& status, PxU32& gaussMapBestEdgeIndex, PxI8& gaussMapBestTriStart, PxI8& gaussMapBestTriEnd)
	{
	
		PX_UNUSED(triFlags);
		const FloatV zero = FZero();

		const Vec3V triNormal = triangle.normal();

		const PxU32 numConvexEdges = polyData.mNbEdges;
		const PxU16* verticesByEdges16 = polyData.mVerticesByEdges;
		const PxU8* facesByEdges8 = polyData.mFacesByEdges;
		const PxVec3* vertices = polyData.mVerts;

		FloatV bestDist = FLoad(-PX_MAX_F32);
		Vec3V axis = V3Zero();
		Vec3V bestAxis = V3Zero();
		const Vec3V eps2 = V3Splat(FLoad(1e-6));


		//Center is in shape space
		const Vec3V shapeSpaceCOM =V3LoadU(polyData.mCenter);
		const Vec3V vertexSpaceCOM = M33MulV3(polyMap->shape2Vertex, shapeSpaceCOM);

		PxVec3 vertexCOM;
		V3StoreU(vertexSpaceCOM, vertexCOM);

		for (PxU32 convexEdgeIdx = 0; convexEdgeIdx < numConvexEdges; ++convexEdgeIdx)
		{

			const PxU16 v0idx1 = verticesByEdges16[convexEdgeIdx*2];
			const PxU32 v1idx1 = verticesByEdges16[convexEdgeIdx * 2 + 1];

			const PxU8 f10 = facesByEdges8[convexEdgeIdx * 2];
			const PxU8 f11 = facesByEdges8[convexEdgeIdx * 2 + 1];

			//shape sapce
			const Vec3V vertex10 = M33MulV3(polyMap->vertex2Shape, V3LoadU(vertices[v0idx1]));
			const Vec3V vertex11 = M33MulV3(polyMap->vertex2Shape, V3LoadU(vertices[v1idx1]));

			Vec3V convexEdge = V3Sub(vertex11, vertex10);

			const Gu::HullPolygonData& face0 = polyData.mPolygons[f10];
			const Gu::HullPolygonData& face1 = polyData.mPolygons[f11];

			const Vec3V convexNormal0 = M33TrnspsMulV3(polyMap->shape2Vertex, V3LoadU(face0.mPlane.n));
			const Vec3V convexNormal1 = M33TrnspsMulV3(polyMap->shape2Vertex, V3LoadU(face1.mPlane.n));

			float signDist0 = face0.mPlane.distance(vertexCOM);
			float signDist1 = face1.mPlane.distance(vertexCOM);

			PX_ASSERT(signDist0 < 0.f);
			PX_ASSERT(signDist1 < 0.f);

			for (PxI8 kEnd = 0, kStart = 2; kEnd<3; kStart = kEnd++)
			{ 
				
				const Vec3V triVert0 = triangle.verts[kStart];
				const Vec3V triVert1 = triangle.verts[kEnd];
				const Vec3V triEdge = V3Sub(triVert1, triVert0);

				//if (isMinkowskiFace(convexNormal0, convexNormal1, V3Neg(convexEdge), V3Neg(triNormal), triNormal, V3Neg(triEdge)))
				if (isMinkowskiFace(convexNormal0, convexNormal1, convexEdge, V3Neg(triNormal), triNormal, triEdge))
				{

					// compute the separation along this axis in vertex space
					axis = V3Cross(convexEdge, triEdge);

					if (!V3AllGrtr(eps2, V3Abs(axis)))
					{
						axis = V3Normalize(axis);

						const Vec3V v = V3Sub(vertex10, shapeSpaceCOM);

						// ensure the axis is outward pointing on the edge on the minkowski sum. Assure normal points from convex hull to triangle
						const FloatV temp = V3Dot(axis, v);
						if (FAllGrtr(zero, temp))
							axis = V3Neg(axis);

						//compute the distance from any of the verts in the triangle to plane(axis, vertex10)(n.dot(p-a))
						const Vec3V ap = V3Sub(triVert0, vertex10);

						const FloatV dist = V3Dot(axis, ap);

						if (FAllGrtr(dist, contactDist))
							return false;

#if SAT_VARIFY
						FloatV min0, max0;
						FloatV min1, max1;
						triMap->doSupport(V3Neg(axis), min0, max0);
						polyMap->doSupport(V3Neg(axis), min1, max1);
						const BoolV con = BOr(FIsGrtr(min1, FAdd(max0, contactDist)), FIsGrtr(min0, FAdd(max1, contactDist)));
						if (BAllEqTTTT(con))
							return false;

						/*const FloatV dist = FSub(max0, min1);

						if (FAllGrtr(bestDist, dist))
						{
						bestDist = dist;
						bestAxis = axis;
						}*/
						const FloatV tempDist = FSub(max0, min1);

						const FloatV dif = FAbs(FAdd(tempDist, dist));
						PX_UNUSED(dif);
						PX_ASSERT(FAllGrtr(FLoad(1e-4f), dif));

#endif

						if (FAllGrtr(dist, bestDist))
						{
							bestDist = dist;
							bestAxis = axis;

							gaussMapBestEdgeIndex = convexEdgeIdx;
							gaussMapBestTriStart = kStart;
							gaussMapBestTriEnd = kEnd;
						}
					}
				}
			}
			
		}

		if (FAllGrtr(minOverlap, bestDist))
		{
			minOverlap = bestDist;
			minNormal = bestAxis;
			status = edgeStatus;
		}
		return true;
			
	}

#endif

	static PX_FORCE_INLINE PxU32 addMeshContacts(MeshPersistentContact* manifoldContacts, const Vec3V& pA, const Vec3V& pB, const Vec4V& normalPen, const PxU32 triangleIndex, const PxU32 numContacts)
	{
		manifoldContacts[numContacts].mLocalPointA = pA;
		manifoldContacts[numContacts].mLocalPointB = pB;
		manifoldContacts[numContacts].mLocalNormalPen = normalPen;
		manifoldContacts[numContacts].mFaceIndex = triangleIndex;
		return numContacts+1;
	}


	static void generatedTriangleContacts(const Gu::TriangleV& triangle, const PxU32 triangleIndex, const PxU8/* _triFlags*/, const Gu::PolygonalData& polyData1, const Gu::HullPolygonData& incidentPolygon,  Gu::SupportLocal* map1, Gu::MeshPersistentContact* manifoldContacts, PxU32& numManifoldContacts, 
		const Ps::aos::FloatVArg contactDist, const Ps::aos::Vec3VArg contactNormal, Cm::RenderOutput* renderOutput)
	{

		PX_UNUSED(renderOutput);
		using namespace Ps::aos;

		//PxU8 triFlags = _triFlags;
		const PxU32 previousContacts = numManifoldContacts;
		
		const FloatV zero = FZero();
	
		const Mat33V rot = findRotationMatrixFromZAxis(contactNormal);

		const PxU8* inds1 = polyData1.mPolygonVertexRefs + incidentPolygon.mVRef8;

		Vec3V points0In0[3];
		Vec3V* points1In0 = reinterpret_cast<Vec3V*>(PxAllocaAligned(sizeof(Vec3V)*incidentPolygon.mNbVerts, 16));
		FloatV* points1In0TValue = reinterpret_cast<FloatV*>(PxAllocaAligned(sizeof(FloatV)*incidentPolygon.mNbVerts, 16));
		bool* points1In0Penetration = reinterpret_cast<bool*>(PxAlloca(sizeof(bool)*incidentPolygon.mNbVerts));
		

		points0In0[0] = triangle.verts[0];
		points0In0[1] = triangle.verts[1];
		points0In0[2] = triangle.verts[2];



		//Transform all the verts from vertex space to shape space
		map1->populateVerts(inds1, incidentPolygon.mNbVerts, polyData1.mVerts, points1In0);

#if PCM_LOW_LEVEL_DEBUG
		Gu::PersistentContactManifold::drawPolygon(*renderOutput, map1->transform, points1In0, incidentPolygon.mNbVerts, (PxU32)PxDebugColor::eARGB_RED);
		//Gu::PersistentContactManifold::drawTriangle(*renderOutput, map1->transform.transform(points1In0[0]), map1->transform.transform(points0In0[1]), map1->transform.transform(points0In0[2]), (PxU32)PxDebugColor::eARGB_BLUE);
#endif
 
		Vec3V eps = Vec3V_From_FloatV(FEps());
		Vec3V max = Vec3V_From_FloatV(FMax());
		Vec3V nmax = V3Neg(max); 

		//transform reference polygon to 2d, calculate min and max
		Vec3V rPolygonMin= max;
		Vec3V rPolygonMax = nmax;
		for(PxU32 i=0; i<3; ++i)
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
			points1In0[i] = M33MulV3(rot, vert1);
			const FloatV z = V3GetZ(points1In0[i]);
			points1In0TValue[i] = FSub(z, d);
			points1In0[i] = V3SetZ(points1In0[i], d);
			iPolygonMin = V3Min(iPolygonMin, points1In0[i]);
			iPolygonMax = V3Max(iPolygonMax, points1In0[i]);

			bool penetrated = false;
			
			if (FAllGrtr(rd, z))
			{
				penetrated = true;

				if (contains(points0In0, 3, points1In0[i], rPolygonMin, rPolygonMax))
				{
					inside++;

					//add this contact to the buffer
					const FloatV t = V3Dot(contactNormal, V3Sub(triangle.verts[0], vert1));
					const Vec3V projectPoint = V3ScaleAdd(contactNormal, t, vert1);
					const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(contactNormal), FNeg(t));
					numManifoldContacts = addMeshContacts(manifoldContacts, vert1, projectPoint, localNormalPen, triangleIndex, numManifoldContacts);

					//if the numContacts are more than GU_MESH_CONTACT_REDUCTION_THRESHOLD, we need to do contact reduction
					const PxU32 numContacts = numManifoldContacts - previousContacts;
					if (numContacts >= GU_MESH_CONTACT_REDUCTION_THRESHOLD)
					{
						//a polygon has more than GU_MESH_CONTACT_REDUCTION_THRESHOLD(16) contacts with this triangle, we will reduce
						//the contacts to GU_SINGLE_MANIFOLD_SINGLE_POLYGONE_CACHE_SIZE(4) points
						Gu::SinglePersistentContactManifold::reduceContacts(&manifoldContacts[previousContacts], numContacts);
						numManifoldContacts = previousContacts + GU_SINGLE_MANIFOLD_SINGLE_POLYGONE_CACHE_SIZE;
					}
				}
			}

			points1In0Penetration[i] = penetrated;
		}



		if(inside == incidentPolygon.mNbVerts)
		{
			return;
		}

		inside = 0;
		iPolygonMin = V3Sub(iPolygonMin, eps);
		iPolygonMax = V3Add(iPolygonMax, eps);

		const Vec3V incidentNormal = V3Normalize(M33TrnspsMulV3(map1->shape2Vertex, V3LoadU(incidentPolygon.mPlane.n)));
		const FloatV iPlaneD = V3Dot(incidentNormal, M33MulV3(map1->vertex2Shape, V3LoadU(polyData1.mVerts[inds1[0]])));

		for(PxU32 i=0; i<3; ++i)
		{
			if(contains(points1In0, incidentPolygon.mNbVerts, points0In0[i], iPolygonMin, iPolygonMax))
			{
				
				inside++;

				const Vec3V vert0 = M33TrnspsMulV3(rot, points0In0[i]);
				const FloatV t = FSub(V3Dot(incidentNormal, vert0), iPlaneD);

				if(FAllGrtr(t, contactDist))
					continue;


				const Vec3V projPoint = V3NegScaleSub(incidentNormal, t, vert0);

				const Vec3V v = V3Sub(projPoint, vert0);
				const FloatV t3 = V3Dot(v, contactNormal);

				const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(contactNormal), t3);
				numManifoldContacts = addMeshContacts(manifoldContacts, projPoint, vert0, localNormalPen, triangleIndex, numManifoldContacts);

				//if the numContacts are more than GU_MESH_CONTACT_REDUCTION_THRESHOLD, we need to do contact reduction
				const PxU32 numContacts = numManifoldContacts - previousContacts;
				if(numContacts >= GU_MESH_CONTACT_REDUCTION_THRESHOLD)
				{
					//a polygon has more than GU_MESH_CONTACT_REDUCTION_THRESHOLD(16) contacts with this triangle, we will reduce
					//the contacts to GU_SINGLE_MANIFOLD_SINGLE_POLYGONE_CACHE_SIZE(4) points
					Gu::SinglePersistentContactManifold::reduceContacts(&manifoldContacts[previousContacts], numContacts);
					numManifoldContacts = previousContacts + GU_SINGLE_MANIFOLD_SINGLE_POLYGONE_CACHE_SIZE;
				}
			}
				
		}


		if(inside == 3)
			return;  
	
		//(2) segment intesection
		for (PxU32 rStart = 0, rEnd = 2; rStart < 3; rEnd = rStart++)
		{


			const Vec3V rpA = points0In0[rStart];
			const Vec3V rpB = points0In0[rEnd];

			const Vec3V rMin = V3Min(rpA, rpB);
			const Vec3V rMax = V3Max(rpA, rpB);

			for (PxU32 iStart = 0, iEnd = PxU32(incidentPolygon.mNbVerts - 1); iStart < incidentPolygon.mNbVerts; iEnd = iStart++)
			{
				if ((!points1In0Penetration[iStart] && !points1In0Penetration[iEnd]))//|| (points1In0[i].status == POINT_OUTSIDE && points1In0[incidentIndex].status == POINT_OUTSIDE))
					continue;

				const Vec3V ipA = points1In0[iStart];
				const Vec3V ipB = points1In0[iEnd];

				const Vec3V iMin = V3Min(ipA, ipB);
				const Vec3V iMax = V3Max(ipA, ipB);

				const BoolV tempCon = BOr(V3IsGrtr(iMin, rMax), V3IsGrtr(rMin, iMax));
				const BoolV con = BOr(BGetX(tempCon), BGetY(tempCon));

				if (BAllEqTTTT(con))
					continue;

				FloatV a1 = signed2DTriArea(rpA, rpB, ipA);
				FloatV a2 = signed2DTriArea(rpA, rpB, ipB);

				if (FAllGrtr(zero, FMul(a1, a2)))
				{
					FloatV a3 = signed2DTriArea(ipA, ipB, rpA);
					FloatV a4 = signed2DTriArea(ipA, ipB, rpB);

					if (FAllGrtr(zero, FMul(a3, a4)))
					{

						//these two segment intersect in 2d
						const FloatV t = FMul(a1, FRecip(FSub(a2, a1)));

						const Vec3V ipAOri = V3SetZ(points1In0[iStart], FAdd(points1In0TValue[iStart], d));
						const Vec3V ipBOri = V3SetZ(points1In0[iEnd], FAdd(points1In0TValue[iEnd], d));

						const Vec3V pBB = V3NegScaleSub(V3Sub(ipBOri, ipAOri), t, ipAOri);
						const Vec3V pAA = V3SetZ(pBB, d);
						const Vec3V pA = M33TrnspsMulV3(rot, pAA);
						const Vec3V pB = M33TrnspsMulV3(rot, pBB);
						const FloatV pen = FSub(V3GetZ(pBB), V3GetZ(pAA));

						if (FAllGrtr(pen, contactDist))
							continue;

						const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(contactNormal), pen);
						numManifoldContacts = addMeshContacts(manifoldContacts, pB, pA, localNormalPen, triangleIndex, numManifoldContacts);

						//if the numContacts are more than GU_MESH_CONTACT_REDUCTION_THRESHOLD, we need to do contact reduction
						const PxU32 numContacts = numManifoldContacts - previousContacts;
						if (numContacts >= GU_MESH_CONTACT_REDUCTION_THRESHOLD)
						{
							//a polygon has more than GU_MESH_CONTACT_REDUCTION_THRESHOLD(16) contacts with this triangle, we will reduce
							//the contacts to GU_SINGLE_MANIFOLD_SINGLE_POLYGONE_CACHE_SIZE(4) points
							Gu::SinglePersistentContactManifold::reduceContacts(&manifoldContacts[previousContacts], numContacts);
							numManifoldContacts = previousContacts + GU_SINGLE_MANIFOLD_SINGLE_POLYGONE_CACHE_SIZE;
						}
					}
				}
			}

		}
		
	}


	static void generatedPolyContacts(const Gu::PolygonalData& polyData0, const Gu::HullPolygonData& referencePolygon, const Gu::TriangleV& triangle, const PxU32 triangleIndex, const PxU8 triFlags, 
		Gu::SupportLocal* map0, Gu::MeshPersistentContact* manifoldContacts, PxU32& numManifoldContacts, const Ps::aos::FloatVArg contactDist, const Ps::aos::Vec3VArg contactNormal, 
		Cm::RenderOutput* renderOutput)
	{
		PX_UNUSED(triFlags);
		PX_UNUSED(renderOutput);

		using namespace Ps::aos;

		const FloatV zero = FZero();

		const PxU32 previousContacts = numManifoldContacts;

		const PxU8* inds0 = polyData0.mPolygonVertexRefs + referencePolygon.mVRef8;

		const Vec3V nContactNormal = V3Neg(contactNormal);

		//this is the matrix transform all points to the 2d plane
		const Mat33V rot = findRotationMatrixFromZAxis(contactNormal);

		Vec3V* points0In0=reinterpret_cast<Vec3V*>(PxAllocaAligned(sizeof(Vec3V)*referencePolygon.mNbVerts, 16));
		Vec3V points1In0[3];
		FloatV points1In0TValue[3];

		bool points1In0Penetration[3];
		
		//Transform all the verts from vertex space to shape space
		map0->populateVerts(inds0, referencePolygon.mNbVerts, polyData0.mVerts, points0In0);

		points1In0[0] = triangle.verts[0];
		points1In0[1] = triangle.verts[1];
		points1In0[2] = triangle.verts[2];


#if PCM_LOW_LEVEL_DEBUG
		Gu::PersistentContactManifold::drawPolygon(*renderOutput, map0->transform, points0In0, referencePolygon.mNbVerts, (PxU32)PxDebugColor::eARGB_GREEN);
		//Gu::PersistentContactManifold::drawTriangle(*gRenderOutPut, map0->transform.transform(points1In0[0]), map0->transform.transform(points1In0[1]), map0->transform.transform(points1In0[2]), (PxU32)PxDebugColor::eARGB_BLUE);
#endif

		//the first point in the reference plane
		const Vec3V referencePoint = points0In0[0];
 
		Vec3V eps = Vec3V_From_FloatV(FEps());
		Vec3V max = Vec3V_From_FloatV(FMax());
		Vec3V nmax = V3Neg(max); 

		//transform reference polygon to 2d, calculate min and max
		Vec3V rPolygonMin= max;
		Vec3V rPolygonMax = nmax;
		for(PxU32 i=0; i<referencePolygon.mNbVerts; ++i)
		{
			//points0In0[i].vertext = M33TrnspsMulV3(rot, Vec3V_From_PxVec3(polyData0.mVerts[inds0[i]]));
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
		for(PxU32 i=0; i<3; ++i)
		{
			const Vec3V vert1 =points1In0[i]; //this still in polyData1's local space
			points1In0[i] = M33MulV3(rot, vert1);
			const FloatV z = V3GetZ(points1In0[i]);
			points1In0TValue[i] = FSub(z, d);
			points1In0[i] = V3SetZ(points1In0[i], d);
			iPolygonMin = V3Min(iPolygonMin, points1In0[i]);
			iPolygonMax = V3Max(iPolygonMax, points1In0[i]);
			if(FAllGrtr(rd, z))
			{
				points1In0Penetration[i] = true;

				//ML : check to see whether all the points of triangles in 2D space are within reference polygon's range
				if(contains(points0In0, referencePolygon.mNbVerts, points1In0[i], rPolygonMin, rPolygonMax))
				{
					inside++;

					//calculate projection point
					const FloatV t = V3Dot(contactNormal, V3Sub(vert1, referencePoint));
					const Vec3V projectPoint = V3NegScaleSub(contactNormal, t, vert1);
					
					const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(nContactNormal), t);
					numManifoldContacts = addMeshContacts(manifoldContacts, projectPoint, vert1, localNormalPen, triangleIndex, numManifoldContacts);

					//if the numContacts are more than GU_MESH_CONTACT_REDUCTION_THRESHOLD, we need to do contact reduction
					const PxU32 numContacts = numManifoldContacts - previousContacts;
					if(numContacts >= GU_MESH_CONTACT_REDUCTION_THRESHOLD)
					{
						//a polygon has more than GU_MESH_CONTACT_REDUCTION_THRESHOLD(16) contacts with this triangle, we will reduce
						//the contacts to GU_SINGLE_MANIFOLD_SINGLE_POLYGONE_CACHE_SIZE(4) points
						Gu::SinglePersistentContactManifold::reduceContacts(&manifoldContacts[previousContacts], numContacts);
						numManifoldContacts = previousContacts + GU_SINGLE_MANIFOLD_SINGLE_POLYGONE_CACHE_SIZE;
					}
				}
			}
			
		}


		if(inside == 3)
		{
			return;
		}

		inside = 0;
		iPolygonMin = V3Sub(iPolygonMin, eps);
		iPolygonMax = V3Add(iPolygonMax, eps);

		const Vec3V incidentNormal = triangle.normal();
		const FloatV iPlaneD = V3Dot(incidentNormal, triangle.verts[0]);
		const FloatV one = FOne();
		for(PxU32 i=0; i<referencePolygon.mNbVerts; ++i)
		{
			if(contains(points1In0, 3, points0In0[i], iPolygonMin, iPolygonMax))
			{
			
				const Vec3V vert0 = M33TrnspsMulV3(rot, points0In0[i]);

				const FloatV t =FSub(V3Dot(incidentNormal, vert0), iPlaneD);
				
				if(FAllGrtr(t, contactDist))
					continue;

				const Vec3V projPoint = V3NegScaleSub(incidentNormal, t, vert0);

				FloatV u, w;
				barycentricCoordinates(projPoint, triangle.verts[0], triangle.verts[1], triangle.verts[2], u, w);
				const BoolV con = BAnd(FIsGrtrOrEq(u, zero), BAnd(FIsGrtrOrEq(w, zero),  FIsGrtrOrEq(one, FAdd(u, w))));
			
				if(BAllEqTTTT(con))
				{
					inside++;

					const Vec3V v = V3Sub(projPoint, vert0);
					const FloatV t3 = V3Dot(v, contactNormal);
					
					const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(nContactNormal), t3);
					numManifoldContacts = addMeshContacts(manifoldContacts, vert0, projPoint, localNormalPen, triangleIndex, numManifoldContacts);
				
					//if the numContacts are more than GU_MESH_CONTACT_REDUCTION_THRESHOLD, we need to do contact reduction
					const PxU32 numContacts = numManifoldContacts - previousContacts;
					if(numContacts >= GU_MESH_CONTACT_REDUCTION_THRESHOLD)
					{
						//a polygon has more than GU_MESH_CONTACT_REDUCTION_THRESHOLD(16) contacts with this triangle, we will reduce
						//the contacts to GU_SINGLE_MANIFOLD_SINGLE_POLYGONE_CACHE_SIZE(4) points
						Gu::SinglePersistentContactManifold::reduceContacts(&manifoldContacts[previousContacts], numContacts);
						numManifoldContacts = previousContacts + GU_SINGLE_MANIFOLD_SINGLE_POLYGONE_CACHE_SIZE;
					}
				}
					
			}
				
		}

		if(inside == referencePolygon.mNbVerts)
			return;


	
		//Always generate segment contacts
		//(2) segment intesection
		for (PxU32 iStart = 0, iEnd = 2; iStart < 3; iEnd = iStart++)
		{
			if((!points1In0Penetration[iStart] && !points1In0Penetration[iEnd] ) )
				continue;
			
			const Vec3V ipA = points1In0[iStart];
			const Vec3V ipB = points1In0[iEnd];

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
						const FloatV t = FMul(a1, FRecip(FSub(a2, a1)));

						const Vec3V ipAOri = V3SetZ(points1In0[iStart], FAdd(points1In0TValue[iStart], d));
						const Vec3V ipBOri = V3SetZ(points1In0[iEnd], FAdd(points1In0TValue[iEnd], d));

						const Vec3V pBB = V3NegScaleSub(V3Sub(ipBOri, ipAOri), t, ipAOri); 
						const Vec3V pAA = V3SetZ(pBB, d);
						const Vec3V pA = M33TrnspsMulV3(rot, pAA);
						const Vec3V pB = M33TrnspsMulV3(rot, pBB);
						const FloatV pen = FSub(V3GetZ(pBB), V3GetZ(pAA));
				
						if(FAllGrtr(pen, contactDist))
							continue;
		
					
						const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(nContactNormal), pen);
						numManifoldContacts = addMeshContacts(manifoldContacts, pA, pB, localNormalPen, triangleIndex, numManifoldContacts);
						
						//if the numContacts are more than GU_MESH_CONTACT_REDUCTION_THRESHOLD, we need to do contact reduction
						const PxU32 numContacts = numManifoldContacts - previousContacts;
						if(numContacts >= GU_MESH_CONTACT_REDUCTION_THRESHOLD)
						{
							//a polygon has more than GU_MESH_CONTACT_REDUCTION_THRESHOLD(16) contacts with this triangle, we will reduce
							//the contacts to GU_SINGLE_MANIFOLD_SINGLE_POLYGONE_CACHE_SIZE(4) points
							Gu::SinglePersistentContactManifold::reduceContacts(&manifoldContacts[previousContacts], numContacts);
							numManifoldContacts = previousContacts + GU_SINGLE_MANIFOLD_SINGLE_POLYGONE_CACHE_SIZE;
						}
					}
				}
			}
		}
			
	}


	bool Gu::PCMConvexVsMeshContactGeneration::generateTriangleFullContactManifold(Gu::TriangleV& localTriangle, const PxU32 triangleIndex, const PxU32* triIndices, const PxU8 triFlags, const Gu::PolygonalData& polyData,  Gu::SupportLocalImpl<Gu::TriangleV>* localTriMap, Gu::SupportLocal* polyMap, Gu::MeshPersistentContact* manifoldContacts, PxU32& numContacts,
		const Ps::aos::FloatVArg contactDist, Ps::aos::Vec3V& patchNormal)
	{
	
		using namespace Ps::aos;

		{
				
			FeatureStatus status = POLYDATA0;
			FloatV minOverlap = FMax();
			//minNormal will be in the local space of polyData
			Vec3V minNormal = V3Zero();


			PxU32 feature0;
			if(!testTriangleFaceNormal(localTriangle, polyData, localTriMap, polyMap, contactDist, minOverlap, feature0, minNormal, POLYDATA0, status))
				return false;

			PxU32 feature1;
			if(!testPolyFaceNormal(localTriangle, polyData, localTriMap, polyMap, contactDist, minOverlap, feature1, minNormal, POLYDATA1, status))
				return false;


			if (!testPolyEdgeNormal(localTriangle, triFlags, polyData, localTriMap, polyMap, contactDist, minOverlap, minNormal, EDGE, status))
				return false;


			const Vec3V triNormal = localTriangle.normal();

			if(status == POLYDATA0)
			{
				//minNormal is the triangle normal and it is in the local space of polydata0
				const Gu::HullPolygonData& referencePolygon = polyData.mPolygons[getPolygonIndex(polyData, polyMap, minNormal)];

				patchNormal = triNormal;
				generatedTriangleContacts(localTriangle, triangleIndex, triFlags, polyData, referencePolygon, polyMap, manifoldContacts, numContacts, contactDist, triNormal, mRenderOutput);
			
			}
			else
			{

				if(status == POLYDATA1)
				{
					const Gu::HullPolygonData* referencePolygon = &polyData.mPolygons[feature1];
			
					const FloatV cosTheta = V3Dot(V3Neg(minNormal), triNormal);
					
					const FloatV threshold = FLoad(0.707106781f);//about 45 degree0
					
					if(FAllGrtr(cosTheta, threshold))
					{
						patchNormal = triNormal;
						generatedTriangleContacts(localTriangle, triangleIndex, triFlags, polyData, *referencePolygon, polyMap, manifoldContacts, numContacts, contactDist, triNormal, mRenderOutput);
					}
					else
					{
						//ML : defer the contacts generation

						if (mSilhouetteEdgesAreActive ||
							!(triFlags & (ETD_SILHOUETTE_EDGE_01 | ETD_SILHOUETTE_EDGE_12 | ETD_SILHOUETTE_EDGE_20)))
						{
							const PxU32 nb = sizeof(PCMDeferredPolyData) / sizeof(PxU32);
							PxU32 newSize = nb + mDeferredContacts->size();
							mDeferredContacts->reserve(newSize);
							PCMDeferredPolyData* PX_RESTRICT data = reinterpret_cast<PCMDeferredPolyData*>(mDeferredContacts->end());
							mDeferredContacts->forceSize_Unsafe(newSize);

							data->mTriangleIndex = triangleIndex;
							data->mFeatureIndex = feature1;
							data->triFlags = triFlags;
							data->mInds[0] = triIndices[0];
							data->mInds[1] = triIndices[1];
							data->mInds[2] = triIndices[2];
							V3StoreU(localTriangle.verts[0], data->mVerts[0]);
							V3StoreU(localTriangle.verts[1], data->mVerts[1]);
							V3StoreU(localTriangle.verts[2], data->mVerts[2]);
						}
						return true;
					}
				}
				else
				{
					feature1 = PxU32(getPolygonIndex(polyData, polyMap, minNormal));
					const Gu::HullPolygonData* referencePolygon = &polyData.mPolygons[feature1];
			
					const Vec3V contactNormal = V3Normalize(M33TrnspsMulV3(polyMap->shape2Vertex, V3LoadU(referencePolygon->mPlane.n)));
					const Vec3V nContactNormal = V3Neg(contactNormal);

					//if the minimum sperating axis is edge case, we don't defer it because it is an activeEdge
					patchNormal = nContactNormal;
					generatedPolyContacts(polyData, *referencePolygon, localTriangle, triangleIndex, triFlags, polyMap, manifoldContacts, numContacts, contactDist, contactNormal, mRenderOutput);

				}
			}
		}

		return true;
	}


	bool Gu::PCMConvexVsMeshContactGeneration::generateTriangleFullContactManifold(Gu::TriangleV& localTriangle,  const PxU32 triangleIndex, const PxU8 triFlags, const Gu::PolygonalData& polyData,  Gu::SupportLocalImpl<Gu::TriangleV>* localTriMap, Gu::SupportLocal* polyMap, Gu::MeshPersistentContact* manifoldContacts, PxU32& numContacts,
		const Ps::aos::FloatVArg contactDist, Ps::aos::Vec3V& patchNormal, Cm::RenderOutput* renderOutput)
	{
	
		using namespace Ps::aos;

		const FloatV threshold = FLoad(0.7071f);//about 45 degree
		PX_UNUSED(threshold);
		{
			
			FeatureStatus status = POLYDATA0;
			FloatV minOverlap = FMax();
			//minNormal will be in the local space of polyData
			Vec3V minNormal = V3Zero();

			PxU32 feature0;
			if(!testTriangleFaceNormal(localTriangle, polyData, localTriMap, polyMap, contactDist, minOverlap, feature0, minNormal, POLYDATA0, status))
				return false;
			
			PxU32 feature1;
			if(!testPolyFaceNormal(localTriangle, polyData, localTriMap, polyMap, contactDist, minOverlap, feature1, minNormal, POLYDATA1, status))
				return false;

			if(!testPolyEdgeNormal(localTriangle, triFlags, polyData, localTriMap, polyMap, contactDist, minOverlap, minNormal, EDGE, status))
				return false;

			const Vec3V triNormal = localTriangle.normal();
			patchNormal = triNormal;

			const Gu::HullPolygonData* referencePolygon = &polyData.mPolygons[getPolygonIndex(polyData, polyMap, triNormal)];
			generatedTriangleContacts(localTriangle, triangleIndex, triFlags, polyData, *referencePolygon, polyMap, manifoldContacts, numContacts, contactDist, triNormal, renderOutput);
			
		}

		return true;
	}

	bool Gu::PCMConvexVsMeshContactGeneration::generatePolyDataContactManifold(Gu::TriangleV& localTriangle, const PxU32 featureIndex,  const PxU32 triangleIndex, const PxU8 triFlags, Gu::MeshPersistentContact* manifoldContacts, PxU32& numContacts, const Ps::aos::FloatVArg contactDist, Ps::aos::Vec3V& patchNormal)
	{
	
		using namespace Ps::aos;

		const Gu::HullPolygonData* referencePolygon = &mPolyData.mPolygons[featureIndex];
		
		const Vec3V contactNormal = V3Normalize(M33TrnspsMulV3(mPolyMap->shape2Vertex, V3LoadU(referencePolygon->mPlane.n)));
		const Vec3V nContactNormal = V3Neg(contactNormal);
		
		patchNormal = nContactNormal;
		generatedPolyContacts(mPolyData, *referencePolygon, localTriangle, triangleIndex, triFlags, mPolyMap, manifoldContacts, numContacts, contactDist, contactNormal, mRenderOutput);

		return true;
	}


}//physx
