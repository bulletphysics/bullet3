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

#include "PsIntrinsics.h"
#include "PsAllocator.h"
#include "GuOverlapTests.h"

#include "PsUtilities.h"
#include "PsVecMath.h"

#include "GuHeightFieldUtil.h"
#include "GuIntersectionBoxBox.h"
#include "GuIntersectionTriangleBox.h"
#include "GuDistancePointSegment.h"
#include "GuDistanceSegmentBox.h"
#include "GuDistanceSegmentSegment.h"
#include "GuDistanceSegmentSegmentSIMD.h"

#include "PxSphereGeometry.h"
#include "PxBoxGeometry.h"
#include "PxCapsuleGeometry.h"
#include "PxPlaneGeometry.h"
#include "PxConvexMeshGeometry.h"

#include "GuCapsule.h"
#include "GuEdgeCache.h"
#include "GuBoxConversion.h"

#include "GuInternal.h"
#include "GuConvexUtilsInternal.h"

#include "GuVecTriangle.h"
#include "GuVecSphere.h"
#include "GuVecCapsule.h"
#include "GuVecConvexHull.h"
#include "GuConvexMesh.h"

using namespace physx;
using namespace Cm;
using namespace Gu;
using namespace Ps::aos;

static bool intersectHeightFieldSphere(const HeightFieldUtil& hfUtil, const Sphere& sphereInHfShape)
{
	const HeightField& hf = hfUtil.getHeightField();

	// sample the sphere center in the heightfield to find out
	// if we have penetration with more than the sphere radius
	if(hfUtil.isShapePointOnHeightField(sphereInHfShape.center.x, sphereInHfShape.center.z))
	{
		// The sphere origin projects within the bounds of the heightfield in the X-Z plane
		PxReal sampleHeight = hfUtil.getHeightAtShapePoint(sphereInHfShape.center.x, sphereInHfShape.center.z);
		PxReal deltaHeight = sphereInHfShape.center.y - sampleHeight;
		if(hf.isDeltaHeightInsideExtent(deltaHeight))
		{
			// The sphere origin is 'below' the heightfield surface
			PxU32 faceIndex = hfUtil.getFaceIndexAtShapePoint(sphereInHfShape.center.x, sphereInHfShape.center.z);
			if(faceIndex != 0xffffffff)
			{
				return true;
			}
			return false;
		}
	}

	const PxReal radiusSquared = sphereInHfShape.radius * sphereInHfShape.radius;

	const PxVec3 sphereInHF = hfUtil.shape2hfp(sphereInHfShape.center);

	const PxReal radiusOverRowScale = sphereInHfShape.radius * PxAbs(hfUtil.getOneOverRowScale());
	const PxReal radiusOverColumnScale = sphereInHfShape.radius * PxAbs(hfUtil.getOneOverColumnScale());

	const PxU32 minRow = hf.getMinRow(sphereInHF.x - radiusOverRowScale);
	const PxU32 maxRow = hf.getMaxRow(sphereInHF.x + radiusOverRowScale);
	const PxU32 minColumn = hf.getMinColumn(sphereInHF.z - radiusOverColumnScale);
	const PxU32 maxColumn = hf.getMaxColumn(sphereInHF.z + radiusOverColumnScale);

	for(PxU32 r = minRow; r < maxRow; r++) 
	{
		for(PxU32 c = minColumn; c < maxColumn; c++) 
		{

			// x--x--x
			// | x   |
			// x  x  x
			// |   x |
			// x--x--x
			PxVec3 pcp[11];
			PxU32 npcp = 0;
			npcp = hfUtil.findClosestPointsOnCell(r, c, sphereInHfShape.center, pcp, NULL, true, true, true);

			for(PxU32 pi = 0; pi < npcp; pi++)
			{
				PxVec3 d = sphereInHfShape.center - pcp[pi];
				
				PxReal ll = d.magnitudeSquared();
					
				if(ll > radiusSquared) 
					// Too far
					continue;

				return true;
			}
		}
	}
	return false;
}

static bool intersectHeightFieldCapsule(const HeightFieldUtil& hfUtil, const PxCapsuleGeometry& capsuleGeom, const PxTransform& capsulePose)
{
	const HeightField& hf = hfUtil.getHeightField();

	PxVec3 verticesInHfShape[2];
	PxVec3 capsuleOrigin, capsuleExtent;
	{
		const PxVec3 capsuleHalfHeightVector = getCapsuleHalfHeightVector(capsulePose, capsuleGeom);

		capsuleOrigin = capsulePose.p + capsuleHalfHeightVector;
		capsuleExtent = -capsuleHalfHeightVector*2.0f;

		verticesInHfShape[0] = capsuleOrigin;
		verticesInHfShape[1] = capsulePose.p - capsuleHalfHeightVector;
	}

	const PxReal radius = capsuleGeom.radius;
	const PxReal radiusOverRowScale = radius * PxAbs(hfUtil.getOneOverRowScale());
	const PxReal radiusOverColumnScale = radius * PxAbs(hfUtil.getOneOverColumnScale());

	PxU32 absMinRow = 0xffffffff;
	PxU32 absMaxRow = 0;
	PxU32 absMinColumn = 0xffffffff;
	PxU32 absMaxColumn = 0;

	PxReal radiusSquared = radius * radius;

	// test both of capsule's corner vertices+radius for HF overlap
	for(PxU32 i = 0; i<2; i++)
	{
		const PxVec3& sphereInHfShape = verticesInHfShape[i];

		// we have to do this first to update the absMin / absMax correctly even if
		// we decide to continue from inside the deep penetration code.

		const PxVec3 sphereInHF = hfUtil.shape2hfp(sphereInHfShape);

		const PxU32 minRow = hf.getMinRow(sphereInHF.x - radiusOverRowScale);
		const PxU32 maxRow = hf.getMaxRow(sphereInHF.x + radiusOverRowScale);
		const PxU32 minColumn = hf.getMinColumn(sphereInHF.z - radiusOverColumnScale);
		const PxU32 maxColumn = hf.getMaxColumn(sphereInHF.z + radiusOverColumnScale);

		if(minRow < absMinRow)			absMinRow = minRow;
		if(minColumn < absMinColumn)	absMinColumn = minColumn;
		if(maxRow > absMaxRow)			absMaxRow = maxRow;
		if(maxColumn > absMaxColumn)	absMaxColumn = maxColumn;

		if(hfUtil.isShapePointOnHeightField(sphereInHfShape.x, sphereInHfShape.z))
		{
			// The sphere origin projects within the bounds of the heightfield in the X-Z plane
			const PxReal sampleHeight = hfUtil.getHeightAtShapePoint(sphereInHfShape.x, sphereInHfShape.z);
			const PxReal deltaHeight = sphereInHfShape.y - sampleHeight;
			if(hf.isDeltaHeightInsideExtent(deltaHeight))
			{
				// The sphere origin is 'below' the heightfield surface
				const PxU32 faceIndex = hfUtil.getFaceIndexAtShapePoint(sphereInHfShape.x, sphereInHfShape.z);
				if(faceIndex != 0xffffffff)
				{
					return true;
				}
				continue;
			}
		}

		for(PxU32 r = minRow; r < maxRow; r++) 
		{
			for(PxU32 c = minColumn; c < maxColumn; c++) 
			{

				// x--x--x
				// | x   |
				// x  x  x
				// |   x |
				// x--x--x
				PxVec3 pcp[11];
				PxU32 npcp = 0;
				npcp = hfUtil.findClosestPointsOnCell(r, c, sphereInHfShape, pcp, NULL, true, true, true);

				for(PxU32 pi = 0; pi < npcp; pi++)
				{
					const PxVec3 d = sphereInHfShape - pcp[pi];

					if(hf.isDeltaHeightOppositeExtent(d.y))
					{
						// We are 'above' the heightfield

						const PxReal ll = d.magnitudeSquared();
						if(ll > radiusSquared) 
							// Too far above
							continue;

						return true;
					}
				}
			}
		}
	}

	const Vec3V p1 = V3LoadU(capsuleOrigin);
	const Vec3V d1 = V3LoadU(capsuleExtent);

	// now test capsule's inflated segment for overlap with HF edges
	PxU32 row, column;
	for(row = absMinRow; row <= absMaxRow; row++)
	{
		for(column = absMinColumn; column <= absMaxColumn; column++)
		{
			const PxU32 vertexIndex = row * hf.getNbColumnsFast() + column;
			const PxU32 firstEdge = 3 * vertexIndex;
			// omg I am sorry about this code but I can't find a simpler way:
			//  last column will only test edge 2
			//  last row will only test edge 0
			//  and most importantly last row and column will not go inside the for
			const PxU32 minEi = (column == absMaxColumn) ? 2u : 0;
			const PxU32 maxEi = (row    == absMaxRow   ) ? 1u : 3u;
			for(PxU32 ei = minEi; ei < maxEi; ei++)
			{
				const PxU32 edgeIndex = firstEdge + ei;

				const PxU32 cell = vertexIndex;
				PX_ASSERT(cell == edgeIndex / 3);
				const PxU32 row_ = row;
				PX_ASSERT(row_ == cell / hf.getNbColumnsFast());
				const PxU32 column_ = column;
				PX_ASSERT(column_ == cell % hf.getNbColumnsFast());

				const PxU32 faceIndex = hfUtil.getEdgeFaceIndex(edgeIndex, cell, row_, column_);
				if(faceIndex != 0xffffffff)
				{
					PxVec3 origin;
					PxVec3 direction;
					hfUtil.getEdge(edgeIndex, cell, row_, column_, origin, direction);

					const Vec3V p2 = V3LoadU(origin);
					const Vec3V d2 = V3LoadU(direction);
					FloatV s, t;
					const FloatV llV = Gu::distanceSegmentSegmentSquared(p1, d1, p2, d2, s, t);

					PxReal ll;
					FStore(llV, &ll);

					if(ll < radiusSquared)
						return true;
				}
			}
		}
	}
	return false;
}

namespace physx
{
namespace Gu
{
	const PxReal signs[24] = 
	{
		-1,-1,-1,
		-1,-1, 1,
		-1, 1,-1,
		-1, 1, 1,
		 1,-1,-1,
		 1,-1, 1,
		 1, 1,-1,
		 1, 1, 1,
	};

	const char edges[24] = 
	{
		0,1,
		1,3,
		3,2,
		2,0,
		4,5,
		5,7,
		7,6,
		6,4,
		0,4,
		1,5,
		2,6,
		3,7,
	};
	
	struct TriggerTraceSegmentCallback
	{
		bool intersection;

		PX_INLINE TriggerTraceSegmentCallback() : intersection(false)
		{
		}

		PX_INLINE bool underFaceHit(
			const HeightFieldUtil&, const PxVec3&,
			const PxVec3&, PxF32, PxF32, PxF32, PxU32)
		{
			return true;
		}

		PX_INLINE bool faceHit(const HeightFieldUtil&, const PxVec3&, PxU32, PxReal, PxReal)
		{
			intersection = true;
			return false;
		}
		bool onEvent(PxU32 , PxU32* )
		{
			return true;
		}
	};


	class OverlapHeightfieldTraceSegmentHelper
	{
		PX_NOCOPY(OverlapHeightfieldTraceSegmentHelper)
	public:
		OverlapHeightfieldTraceSegmentHelper(const HeightFieldTraceUtil& hfUtil) : mHfUtil(hfUtil)
		{
			mHfUtil.computeLocalBounds(mLocalBounds);
		}

		PX_INLINE void traceSegment(const PxVec3& aP0, const PxVec3& aP1, TriggerTraceSegmentCallback* aCallback) const
		{
			mHfUtil.traceSegment<TriggerTraceSegmentCallback, false, false>(aP0, aP1 - aP0, 1.0f, aCallback, mLocalBounds, false, NULL);
		}

	private:
		const HeightFieldTraceUtil&	mHfUtil;
		PxBounds3					mLocalBounds;
	};

} // namespace
}

static bool intersectHeightFieldBox(const HeightFieldTraceUtil& hfUtil, const Box& boxInHfShape)
{
	const HeightField& hf = hfUtil.getHeightField();

	// Get box vertices
	PxVec3 boxVertices[8];
	for(PxU32 i=0; i<8; i++) 
		boxVertices[i] = PxVec3(boxInHfShape.extents.x*signs[3*i], boxInHfShape.extents.y*signs[3*i+1], boxInHfShape.extents.z*signs[3*i+2]);

	// Transform box vertices to HeightFieldShape space
	PxVec3 boxVerticesInHfShape[8];
	for(PxU32 i=0; i<8; i++) 
		boxVerticesInHfShape[i] = boxInHfShape.transform(boxVertices[i]);

	// Test box vertices.
	{
		for(PxU32 i=0; i<8; i++) 
		{
			const PxVec3& boxVertexInHfShape = boxVerticesInHfShape[i];
			if(hfUtil.isShapePointOnHeightField(boxVertexInHfShape.x, boxVertexInHfShape.z))
			{
				const PxReal y = hfUtil.getHeightAtShapePoint(boxVertexInHfShape.x, boxVertexInHfShape.z);
				const PxReal dy = boxVertexInHfShape.y - y;
				if(hf.isDeltaHeightInsideExtent(dy))
				{
					PxU32 faceIndex = hfUtil.getFaceIndexAtShapePoint(boxVertexInHfShape.x, boxVertexInHfShape.z);
					if(faceIndex != 0xffffffff)
					{
						return true;
					}
				}
			}
		}
	}

	// Test box edges.
	{
		OverlapHeightfieldTraceSegmentHelper traceSegmentHelper(hfUtil);

		for(PxU32 i=0; i<12; i++) 
		{
			const PxVec3 v0 = boxVerticesInHfShape[PxU8(edges[2*i])];
			const PxVec3 v1 = boxVerticesInHfShape[PxU8(edges[2*i+1])];
			TriggerTraceSegmentCallback cb;
			traceSegmentHelper.traceSegment(v0, v1, &cb);
			if(cb.intersection)
				return true;
		}
	}

	// Test HeightField vertices.
	{
		PsTransformV _hfShape2BoxShape;					
		const PxQuat bq(boxInHfShape.rot);
		const QuatV q1 = QuatVLoadU(&bq.x);
		const Vec3V p1 = V3LoadU(&boxInHfShape.center.x);
		const PsTransformV _boxPose(p1, q1);
		_hfShape2BoxShape = _boxPose.getInverse();

		PxReal minx(PX_MAX_REAL);
		PxReal minz(PX_MAX_REAL);
		PxReal maxx(-PX_MAX_REAL);
		PxReal maxz(-PX_MAX_REAL);

		for(PxU32 i=0; i<8; i++) 
		{
			const PxVec3& boxVertexInHfShape = boxVerticesInHfShape[i];

/*			if(boxVertexInHfShape.x < minx) minx = boxVertexInHfShape.x;
			if(boxVertexInHfShape.z < minz) minz = boxVertexInHfShape.z;
			if(boxVertexInHfShape.x > maxx) maxx = boxVertexInHfShape.x;
			if(boxVertexInHfShape.z > maxz) maxz = boxVertexInHfShape.z;*/
			minx = physx::intrinsics::selectMin(boxVertexInHfShape.x, minx);
			minz = physx::intrinsics::selectMin(boxVertexInHfShape.z, minz);
			maxx = physx::intrinsics::selectMax(boxVertexInHfShape.x, maxx);
			maxz = physx::intrinsics::selectMax(boxVertexInHfShape.z, maxz);
		}

		const PxReal oneOverRowScale = hfUtil.getOneOverRowScale();
		const PxReal oneOverColumnScale = hfUtil.getOneOverColumnScale();
		const PxU32 minRow = hf.getMinRow(minx * oneOverRowScale);
		const PxU32 maxRow = hf.getMaxRow(maxx * oneOverRowScale);
		const PxU32 minColumn = hf.getMinColumn(minz * oneOverColumnScale);
		const PxU32 maxColumn = hf.getMaxColumn(maxz * oneOverColumnScale);

		const Vec4V extentV = V4LoadXYZW(boxInHfShape.extents.x, boxInHfShape.extents.y, boxInHfShape.extents.z, PX_MAX_REAL);
		const PxHeightFieldGeometry& geom = hfUtil.getHeightFieldGeometry();

		for(PxU32 row = minRow; row <= maxRow; row++)
		{
			for(PxU32 column = minColumn; column <= maxColumn; column++)
			{
				PxU32 vertexIndex = row * hf.getNbColumnsFast() + column;
				if(hfUtil.isQueryVertex(vertexIndex, row, column))
				{
					// check if hf vertex is inside the box
					const Vec4V hfVertex = V4LoadXYZW(geom.rowScale * row, geom.heightScale * hf.getHeight(vertexIndex), geom.columnScale * column, 0.0f);
					const Vec4V hfVertexInBoxShape = Vec4V_From_Vec3V(_hfShape2BoxShape.transform(Vec3V_From_Vec4V(hfVertex)));
					const Vec4V hfVertexInBoxShapeAbs = V4Abs(hfVertexInBoxShape);

					if(V4AllGrtr(extentV, hfVertexInBoxShapeAbs))
					{
						return true;
					}
				}
			}
		}
	}
	return false;
}

static Matrix34 multiplyInverseRTLeft(const Matrix34& left, const Matrix34& right)
{
//	t = left.M % (right.t - left.t);
	PxVec3 t = left.rotateTranspose(right.p - left.p);

//	M.setMultiplyTransposeLeft(left.M, right.M);
	const PxMat33& left33 = left.m;
	const PxMat33& right33 = right.m;
	PxMat33 multiplyTransposeLeft33 = (left33.getTranspose()) * right33;

	return Matrix34(multiplyTransposeLeft33, t);
}

static bool intersectHeightFieldConvex(
	const HeightFieldTraceUtil& hfUtil, const PxTransform& _hfAbsPose, const ConvexMesh& convexMesh,
	const PxTransform& _convexAbsPose, const PxMeshScale& convexMeshScaling)
{
	const Matrix34 hfAbsPose34(_hfAbsPose);
	const Matrix34 convexAbsPose34(_convexAbsPose);
	const Matrix34 vertexToShapeSkew34(convexMeshScaling.toMat33());
	const Matrix34 temp34 = convexAbsPose34 * vertexToShapeSkew34;
	const Matrix34 convexShape2HfShapeSkew34 = multiplyInverseRTLeft(hfAbsPose34, temp34);

	const ConvexHullData* hull = &convexMesh.getHull();

	// Allocate space for transformed vertices.
	PxVec3* convexVerticesInHfShape = reinterpret_cast<PxVec3*>(PxAlloca(hull->mNbHullVertices*sizeof(PxVec3)));

	// Transform vertices to height field shape
	const PxVec3* hullVerts = hull->getHullVertices();
	for(PxU32 i=0; i<hull->mNbHullVertices; i++)
		convexVerticesInHfShape[i] = convexShape2HfShapeSkew34.transform(hullVerts[i]);

	// Compute bounds of convex in hf space
	PxBounds3 convexBoundsInHfShape;
	computeBoundsAroundVertices(convexBoundsInHfShape, hull->mNbHullVertices, convexVerticesInHfShape);

	// Compute the height field extreme over the bounds area.
	const HeightField& hf = hfUtil.getHeightField();
	PxReal hfExtreme = -PX_MAX_REAL;
	const PxReal oneOverRowScale = hfUtil.getOneOverRowScale();
	const PxReal oneOverColumnScale = hfUtil.getOneOverColumnScale();
	const PxReal rowScale = (1.0f / hfUtil.getOneOverRowScale());
	const PxReal columnScale = (1.0f / hfUtil.getOneOverColumnScale());
	const PxReal heightScale = (1.0f / hfUtil.getOneOverHeightScale());

	// negative scale support
	PxU32 minRow;
	PxU32 maxRow;
	if(oneOverRowScale > 0.0f)
	{
		minRow = hf.getMinRow(convexBoundsInHfShape.minimum.x * oneOverRowScale);
		maxRow = hf.getMaxRow(convexBoundsInHfShape.maximum.x * oneOverRowScale);
	}
	else
	{
		minRow = hf.getMinRow(convexBoundsInHfShape.maximum.x * oneOverRowScale);
		maxRow = hf.getMaxRow(convexBoundsInHfShape.minimum.x * oneOverRowScale);
	}

	PxU32 minColumn;
	PxU32 maxColumn;
	if(oneOverColumnScale > 0.0f)
	{
		minColumn = hf.getMinColumn(convexBoundsInHfShape.minimum.z * oneOverColumnScale);
		maxColumn = hf.getMaxColumn(convexBoundsInHfShape.maximum.z * oneOverColumnScale);
	}
	else
	{
		minColumn = hf.getMinColumn(convexBoundsInHfShape.maximum.z * oneOverColumnScale);
		maxColumn = hf.getMaxColumn(convexBoundsInHfShape.minimum.z * oneOverColumnScale);
	}

	for(PxU32 row = minRow; row <= maxRow; row++)
	{
		for(PxU32 column = minColumn; column <= maxColumn; column++)
		{
			const PxReal h = hf.getHeight(row * hf.getNbColumnsFast() + column);
			hfExtreme = PxMax(hfExtreme, h);
		}
	}
	hfExtreme *= heightScale;


	// Return if convex is on the wrong side of the extreme.
	if(convexBoundsInHfShape.minimum.y > hfExtreme)
		return false;

	// Test convex vertices
	{
		for(PxU32 i=0; i<hull->mNbHullVertices; i++) 
		{
			const PxVec3& convexVertexInHfShape = convexVerticesInHfShape[i];
			bool insideExtreme = convexVertexInHfShape.y < hfExtreme;
			if(insideExtreme && hfUtil.isShapePointOnHeightField(convexVertexInHfShape.x, convexVertexInHfShape.z))
			{
				const PxReal y = hfUtil.getHeightAtShapePoint(convexVertexInHfShape.x, convexVertexInHfShape.z);
				const PxReal dy = convexVertexInHfShape.y - y;
				if(hf.isDeltaHeightInsideExtent(dy))
				{
					const PxU32 faceIndex = hfUtil.getFaceIndexAtShapePoint(convexVertexInHfShape.x, convexVertexInHfShape.z);
					if(faceIndex != 0xffffffff)
						return true;
				}
			}
		} 
	}

	// Test convex edges.
	{
		EdgeCache edgeCache;
		PxU32 numPolygons = hull->mNbPolygons;
		const HullPolygonData* polygons = hull->mPolygons;
		const PxU8* const vertexData = hull->getVertexData8();

		OverlapHeightfieldTraceSegmentHelper traceSegmentHelper(hfUtil);

		while(numPolygons--)
		{
			const HullPolygonData& polygon = *polygons++;

			const PxU8* verts = vertexData + polygon.mVRef8;

			PxU32 numEdges = polygon.mNbVerts;

			PxU32 a = numEdges - 1;
			PxU32 b = 0;
			while(numEdges--)
			{
				PxU8 vi0 =  verts[a];
				PxU8 vi1 =	verts[b];

				if(vi1 < vi0)
				{
					PxU8 tmp = vi0;
					vi0 = vi1;
					vi1 = tmp;
				}

				if(edgeCache.isInCache(vi0, vi1))	//avoid processing edges 2x if possible (this will typically have cache misses about 5% of the time leading to 5% redundant work.
					continue;

				const PxVec3& sv0 = convexVerticesInHfShape[vi0];
				const PxVec3& sv1 = convexVerticesInHfShape[vi1];
				a = b;
				b++;


				if((sv0.y > hfExtreme) && (sv1.y > hfExtreme))
					continue;

				const PxVec3 v0 = sv0;
				const PxVec3 v1 = sv1;
				TriggerTraceSegmentCallback cb;
				traceSegmentHelper.traceSegment(v0, v1, &cb);
				if(cb.intersection)
					return true;
			}
		}
	}

	// Test HeightField vertices
	{
		const Matrix34 tmp34 = multiplyInverseRTLeft(convexAbsPose34, hfAbsPose34);
		const Matrix34 hfShape2ConvexShapeSkew34 = vertexToShapeSkew34 * tmp34;

		for(PxU32 row = minRow; row <= maxRow; row++)
		{
			for(PxU32 column = minColumn; column <= maxColumn; column++)
			{
				const PxU32 hfVertexIndex = row * hf.getNbColumnsFast() + column;
				if(hfUtil.isQueryVertex(hfVertexIndex, row, column))
				{
					// Check if hf vertex is inside the convex
					const PxVec3 hfVertex(rowScale * row, heightScale * hf.getHeight(hfVertexIndex), columnScale * column);
					const PxVec3 hfVertexInConvexShape = hfShape2ConvexShapeSkew34.transform(hfVertex);

					bool inside = true;
					for(PxU32 poly = 0; poly < hull->mNbPolygons; poly++)
					{
						PxReal d = hull->mPolygons[poly].mPlane.distance(hfVertexInConvexShape);
						if(d >= 0)
						{
							inside = false;
							break;
						}
					}
					if(inside)
						return true;
				}
			}
		}
	}
	return false;
}

bool Gu::checkOverlapAABB_heightFieldGeom(const PxGeometry& geom, const PxTransform& pose, const PxBounds3& box)
{
	PX_ASSERT(geom.getType() == PxGeometryType::eHEIGHTFIELD);
	const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom);

	const Matrix34 invAbsPose(pose.getInverse());

	const Box boxInHfShape(
		invAbsPose.transform(box.getCenter()),
		box.getExtents(),
		invAbsPose.m);

	HeightFieldTraceUtil hfUtil(hfGeom);
	return intersectHeightFieldBox(hfUtil, boxInHfShape);
}

bool GeomOverlapCallback_SphereHeightfield(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eSPHERE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eHEIGHTFIELD);
	PX_UNUSED(cache);

	const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);
	const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom1);

	const Sphere sphereInHf(pose1.transformInv(pose0.p), sphereGeom.radius);

	HeightFieldUtil hfUtil(hfGeom);
	return intersectHeightFieldSphere(hfUtil, sphereInHf);
}

bool GeomOverlapCallback_CapsuleHeightfield(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCAPSULE);
	PX_ASSERT(geom1.getType()==PxGeometryType::eHEIGHTFIELD);
	PX_UNUSED(cache);

	const PxCapsuleGeometry& capsuleGeom = static_cast<const PxCapsuleGeometry&>(geom0);
	const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom1);

	const PxTransform capsuleShapeToHfShape = pose1.transformInv(pose0);

	const HeightFieldUtil hfUtil(hfGeom);
	return intersectHeightFieldCapsule(hfUtil, capsuleGeom, capsuleShapeToHfShape);
}

bool GeomOverlapCallback_BoxHeightfield(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eBOX);
	PX_ASSERT(geom1.getType()==PxGeometryType::eHEIGHTFIELD);
	PX_UNUSED(cache);	

	const PxBoxGeometry& boxGeom = static_cast<const PxBoxGeometry&>(geom0);
	const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom1);

	const PxTransform boxShape2HfShape = pose1.transformInv(pose0);

	Box box;
	buildFrom(box, boxShape2HfShape.p, boxGeom.halfExtents, boxShape2HfShape.q);

	HeightFieldTraceUtil hfUtil(hfGeom);
	return intersectHeightFieldBox(hfUtil, box);
}

///////////////////////////////////////////////////////////////////////////////
bool GeomOverlapCallback_ConvexHeightfield(GU_OVERLAP_FUNC_PARAMS)
{
	PX_ASSERT(geom0.getType()==PxGeometryType::eCONVEXMESH);
	PX_ASSERT(geom1.getType()==PxGeometryType::eHEIGHTFIELD);
	PX_UNUSED(cache);	

	const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom0);
	const PxHeightFieldGeometry& hfGeom = static_cast<const PxHeightFieldGeometry&>(geom1);

	ConvexMesh* cm = static_cast<ConvexMesh*>(convexGeom.convexMesh);

	HeightFieldTraceUtil hfUtil(hfGeom);
	return intersectHeightFieldConvex(hfUtil, pose1, *cm, pose0, convexGeom.scale);
}
