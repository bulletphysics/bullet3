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

#include "GuBounds.h"
#include "PxBoxGeometry.h"
#include "PxSphereGeometry.h"
#include "PxCapsuleGeometry.h"
#include "PxPlaneGeometry.h"
#include "PxConvexMeshGeometry.h"
#include "PxTriangleMeshGeometry.h"
#include "PxHeightFieldGeometry.h"
#include "GuInternal.h"
#include "CmUtils.h"
#include "GuConvexMesh.h"
#include "GuConvexMeshData.h"
#include "GuTriangleMesh.h"
#include "GuHeightFieldData.h"
#include "GuHeightField.h"
#include "PsFoundation.h"
#include "GuConvexUtilsInternal.h"
#include "GuBoxConversion.h"

using namespace physx;
using namespace Gu;
using namespace Ps::aos;

static PX_FORCE_INLINE void transformNoEmptyTest(Vec3p& c, Vec3p& ext, const PxMat33& rot, const PxVec3& pos, const CenterExtentsPadded& bounds)
{
	c = rot.transform(bounds.mCenter) + pos;
	ext = Cm::basisExtent(rot.column0, rot.column1, rot.column2, bounds.mExtents);
}

// PT: this one may have duplicates in GuBV4_BoxSweep_Internal.h & GuBV4_Raycast.cpp
static PX_FORCE_INLINE Vec4V multiply3x3V(const Vec4V p, const PxMat33Padded& mat_Padded)
{
	Vec4V ResV = V4Scale(V4LoadU(&mat_Padded.column0.x), V4GetX(p));
	ResV = V4Add(ResV, V4Scale(V4LoadU(&mat_Padded.column1.x), V4GetY(p)));
	ResV = V4Add(ResV, V4Scale(V4LoadU(&mat_Padded.column2.x), V4GetZ(p)));
	return ResV;
}

static PX_FORCE_INLINE void transformNoEmptyTestV(Vec3p& c, Vec3p& ext, const PxMat33Padded& rot, const PxVec3& pos, const CenterExtentsPadded& bounds)
{
	const Vec4V boundsCenterV = V4LoadU(&bounds.mCenter.x);	// PT: this load is safe since extents follow center in the class

	// PT: unfortunately we can't V4LoadU 'pos' directly (it can come directly from users!). So we have to live with this for now:
	const Vec4V posV = Vec4V_From_Vec3V(V3LoadU(&pos.x));
	// PT: but eventually we'd like to use the "unsafe" version (e.g. by switching p&q in PxTransform), which would save 6 instructions on Win32
	const Vec4V cV = V4Add(multiply3x3V(boundsCenterV, rot), posV);
//	const Vec4V cV = V4Add(multiply3x3V(boundsCenterV, rot), V4LoadU(&pos.x));	// ### unsafe
	V4StoreU(cV, &c.x);

	// extended basis vectors
	const Vec4V boundsExtentsV = V4LoadU(&bounds.mExtents.x);	// PT: this load is safe since bounds are padded
	const Vec4V c0V = V4Scale(V4LoadU(&rot.column0.x), V4GetX(boundsExtentsV));
	const Vec4V c1V = V4Scale(V4LoadU(&rot.column1.x), V4GetY(boundsExtentsV));
	const Vec4V c2V = V4Scale(V4LoadU(&rot.column2.x), V4GetZ(boundsExtentsV));

	// find combination of base vectors that produces max. distance for each component = sum of abs()
	Vec4V extentsV = V4Add(V4Abs(c0V), V4Abs(c1V));
	extentsV = V4Add(extentsV, V4Abs(c2V));
	V4StoreU(extentsV, &ext.x);
}

static PX_FORCE_INLINE PxU32 isNonIdentity(const PxVec3& scale)
{
	#define IEEE_1_0	0x3f800000	//!< integer representation of 1.0
	const PxU32* binary = reinterpret_cast<const PxU32*>(&scale.x);
	return (binary[0] - IEEE_1_0)|(binary[1] - IEEE_1_0)|(binary[2] - IEEE_1_0);
}

// PT: please don't inline this one - 300+ lines of rarely used code
static void computeScaledMatrix(PxMat33Padded& rot, const PxMeshScale& scale)
{
	rot = rot * scale.toMat33();
}

static PX_FORCE_INLINE void transformNoEmptyTest(Vec3p& c, Vec3p& ext, const PxTransform& transform, const PxMeshScale& scale, const CenterExtentsPadded& bounds)
{
	PxMat33Padded rot(transform.q);

	if(isNonIdentity(scale.scale))
		computeScaledMatrix(rot, scale);

	transformNoEmptyTestV(c, ext, rot, transform.p, bounds);
}

static PX_FORCE_INLINE void transformNoEmptyTest(Vec3p& c, Vec3p& ext, const PxVec3& pos, const PxMat33Padded& rot, const PxMeshScale& scale, const CenterExtentsPadded& bounds)
{
	if(scale.isIdentity())
		transformNoEmptyTest(c, ext, rot, pos, bounds);
	else
		transformNoEmptyTest(c, ext, rot * scale.toMat33(), pos, bounds);
}

static void computeMeshBounds(const PxTransform& pose, const CenterExtentsPadded* PX_RESTRICT localSpaceBounds, const PxMeshScale& meshScale, Vec3p& origin, Vec3p& extent)
{
	transformNoEmptyTest(origin, extent, pose, meshScale, *localSpaceBounds);
}

static void computePlaneBounds(PxBounds3& bounds, const PxTransform& pose, float contactOffset, float inflation)
{
	// PT: A plane is infinite, so usually the bounding box covers the whole world.
	// Now, in particular cases when the plane is axis-aligned, we can take
	// advantage of this to compute a smaller bounding box anyway.

	// PT: we use PX_MAX_BOUNDS_EXTENTS to be compatible with PxBounds3::setMaximal,
	// and to make sure that the value doesn't collide with the BP's sentinels.
	const PxF32 bigValue = PX_MAX_BOUNDS_EXTENTS;
//	const PxF32 bigValue = 1000000.0f;
	PxVec3 minPt = PxVec3(-bigValue, -bigValue, -bigValue);
	PxVec3 maxPt = PxVec3(bigValue, bigValue, bigValue);

	const PxVec3 planeNormal = pose.q.getBasisVector0();
	const PxPlane plane(pose.p, planeNormal);

	const float nx = PxAbs(planeNormal.x);
	const float ny = PxAbs(planeNormal.y);
	const float nz = PxAbs(planeNormal.z);
	const float epsilon = 1e-6f;
	const float oneMinusEpsilon = 1.0f - epsilon;
	if(nx>oneMinusEpsilon && ny<epsilon && nz<epsilon)
	{
		if(planeNormal.x>0.0f)	maxPt.x = -plane.d + contactOffset;
		else					minPt.x = plane.d - contactOffset;
	}
	else if(nx<epsilon && ny>oneMinusEpsilon && nz<epsilon)
	{
		if(planeNormal.y>0.0f)	maxPt.y = -plane.d + contactOffset;
		else					minPt.y = plane.d - contactOffset;
	}
	else if(nx<epsilon && ny<epsilon && nz>oneMinusEpsilon)
	{
		if(planeNormal.z>0.0f)	maxPt.z = -plane.d + contactOffset;
		else					minPt.z = plane.d - contactOffset;
	}

	// PT: it is important to compute the min/max form directly without going through the
	// center/extents intermediate form. With PX_MAX_BOUNDS_EXTENTS, those back-and-forth
	// computations destroy accuracy.

	// PT: inflation actually destroys the bounds really. We keep it to please UTs but this is broken (DE10595).
	// (e.g. for SQ 1% of PX_MAX_BOUNDS_EXTENTS is still a huge number, effectively making the AABB infinite and defeating the point of the above computation)
	if(inflation!=1.0f)
	{
		const PxVec3 c = (maxPt + minPt)*0.5f;
		const PxVec3 e = (maxPt - minPt)*0.5f*inflation;
		minPt = c - e;
		maxPt = c + e;
	}

	bounds.minimum = minPt;
	bounds.maximum = maxPt;
}

static PX_FORCE_INLINE void inflateBounds(PxBounds3& bounds, const Vec3p& origin, const Vec3p& extents, float contactOffset, float inflation)
{
	Vec4V extentsV = V4LoadU(&extents.x);
	extentsV = V4Add(extentsV, V4Load(contactOffset));
	extentsV = V4Scale(extentsV, FLoad(inflation));

	const Vec4V originV = V4LoadU(&origin.x);
	const Vec4V minV = V4Sub(originV, extentsV);
	const Vec4V maxV = V4Add(originV, extentsV);

	StoreBounds(bounds, minV, maxV);
}

static PX_FORCE_INLINE Vec4V basisExtentV(const PxMat33Padded& basis, const PxVec3& extent, float offset, float inflation)
{
	// extended basis vectors
	const Vec4V c0V = V4Scale(V4LoadU(&basis.column0.x), FLoad(extent.x));
	const Vec4V c1V = V4Scale(V4LoadU(&basis.column1.x), FLoad(extent.y));
	const Vec4V c2V = V4Scale(V4LoadU(&basis.column2.x), FLoad(extent.z));

	// find combination of base vectors that produces max. distance for each component = sum of abs()
	Vec4V extentsV = V4Add(V4Abs(c0V), V4Abs(c1V));
	extentsV = V4Add(extentsV, V4Abs(c2V));
	extentsV = V4Add(extentsV, V4Load(offset));
	extentsV = V4Scale(extentsV, FLoad(inflation));
	return extentsV;
}

void Gu::computeBounds(PxBounds3& bounds, const PxGeometry& geometry, const PxTransform& pose, float contactOffset, const CenterExtentsPadded* PX_RESTRICT localSpaceBounds, float inflation)
{
	PX_ASSERT(contactOffset==0.0f || inflation==1.0f);

	// Box, Convex, Mesh and HeightField will compute local bounds and pose to world space.
	// Sphere, Capsule & Plane will compute world space bounds directly.

	switch(geometry.getType())
	{
		case PxGeometryType::eSPHERE:
		{
			PX_ASSERT(!localSpaceBounds);

			const PxSphereGeometry& shape = static_cast<const PxSphereGeometry&>(geometry);
			const PxVec3 extents((shape.radius+contactOffset)*inflation);
			bounds.minimum = pose.p - extents;
			bounds.maximum = pose.p + extents;
		}
		break;

		case PxGeometryType::ePLANE:
		{
			PX_ASSERT(!localSpaceBounds);

			computePlaneBounds(bounds, pose, contactOffset, inflation);
		}
		break;

		case PxGeometryType::eCAPSULE:
		{
			PX_ASSERT(!localSpaceBounds);

			const PxCapsuleGeometry& shape = static_cast<const PxCapsuleGeometry& >(geometry);
			const PxVec3 d = pose.q.getBasisVector0();
			PxVec3 extents;
			for(PxU32 ax = 0; ax<3; ax++)
				extents[ax] = (PxAbs(d[ax]) * shape.halfHeight + shape.radius + contactOffset)*inflation;
			bounds.minimum = pose.p - extents;
			bounds.maximum = pose.p + extents;
		}
		break;

		case PxGeometryType::eBOX:
		{
			PX_ASSERT(!localSpaceBounds);

			const PxBoxGeometry& shape = static_cast<const PxBoxGeometry& >(geometry);

			const Vec3p origin(pose.p);

			const PxMat33Padded basis(pose.q);

			const Vec4V extentsV = basisExtentV(basis, shape.halfExtents, contactOffset, inflation);

			const Vec4V originV = V4LoadU(&origin.x);
			const Vec4V minV = V4Sub(originV, extentsV);
			const Vec4V maxV = V4Add(originV, extentsV);

			StoreBounds(bounds, minV, maxV);
		}
		break;

		case PxGeometryType::eCONVEXMESH:
		{
			const PxConvexMeshGeometry& shape = static_cast<const PxConvexMeshGeometry& >(geometry);
			const Gu::ConvexHullData& hullData = static_cast<const Gu::ConvexMesh*>(shape.convexMesh)->getHull();

			const bool useTightBounds = shape.meshFlags & PxConvexMeshGeometryFlag::eTIGHT_BOUNDS;
			if(useTightBounds)
			{
				PxMat33Padded rot(pose.q);

				if(isNonIdentity(shape.scale.scale))
					computeScaledMatrix(rot, shape.scale);

				PxU32 nb = hullData.mNbHullVertices;
				const PxVec3* v = hullData.getHullVertices();
				Vec4V minV;
				Vec4V maxV;

				{
					const Vec4V vertexV = multiply3x3V(V4LoadU(&v->x), rot);
					v++;

					minV = vertexV;
					maxV = vertexV;
					nb--;
				}

				while(nb--)
				{
					const Vec4V vertexV = multiply3x3V(V4LoadU(&v->x), rot);
					v++;

					minV = V4Min(minV, vertexV);
					maxV = V4Max(maxV, vertexV);
				}

				const Vec4V offsetV = V4Load(contactOffset);
				minV = V4Sub(minV, offsetV);
				maxV = V4Add(maxV, offsetV);

				const Vec4V posV = Vec4V_From_Vec3V(V3LoadU(&pose.p.x));
				maxV = V4Add(maxV, posV);
				minV = V4Add(minV, posV);

				// Inflation
				{
					const Vec4V centerV = V4Scale(V4Add(maxV, minV), FLoad(0.5f));
					const Vec4V extentsV = V4Scale(V4Sub(maxV, minV), FLoad(0.5f*inflation));
					maxV = V4Add(centerV, extentsV);
					minV = V4Sub(centerV, extentsV);
				}

				StoreBounds(bounds, minV, maxV);
			}
			else
			{
				Vec3p origin, extents;
				computeMeshBounds(pose, localSpaceBounds ? localSpaceBounds : &hullData.getPaddedBounds(), shape.scale, origin, extents);

				inflateBounds(bounds, origin, extents, contactOffset, inflation);
			}
		}
		break;

		case PxGeometryType::eTRIANGLEMESH:
		{
			Vec3p origin, extents;
			const PxTriangleMeshGeometry& shape = static_cast<const PxTriangleMeshGeometry& >(geometry);
			computeMeshBounds(pose, localSpaceBounds ? localSpaceBounds : &static_cast<const Gu::TriangleMesh*>(shape.triangleMesh)->getPaddedBounds(), shape.scale, origin, extents);

			inflateBounds(bounds, origin, extents, contactOffset, inflation);
		}
		break;

		case PxGeometryType::eHEIGHTFIELD:
		{
			const PxHeightFieldGeometry& shape = static_cast<const PxHeightFieldGeometry& >(geometry);
			const PxMeshScale scale(PxVec3(shape.rowScale, shape.heightScale, shape.columnScale), PxQuat(PxIdentity));
			
			if(!localSpaceBounds)
				localSpaceBounds = &static_cast<const Gu::HeightField*>(shape.heightField)->getData().getPaddedBounds();

			//Compute and inflate the bounds from the pose, scale and center/extents.
			Vec3p origin, extents;
			computeMeshBounds(pose, localSpaceBounds, scale, origin, extents);
			inflateBounds(bounds, origin, extents, contactOffset, inflation);
		}
		break;
		case PxGeometryType::eGEOMETRY_COUNT:
		case PxGeometryType::eINVALID:
		{
			PX_ASSERT(0);		
			Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "Gu::GeometryUnion::computeBounds: Unknown shape type.");
		}
	}
}

// PT: TODO: refactor this with regular function
PxF32 Gu::computeBoundsWithCCDThreshold(Vec3p& origin, Vec3p& extent, const PxGeometry& geometry, const PxTransform& pose, const CenterExtentsPadded* PX_RESTRICT localSpaceBounds)
{
	// Box, Convex, Mesh and HeightField will compute local bounds and pose to world space.
	// Sphere, Capsule & Plane will compute world space bounds directly.

	const PxReal inSphereRatio = 0.75f;

	//The CCD thresholds are as follows:
	//(1) sphere = inSphereRatio * radius
	//(2) plane = inf (we never need CCD against this shape)
	//(3) capsule = inSphereRatio * radius
	//(4) box = inSphereRatio * (box minimum extent axis)
	//(5) convex = inSphereRatio * convex in-sphere * min scale
	//(6) triangle mesh = 0.f (polygons have 0 thickness)
	//(7) heightfields = 0.f (polygons have 0 thickness)

	//The decision to enter CCD depends on the sum of the shapes' CCD thresholds. One of the 2 shapes must be a 
	//sphere/capsule/box/convex so the sum of the CCD thresholds will be non-zero.

	switch (geometry.getType())
	{
		case PxGeometryType::eSPHERE:
		{
			PX_ASSERT(!localSpaceBounds);

			const PxSphereGeometry& shape = static_cast<const PxSphereGeometry&>(geometry);
			origin = pose.p;
			extent = PxVec3(shape.radius, shape.radius, shape.radius);
			return shape.radius*inSphereRatio;
		}
		case PxGeometryType::ePLANE:
		{
			PX_ASSERT(!localSpaceBounds);

			PxBounds3 bounds;
			computePlaneBounds(bounds, pose, 0.0f, 1.0f);
			origin = bounds.getCenter();
			extent = bounds.getExtents();
			return PX_MAX_REAL;
		}
		case PxGeometryType::eCAPSULE:
		{
			PX_ASSERT(!localSpaceBounds);

			const PxCapsuleGeometry& shape = static_cast<const PxCapsuleGeometry&>(geometry);
			origin = pose.p;
			const PxVec3 d = pose.q.getBasisVector0();
			for(PxU32 ax = 0; ax<3; ax++)
				extent[ax] = PxAbs(d[ax]) * shape.halfHeight + shape.radius;
			return shape.radius * inSphereRatio;
		}

		case PxGeometryType::eBOX:
		{
			PX_ASSERT(!localSpaceBounds);

			const PxBoxGeometry& shape = static_cast<const PxBoxGeometry&>(geometry);

			const PxMat33 rot(pose.q);
			extent = Cm::basisExtent(rot.column0, rot.column1, rot.column2, shape.halfExtents);

			origin = pose.p;

			return PxMin(PxMin(shape.halfExtents.x, shape.halfExtents.y), shape.halfExtents.z)*inSphereRatio;
		}

		case PxGeometryType::eCONVEXMESH:
		{
			const PxConvexMeshGeometry& shape = static_cast<const PxConvexMeshGeometry&>(geometry);
			const Gu::ConvexHullData& hullData = static_cast<const Gu::ConvexMesh*>(shape.convexMesh)->getHull();
			computeMeshBounds(pose, localSpaceBounds ? localSpaceBounds : &hullData.getPaddedBounds(), shape.scale, origin, extent);
			return PxMin(shape.scale.scale.z, PxMin(shape.scale.scale.x, shape.scale.scale.y)) * hullData.mInternal.mRadius * inSphereRatio;
		}

		case PxGeometryType::eTRIANGLEMESH:
		{
			const PxTriangleMeshGeometry& shape = static_cast<const PxTriangleMeshGeometry&>(geometry);
			computeMeshBounds(pose, localSpaceBounds ? localSpaceBounds : &static_cast<const Gu::TriangleMesh*>(shape.triangleMesh)->getPaddedBounds(), shape.scale, origin, extent);
			return 0.0f;
		}

		case PxGeometryType::eHEIGHTFIELD:
		{
			const PxHeightFieldGeometry& shape = static_cast<const PxHeightFieldGeometry&>(geometry);
			const PxMeshScale scale(PxVec3(shape.rowScale, shape.heightScale, shape.columnScale), PxQuat(PxIdentity));
			const Gu::HeightFieldData& data = static_cast<const Gu::HeightField*>(shape.heightField)->getData();
			computeMeshBounds(pose, localSpaceBounds ? localSpaceBounds : &data.getPaddedBounds(), scale, origin, extent);
			return 0.f;
		}

		case PxGeometryType::eGEOMETRY_COUNT:
		case PxGeometryType::eINVALID:
		{
			PX_ASSERT(0);		
			Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "Gu::GeometryUnion::computeBounds: Unknown shape type.");
		}
	}
	return PX_MAX_REAL;
}


static PX_FORCE_INLINE void computeBoxExtentsAroundCapsule(PxVec3& extents, const PxCapsuleGeometry& capsuleGeom, float inflation)
{
	extents.x = (capsuleGeom.radius + capsuleGeom.halfHeight) * inflation;
	extents.y = capsuleGeom.radius * inflation;
	extents.z = capsuleGeom.radius * inflation;
}

static const PxReal SQ_PRUNER_INFLATION = 1.01f; // pruner test shape inflation (not narrow phase shape)

static void computeMeshBounds(const PxVec3& pos, const PxMat33Padded& rot, const CenterExtentsPadded* PX_RESTRICT localSpaceBounds, const PxMeshScale& meshScale, Vec3p& origin, Vec3p& extent)
{
	Ps::prefetchLine(localSpaceBounds);	// PT: this one helps reducing L2 misses in transformNoEmptyTest
	transformNoEmptyTest(origin, extent, pos, rot, meshScale, *localSpaceBounds);
}

// PT: warning: this writes 4 bytes after the end of 'bounds'. Calling code must ensure it is safe to do so.
static PX_FORCE_INLINE void computeMinMaxBounds(PxBounds3* PX_RESTRICT bounds, const Vec3p& c, const Vec3p& e, float prunerInflation, float offset)
{
	const Vec4V extentsV = V4Scale(V4Add(V4LoadU(&e.x), V4Load(offset)), FLoad(prunerInflation));
	const Vec4V centerV = V4LoadU(&c.x);
	const Vec4V minV = V4Sub(centerV, extentsV);
	const Vec4V maxV = V4Add(centerV, extentsV);
	V4StoreU(minV, &bounds->minimum.x);
	V4StoreU(maxV, &bounds->maximum.x);
}

ShapeData::ShapeData(const PxGeometry& g, const PxTransform& t, PxReal inflation)
{
	using namespace physx::shdfnd::aos;

	// PT: this cast to matrix is already done in GeometryUnion::computeBounds (e.g. for boxes). So we do it first,
	// then we'll pass the matrix directly to computeBoundsShapeData, to avoid the double conversion.
	const bool isOBB = PxAbs(t.q.w) < 0.999999f;
	if(isOBB)
	{
		// PT: writes 4 bytes after 'rot' but it's safe since we then write 'center' just afterwards
		buildFrom(mGuBox, t.q);
	}
	else
	{
		mGuBox.rot = PxMat33(PxIdentity);
	}

	// PT: can't use V4Load here since there's no guarantee on 't.p'
	// PT: must store 'center'  after 'rot' now
	mGuBox.center = t.p;

	// Compute AABB, used by the BucketPruner as cullBox
	switch(g.getType())
	{
		case PxGeometryType::eSPHERE:
		{
			const PxSphereGeometry& shape = static_cast<const PxSphereGeometry&>(g);
			computeMinMaxBounds(&mPrunerInflatedAABB, mGuBox.center, PxVec3(0.0f), SQ_PRUNER_INFLATION, shape.radius+inflation);

			//

			reinterpret_cast<Sphere&>(mGuSphere) = Sphere(t.p, shape.radius);
		}
		break;

		case PxGeometryType::eCAPSULE:
		{
			const PxCapsuleGeometry& shape = static_cast<const PxCapsuleGeometry&>(g);
			const Vec3p extents = mGuBox.rot.column0.abs() * shape.halfHeight;
			computeMinMaxBounds(&mPrunerInflatedAABB, mGuBox.center, extents, SQ_PRUNER_INFLATION, shape.radius+inflation);

			//

			Capsule& dstWorldCapsule = reinterpret_cast<Capsule&>(mGuCapsule); // store a narrow phase version copy
			getCapsule(dstWorldCapsule, shape, t);

			mGuBox.extents.x = shape.halfHeight;

			// compute PxBoxGeometry pruner geom around input capsule geom; transform remains unchanged

			computeBoxExtentsAroundCapsule(mPrunerBoxGeomExtents, shape, SQ_PRUNER_INFLATION);
		}
		break;

		case PxGeometryType::eBOX:
		{
			const PxBoxGeometry& shape = static_cast<const PxBoxGeometry&>(g);
			// PT: cast is safe because 'rot' followed by other members
			Vec4V extentsV = basisExtentV(static_cast<const PxMat33Padded&>(mGuBox.rot), shape.halfExtents, inflation, SQ_PRUNER_INFLATION);

			// PT: c/e-to-m/M conversion
			const Vec4V centerV = V4LoadU(&mGuBox.center.x);
			const Vec4V minV = V4Sub(centerV, extentsV);
			const Vec4V maxV = V4Add(centerV, extentsV);
			V4StoreU(minV, &mPrunerInflatedAABB.minimum.x);
			V4StoreU(maxV, &mPrunerInflatedAABB.maximum.x);	// PT: WARNING: writes past end of class

			//

			mGuBox.extents	= shape.halfExtents;	// PT: TODO: use SIMD
			mPrunerBoxGeomExtents = shape.halfExtents*SQ_PRUNER_INFLATION;
		}
		break;

		case PxGeometryType::eCONVEXMESH:
		{
			const PxConvexMeshGeometry& shape = static_cast<const PxConvexMeshGeometry&>(g);

			const ConvexMesh* cm = static_cast<const ConvexMesh*>(shape.convexMesh);
			const ConvexHullData* hullData = &cm->getHull();

			// PT: cast is safe since 'rot' is followed by other members of the box
			Vec3p center, extents;
			computeMeshBounds(mGuBox.center, static_cast<const PxMat33Padded&>(mGuBox.rot), &hullData->getPaddedBounds(), shape.scale, center, extents);

			computeMinMaxBounds(&mPrunerInflatedAABB, center, extents, SQ_PRUNER_INFLATION, inflation);

			//

			Box prunerBox;
			computeOBBAroundConvex(prunerBox, shape, cm, t);
			mGuBox.rot = prunerBox.rot;	// PT: TODO: optimize this copy

			// AP: pruners are now responsible for growing the OBB by 1% for overlap/sweep/GJK accuracy
			mPrunerBoxGeomExtents = prunerBox.extents*SQ_PRUNER_INFLATION;
			mGuBox.center = prunerBox.center;
		}
		break;

		case PxGeometryType::ePLANE:
		case PxGeometryType::eTRIANGLEMESH:
		case PxGeometryType::eHEIGHTFIELD:
		case PxGeometryType::eGEOMETRY_COUNT:
		case PxGeometryType::eINVALID:
			PX_ALWAYS_ASSERT_MESSAGE("PhysX internal error: Invalid shape in ShapeData contructor.");
	}

	// PT: WARNING: these writes must stay after the above code
	mIsOBB = PxU32(isOBB);
	mType = PxU16(g.getType());
}




