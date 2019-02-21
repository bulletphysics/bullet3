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

#ifndef GU_SWEEP_BOX_TRIANGLE_SAT_H
#define GU_SWEEP_BOX_TRIANGLE_SAT_H

#include "GuSweepSharedTests.h"
#include "PxTriangle.h"

	#define	RetType	int
	#define	MTDType	bool

namespace physx
{
namespace Gu
{

// We have separation if one of those conditions is true:
//     -BoxExt > TriMax (box strictly to the right of the triangle)
//      BoxExt < TriMin (box strictly to the left of the triangle
// <=>  d0 = -BoxExt - TriMax > 0
//      d1 = BoxExt - TriMin < 0
// Hence we have overlap if d0 <= 0 and d1 >= 0
// overlap = (d0<=0.0f && d1>=0.0f)
#define TEST_OVERLAP									\
	const float d0 = -BoxExt - TriMax;					\
	const float d1 = BoxExt - TriMin;					\
	const bool bIntersect = (d0<=0.0f && d1>=0.0f);		\
	bValidMTD &= bIntersect;

	// PT: inlining this one is important. Returning floats looks bad but is faster on Xbox.
	static PX_FORCE_INLINE RetType testAxis(const PxTriangle& tri, const PxVec3& extents, const PxVec3& dir, const PxVec3& axis, MTDType& bValidMTD, float& tfirst, float& tlast)
	{
		const float d0t = tri.verts[0].dot(axis);
		const float d1t = tri.verts[1].dot(axis);
		const float d2t = tri.verts[2].dot(axis);

		float TriMin = PxMin(d0t, d1t);
		float TriMax = PxMax(d0t, d1t);
		TriMin = PxMin(TriMin, d2t);
		TriMax = PxMax(TriMax, d2t);

		////////

		const float BoxExt = PxAbs(axis.x)*extents.x + PxAbs(axis.y)*extents.y + PxAbs(axis.z)*extents.z;
		TEST_OVERLAP

		const float v = dir.dot(axis);
		if(PxAbs(v) < 1.0E-6f)
			return bIntersect;
		const float oneOverV = -1.0f / v;

	//	float t0 = d0 * oneOverV;
	//	float t1 = d1 * oneOverV;
	//	if(t0 > t1)	TSwap(t0, t1);
		const float t0_ = d0 * oneOverV;
		const float t1_ = d1 * oneOverV;
		float t0 = PxMin(t0_, t1_);
		float t1 = PxMax(t0_, t1_);

		if(t0 > tlast)	return false;
		if(t1 < tfirst)	return false;

	//	if(t1 < tlast)	tlast = t1;
		tlast = PxMin(t1, tlast);

	//	if(t0 > tfirst)	tfirst = t0;
		tfirst = PxMax(t0, tfirst);

		return true;
	}

	template<const int XYZ>
	static PX_FORCE_INLINE RetType testAxisXYZ(const PxTriangle& tri, const PxVec3& extents, const PxVec3& dir, float oneOverDir, MTDType& bValidMTD, float& tfirst, float& tlast)
	{
		const float d0t = tri.verts[0][XYZ];
		const float d1t = tri.verts[1][XYZ];
		const float d2t = tri.verts[2][XYZ];

		float TriMin = PxMin(d0t, d1t);
		float TriMax = PxMax(d0t, d1t);
		TriMin = PxMin(TriMin, d2t);
		TriMax = PxMax(TriMax, d2t);

		////////

		const float BoxExt = extents[XYZ];
		TEST_OVERLAP

		const float v = dir[XYZ];
		if(PxAbs(v) < 1.0E-6f)
			return bIntersect;

		const float oneOverV = -oneOverDir;

	//	float t0 = d0 * oneOverV;
	//	float t1 = d1 * oneOverV;
	//	if(t0 > t1)	TSwap(t0, t1);
		const float t0_ = d0 * oneOverV;
		const float t1_ = d1 * oneOverV;
		float t0 = PxMin(t0_, t1_);
		float t1 = PxMax(t0_, t1_);

		if(t0 > tlast)	return false;
		if(t1 < tfirst)	return false;

	//	if(t1 < tlast)	tlast = t1;
		tlast = PxMin(t1, tlast);

	//	if(t0 > tfirst)	tfirst = t0;
		tfirst = PxMax(t0, tfirst);

		return true;
	}

	PX_FORCE_INLINE int testSeparationAxes(	const PxTriangle& tri, const PxVec3& extents,
											const PxVec3& normal, const PxVec3& dir, const PxVec3& oneOverDir, float tmax, float& tcoll)
	{
		bool bValidMTD = true;
		float tfirst = -FLT_MAX;
		float tlast  = FLT_MAX;

		// Triangle normal
		if(!testAxis(tri, extents, dir, normal, bValidMTD, tfirst, tlast))
			return 0;

		// Box normals
		if(!testAxisXYZ<0>(tri, extents, dir, oneOverDir.x, bValidMTD, tfirst, tlast))
			return 0;
		if(!testAxisXYZ<1>(tri, extents, dir, oneOverDir.y, bValidMTD, tfirst, tlast))
			return 0;
		if(!testAxisXYZ<2>(tri, extents, dir, oneOverDir.z, bValidMTD, tfirst, tlast))
			return 0;

		// Edges
		for(PxU32 i=0; i<3; i++)
		{
			int ip1 = int(i+1);
			if(i>=2)	ip1 = 0;
			const PxVec3 TriEdge = tri.verts[ip1] - tri.verts[i];

			{
				const PxVec3 Sep = Ps::cross100(TriEdge);
				if((Sep.dot(Sep))>=1.0E-6f && !testAxis(tri, extents, dir, Sep, bValidMTD, tfirst, tlast))
					return 0;
			}
			{
				const PxVec3 Sep = Ps::cross010(TriEdge);
				if((Sep.dot(Sep))>=1.0E-6f && !testAxis(tri, extents, dir, Sep, bValidMTD, tfirst, tlast))
					return 0;
			}
			{
				const PxVec3 Sep = Ps::cross001(TriEdge);
				if((Sep.dot(Sep))>=1.0E-6f && !testAxis(tri, extents, dir, Sep, bValidMTD, tfirst, tlast))
					return 0;
			}
		}

		if(tfirst > tmax || tlast < 0.0f)
			return 0;

		if(tfirst <= 0.0f)
		{
			if(!bValidMTD)
				return 0;
			tcoll = 0.0f;
		}
		else tcoll = tfirst;

		return 1;
	}

	//! Inlined version of triBoxSweepTestBoxSpace. See that other function for comments.
	PX_FORCE_INLINE int triBoxSweepTestBoxSpace_inlined(const PxTriangle& tri, const PxVec3& extents, const PxVec3& dir, const PxVec3& oneOverDir, float tmax, float& toi, PxU32 doBackfaceCulling)
	{
		// Create triangle normal
		PxVec3 triNormal;
		tri.denormalizedNormal(triNormal);

		// Backface culling
		if(doBackfaceCulling && (triNormal.dot(dir)) >= 0.0f)	// ">=" is important !
			return 0;

		// The SAT test will properly detect initial overlaps, no need for extra tests
		return testSeparationAxes(tri, extents, triNormal, dir, oneOverDir, tmax, toi);
	}

	/**
	Sweeps a box against a triangle, using a 'SAT' approach (Separating Axis Theorem).

	The test is performed in box-space, i.e. the box is axis-aligned and its center is (0,0,0). In other words it is
	defined by its extents alone. The triangle must have been transformed to this "box-space" before calling the function.

	\param tri					[in] triangle in box-space
	\param extents				[in] box extents
	\param dir					[in] sweep direction. Does not need to be normalized.
	\param oneOverDir			[in] precomputed inverse of sweep direction
	\param tmax					[in] sweep length
	\param toi					[out] time of impact/impact distance. Does not need to be initialized before calling the function.
	\param doBackfaceCulling	[in] true to enable backface culling, false for double-sided triangles
	\return	non-zero value if an impact has been found (in which case returned 'toi' value is valid)
	*/
	int triBoxSweepTestBoxSpace(const PxTriangle& tri, const PxVec3& extents, const PxVec3& dir, const PxVec3& oneOverDir, float tmax, float& toi, bool doBackfaceCulling);

} // namespace Gu

}

#endif
