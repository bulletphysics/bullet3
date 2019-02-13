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


#ifndef PX_PHYSICS_COMMON_SCALING
#define PX_PHYSICS_COMMON_SCALING

#include "foundation/PxBounds3.h"
#include "foundation/PxMat33.h"
#include "geometry/PxMeshScale.h"
#include "CmMatrix34.h"
#include "CmUtils.h"
#include "PsMathUtils.h"

namespace physx
{
namespace Cm
{
	// class that can perform scaling fast.  Relatively large size, generated from PxMeshScale on demand.
	// CS: I've removed most usages of this class, because most of the time only one-way transform is needed.
	// If you only need a temporary FastVertex2ShapeScaling, setup your transform as PxMat34Legacy and use
	// normal matrix multiplication or a transform() overload to convert points and bounds between spaces.
	class FastVertex2ShapeScaling
	{
	public:
		PX_INLINE FastVertex2ShapeScaling()
		{
			//no scaling by default:
			vertex2ShapeSkew = PxMat33(PxIdentity);
			shape2VertexSkew = PxMat33(PxIdentity);
			mFlipNormal = false;
		}

		PX_INLINE explicit FastVertex2ShapeScaling(const PxMeshScale& scale)
		{
			init(scale);
		}

		PX_INLINE FastVertex2ShapeScaling(const PxVec3& scale, const PxQuat& rotation)
		{
			init(scale, rotation);
		}

		PX_INLINE void init(const PxMeshScale& scale)
		{
			init(scale.scale, scale.rotation);
		}

		PX_INLINE void setIdentity()
		{
			vertex2ShapeSkew = PxMat33(PxIdentity);
			shape2VertexSkew = PxMat33(PxIdentity);
			mFlipNormal = false;
		}

		PX_INLINE void init(const PxVec3& scale, const PxQuat& rotation)
		{
			// TODO: may want to optimize this for cases where we have uniform or axis aligned scaling!
			// That would introduce branches and it's unclear to me whether that's faster than just doing the math.
			// Lazy computation would be another option, at the cost of introducing even more branches.

			const PxMat33 R(rotation);
			vertex2ShapeSkew = R.getTranspose();
			const PxMat33 diagonal = PxMat33::createDiagonal(scale);
			vertex2ShapeSkew = vertex2ShapeSkew * diagonal;
			vertex2ShapeSkew = vertex2ShapeSkew * R;

			/*
			The inverse, is, explicitly:
			shape2VertexSkew.setTransposed(R);
			shape2VertexSkew.multiplyDiagonal(PxVec3(1.0f/scale.x, 1.0f/scale.y, 1.0f/scale.z));
			shape2VertexSkew *= R;

			It may be competitive to compute the inverse -- though this has a branch in it:
			*/
			
			shape2VertexSkew = vertex2ShapeSkew.getInverse();			

			mFlipNormal = ((scale.x * scale.y * scale.z) < 0.0f);
		}

		PX_FORCE_INLINE void flipNormal(PxVec3& v1, PxVec3& v2) const
		{
			if (mFlipNormal)
			{
				PxVec3 tmp = v1; v1 = v2; v2 = tmp;
			}
		}

		PX_FORCE_INLINE PxVec3 operator*  (const PxVec3& src) const
		{
			return vertex2ShapeSkew * src;
		}
		PX_FORCE_INLINE PxVec3 operator%  (const PxVec3& src) const
		{
			return shape2VertexSkew * src;
		}

		PX_FORCE_INLINE const PxMat33& getVertex2ShapeSkew() const
		{
			return vertex2ShapeSkew;
		}

		PX_FORCE_INLINE const PxMat33& getShape2VertexSkew() const
		{
			return shape2VertexSkew;
		}

		PX_INLINE Cm::Matrix34 getVertex2WorldSkew(const Cm::Matrix34& shape2world) const
		{
			const Cm::Matrix34 vertex2worldSkew = shape2world * getVertex2ShapeSkew();
			//vertex2worldSkew = shape2world * [vertex2shapeSkew, 0]
			//[aR at] * [bR bt] = [aR * bR		aR * bt + at]  NOTE: order of operations important so it works when this ?= left ?= right.
			return vertex2worldSkew;
		}

		PX_INLINE Cm::Matrix34 getWorld2VertexSkew(const Cm::Matrix34& shape2world) const
		{
			//world2vertexSkew = shape2vertex * invPQ(shape2world)
			//[aR 0] * [bR' -bR'*bt] = [aR * bR'		-aR * bR' * bt + 0]

			const PxMat33 rotate( shape2world[0], shape2world[1], shape2world[2] );
			const PxMat33 M = getShape2VertexSkew() * rotate.getTranspose();
			return Cm::Matrix34(M[0], M[1], M[2], -M * shape2world[3]);			
		}

		//! Transforms a shape space OBB to a vertex space OBB.  All 3 params are in and out.
		void transformQueryBounds(PxVec3& center, PxVec3& extents, PxMat33& basis)	const
		{
			basis.column0 = shape2VertexSkew * (basis.column0 * extents.x);
			basis.column1 = shape2VertexSkew * (basis.column1 * extents.y);
			basis.column2 = shape2VertexSkew * (basis.column2 * extents.z);

			center = shape2VertexSkew * center;
			extents = Ps::optimizeBoundingBox(basis);
		}

		void transformPlaneToShapeSpace(const PxVec3& nIn, const PxReal dIn, PxVec3& nOut, PxReal& dOut)	const
		{
			const PxVec3 tmp = shape2VertexSkew.transformTranspose(nIn);
			const PxReal denom = 1.0f / tmp.magnitude();
			nOut = tmp * denom;
			dOut = dIn * denom;
		}

		PX_FORCE_INLINE bool flipsNormal() const { return mFlipNormal; }

	private:
		PxMat33 vertex2ShapeSkew;
		PxMat33 shape2VertexSkew;
		bool mFlipNormal;
	};

	PX_FORCE_INLINE void getScaledVertices(PxVec3* v, const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, bool idtMeshScale, const Cm::FastVertex2ShapeScaling& scaling)
	{
		if(idtMeshScale)
		{
			v[0] = v0;
			v[1] = v1;
			v[2] = v2;
		}
		else
		{
			const PxI32 winding = scaling.flipsNormal() ? 1 : 0;
			v[0] = scaling * v0;
			v[1+winding] = scaling * v1;
			v[2-winding] = scaling * v2;
		}
	}

} // namespace Cm


PX_INLINE Cm::Matrix34 operator*(const PxTransform& transform, const PxMeshScale& scale) 
{
	return Cm::Matrix34(PxMat33(transform.q) * scale.toMat33(), transform.p);
}

PX_INLINE Cm::Matrix34 operator*(const PxMeshScale& scale, const PxTransform& transform) 
{
	const PxMat33 scaleMat = scale.toMat33();
	const PxMat33 t = PxMat33(transform.q);
	const PxMat33 r = scaleMat * t;
	const PxVec3 p = scaleMat * transform.p;
	return Cm::Matrix34(r, p);
}

PX_INLINE Cm::Matrix34 operator*(const Cm::Matrix34& transform, const PxMeshScale& scale) 
{
	return Cm::Matrix34(transform.m * scale.toMat33(), transform.p);
}

PX_INLINE Cm::Matrix34 operator*(const PxMeshScale& scale, const Cm::Matrix34& transform) 
{
	const PxMat33 scaleMat = scale.toMat33();
	return Cm::Matrix34(scaleMat * transform.m, scaleMat * transform.p);
}

}

#endif
