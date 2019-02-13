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

#ifndef GU_VEC_PLANE_H
#define GU_VEC_PLANE_H
/** \addtogroup geomutils
@{
*/

#include "foundation/PxVec3.h"
#include "foundation/PxPlane.h"
#include "CmPhysXCommon.h"
#include "PsVecMath.h"

/**
\brief Representation of a plane.

Plane equation used: a*x + b*y + c*z + d = 0
*/
namespace physx
{
namespace Gu
{
	
	class PlaneV
	{
	public:
		/**
		\brief Constructor
		*/
		PX_FORCE_INLINE PlaneV()
		{
		}

		/**
		\brief Constructor from a normal and a distance
		*/
		PX_FORCE_INLINE PlaneV(const Ps::aos::FloatVArg nx, const Ps::aos::FloatVArg ny, const Ps::aos::FloatVArg nz, const Ps::aos::FloatVArg _d)
		{
			set(nx, ny, nz, _d);
		}

		PX_FORCE_INLINE PlaneV(const PxPlane& plane)
		{
			using namespace Ps::aos;
			const Vec3V _n = V3LoadU(plane.n); 
			const FloatV _d = FLoad(plane.d);
			nd = V4SetW(Vec4V_From_Vec3V(_n), _d);
		}


		/**
		\brief Constructor from three points
		*/
		PX_FORCE_INLINE PlaneV(const Ps::aos::Vec3VArg p0, const Ps::aos::Vec3VArg p1, const Ps::aos::Vec3VArg p2)
		{
			set(p0, p1, p2);
		}

		/**
		\brief Constructor from a normal and a distance
		*/ 
		PX_FORCE_INLINE PlaneV(const Ps::aos::Vec3VArg _n, const Ps::aos::FloatVArg _d) 
		{
			nd = Ps::aos::V4SetW(Ps::aos::Vec4V_From_Vec3V(_n), _d);
		}

		/**
		\brief Copy constructor
		*/
		PX_FORCE_INLINE PlaneV(const PlaneV& plane) : nd(plane.nd)
		{
		}

		/**
		\brief Destructor
		*/
		PX_FORCE_INLINE ~PlaneV()
		{
		}

		/**
		\brief Sets plane to zero.
		*/
		PX_FORCE_INLINE PlaneV& setZero()
		{
			nd = Ps::aos::V4Zero();
			return *this;
		}

		PX_FORCE_INLINE PlaneV& set(const Ps::aos::FloatVArg nx, const Ps::aos::FloatVArg ny, const Ps::aos::FloatVArg nz, const Ps::aos::FloatVArg _d)
		{

			using namespace Ps::aos;
			const Vec3V n= V3Merge(nx, ny, nz);
			nd = V4SetW(Vec4V_From_Vec3V(n), _d);
			return *this;
		}

		PX_FORCE_INLINE PlaneV& set(const Ps::aos::Vec3VArg _normal, Ps::aos::FloatVArg _d)
		{
			nd = Ps::aos::V4SetW(Ps::aos::Vec4V_From_Vec3V(_normal), _d);
			return *this;
		}

		/**
		\brief Computes the plane equation from 3 points.
		*/
		PlaneV& set(const Ps::aos::Vec3VArg p0, const Ps::aos::Vec3VArg p1, const Ps::aos::Vec3VArg p2)
		{
			using namespace Ps::aos;
			const Vec3V edge0 = V3Sub(p1, p0);
			const Vec3V edge1 = V3Sub(p2, p0);

			const Vec3V n = V3Normalize(V3Cross(edge0, edge1));
			// See comments in set() for computation of d
			const FloatV d = FNeg(V3Dot(p0, n));
			nd = V4SetW(Vec4V_From_Vec3V(n), d);
			return	*this;
		}

		/***
		\brief Computes distance, assuming plane is normalized
		\sa normalize
		*/
		PX_FORCE_INLINE Ps::aos::FloatV distance(const Ps::aos::Vec3VArg p) const
		{
			// Valid for plane equation a*x + b*y + c*z + d = 0
			using namespace Ps::aos;
			const Vec3V n = Vec3V_From_Vec4V(nd);
			return FAdd(V3Dot(p, n), V4GetW(nd));
		}

		PX_FORCE_INLINE Ps::aos::BoolV belongs(const Ps::aos::Vec3VArg p) const
		{
			using namespace Ps::aos;
			const FloatV eps = FLoad(1.0e-7f);
			return FIsGrtr(eps, FAbs(distance(p)));
		}

		/**
		\brief projects p into the plane
		*/
		PX_FORCE_INLINE Ps::aos::Vec3V project(const Ps::aos::Vec3VArg p) const
		{
			// Pretend p is on positive side of plane, i.e. plane.distance(p)>0.
			// To project the point we have to go in a direction opposed to plane's normal, i.e.:
			using namespace Ps::aos;
			const Vec3V n = Vec3V_From_Vec4V(nd);
			return V3Sub(p, V3Scale(n, V4GetW(nd)));

		}

		PX_FORCE_INLINE Ps::aos::FloatV signedDistanceHessianNormalForm(const Ps::aos::Vec3VArg point) const
		{
			using namespace Ps::aos;
			const Vec3V n = Vec3V_From_Vec4V(nd);
			return FAdd(V3Dot(n, point), V4GetW(nd));
		}

		PX_FORCE_INLINE Ps::aos::Vec3V getNormal() const
		{
			return Ps::aos::Vec3V_From_Vec4V(nd);
		}

		PX_FORCE_INLINE Ps::aos::FloatV getSignDist() const
		{
			return Ps::aos::V4GetW(nd);
		}

		/**
		\brief find an arbitrary point in the plane
		*/
		PX_FORCE_INLINE Ps::aos::Vec3V pointInPlane() const
		{
			// Project origin (0,0,0) to plane:
			// (0) - normal * distance(0) = - normal * ((p|(0)) + d) = -normal*d
			using namespace Ps::aos;
			const Vec3V n = Vec3V_From_Vec4V(nd);
			return V3Neg(V3Scale(n, V4GetW(nd)));
		}

		PX_FORCE_INLINE void normalize()
		{
			using namespace Ps::aos;
			const Vec3V n = Vec3V_From_Vec4V(nd);
			const FloatV denom = FRecip(V3Length(n));
			V4Scale(nd, denom);
		}

		Ps::aos::Vec4V	nd;		//!< The normal to the plan , w store the distance from the origin
	};
}

}

/** @} */
#endif
