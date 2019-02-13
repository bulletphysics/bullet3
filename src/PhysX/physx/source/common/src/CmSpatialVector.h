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


#ifndef PX_PHYSICS_COMMON_VECTOR
#define PX_PHYSICS_COMMON_VECTOR

#include "foundation/PxVec3.h"
#include "CmPhysXCommon.h"
#include "PsVecMath.h"
#include "foundation/PxTransform.h"

/*!
Combination of two R3 vectors.
*/

namespace physx
{
namespace Cm
{
PX_ALIGN_PREFIX(16)
class SpatialVector
{
public:
	//! Default constructor
	PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialVector()
	{}

	//! Construct from two PxcVectors
	PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialVector(const PxVec3& lin, const PxVec3& ang)
		: linear(lin), pad0(0.0f), angular(ang), pad1(0.0f)
	{
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE ~SpatialVector()
	{}



	// PT: this one is very important. Without it, the Xbox compiler generates weird "float-to-int" and "int-to-float" LHS
	// each time we copy a SpatialVector (see for example PIX on "solveSimpleGroupA" without this operator).
	PX_CUDA_CALLABLE PX_FORCE_INLINE	void	operator = (const SpatialVector& v)
	{
		linear = v.linear;
		pad0 = 0.0f;
		angular = v.angular;
		pad1 = 0.0f;
	}


	static PX_CUDA_CALLABLE  PX_FORCE_INLINE SpatialVector zero() {	return SpatialVector(PxVec3(0),PxVec3(0)); }

	PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialVector operator+(const SpatialVector& v) const
	{
		return SpatialVector(linear+v.linear,angular+v.angular);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialVector operator-(const SpatialVector& v) const
	{
		return SpatialVector(linear-v.linear,angular-v.angular);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialVector operator-() const
	{
		return SpatialVector(-linear,-angular);
	}


	PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialVector operator *(PxReal s) const
	{	
		return SpatialVector(linear*s,angular*s);	
	}
		
	PX_CUDA_CALLABLE PX_FORCE_INLINE void operator+=(const SpatialVector& v)
	{
		linear+=v.linear;
		angular+=v.angular;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE void operator-=(const SpatialVector& v)
	{
		linear-=v.linear;
		angular-=v.angular;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal magnitude()	const
	{
		return angular.magnitude() + linear.magnitude();
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal dot(const SpatialVector& v) const
	{
		return linear.dot(v.linear) + angular.dot(v.angular);
	}
		
	PX_CUDA_CALLABLE PX_FORCE_INLINE bool isFinite() const
	{
		return linear.isFinite() && angular.isFinite();
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE Cm::SpatialVector scale(PxReal l, PxReal a) const
	{
		return Cm::SpatialVector(linear*l, angular*a);
	}

	PxVec3 linear;
	PxReal pad0;
	PxVec3 angular;
	PxReal pad1;
}
PX_ALIGN_SUFFIX(16);

PX_ALIGN_PREFIX(16)
struct SpatialVectorF
{
public:
	//! Default constructor
	PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialVectorF()
	{}

	PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialVectorF(const PxReal* v)
		: pad0(0.0f), pad1(0.0f)
	{
		top.x = v[0]; top.y = v[1]; top.z = v[2];
		bottom.x = v[3]; bottom.y = v[4]; bottom.z = v[5];
	}
	//! Construct from two PxcVectors
	PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialVectorF(const PxVec3& top_, const PxVec3& bottom_)
		: top(top_), pad0(0.0f), bottom(bottom_), pad1(0.0f)
	{
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE ~SpatialVectorF()
	{}



	// PT: this one is very important. Without it, the Xbox compiler generates weird "float-to-int" and "int-to-float" LHS
	// each time we copy a SpatialVector (see for example PIX on "solveSimpleGroupA" without this operator).
	PX_CUDA_CALLABLE PX_FORCE_INLINE	void	operator = (const SpatialVectorF& v)
	{
		top = v.top;
		pad0 = 0.0f;
		bottom = v.bottom;
		pad1 = 0.0f;
	}


	static PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialVectorF Zero() { return SpatialVectorF(PxVec3(0), PxVec3(0)); }

	PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialVectorF operator+(const SpatialVectorF& v) const
	{
		return SpatialVectorF(top + v.top, bottom + v.bottom);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialVectorF operator-(const SpatialVectorF& v) const
	{
		return SpatialVectorF(top - v.top, bottom - v.bottom);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialVectorF operator-() const
	{
		return SpatialVectorF(-top, -bottom);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialVectorF operator *(PxReal s) const
	{
		return SpatialVectorF(top*s, bottom*s);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE void operator *= (const PxReal s)
	{
		top *= s;
		bottom *= s;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE void operator += (const SpatialVectorF& v)
	{
		top += v.top;
		bottom += v.bottom;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE void operator -= (const SpatialVectorF& v)
	{
		top -= v.top;
		bottom -= v.bottom;
	}


	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal magnitude()	const
	{
		return top.magnitude() + bottom.magnitude();
	}

	PX_FORCE_INLINE PxReal magnitudeSquared()	const
	{
		return top.magnitudeSquared() + bottom.magnitudeSquared();
	}

	//This is inner product 
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal innerProduct(const SpatialVectorF& v) const
	{
		return bottom.dot(v.top) + top.dot(v.bottom);
		/*PxVec3 p0 = bottom.multiply(v.top);
		PxVec3 p1 = top.multiply(v.bottom);

		PxReal result = (((p1.y + p1.z) + (p0.z + p1.x)) + (p0.x + p0.y));
		return result;*/
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal dot(const SpatialVectorF& v) const
	{
		return top.dot(v.top) + bottom.dot(v.bottom);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal dot(const SpatialVector& v) const
	{
		return bottom.dot(v.angular) + top.dot(v.linear);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialVectorF cross(const SpatialVectorF& v) const
	{
		SpatialVectorF a;
		a.top = top.cross(v.top);
		a.bottom = top.cross(v.bottom) + bottom.cross(v.top);
		return a;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialVectorF abs() const
	{
		return SpatialVectorF(top.abs(), bottom.abs());
	}


	PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialVectorF rotate(const PxTransform& rot) const
	{
		return SpatialVectorF(rot.rotate(top), rot.rotate(bottom));
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE SpatialVectorF rotateInv(const PxTransform& rot) const
	{
		return SpatialVectorF(rot.rotateInv(top), rot.rotateInv(bottom));
	}



	PX_CUDA_CALLABLE PX_FORCE_INLINE bool isFinite() const
	{
		return top.isFinite() && bottom.isFinite();
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE bool isValid(const PxReal maxV) const
	{
		const bool tValid = ((PxAbs(top.x) <= maxV) && (PxAbs(top.y) <= maxV) && (PxAbs(top.z) <= maxV));
		const bool bValid = ((PxAbs(bottom.x) <= maxV) && (PxAbs(bottom.y) <= maxV) && (PxAbs(bottom.z) <= maxV));

		return tValid && bValid;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE Cm::SpatialVectorF scale(PxReal l, PxReal a) const
	{
		return Cm::SpatialVectorF(top*l, bottom*a);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE void assignTo(PxReal* val) const
	{
		val[0] = top.x; val[1] = top.y; val[2] = top.z;
		val[3] = bottom.x; val[4] = bottom.y; val[5] = bottom.z;
	}

	PxVec3 top;
	PxReal pad0;
	PxVec3 bottom;
	PxReal pad1;
} PX_ALIGN_SUFFIX(16);


struct UnAlignedSpatialVector
{
public:
	//! Default constructor
	PX_CUDA_CALLABLE PX_FORCE_INLINE UnAlignedSpatialVector()
	{}

	PX_CUDA_CALLABLE PX_FORCE_INLINE UnAlignedSpatialVector(const PxReal* v)
	{
		top.x = v[0]; top.y = v[1]; top.z = v[2];
		bottom.x = v[3]; bottom.y = v[4]; bottom.z = v[5];
	}
	//! Construct from two PxcVectors
	PX_CUDA_CALLABLE PX_FORCE_INLINE UnAlignedSpatialVector(const PxVec3& top_, const PxVec3& bottom_)
		: top(top_), bottom(bottom_)
	{
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE ~UnAlignedSpatialVector()
	{}



	
	PX_CUDA_CALLABLE PX_FORCE_INLINE	void	operator = (const SpatialVectorF& v)
	{
		top = v.top;
		bottom = v.bottom;
	}


	static PX_CUDA_CALLABLE PX_FORCE_INLINE UnAlignedSpatialVector Zero() { return UnAlignedSpatialVector(PxVec3(0), PxVec3(0)); }

	PX_CUDA_CALLABLE PX_FORCE_INLINE UnAlignedSpatialVector operator+(const UnAlignedSpatialVector& v) const
	{
		return UnAlignedSpatialVector(top + v.top, bottom + v.bottom);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE UnAlignedSpatialVector operator-(const UnAlignedSpatialVector& v) const
	{
		return UnAlignedSpatialVector(top - v.top, bottom - v.bottom);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE UnAlignedSpatialVector operator-() const
	{
		return UnAlignedSpatialVector(-top, -bottom);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE UnAlignedSpatialVector operator *(PxReal s) const
	{
		return UnAlignedSpatialVector(top*s, bottom*s);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE void operator *= (const PxReal s)
	{
		top *= s;
		bottom *= s;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE void operator += (const UnAlignedSpatialVector& v)
	{
		top += v.top;
		bottom += v.bottom;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE void operator += (const SpatialVectorF& v)
	{
		top += v.top;
		bottom += v.bottom;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE void operator -= (const UnAlignedSpatialVector& v)
	{
		top -= v.top;
		bottom -= v.bottom;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE void operator -= (const SpatialVectorF& v)
	{
		top -= v.top;
		bottom -= v.bottom;
	}


	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal magnitude()	const
	{
		return top.magnitude() + bottom.magnitude();
	}

	PX_FORCE_INLINE PxReal magnitudeSquared()	const
	{
		return top.magnitudeSquared() + bottom.magnitudeSquared();
	}

	//This is inner product 
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal innerProduct(const UnAlignedSpatialVector& v) const
	{
		return bottom.dot(v.top) + top.dot(v.bottom);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal innerProduct(const SpatialVectorF& v) const
	{
		return bottom.dot(v.top) + top.dot(v.bottom);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal dot(const UnAlignedSpatialVector& v) const
	{
		return top.dot(v.top) + bottom.dot(v.bottom);
	}


	PX_CUDA_CALLABLE PX_FORCE_INLINE UnAlignedSpatialVector cross(const UnAlignedSpatialVector& v) const
	{
		UnAlignedSpatialVector a;
		a.top = top.cross(v.top);
		a.bottom = top.cross(v.bottom) + bottom.cross(v.top);
		return a;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE UnAlignedSpatialVector abs() const
	{
		return UnAlignedSpatialVector(top.abs(), bottom.abs());
	}


	PX_CUDA_CALLABLE PX_FORCE_INLINE UnAlignedSpatialVector rotate(const PxTransform& rot) const
	{
		return UnAlignedSpatialVector(rot.rotate(top), rot.rotate(bottom));
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE UnAlignedSpatialVector rotateInv(const PxTransform& rot) const
	{
		return UnAlignedSpatialVector(rot.rotateInv(top), rot.rotateInv(bottom));
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE bool isFinite() const
	{
		return top.isFinite() && bottom.isFinite();
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE bool isValid(const PxReal maxV) const
	{
		const bool tValid = ((top.x <= maxV) && (top.y <= maxV) && (top.z <= maxV));
		const bool bValid = ((bottom.x <= maxV) && (bottom.y <= maxV) && (bottom.z <= maxV));

		return tValid && bValid;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE Cm::UnAlignedSpatialVector scale(PxReal l, PxReal a) const
	{
		return Cm::UnAlignedSpatialVector(top*l, bottom*a);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE void assignTo(PxReal* val) const
	{
		val[0] = top.x; val[1] = top.y; val[2] = top.z;
		val[3] = bottom.x; val[4] = bottom.y; val[5] = bottom.z;
	}

	PxVec3 top;					//12		12
	PxVec3 bottom;				//12		24
};


PX_ALIGN_PREFIX(16)
struct SpatialVectorV
{
	Ps::aos::Vec3V linear;
	Ps::aos::Vec3V angular;

	PX_FORCE_INLINE SpatialVectorV() {}
	PX_FORCE_INLINE SpatialVectorV(PxZERO): linear(Ps::aos::V3Zero()), angular(Ps::aos::V3Zero()) {}
	PX_FORCE_INLINE SpatialVectorV(const Cm::SpatialVector& v): linear(Ps::aos::V3LoadU(v.linear)), angular(Ps::aos::V3LoadU(v.angular)) {}
	PX_FORCE_INLINE SpatialVectorV(const Ps::aos::Vec3VArg l, const Ps::aos::Vec3VArg a): linear(l), angular(a) {}
	PX_FORCE_INLINE SpatialVectorV(const SpatialVectorV& other): linear(other.linear), angular(other.angular) {}
	PX_FORCE_INLINE SpatialVectorV& operator=(const SpatialVectorV& other) { linear = other.linear; angular = other.angular; return *this; }

	PX_FORCE_INLINE SpatialVectorV operator+(const SpatialVectorV& other) const { return SpatialVectorV(Ps::aos::V3Add(linear,other.linear),
																								  Ps::aos::V3Add(angular, other.angular)); }
	
	PX_FORCE_INLINE SpatialVectorV& operator+=(const SpatialVectorV& other) { linear = Ps::aos::V3Add(linear,other.linear); 
																			  angular = Ps::aos::V3Add(angular, other.angular);
																			  return *this;
																			}
																								    
	PX_FORCE_INLINE SpatialVectorV operator-(const SpatialVectorV& other) const { return SpatialVectorV(Ps::aos::V3Sub(linear,other.linear),
																								  Ps::aos::V3Sub(angular, other.angular)); }
	
	PX_FORCE_INLINE SpatialVectorV operator-() const { return SpatialVectorV(Ps::aos::V3Neg(linear), Ps::aos::V3Neg(angular)); }

	PX_FORCE_INLINE SpatialVectorV operator*(const Ps::aos::FloatVArg r) const { return SpatialVectorV(Ps::aos::V3Scale(linear,r), Ps::aos::V3Scale(angular,r)); }

	PX_FORCE_INLINE SpatialVectorV& operator-=(const SpatialVectorV& other) { linear = Ps::aos::V3Sub(linear,other.linear); 
																			  angular = Ps::aos::V3Sub(angular, other.angular);
																			  return *this;
																			}

	PX_FORCE_INLINE Ps::aos::FloatV dot(const SpatialVectorV& other) const { return Ps::aos::FAdd(Ps::aos::V3Dot(linear, other.linear), Ps::aos::V3Dot(angular, other.angular)); }

	
}PX_ALIGN_SUFFIX(16);

} // namespace Cm

PX_COMPILE_TIME_ASSERT(sizeof(Cm::SpatialVector) == 32);
PX_COMPILE_TIME_ASSERT(sizeof(Cm::SpatialVectorV) == 32);

}

#endif
