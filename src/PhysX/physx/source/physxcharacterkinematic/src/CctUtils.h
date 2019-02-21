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


#ifndef CCT_UTILS
#define CCT_UTILS

#include "PxExtended.h"
#include "PxShapeExt.h"

namespace physx
{

PX_FORCE_INLINE	bool testSlope(const PxVec3& normal, const PxVec3& upDirection, PxF32 slopeLimit)
{
	const float dp = normal.dot(upDirection);
	return dp>=0.0f && dp<slopeLimit;
}

PX_INLINE PxTransform getShapeGlobalPose(const PxShape& shape, const PxRigidActor& actor)
{
	return PxShapeExt::getGlobalPose(shape, actor);
}

#ifdef PX_BIG_WORLDS

	class PxExtendedBox
	{
	public:
		PX_INLINE	PxExtendedBox()	{}
		PX_INLINE	PxExtendedBox(const PxExtendedVec3& _center, const PxVec3& _extents, const PxQuat& _rot) : center(_center), extents(_extents), rot(_rot){}
		PX_INLINE	~PxExtendedBox()	{}

		PxExtendedVec3	center;
		PxVec3			extents;
		PxQuat			rot;
	};

	class PxExtendedSphere
	{
	public:
		PX_INLINE PxExtendedSphere()																				{}
		PX_INLINE ~PxExtendedSphere()																				{}
		PX_INLINE PxExtendedSphere(const PxExtendedVec3& _center, PxF32 _radius) : center(_center), radius(_radius)	{}
		PX_INLINE PxExtendedSphere(const PxExtendedSphere& sphere) : center(sphere.center), radius(sphere.radius)	{}

		PxExtendedVec3	center;		//!< Sphere's center
		PxF32			radius;		//!< Sphere's radius
	};

	struct PxExtendedSegment
	{
		PX_INLINE const PxExtendedVec3& getOrigin() const
		{
			return p0;
		}

		PX_INLINE void computeDirection(PxVec3& dir) const
		{
			dir = p1 - p0;
		}

		PX_INLINE void computePoint(PxExtendedVec3& pt, PxExtended t) const
		{
			pt.x = p0.x + t * (p1.x - p0.x);
			pt.y = p0.y + t * (p1.y - p0.y);
			pt.z = p0.z + t * (p1.z - p0.z);
		}

		PxExtendedVec3	p0;		//!< Start of segment
		PxExtendedVec3	p1;		//!< End of segment
	};

	struct PxExtendedCapsule : public PxExtendedSegment
	{
		PxReal	radius;
	};

	struct PxExtendedBounds3
	{
		PX_INLINE PxExtendedBounds3()
		{
		}

		PX_INLINE void setEmpty()
		{
			// We now use this particular pattern for empty boxes
			set(PX_MAX_EXTENDED, PX_MAX_EXTENDED, PX_MAX_EXTENDED,
				-PX_MAX_EXTENDED, -PX_MAX_EXTENDED, -PX_MAX_EXTENDED);
		}

		PX_INLINE void	set(PxExtended minx, PxExtended miny, PxExtended minz, PxExtended maxx, PxExtended maxy, PxExtended maxz)
		{
			minimum.set(minx, miny, minz);
			maximum.set(maxx, maxy, maxz);
		}

		PX_INLINE bool	isInside(const PxExtendedBounds3& box) const
		{
			if(box.minimum.x > minimum.x)	return false;
			if(box.minimum.y > minimum.y)	return false;
			if(box.minimum.z > minimum.z)	return false;
			if(box.maximum.x < maximum.x)	return false;
			if(box.maximum.y < maximum.y)	return false;
			if(box.maximum.z < maximum.z)	return false;
			return true;
		}
		PxExtendedVec3 minimum, maximum;
	};

	PX_INLINE void	getCenter(const PxExtendedBounds3& b, PxExtendedVec3& center)
	{
		center = b.minimum + b.maximum;
		center *= 0.5;
	}

	PX_INLINE void	getExtents(const PxExtendedBounds3& b, PxVec3& extents)
	{
		extents = b.maximum - b.minimum;
		extents *= 0.5f;
	}

	PX_INLINE void	setCenterExtents(PxExtendedBounds3& b, const PxExtendedVec3& c, const PxVec3& e)
	{
		b.minimum = c;	b.minimum -= e;
		b.maximum = c;	b.maximum += e;
	}

	PX_INLINE void	add(PxExtendedBounds3& b, const PxExtendedBounds3& b2)
	{
		// - if we're empty, minimum = MAX,MAX,MAX => minimum will be b2 in all cases => it will copy b2, ok
		// - if b2 is empty, the opposite happens => keep us unchanged => ok
		// => same behaviour as before, automatically
		b.minimum.minimum(b2.minimum);
		b.maximum.maximum(b2.maximum);
	}
#else
	
	#include "foundation/PxBounds3.h"
	#include "GuBox.h"
	#include "GuCapsule.h"
	#include "GuPlane.h"

	typedef	Gu::Box		PxExtendedBox;
	typedef	Gu::Sphere	PxExtendedSphere;
	typedef Gu::Segment	PxExtendedSegment;
	typedef Gu::Capsule	PxExtendedCapsule;
	typedef	PxBounds3	PxExtendedBounds3;

	PX_INLINE PxExtended	distance(const PxVec3& v2, const PxVec3& v)
	{
		const PxExtended dx = v2.x - v.x;
		const PxExtended dy = v2.y - v.y;
		const PxExtended dz = v2.z - v.z;
		return PxSqrt(dx * dx + dy * dy + dz * dz);
	}

	PX_INLINE void	getCenter(const PxBounds3& b, PxVec3& center)
	{
		center = b.minimum + b.maximum;
		center *= 0.5;
	}

	PX_INLINE void	getExtents(const PxBounds3& b, PxVec3& extents)
	{
		extents = b.maximum - b.minimum;
		extents *= 0.5f;
	}

	PX_INLINE void	setCenterExtents(PxBounds3& b, const PxVec3& center, const PxVec3& extents)
	{
		b.minimum	= center - extents;
		b.maximum	= center + extents;
	}

	PX_INLINE void	add(PxBounds3& b, const PxBounds3& b2)
	{
		// - if we're empty, minimum = MAX,MAX,MAX => minimum will be b2 in all cases => it will copy b2, ok
		// - if b2 is empty, the opposite happens => keep us unchanged => ok
		// => same behaviour as before, automatically
		b.minimum.minimum(b2.minimum);
		b.maximum.maximum(b2.maximum);
	}
#endif

}

#endif
