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

#ifndef GU_VEC_CONVEX_H
#define GU_VEC_CONVEX_H

#include "CmPhysXCommon.h"
#include "PsVecMath.h"

#define PX_SUPPORT_INLINE PX_FORCE_INLINE
#define PX_SUPPORT_FORCE_INLINE PX_FORCE_INLINE

namespace physx
{
namespace Gu
{

	struct ConvexType
	{
		enum Type
		{
			eCONVEXHULL = 0,
			eCONVEXHULLNOSCALE = 1,
			eSPHERE = 2,
			eBOX = 3,
			eCAPSULE = 4,
			eTRIANGLE = 5
		};
	};
	
	class ConvexV
	{
	public:

		PX_FORCE_INLINE ConvexV(const ConvexType::Type type_) : type(type_), bMarginIsRadius(false)
		{
			margin = 0.f;
			minMargin = 0.f;
			sweepMargin = 0.f;
			center = Ps::aos::V3Zero();
		}

		PX_FORCE_INLINE ConvexV(const ConvexType::Type type_, const Ps::aos::Vec3VArg center_) : type(type_), bMarginIsRadius(false)
		{
			using namespace Ps::aos;
			center = center_;
			margin = 0.f;
			minMargin = 0.f;
			sweepMargin = 0.f;
		}

		//everytime when someone transform the object, they need to up
		PX_FORCE_INLINE void setCenter(const Ps::aos::Vec3VArg _center)
		{
			center = _center;
		}

		PX_FORCE_INLINE void setMargin(const Ps::aos::FloatVArg margin_)
		{
			Ps::aos::FStore(margin_, &margin);
		}

		PX_FORCE_INLINE void setMargin(const PxReal margin_)
		{
			margin = margin_;
		}


		PX_FORCE_INLINE void setMinMargin(const Ps::aos::FloatVArg minMargin_)
		{
			Ps::aos::FStore(minMargin_, & minMargin);
		}

		PX_FORCE_INLINE void setSweepMargin(const Ps::aos::FloatVArg sweepMargin_)
		{
			Ps::aos::FStore(sweepMargin_, &sweepMargin);
		}

		PX_FORCE_INLINE Ps::aos::Vec3V getCenter()const 
		{
			return center;
		}

		PX_FORCE_INLINE Ps::aos::FloatV getMargin() const
		{
			return Ps::aos::FLoad(margin);
		}

		PX_FORCE_INLINE Ps::aos::FloatV getMinMargin() const
		{
			return Ps::aos::FLoad(minMargin);
		}

		PX_FORCE_INLINE Ps::aos::FloatV getSweepMargin() const
		{
			return Ps::aos::FLoad(sweepMargin);
		}

		PX_FORCE_INLINE ConvexType::Type getType() const
		{
			return type;
		}

		PX_FORCE_INLINE Ps::aos::BoolV isMarginEqRadius()const
		{
			return Ps::aos::BLoad(bMarginIsRadius);
		}

		PX_FORCE_INLINE bool getMarginIsRadius() const
		{
			return bMarginIsRadius;
		}

		PX_FORCE_INLINE PxReal getMarginF() const
		{
			return margin;
		}


	protected:
		~ConvexV(){}
		Ps::aos::Vec3V center;
		PxReal margin;				//margin is the amount by which we shrunk the shape for a convex or box. If the shape are sphere/capsule, margin is the radius
		PxReal minMargin;			//minMargin is some percentage of marginBase, which is used to determine the termination condition for gjk
		PxReal sweepMargin;			//sweepMargin minMargin is some percentage of marginBase, which is used to determine the termination condition for gjkRaycast
		ConvexType::Type	type;
		bool bMarginIsRadius;
	};

	PX_FORCE_INLINE Ps::aos::FloatV getContactEps(const Ps::aos::FloatV& _marginA, const Ps::aos::FloatV& _marginB)
	{
		using namespace Ps::aos;

		const FloatV ratio = FLoad(0.25f);
		const FloatV minMargin = FMin(_marginA, _marginB);
		
		return FMul(minMargin, ratio);
	}

	PX_FORCE_INLINE Ps::aos::FloatV getSweepContactEps(const Ps::aos::FloatV& _marginA, const Ps::aos::FloatV& _marginB)
	{
		using namespace Ps::aos;

		const FloatV ratio = FLoad(100.f);
		const FloatV minMargin = FAdd(_marginA, _marginB);
		
		return FMul(minMargin, ratio);
	}
}

}

#endif
