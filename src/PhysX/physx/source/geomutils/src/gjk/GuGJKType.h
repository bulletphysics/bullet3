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

#ifndef GU_GJKTYPE_H
#define GU_GJKTYPE_H

#include "GuVecConvex.h"
#include "PsVecTransform.h"

namespace physx
{
namespace Gu
{
	class ConvexHullV;
	class ConvexHullNoScaleV;
	class BoxV;

	template <typename Convex> struct ConvexGeom { typedef Convex Type; };
	template <> struct ConvexGeom<ConvexHullV> { typedef ConvexHullV Type; };
	template <> struct ConvexGeom<ConvexHullNoScaleV> { typedef ConvexHullNoScaleV Type; };
	template <> struct ConvexGeom<BoxV> { typedef BoxV Type; };

	struct GjkConvexBase
	{
		
		GjkConvexBase(const ConvexV& convex) : mConvex(convex){}
		PX_FORCE_INLINE Ps::aos::FloatV	getMinMargin()		const { return mConvex.getMinMargin();		} 
		PX_FORCE_INLINE Ps::aos::BoolV	isMarginEqRadius()	const { return mConvex.isMarginEqRadius();	}
		PX_FORCE_INLINE bool			getMarginIsRadius()	const { return mConvex.getMarginIsRadius();	}
		PX_FORCE_INLINE Ps::aos::FloatV	getMargin()			const { return mConvex.getMargin();			}
		

		template <typename Convex>
		PX_FORCE_INLINE const Convex& getConvex() const { return static_cast<const Convex&>(mConvex); }

		virtual Ps::aos::Vec3V supportPoint(const PxI32 index) const = 0;
		virtual Ps::aos::Vec3V support(const Ps::aos::Vec3VArg v) const = 0;
		virtual Ps::aos::Vec3V support(const Ps::aos::Vec3VArg dir, PxI32& index) const = 0;
		virtual Ps::aos::FloatV getSweepMargin() const = 0;
		virtual Ps::aos::Vec3V	getCenter() const = 0;
		virtual ~GjkConvexBase(){}
	
	
	private:
		GjkConvexBase& operator = (const GjkConvexBase&);
	protected:
		const ConvexV& mConvex;
	};

	struct GjkConvex : public GjkConvexBase
	{
		GjkConvex(const ConvexV& convex) : GjkConvexBase(convex) { }

		virtual Ps::aos::Vec3V supportPoint(const PxI32 index) const { return doVirtualSupportPoint(index); }
		virtual Ps::aos::Vec3V support(const Ps::aos::Vec3VArg v) const { return doVirtualSupport(v); }
		virtual Ps::aos::Vec3V support(const Ps::aos::Vec3VArg dir, PxI32& index) const{ return doVirtualSupport(dir, index); }

		virtual Ps::aos::FloatV getSweepMargin() const { return doVirtualGetSweepMargin(); }

	private:
		Ps::aos::Vec3V doVirtualSupportPoint(const PxI32 index) const
		{
			return supportPoint(index); //Call the v-table
		}
		Ps::aos::Vec3V doVirtualSupport(const Ps::aos::Vec3VArg v) const
		{
			return support(v); //Call the v-table
		}
		Ps::aos::Vec3V doVirtualSupport(const Ps::aos::Vec3VArg dir, PxI32& index) const
		{
			return support(dir, index); //Call the v-table
		}

		Ps::aos::FloatV doVirtualGetSweepMargin() const { return getSweepMargin(); }

		GjkConvex& operator = (const GjkConvex&);
	};

	template <typename Convex>
	struct LocalConvex : public GjkConvex
	{
		LocalConvex(const Convex& convex) : GjkConvex(convex){}


		PX_FORCE_INLINE Ps::aos::Vec3V supportPoint(const PxI32 index) const { return getConvex<Convex>().supportPoint(index); }
		Ps::aos::Vec3V support(const Ps::aos::Vec3VArg v) const { return getConvex<Convex>().supportLocal(v); }
		Ps::aos::Vec3V support(const Ps::aos::Vec3VArg dir, PxI32& index) const
		{
			return getConvex<Convex>().supportLocal(dir, index);
		}

		virtual Ps::aos::Vec3V getCenter() const { return getConvex<Convex>().getCenter(); }

		//ML: we can't force inline function, otherwise win modern will throw compiler error
		PX_INLINE LocalConvex<typename ConvexGeom<Convex>::Type > getGjkConvex() const
		{
			return LocalConvex<typename ConvexGeom<Convex>::Type >(static_cast<const typename ConvexGeom<Convex>::Type&>(GjkConvex::mConvex));
		}

		PX_INLINE Ps::aos::FloatV getSweepMargin() const { return getConvex<Convex>().getSweepMargin(); }

		typedef LocalConvex<typename ConvexGeom<Convex>::Type > ConvexGeomType;

		typedef Convex	Type;


	private:
		LocalConvex<Convex>& operator = (const LocalConvex<Convex>&);
	};

	template <typename Convex>
	struct RelativeConvex : public GjkConvex
	{
		RelativeConvex(const Convex& convex, const Ps::aos::PsMatTransformV& aToB) : GjkConvex(convex), mAToB(aToB), mAToBTransposed(aToB)
		{
			shdfnd::aos::V3Transpose(mAToBTransposed.rot.col0, mAToBTransposed.rot.col1, mAToBTransposed.rot.col2);
		}

		PX_FORCE_INLINE Ps::aos::Vec3V supportPoint(const PxI32 index) const { return mAToB.transform(getConvex<Convex>().supportPoint(index)); }
		Ps::aos::Vec3V support(const Ps::aos::Vec3VArg v) const { return getConvex<Convex>().supportRelative(v, mAToB, mAToBTransposed); }
		Ps::aos::Vec3V support(const Ps::aos::Vec3VArg dir, PxI32& index) const
		{
			return getConvex<Convex>().supportRelative(dir, mAToB, mAToBTransposed, index);
		}

		virtual Ps::aos::Vec3V getCenter() const { return mAToB.transform(getConvex<Convex>().getCenter()); }

		PX_FORCE_INLINE Ps::aos::PsMatTransformV& getRelativeTransform(){ return mAToB; }

		//ML: we can't force inline function, otherwise win modern will throw compiler error
		PX_INLINE RelativeConvex<typename ConvexGeom<Convex>::Type > getGjkConvex() const
		{
			return RelativeConvex<typename ConvexGeom<Convex>::Type >(static_cast<const typename ConvexGeom<Convex>::Type&>(GjkConvex::mConvex), mAToB);
		}

		PX_INLINE Ps::aos::FloatV getSweepMargin() const { return getConvex<Convex>().getSweepMargin(); }

		typedef RelativeConvex<typename ConvexGeom<Convex>::Type > ConvexGeomType;

		typedef Convex	Type;

	private:
		RelativeConvex<Convex>& operator = (const RelativeConvex<Convex>&);
		const Ps::aos::PsMatTransformV& mAToB;
		Ps::aos::PsMatTransformV mAToBTransposed;	// PT: precomputed mAToB transpose (because 'rotate' is faster than 'rotateInv')
	};

}
}

#endif
