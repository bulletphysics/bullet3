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

#ifndef GU_CENTER_EXTENTS_H
#define GU_CENTER_EXTENTS_H

/** \addtogroup geomutils
@{
*/

#include "CmMatrix34.h"
#include "CmUtils.h"
#include "PsUserAllocated.h"

namespace physx
{
namespace Gu
{
	class CenterExtents : public physx::shdfnd::UserAllocated
	{
		public:
		PX_FORCE_INLINE				CenterExtents()						{}
		PX_FORCE_INLINE				CenterExtents(const PxBounds3& b)	{ mCenter = b.getCenter();	mExtents = b.getExtents();	}
		PX_FORCE_INLINE				~CenterExtents()					{}

		PX_FORCE_INLINE	void		getMin(PxVec3& min)		const		{ min = mCenter - mExtents;					}
		PX_FORCE_INLINE	void		getMax(PxVec3& max)		const		{ max = mCenter + mExtents;					}

		PX_FORCE_INLINE	float		getMin(PxU32 axis)		const		{ return mCenter[axis] - mExtents[axis];	}
		PX_FORCE_INLINE	float		getMax(PxU32 axis)		const		{ return mCenter[axis] + mExtents[axis];	}

		PX_FORCE_INLINE	PxVec3		getMin()				const		{ return mCenter - mExtents;				}
		PX_FORCE_INLINE	PxVec3		getMax()				const		{ return mCenter + mExtents;				}

		PX_FORCE_INLINE	void		setMinMax(const PxVec3& min, const PxVec3& max)
									{
										mCenter = (max + min)*0.5f;
										mExtents = (max - min)*0.5f;
									}

		PX_FORCE_INLINE	PxU32		isInside(const CenterExtents& box)	const
									{
										if(box.getMin(0)>getMin(0))	return 0;
										if(box.getMin(1)>getMin(1))	return 0;
										if(box.getMin(2)>getMin(2))	return 0;
										if(box.getMax(0)<getMax(0))	return 0;
										if(box.getMax(1)<getMax(1))	return 0;
										if(box.getMax(2)<getMax(2))	return 0;
										return 1;
									}

		PX_FORCE_INLINE	void		setEmpty()
									{
										mExtents = PxVec3(-PX_MAX_BOUNDS_EXTENTS);
									}

		PX_FORCE_INLINE	bool		isEmpty()	const
									{
										return Cm::isEmpty(mCenter, mExtents);
									}

		PX_FORCE_INLINE	bool		isFinite()	const
									{
										return mCenter.isFinite() && mExtents.isFinite();
									}

		PX_FORCE_INLINE	bool		isValid()	const
									{
										return Cm::isValid(mCenter, mExtents);
									}

		PX_FORCE_INLINE	PxBounds3	transformFast(const PxMat33& matrix)	const
									{
										PX_ASSERT(isValid());
										return PxBounds3::basisExtent(matrix * mCenter, matrix, mExtents);
									}

		PX_INLINE		PxBounds3	transformSafe(const Cm::Matrix34& matrix)	const
									{
										if(isEmpty())
											return PxBounds3::centerExtents(mCenter, mExtents);
										else
											return Cm::basisExtent(matrix.transform(mCenter), matrix.m.column0, matrix.m.column1, matrix.m.column2, mExtents);
									}

						PxVec3		mCenter;
						PxVec3		mExtents;
	};

	//! A padded version of CenterExtents, to safely load its data using SIMD
	class CenterExtentsPadded : public CenterExtents
	{
	public:
		PX_FORCE_INLINE CenterExtentsPadded()	{}
		PX_FORCE_INLINE ~CenterExtentsPadded()	{}
		PxU32	padding;
	};
	PX_COMPILE_TIME_ASSERT(sizeof(CenterExtentsPadded) == 7*4);

}

}

/** @} */
#endif
