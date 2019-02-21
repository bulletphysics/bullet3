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

#ifndef GU_SHAPECONVEX_H
#define GU_SHAPECONVEX_H

#include "GuConvexMeshData.h"
#include "CmScaling.h"

namespace physx
{
namespace Gu
{
	struct PolygonalData;
	typedef void	(*HullPrefetchCB)		(PxU32 numVerts, const PxVec3* PX_RESTRICT verts);
	typedef void	(*HullProjectionCB)		(const PolygonalData& data, const PxVec3& dir, const Cm::Matrix34& world2hull, const Cm::FastVertex2ShapeScaling& scaling, PxReal& minimum, PxReal& maximum);
	typedef PxU32	(*SelectClosestEdgeCB)	(const PolygonalData& data, const Cm::FastVertex2ShapeScaling& scaling, const PxVec3& localDirection);

	struct PolygonalData
	{
		// Data
		PxVec3								mCenter;
		PxU32								mNbVerts;
		PxU32								mNbPolygons;
		PxU32								mNbEdges;
		const Gu::HullPolygonData*			mPolygons;
		const PxVec3*						mVerts;
		const PxU8*							mPolygonVertexRefs;
		const PxU8*							mFacesByEdges;
		const PxU16*						mVerticesByEdges;

		Gu::InternalObjectsData				mInternal;
		union
		{
			const Gu::BigConvexRawData*		mBigData;	// Only for big convexes
			const PxVec3*					mHalfSide;	// Only for boxes
		};

		// Code
		HullProjectionCB					mProjectHull;
		SelectClosestEdgeCB					mSelectClosestEdgeCB;

		PX_FORCE_INLINE const PxU8*	getPolygonVertexRefs(const Gu::HullPolygonData& poly)	const
		{
			return mPolygonVertexRefs + poly.mVRef8;
		}
	};

#if PX_VC 
    #pragma warning(push)
	#pragma warning( disable : 4251 ) // class needs to have dll-interface to be used by clients of class
#endif
	class PX_PHYSX_COMMON_API PolygonalBox
	{
	public:
									PolygonalBox(const PxVec3& halfSide);

			void					getPolygonalData(PolygonalData* PX_RESTRICT dst)	const;

			const PxVec3&			mHalfSide;
			PxVec3					mVertices[8];
			Gu::HullPolygonData		mPolygons[6];
	private:
			PolygonalBox& operator=(const PolygonalBox&);
	};
#if PX_VC 
     #pragma warning(pop) 
#endif

	void getPolygonalData_Convex(PolygonalData* PX_RESTRICT dst, const Gu::ConvexHullData* PX_RESTRICT src, const Cm::FastVertex2ShapeScaling& scaling);
}
}

#endif
