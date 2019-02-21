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

#ifndef BIG_CONVEX_DATA_BUILDER_H
#define BIG_CONVEX_DATA_BUILDER_H

#include "foundation/PxMemory.h"
#include "PsVecMath.h"

namespace physx
{
	struct HullTriangleData;
	class BigConvexData;
	class ConvexHullBuilder;

	//////////////////////////////////////////////////////////////////////////
	//! Valencies creation structure
	struct ValenciesCreate
	{
		//! Constructor
								ValenciesCreate()	{ PxMemZero(this, sizeof(*this)); }

				PxU32			nbVerts;		//!< Number of vertices
				PxU32			nbFaces;		//!< Number of faces
		const	PxU32*			dFaces;			//!< List of faces (triangle list)
		const	PxU16*			wFaces;			//!< List of faces (triangle list)
				bool			adjacentList;	//!< Compute list of adjacent vertices or not
	};

	//////////////////////////////////////////////////////////////////////////

	class BigConvexDataBuilder : public Ps::UserAllocated
	{
		public:
									BigConvexDataBuilder(const Gu::ConvexHullData* hull, BigConvexData* gm, const PxVec3* hullVerts);
									~BigConvexDataBuilder();
	// Support vertex map
				bool				precompute(PxU32 subdiv);				

				bool				initialize();				

				bool				save(PxOutputStream& stream, bool platformMismatch)	const;

				bool				computeValencies(const ConvexHullBuilder& meshBuilder);
	//~Support vertex map

	// Valencies

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Computes valencies and adjacent vertices.
		 *	After the call, get results with the appropriate accessors.
		 *
		 *	\param		vc		[in] creation structure
		 *	\return		true if success.
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				bool				compute(const ValenciesCreate& vc)	const;

				bool				saveValencies(PxOutputStream& stream, bool platformMismatch)		const;
	//~Valencies
	protected:		
		PX_FORCE_INLINE void		precomputeSample(const PxVec3& dir, PxU8& startIndex, float negativeDir);

	private:
		const Gu::ConvexHullData*	mHull;
		BigConvexData*				mSVM;
		const	PxVec3*				mHullVerts;

	};

}

#endif // BIG_CONVEX_DATA_BUILDER_H
