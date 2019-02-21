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

#ifndef GU_TRIANGLE_CACHE_H
#define GU_TRIANGLE_CACHE_H
#include "PsHash.h"
#include "PsUtilities.h"

namespace physx
{
	namespace Gu
	{
		struct CachedEdge
		{
		protected:
			PxU32 mId0, mId1;
		public:
			CachedEdge(PxU32 i0, PxU32 i1)
			{
				mId0 = PxMin(i0, i1);
				mId1 = PxMax(i0, i1);
			}

			CachedEdge()
			{
			}

			PxU32 getId0() const { return mId0; }
			PxU32 getId1() const { return mId1; }

			bool operator == (const CachedEdge& other) const
			{
				return mId0 == other.mId0 && mId1 == other.mId1;
			}

			PxU32 getHashCode() const
			{
				return Ps::hash(mId0 << 16 | mId1);
			}
		};

		struct CachedVertex
		{
		private:
			PxU32 mId;
		public:
			CachedVertex(PxU32 id)
			{
				mId = id;
			}

			CachedVertex()
			{
			}

			PxU32 getId() const { return mId; }

			PxU32 getHashCode() const
			{
				return mId;
			}

			bool operator == (const CachedVertex& other) const 
			{
				return mId == other.mId;
			}
		};

		template <typename Elem, PxU32 MaxCount>
		struct CacheMap
		{
			PX_COMPILE_TIME_ASSERT(MaxCount < 0xFF);
			Elem mCache[MaxCount];
			PxU8 mNextInd[MaxCount];
			PxU8 mIndex[MaxCount];
			PxU32 mSize;

			CacheMap() : mSize(0)
			{
				for(PxU32 a = 0; a < MaxCount; ++a)
				{
					mIndex[a] = 0xFF;
				}
			}

			bool addData(const Elem& data)
			{
				if(mSize == MaxCount)
					return false;

				const PxU8 hash = PxU8(data.getHashCode() % MaxCount);

				PxU8 index = hash;
				PxU8 nextInd = mIndex[hash];
				while(nextInd != 0xFF)
				{
					index = nextInd;
					if(mCache[index] == data)
						return false;
					nextInd = mNextInd[nextInd];
				}

				if(mIndex[hash] == 0xFF)
				{
					mIndex[hash] = Ps::to8(mSize);
				}
				else
				{
					mNextInd[index] = Ps::to8(mSize);
				}
				mNextInd[mSize] = 0xFF;
				mCache[mSize++] = data;
				return true;
			}

			bool contains(const Elem& data) const
			{
				PxU32 hash = (data.getHashCode() % MaxCount);
				PxU8 index = mIndex[hash];

				while(index != 0xFF)
				{
					if(mCache[index] == data)
						return true;
					index = mNextInd[index];
				}
				return false;
			}

			const Elem* get(const Elem& data) const
			{
				PxU32 hash = (data.getHashCode() % MaxCount);
				PxU8 index = mIndex[hash];

				while(index != 0xFF)
				{
					if(mCache[index] == data)
						return &mCache[index];
					index = mNextInd[index];
				}
				return NULL;
			}
		};

		template <PxU32 MaxTriangles>
		struct TriangleCache
		{
			PxVec3 mVertices[3*MaxTriangles];
			PxU32 mIndices[3*MaxTriangles];
			PxU32 mTriangleIndex[MaxTriangles];
			PxU8 mEdgeFlags[MaxTriangles];
			PxU32 mNumTriangles;

			TriangleCache() : mNumTriangles(0)
			{
			}

			PX_FORCE_INLINE bool isEmpty() const { return mNumTriangles == 0; }
			PX_FORCE_INLINE bool isFull() const { return mNumTriangles == MaxTriangles; }
			PX_FORCE_INLINE void reset() { mNumTriangles = 0; }

			void addTriangle(const PxVec3* verts, const PxU32* indices, PxU32 triangleIndex, PxU8 edgeFlag)
			{
				PX_ASSERT(mNumTriangles < MaxTriangles);
				PxU32 triInd = mNumTriangles++;
				PxU32 triIndMul3 = triInd*3;
				mVertices[triIndMul3] = verts[0];
				mVertices[triIndMul3+1] = verts[1];
				mVertices[triIndMul3+2] = verts[2];
				mIndices[triIndMul3] = indices[0];
				mIndices[triIndMul3+1] = indices[1];
				mIndices[triIndMul3+2] = indices[2];
				mTriangleIndex[triInd] = triangleIndex;
				mEdgeFlags[triInd] = edgeFlag;
			}
		};
	}
}

#endif

