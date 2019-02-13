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


#ifndef PXD_THRESHOLDTABLE_H
#define PXD_THRESHOLDTABLE_H
#include "Ps.h"
#include "PsArray.h"
#include "CmPhysXCommon.h"
#include "PsAllocator.h"
#include "PsHash.h"
#include "foundation/PxMemory.h"
#include "PxsIslandNodeIndex.h"

namespace physx
{

class PxsRigidBody;

namespace Sc
{
	class ShapeInteraction;
}

namespace Dy
{

struct ThresholdStreamElement
{
	Sc::ShapeInteraction*	shapeInteraction;			//4		8
	PxReal					normalForce;				//8		12
	PxReal					threshold;					//12	16
	IG::NodeIndex			nodeIndexA; //this is the unique node index in island gen which corresonding to that body and it is persistent	16	20
	IG::NodeIndex			nodeIndexB; //This is the unique node index in island gen which corresonding to that body and it is persistent	20	24
	PxReal					accumulatedForce;			//24	28
	PxU32					pad;						//28	32

#if !PX_P64_FAMILY
	PxU32					pad1;						//32
#endif // !PX_X64

	PX_CUDA_CALLABLE bool operator <= (const ThresholdStreamElement& otherPair) const
	{
		return ((nodeIndexA < otherPair.nodeIndexA) ||(nodeIndexA == otherPair.nodeIndexA && nodeIndexB <= otherPair.nodeIndexB));
	}

};

typedef Ps::Array<ThresholdStreamElement, Ps::VirtualAllocator> ThresholdArray;

class ThresholdStream : public ThresholdArray
{
public:
	ThresholdStream(Ps::VirtualAllocatorCallback& allocatorCallback) : ThresholdArray(Ps::VirtualAllocator(&allocatorCallback))
	{
	}

};

class ThresholdTable
{
public:

	ThresholdTable()
		:	mBuffer(NULL),
			mHash(NULL),
			mHashSize(0),
			mHashCapactiy(0),
			mPairs(NULL),
			mNexts(NULL),
			mPairsSize(0),
			mPairsCapacity(0)
	{
	}

	~ThresholdTable()
	{
		if(mBuffer) PX_FREE(mBuffer);
	}

	void build(const ThresholdStream& stream);

	bool check(const ThresholdStream& stream, const PxU32 nodexIndexA, const PxU32 nodexIndexB, PxReal dt);

	bool check(const ThresholdStream& stream, const ThresholdStreamElement& elem, PxU32& thresholdIndex);

//private:

	static const PxU32 NO_INDEX = 0xffffffff;

	struct Pair 
	{	
		PxU32			thresholdStreamIndex;
		PxReal			accumulatedForce;
		//PxU32			next;		// hash key & next ptr
	};

	PxU8*					mBuffer;

	PxU32*					mHash;
	PxU32					mHashSize;
	PxU32					mHashCapactiy;

	Pair*					mPairs;
	PxU32*					mNexts;
	PxU32					mPairsSize;
	PxU32					mPairsCapacity;
};

namespace
{
	static PX_FORCE_INLINE PxU32 computeHashKey(const PxU32 nodeIndexA, const PxU32 nodeIndexB, const PxU32 hashCapacity)
	{
		return (Ps::hash(PxU64(nodeIndexA)<<32 | PxU64(nodeIndexB)) % hashCapacity);
	}
}

inline bool ThresholdTable::check(const ThresholdStream& stream, const ThresholdStreamElement& elem, PxU32& thresholdIndex)
{
	PxU32* PX_RESTRICT hashes = mHash;
	PxU32* PX_RESTRICT nextIndices = mNexts;
	Pair* PX_RESTRICT pairs = mPairs;

	PX_ASSERT(elem.nodeIndexA < elem.nodeIndexB);
	PxU32 hashKey = computeHashKey(elem.nodeIndexA.index(), elem.nodeIndexB.index(), mHashSize);

	PxU32 pairIndex = hashes[hashKey];

	while(NO_INDEX != pairIndex)
	{
		Pair& pair = pairs[pairIndex];
		const PxU32 thresholdStreamIndex = pair.thresholdStreamIndex;
		PX_ASSERT(thresholdStreamIndex < stream.size());
		const ThresholdStreamElement& otherElement = stream[thresholdStreamIndex];
		if(otherElement.nodeIndexA==elem.nodeIndexA  && otherElement.nodeIndexB==elem.nodeIndexB && otherElement.shapeInteraction == elem.shapeInteraction)
		{
			thresholdIndex = thresholdStreamIndex;
			return true;
		}
		pairIndex = nextIndices[pairIndex];
	}

	thresholdIndex = NO_INDEX;
	return false;
}


inline void ThresholdTable::build(const ThresholdStream& stream)
{
	//Handle the case of an empty stream.
	if(0==stream.size())
	{
		mPairsSize=0;
		mPairsCapacity=0;
		mHashSize=0;
		mHashCapactiy=0;
		if(mBuffer) PX_FREE(mBuffer);
		mBuffer = NULL;
		return;
	}

	//Realloc/resize if necessary.
	const PxU32 pairsCapacity = stream.size();
	const PxU32 hashCapacity = pairsCapacity*2+1;
	if((pairsCapacity > mPairsCapacity) || (pairsCapacity < (mPairsCapacity >> 2)))
	{
		if(mBuffer) PX_FREE(mBuffer);
		const PxU32 pairsByteSize = sizeof(Pair)*pairsCapacity;
		const PxU32 nextsByteSize = sizeof(PxU32)*pairsCapacity;
		const PxU32 hashByteSize = sizeof(PxU32)*hashCapacity;
		const PxU32 totalByteSize = pairsByteSize + nextsByteSize + hashByteSize;
		mBuffer = reinterpret_cast<PxU8*>(PX_ALLOC(totalByteSize, "PxThresholdStream"));

		PxU32 offset = 0;
		mPairs = reinterpret_cast<Pair*>(mBuffer + offset);
		offset += pairsByteSize;
		mNexts = reinterpret_cast<PxU32*>(mBuffer + offset);
		offset += nextsByteSize;
		mHash = reinterpret_cast<PxU32*>(mBuffer + offset);
		offset += hashByteSize;
		PX_ASSERT(totalByteSize == offset);

		mPairsCapacity = pairsCapacity;
		mHashCapactiy = hashCapacity;
	}


	//Set each entry of the hash table to 0xffffffff
	PxMemSet(mHash, 0xff, sizeof(PxU32)*hashCapacity);

	//Init the sizes of the pairs array and hash array.
	mPairsSize = 0;
	mHashSize = hashCapacity;

	PxU32* PX_RESTRICT hashes = mHash;
	PxU32* PX_RESTRICT nextIndices = mNexts;
	Pair* PX_RESTRICT pairs = mPairs;

	//Add all the pairs from the stream.
	PxU32 pairsSize = 0;
	for(PxU32 i = 0; i < pairsCapacity; i++)
	{
		const ThresholdStreamElement& element = stream[i];
		const IG::NodeIndex nodeIndexA = element.nodeIndexA;
		const IG::NodeIndex nodeIndexB = element.nodeIndexB;

		const PxF32 force = element.normalForce;
				
		PX_ASSERT(nodeIndexA < nodeIndexB);

		const PxU32 hashKey = computeHashKey(nodeIndexA.index(), nodeIndexB.index(), hashCapacity);

		//Get the index of the first pair found that resulted in a hash that matched hashKey.
		PxU32 prevPairIndex = hashKey;
		PxU32 pairIndex = hashes[hashKey];

		//Search through all pairs found that resulted in a hash that matched hashKey.
		//Search until the exact same body pair is found.
		//Increment the accumulated force if the exact same body pair is found.
		while(NO_INDEX != pairIndex)
		{
			Pair& pair = pairs[pairIndex];
			const PxU32 thresholdStreamIndex = pair.thresholdStreamIndex;
			PX_ASSERT(thresholdStreamIndex < stream.size());
			const ThresholdStreamElement& otherElement = stream[thresholdStreamIndex];
			if(nodeIndexA == otherElement.nodeIndexA && nodeIndexB==otherElement.nodeIndexB)
			{	
				pair.accumulatedForce += force;
				prevPairIndex = NO_INDEX;
				pairIndex = NO_INDEX;
				break;
			}
			prevPairIndex = pairIndex;
			pairIndex = nextIndices[pairIndex];
		}

		if(NO_INDEX != prevPairIndex)
		{
			nextIndices[pairsSize] = hashes[hashKey];
			hashes[hashKey] = pairsSize;
			Pair& newPair = pairs[pairsSize];
			newPair.thresholdStreamIndex = i;
			newPair.accumulatedForce = force;
			pairsSize++;
		}
	}
	mPairsSize = pairsSize;
}

}

}

#endif //DY_THRESHOLDTABLE_H
