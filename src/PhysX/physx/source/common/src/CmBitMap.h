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


#ifndef PX_PHYSICS_COMMON_BITMAP
#define PX_PHYSICS_COMMON_BITMAP

#include "foundation/PxAssert.h"
#include "foundation/PxMath.h"
#include "foundation/PxMemory.h"
#include "PsAllocator.h"
#include "PsUserAllocated.h"
#include "PsIntrinsics.h"
#include "PsMathUtils.h"
#include "CmPhysXCommon.h"
#include "PsBitUtils.h"
// PX_SERIALIZATION
#include "PxSerialFramework.h"
//~PX_SERIALIZATION

namespace physx
{
namespace Cm
{

	/*!
	Hold a bitmap with operations to set,reset or test given bit.

	We inhibit copy to prevent unintentional copies. If a copy is desired copy() should be used or
	alternatively a copy constructor implemented.
	*/
	template<class Allocator>
	class BitMapBase : public Ps::UserAllocated
	{
	//= ATTENTION! =====================================================================================
	// Changing the data layout of this class breaks the binary serialization format.  See comments for 
	// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
	// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
	// accordingly.
	//==================================================================================================

		PX_NOCOPY(BitMapBase)

	public:

		// PX_SERIALIZATION
		/* todo: explicit */ BitMapBase(const PxEMPTY) 
		{
			if(mMap)
				mWordCount |= PX_SIGN_BITMASK;
		}

		void	exportExtraData(PxSerializationContext& stream, void*)	
		{
			if(mMap && getWordCount())
			{
				stream.alignData(PX_SERIAL_ALIGN);
				stream.writeData(mMap, getWordCount()*sizeof(PxU32));
			}
		}
		void	importExtraData(PxDeserializationContext& context)
		{
			if(mMap && getWordCount())
				mMap = context.readExtraData<PxU32, PX_SERIAL_ALIGN>(getWordCount());
		}
		//~PX_SERIALIZATION

		//sschirm: function for placement new. Almost the same as importExtraData above, but lets you set word count and map after default construction
		void importData(PxU32 worldCount, PxU32* words)
		{
			PX_ASSERT(mWordCount == 0 && !mMap);
			mMap = words;
			mWordCount = worldCount | PX_SIGN_BITMASK;
		}
		
		PX_INLINE BitMapBase(Allocator& allocator) : mMap(0), mWordCount(0), mAllocator(allocator) {}

		PX_INLINE BitMapBase() : mMap(0), mWordCount(0) {}

		PX_INLINE ~BitMapBase()
		{
			if(mMap && !isInUserMemory())
				mAllocator.deallocate(mMap);
			mMap = NULL;
		}

		PX_INLINE Allocator&	getAllocator() { return mAllocator; }

		PX_INLINE void growAndSet(PxU32 index)
		{
			extend(index+1);
			mMap[index>>5] |= 1<<(index&31);
		}

		PX_INLINE void growAndReset(PxU32 index)
		{
			extend(index+1);
			mMap[index>>5] &= ~(1<<(index&31));
		}

		PX_INLINE Ps::IntBool boundedTest(PxU32 index) const
		{
			return Ps::IntBool(index>>5 >= getWordCount() ? Ps::IntFalse : (mMap[index>>5]&(1<<(index&31))));
		}

		// Special optimized versions, when you _know_ your index is in range
		PX_INLINE void set(PxU32 index)
		{
			PX_ASSERT(index<getWordCount()*32);
			mMap[index>>5] |= 1<<(index&31);
		}

		PX_INLINE void reset(PxU32 index)
		{
			PX_ASSERT(index<getWordCount()*32);
			mMap[index>>5] &= ~(1<<(index&31));
		}

		PX_INLINE Ps::IntBool test(PxU32 index) const
		{
			PX_ASSERT(index<getWordCount()*32);
			return Ps::IntBool(mMap[index>>5]&(1<<(index&31)));
		}

		// nibble == 4 bits
		PX_INLINE PxU32 getNibbleFast(PxU32 nibIndex) const
		{
			PxU32 bitIndex = nibIndex << 2;
			PX_ASSERT(bitIndex < getWordCount()*32);
			return (mMap[bitIndex >> 5] >> (bitIndex & 31)) & 0xf;
		}

		PX_INLINE void andNibbleFast(PxU32 nibIndex, PxU32 mask)
		{
			//TODO: there has to be a faster way...
			PxU32 bitIndex = nibIndex << 2;
			PxU32 shift = (bitIndex & 31);
			PxU32 nibMask = 0xf << shift;

			PX_ASSERT(bitIndex < getWordCount()*32);

			mMap[bitIndex >> 5] &= ((mask << shift) | ~nibMask);
		}

		PX_INLINE void orNibbleFast(PxU32 nibIndex, PxU32 mask)
		{
			PX_ASSERT(!(mask & ~0xf)); //check extra bits are not set

			PxU32 bitIndex = nibIndex << 2;
			PxU32 shift = bitIndex & 31;

			PX_ASSERT(bitIndex < getWordCount()*32);

			mMap[bitIndex >> 5] |= (mask << shift);
		}

		void clear()
		{
			PxMemSet(mMap, 0, getWordCount()*sizeof(PxU32));
		}

		void resizeAndClear(PxU32 newBitCount)
		{
			extendUninitialized(newBitCount);
			PxMemSet(mMap, 0, getWordCount()*sizeof(PxU32));
		}

		void setEmpty()
		{
			mMap=NULL;
			mWordCount=0;
		}

		void setWords(PxU32* map, PxU32 wordCount)
		{
			mMap=map;
			mWordCount=wordCount;
			mWordCount |= PX_SIGN_BITMASK;
		}
	
		// !!! only sets /last/ bit to value
		void resize(PxU32 newBitCount, bool value = false)
		{
			PX_ASSERT(!value); // only new class supports this
			PX_UNUSED(value);
			extend(newBitCount);
		}
		PxU32 size() const { return getWordCount()*32; }

		void copy(const BitMapBase& a)
		{
			extendUninitialized(a.getWordCount()<<5);
			PxMemCopy(mMap, a.mMap, a.getWordCount() * sizeof(PxU32));
			if(getWordCount() > a.getWordCount())
				PxMemSet(mMap + a.getWordCount(), 0, (getWordCount() - a.getWordCount()) * sizeof(PxU32));
		}

		PX_INLINE PxU32 count()		const
		{
			// NOTE: we can probably do this faster, since the last steps in PxcBitCount32 can be defered to
			// the end of the seq. + 64/128bits at a time + native bit counting instructions(360 is fast non micro code).
			PxU32 count = 0;
			PxU32 wordCount = getWordCount();
			for(PxU32 i=0; i<wordCount; i++)
				count += Ps::bitCount(mMap[i]);

			return count;
		}

		PX_INLINE PxU32 count(PxU32 start, PxU32 length) const
		{
			PxU32 end = PxMin(getWordCount()<<5,start+length);
			PxU32 count = 0;
			for(PxU32 i=start; i<end; i++)
				count+= (test(i)!=0);
			return count;
		}

		//! returns 0 if no bits set (!!!)
		PxU32 findLast() const
		{
			for(PxU32 i = getWordCount(); i-- > 0;)
			{
				if(mMap[i])
					return (i<<5)+Ps::highestSetBit(mMap[i]);
			}
			return PxU32(0); 
		}



		// the obvious combiners and some used in the SDK

		struct OR		{ PX_INLINE PxU32 operator()(PxU32 a, PxU32 b) {	return a|b;		}	};
		struct AND		{ PX_INLINE PxU32 operator()(PxU32 a, PxU32 b) {	return a&b;		}	};
		struct XOR		{ PX_INLINE PxU32 operator()(PxU32 a, PxU32 b) {	return a^b;		}	};

		// we use auxiliary functions here so as not to generate combiners for every combination
		// of allocators

		template<class Combiner, class _> 
		PX_INLINE void combineInPlace(const BitMapBase<_>& b)
		{
			combine1<Combiner>(b.mMap,b.getWordCount());
		}

		template<class Combiner, class _1, class _2>
		PX_INLINE void combine(const BitMapBase<_1>& a, const BitMapBase<_2>& b)
		{
			combine2<Combiner>(a.mMap,a.getWordCount(),b.mMap,b.getWordCount());
		}

		PX_FORCE_INLINE const PxU32*	getWords()			const	{ return mMap;							}
		PX_FORCE_INLINE PxU32*			getWords()					{ return mMap;							}
		
		// PX_SERIALIZATION
		PX_FORCE_INLINE PxU32			getWordCount()		const	{ return mWordCount & ~PX_SIGN_BITMASK;	}
		
		// We need one bit to mark arrays that have been deserialized from a user-provided memory block.
		PX_FORCE_INLINE	PxU32			isInUserMemory()	const	{ return mWordCount & PX_SIGN_BITMASK;	}
		//~PX_SERIALIZATION

		/*!
		Iterate over indices in a bitmap

		This iterator is good because it finds the set bit without looping over the cached bits upto 31 times.
		However it does require a variable shift.
		*/

		class Iterator
		{
		public:
			static const PxU32 DONE = 0xffffffff;

			PX_INLINE Iterator(const BitMapBase &map) :	mBitMap(map)
			{
				reset();
			}

			PX_INLINE Iterator& operator=(const Iterator& other)
			{
				PX_ASSERT(&mBitMap == &other.mBitMap);
				mBlock = other.mBlock;
				mIndex = other.mIndex;
				return *this;
			}

			PX_INLINE PxU32	getNext()
			{
				if(mBlock)
				{
					PxU32 bitIndex = mIndex<<5 | Ps::lowestSetBit(mBlock);
					mBlock &= mBlock-1;
					PxU32 wordCount = mBitMap.getWordCount();
					while(!mBlock && ++mIndex < wordCount)
						mBlock = mBitMap.mMap[mIndex];
					return bitIndex;
				}
				return DONE;
			}

			PX_INLINE void reset()
			{
				mIndex = mBlock = 0;
				PxU32 wordCount = mBitMap.getWordCount();
				while(mIndex < wordCount && ((mBlock = mBitMap.mMap[mIndex]) == 0))
					++mIndex;
			}
		private:
			PxU32 mBlock, mIndex;
			const BitMapBase& mBitMap;
		};

	// DS: faster but less general: hasBits() must be true or getNext() is illegal so it is the calling code's responsibility to ensure that getNext() is not called illegally.
	class LoopIterator
	{
		PX_NOCOPY(LoopIterator)

	public:
		PX_FORCE_INLINE LoopIterator(const BitMapBase &map) : mMap(map.getWords()), mBlock(0), mIndex(-1), mWordCount(PxI32(map.getWordCount()))	{}

		PX_FORCE_INLINE bool hasBits()
		{
			PX_ASSERT(mIndex<mWordCount);
			while (mBlock == 0)
			{
				if (++mIndex == mWordCount)
					return false;
				mBlock = mMap[mIndex];
			}
			return true;
		}

		PX_FORCE_INLINE PxU32 getNext()
		{
			PX_ASSERT(mIndex<mWordCount && mBlock != 0);
			PxU32 result = PxU32(mIndex) << 5 | Ps::lowestSetBit(mBlock);	// will assert if mask is zero
			mBlock &= (mBlock - 1);
			return result;
		}

	private:
		const PxU32*const mMap;
		PxU32 mBlock;		// the word we're currently scanning
		PxI32 mIndex;		// the index of the word we're currently looking at
		PxI32 mWordCount;
	};

	//Class to iterate over the bitmap from a particular start location rather than the beginning of the list
	class CircularIterator
	{
	public:
		static const PxU32 DONE = 0xffffffff;

		PX_INLINE CircularIterator(const BitMapBase &map, PxU32 index) : mBitMap(map)
		{
			mIndex = mBlock = mStartIndex = 0;
			const PxU32 wordCount = mBitMap.getWordCount();
			if ((index << 5) < wordCount)
			{
				mIndex = index << 5;
				mStartIndex = mIndex;
			}

			if (mIndex < wordCount)
			{
				mBlock = mBitMap.mMap[mIndex];
				if (mBlock == 0)
				{
					mIndex = (mIndex + 1) % wordCount;
					while (mIndex != mStartIndex && (mBlock = mBitMap.mMap[mIndex]) == 0)
						mIndex = (mIndex + 1) % wordCount;
				}
			}
		}

		PX_INLINE PxU32	getNext()
		{
			if (mBlock)
			{
				PxU32 bitIndex = mIndex << 5 | Ps::lowestSetBit(mBlock);
				mBlock &= mBlock - 1;
				PxU32 wordCount = mBitMap.getWordCount();
				while (!mBlock && (mIndex = ((mIndex+1)%wordCount)) != mStartIndex)
					mBlock = mBitMap.mMap[mIndex];
				return bitIndex;
			}
			return DONE;
		}

	private:
		PxU32 mBlock, mIndex;
		PxU32 mStartIndex;
		const BitMapBase& mBitMap;

		PX_NOCOPY(CircularIterator)
	};




	protected:
		PxU32*			mMap;	//one bit per index
		PxU32			mWordCount;
		Allocator		mAllocator;
		PxU8			mPadding[3];	// PT: "mAllocator" is empty but consumes 1 byte

		void extend(PxU32 size)
		{
			PxU32 newWordCount = (size+31)>>5;
			if(newWordCount > getWordCount())
			{
				PxU32* newMap = reinterpret_cast<PxU32*>(mAllocator.allocate(newWordCount*sizeof(PxU32), __FILE__, __LINE__));
				if(mMap)
				{
					PxMemCopy(newMap, mMap, getWordCount()*sizeof(PxU32));
					if (!isInUserMemory())
						mAllocator.deallocate(mMap);
				}
				PxMemSet(newMap+getWordCount(), 0, (newWordCount-getWordCount())*sizeof(PxU32));
				mMap = newMap;
				// also resets the isInUserMemory bit
				mWordCount = newWordCount;
			}
		}

		void extendUninitialized(PxU32 size)
		{
			PxU32 newWordCount = (size+31)>>5;
			if(newWordCount > getWordCount())
			{
				if(mMap && !isInUserMemory())
					mAllocator.deallocate(mMap);
				// also resets the isInUserMemory bit
				mWordCount = newWordCount;
				mMap = reinterpret_cast<PxU32*>(mAllocator.allocate(mWordCount*sizeof(PxU32), __FILE__, __LINE__));
			}
		}

		template<class Combiner>
		void combine1(const PxU32* words, PxU32 length)
		{
			extend(length<<5);
			PxU32 combineLength = PxMin(getWordCount(), length);
			for(PxU32 i=0;i<combineLength;i++)
				mMap[i] = Combiner()(mMap[i], words[i]);
		}

		template<class Combiner>
		void combine2(const PxU32* words1, PxU32 length1,
			const PxU32* words2, PxU32 length2)
		{
			extendUninitialized(PxMax(length1,length2)<<5);

			PxU32 commonSize = PxMin(length1,length2);

			for(PxU32 i=0;i<commonSize;i++)
				mMap[i] = Combiner()(words1[i],words2[i]);

			for(PxU32 i=commonSize;i<length1;i++)
				mMap[i] = Combiner()(words1[i],0);

			for(PxU32 i=commonSize;i<length2;i++)
				mMap[i] = Combiner()(0,words2[i]);
		}

		friend class Iterator;
	};

	typedef BitMapBase<Ps::NonTrackingAllocator> BitMap;
	typedef BitMapBase<Ps::VirtualAllocator> BitMapPinned;


} // namespace Cm

}

#endif
