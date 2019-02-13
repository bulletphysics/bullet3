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


#ifndef PX_PHYSICS_NP_SCENEQUERY
#define PX_PHYSICS_NP_SCENEQUERY
/** \addtogroup physics 
@{ */

#include "PxBatchQuery.h"
#include "PsArray.h"
#include "PsUserAllocated.h"
#include "CmPhysXCommon.h"
#include "PsSync.h"

namespace physx
{

class NpSceneQueryManager;
struct BatchStreamHeader;
class NpScene;

namespace Sq
{
class SceneQueryManager;
}

struct BatchQueryStream : Ps::Array<char>
{
	BatchQueryStream() { rewind(); }

	void rewind() { mPosition = 0; }

	PX_FORCE_INLINE PxI32 getPos() { return PxI32(mPosition); } // signed to avoid casts elsewhere

	// write an object of type T to the stream, copying by value
	template<typename T>
	PX_FORCE_INLINE void write(const T* val, PxU32 count = 1)
	{
		PX_COMPILE_TIME_ASSERT(sizeof(T) > 0);
		PxU32 newPosition = mPosition + sizeof(T)*count;
		if (newPosition > capacity())
			reserve(newPosition+(newPosition<<1));
		resizeUninitialized(newPosition);
		T* dest = reinterpret_cast<T*>(begin() + mPosition);
		for (PxU32 i = 0; i < count; i++)
		{
			*dest = *(val+i);
			dest++;
		}
		mPosition = newPosition;
	}

	template<typename T>
	PX_FORCE_INLINE void write(const T& val)
	{
		write(&val, 1);
	}

	PX_FORCE_INLINE bool atEnd() const { return mPosition >= size(); }

protected:
	mutable PxU32 mPosition;
};

struct BatchQueryStreamReader
{
	BatchQueryStreamReader(char* buffer) : mBuffer(buffer), mReadPos(0) {}

	// read an object of type T from the stream (simply returns a pointer without copying)
	template<typename T>
	PX_FORCE_INLINE T* read(PxU32 count = 1)
	{
		//PX_ASSERT(mPosition+sizeof(T)*count <= size());
		T* result = reinterpret_cast<T*>(mBuffer+mReadPos);
		mReadPos += sizeof(T)*count;
		return result;
	}

	char* mBuffer;
	PxU32 mReadPos;
};

class NpBatchQuery : public PxBatchQuery, public Ps::UserAllocated
{
public:
											NpBatchQuery(NpScene& owner, const PxBatchQueryDesc& d);
	virtual									~NpBatchQuery();

	// PxBatchQuery interface
	virtual	void							execute();
	virtual void							release();
	virtual	PxBatchQueryPreFilterShader		getPreFilterShader() const;
	virtual	PxBatchQueryPostFilterShader	getPostFilterShader() const;
	virtual	const void*						getFilterShaderData() const;
	virtual	PxU32							getFilterShaderDataSize() const;
	virtual void							setUserMemory(const PxBatchQueryMemory& );
	virtual const PxBatchQueryMemory&		getUserMemory();

	virtual void							raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal distance, PxU16 maxTouchHits,
													PxHitFlags hitFlags, const PxQueryFilterData& filterData,
													void* userData, const PxQueryCache* cache);

	virtual void							overlap(const PxGeometry& geometry, const PxTransform& pose, PxU16 maxTouchHits,
													const PxQueryFilterData& filterData, void* userData,
													const PxQueryCache* cache);

	virtual void							sweep(const PxGeometry& geometry, const PxTransform& pose,
												const PxVec3& unitDir, const PxReal distance, PxU16 maxTouchHits,
												PxHitFlags hitFlags, const PxQueryFilterData& filterData,
												void* userData, const PxQueryCache* cache, const PxReal inflation);

	PxBatchQueryDesc&						getDesc() { return mDesc; }
	virtual const PxBatchQueryDesc&			getDesc() const { return mDesc; }

	enum { eTERMINAL = PxU32(-16) }; // -16 so it's aligned to avoid SPU checks

	// sync object for batch query completion wait
	shdfnd::Sync							mSync;
private:
			void							resetResultBuffers();
			void							finalizeExecute();
			void							writeBatchHeader(const BatchStreamHeader& h);

						NpScene*			mNpScene;
						BatchQueryStream	mStream;
						PxU32				mNbRaycasts, mNbOverlaps, mNbSweeps;
			volatile	PxI32				mBatchQueryIsRunning;
						PxBatchQueryDesc	mDesc;
	// offset in mStream of the offset to the next query for the last header written by BQ query functions
						PxU32				mPrevOffset;
						bool				mHasMtdSweep;

	friend class physx::Sq::SceneQueryManager;
};

}

/** @} */
#endif
