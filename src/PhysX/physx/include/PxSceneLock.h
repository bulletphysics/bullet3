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


#ifndef PX_PHYSICS_NX_SCENELOCK
#define PX_PHYSICS_NX_SCENELOCK
/** \addtogroup physics
@{
*/

#include "PxPhysXConfig.h"
#include "PxScene.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief RAII wrapper for the PxScene read lock.

Use this class as follows to lock the scene for reading by the current thread 
for the duration of the enclosing scope:

	PxSceneReadLock lock(sceneRef);

\see PxScene::lockRead(), PxScene::unlockRead(), PxSceneFlag::eREQUIRE_RW_LOCK
*/
class PxSceneReadLock
{
	PxSceneReadLock(const PxSceneReadLock&);
	PxSceneReadLock& operator=(const PxSceneReadLock&);

public:
	
	/**
	\brief Constructor
	\param scene The scene to lock for reading
	\param file Optional string for debugging purposes
	\param line Optional line number for debugging purposes
	*/
	PxSceneReadLock(PxScene& scene, const char* file=NULL, PxU32 line=0)
		: mScene(scene)
	{
		mScene.lockRead(file, line);
	}

	~PxSceneReadLock()
	{
		mScene.unlockRead();
	}

private:

	PxScene& mScene;
};

/**
\brief RAII wrapper for the PxScene write lock.

Use this class as follows to lock the scene for writing by the current thread 
for the duration of the enclosing scope:

	PxSceneWriteLock lock(sceneRef);

\see PxScene::lockWrite(), PxScene::unlockWrite(), PxSceneFlag::eREQUIRE_RW_LOCK
*/
class PxSceneWriteLock
{
	PxSceneWriteLock(const PxSceneWriteLock&);
	PxSceneWriteLock& operator=(const PxSceneWriteLock&);

public:

	/**
	\brief Constructor
	\param scene The scene to lock for writing
	\param file Optional string for debugging purposes
	\param line Optional line number for debugging purposes
	*/
	PxSceneWriteLock(PxScene& scene, const char* file=NULL, PxU32 line=0)
		: mScene(scene)
	{
		mScene.lockWrite(file, line);
	}

	~PxSceneWriteLock()
	{
		mScene.unlockWrite();
	}

private:

	PxScene& mScene;
};


#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
