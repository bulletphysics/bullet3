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

#ifndef PVD_PHYSICS_CLIENT_H
#define PVD_PHYSICS_CLIENT_H
#if PX_SUPPORT_PVD
#include "foundation/PxErrorCallback.h"
#include "PxPvdClient.h"
#include "PvdMetaDataPvdBinding.h"
#include "NpFactory.h"
#include "PsHashMap.h"
#include "PsMutex.h"
#include "PsPvd.h"

namespace physx
{
class PxProfileMemoryEventBuffer;

namespace Vd
{

class PvdPhysicsClient : public PvdClient, public PxErrorCallback, public NpFactoryListener, public shdfnd::UserAllocated
{
	PX_NOCOPY(PvdPhysicsClient)
  public:
	PvdPhysicsClient(PsPvd* pvd);
	virtual ~PvdPhysicsClient();

	bool isConnected() const;
	void onPvdConnected();
	void onPvdDisconnected();
	void flush();

	physx::pvdsdk::PvdDataStream* getDataStream();
	PvdMetaDataBinding* getMetaDataBinding();
	PvdUserRenderer* getUserRender();
	
	void sendEntireSDK();	
	void destroyPvdInstance(const PxPhysics* physics);

	// NpFactoryListener
	virtual void onGuMeshFactoryBufferRelease(const PxBase* object, PxType typeID);
	/// NpFactoryListener

	// PxErrorCallback
	void reportError(PxErrorCode::Enum code, const char* message, const char* file, int line);

  private:
	void createPvdInstance(const PxTriangleMesh* triMesh);
	void destroyPvdInstance(const PxTriangleMesh* triMesh);
	void createPvdInstance(const PxConvexMesh* convexMesh);
	void destroyPvdInstance(const PxConvexMesh* convexMesh);
	void createPvdInstance(const PxHeightField* heightField);
	void destroyPvdInstance(const PxHeightField* heightField);
	void createPvdInstance(const PxMaterial* mat);
	void destroyPvdInstance(const PxMaterial* mat);
	void updatePvdProperties(const PxMaterial* mat);

	PsPvd*  mPvd;
	PvdDataStream* mPvdDataStream;
	PvdMetaDataBinding mMetaDataBinding;	
	bool mIsConnected;	
};

} // namespace Vd
} // namespace physx

#endif // PX_SUPPORT_PVD
#endif // PVD_PHYSICS_CLIENT_H
