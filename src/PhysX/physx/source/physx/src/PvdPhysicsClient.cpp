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

// PX_DUMMY_SYMBOL

#if PX_SUPPORT_PVD

#include "pvd/PxPvdTransport.h"
#include "PxPhysics.h"
#include "PxPvdClient.h"
#include "PxPvdDataStream.h"
#include "PxPvdObjectModelBaseTypes.h"
#include "PvdPhysicsClient.h"
#include "PvdTypeNames.h"

using namespace physx;
using namespace physx::Vd;

PvdPhysicsClient::PvdPhysicsClient(PsPvd* pvd)
: mPvd(pvd), mPvdDataStream(NULL), mIsConnected(false)
{	
}

PvdPhysicsClient::~PvdPhysicsClient()
{
	mPvd->removeClient(this);	
}

PvdDataStream* PvdPhysicsClient::getDataStream()
{
	return mPvdDataStream;
}

PvdMetaDataBinding* PvdPhysicsClient::getMetaDataBinding()
{
	return &mMetaDataBinding;
}

PvdUserRenderer* PvdPhysicsClient::getUserRender()
{
	PX_ASSERT(0);
	return NULL;
}

bool PvdPhysicsClient::isConnected() const
{
	return mIsConnected;
}

void PvdPhysicsClient::onPvdConnected()
{
	if(mIsConnected || !mPvd)
		return;

	mIsConnected = true;	
	mPvdDataStream = PvdDataStream::create(mPvd); 	
	sendEntireSDK();
}

void PvdPhysicsClient::onPvdDisconnected()
{
	if(!mIsConnected)
		return;
	mIsConnected = false;

	mPvdDataStream->release();
	mPvdDataStream = NULL;	
}

void PvdPhysicsClient::flush()
{
}

void PvdPhysicsClient::sendEntireSDK()
{
	PxPhysics& physics = PxGetPhysics();
	
	mMetaDataBinding.registerSDKProperties(*mPvdDataStream);
	mPvdDataStream->createInstance(&physics);
	
	mPvdDataStream->setIsTopLevelUIElement(&physics, true);
	mMetaDataBinding.sendAllProperties(*mPvdDataStream, physics);

#define SEND_BUFFER_GROUP(type, name)                                                                                  \
	{                                                                                                                  \
		physx::shdfnd::Array<type*> buffers;                                                                          \
		PxU32 numBuffers = physics.getNb##name();                                                                      \
		buffers.resize(numBuffers);                                                                                    \
		physics.get##name(buffers.begin(), numBuffers);                                                                \
		for(PxU32 i = 0; i < numBuffers; i++)                                                                          \
		{                                                                                                              \
		if(mPvd->registerObject(buffers[i]))                                                                   \
				createPvdInstance(buffers[i]);                                                                         \
		}                                                                                                         \
	}
	
	SEND_BUFFER_GROUP(PxMaterial, Materials);
	SEND_BUFFER_GROUP(PxTriangleMesh, TriangleMeshes);
	SEND_BUFFER_GROUP(PxConvexMesh, ConvexMeshes);
	SEND_BUFFER_GROUP(PxHeightField, HeightFields);
}

void PvdPhysicsClient::destroyPvdInstance(const PxPhysics* physics)
{
	if(mPvdDataStream)
	     mPvdDataStream->destroyInstance(physics);
}

void PvdPhysicsClient::createPvdInstance(const PxTriangleMesh* triMesh)
{
	mMetaDataBinding.createInstance(*mPvdDataStream, *triMesh, PxGetPhysics());
}

void PvdPhysicsClient::destroyPvdInstance(const PxTriangleMesh* triMesh)
{
	mMetaDataBinding.destroyInstance(*mPvdDataStream, *triMesh, PxGetPhysics());
}

void PvdPhysicsClient::createPvdInstance(const PxConvexMesh* convexMesh)
{
	mMetaDataBinding.createInstance(*mPvdDataStream, *convexMesh, PxGetPhysics());
}

void PvdPhysicsClient::destroyPvdInstance(const PxConvexMesh* convexMesh)
{
	mMetaDataBinding.destroyInstance(*mPvdDataStream, *convexMesh, PxGetPhysics());
}

void PvdPhysicsClient::createPvdInstance(const PxHeightField* heightField)
{
	mMetaDataBinding.createInstance(*mPvdDataStream, *heightField, PxGetPhysics());
}

void PvdPhysicsClient::destroyPvdInstance(const PxHeightField* heightField)
{
	mMetaDataBinding.destroyInstance(*mPvdDataStream, *heightField, PxGetPhysics());
}

void PvdPhysicsClient::createPvdInstance(const PxMaterial* mat)
{
	mMetaDataBinding.createInstance(*mPvdDataStream, *mat, PxGetPhysics());
}

void PvdPhysicsClient::updatePvdProperties(const PxMaterial* mat)
{
	mMetaDataBinding.sendAllProperties(*mPvdDataStream, *mat);
}

void PvdPhysicsClient::destroyPvdInstance(const PxMaterial* mat)
{
	mMetaDataBinding.destroyInstance(*mPvdDataStream, *mat, PxGetPhysics());
}

void PvdPhysicsClient::onGuMeshFactoryBufferRelease(const PxBase* object, PxType typeID)
{
	if(!mIsConnected || !mPvd)
		return;

	if(mPvd->unRegisterObject(object))
	{
		switch(typeID)
		{
		case PxConcreteType::eHEIGHTFIELD:
			destroyPvdInstance(static_cast<const PxHeightField*>(object));
			break;

		case PxConcreteType::eCONVEX_MESH:
			destroyPvdInstance(static_cast<const PxConvexMesh*>(object));
			break;

		case PxConcreteType::eTRIANGLE_MESH_BVH33:
		case PxConcreteType::eTRIANGLE_MESH_BVH34:
			destroyPvdInstance(static_cast<const PxTriangleMesh*>(object));
			break;

		default:
			break;
		}
	}
}

void PvdPhysicsClient::reportError(PxErrorCode::Enum code, const char* message, const char* file, int line)
{
    if(mIsConnected)
	{
		mPvdDataStream->sendErrorMessage(code, message, file, PxU32(line));
	}
}

#endif // PX_SUPPORT_PVD
