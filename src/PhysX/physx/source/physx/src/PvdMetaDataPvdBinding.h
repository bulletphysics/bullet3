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

#ifndef PX_META_DATA_PVD_BINDING_H
#define PX_META_DATA_PVD_BINDING_H

#if PX_SUPPORT_PVD

#include "PxPhysXConfig.h"
#include "PsArray.h"

namespace physx
{
	namespace pvdsdk
	{
		class PsPvd;
		class PvdDataStream;
        struct PvdMetaDataBindingData;
	}
}

namespace physx
{

namespace Sc
{
struct Contact;
}

namespace Vd
{

using namespace physx::pvdsdk;

class PvdVisualizer
{
  protected:
	virtual ~PvdVisualizer()
	{
	}

  public:
	virtual void visualize(PxArticulationLink& link) = 0;
};

class PvdMetaDataBinding
{
	PvdMetaDataBindingData* mBindingData;
	
  public:
	PvdMetaDataBinding();
	~PvdMetaDataBinding();
	void registerSDKProperties(PvdDataStream& inStream);

	void sendAllProperties(PvdDataStream& inStream, const PxPhysics& inPhysics);

	void sendAllProperties(PvdDataStream& inStream, const PxScene& inScene);
	// per frame update
	void sendBeginFrame(PvdDataStream& inStream, const PxScene* inScene, PxReal simulateElapsedTime);
	void sendContacts(PvdDataStream& inStream, const PxScene& inScene, shdfnd::Array<Sc::Contact>& inContacts);
	void sendContacts(PvdDataStream& inStream, const PxScene& inScene);
	void sendStats(PvdDataStream& inStream, const PxScene* inScene);
	void sendSceneQueries(PvdDataStream& inStream, const PxScene& inScene, PsPvd* pvd);
	void sendEndFrame(PvdDataStream& inStream, const PxScene* inScene);

	void createInstance(PvdDataStream& inStream, const PxMaterial& inMaterial, const PxPhysics& ownerPhysics);
	void sendAllProperties(PvdDataStream& inStream, const PxMaterial& inMaterial);
	void destroyInstance(PvdDataStream& inStream, const PxMaterial& inMaterial, const PxPhysics& ownerPhysics);

	void createInstance(PvdDataStream& inStream, const PxHeightField& inData, const PxPhysics& ownerPhysics);
	void sendAllProperties(PvdDataStream& inStream, const PxHeightField& inData);
	void destroyInstance(PvdDataStream& inStream, const PxHeightField& inData, const PxPhysics& ownerPhysics);

	void createInstance(PvdDataStream& inStream, const PxConvexMesh& inData, const PxPhysics& ownerPhysics);
	void destroyInstance(PvdDataStream& inStream, const PxConvexMesh& inData, const PxPhysics& ownerPhysics);

	void createInstance(PvdDataStream& inStream, const PxTriangleMesh& inData, const PxPhysics& ownerPhysics);
	void destroyInstance(PvdDataStream& inStream, const PxTriangleMesh& inData, const PxPhysics& ownerPhysics);

	void createInstance(PvdDataStream& inStream, const PxRigidStatic& inObj, const PxScene& ownerScene, const PxPhysics& ownerPhysics, PsPvd* pvd);
	void sendAllProperties(PvdDataStream& inStream, const PxRigidStatic& inObj);
	void destroyInstance(PvdDataStream& inStream, const PxRigidStatic& inObj, const PxScene& ownerScene);

	void createInstance(PvdDataStream& inStream, const PxRigidDynamic& inObj, const PxScene& ownerScene, const PxPhysics& ownerPhysics, PsPvd* pvd);
	void sendAllProperties(PvdDataStream& inStream, const PxRigidDynamic& inObj);
	void destroyInstance(PvdDataStream& inStream, const PxRigidDynamic& inObj, const PxScene& ownerScene);

	void createInstance(PvdDataStream& inStream, const PxArticulationBase& inObj, const PxScene& ownerScene, const PxPhysics& ownerPhysics, PsPvd* pvd);
	void sendAllProperties(PvdDataStream& inStream, const PxArticulationBase& inObj);
	void destroyInstance(PvdDataStream& inStream, const PxArticulationBase& inObj, const PxScene& ownerScene);

	void createInstance(PvdDataStream& inStream, const PxArticulationLink& inObj, const PxPhysics& ownerPhysics, PsPvd* pvd);
	void sendAllProperties(PvdDataStream& inStream, const PxArticulationLink& inObj);
	void destroyInstance(PvdDataStream& inStream, const PxArticulationLink& inObj);

	void createInstance(PvdDataStream& inStream, const PxShape& inObj, const PxRigidActor& owner, const PxPhysics& ownerPhysics, PsPvd* pvd);
	void sendAllProperties(PvdDataStream& inStream, const PxShape& inObj);
	void releaseAndRecreateGeometry(PvdDataStream& inStream, const PxShape& inObj, PxPhysics& ownerPhysics, PsPvd* pvd);
	void updateMaterials(PvdDataStream& inStream, const PxShape& inObj, PsPvd* pvd);
	void destroyInstance(PvdDataStream& inStream, const PxShape& inObj, const PxRigidActor& owner);

	// These are created as part of the articulation link's creation process, so outside entities don't need to
	// create them.
	void sendAllProperties(PvdDataStream& inStream, const PxArticulationJointBase& inObj);

	// per frame update
	void updateDynamicActorsAndArticulations(PvdDataStream& inStream, const PxScene* inScene, PvdVisualizer* linkJointViz);

	// Origin Shift
	void originShift(PvdDataStream& inStream, const PxScene* inScene, PxVec3 shift);

	void createInstance(PvdDataStream& inStream, const PxAggregate& inObj, const PxScene& ownerScene);
	void sendAllProperties(PvdDataStream& inStream, const PxAggregate& inObj);
	void destroyInstance(PvdDataStream& inStream, const PxAggregate& inObj, const PxScene& ownerScene);
	void detachAggregateActor(PvdDataStream& inStream, const PxAggregate& inObj, const PxActor& inActor);
	void attachAggregateActor(PvdDataStream& inStream, const PxAggregate& inObj, const PxActor& inActor);

	template <typename TDataType>
	void registrarPhysicsObject(PvdDataStream&, const TDataType&, PsPvd*);
};
}
}

#endif
#endif
