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

#ifndef GU_MESH_FACTORY_H
#define GU_MESH_FACTORY_H

#include "foundation/PxIO.h"
#include "PxTriangleMesh.h"
#include "PxConvexMesh.h"
#include "PxHeightField.h"
#include "PxBVHStructure.h"

#include "CmPhysXCommon.h"
#include "PxPhysXConfig.h"
#include "PsMutex.h"
#include "PsArray.h"

#include "PsUserAllocated.h"
#include "PsHashSet.h"

namespace physx
{

class PxHeightFieldDesc;

namespace Gu
{
	class ConvexMesh;
	class HeightField;
	class TriangleMesh;
	class TriangleMeshData;
	class BVHStructure;
	struct ConvexHullData;
	struct BVHStructureData;
}

class GuMeshFactoryListener
{
protected:
	virtual ~GuMeshFactoryListener(){}
public:
	virtual void onGuMeshFactoryBufferRelease(const PxBase* object, PxType type) = 0;
};

#if PX_VC 
    #pragma warning(push)
	#pragma warning( disable : 4251 ) // class needs to have dll-interface to be used by clients of class
#endif
class PX_PHYSX_COMMON_API GuMeshFactory : public Ps::UserAllocated
{
	PX_NOCOPY(GuMeshFactory)
public:
									GuMeshFactory();
protected:
	virtual							~GuMeshFactory();

public:
	void							release();

	// Triangle meshes
	void							addTriangleMesh(Gu::TriangleMesh* np, bool lock=true);
	PxTriangleMesh*					createTriangleMesh(PxInputStream& stream);
	PxTriangleMesh*					createTriangleMesh(void* triangleMeshData);
	bool							removeTriangleMesh(PxTriangleMesh&);
	PxU32							getNbTriangleMeshes()	const;
	PxU32							getTriangleMeshes(PxTriangleMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

	// Convexes
	void							addConvexMesh(Gu::ConvexMesh* np, bool lock=true);
	PxConvexMesh*					createConvexMesh(PxInputStream&);
	PxConvexMesh*					createConvexMesh(void* convexMeshData);
	bool							removeConvexMesh(PxConvexMesh&);
	PxU32							getNbConvexMeshes() const;
	PxU32							getConvexMeshes(PxConvexMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

	// Heightfields
	void							addHeightField(Gu::HeightField* np, bool lock=true);
	PxHeightField*					createHeightField(void* heightFieldMeshData);
	PxHeightField*					createHeightField(PxInputStream&);
	bool							removeHeightField(PxHeightField&);
	PxU32							getNbHeightFields()	const;
	PxU32							getHeightFields(PxHeightField** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

	// BVHStructure
	void							addBVHStructure(Gu::BVHStructure* np, bool lock=true);
	PxBVHStructure*					createBVHStructure(PxInputStream&);
	PxBVHStructure*					createBVHStructure(void* bvhData);
	bool							removeBVHStructure(PxBVHStructure&);
	PxU32							getNbBVHStructures() const;
	PxU32							getBVHStructures(PxBVHStructure** userBuffer, PxU32 bufferSize, PxU32 startIndex)	const;

	void							addFactoryListener( GuMeshFactoryListener& listener );
	void							removeFactoryListener( GuMeshFactoryListener& listener );
	void							notifyFactoryListener(const PxBase*, PxType typeID);

protected:

	PxTriangleMesh*					createTriangleMesh(Gu::TriangleMeshData& data);
	PxConvexMesh*					createConvexMesh(Gu::ConvexHullData& data);
	PxBVHStructure*					createBVHStructure(Gu::BVHStructureData& data);

	mutable Ps::Mutex				mTrackingMutex;
private:
	Ps::CoalescedHashSet<Gu::TriangleMesh*>		mTriangleMeshes;
	Ps::CoalescedHashSet<Gu::ConvexMesh*>		mConvexMeshes;
	Ps::CoalescedHashSet<Gu::HeightField*>		mHeightFields;
	Ps::CoalescedHashSet<Gu::BVHStructure*>		mBVHStructures;

	Ps::Array<GuMeshFactoryListener*>		mFactoryListeners;
};
#if PX_VC 
     #pragma warning(pop) 
#endif
}

#endif
