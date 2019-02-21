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

#include "GuHeightField.h"
#include "GuConvexMesh.h"
#include "GuTriangleMesh.h"
#include "GuTriangleMeshBV4.h"
#include "GuTriangleMeshRTree.h"

#include "NpRigidStatic.h"
#include "NpRigidDynamic.h"
#include "NpArticulation.h"
#include "NpArticulationLink.h"
#include "NpArticulationJoint.h"
#include "NpMaterial.h"
#include "NpAggregate.h"
#include "GuHeightFieldData.h"

#include "SqPruningStructure.h"

#include "PxBase.h"
#include "PxSerialFramework.h"
#include "PxSerializer.h"
#include "PxPhysicsSerialization.h"

namespace physx
{
	using namespace physx::Gu;

	template<>
	void PxSerializerDefaultAdapter<NpRigidDynamic>::exportData(PxBase& obj, PxSerializationContext& s) const 
	{
		NpRigidDynamic& rigidDynamic = static_cast<NpRigidDynamic&>(obj);
		rigidDynamic.exportData(s);
	}

	template<>
	void PxSerializerDefaultAdapter<NpRigidDynamic>::registerReferences(PxBase& obj, PxSerializationContext& s) const 
	{
		NpRigidDynamic& dynamic = static_cast<NpRigidDynamic&>(obj);

		s.registerReference(obj, PX_SERIAL_REF_KIND_PXBASE, size_t(&obj));

		struct RequiresCallback : public PxProcessPxBaseCallback
		{
			RequiresCallback(physx::PxSerializationContext& c) : context(c) {}
			RequiresCallback& operator=(const RequiresCallback&) { PX_ASSERT(0); return *this; } //PX_NOCOPY doesn't work for local classes
			void process(PxBase& base)
			{
				context.registerReference(base, PX_SERIAL_REF_KIND_PXBASE, size_t(&base));
			}
			PxSerializationContext& context;
		};

		RequiresCallback callback(s);
		dynamic.requiresObjects(callback);
	}

	template<>
	void PxSerializerDefaultAdapter<NpRigidStatic>::exportData(PxBase& obj, PxSerializationContext& s) const
	{
		NpRigidStatic& rigidStatic = static_cast<NpRigidStatic&>(obj);
		rigidStatic.exportData(s);
	}

	template<>
	void PxSerializerDefaultAdapter<NpArticulationLink>::exportData(PxBase& obj, PxSerializationContext& s) const
	{
		NpArticulationLink& articulationLink = static_cast<NpArticulationLink&>(obj);
		articulationLink.exportData(s);
	}

	template<>
	void PxSerializerDefaultAdapter<NpShape>::registerReferences(PxBase& obj, PxSerializationContext& s) const 
	{	
		NpShape& shape = static_cast<NpShape&>(obj);

		s.registerReference(obj, PX_SERIAL_REF_KIND_PXBASE, size_t(&obj));

		struct RequiresCallback : public PxProcessPxBaseCallback
		{
			RequiresCallback(physx::PxSerializationContext& c) : context(c) {}
			RequiresCallback &operator=(const RequiresCallback&) { PX_ASSERT(0); return *this; } //PX_NOCOPY doesn't work for local classes
			void process(PxBase& base)
			{
				PxMaterial* pxMaterial = base.is<PxMaterial>();
				if (!pxMaterial)
				{
					context.registerReference(base, PX_SERIAL_REF_KIND_PXBASE, size_t(&base));
				}
				else
				{
					//ideally we would move this part to ScShapeCore but we don't yet have a MaterialManager available there.
					PxU32 index = static_cast<NpMaterial*>(pxMaterial)->getHandle();
					context.registerReference(base, PX_SERIAL_REF_KIND_MATERIAL_IDX, size_t(index));
				}
			}
			PxSerializationContext& context;
		};

		RequiresCallback callback(s);
		shape.requiresObjects(callback);
	}

	template<>
	bool PxSerializerDefaultAdapter<NpConstraint>::isSubordinate() const
	{
		return true;
	}

	template<>
	bool PxSerializerDefaultAdapter<NpArticulationJoint>::isSubordinate() const
	{
		return true;
	}

	template<>
	bool PxSerializerDefaultAdapter<NpArticulationLink>::isSubordinate() const
	{
		return true;
	}
}

using namespace physx;

void PxRegisterPhysicsSerializers(PxSerializationRegistry& sr)
{
	sr.registerSerializer(PxConcreteType::eCONVEX_MESH,			PX_NEW_SERIALIZER_ADAPTER(ConvexMesh));
	sr.registerSerializer(PxConcreteType::eTRIANGLE_MESH_BVH33,	PX_NEW_SERIALIZER_ADAPTER(RTreeTriangleMesh));
	sr.registerSerializer(PxConcreteType::eTRIANGLE_MESH_BVH34,	PX_NEW_SERIALIZER_ADAPTER(BV4TriangleMesh));
	sr.registerSerializer(PxConcreteType::eHEIGHTFIELD,			PX_NEW_SERIALIZER_ADAPTER(HeightField));
	sr.registerSerializer(PxConcreteType::eRIGID_DYNAMIC,		PX_NEW_SERIALIZER_ADAPTER(NpRigidDynamic));
	sr.registerSerializer(PxConcreteType::eRIGID_STATIC,		PX_NEW_SERIALIZER_ADAPTER(NpRigidStatic));
	sr.registerSerializer(PxConcreteType::eSHAPE,				PX_NEW_SERIALIZER_ADAPTER(NpShape));
	sr.registerSerializer(PxConcreteType::eMATERIAL,			PX_NEW_SERIALIZER_ADAPTER(NpMaterial));
	sr.registerSerializer(PxConcreteType::eCONSTRAINT,			PX_NEW_SERIALIZER_ADAPTER(NpConstraint));
	sr.registerSerializer(PxConcreteType::eAGGREGATE,			PX_NEW_SERIALIZER_ADAPTER(NpAggregate));
	sr.registerSerializer(PxConcreteType::eARTICULATION,		PX_NEW_SERIALIZER_ADAPTER(NpArticulation));
	sr.registerSerializer(PxConcreteType::eARTICULATION_LINK,	PX_NEW_SERIALIZER_ADAPTER(NpArticulationLink));
	sr.registerSerializer(PxConcreteType::eARTICULATION_JOINT,	PX_NEW_SERIALIZER_ADAPTER(NpArticulationJoint));
	sr.registerSerializer(PxConcreteType::ePRUNING_STRUCTURE,	PX_NEW_SERIALIZER_ADAPTER(Sq::PruningStructure));
}


void PxUnregisterPhysicsSerializers(PxSerializationRegistry& sr)
{
	PX_DELETE_SERIALIZER_ADAPTER(sr.unregisterSerializer(PxConcreteType::eCONVEX_MESH));
	PX_DELETE_SERIALIZER_ADAPTER(sr.unregisterSerializer(PxConcreteType::eTRIANGLE_MESH_BVH33));
	PX_DELETE_SERIALIZER_ADAPTER(sr.unregisterSerializer(PxConcreteType::eTRIANGLE_MESH_BVH34));
	PX_DELETE_SERIALIZER_ADAPTER(sr.unregisterSerializer(PxConcreteType::eHEIGHTFIELD));
	PX_DELETE_SERIALIZER_ADAPTER(sr.unregisterSerializer(PxConcreteType::eRIGID_DYNAMIC));
	PX_DELETE_SERIALIZER_ADAPTER(sr.unregisterSerializer(PxConcreteType::eRIGID_STATIC));
	PX_DELETE_SERIALIZER_ADAPTER(sr.unregisterSerializer(PxConcreteType::eSHAPE));
	PX_DELETE_SERIALIZER_ADAPTER(sr.unregisterSerializer(PxConcreteType::eMATERIAL));
	PX_DELETE_SERIALIZER_ADAPTER(sr.unregisterSerializer(PxConcreteType::eCONSTRAINT));
	PX_DELETE_SERIALIZER_ADAPTER(sr.unregisterSerializer(PxConcreteType::eAGGREGATE));
	PX_DELETE_SERIALIZER_ADAPTER(sr.unregisterSerializer(PxConcreteType::eARTICULATION));
	PX_DELETE_SERIALIZER_ADAPTER(sr.unregisterSerializer(PxConcreteType::eARTICULATION_LINK));
	PX_DELETE_SERIALIZER_ADAPTER(sr.unregisterSerializer(PxConcreteType::eARTICULATION_JOINT));
	PX_DELETE_SERIALIZER_ADAPTER(sr.unregisterSerializer(PxConcreteType::ePRUNING_STRUCTURE));
}
