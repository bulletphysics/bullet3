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
#ifndef SN_REPX_CORE_SERIALIZER_H
#define SN_REPX_CORE_SERIALIZER_H
/** \addtogroup RepXSerializers
  @{
*/
#include "foundation/PxSimpleTypes.h"
#include "SnRepXSerializerImpl.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	class XmlReader;
	class XmlMemoryAllocator;
	class XmlWriter;
	class MemoryBuffer;
				
	struct PxMaterialRepXSerializer : RepXSerializerImpl<PxMaterial>
	{
		PxMaterialRepXSerializer( PxAllocatorCallback& inCallback ) : RepXSerializerImpl<PxMaterial>( inCallback ) {}
		virtual PxMaterial* allocateObject( PxRepXInstantiationArgs& );
	};

	struct PxShapeRepXSerializer : public RepXSerializerImpl<PxShape>
	{
		PxShapeRepXSerializer( PxAllocatorCallback& inCallback ) : RepXSerializerImpl<PxShape>( inCallback ) {}
		virtual PxRepXObject fileToObject( XmlReader&, XmlMemoryAllocator&, PxRepXInstantiationArgs&, PxCollection* );
		virtual PxShape* allocateObject( PxRepXInstantiationArgs& ) { return NULL; }
	};
	
	struct PxBVH33TriangleMeshRepXSerializer  : public RepXSerializerImpl<PxBVH33TriangleMesh>
	{
		PxBVH33TriangleMeshRepXSerializer( PxAllocatorCallback& inCallback ) : RepXSerializerImpl<PxBVH33TriangleMesh>( inCallback ) {}
		virtual void objectToFileImpl( const PxBVH33TriangleMesh*, PxCollection*, XmlWriter&, MemoryBuffer&, PxRepXInstantiationArgs& );
		virtual PxRepXObject fileToObject( XmlReader&, XmlMemoryAllocator&, PxRepXInstantiationArgs&, PxCollection* );
		virtual PxBVH33TriangleMesh* allocateObject( PxRepXInstantiationArgs&  ) { return NULL; }
	};
	struct PxBVH34TriangleMeshRepXSerializer  : public RepXSerializerImpl<PxBVH34TriangleMesh>
	{
		PxBVH34TriangleMeshRepXSerializer( PxAllocatorCallback& inCallback ) : RepXSerializerImpl<PxBVH34TriangleMesh>( inCallback ) {}
		virtual void objectToFileImpl( const PxBVH34TriangleMesh*, PxCollection*, XmlWriter&, MemoryBuffer&, PxRepXInstantiationArgs& );
		virtual PxRepXObject fileToObject( XmlReader&, XmlMemoryAllocator&, PxRepXInstantiationArgs&, PxCollection* );
		virtual PxBVH34TriangleMesh* allocateObject( PxRepXInstantiationArgs&  ) { return NULL; }
	};

	struct PxHeightFieldRepXSerializer : public RepXSerializerImpl<PxHeightField>
	{
		PxHeightFieldRepXSerializer( PxAllocatorCallback& inCallback ) : RepXSerializerImpl<PxHeightField>( inCallback ) {}
		virtual void objectToFileImpl( const PxHeightField*, PxCollection*, XmlWriter&, MemoryBuffer&, PxRepXInstantiationArgs& );
		virtual PxRepXObject fileToObject( XmlReader&, XmlMemoryAllocator&, PxRepXInstantiationArgs&, PxCollection* );
		virtual PxHeightField* allocateObject( PxRepXInstantiationArgs& ) { return NULL; }
	};
	
	struct PxConvexMeshRepXSerializer  : public RepXSerializerImpl<PxConvexMesh>
	{
		PxConvexMeshRepXSerializer( PxAllocatorCallback& inCallback ) : RepXSerializerImpl<PxConvexMesh>( inCallback ) {}
		virtual void objectToFileImpl( const PxConvexMesh*, PxCollection*, XmlWriter&, MemoryBuffer&, PxRepXInstantiationArgs& );
		virtual PxRepXObject fileToObject( XmlReader&, XmlMemoryAllocator&, PxRepXInstantiationArgs&, PxCollection* );
		virtual PxConvexMesh* allocateObject( PxRepXInstantiationArgs& ) { return NULL; }
	};

	struct PxRigidStaticRepXSerializer : public RepXSerializerImpl<PxRigidStatic>
	{
		PxRigidStaticRepXSerializer( PxAllocatorCallback& inCallback ) : RepXSerializerImpl<PxRigidStatic>( inCallback ) {}
		virtual PxRigidStatic* allocateObject( PxRepXInstantiationArgs& );
	};

	struct PxRigidDynamicRepXSerializer : public RepXSerializerImpl<PxRigidDynamic>
	{
		PxRigidDynamicRepXSerializer( PxAllocatorCallback& inCallback ) : RepXSerializerImpl<PxRigidDynamic>( inCallback ) {}
		virtual PxRigidDynamic* allocateObject( PxRepXInstantiationArgs& );
	};
	
	struct PxArticulationRepXSerializer  : public RepXSerializerImpl<PxArticulation>
	{
		PxArticulationRepXSerializer( PxAllocatorCallback& inCallback ) : RepXSerializerImpl<PxArticulation>( inCallback ) {}
		virtual void objectToFileImpl( const PxArticulation*, PxCollection*, XmlWriter&, MemoryBuffer&, PxRepXInstantiationArgs& );
		virtual PxArticulation* allocateObject( PxRepXInstantiationArgs& );
	};
	
	struct PxAggregateRepXSerializer :  public RepXSerializerImpl<PxAggregate>
	{
		PxAggregateRepXSerializer( PxAllocatorCallback& inCallback ) : RepXSerializerImpl<PxAggregate>( inCallback ) {}
		virtual void objectToFileImpl( const PxAggregate*, PxCollection*, XmlWriter& , MemoryBuffer&, PxRepXInstantiationArgs& );
		virtual PxRepXObject fileToObject( XmlReader&, XmlMemoryAllocator&, PxRepXInstantiationArgs&, PxCollection* );
		virtual PxAggregate* allocateObject( PxRepXInstantiationArgs& ) { return NULL; }	
	};


#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
/** @} */

