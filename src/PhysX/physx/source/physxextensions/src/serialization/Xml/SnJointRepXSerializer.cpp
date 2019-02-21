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
#include "PxMetaDataObjects.h"
#include "PxExtensionMetaDataObjects.h"
#include "ExtJointMetaDataExtensions.h" 
#include "SnJointRepXSerializer.h"

namespace physx { 

	template<typename TJointType>
	inline TJointType* createJoint( PxPhysics& physics, 
									   PxRigidActor* actor0, const PxTransform& localFrame0, 
									   PxRigidActor* actor1, const PxTransform& localFrame1 )
	{
		PX_UNUSED(physics);
		PX_UNUSED(actor0);
		PX_UNUSED(actor1);
		PX_UNUSED(localFrame0);
		PX_UNUSED(localFrame1);
		return NULL;
	}

	template<>
	inline PxD6Joint* createJoint<PxD6Joint>(PxPhysics& physics, 
										PxRigidActor* actor0, const PxTransform& localFrame0, 
										PxRigidActor* actor1, const PxTransform& localFrame1)
	{
		return PxD6JointCreate( physics, actor0, localFrame0, actor1, localFrame1 );
	}

	template<>
	inline PxDistanceJoint*	createJoint<PxDistanceJoint>(PxPhysics& physics, 
									 		  PxRigidActor* actor0, const PxTransform& localFrame0, 
											  PxRigidActor* actor1, const PxTransform& localFrame1)
	{
		return PxDistanceJointCreate( physics, actor0, localFrame0, actor1, localFrame1 );
	}

	template<>
	inline PxContactJoint*	createJoint<PxContactJoint>(PxPhysics& physics, 
									 		  PxRigidActor* actor0, const PxTransform& localFrame0, 
											  PxRigidActor* actor1, const PxTransform& localFrame1)
	{
		return PxContactJointCreate( physics, actor0, localFrame0, actor1, localFrame1 );
	}
	
	template<>
	inline PxFixedJoint* createJoint<PxFixedJoint>(PxPhysics& physics, 
										   PxRigidActor* actor0, const PxTransform& localFrame0, 
										   PxRigidActor* actor1, const PxTransform& localFrame1)
	{
		return PxFixedJointCreate( physics, actor0, localFrame0, actor1, localFrame1 );
	}

	template<>
	inline PxPrismaticJoint* createJoint<PxPrismaticJoint>(PxPhysics& physics, 
											   PxRigidActor* actor0, const PxTransform& localFrame0, 
											   PxRigidActor* actor1, const PxTransform& localFrame1)
	{
		return PxPrismaticJointCreate( physics, actor0, localFrame0, actor1, localFrame1 );
	}

	template<>
	inline PxRevoluteJoint*	createJoint<PxRevoluteJoint>(PxPhysics& physics, 
											  PxRigidActor* actor0, const PxTransform& localFrame0, 
											  PxRigidActor* actor1, const PxTransform& localFrame1)
	{
		return PxRevoluteJointCreate( physics, actor0, localFrame0, actor1, localFrame1 );
	}

	template<>
	inline PxSphericalJoint* createJoint<PxSphericalJoint>(PxPhysics& physics, 
											   PxRigidActor* actor0, const PxTransform& localFrame0, 
											   PxRigidActor* actor1, const PxTransform& localFrame1)
	{
		return PxSphericalJointCreate( physics, actor0, localFrame0, actor1, localFrame1 );
	}
	
	template<typename TJointType>
	PxRepXObject PxJointRepXSerializer<TJointType>::fileToObject( XmlReader& inReader, XmlMemoryAllocator& inAllocator, PxRepXInstantiationArgs& inArgs, PxCollection* inCollection )
	{
		PxRigidActor* actor0 = NULL;
		PxRigidActor* actor1 = NULL;
		PxTransform localPose0 = PxTransform(PxIdentity);
		PxTransform localPose1 = PxTransform(PxIdentity);
		bool ok = true;
		if ( inReader.gotoChild( "Actors" ) )
		{
			ok = readReference<PxRigidActor>( inReader, *inCollection, "actor0", actor0 );
			ok &= readReference<PxRigidActor>( inReader, *inCollection, "actor1", actor1 );
			inReader.leaveChild();
		}
		TJointType* theJoint = !ok ? NULL : createJoint<TJointType>( inArgs.physics, actor0, localPose0, actor1, localPose1 );
		
		if ( theJoint )
        {
            PxConstraint* constraint = theJoint->getConstraint();
			PX_ASSERT( constraint );
			inCollection->add( *constraint ); 
			this->fileToObjectImpl( theJoint, inReader, inAllocator, inArgs, inCollection );
        }
		return PxCreateRepXObject(theJoint);
	}

	template<typename TJointType>
	void PxJointRepXSerializer<TJointType>::objectToFileImpl( const TJointType* inObj, PxCollection* inCollection, XmlWriter& inWriter, MemoryBuffer& inTempBuffer, PxRepXInstantiationArgs&   ) 
	{
		writeAllProperties( inObj, inWriter, inTempBuffer, *inCollection );
	}
	
	// explicit template instantiations
	template struct PxJointRepXSerializer<PxFixedJoint>;
	template struct PxJointRepXSerializer<PxDistanceJoint>;
	template struct PxJointRepXSerializer<PxContactJoint>;
	template struct PxJointRepXSerializer<PxD6Joint>;
	template struct PxJointRepXSerializer<PxPrismaticJoint>;
	template struct PxJointRepXSerializer<PxRevoluteJoint>;
	template struct PxJointRepXSerializer<PxSphericalJoint>;
}
