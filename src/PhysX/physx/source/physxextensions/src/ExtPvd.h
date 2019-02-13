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


#ifndef EXT_PVD_H
#define EXT_PVD_H

#if PX_SUPPORT_PVD

#include "CmPhysXCommon.h"
#include "PsUserAllocated.h"
#include "PxJoint.h"
#include "PxPvdDataStream.h"
#include "PxExtensionMetaDataObjects.h"
#include "PvdTypeNames.h"
#include "PxPvdObjectModelBaseTypes.h"

namespace physx
{

class PxJoint;
class PxD6Joint;
class PxDistanceJoint;
class PxFixedJoint;
class PxPrismaticJoint;
class PxRevoluteJoint;
class PxSphericalJoint;
class PxContactJoint;
}

#define JOINT_GROUP 3
namespace physx
{
namespace pvdsdk {
	#define DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP( type ) DEFINE_PVD_TYPE_NAME_MAP( physx::type, "physx3", #type )

	DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxJoint)
	DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxJointGeneratedValues)
	DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxFixedJoint)
	DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxFixedJointGeneratedValues)
	DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxDistanceJoint)
	DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxDistanceJointGeneratedValues)
	DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxContactJoint)
	DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxContactJointGeneratedValues)
	DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxPrismaticJoint)
	DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxPrismaticJointGeneratedValues)
	DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxRevoluteJoint)
	DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxRevoluteJointGeneratedValues)
	DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxSphericalJoint)
	DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxSphericalJointGeneratedValues)
	DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxD6Joint)
	DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxD6JointGeneratedValues)
#undef DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP	
} //pvdsdk
} // physx

namespace physx
{
namespace Ext
{
	using namespace physx::pvdsdk;	
	
	class Pvd: public physx::shdfnd::UserAllocated
	{
		Pvd& operator=(const Pvd&);
	public:
		class PvdNameSpace
		{
		
		public:
			PvdNameSpace(PvdDataStream& conn, const char* name);
			~PvdNameSpace();
		private:
			PvdNameSpace& operator=(const PvdNameSpace&);
			PvdDataStream& mConnection;
		};

		static void setActors( PvdDataStream& PvdDataStream, 
			const PxJoint& inJoint, const PxConstraint& c, const PxActor* newActor0, const PxActor* newActor1 );
		
		template<typename TObjType>
		static void createInstance( PvdDataStream& inStream, const PxConstraint& c, const TObjType& inSource )
		{				
			inStream.createInstance( &inSource );
			inStream.pushBackObjectRef( c.getScene(), "Joints", &inSource );

			class ConstraintUpdateCmd : public PvdDataStream::PvdCommand
			{
				ConstraintUpdateCmd &operator=(const ConstraintUpdateCmd&) { PX_ASSERT(0); return *this; } //PX_NOCOPY doesn't work for local classes
			public:

				const PxConstraint& mConstraint;
				const PxJoint& mJoint;

				PxRigidActor* actor0, *actor1;
				ConstraintUpdateCmd(const PxConstraint& constraint, const PxJoint& joint):PvdDataStream::PvdCommand(), mConstraint(constraint), mJoint(joint)
				{
					mConstraint.getActors( actor0, actor1 ); 
				}

							//Assigned is needed for copying
				ConstraintUpdateCmd(const ConstraintUpdateCmd& cmd)
					:PvdDataStream::PvdCommand(), mConstraint(cmd.mConstraint), mJoint(cmd.mJoint)
				{					
				}

				virtual bool canRun(PvdInstanceDataStream &inStream_ )
				{
					PX_ASSERT(inStream_.isInstanceValid(&mJoint));
					//When run this command, the constraint maybe buffer removed
					return ((actor0 == NULL) || inStream_.isInstanceValid(actor0))
						&&  ((actor1 == NULL) || inStream_.isInstanceValid(actor1));
				}
				virtual void run( PvdInstanceDataStream &inStream_ )
				{
					//When run this command, the constraint maybe buffer removed
					if(!inStream_.isInstanceValid(&mJoint))
						return;

					PxRigidActor* actor0_, *actor1_;
					mConstraint.getActors( actor0_, actor1_ );

					if ( actor0_ && (inStream_.isInstanceValid(actor0_)) )
						inStream_.pushBackObjectRef( actor0_, "Joints", &mJoint );
					if ( actor1_ && (inStream_.isInstanceValid(actor1_)) )
						inStream_.pushBackObjectRef( actor1_, "Joints", &mJoint );
					const void* parent = actor0_ ? actor0_ : actor1_;
					inStream_.setPropertyValue( &mJoint, "Parent", parent );
				}
			};

			ConstraintUpdateCmd* cmd = PX_PLACEMENT_NEW(inStream.allocateMemForCmd(sizeof(ConstraintUpdateCmd)),
				ConstraintUpdateCmd)(c, inSource);
			
			if(cmd->canRun( inStream ))
				cmd->run( inStream );
			else
				inStream.pushPvdCommand( *cmd );
		}

		template<typename jointtype, typename structValue>
		static void updatePvdProperties(PvdDataStream& pvdConnection, const jointtype& joint)
		{
			structValue theValueStruct( &joint );
			pvdConnection.setPropertyMessage( &joint, theValueStruct );
		}
		
		template<typename jointtype>
		static void simUpdate(PvdDataStream& /*pvdConnection*/, const jointtype& /*joint*/) {}		
		
		template<typename jointtype>
		static void createPvdInstance(PvdDataStream& pvdConnection, const PxConstraint& c, const jointtype& joint)
		{
			createInstance<jointtype>( pvdConnection, c, joint );		
		}

		static void releasePvdInstance(PvdDataStream& pvdConnection, const PxConstraint& c, const PxJoint& joint);
		static void sendClassDescriptions(PvdDataStream& pvdConnection);
	};
} // ext

} // physx

#endif // PX_SUPPORT_PVD
#endif // EXT_PVD_H
