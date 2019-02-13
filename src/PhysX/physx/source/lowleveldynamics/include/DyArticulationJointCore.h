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


#ifndef DV_ARTICULATION_JOINT_CORE_H
#define DV_ARTICULATION_JOINT_CORE_H

#include "DyArticulationCore.h"
#include "PxArticulationJoint.h"

namespace physx
{
	namespace Dy
	{

		struct ArticulationLimit
		{
			PxReal low, high;
		};

		struct ArticulationDrive
		{
			PxReal stiffness, damping, maxForce;
			bool isAcceleration;
		};

		struct ArticulationJointCoreBase
		{
			//= ATTENTION! =====================================================================================
			// Changing the data layout of this class breaks the binary serialization format.  See comments for 
			// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
			// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
			// accordingly.
			//==================================================================================================
		public:

			PX_CUDA_CALLABLE PX_FORCE_INLINE bool setJointPose()
			{
				if (dirtyFlag & ArticulationJointCoreDirtyFlag::ePOSE)
				{
					relativeQuat = (childPose.q * (parentPose.q.getConjugate())).getNormalized();

					//ML: this way work in GPU
					PxU8 flag = PxU8(ArticulationJointCoreDirtyFlag::ePOSE);
					dirtyFlag &= ArticulationJointCoreDirtyFlags(~flag);
				
					return true;
				}

				return false;
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE void operator=(ArticulationJointCoreBase& other)
			{
				parentPose = other.parentPose;
				childPose = other.childPose;

				dirtyFlag = other.dirtyFlag;

				prismaticLimited = other.prismaticLimited;

				//KS - temp place to put reduced coordinate limit and drive values
				for (PxU32 i = 0; i < 6; ++i)
				{
					limits[i] = other.limits[i];
					drives[i] = other.drives[i];
					targetP[i] = other.targetP[i];
					targetV[i] = other.targetV[i];

					dofIds[i] = other.dofIds[i];
					motion[i] = other.motion[i];
				}

				frictionCoefficient = other.frictionCoefficient;
				relativeQuat = other.relativeQuat;
				jointType = other.jointType;
				jointOffset = other.jointOffset; //this is the dof offset for the joint in the cache

			}

			// attachment points, don't change the order, otherwise it will break GPU code
			PxTransform							parentPose;			//28			28
			PxTransform							childPose;			//28			56

			//KS - temp place to put reduced coordinate limit and drive values
			ArticulationLimit					limits[6];			//48			104
			ArticulationDrive					drives[6];			//96			200
			PxReal								targetP[6];			//24			224
			PxReal								targetV[6];			//24			248
			
			// initial parent to child rotation
			PxQuat								relativeQuat;			//16		264
			PxReal								frictionCoefficient;	//4			268
			//this is the dof offset for the joint in the cache
			PxU32								jointOffset;			//4			272

			PxU8								dofIds[6];				//6			278
			PxArticulationMotions				motion[6];				//6			284

			PxReal								maxJointVelocity;		//4			288

			ArticulationJointCoreDirtyFlags		dirtyFlag;				//1			289
			bool								prismaticLimited;		//1			290
			PxU8								jointType;				//1			291
			PxU8								pad[13];				//13		304



			ArticulationJointCoreBase() { maxJointVelocity = 100.f; }
			// PX_SERIALIZATION
			ArticulationJointCoreBase(const PxEMPTY&) {}
			//~PX_SERIALIZATION

		};
	}
}

#endif
