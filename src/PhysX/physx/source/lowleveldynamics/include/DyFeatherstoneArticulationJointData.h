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


#ifndef PXD_FEATHERSTONE_ARTICULATION_JOINTCORE_H
#define PXD_FEATHERSTONE_ARTICULATION_JOINTCORE_H

#include "foundation/PxVec3.h"
#include "foundation/PxQuat.h"
#include "foundation/PxTransform.h"
#include "PsVecMath.h"
#include "CmUtils.h"
#include "CmSpatialVector.h"
#include "DyVArticulation.h"
#include "DyFeatherstoneArticulationUtils.h"
#include <stdio.h>

namespace physx
{
	namespace Dy
	{

	
		class ArticulationJointCoreData
		{
		public:

			ArticulationJointCoreData() : jointOffset(0xffffffff), dofInternalConstraintMask(0)
			{
				for (PxU32 i = 0; i < 6; ++i)
				{
					targetJointPosition[i] = 0.f;
					targetJointVelocity[i] = 0.f;
				}
			}


			PX_CUDA_CALLABLE PX_FORCE_INLINE void computeMotionMatrix(ArticulationJointCoreBase* joint)
			{
				const PxVec3 childOffset = -joint->childPose.p;

				//transpose(Tc)*S = 0
				//transpose(Ta)*S = 1
				switch (joint->jointType)
				{
				case PxArticulationJointType::ePRISMATIC:
				{
					PxReal* jJointAxis = jointAxis[0];
					PxVec3 tJointAxis(jJointAxis[3], jJointAxis[4], jJointAxis[5]);
					const PxVec3 u = (joint->childPose.rotate(tJointAxis)).getNormalized();

					motionMatrix.setNumColumns(1);
					motionMatrix.setColumn(0, PxVec3(0.f), u);

					PX_ASSERT(dof == 1);

					break;
				}
				case PxArticulationJointType::eREVOLUTE:
				{
					PxReal* jJointAxis = jointAxis[0];
					PxVec3 tJointAxis(jJointAxis[0], jJointAxis[1], jJointAxis[2]);
					const PxVec3 u = (joint->childPose.rotate(tJointAxis)).getNormalized();
					const PxVec3 uXd = u.cross(childOffset);

					motionMatrix.setNumColumns(1);
					motionMatrix.setColumn(0, u, uXd);

					break;
				}
				case PxArticulationJointType::eSPHERICAL:
				{
					motionMatrix.setNumColumns(dof);

					for (PxU32 ind = 0; ind <dof; ++ind)
					{
						PxReal* jJointAxis = jointAxis[ind];
						PxVec3 tJointAxis = PxVec3(jJointAxis[0], jJointAxis[1], jJointAxis[2]);
						const PxVec3 u = (joint->childPose.rotate(tJointAxis)).getNormalized();

						const PxVec3 uXd = u.cross(childOffset);
						motionMatrix.setColumn(ind, u, uXd);
					}

					break;
				}
				case PxArticulationJointType::eFIX:
				{
					motionMatrix.setNumColumns(0);

					PX_ASSERT(dof == 0);
					break;
				}
				default:
					break;
				}
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE PxU8 computeJointDofs(ArticulationJointCoreBase* joint) const
			{
				PxU8 tDof = 0;

				for (PxU32 i = 0; i < DY_MAX_DOF; ++i)
				{
					if (joint->motion[i] != PxArticulationMotion::eLOCKED)
					{
						tDof++;
					}
				}

				return tDof;
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE void computeJointDof(ArticulationJointCoreBase* joint, const bool forceRecompute)
			{
				if (joint->dirtyFlag & ArticulationJointCoreDirtyFlag::eMOTION || forceRecompute)
				{
					
					dof = 0;
					lockedAxes = 0;
					limitedAxes = 0;

					joint->prismaticLimited = false;

					memset(jointAxis, 0, sizeof(PxReal) * 6 * 6);

					for (PxU8 i = 0; i < DY_MAX_DOF; ++i)
					{
						if (joint->motion[i] != PxArticulationMotion::eLOCKED)
						{
							//axis is in the local space of joint
							jointAxis[dof][i] = 1.f;

							if (joint->motion[i] == PxArticulationMotion::eLIMITED)
							{
							
								if(i == PxArticulationAxis::eX 
									|| i == PxArticulationAxis::eY
									||i == PxArticulationAxis::eZ)
									joint->prismaticLimited = true;

								limitedAxes++;
							}

							joint->dofIds[dof++] = i;
						}
					}

					lockedAxes = 0;

					//Spherical joints treat locked axes as free axes with a constraint. This produces better
					//results for spherical joints with 2 dofs free, where keeping the 3rd axis locked can lead to
					//an over-consrtained behaviour that is undesirable. However, the drawback is that there will be
					//some drift and error on the joint axes
					if (joint->jointType == PxArticulationJointType::eSPHERICAL && dof == 2)
					{
						for (PxU32 i = 0; i < PxArticulationAxis::eX; ++i)
						{
							if (joint->motion[i] == PxArticulationMotion::eLOCKED)
							{
								//axis is in the local space of joint
								jointAxis[dof][i] = 1.f;
								joint->dofIds[dof++] = PxU8(i);
								lockedAxes++;
							}
						}
					}

					joint->dirtyFlag &= (~ArticulationJointCoreDirtyFlag::eMOTION);
				}

			}


			PX_CUDA_CALLABLE PX_FORCE_INLINE void setJointVelocityDrive(ArticulationJointCoreBase* joint)
			{
				if (joint->dirtyFlag & ArticulationJointCoreDirtyFlag::eTARGETVELOCITY)
				{
					PxU32 count = 0;
					for (PxU32 i = 0; i < DY_MAX_DOF; ++i)
					{
						if (joint->motion[i] != PxArticulationMotion::eLOCKED)
						{
							targetJointVelocity[count] = joint->targetV[i];
							count++;
						}
					}
					joint->dirtyFlag &= ~ArticulationJointCoreDirtyFlag::eTARGETVELOCITY;
				}
			}

			PX_CUDA_CALLABLE PX_FORCE_INLINE void setJointPoseDrive(ArticulationJointCoreBase* joint)
			{
				if (joint->dirtyFlag & ArticulationJointCoreDirtyFlag::eTARGETPOSE)
				{
					PxU32 count = 0;
					for (PxU32 i = 0; i < DY_MAX_DOF; ++i)
					{
						if (joint->motion[i] != PxArticulationMotion::eLOCKED)
						{
							targetJointPosition[count] = joint->targetP[i];
							count++;
						}
					}

					joint->dirtyFlag &= ~ArticulationJointCoreDirtyFlag::eTARGETPOSE;
				}
			}

			//in the joint space
			PxReal								jointAxis[6][6];				//144		144

			//this is in child body space(S)
			SpatialSubspaceMatrix				motionMatrix;					//196		340
			PxReal								targetJointVelocity[6];			//24		364
			PxReal								targetJointPosition[6];			//24		388
			PxReal								maxDriveForce[6];				//24		412

			//this is the dof offset for the joint in the cache
			PxU32								jointOffset;					//4			416
			//degree of freedom
			PxU8								dof;							//1			417
			PxU8								limitedAxes;					//1			418
			PxU8								dofInternalConstraintMask;		//1			419
			PxU8								lockedAxes;						//1			420
			PxU8								padding[12];					//12		432

			

		};

	}//namespace Dy
}

#endif
