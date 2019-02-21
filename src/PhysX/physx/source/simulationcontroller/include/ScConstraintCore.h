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


#ifndef PX_PHYSICS_CONSTRAINTCORE
#define PX_PHYSICS_CONSTRAINTCORE

#include "CmPhysXCommon.h"
#include "PxConstraintDesc.h"
#include "PsAllocator.h"
#include "PxConstraint.h"

namespace physx
{

class PxConstraint;

namespace Sc
{
	class ConstraintCore;
	class ConstraintSim;
	class RigidCore;


	class ConstraintCore : public Ps::UserAllocated	
	{
	//= ATTENTION! =====================================================================================
	// Changing the data layout of this class breaks the binary serialization format.  See comments for 
	// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
	// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
	// accordingly.
	//==================================================================================================
	public:
// PX_SERIALIZATION
											ConstraintCore(const PxEMPTY) : mFlags(PxEmpty), mConnector(NULL), mSim(NULL)	{}
	PX_FORCE_INLINE	void					setConstraintFunctions(PxConstraintConnector& n,
																   const PxConstraintShaderTable& shaders)
											{ 
												mConnector = &n;	
												mSolverPrep = shaders.solverPrep;
												mProject = shaders.project;
												mVisualize = shaders.visualize;
											}
		static		void					getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION
											ConstraintCore(PxConstraintConnector& connector, const PxConstraintShaderTable& shaders, PxU32 dataSize);
											~ConstraintCore();

					// The two-step protocol here allows us to unlink the constraint prior to deleting
					// the actors when synchronizing the scene, then set the bodies after new actors have been inserted

					void					prepareForSetBodies();
					void					setBodies(RigidCore* r0v, RigidCore* r1v);

					PxConstraint*			getPxConstraint();
					const PxConstraint*		getPxConstraint()									const;
	PX_FORCE_INLINE	PxConstraintConnector*	getPxConnector()									const	{ return mConnector;	}

	PX_FORCE_INLINE	PxConstraintFlags		getFlags()											const	{ return mFlags;	}
					void					setFlags(PxConstraintFlags flags);

					void					getForce(PxVec3& force, PxVec3& torque)				const;

					bool					updateConstants(void* addr); 

					void					setBreakForce(PxReal linear, PxReal angular);
					void					getBreakForce(PxReal& linear, PxReal& angular)	const;

					void					setMinResponseThreshold(PxReal threshold);
					PxReal					getMinResponseThreshold()							const	{ return mMinResponseThreshold; }

					void					breakApart();

	PX_FORCE_INLINE	PxConstraintVisualize	getVisualize()										const	{ return mVisualize;				}
	PX_FORCE_INLINE	PxConstraintProject		getProject()										const	{ return mProject;					}
	PX_FORCE_INLINE	PxConstraintSolverPrep	getSolverPrep()										const	{ return mSolverPrep;				}
	PX_FORCE_INLINE	PxU32					getConstantBlockSize()								const	{ return mDataSize;					}

	PX_FORCE_INLINE	void					setSim(ConstraintSim* sim)
											{
												PX_ASSERT((sim==0) ^ (mSim == 0));
												mSim = sim;
											}
	PX_FORCE_INLINE	ConstraintSim*			getSim()											const	{ return mSim;	}
	private:
					PxConstraintFlags		mFlags;
					PxU16					mPaddingFromFlags;	// PT: because flags are PxU16

					PxVec3					mAppliedForce;
					PxVec3					mAppliedTorque;

					PxConstraintConnector*	mConnector;
					PxConstraintProject		mProject;
					PxConstraintSolverPrep	mSolverPrep;
					PxConstraintVisualize	mVisualize;
					PxU32					mDataSize;
					PxReal					mLinearBreakForce;
					PxReal					mAngularBreakForce;
					PxReal					mMinResponseThreshold;

					ConstraintSim*			mSim;
	};

} // namespace Sc

}

#endif
