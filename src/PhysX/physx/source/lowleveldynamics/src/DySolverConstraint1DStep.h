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


#ifndef DY_SOLVER_CONSTRAINT_1D_STEP_H
#define DY_SOLVER_CONSTRAINT_1D_STEP_H

#include "foundation/PxVec3.h"
#include "PxvConfig.h"
#include "DyArticulationUtils.h"
#include "DySolverConstraintTypes.h"
#include "DySolverBody.h"
#include "PxConstraintDesc.h"
#include "DySolverConstraintDesc.h"


namespace physx
{
	namespace Dy
	{
		struct SolverContactHeaderStep
		{
			enum DySolverContactFlags
			{
				eHAS_FORCE_THRESHOLDS = 0x1
			};

			PxU8	type;					//Note: mType should be first as the solver expects a type in the first byte.
			PxU8	flags;
			PxU8	numNormalConstr;
			PxU8	numFrictionConstr;					//4

			PxReal	angDom0;							//8
			PxReal	angDom1;							//12
			PxReal	invMass0;							//16

			Vec4V   staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W;		//32
			PxVec3	normal;															//48
			
			PxReal	maxPenBias;														//52
			PxReal	invMass1;														//56
			PxReal	minNormalForce;													//60
			PxU32 broken;															//64
			PxU8* frictionBrokenWritebackByte;										//68	72
			Sc::ShapeInteraction* shapeInteraction;									//72	80


			PX_FORCE_INLINE FloatV getStaticFriction() const { return V4GetX(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W); }
			PX_FORCE_INLINE FloatV getDynamicFriction() const { return V4GetY(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W); }
			PX_FORCE_INLINE FloatV getDominance0() const { return V4GetZ(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W); }
			PX_FORCE_INLINE FloatV getDominance1() const { return V4GetW(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W); }
		};

		struct SolverContactPointStep
		{
			PxVec3 raXnI;
			PxF32 separation;
			PxVec3 rbXnI;
			PxF32 velMultiplier;
			PxF32 targetVelocity;
			PxF32 biasCoefficient;
			//2 more slots here for extra data
			PxU32 pad[2];
		};

		struct SolverContactPointStepExt : public SolverContactPointStep
		{
			Vec3V linDeltaVA;
			Vec3V linDeltaVB;
			Vec3V angDeltaVA;
			Vec3V angDeltaVB;
		};

		struct SolverContactFrictionStep
		{
			Vec4V normalXYZ_ErrorW;		//16
			Vec4V raXnI_targetVelW;
			Vec4V rbXnI_velMultiplierW;
			PxReal biasScale;
			PxReal appliedForce;
			PxReal frictionScale;
			PxU32 pad[1];

			PX_FORCE_INLINE void setAppliedForce(const FloatV f) { FStore(f, &appliedForce); }
		};

		struct SolverContactFrictionStepExt : public SolverContactFrictionStep
		{
			Vec3V linDeltaVA;
			Vec3V linDeltaVB;
			Vec3V angDeltaVA;
			Vec3V angDeltaVB;
		};

		struct SolverConstraint1DHeaderStep
		{
			PxU8	type;			// enum SolverConstraintType - must be first byte
			PxU8	count;			// count of following 1D constraints
			PxU8	dominance;
			PxU8	breakable;		// indicate whether this constraint is breakable or not						
			PxReal	linBreakImpulse;
			PxReal	angBreakImpulse;
			PxReal	invMass0D0;

			PxVec3	body0WorldOffset;
			PxReal	invMass1D1;

			PxVec3	rAWorld;
			PxReal linearInvMassScale0;		// only used by articulations

			PxVec3	rBWorld;
			PxReal	angularInvMassScale0;

			PxReal	linearInvMassScale1;		// only used by articulations
			PxReal	angularInvMassScale1;
			PxU32	pad[2];

			//Ortho axes for body 0, recipResponse in W component
			PxVec4    angOrthoAxis0_recipResponseW[3];
			//Ortho axes for body 1, error of body in W component
			PxVec4    angOrthoAxis1_Error[3];
		};


		PX_FORCE_INLINE void init(SolverConstraint1DHeaderStep& h,
			PxU8 count,
			bool isExtended,
			const PxConstraintInvMassScale& ims)
		{
			h.type = PxU8(isExtended ? DY_SC_TYPE_EXT_1D : DY_SC_TYPE_RB_1D);
			h.count = count;
			h.dominance = 0;
			h.linearInvMassScale0 = ims.linear0;
			h.angularInvMassScale0 = ims.angular0;
			h.linearInvMassScale1 = -ims.linear1;
			h.angularInvMassScale1 = -ims.angular1;
		}


		PX_ALIGN_PREFIX(16)
		struct SolverConstraint1DStep
		{
		public:
			PxVec3		lin0;					//!< linear velocity projection (body 0)	
			PxReal		error;					//!< constraint error term - must be scaled by biasScale. Can be adjusted at run-time

			PxVec3		lin1;					//!< linear velocity projection (body 1)
			PxReal		biasScale;				//!< constraint constant bias scale. Constant

			PxVec3		ang0;					//!< angular velocity projection (body 0)
			PxReal		velMultiplier;			//!< constraint velocity multiplier

			PxVec3		ang1;					//!< angular velocity projection (body 1)
			PxReal		impulseMultiplier;		//!< constraint impulse multiplier

			PxReal		velTarget;				//!< Scaled target velocity of the constraint drive

			PxReal		minImpulse;				//!< Lower bound on impulse magnitude	 
			PxReal		maxImpulse;				//!< Upper bound on impulse magnitude
			PxReal		appliedForce;			//!< applied force to correct velocity+bias

			PxReal		maxBias;
			PxU32		flags;
			PxReal		recipResponse;			//Constant. Only used for articulations;
			PxReal		angularErrorScale;		//Constant
		} PX_ALIGN_SUFFIX(16);

		struct SolverConstraint1DExtStep : public SolverConstraint1DStep
		{
		public:
			Cm::SpatialVectorV deltaVA;
			Cm::SpatialVectorV deltaVB;
		};

		PX_FORCE_INLINE void init(SolverConstraint1DStep& c,
			const PxVec3& _linear0, const PxVec3& _linear1,
			const PxVec3& _angular0, const PxVec3& _angular1,
			PxReal _minImpulse, PxReal _maxImpulse)
		{
			PX_ASSERT(_linear0.isFinite());
			PX_ASSERT(_linear1.isFinite());
			c.lin0 = _linear0;
			c.lin1 = _linear1;
			c.ang0 = _angular0;
			c.ang1 = _angular1;
			c.minImpulse = _minImpulse;
			c.maxImpulse = _maxImpulse;
			c.flags = 0;
			c.appliedForce = 0;
			c.angularErrorScale = 1.f;
		}

	}//namespace Dy
}

#endif
